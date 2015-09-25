/*
 * AC(Alleviate Conflict I/O scheduler)
 * Policy
 * 1.AOS(Async Over Sync)
 * 2.ROW(Read Over Write)
 * 3.Each Requests have soft deadline.
 * 4.Distributed Read And Concentrated Write
 */

/*
  Idea
  change tree to fifo
  avoid not only start index but olso end index.
 */
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/sched.h>

/*
  Priority
  1.Async WRITE
  2.Async READ
  3.Sync WRITE
  4.Sync READ
 */
enum {
	ASYNC,
	SYNC
};

enum Flag {
	ZERO,
	END,
	PROC,
	PEND
};

static const int sync_read_expire = HZ / 2;
static const int sync_write_expire = 2 * HZ;
static const int async_read_expire = 4 * HZ;
static const int async_write_expire = 16 * HZ;

static const int batch_read = 16;
static const int fifo_batch = 8;
static const int writes_starved = 4;

//static const unsigned long device_sector = ((long)500118192);	// ubuntu
static const unsigned long device_sector = (long)(51584); // QEMU
//static const unsigned long device_sector = (long)(134217728); // MyArch

//#define AC_DEBUG
#define FLASH_CHIP_NUM 8
#define SQ_NUM FLASH_CHIP_NUM 

struct sub_queue {
	struct rb_root sorted_list[2][2];
};


// TMP
struct ac_matrix {
	atomic_t rq_num[SQ_NUM];
	atomic_t flg[SQ_NUM];
};

struct ac_data {
	struct request_queue *queue;
	struct sub_queue sq[SQ_NUM];

	/* for alleviate I/O conflict */
	struct ac_matrix matrix[2][2];

	struct list_head deadline_list[2][2];	// common queue for deadline 

	int batched;
	int starved;
	int fifo_batch;

	int fifo_expire[2][2];
	int batch_read;
	int writes_starved;
};

static inline int ac_get_sq_idx(struct request *rq);
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir);
static inline void ac_del_rq_rb(struct ac_data *ad, struct request *rq);
static inline struct rb_root *ac_rb_root(struct ac_data *ad, struct request *rq);
static void ac_add_rq_rb(struct ac_data *ad, struct request *rq);

static void
ac_merged_requests(struct request_queue *q, struct request *req, struct request *next)
{
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before(next->fifo_time, req->fifo_time)){
			list_move(&req->queuelist, &next->queuelist);
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	 Delete next request.
	*/
	rq_fifo_clear(next);
	ac_del_rq_rb((struct ac_data *)&q->elevator->elevator_data, next);
}

static void ac_merged_request(struct request_queue *q,
				    struct request *req, int type)
{
	struct ac_data *ad = q->elevator->elevator_data;

	/*
	 * if the merge was a front merge, we need to reposition request
	 */
	if (type == ELEVATOR_FRONT_MERGE) {
		elv_rb_del(ac_rb_root(ad, req), req);
		ac_add_rq_rb(ad, req);
	}
}

static int ac_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}


static struct request *ac_expired_request(struct ac_data *ad, int sync,
					  int data_dir)
{
	struct list_head *list = &ad->deadline_list[sync][data_dir];
	struct request *rq;
	if (list_empty(list))
		return NULL;

	rq = rq_entry_fifo(list->next);

	/* request has expired */
	if (time_after(jiffies, rq->fifo_time))
		return rq;

	return NULL;
}

static inline struct rb_root *ac_rb_root(struct ac_data *ad, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	const int sync = rq_is_sync(rq);
	int idx = ac_get_sq_idx(rq);

	return &ad->sq[idx].sorted_list[sync][data_dir];
}

/*
 * register ONLY READ rq to adequate tree
 */
static void ac_add_rq_rb(struct ac_data *ad, struct request *rq)
{
	struct rb_root *root;

	root = ac_rb_root(ad, rq);
	elv_rb_add(root, rq);
}

/*
 * unregister read rq from tree and change next rq
 */
static inline void ac_del_rq_rb(struct ac_data *ad, struct request *rq)
{
	struct rb_root *root = ac_rb_root(ad, rq);
	if(root) {
		printk("KERN_DEBUG ac_del_rq_rb\n");
		printk("KERN_DEBUG root:%p, rq:%p\n", root, rq);
		elv_rb_del(root, rq);
	}
}

/*
 * return rq's adequate sublayer index // TODO:regret
 */
static inline int ac_get_sq_idx(struct request *rq)
{
	const int sector = blk_rq_pos(rq);
	int sl_sector_num = (int)(device_sector / SQ_NUM);
	int idx = (int)(sector / sl_sector_num);
	printk("KERN_DEBUG ac_get_sq_idx idx:%d\n", idx);
	return idx;
}

/*
 * add rq to rbtree and fifo
 * registerd to elevator_add_req_fn
 */
static void ac_add_request(struct request_queue *q, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	const int sync = rq_is_sync(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_data *ad = q->elevator->elevator_data;
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];

#ifdef AC_DEBUG
	printk("KERN_DEBUG insert rq is data_dir:%d, sync:%d idx:%d\n",
	       data_dir, sync, idx);
#endif

	/*
	 * set expire time and add to fifo list and tree
	 */
	rq->fifo_time = jiffies + ad->fifo_expire[sync][data_dir];
	list_add_tail(&rq->queuelist, &ad->deadline_list[sync][data_dir]); 
	ac_add_rq_rb(ad, rq);

	// update matrix information
	atomic_inc(&mat->rq_num[idx]);
	atomic_set(&mat->flg[idx], PEND);
}

static int ac_allow_merge(struct request_queue *q, struct request *rq,
			  struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}

static void ac_update_matrix(struct ac_data *ad, struct request *rq)
{
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];
	int i;

	atomic_dec(&mat->rq_num[idx]);
	if (data_dir == READ) {	// read
		if (atomic_read(&mat->rq_num[idx]) == 0) {
			atomic_set(&mat->flg[idx], ZERO);
		} else {
			atomic_set(&mat->flg[idx], END);
		}
		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->flg[i]) == PEND) {
				return;
			}
		}

		for (i = 0; i < SQ_NUM; i++) {	// if all non zero entry is finished
			if (atomic_read(&mat->rq_num[i]) != 0) {
				atomic_set(&mat->flg[i], PEND);
			}
		}

	} else {		// write
		if (atomic_read(&mat->rq_num[idx]) == 0) {
			atomic_set(&mat->flg[idx], ZERO);
		} else {
			atomic_set(&mat->flg[idx], PROC);
		}

		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->flg[i]) == PEND) {
				return;
			}
		}

		for (i = 0; i < SQ_NUM; i++) {	// if all non zero entry is finished
			if (atomic_read(&mat->rq_num[i]) != 0) {
				atomic_set(&mat->flg[i],PEND);
			}
		}

	}
//	spin_unlock_irq(&ad->matrix[sync][data_dir].my_lock);

#ifdef AC_DEBUG
	printk("KERN_DEBUG ac_update_matrix done.\n");
	ac_display_matrix(ad, ASYNC, READ);
	ac_display_matrix(ad, ASYNC, WRITE);
	ac_display_matrix(ad, SYNC, READ);
	ac_display_matrix(ad, SYNC, WRITE);
#endif
}

static struct request *ac_choose_expired_request(struct ac_data *ad)
{
	struct request *rq;
	rq = ac_expired_request(ad, ASYNC, READ);
	if (rq) {
		return rq;
	}

	rq = ac_expired_request(ad, ASYNC, WRITE);
	if (rq) {
		return rq;
	}

	rq = ac_expired_request(ad, SYNC, READ);
	if (rq) {
		return rq;
	}

	rq = ac_expired_request(ad, SYNC, WRITE);
	if (rq) {
		return rq;
	}

	return NULL;		// no expired request
}

// zero proc end pend
// TODO:emplement choose rq function
static struct request *ac_choose_request(struct ac_data *ad, int data_dir)
{
	struct ac_matrix *async_mat = &ad->matrix[ASYNC][data_dir];
	struct ac_matrix *sync_mat = &ad->matrix[SYNC][data_dir];
	struct request *rq = NULL;
	int i;
	if (data_dir == READ) {
		/* ASYNC READ */
		if(!list_empty(&ad->deadline_list[ASYNC][READ])) {
			for (i = 0; i < SQ_NUM; i++) {
				if (atomic_read(&async_mat->rq_num[i]) != 0
					&& atomic_read(&async_mat->flg[i]) != PEND ) {
					rq = rb_entry_rq(rb_first
									 (&ad->sq[i].
									  sorted_list[ASYNC][READ]));
					break;
				}
			}
			if (rq == NULL) {
				for (i = 0; i < SQ_NUM; i++) {
					if (atomic_read(&async_mat->flg[i]) == PEND) {
						rq = rb_entry_rq(rb_first
										 (&ad->sq[i].
										  sorted_list[ASYNC]
										  [READ]));
						break;
					}
				}
			}
		}

		if (rq != NULL) {
			return rq;
		}
		/* SYNC READ */
		if(!list_empty(&ad->deadline_list[SYNC][READ])) {
			for (i = 0; i < SQ_NUM; i++) {
				if (atomic_read(&sync_mat->rq_num[i]) != 0
					&& atomic_read(&sync_mat->flg[i]) != PEND) {
					rq = rb_entry_rq(rb_first
									 (&ad->sq[i].
									  sorted_list[SYNC][READ]));
					break;
				}
			}
			if (rq == NULL) {
				for (i = 0; i < SQ_NUM; i++) {
					if (atomic_read(&sync_mat->flg[i]) == PEND) {
						rq = rb_entry_rq(rb_first
										 (&ad->sq[i].
										  sorted_list[SYNC]
										  [READ]));
						break;
					}
				}
			}
		}
	} else {		// write
		if(!list_empty(&ad->deadline_list[ASYNC][WRITE])) {
			for (i = 0; i < SQ_NUM; i++) {
				if (atomic_read(&async_mat->flg[i]) == PROC) {
					rq = rb_entry_rq(rb_first(&ad->sq[i]. sorted_list[ASYNC][WRITE]));
					break;
				}
			}
			if (rq == NULL) {
				for (i = 0; i < SQ_NUM; i++) {
					if (atomic_read(&async_mat->flg[i]) == PEND) {
						rq = rb_entry_rq(rb_first
										 (&ad->sq[i].
										  sorted_list[ASYNC]
										  [WRITE]));
						break;
					}
				}
			}
		}

		if (rq != NULL) {
			return rq;
		}
		/* SYNC WRITE */
		if(!list_empty(&ad->deadline_list[SYNC][WRITE])) {
			for (i = 0; i < SQ_NUM; i++) {
				if (atomic_read(&sync_mat->flg[i]) == PROC) {
					rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][WRITE]));
					break;
				}
			}
			if (rq == NULL) {
				for (i = 0; i < SQ_NUM; i++) {
					if (atomic_read(&sync_mat->flg[i]) == PEND) {
						rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][WRITE]));
						break;
					}
				}
			}
		}
	}
			
			
	if (rq != NULL) {
		return rq;
	}

#ifdef AC_DEBUG
	printk("KERN_DEBUG not exist data_dir:%d  request\n", data_dir);
#endif
	return NULL;
}

static void ac_dispatch_request(struct ac_data *ad, struct request *rq)
{
	rq_fifo_clear(rq);
	ac_del_rq_rb(ad, rq);
	elv_dispatch_add_tail(rq->q, rq);
	ac_update_matrix(ad, rq);

	if (rq_data_dir(rq))
		ad->starved = 0;
	else
		ad->starved++;
}

#ifdef AC_DEBUG
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir)
{
	int i;
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", i);
	}
	printk("\n");
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", atomic_read(&ad->matrix[sync][data_dir].rq_num[i]));
	}
	printk("\n");
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", atomic_read(&ad->matrix[sync][data_dir].flg[i]));
	}
	printk("\n");
}
#endif

static int ac_dispatch_requests(struct request_queue *q, int force)
{
	struct ac_data *ad = q->elevator->elevator_data;
	struct request *rq = NULL;
	int data_dir = READ;

#ifdef AC_DEBUG
	printk("KERN_DEBUG dispatch reqests\n");
	printk("KERN_DEBUG async read list%d\n",
	       list_empty(&ad->deadline_list[ASYNC][READ]));
	printk("KERN_DEBUG async write list%d\n",
	       list_empty(&ad->deadline_list[ASYNC][WRITE]));
	printk("KERN_DEBUG sync read list%d\n",
	       list_empty(&ad->deadline_list[SYNC][READ]));
	printk("KERN_DEBUG sync write list%d\n",
	       list_empty(&ad->deadline_list[SYNC][WRITE]));
	ac_display_matrix(ad, 0, 0);
	ac_display_matrix(ad, 0, 1);
	ac_display_matrix(ad, 1, 0);
	ac_display_matrix(ad, 1, 1);
#endif

	if (ad->batched > ad->fifo_batch) {
		ad->batched = 0;
		rq = ac_choose_expired_request(ad);
	}

	if (!rq) {
		if (ad->starved > ad->writes_starved)
			data_dir = WRITE;
		

		rq = ac_choose_request(ad, data_dir);

		if (!rq && data_dir == WRITE) {
			rq = ac_choose_request(ad, READ);
		} else if (!rq && data_dir == READ) {
			rq = ac_choose_request(ad, WRITE);
		}

		if (!rq)
			return 0;
	}
#ifdef AC_DEBUG
	printk("KERN_DEBUG dispatch reqests\n");
	printk("KERN_DEBUG dispatch rq:%p\n", rq);
#endif
	ac_dispatch_request(ad, rq);
	return 1;
}

/*
 * initialize elevator private data (ac_data).
 */
static int ac_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct ac_data *ad;
	struct elevator_queue *eq;
	int i;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	ad = kzalloc_node(sizeof(*ad), GFP_KERNEL, q->node);
	if (!ad) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}

	eq->elevator_data = ad;
	ad->queue = q;
	// init deadline queues

	INIT_LIST_HEAD(&ad->deadline_list[ASYNC][READ]);
	INIT_LIST_HEAD(&ad->deadline_list[ASYNC][WRITE]);
	INIT_LIST_HEAD(&ad->deadline_list[SYNC][READ]);
	INIT_LIST_HEAD(&ad->deadline_list[SYNC][WRITE]);


	// init sub queue list
	for (i = 0; i < SQ_NUM; i++) {
		ad->sq[i].sorted_list[ASYNC][READ] = RB_ROOT;
		ad->sq[i].sorted_list[ASYNC][WRITE] = RB_ROOT;
		ad->sq[i].sorted_list[SYNC][READ] = RB_ROOT;
		ad->sq[i].sorted_list[SYNC][WRITE] = RB_ROOT;
	}
	// init data matrix
	for (i = 0; i < SQ_NUM; i++) {
		atomic_set(&ad->matrix[ASYNC][READ].rq_num[i], 0);
		atomic_set(&ad->matrix[ASYNC][READ].flg[i], 0);
		atomic_set(&ad->matrix[ASYNC][WRITE].rq_num[i], 0);
		atomic_set(&ad->matrix[ASYNC][WRITE].flg[i], 0);
		atomic_set(&ad->matrix[SYNC][READ].rq_num[i], 0);
		atomic_set(&ad->matrix[SYNC][READ].flg[i], 0);
		atomic_set(&ad->matrix[SYNC][WRITE].rq_num[i], 0);
		atomic_set(&ad->matrix[SYNC][WRITE].flg[i], 0);


	}

	// init preferences
//	ad->pre_mode = READ;
	ad->batched = 0;
	ad->starved = 0;
	ad->fifo_batch = fifo_batch;
	ad->writes_starved = writes_starved;

	ad->fifo_expire[SYNC][READ] = sync_read_expire;
	ad->fifo_expire[SYNC][WRITE] = sync_write_expire;
	ad->fifo_expire[ASYNC][READ] = async_read_expire;
	ad->fifo_expire[ASYNC][WRITE] = async_write_expire;

	ad->batch_read = batch_read;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void ac_exit_queue(struct elevator_queue *e)
{
	struct ac_data *ad = e->elevator_data;
	int i;

	BUG_ON(!list_empty(&ad->deadline_list[ASYNC][READ]));
	BUG_ON(!list_empty(&ad->deadline_list[ASYNC][WRITE]));
	BUG_ON(!list_empty(&ad->deadline_list[SYNC][READ]));
	BUG_ON(!list_empty(&ad->deadline_list[SYNC][WRITE]));

	for (i = 0; i < SQ_NUM; i++) {
		BUG_ON(!RB_EMPTY_ROOT(&ad->sq[i].sorted_list[ASYNC][READ]));
		BUG_ON(!RB_EMPTY_ROOT(&ad->sq[i].sorted_list[ASYNC][WRITE]));
		BUG_ON(!RB_EMPTY_ROOT(&ad->sq[i].sorted_list[SYNC][READ]));
		BUG_ON(!RB_EMPTY_ROOT(&ad->sq[i].sorted_list[SYNC][WRITE]));
	}

	kfree(ad);
}

static struct elevator_type iosched_ac = {
	.ops = {
		.elevator_merge_fn = 		ac_merge,
		.elevator_merged_fn =		ac_merged_request,
		.elevator_merge_req_fn =	ac_merged_requests,
//		.elevator_allow_merge_fn = ac_allow_merge, // NG
		.elevator_dispatch_fn = ac_dispatch_requests,
		.elevator_add_req_fn = ac_add_request,
		.elevator_former_req_fn = elv_rb_former_request, // ok
		.elevator_latter_req_fn = elv_rb_latter_request, // ok
		.elevator_init_fn = ac_init_queue, // ok
		.elevator_exit_fn = ac_exit_queue, // ok
		},

	.elevator_name = "acp",
	.elevator_owner = THIS_MODULE,
};

static int __init ac_init(void)
{
	elv_register(&iosched_ac);
	return 0;
}

static void __exit ac_exit(void)
{
	elv_unregister(&iosched_ac);
}

module_init(ac_init);
module_exit(ac_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ac(alleviate conflict) plus IO scheduler");
MODULE_VERSION("0.1");
