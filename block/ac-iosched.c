/*
 * AC(Alleviate Conflict I/O scheduler)
 * Policy
 * 1.AOS(Async Over Sync)
 * 2.ROW(Read Over Write)
 * 3.Each Requests have soft deadline.
 * 4.Distributed Read And Concentrated Write
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

//#define AC_DEBUG
/*
  Priority
  1.Async WRITE
  2.Async READ
  3.Sync WRITE
  4.Sync READ
 */
   
static const int sync_read_expire = HZ /2;
static const int sync_write_expire = 2 * HZ;
static const int async_read_expire = 4 * HZ;
static const int async_write_expire = 16* HZ;

static const int batch_read = 16;
static const int fifo_batch = 8;
static const int writes_starved = 4;

enum{ ASYNC,
	  SYNC };

enum Flag { ZERO,
			END,
			PROC,
			PEND };


// cat /sys/class/block/sda/size * QEMU:51584
//static const unsigned long device_sector = ((long)1024 * 50); // QEMU
static const unsigned long device_sector = (long)(51584); // QEMU

//static const unsigned long device_sector = ((long)1024 * 1024 * 128); // My Arch

#define FLASH_CHIP_NUM 8
#define SQ_NUM FLASH_CHIP_NUM * 2


// tmp
struct sub_queue {
	struct rb_root sorted_list[2][2];
};

struct ac_matrix {
	int rq_num[SQ_NUM];
	int flg[SQ_NUM];
	//spinlock_t *ac_lock;
};

struct ac_data {
	struct request_queue *queue;
	struct sub_queue sq[SQ_NUM];
	/* for data */
	struct ac_matrix matrix[2][2];

	struct list_head deadline_list[2][2]; // common queue for deadline 

	int pre_mode; // AR or AW or SR or SW
	int batched;
	int starved;
	int fifo_batch;

	int fifo_expire[2][2];
	int batch_read;
	int writes_starved;
};


static inline int ac_get_sq_idx(struct request *rq);
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir);
// get expired request
static struct request *
ac_expired_request(struct ac_data *ad, int sync, int data_dir) {
	struct list_head *list = &ad->deadline_list[sync][data_dir];
	struct request *rq;
	if (list_empty(list))
		return NULL;

	
	rq = rq_entry_fifo(list->next);
	return rq; // for exp

	/* request has expired */
	if (time_after(jiffies , rq->fifo_time))
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
	elv_rb_del(ac_rb_root(ad, rq), rq);
}

/*
 * return rq's adequate sublayer index // TODO:regret
 */
static inline int ac_get_sq_idx(struct request *rq)
{
	const int sector = blk_rq_pos(rq); // ok
	int sl_sector_num = (int)(51584 / SQ_NUM);
	int idx = (int)(sector / sl_sector_num);


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
	printk("KERN_DEBUG insert rq is data_dir:%d, sync:%d idx:%d\n", data_dir, sync, idx);
#endif
	ac_add_rq_rb(ad, rq); // insert tree

	/*
	 * set expire time and add to fifo list
	 */
	rq->fifo_time = jiffies + ad->fifo_expire[sync][data_dir];
	list_add_tail(&rq->queuelist, &ad->deadline_list[sync][data_dir]);

	// update matrix information
	mat->rq_num[idx]++;
	mat->flg[idx] = PEND;
#ifdef AC_DEBUG	
	printk("KERN_DEBUG after add rq");
	ac_display_matrix(ad, ASYNC, READ);
	ac_display_matrix(ad, ASYNC, WRITE);
	ac_display_matrix(ad, SYNC, READ);
	ac_display_matrix(ad, SYNC, WRITE);
	printk("KERN_DEBUG end after add rq");
#endif
}

static int ac_allow_merge(struct request_queue *q, struct request *rq, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}

static void
ac_update_matrix(struct ac_data *ad, struct request *rq)
{
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];
	int i;
	mat->rq_num[idx]--;
	if (data_dir == READ) { // read
		if (mat->rq_num[idx] == 0) { 
			mat->flg[idx] = ZERO;
		} else {
			mat->flg[idx] = END;
		}
		for (i = 0; i < SQ_NUM; i++) {
			if (mat->flg[i] == PEND) {
				return;
			}
		}
		
		for (i = 0; i < SQ_NUM; i++) { // if all non zero entry is finished
			if (mat->rq_num[i] != 0) {
				mat->flg[i] = PEND;
			}
		}
		
	} else { // write
		if (mat->rq_num[idx] == 0) {
			mat->flg[idx] = ZERO;
		} else {
			mat->flg[idx] = PROC;
		}

		for (i = 0; i < SQ_NUM; i++) {
			if (mat->flg[i] == PEND) {
				return;
			}
		}
		
		for (i = 0; i < SQ_NUM; i++) { // if all non zero entry is finished
			if (mat->rq_num[i] != 0) {
				mat->flg[i] = PEND;
			}
		}
		
	}

#ifdef AC_DEBUG
	printk("KERN_DEBUG ac_update_matrix done.\n");
	ac_display_matrix(ad, ASYNC, READ);
	ac_display_matrix(ad, ASYNC, WRITE);
	ac_display_matrix(ad, SYNC, READ);
	ac_display_matrix(ad, SYNC, WRITE);
#endif 
}

static struct request *
ac_choose_expired_request(struct ac_data *ad)
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

	return NULL; // no expired request
}
// zero proc end pend
// TODO:emplement choose rq function
static struct request *
ac_choose_request(struct ac_data *ad, int data_dir)
{
	struct ac_matrix *async_mat = &ad->matrix[ASYNC][data_dir];
	struct ac_matrix *sync_mat = &ad->matrix[SYNC][data_dir];
	struct request *rq = NULL;
	int i;
	if (data_dir == READ) {
		/* ASYNC READ*/
		for (i = 0; i < SQ_NUM; i++) {
			if (async_mat->rq_num[i] != 0 && async_mat->flg[i] != PEND) {
				rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[ASYNC][READ]));
				break;
			} 
		}
		if (rq == NULL) {
			for (i = 0; i < SQ_NUM; i++) {
				if (async_mat->flg[i] == PEND) {
					rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[ASYNC][READ]));
					break;
				}
			}
		}

		if (rq != NULL)
			return rq;
		/* SYNC READ*/
		for (i = 0; i < SQ_NUM; i++) {
			if (sync_mat->rq_num[i] != 0 && sync_mat->flg[i] != PEND) {
				rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][READ]));
				break;
			} 
		}
		if (rq == NULL) {
			for (i = 0; i < SQ_NUM; i++) {
				if (sync_mat->flg[i] == PEND) {
					rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][READ]));
					break;
				}
			}
		}

		if (rq != NULL) 
			return rq;
	} else { // write
		for (i = 0; i < SQ_NUM; i++) {
			if (async_mat->flg[i] == PROC) {
				rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[ASYNC][WRITE]));
				break;
			}
		}
		if (rq == NULL) {
			for (i = 0; i < SQ_NUM; i++) {
				if(async_mat->flg[i] == PEND) {
					rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[ASYNC][WRITE]));
					break;
				}
			}
		}

		if (rq != NULL)
			return rq;
	
		for (i = 0; i < SQ_NUM; i++) {
			if (sync_mat->flg[i] == PROC) {
				rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][WRITE]));
				break;
			}
		}
		if (rq == NULL) {
			for (i = 0; i < SQ_NUM; i++) {
				if(sync_mat->flg[i] == PEND) {
					rq = rb_entry_rq(rb_first(&ad->sq[i].sorted_list[SYNC][WRITE]));
					break;
				}
			}
		}
		if (rq != NULL)
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

	if(rq_data_dir(rq))
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
		printk("%2d", ad->matrix[sync][data_dir].rq_num[i]);
	}
	printk("\n");
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", ad->matrix[sync][data_dir].flg[i]);
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
	printk("KERN_DEBUG list%d\n", list_empty(&ad->deadline_list[ASYNC][READ]));
	printk("KERN_DEBUG list%d\n", list_empty(&ad->deadline_list[ASYNC][WRITE]));
	printk("KERN_DEBUG list%d\n", list_empty(&ad->deadline_list[SYNC][READ]));
	printk("KERN_DEBUG list%d\n", list_empty(&ad->deadline_list[SYNC][WRITE]));
	ac_display_matrix(ad, 0, 0);
	ac_display_matrix(ad, 0, 1);
	ac_display_matrix(ad, 1, 0);
	ac_display_matrix(ad, 1, 1);
#endif 

	if (ad->batched > ad->fifo_batch) {
		ad->batched = 0;
		rq = ac_choose_expired_request(ad);
	}


	if(!rq) {
		if (ad->starved > ad->writes_starved)
			data_dir = WRITE;

		rq = ac_choose_request(ad, data_dir);

		if(!rq)
			rq = ac_choose_request(ad, !data_dir);

		if(!rq) 
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
//	spin_lock_init(ad->matrix[ASYNC][READ].ac_lock);
//	spin_lock_init(ad->matrix[ASYNC][WRITE].ac_lock);
//	spin_lock_init(ad->matrix[SYNC][READ].ac_lock);

//	spin_lock_init(ad->matrix[SYNC][WRITE].ac_lock);

	for (i = 0; i < SQ_NUM; i++) {
		ad->matrix[ASYNC][READ].rq_num[i] = 0;
		ad->matrix[ASYNC][READ].flg[i] = ZERO;
		ad->matrix[ASYNC][WRITE].rq_num[i] = 0;
		ad->matrix[ASYNC][WRITE].flg[i] = ZERO;
		ad->matrix[SYNC][READ].rq_num[i] = 0;
		ad->matrix[SYNC][READ].flg[i] = ZERO;
		ad->matrix[SYNC][WRITE].rq_num[i] = 0;
		ad->matrix[SYNC][WRITE].flg[i] = ZERO;

	}

	// init preferences
	ad->pre_mode = READ;
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
		.elevator_allow_merge_fn = ac_allow_merge,
		.elevator_dispatch_fn = ac_dispatch_requests,
		.elevator_add_req_fn = ac_add_request,
		.elevator_init_fn = ac_init_queue,
		.elevator_exit_fn = ac_exit_queue,
		},

	.elevator_name = "ac",
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
MODULE_DESCRIPTION("ac(alleviate conflict) IO scheduler");
MODULE_VERSION("0.1");
