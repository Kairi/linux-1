#include <linux/kernel.h>
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

/* for debug */
/* #define AC_DEBUG */

#define FLASH_CHIP_NUM 8
#define SQ_NUM 8

struct sub_queue {
	struct list_head fifo_list[2][2];
};

struct ac_matrix {
	atomic_t rq_num[SQ_NUM];
	atomic_t flg[SQ_NUM];
};

struct ac_data {
	struct request_queue *queue;
	struct sub_queue sq[SQ_NUM];

	struct ac_matrix matrix[2][2];

	struct list_head deadline_list[2][2];

	int batched;
	int starved;
	int fifo_batch;
	int pre_idx;

	int fifo_expire[2][2];
	int batch_read;
	int writes_starved;
};

static inline int ac_get_sq_idx(struct request *rq);
#ifdef AC_DEBUG
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir);
#endif

static void
ac_merged_requests(struct request_queue *q, struct request *req,
		   struct request *next)
{
	struct ac_data *ad = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(req);
	const int sync = rq_is_sync(req);
	const int idx = ac_get_sq_idx(next);
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];

	printk("KERN_DEBUG ac_merged_requests\n");
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before(next->fifo_time, req->fifo_time)) {
			list_move(&req->queuelist, &next->queuelist);
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	   Delete next request.
	 */

	list_del_init(&next->queuelist);
	list_del_init(&next->flush.list);

	/*        
	   Update matrix info.
	 */
	atomic_dec(&mat->rq_num[idx]);
	if (atomic_read(&mat->rq_num[idx]) == 0) {
		atomic_set(&mat->flg[idx], ZERO);
	} else {
		atomic_set(&mat->flg[idx], PEND);
	}
}

static void ac_merged_request(struct request_queue *q,
			      struct request *req, int type)
{
}

static int ac_merge(struct request_queue *q, struct request **req,
		    struct bio *bio)
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

/*
 * return rq's adequate sublayer index 
 */
static inline int ac_get_sq_idx(struct request *rq)
{
	const int start_sector = blk_rq_pos(rq);
	const int end_sector = rq_end_sector(rq);
	int sl_sector_num = (int)(get_capacity(rq->rq_disk) / SQ_NUM);
	int start_idx = (int)(start_sector / sl_sector_num);
	int end_idx = (int)(end_sector / sl_sector_num);
#ifdef AC_DEBUG
	printk("KERN_DEBUG ac_get_sq_idx start_idx:%d\n", start_idx);
	printk("KERN_DEBUG ac_get_sq_idx end_idx:%d\n", end_idx);
#endif
	BUG_ON(start_idx >= SQ_NUM);
	BUG_ON(start_idx < 0);
	BUG_ON(end_idx >= SQ_NUM);
	BUG_ON(end_idx < 0);

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

	rq->fifo_time = jiffies + ad->fifo_expire[sync][data_dir];

	list_add_tail(&rq->queuelist, &ad->sq[idx].fifo_list[sync][data_dir]);
	list_add_tail(&rq->flush.list, &ad->deadline_list[sync][data_dir]);

	/*
	   update matrix information
	 */
	atomic_inc(&mat->rq_num[idx]);
	atomic_set(&mat->flg[idx], PEND);
}

static void ac_update_matrix(struct ac_data *ad, struct request *rq)
{
	const int sync = rq_is_sync(rq);
	const int data_dir = rq_data_dir(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];
	int i;

	atomic_dec(&mat->rq_num[idx]);
	if (data_dir == READ) {	/* read */
		if (atomic_read(&mat->rq_num[idx]) == 0) 
			atomic_set(&mat->flg[idx], ZERO);
		else
			atomic_set(&mat->flg[idx], END);
		
		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->flg[i]) == PEND) 
				return;
		}

		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->rq_num[i]) != 0) 
				atomic_set(&mat->flg[i], PEND);
		}
	} else {		/* write */
		if (atomic_read(&mat->rq_num[idx]) == 0) 
			atomic_set(&mat->flg[idx], ZERO);
		else {
			atomic_set(&mat->flg[idx], PROC);
			return;
		}

		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->flg[i]) == PEND
			    || atomic_read(&mat->flg[i]) == PROC) 
				return;
		}

		for (i = 0; i < SQ_NUM; i++) {
			if (atomic_read(&mat->rq_num[i]) != 0) 
				atomic_set(&mat->flg[i], PEND);
		}

	}
}

static struct request *ac_choose_expired_request(struct ac_data *ad)
{
	struct request *rq;
	rq = ac_expired_request(ad, SYNC, READ);
	if (rq) 
		return rq;

	rq = ac_expired_request(ad, SYNC, WRITE);
	if (rq) 
		return rq;

	rq = ac_expired_request(ad, ASYNC, READ);
	if (rq) 
		return rq;

	rq = ac_expired_request(ad, ASYNC, WRITE);
	if (rq) 
		return rq;

	/*
	  NO expired request.
	*/
	return NULL;
}

static struct request *ac_choose_request(struct ac_data *ad, int sync, int data_dir)
{
	struct ac_matrix *mat = &ad->matrix[sync][data_dir];
	struct request *rq = NULL;
	int i;

	if (list_empty(&ad->deadline_list[sync][data_dir]))
		return rq;


	for (i = ad->pre_idx; i < SQ_NUM; i++) {
		if (
			(data_dir == READ && ((atomic_read(&mat->rq_num[i]) != 0) && (atomic_read(&mat->flg[i]) != PEND))) ||
			(data_dir == WRITE && (atomic_read(&mat->flg[i]) == PROC))
			) 
			return rq_entry_fifo(ad->sq[i].fifo_list[sync][data_dir].next);
	}
	
	for (i = 0; i < ad->pre_idx; i++) {
		if (
			(data_dir == READ && ((atomic_read(&mat->rq_num[i]) != 0) && (atomic_read(&mat->flg[i]) != PEND))) ||
			(data_dir == WRITE && (atomic_read(&mat->flg[i]) == PROC))
			) 
			return rq_entry_fifo(ad->sq[i].fifo_list[sync][data_dir].next);
	}


	for (i = 0; i < SQ_NUM; i++) {
		if (atomic_read(&mat->flg[i]) == PEND) 
			return rq_entry_fifo(ad->sq[i].fifo_list[sync][data_dir].next);
	}

	return rq;
}

static void ac_dispatch_request(struct ac_data *ad, struct request *rq)
{
	list_del_init(&rq->queuelist);
	list_del_init(&rq->flush.list);
	elv_dispatch_add_tail(rq->q, rq);
	ac_update_matrix(ad, rq);

	if (rq_data_dir(rq))
		ad->starved = 0;
	else
		ad->starved++;

	ad->pre_idx = ac_get_sq_idx(rq);
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
		printk("%2d",
		       atomic_read(&ad->matrix[sync][data_dir].rq_num[i]));
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
	printk("---KERN_DEBUG ac_dispatch_reqests()---\n");
	printk("KERN_DEBUG async read list%d\n",
	       list_empty(&ad->deadline_list[ASYNC][READ]));
	ac_display_matrix(ad, ASYNC, READ);
	printk("KERN_DEBUG async write list%d\n",
	       list_empty(&ad->deadline_list[ASYNC][WRITE]));
	ac_display_matrix(ad, ASYNC, WRITE);
	printk("KERN_DEBUG sync read list%d\n",
	       list_empty(&ad->deadline_list[SYNC][READ]));
	ac_display_matrix(ad, SYNC, READ);
	printk("KERN_DEBUG sync write list%d\n",
	       list_empty(&ad->deadline_list[SYNC][WRITE]));
	ac_display_matrix(ad, SYNC, WRITE);
#endif

	if (ad->batched > ad->fifo_batch) {
		ad->batched = 0;
		rq = ac_choose_expired_request(ad);
	}

	if (!rq) {
		if (ad->starved > ad->writes_starved)
			data_dir = WRITE;

		rq = ac_choose_request(ad, ASYNC, data_dir);
		
		if (!rq)
			rq = ac_choose_request(ad, SYNC, data_dir);

		if (!rq)
			rq = ac_choose_request(ad, ASYNC, !data_dir);

		if(!rq)
			rq = ac_choose_request(ad, SYNC, !data_dir);

		if (!rq)
			return 0;
	}
#ifdef AC_DEBUG
	printk("KERN_DEBUG dispatch rq:%p idx:%d\n", rq, ac_get_sq_idx(rq));
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

	INIT_LIST_HEAD(&ad->deadline_list[ASYNC][READ]);
	INIT_LIST_HEAD(&ad->deadline_list[ASYNC][WRITE]);
	INIT_LIST_HEAD(&ad->deadline_list[SYNC][READ]);
	INIT_LIST_HEAD(&ad->deadline_list[SYNC][WRITE]);

	for (i = 0; i < SQ_NUM; i++) {
		INIT_LIST_HEAD(&ad->sq[i].fifo_list[ASYNC][READ]);
		INIT_LIST_HEAD(&ad->sq[i].fifo_list[SYNC][READ]);
		INIT_LIST_HEAD(&ad->sq[i].fifo_list[ASYNC][WRITE]);
		INIT_LIST_HEAD(&ad->sq[i].fifo_list[SYNC][WRITE]);
	}

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

	for (i = 0; i < SQ_NUM; i++) {
		BUG_ON(!list_empty(&ad->sq[i].fifo_list[ASYNC][READ]));
		BUG_ON(!list_empty(&ad->sq[i].fifo_list[ASYNC][WRITE]));
		BUG_ON(!list_empty(&ad->sq[i].fifo_list[SYNC][READ]));
		BUG_ON(!list_empty(&ad->sq[i].fifo_list[SYNC][WRITE]));
	}

	BUG_ON(!list_empty(&ad->deadline_list[ASYNC][READ]));
	BUG_ON(!list_empty(&ad->deadline_list[ASYNC][WRITE]));
	BUG_ON(!list_empty(&ad->deadline_list[SYNC][READ]));
	BUG_ON(!list_empty(&ad->deadline_list[SYNC][WRITE]));

	kfree(ad);
}

static struct elevator_type iosched_ac = {
	.ops = {
		.elevator_merge_fn = ac_merge,
		.elevator_merged_fn = ac_merged_request,
		.elevator_merge_req_fn = ac_merged_requests,
		.elevator_dispatch_fn = ac_dispatch_requests,
		.elevator_add_req_fn = ac_add_request,
		.elevator_init_fn = ac_init_queue,
		.elevator_exit_fn = ac_exit_queue,
		},

	.elevator_name = "aacp",
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
MODULE_DESCRIPTION("another ac(alleviate conflict) plus IO scheduler");
MODULE_VERSION("0.1");
