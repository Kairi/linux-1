/*
 * Alleviate Conflict i/o scheduler.
 *
 * Copyright (C) 2015 Kairi <kairi199088@gmail.com>
 */
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

static const int sync_read_expire = HZ / 2;	/* max time before a syn read is submitted. */
static const int sync_write_expire = 2 * HZ;	/* ditto for sync writes, thesee limits are SOFT! */
static const int async_read_expire = 4 * HZ;	/* ditto for async read, thesee limits are SOFT! */
static const int async_write_expire = 16 * HZ;	/* ditto for async writes, thesee limits are SOFT! */
static const int writes_starved = 4;	/* max times reads can starve a write */
static const int fifo_batch = 8;	/* # of sequential requests treated as one by the above parameters. For throghputs. */

/* for debugging parameter.*/
/* #define AC_DEBUG */

#define FLASH_CHIP_NUM 8
#define SQ_NUM 8
#define rq_entry_timeout(ptr) list_entry((ptr), struct request, timeout_list)
#define rq_timeout_clear(rq)	list_del_init(&(rq)->timeout_list)

struct sub_queue {
	struct list_head fifo_list[2][2];
};

/*
 * for dispatch sort information 
 * See Documentation/atomic_ops.txt
 */
struct ac_matrix {
	atomic_t rq_num[SQ_NUM];
	atomic_t flg[SQ_NUM];
};

/* runtime data */
struct ac_data {
	struct request_queue *queue;
	struct sub_queue sq[SQ_NUM];

	struct ac_matrix matrix[2];

	struct list_head deadline_list[2][2];

	int batched;
	int starved;
	int fifo_batch;
	int pre_idx;

	int fifo_expire[2][2];
	int writes_starved;
};

static inline int ac_get_sq_idx(struct request *rq);
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir);

/*
  reg to .elevator_merge_req_fn
*/
static void
ac_merged_requests(struct request_queue *q, struct request *rq,
		   struct request *next)
{
	struct ac_data *ad = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(rq);

	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist)
	    && !list_empty(&rq->timeout_list)
	    && !list_empty(&next->timeout_list)) {
		if (time_before(next->fifo_time, rq->fifo_time)) {
			list_move(&rq->queuelist, &next->queuelist);
			list_move(&rq->timeout_list, &next->timeout_list);
			rq->fifo_time = next->fifo_time;
		}
	}

	/*
	   Delete next request.
	 */
	rq_fifo_clear(next);
	rq_timeout_clear(next);
}

/*
  reg to .elevator_merged_fn
*/
static void ac_merged_request(struct request_queue *q,
			      struct request *rq, int type)
{
	/* struct ac_data *ad = q->elevator->elevator_data; */
	/* const int data_dir = rq_data_dir(rq); */
	/* const int next_idx = ac_get_sq_idx(next); */
	/* struct ac_matrix *mat = &ad->matrix[data_dir]; */
	return;
	/*
	 * if the merge was a front merge, we need to update matrix.
	 */
	/* if (type == ELEVAOTR_FRONT_MERGE) { */
	/*      /\*         */
	/*        Update matrix info. */
	/*      *\/ */
	/*      atomic_dec(&mat->rq_num[next_idx]); */
	/*      if (atomic_read(&mat->rq_num[next_idx]) == 0) { */
	/*              atomic_set(&mat->flg[next_idx], ZERO); */
	/*      } else { */
	/*              atomic_set(&mat->flg[next_idx], PEND); */
	/*      } */
	/* }  */
}

/*
  reg to .elevator_merge_fn
*/
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

	rq = rq_entry_timeout(list->next);

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

	BUG_ON(start_idx >= SQ_NUM);
	BUG_ON(start_idx < 0);
	BUG_ON(end_idx >= SQ_NUM);
	BUG_ON(end_idx < 0);

	return start_idx;
}

/*
 * registerd to elevator_add_req_fn
 */
static void ac_add_request(struct request_queue *q, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	const int sync = rq_is_sync(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_data *ad = q->elevator->elevator_data;
	struct ac_matrix *mat = &ad->matrix[data_dir];
	/*
	 * Add request to the proper fifo list and set its
	 * expire time.
	 */

#ifdef AC_DEBUG
	printk("KERN_DEBUG insert rq is data_dir:%d, sync:%d idx:%d\n",
	       data_dir, sync, idx);
#endif

	rq->fifo_time = jiffies + ad->fifo_expire[sync][data_dir];

	list_add_tail(&rq->queuelist, &ad->sq[idx].fifo_list[sync][data_dir]);
	list_add_tail(&rq->timeout_list, &ad->deadline_list[sync][data_dir]);

	/*
	 * update matrix information
	 */
	atomic_inc(&mat->rq_num[idx]);
	atomic_set(&mat->flg[idx], PEND);
}

/*
 * update matrix information.
 */
static inline void ac_update_matrix(struct ac_data *ad, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	const int idx = ac_get_sq_idx(rq);
	struct ac_matrix *mat = &ad->matrix[data_dir];
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

static inline struct request *ac_choose_expired_request(struct ac_data *ad)
{
	struct request *rq;
	/*
	 * Check expired requests.
	 * Read requests have priority over write.
	 * Synchronous requests have priority over asynchronous.
	 */
	rq = ac_expired_request(ad, SYNC, READ);
	if (rq)
		return rq;

	rq = ac_expired_request(ad, ASYNC, READ);
	if (rq)
		return rq;

	rq = ac_expired_request(ad, SYNC, WRITE);
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

static inline struct request *ac_choose_request(struct ac_data *ad, int sync,
						int data_dir)
{
	struct ac_matrix *mat = &ad->matrix[data_dir];
	struct ac_matrix *another_mat = &ad->matrix[!data_dir];
	struct request *rq = NULL;
	int i;

	/*
	 * Retrieve request from available fifo list.
	 * Read requests have priority over write.
	 * Synchronous requests have priority over asynchronous.
	 */
	if (list_empty(&ad->deadline_list[sync][data_dir]))
		return rq;

	for (i = ad->pre_idx; i < SQ_NUM; i++) {
		if (data_dir == READ) {
			if ((atomic_read(&mat->rq_num[i]) != 0) && (atomic_read(&mat->flg[i]) == PEND) && (atomic_read(&another_mat->flg[i]) != PROC)) {	/* alleviate cross penalty */
				if (!list_empty
				    (&ad->sq[i].fifo_list[sync][data_dir])) {
					return rq_entry_fifo(ad->
							     sq[i].fifo_list
							     [sync]
							     [data_dir].next);
				}
			}
		}

		if (data_dir == WRITE) {
			if (atomic_read(&mat->flg[i]) == PROC)
				if (!list_empty
				    (&ad->sq[i].fifo_list[sync][data_dir])) {
					return rq_entry_fifo(ad->
							     sq[i].fifo_list
							     [sync]
							     [data_dir].next);
				}
		}
	}

	for (i = 0; i < ad->pre_idx; i++) {
		if (data_dir == READ) {
			if ((atomic_read(&mat->rq_num[i]) != 0) && (atomic_read(&mat->flg[i]) == PEND) && (atomic_read(&another_mat->flg[i]) != PROC)) {	/* alleviate cross penalty */
				if (!list_empty
				    (&ad->sq[i].fifo_list[sync][data_dir])) {
					return rq_entry_fifo(ad->
							     sq[i].fifo_list
							     [sync]
							     [data_dir].next);
				}
			}
		}

		if (data_dir == WRITE) {
			if (atomic_read(&mat->flg[i]) == PROC) {
				if (!list_empty
				    (&ad->sq[i].fifo_list[sync][data_dir])) {
					return rq_entry_fifo(ad->
							     sq[i].fifo_list
							     [sync]
							     [data_dir].next);
				}
			}
		}
	}

	if (data_dir == READ) {	// other entry is empty. PEND PROC pair
		printk("KERN_DEBUG collision req reluctantly\n");
		/* collision req reluctantly */
		ac_display_matrix(ad, SYNC, READ);

		for (i = 0; i < SQ_NUM; i++) {
			if ((atomic_read(&mat->flg[i]) == PEND) &&
			    (atomic_read(&another_mat->flg[i]) == PROC)) {
				if (!list_empty
				    (&ad->sq[i].fifo_list[sync][data_dir])) {
					return rq_entry_fifo(ad->
							     sq[i].fifo_list
							     [sync]
							     [data_dir].next);
				}
			}
		}
	}

	for (i = 0; i < SQ_NUM; i++) {
		if (atomic_read(&mat->flg[i]) == PEND) {
			if (!list_empty(&ad->sq[i].fifo_list[sync][data_dir])) {
				return rq_entry_fifo(ad->
						     sq[i].fifo_list[sync]
						     [data_dir].next);
			}
		}
	}

	return rq;
}

static inline void ac_dispatch_request(struct ac_data *ad, struct request *rq)
{
	/*
	 * Remove the request from the fifo list
	 * and dispatch it.
	 */
	ad->pre_idx = ac_get_sq_idx(rq);

	ac_update_matrix(ad, rq);

	if (rq_data_dir(rq))
		ad->starved = 0;
	else
		ad->starved++;

	rq_fifo_clear(rq);
	rq_timeout_clear(rq);
	elv_dispatch_add_tail(rq->q, rq);
}

/* for debugging function */
static void ac_display_matrix(struct ac_data *ad, int sync, int data_dir)
{
	int i;
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", i);
	}
	printk("\n");
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", atomic_read(&ad->matrix[data_dir].rq_num[i]));
	}
	printk("\n");
	for (i = 0; i < SQ_NUM; i++) {
		printk("%2d", atomic_read(&ad->matrix[data_dir].flg[i]));
	}
	printk("\n");
}

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

	/*
	 * Retrieve any expired request after a batch of sequential requests.
	 */
	if (ad->batched > ad->fifo_batch) {
		ad->batched = 0;
		rq = ac_choose_expired_request(ad);
#ifdef AC_DEBUG
		printk("KERN_DEBUG exist expired request. idx:%d\n",
		       ac_get_sq_idx(rq));
#endif
	}

	/* Retrieve request */
	if (!rq) {
		if (ad->starved > ad->writes_starved)
			data_dir = WRITE;

		rq = ac_choose_request(ad, SYNC, data_dir);

		if (!rq)
			rq = ac_choose_request(ad, ASYNC, data_dir);

		if (!rq)
			rq = ac_choose_request(ad, SYNC, !data_dir);

		if (!rq)
			rq = ac_choose_request(ad, ASYNC, !data_dir);

		if (!rq)
			return 0;
	}
#ifdef AC_DEBUG
	printk("KERN_DEBUG dispatch rq:%p idx:%d data_dir:%d\n", rq,
	       ac_get_sq_idx(rq), rq_data_dir(rq));
#endif

	/* Dispatch request */
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
		atomic_set(&ad->matrix[READ].rq_num[i], 0);
		atomic_set(&ad->matrix[READ].flg[i], 0);
		atomic_set(&ad->matrix[WRITE].rq_num[i], 0);
		atomic_set(&ad->matrix[WRITE].flg[i], 0);
	}

	ad->batched = 0;
	ad->starved = 0;
	ad->fifo_batch = fifo_batch;
	ad->writes_starved = writes_starved;

	ad->fifo_expire[SYNC][READ] = sync_read_expire;
	ad->fifo_expire[SYNC][WRITE] = sync_write_expire;
	ad->fifo_expire[ASYNC][READ] = async_read_expire;
	ad->fifo_expire[ASYNC][WRITE] = async_write_expire;

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
	/* Free structure */
	kfree(ad);
}

/*
  For sysfs code.
 */
static ssize_t ac_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static ssize_t ac_var_store(int *var, const char *page, size_t count)
{
	char *p = (char *)page;

	*var = simple_strtol(p, &p, 10);
	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct ac_data *ad = e->elevator_data;			\
	int __data = __VAR;						\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return ac_var_show(__data, (page));			\
}
SHOW_FUNCTION(ac_sync_read_expire_show, ad->fifo_expire[SYNC][READ], 1);
SHOW_FUNCTION(ac_sync_write_expire_show, ad->fifo_expire[SYNC][WRITE], 1);
SHOW_FUNCTION(ac_async_read_expire_show, ad->fifo_expire[ASYNC][READ], 1);
SHOW_FUNCTION(ac_async_write_expire_show, ad->fifo_expire[ASYNC][WRITE], 1);
SHOW_FUNCTION(ac_fifo_batch_show, ad->fifo_batch, 0);
SHOW_FUNCTION(ac_writes_starved_show, ad->writes_starved, 0);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct ac_data *ad = e->elevator_data;			\
	int __data;							\
	int ret = ac_var_store(&__data, (page), count);		\
	if (__data < (MIN))						\
		__data = (MIN);						\
	else if (__data > (MAX))					\
		__data = (MAX);						\
	if (__CONV)							\
		*(__PTR) = msecs_to_jiffies(__data);			\
	else								\
		*(__PTR) = __data;					\
	return ret;							\
}
STORE_FUNCTION(ac_sync_read_expire_store, &ad->fifo_expire[SYNC][READ], 0,
	       INT_MAX, 1);
STORE_FUNCTION(ac_sync_write_expire_store, &ad->fifo_expire[SYNC][WRITE], 0,
	       INT_MAX, 1);
STORE_FUNCTION(ac_async_read_expire_store, &ad->fifo_expire[ASYNC][READ], 0,
	       INT_MAX, 1);
STORE_FUNCTION(ac_async_write_expire_store, &ad->fifo_expire[ASYNC][WRITE], 0,
	       INT_MAX, 1);
STORE_FUNCTION(ac_fifo_batch_store, &ad->fifo_batch, 0, INT_MAX, 0);
STORE_FUNCTION(ac_writes_starved_store, &ad->writes_starved, 0, INT_MAX, 0);
#undef STORE_FUNCTION
// S_IRUGIOS|S_IWUSR means sysfs be enable to writed and readed when running
#define DD_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, ac_##name##_show, \
				      ac_##name##_store)

static struct elv_fs_entry ac_attrs[] = {
	DD_ATTR(sync_read_expire),
	DD_ATTR(sync_write_expire),
	DD_ATTR(async_read_expire),
	DD_ATTR(async_write_expire),
	DD_ATTR(fifo_batch),
	DD_ATTR(writes_starved),
	__ATTR_NULL
};

static struct elevator_type iosched_ac = {
	.ops = {
//		.elevator_merge_fn = ac_merge,
//              .elevator_merged_fn = ac_merged_request,
//		.elevator_merge_req_fn = ac_merged_requests,
		.elevator_dispatch_fn = ac_dispatch_requests,
		.elevator_add_req_fn = ac_add_request,
		.elevator_init_fn = ac_init_queue,
		.elevator_exit_fn = ac_exit_queue,
		},

	.elevator_attrs = ac_attrs,
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
MODULE_AUTHOR("Kairi");
MODULE_DESCRIPTION("AC(Alleviate Conflict) IO scheduler");
MODULE_VERSION("0.2");
