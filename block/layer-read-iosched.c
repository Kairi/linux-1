/*
 *  LR(Layer Read) i/o scheduler.
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

static const int read_expire = HZ / 2;	/* max time before a read is submitted. */
static const int write_expire = 5 * HZ;	/* ditto for writes, these limits are SOFT! */
static const int writes_starved = 2;	/* max times reads can starve a write */
static const int batch_read = 16;
static const int batch_write = 8;

//static const int fifo_batch = 16;	/* # of sequential requests treated as one
//					   by the above parameters. For throughput. */
static const unsigned long device_byte_size = ((long)1024 * 1024 * 1024 * 64);

#define FLASH_CHIP_NUM 8
#define SL_NUM FLASH_CHIP_NUM * 2

struct sub_layer {
	struct list_head read_fifo_list;
	struct rb_root read_sort_list;
};

struct lr_data {
	struct list_head write_fifo_list;	// this is used for WRITE ONLY
	struct sub_layer sl[FLASH_CHIP_NUM * 2];
	/*
	 * next in sort order. read, write or both are NULL
	 */
	struct request *next_rq[2];
	unsigned int batching;	/* number of sequential requests made */
	sector_t last_sector;	/* head position */

	/*
	 * settings that change how the i/o scheduler behaves
	 */
	int mode;		// write or read 
	int fifo_expire[2];	// limit time
	int batch_read;
	int batch_write;
	int cnt;		// dispatch in same layer count
	int pre_idx;		// pre dispatched sub layer index
	int pre_rq_dir;
};

static void lr_move_request(struct lr_data *, struct request *);
static inline int lr_get_sl_idx(struct request *rq);

static inline struct rb_root *lr_rb_root(struct lr_data *ld, struct request *rq)
{
	int idx;
	const int data_dir = rq_data_dir(rq);

	if (data_dir == READ) {
		idx = lr_get_sl_idx(rq);
		return &ld->sl[idx].read_sort_list;
	}

	return NULL;
}

/*
 * when `rq' is read ,get the request after `rq' in sector-sorted order.
 * when `rq' is write ,get the request after `rq' in time-sorted order.
 * when all queue is empty, return NULL.
 */
static inline struct request *lr_latter_request(struct lr_data *ld,
						struct request *rq)
{
	struct rb_node *node = NULL;
	const int data_dir = rq_data_dir(rq);
	int idx, i;

	printk("KERN_DEBUG latter_rq\n");
	if (data_dir == WRITE) {
//		if (!list_empty(&ld->write_fifo_list))
		if(rq->queuelist.next == &ld->write_fifo_list)
			return NULL;
		return rq_entry_fifo(ld->write_fifo_list.next);
	} else {
		struct rb_node *tmp_node = &rq->rb_node;
		do {
			printk("KERN_DEBUG original rq:%p", rq);
			printk("KERN_DEBUG rb_entry_rq(tmp_node):%p", rb_entry_rq(tmp_node));
		}while(tmp_node = rb_next(tmp_node));
		
		node = rb_next(&rq->rb_node);
		

		if (node) {
			printk("KERN_DEBUG latter_request first return\n");
			return rb_entry_rq(node);
		}

		return NULL;

		idx = lr_get_sl_idx(rq);
		for (i = idx + 1; i < SL_NUM; i++) {	// if rq's sub layer is empty
			node = rb_first(&ld->sl[idx].read_sort_list);
			if (node) {
				printk("KERN_DEBUG latter_request first return\n");
				return rb_entry_rq(node);
			}
		}

		for (i = 0; i < idx; i++) {
			node = rb_first(&ld->sl[idx].read_sort_list);
			if (node) {
				printk("KERN_DEBUG latter_request first return\n");
				return rb_entry_rq(node);
			}
		}
	}

	return NULL;

}

/*
 * register ONLY READ rq to adequate tree
 */
static void lr_add_rq_rb(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	struct rb_root *root;

	if (data_dir == READ) {
		root = lr_rb_root(ld, rq);
		elv_rb_add(root, rq);
	}
}

/*
 * unregister read rq from tree and change next rq
 */
static inline void lr_del_rq_rb(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	struct rb_root *root;
	int idx;

	if (ld->next_rq[data_dir] == rq)
		ld->next_rq[data_dir] = lr_latter_request(ld, rq);

	if (data_dir == WRITE)
		return;

	idx = lr_get_sl_idx(rq);
	root = lr_rb_root(ld, rq);
	if (root) {
		elv_rb_del(root, rq);
	}
}

/*
 * return rq's adequate sublayer index
 */
static inline int lr_get_sl_idx(struct request *rq)
{
	const int sector = blk_rq_pos(rq);
	int idx = (int)(sector / (device_byte_size / (2 * FLASH_CHIP_NUM) + 1)); //NEED check
	/* printk */
	/*     ("KERN_DEBUG sector:%d DEVICE_BYTE_SIZE:%10lu FLASH_CHIP_NUM:%d index:%d\n", */
	/*      sector, device_byte_size, FLASH_CHIP_NUM, idx); */
	return idx;
}

/*
 * add rq to rbtree and fifo
 * registerd to elevator_add_req_fn
 */
static void lr_add_request(struct request_queue *q, struct request *rq)
{
	struct lr_data *ld = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(rq);
	int idx;

	rq->fifo_time = jiffies + ld->fifo_expire[data_dir];

	if (data_dir == WRITE) {
		printk("KERN_DEBUG lr_add_request(insert write)\n");
		list_add_tail(&rq->queuelist, &ld->write_fifo_list);
	} else {
		printk("KERN_DEBUG lr_add_request(insert read)\n");
		idx = lr_get_sl_idx(rq);
		lr_add_rq_rb(ld, rq);
//		elv_rb_add(&ld->sl[idx].read_sort_list, rq);	// insert sorted list
		list_add_tail(&rq->queuelist, &ld->sl[idx].read_fifo_list);	// insert fifo list
	}
}

static int lr_allow_merge(struct request_queue *q, struct request *rq, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}

/*
 * remove request
 */
static void lr_remove_request(struct request_queue *q, struct request *rq)
{
	struct lr_data *ld = q->elevator->elevator_data;

	rq_fifo_clear(rq);
	lr_del_rq_rb(ld, rq);
}

/*
  change from deadline to this scheduler do no merge.
 */
static int
lr_merge(struct request_queue *q, struct request **req, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}

static void lr_merged_request(struct request_queue *q,
			      struct request *req, int type)
{
	struct lr_data *ld = q->elevator->elevator_data;
	struct rb_root *root;

      printk("KERN_DEBUG lr_merged_request\n");
	/*
	 * if the merge was a front merge, we need to reposition request
	 */
	if (type == ELEVATOR_FRONT_MERGE) {
		root = lr_rb_root(ld, req);
		if (root != NULL) {
			elv_rb_del(lr_rb_root(ld, req), req);
			lr_add_rq_rb(ld, req);
		}
	}
}

static void
lr_merged_requests(struct request_queue *q, struct request *req,
		   struct request *next)
{
      printk("KERN_DEBUG lr_merged_requests\n");
	/*
	 * if next expires before rq, assign its expire time to rq
	 * and move into next position (next will be deleted) in fifo
	 */
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before(next->fifo_time, req->fifo_time)) {
			list_move(&req->queuelist, &next->queuelist);	// req->queuelist:the entry to move, next->queuelist:the head that will precede our entry
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	 * kill knowledge of next, this one is a goner
	 */
	lr_remove_request(q, next);
}

/*
 * move request from scheduler queue to dispatch queue.
 */
static inline void lr_move_to_dispatch(struct lr_data *ld, struct request *rq)
{
	struct request_queue *q = rq->q;

	lr_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
}

/*
 * move an entry to dispatch queue
 */
static void lr_move_request(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);

	ld->next_rq[READ] = NULL;
	ld->next_rq[WRITE] = NULL;
	ld->next_rq[data_dir] = lr_latter_request(ld, rq);

	ld->last_sector = rq_end_sector(rq);

	/*
	 * take it off the sort and fifo list, move
	 * to dispatch queue
	 */
	lr_move_to_dispatch(ld, rq);
}

/*
 * lr_check_fifo returns 0 if there are no expired requests on the fifo(read and write),
 * 1 otherwise. Requires !list_empty(&ld->fifo_list[data_dir])
 UNNECCESARY
 */
static inline int lr_check_fifo(struct lr_data *ld, int ddir)
{
	struct request *rq = NULL;
	int i;

	if (ddir == WRITE) {
		if (!list_empty(&ld->write_fifo_list))
			rq = rq_entry_fifo(ld->write_fifo_list.next);
	} else {
		for (i = 0; i < SL_NUM; i++) {
			rq = rq_entry_fifo(ld->sl[i].read_fifo_list.next);
			if (rq)
				break;
		}
	}

	if (!rq)
		return 0;
	/*
	 * rq is expired!
	 */
	if (time_after_eq(jiffies, rq->fifo_time))
		return 1;

	return 0;
}

/*
 * return  request in sub layer
 * when sub layer is empty or dispatch 8 times in same layer, function retuns  NULL.
 */
static inline struct request *lr_check_sub_layer(struct lr_data *ld)
{
	struct request *first_rq_in_fifo = NULL;
	struct request *first_rq_in_tree = NULL;
	int i;

	for (i = ld->pre_idx; i < SL_NUM; i++) {
		if (!list_empty(&ld->sl[i].read_fifo_list)) {
			printk("KERN_DEBUG there is rq in %d th sub layer\n", i);
			printk("KERN_DEBUG it's rq pointer is %p\n", rq_entry_fifo(ld->sl[i].read_fifo_list.next));
			if (ld->pre_idx == i) {
				ld->cnt++;
			}

			if (ld->cnt >= 8) {
				ld->cnt = 0;
				continue;
			}

			first_rq_in_fifo =
			    rq_entry_fifo(ld->sl[i].read_fifo_list.next);
			ld->pre_idx = i;
			break;
		}
	}

	if (!first_rq_in_fifo) {
		for (i = 0; i < ld->pre_idx; i++) {
			if (!list_empty(&ld->sl[i].read_fifo_list)) {
				printk
				    ("KERN_DEBUG there is rq in %d th sub layer",
				     i);
				ld->cnt = 1;
				ld->pre_idx = i;
				first_rq_in_fifo =
				    rq_entry_fifo(ld->sl[i].read_fifo_list.
						  next);
				break;
			}
		}
		if (!first_rq_in_fifo)
			return NULL;
	}

	if (time_after_eq(jiffies, first_rq_in_fifo->fifo_time))	// rq is expired!
		return first_rq_in_fifo;	// if expired

	first_rq_in_tree = rb_entry_rq(rb_first(&ld->sl[i].read_sort_list));

	if (ld->batching > ld->batch_read)
		return NULL;

	return first_rq_in_tree;	// rq is NOT expired!
}

static void lr_display_write_requests(struct lr_data *ld)
{
	struct request *__rq;
	int cnt = 0;

	list_for_each_entry(__rq, &ld->write_fifo_list, queuelist) {
		printk("write rq in fifo:%p\n", __rq);
		cnt++;
	}
	printk("there is %d rq in write fifo\n", cnt);
}

static void lr_display_read_requests(struct lr_data *ld)
{
	struct request *__rq;
	int i, cnt = 0;

	for (i = 0; i < SL_NUM; i++) {
//		printk("KERN_DEBUG RB_EMPTY_ROOT:%d\n", RB_EMPTY_ROOT(&ld->sl[i].read_sort_list));
		list_for_each_entry(__rq, &ld->sl[i].read_fifo_list, queuelist) {
			printk("read rq in fifo:%p\n", __rq);
			cnt++;
		}
	}
	printk("KERN_DEBUGthere is %d rq in read fifo\n", cnt);
}

/*
 * lr_dispatch_requests selects the best request according to
 * read/write expire, fifo_batch, etc
 * when request is empty, return 0 else return 1.
 */
static int lr_dispatch_requests(struct request_queue *q, int force)
{
	struct lr_data *ld = q->elevator->elevator_data;
	struct request *rq = NULL;
	struct request *reads  = ld->next_rq[READ];
	struct request *writes  = ld->next_rq[WRITE];
	int expire = (ld->batching > ld->batch_read || ld->batching > ld->batch_write);
	int over = (ld->cnt > 8);

//	lr_display_read_requests(ld);
//	lr_display_write_requests(ld);
	printk("KERN_DEBUG start dispatch\n");
	printk("KERN_DEBUG reads:%p writes:%p expire:%d over:%d\n",reads, writes, expire, over);
	if (ld->next_rq[READ]) {
		rq = ld->next_rq[READ];
		if (ld->batching < ld->batch_read) {
			if(ld->cnt < 8) {
				goto dispatch_request;
			} else {
				ld->pre_idx = lr_get_sl_idx(rq) + 1;
				ld->cnt = 0;
				goto dispatch_layer;
			}
		} else {
			ld->mode = WRITE;
			goto dispatch_writes;
		}
	}

	/* if(ld->next_rq[READ]) */
	/* 	rq = ld->next_rq[READ]; */

	if (!rq) {
		if (ld->mode == WRITE) {
			goto dispatch_writes;
		} else {
			goto dispatch_layer;
		}
	} 

dispatch_layer:
	rq = lr_check_sub_layer(ld);
	if (!rq) {		// all sub layer is empty or exceed batch_read
		goto dispatch_writes;
	} else {
		goto dispatch_request;
	}

dispatch_writes:	// dipatch write request
	if (ld->next_rq[WRITE])
		rq = ld->next_rq[WRITE];

	if (!rq && !list_empty(&ld->write_fifo_list))
		rq = rq_entry_fifo(ld->write_fifo_list.next);

	if (rq && ld->batching < ld->batch_write) {
		goto dispatch_request;
	}

	if (!rq || ld->batching > ld->batch_write) {
		ld->mode = READ;
		return 0;
	}
	return 0; // no request
dispatch_request:
	if (rq) {
		if(ld->pre_rq_dir == rq_data_dir(rq)) {
			ld->batching++;
		} else {
			ld->pre_rq_dir  = rq_data_dir(rq);
			ld->batching = 1;
		}
		lr_move_request(ld, rq);
		printk("KERN_DEBUG end lr_move_request dispatched %p rq. ld->next_rq[READ]:%p, ld->next_rq[WRITE]:%p",rq, ld->next_rq[READ], ld->next_rq[WRITE]);
	}
	

	return 1;
}

static void lr_exit_queue(struct elevator_queue *e)
{
	struct lr_data *ld = e->elevator_data;
	int i;

	BUG_ON(!list_empty(&ld->write_fifo_list));
	for (i = 0; i < FLASH_CHIP_NUM * 2; i++) {
		BUG_ON(!list_empty(&ld->sl[i].read_fifo_list));
	}

	kfree(ld);
}

/*
 * initialize elevator private data (lr_data).
 */
static int lr_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct lr_data *ld;
	struct elevator_queue *eq;
	int i;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	ld = kzalloc_node(sizeof(*ld), GFP_KERNEL, q->node);
	if (!ld) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = ld;

	INIT_LIST_HEAD(&ld->write_fifo_list);	// for write only

	for (i = 0; i < FLASH_CHIP_NUM * 2; i++) { // for read only
		ld->sl[i].read_sort_list = RB_ROOT;
		INIT_LIST_HEAD(&ld->sl[i].read_fifo_list);
	}
	ld->batching = 0;
	ld->mode = READ;
	ld->fifo_expire[READ] = read_expire;
	ld->fifo_expire[WRITE] = write_expire;
	ld->batch_read = batch_read;
	ld->batch_write = batch_write;
	ld->cnt = 0;
	ld->pre_idx = 0;
	
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static ssize_t lr_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static ssize_t lr_var_store(int *var, const char *page, size_t count)
{
	char *p = (char *)page;

	*var = simple_strtol(p, &p, 10);
	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct lr_data *dd = e->elevator_data;			\
	int __data = __VAR;						\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return lr_var_show(__data, (page));			\
}
SHOW_FUNCTION(lr_read_expire_show, dd->fifo_expire[READ], 1);
SHOW_FUNCTION(lr_write_expire_show, dd->fifo_expire[WRITE], 1);
SHOW_FUNCTION(lr_batch_read_show, dd->batch_read, 1);
SHOW_FUNCTION(lr_batch_write_show, dd->batch_write, 1);

#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct lr_data *dd = e->elevator_data;			\
	int __data;							\
	int ret = lr_var_store(&__data, (page), count);		\
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
STORE_FUNCTION(lr_read_expire_store, &dd->fifo_expire[READ], 0, INT_MAX, 1);
STORE_FUNCTION(lr_write_expire_store, &dd->fifo_expire[WRITE], 0, INT_MAX, 1);
STORE_FUNCTION(lr_batch_read_store, &dd->batch_read, 0, INT_MAX, 1);
STORE_FUNCTION(lr_batch_write_store, &dd->batch_write, 0, INT_MAX, 1);
#undef STORE_FUNCTION

#define DD_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, lr_##name##_show, \
				      lr_##name##_store)

static struct elv_fs_entry lr_attrs[] = {
	DD_ATTR(read_expire),
	DD_ATTR(write_expire),
	DD_ATTR(batch_read),
	DD_ATTR(batch_write),
	__ATTR_NULL
};

static struct elevator_type iosched_lr = {
	.ops = {
//		.elevator_merge_fn = lr_merge,
//		.elevator_merged_fn = lr_merged_request,
//		.elevator_merge_req_fn = lr_merged_requests,
		.elevator_allow_merge_fn = lr_allow_merge,
		.elevator_dispatch_fn = lr_dispatch_requests,
		.elevator_add_req_fn = lr_add_request,
		.elevator_former_req_fn = elv_rb_former_request,
		.elevator_latter_req_fn = elv_rb_latter_request,
		.elevator_init_fn = lr_init_queue,
		.elevator_exit_fn = lr_exit_queue,
		},

	.elevator_attrs = lr_attrs,
	.elevator_name = "lr",
	.elevator_owner = THIS_MODULE,
};

static int __init lr_init(void)
{
	return elv_register(&iosched_lr);
}

static void __exit lr_exit(void)
{
	elv_unregister(&iosched_lr);
}

module_init(lr_init);
module_exit(lr_exit);

MODULE_AUTHOR("Kairi OKUMURA");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("lr(layer read) IO scheduler");
