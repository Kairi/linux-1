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
// cat /sys/class/block/sda/size
static const unsigned long device_byte_size = ((long)1024 * 1024  * 128);

#define FLASH_CHIP_NUM 8
#define SL_NUM FLASH_CHIP_NUM * 2

struct sub_layer {
	struct list_head read_fifo_list;
	struct rb_root read_sort_list;
};

struct lr_data {
	struct list_head write_fifo_list;
	struct sub_layer sl[FLASH_CHIP_NUM * 2];

	struct request *next_rq[2];
	unsigned int batching;	/* number of sequential requests made */
	sector_t last_sector;	/* head position */

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
	/* printk("KERN_DEBUG ****************************************\n"); */
	/* printk("KERN_DEBUG start add rq\n"); */
	/* printk("KERN_DEBUG ****************************************\n"); */

	rq->fifo_time = jiffies + ld->fifo_expire[data_dir];

	if (data_dir == WRITE) {
		printk("KERN_DEBUG lr_add_request(insert write)\n");
		list_add_tail(&rq->queuelist, &ld->write_fifo_list);
	} else {
		printk("KERN_DEBUG lr_add_request(insert read)\n");
		lr_add_rq_rb(ld, rq); // insert tree
		idx = lr_get_sl_idx(rq);
		list_add_tail(&rq->queuelist, &ld->sl[idx].read_fifo_list);	// insert fifo list
	}
	/* printk("KERN_DEBUG ****************************************\n"); */
	/* printk("KERN_DEBUG end add rq\n"); */
	/* printk("KERN_DEBUG ****************************************\n"); */
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



static inline struct request *lr_latter_write_rq(struct lr_data *ld, struct request *rq)
{
	if(!list_empty(&ld->write_fifo_list))
		return rq_entry_fifo(&ld->write_fifo_list);
	return NULL;
}

/*
 * move an entry to dispatch queue
 */
static void lr_move_request(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	struct request_queue *q = rq->q;

	ld->next_rq[READ] = NULL;
	ld->next_rq[WRITE] = NULL;

//	if(data_dir == WRITE)
		//	ld->next_rq[WRITE] = lr_latter_write_rq(ld, rq);

	ld->last_sector = rq_end_sector(rq);
	
	lr_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
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

	for (i = 0; i < SL_NUM; i++) {
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
			return first_rq_in_fifo;  //tmp
			break;
		}
		return NULL; //tmp
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
	struct rb_node *node;
	int i, cnt = 0;
	
	for (i = 0; i < SL_NUM; i++) {
		list_for_each_entry(__rq, &ld->sl[i].read_fifo_list, queuelist) {
			printk("read rq in fifo:%p\n", __rq);
			cnt++;
		}
	}
	printk("KERN_DEBUGthere is %d rq in read fifo\n", cnt);

	for (i = 0; i < SL_NUM; i++) {
		printk("%d th sub layer\n", i);
		for(node = rb_first(&ld->sl[i].read_sort_list); node; node = rb_next(node))
			printk("READ sorted list rq:%p\n", rb_entry_rq(node));
	}
}

/*
 * lr_dispatch_requests selects the best request according to
 * read/write expire, fifo_batch, etc
 * when request is empty, return 0 else return 1.
 */


// TODO:next write
// TODO:next read

// TODO:batch_write
// TODO:batch_read

// cnt
static int lr_dispatch_requests(struct request_queue *q, int force)
{
	struct lr_data *ld = q->elevator->elevator_data;
	struct request *rq = NULL;
	
//	struct request *next_write = ld->next_rq[WRITE];
//	struct request *next_read = ld->next_rq[READ];

//	int exceed_batch_write = (ld->batching >= ld->batch_write);
//	int exceed_batch_read = (ld->batching >= ld->batch_read);
//	int ddir = rq_data_dir(rq);
	
//	lr_display_read_requests(ld);
//	lr_display_write_requests(ld);

	rq = lr_check_sub_layer(ld);

	if (!rq) {
		goto dispatch_writes;
	} else {
		goto dispatch_request;
	}

dispatch_writes:
	if(!list_empty(&ld->write_fifo_list))
		rq = rq_entry_fifo(ld->write_fifo_list.next);

	return 0;
dispatch_request:
	lr_move_request(ld, rq);
	return 1;
}

/* static int lr_dispatch_requests(struct request_queue *q, int force) */
/* { */
/* 	struct lr_data *ld = q->elevator->elevator_data; */
/* 	struct request *rq = NULL; */
	
/* 	struct request *next_write = ld->next_rq[WRITE]; */
/* 	struct request *next_read = ld->next_rq[READ]; */

/* 	int exceed_batch_write = (ld->batching >= ld->batch_write); */
/* 	int exceed_batch_read = (ld->batching >= ld->batch_read); */
/* 	int ddir = rq_data_dir(rq); */
	
/* //	lr_display_read_requests(ld); */
/* //	lr_display_write_requests(ld); */

/* 	rq = lr_check_sub_layer(ld); */

/* 	if (!rq) { */
/* 		goto dispatch_writes; */
/* 	} else { */
/* 		goto dispatch_request; */
/* 	} */

/* dispatch_writes:  */
/* 	if (next_write && !exceed_batch_write) { */
/* 		rq = next_write; */
/* 		goto dispatch_request; */
/* 	} else if(next_write && exceed_batch_write) { */
/* 		ld->mode = READ; */
/* 		return 1; */
/* 	} */
	
/* 	if (!rq && !list_empty(&ld->write_fifo_list)) { */
/* 		rq = rq_entry_fifo(ld->write_fifo_list.next); */
/* 	} */
	
/* 	if(rq && !exceed_batch_write) { */
/* 		goto dispatch_request; */
/* 	} else { */
/* 		ld->mode = READ; */
/* 		return 1; */
/* 	}  */
/* dispatch_request: */
/* 	if(ld->pre_rq_dir == ddir) { */
/* 		ld->batching++; */
/* 	} else { */
/* 		ld->pre_rq_dir = ddir; */
/* 		ld->batching = 1; */
/* 	} */

/* 	lr_move_request(ld, rq); */
/* 	return 1; */
/* } */


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
		.elevator_allow_merge_fn = lr_allow_merge,
		.elevator_dispatch_fn = lr_dispatch_requests,
		.elevator_add_req_fn = lr_add_request,
//		.elevator_former_req_fn = elv_rb_former_request,
//		.elevator_latter_req_fn = elv_rb_latter_request,
		.elevator_init_fn = lr_init_queue,
		.elevator_exit_fn = lr_exit_queue,
		},

	.elevator_attrs = lr_attrs,
	.elevator_name = "alr",
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
