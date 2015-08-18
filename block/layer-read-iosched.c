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

static const int read_expire = HZ / 2;  /* max time before a read is submitted. */
static const int write_expire = 5 * HZ; /* ditto for writes, these limits are SOFT! */
static const int writes_starved = 2;    /* max times reads can starve a write */
static const int fifo_batch = 16;       /* # of sequential requests treated as one
				     by the above parameters. For throughput. */
#define FLASH_CHIP_NUM 8
#define SL_NUM FLASH_CHIP_NUM * 2
#define DEVICE_SIZE ((long)1024 * 1024 * 1024 * 64) // 64GB


// this is used for READ requests ONLY.
struct sub_layer {
	struct list_head read_fifo_list;
	struct rb_root read_sort_list;
};

struct lr_data {
	struct list_head write_fifo_list; // this is used for WRITE ONLY
	struct sub_layer sl[FLASH_CHIP_NUM * 2]; 
	/*
	 * next in sort order. read, write or both are NULL
	 */
	struct request *next_rq[2];
	unsigned int batching;		/* number of sequential requests made */
	sector_t last_sector;		/* head position */
	unsigned int starved;		/* times reads have starved writes */

	/*
	 * settings that change how the i/o scheduler behaves
	 */
	int mode; // write or read 
	int fifo_expire[2];
	int fifo_batch;
	int batch_read;
	int batch_write;
	int writes_starved;
};

static void lr_move_request(struct lr_data *, struct request *);
static inline int lr_get_sl_idx(struct request *rq);

static inline void lr_puts_data(struct lr_data *ld)
{
	printk("KERN_DEBUG mode=%d batch_read=%d batch_write=%d", ld->mode, ld->batch_read, ld->batch_write);
	printk("KERN_DEBUG write fifo list");
	struct list_head *pos;
	
	list_for_each(pos, &ld->write_fifo_list)
	{
		printk("KERN_DEBUG next=%p, prev=%p", pos->next, pos->prev);
	}
	
}


static inline struct rb_root *
lr_rb_root(struct lr_data *ld, struct request *rq)
{
	int idx;
	const int data_dir = rq_data_dir(rq);

	if (data_dir == READ) {
		idx = lr_get_sl_idx(rq);		
		return &ld->sl[idx].read_sort_list;
	}

	return NULL; // if write
}


/*
 * when `rq' is read ,get the request after `rq' in sector-sorted order.
 * when `rq' is write ,get the request after `rq' in time-sorted order.
 */
static inline struct request *
lr_latter_request(struct lr_data *ld, struct request *rq)
{
	
	struct rb_node *node;
	const int data_dir = rq_data_dir(rq);
	int idx, i;
	
	if(data_dir == WRITE) {
		if(!list_empty(&ld->write_fifo_list))
			return rq_entry_fifo(&ld->write_fifo_list);
	} else { // if read
		node = rb_next(&rq->rb_node);
		if(node) 
			return rb_entry_rq(node);
		
		idx = lr_get_sl_idx(rq);
		for(i = idx + 1; i < SL_NUM; i ++) { // if sub layer is empty
			node = rb_first(&ld->sl[idx].read_sort_list);
			if(node)
				return rb_entry_rq(node);
		}
	}
	return NULL;
}


static void
lr_add_rq_rb(struct lr_data *ld, struct request *rq)
{
	struct rb_root *root = lr_rb_root(ld, rq);
	const int data_dir = rq_data_dir(rq);

	if (data_dir == READ)
		elv_rb_add(root, rq);
}


static inline void
lr_del_rq_rb(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);
	struct rb_root *root;
	int idx;
	printk("KERN_DEBUG lr_del_rq_rb\n");
		
	if (ld->next_rq[data_dir] == rq)
		ld->next_rq[data_dir] = lr_latter_request(ld, rq);

	if (data_dir == WRITE) {
		return;
	}
	
		

	if (data_dir == READ) {
		idx = lr_get_sl_idx(rq);
	}
	
	root = lr_rb_root(ld, rq);
	if (root != NULL) {
		elv_rb_del(root, rq);
	}
}

	

static inline int
lr_get_sl_idx(struct request *rq)
{
	const int sector = blk_rq_sectors(rq);
	int idx = (int)(sector / (DEVICE_SIZE / (2 * FLASH_CHIP_NUM) + 1));
	return idx;
}


/*
 * add rq to rbtree and fifo
 * registerd to elevator_add_req_fn
 */
static void
lr_add_request(struct request_queue *q, struct request *rq)
{
	struct lr_data *ld = q->elevator->elevator_data;
	const int data_dir = rq_data_dir(rq);
	int idx;



	rq->fifo_time = jiffies + ld->fifo_expire[data_dir];	

	if(data_dir == WRITE) { // write is fifo only
		printk("KERN_DEBUG lr_add_request(insert write)\n");
		list_add_tail(&rq->queuelist, &ld->write_fifo_list); 
	} else { //read only sub layer
		printk("KERN_DEBUG lr_add_request(insert read)\n");
		idx = lr_get_sl_idx(rq);		
		elv_rb_add(&ld->sl[idx].read_sort_list, rq); // insert sorted list
		list_add_tail(&rq->queuelist, &ld->sl[idx].read_fifo_list); // insert fifo list
	}
}

static void lr_remove_request(struct request_queue *q, struct request *rq)
{
	struct lr_data *ld = q->elevator->elevator_data;
	printk("KERN_DEBUG lr_remove_request\n");
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
	
	/*
	 * if the merge was a front merge, we need to reposition request
	 */
	if (type == ELEVATOR_FRONT_MERGE) {
		root = lr_rb_root(ld, req);
		if(root != NULL) {
			elv_rb_del(lr_rb_root(ld, req), req);
			lr_add_rq_rb(ld, req);
		}
	}
}

static void
lr_merged_requests(struct request_queue *q, struct request *req,
			 struct request *next)
{
	/*
	 * if next expires before rq, assign its expire time to rq
	 * and move into next position (next will be deleted) in fifo
	 */
	if (!list_empty(&req->queuelist) && !list_empty(&next->queuelist)) {
		if (time_before(next->fifo_time, req->fifo_time)) {
			list_move(&req->queuelist, &next->queuelist); // req->queuelist:the entry to move, next->queuelist:the head that will precede our entry
			req->fifo_time = next->fifo_time;
		}
	}

	/*
	 * kill knowledge of next, this one is a goner
	 */
	lr_remove_request(q, next);
}

/*
 * move request from sort list to dispatch queue.
 */
static inline void
lr_move_to_dispatch(struct lr_data *ld, struct request *rq)
{
	struct request_queue *q = rq->q;
	printk("KERN_DEBUG lr_move_to_dispatch\n");
	
	lr_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
}

/*
 * move an entry to dispatch queue
 */
static void
lr_move_request(struct lr_data *ld, struct request *rq)
{
	const int data_dir = rq_data_dir(rq);

	printk("KERN_DEBUG lr_move_request\n");
	
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
 * lr_check_fifo returns 0 if there are no expired requests on the fifo,
 * 1 otherwise. Requires !list_empty(&ld->fifo_list[data_dir])
 */

// NOT completed
static inline int lr_check_fifo(struct lr_data *ld, int ddir)
{
	struct request *rq = NULL;
	
	if(ddir == WRITE) {
		rq = rq_entry_fifo(ld->write_fifo_list.next);
	} else {
		int i;
		for (i = 0; i < FLASH_CHIP_NUM * 2; i++) {
			rq = rq_entry_fifo(ld->sl[i].read_fifo_list.next);
			if (rq)
				break;
		}
	}
		
	/*
	 * rq is expired!
	 */
	if (time_after_eq(jiffies, rq->fifo_time))
		return 1;

	return 0;
}


/* return  request in sub layer
   when sub layer is empty or dispatch 8 times in same layer, function retuns  NULL.
 */
static inline struct request *lr_check_sub_layer(struct lr_data *ld, int pre_idx) 
{
	int i, cnt = 0;	
	struct request *first_rq_in_fifo = NULL;
	struct request  *first_rq_in_tree = NULL; 
	
	for (i = pre_idx; i < SL_NUM; i++) {
		if (!list_empty(&ld->sl[i].read_fifo_list)) {

			if (pre_idx == i)
				cnt++;

			if (cnt >= 8)
				continue;

//			first_rq_in_fifo = list_entry(&ld->sl[i].read_fifo_list.next, struct request, queuelist);
			first_rq_in_fifo = rq_entry_fifo(ld->sl[i].read_fifo_list.next);
			break;
		}
	}
	if (!first_rq_in_fifo) // no appropriate requests
		return NULL; 
	
	if (time_after_eq(jiffies, first_rq_in_fifo->fifo_time)) // rq is expired!
		return first_rq_in_fifo;

	first_rq_in_tree = rb_entry_rq(rb_first(&ld->sl[i].read_sort_list));

	if (first_rq_in_tree && ld->batching > ld->batch_read) 
		return NULL;
		
	return first_rq_in_tree; // rq is NOT expired!
}


/*
 * lr_dispatch_requests selects the best request according to
 * read/write expire, fifo_batch, etc
 */
static int lr_dispatch_requests(struct request_queue *q, int force)
{
	struct lr_data *ld = q->elevator->elevator_data;
	struct request *rq;
	static int idx;
//	lr_puts_data(ld);
	
//	printk("KERN_DEBUG lr_dispatch_requests\n");

	if (ld->next_rq[READ]) 
		rq = ld->next_rq[READ];
	
	if (!rq) {
		if(ld->mode == WRITE)
			goto dispatch_writes;

		goto dispatch_layer;
	}

	
dispatch_layer: // dispatch sub layer(read)
//	printk("KERN_DEBUG dispatch_layer\n");
	rq = lr_check_sub_layer(ld, idx);
	if(!rq) {// all sub layer is empty or exceed batch_read
		ld->batching = 0;
		ld->batch_read = 0;
		goto dispatch_writes;
	} else {
		ld->batch_write = 0;
		ld->batching = 0;
		goto dispatch_request;
	}
	
	

dispatch_writes: // dipatch write request
//	printk("KERN_DEBUG dispatch_writes\n");
	ld->batch_read = 0;
	if (ld->next_rq[WRITE])
		rq = ld->next_rq[WRITE];

	if (!rq)
		rq = list_entry(ld->write_fifo_list.next, struct request, queuelist);

	if(rq)
		printk("KERN_DEBUG exist write req\n");
	
	
	if (!rq || ld->batching > ld->batch_write) {// exist request && NOT Excess batch limit./* we have a next request are still entitled to batch */
		ld->mode = READ;
		printk("KERN_DEBUG end dispatch_writes\n");
		return 0;
	}

dispatch_request:
//	printk("KERN_DEBUG dispatch_request\n");
	/*
	 * rq is the selected appropriate request.
	 */
	ld->batching++;
	lr_move_request(ld, rq);

	return 0;
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

//	INIT_LIST_HEAD(&dd->fifo_list[READ]);
//	INIT_LIST_HEAD(&dd->fifo_list[WRITE]);
	INIT_LIST_HEAD(&ld->write_fifo_list); // for write only

	for (i = 0; i < FLASH_CHIP_NUM * 2; i++) {
		ld->sl[i].read_sort_list = RB_ROOT;
		INIT_LIST_HEAD(&ld->sl[i].read_fifo_list);	
	}
	ld->fifo_expire[READ] = read_expire;
	ld->fifo_expire[WRITE] = write_expire;
	ld->writes_starved = writes_starved;
	ld->fifo_batch = fifo_batch;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/*
 * sysfs parts below
 */

static ssize_t
lr_var_show(int var, char *page)
{
	return sprintf(page, "%d\n", var);
}

static ssize_t
lr_var_store(int *var, const char *page, size_t count)
{
	char *p = (char *) page;

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
SHOW_FUNCTION(lr_writes_starved_show, dd->writes_starved, 0);
SHOW_FUNCTION(lr_fifo_batch_show, dd->fifo_batch, 0);
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
STORE_FUNCTION(lr_writes_starved_store, &dd->writes_starved, INT_MIN, INT_MAX, 0);
STORE_FUNCTION(lr_fifo_batch_store, &dd->fifo_batch, 0, INT_MAX, 0);
#undef STORE_FUNCTION

#define DD_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, lr_##name##_show, \
				      lr_##name##_store)

static struct elv_fs_entry lr_attrs[] = {
	DD_ATTR(read_expire),
	DD_ATTR(write_expire),
	DD_ATTR(writes_starved),
	DD_ATTR(fifo_batch),
	__ATTR_NULL
};

static struct elevator_type iosched_lr = {
	.ops = {
		.elevator_merge_fn = 		lr_merge,
		.elevator_merged_fn =		lr_merged_request,
		.elevator_merge_req_fn =	lr_merged_requests,
		.elevator_dispatch_fn =		lr_dispatch_requests,
		.elevator_add_req_fn =		lr_add_request,
		.elevator_former_req_fn =	elv_rb_former_request,
		.elevator_latter_req_fn =	elv_rb_latter_request,
		.elevator_init_fn =		lr_init_queue,
		.elevator_exit_fn =		lr_exit_queue,
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
