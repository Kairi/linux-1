/*
 *  LR(Layer Read) Plus i/o scheduler.
 * 現行案:SYNC ASYNCに対しても追加の処理を行う
 * 1. SYNC READ
 * 2. SYNC WRITE
 * 3. ASYNC READ
 * 4. ASYNC WIRTE
 * の4種類に対して
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

//static const int read_expire = HZ / 2;	/* max time before a read is submitted. */
//static const int write_expire = 5 * HZ;	/* ditto for writes, these limits are SOFT! */

static const int sync_read_expire = HZ /2;
static const int sync_write_expire = 2 * HZ;
static const int async_read_expire = 4 * HZ;
static const int async_write_expire = 16* HZ;

static const int writes_starved = 2;	/* max times reads can starve a write */
static const int batch_read = 16;
static const int batch_write = 8;

enum{ ASYNC, SYNC };

// cat /sys/class/block/sda/size * QEMU:51584
static const unsigned long device_sector = ((long)1024 * 50); // QEMU
//static const unsigned long device_sector = ((long)1024 * 1024 * 128); // My Arch

#define FLASH_CHIP_NUM 16
#define SL_NUM FLASH_CHIP_NUM * 2

struct sub_layer {
	struct list_head read_fifo_list;
	struct rb_root read_sort_list;
};

struct lr_data {
	struct list_head write_fifo_list;
	struct sub_layer sl[FLASH_CHIP_NUM * 2];

	struct request *next_rq[2];
	sector_t last_sector;	/* head position */
	int fifo_expire[2][2];
	int pre_idx;
	int sl_cnt; 
};

static void lr_move_request(struct lr_data *, struct request *);
static inline int lr_get_sl_idx(struct request *rq);

static inline struct rb_root *lr_rb_root(struct lr_data *ld, struct request *rq)
{
	int idx;
	const int data_dir = rq_data_dir(rq);
	const int sync = rq_is_sync(rq);

	if (data_dir == READ && sync == ASYNC) {
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
	const int sync = rq_is_sync(rq);
	struct rb_root *root;

	if (data_dir == READ && sync == ASYNC) {
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
	const int sync = rq_is_sync(rq);
	struct rb_root *root;
	int idx;

	if (data_dir == WRITE || sync == SYNC)
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
	
	int sl_sector_num = device_sector / (2 * FLASH_CHIP_NUM) + 1;
	int idx = (int)(sector / sl_sector_num);
	printk("KERN_DEBUG sector is %d, device_byte_size:%lo index is %d.\n", sector, device_sector, idx);
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
	const int sync = rq_is_sync(rq);
	int idx;

	rq->fifo_time = jiffies + ld->fifo_expire[sync][data_dir];

	printk("KERN_DEBUG data_dir:%d, sync:%d", data_dir, sync);

	if (data_dir == WRITE || sync == SYNC) {
		list_add_tail(&rq->queuelist, &ld->write_fifo_list);
	} else {
		lr_add_rq_rb(ld, rq); // insert tree
		idx = lr_get_sl_idx(rq);
		list_add_tail(&rq->queuelist, &ld->sl[idx].read_fifo_list);	// insert fifo list
	}
}

static int lr_allow_merge(struct request_queue *q, struct request *rq, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}


static inline struct request *lr_latter_write_rq(struct lr_data *ld, struct request *rq)
{
	if(!list_empty(&ld->write_fifo_list))
		return rq_entry_fifo(&ld->write_fifo_list);
	return NULL;
}
static void lr_remove_request(struct request_queue *q, struct request *rq)
{
	struct lr_data *ld = q->elevator->elevator_data;

	rq_fifo_clear(rq);
	lr_del_rq_rb(ld, rq);
}

/*
 * move an entry to dispatch queue
 */
static void lr_move_request(struct lr_data *ld, struct request *rq)
{
	struct request_queue *q = rq->q;

	ld->next_rq[READ] = NULL;
	ld->next_rq[WRITE] = NULL;

	ld->last_sector = rq_end_sector(rq);
	
	lr_remove_request(q, rq);
	elv_dispatch_add_tail(q, rq);
}
static inline struct request *lr_get_sub_layer_rq(struct lr_data *ld)
{
	struct request *first_rq_in_fifo = NULL;
	struct request *first_rq_in_tree = NULL;
	int i;

	for (i = ld->pre_idx; i < SL_NUM; i++) {
		if (!list_empty(&ld->sl[i].read_fifo_list)) {
			first_rq_in_fifo = rq_entry_fifo(ld->sl[i].read_fifo_list.next);
			ld->pre_idx = i;
			break;
		}
	}

	if(!first_rq_in_fifo) {
		for(i = 0; i < ld->pre_idx; i++) {
			if (!list_empty(&ld->sl[i].read_fifo_list)) {
				first_rq_in_fifo = rq_entry_fifo(ld->sl[i].read_fifo_list.next);
				ld->pre_idx = i;
				break;
			}
		}
		if(!first_rq_in_fifo)
			return NULL;
	}
		
	if (time_after_eq(jiffies, first_rq_in_fifo->fifo_time)) {
		return first_rq_in_fifo;
	}

	first_rq_in_tree = rb_entry_rq(rb_first(&ld->sl[i].read_sort_list));
	return first_rq_in_tree;
	
}


static int lr_dispatch_requests(struct request_queue *q, int force)
{
	struct lr_data *ld = q->elevator->elevator_data;
	struct request *rq = NULL;

	rq = lr_get_sub_layer_rq(ld);

	if (!rq) {
		goto dispatch_writes;
	} else {
		goto dispatch_request;
	}

dispatch_writes:
	if(!list_empty(&ld->write_fifo_list)) {
		rq = rq_entry_fifo(ld->write_fifo_list.next);
		goto dispatch_request;
	}	
	return 0;
dispatch_request:
	lr_move_request(ld, rq);
	return 1;
}

static void lr_exit_queue(struct elevator_queue *e)
{
	struct lr_data *ld = e->elevator_data;
	int i;

	BUG_ON(!list_empty(&ld->write_fifo_list));
	for (i = 0; i < SL_NUM; i++) {
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

	for (i = 0; i < SL_NUM; i++) { // for read only
		ld->sl[i].read_sort_list = RB_ROOT;
		INIT_LIST_HEAD(&ld->sl[i].read_fifo_list);
	}

	ld->pre_idx = 0;

	ld->fifo_expire[SYNC][READ] = sync_read_expire;
	ld->fifo_expire[SYNC][WRITE] = sync_write_expire;
	ld->fifo_expire[ASYNC][READ] = async_read_expire;
	ld->fifo_expire[ASYNC][WRITE] = async_write_expire;
	
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static struct elevator_type iosched_lr = {
	.ops = {
		.elevator_allow_merge_fn = lr_allow_merge,
		.elevator_dispatch_fn = lr_dispatch_requests,
		.elevator_add_req_fn = lr_add_request,
		.elevator_init_fn = lr_init_queue,
		.elevator_exit_fn = lr_exit_queue,
		},

	.elevator_name = "lrp",
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
