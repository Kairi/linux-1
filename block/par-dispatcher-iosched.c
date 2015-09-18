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

static const unsigned long device_sector = ((long)1024 * 50); // QEMU

#define FLASH_CHIP_NUM 16
#define SQ_NUM FLASH_CHIP_NUM * 2

struct sub_queue {
	struct list_head fifo;
	struct rb_root tree;
};
struct pd_data {
	struct sub_queue sq[SQ_NUM];
	sector_t last_sector;	/* head position */
	int fifo_expire[2][2];
	int pre_idx;
	int cnt; 
};


//TODO
static inline int pd_get_sq_idx(struct request *rq)
{
	const int sector = blk_rq_pos(rq);
	int pd_sector_num = device_sector / (2 * FLASH_CHIP_NUM) + 1;
	int idx = (int)(sector / pd_sector_num);
	return idx;
}

static inline struct rb_root *
pd_rb_root(struct pd_data *pd, struct request *rq)
{
	return &pd->sq[pd_get_sq_idx(rq)].tree;
}

static inline struct list_head *pd_get_fifo(struct pd_data *pd, struct request *rq)
{
	return &pd->sq[pd_get_sq_idx(rq)].fifo;
}

// register to elevator_add_req_fn
static void pd_add_rq(struct request_queue *q, struct request *rq)
{
	struct pd_data *pd = q->elevator->elevator_data;
	int data_dir = rq_data_dir(rq);
	int sync = rq_is_sync(rq);

	elv_rb_add(pd_rb_root(pd, rq), rq);
	
	rq->fifo_time = jiffies + pd->fifo_expire[sync][data_dir];
	list_add_tail(&rq->queuelist, pd_get_fifo(pd, rq));
}

/*
 * unregister read rq from tree and change next rq
 */
static inline void pd_del_rq(struct pd_data *pd, struct request *rq)
{
	struct rb_root *root;
	root = pd_rb_root(pd, rq);
	elv_rb_del(root, rq);
	rq_fifo_clear(rq);
}

static int pd_allow_merge(struct request_queue *q, struct request *rq, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}

/*
 * move an entry to dispatch queue
 */
static void pd_move_request(struct pd_data *pd, struct request *rq)
{
	struct request_queue *q = rq->q;
	printk("KREN_DEBUG pd_move_request");
	pd->last_sector = rq_end_sector(rq);
	
	pd_del_rq(pd, rq);
	elv_dispatch_add_tail(q, rq);
}

static int pd_dispatch_requests(struct request_queue *q, int force)
{
	struct pd_data *pd = q->elevator->elevator_data;
	struct request *rq = NULL;
	int i;
	for (i = 0; i < SQ_NUM; i++) {
		if(!list_empty(&pd->sq[i].fifo)) {
			rq = rq_entry_fifo(&pd->sq[i].fifo);
			break;
		}
	}

	if(!rq) {
		return 0;
	}
	
//	pd_move_request(pd, rq);
	pd->last_sector = rq_end_sector(rq);
	elv_dispatch_add_tail(q, rq);
	rq_fifo_clear(rq);
	elv_rb_del(pd_rb_root(pd, rq), rq);
	
	return 1;
}

static void pd_exit_queue(struct elevator_queue *e)
{
	struct pd_data *pd = e->elevator_data;
	int i;

	for (i = 0; i < SQ_NUM; i++) {
		BUG_ON(!list_empty(&pd->sq[i].fifo));
	}

	kfree(pd);
}

/*
 */
static int pd_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct pd_data *pd;
	struct elevator_queue *eq;
	int i;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	pd = kzalloc_node(sizeof(*pd), GFP_KERNEL, q->node);
	if (!pd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = pd;

	for (i = 0; i < SQ_NUM; i++) { 
		pd->sq[i].tree = RB_ROOT;
		INIT_LIST_HEAD(&pd->sq[i].fifo);
	}

	pd->pre_idx = 0;

	pd->fifo_expire[SYNC][READ] = sync_read_expire;
	pd->fifo_expire[SYNC][WRITE] = sync_write_expire;
	pd->fifo_expire[ASYNC][READ] = async_read_expire;
	pd->fifo_expire[ASYNC][WRITE] = async_write_expire;
	
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static struct elevator_type iosched_pd = {
	.ops = {
		.elevator_allow_merge_fn = pd_allow_merge,
		.elevator_dispatch_fn = pd_dispatch_requests,
		.elevator_add_req_fn = pd_add_rq,
		.elevator_init_fn = pd_init_queue,
		.elevator_exit_fn = pd_exit_queue,
		},

	.elevator_name = "pd",
	.elevator_owner = THIS_MODULE,
};

static int __init pd_init(void)
{
	return elv_register(&iosched_pd);
}

static void __exit pd_exit(void)
{
	elv_unregister(&iosched_pd);
}

module_init(pd_init);
module_exit(pd_exit);

MODULE_AUTHOR("Kairi OKUMURA");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("par-dispatcher IO scheduler");
