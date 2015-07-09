// really NO Operation I/O Scheduler
#include<linux/blkdev.h>
#include<linux/elevator.h>
#include<linux/module.h>
#include<linux/init.h>
#include<linux/kernel.h>

struct fifo_data {
	struct list_head queue;
};

static int fifo_dispatch(struct request_queue *q, int force)
{
	struct fifo_data *fd = q->elevator->elevator_data;

	if(!list_empty(&fd->queue)) {
		struct request *rq;
		rq = list_entry(fd->queue.next, struct request, queuelist);
		list_del_init(&rq->queuelist);
		elv_dispatch_add_tail(q, rq); // differ from NOOP
		return 1;
	}
	return 0;
}

static int fifo_allow_merge(struct request_queue *q, struct request *rq, struct bio *bio)
{
	return ELEVATOR_NO_MERGE;
}


static void fifo_add_request(struct request_queue *q, struct request *rq)
{
	struct fifo_data *fd = q->elevator->elevator_data;
	list_add_tail(&rq->queuelist, &fd->queue);
}

static int fifo_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct fifo_data *fd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	fd = kmalloc_node(sizeof(*fd), GFP_KERNEL, q->node);
	if(!fd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}

	eq->elevator_data = fd;
	
	INIT_LIST_HEAD(&fd->queue);

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}


static void fifo_exit_queue(struct elevator_queue *e)
{
	struct fifo_data *fd = e->elevator_data;

	BUG_ON(!list_empty(&fd->queue));
	kfree(fd);
}

static struct elevator_type elevator_fifo = {
	.ops = {
		.elevator_dispatch_fn = fifo_dispatch,
		.elevator_allow_merge_fn = fifo_allow_merge,
		.elevator_add_req_fn = fifo_add_request,
		.elevator_init_fn = fifo_init_queue,
		.elevator_exit_fn = fifo_exit_queue,
	},
	.elevator_name = "fifo",
	.elevator_owner  =THIS_MODULE,
};

	

static int __init fifo_init(void)
{
	return elv_register(&elevator_fifo);
}

static void __exit(fifo_exit)(void)
{
	elv_unregister(&elevator_fifo);
}
	

module_init(fifo_init);
module_exit(fifo_exit);

MODULE_AUTHOR("Kairi Okumura");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Really Do Nothing I/O Scheduler");


