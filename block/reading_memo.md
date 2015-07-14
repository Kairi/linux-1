#+TITLE: I/O Block Layer Code Reading Memo
* Block I/O Controller
Block I/O Controller is Cgroup Subsystem which has benn included into Linux Kernel since v2.6.33

#TODO
research NCQ(native commnad queueing) and RAID Controller.

** Build Option
- BLOCK
Enable The Block Layer

- CGROUPS
Control Group Support 

- BLK_CGROUP
Block IO controller
Depends on:CGROUPS && BLOCK

-DEBUG_BLK_CGROUP
Enable Block IO controller debugging
Depends on:CGROUPS && BLK_CGROUP

-CFQ_GROUP_IOSCHED
CFQ group scheduling support
Depends on:BLOCK && IOCHED_CFQ && BLK_CGROUP

** I/O Context
I/O Context provide a  dynamically allocated per process data area.
They are used in I/O schedulers, and in the block layer.

** Request 
In this, explain about members refers to I/O scheduler.
Member
------
- struct list_head queuelist: Organization on vatious internal queues.

- void *elevator_private: I/O scheduler private data

- unsigned long flags Contains info about data direction, request type, etc.
** Fundamental Data Structure

** Fundamental Data in Block Layer
- request_queue: Block Device Queue
- 

** memo
The interesting idea here is how to get the data description of noop_data. 
As you can find the type of elevator_data is void. Actually, the idea is to use pointer to record the memory address. 
For example, in kernel mode, every process has its own task_struct to describe its own property, please remember,
 each data structure has a confirmed memory address. 
Through pointer, we can easily get the data structure in any level. 
In another word, if I want to do something about block device level in md level, 
we can use memory address to record the correspondent data structure such as noop_data, cfq_data, as_data, etc.

** request flag types
/*
 * Request flags.  For use in the cmd_flags field of struct request, and in
 * bi_rw of struct bio.  Note that some flags are only valid in either one.
 */
enum rq_flag_bits {
	/* common flags */
	__REQ_WRITE,		/* not set, read. set, write */
	__REQ_FAILFAST_DEV,	/* no driver retries of device errors */
	__REQ_FAILFAST_TRANSPORT, /* no driver retries of transport errors */
	__REQ_FAILFAST_DRIVER,	/* no driver retries of driver errors */

	__REQ_SYNC,		/* request is sync (sync write or read) */
	__REQ_META,		/* metadata io request */
	__REQ_PRIO,		/* boost priority in cfq */
	__REQ_DISCARD,		/* request to discard sectors */
	__REQ_SECURE,		/* secure discard (used with __REQ_DISCARD) */
	__REQ_WRITE_SAME,	/* write same block many times */

	__REQ_NOIDLE,		/* don't anticipate more IO after this one */
	__REQ_INTEGRITY,	/* I/O includes block integrity payload */
	__REQ_FUA,		/* forced unit access */
	__REQ_FLUSH,		/* request for cache flush */

	/* bio only flags */
	__REQ_RAHEAD,		/* read ahead, can fail anytime */
	__REQ_THROTTLED,	/* This bio has already been subjected to
				 * throttling rules. Don't do it again. */

	/* request only flags */
	__REQ_SORTED,		/* elevator knows about this request */
	__REQ_SOFTBARRIER,	/* may not be passed by ioscheduler */
	__REQ_NOMERGE,		/* don't touch this for merging */
	__REQ_STARTED,		/* drive already may have started this one */
	__REQ_DONTPREP,		/* don't call prep for this one */
	__REQ_QUEUED,		/* uses queueing */
	__REQ_ELVPRIV,		/* elevator private data attached */
	__REQ_FAILED,		/* set if the request failed */
	__REQ_QUIET,		/* don't worry about errors */
	__REQ_PREEMPT,		/* set for "ide_preempt" requests and also
				   for requests for which the SCSI "quiesce"
				   state must be ignored. */
	__REQ_ALLOCED,		/* request came from our alloc pool */
	__REQ_COPY_USER,	/* contains copies of user pages */
	__REQ_FLUSH_SEQ,	/* request for flush sequence */
	__REQ_IO_STAT,		/* account I/O stat */
	__REQ_MIXED_MERGE,	/* merge of different types, fail separately */
	__REQ_PM,		/* runtime pm request */
	__REQ_HASHED,		/* on IO scheduler merge hash */
	__REQ_MQ_INFLIGHT,	/* track inflight for MQ */
	__REQ_NO_TIMEOUT,	/* requests may never expire */
	__REQ_NR_BITS,		/* stops here */
};

** Deadline Reading
*** Add Req to Queue Algorithms
deadline_add_request():add rq to rbtree and fifo
    deadline_add_rq_rb():
        elv_rb_add():

rq->fifo+time = jiffies + dd->fifo_expire[data_dir]; set limit time to deadline_data
lis_add_tail(&rq->queuelist, &&dd->fifo_list[data_dir]);

*** Dispath Algorithms
- exist next req and NOT excess batch num limit.(dispatch_request)
dd->batching++
deadline_move_request(): move an entry to dispatch queue
deadline_move_to_dispatch(): move request from sort list to dispatch queue
deaddline_remove_request(): remove rq from rbtree and fifo
rq_fifo_clear():
deadline_del_rq_rb():
elv_dispatch_add_tail():

- (dispatch_writes)
- (dispatch_find_request)


** TPPS Data Structure
- kmem_cache *tpps_pool
- struct tpps_queue
int ref: reference count 
struct tpps_data *tppd: parent tpps_data 
struct list_head tppg_node: tpps_group member 
/* sorted list of pending requests */
struct list_head sort_list;
struct tpps_group *tppg;
pid_t pid;
int online;
int rq_queued;
};
-tpps_queue

-tppg_stats
-tpps_group


