/*
 *  LB, or complete fairness queueing, disk scheduler.
 *
 *  Based on ideas from a previously unfinished io
 *  scheduler (round robin per-process disk scheduling) and Andrea Arcangeli.
 *
 *  Copyright (C) 2003 Jens Axboe <axboe@kernel.dk>
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/jiffies.h>
#include <linux/rbtree.h>
#include <linux/ioprio.h>
#include <linux/blktrace_api.h>
#include "blk.h"
#include "blk-cgroup.h"

/*
 * tunables
 */
/* max queue in one round of service */
static const int lb_quantum = 8;
static const int lb_fifo_expire[2] = { HZ / 4, HZ / 8 };
/* maximum backwards seek, in KiB */
static const int lb_back_max = 16 * 1024;
/* penalty of a backwards seek */
static const int lb_back_penalty = 2;
static const int lb_slice_sync = HZ / 10;
static int lb_slice_async = HZ / 25;
static const int lb_slice_async_rq = 2;
static int lb_slice_idle = HZ / 125; // if 0, and if storage supports NCQ, LB internally switches to IOPS mode.
static int lb_group_idle = HZ / 125;
static const int lb_target_latency = HZ * 3/10; /* 300 ms */
static const int lb_hist_divisor = 4;

/*
 * offset from end of service tree
 */
#define LB_IDLE_DELAY		(HZ / 5)

/*
 * below this threshold, we consider thinktime immediate
 */
#define LB_MIN_TT		(2)

#define LB_SLICE_SCALE		(5)
#define LB_HW_QUEUE_MIN	(5)
#define LB_SERVICE_SHIFT       12

#define LBQ_SEEK_THR		(sector_t)(8 * 100)
#define LBQ_CLOSE_THR		(sector_t)(8 * 1024)
#define LBQ_SECT_THR_NONROT	(sector_t)(2 * 32)
#define LBQ_SEEKY(lbq)	(hweight32(lbq->seek_history) > 32/8)

#define RQ_CIC(rq)		icq_to_cic((rq)->elv.icq)
#define RQ_LBQ(rq)		(struct lb_queue *) ((rq)->elv.priv[0])
#define RQ_LBG(rq)		(struct lb_group *) ((rq)->elv.priv[1])

static struct kmem_cache *lb_pool;

#define LB_PRIO_LISTS		IOPRIO_BE_NR
#define lb_class_idle(lbq)	((lbq)->ioprio_class == IOPRIO_CLASS_IDLE)
#define lb_class_rt(lbq)	((lbq)->ioprio_class == IOPRIO_CLASS_RT)

#define sample_valid(samples)	((samples) > 80)
#define rb_entry_lbg(node)	rb_entry((node), struct lb_group, rb_node)

struct lb_ttime {
	unsigned long last_end_request;

	unsigned long ttime_total;
	unsigned long ttime_samples;
	unsigned long ttime_mean;
};

/*
 * Most of our rbtree usage is for sorting with min extraction, so
 * if we cache the leftmost node we don't have to walk down the tree
 * to find it. Idea borrowed from Ingo Molnars CFS scheduler. We should
 * move this into the elevator for the rq sorting as well.
 */
struct lb_rb_root {
	struct rb_root rb;
	struct rb_node *left;
	unsigned count;
	u64 min_vdisktime;
	struct lb_ttime ttime;
};
#define LB_RB_ROOT	(struct lb_rb_root) { .rb = RB_ROOT, \
			.ttime = {.last_end_request = jiffies,},}

/*
 * Per process-grouping structure
 */
struct lb_queue {
	/* reference count */
	int ref;
	/* various state flags, see below */
	unsigned int flags;
	/* parent lb_data */
	struct lb_data *lbd;
	/* service_tree member */
	struct rb_node rb_node;
	/* service_tree key */
	unsigned long rb_key;
	/* prio tree member */
	struct rb_node p_node;
	/* prio tree root we belong to, if any */
	struct rb_root *p_root;
	/* sorted list of pending requests */
	struct rb_root sort_list;
	/* if fifo isn't expired, next request to serve */
	struct request *next_rq;
	/* requests queued in sort_list */
	int queued[2];
	/* currently allocated requests */
	int allocated[2];
	/* fifo list of requests in sort_list */
	struct list_head fifo;

	/* time when queue got scheduled in to dispatch first request. */
	unsigned long dispatch_start;
	unsigned int allocated_slice;
	unsigned int slice_dispatch;
	/* time when first request from queue completed and slice started. */
	unsigned long slice_start;
	unsigned long slice_end;
	long slice_resid;

	/* pending priority requests */
	int prio_pending;
	/* number of requests that are on the dispatch list or inside driver */
	int dispatched;

	/* io prio of this group */
	unsigned short ioprio, org_ioprio;
	unsigned short ioprio_class;

	pid_t pid;

	u32 seek_history;
	sector_t last_request_pos;

	struct lb_rb_root *service_tree;
	struct lb_queue *new_lbq;
	struct lb_group *lbg;
	/* Number of sectors dispatched from queue in single dispatch round */
	unsigned long nr_sectors;
};

/*
 * First index in the service_trees.
 * IDLE is handled separately, so it has negative index
 */
enum wl_class_t {
	BE_WORKLOAD = 0,
	RT_WORKLOAD = 1,
	IDLE_WORKLOAD = 2,
	LB_PRIO_NR,
};

/*
 * Second index in the service_trees.
 */
enum wl_type_t {
	ASYNC_WORKLOAD = 0,
	SYNC_NOIDLE_WORKLOAD = 1,
	SYNC_WORKLOAD = 2
};

struct lbg_stats {
#ifdef CONFIG_LB_GROUP_IOSCHED
	/* total bytes transferred */
	struct blkg_rwstat		service_bytes;
	/* total IOs serviced, post merge */
	struct blkg_rwstat		serviced;
	/* number of ios merged */
	struct blkg_rwstat		merged;
	/* total time spent on device in ns, may not be accurate w/ queueing */
	struct blkg_rwstat		service_time;
	/* total time spent waiting in scheduler queue in ns */
	struct blkg_rwstat		wait_time;
	/* number of IOs queued up */
	struct blkg_rwstat		queued;
	/* total sectors transferred */
	struct blkg_stat		sectors;
	/* total disk time and nr sectors dispatched by this group */
	struct blkg_stat		time;
#ifdef CONFIG_DEBUG_BLK_CGROUP
	/* time not charged to this cgroup */
	struct blkg_stat		unaccounted_time;
	/* sum of number of ios queued across all samples */
	struct blkg_stat		avg_queue_size_sum;
	/* count of samples taken for average */
	struct blkg_stat		avg_queue_size_samples;
	/* how many times this group has been removed from service tree */
	struct blkg_stat		dequeue;
	/* total time spent waiting for it to be assigned a timeslice. */
	struct blkg_stat		group_wait_time;
	/* time spent idling for this blkcg_gq */
	struct blkg_stat		idle_time;
	/* total time with empty current active q with other requests queued */
	struct blkg_stat		empty_time;
	/* fields after this shouldn't be cleared on stat reset */
	uint64_t			start_group_wait_time;
	uint64_t			start_idle_time;
	uint64_t			start_empty_time;
	uint16_t			flags;
#endif	/* CONFIG_DEBUG_BLK_CGROUP */
#endif	/* CONFIG_LB_GROUP_IOSCHED */
};

/* This is per cgroup per device grouping structure */
struct lb_group {
	/* must be the first member */
	struct blkg_policy_data pd;

	/* group service_tree member */
	struct rb_node rb_node;

	/* group service_tree key */
	u64 vdisktime;

	/*
	 * The number of active lbgs and sum of their weights under this
	 * lbg.  This covers this lbg's leaf_weight and all children's
	 * weights, but does not cover weights of further descendants.
	 *
	 * If a lbg is on the service tree, it's active.  An active lbg
	 * also activates its parent and contributes to the children_weight
	 * of the parent.
	 */
	int nr_active;
	unsigned int children_weight;

	/*
	 * vfraction is the fraction of vdisktime that the tasks in this
	 * lbg are entitled to.  This is determined by compounding the
	 * ratios walking up from this lbg to the root.
	 *
	 * It is in fixed point w/ LB_SERVICE_SHIFT and the sum of all
	 * vfractions on a service tree is approximately 1.  The sum may
	 * deviate a bit due to rounding errors and fluctuations caused by
	 * lbgs entering and leaving the service tree.
	 */
	unsigned int vfraction;

	/*
	 * There are two weights - (internal) weight is the weight of this
	 * lbg against the sibling lbgs.  leaf_weight is the wight of
	 * this lbg against the child lbgs.  For the root lbg, both
	 * weights are kept in sync for backward compatibility.
	 */
	unsigned int weight;
	unsigned int new_weight;
	unsigned int dev_weight;

	unsigned int leaf_weight;
	unsigned int new_leaf_weight;
	unsigned int dev_leaf_weight;

	/* number of lbq currently on this group */
	int nr_lbq;

	/*
	 * Per group busy queues average. Useful for workload slice calc. We
	 * create the array for each prio class but at run time it is used
	 * only for RT and BE class and slot for IDLE class remains unused.
	 * This is primarily done to avoid confusion and a gcc warning.
	 */
	unsigned int busy_queues_avg[LB_PRIO_NR];
	/*
	 * rr lists of queues with requests. We maintain service trees for
	 * RT and BE classes. These trees are subdivided in subclasses
	 * of SYNC, SYNC_NOIDLE and ASYNC based on workload type. For IDLE
	 * class there is no subclassification and all the lb queues go on
	 * a single tree service_tree_idle.
	 * Counts are embedded in the lb_rb_root
	 */
	struct lb_rb_root service_trees[2][3];
	struct lb_rb_root service_tree_idle;

	unsigned long saved_wl_slice;
	enum wl_type_t saved_wl_type;
	enum wl_class_t saved_wl_class;

	/* number of requests that are on the dispatch list or inside driver */
	int dispatched;
	struct lb_ttime ttime;
	struct lbg_stats stats;	/* stats for this lbg */
	struct lbg_stats dead_stats;	/* stats pushed from dead children */
};

struct lb_io_cq {
	struct io_cq		icq;		/* must be the first member */
	struct lb_queue	*lbq[2];
	struct lb_ttime	ttime;
	int			ioprio;		/* the current ioprio */
#ifdef CONFIG_LB_GROUP_IOSCHED
	uint64_t		blkcg_serial_nr; /* the current blkcg serial */
#endif
};

/*
 * Per block device queue structure
 */
struct lb_data {
	struct request_queue *queue;
	/* Root service tree for lb_groups */
	struct lb_rb_root grp_service_tree;
	struct lb_group *root_group;

	/*
	 * The priority currently being served
	 */
	enum wl_class_t serving_wl_class;
	enum wl_type_t serving_wl_type;
	unsigned long workload_expires;
	struct lb_group *serving_group;

	/*
	 * Each priority tree is sorted by next_request position.  These
	 * trees are used when determining if two or more queues are
	 * interleaving requests (see lb_close_cooperator).
	 */
	struct rb_root prio_trees[LB_PRIO_LISTS];

	unsigned int busy_queues;
	unsigned int busy_sync_queues;

	int rq_in_driver;
	int rq_in_flight[2];

	/*
	 * queue-depth detection
	 */
	int rq_queued;
	int hw_tag;
	/*
	 * hw_tag can be
	 * -1 => indeterminate, (lb will behave as if NCQ is present, to allow better detection)
	 *  1 => NCQ is present (hw_tag_est_depth is the estimated max depth)
	 *  0 => no NCQ
	 */
	int hw_tag_est_depth;
	unsigned int hw_tag_samples;

	/*
	 * idle window management
	 */
	struct timer_list idle_slice_timer;
	struct work_struct unplug_work;

	struct lb_queue *active_queue;
	struct lb_io_cq *active_cic;

	/*
	 * async queue for each priority case
	 */
	struct lb_queue *async_lbq[2][IOPRIO_BE_NR];
	struct lb_queue *async_idle_lbq;

	sector_t last_position;

	/*
	 * tunables, see top of file
	 */
	unsigned int lb_quantum;
	unsigned int lb_fifo_expire[2];
	unsigned int lb_back_penalty;
	unsigned int lb_back_max;
	unsigned int lb_slice[2];
	unsigned int lb_slice_async_rq;
	unsigned int lb_slice_idle;
	unsigned int lb_group_idle;
	unsigned int lb_latency;
	unsigned int lb_target_latency;

	/*
	 * Fallback dummy lbq for extreme OOM conditions
	 */
	struct lb_queue oom_lbq;

	unsigned long last_delayed_sync;
};

static struct lb_group *lb_get_next_lbg(struct lb_data *lbd);

static struct lb_rb_root *st_for(struct lb_group *lbg,
					    enum wl_class_t class,
					    enum wl_type_t type)
{
	if (!lbg)
		return NULL;

	if (class == IDLE_WORKLOAD)
		return &lbg->service_tree_idle;

	return &lbg->service_trees[class][type];
}

enum lbq_state_flags {
	LB_LBQ_FLAG_on_rr = 0,	/* on round-robin busy list */
	LB_LBQ_FLAG_wait_request,	/* waiting for a request */
	LB_LBQ_FLAG_must_dispatch,	/* must be allowed a dispatch */
	LB_LBQ_FLAG_must_alloc_slice,	/* per-slice must_alloc flag */
	LB_LBQ_FLAG_fifo_expire,	/* FIFO checked in this slice */
	LB_LBQ_FLAG_idle_window,	/* slice idling enabled */
	LB_LBQ_FLAG_prio_changed,	/* task priority has changed */
	LB_LBQ_FLAG_slice_new,	/* no requests dispatched in slice */
	LB_LBQ_FLAG_sync,		/* synchronous queue */
	LB_LBQ_FLAG_coop,		/* lbq is shared */
	LB_LBQ_FLAG_split_coop,	/* shared lbq will be splitted */
	LB_LBQ_FLAG_deep,		/* sync lbq experienced large depth */
	LB_LBQ_FLAG_wait_busy,	/* Waiting for next request */
};

#define LB_LBQ_FNS(name)						\
static inline void lb_mark_lbq_##name(struct lb_queue *lbq)		\
{									\
	(lbq)->flags |= (1 << LB_LBQ_FLAG_##name);			\
}									\
static inline void lb_clear_lbq_##name(struct lb_queue *lbq)	\
{									\
	(lbq)->flags &= ~(1 << LB_LBQ_FLAG_##name);			\
}									\
static inline int lb_lbq_##name(const struct lb_queue *lbq)		\
{									\
	return ((lbq)->flags & (1 << LB_LBQ_FLAG_##name)) != 0;	\
}

LB_LBQ_FNS(on_rr);
LB_LBQ_FNS(wait_request);
LB_LBQ_FNS(must_dispatch);
LB_LBQ_FNS(must_alloc_slice);
LB_LBQ_FNS(fifo_expire);
LB_LBQ_FNS(idle_window);
LB_LBQ_FNS(prio_changed);
LB_LBQ_FNS(slice_new);
LB_LBQ_FNS(sync);
LB_LBQ_FNS(coop);
LB_LBQ_FNS(split_coop);
LB_LBQ_FNS(deep);
LB_LBQ_FNS(wait_busy);
#undef LB_LBQ_FNS

static inline struct lb_group *pd_to_lbg(struct blkg_policy_data *pd)
{
	return pd ? container_of(pd, struct lb_group, pd) : NULL;
}

static inline struct blkcg_gq *lbg_to_blkg(struct lb_group *lbg)
{
	return pd_to_blkg(&lbg->pd);
}

#if defined(CONFIG_LB_GROUP_IOSCHED) && defined(CONFIG_DEBUG_BLK_CGROUP)

/* lbg stats flags */
enum lbg_stats_flags {
	LBG_stats_waiting = 0,
	LBG_stats_idling,
	LBG_stats_empty,
};

#define LBG_FLAG_FNS(name)						\
static inline void lbg_stats_mark_##name(struct lbg_stats *stats)	\
{									\
	stats->flags |= (1 << LBG_stats_##name);			\
}									\
static inline void lbg_stats_clear_##name(struct lbg_stats *stats)	\
{									\
	stats->flags &= ~(1 << LBG_stats_##name);			\
}									\
static inline int lbg_stats_##name(struct lbg_stats *stats)		\
{									\
	return (stats->flags & (1 << LBG_stats_##name)) != 0;		\
}									\

LBG_FLAG_FNS(waiting)
LBG_FLAG_FNS(idling)
LBG_FLAG_FNS(empty)
#undef LBG_FLAG_FNS

/* This should be called with the queue_lock held. */
static void lbg_stats_update_group_wait_time(struct lbg_stats *stats)
{
	unsigned long long now;

	if (!lbg_stats_waiting(stats))
		return;

	now = sched_clock();
	if (time_after64(now, stats->start_group_wait_time))
		blkg_stat_add(&stats->group_wait_time,
			      now - stats->start_group_wait_time);
	lbg_stats_clear_waiting(stats);
}

/* This should be called with the queue_lock held. */
static void lbg_stats_set_start_group_wait_time(struct lb_group *lbg,
						 struct lb_group *curr_lbg)
{
	struct lbg_stats *stats = &lbg->stats;

	if (lbg_stats_waiting(stats))
		return;
	if (lbg == curr_lbg)
		return;
	stats->start_group_wait_time = sched_clock();
	lbg_stats_mark_waiting(stats);
}

/* This should be called with the queue_lock held. */
static void lbg_stats_end_empty_time(struct lbg_stats *stats)
{
	unsigned long long now;

	if (!lbg_stats_empty(stats))
		return;

	now = sched_clock();
	if (time_after64(now, stats->start_empty_time))
		blkg_stat_add(&stats->empty_time,
			      now - stats->start_empty_time);
	lbg_stats_clear_empty(stats);
}

static void lbg_stats_update_dequeue(struct lb_group *lbg)
{
	blkg_stat_add(&lbg->stats.dequeue, 1);
}

static void lbg_stats_set_start_empty_time(struct lb_group *lbg)
{
	struct lbg_stats *stats = &lbg->stats;

	if (blkg_rwstat_total(&stats->queued))
		return;

	/*
	 * group is already marked empty. This can happen if lbq got new
	 * request in parent group and moved to this group while being added
	 * to service tree. Just ignore the event and move on.
	 */
	if (lbg_stats_empty(stats))
		return;

	stats->start_empty_time = sched_clock();
	lbg_stats_mark_empty(stats);
}

static void lbg_stats_update_idle_time(struct lb_group *lbg)
{
	struct lbg_stats *stats = &lbg->stats;

	if (lbg_stats_idling(stats)) {
		unsigned long long now = sched_clock();

		if (time_after64(now, stats->start_idle_time))
			blkg_stat_add(&stats->idle_time,
				      now - stats->start_idle_time);
		lbg_stats_clear_idling(stats);
	}
}

static void lbg_stats_set_start_idle_time(struct lb_group *lbg)
{
	struct lbg_stats *stats = &lbg->stats;

	BUG_ON(lbg_stats_idling(stats));

	stats->start_idle_time = sched_clock();
	lbg_stats_mark_idling(stats);
}

static void lbg_stats_update_avg_queue_size(struct lb_group *lbg)
{
	struct lbg_stats *stats = &lbg->stats;

	blkg_stat_add(&stats->avg_queue_size_sum,
		      blkg_rwstat_total(&stats->queued));
	blkg_stat_add(&stats->avg_queue_size_samples, 1);
	lbg_stats_update_group_wait_time(stats);
}

#else	/* CONFIG_LB_GROUP_IOSCHED && CONFIG_DEBUG_BLK_CGROUP */

static inline void lbg_stats_set_start_group_wait_time(struct lb_group *lbg, struct lb_group *curr_lbg) { }
static inline void lbg_stats_end_empty_time(struct lbg_stats *stats) { }
static inline void lbg_stats_update_dequeue(struct lb_group *lbg) { }
static inline void lbg_stats_set_start_empty_time(struct lb_group *lbg) { }
static inline void lbg_stats_update_idle_time(struct lb_group *lbg) { }
static inline void lbg_stats_set_start_idle_time(struct lb_group *lbg) { }
static inline void lbg_stats_update_avg_queue_size(struct lb_group *lbg) { }

#endif	/* CONFIG_LB_GROUP_IOSCHED && CONFIG_DEBUG_BLK_CGROUP */

#ifdef CONFIG_LB_GROUP_IOSCHED

static struct blkcg_policy blkcg_policy_lb;

static inline struct lb_group *blkg_to_lbg(struct blkcg_gq *blkg)
{
	return pd_to_lbg(blkg_to_pd(blkg, &blkcg_policy_lb));
}

static inline struct lb_group *lbg_parent(struct lb_group *lbg)
{
	struct blkcg_gq *pblkg = lbg_to_blkg(lbg)->parent;

	return pblkg ? blkg_to_lbg(pblkg) : NULL;
}

static inline void lbg_get(struct lb_group *lbg)
{
	return blkg_get(lbg_to_blkg(lbg));
}

static inline void lbg_put(struct lb_group *lbg)
{
	return blkg_put(lbg_to_blkg(lbg));
}

#define lb_log_lbq(lbd, lbq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(lbg_to_blkg((lbq)->lbg), __pbuf, sizeof(__pbuf));	\
	blk_add_trace_msg((lbd)->queue, "lb%d%c%c %s " fmt, (lbq)->pid, \
			lb_lbq_sync((lbq)) ? 'S' : 'A',		\
			lbq_type((lbq)) == SYNC_NOIDLE_WORKLOAD ? 'N' : ' ',\
			  __pbuf, ##args);				\
} while (0)

#define lb_log_lbg(lbd, lbg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(lbg_to_blkg(lbg), __pbuf, sizeof(__pbuf));		\
	blk_add_trace_msg((lbd)->queue, "%s " fmt, __pbuf, ##args);	\
} while (0)

static inline void lbg_stats_update_io_add(struct lb_group *lbg,
					    struct lb_group *curr_lbg, int rw)
{
	blkg_rwstat_add(&lbg->stats.queued, rw, 1);
	lbg_stats_end_empty_time(&lbg->stats);
	lbg_stats_set_start_group_wait_time(lbg, curr_lbg);
}

static inline void lbg_stats_update_timeslice_used(struct lb_group *lbg,
			unsigned long time, unsigned long unaccounted_time)
{
	blkg_stat_add(&lbg->stats.time, time);
#ifdef CONFIG_DEBUG_BLK_CGROUP
	blkg_stat_add(&lbg->stats.unaccounted_time, unaccounted_time);
#endif
}

static inline void lbg_stats_update_io_remove(struct lb_group *lbg, int rw)
{
	blkg_rwstat_add(&lbg->stats.queued, rw, -1);
}

static inline void lbg_stats_update_io_merged(struct lb_group *lbg, int rw)
{
	blkg_rwstat_add(&lbg->stats.merged, rw, 1);
}

static inline void lbg_stats_update_dispatch(struct lb_group *lbg,
					      uint64_t bytes, int rw)
{
	blkg_stat_add(&lbg->stats.sectors, bytes >> 9);
	blkg_rwstat_add(&lbg->stats.serviced, rw, 1);
	blkg_rwstat_add(&lbg->stats.service_bytes, rw, bytes);
}

static inline void lbg_stats_update_completion(struct lb_group *lbg,
			uint64_t start_time, uint64_t io_start_time, int rw)
{
	struct lbg_stats *stats = &lbg->stats;
	unsigned long long now = sched_clock();

	if (time_after64(now, io_start_time))
		blkg_rwstat_add(&stats->service_time, rw, now - io_start_time);
	if (time_after64(io_start_time, start_time))
		blkg_rwstat_add(&stats->wait_time, rw,
				io_start_time - start_time);
}

/* @stats = 0 */
static void lbg_stats_reset(struct lbg_stats *stats)
{
	/* queued stats shouldn't be cleared */
	blkg_rwstat_reset(&stats->service_bytes);
	blkg_rwstat_reset(&stats->serviced);
	blkg_rwstat_reset(&stats->merged);
	blkg_rwstat_reset(&stats->service_time);
	blkg_rwstat_reset(&stats->wait_time);
	blkg_stat_reset(&stats->time);
#ifdef CONFIG_DEBUG_BLK_CGROUP
	blkg_stat_reset(&stats->unaccounted_time);
	blkg_stat_reset(&stats->avg_queue_size_sum);
	blkg_stat_reset(&stats->avg_queue_size_samples);
	blkg_stat_reset(&stats->dequeue);
	blkg_stat_reset(&stats->group_wait_time);
	blkg_stat_reset(&stats->idle_time);
	blkg_stat_reset(&stats->empty_time);
#endif
}

/* @to += @from */
static void lbg_stats_merge(struct lbg_stats *to, struct lbg_stats *from)
{
	/* queued stats shouldn't be cleared */
	blkg_rwstat_merge(&to->service_bytes, &from->service_bytes);
	blkg_rwstat_merge(&to->serviced, &from->serviced);
	blkg_rwstat_merge(&to->merged, &from->merged);
	blkg_rwstat_merge(&to->service_time, &from->service_time);
	blkg_rwstat_merge(&to->wait_time, &from->wait_time);
	blkg_stat_merge(&from->time, &from->time);
#ifdef CONFIG_DEBUG_BLK_CGROUP
	blkg_stat_merge(&to->unaccounted_time, &from->unaccounted_time);
	blkg_stat_merge(&to->avg_queue_size_sum, &from->avg_queue_size_sum);
	blkg_stat_merge(&to->avg_queue_size_samples, &from->avg_queue_size_samples);
	blkg_stat_merge(&to->dequeue, &from->dequeue);
	blkg_stat_merge(&to->group_wait_time, &from->group_wait_time);
	blkg_stat_merge(&to->idle_time, &from->idle_time);
	blkg_stat_merge(&to->empty_time, &from->empty_time);
#endif
}

/*
 * Transfer @lbg's stats to its parent's dead_stats so that the ancestors'
 * recursive stats can still account for the amount used by this lbg after
 * it's gone.
 */
static void lbg_stats_xfer_dead(struct lb_group *lbg)
{
	struct lb_group *parent = lbg_parent(lbg);

	lockdep_assert_held(lbg_to_blkg(lbg)->q->queue_lock);

	if (unlikely(!parent))
		return;

	lbg_stats_merge(&parent->dead_stats, &lbg->stats);
	lbg_stats_merge(&parent->dead_stats, &lbg->dead_stats);
	lbg_stats_reset(&lbg->stats);
	lbg_stats_reset(&lbg->dead_stats);
}

#else	/* CONFIG_LB_GROUP_IOSCHED */

static inline struct lb_group *lbg_parent(struct lb_group *lbg) { return NULL; }
static inline void lbg_get(struct lb_group *lbg) { }
static inline void lbg_put(struct lb_group *lbg) { }

#define lb_log_lbq(lbd, lbq, fmt, args...)	\
	blk_add_trace_msg((lbd)->queue, "lb%d%c%c " fmt, (lbq)->pid,	\
			lb_lbq_sync((lbq)) ? 'S' : 'A',		\
			lbq_type((lbq)) == SYNC_NOIDLE_WORKLOAD ? 'N' : ' ',\
				##args)
#define lb_log_lbg(lbd, lbg, fmt, args...)		do {} while (0)

static inline void lbg_stats_update_io_add(struct lb_group *lbg,
			struct lb_group *curr_lbg, int rw) { }
static inline void lbg_stats_update_timeslice_used(struct lb_group *lbg,
			unsigned long time, unsigned long unaccounted_time) { }
static inline void lbg_stats_update_io_remove(struct lb_group *lbg, int rw) { }
static inline void lbg_stats_update_io_merged(struct lb_group *lbg, int rw) { }
static inline void lbg_stats_update_dispatch(struct lb_group *lbg,
					      uint64_t bytes, int rw) { }
static inline void lbg_stats_update_completion(struct lb_group *lbg,
			uint64_t start_time, uint64_t io_start_time, int rw) { }

#endif	/* CONFIG_LB_GROUP_IOSCHED */

#define lb_log(lbd, fmt, args...)	\
	blk_add_trace_msg((lbd)->queue, "lb " fmt, ##args)

/* Traverses through lb group service trees */
#define for_each_lbg_st(lbg, i, j, st) \
	for (i = 0; i <= IDLE_WORKLOAD; i++) \
		for (j = 0, st = i < IDLE_WORKLOAD ? &lbg->service_trees[i][j]\
			: &lbg->service_tree_idle; \
			(i < IDLE_WORKLOAD && j <= SYNC_WORKLOAD) || \
			(i == IDLE_WORKLOAD && j == 0); \
			j++, st = i < IDLE_WORKLOAD ? \
			&lbg->service_trees[i][j]: NULL) \

static inline bool lb_io_thinktime_big(struct lb_data *lbd,
	struct lb_ttime *ttime, bool group_idle)
{
	unsigned long slice;
	if (!sample_valid(ttime->ttime_samples))
		return false;
	if (group_idle)
		slice = lbd->lb_group_idle;
	else
		slice = lbd->lb_slice_idle;
	return ttime->ttime_mean > slice;
}

static inline bool iops_mode(struct lb_data *lbd)
{
	/*
	 * If we are not idling on queues and it is a NCQ drive, parallel
	 * execution of requests is on and measuring time is not possible
	 * in most of the cases until and unless we drive shallower queue
	 * depths and that becomes a performance bottleneck. In such cases
	 * switch to start providing fairness in terms of number of IOs.
	 */
	if (!lbd->lb_slice_idle && lbd->hw_tag)
		return true;
	else
		return false;
}

static inline enum wl_class_t lbq_class(struct lb_queue *lbq)
{
	if (lb_class_idle(lbq))
		return IDLE_WORKLOAD;
	if (lb_class_rt(lbq))
		return RT_WORKLOAD;
	return BE_WORKLOAD;
}


static enum wl_type_t lbq_type(struct lb_queue *lbq)
{
	if (!lb_lbq_sync(lbq))
		return ASYNC_WORKLOAD;
	if (!lb_lbq_idle_window(lbq))
		return SYNC_NOIDLE_WORKLOAD;
	return SYNC_WORKLOAD;
}

static inline int lb_group_busy_queues_wl(enum wl_class_t wl_class,
					struct lb_data *lbd,
					struct lb_group *lbg)
{
	if (wl_class == IDLE_WORKLOAD)
		return lbg->service_tree_idle.count;

	return lbg->service_trees[wl_class][ASYNC_WORKLOAD].count +
		lbg->service_trees[wl_class][SYNC_NOIDLE_WORKLOAD].count +
		lbg->service_trees[wl_class][SYNC_WORKLOAD].count;
}

static inline int lbg_busy_async_queues(struct lb_data *lbd,
					struct lb_group *lbg)
{
	return lbg->service_trees[RT_WORKLOAD][ASYNC_WORKLOAD].count +
		lbg->service_trees[BE_WORKLOAD][ASYNC_WORKLOAD].count;
}

static void lb_dispatch_insert(struct request_queue *, struct request *);
static struct lb_queue *lb_get_queue(struct lb_data *lbd, bool is_sync,
				       struct lb_io_cq *cic, struct bio *bio,
				       gfp_t gfp_mask);

static inline struct lb_io_cq *icq_to_cic(struct io_cq *icq)
{
	/* cic->icq is the first member, %NULL will convert to %NULL */
	return container_of(icq, struct lb_io_cq, icq);
}

static inline struct lb_io_cq *lb_cic_lookup(struct lb_data *lbd,
					       struct io_context *ioc)
{
	if (ioc)
		return icq_to_cic(ioc_lookup_icq(ioc, lbd->queue));
	return NULL;
}

static inline struct lb_queue *cic_to_lbq(struct lb_io_cq *cic, bool is_sync)
{
	return cic->lbq[is_sync];
}

static inline void cic_set_lbq(struct lb_io_cq *cic, struct lb_queue *lbq,
				bool is_sync)
{
	cic->lbq[is_sync] = lbq;
}

static inline struct lb_data *cic_to_lbd(struct lb_io_cq *cic)
{
	return cic->icq.q->elevator->elevator_data;
}

/*
 * We regard a request as SYNC, if it's either a read or has the SYNC bit
 * set (in which case it could also be direct WRITE).
 */
static inline bool lb_bio_sync(struct bio *bio)
{
	return bio_data_dir(bio) == READ || (bio->bi_rw & REQ_SYNC);
}

/*
 * scheduler run of queue, if there are requests pending and no one in the
 * driver that will restart queueing
 */
static inline void lb_schedule_dispatch(struct lb_data *lbd)
{
	if (lbd->busy_queues) {
		lb_log(lbd, "schedule dispatch");
		kblockd_schedule_work(&lbd->unplug_work);
	}
}

/*
 * Scale schedule slice based on io priority. Use the sync time slice only
 * if a queue is marked sync and has sync io queued. A sync queue with async
 * io only, should not get full sync slice length.
 */
static inline int lb_prio_slice(struct lb_data *lbd, bool sync,
				 unsigned short prio)
{
	const int base_slice = lbd->lb_slice[sync];

	WARN_ON(prio >= IOPRIO_BE_NR);

	return base_slice + (base_slice/LB_SLICE_SCALE * (4 - prio));
}

static inline int
lb_prio_to_slice(struct lb_data *lbd, struct lb_queue *lbq)
{
	return lb_prio_slice(lbd, lb_lbq_sync(lbq), lbq->ioprio);
}

/**
 * lbg_scale_charge - scale disk time charge according to lbg weight
 * @charge: disk time being charged
 * @vfraction: vfraction of the lbg, fixed point w/ LB_SERVICE_SHIFT
 *
 * Scale @charge according to @vfraction, which is in range (0, 1].  The
 * scaling is inversely proportional.
 *
 * scaled = charge / vfraction
 *
 * The result is also in fixed point w/ LB_SERVICE_SHIFT.
 */
static inline u64 lbg_scale_charge(unsigned long charge,
				    unsigned int vfraction)
{
	u64 c = charge << LB_SERVICE_SHIFT;	/* make it fixed point */

	/* charge / vfraction */
	c <<= LB_SERVICE_SHIFT;
	do_div(c, vfraction);
	return c;
}

static inline u64 max_vdisktime(u64 min_vdisktime, u64 vdisktime)
{
	s64 delta = (s64)(vdisktime - min_vdisktime);
	if (delta > 0)
		min_vdisktime = vdisktime;

	return min_vdisktime;
}

static inline u64 min_vdisktime(u64 min_vdisktime, u64 vdisktime)
{
	s64 delta = (s64)(vdisktime - min_vdisktime);
	if (delta < 0)
		min_vdisktime = vdisktime;

	return min_vdisktime;
}

static void update_min_vdisktime(struct lb_rb_root *st)
{
	struct lb_group *lbg;

	if (st->left) {
		lbg = rb_entry_lbg(st->left);
		st->min_vdisktime = max_vdisktime(st->min_vdisktime,
						  lbg->vdisktime);
	}
}

/*
 * get averaged number of queues of RT/BE priority.
 * average is updated, with a formula that gives more weight to higher numbers,
 * to quickly follows sudden increases and decrease slowly
 */

static inline unsigned lb_group_get_avg_queues(struct lb_data *lbd,
					struct lb_group *lbg, bool rt)
{
	unsigned min_q, max_q;
	unsigned mult  = lb_hist_divisor - 1;
	unsigned round = lb_hist_divisor / 2;
	unsigned busy = lb_group_busy_queues_wl(rt, lbd, lbg);

	min_q = min(lbg->busy_queues_avg[rt], busy);
	max_q = max(lbg->busy_queues_avg[rt], busy);
	lbg->busy_queues_avg[rt] = (mult * max_q + min_q + round) /
		lb_hist_divisor;
	return lbg->busy_queues_avg[rt];
}

static inline unsigned
lb_group_slice(struct lb_data *lbd, struct lb_group *lbg)
{
	return lbd->lb_target_latency * lbg->vfraction >> LB_SERVICE_SHIFT;
}

static inline unsigned
lb_scaled_lbq_slice(struct lb_data *lbd, struct lb_queue *lbq)
{
	unsigned slice = lb_prio_to_slice(lbd, lbq);
	if (lbd->lb_latency) {
		/*
		 * interested queues (we consider only the ones with the same
		 * priority class in the lb group)
		 */
		unsigned iq = lb_group_get_avg_queues(lbd, lbq->lbg,
						lb_class_rt(lbq));
		unsigned sync_slice = lbd->lb_slice[1];
		unsigned expect_latency = sync_slice * iq;
		unsigned group_slice = lb_group_slice(lbd, lbq->lbg);

		if (expect_latency > group_slice) {
			unsigned base_low_slice = 2 * lbd->lb_slice_idle;
			/* scale low_slice according to IO priority
			 * and sync vs async */
			unsigned low_slice =
				min(slice, base_low_slice * slice / sync_slice);
			/* the adapted slice value is scaled to fit all iqs
			 * into the target latency */
			slice = max(slice * group_slice / expect_latency,
				    low_slice);
		}
	}
	return slice;
}

static inline void
lb_set_prio_slice(struct lb_data *lbd, struct lb_queue *lbq)
{
	unsigned slice = lb_scaled_lbq_slice(lbd, lbq);

	lbq->slice_start = jiffies;
	lbq->slice_end = jiffies + slice;
	lbq->allocated_slice = slice;
	lb_log_lbq(lbd, lbq, "set_slice=%lu", lbq->slice_end - jiffies);
}

/*
 * We need to wrap this check in lb_lbq_slice_new(), since ->slice_end
 * isn't valid until the first request from the dispatch is activated
 * and the slice time set.
 */
static inline bool lb_slice_used(struct lb_queue *lbq)
{
	if (lb_lbq_slice_new(lbq))
		return false;
	if (time_before(jiffies, lbq->slice_end))
		return false;

	return true;
}

/*
 * Lifted from AS - choose which of rq1 and rq2 that is best served now.
 * We choose the request that is closest to the head right now. Distance
 * behind the head is penalized and only allowed to a certain extent.
 */
static struct request *
lb_choose_req(struct lb_data *lbd, struct request *rq1, struct request *rq2, sector_t last)
{
	sector_t s1, s2, d1 = 0, d2 = 0;
	unsigned long back_max;
#define LB_RQ1_WRAP	0x01 /* request 1 wraps */
#define LB_RQ2_WRAP	0x02 /* request 2 wraps */
	unsigned wrap = 0; /* bit mask: requests behind the disk head? */

	if (rq1 == NULL || rq1 == rq2)
		return rq2;
	if (rq2 == NULL)
		return rq1;

	if (rq_is_sync(rq1) != rq_is_sync(rq2))
		return rq_is_sync(rq1) ? rq1 : rq2;

	if ((rq1->cmd_flags ^ rq2->cmd_flags) & REQ_PRIO)
		return rq1->cmd_flags & REQ_PRIO ? rq1 : rq2;

	s1 = blk_rq_pos(rq1);
	s2 = blk_rq_pos(rq2);

	/*
	 * by definition, 1KiB is 2 sectors
	 */
	back_max = lbd->lb_back_max * 2;

	/*
	 * Strict one way elevator _except_ in the case where we allow
	 * short backward seeks which are biased as twice the cost of a
	 * similar forward seek.
	 */
	if (s1 >= last)
		d1 = s1 - last;
	else if (s1 + back_max >= last)
		d1 = (last - s1) * lbd->lb_back_penalty;
	else
		wrap |= LB_RQ1_WRAP;

	if (s2 >= last)
		d2 = s2 - last;
	else if (s2 + back_max >= last)
		d2 = (last - s2) * lbd->lb_back_penalty;
	else
		wrap |= LB_RQ2_WRAP;

	/* Found required data */

	/*
	 * By doing switch() on the bit mask "wrap" we avoid having to
	 * check two variables for all permutations: --> faster!
	 */
	switch (wrap) {
	case 0: /* common case for LB: rq1 and rq2 not wrapped */
		if (d1 < d2)
			return rq1;
		else if (d2 < d1)
			return rq2;
		else {
			if (s1 >= s2)
				return rq1;
			else
				return rq2;
		}

	case LB_RQ2_WRAP:
		return rq1;
	case LB_RQ1_WRAP:
		return rq2;
	case (LB_RQ1_WRAP|LB_RQ2_WRAP): /* both rqs wrapped */
	default:
		/*
		 * Since both rqs are wrapped,
		 * start with the one that's further behind head
		 * (--> only *one* back seek required),
		 * since back seek takes more time than forward.
		 */
		if (s1 <= s2)
			return rq1;
		else
			return rq2;
	}
}

/*
 * The below is leftmost cache rbtree addon
 */
static struct lb_queue *lb_rb_first(struct lb_rb_root *root)
{
	/* Service tree is empty */
	if (!root->count)
		return NULL;

	if (!root->left)
		root->left = rb_first(&root->rb);

	if (root->left)
		return rb_entry(root->left, struct lb_queue, rb_node);

	return NULL;
}

static struct lb_group *lb_rb_first_group(struct lb_rb_root *root)
{
	if (!root->left)
		root->left = rb_first(&root->rb);

	if (root->left)
		return rb_entry_lbg(root->left);

	return NULL;
}

static void rb_erase_init(struct rb_node *n, struct rb_root *root)
{
	rb_erase(n, root);
	RB_CLEAR_NODE(n);
}

static void lb_rb_erase(struct rb_node *n, struct lb_rb_root *root)
{
	if (root->left == n)
		root->left = NULL;
	rb_erase_init(n, &root->rb);
	--root->count;
}

/*
 * would be nice to take fifo expire time into account as well
 */
static struct request *
lb_find_next_rq(struct lb_data *lbd, struct lb_queue *lbq,
		  struct request *last)
{
	struct rb_node *rbnext = rb_next(&last->rb_node);
	struct rb_node *rbprev = rb_prev(&last->rb_node);
	struct request *next = NULL, *prev = NULL;

	BUG_ON(RB_EMPTY_NODE(&last->rb_node));

	if (rbprev)
		prev = rb_entry_rq(rbprev);

	if (rbnext)
		next = rb_entry_rq(rbnext);
	else {
		rbnext = rb_first(&lbq->sort_list);
		if (rbnext && rbnext != &last->rb_node)
			next = rb_entry_rq(rbnext);
	}

	return lb_choose_req(lbd, next, prev, blk_rq_pos(last));
}

static unsigned long lb_slice_offset(struct lb_data *lbd,
				      struct lb_queue *lbq)
{
	/*
	 * just an approximation, should be ok.
	 */
	return (lbq->lbg->nr_lbq - 1) * (lb_prio_slice(lbd, 1, 0) -
		       lb_prio_slice(lbd, lb_lbq_sync(lbq), lbq->ioprio));
}

static inline s64
lbg_key(struct lb_rb_root *st, struct lb_group *lbg)
{
	return lbg->vdisktime - st->min_vdisktime;
}

static void
__lb_group_service_tree_add(struct lb_rb_root *st, struct lb_group *lbg)
{
	struct rb_node **node = &st->rb.rb_node;
	struct rb_node *parent = NULL;
	struct lb_group *__lbg;
	s64 key = lbg_key(st, lbg);
	int left = 1;

	while (*node != NULL) {
		parent = *node;
		__lbg = rb_entry_lbg(parent);

		if (key < lbg_key(st, __lbg))
			node = &parent->rb_left;
		else {
			node = &parent->rb_right;
			left = 0;
		}
	}

	if (left)
		st->left = &lbg->rb_node;

	rb_link_node(&lbg->rb_node, parent, node);
	rb_insert_color(&lbg->rb_node, &st->rb);
}

/*
 * This has to be called only on activation of lbg
 */
static void
lb_update_group_weight(struct lb_group *lbg)
{
	if (lbg->new_weight) {
		lbg->weight = lbg->new_weight;
		lbg->new_weight = 0;
	}
}

static void
lb_update_group_leaf_weight(struct lb_group *lbg)
{
	BUG_ON(!RB_EMPTY_NODE(&lbg->rb_node));

	if (lbg->new_leaf_weight) {
		lbg->leaf_weight = lbg->new_leaf_weight;
		lbg->new_leaf_weight = 0;
	}
}

static void
lb_group_service_tree_add(struct lb_rb_root *st, struct lb_group *lbg)
{
	unsigned int vfr = 1 << LB_SERVICE_SHIFT;	/* start with 1 */
	struct lb_group *pos = lbg;
	struct lb_group *parent;
	bool propagate;

	/* add to the service tree */
	BUG_ON(!RB_EMPTY_NODE(&lbg->rb_node));

	/*
	 * Update leaf_weight.  We cannot update weight at this point
	 * because lbg might already have been activated and is
	 * contributing its current weight to the parent's child_weight.
	 */
	lb_update_group_leaf_weight(lbg);
	__lb_group_service_tree_add(st, lbg);

	/*
	 * Activate @lbg and calculate the portion of vfraction @lbg is
	 * entitled to.  vfraction is calculated by walking the tree
	 * towards the root calculating the fraction it has at each level.
	 * The compounded ratio is how much vfraction @lbg owns.
	 *
	 * Start with the proportion tasks in this lbg has against active
	 * children lbgs - its leaf_weight against children_weight.
	 */
	propagate = !pos->nr_active++;
	pos->children_weight += pos->leaf_weight;
	vfr = vfr * pos->leaf_weight / pos->children_weight;

	/*
	 * Compound ->weight walking up the tree.  Both activation and
	 * vfraction calculation are done in the same loop.  Propagation
	 * stops once an already activated node is met.  vfraction
	 * calculation should always continue to the root.
	 */
	while ((parent = lbg_parent(pos))) {
		if (propagate) {
			lb_update_group_weight(pos);
			propagate = !parent->nr_active++;
			parent->children_weight += pos->weight;
		}
		vfr = vfr * pos->weight / parent->children_weight;
		pos = parent;
	}

	lbg->vfraction = max_t(unsigned, vfr, 1);
}

static void
lb_group_notify_queue_add(struct lb_data *lbd, struct lb_group *lbg)
{
	struct lb_rb_root *st = &lbd->grp_service_tree;
	struct lb_group *__lbg;
	struct rb_node *n;

	lbg->nr_lbq++;
	if (!RB_EMPTY_NODE(&lbg->rb_node))
		return;

	/*
	 * Currently put the group at the end. Later implement something
	 * so that groups get lesser vtime based on their weights, so that
	 * if group does not loose all if it was not continuously backlogged.
	 */
	n = rb_last(&st->rb);
	if (n) {
		__lbg = rb_entry_lbg(n);
		lbg->vdisktime = __lbg->vdisktime + LB_IDLE_DELAY;
	} else
		lbg->vdisktime = st->min_vdisktime;
	lb_group_service_tree_add(st, lbg);
}

static void
lb_group_service_tree_del(struct lb_rb_root *st, struct lb_group *lbg)
{
	struct lb_group *pos = lbg;
	bool propagate;

	/*
	 * Undo activation from lb_group_service_tree_add().  Deactivate
	 * @lbg and propagate deactivation upwards.
	 */
	propagate = !--pos->nr_active;
	pos->children_weight -= pos->leaf_weight;

	while (propagate) {
		struct lb_group *parent = lbg_parent(pos);

		/* @pos has 0 nr_active at this point */
		WARN_ON_ONCE(pos->children_weight);
		pos->vfraction = 0;

		if (!parent)
			break;

		propagate = !--parent->nr_active;
		parent->children_weight -= pos->weight;
		pos = parent;
	}

	/* remove from the service tree */
	if (!RB_EMPTY_NODE(&lbg->rb_node))
		lb_rb_erase(&lbg->rb_node, st);
}

static void
lb_group_notify_queue_del(struct lb_data *lbd, struct lb_group *lbg)
{
	struct lb_rb_root *st = &lbd->grp_service_tree;

	BUG_ON(lbg->nr_lbq < 1);
	lbg->nr_lbq--;

	/* If there are other lb queues under this group, don't delete it */
	if (lbg->nr_lbq)
		return;

	lb_log_lbg(lbd, lbg, "del_from_rr group");
	lb_group_service_tree_del(st, lbg);
	lbg->saved_wl_slice = 0;
	lbg_stats_update_dequeue(lbg);
}

static inline unsigned int lb_lbq_slice_usage(struct lb_queue *lbq,
						unsigned int *unaccounted_time)
{
	unsigned int slice_used;

	/*
	 * Queue got expired before even a single request completed or
	 * got expired immediately after first request completion.
	 */
	if (!lbq->slice_start || lbq->slice_start == jiffies) {
		/*
		 * Also charge the seek time incurred to the group, otherwise
		 * if there are mutiple queues in the group, each can dispatch
		 * a single request on seeky media and cause lots of seek time
		 * and group will never know it.
		 */
		slice_used = max_t(unsigned, (jiffies - lbq->dispatch_start),
					1);
	} else {
		slice_used = jiffies - lbq->slice_start;
		if (slice_used > lbq->allocated_slice) {
			*unaccounted_time = slice_used - lbq->allocated_slice;
			slice_used = lbq->allocated_slice;
		}
		if (time_after(lbq->slice_start, lbq->dispatch_start))
			*unaccounted_time += lbq->slice_start -
					lbq->dispatch_start;
	}

	return slice_used;
}

static void lb_group_served(struct lb_data *lbd, struct lb_group *lbg,
				struct lb_queue *lbq)
{
	struct lb_rb_root *st = &lbd->grp_service_tree;
	unsigned int used_sl, charge, unaccounted_sl = 0;
	int nr_sync = lbg->nr_lbq - lbg_busy_async_queues(lbd, lbg)
			- lbg->service_tree_idle.count;
	unsigned int vfr;

	BUG_ON(nr_sync < 0);
	used_sl = charge = lb_lbq_slice_usage(lbq, &unaccounted_sl);

	if (iops_mode(lbd)) // change iops mode
		charge = lbq->slice_dispatch;
	else if (!lb_lbq_sync(lbq) && !nr_sync)
		charge = lbq->allocated_slice;

	/*
	 * Can't update vdisktime while on service tree and lbg->vfraction
	 * is valid only while on it.  Cache vfr, leave the service tree,
	 * update vdisktime and go back on.  The re-addition to the tree
	 * will also update the weights as necessary.
	 */
	vfr = lbg->vfraction;
	lb_group_service_tree_del(st, lbg);
	lbg->vdisktime += lbg_scale_charge(charge, vfr);
	lb_group_service_tree_add(st, lbg);

	/* This group is being expired. Save the context */
	if (time_after(lbd->workload_expires, jiffies)) {
		lbg->saved_wl_slice = lbd->workload_expires
						- jiffies;
		lbg->saved_wl_type = lbd->serving_wl_type;
		lbg->saved_wl_class = lbd->serving_wl_class;
	} else
		lbg->saved_wl_slice = 0;

	lb_log_lbg(lbd, lbg, "served: vt=%llu min_vt=%llu", lbg->vdisktime,
					st->min_vdisktime);
	lb_log_lbq(lbq->lbd, lbq,
		     "sl_used=%u disp=%u charge=%u iops=%u sect=%lu",
		     used_sl, lbq->slice_dispatch, charge,
		     iops_mode(lbd), lbq->nr_sectors);
	lbg_stats_update_timeslice_used(lbg, used_sl, unaccounted_sl);
	lbg_stats_set_start_empty_time(lbg);
}

/**
 * lb_init_lbg_base - initialize base part of a lb_group
 * @lbg: lb_group to initialize
 *
 * Initialize the base part which is used whether %CONFIG_LB_GROUP_IOSCHED
 * is enabled or not.
 */
static void lb_init_lbg_base(struct lb_group *lbg)
{
	struct lb_rb_root *st;
	int i, j;

	for_each_lbg_st(lbg, i, j, st)
		*st = LB_RB_ROOT;
	RB_CLEAR_NODE(&lbg->rb_node);

	lbg->ttime.last_end_request = jiffies;
}

#ifdef CONFIG_LB_GROUP_IOSCHED
static void lbg_stats_init(struct lbg_stats *stats)
{
	blkg_rwstat_init(&stats->service_bytes);
	blkg_rwstat_init(&stats->serviced);
	blkg_rwstat_init(&stats->merged);
	blkg_rwstat_init(&stats->service_time);
	blkg_rwstat_init(&stats->wait_time);
	blkg_rwstat_init(&stats->queued);

	blkg_stat_init(&stats->sectors);
	blkg_stat_init(&stats->time);

#ifdef CONFIG_DEBUG_BLK_CGROUP
	blkg_stat_init(&stats->unaccounted_time);
	blkg_stat_init(&stats->avg_queue_size_sum);
	blkg_stat_init(&stats->avg_queue_size_samples);
	blkg_stat_init(&stats->dequeue);
	blkg_stat_init(&stats->group_wait_time);
	blkg_stat_init(&stats->idle_time);
	blkg_stat_init(&stats->empty_time);
#endif
}

static void lb_pd_init(struct blkcg_gq *blkg)
{
	struct lb_group *lbg = blkg_to_lbg(blkg);

	lb_init_lbg_base(lbg);
	lbg->weight = blkg->blkcg->lb_weight;
	lbg->leaf_weight = blkg->blkcg->lb_leaf_weight;
	lbg_stats_init(&lbg->stats);
	lbg_stats_init(&lbg->dead_stats);
}

static void lb_pd_offline(struct blkcg_gq *blkg)
{
	/*
	 * @blkg is going offline and will be ignored by
	 * blkg_[rw]stat_recursive_sum().  Transfer stats to the parent so
	 * that they don't get lost.  If IOs complete after this point, the
	 * stats for them will be lost.  Oh well...
	 */
	lbg_stats_xfer_dead(blkg_to_lbg(blkg));
}

/* offset delta from lbg->stats to lbg->dead_stats */
static const int dead_stats_off_delta = offsetof(struct lb_group, dead_stats) -
					offsetof(struct lb_group, stats);

/* to be used by recursive prfill, sums live and dead stats recursively */
static u64 lbg_stat_pd_recursive_sum(struct blkg_policy_data *pd, int off)
{
	u64 sum = 0;

	sum += blkg_stat_recursive_sum(pd, off);
	sum += blkg_stat_recursive_sum(pd, off + dead_stats_off_delta);
	return sum;
}

/* to be used by recursive prfill, sums live and dead rwstats recursively */
static struct blkg_rwstat lbg_rwstat_pd_recursive_sum(struct blkg_policy_data *pd,
						       int off)
{
	struct blkg_rwstat a, b;

	a = blkg_rwstat_recursive_sum(pd, off);
	b = blkg_rwstat_recursive_sum(pd, off + dead_stats_off_delta);
	blkg_rwstat_merge(&a, &b);
	return a;
}

static void lb_pd_reset_stats(struct blkcg_gq *blkg)
{
	struct lb_group *lbg = blkg_to_lbg(blkg);

	lbg_stats_reset(&lbg->stats);
	lbg_stats_reset(&lbg->dead_stats);
}

/*
 * Search for the lb group current task belongs to. request_queue lock must
 * be held.
 */
static struct lb_group *lb_lookup_create_lbg(struct lb_data *lbd,
						struct blkcg *blkcg)
{
	struct request_queue *q = lbd->queue;
	struct lb_group *lbg = NULL;

	/* avoid lookup for the common case where there's no blkcg */
	if (blkcg == &blkcg_root) {
		lbg = lbd->root_group;
	} else {
		struct blkcg_gq *blkg;

		blkg = blkg_lookup_create(blkcg, q);
		if (!IS_ERR(blkg))
			lbg = blkg_to_lbg(blkg);
	}

	return lbg;
}

static void lb_link_lbq_lbg(struct lb_queue *lbq, struct lb_group *lbg)
{
	/* Currently, all async queues are mapped to root group */
	if (!lb_lbq_sync(lbq))
		lbg = lbq->lbd->root_group;

	lbq->lbg = lbg;
	/* lbq reference on lbg */
	lbg_get(lbg);
}

static u64 lbg_prfill_weight_device(struct seq_file *sf,
				     struct blkg_policy_data *pd, int off)
{
	struct lb_group *lbg = pd_to_lbg(pd);

	if (!lbg->dev_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, lbg->dev_weight);
}

static int lbg_print_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  lbg_prfill_weight_device, &blkcg_policy_lb,
			  0, false);
	return 0;
}

static u64 lbg_prfill_leaf_weight_device(struct seq_file *sf,
					  struct blkg_policy_data *pd, int off)
{
	struct lb_group *lbg = pd_to_lbg(pd);

	if (!lbg->dev_leaf_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, lbg->dev_leaf_weight);
}

static int lbg_print_leaf_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  lbg_prfill_leaf_weight_device, &blkcg_policy_lb,
			  0, false);
	return 0;
}

static int lb_print_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->lb_weight);
	return 0;
}

static int lb_print_leaf_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->lb_leaf_weight);
	return 0;
}

static ssize_t __lbg_set_weight_device(struct kernfs_open_file *of,
					char *buf, size_t nbytes, loff_t off,
					bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(of_css(of));
	struct blkg_conf_ctx ctx;
	struct lb_group *lbg;
	int ret;

	ret = blkg_conf_prep(blkcg, &blkcg_policy_lb, buf, &ctx);
	if (ret)
		return ret;

	ret = -EINVAL;
	lbg = blkg_to_lbg(ctx.blkg);
	if (!ctx.v || (ctx.v >= LB_WEIGHT_MIN && ctx.v <= LB_WEIGHT_MAX)) {
		if (!is_leaf_weight) {
			lbg->dev_weight = ctx.v;
			lbg->new_weight = ctx.v ?: blkcg->lb_weight;
		} else {
			lbg->dev_leaf_weight = ctx.v;
			lbg->new_leaf_weight = ctx.v ?: blkcg->lb_leaf_weight;
		}
		ret = 0;
	}

	blkg_conf_finish(&ctx);
	return ret ?: nbytes;
}

static ssize_t lbg_set_weight_device(struct kernfs_open_file *of,
				      char *buf, size_t nbytes, loff_t off)
{
	return __lbg_set_weight_device(of, buf, nbytes, off, false);
}

static ssize_t lbg_set_leaf_weight_device(struct kernfs_open_file *of,
					   char *buf, size_t nbytes, loff_t off)
{
	return __lbg_set_weight_device(of, buf, nbytes, off, true);
}

static int __lb_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
			    u64 val, bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(css);
	struct blkcg_gq *blkg;

	if (val < LB_WEIGHT_MIN || val > LB_WEIGHT_MAX)
		return -EINVAL;

	spin_lock_irq(&blkcg->lock);

	if (!is_leaf_weight)
		blkcg->lb_weight = val;
	else
		blkcg->lb_leaf_weight = val;

	hlist_for_each_entry(blkg, &blkcg->blkg_list, blkcg_node) {
		struct lb_group *lbg = blkg_to_lbg(blkg);

		if (!lbg)
			continue;

		if (!is_leaf_weight) {
			if (!lbg->dev_weight)
				lbg->new_weight = blkcg->lb_weight;
		} else {
			if (!lbg->dev_leaf_weight)
				lbg->new_leaf_weight = blkcg->lb_leaf_weight;
		}
	}

	spin_unlock_irq(&blkcg->lock);
	return 0;
}

static int lb_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
			  u64 val)
{
	return __lb_set_weight(css, cft, val, false);
}

static int lb_set_leaf_weight(struct cgroup_subsys_state *css,
			       struct cftype *cft, u64 val)
{
	return __lb_set_weight(css, cft, val, true);
}

static int lbg_print_stat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_stat,
			  &blkcg_policy_lb, seq_cft(sf)->private, false);
	return 0;
}

static int lbg_print_rwstat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_rwstat,
			  &blkcg_policy_lb, seq_cft(sf)->private, true);
	return 0;
}

static u64 lbg_prfill_stat_recursive(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	u64 sum = lbg_stat_pd_recursive_sum(pd, off);

	return __blkg_prfill_u64(sf, pd, sum);
}

static u64 lbg_prfill_rwstat_recursive(struct seq_file *sf,
					struct blkg_policy_data *pd, int off)
{
	struct blkg_rwstat sum = lbg_rwstat_pd_recursive_sum(pd, off);

	return __blkg_prfill_rwstat(sf, pd, &sum);
}

static int lbg_print_stat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  lbg_prfill_stat_recursive, &blkcg_policy_lb,
			  seq_cft(sf)->private, false);
	return 0;
}

static int lbg_print_rwstat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  lbg_prfill_rwstat_recursive, &blkcg_policy_lb,
			  seq_cft(sf)->private, true);
	return 0;
}

#ifdef CONFIG_DEBUG_BLK_CGROUP
static u64 lbg_prfill_avg_queue_size(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	struct lb_group *lbg = pd_to_lbg(pd);
	u64 samples = blkg_stat_read(&lbg->stats.avg_queue_size_samples);
	u64 v = 0;

	if (samples) {
		v = blkg_stat_read(&lbg->stats.avg_queue_size_sum);
		v = div64_u64(v, samples);
	}
	__blkg_prfill_u64(sf, pd, v);
	return 0;
}

/* print avg_queue_size */
static int lbg_print_avg_queue_size(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  lbg_prfill_avg_queue_size, &blkcg_policy_lb,
			  0, false);
	return 0;
}
#endif	/* CONFIG_DEBUG_BLK_CGROUP */

static struct cftype lb_blkcg_files[] = {
	/* on root, weight is mapped to leaf_weight */
	{
		.name = "weight_device",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = lbg_print_leaf_weight_device,
		.write = lbg_set_leaf_weight_device,
	},
	{
		.name = "weight",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = lb_print_leaf_weight,
		.write_u64 = lb_set_leaf_weight,
	},

	/* no such mapping necessary for !roots */
	{
		.name = "weight_device",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = lbg_print_weight_device,
		.write = lbg_set_weight_device,
	},
	{
		.name = "weight",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = lb_print_weight,
		.write_u64 = lb_set_weight,
	},

	{
		.name = "leaf_weight_device",
		.seq_show = lbg_print_leaf_weight_device,
		.write = lbg_set_leaf_weight_device,
	},
	{
		.name = "leaf_weight",
		.seq_show = lb_print_leaf_weight,
		.write_u64 = lb_set_leaf_weight,
	},

	/* statistics, covers only the tasks in the lbg */
	{
		.name = "time",
		.private = offsetof(struct lb_group, stats.time),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "sectors",
		.private = offsetof(struct lb_group, stats.sectors),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "io_service_bytes",
		.private = offsetof(struct lb_group, stats.service_bytes),
		.seq_show = lbg_print_rwstat,
	},
	{
		.name = "io_serviced",
		.private = offsetof(struct lb_group, stats.serviced),
		.seq_show = lbg_print_rwstat,
	},
	{
		.name = "io_service_time",
		.private = offsetof(struct lb_group, stats.service_time),
		.seq_show = lbg_print_rwstat,
	},
	{
		.name = "io_wait_time",
		.private = offsetof(struct lb_group, stats.wait_time),
		.seq_show = lbg_print_rwstat,
	},
	{
		.name = "io_merged",
		.private = offsetof(struct lb_group, stats.merged),
		.seq_show = lbg_print_rwstat,
	},
	{
		.name = "io_queued",
		.private = offsetof(struct lb_group, stats.queued),
		.seq_show = lbg_print_rwstat,
	},

	/* the same statictics which cover the lbg and its descendants */
	{
		.name = "time_recursive",
		.private = offsetof(struct lb_group, stats.time),
		.seq_show = lbg_print_stat_recursive,
	},
	{
		.name = "sectors_recursive",
		.private = offsetof(struct lb_group, stats.sectors),
		.seq_show = lbg_print_stat_recursive,
	},
	{
		.name = "io_service_bytes_recursive",
		.private = offsetof(struct lb_group, stats.service_bytes),
		.seq_show = lbg_print_rwstat_recursive,
	},
	{
		.name = "io_serviced_recursive",
		.private = offsetof(struct lb_group, stats.serviced),
		.seq_show = lbg_print_rwstat_recursive,
	},
	{
		.name = "io_service_time_recursive",
		.private = offsetof(struct lb_group, stats.service_time),
		.seq_show = lbg_print_rwstat_recursive,
	},
	{
		.name = "io_wait_time_recursive",
		.private = offsetof(struct lb_group, stats.wait_time),
		.seq_show = lbg_print_rwstat_recursive,
	},
	{
		.name = "io_merged_recursive",
		.private = offsetof(struct lb_group, stats.merged),
		.seq_show = lbg_print_rwstat_recursive,
	},
	{
		.name = "io_queued_recursive",
		.private = offsetof(struct lb_group, stats.queued),
		.seq_show = lbg_print_rwstat_recursive,
	},
#ifdef CONFIG_DEBUG_BLK_CGROUP
	{
		.name = "avg_queue_size",
		.seq_show = lbg_print_avg_queue_size,
	},
	{
		.name = "group_wait_time",
		.private = offsetof(struct lb_group, stats.group_wait_time),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "idle_time",
		.private = offsetof(struct lb_group, stats.idle_time),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "empty_time",
		.private = offsetof(struct lb_group, stats.empty_time),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "dequeue",
		.private = offsetof(struct lb_group, stats.dequeue),
		.seq_show = lbg_print_stat,
	},
	{
		.name = "unaccounted_time",
		.private = offsetof(struct lb_group, stats.unaccounted_time),
		.seq_show = lbg_print_stat,
	},
#endif	/* CONFIG_DEBUG_BLK_CGROUP */
	{ }	/* terminate */
};
#else /* GROUP_IOSCHED */
static struct lb_group *lb_lookup_create_lbg(struct lb_data *lbd,
						struct blkcg *blkcg)
{
	return lbd->root_group;
}

static inline void
lb_link_lbq_lbg(struct lb_queue *lbq, struct lb_group *lbg) {
	lbq->lbg = lbg;
}

#endif /* GROUP_IOSCHED */

/*
 * The lbd->service_trees holds all pending lb_queue's that have
 * requests waiting to be processed. It is sorted in the order that
 * we will service the queues.
 */
static void lb_service_tree_add(struct lb_data *lbd, struct lb_queue *lbq,
				 bool add_front)
{
	struct rb_node **p, *parent;
	struct lb_queue *__lbq;
	unsigned long rb_key;
	struct lb_rb_root *st;
	int left;
	int new_lbq = 1;

	st = st_for(lbq->lbg, lbq_class(lbq), lbq_type(lbq));
	if (lb_class_idle(lbq)) {
		rb_key = LB_IDLE_DELAY;
		parent = rb_last(&st->rb);
		if (parent && parent != &lbq->rb_node) {
			__lbq = rb_entry(parent, struct lb_queue, rb_node);
			rb_key += __lbq->rb_key;
		} else
			rb_key += jiffies;
	} else if (!add_front) {
		/*
		 * Get our rb key offset. Subtract any residual slice
		 * value carried from last service. A negative resid
		 * count indicates slice overrun, and this should position
		 * the next service time further away in the tree.
		 */
		rb_key = lb_slice_offset(lbd, lbq) + jiffies;
		rb_key -= lbq->slice_resid;
		lbq->slice_resid = 0;
	} else {
		rb_key = -HZ;
		__lbq = lb_rb_first(st);
		rb_key += __lbq ? __lbq->rb_key : jiffies;
	}

	if (!RB_EMPTY_NODE(&lbq->rb_node)) {
		new_lbq = 0;
		/*
		 * same position, nothing more to do
		 */
		if (rb_key == lbq->rb_key && lbq->service_tree == st)
			return;

		lb_rb_erase(&lbq->rb_node, lbq->service_tree);
		lbq->service_tree = NULL;
	}

	left = 1;
	parent = NULL;
	lbq->service_tree = st;
	p = &st->rb.rb_node;
	while (*p) {
		parent = *p;
		__lbq = rb_entry(parent, struct lb_queue, rb_node);

		/*
		 * sort by key, that represents service time.
		 */
		if (time_before(rb_key, __lbq->rb_key))
			p = &parent->rb_left;
		else {
			p = &parent->rb_right;
			left = 0;
		}
	}

	if (left)
		st->left = &lbq->rb_node;

	lbq->rb_key = rb_key;
	rb_link_node(&lbq->rb_node, parent, p);
	rb_insert_color(&lbq->rb_node, &st->rb);
	st->count++;
	if (add_front || !new_lbq)
		return;
	lb_group_notify_queue_add(lbd, lbq->lbg);
}

static struct lb_queue *
lb_prio_tree_lookup(struct lb_data *lbd, struct rb_root *root,
		     sector_t sector, struct rb_node **ret_parent,
		     struct rb_node ***rb_link)
{
	struct rb_node **p, *parent;
	struct lb_queue *lbq = NULL;

	parent = NULL;
	p = &root->rb_node;
	while (*p) {
		struct rb_node **n;

		parent = *p;
		lbq = rb_entry(parent, struct lb_queue, p_node);

		/*
		 * Sort strictly based on sector.  Smallest to the left,
		 * largest to the right.
		 */
		if (sector > blk_rq_pos(lbq->next_rq))
			n = &(*p)->rb_right;
		else if (sector < blk_rq_pos(lbq->next_rq))
			n = &(*p)->rb_left;
		else
			break;
		p = n;
		lbq = NULL;
	}

	*ret_parent = parent;
	if (rb_link)
		*rb_link = p;
	return lbq;
}

static void lb_prio_tree_add(struct lb_data *lbd, struct lb_queue *lbq)
{
	struct rb_node **p, *parent;
	struct lb_queue *__lbq;

	if (lbq->p_root) {
		rb_erase(&lbq->p_node, lbq->p_root);
		lbq->p_root = NULL;
	}

	if (lb_class_idle(lbq))
		return;
	if (!lbq->next_rq)
		return;

	lbq->p_root = &lbd->prio_trees[lbq->org_ioprio];
	__lbq = lb_prio_tree_lookup(lbd, lbq->p_root,
				      blk_rq_pos(lbq->next_rq), &parent, &p);
	if (!__lbq) {
		rb_link_node(&lbq->p_node, parent, p);
		rb_insert_color(&lbq->p_node, lbq->p_root);
	} else
		lbq->p_root = NULL;
}

/*
 * Update lbq's position in the service tree.
 */
static void lb_resort_rr_list(struct lb_data *lbd, struct lb_queue *lbq)
{
	/*
	 * Resorting requires the lbq to be on the RR list already.
	 */
	if (lb_lbq_on_rr(lbq)) {
		lb_service_tree_add(lbd, lbq, 0);
		lb_prio_tree_add(lbd, lbq);
	}
}

/*
 * add to busy list of queues for service, trying to be fair in ordering
 * the pending list according to last request service
 */
static void lb_add_lbq_rr(struct lb_data *lbd, struct lb_queue *lbq)
{
	lb_log_lbq(lbd, lbq, "add_to_rr");
	BUG_ON(lb_lbq_on_rr(lbq));
	lb_mark_lbq_on_rr(lbq);
	lbd->busy_queues++;
	if (lb_lbq_sync(lbq))
		lbd->busy_sync_queues++;

	lb_resort_rr_list(lbd, lbq);
}

/*
 * Called when the lbq no longer has requests pending, remove it from
 * the service tree.
 */
static void lb_del_lbq_rr(struct lb_data *lbd, struct lb_queue *lbq)
{
	lb_log_lbq(lbd, lbq, "del_from_rr");
	BUG_ON(!lb_lbq_on_rr(lbq));
	lb_clear_lbq_on_rr(lbq);

	if (!RB_EMPTY_NODE(&lbq->rb_node)) {
		lb_rb_erase(&lbq->rb_node, lbq->service_tree);
		lbq->service_tree = NULL;
	}
	if (lbq->p_root) {
		rb_erase(&lbq->p_node, lbq->p_root);
		lbq->p_root = NULL;
	}

	lb_group_notify_queue_del(lbd, lbq->lbg);
	BUG_ON(!lbd->busy_queues);
	lbd->busy_queues--;
	if (lb_lbq_sync(lbq))
		lbd->busy_sync_queues--;
}

/*
 * rb tree support functions
 */
static void lb_del_rq_rb(struct request *rq)
{
	struct lb_queue *lbq = RQ_LBQ(rq);
	const int sync = rq_is_sync(rq);

	BUG_ON(!lbq->queued[sync]);
	lbq->queued[sync]--;

	elv_rb_del(&lbq->sort_list, rq);

	if (lb_lbq_on_rr(lbq) && RB_EMPTY_ROOT(&lbq->sort_list)) {
		/*
		 * Queue will be deleted from service tree when we actually
		 * expire it later. Right now just remove it from prio tree
		 * as it is empty.
		 */
		if (lbq->p_root) {
			rb_erase(&lbq->p_node, lbq->p_root);
			lbq->p_root = NULL;
		}
	}
}

static void lb_add_rq_rb(struct request *rq)
{
	struct lb_queue *lbq = RQ_LBQ(rq);
	struct lb_data *lbd = lbq->lbd;
	struct request *prev;

	lbq->queued[rq_is_sync(rq)]++;

	elv_rb_add(&lbq->sort_list, rq);

	if (!lb_lbq_on_rr(lbq))
		lb_add_lbq_rr(lbd, lbq);

	/*
	 * check if this request is a better next-serve candidate
	 */
	prev = lbq->next_rq;
	lbq->next_rq = lb_choose_req(lbd, lbq->next_rq, rq, lbd->last_position);

	/*
	 * adjust priority tree position, if ->next_rq changes
	 */
	if (prev != lbq->next_rq)
		lb_prio_tree_add(lbd, lbq);

	BUG_ON(!lbq->next_rq);
}

static void lb_reposition_rq_rb(struct lb_queue *lbq, struct request *rq)
{
	elv_rb_del(&lbq->sort_list, rq);
	lbq->queued[rq_is_sync(rq)]--;
	lbg_stats_update_io_remove(RQ_LBG(rq), rq->cmd_flags);
	lb_add_rq_rb(rq);
	lbg_stats_update_io_add(RQ_LBG(rq), lbq->lbd->serving_group,
				 rq->cmd_flags);
}

static struct request *
lb_find_rq_fmerge(struct lb_data *lbd, struct bio *bio)
{
	struct task_struct *tsk = current;
	struct lb_io_cq *cic;
	struct lb_queue *lbq;

	cic = lb_cic_lookup(lbd, tsk->io_context);
	if (!cic)
		return NULL;

	lbq = cic_to_lbq(cic, lb_bio_sync(bio));
	if (lbq)
		return elv_rb_find(&lbq->sort_list, bio_end_sector(bio));

	return NULL;
}

static void lb_activate_request(struct request_queue *q, struct request *rq)
{
	struct lb_data *lbd = q->elevator->elevator_data;

	lbd->rq_in_driver++;
	lb_log_lbq(lbd, RQ_LBQ(rq), "activate rq, drv=%d",
						lbd->rq_in_driver);

	lbd->last_position = blk_rq_pos(rq) + blk_rq_sectors(rq);
}

static void lb_deactivate_request(struct request_queue *q, struct request *rq)
{
	struct lb_data *lbd = q->elevator->elevator_data;

	WARN_ON(!lbd->rq_in_driver);
	lbd->rq_in_driver--;
	lb_log_lbq(lbd, RQ_LBQ(rq), "deactivate rq, drv=%d",
						lbd->rq_in_driver);
}

static void lb_remove_request(struct request *rq)
{
	struct lb_queue *lbq = RQ_LBQ(rq);

	if (lbq->next_rq == rq)
		lbq->next_rq = lb_find_next_rq(lbq->lbd, lbq, rq);

	list_del_init(&rq->queuelist);
	lb_del_rq_rb(rq);

	lbq->lbd->rq_queued--;
	lbg_stats_update_io_remove(RQ_LBG(rq), rq->cmd_flags);
	if (rq->cmd_flags & REQ_PRIO) {
		WARN_ON(!lbq->prio_pending);
		lbq->prio_pending--;
	}
}

static int lb_merge(struct request_queue *q, struct request **req,
		     struct bio *bio)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct request *__rq;

	__rq = lb_find_rq_fmerge(lbd, bio);
	if (__rq && elv_rq_merge_ok(__rq, bio)) {
		*req = __rq;
		return ELEVATOR_FRONT_MERGE;
	}

	return ELEVATOR_NO_MERGE;
}

static void lb_merged_request(struct request_queue *q, struct request *req,
			       int type)
{
	if (type == ELEVATOR_FRONT_MERGE) {
		struct lb_queue *lbq = RQ_LBQ(req);

		lb_reposition_rq_rb(lbq, req);
	}
}

static void lb_bio_merged(struct request_queue *q, struct request *req,
				struct bio *bio)
{
	lbg_stats_update_io_merged(RQ_LBG(req), bio->bi_rw);
}

static void
lb_merged_requests(struct request_queue *q, struct request *rq,
		    struct request *next)
{
	struct lb_queue *lbq = RQ_LBQ(rq);
	struct lb_data *lbd = q->elevator->elevator_data;

	/*
	 * reposition in fifo if next is older than rq
	 */
	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist) &&
	    time_before(next->fifo_time, rq->fifo_time) &&
	    lbq == RQ_LBQ(next)) {
		list_move(&rq->queuelist, &next->queuelist);
		rq->fifo_time = next->fifo_time;
	}

	if (lbq->next_rq == next)
		lbq->next_rq = rq;
	lb_remove_request(next);
	lbg_stats_update_io_merged(RQ_LBG(rq), next->cmd_flags);

	lbq = RQ_LBQ(next);
	/*
	 * all requests of this queue are merged to other queues, delete it
	 * from the service tree. If it's the active_queue,
	 * lb_dispatch_requests() will choose to expire it or do idle
	 */
	if (lb_lbq_on_rr(lbq) && RB_EMPTY_ROOT(&lbq->sort_list) &&
	    lbq != lbd->active_queue)
		lb_del_lbq_rr(lbd, lbq);
}

static int lb_allow_merge(struct request_queue *q, struct request *rq,
			   struct bio *bio)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct lb_io_cq *cic;
	struct lb_queue *lbq;

	/*
	 * Disallow merge of a sync bio into an async request.
	 */
	if (lb_bio_sync(bio) && !rq_is_sync(rq))
		return false;

	/*
	 * Lookup the lbq that this bio will be queued with and allow
	 * merge only if rq is queued there.
	 */
	cic = lb_cic_lookup(lbd, current->io_context);
	if (!cic)
		return false;

	lbq = cic_to_lbq(cic, lb_bio_sync(bio));
	return lbq == RQ_LBQ(rq);
}

static inline void lb_del_timer(struct lb_data *lbd, struct lb_queue *lbq)
{
	del_timer(&lbd->idle_slice_timer);
	lbg_stats_update_idle_time(lbq->lbg);
}

static void __lb_set_active_queue(struct lb_data *lbd,
				   struct lb_queue *lbq)
{
	if (lbq) {
		lb_log_lbq(lbd, lbq, "set_active wl_class:%d wl_type:%d",
				lbd->serving_wl_class, lbd->serving_wl_type);
		lbg_stats_update_avg_queue_size(lbq->lbg);
		lbq->slice_start = 0;
		lbq->dispatch_start = jiffies;
		lbq->allocated_slice = 0;
		lbq->slice_end = 0;
		lbq->slice_dispatch = 0;
		lbq->nr_sectors = 0;

		lb_clear_lbq_wait_request(lbq);
		lb_clear_lbq_must_dispatch(lbq);
		lb_clear_lbq_must_alloc_slice(lbq);
		lb_clear_lbq_fifo_expire(lbq);
		lb_mark_lbq_slice_new(lbq);

		lb_del_timer(lbd, lbq);
	}

	lbd->active_queue = lbq;
}

/*
 * current lbq expired its slice (or was too idle), select new one
 */
static void
__lb_slice_expired(struct lb_data *lbd, struct lb_queue *lbq,
		    bool timed_out)
{
	lb_log_lbq(lbd, lbq, "slice expired t=%d", timed_out);

	if (lb_lbq_wait_request(lbq))
		lb_del_timer(lbd, lbq);

	lb_clear_lbq_wait_request(lbq);
	lb_clear_lbq_wait_busy(lbq);

	/*
	 * If this lbq is shared between multiple processes, check to
	 * make sure that those processes are still issuing I/Os within
	 * the mean seek distance.  If not, it may be time to break the
	 * queues apart again.
	 */
	if (lb_lbq_coop(lbq) && LBQ_SEEKY(lbq))
		lb_mark_lbq_split_coop(lbq);

	/*
	 * store what was left of this slice, if the queue idled/timed out
	 */
	if (timed_out) {
		if (lb_lbq_slice_new(lbq))
			lbq->slice_resid = lb_scaled_lbq_slice(lbd, lbq);
		else
			lbq->slice_resid = lbq->slice_end - jiffies;
		lb_log_lbq(lbd, lbq, "resid=%ld", lbq->slice_resid);
	}

	lb_group_served(lbd, lbq->lbg, lbq);

	if (lb_lbq_on_rr(lbq) && RB_EMPTY_ROOT(&lbq->sort_list))
		lb_del_lbq_rr(lbd, lbq);

	lb_resort_rr_list(lbd, lbq);

	if (lbq == lbd->active_queue)
		lbd->active_queue = NULL;

	if (lbd->active_cic) {
		put_io_context(lbd->active_cic->icq.ioc);
		lbd->active_cic = NULL;
	}
}

static inline void lb_slice_expired(struct lb_data *lbd, bool timed_out)
{
	struct lb_queue *lbq = lbd->active_queue;

	if (lbq)
		__lb_slice_expired(lbd, lbq, timed_out);
}

/*
 * Get next queue for service. Unless we have a queue preemption,
 * we'll simply select the first lbq in the service tree.
 */
static struct lb_queue *lb_get_next_queue(struct lb_data *lbd)
{
	struct lb_rb_root *st = st_for(lbd->serving_group,
			lbd->serving_wl_class, lbd->serving_wl_type);

	if (!lbd->rq_queued)
		return NULL;

	/* There is nothing to dispatch */
	if (!st)
		return NULL;
	if (RB_EMPTY_ROOT(&st->rb))
		return NULL;
	return lb_rb_first(st);
}

static struct lb_queue *lb_get_next_queue_forced(struct lb_data *lbd)
{
	struct lb_group *lbg;
	struct lb_queue *lbq;
	int i, j;
	struct lb_rb_root *st;

	if (!lbd->rq_queued)
		return NULL;

	lbg = lb_get_next_lbg(lbd);
	if (!lbg)
		return NULL;

	for_each_lbg_st(lbg, i, j, st)
		if ((lbq = lb_rb_first(st)) != NULL)
			return lbq;
	return NULL;
}

/*
 * Get and set a new active queue for service.
 */
static struct lb_queue *lb_set_active_queue(struct lb_data *lbd,
					      struct lb_queue *lbq)
{
	if (!lbq)
		lbq = lb_get_next_queue(lbd);

	__lb_set_active_queue(lbd, lbq);
	return lbq;
}

static inline sector_t lb_dist_from_last(struct lb_data *lbd,
					  struct request *rq)
{
	if (blk_rq_pos(rq) >= lbd->last_position)
		return blk_rq_pos(rq) - lbd->last_position;
	else
		return lbd->last_position - blk_rq_pos(rq);
}

static inline int lb_rq_close(struct lb_data *lbd, struct lb_queue *lbq,
			       struct request *rq)
{
	return lb_dist_from_last(lbd, rq) <= LBQ_CLOSE_THR;
}

static struct lb_queue *lbq_close(struct lb_data *lbd,
				    struct lb_queue *cur_lbq)
{
	struct rb_root *root = &lbd->prio_trees[cur_lbq->org_ioprio];
	struct rb_node *parent, *node;
	struct lb_queue *__lbq;
	sector_t sector = lbd->last_position;

	if (RB_EMPTY_ROOT(root))
		return NULL;

	/*
	 * First, if we find a request starting at the end of the last
	 * request, choose it.
	 */
	__lbq = lb_prio_tree_lookup(lbd, root, sector, &parent, NULL);
	if (__lbq)
		return __lbq;

	/*
	 * If the exact sector wasn't found, the parent of the NULL leaf
	 * will contain the closest sector.
	 */
	__lbq = rb_entry(parent, struct lb_queue, p_node);
	if (lb_rq_close(lbd, cur_lbq, __lbq->next_rq))
		return __lbq;

	if (blk_rq_pos(__lbq->next_rq) < sector)
		node = rb_next(&__lbq->p_node);
	else
		node = rb_prev(&__lbq->p_node);
	if (!node)
		return NULL;

	__lbq = rb_entry(node, struct lb_queue, p_node);
	if (lb_rq_close(lbd, cur_lbq, __lbq->next_rq))
		return __lbq;

	return NULL;
}

/*
 * lbd - obvious
 * cur_lbq - passed in so that we don't decide that the current queue is
 * 	      closely cooperating with itself.
 *
 * So, basically we're assuming that that cur_lbq has dispatched at least
 * one request, and that lbd->last_position reflects a position on the disk
 * associated with the I/O issued by cur_lbq.  I'm not sure this is a valid
 * assumption.
 */
static struct lb_queue *lb_close_cooperator(struct lb_data *lbd,
					      struct lb_queue *cur_lbq)
{
	struct lb_queue *lbq;

	if (lb_class_idle(cur_lbq))
		return NULL;
	if (!lb_lbq_sync(cur_lbq))
		return NULL;
	if (LBQ_SEEKY(cur_lbq))
		return NULL;

	/*
	 * Don't search priority tree if it's the only queue in the group.
	 */
	if (cur_lbq->lbg->nr_lbq == 1)
		return NULL;

	/*
	 * We should notice if some of the queues are cooperating, eg
	 * working closely on the same area of the disk. In that case,
	 * we can group them together and don't waste time idling.
	 */
	lbq = lbq_close(lbd, cur_lbq);
	if (!lbq)
		return NULL;

	/* If new queue belongs to different lb_group, don't choose it */
	if (cur_lbq->lbg != lbq->lbg)
		return NULL;

	/*
	 * It only makes sense to merge sync queues.
	 */
	if (!lb_lbq_sync(lbq))
		return NULL;
	if (LBQ_SEEKY(lbq))
		return NULL;

	/*
	 * Do not merge queues of different priority classes
	 */
	if (lb_class_rt(lbq) != lb_class_rt(cur_lbq))
		return NULL;

	return lbq;
}

/*
 * Determine whether we should enforce idle window for this queue.
 */

static bool lb_should_idle(struct lb_data *lbd, struct lb_queue *lbq)
{
	enum wl_class_t wl_class = lbq_class(lbq);
	struct lb_rb_root *st = lbq->service_tree;

	BUG_ON(!st);
	BUG_ON(!st->count);

	if (!lbd->lb_slice_idle)
		return false;

	/* We never do for idle class queues. */
	if (wl_class == IDLE_WORKLOAD)
		return false;

	/* We do for queues that were marked with idle window flag. */
	if (lb_lbq_idle_window(lbq) &&
	   !(blk_queue_nonrot(lbd->queue) && lbd->hw_tag))
		return true;

	/*
	 * Otherwise, we do only if they are the last ones
	 * in their service tree.
	 */
	if (st->count == 1 && lb_lbq_sync(lbq) &&
	   !lb_io_thinktime_big(lbd, &st->ttime, false))
		return true;
	lb_log_lbq(lbd, lbq, "Not idling. st->count:%d", st->count);
	return false;
}

static void lb_arm_slice_timer(struct lb_data *lbd)
{
	struct lb_queue *lbq = lbd->active_queue;
	struct lb_io_cq *cic;
	unsigned long sl, group_idle = 0;

	/*
	 * SSD device without seek penalty, disable idling. But only do so
	 * for devices that support queuing, otherwise we still have a problem
	 * with sync vs async workloads.
	 */
	if (blk_queue_nonrot(lbd->queue) && lbd->hw_tag)
		return;

	WARN_ON(!RB_EMPTY_ROOT(&lbq->sort_list));
	WARN_ON(lb_lbq_slice_new(lbq));

	/*
	 * idle is disabled, either manually or by past process history
	 */
	if (!lb_should_idle(lbd, lbq)) {
		/* no queue idling. Check for group idling */
		if (lbd->lb_group_idle)
			group_idle = lbd->lb_group_idle;
		else
			return;
	}

	/*
	 * still active requests from this queue, don't idle
	 */
	if (lbq->dispatched)
		return;

	/*
	 * task has exited, don't wait
	 */
	cic = lbd->active_cic;
	if (!cic || !atomic_read(&cic->icq.ioc->active_ref))
		return;

	/*
	 * If our average think time is larger than the remaining time
	 * slice, then don't idle. This avoids overrunning the allotted
	 * time slice.
	 */
	if (sample_valid(cic->ttime.ttime_samples) &&
	    (lbq->slice_end - jiffies < cic->ttime.ttime_mean)) {
		lb_log_lbq(lbd, lbq, "Not idling. think_time:%lu",
			     cic->ttime.ttime_mean);
		return;
	}

	/* There are other queues in the group, don't do group idle */
	if (group_idle && lbq->lbg->nr_lbq > 1)
		return;

	lb_mark_lbq_wait_request(lbq);

	if (group_idle)
		sl = lbd->lb_group_idle;
	else
		sl = lbd->lb_slice_idle;

	mod_timer(&lbd->idle_slice_timer, jiffies + sl);
	lbg_stats_set_start_idle_time(lbq->lbg);
	lb_log_lbq(lbd, lbq, "arm_idle: %lu group_idle: %d", sl,
			group_idle ? 1 : 0);
}

/*
 * Move request from internal lists to the request queue dispatch list.
 */
static void lb_dispatch_insert(struct request_queue *q, struct request *rq)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct lb_queue *lbq = RQ_LBQ(rq);

	lb_log_lbq(lbd, lbq, "dispatch_insert");

	lbq->next_rq = lb_find_next_rq(lbd, lbq, rq);
	lb_remove_request(rq);
	lbq->dispatched++;
	(RQ_LBG(rq))->dispatched++;
	elv_dispatch_sort(q, rq);

	lbd->rq_in_flight[lb_lbq_sync(lbq)]++;
	lbq->nr_sectors += blk_rq_sectors(rq);
	lbg_stats_update_dispatch(lbq->lbg, blk_rq_bytes(rq), rq->cmd_flags);
}

/*
 * return expired entry, or NULL to just start from scratch in rbtree
 */
static struct request *lb_check_fifo(struct lb_queue *lbq)
{
	struct request *rq = NULL;

	if (lb_lbq_fifo_expire(lbq))
		return NULL;

	lb_mark_lbq_fifo_expire(lbq);

	if (list_empty(&lbq->fifo))
		return NULL;

	rq = rq_entry_fifo(lbq->fifo.next);
	if (time_before(jiffies, rq->fifo_time))
		rq = NULL;

	lb_log_lbq(lbq->lbd, lbq, "fifo=%p", rq);
	return rq;
}

static inline int
lb_prio_to_maxrq(struct lb_data *lbd, struct lb_queue *lbq)
{
	const int base_rq = lbd->lb_slice_async_rq;

	WARN_ON(lbq->ioprio >= IOPRIO_BE_NR);

	return 2 * base_rq * (IOPRIO_BE_NR - lbq->ioprio);
}

/*
 * Must be called with the queue_lock held.
 */
static int lbq_process_refs(struct lb_queue *lbq)
{
	int process_refs, io_refs;

	io_refs = lbq->allocated[READ] + lbq->allocated[WRITE];
	process_refs = lbq->ref - io_refs;
	BUG_ON(process_refs < 0);
	return process_refs;
}

static void lb_setup_merge(struct lb_queue *lbq, struct lb_queue *new_lbq)
{
	int process_refs, new_process_refs;
	struct lb_queue *__lbq;

	/*
	 * If there are no process references on the new_lbq, then it is
	 * unsafe to follow the ->new_lbq chain as other lbq's in the
	 * chain may have dropped their last reference (not just their
	 * last process reference).
	 */
	if (!lbq_process_refs(new_lbq))
		return;

	/* Avoid a circular list and skip interim queue merges */
	while ((__lbq = new_lbq->new_lbq)) {
		if (__lbq == lbq)
			return;
		new_lbq = __lbq;
	}

	process_refs = lbq_process_refs(lbq);
	new_process_refs = lbq_process_refs(new_lbq);
	/*
	 * If the process for the lbq has gone away, there is no
	 * sense in merging the queues.
	 */
	if (process_refs == 0 || new_process_refs == 0)
		return;

	/*
	 * Merge in the direction of the lesser amount of work.
	 */
	if (new_process_refs >= process_refs) {
		lbq->new_lbq = new_lbq;
		new_lbq->ref += process_refs;
	} else {
		new_lbq->new_lbq = lbq;
		lbq->ref += new_process_refs;
	}
}

static enum wl_type_t lb_choose_wl_type(struct lb_data *lbd,
			struct lb_group *lbg, enum wl_class_t wl_class)
{
	struct lb_queue *queue;
	int i;
	bool key_valid = false;
	unsigned long lowest_key = 0;
	enum wl_type_t cur_best = SYNC_NOIDLE_WORKLOAD;

	for (i = 0; i <= SYNC_WORKLOAD; ++i) {
		/* select the one with lowest rb_key */
		queue = lb_rb_first(st_for(lbg, wl_class, i));
		if (queue &&
		    (!key_valid || time_before(queue->rb_key, lowest_key))) {
			lowest_key = queue->rb_key;
			cur_best = i;
			key_valid = true;
		}
	}

	return cur_best;
}

static void
choose_wl_class_and_type(struct lb_data *lbd, struct lb_group *lbg)
{
	unsigned slice;
	unsigned count;
	struct lb_rb_root *st;
	unsigned group_slice;
	enum wl_class_t original_class = lbd->serving_wl_class;

	/* Choose next priority. RT > BE > IDLE */
	if (lb_group_busy_queues_wl(RT_WORKLOAD, lbd, lbg))
		lbd->serving_wl_class = RT_WORKLOAD;
	else if (lb_group_busy_queues_wl(BE_WORKLOAD, lbd, lbg))
		lbd->serving_wl_class = BE_WORKLOAD;
	else {
		lbd->serving_wl_class = IDLE_WORKLOAD;
		lbd->workload_expires = jiffies + 1;
		return;
	}

	if (original_class != lbd->serving_wl_class)
		goto new_workload;

	/*
	 * For RT and BE, we have to choose also the type
	 * (SYNC, SYNC_NOIDLE, ASYNC), and to compute a workload
	 * expiration time
	 */
	st = st_for(lbg, lbd->serving_wl_class, lbd->serving_wl_type);
	count = st->count;

	/*
	 * check workload expiration, and that we still have other queues ready
	 */
	if (count && !time_after(jiffies, lbd->workload_expires))
		return;

new_workload:
	/* otherwise select new workload type */
	lbd->serving_wl_type = lb_choose_wl_type(lbd, lbg,
					lbd->serving_wl_class);
	st = st_for(lbg, lbd->serving_wl_class, lbd->serving_wl_type);
	count = st->count;

	/*
	 * the workload slice is computed as a fraction of target latency
	 * proportional to the number of queues in that workload, over
	 * all the queues in the same priority class
	 */
	group_slice = lb_group_slice(lbd, lbg);

	slice = group_slice * count /
		max_t(unsigned, lbg->busy_queues_avg[lbd->serving_wl_class],
		      lb_group_busy_queues_wl(lbd->serving_wl_class, lbd,
					lbg));

	if (lbd->serving_wl_type == ASYNC_WORKLOAD) {
		unsigned int tmp;

		/*
		 * Async queues are currently system wide. Just taking
		 * proportion of queues with-in same group will lead to higher
		 * async ratio system wide as generally root group is going
		 * to have higher weight. A more accurate thing would be to
		 * calculate system wide asnc/sync ratio.
		 */
		tmp = lbd->lb_target_latency *
			lbg_busy_async_queues(lbd, lbg);
		tmp = tmp/lbd->busy_queues;
		slice = min_t(unsigned, slice, tmp);

		/* async workload slice is scaled down according to
		 * the sync/async slice ratio. */
		slice = slice * lbd->lb_slice[0] / lbd->lb_slice[1];
	} else
		/* sync workload slice is at least 2 * lb_slice_idle */
		slice = max(slice, 2 * lbd->lb_slice_idle);

	slice = max_t(unsigned, slice, LB_MIN_TT);
	lb_log(lbd, "workload slice:%d", slice);
	lbd->workload_expires = jiffies + slice;
}

static struct lb_group *lb_get_next_lbg(struct lb_data *lbd)
{
	struct lb_rb_root *st = &lbd->grp_service_tree;
	struct lb_group *lbg;

	if (RB_EMPTY_ROOT(&st->rb))
		return NULL;
	lbg = lb_rb_first_group(st);
	update_min_vdisktime(st);
	return lbg;
}

static void lb_choose_lbg(struct lb_data *lbd)
{
	struct lb_group *lbg = lb_get_next_lbg(lbd);

	lbd->serving_group = lbg;

	/* Restore the workload type data */
	if (lbg->saved_wl_slice) {
		lbd->workload_expires = jiffies + lbg->saved_wl_slice;
		lbd->serving_wl_type = lbg->saved_wl_type;
		lbd->serving_wl_class = lbg->saved_wl_class;
	} else
		lbd->workload_expires = jiffies - 1;

	choose_wl_class_and_type(lbd, lbg);
}

/*
 * Select a queue for service. If we have a current active queue,
 * check whether to continue servicing it, or retrieve and set a new one.
 */
static struct lb_queue *lb_select_queue(struct lb_data *lbd)
{
	struct lb_queue *lbq, *new_lbq = NULL;

	lbq = lbd->active_queue;
	if (!lbq)
		goto new_queue;

	if (!lbd->rq_queued)
		return NULL;

	/*
	 * We were waiting for group to get backlogged. Expire the queue
	 */
	if (lb_lbq_wait_busy(lbq) && !RB_EMPTY_ROOT(&lbq->sort_list))
		goto expire;

	/*
	 * The active queue has run out of time, expire it and select new.
	 */
	if (lb_slice_used(lbq) && !lb_lbq_must_dispatch(lbq)) {
		/*
		 * If slice had not expired at the completion of last request
		 * we might not have turned on wait_busy flag. Don't expire
		 * the queue yet. Allow the group to get backlogged.
		 *
		 * The very fact that we have used the slice, that means we
		 * have been idling all along on this queue and it should be
		 * ok to wait for this request to complete.
		 */
		if (lbq->lbg->nr_lbq == 1 && RB_EMPTY_ROOT(&lbq->sort_list)
		    && lbq->dispatched && lb_should_idle(lbd, lbq)) {
			lbq = NULL;
			goto keep_queue;
		} else
			goto check_group_idle;
	}

	/*
	 * The active queue has requests and isn't expired, allow it to
	 * dispatch.
	 */
	if (!RB_EMPTY_ROOT(&lbq->sort_list))
		goto keep_queue;

	/*
	 * If another queue has a request waiting within our mean seek
	 * distance, let it run.  The expire code will check for close
	 * cooperators and put the close queue at the front of the service
	 * tree.  If possible, merge the expiring queue with the new lbq.
	 */
	new_lbq = lb_close_cooperator(lbd, lbq);
	if (new_lbq) {
		if (!lbq->new_lbq)
			lb_setup_merge(lbq, new_lbq);
		goto expire;
	}

	/*
	 * No requests pending. If the active queue still has requests in
	 * flight or is idling for a new request, allow either of these
	 * conditions to happen (or time out) before selecting a new queue.
	 */
	if (timer_pending(&lbd->idle_slice_timer)) {
		lbq = NULL;
		goto keep_queue;
	}

	/*
	 * This is a deep seek queue, but the device is much faster than
	 * the queue can deliver, don't idle
	 **/
	if (LBQ_SEEKY(lbq) && lb_lbq_idle_window(lbq) &&
	    (lb_lbq_slice_new(lbq) ||
	    (lbq->slice_end - jiffies > jiffies - lbq->slice_start))) {
		lb_clear_lbq_deep(lbq);
		lb_clear_lbq_idle_window(lbq);
	}

	if (lbq->dispatched && lb_should_idle(lbd, lbq)) {
		lbq = NULL;
		goto keep_queue;
	}

	/*
	 * If group idle is enabled and there are requests dispatched from
	 * this group, wait for requests to complete.
	 */
check_group_idle:
	if (lbd->lb_group_idle && lbq->lbg->nr_lbq == 1 &&
	    lbq->lbg->dispatched &&
	    !lb_io_thinktime_big(lbd, &lbq->lbg->ttime, true)) {
		lbq = NULL;
		goto keep_queue;
	}

expire:
	lb_slice_expired(lbd, 0);
new_queue:
	/*
	 * Current queue expired. Check if we have to switch to a new
	 * service tree
	 */
	if (!new_lbq)
		lb_choose_lbg(lbd);

	lbq = lb_set_active_queue(lbd, new_lbq);
keep_queue:
	return lbq;
}

static int __lb_forced_dispatch_lbq(struct lb_queue *lbq)
{
	int dispatched = 0;

	while (lbq->next_rq) {
		lb_dispatch_insert(lbq->lbd->queue, lbq->next_rq);
		dispatched++;
	}

	BUG_ON(!list_empty(&lbq->fifo));

	/* By default lbq is not expired if it is empty. Do it explicitly */
	__lb_slice_expired(lbq->lbd, lbq, 0);
	return dispatched;
}

/*
 * Drain our current requests. Used for barriers and when switching
 * io schedulers on-the-fly.
 */
static int lb_forced_dispatch(struct lb_data *lbd)
{
	struct lb_queue *lbq;
	int dispatched = 0;

	/* Expire the timeslice of the current active queue first */
	lb_slice_expired(lbd, 0);
	while ((lbq = lb_get_next_queue_forced(lbd)) != NULL) {
		__lb_set_active_queue(lbd, lbq);
		dispatched += __lb_forced_dispatch_lbq(lbq);
	}

	BUG_ON(lbd->busy_queues);

	lb_log(lbd, "forced_dispatch=%d", dispatched);
	return dispatched;
}

static inline bool lb_slice_used_soon(struct lb_data *lbd,
	struct lb_queue *lbq)
{
	/* the queue hasn't finished any request, can't estimate */
	if (lb_lbq_slice_new(lbq))
		return true;
	if (time_after(jiffies + lbd->lb_slice_idle * lbq->dispatched,
		lbq->slice_end))
		return true;

	return false;
}

static bool lb_may_dispatch(struct lb_data *lbd, struct lb_queue *lbq)
{
	unsigned int max_dispatch;

	/*
	 * Drain async requests before we start sync IO
	 */
	if (lb_should_idle(lbd, lbq) && lbd->rq_in_flight[BLK_RW_ASYNC])
		return false;

	/*
	 * If this is an async queue and we have sync IO in flight, let it wait
	 */
	if (lbd->rq_in_flight[BLK_RW_SYNC] && !lb_lbq_sync(lbq))
		return false;

	max_dispatch = max_t(unsigned int, lbd->lb_quantum / 2, 1);
	if (lb_class_idle(lbq))
		max_dispatch = 1;

	/*
	 * Does this lbq already have too much IO in flight?
	 */
	if (lbq->dispatched >= max_dispatch) {
		bool promote_sync = false;
		/*
		 * idle queue must always only have a single IO in flight
		 */
		if (lb_class_idle(lbq))
			return false;

		/*
		 * If there is only one sync queue
		 * we can ignore async queue here and give the sync
		 * queue no dispatch limit. The reason is a sync queue can
		 * preempt async queue, limiting the sync queue doesn't make
		 * sense. This is useful for aiostress test.
		 */
		if (lb_lbq_sync(lbq) && lbd->busy_sync_queues == 1)
			promote_sync = true;

		/*
		 * We have other queues, don't allow more IO from this one
		 */
		if (lbd->busy_queues > 1 && lb_slice_used_soon(lbd, lbq) &&
				!promote_sync)
			return false;

		/*
		 * Sole queue user, no limit
		 */
		if (lbd->busy_queues == 1 || promote_sync)
			max_dispatch = -1;
		else
			/*
			 * Normally we start throttling lbq when lb_quantum/2
			 * requests have been dispatched. But we can drive
			 * deeper queue depths at the beginning of slice
			 * subjected to upper limit of lb_quantum.
			 * */
			max_dispatch = lbd->lb_quantum;
	}

	/*
	 * Async queues must wait a bit before being allowed dispatch.
	 * We also ramp up the dispatch depth gradually for async IO,
	 * based on the last sync IO we serviced
	 */
	if (!lb_lbq_sync(lbq) && lbd->lb_latency) {
		unsigned long last_sync = jiffies - lbd->last_delayed_sync;
		unsigned int depth;

		depth = last_sync / lbd->lb_slice[1];
		if (!depth && !lbq->dispatched)
			depth = 1;
		if (depth < max_dispatch)
			max_dispatch = depth;
	}

	/*
	 * If we're below the current max, allow a dispatch
	 */
	return lbq->dispatched < max_dispatch;
}

/*
 * Dispatch a request from lbq, moving them to the request queue
 * dispatch list.
 */
static bool lb_dispatch_request(struct lb_data *lbd, struct lb_queue *lbq)
{
	struct request *rq;

	BUG_ON(RB_EMPTY_ROOT(&lbq->sort_list));

	if (!lb_may_dispatch(lbd, lbq))
		return false;

	/*
	 * follow expired path, else get first next available
	 */
	rq = lb_check_fifo(lbq);
	if (!rq)
		rq = lbq->next_rq;

	/*
	 * insert request into driver dispatch list
	 */
	lb_dispatch_insert(lbd->queue, rq);

	if (!lbd->active_cic) {
		struct lb_io_cq *cic = RQ_CIC(rq);

		atomic_long_inc(&cic->icq.ioc->refcount);
		lbd->active_cic = cic;
	}

	return true;
}

/*
 * Find the lbq that we need to service and move a request from that to the
 * dispatch list
 */
static int lb_dispatch_requests(struct request_queue *q, int force)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct lb_queue *lbq;

	if (!lbd->busy_queues)
		return 0;

	if (unlikely(force))
		return lb_forced_dispatch(lbd);

	lbq = lb_select_queue(lbd);
	if (!lbq)
		return 0;

	/*
	 * Dispatch a request from this lbq, if it is allowed
	 */
	if (!lb_dispatch_request(lbd, lbq))
		return 0;

	lbq->slice_dispatch++;
	lb_clear_lbq_must_dispatch(lbq);

	/*
	 * expire an async queue immediately if it has used up its slice. idle
	 * queue always expire after 1 dispatch round.
	 */
	if (lbd->busy_queues > 1 && ((!lb_lbq_sync(lbq) &&
	    lbq->slice_dispatch >= lb_prio_to_maxrq(lbd, lbq)) ||
	    lb_class_idle(lbq))) {
		lbq->slice_end = jiffies + 1;
		lb_slice_expired(lbd, 0);
	}

	lb_log_lbq(lbd, lbq, "dispatched a request");
	return 1;
}

/*
 * task holds one reference to the queue, dropped when task exits. each rq
 * in-flight on this queue also holds a reference, dropped when rq is freed.
 *
 * Each lb queue took a reference on the parent group. Drop it now.
 * queue lock must be held here.
 */
static void lb_put_queue(struct lb_queue *lbq)
{
	struct lb_data *lbd = lbq->lbd;
	struct lb_group *lbg;

	BUG_ON(lbq->ref <= 0);

	lbq->ref--;
	if (lbq->ref)
		return;

	lb_log_lbq(lbd, lbq, "put_queue");
	BUG_ON(rb_first(&lbq->sort_list));
	BUG_ON(lbq->allocated[READ] + lbq->allocated[WRITE]);
	lbg = lbq->lbg;

	if (unlikely(lbd->active_queue == lbq)) {
		__lb_slice_expired(lbd, lbq, 0);
		lb_schedule_dispatch(lbd);
	}

	BUG_ON(lb_lbq_on_rr(lbq));
	kmem_cache_free(lb_pool, lbq);
	lbg_put(lbg);
}

static void lb_put_cooperator(struct lb_queue *lbq)
{
	struct lb_queue *__lbq, *next;

	/*
	 * If this queue was scheduled to merge with another queue, be
	 * sure to drop the reference taken on that queue (and others in
	 * the merge chain).  See lb_setup_merge and lb_merge_lbqs.
	 */
	__lbq = lbq->new_lbq;
	while (__lbq) {
		if (__lbq == lbq) {
			WARN(1, "lbq->new_lbq loop detected\n");
			break;
		}
		next = __lbq->new_lbq;
		lb_put_queue(__lbq);
		__lbq = next;
	}
}

static void lb_exit_lbq(struct lb_data *lbd, struct lb_queue *lbq)
{
	if (unlikely(lbq == lbd->active_queue)) {
		__lb_slice_expired(lbd, lbq, 0);
		lb_schedule_dispatch(lbd);
	}

	lb_put_cooperator(lbq);

	lb_put_queue(lbq);
}

static void lb_init_icq(struct io_cq *icq)
{
	struct lb_io_cq *cic = icq_to_cic(icq);

	cic->ttime.last_end_request = jiffies;
}

static void lb_exit_icq(struct io_cq *icq)
{
	struct lb_io_cq *cic = icq_to_cic(icq);
	struct lb_data *lbd = cic_to_lbd(cic);

	if (cic->lbq[BLK_RW_ASYNC]) {
		lb_exit_lbq(lbd, cic->lbq[BLK_RW_ASYNC]);
		cic->lbq[BLK_RW_ASYNC] = NULL;
	}

	if (cic->lbq[BLK_RW_SYNC]) {
		lb_exit_lbq(lbd, cic->lbq[BLK_RW_SYNC]);
		cic->lbq[BLK_RW_SYNC] = NULL;
	}
}

static void lb_init_prio_data(struct lb_queue *lbq, struct lb_io_cq *cic)
{
	struct task_struct *tsk = current;
	int ioprio_class;

	if (!lb_lbq_prio_changed(lbq))
		return;

	ioprio_class = IOPRIO_PRIO_CLASS(cic->ioprio);
	switch (ioprio_class) {
	default:
		printk(KERN_ERR "lb: bad prio %x\n", ioprio_class);
	case IOPRIO_CLASS_NONE:
		/*
		 * no prio set, inherit CPU scheduling settings
		 */
		lbq->ioprio = task_nice_ioprio(tsk);
		lbq->ioprio_class = task_nice_ioclass(tsk);
		break;
	case IOPRIO_CLASS_RT:
		lbq->ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
		lbq->ioprio_class = IOPRIO_CLASS_RT;
		break;
	case IOPRIO_CLASS_BE:
		lbq->ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
		lbq->ioprio_class = IOPRIO_CLASS_BE;
		break;
	case IOPRIO_CLASS_IDLE:
		lbq->ioprio_class = IOPRIO_CLASS_IDLE;
		lbq->ioprio = 7;
		lb_clear_lbq_idle_window(lbq);
		break;
	}

	/*
	 * keep track of original prio settings in case we have to temporarily
	 * elevate the priority of this queue
	 */
	lbq->org_ioprio = lbq->ioprio;
	lb_clear_lbq_prio_changed(lbq);
}

static void check_ioprio_changed(struct lb_io_cq *cic, struct bio *bio)
{
	int ioprio = cic->icq.ioc->ioprio;
	struct lb_data *lbd = cic_to_lbd(cic);
	struct lb_queue *lbq;

	/*
	 * Check whether ioprio has changed.  The condition may trigger
	 * spuriously on a newly created cic but there's no harm.
	 */
	if (unlikely(!lbd) || likely(cic->ioprio == ioprio))
		return;

	lbq = cic->lbq[BLK_RW_ASYNC];
	if (lbq) {
		struct lb_queue *new_lbq;
		new_lbq = lb_get_queue(lbd, BLK_RW_ASYNC, cic, bio,
					 GFP_ATOMIC);
		if (new_lbq) {
			cic->lbq[BLK_RW_ASYNC] = new_lbq;
			lb_put_queue(lbq);
		}
	}

	lbq = cic->lbq[BLK_RW_SYNC];
	if (lbq)
		lb_mark_lbq_prio_changed(lbq);

	cic->ioprio = ioprio;
}

static void lb_init_lbq(struct lb_data *lbd, struct lb_queue *lbq,
			  pid_t pid, bool is_sync)
{
	RB_CLEAR_NODE(&lbq->rb_node);
	RB_CLEAR_NODE(&lbq->p_node);
	INIT_LIST_HEAD(&lbq->fifo);

	lbq->ref = 0;
	lbq->lbd = lbd;

	lb_mark_lbq_prio_changed(lbq);

	if (is_sync) {
		if (!lb_class_idle(lbq))
			lb_mark_lbq_idle_window(lbq);
		lb_mark_lbq_sync(lbq);
	}
	lbq->pid = pid;
}

#ifdef CONFIG_LB_GROUP_IOSCHED
static void check_blkcg_changed(struct lb_io_cq *cic, struct bio *bio)
{
	struct lb_data *lbd = cic_to_lbd(cic);
	struct lb_queue *sync_lbq;
	uint64_t serial_nr;

	rcu_read_lock();
	serial_nr = bio_blkcg(bio)->css.serial_nr;
	rcu_read_unlock();

	/*
	 * Check whether blkcg has changed.  The condition may trigger
	 * spuriously on a newly created cic but there's no harm.
	 */
	if (unlikely(!lbd) || likely(cic->blkcg_serial_nr == serial_nr))
		return;

	sync_lbq = cic_to_lbq(cic, 1);
	if (sync_lbq) {
		/*
		 * Drop reference to sync queue. A new sync queue will be
		 * assigned in new group upon arrival of a fresh request.
		 */
		lb_log_lbq(lbd, sync_lbq, "changed cgroup");
		cic_set_lbq(cic, NULL, 1);
		lb_put_queue(sync_lbq);
	}

	cic->blkcg_serial_nr = serial_nr;
}
#else
static inline void check_blkcg_changed(struct lb_io_cq *cic, struct bio *bio) { }
#endif  /* CONFIG_LB_GROUP_IOSCHED */

static struct lb_queue *
lb_find_alloc_queue(struct lb_data *lbd, bool is_sync, struct lb_io_cq *cic,
		     struct bio *bio, gfp_t gfp_mask)
{
	struct blkcg *blkcg;
	struct lb_queue *lbq, *new_lbq = NULL;
	struct lb_group *lbg;

retry:
	rcu_read_lock();

	blkcg = bio_blkcg(bio);
	lbg = lb_lookup_create_lbg(lbd, blkcg);
	if (!lbg) {
		lbq = &lbd->oom_lbq;
		goto out;
	}

	lbq = cic_to_lbq(cic, is_sync);

	/*
	 * Always try a new alloc if we fell back to the OOM lbq
	 * originally, since it should just be a temporary situation.
	 */
	if (!lbq || lbq == &lbd->oom_lbq) {
		lbq = NULL;
		if (new_lbq) {
			lbq = new_lbq;
			new_lbq = NULL;
		} else if (gfp_mask & __GFP_WAIT) {
			rcu_read_unlock();
			spin_unlock_irq(lbd->queue->queue_lock);
			new_lbq = kmem_cache_alloc_node(lb_pool,
					gfp_mask | __GFP_ZERO,
					lbd->queue->node);
			spin_lock_irq(lbd->queue->queue_lock);
			if (new_lbq)
				goto retry;
			else
				return &lbd->oom_lbq;
		} else {
			lbq = kmem_cache_alloc_node(lb_pool,
					gfp_mask | __GFP_ZERO,
					lbd->queue->node);
		}

		if (lbq) {
			lb_init_lbq(lbd, lbq, current->pid, is_sync);
			lb_init_prio_data(lbq, cic);
			lb_link_lbq_lbg(lbq, lbg);
			lb_log_lbq(lbd, lbq, "alloced");
		} else
			lbq = &lbd->oom_lbq;
	}
out:
	if (new_lbq)
		kmem_cache_free(lb_pool, new_lbq);

	rcu_read_unlock();
	return lbq;
}

static struct lb_queue **
lb_async_queue_prio(struct lb_data *lbd, int ioprio_class, int ioprio)
{
	switch (ioprio_class) {
	case IOPRIO_CLASS_RT:
		return &lbd->async_lbq[0][ioprio];
	case IOPRIO_CLASS_NONE:
		ioprio = IOPRIO_NORM;
		/* fall through */
	case IOPRIO_CLASS_BE:
		return &lbd->async_lbq[1][ioprio];
	case IOPRIO_CLASS_IDLE:
		return &lbd->async_idle_lbq;
	default:
		BUG();
	}
}

static struct lb_queue *
lb_get_queue(struct lb_data *lbd, bool is_sync, struct lb_io_cq *cic,
	      struct bio *bio, gfp_t gfp_mask)
{
	int ioprio_class = IOPRIO_PRIO_CLASS(cic->ioprio);
	int ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
	struct lb_queue **async_lbq = NULL;
	struct lb_queue *lbq = NULL;

	if (!is_sync) {
		if (!ioprio_valid(cic->ioprio)) {
			struct task_struct *tsk = current;
			ioprio = task_nice_ioprio(tsk);
			ioprio_class = task_nice_ioclass(tsk);
		}
		async_lbq = lb_async_queue_prio(lbd, ioprio_class, ioprio);
		lbq = *async_lbq;
	}

	if (!lbq)
		lbq = lb_find_alloc_queue(lbd, is_sync, cic, bio, gfp_mask);

	/*
	 * pin the queue now that it's allocated, scheduler exit will prune it
	 */
	if (!is_sync && !(*async_lbq)) {
		lbq->ref++;
		*async_lbq = lbq;
	}

	lbq->ref++;
	return lbq;
}

static void
__lb_update_io_thinktime(struct lb_ttime *ttime, unsigned long slice_idle)
{
	unsigned long elapsed = jiffies - ttime->last_end_request;
	elapsed = min(elapsed, 2UL * slice_idle);

	ttime->ttime_samples = (7*ttime->ttime_samples + 256) / 8;
	ttime->ttime_total = (7*ttime->ttime_total + 256*elapsed) / 8;
	ttime->ttime_mean = (ttime->ttime_total + 128) / ttime->ttime_samples;
}

static void
lb_update_io_thinktime(struct lb_data *lbd, struct lb_queue *lbq,
			struct lb_io_cq *cic)
{
	if (lb_lbq_sync(lbq)) {
		__lb_update_io_thinktime(&cic->ttime, lbd->lb_slice_idle);
		__lb_update_io_thinktime(&lbq->service_tree->ttime,
			lbd->lb_slice_idle);
	}
#ifdef CONFIG_LB_GROUP_IOSCHED
	__lb_update_io_thinktime(&lbq->lbg->ttime, lbd->lb_group_idle);
#endif
}

static void
lb_update_io_seektime(struct lb_data *lbd, struct lb_queue *lbq,
		       struct request *rq)
{
	sector_t sdist = 0;
	sector_t n_sec = blk_rq_sectors(rq);
	if (lbq->last_request_pos) {
		if (lbq->last_request_pos < blk_rq_pos(rq))
			sdist = blk_rq_pos(rq) - lbq->last_request_pos;
		else
			sdist = lbq->last_request_pos - blk_rq_pos(rq);
	}

	lbq->seek_history <<= 1;
	if (blk_queue_nonrot(lbd->queue))
		lbq->seek_history |= (n_sec < LBQ_SECT_THR_NONROT);
	else
		lbq->seek_history |= (sdist > LBQ_SEEK_THR);
}

/*
 * Disable idle window if the process thinks too long or seeks so much that
 * it doesn't matter
 */
static void
lb_update_idle_window(struct lb_data *lbd, struct lb_queue *lbq,
		       struct lb_io_cq *cic)
{
	int old_idle, enable_idle;

	/*
	 * Don't idle for async or idle io prio class
	 */
	if (!lb_lbq_sync(lbq) || lb_class_idle(lbq))
		return;

	enable_idle = old_idle = lb_lbq_idle_window(lbq);

	if (lbq->queued[0] + lbq->queued[1] >= 4)
		lb_mark_lbq_deep(lbq);

	if (lbq->next_rq && (lbq->next_rq->cmd_flags & REQ_NOIDLE))
		enable_idle = 0;
	else if (!atomic_read(&cic->icq.ioc->active_ref) ||
		 !lbd->lb_slice_idle ||
		 (!lb_lbq_deep(lbq) && LBQ_SEEKY(lbq)))
		enable_idle = 0;
	else if (sample_valid(cic->ttime.ttime_samples)) {
		if (cic->ttime.ttime_mean > lbd->lb_slice_idle)
			enable_idle = 0;
		else
			enable_idle = 1;
	}

	if (old_idle != enable_idle) {
		lb_log_lbq(lbd, lbq, "idle=%d", enable_idle);
		if (enable_idle)
			lb_mark_lbq_idle_window(lbq);
		else
			lb_clear_lbq_idle_window(lbq);
	}
}

/*
 * Check if new_lbq should preempt the currently active queue. Return 0 for
 * no or if we aren't sure, a 1 will cause a preempt.
 */
static bool
lb_should_preempt(struct lb_data *lbd, struct lb_queue *new_lbq,
		   struct request *rq)
{
	struct lb_queue *lbq;

	lbq = lbd->active_queue;
	if (!lbq)
		return false;

	if (lb_class_idle(new_lbq))
		return false;

	if (lb_class_idle(lbq))
		return true;

	/*
	 * Don't allow a non-RT request to preempt an ongoing RT lbq timeslice.
	 */
	if (lb_class_rt(lbq) && !lb_class_rt(new_lbq))
		return false;

	/*
	 * if the new request is sync, but the currently running queue is
	 * not, let the sync request have priority.
	 */
	if (rq_is_sync(rq) && !lb_lbq_sync(lbq))
		return true;

	if (new_lbq->lbg != lbq->lbg)
		return false;

	if (lb_slice_used(lbq))
		return true;

	/* Allow preemption only if we are idling on sync-noidle tree */
	if (lbd->serving_wl_type == SYNC_NOIDLE_WORKLOAD &&
	    lbq_type(new_lbq) == SYNC_NOIDLE_WORKLOAD &&
	    new_lbq->service_tree->count == 2 &&
	    RB_EMPTY_ROOT(&lbq->sort_list))
		return true;

	/*
	 * So both queues are sync. Let the new request get disk time if
	 * it's a metadata request and the current queue is doing regular IO.
	 */
	if ((rq->cmd_flags & REQ_PRIO) && !lbq->prio_pending)
		return true;

	/*
	 * Allow an RT request to pre-empt an ongoing non-RT lbq timeslice.
	 */
	if (lb_class_rt(new_lbq) && !lb_class_rt(lbq))
		return true;

	/* An idle queue should not be idle now for some reason */
	if (RB_EMPTY_ROOT(&lbq->sort_list) && !lb_should_idle(lbd, lbq))
		return true;

	if (!lbd->active_cic || !lb_lbq_wait_request(lbq))
		return false;

	/*
	 * if this request is as-good as one we would expect from the
	 * current lbq, let it preempt
	 */
	if (lb_rq_close(lbd, lbq, rq))
		return true;

	return false;
}

/*
 * lbq preempts the active queue. if we allowed preempt with no slice left,
 * let it have half of its nominal slice.
 */
static void lb_preempt_queue(struct lb_data *lbd, struct lb_queue *lbq)
{
	enum wl_type_t old_type = lbq_type(lbd->active_queue);

	lb_log_lbq(lbd, lbq, "preempt");
	lb_slice_expired(lbd, 1);

	/*
	 * workload type is changed, don't save slice, otherwise preempt
	 * doesn't happen
	 */
	if (old_type != lbq_type(lbq))
		lbq->lbg->saved_wl_slice = 0;

	/*
	 * Put the new queue at the front of the of the current list,
	 * so we know that it will be selected next.
	 */
	BUG_ON(!lb_lbq_on_rr(lbq));

	lb_service_tree_add(lbd, lbq, 1);

	lbq->slice_end = 0;
	lb_mark_lbq_slice_new(lbq);
}

/*
 * Called when a new fs request (rq) is added (to lbq). Check if there's
 * something we should do about it
 */
static void
lb_rq_enqueued(struct lb_data *lbd, struct lb_queue *lbq,
		struct request *rq)
{
	struct lb_io_cq *cic = RQ_CIC(rq);

	lbd->rq_queued++;
	if (rq->cmd_flags & REQ_PRIO)
		lbq->prio_pending++;

	lb_update_io_thinktime(lbd, lbq, cic);
	lb_update_io_seektime(lbd, lbq, rq);
	lb_update_idle_window(lbd, lbq, cic);

	lbq->last_request_pos = blk_rq_pos(rq) + blk_rq_sectors(rq);

	if (lbq == lbd->active_queue) {
		/*
		 * Remember that we saw a request from this process, but
		 * don't start queuing just yet. Otherwise we risk seeing lots
		 * of tiny requests, because we disrupt the normal plugging
		 * and merging. If the request is already larger than a single
		 * page, let it rip immediately. For that case we assume that
		 * merging is already done. Ditto for a busy system that
		 * has other work pending, don't risk delaying until the
		 * idle timer unplug to continue working.
		 */
		if (lb_lbq_wait_request(lbq)) {
			if (blk_rq_bytes(rq) > PAGE_CACHE_SIZE ||
			    lbd->busy_queues > 1) {
				lb_del_timer(lbd, lbq);
				lb_clear_lbq_wait_request(lbq);
				__blk_run_queue(lbd->queue);
			} else {
				lbg_stats_update_idle_time(lbq->lbg);
				lb_mark_lbq_must_dispatch(lbq);
			}
		}
	} else if (lb_should_preempt(lbd, lbq, rq)) {
		/*
		 * not the active queue - expire current slice if it is
		 * idle and has expired it's mean thinktime or this new queue
		 * has some old slice time left and is of higher priority or
		 * this new queue is RT and the current one is BE
		 */
		lb_preempt_queue(lbd, lbq);
		__blk_run_queue(lbd->queue);
	}
}

static void lb_insert_request(struct request_queue *q, struct request *rq)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct lb_queue *lbq = RQ_LBQ(rq);

	lb_log_lbq(lbd, lbq, "insert_request");
	lb_init_prio_data(lbq, RQ_CIC(rq));

	rq->fifo_time = jiffies + lbd->lb_fifo_expire[rq_is_sync(rq)];
	list_add_tail(&rq->queuelist, &lbq->fifo);
	lb_add_rq_rb(rq);
	lbg_stats_update_io_add(RQ_LBG(rq), lbd->serving_group,
				 rq->cmd_flags);
	lb_rq_enqueued(lbd, lbq, rq);
}

/*
 * Update hw_tag based on peak queue depth over 50 samples under
 * sufficient load.
 */
static void lb_update_hw_tag(struct lb_data *lbd)
{
	struct lb_queue *lbq = lbd->active_queue;

	if (lbd->rq_in_driver > lbd->hw_tag_est_depth)
		lbd->hw_tag_est_depth = lbd->rq_in_driver;

	if (lbd->hw_tag == 1)
		return;

	if (lbd->rq_queued <= LB_HW_QUEUE_MIN &&
	    lbd->rq_in_driver <= LB_HW_QUEUE_MIN)
		return;

	/*
	 * If active queue hasn't enough requests and can idle, lb might not
	 * dispatch sufficient requests to hardware. Don't zero hw_tag in this
	 * case
	 */
	if (lbq && lb_lbq_idle_window(lbq) &&
	    lbq->dispatched + lbq->queued[0] + lbq->queued[1] <
	    LB_HW_QUEUE_MIN && lbd->rq_in_driver < LB_HW_QUEUE_MIN)
		return;

	if (lbd->hw_tag_samples++ < 50)
		return;

	if (lbd->hw_tag_est_depth >= LB_HW_QUEUE_MIN)
		lbd->hw_tag = 1;
	else
		lbd->hw_tag = 0;
}

static bool lb_should_wait_busy(struct lb_data *lbd, struct lb_queue *lbq)
{
	struct lb_io_cq *cic = lbd->active_cic;

	/* If the queue already has requests, don't wait */
	if (!RB_EMPTY_ROOT(&lbq->sort_list))
		return false;

	/* If there are other queues in the group, don't wait */
	if (lbq->lbg->nr_lbq > 1)
		return false;

	/* the only queue in the group, but think time is big */
	if (lb_io_thinktime_big(lbd, &lbq->lbg->ttime, true))
		return false;

	if (lb_slice_used(lbq))
		return true;

	/* if slice left is less than think time, wait busy */
	if (cic && sample_valid(cic->ttime.ttime_samples)
	    && (lbq->slice_end - jiffies < cic->ttime.ttime_mean))
		return true;

	/*
	 * If think times is less than a jiffy than ttime_mean=0 and above
	 * will not be true. It might happen that slice has not expired yet
	 * but will expire soon (4-5 ns) during select_queue(). To cover the
	 * case where think time is less than a jiffy, mark the queue wait
	 * busy if only 1 jiffy is left in the slice.
	 */
	if (lbq->slice_end - jiffies == 1)
		return true;

	return false;
}

static void lb_completed_request(struct request_queue *q, struct request *rq)
{
	struct lb_queue *lbq = RQ_LBQ(rq);
	struct lb_data *lbd = lbq->lbd;
	const int sync = rq_is_sync(rq);
	unsigned long now;

	now = jiffies;
	lb_log_lbq(lbd, lbq, "complete rqnoidle %d",
		     !!(rq->cmd_flags & REQ_NOIDLE));

	lb_update_hw_tag(lbd);

	WARN_ON(!lbd->rq_in_driver);
	WARN_ON(!lbq->dispatched);
	lbd->rq_in_driver--;
	lbq->dispatched--;
	(RQ_LBG(rq))->dispatched--;
	lbg_stats_update_completion(lbq->lbg, rq_start_time_ns(rq),
				     rq_io_start_time_ns(rq), rq->cmd_flags);

	lbd->rq_in_flight[lb_lbq_sync(lbq)]--;

	if (sync) {
		struct lb_rb_root *st;

		RQ_CIC(rq)->ttime.last_end_request = now;

		if (lb_lbq_on_rr(lbq))
			st = lbq->service_tree;
		else
			st = st_for(lbq->lbg, lbq_class(lbq),
					lbq_type(lbq));

		st->ttime.last_end_request = now;
		if (!time_after(rq->start_time + lbd->lb_fifo_expire[1], now))
			lbd->last_delayed_sync = now;
	}

#ifdef CONFIG_LB_GROUP_IOSCHED
	lbq->lbg->ttime.last_end_request = now;
#endif

	/*
	 * If this is the active queue, check if it needs to be expired,
	 * or if we want to idle in case it has no pending requests.
	 */
	if (lbd->active_queue == lbq) {
		const bool lbq_empty = RB_EMPTY_ROOT(&lbq->sort_list);

		if (lb_lbq_slice_new(lbq)) {
			lb_set_prio_slice(lbd, lbq);
			lb_clear_lbq_slice_new(lbq);
		}

		/*
		 * Should we wait for next request to come in before we expire
		 * the queue.
		 */
		if (lb_should_wait_busy(lbd, lbq)) {
			unsigned long extend_sl = lbd->lb_slice_idle;
			if (!lbd->lb_slice_idle)
				extend_sl = lbd->lb_group_idle;
			lbq->slice_end = jiffies + extend_sl;
			lb_mark_lbq_wait_busy(lbq);
			lb_log_lbq(lbd, lbq, "will busy wait");
		}

		/*
		 * Idling is not enabled on:
		 * - expired queues
		 * - idle-priority queues
		 * - async queues
		 * - queues with still some requests queued
		 * - when there is a close cooperator
		 */
		if (lb_slice_used(lbq) || lb_class_idle(lbq))
			lb_slice_expired(lbd, 1);
		else if (sync && lbq_empty &&
			 !lb_close_cooperator(lbd, lbq)) {
			lb_arm_slice_timer(lbd);
		}
	}

	if (!lbd->rq_in_driver)
		lb_schedule_dispatch(lbd);
}

static inline int __lb_may_queue(struct lb_queue *lbq)
{
	if (lb_lbq_wait_request(lbq) && !lb_lbq_must_alloc_slice(lbq)) {
		lb_mark_lbq_must_alloc_slice(lbq);
		return ELV_MQUEUE_MUST;
	}

	return ELV_MQUEUE_MAY;
}

static int lb_may_queue(struct request_queue *q, int rw)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct task_struct *tsk = current;
	struct lb_io_cq *cic;
	struct lb_queue *lbq;

	/*
	 * don't force setup of a queue from here, as a call to may_queue
	 * does not necessarily imply that a request actually will be queued.
	 * so just lookup a possibly existing queue, or return 'may queue'
	 * if that fails
	 */
	cic = lb_cic_lookup(lbd, tsk->io_context);
	if (!cic)
		return ELV_MQUEUE_MAY;

	lbq = cic_to_lbq(cic, rw_is_sync(rw));
	if (lbq) {
		lb_init_prio_data(lbq, cic);

		return __lb_may_queue(lbq);
	}

	return ELV_MQUEUE_MAY;
}

/*
 * queue lock held here
 */
static void lb_put_request(struct request *rq)
{
	struct lb_queue *lbq = RQ_LBQ(rq);

	if (lbq) {
		const int rw = rq_data_dir(rq);

		BUG_ON(!lbq->allocated[rw]);
		lbq->allocated[rw]--;

		/* Put down rq reference on lbg */
		lbg_put(RQ_LBG(rq));
		rq->elv.priv[0] = NULL;
		rq->elv.priv[1] = NULL;

		lb_put_queue(lbq);
	}
}

static struct lb_queue *
lb_merge_lbqs(struct lb_data *lbd, struct lb_io_cq *cic,
		struct lb_queue *lbq)
{
	lb_log_lbq(lbd, lbq, "merging with queue %p", lbq->new_lbq);
	cic_set_lbq(cic, lbq->new_lbq, 1);
	lb_mark_lbq_coop(lbq->new_lbq);
	lb_put_queue(lbq);
	return cic_to_lbq(cic, 1);
}

/*
 * Returns NULL if a new lbq should be allocated, or the old lbq if this
 * was the last process referring to said lbq.
 */
static struct lb_queue *
split_lbq(struct lb_io_cq *cic, struct lb_queue *lbq)
{
	if (lbq_process_refs(lbq) == 1) {
		lbq->pid = current->pid;
		lb_clear_lbq_coop(lbq);
		lb_clear_lbq_split_coop(lbq);
		return lbq;
	}

	cic_set_lbq(cic, NULL, 1);

	lb_put_cooperator(lbq);

	lb_put_queue(lbq);
	return NULL;
}
/*
 * Allocate lb data structures associated with this request.
 */
static int
lb_set_request(struct request_queue *q, struct request *rq, struct bio *bio,
		gfp_t gfp_mask)
{
	struct lb_data *lbd = q->elevator->elevator_data;
	struct lb_io_cq *cic = icq_to_cic(rq->elv.icq);
	const int rw = rq_data_dir(rq);
	const bool is_sync = rq_is_sync(rq);
	struct lb_queue *lbq;

	might_sleep_if(gfp_mask & __GFP_WAIT);

	spin_lock_irq(q->queue_lock);

	check_ioprio_changed(cic, bio);
	check_blkcg_changed(cic, bio);
new_queue:
	lbq = cic_to_lbq(cic, is_sync);
	if (!lbq || lbq == &lbd->oom_lbq) {
		lbq = lb_get_queue(lbd, is_sync, cic, bio, gfp_mask);
		cic_set_lbq(cic, lbq, is_sync);
	} else {
		/*
		 * If the queue was seeky for too long, break it apart.
		 */
		if (lb_lbq_coop(lbq) && lb_lbq_split_coop(lbq)) {
			lb_log_lbq(lbd, lbq, "breaking apart lbq");
			lbq = split_lbq(cic, lbq);
			if (!lbq)
				goto new_queue;
		}

		/*
		 * Check to see if this queue is scheduled to merge with
		 * another, closely cooperating queue.  The merging of
		 * queues happens here as it must be done in process context.
		 * The reference on new_lbq was taken in merge_lbqs.
		 */
		if (lbq->new_lbq)
			lbq = lb_merge_lbqs(lbd, cic, lbq);
	}

	lbq->allocated[rw]++;

	lbq->ref++;
	lbg_get(lbq->lbg);
	rq->elv.priv[0] = lbq;
	rq->elv.priv[1] = lbq->lbg;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void lb_kick_queue(struct work_struct *work)
{
	struct lb_data *lbd =
		container_of(work, struct lb_data, unplug_work);
	struct request_queue *q = lbd->queue;

	spin_lock_irq(q->queue_lock);
	__blk_run_queue(lbd->queue);
	spin_unlock_irq(q->queue_lock);
}

/*
 * Timer running if the active_queue is currently idling inside its time slice
 */
static void lb_idle_slice_timer(unsigned long data)
{
	struct lb_data *lbd = (struct lb_data *) data;
	struct lb_queue *lbq;
	unsigned long flags;
	int timed_out = 1;

	lb_log(lbd, "idle timer fired");

	spin_lock_irqsave(lbd->queue->queue_lock, flags);

	lbq = lbd->active_queue;
	if (lbq) {
		timed_out = 0;

		/*
		 * We saw a request before the queue expired, let it through
		 */
		if (lb_lbq_must_dispatch(lbq))
			goto out_kick;

		/*
		 * expired
		 */
		if (lb_slice_used(lbq))
			goto expire;

		/*
		 * only expire and reinvoke request handler, if there are
		 * other queues with pending requests
		 */
		if (!lbd->busy_queues)
			goto out_cont;

		/*
		 * not expired and it has a request pending, let it dispatch
		 */
		if (!RB_EMPTY_ROOT(&lbq->sort_list))
			goto out_kick;

		/*
		 * Queue depth flag is reset only when the idle didn't succeed
		 */
		lb_clear_lbq_deep(lbq);
	}
expire:
	lb_slice_expired(lbd, timed_out);
out_kick:
	lb_schedule_dispatch(lbd);
out_cont:
	spin_unlock_irqrestore(lbd->queue->queue_lock, flags);
}

static void lb_shutdown_timer_wq(struct lb_data *lbd)
{
	del_timer_sync(&lbd->idle_slice_timer);
	cancel_work_sync(&lbd->unplug_work);
}

static void lb_put_async_queues(struct lb_data *lbd)
{
	int i;

	for (i = 0; i < IOPRIO_BE_NR; i++) {
		if (lbd->async_lbq[0][i])
			lb_put_queue(lbd->async_lbq[0][i]);
		if (lbd->async_lbq[1][i])
			lb_put_queue(lbd->async_lbq[1][i]);
	}

	if (lbd->async_idle_lbq)
		lb_put_queue(lbd->async_idle_lbq);
}

static void lb_exit_queue(struct elevator_queue *e)
{
	struct lb_data *lbd = e->elevator_data;
	struct request_queue *q = lbd->queue;

	lb_shutdown_timer_wq(lbd);

	spin_lock_irq(q->queue_lock);

	if (lbd->active_queue)
		__lb_slice_expired(lbd, lbd->active_queue, 0);

	lb_put_async_queues(lbd);

	spin_unlock_irq(q->queue_lock);

	lb_shutdown_timer_wq(lbd);

#ifdef CONFIG_LB_GROUP_IOSCHED
	blkcg_deactivate_policy(q, &blkcg_policy_lb);
#else
	kfree(lbd->root_group);
#endif
	kfree(lbd);
}

static int lb_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct lb_data *lbd;
	struct blkcg_gq *blkg __maybe_unused;
	int i, ret;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	lbd = kzalloc_node(sizeof(*lbd), GFP_KERNEL, q->node);
	if (!lbd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = lbd;

	lbd->queue = q;
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);

	/* Init root service tree */
	lbd->grp_service_tree = LB_RB_ROOT;

	/* Init root group and prefer root group over other groups by default */
#ifdef CONFIG_LB_GROUP_IOSCHED
	ret = blkcg_activate_policy(q, &blkcg_policy_lb);
	if (ret)
		goto out_free;

	lbd->root_group = blkg_to_lbg(q->root_blkg);
#else
	ret = -ENOMEM;
	lbd->root_group = kzalloc_node(sizeof(*lbd->root_group),
					GFP_KERNEL, lbd->queue->node);
	if (!lbd->root_group)
		goto out_free;

	lb_init_lbg_base(lbd->root_group);
#endif
	lbd->root_group->weight = 2 * LB_WEIGHT_DEFAULT;
	lbd->root_group->leaf_weight = 2 * LB_WEIGHT_DEFAULT;

	/*
	 * Not strictly needed (since RB_ROOT just clears the node and we
	 * zeroed lbd on alloc), but better be safe in case someone decides
	 * to add magic to the rb code
	 */
	for (i = 0; i < LB_PRIO_LISTS; i++)
		lbd->prio_trees[i] = RB_ROOT;

	/*
	 * Our fallback lbq if lb_find_alloc_queue() runs into OOM issues.
	 * Grab a permanent reference to it, so that the normal code flow
	 * will not attempt to free it.  oom_lbq is linked to root_group
	 * but shouldn't hold a reference as it'll never be unlinked.  Lose
	 * the reference from linking right away.
	 */
	lb_init_lbq(lbd, &lbd->oom_lbq, 1, 0);
	lbd->oom_lbq.ref++;

	spin_lock_irq(q->queue_lock);
	lb_link_lbq_lbg(&lbd->oom_lbq, lbd->root_group);
	lbg_put(lbd->root_group);
	spin_unlock_irq(q->queue_lock);

	init_timer(&lbd->idle_slice_timer);
	lbd->idle_slice_timer.function = lb_idle_slice_timer;
	lbd->idle_slice_timer.data = (unsigned long) lbd;

	INIT_WORK(&lbd->unplug_work, lb_kick_queue);

	lbd->lb_quantum = lb_quantum;
	lbd->lb_fifo_expire[0] = lb_fifo_expire[0];
	lbd->lb_fifo_expire[1] = lb_fifo_expire[1];
	lbd->lb_back_max = lb_back_max;
	lbd->lb_back_penalty = lb_back_penalty;
	lbd->lb_slice[0] = lb_slice_async;
	lbd->lb_slice[1] = lb_slice_sync;
	lbd->lb_target_latency = lb_target_latency;
	lbd->lb_slice_async_rq = lb_slice_async_rq;
//	lbd->lb_slice_idle = lb_slice_idle;
	lbd->lb_slice_idle = blk_queue_nonrot(q) ? 0 : lb_slice_idle; // if drive is NON ROTATION  device, change sched to iops mode.
	lbd->lb_group_idle = lb_group_idle;
	lbd->lb_latency = 1;
	lbd->hw_tag = -1;
	/*
	 * we optimistically start assuming sync ops weren't delayed in last
	 * second, in order to have larger depth for async operations.
	 */
	lbd->last_delayed_sync = jiffies - HZ;
	return 0;

out_free:
	kfree(lbd);
	kobject_put(&eq->kobj);
	return ret;
}

/*
 * sysfs parts below -->
 */
static ssize_t
lb_var_show(unsigned int var, char *page)
{
	return sprintf(page, "%u\n", var);
}

static ssize_t
lb_var_store(unsigned int *var, const char *page, size_t count)
{
	char *p = (char *) page;

	*var = simple_strtoul(p, &p, 10);
	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct lb_data *lbd = e->elevator_data;			\
	unsigned int __data = __VAR;					\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return lb_var_show(__data, (page));				\
}
SHOW_FUNCTION(lb_quantum_show, lbd->lb_quantum, 0);
SHOW_FUNCTION(lb_fifo_expire_sync_show, lbd->lb_fifo_expire[1], 1);
SHOW_FUNCTION(lb_fifo_expire_async_show, lbd->lb_fifo_expire[0], 1);
SHOW_FUNCTION(lb_back_seek_max_show, lbd->lb_back_max, 0);
SHOW_FUNCTION(lb_back_seek_penalty_show, lbd->lb_back_penalty, 0);
SHOW_FUNCTION(lb_slice_idle_show, lbd->lb_slice_idle, 1);
SHOW_FUNCTION(lb_group_idle_show, lbd->lb_group_idle, 1);
SHOW_FUNCTION(lb_slice_sync_show, lbd->lb_slice[1], 1);
SHOW_FUNCTION(lb_slice_async_show, lbd->lb_slice[0], 1);
SHOW_FUNCTION(lb_slice_async_rq_show, lbd->lb_slice_async_rq, 0);
SHOW_FUNCTION(lb_low_latency_show, lbd->lb_latency, 0);
SHOW_FUNCTION(lb_target_latency_show, lbd->lb_target_latency, 1);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct lb_data *lbd = e->elevator_data;			\
	unsigned int __data;						\
	int ret = lb_var_store(&__data, (page), count);		\
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
STORE_FUNCTION(lb_quantum_store, &lbd->lb_quantum, 1, UINT_MAX, 0);
STORE_FUNCTION(lb_fifo_expire_sync_store, &lbd->lb_fifo_expire[1], 1,
		UINT_MAX, 1);
STORE_FUNCTION(lb_fifo_expire_async_store, &lbd->lb_fifo_expire[0], 1,
		UINT_MAX, 1);
STORE_FUNCTION(lb_back_seek_max_store, &lbd->lb_back_max, 0, UINT_MAX, 0);
STORE_FUNCTION(lb_back_seek_penalty_store, &lbd->lb_back_penalty, 1,
		UINT_MAX, 0);
STORE_FUNCTION(lb_slice_idle_store, &lbd->lb_slice_idle, 0, UINT_MAX, 1);
STORE_FUNCTION(lb_group_idle_store, &lbd->lb_group_idle, 0, UINT_MAX, 1);
STORE_FUNCTION(lb_slice_sync_store, &lbd->lb_slice[1], 1, UINT_MAX, 1);
STORE_FUNCTION(lb_slice_async_store, &lbd->lb_slice[0], 1, UINT_MAX, 1);
STORE_FUNCTION(lb_slice_async_rq_store, &lbd->lb_slice_async_rq, 1,
		UINT_MAX, 0);
STORE_FUNCTION(lb_low_latency_store, &lbd->lb_latency, 0, 1, 0);
STORE_FUNCTION(lb_target_latency_store, &lbd->lb_target_latency, 1, UINT_MAX, 1);
#undef STORE_FUNCTION

#define LB_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, lb_##name##_show, lb_##name##_store)

static struct elv_fs_entry lb_attrs[] = {
	LB_ATTR(quantum),
	LB_ATTR(fifo_expire_sync),
	LB_ATTR(fifo_expire_async),
	LB_ATTR(back_seek_max),
	LB_ATTR(back_seek_penalty),
	LB_ATTR(slice_sync),
	LB_ATTR(slice_async),
	LB_ATTR(slice_async_rq),
	LB_ATTR(slice_idle),
	LB_ATTR(group_idle),
	LB_ATTR(low_latency),
	LB_ATTR(target_latency),
	__ATTR_NULL
};

static struct elevator_type iosched_lb = {
	.ops = {
		.elevator_merge_fn = 		lb_merge,
		.elevator_merged_fn =		lb_merged_request,
		.elevator_merge_req_fn =	lb_merged_requests,
		.elevator_allow_merge_fn =	lb_allow_merge,
		.elevator_bio_merged_fn =	lb_bio_merged,
		.elevator_dispatch_fn =		lb_dispatch_requests,
		.elevator_add_req_fn =		lb_insert_request,
		.elevator_activate_req_fn =	lb_activate_request,
		.elevator_deactivate_req_fn =	lb_deactivate_request,
		.elevator_completed_req_fn =	lb_completed_request,
		.elevator_former_req_fn =	elv_rb_former_request,
		.elevator_latter_req_fn =	elv_rb_latter_request,
		.elevator_init_icq_fn =		lb_init_icq,
		.elevator_exit_icq_fn =		lb_exit_icq,
		.elevator_set_req_fn =		lb_set_request,
		.elevator_put_req_fn =		lb_put_request,
		.elevator_may_queue_fn =	lb_may_queue,
		.elevator_init_fn =		lb_init_queue,
		.elevator_exit_fn =		lb_exit_queue,
	},
	.icq_size	=	sizeof(struct lb_io_cq),
	.icq_align	=	__alignof__(struct lb_io_cq),
	.elevator_attrs =	lb_attrs,
	.elevator_name	=	"lb",
	.elevator_owner =	THIS_MODULE,
};

#ifdef CONFIG_LB_GROUP_IOSCHED
static struct blkcg_policy blkcg_policy_lb = {
	.pd_size		= sizeof(struct lb_group),
	.cftypes		= lb_blkcg_files,

	.pd_init_fn		= lb_pd_init,
	.pd_offline_fn		= lb_pd_offline,
	.pd_reset_stats_fn	= lb_pd_reset_stats,
};
#endif

static int __init lb_init(void)
{
	int ret;

	/*
	 * could be 0 on HZ < 1000 setups
	 */
	if (!lb_slice_async)
		lb_slice_async = 1;
	if (!lb_slice_idle)
		lb_slice_idle = 1;

#ifdef CONFIG_LB_GROUP_IOSCHED
	if (!lb_group_idle)
		lb_group_idle = 1;

	ret = blkcg_policy_register(&blkcg_policy_lb);
	if (ret)
		return ret;
#else
	lb_group_idle = 0;
#endif

	ret = -ENOMEM;
	lb_pool = KMEM_CACHE(lb_queue, 0);
	if (!lb_pool)
		goto err_pol_unreg;
	
	/** blkcg_policy_register - register a blkcg policy
	 * @pol: blkcg policy to register
	 *
	 * Register @pol with blkcg core.  Might sleep and @pol may be modified on
	 * successful registration.  Returns 0 on success and -errno on failure.
	 */
	ret = elv_register(&iosched_lb); 
	if (ret)
		goto err_free_pool;

	return 0;

err_free_pool:
	kmem_cache_destroy(lb_pool);
err_pol_unreg:
#ifdef CONFIG_LB_GROUP_IOSCHED
	blkcg_policy_unregister(&blkcg_policy_lb);
#endif
	return ret;
}

static void __exit lb_exit(void)
{
#ifdef CONFIG_LB_GROUP_IOSCHED
	blkcg_policy_unregister(&blkcg_policy_lb);
#endif
	elv_unregister(&iosched_lb);
	kmem_cache_destroy(lb_pool);
}

module_init(lb_init);
module_exit(lb_exit);

MODULE_AUTHOR("Jens Axboe");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Completely Fair Queueing IO scheduler");
