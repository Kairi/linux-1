/*
 *  GFQ, or group fairness queueing, for ssd i/o scheduler.
 *
 *  Based on ideas from a previously unfinished io
 *  scheduler (round robin per-process disk scheduling) and Andrea Arcangeli and cfq scheduler.
 *
 *  Copyright (C) 2015 Kairi OKUMURA (kairi199088@gmail.com)
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
static const int gfq_quantum = 8;
static const int gfq_fifo_expire[2] = { HZ / 4, HZ / 8 };
/* maximum backwards seek, in KiB */
static const int gfq_back_max = 16 * 1024;
/* penalty of a backwards seek */
static const int gfq_back_penalty = 2;
static const int gfq_slice_sync = HZ / 10;
static int gfq_slice_async = HZ / 25;
static const int gfq_slice_async_rq = 2;
static int gfq_slice_idle = 0; // if 0, and if storage supports NCQ, GFQ internally switches to IOPS mode.
static int gfq_group_idle = HZ / 125;
static const int gfq_target_latency = HZ * 3/10; /* 300 ms */
static const int gfq_hist_divisor = 4;
static const int lb_size = 2 * 1024 * 1024; // 2MB


/*
 * offset from end of service tree
 */
#define GFQ_IDLE_DELAY		(HZ / 5)

/*
 * below this threshold, we consider thinktime immediate
 */
#define GFQ_MIN_TT		(2)

#define GFQ_SLICE_SCALE		(5)
#define GFQ_HW_QUEUE_MIN	(5)
#define GFQ_SERVICE_SHIFT       12

#define GFQQ_SEEK_THR		(sector_t)(8 * 100)
#define GFQQ_CLOSE_THR		(sector_t)(8 * 1024)
#define GFQQ_SECT_THR_NONROT	(sector_t)(2 * 32)
#define GFQQ_SEEKY(gfqq)	(hweight32(gfqq->seek_history) > 32/8)

#define RQ_CIC(rq)		icq_to_cic((rq)->elv.icq)
#define RQ_GFQQ(rq)		(struct gfq_queue *) ((rq)->elv.priv[0])
#define RQ_GFQG(rq)		(struct gfq_group *) ((rq)->elv.priv[1])

static struct kmem_cache *gfq_pool;

#define GFQ_PRIO_LISTS		IOPRIO_BE_NR
#define gfq_class_idle(gfqq)	((gfqq)->ioprio_class == IOPRIO_CLASS_IDLE)
#define gfq_class_rt(gfqq)	((gfqq)->ioprio_class == IOPRIO_CLASS_RT)

#define sample_valid(samples)	((samples) > 80)
#define rb_entry_gfqg(node)	rb_entry((node), struct gfq_group, rb_node)

struct gfq_ttime {
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
struct gfq_rb_root {
	struct rb_root rb;
	struct rb_node *left;
	unsigned count;
	u64 min_vdisktime;
	struct gfq_ttime ttime;
};
#define GFQ_RB_ROOT	(struct gfq_rb_root) { .rb = RB_ROOT, \
			.ttime = {.last_end_request = jiffies,},}

/*
 * Per process-grouping structure
 */
struct gfq_queue {
	/* reference count */
	int ref;
	/* various state flags, see below */
	unsigned int flags;
	/* parent gfq_data */
	struct gfq_data *gfqd;
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

	struct gfq_rb_root *service_tree;
	struct gfq_queue *new_gfqq;
	struct gfq_group *gfqg;
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
	GFQ_PRIO_NR,
};

/*
 * Second index in the service_trees.
 */
enum wl_type_t {
	ASYNC_WORKLOAD = 0,
	SYNC_NOIDLE_WORKLOAD = 1,
	SYNC_WORKLOAD = 2
};

struct gfqg_stats {
#ifdef CONFIG_GFQ_GROUP_IOSCHED
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
#endif	/* CONFIG_GFQ_GROUP_IOSCHED */
};

/* This is per cgroup per device grouping structure */
struct gfq_group {
	/* must be the first member */
	struct blkg_policy_data pd;

	/* group service_tree member */
	struct rb_node rb_node;

	/* group service_tree key */
	u64 vdisktime;

	/*
	 * The number of active gfqgs and sum of their weights under this
	 * gfqg.  This covers this gfqg's leaf_weight and all children's
	 * weights, but does not cover weights of further descendants.
	 *
	 * If a gfqg is on the service tree, it's active.  An active gfqg
	 * also activates its parent and contributes to the children_weight
	 * of the parent.
	 */
	int nr_active;
	unsigned int children_weight;

	/*
	 * vfraction is the fraction of vdisktime that the tasks in this
	 * gfqg are entitled to.  This is determined by compounding the
	 * ratios walking up from this gfqg to the root.
	 *
	 * It is in fixed point w/ GFQ_SERVICE_SHIFT and the sum of all
	 * vfractions on a service tree is approximately 1.  The sum may
	 * deviate a bit due to rounding errors and fluctuations caused by
	 * gfqgs entering and leaving the service tree.
	 */
	unsigned int vfraction;

	/*
	 * There are two weights - (internal) weight is the weight of this
	 * gfqg against the sibling gfqgs.  leaf_weight is the wight of
	 * this gfqg against the child gfqgs.  For the root gfqg, both
	 * weights are kept in sync for backward compatibility.
	 */
	unsigned int weight;
	unsigned int new_weight;
	unsigned int dev_weight;

	unsigned int leaf_weight;
	unsigned int new_leaf_weight;
	unsigned int dev_leaf_weight;

	/* number of gfqq currently on this group */
	int nr_gfqq;

	/*
	 * Per group busy queues average. Useful for workload slice calc. We
	 * create the array for each prio class but at run time it is used
	 * only for RT and BE class and slot for IDLE class remains unused.
	 * This is primarily done to avoid confusion and a gcc warning.
	 */
	unsigned int busy_queues_avg[GFQ_PRIO_NR];
	/*
	 * rr lists of queues with requests. We maintain service trees for
	 * RT and BE classes. These trees are subdivided in subclasses
	 * of SYNC, SYNC_NOIDLE and ASYNC based on workload type. For IDLE
	 * class there is no subclassification and all the gfq queues go on
	 * a single tree service_tree_idle.
	 * Counts are embedded in the gfq_rb_root
	 */
	struct gfq_rb_root service_trees[2][3];
	struct gfq_rb_root service_tree_idle;

	unsigned long saved_wl_slice;
	enum wl_type_t saved_wl_type;
	enum wl_class_t saved_wl_class;

	/* number of requests that are on the dispatch list or inside driver */
	int dispatched;
	struct gfq_ttime ttime;
	struct gfqg_stats stats;	/* stats for this gfqg */
	struct gfqg_stats dead_stats;	/* stats pushed from dead children */
};

struct gfq_io_cq {
	struct io_cq		icq;		/* must be the first member */
	struct gfq_queue	*gfqq[2];
	struct gfq_ttime	ttime;
	int			ioprio;		/* the current ioprio */
#ifdef CONFIG_GFQ_GROUP_IOSCHED
	uint64_t		blkcg_serial_nr; /* the current blkcg serial */
#endif
};

/*
 * Per block device queue structure
 */
struct gfq_data {
	struct request_queue *queue;
	/* Root service tree for gfq_groups */
	struct gfq_rb_root grp_service_tree;
	struct gfq_group *root_group;

	/*
	 * The priority currently being served
	 */
	enum wl_class_t serving_wl_class;
	enum wl_type_t serving_wl_type;
	unsigned long workload_expires;
	struct gfq_group *serving_group;

	/*
	 * Each priority tree is sorted by next_request position.  These
	 * trees are used when determining if two or more queues are
	 * interleaving requests (see gfq_close_cooperator).
	 */
	struct rb_root prio_trees[GFQ_PRIO_LISTS];

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
	 * -1 => indeterminate, (gfq will behave as if NCQ is present, to allow better detection)
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

	struct gfq_queue *active_queue;
	struct gfq_io_cq *active_cic;

	/*
	 * async queue for each priority case
	 */
	struct gfq_queue *async_gfqq[2][IOPRIO_BE_NR];
	struct gfq_queue *async_idle_gfqq;

	sector_t last_position;

	/*
	 * tunables, see top of file
	 */
	unsigned int gfq_quantum;
	unsigned int gfq_fifo_expire[2];
	unsigned int gfq_back_penalty;
	unsigned int gfq_back_max;
	unsigned int gfq_slice[2];
	unsigned int gfq_slice_async_rq;
	unsigned int gfq_slice_idle;
	unsigned int gfq_group_idle;
	unsigned int gfq_latency;
	unsigned int gfq_target_latency;

	/*
	 * Fallback dummy gfqq for extreme OOM conditions
	 */
	struct gfq_queue oom_gfqq;

	unsigned long last_delayed_sync;
};

static struct gfq_group *gfq_get_next_gfqg(struct gfq_data *gfqd);

static struct gfq_rb_root *st_for(struct gfq_group *gfqg,
					    enum wl_class_t class,
					    enum wl_type_t type)
{
	if (!gfqg)
		return NULL;

	if (class == IDLE_WORKLOAD)
		return &gfqg->service_tree_idle;

	return &gfqg->service_trees[class][type];
}

enum gfqq_state_flags {
	GFQ_GFQQ_FLAG_on_rr = 0,	/* on round-robin busy list */
	GFQ_GFQQ_FLAG_wait_request,	/* waiting for a request */
	GFQ_GFQQ_FLAG_must_dispatch,	/* must be allowed a dispatch */
	GFQ_GFQQ_FLAG_must_alloc_slice,	/* per-slice must_alloc flag */
	GFQ_GFQQ_FLAG_fifo_expire,	/* FIFO checked in this slice */
	GFQ_GFQQ_FLAG_idle_window,	/* slice idling enabled */
	GFQ_GFQQ_FLAG_prio_changed,	/* task priority has changed */
	GFQ_GFQQ_FLAG_slice_new,	/* no requests dispatched in slice */
	GFQ_GFQQ_FLAG_sync,		/* synchronous queue */
	GFQ_GFQQ_FLAG_coop,		/* gfqq is shared */
	GFQ_GFQQ_FLAG_split_coop,	/* shared gfqq will be splitted */
	GFQ_GFQQ_FLAG_deep,		/* sync gfqq experienced large depth */
	GFQ_GFQQ_FLAG_wait_busy,	/* Waiting for next request */
};

#define GFQ_GFQQ_FNS(name)						\
static inline void gfq_mark_gfqq_##name(struct gfq_queue *gfqq)		\
{									\
	(gfqq)->flags |= (1 << GFQ_GFQQ_FLAG_##name);			\
}									\
static inline void gfq_clear_gfqq_##name(struct gfq_queue *gfqq)	\
{									\
	(gfqq)->flags &= ~(1 << GFQ_GFQQ_FLAG_##name);			\
}									\
static inline int gfq_gfqq_##name(const struct gfq_queue *gfqq)		\
{									\
	return ((gfqq)->flags & (1 << GFQ_GFQQ_FLAG_##name)) != 0;	\
}

GFQ_GFQQ_FNS(on_rr);
GFQ_GFQQ_FNS(wait_request);
GFQ_GFQQ_FNS(must_dispatch);
GFQ_GFQQ_FNS(must_alloc_slice);
GFQ_GFQQ_FNS(fifo_expire);
GFQ_GFQQ_FNS(idle_window);
GFQ_GFQQ_FNS(prio_changed);
GFQ_GFQQ_FNS(slice_new);
GFQ_GFQQ_FNS(sync);
GFQ_GFQQ_FNS(coop);
GFQ_GFQQ_FNS(split_coop);
GFQ_GFQQ_FNS(deep);
GFQ_GFQQ_FNS(wait_busy);
#undef GFQ_GFQQ_FNS

static inline struct gfq_group *pd_to_gfqg(struct blkg_policy_data *pd)
{
	return pd ? container_of(pd, struct gfq_group, pd) : NULL;
}

static inline struct blkcg_gq *gfqg_to_blkg(struct gfq_group *gfqg)
{
	return pd_to_blkg(&gfqg->pd);
}

#if defined(CONFIG_GFQ_GROUP_IOSCHED) && defined(CONFIG_DEBUG_BLK_CGROUP)

/* gfqg stats flags */
enum gfqg_stats_flags {
	GFQG_stats_waiting = 0,
	GFQG_stats_idling,
	GFQG_stats_empty,
};

#define GFQG_FLAG_FNS(name)						\
static inline void gfqg_stats_mark_##name(struct gfqg_stats *stats)	\
{									\
	stats->flags |= (1 << GFQG_stats_##name);			\
}									\
static inline void gfqg_stats_clear_##name(struct gfqg_stats *stats)	\
{									\
	stats->flags &= ~(1 << GFQG_stats_##name);			\
}									\
static inline int gfqg_stats_##name(struct gfqg_stats *stats)		\
{									\
	return (stats->flags & (1 << GFQG_stats_##name)) != 0;		\
}									\

GFQG_FLAG_FNS(waiting)
GFQG_FLAG_FNS(idling)
GFQG_FLAG_FNS(empty)
#undef GFQG_FLAG_FNS

/* This should be called with the queue_lock held. */
static void gfqg_stats_update_group_wait_time(struct gfqg_stats *stats)
{
	unsigned long long now;

	if (!gfqg_stats_waiting(stats))
		return;

	now = sched_clock();
	if (time_after64(now, stats->start_group_wait_time))
		blkg_stat_add(&stats->group_wait_time,
			      now - stats->start_group_wait_time);
	gfqg_stats_clear_waiting(stats);
}

/* This should be called with the queue_lock held. */
static void gfqg_stats_set_start_group_wait_time(struct gfq_group *gfqg,
						 struct gfq_group *curr_gfqg)
{
	struct gfqg_stats *stats = &gfqg->stats;

	if (gfqg_stats_waiting(stats))
		return;
	if (gfqg == curr_gfqg)
		return;
	stats->start_group_wait_time = sched_clock();
	gfqg_stats_mark_waiting(stats);
}

/* This should be called with the queue_lock held. */
static void gfqg_stats_end_empty_time(struct gfqg_stats *stats)
{
	unsigned long long now;

	if (!gfqg_stats_empty(stats))
		return;

	now = sched_clock();
	if (time_after64(now, stats->start_empty_time))
		blkg_stat_add(&stats->empty_time,
			      now - stats->start_empty_time);
	gfqg_stats_clear_empty(stats);
}

static void gfqg_stats_update_dequeue(struct gfq_group *gfqg)
{
	blkg_stat_add(&gfqg->stats.dequeue, 1);
}

static void gfqg_stats_set_start_empty_time(struct gfq_group *gfqg)
{
	struct gfqg_stats *stats = &gfqg->stats;

	if (blkg_rwstat_total(&stats->queued))
		return;

	/*
	 * group is already marked empty. This can happen if gfqq got new
	 * request in parent group and moved to this group while being added
	 * to service tree. Just ignore the event and move on.
	 */
	if (gfqg_stats_empty(stats))
		return;

	stats->start_empty_time = sched_clock();
	gfqg_stats_mark_empty(stats);
}

static void gfqg_stats_update_idle_time(struct gfq_group *gfqg)
{
	struct gfqg_stats *stats = &gfqg->stats;

	if (gfqg_stats_idling(stats)) {
		unsigned long long now = sched_clock();

		if (time_after64(now, stats->start_idle_time))
			blkg_stat_add(&stats->idle_time,
				      now - stats->start_idle_time);
		gfqg_stats_clear_idling(stats);
	}
}

static void gfqg_stats_set_start_idle_time(struct gfq_group *gfqg)
{
	struct gfqg_stats *stats = &gfqg->stats;

	BUG_ON(gfqg_stats_idling(stats));

	stats->start_idle_time = sched_clock();
	gfqg_stats_mark_idling(stats);
}

static void gfqg_stats_update_avg_queue_size(struct gfq_group *gfqg)
{
	struct gfqg_stats *stats = &gfqg->stats;

	blkg_stat_add(&stats->avg_queue_size_sum,
		      blkg_rwstat_total(&stats->queued));
	blkg_stat_add(&stats->avg_queue_size_samples, 1);
	gfqg_stats_update_group_wait_time(stats);
}

#else	/* CONFIG_GFQ_GROUP_IOSCHED && CONFIG_DEBUG_BLK_CGROUP */

static inline void gfqg_stats_set_start_group_wait_time(struct gfq_group *gfqg, struct gfq_group *curr_gfqg) { }
static inline void gfqg_stats_end_empty_time(struct gfqg_stats *stats) { }
static inline void gfqg_stats_update_dequeue(struct gfq_group *gfqg) { }
static inline void gfqg_stats_set_start_empty_time(struct gfq_group *gfqg) { }
static inline void gfqg_stats_update_idle_time(struct gfq_group *gfqg) { }
static inline void gfqg_stats_set_start_idle_time(struct gfq_group *gfqg) { }
static inline void gfqg_stats_update_avg_queue_size(struct gfq_group *gfqg) { }

#endif	/* CONFIG_GFQ_GROUP_IOSCHED && CONFIG_DEBUG_BLK_CGROUP */

#ifdef CONFIG_GFQ_GROUP_IOSCHED

static struct blkcg_policy blkcg_policy_gfq;

static inline struct gfq_group *blkg_to_gfqg(struct blkcg_gq *blkg)
{
	return pd_to_gfqg(blkg_to_pd(blkg, &blkcg_policy_gfq));
}

static inline struct gfq_group *gfqg_parent(struct gfq_group *gfqg)
{
	struct blkcg_gq *pblkg = gfqg_to_blkg(gfqg)->parent;

	return pblkg ? blkg_to_gfqg(pblkg) : NULL;
}

static inline void gfqg_get(struct gfq_group *gfqg)
{
	return blkg_get(gfqg_to_blkg(gfqg));
}

static inline void gfqg_put(struct gfq_group *gfqg)
{
	return blkg_put(gfqg_to_blkg(gfqg));
}

#define gfq_log_gfqq(gfqd, gfqq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(gfqg_to_blkg((gfqq)->gfqg), __pbuf, sizeof(__pbuf));	\
	blk_add_trace_msg((gfqd)->queue, "gfq%d%c%c %s " fmt, (gfqq)->pid, \
			gfq_gfqq_sync((gfqq)) ? 'S' : 'A',		\
			gfqq_type((gfqq)) == SYNC_NOIDLE_WORKLOAD ? 'N' : ' ',\
			  __pbuf, ##args);				\
} while (0)

#define gfq_log_gfqg(gfqd, gfqg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(gfqg_to_blkg(gfqg), __pbuf, sizeof(__pbuf));		\
	blk_add_trace_msg((gfqd)->queue, "%s " fmt, __pbuf, ##args);	\
} while (0)

static inline void gfqg_stats_update_io_add(struct gfq_group *gfqg,
					    struct gfq_group *curr_gfqg, int rw)
{
	blkg_rwstat_add(&gfqg->stats.queued, rw, 1);
	gfqg_stats_end_empty_time(&gfqg->stats);
	gfqg_stats_set_start_group_wait_time(gfqg, curr_gfqg);
}

static inline void gfqg_stats_update_timeslice_used(struct gfq_group *gfqg,
			unsigned long time, unsigned long unaccounted_time)
{
	blkg_stat_add(&gfqg->stats.time, time);
#ifdef CONFIG_DEBUG_BLK_CGROUP
	blkg_stat_add(&gfqg->stats.unaccounted_time, unaccounted_time);
#endif
}

static inline void gfqg_stats_update_io_remove(struct gfq_group *gfqg, int rw)
{
	blkg_rwstat_add(&gfqg->stats.queued, rw, -1);
}

static inline void gfqg_stats_update_io_merged(struct gfq_group *gfqg, int rw)
{
	blkg_rwstat_add(&gfqg->stats.merged, rw, 1);
}

static inline void gfqg_stats_update_dispatch(struct gfq_group *gfqg,
					      uint64_t bytes, int rw)
{
	blkg_stat_add(&gfqg->stats.sectors, bytes >> 9);
	blkg_rwstat_add(&gfqg->stats.serviced, rw, 1);
	blkg_rwstat_add(&gfqg->stats.service_bytes, rw, bytes);
}

static inline void gfqg_stats_update_completion(struct gfq_group *gfqg,
			uint64_t start_time, uint64_t io_start_time, int rw)
{
	struct gfqg_stats *stats = &gfqg->stats;
	unsigned long long now = sched_clock();

	if (time_after64(now, io_start_time))
		blkg_rwstat_add(&stats->service_time, rw, now - io_start_time);
	if (time_after64(io_start_time, start_time))
		blkg_rwstat_add(&stats->wait_time, rw,
				io_start_time - start_time);
}

/* @stats = 0 */
static void gfqg_stats_reset(struct gfqg_stats *stats)
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
static void gfqg_stats_merge(struct gfqg_stats *to, struct gfqg_stats *from)
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
 * Transfer @gfqg's stats to its parent's dead_stats so that the ancestors'
 * recursive stats can still account for the amount used by this gfqg after
 * it's gone.
 */
static void gfqg_stats_xfer_dead(struct gfq_group *gfqg)
{
	struct gfq_group *parent = gfqg_parent(gfqg);

	lockdep_assert_held(gfqg_to_blkg(gfqg)->q->queue_lock);

	if (unlikely(!parent))
		return;

	gfqg_stats_merge(&parent->dead_stats, &gfqg->stats);
	gfqg_stats_merge(&parent->dead_stats, &gfqg->dead_stats);
	gfqg_stats_reset(&gfqg->stats);
	gfqg_stats_reset(&gfqg->dead_stats);
}

#else	/* CONFIG_GFQ_GROUP_IOSCHED */

static inline struct gfq_group *gfqg_parent(struct gfq_group *gfqg) { return NULL; }
static inline void gfqg_get(struct gfq_group *gfqg) { }
static inline void gfqg_put(struct gfq_group *gfqg) { }

#define gfq_log_gfqq(gfqd, gfqq, fmt, args...)	\
	blk_add_trace_msg((gfqd)->queue, "gfq%d%c%c " fmt, (gfqq)->pid,	\
			gfq_gfqq_sync((gfqq)) ? 'S' : 'A',		\
			gfqq_type((gfqq)) == SYNC_NOIDLE_WORKLOAD ? 'N' : ' ',\
				##args)
#define gfq_log_gfqg(gfqd, gfqg, fmt, args...)		do {} while (0)

static inline void gfqg_stats_update_io_add(struct gfq_group *gfqg,
			struct gfq_group *curr_gfqg, int rw) { }
static inline void gfqg_stats_update_timeslice_used(struct gfq_group *gfqg,
			unsigned long time, unsigned long unaccounted_time) { }
static inline void gfqg_stats_update_io_remove(struct gfq_group *gfqg, int rw) { }
static inline void gfqg_stats_update_io_merged(struct gfq_group *gfqg, int rw) { }
static inline void gfqg_stats_update_dispatch(struct gfq_group *gfqg,
					      uint64_t bytes, int rw) { }
static inline void gfqg_stats_update_completion(struct gfq_group *gfqg,
			uint64_t start_time, uint64_t io_start_time, int rw) { }

#endif	/* CONFIG_GFQ_GROUP_IOSCHED */

#define gfq_log(gfqd, fmt, args...)	\
	blk_add_trace_msg((gfqd)->queue, "gfq " fmt, ##args)

/* Traverses through gfq group service trees */
#define for_each_gfqg_st(gfqg, i, j, st) \
	for (i = 0; i <= IDLE_WORKLOAD; i++) \
		for (j = 0, st = i < IDLE_WORKLOAD ? &gfqg->service_trees[i][j]\
			: &gfqg->service_tree_idle; \
			(i < IDLE_WORKLOAD && j <= SYNC_WORKLOAD) || \
			(i == IDLE_WORKLOAD && j == 0); \
			j++, st = i < IDLE_WORKLOAD ? \
			&gfqg->service_trees[i][j]: NULL) \

static inline bool gfq_io_thinktime_big(struct gfq_data *gfqd,
	struct gfq_ttime *ttime, bool group_idle)
{
	unsigned long slice;
	if (!sample_valid(ttime->ttime_samples))
		return false;
	if (group_idle)
		slice = gfqd->gfq_group_idle;
	else
		slice = gfqd->gfq_slice_idle;
	return ttime->ttime_mean > slice;
}

static inline enum wl_class_t gfqq_class(struct gfq_queue *gfqq)
{
	if (gfq_class_idle(gfqq))
		return IDLE_WORKLOAD;
	if (gfq_class_rt(gfqq))
		return RT_WORKLOAD;
	return BE_WORKLOAD;
}


static enum wl_type_t gfqq_type(struct gfq_queue *gfqq)
{
	if (!gfq_gfqq_sync(gfqq))
		return ASYNC_WORKLOAD;
	if (!gfq_gfqq_idle_window(gfqq))
		return SYNC_NOIDLE_WORKLOAD;
	return SYNC_WORKLOAD;
}

static inline int gfq_group_busy_queues_wl(enum wl_class_t wl_class,
					struct gfq_data *gfqd,
					struct gfq_group *gfqg)
{
	if (wl_class == IDLE_WORKLOAD)
		return gfqg->service_tree_idle.count;

	return gfqg->service_trees[wl_class][ASYNC_WORKLOAD].count +
		gfqg->service_trees[wl_class][SYNC_NOIDLE_WORKLOAD].count +
		gfqg->service_trees[wl_class][SYNC_WORKLOAD].count;
}

static inline int gfqg_busy_async_queues(struct gfq_data *gfqd,
					struct gfq_group *gfqg)
{
	return gfqg->service_trees[RT_WORKLOAD][ASYNC_WORKLOAD].count +
		gfqg->service_trees[BE_WORKLOAD][ASYNC_WORKLOAD].count;
}

static void gfq_dispatch_insert(struct request_queue *, struct request *);
static struct gfq_queue *gfq_get_queue(struct gfq_data *gfqd, bool is_sync,
				       struct gfq_io_cq *cic, struct bio *bio,
				       gfp_t gfp_mask);

static inline struct gfq_io_cq *icq_to_cic(struct io_cq *icq)
{
	/* cic->icq is the first member, %NULL will convert to %NULL */
	return container_of(icq, struct gfq_io_cq, icq);
}

static inline struct gfq_io_cq *gfq_cic_lookup(struct gfq_data *gfqd,
					       struct io_context *ioc)
{
	if (ioc)
		return icq_to_cic(ioc_lookup_icq(ioc, gfqd->queue));
	return NULL;
}

static inline struct gfq_queue *cic_to_gfqq(struct gfq_io_cq *cic, bool is_sync)
{
	return cic->gfqq[is_sync];
}

static inline void cic_set_gfqq(struct gfq_io_cq *cic, struct gfq_queue *gfqq,
				bool is_sync)
{
	cic->gfqq[is_sync] = gfqq;
}

static inline struct gfq_data *cic_to_gfqd(struct gfq_io_cq *cic)
{
	return cic->icq.q->elevator->elevator_data;
}

/*
 * We regard a request as SYNC, if it's either a read or has the SYNC bit
 * set (in which case it could also be direct WRITE).
 */
static inline bool gfq_bio_sync(struct bio *bio)
{
	return bio_data_dir(bio) == READ || (bio->bi_rw & REQ_SYNC);
}

/*
 * scheduler run of queue, if there are requests pending and no one in the
 * driver that will restart queueing
 */
static inline void gfq_schedule_dispatch(struct gfq_data *gfqd)
{
	if (gfqd->busy_queues) {
		gfq_log(gfqd, "schedule dispatch");
		kblockd_schedule_work(&gfqd->unplug_work);
	}
}

/*
 * Scale schedule slice based on io priority. Use the sync time slice only
 * if a queue is marked sync and has sync io queued. A sync queue with async
 * io only, should not get full sync slice length.
 */
static inline int gfq_prio_slice(struct gfq_data *gfqd, bool sync,
				 unsigned short prio)
{
	const int base_slice = gfqd->gfq_slice[sync];

	WARN_ON(prio >= IOPRIO_BE_NR);

	return base_slice + (base_slice/GFQ_SLICE_SCALE * (4 - prio));
}

static inline int
gfq_prio_to_slice(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	return gfq_prio_slice(gfqd, gfq_gfqq_sync(gfqq), gfqq->ioprio);
}

/**
 * gfqg_scale_charge - scale disk time charge according to gfqg weight
 * @charge: disk time being charged
 * @vfraction: vfraction of the gfqg, fixed point w/ GFQ_SERVICE_SHIFT
 *
 * Scale @charge according to @vfraction, which is in range (0, 1].  The
 * scaling is inversely proportional.
 *
 * scaled = charge / vfraction
 *
 * The result is also in fixed point w/ GFQ_SERVICE_SHIFT.
 */
static inline u64 gfqg_scale_charge(unsigned long charge,
				    unsigned int vfraction)
{
	u64 c = charge << GFQ_SERVICE_SHIFT;	/* make it fixed point */

	/* charge / vfraction */
	c <<= GFQ_SERVICE_SHIFT;
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

static void update_min_vdisktime(struct gfq_rb_root *st)
{
	struct gfq_group *gfqg;

	if (st->left) {
		gfqg = rb_entry_gfqg(st->left);
		st->min_vdisktime = max_vdisktime(st->min_vdisktime,
						  gfqg->vdisktime);
	}
}

/*
 * get averaged number of queues of RT/BE priority.
 * average is updated, with a formula that gives more weight to higher numbers,
 * to quickly follows sudden increases and decrease slowly
 */

static inline unsigned gfq_group_get_avg_queues(struct gfq_data *gfqd,
					struct gfq_group *gfqg, bool rt)
{
	unsigned min_q, max_q;
	unsigned mult  = gfq_hist_divisor - 1;
	unsigned round = gfq_hist_divisor / 2;
	unsigned busy = gfq_group_busy_queues_wl(rt, gfqd, gfqg);

	min_q = min(gfqg->busy_queues_avg[rt], busy);
	max_q = max(gfqg->busy_queues_avg[rt], busy);
	gfqg->busy_queues_avg[rt] = (mult * max_q + min_q + round) /
		gfq_hist_divisor;
	return gfqg->busy_queues_avg[rt];
}

static inline unsigned
gfq_group_slice(struct gfq_data *gfqd, struct gfq_group *gfqg)
{
	return gfqd->gfq_target_latency * gfqg->vfraction >> GFQ_SERVICE_SHIFT;
}

static inline unsigned
gfq_scaled_gfqq_slice(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	unsigned slice = gfq_prio_to_slice(gfqd, gfqq);
	if (gfqd->gfq_latency) {
		/*
		 * interested queues (we consider only the ones with the same
		 * priority class in the gfq group)
		 */
		unsigned iq = gfq_group_get_avg_queues(gfqd, gfqq->gfqg,
						gfq_class_rt(gfqq));
		unsigned sync_slice = gfqd->gfq_slice[1];
		unsigned expect_latency = sync_slice * iq;
		unsigned group_slice = gfq_group_slice(gfqd, gfqq->gfqg);

		if (expect_latency > group_slice) {
			unsigned base_low_slice = 2 * gfqd->gfq_slice_idle;
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
gfq_set_prio_slice(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	unsigned slice = gfq_scaled_gfqq_slice(gfqd, gfqq);

	gfqq->slice_start = jiffies;
	gfqq->slice_end = jiffies + slice;
	gfqq->allocated_slice = slice;
	gfq_log_gfqq(gfqd, gfqq, "set_slice=%lu", gfqq->slice_end - jiffies);
}

/*
 * We need to wrap this check in gfq_gfqq_slice_new(), since ->slice_end
 * isn't valid until the first request from the dispatch is activated
 * and the slice time set.
 */
static inline bool gfq_slice_used(struct gfq_queue *gfqq)
{
	if (gfq_gfqq_slice_new(gfqq))
		return false;
	if (time_before(jiffies, gfqq->slice_end))
		return false;

	return true;
}

/*
 * Lifted from AS - choose which of rq1 and rq2 that is best served now.
 * We choose the request that is closest to the head right now. Distance
 * behind the head is penalized and only allowed to a certain extent.
 */
static struct request *
gfq_choose_req(struct gfq_data *gfqd, struct request *rq1, struct request *rq2, sector_t last)
{
	sector_t s1, s2, d1 = 0, d2 = 0;
	unsigned long back_max;
#define GFQ_RQ1_WRAP	0x01 /* request 1 wraps */
#define GFQ_RQ2_WRAP	0x02 /* request 2 wraps */
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
	back_max = gfqd->gfq_back_max * 2;

	/*
	 * Strict one way elevator _except_ in the case where we allow
	 * short backward seeks which are biased as twice the cost of a
	 * similar forward seek.
	 */
	if (s1 >= last)
		d1 = s1 - last;
	else if (s1 + back_max >= last)
		d1 = (last - s1) * gfqd->gfq_back_penalty;
	else
		wrap |= GFQ_RQ1_WRAP;

	if (s2 >= last)
		d2 = s2 - last;
	else if (s2 + back_max >= last)
		d2 = (last - s2) * gfqd->gfq_back_penalty;
	else
		wrap |= GFQ_RQ2_WRAP;

	/* Found required data */

	/*
	 * By doing switch() on the bit mask "wrap" we avoid having to
	 * check two variables for all permutations: --> faster!
	 */
	switch (wrap) {
	case 0: /* common case for GFQ: rq1 and rq2 not wrapped */
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

	case GFQ_RQ2_WRAP:
		return rq1;
	case GFQ_RQ1_WRAP:
		return rq2;
	case (GFQ_RQ1_WRAP|GFQ_RQ2_WRAP): /* both rqs wrapped */
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
static struct gfq_queue *gfq_rb_first(struct gfq_rb_root *root)
{
	/* Service tree is empty */
	if (!root->count)
		return NULL;

	if (!root->left)
		root->left = rb_first(&root->rb);

	if (root->left)
		return rb_entry(root->left, struct gfq_queue, rb_node);

	return NULL;
}

static struct gfq_group *gfq_rb_first_group(struct gfq_rb_root *root)
{
	if (!root->left)
		root->left = rb_first(&root->rb);

	if (root->left)
		return rb_entry_gfqg(root->left);

	return NULL;
}

static void rb_erase_init(struct rb_node *n, struct rb_root *root)
{
	rb_erase(n, root);
	RB_CLEAR_NODE(n);
}

static void gfq_rb_erase(struct rb_node *n, struct gfq_rb_root *root)
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
gfq_find_next_rq(struct gfq_data *gfqd, struct gfq_queue *gfqq,
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
		rbnext = rb_first(&gfqq->sort_list);
		if (rbnext && rbnext != &last->rb_node)
			next = rb_entry_rq(rbnext);
	}

	return gfq_choose_req(gfqd, next, prev, blk_rq_pos(last));
}

static unsigned long gfq_slice_offset(struct gfq_data *gfqd,
				      struct gfq_queue *gfqq)
{
	/*
	 * just an approximation, should be ok.
	 */
	return (gfqq->gfqg->nr_gfqq - 1) * (gfq_prio_slice(gfqd, 1, 0) -
		       gfq_prio_slice(gfqd, gfq_gfqq_sync(gfqq), gfqq->ioprio));
}

static inline s64
gfqg_key(struct gfq_rb_root *st, struct gfq_group *gfqg)
{
	return gfqg->vdisktime - st->min_vdisktime;
}

static void
__gfq_group_service_tree_add(struct gfq_rb_root *st, struct gfq_group *gfqg)
{
	struct rb_node **node = &st->rb.rb_node;
	struct rb_node *parent = NULL;
	struct gfq_group *__gfqg;
	s64 key = gfqg_key(st, gfqg);
	int left = 1;

	while (*node != NULL) {
		parent = *node;
		__gfqg = rb_entry_gfqg(parent);

		if (key < gfqg_key(st, __gfqg))
			node = &parent->rb_left;
		else {
			node = &parent->rb_right;
			left = 0;
		}
	}

	if (left)
		st->left = &gfqg->rb_node;

	rb_link_node(&gfqg->rb_node, parent, node);
	rb_insert_color(&gfqg->rb_node, &st->rb);
}

/*
 * This has to be called only on activation of gfqg
 */
static void
gfq_update_group_weight(struct gfq_group *gfqg)
{
	if (gfqg->new_weight) {
		gfqg->weight = gfqg->new_weight;
		gfqg->new_weight = 0;
	}
}

static void
gfq_update_group_leaf_weight(struct gfq_group *gfqg)
{
	BUG_ON(!RB_EMPTY_NODE(&gfqg->rb_node));

	if (gfqg->new_leaf_weight) {
		gfqg->leaf_weight = gfqg->new_leaf_weight;
		gfqg->new_leaf_weight = 0;
	}
}

static void
gfq_group_service_tree_add(struct gfq_rb_root *st, struct gfq_group *gfqg)
{
	unsigned int vfr = 1 << GFQ_SERVICE_SHIFT;	/* start with 1 */
	struct gfq_group *pos = gfqg;
	struct gfq_group *parent;
	bool propagate;

	/* add to the service tree */
	BUG_ON(!RB_EMPTY_NODE(&gfqg->rb_node));

	/*
	 * Update leaf_weight.  We cannot update weight at this point
	 * because gfqg might already have been activated and is
	 * contributing its current weight to the parent's child_weight.
	 */
	gfq_update_group_leaf_weight(gfqg);
	__gfq_group_service_tree_add(st, gfqg);

	/*
	 * Activate @gfqg and calculate the portion of vfraction @gfqg is
	 * entitled to.  vfraction is calculated by walking the tree
	 * towards the root calculating the fraction it has at each level.
	 * The compounded ratio is how much vfraction @gfqg owns.
	 *
	 * Start with the proportion tasks in this gfqg has against active
	 * children gfqgs - its leaf_weight against children_weight.
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
	while ((parent = gfqg_parent(pos))) {
		if (propagate) {
			gfq_update_group_weight(pos);
			propagate = !parent->nr_active++;
			parent->children_weight += pos->weight;
		}
		vfr = vfr * pos->weight / parent->children_weight;
		pos = parent;
	}

	gfqg->vfraction = max_t(unsigned, vfr, 1);
}

static void
gfq_group_notify_queue_add(struct gfq_data *gfqd, struct gfq_group *gfqg)
{
	struct gfq_rb_root *st = &gfqd->grp_service_tree;
	struct gfq_group *__gfqg;
	struct rb_node *n;

	gfqg->nr_gfqq++;
	if (!RB_EMPTY_NODE(&gfqg->rb_node))
		return;

	/*
	 * Currently put the group at the end. Later implement something
	 * so that groups get lesser vtime based on their weights, so that
	 * if group does not loose all if it was not continuously backlogged.
	 */
	n = rb_last(&st->rb);
	if (n) {
		__gfqg = rb_entry_gfqg(n);
		gfqg->vdisktime = __gfqg->vdisktime + GFQ_IDLE_DELAY;
	} else
		gfqg->vdisktime = st->min_vdisktime;
	gfq_group_service_tree_add(st, gfqg);
}

static void
gfq_group_service_tree_del(struct gfq_rb_root *st, struct gfq_group *gfqg)
{
	struct gfq_group *pos = gfqg;
	bool propagate;

	/*
	 * Undo activation from gfq_group_service_tree_add().  Deactivate
	 * @gfqg and propagate deactivation upwards.
	 */
	propagate = !--pos->nr_active;
	pos->children_weight -= pos->leaf_weight;

	while (propagate) {
		struct gfq_group *parent = gfqg_parent(pos);

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
	if (!RB_EMPTY_NODE(&gfqg->rb_node))
		gfq_rb_erase(&gfqg->rb_node, st);
}

static void
gfq_group_notify_queue_del(struct gfq_data *gfqd, struct gfq_group *gfqg)
{
	struct gfq_rb_root *st = &gfqd->grp_service_tree;

	BUG_ON(gfqg->nr_gfqq < 1);
	gfqg->nr_gfqq--;

	/* If there are other gfq queues under this group, don't delete it */
	if (gfqg->nr_gfqq)
		return;

	gfq_log_gfqg(gfqd, gfqg, "del_from_rr group");
	gfq_group_service_tree_del(st, gfqg);
	gfqg->saved_wl_slice = 0;
	gfqg_stats_update_dequeue(gfqg);
}

static inline unsigned int gfq_gfqq_slice_usage(struct gfq_queue *gfqq,
						unsigned int *unaccounted_time)
{
	unsigned int slice_used;

	/*
	 * Queue got expired before even a single request completed or
	 * got expired immediately after first request completion.
	 */
	if (!gfqq->slice_start || gfqq->slice_start == jiffies) {
		/*
		 * Also charge the seek time incurred to the group, otherwise
		 * if there are mutiple queues in the group, each can dispatch
		 * a single request on seeky media and cause lots of seek time
		 * and group will never know it.
		 */
		slice_used = max_t(unsigned, (jiffies - gfqq->dispatch_start),
					1);
	} else {
		slice_used = jiffies - gfqq->slice_start;
		if (slice_used > gfqq->allocated_slice) {
			*unaccounted_time = slice_used - gfqq->allocated_slice;
			slice_used = gfqq->allocated_slice;
		}
		if (time_after(gfqq->slice_start, gfqq->dispatch_start))
			*unaccounted_time += gfqq->slice_start -
					gfqq->dispatch_start;
	}

	return slice_used;
}

static void gfq_group_served(struct gfq_data *gfqd, struct gfq_group *gfqg,
				struct gfq_queue *gfqq)
{
	struct gfq_rb_root *st = &gfqd->grp_service_tree;
	unsigned int used_sl, charge, unaccounted_sl = 0;
	int nr_sync = gfqg->nr_gfqq - gfqg_busy_async_queues(gfqd, gfqg)
			- gfqg->service_tree_idle.count;
	unsigned int vfr;

	BUG_ON(nr_sync < 0);
	used_sl = charge = gfq_gfqq_slice_usage(gfqq, &unaccounted_sl);

	charge = gfqq->slice_dispatch; // change to iops mode.

	/*
	 * Can't update vdisktime while on service tree and gfqg->vfraction
	 * is valid only while on it.  Cache vfr, leave the service tree,
	 * update vdisktime and go back on.  The re-addition to the tree
	 * will also update the weights as necessary.
	 */
	vfr = gfqg->vfraction;
	gfq_group_service_tree_del(st, gfqg);
	gfqg->vdisktime += gfqg_scale_charge(charge, vfr);
	gfq_group_service_tree_add(st, gfqg);

	/* This group is being expired. Save the context */
	if (time_after(gfqd->workload_expires, jiffies)) {
		gfqg->saved_wl_slice = gfqd->workload_expires
						- jiffies;
		gfqg->saved_wl_type = gfqd->serving_wl_type;
		gfqg->saved_wl_class = gfqd->serving_wl_class;
	} else
		gfqg->saved_wl_slice = 0;

	gfq_log_gfqg(gfqd, gfqg, "served: vt=%llu min_vt=%llu", gfqg->vdisktime,
					st->min_vdisktime);
	gfq_log_gfqq(gfqq->gfqd, gfqq,
		     "sl_used=%u disp=%u charge=%u sect=%lu",
		     used_sl, gfqq->slice_dispatch, charge, gfqq->nr_sectors);
	gfqg_stats_update_timeslice_used(gfqg, used_sl, unaccounted_sl);
	gfqg_stats_set_start_empty_time(gfqg);
}

/**
 * gfq_init_gfqg_base - initialize base part of a gfq_group
 * @gfqg: gfq_group to initialize
 *
 * Initialize the base part which is used whether %CONFIG_GFQ_GROUP_IOSCHED
 * is enabled or not.
 */
static void gfq_init_gfqg_base(struct gfq_group *gfqg)
{
	struct gfq_rb_root *st;
	int i, j;

	for_each_gfqg_st(gfqg, i, j, st)
		*st = GFQ_RB_ROOT;
	RB_CLEAR_NODE(&gfqg->rb_node);

	gfqg->ttime.last_end_request = jiffies;
}

#ifdef CONFIG_GFQ_GROUP_IOSCHED
static void gfqg_stats_init(struct gfqg_stats *stats)
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

static void gfq_pd_init(struct blkcg_gq *blkg)
{
	struct gfq_group *gfqg = blkg_to_gfqg(blkg);

	gfq_init_gfqg_base(gfqg);
	gfqg->weight = blkg->blkcg->gfq_weight;
	gfqg->leaf_weight = blkg->blkcg->gfq_leaf_weight;
	gfqg_stats_init(&gfqg->stats);
	gfqg_stats_init(&gfqg->dead_stats);
}

static void gfq_pd_offline(struct blkcg_gq *blkg)
{
	/*
	 * @blkg is going offline and will be ignored by
	 * blkg_[rw]stat_recursive_sum().  Transfer stats to the parent so
	 * that they don't get lost.  If IOs complete after this point, the
	 * stats for them will be lost.  Oh well...
	 */
	gfqg_stats_xfer_dead(blkg_to_gfqg(blkg));
}

/* offset delta from gfqg->stats to gfqg->dead_stats */
static const int dead_stats_off_delta = offsetof(struct gfq_group, dead_stats) -
					offsetof(struct gfq_group, stats);

/* to be used by recursive prfill, sums live and dead stats recursively */
static u64 gfqg_stat_pd_recursive_sum(struct blkg_policy_data *pd, int off)
{
	u64 sum = 0;

	sum += blkg_stat_recursive_sum(pd, off);
	sum += blkg_stat_recursive_sum(pd, off + dead_stats_off_delta);
	return sum;
}

/* to be used by recursive prfill, sums live and dead rwstats recursively */
static struct blkg_rwstat gfqg_rwstat_pd_recursive_sum(struct blkg_policy_data *pd,
						       int off)
{
	struct blkg_rwstat a, b;

	a = blkg_rwstat_recursive_sum(pd, off);
	b = blkg_rwstat_recursive_sum(pd, off + dead_stats_off_delta);
	blkg_rwstat_merge(&a, &b);
	return a;
}

static void gfq_pd_reset_stats(struct blkcg_gq *blkg)
{
	struct gfq_group *gfqg = blkg_to_gfqg(blkg);

	gfqg_stats_reset(&gfqg->stats);
	gfqg_stats_reset(&gfqg->dead_stats);
}

/*
 * Search for the gfq group current task belongs to. request_queue lock must
 * be held.
 */
static struct gfq_group *gfq_lookup_create_gfqg(struct gfq_data *gfqd,
						struct blkcg *blkcg)
{
	struct request_queue *q = gfqd->queue;
	struct gfq_group *gfqg = NULL;

	/* avoid lookup for the common case where there's no blkcg */
	if (blkcg == &blkcg_root) {
		gfqg = gfqd->root_group;
	} else {
		struct blkcg_gq *blkg;

		blkg = blkg_lookup_create(blkcg, q);
		if (!IS_ERR(blkg))
			gfqg = blkg_to_gfqg(blkg);
	}

	return gfqg;
}

static void gfq_link_gfqq_gfqg(struct gfq_queue *gfqq, struct gfq_group *gfqg)
{
	/* Currently, all async queues are mapped to root group */
	if (!gfq_gfqq_sync(gfqq))
		gfqg = gfqq->gfqd->root_group;

	gfqq->gfqg = gfqg;
	/* gfqq reference on gfqg */
	gfqg_get(gfqg);
}

static u64 gfqg_prfill_weight_device(struct seq_file *sf,
				     struct blkg_policy_data *pd, int off)
{
	struct gfq_group *gfqg = pd_to_gfqg(pd);

	if (!gfqg->dev_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, gfqg->dev_weight);
}

static int gfqg_print_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  gfqg_prfill_weight_device, &blkcg_policy_gfq,
			  0, false);
	return 0;
}

static u64 gfqg_prfill_leaf_weight_device(struct seq_file *sf,
					  struct blkg_policy_data *pd, int off)
{
	struct gfq_group *gfqg = pd_to_gfqg(pd);

	if (!gfqg->dev_leaf_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, gfqg->dev_leaf_weight);
}

static int gfqg_print_leaf_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  gfqg_prfill_leaf_weight_device, &blkcg_policy_gfq,
			  0, false);
	return 0;
}

static int gfq_print_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->gfq_weight);
	return 0;
}

static int gfq_print_leaf_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->gfq_leaf_weight);
	return 0;
}

static ssize_t __gfqg_set_weight_device(struct kernfs_open_file *of,
					char *buf, size_t nbytes, loff_t off,
					bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(of_css(of));
	struct blkg_conf_ctx ctx;
	struct gfq_group *gfqg;
	int ret;

	ret = blkg_conf_prep(blkcg, &blkcg_policy_gfq, buf, &ctx);
	if (ret)
		return ret;

	ret = -EINVAL;
	gfqg = blkg_to_gfqg(ctx.blkg);
	if (!ctx.v || (ctx.v >= GFQ_WEIGHT_MIN && ctx.v <= GFQ_WEIGHT_MAX)) {
		if (!is_leaf_weight) {
			gfqg->dev_weight = ctx.v;
			gfqg->new_weight = ctx.v ?: blkcg->gfq_weight;
		} else {
			gfqg->dev_leaf_weight = ctx.v;
			gfqg->new_leaf_weight = ctx.v ?: blkcg->gfq_leaf_weight;
		}
		ret = 0;
	}

	blkg_conf_finish(&ctx);
	return ret ?: nbytes;
}

static ssize_t gfqg_set_weight_device(struct kernfs_open_file *of,
				      char *buf, size_t nbytes, loff_t off)
{
	return __gfqg_set_weight_device(of, buf, nbytes, off, false);
}

static ssize_t gfqg_set_leaf_weight_device(struct kernfs_open_file *of,
					   char *buf, size_t nbytes, loff_t off)
{
	return __gfqg_set_weight_device(of, buf, nbytes, off, true);
}

static int __gfq_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
			    u64 val, bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(css);
	struct blkcg_gq *blkg;

	if (val < GFQ_WEIGHT_MIN || val > GFQ_WEIGHT_MAX)
		return -EINVAL;

	spin_lock_irq(&blkcg->lock);

	if (!is_leaf_weight)
		blkcg->gfq_weight = val;
	else
		blkcg->gfq_leaf_weight = val;

	hlist_for_each_entry(blkg, &blkcg->blkg_list, blkcg_node) {
		struct gfq_group *gfqg = blkg_to_gfqg(blkg);

		if (!gfqg)
			continue;

		if (!is_leaf_weight) {
			if (!gfqg->dev_weight)
				gfqg->new_weight = blkcg->gfq_weight;
		} else {
			if (!gfqg->dev_leaf_weight)
				gfqg->new_leaf_weight = blkcg->gfq_leaf_weight;
		}
	}

	spin_unlock_irq(&blkcg->lock);
	return 0;
}

static int gfq_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
			  u64 val)
{
	return __gfq_set_weight(css, cft, val, false);
}

static int gfq_set_leaf_weight(struct cgroup_subsys_state *css,
			       struct cftype *cft, u64 val)
{
	return __gfq_set_weight(css, cft, val, true);
}

static int gfqg_print_stat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_stat,
			  &blkcg_policy_gfq, seq_cft(sf)->private, false);
	return 0;
}

static int gfqg_print_rwstat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_rwstat,
			  &blkcg_policy_gfq, seq_cft(sf)->private, true);
	return 0;
}

static u64 gfqg_prfill_stat_recursive(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	u64 sum = gfqg_stat_pd_recursive_sum(pd, off);

	return __blkg_prfill_u64(sf, pd, sum);
}

static u64 gfqg_prfill_rwstat_recursive(struct seq_file *sf,
					struct blkg_policy_data *pd, int off)
{
	struct blkg_rwstat sum = gfqg_rwstat_pd_recursive_sum(pd, off);

	return __blkg_prfill_rwstat(sf, pd, &sum);
}

static int gfqg_print_stat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  gfqg_prfill_stat_recursive, &blkcg_policy_gfq,
			  seq_cft(sf)->private, false);
	return 0;
}

static int gfqg_print_rwstat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  gfqg_prfill_rwstat_recursive, &blkcg_policy_gfq,
			  seq_cft(sf)->private, true);
	return 0;
}

#ifdef CONFIG_DEBUG_BLK_CGROUP
static u64 gfqg_prfill_avg_queue_size(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	struct gfq_group *gfqg = pd_to_gfqg(pd);
	u64 samples = blkg_stat_read(&gfqg->stats.avg_queue_size_samples);
	u64 v = 0;

	if (samples) {
		v = blkg_stat_read(&gfqg->stats.avg_queue_size_sum);
		v = div64_u64(v, samples);
	}
	__blkg_prfill_u64(sf, pd, v);
	return 0;
}

/* print avg_queue_size */
static int gfqg_print_avg_queue_size(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  gfqg_prfill_avg_queue_size, &blkcg_policy_gfq,
			  0, false);
	return 0;
}
#endif	/* CONFIG_DEBUG_BLK_CGROUP */

static struct cftype gfq_blkcg_files[] = {
	/* on root, weight is mapped to leaf_weight */
	{
		.name = "weight_device",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = gfqg_print_leaf_weight_device,
		.write = gfqg_set_leaf_weight_device,
	},
	{
		.name = "weight",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = gfq_print_leaf_weight,
		.write_u64 = gfq_set_leaf_weight,
	},

	/* no such mapping necessary for !roots */
	{
		.name = "weight_device",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = gfqg_print_weight_device,
		.write = gfqg_set_weight_device,
	},
	{
		.name = "weight",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = gfq_print_weight,
		.write_u64 = gfq_set_weight,
	},

	{
		.name = "leaf_weight_device",
		.seq_show = gfqg_print_leaf_weight_device,
		.write = gfqg_set_leaf_weight_device,
	},
	{
		.name = "leaf_weight",
		.seq_show = gfq_print_leaf_weight,
		.write_u64 = gfq_set_leaf_weight,
	},

	/* statistics, covers only the tasks in the gfqg */
	{
		.name = "time",
		.private = offsetof(struct gfq_group, stats.time),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "sectors",
		.private = offsetof(struct gfq_group, stats.sectors),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "io_service_bytes",
		.private = offsetof(struct gfq_group, stats.service_bytes),
		.seq_show = gfqg_print_rwstat,
	},
	{
		.name = "io_serviced",
		.private = offsetof(struct gfq_group, stats.serviced),
		.seq_show = gfqg_print_rwstat,
	},
	{
		.name = "io_service_time",
		.private = offsetof(struct gfq_group, stats.service_time),
		.seq_show = gfqg_print_rwstat,
	},
	{
		.name = "io_wait_time",
		.private = offsetof(struct gfq_group, stats.wait_time),
		.seq_show = gfqg_print_rwstat,
	},
	{
		.name = "io_merged",
		.private = offsetof(struct gfq_group, stats.merged),
		.seq_show = gfqg_print_rwstat,
	},
	{
		.name = "io_queued",
		.private = offsetof(struct gfq_group, stats.queued),
		.seq_show = gfqg_print_rwstat,
	},

	/* the same statictics which cover the gfqg and its descendants */
	{
		.name = "time_recursive",
		.private = offsetof(struct gfq_group, stats.time),
		.seq_show = gfqg_print_stat_recursive,
	},
	{
		.name = "sectors_recursive",
		.private = offsetof(struct gfq_group, stats.sectors),
		.seq_show = gfqg_print_stat_recursive,
	},
	{
		.name = "io_service_bytes_recursive",
		.private = offsetof(struct gfq_group, stats.service_bytes),
		.seq_show = gfqg_print_rwstat_recursive,
	},
	{
		.name = "io_serviced_recursive",
		.private = offsetof(struct gfq_group, stats.serviced),
		.seq_show = gfqg_print_rwstat_recursive,
	},
	{
		.name = "io_service_time_recursive",
		.private = offsetof(struct gfq_group, stats.service_time),
		.seq_show = gfqg_print_rwstat_recursive,
	},
	{
		.name = "io_wait_time_recursive",
		.private = offsetof(struct gfq_group, stats.wait_time),
		.seq_show = gfqg_print_rwstat_recursive,
	},
	{
		.name = "io_merged_recursive",
		.private = offsetof(struct gfq_group, stats.merged),
		.seq_show = gfqg_print_rwstat_recursive,
	},
	{
		.name = "io_queued_recursive",
		.private = offsetof(struct gfq_group, stats.queued),
		.seq_show = gfqg_print_rwstat_recursive,
	},
#ifdef CONFIG_DEBUG_BLK_CGROUP
	{
		.name = "avg_queue_size",
		.seq_show = gfqg_print_avg_queue_size,
	},
	{
		.name = "group_wait_time",
		.private = offsetof(struct gfq_group, stats.group_wait_time),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "idle_time",
		.private = offsetof(struct gfq_group, stats.idle_time),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "empty_time",
		.private = offsetof(struct gfq_group, stats.empty_time),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "dequeue",
		.private = offsetof(struct gfq_group, stats.dequeue),
		.seq_show = gfqg_print_stat,
	},
	{
		.name = "unaccounted_time",
		.private = offsetof(struct gfq_group, stats.unaccounted_time),
		.seq_show = gfqg_print_stat,
	},
#endif	/* CONFIG_DEBUG_BLK_CGROUP */
	{ }	/* terminate */
};
#else /* GROUP_IOSCHED */
static struct gfq_group *gfq_lookup_create_gfqg(struct gfq_data *gfqd,
						struct blkcg *blkcg)
{
	return gfqd->root_group;
}

static inline void
gfq_link_gfqq_gfqg(struct gfq_queue *gfqq, struct gfq_group *gfqg) {
	gfqq->gfqg = gfqg;
}

#endif /* GROUP_IOSCHED */

/*
 * The gfqd->service_trees holds all pending gfq_queue's that have
 * requests waiting to be processed. It is sorted in the order that
 * we will service the queues.
 */
static void gfq_service_tree_add(struct gfq_data *gfqd, struct gfq_queue *gfqq,
				 bool add_front)
{
	struct rb_node **p, *parent;
	struct gfq_queue *__gfqq;
	unsigned long rb_key;
	struct gfq_rb_root *st;
	int left;
	int new_gfqq = 1;

	st = st_for(gfqq->gfqg, gfqq_class(gfqq), gfqq_type(gfqq));
	if (gfq_class_idle(gfqq)) {
		rb_key = GFQ_IDLE_DELAY;
		parent = rb_last(&st->rb);
		if (parent && parent != &gfqq->rb_node) {
			__gfqq = rb_entry(parent, struct gfq_queue, rb_node);
			rb_key += __gfqq->rb_key;
		} else
			rb_key += jiffies;
	} else if (!add_front) {
		/*
		 * Get our rb key offset. Subtract any residual slice
		 * value carried from last service. A negative resid
		 * count indicates slice overrun, and this should position
		 * the next service time further away in the tree.
		 */
		rb_key = gfq_slice_offset(gfqd, gfqq) + jiffies;
		rb_key -= gfqq->slice_resid;
		gfqq->slice_resid = 0;
	} else {
		rb_key = -HZ;
		__gfqq = gfq_rb_first(st);
		rb_key += __gfqq ? __gfqq->rb_key : jiffies;
	}

	if (!RB_EMPTY_NODE(&gfqq->rb_node)) {
		new_gfqq = 0;
		/*
		 * same position, nothing more to do
		 */
		if (rb_key == gfqq->rb_key && gfqq->service_tree == st)
			return;

		gfq_rb_erase(&gfqq->rb_node, gfqq->service_tree);
		gfqq->service_tree = NULL;
	}

	left = 1;
	parent = NULL;
	gfqq->service_tree = st;
	p = &st->rb.rb_node;
	while (*p) {
		parent = *p;
		__gfqq = rb_entry(parent, struct gfq_queue, rb_node);

		/*
		 * sort by key, that represents service time.
		 */
		if (time_before(rb_key, __gfqq->rb_key))
			p = &parent->rb_left;
		else {
			p = &parent->rb_right;
			left = 0;
		}
	}

	if (left)
		st->left = &gfqq->rb_node;

	gfqq->rb_key = rb_key;
	rb_link_node(&gfqq->rb_node, parent, p);
	rb_insert_color(&gfqq->rb_node, &st->rb);
	st->count++;
	if (add_front || !new_gfqq)
		return;
	gfq_group_notify_queue_add(gfqd, gfqq->gfqg);
}

static struct gfq_queue *
gfq_prio_tree_lookup(struct gfq_data *gfqd, struct rb_root *root,
		     sector_t sector, struct rb_node **ret_parent,
		     struct rb_node ***rb_link)
{
	struct rb_node **p, *parent;
	struct gfq_queue *gfqq = NULL;

	parent = NULL;
	p = &root->rb_node;
	while (*p) {
		struct rb_node **n;

		parent = *p;
		gfqq = rb_entry(parent, struct gfq_queue, p_node);

		/*
		 * Sort strictly based on sector.  Smallest to the left,
		 * largest to the right.
		 */
		if (sector > blk_rq_pos(gfqq->next_rq))
			n = &(*p)->rb_right;
		else if (sector < blk_rq_pos(gfqq->next_rq))
			n = &(*p)->rb_left;
		else
			break;
		p = n;
		gfqq = NULL;
	}

	*ret_parent = parent;
	if (rb_link)
		*rb_link = p;
	return gfqq;
}

static void gfq_prio_tree_add(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	struct rb_node **p, *parent;
	struct gfq_queue *__gfqq;

	if (gfqq->p_root) {
		rb_erase(&gfqq->p_node, gfqq->p_root);
		gfqq->p_root = NULL;
	}

	if (gfq_class_idle(gfqq))
		return;
	if (!gfqq->next_rq)
		return;

	gfqq->p_root = &gfqd->prio_trees[gfqq->org_ioprio];
	__gfqq = gfq_prio_tree_lookup(gfqd, gfqq->p_root,
				      blk_rq_pos(gfqq->next_rq), &parent, &p);
	if (!__gfqq) {
		rb_link_node(&gfqq->p_node, parent, p);
		rb_insert_color(&gfqq->p_node, gfqq->p_root);
	} else
		gfqq->p_root = NULL;
}

/*
 * Update gfqq's position in the service tree.
 */
static void gfq_resort_rr_list(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	/*
	 * Resorting requires the gfqq to be on the RR list already.
	 */
	if (gfq_gfqq_on_rr(gfqq)) {
		gfq_service_tree_add(gfqd, gfqq, 0);
		gfq_prio_tree_add(gfqd, gfqq);
	}
}

/*
 * add to busy list of queues for service, trying to be fair in ordering
 * the pending list according to last request service
 */
static void gfq_add_gfqq_rr(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	gfq_log_gfqq(gfqd, gfqq, "add_to_rr");
	BUG_ON(gfq_gfqq_on_rr(gfqq));
	gfq_mark_gfqq_on_rr(gfqq);
	gfqd->busy_queues++;
	if (gfq_gfqq_sync(gfqq))
		gfqd->busy_sync_queues++;

	gfq_resort_rr_list(gfqd, gfqq);
}

/*
 * Called when the gfqq no longer has requests pending, remove it from
 * the service tree.
 */
static void gfq_del_gfqq_rr(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	gfq_log_gfqq(gfqd, gfqq, "del_from_rr");
	BUG_ON(!gfq_gfqq_on_rr(gfqq));
	gfq_clear_gfqq_on_rr(gfqq);

	if (!RB_EMPTY_NODE(&gfqq->rb_node)) {
		gfq_rb_erase(&gfqq->rb_node, gfqq->service_tree);
		gfqq->service_tree = NULL;
	}
	if (gfqq->p_root) {
		rb_erase(&gfqq->p_node, gfqq->p_root);
		gfqq->p_root = NULL;
	}

	gfq_group_notify_queue_del(gfqd, gfqq->gfqg);
	BUG_ON(!gfqd->busy_queues);
	gfqd->busy_queues--;
	if (gfq_gfqq_sync(gfqq))
		gfqd->busy_sync_queues--;
}

/*
 * rb tree support functions
 */
static void gfq_del_rq_rb(struct request *rq)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);
	const int sync = rq_is_sync(rq);

	BUG_ON(!gfqq->queued[sync]);
	gfqq->queued[sync]--;

	elv_rb_del(&gfqq->sort_list, rq);

	if (gfq_gfqq_on_rr(gfqq) && RB_EMPTY_ROOT(&gfqq->sort_list)) {
		/*
		 * Queue will be deleted from service tree when we actually
		 * expire it later. Right now just remove it from prio tree
		 * as it is empty.
		 */
		if (gfqq->p_root) {
			rb_erase(&gfqq->p_node, gfqq->p_root);
			gfqq->p_root = NULL;
		}
	}
}

static void gfq_add_rq_rb(struct request *rq)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);
	struct gfq_data *gfqd = gfqq->gfqd;
	struct request *prev;

	gfqq->queued[rq_is_sync(rq)]++;

	elv_rb_add(&gfqq->sort_list, rq);

	if (!gfq_gfqq_on_rr(gfqq))
		gfq_add_gfqq_rr(gfqd, gfqq);

	/*
	 * check if this request is a better next-serve candidate
	 */
	prev = gfqq->next_rq;
	gfqq->next_rq = gfq_choose_req(gfqd, gfqq->next_rq, rq, gfqd->last_position);

	/*
	 * adjust priority tree position, if ->next_rq changes
	 */
	if (prev != gfqq->next_rq)
		gfq_prio_tree_add(gfqd, gfqq);

	BUG_ON(!gfqq->next_rq);
}

static void gfq_reposition_rq_rb(struct gfq_queue *gfqq, struct request *rq)
{
	elv_rb_del(&gfqq->sort_list, rq);
	gfqq->queued[rq_is_sync(rq)]--;
	gfqg_stats_update_io_remove(RQ_GFQG(rq), rq->cmd_flags);
	gfq_add_rq_rb(rq);
	gfqg_stats_update_io_add(RQ_GFQG(rq), gfqq->gfqd->serving_group,
				 rq->cmd_flags);
}

static struct request *
gfq_find_rq_fmerge(struct gfq_data *gfqd, struct bio *bio)
{
	struct task_struct *tsk = current;
	struct gfq_io_cq *cic;
	struct gfq_queue *gfqq;

	cic = gfq_cic_lookup(gfqd, tsk->io_context);
	if (!cic)
		return NULL;

	gfqq = cic_to_gfqq(cic, gfq_bio_sync(bio));
	if (gfqq)
		return elv_rb_find(&gfqq->sort_list, bio_end_sector(bio));

	return NULL;
}

static void gfq_activate_request(struct request_queue *q, struct request *rq)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;

	gfqd->rq_in_driver++;
	gfq_log_gfqq(gfqd, RQ_GFQQ(rq), "activate rq, drv=%d",
						gfqd->rq_in_driver);

	gfqd->last_position = blk_rq_pos(rq) + blk_rq_sectors(rq);
}

static void gfq_deactivate_request(struct request_queue *q, struct request *rq)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;

	WARN_ON(!gfqd->rq_in_driver);
	gfqd->rq_in_driver--;
	gfq_log_gfqq(gfqd, RQ_GFQQ(rq), "deactivate rq, drv=%d",
						gfqd->rq_in_driver);
}

static void gfq_remove_request(struct request *rq)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);

	if (gfqq->next_rq == rq)
		gfqq->next_rq = gfq_find_next_rq(gfqq->gfqd, gfqq, rq);

	list_del_init(&rq->queuelist);
	gfq_del_rq_rb(rq);

	gfqq->gfqd->rq_queued--;
	gfqg_stats_update_io_remove(RQ_GFQG(rq), rq->cmd_flags);
	if (rq->cmd_flags & REQ_PRIO) {
		WARN_ON(!gfqq->prio_pending);
		gfqq->prio_pending--;
	}
}

static int gfq_merge(struct request_queue *q, struct request **req,
		     struct bio *bio)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct request *__rq;

	__rq = gfq_find_rq_fmerge(gfqd, bio);
	if (__rq && elv_rq_merge_ok(__rq, bio)) {
		*req = __rq;
		return ELEVATOR_FRONT_MERGE;
	}

	return ELEVATOR_NO_MERGE;
}

static void gfq_merged_request(struct request_queue *q, struct request *req,
			       int type)
{
	if (type == ELEVATOR_FRONT_MERGE) {
		struct gfq_queue *gfqq = RQ_GFQQ(req);

		gfq_reposition_rq_rb(gfqq, req);
	}
}

static void gfq_bio_merged(struct request_queue *q, struct request *req,
				struct bio *bio)
{
	gfqg_stats_update_io_merged(RQ_GFQG(req), bio->bi_rw);
}

static void
gfq_merged_requests(struct request_queue *q, struct request *rq,
		    struct request *next)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);
	struct gfq_data *gfqd = q->elevator->elevator_data;

	/*
	 * reposition in fifo if next is older than rq
	 */
	if (!list_empty(&rq->queuelist) && !list_empty(&next->queuelist) &&
	    time_before(next->fifo_time, rq->fifo_time) &&
	    gfqq == RQ_GFQQ(next)) {
		list_move(&rq->queuelist, &next->queuelist);
		rq->fifo_time = next->fifo_time;
	}

	if (gfqq->next_rq == next)
		gfqq->next_rq = rq;
	gfq_remove_request(next);
	gfqg_stats_update_io_merged(RQ_GFQG(rq), next->cmd_flags);

	gfqq = RQ_GFQQ(next);
	/*
	 * all requests of this queue are merged to other queues, delete it
	 * from the service tree. If it's the active_queue,
	 * gfq_dispatch_requests() will choose to expire it or do idle
	 */
	if (gfq_gfqq_on_rr(gfqq) && RB_EMPTY_ROOT(&gfqq->sort_list) &&
	    gfqq != gfqd->active_queue)
		gfq_del_gfqq_rr(gfqd, gfqq);
}

static int gfq_allow_merge(struct request_queue *q, struct request *rq,
			   struct bio *bio)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct gfq_io_cq *cic;
	struct gfq_queue *gfqq;

	/*
	 * Disallow merge of a sync bio into an async request.
	 */
	if (gfq_bio_sync(bio) && !rq_is_sync(rq))
		return false;

	/*
	 * Lookup the gfqq that this bio will be queued with and allow
	 * merge only if rq is queued there.
	 */
	cic = gfq_cic_lookup(gfqd, current->io_context);
	if (!cic)
		return false;

	gfqq = cic_to_gfqq(cic, gfq_bio_sync(bio));
	return gfqq == RQ_GFQQ(rq);
}

static inline void gfq_del_timer(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	del_timer(&gfqd->idle_slice_timer);
	gfqg_stats_update_idle_time(gfqq->gfqg);
}

static void __gfq_set_active_queue(struct gfq_data *gfqd,
				   struct gfq_queue *gfqq)
{
	if (gfqq) {
		gfq_log_gfqq(gfqd, gfqq, "set_active wl_class:%d wl_type:%d",
				gfqd->serving_wl_class, gfqd->serving_wl_type);
		gfqg_stats_update_avg_queue_size(gfqq->gfqg);
		gfqq->slice_start = 0;
		gfqq->dispatch_start = jiffies;
		gfqq->allocated_slice = 0;
		gfqq->slice_end = 0;
		gfqq->slice_dispatch = 0;
		gfqq->nr_sectors = 0;

		gfq_clear_gfqq_wait_request(gfqq);
		gfq_clear_gfqq_must_dispatch(gfqq);
		gfq_clear_gfqq_must_alloc_slice(gfqq);
		gfq_clear_gfqq_fifo_expire(gfqq);
		gfq_mark_gfqq_slice_new(gfqq);

		gfq_del_timer(gfqd, gfqq);
	}

	gfqd->active_queue = gfqq;
}

/*
 * current gfqq expired its slice (or was too idle), select new one
 */
static void
__gfq_slice_expired(struct gfq_data *gfqd, struct gfq_queue *gfqq,
		    bool timed_out)
{
	gfq_log_gfqq(gfqd, gfqq, "slice expired t=%d", timed_out);

	if (gfq_gfqq_wait_request(gfqq))
		gfq_del_timer(gfqd, gfqq);

	gfq_clear_gfqq_wait_request(gfqq);
	gfq_clear_gfqq_wait_busy(gfqq);

	/*
	 * If this gfqq is shared between multiple processes, check to
	 * make sure that those processes are still issuing I/Os within
	 * the mean seek distance.  If not, it may be time to break the
	 * queues apart again.
	 */
	if (gfq_gfqq_coop(gfqq) && GFQQ_SEEKY(gfqq))
		gfq_mark_gfqq_split_coop(gfqq);

	/*
	 * store what was left of this slice, if the queue idled/timed out
	 */
	if (timed_out) {
		if (gfq_gfqq_slice_new(gfqq))
			gfqq->slice_resid = gfq_scaled_gfqq_slice(gfqd, gfqq);
		else
			gfqq->slice_resid = gfqq->slice_end - jiffies;
		gfq_log_gfqq(gfqd, gfqq, "resid=%ld", gfqq->slice_resid);
	}

	gfq_group_served(gfqd, gfqq->gfqg, gfqq);

	if (gfq_gfqq_on_rr(gfqq) && RB_EMPTY_ROOT(&gfqq->sort_list))
		gfq_del_gfqq_rr(gfqd, gfqq);

	gfq_resort_rr_list(gfqd, gfqq);

	if (gfqq == gfqd->active_queue)
		gfqd->active_queue = NULL;

	if (gfqd->active_cic) {
		put_io_context(gfqd->active_cic->icq.ioc);
		gfqd->active_cic = NULL;
	}
}

static inline void gfq_slice_expired(struct gfq_data *gfqd, bool timed_out)
{
	struct gfq_queue *gfqq = gfqd->active_queue;

	if (gfqq)
		__gfq_slice_expired(gfqd, gfqq, timed_out);
}

/*
 * Get next queue for service. Unless we have a queue preemption,
 * we'll simply select the first gfqq in the service tree.
 */
static struct gfq_queue *gfq_get_next_queue(struct gfq_data *gfqd)
{
	struct gfq_rb_root *st = st_for(gfqd->serving_group,
			gfqd->serving_wl_class, gfqd->serving_wl_type);

	if (!gfqd->rq_queued)
		return NULL;

	/* There is nothing to dispatch */
	if (!st)
		return NULL;
	if (RB_EMPTY_ROOT(&st->rb))
		return NULL;
	return gfq_rb_first(st);
}

static struct gfq_queue *gfq_get_next_queue_forced(struct gfq_data *gfqd)
{
	struct gfq_group *gfqg;
	struct gfq_queue *gfqq;
	int i, j;
	struct gfq_rb_root *st;

	if (!gfqd->rq_queued)
		return NULL;

	gfqg = gfq_get_next_gfqg(gfqd);
	if (!gfqg)
		return NULL;

	for_each_gfqg_st(gfqg, i, j, st)
		if ((gfqq = gfq_rb_first(st)) != NULL)
			return gfqq;
	return NULL;
}

/*
 * Get and set a new active queue for service.
 */
static struct gfq_queue *gfq_set_active_queue(struct gfq_data *gfqd,
					      struct gfq_queue *gfqq)
{
	if (!gfqq)
		gfqq = gfq_get_next_queue(gfqd);

	__gfq_set_active_queue(gfqd, gfqq);
	return gfqq;
}

static inline sector_t gfq_dist_from_last(struct gfq_data *gfqd,
					  struct request *rq)
{
	if (blk_rq_pos(rq) >= gfqd->last_position)
		return blk_rq_pos(rq) - gfqd->last_position;
	else
		return gfqd->last_position - blk_rq_pos(rq);
}

static inline int gfq_rq_close(struct gfq_data *gfqd, struct gfq_queue *gfqq,
			       struct request *rq)
{
	return gfq_dist_from_last(gfqd, rq) <= GFQQ_CLOSE_THR;
}

static struct gfq_queue *gfqq_close(struct gfq_data *gfqd,
				    struct gfq_queue *cur_gfqq)
{
	struct rb_root *root = &gfqd->prio_trees[cur_gfqq->org_ioprio];
	struct rb_node *parent, *node;
	struct gfq_queue *__gfqq;
	sector_t sector = gfqd->last_position;

	if (RB_EMPTY_ROOT(root))
		return NULL;

	/*
	 * First, if we find a request starting at the end of the last
	 * request, choose it.
	 */
	__gfqq = gfq_prio_tree_lookup(gfqd, root, sector, &parent, NULL);
	if (__gfqq)
		return __gfqq;

	/*
	 * If the exact sector wasn't found, the parent of the NULL leaf
	 * will contain the closest sector.
	 */
	__gfqq = rb_entry(parent, struct gfq_queue, p_node);
	if (gfq_rq_close(gfqd, cur_gfqq, __gfqq->next_rq))
		return __gfqq;

	if (blk_rq_pos(__gfqq->next_rq) < sector)
		node = rb_next(&__gfqq->p_node);
	else
		node = rb_prev(&__gfqq->p_node);
	if (!node)
		return NULL;

	__gfqq = rb_entry(node, struct gfq_queue, p_node);
	if (gfq_rq_close(gfqd, cur_gfqq, __gfqq->next_rq))
		return __gfqq;

	return NULL;
}

/*
 * gfqd - obvious
 * cur_gfqq - passed in so that we don't decide that the current queue is
 * 	      closely cooperating with itself.
 *
 * So, basically we're assuming that that cur_gfqq has dispatched at least
 * one request, and that gfqd->last_position reflects a position on the disk
 * associated with the I/O issued by cur_gfqq.  I'm not sure this is a valid
 * assumption.
 */
static struct gfq_queue *gfq_close_cooperator(struct gfq_data *gfqd,
					      struct gfq_queue *cur_gfqq)
{
	struct gfq_queue *gfqq;

	if (gfq_class_idle(cur_gfqq))
		return NULL;
	if (!gfq_gfqq_sync(cur_gfqq))
		return NULL;
	if (GFQQ_SEEKY(cur_gfqq))
		return NULL;

	/*
	 * Don't search priority tree if it's the only queue in the group.
	 */
	if (cur_gfqq->gfqg->nr_gfqq == 1)
		return NULL;

	/*
	 * We should notice if some of the queues are cooperating, eg
	 * working closely on the same area of the disk. In that case,
	 * we can group them together and don't waste time idling.
	 */
	gfqq = gfqq_close(gfqd, cur_gfqq);
	if (!gfqq)
		return NULL;

	/* If new queue belongs to different gfq_group, don't choose it */
	if (cur_gfqq->gfqg != gfqq->gfqg)
		return NULL;

	/*
	 * It only makes sense to merge sync queues.
	 */
	if (!gfq_gfqq_sync(gfqq))
		return NULL;
	if (GFQQ_SEEKY(gfqq))
		return NULL;

	/*
	 * Do not merge queues of different priority classes
	 */
	if (gfq_class_rt(gfqq) != gfq_class_rt(cur_gfqq))
		return NULL;

	return gfqq;
}

/*
 * Determine whether we should enforce idle window for this queue.
 */

static bool gfq_should_idle(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	enum wl_class_t wl_class = gfqq_class(gfqq);
	struct gfq_rb_root *st = gfqq->service_tree;

	BUG_ON(!st);
	BUG_ON(!st->count);

	if (!gfqd->gfq_slice_idle)
		return false;

	/* We never do for idle class queues. */
	if (wl_class == IDLE_WORKLOAD)
		return false;

	/* We do for queues that were marked with idle window flag. */
	if (gfq_gfqq_idle_window(gfqq) &&
	   !(blk_queue_nonrot(gfqd->queue) && gfqd->hw_tag))
		return true;

	/*
	 * Otherwise, we do only if they are the last ones
	 * in their service tree.
	 */
	if (st->count == 1 && gfq_gfqq_sync(gfqq) &&
	   !gfq_io_thinktime_big(gfqd, &st->ttime, false))
		return true;
	gfq_log_gfqq(gfqd, gfqq, "Not idling. st->count:%d", st->count);
	return false;
}

static void gfq_arm_slice_timer(struct gfq_data *gfqd)
{
	struct gfq_queue *gfqq = gfqd->active_queue;
	struct gfq_io_cq *cic;
	unsigned long sl, group_idle = 0;

	/*
	 * SSD device without seek penalty, disable idling. But only do so
	 * for devices that support queuing, otherwise we still have a problem
	 * with sync vs async workloads.
	 */
	if (blk_queue_nonrot(gfqd->queue) && gfqd->hw_tag)
		return;

	WARN_ON(!RB_EMPTY_ROOT(&gfqq->sort_list));
	WARN_ON(gfq_gfqq_slice_new(gfqq));

	/*
	 * idle is disabled, either manually or by past process history
	 */
	if (!gfq_should_idle(gfqd, gfqq)) {
		/* no queue idling. Check for group idling */
		if (gfqd->gfq_group_idle)
			group_idle = gfqd->gfq_group_idle;
		else
			return;
	}

	/*
	 * still active requests from this queue, don't idle
	 */
	if (gfqq->dispatched)
		return;

	/*
	 * task has exited, don't wait
	 */
	cic = gfqd->active_cic;
	if (!cic || !atomic_read(&cic->icq.ioc->active_ref))
		return;

	/*
	 * If our average think time is larger than the remaining time
	 * slice, then don't idle. This avoids overrunning the allotted
	 * time slice.
	 */
	if (sample_valid(cic->ttime.ttime_samples) &&
	    (gfqq->slice_end - jiffies < cic->ttime.ttime_mean)) {
		gfq_log_gfqq(gfqd, gfqq, "Not idling. think_time:%lu",
			     cic->ttime.ttime_mean);
		return;
	}

	/* There are other queues in the group, don't do group idle */
	if (group_idle && gfqq->gfqg->nr_gfqq > 1)
		return;

	gfq_mark_gfqq_wait_request(gfqq);

	if (group_idle)
		sl = gfqd->gfq_group_idle;
	else
		sl = gfqd->gfq_slice_idle;

	mod_timer(&gfqd->idle_slice_timer, jiffies + sl);
	gfqg_stats_set_start_idle_time(gfqq->gfqg);
	gfq_log_gfqq(gfqd, gfqq, "arm_idle: %lu group_idle: %d", sl,
			group_idle ? 1 : 0);
}

/*
 * Move request from internal lists to the request queue dispatch list.
 */
static void gfq_dispatch_insert(struct request_queue *q, struct request *rq)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct gfq_queue *gfqq = RQ_GFQQ(rq);

	gfq_log_gfqq(gfqd, gfqq, "dispatch_insert");

	gfqq->next_rq = gfq_find_next_rq(gfqd, gfqq, rq);
	gfq_remove_request(rq);
	gfqq->dispatched++;
	(RQ_GFQG(rq))->dispatched++;
	elv_dispatch_sort(q, rq);

	gfqd->rq_in_flight[gfq_gfqq_sync(gfqq)]++;
	gfqq->nr_sectors += blk_rq_sectors(rq);
	gfqg_stats_update_dispatch(gfqq->gfqg, blk_rq_bytes(rq), rq->cmd_flags);
}

/*
 * return expired entry, or NULL to just start from scratch in rbtree
 */
static struct request *gfq_check_fifo(struct gfq_queue *gfqq)
{
	struct request *rq = NULL;

	if (gfq_gfqq_fifo_expire(gfqq))
		return NULL;

	gfq_mark_gfqq_fifo_expire(gfqq);

	if (list_empty(&gfqq->fifo))
		return NULL;

	rq = rq_entry_fifo(gfqq->fifo.next);
	if (time_before(jiffies, rq->fifo_time))
		rq = NULL;

	gfq_log_gfqq(gfqq->gfqd, gfqq, "fifo=%p", rq);
	return rq;
}

static inline int
gfq_prio_to_maxrq(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	const int base_rq = gfqd->gfq_slice_async_rq;

	WARN_ON(gfqq->ioprio >= IOPRIO_BE_NR);

	return 2 * base_rq * (IOPRIO_BE_NR - gfqq->ioprio);
}

/*
 * Must be called with the queue_lock held.
 */
static int gfqq_process_refs(struct gfq_queue *gfqq)
{
	int process_refs, io_refs;

	io_refs = gfqq->allocated[READ] + gfqq->allocated[WRITE];
	process_refs = gfqq->ref - io_refs;
	BUG_ON(process_refs < 0);
	return process_refs;
}

static void gfq_setup_merge(struct gfq_queue *gfqq, struct gfq_queue *new_gfqq)
{
	int process_refs, new_process_refs;
	struct gfq_queue *__gfqq;

	/*
	 * If there are no process references on the new_gfqq, then it is
	 * unsafe to follow the ->new_gfqq chain as other gfqq's in the
	 * chain may have dropped their last reference (not just their
	 * last process reference).
	 */
	if (!gfqq_process_refs(new_gfqq))
		return;

	/* Avoid a circular list and skip interim queue merges */
	while ((__gfqq = new_gfqq->new_gfqq)) {
		if (__gfqq == gfqq)
			return;
		new_gfqq = __gfqq;
	}

	process_refs = gfqq_process_refs(gfqq);
	new_process_refs = gfqq_process_refs(new_gfqq);
	/*
	 * If the process for the gfqq has gone away, there is no
	 * sense in merging the queues.
	 */
	if (process_refs == 0 || new_process_refs == 0)
		return;

	/*
	 * Merge in the direction of the lesser amount of work.
	 */
	if (new_process_refs >= process_refs) {
		gfqq->new_gfqq = new_gfqq;
		new_gfqq->ref += process_refs;
	} else {
		new_gfqq->new_gfqq = gfqq;
		gfqq->ref += new_process_refs;
	}
}

static enum wl_type_t gfq_choose_wl_type(struct gfq_data *gfqd,
			struct gfq_group *gfqg, enum wl_class_t wl_class)
{
	struct gfq_queue *queue;
	int i;
	bool key_valid = false;
	unsigned long lowest_key = 0;
	enum wl_type_t cur_best = SYNC_NOIDLE_WORKLOAD;

	for (i = 0; i <= SYNC_WORKLOAD; ++i) {
		/* select the one with lowest rb_key */
		queue = gfq_rb_first(st_for(gfqg, wl_class, i));
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
choose_wl_class_and_type(struct gfq_data *gfqd, struct gfq_group *gfqg)
{
	unsigned slice;
	unsigned count;
	struct gfq_rb_root *st;
	unsigned group_slice;
	enum wl_class_t original_class = gfqd->serving_wl_class;

	/* Choose next priority. RT > BE > IDLE */
	if (gfq_group_busy_queues_wl(RT_WORKLOAD, gfqd, gfqg))
		gfqd->serving_wl_class = RT_WORKLOAD;
	else if (gfq_group_busy_queues_wl(BE_WORKLOAD, gfqd, gfqg))
		gfqd->serving_wl_class = BE_WORKLOAD;
	else {
		gfqd->serving_wl_class = IDLE_WORKLOAD;
		gfqd->workload_expires = jiffies + 1;
		return;
	}

	if (original_class != gfqd->serving_wl_class)
		goto new_workload;

	/*
	 * For RT and BE, we have to choose also the type
	 * (SYNC, SYNC_NOIDLE, ASYNC), and to compute a workload
	 * expiration time
	 */
	st = st_for(gfqg, gfqd->serving_wl_class, gfqd->serving_wl_type);
	count = st->count;

	/*
	 * check workload expiration, and that we still have other queues ready
	 */
	if (count && !time_after(jiffies, gfqd->workload_expires))
		return;

new_workload:
	/* otherwise select new workload type */
	gfqd->serving_wl_type = gfq_choose_wl_type(gfqd, gfqg,
					gfqd->serving_wl_class);
	st = st_for(gfqg, gfqd->serving_wl_class, gfqd->serving_wl_type);
	count = st->count;

	/*
	 * the workload slice is computed as a fraction of target latency
	 * proportional to the number of queues in that workload, over
	 * all the queues in the same priority class
	 */
	group_slice = gfq_group_slice(gfqd, gfqg);

	slice = group_slice * count /
		max_t(unsigned, gfqg->busy_queues_avg[gfqd->serving_wl_class],
		      gfq_group_busy_queues_wl(gfqd->serving_wl_class, gfqd,
					gfqg));

	if (gfqd->serving_wl_type == ASYNC_WORKLOAD) {
		unsigned int tmp;

		/*
		 * Async queues are currently system wide. Just taking
		 * proportion of queues with-in same group will lead to higher
		 * async ratio system wide as generally root group is going
		 * to have higher weight. A more accurate thing would be to
		 * calculate system wide asnc/sync ratio.
		 */
		tmp = gfqd->gfq_target_latency *
			gfqg_busy_async_queues(gfqd, gfqg);
		tmp = tmp/gfqd->busy_queues;
		slice = min_t(unsigned, slice, tmp);

		/* async workload slice is scaled down according to
		 * the sync/async slice ratio. */
		slice = slice * gfqd->gfq_slice[0] / gfqd->gfq_slice[1];
	} else
		/* sync workload slice is at least 2 * gfq_slice_idle */
		slice = max(slice, 2 * gfqd->gfq_slice_idle);

	slice = max_t(unsigned, slice, GFQ_MIN_TT);
	gfq_log(gfqd, "workload slice:%d", slice);
	gfqd->workload_expires = jiffies + slice;
}

static struct gfq_group *gfq_get_next_gfqg(struct gfq_data *gfqd)
{
	struct gfq_rb_root *st = &gfqd->grp_service_tree;
	struct gfq_group *gfqg;

	if (RB_EMPTY_ROOT(&st->rb))
		return NULL;
	gfqg = gfq_rb_first_group(st);
	update_min_vdisktime(st);
	return gfqg;
}

static void gfq_choose_gfqg(struct gfq_data *gfqd)
{
	struct gfq_group *gfqg = gfq_get_next_gfqg(gfqd);

	gfqd->serving_group = gfqg;

	/* Restore the workload type data */
	if (gfqg->saved_wl_slice) {
		gfqd->workload_expires = jiffies + gfqg->saved_wl_slice;
		gfqd->serving_wl_type = gfqg->saved_wl_type;
		gfqd->serving_wl_class = gfqg->saved_wl_class;
	} else
		gfqd->workload_expires = jiffies - 1;

	choose_wl_class_and_type(gfqd, gfqg);
}

/*
 * Select a queue for service. If we have a current active queue,
 * check whether to continue servicing it, or retrieve and set a new one.
 */
static struct gfq_queue *gfq_select_queue(struct gfq_data *gfqd)
{
	struct gfq_queue *gfqq, *new_gfqq = NULL;

	gfqq = gfqd->active_queue;
	if (!gfqq)
		goto new_queue;

	if (!gfqd->rq_queued)
		return NULL;

	/*
	 * We were waiting for group to get backlogged. Expire the queue
	 */
	if (gfq_gfqq_wait_busy(gfqq) && !RB_EMPTY_ROOT(&gfqq->sort_list))
		goto expire;

	/*
	 * The active queue has run out of time, expire it and select new.
	 */
	if (gfq_slice_used(gfqq) && !gfq_gfqq_must_dispatch(gfqq)) {
		/*
		 * If slice had not expired at the completion of last request
		 * we might not have turned on wait_busy flag. Don't expire
		 * the queue yet. Allow the group to get backlogged.
		 *
		 * The very fact that we have used the slice, that means we
		 * have been idling all along on this queue and it should be
		 * ok to wait for this request to complete.
		 */
		if (gfqq->gfqg->nr_gfqq == 1 && RB_EMPTY_ROOT(&gfqq->sort_list)
		    && gfqq->dispatched && gfq_should_idle(gfqd, gfqq)) {
			gfqq = NULL;
			goto keep_queue;
		} else
			goto check_group_idle;
	}

	/*
	 * The active queue has requests and isn't expired, allow it to
	 * dispatch.
	 */
	if (!RB_EMPTY_ROOT(&gfqq->sort_list))
		goto keep_queue;

	/*
	 * If another queue has a request waiting within our mean seek
	 * distance, let it run.  The expire code will check for close
	 * cooperators and put the close queue at the front of the service
	 * tree.  If possible, merge the expiring queue with the new gfqq.
	 */
	new_gfqq = gfq_close_cooperator(gfqd, gfqq);
	if (new_gfqq) {
		if (!gfqq->new_gfqq)
			gfq_setup_merge(gfqq, new_gfqq);
		goto expire;
	}

	/*
	 * No requests pending. If the active queue still has requests in
	 * flight or is idling for a new request, allow either of these
	 * conditions to happen (or time out) before selecting a new queue.
	 */
	if (timer_pending(&gfqd->idle_slice_timer)) {
		gfqq = NULL;
		goto keep_queue;
	}

	/*
	 * This is a deep seek queue, but the device is much faster than
	 * the queue can deliver, don't idle
	 **/
	if (GFQQ_SEEKY(gfqq) && gfq_gfqq_idle_window(gfqq) &&
	    (gfq_gfqq_slice_new(gfqq) ||
	    (gfqq->slice_end - jiffies > jiffies - gfqq->slice_start))) {
		gfq_clear_gfqq_deep(gfqq);
		gfq_clear_gfqq_idle_window(gfqq);
	}

	if (gfqq->dispatched && gfq_should_idle(gfqd, gfqq)) {
		gfqq = NULL;
		goto keep_queue;
	}

	/*
	 * If group idle is enabled and there are requests dispatched from
	 * this group, wait for requests to complete.
	 */
check_group_idle:
	if (gfqd->gfq_group_idle && gfqq->gfqg->nr_gfqq == 1 &&
	    gfqq->gfqg->dispatched &&
	    !gfq_io_thinktime_big(gfqd, &gfqq->gfqg->ttime, true)) {
		gfqq = NULL;
		goto keep_queue;
	}

expire:
	gfq_slice_expired(gfqd, 0);
new_queue:
	/*
	 * Current queue expired. Check if we have to switch to a new
	 * service tree
	 */
	if (!new_gfqq)
		gfq_choose_gfqg(gfqd);

	gfqq = gfq_set_active_queue(gfqd, new_gfqq);
keep_queue:
	return gfqq;
}

static int __gfq_forced_dispatch_gfqq(struct gfq_queue *gfqq)
{
	int dispatched = 0;

	while (gfqq->next_rq) {
		gfq_dispatch_insert(gfqq->gfqd->queue, gfqq->next_rq);
		dispatched++;
	}

	BUG_ON(!list_empty(&gfqq->fifo));

	/* By default gfqq is not expired if it is empty. Do it explicitly */
	__gfq_slice_expired(gfqq->gfqd, gfqq, 0);
	return dispatched;
}

/*
 * Drain our current requests. Used for barriers and when switching
 * io schedulers on-the-fly.
 */
static int gfq_forced_dispatch(struct gfq_data *gfqd)
{
	struct gfq_queue *gfqq;
	int dispatched = 0;

	/* Expire the timeslice of the current active queue first */
	gfq_slice_expired(gfqd, 0);
	while ((gfqq = gfq_get_next_queue_forced(gfqd)) != NULL) {
		__gfq_set_active_queue(gfqd, gfqq);
		dispatched += __gfq_forced_dispatch_gfqq(gfqq);
	}

	BUG_ON(gfqd->busy_queues);

	gfq_log(gfqd, "forced_dispatch=%d", dispatched);
	return dispatched;
}

static inline bool gfq_slice_used_soon(struct gfq_data *gfqd,
	struct gfq_queue *gfqq)
{
	/* the queue hasn't finished any request, can't estimate */
	if (gfq_gfqq_slice_new(gfqq))
		return true;
	if (time_after(jiffies + gfqd->gfq_slice_idle * gfqq->dispatched,
		gfqq->slice_end))
		return true;

	return false;
}

static bool gfq_may_dispatch(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	unsigned int max_dispatch;

	/*
	 * Drain async requests before we start sync IO
	 */
	if (gfq_should_idle(gfqd, gfqq) && gfqd->rq_in_flight[BLK_RW_ASYNC])
		return false;

	/*
	 * If this is an async queue and we have sync IO in flight, let it wait
	 */
	if (gfqd->rq_in_flight[BLK_RW_SYNC] && !gfq_gfqq_sync(gfqq))
		return false;

	max_dispatch = max_t(unsigned int, gfqd->gfq_quantum / 2, 1);
	if (gfq_class_idle(gfqq))
		max_dispatch = 1;

	/*
	 * Does this gfqq already have too much IO in flight?
	 */
	if (gfqq->dispatched >= max_dispatch) {
		bool promote_sync = false;
		/*
		 * idle queue must always only have a single IO in flight
		 */
		if (gfq_class_idle(gfqq))
			return false;

		/*
		 * If there is only one sync queue
		 * we can ignore async queue here and give the sync
		 * queue no dispatch limit. The reason is a sync queue can
		 * preempt async queue, limiting the sync queue doesn't make
		 * sense. This is useful for aiostress test.
		 */
		if (gfq_gfqq_sync(gfqq) && gfqd->busy_sync_queues == 1)
			promote_sync = true;

		/*
		 * We have other queues, don't allow more IO from this one
		 */
		if (gfqd->busy_queues > 1 && gfq_slice_used_soon(gfqd, gfqq) &&
				!promote_sync)
			return false;

		/*
		 * Sole queue user, no limit
		 */
		if (gfqd->busy_queues == 1 || promote_sync)
			max_dispatch = -1;
		else
			/*
			 * Normally we start throttling gfqq when gfq_quantum/2
			 * requests have been dispatched. But we can drive
			 * deeper queue depths at the beginning of slice
			 * subjected to upper limit of gfq_quantum.
			 * */
			max_dispatch = gfqd->gfq_quantum;
	}

	/*
	 * Async queues must wait a bit before being allowed dispatch.
	 * We also ramp up the dispatch depth gradually for async IO,
	 * based on the last sync IO we serviced
	 */
	if (!gfq_gfqq_sync(gfqq) && gfqd->gfq_latency) {
		unsigned long last_sync = jiffies - gfqd->last_delayed_sync;
		unsigned int depth;

		depth = last_sync / gfqd->gfq_slice[1];
		if (!depth && !gfqq->dispatched)
			depth = 1;
		if (depth < max_dispatch)
			max_dispatch = depth;
	}

	/*
	 * If we're below the current max, allow a dispatch
	 */
	return gfqq->dispatched < max_dispatch;
}

/*
 * Dispatch a request from gfqq, moving them to the request queue
 * dispatch list.
 */
static bool gfq_dispatch_request(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	struct request *rq;

	BUG_ON(RB_EMPTY_ROOT(&gfqq->sort_list));

	if (!gfq_may_dispatch(gfqd, gfqq))
		return false;

	/*
	 * follow expired path, else get first next available
	 */
	rq = gfq_check_fifo(gfqq);
	if (!rq)
		rq = gfqq->next_rq;

	/*
	 * insert request into driver dispatch list
	 */
	gfq_dispatch_insert(gfqd->queue, rq);

	if (!gfqd->active_cic) {
		struct gfq_io_cq *cic = RQ_CIC(rq);

		atomic_long_inc(&cic->icq.ioc->refcount);
		gfqd->active_cic = cic;
	}

	return true;
}

/*
 * Find the gfqq that we need to service and move a request from that to the
 * dispatch list
 */
static int gfq_dispatch_requests(struct request_queue *q, int force)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct gfq_queue *gfqq;

	if (!gfqd->busy_queues)
		return 0;

	if (unlikely(force))
		return gfq_forced_dispatch(gfqd);

	gfqq = gfq_select_queue(gfqd);
	if (!gfqq)
		return 0;

	/*
	 * Dispatch a request from this gfqq, if it is allowed
	 */
	if (!gfq_dispatch_request(gfqd, gfqq))
		return 0;

	gfqq->slice_dispatch++;
	gfq_clear_gfqq_must_dispatch(gfqq);

	/*
	 * expire an async queue immediately if it has used up its slice. idle
	 * queue always expire after 1 dispatch round.
	 */
	if (gfqd->busy_queues > 1 && ((!gfq_gfqq_sync(gfqq) &&
	    gfqq->slice_dispatch >= gfq_prio_to_maxrq(gfqd, gfqq)) ||
	    gfq_class_idle(gfqq))) {
		gfqq->slice_end = jiffies + 1;
		gfq_slice_expired(gfqd, 0);
	}

	gfq_log_gfqq(gfqd, gfqq, "dispatched a request");
	return 1;
}

/*
 * task holds one reference to the queue, dropped when task exits. each rq
 * in-flight on this queue also holds a reference, dropped when rq is freed.
 *
 * Each gfq queue took a reference on the parent group. Drop it now.
 * queue lock must be held here.
 */
static void gfq_put_queue(struct gfq_queue *gfqq)
{
	struct gfq_data *gfqd = gfqq->gfqd;
	struct gfq_group *gfqg;

	BUG_ON(gfqq->ref <= 0);

	gfqq->ref--;
	if (gfqq->ref)
		return;

	gfq_log_gfqq(gfqd, gfqq, "put_queue");
	BUG_ON(rb_first(&gfqq->sort_list));
	BUG_ON(gfqq->allocated[READ] + gfqq->allocated[WRITE]);
	gfqg = gfqq->gfqg;

	if (unlikely(gfqd->active_queue == gfqq)) {
		__gfq_slice_expired(gfqd, gfqq, 0);
		gfq_schedule_dispatch(gfqd);
	}

	BUG_ON(gfq_gfqq_on_rr(gfqq));
	kmem_cache_free(gfq_pool, gfqq);
	gfqg_put(gfqg);
}

static void gfq_put_cooperator(struct gfq_queue *gfqq)
{
	struct gfq_queue *__gfqq, *next;

	/*
	 * If this queue was scheduled to merge with another queue, be
	 * sure to drop the reference taken on that queue (and others in
	 * the merge chain).  See gfq_setup_merge and gfq_merge_gfqqs.
	 */
	__gfqq = gfqq->new_gfqq;
	while (__gfqq) {
		if (__gfqq == gfqq) {
			WARN(1, "gfqq->new_gfqq loop detected\n");
			break;
		}
		next = __gfqq->new_gfqq;
		gfq_put_queue(__gfqq);
		__gfqq = next;
	}
}

static void gfq_exit_gfqq(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	if (unlikely(gfqq == gfqd->active_queue)) {
		__gfq_slice_expired(gfqd, gfqq, 0);
		gfq_schedule_dispatch(gfqd);
	}

	gfq_put_cooperator(gfqq);

	gfq_put_queue(gfqq);
}

static void gfq_init_icq(struct io_cq *icq)
{
	struct gfq_io_cq *cic = icq_to_cic(icq);

	cic->ttime.last_end_request = jiffies;
}

static void gfq_exit_icq(struct io_cq *icq)
{
	struct gfq_io_cq *cic = icq_to_cic(icq);
	struct gfq_data *gfqd = cic_to_gfqd(cic);

	if (cic->gfqq[BLK_RW_ASYNC]) {
		gfq_exit_gfqq(gfqd, cic->gfqq[BLK_RW_ASYNC]);
		cic->gfqq[BLK_RW_ASYNC] = NULL;
	}

	if (cic->gfqq[BLK_RW_SYNC]) {
		gfq_exit_gfqq(gfqd, cic->gfqq[BLK_RW_SYNC]);
		cic->gfqq[BLK_RW_SYNC] = NULL;
	}
}

static void gfq_init_prio_data(struct gfq_queue *gfqq, struct gfq_io_cq *cic)
{
	struct task_struct *tsk = current;
	int ioprio_class;

	if (!gfq_gfqq_prio_changed(gfqq)) // ?
		return;

	ioprio_class = IOPRIO_PRIO_CLASS(cic->ioprio);
	switch (ioprio_class) {
	default:
		printk(KERN_ERR "gfq: bad prio %x\n", ioprio_class);
	case IOPRIO_CLASS_NONE:
		/*
		 * no prio set, inherit CPU scheduling settings
		 */
		gfqq->ioprio = task_nice_ioprio(tsk);
		gfqq->ioprio_class = task_nice_ioclass(tsk);
		break;
	case IOPRIO_CLASS_RT:
		gfqq->ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
		gfqq->ioprio_class = IOPRIO_CLASS_RT;
		break;
	case IOPRIO_CLASS_BE:
		gfqq->ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
		gfqq->ioprio_class = IOPRIO_CLASS_BE;
		break;
	case IOPRIO_CLASS_IDLE:
		gfqq->ioprio_class = IOPRIO_CLASS_IDLE;
		gfqq->ioprio = 7;
		gfq_clear_gfqq_idle_window(gfqq);
		break;
	}

	/*
	 * keep track of original prio settings in case we have to temporarily
	 * elevate the priority of this queue
	 */
	gfqq->org_ioprio = gfqq->ioprio;
	gfq_clear_gfqq_prio_changed(gfqq);
}

static void check_ioprio_changed(struct gfq_io_cq *cic, struct bio *bio)
{
	int ioprio = cic->icq.ioc->ioprio;
	struct gfq_data *gfqd = cic_to_gfqd(cic);
	struct gfq_queue *gfqq;

	/*
	 * Check whether ioprio has changed.  The condition may trigger
	 * spuriously on a newly created cic but there's no harm.
	 */
	if (unlikely(!gfqd) || likely(cic->ioprio == ioprio))
		return;

	gfqq = cic->gfqq[BLK_RW_ASYNC];
	if (gfqq) {
		struct gfq_queue *new_gfqq;
		new_gfqq = gfq_get_queue(gfqd, BLK_RW_ASYNC, cic, bio,
					 GFP_ATOMIC);
		if (new_gfqq) {
			cic->gfqq[BLK_RW_ASYNC] = new_gfqq;
			gfq_put_queue(gfqq);
		}
	}

	gfqq = cic->gfqq[BLK_RW_SYNC];
	if (gfqq)
		gfq_mark_gfqq_prio_changed(gfqq);

	cic->ioprio = ioprio;
}

static void gfq_init_gfqq(struct gfq_data *gfqd, struct gfq_queue *gfqq,
			  pid_t pid, bool is_sync)
{
	RB_CLEAR_NODE(&gfqq->rb_node);
	RB_CLEAR_NODE(&gfqq->p_node);
	INIT_LIST_HEAD(&gfqq->fifo);

	gfqq->ref = 0;
	gfqq->gfqd = gfqd;

	gfq_mark_gfqq_prio_changed(gfqq);

	if (is_sync) {
		if (!gfq_class_idle(gfqq))
			gfq_mark_gfqq_idle_window(gfqq);
		gfq_mark_gfqq_sync(gfqq);
	}
	gfqq->pid = pid;
}

#ifdef CONFIG_GFQ_GROUP_IOSCHED
static void check_blkcg_changed(struct gfq_io_cq *cic, struct bio *bio)
{
	struct gfq_data *gfqd = cic_to_gfqd(cic);
	struct gfq_queue *sync_gfqq;
	uint64_t serial_nr;

	rcu_read_lock();
	serial_nr = bio_blkcg(bio)->css.serial_nr;
	rcu_read_unlock();

	/*
	 * Check whether blkcg has changed.  The condition may trigger
	 * spuriously on a newly created cic but there's no harm.
	 */
	if (unlikely(!gfqd) || likely(cic->blkcg_serial_nr == serial_nr))
		return;

	sync_gfqq = cic_to_gfqq(cic, 1);
	if (sync_gfqq) {
		/*
		 * Drop reference to sync queue. A new sync queue will be
		 * assigned in new group upon arrival of a fresh request.
		 */
		gfq_log_gfqq(gfqd, sync_gfqq, "changed cgroup");
		cic_set_gfqq(cic, NULL, 1);
		gfq_put_queue(sync_gfqq);
	}

	cic->blkcg_serial_nr = serial_nr;
}
#else
static inline void check_blkcg_changed(struct gfq_io_cq *cic, struct bio *bio) { }
#endif  /* CONFIG_GFQ_GROUP_IOSCHED */

static struct gfq_queue *
gfq_find_alloc_queue(struct gfq_data *gfqd, bool is_sync, struct gfq_io_cq *cic,
		     struct bio *bio, gfp_t gfp_mask)
{
	struct blkcg *blkcg;
	struct gfq_queue *gfqq, *new_gfqq = NULL;
	struct gfq_group *gfqg;

retry:
	rcu_read_lock();

	blkcg = bio_blkcg(bio);
	gfqg = gfq_lookup_create_gfqg(gfqd, blkcg);
	if (!gfqg) {
		gfqq = &gfqd->oom_gfqq;
		goto out;
	}

	gfqq = cic_to_gfqq(cic, is_sync);

	/*
	 * Always try a new alloc if we fell back to the OOM gfqq
	 * originally, since it should just be a temporary situation.
	 */
	if (!gfqq || gfqq == &gfqd->oom_gfqq) {
		gfqq = NULL;
		if (new_gfqq) {
			gfqq = new_gfqq;
			new_gfqq = NULL;
		} else if (gfp_mask & __GFP_WAIT) {
			rcu_read_unlock();
			spin_unlock_irq(gfqd->queue->queue_lock);
			new_gfqq = kmem_cache_alloc_node(gfq_pool,
					gfp_mask | __GFP_ZERO,
					gfqd->queue->node);
			spin_lock_irq(gfqd->queue->queue_lock);
			if (new_gfqq)
				goto retry;
			else
				return &gfqd->oom_gfqq;
		} else {
			gfqq = kmem_cache_alloc_node(gfq_pool,
					gfp_mask | __GFP_ZERO,
					gfqd->queue->node);
		}

		if (gfqq) {
			gfq_init_gfqq(gfqd, gfqq, current->pid, is_sync);
			gfq_init_prio_data(gfqq, cic);
			gfq_link_gfqq_gfqg(gfqq, gfqg);
			gfq_log_gfqq(gfqd, gfqq, "alloced");
		} else
			gfqq = &gfqd->oom_gfqq;
	}
out:
	if (new_gfqq)
		kmem_cache_free(gfq_pool, new_gfqq);

	rcu_read_unlock();
	return gfqq;
}

static struct gfq_queue **
gfq_async_queue_prio(struct gfq_data *gfqd, int ioprio_class, int ioprio)
{
	switch (ioprio_class) {
	case IOPRIO_CLASS_RT:
		return &gfqd->async_gfqq[0][ioprio];
	case IOPRIO_CLASS_NONE:
		ioprio = IOPRIO_NORM;
		/* fall through */
	case IOPRIO_CLASS_BE:
		return &gfqd->async_gfqq[1][ioprio];
	case IOPRIO_CLASS_IDLE:
		return &gfqd->async_idle_gfqq;
	default:
		BUG();
	}
}

static struct gfq_queue *
gfq_get_queue(struct gfq_data *gfqd, bool is_sync, struct gfq_io_cq *cic,
	      struct bio *bio, gfp_t gfp_mask)
{
	int ioprio_class = IOPRIO_PRIO_CLASS(cic->ioprio);
	int ioprio = IOPRIO_PRIO_DATA(cic->ioprio);
	struct gfq_queue **async_gfqq = NULL;
	struct gfq_queue *gfqq = NULL;

	if (!is_sync) {
		if (!ioprio_valid(cic->ioprio)) {
			struct task_struct *tsk = current;
			ioprio = task_nice_ioprio(tsk);
			ioprio_class = task_nice_ioclass(tsk);
		}
		async_gfqq = gfq_async_queue_prio(gfqd, ioprio_class, ioprio);
		gfqq = *async_gfqq;
	}

	if (!gfqq)
		gfqq = gfq_find_alloc_queue(gfqd, is_sync, cic, bio, gfp_mask);

	/*
	 * pin the queue now that it's allocated, scheduler exit will prune it
	 */
	if (!is_sync && !(*async_gfqq)) {
		gfqq->ref++;
		*async_gfqq = gfqq;
	}

	gfqq->ref++;
	return gfqq;
}

static void
__gfq_update_io_thinktime(struct gfq_ttime *ttime, unsigned long slice_idle)
{
	unsigned long elapsed = jiffies - ttime->last_end_request;
	elapsed = min(elapsed, 2UL * slice_idle);

	ttime->ttime_samples = (7*ttime->ttime_samples + 256) / 8;
	ttime->ttime_total = (7*ttime->ttime_total + 256*elapsed) / 8;
	ttime->ttime_mean = (ttime->ttime_total + 128) / ttime->ttime_samples;
}

static void
gfq_update_io_thinktime(struct gfq_data *gfqd, struct gfq_queue *gfqq,
			struct gfq_io_cq *cic)
{
	if (gfq_gfqq_sync(gfqq)) {
		__gfq_update_io_thinktime(&cic->ttime, gfqd->gfq_slice_idle);
		__gfq_update_io_thinktime(&gfqq->service_tree->ttime,
			gfqd->gfq_slice_idle);
	}
#ifdef CONFIG_GFQ_GROUP_IOSCHED
	__gfq_update_io_thinktime(&gfqq->gfqg->ttime, gfqd->gfq_group_idle);
#endif
}

static void
gfq_update_io_seektime(struct gfq_data *gfqd, struct gfq_queue *gfqq,
		       struct request *rq)
{
	sector_t sdist = 0;
	sector_t n_sec = blk_rq_sectors(rq);
	if (gfqq->last_request_pos) {
		if (gfqq->last_request_pos < blk_rq_pos(rq))
			sdist = blk_rq_pos(rq) - gfqq->last_request_pos;
		else
			sdist = gfqq->last_request_pos - blk_rq_pos(rq);
	}

	gfqq->seek_history <<= 1;
	if (blk_queue_nonrot(gfqd->queue))
		gfqq->seek_history |= (n_sec < GFQQ_SECT_THR_NONROT);
	else
		gfqq->seek_history |= (sdist > GFQQ_SEEK_THR);
}

/*
 * Disable idle window if the process thinks too long or seeks so much that
 * it doesn't matter
 */
static void
gfq_update_idle_window(struct gfq_data *gfqd, struct gfq_queue *gfqq,
		       struct gfq_io_cq *cic)
{
	int old_idle, enable_idle;

	/*
	 * Don't idle for async or idle io prio class
	 */
	if (!gfq_gfqq_sync(gfqq) || gfq_class_idle(gfqq))
		return;

	enable_idle = old_idle = gfq_gfqq_idle_window(gfqq);

	if (gfqq->queued[0] + gfqq->queued[1] >= 4)
		gfq_mark_gfqq_deep(gfqq);

	if (gfqq->next_rq && (gfqq->next_rq->cmd_flags & REQ_NOIDLE))
		enable_idle = 0;
	else if (!atomic_read(&cic->icq.ioc->active_ref) ||
		 !gfqd->gfq_slice_idle ||
		 (!gfq_gfqq_deep(gfqq) && GFQQ_SEEKY(gfqq)))
		enable_idle = 0;
	else if (sample_valid(cic->ttime.ttime_samples)) {
		if (cic->ttime.ttime_mean > gfqd->gfq_slice_idle)
			enable_idle = 0;
		else
			enable_idle = 1;
	}

	if (old_idle != enable_idle) {
		gfq_log_gfqq(gfqd, gfqq, "idle=%d", enable_idle);
		if (enable_idle)
			gfq_mark_gfqq_idle_window(gfqq);
		else
			gfq_clear_gfqq_idle_window(gfqq);
	}
}

/*
 * Check if new_gfqq should preempt the currently active queue. Return 0 for
 * no or if we aren't sure, a 1 will cause a preempt.
 */
static bool
gfq_should_preempt(struct gfq_data *gfqd, struct gfq_queue *new_gfqq,
		   struct request *rq)
{
	struct gfq_queue *gfqq;

	gfqq = gfqd->active_queue;
	if (!gfqq)
		return false;

	if (gfq_class_idle(new_gfqq))
		return false;

	if (gfq_class_idle(gfqq))
		return true;

	/*
	 * Don't allow a non-RT request to preempt an ongoing RT gfqq timeslice.
	 */
	if (gfq_class_rt(gfqq) && !gfq_class_rt(new_gfqq))
		return false;

	/*
	 * if the new request is sync, but the currently running queue is
	 * not, let the sync request have priority.
	 */
	if (rq_is_sync(rq) && !gfq_gfqq_sync(gfqq))
		return true;

	if (new_gfqq->gfqg != gfqq->gfqg)
		return false;

	if (gfq_slice_used(gfqq))
		return true;

	/* Allow preemption only if we are idling on sync-noidle tree */
	if (gfqd->serving_wl_type == SYNC_NOIDLE_WORKLOAD &&
	    gfqq_type(new_gfqq) == SYNC_NOIDLE_WORKLOAD &&
	    new_gfqq->service_tree->count == 2 &&
	    RB_EMPTY_ROOT(&gfqq->sort_list))
		return true;

	/*
	 * So both queues are sync. Let the new request get disk time if
	 * it's a metadata request and the current queue is doing regular IO.
	 */
	if ((rq->cmd_flags & REQ_PRIO) && !gfqq->prio_pending)
		return true;

	/*
	 * Allow an RT request to pre-empt an ongoing non-RT gfqq timeslice.
	 */
	if (gfq_class_rt(new_gfqq) && !gfq_class_rt(gfqq))
		return true;

	/* An idle queue should not be idle now for some reason */
	if (RB_EMPTY_ROOT(&gfqq->sort_list) && !gfq_should_idle(gfqd, gfqq))
		return true;

	if (!gfqd->active_cic || !gfq_gfqq_wait_request(gfqq))
		return false;

	/*
	 * if this request is as-good as one we would expect from the
	 * current gfqq, let it preempt
	 */
	if (gfq_rq_close(gfqd, gfqq, rq))
		return true;

	return false;
}

/*
 * gfqq preempts the active queue. if we allowed preempt with no slice left,
 * let it have half of its nominal slice.
 */
static void gfq_preempt_queue(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	enum wl_type_t old_type = gfqq_type(gfqd->active_queue);

	gfq_log_gfqq(gfqd, gfqq, "preempt");
	gfq_slice_expired(gfqd, 1);

	/*
	 * workload type is changed, don't save slice, otherwise preempt
	 * doesn't happen
	 */
	if (old_type != gfqq_type(gfqq))
		gfqq->gfqg->saved_wl_slice = 0;

	/*
	 * Put the new queue at the front of the of the current list,
	 * so we know that it will be selected next.
	 */
	BUG_ON(!gfq_gfqq_on_rr(gfqq));

	gfq_service_tree_add(gfqd, gfqq, 1);

	gfqq->slice_end = 0;
	gfq_mark_gfqq_slice_new(gfqq);
}

/*
 * Called when a new fs request (rq) is added (to gfqq). Check if there's
 * something we should do about it
 */
static void
gfq_rq_enqueued(struct gfq_data *gfqd, struct gfq_queue *gfqq,
		struct request *rq)
{
	struct gfq_io_cq *cic = RQ_CIC(rq);

	gfqd->rq_queued++;
	if (rq->cmd_flags & REQ_PRIO)
		gfqq->prio_pending++;

	gfq_update_io_thinktime(gfqd, gfqq, cic);
	gfq_update_io_seektime(gfqd, gfqq, rq);
	gfq_update_idle_window(gfqd, gfqq, cic);

	gfqq->last_request_pos = blk_rq_pos(rq) + blk_rq_sectors(rq);

	if (gfqq == gfqd->active_queue) {
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
		if (gfq_gfqq_wait_request(gfqq)) {
			if (blk_rq_bytes(rq) > PAGE_CACHE_SIZE ||
			    gfqd->busy_queues > 1) {
				gfq_del_timer(gfqd, gfqq);
				gfq_clear_gfqq_wait_request(gfqq);
				__blk_run_queue(gfqd->queue);
			} else {
				gfqg_stats_update_idle_time(gfqq->gfqg);
				gfq_mark_gfqq_must_dispatch(gfqq);
			}
		}
	} else if (gfq_should_preempt(gfqd, gfqq, rq)) {
		/*
		 * not the active queue - expire current slice if it is
		 * idle and has expired it's mean thinktime or this new queue
		 * has some old slice time left and is of higher priority or
		 * this new queue is RT and the current one is BE
		 */
		gfq_preempt_queue(gfqd, gfqq);
		__blk_run_queue(gfqd->queue);
	}
}

// registerd to elevator_add_req_fn
static void gfq_insert_request(struct request_queue *q, struct request *rq)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct gfq_queue *gfqq = RQ_GFQQ(rq);

	gfq_log_gfqq(gfqd, gfqq, "insert_request");
	gfq_init_prio_data(gfqq, RQ_CIC(rq));

	rq->fifo_time = jiffies + gfqd->gfq_fifo_expire[rq_is_sync(rq)];
	list_add_tail(&rq->queuelist, &gfqq->fifo);
	gfq_add_rq_rb(rq);
	gfqg_stats_update_io_add(RQ_GFQG(rq), gfqd->serving_group,
				 rq->cmd_flags);
	gfq_rq_enqueued(gfqd, gfqq, rq);
}

/*
 * Update hw_tag based on peak queue depth over 50 samples under
 * sufficient load.
 */
static void gfq_update_hw_tag(struct gfq_data *gfqd)
{
	struct gfq_queue *gfqq = gfqd->active_queue;

	if (gfqd->rq_in_driver > gfqd->hw_tag_est_depth)
		gfqd->hw_tag_est_depth = gfqd->rq_in_driver;

	if (gfqd->hw_tag == 1)
		return;

	if (gfqd->rq_queued <= GFQ_HW_QUEUE_MIN &&
	    gfqd->rq_in_driver <= GFQ_HW_QUEUE_MIN)
		return;

	/*
	 * If active queue hasn't enough requests and can idle, gfq might not
	 * dispatch sufficient requests to hardware. Don't zero hw_tag in this
	 * case
	 */
	if (gfqq && gfq_gfqq_idle_window(gfqq) &&
	    gfqq->dispatched + gfqq->queued[0] + gfqq->queued[1] <
	    GFQ_HW_QUEUE_MIN && gfqd->rq_in_driver < GFQ_HW_QUEUE_MIN)
		return;

	if (gfqd->hw_tag_samples++ < 50)
		return;

	if (gfqd->hw_tag_est_depth >= GFQ_HW_QUEUE_MIN)
		gfqd->hw_tag = 1;
	else
		gfqd->hw_tag = 0;
}

static bool gfq_should_wait_busy(struct gfq_data *gfqd, struct gfq_queue *gfqq)
{
	struct gfq_io_cq *cic = gfqd->active_cic;

	/* If the queue already has requests, don't wait */
	if (!RB_EMPTY_ROOT(&gfqq->sort_list))
		return false;

	/* If there are other queues in the group, don't wait */
	if (gfqq->gfqg->nr_gfqq > 1)
		return false;

	/* the only queue in the group, but think time is big */
	if (gfq_io_thinktime_big(gfqd, &gfqq->gfqg->ttime, true))
		return false;

	if (gfq_slice_used(gfqq))
		return true;

	/* if slice left is less than think time, wait busy */
	if (cic && sample_valid(cic->ttime.ttime_samples)
	    && (gfqq->slice_end - jiffies < cic->ttime.ttime_mean))
		return true;

	/*
	 * If think times is less than a jiffy than ttime_mean=0 and above
	 * will not be true. It might happen that slice has not expired yet
	 * but will expire soon (4-5 ns) during select_queue(). To cover the
	 * case where think time is less than a jiffy, mark the queue wait
	 * busy if only 1 jiffy is left in the slice.
	 */
	if (gfqq->slice_end - jiffies == 1)
		return true;

	return false;
}

static void gfq_completed_request(struct request_queue *q, struct request *rq)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);
	struct gfq_data *gfqd = gfqq->gfqd;
	const int sync = rq_is_sync(rq);
	unsigned long now;

	now = jiffies;
	gfq_log_gfqq(gfqd, gfqq, "complete rqnoidle %d",
		     !!(rq->cmd_flags & REQ_NOIDLE));

	gfq_update_hw_tag(gfqd);

	WARN_ON(!gfqd->rq_in_driver);
	WARN_ON(!gfqq->dispatched);
	gfqd->rq_in_driver--;
	gfqq->dispatched--;
	(RQ_GFQG(rq))->dispatched--;
	gfqg_stats_update_completion(gfqq->gfqg, rq_start_time_ns(rq),
				     rq_io_start_time_ns(rq), rq->cmd_flags);

	gfqd->rq_in_flight[gfq_gfqq_sync(gfqq)]--;

	if (sync) {
		struct gfq_rb_root *st;

		RQ_CIC(rq)->ttime.last_end_request = now;

		if (gfq_gfqq_on_rr(gfqq))
			st = gfqq->service_tree;
		else
			st = st_for(gfqq->gfqg, gfqq_class(gfqq),
					gfqq_type(gfqq));

		st->ttime.last_end_request = now;
		if (!time_after(rq->start_time + gfqd->gfq_fifo_expire[1], now))
			gfqd->last_delayed_sync = now;
	}

#ifdef CONFIG_GFQ_GROUP_IOSCHED
	gfqq->gfqg->ttime.last_end_request = now;
#endif

	/*
	 * If this is the active queue, check if it needs to be expired,
	 * or if we want to idle in case it has no pending requests.
	 */
	if (gfqd->active_queue == gfqq) {
		const bool gfqq_empty = RB_EMPTY_ROOT(&gfqq->sort_list);

		if (gfq_gfqq_slice_new(gfqq)) {
			gfq_set_prio_slice(gfqd, gfqq);
			gfq_clear_gfqq_slice_new(gfqq);
		}

		/*
		 * Should we wait for next request to come in before we expire
		 * the queue.
		 */
		if (gfq_should_wait_busy(gfqd, gfqq)) {
			unsigned long extend_sl = gfqd->gfq_slice_idle;
			if (!gfqd->gfq_slice_idle)
				extend_sl = gfqd->gfq_group_idle;
			gfqq->slice_end = jiffies + extend_sl;
			gfq_mark_gfqq_wait_busy(gfqq);
			gfq_log_gfqq(gfqd, gfqq, "will busy wait");
		}

		/*
		 * Idling is not enabled on:
		 * - expired queues
		 * - idle-priority queues
		 * - async queues
		 * - queues with still some requests queued
		 * - when there is a close cooperator
		 */
		if (gfq_slice_used(gfqq) || gfq_class_idle(gfqq))
			gfq_slice_expired(gfqd, 1);
		else if (sync && gfqq_empty &&
			 !gfq_close_cooperator(gfqd, gfqq)) {
			gfq_arm_slice_timer(gfqd);
		}
	}

	if (!gfqd->rq_in_driver)
		gfq_schedule_dispatch(gfqd);
}

static inline int __gfq_may_queue(struct gfq_queue *gfqq)
{
	if (gfq_gfqq_wait_request(gfqq) && !gfq_gfqq_must_alloc_slice(gfqq)) {
		gfq_mark_gfqq_must_alloc_slice(gfqq);
		return ELV_MQUEUE_MUST;
	}

	return ELV_MQUEUE_MAY;
}

static int gfq_may_queue(struct request_queue *q, int rw)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct task_struct *tsk = current;
	struct gfq_io_cq *cic;
	struct gfq_queue *gfqq;

	/*
	 * don't force setup of a queue from here, as a call to may_queue
	 * does not necessarily imply that a request actually will be queued.
	 * so just lookup a possibly existing queue, or return 'may queue'
	 * if that fails
	 */
	cic = gfq_cic_lookup(gfqd, tsk->io_context);
	if (!cic)
		return ELV_MQUEUE_MAY;

	gfqq = cic_to_gfqq(cic, rw_is_sync(rw));
	if (gfqq) {
		gfq_init_prio_data(gfqq, cic);

		return __gfq_may_queue(gfqq);
	}

	return ELV_MQUEUE_MAY;
}

/*
 * queue lock held here
 */
static void gfq_put_request(struct request *rq)
{
	struct gfq_queue *gfqq = RQ_GFQQ(rq);

	if (gfqq) {
		const int rw = rq_data_dir(rq);

		BUG_ON(!gfqq->allocated[rw]);
		gfqq->allocated[rw]--;

		/* Put down rq reference on gfqg */
		gfqg_put(RQ_GFQG(rq));
		rq->elv.priv[0] = NULL;
		rq->elv.priv[1] = NULL;

		gfq_put_queue(gfqq);
	}
}

static struct gfq_queue *
gfq_merge_gfqqs(struct gfq_data *gfqd, struct gfq_io_cq *cic,
		struct gfq_queue *gfqq)
{
	gfq_log_gfqq(gfqd, gfqq, "merging with queue %p", gfqq->new_gfqq);
	cic_set_gfqq(cic, gfqq->new_gfqq, 1);
	gfq_mark_gfqq_coop(gfqq->new_gfqq);
	gfq_put_queue(gfqq);
	return cic_to_gfqq(cic, 1);
}

/*
 * Returns NULL if a new gfqq should be allocated, or the old gfqq if this
 * was the last process referring to said gfqq.
 */
static struct gfq_queue *
split_gfqq(struct gfq_io_cq *cic, struct gfq_queue *gfqq)
{
	if (gfqq_process_refs(gfqq) == 1) {
		gfqq->pid = current->pid;
		gfq_clear_gfqq_coop(gfqq);
		gfq_clear_gfqq_split_coop(gfqq);
		return gfqq;
	}

	cic_set_gfqq(cic, NULL, 1);

	gfq_put_cooperator(gfqq);

	gfq_put_queue(gfqq);
	return NULL;
}
/*
 * Allocate gfq data structures associated with this request.
 */
static int
gfq_set_request(struct request_queue *q, struct request *rq, struct bio *bio,
		gfp_t gfp_mask)
{
	struct gfq_data *gfqd = q->elevator->elevator_data;
	struct gfq_io_cq *cic = icq_to_cic(rq->elv.icq);
	const int rw = rq_data_dir(rq);
	const bool is_sync = rq_is_sync(rq);
	struct gfq_queue *gfqq;

	might_sleep_if(gfp_mask & __GFP_WAIT);

	spin_lock_irq(q->queue_lock);

	check_ioprio_changed(cic, bio);
	check_blkcg_changed(cic, bio);
new_queue:
	gfqq = cic_to_gfqq(cic, is_sync);
	if (!gfqq || gfqq == &gfqd->oom_gfqq) {
		gfqq = gfq_get_queue(gfqd, is_sync, cic, bio, gfp_mask);
		cic_set_gfqq(cic, gfqq, is_sync);
	} else {
		/*
		 * If the queue was seeky for too long, break it apart.
		 */
		if (gfq_gfqq_coop(gfqq) && gfq_gfqq_split_coop(gfqq)) {
			gfq_log_gfqq(gfqd, gfqq, "breaking apart gfqq");
			gfqq = split_gfqq(cic, gfqq);
			if (!gfqq)
				goto new_queue;
		}

		/*
		 * Check to see if this queue is scheduled to merge with
		 * another, closely cooperating queue.  The merging of
		 * queues happens here as it must be done in process context.
		 * The reference on new_gfqq was taken in merge_gfqqs.
		 */
		if (gfqq->new_gfqq)
			gfqq = gfq_merge_gfqqs(gfqd, cic, gfqq);
	}

	gfqq->allocated[rw]++;

	gfqq->ref++;
	gfqg_get(gfqq->gfqg);
	rq->elv.priv[0] = gfqq;
	rq->elv.priv[1] = gfqq->gfqg;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void gfq_kick_queue(struct work_struct *work)
{
	struct gfq_data *gfqd =
		container_of(work, struct gfq_data, unplug_work);
	struct request_queue *q = gfqd->queue;

	spin_lock_irq(q->queue_lock);
	__blk_run_queue(gfqd->queue);
	spin_unlock_irq(q->queue_lock);
}

/*
 * Timer running if the active_queue is currently idling inside its time slice
 */
static void gfq_idle_slice_timer(unsigned long data)
{
	struct gfq_data *gfqd = (struct gfq_data *) data;
	struct gfq_queue *gfqq;
	unsigned long flags;
	int timed_out = 1;

	gfq_log(gfqd, "idle timer fired");

	spin_lock_irqsave(gfqd->queue->queue_lock, flags);

	gfqq = gfqd->active_queue;
	if (gfqq) {
		timed_out = 0;

		/*
		 * We saw a request before the queue expired, let it through
		 */
		if (gfq_gfqq_must_dispatch(gfqq))
			goto out_kick;

		/*
		 * expired
		 */
		if (gfq_slice_used(gfqq))
			goto expire;

		/*
		 * only expire and reinvoke request handler, if there are
		 * other queues with pending requests
		 */
		if (!gfqd->busy_queues)
			goto out_cont;

		/*
		 * not expired and it has a request pending, let it dispatch
		 */
		if (!RB_EMPTY_ROOT(&gfqq->sort_list))
			goto out_kick;

		/*
		 * Queue depth flag is reset only when the idle didn't succeed
		 */
		gfq_clear_gfqq_deep(gfqq);
	}
expire:
	gfq_slice_expired(gfqd, timed_out);
out_kick:
	gfq_schedule_dispatch(gfqd);
out_cont:
	spin_unlock_irqrestore(gfqd->queue->queue_lock, flags);
}

static void gfq_shutdown_timer_wq(struct gfq_data *gfqd)
{
	del_timer_sync(&gfqd->idle_slice_timer);
	cancel_work_sync(&gfqd->unplug_work);
}

static void gfq_put_async_queues(struct gfq_data *gfqd)
{
	int i;

	for (i = 0; i < IOPRIO_BE_NR; i++) {
		if (gfqd->async_gfqq[0][i])
			gfq_put_queue(gfqd->async_gfqq[0][i]);
		if (gfqd->async_gfqq[1][i])
			gfq_put_queue(gfqd->async_gfqq[1][i]);
	}

	if (gfqd->async_idle_gfqq)
		gfq_put_queue(gfqd->async_idle_gfqq);
}

static void gfq_exit_queue(struct elevator_queue *e)
{
	struct gfq_data *gfqd = e->elevator_data;
	struct request_queue *q = gfqd->queue;

	gfq_shutdown_timer_wq(gfqd);

	spin_lock_irq(q->queue_lock);

	if (gfqd->active_queue)
		__gfq_slice_expired(gfqd, gfqd->active_queue, 0);

	gfq_put_async_queues(gfqd);

	spin_unlock_irq(q->queue_lock);

	gfq_shutdown_timer_wq(gfqd);

#ifdef CONFIG_GFQ_GROUP_IOSCHED
	blkcg_deactivate_policy(q, &blkcg_policy_gfq);
#else
	kfree(gfqd->root_group);
#endif
	kfree(gfqd);
}

static int gfq_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct gfq_data *gfqd;
	struct blkcg_gq *blkg __maybe_unused;
	int i, ret;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	gfqd = kzalloc_node(sizeof(*gfqd), GFP_KERNEL, q->node);
	if (!gfqd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = gfqd;

	gfqd->queue = q;
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);

	/* Init root service tree */
	gfqd->grp_service_tree = GFQ_RB_ROOT;

	/* Init root group and prefer root group over other groups by default */
#ifdef CONFIG_GFQ_GROUP_IOSCHED
	ret = blkcg_activate_policy(q, &blkcg_policy_gfq);
	if (ret)
		goto out_free;

	gfqd->root_group = blkg_to_gfqg(q->root_blkg);
#else
	ret = -ENOMEM;
	gfqd->root_group = kzalloc_node(sizeof(*gfqd->root_group),
					GFP_KERNEL, gfqd->queue->node);
	if (!gfqd->root_group)
		goto out_free;

	gfq_init_gfqg_base(gfqd->root_group);
#endif
	gfqd->root_group->weight = 2 * GFQ_WEIGHT_DEFAULT;
	gfqd->root_group->leaf_weight = 2 * GFQ_WEIGHT_DEFAULT;

	/*
	 * Not strictly needed (since RB_ROOT just clears the node and we
	 * zeroed gfqd on alloc), but better be safe in case someone decides
	 * to add magic to the rb code
	 */
	for (i = 0; i < GFQ_PRIO_LISTS; i++)
		gfqd->prio_trees[i] = RB_ROOT;

	/*
	 * Our fallback gfqq if gfq_find_alloc_queue() runs into OOM issues.
	 * Grab a permanent reference to it, so that the normal code flow
	 * will not attempt to free it.  oom_gfqq is linked to root_group
	 * but shouldn't hold a reference as it'll never be unlinked.  Lose
	 * the reference from linking right away.
	 */
	gfq_init_gfqq(gfqd, &gfqd->oom_gfqq, 1, 0);
	gfqd->oom_gfqq.ref++;

	spin_lock_irq(q->queue_lock);
	gfq_link_gfqq_gfqg(&gfqd->oom_gfqq, gfqd->root_group);
	gfqg_put(gfqd->root_group);
	spin_unlock_irq(q->queue_lock);

	init_timer(&gfqd->idle_slice_timer);
	gfqd->idle_slice_timer.function = gfq_idle_slice_timer;
	gfqd->idle_slice_timer.data = (unsigned long) gfqd;

	INIT_WORK(&gfqd->unplug_work, gfq_kick_queue);

	gfqd->gfq_quantum = gfq_quantum;
	gfqd->gfq_fifo_expire[0] = gfq_fifo_expire[0];
	gfqd->gfq_fifo_expire[1] = gfq_fifo_expire[1];
	gfqd->gfq_back_max = gfq_back_max;
	gfqd->gfq_back_penalty = gfq_back_penalty;
	gfqd->gfq_slice[0] = gfq_slice_async;
	gfqd->gfq_slice[1] = gfq_slice_sync;
	gfqd->gfq_target_latency = gfq_target_latency;
	gfqd->gfq_slice_async_rq = gfq_slice_async_rq;
	gfqd->gfq_slice_idle = gfq_slice_idle;
	gfqd->gfq_group_idle = gfq_group_idle;
	gfqd->gfq_latency = 1;
	gfqd->hw_tag = -1;
	/*
	 * we optimistically start assuming sync ops weren't delayed in last
	 * second, in order to have larger depth for async operations.
	 */
	gfqd->last_delayed_sync = jiffies - HZ;
	return 0;

out_free:
	kfree(gfqd);
	kobject_put(&eq->kobj);
	return ret;
}

/*
 * sysfs parts below -->
 */
static ssize_t
gfq_var_show(unsigned int var, char *page)
{
	return sprintf(page, "%u\n", var);
}

static ssize_t
gfq_var_store(unsigned int *var, const char *page, size_t count)
{
	char *p = (char *) page;

	*var = simple_strtoul(p, &p, 10);
	return count;
}

#define SHOW_FUNCTION(__FUNC, __VAR, __CONV)				\
static ssize_t __FUNC(struct elevator_queue *e, char *page)		\
{									\
	struct gfq_data *gfqd = e->elevator_data;			\
	unsigned int __data = __VAR;					\
	if (__CONV)							\
		__data = jiffies_to_msecs(__data);			\
	return gfq_var_show(__data, (page));				\
}
SHOW_FUNCTION(gfq_quantum_show, gfqd->gfq_quantum, 0);
SHOW_FUNCTION(gfq_fifo_expire_sync_show, gfqd->gfq_fifo_expire[1], 1);
SHOW_FUNCTION(gfq_fifo_expire_async_show, gfqd->gfq_fifo_expire[0], 1);
SHOW_FUNCTION(gfq_back_seek_max_show, gfqd->gfq_back_max, 0);
SHOW_FUNCTION(gfq_back_seek_penalty_show, gfqd->gfq_back_penalty, 0);
SHOW_FUNCTION(gfq_slice_idle_show, gfqd->gfq_slice_idle, 1);
SHOW_FUNCTION(gfq_group_idle_show, gfqd->gfq_group_idle, 1);
SHOW_FUNCTION(gfq_slice_sync_show, gfqd->gfq_slice[1], 1);
SHOW_FUNCTION(gfq_slice_async_show, gfqd->gfq_slice[0], 1);
SHOW_FUNCTION(gfq_slice_async_rq_show, gfqd->gfq_slice_async_rq, 0);
SHOW_FUNCTION(gfq_low_latency_show, gfqd->gfq_latency, 0);
SHOW_FUNCTION(gfq_target_latency_show, gfqd->gfq_target_latency, 1);
#undef SHOW_FUNCTION

#define STORE_FUNCTION(__FUNC, __PTR, MIN, MAX, __CONV)			\
static ssize_t __FUNC(struct elevator_queue *e, const char *page, size_t count)	\
{									\
	struct gfq_data *gfqd = e->elevator_data;			\
	unsigned int __data;						\
	int ret = gfq_var_store(&__data, (page), count);		\
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
STORE_FUNCTION(gfq_quantum_store, &gfqd->gfq_quantum, 1, UINT_MAX, 0);
STORE_FUNCTION(gfq_fifo_expire_sync_store, &gfqd->gfq_fifo_expire[1], 1,
		UINT_MAX, 1);
STORE_FUNCTION(gfq_fifo_expire_async_store, &gfqd->gfq_fifo_expire[0], 1,
		UINT_MAX, 1);
STORE_FUNCTION(gfq_back_seek_max_store, &gfqd->gfq_back_max, 0, UINT_MAX, 0);
STORE_FUNCTION(gfq_back_seek_penalty_store, &gfqd->gfq_back_penalty, 1,
		UINT_MAX, 0);
STORE_FUNCTION(gfq_slice_idle_store, &gfqd->gfq_slice_idle, 0, UINT_MAX, 1);
STORE_FUNCTION(gfq_group_idle_store, &gfqd->gfq_group_idle, 0, UINT_MAX, 1);
STORE_FUNCTION(gfq_slice_sync_store, &gfqd->gfq_slice[1], 1, UINT_MAX, 1);
STORE_FUNCTION(gfq_slice_async_store, &gfqd->gfq_slice[0], 1, UINT_MAX, 1);
STORE_FUNCTION(gfq_slice_async_rq_store, &gfqd->gfq_slice_async_rq, 1,
		UINT_MAX, 0);
STORE_FUNCTION(gfq_low_latency_store, &gfqd->gfq_latency, 0, 1, 0);
STORE_FUNCTION(gfq_target_latency_store, &gfqd->gfq_target_latency, 1, UINT_MAX, 1);
#undef STORE_FUNCTION

#define GFQ_ATTR(name) \
	__ATTR(name, S_IRUGO|S_IWUSR, gfq_##name##_show, gfq_##name##_store)

static struct elv_fs_entry gfq_attrs[] = {
	GFQ_ATTR(quantum),
	GFQ_ATTR(fifo_expire_sync),
	GFQ_ATTR(fifo_expire_async),
	GFQ_ATTR(back_seek_max),
	GFQ_ATTR(back_seek_penalty),
	GFQ_ATTR(slice_sync),
	GFQ_ATTR(slice_async),
	GFQ_ATTR(slice_async_rq),
	GFQ_ATTR(slice_idle),
	GFQ_ATTR(group_idle),
	GFQ_ATTR(low_latency),
	GFQ_ATTR(target_latency),
	__ATTR_NULL
};

static struct elevator_type iosched_gfq = {
	.ops = {
		.elevator_merge_fn = 		gfq_merge,
		.elevator_merged_fn =		gfq_merged_request,
		.elevator_merge_req_fn =	gfq_merged_requests,
		.elevator_allow_merge_fn =	gfq_allow_merge,
		.elevator_bio_merged_fn =	gfq_bio_merged,
		.elevator_dispatch_fn =		gfq_dispatch_requests,
		.elevator_add_req_fn =		gfq_insert_request,
		.elevator_activate_req_fn =	gfq_activate_request,
		.elevator_deactivate_req_fn =	gfq_deactivate_request,
		.elevator_completed_req_fn =	gfq_completed_request,
		.elevator_former_req_fn =	elv_rb_former_request,
		.elevator_latter_req_fn =	elv_rb_latter_request,
		.elevator_init_icq_fn =		gfq_init_icq,
		.elevator_exit_icq_fn =		gfq_exit_icq,
		.elevator_set_req_fn =		gfq_set_request,
		.elevator_put_req_fn =		gfq_put_request,
		.elevator_may_queue_fn =	gfq_may_queue,
		.elevator_init_fn =		gfq_init_queue,
		.elevator_exit_fn =		gfq_exit_queue,
	},
	.icq_size	=	sizeof(struct gfq_io_cq),
	.icq_align	=	__alignof__(struct gfq_io_cq),
	.elevator_attrs =	gfq_attrs,
	.elevator_name	=	"gfq",
	.elevator_owner =	THIS_MODULE,
};

#ifdef CONFIG_GFQ_GROUP_IOSCHED
static struct blkcg_policy blkcg_policy_gfq = {
	.pd_size		= sizeof(struct gfq_group),
	.cftypes		= gfq_blkcg_files,

	.pd_init_fn		= gfq_pd_init,
	.pd_offline_fn		= gfq_pd_offline,
	.pd_reset_stats_fn	= gfq_pd_reset_stats,
};
#endif


// module initialization
static int __init gfq_init(void)
{
	int ret;

	/*
	 * could be 0 on HZ < 1000 setups
	 */
	if (!gfq_slice_async)
		gfq_slice_async = 1;
	if (!gfq_slice_idle)
		gfq_slice_idle = 1;

#ifdef CONFIG_GFQ_GROUP_IOSCHED
	if (!gfq_group_idle)
		gfq_group_idle = 1;

	ret = blkcg_policy_register(&blkcg_policy_gfq);
	if (ret)
		return ret;
#else
	gfq_group_idle = 0;
#endif

	ret = -ENOMEM;
	gfq_pool = KMEM_CACHE(gfq_queue, 0);
	if (!gfq_pool)
		goto err_pol_unreg;

	ret = elv_register(&iosched_gfq);
	if (ret)
		goto err_free_pool;

	return 0;

err_free_pool:
	kmem_cache_destroy(gfq_pool);
err_pol_unreg:
#ifdef CONFIG_GFQ_GROUP_IOSCHED
	blkcg_policy_unregister(&blkcg_policy_gfq);
#endif
	return ret;
}

static void __exit gfq_exit(void)
{
#ifdef CONFIG_GFQ_GROUP_IOSCHED
	blkcg_policy_unregister(&blkcg_policy_gfq);
#endif
	elv_unregister(&iosched_gfq);
	kmem_cache_destroy(gfq_pool);
}

module_init(gfq_init);
module_exit(gfq_exit);

MODULE_AUTHOR("Kairi Okumura");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Group Fair Queueing IO scheduler");
