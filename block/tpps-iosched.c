/*
 *  TPPS, or Tiny Parallel Proportion disk Scheduler.
 *
 *  Based on ideas from Zhu Yanhai <gaoyang.zyh@taobao.com>
 *
 *  Copyright (C) 2013 Robin Dong <sanbai@taobao.com>
 */
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/jiffies.h>
#include <linux/rbtree.h>
#include <linux/ioprio.h>
#include <linux/blktrace_api.h>
#include "blk-cgroup.h"
#include "blk.h"

static struct kmem_cache *tpps_pool;

// tpps_queue -> tpps_group
//            -> tppg_node 

struct tpps_queue {
	/* reference count */
	int ref;
	/* parent tpps_data */
	struct tpps_data *tppd;
	/* tpps_group member */
	struct list_head tppg_node;
	/* sorted list of pending requests */
	struct list_head sort_list;
	struct tpps_group *tppg;
	pid_t pid;
	int online;
	int rq_queued;
};

struct tppg_stats {
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
};

/* This is per cgroup per device grouping structure */
struct tpps_group {
	/* must be the first member */
	struct blkg_policy_data pd;
	
	/* tpps_data member */
	struct list_head tppd_node;
	struct list_head *cur_dispatcher;

	unsigned int weight;
	unsigned int new_weight;
	unsigned int dev_weight;
	unsigned int leaf_weight;
	unsigned int new_leaf_weight;
	unsigned int dev_leaf_weight;

	bool needs_update;

	/*
	 * lists of queues with requests.
	 */
	struct list_head queue_list;
	int nr_tppq;
	int rq_queued;
	int rq_in_driver;

	struct tppg_stats stats;	/* stats for this tppg */
	struct tppg_stats dead_stats;	/* stats pushed from dead children */
};

struct tpps_io_cq {
	struct io_cq		icq;		/* must be the first member */
	struct tpps_queue	*tppq;
	uint64_t			blkcg_serial_nr;	/* the current blkcg ID */
};

struct tpps_data {
	struct request_queue *queue;
	struct tpps_group *root_group;

	/* List of tpps groups being managed on this device*/
	struct list_head group_list;

	unsigned int busy_queues;
	int dispatched;
	int rq_in_driver;

	struct work_struct unplug_work;

	/* Number of groups which are on blkcg->blkg_list */
	unsigned int nr_blkcg_linked_grps;

	unsigned total_weight;
};

static inline struct blkcg_gq *tppg_to_blkg(struct tpps_group *tppg)
{
	return pd_to_blkg(&tppg->pd);
}

#define tpps_log_tppq(tppd, tppq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(tppg_to_blkg((tppq)->tppg), __pbuf, sizeof(__pbuf));	\
	blk_add_trace_msg((tppd)->queue, "tpps%d %s " fmt, (tppq)->pid, \
			  __pbuf, ##args);				\
} while (0)

#define tpps_log_tppg(tppd, tppg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(tppg_to_blkg(tppg), __pbuf, sizeof(__pbuf));		\
	blk_add_trace_msg((tppd)->queue, "%s " fmt, __pbuf, ##args);	\
} while (0)
#define tpps_log(tppd, fmt, args...)	\
	blk_add_trace_msg((tppd)->queue, "tpps " fmt, ##args)

static inline struct tpps_io_cq *icq_to_tic(struct io_cq *icq)
{
	/* tic->icq is the first member, %NULL will convert to %NULL */
	return container_of(icq, struct tpps_io_cq, icq);
}

#define RQ_TIC(rq)	icq_to_tic((rq)->elv.icq)
#define RQ_TPPQ(rq)	(struct tpps_queue *) ((rq)->elv.priv[0])
#define RQ_TPPG(rq)	(struct tpps_group *) ((rq)->elv.priv[1])

#define TPPS_WEIGHT_DEFAULT	(500)
#define MIN_DISPATCH_RQ		(8)

static struct blkcg_policy blkcg_policy_tpps;

static inline struct tpps_group *pd_to_tppg(struct blkg_policy_data *pd)
{
	return pd ? container_of(pd, struct tpps_group, pd) : NULL;
}

static inline struct tpps_group *blkg_to_tppg(struct blkcg_gq *blkg)
{
	return pd_to_tppg(blkg_to_pd(blkg, &blkcg_policy_tpps));
}

static inline struct tpps_io_cq *
tpps_tic_lookup(struct tpps_data *tppd, struct io_context *ioc)
{
	if (ioc)
		return icq_to_tic(ioc_lookup_icq(ioc, tppd->queue));
	return NULL;
}

static inline struct tpps_queue *tic_to_tppq(struct tpps_io_cq *tic)
{
	return tic->tppq;
}

static inline void tic_set_tppq(struct tpps_io_cq *tic, struct tpps_queue *tppq)
{
	tic->tppq = tppq;
}

static inline struct tpps_data *tic_to_tppd(struct tpps_io_cq *tic)
{
	return tic->icq.q->elevator->elevator_data;
}

static inline void tppg_get(struct tpps_group *tppg)
{
	return blkg_get(tppg_to_blkg(tppg));
}

static inline void tppg_put(struct tpps_group *tppg)
{
	return blkg_put(tppg_to_blkg(tppg));
}

static inline void tppg_stats_update_io_add(struct tpps_group *tppg,
					    struct tpps_group *curr_tppg, int rw)
{
	blkg_rwstat_add(&tppg->stats.queued, rw, 1);
}

static inline void tppg_stats_update_io_remove(struct tpps_group *tppg, int rw)
{
	blkg_rwstat_add(&tppg->stats.queued, rw, -1);
}

static inline void tppg_stats_update_io_merged(struct tpps_group *tppg, int rw)
{
	blkg_rwstat_add(&tppg->stats.merged, rw, 1);
}

static inline void tppg_stats_update_dispatch(struct tpps_group *tppg,
					      uint64_t bytes, int rw)
{
	blkg_stat_add(&tppg->stats.sectors, bytes >> 9);
	blkg_rwstat_add(&tppg->stats.serviced, rw, 1);
	blkg_rwstat_add(&tppg->stats.service_bytes, rw, bytes);
}

static inline void tppg_stats_update_completion(struct tpps_group *tppg,
			uint64_t start_time, uint64_t io_start_time, int rw)
{
	struct tppg_stats *stats = &tppg->stats;
	unsigned long long now = sched_clock();

	if (time_after64(now, io_start_time))
		blkg_rwstat_add(&stats->service_time, rw, now - io_start_time);
	if (time_after64(io_start_time, start_time))
		blkg_rwstat_add(&stats->wait_time, rw,
				io_start_time - start_time);
}

static void tpps_del_queue(struct tpps_queue *tppq)
{
	struct tpps_data *tppd = tppq->tppd;
	struct tpps_group *tppg = tppq->tppg;

	if (!list_empty(&tppq->tppg_node)) {
		list_del_init(&tppq->tppg_node);
		tpps_log_tppq(tppd, tppq, "del queue\n");
		tppg->cur_dispatcher = NULL;
		tppq->tppg = NULL;
	}

	printk("%p nr_tppq:%d\n", tppg, tppg->nr_tppq);
	BUG_ON(tppg->nr_tppq < 1);
	tppg->nr_tppq--;
	if (!tppg->nr_tppq)
		tppd->total_weight -= tppg->pd.blkg->blkcg->cfq_weight;

	BUG_ON(!tppd->busy_queues);
	tppd->busy_queues--;
}

/*
 * task holds one reference to the queue, dropped when task exits. each rq
 * in-flight on this queue also holds a reference, dropped when rq is freed.
 *
 * Each tpps queue took a reference on the parent group. Drop it now.
 * queue lock must be held here.
 */
static void tpps_put_queue(struct tpps_queue *tppq)
{
	struct tpps_data *tppd = tppq->tppd;
	struct tpps_group *tppg;

	BUG_ON(tppq->ref <= 0);

	tppq->ref--;
	if (tppq->ref)
		return;

	tpps_log_tppq(tppd, tppq, "put_queue");
	BUG_ON(!list_empty(&tppq->sort_list));
	tppg = tppq->tppg;

	tpps_del_queue(tppq);
	kmem_cache_free(tpps_pool, tppq);
	tppg_put(tppg);
}

static void tpps_init_tppq(struct tpps_data *tppd, struct tpps_queue *tppq,
			  pid_t pid)
{
	INIT_LIST_HEAD(&tppq->tppg_node);
	INIT_LIST_HEAD(&tppq->sort_list);

	tppq->ref = 0;
	tppq->tppd = tppd;
	tppq->pid = pid;

}

static void tpps_link_tppq_tppg(struct tpps_queue *tppq,
		struct tpps_group *tppg)
{
	tppq->tppg = tppg;
	/* tppq reference on tppg */
	tppg_get(tppg);
}

static struct tpps_group *tpps_lookup_create_tppg(struct tpps_data *tppd,
						struct blkcg *blkcg)
{
	struct request_queue *q = tppd->queue;
	struct tpps_group *tppg = NULL;

	/* avoid lookup for the common case where there's no blkcg */
	if (blkcg == &blkcg_root) {
		tppg = tppd->root_group;
	} else {
		struct blkcg_gq *blkg;

		blkg = blkg_lookup_create(blkcg, q);
		if (!IS_ERR(blkg))
			tppg = blkg_to_tppg(blkg);
	}

	return tppg;
}

static struct tpps_queue *
tpps_find_alloc_queue(struct tpps_data *tppd, struct tpps_io_cq* tic, struct bio *bio,
		gfp_t gfp_mask)
{
	struct tpps_queue *tppq, *new_tppq = NULL;
	struct tpps_group *tppg;
	struct blkcg *blkcg;

retry:
	rcu_read_lock();

	blkcg = bio_blkcg(bio);
	tppg = tpps_lookup_create_tppg(tppd, blkcg);
	tppq = tic_to_tppq(tic);

	if (!tppq) {
		if (new_tppq) {
			tppq = new_tppq;
			new_tppq = NULL;
		} else if (gfp_mask & __GFP_WAIT) {
			rcu_read_unlock();
			spin_unlock_irq(tppd->queue->queue_lock);
			new_tppq = kmem_cache_alloc_node(tpps_pool,
					gfp_mask | __GFP_ZERO,
					tppd->queue->node);
			spin_lock_irq(tppd->queue->queue_lock);
			if (new_tppq)
				goto retry;
		} else
			tppq = kmem_cache_alloc_node(tpps_pool,
					gfp_mask | __GFP_ZERO,
					tppd->queue->node);

		if (tppq) {
			tpps_init_tppq(tppd, tppq, current->pid);
			tpps_link_tppq_tppg(tppq, tppg);
			tpps_log_tppq(tppd, tppq, "alloced");
		}
	}

	if (new_tppq)
		kmem_cache_free(tpps_pool, new_tppq);

	rcu_read_unlock();
	return tppq;
}

static struct tpps_queue *
tpps_get_queue(struct tpps_data *tppd, struct tpps_io_cq *tic, struct bio *bio,
			gfp_t gfp_mask)
{
	struct tpps_queue *tppq;

	tppq = tpps_find_alloc_queue(tppd, tic, bio, gfp_mask);
	tppq->ref++;
	return tppq;
}

/*
 * scheduler run of queue, if there are requests pending and no one in the
 * driver that will restart queueing
 */
static inline void tpps_schedule_dispatch(struct tpps_data *tppd)
{
	if (tppd->busy_queues) {
		tpps_log(tppd, "schedule dispatch");
		kblockd_schedule_work(&tppd->unplug_work);
	}
}

static void check_blkcg_changed(struct tpps_io_cq *tic, struct bio *bio)
{
	struct tpps_data *tppd = tic_to_tppd(tic);
	struct tpps_queue *tppq;
	uint64_t serial_nr;

	rcu_read_lock();
	serial_nr = bio_blkcg(bio)->css.serial_nr;	
	rcu_read_unlock();

	/*
	 * Check whether blkcg has changed.  The condition may trigger
	 * spuriously on a newly created tic but there's no harm.
	 */
	if (unlikely(!tppd) || likely(tic->blkcg_serial_nr == serial_nr))
		return;

	tppq = tic_to_tppq(tic);
	if (tppq) {
		/*
		 * Drop reference to sync queue. A new sync queue will be
		 * assigned in new group upon arrival of a fresh request.
		 */
		tpps_log_tppq(tppd, tppq, "changed cgroup");
		tic_set_tppq(tic, NULL);
		tpps_put_queue(tppq);
	}

	tic->blkcg_serial_nr = serial_nr;
}

static int
tpps_set_request(struct request_queue *q, struct request *rq, struct bio *bio,
			gfp_t gfp_mask)
{
	struct tpps_data *tppd = q->elevator->elevator_data;
	struct tpps_io_cq *tic = icq_to_tic(rq->elv.icq);
	struct tpps_queue *tppq;

	might_sleep_if(gfp_mask & __GFP_WAIT);

	spin_lock_irq(q->queue_lock);

	check_blkcg_changed(tic, bio);

	tppq = tic_to_tppq(tic);
	if (!tppq) {
		tppq = tpps_get_queue(tppd, tic, bio, gfp_mask);
		tic_set_tppq(tic, tppq);
	}

	tppq->ref++;
	tppg_get(tppq->tppg);
	rq->elv.priv[0] = tppq;
	rq->elv.priv[1] = tppq->tppg;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/*
 * queue lock held here
 */
static void tpps_put_request(struct request *rq)
{
	struct tpps_queue *tppq = RQ_TPPQ(rq);

	if (tppq) {
		WARN_ON(tppq->tppg != RQ_TPPG(rq));

		/* Put down rq reference on cfqg */
		tppg_put(RQ_TPPG(rq));
		rq->elv.priv[0] = NULL;
		rq->elv.priv[1] = NULL;

		tpps_put_queue(tppq);
	}
}

static void
tpps_update_group_weight(struct tpps_group *tppg)
{
	if (tppg->needs_update) {
		tppg->weight = tppg->new_weight;
		tppg->needs_update = false;
	}
}

static void tpps_add_queue(struct tpps_data *tppd, struct tpps_queue *tppq)
{
	struct tpps_group *tppg;

	if (!tppq->online) {
		tppq->online = 1;
		tppg = tppq->tppg;
		tpps_log_tppq(tppd, tppq, "add queue");
		tppg->nr_tppq++;
		tppd->busy_queues++;
		list_add(&tppq->tppg_node, &tppg->queue_list);
		printk("add tppq %p to %p\n", tppq, tppg);
		tpps_update_group_weight(tppg);
		if (tppg->nr_tppq <= 1) {
			tppd->total_weight += tppg->pd.blkg->blkcg->cfq_weight;
			list_add(&tppg->tppd_node, &tppd->group_list);
			printk("twt:%u, wt:%u %u %d %p\n", tppd->total_weight, tppg->weight,
					tppg->pd.blkg->blkcg->cfq_weight,
					tppg->nr_tppq,
					tppg);
		}
	}
}

static void tpps_insert_request(struct request_queue *q, struct request *rq)
{
	struct tpps_data *tppd = q->elevator->elevator_data;
	struct tpps_queue *tppq = RQ_TPPQ(rq);

	tpps_log_tppq(tppd, tppq, "insert_request");

	list_add_tail(&rq->queuelist, &tppq->sort_list);
	tppq->rq_queued++;
	tppq->tppg->rq_queued++;
	tppd->dispatched++;
	tpps_add_queue(tppd, tppq);
	tppg_stats_update_io_add(RQ_TPPG(rq), tppq->tppg, rq->cmd_flags);
}

static void tpps_remove_request(struct request *rq)
{
	struct tpps_queue *tppq = RQ_TPPQ(rq);

	list_del_init(&rq->queuelist);
	tppq->rq_queued--;
	tppq->tppg->rq_queued--;
	tppg_stats_update_io_remove(RQ_TPPG(rq), rq->cmd_flags);
}

/*
 * Move request from internal lists to the request queue dispatch list.
 */
static int tpps_dispatch_insert(struct request_queue *q,
				struct tpps_queue *tppq)
{
	struct list_head *rbnext = tppq->sort_list.next;
	struct request *rq;

	if (rbnext == &tppq->sort_list)
		return 0;

	rq = rq_entry_fifo(rbnext);
	tpps_remove_request(rq);
	elv_dispatch_sort(q, rq);
	tppg_stats_update_dispatch(tppq->tppg, blk_rq_bytes(rq), rq->cmd_flags);
	return 1;
}

static int tpps_dispatch_requests_nr(struct tpps_data *tppd,
				struct tpps_queue *tppq, int count)
{
	int cnt = 0, ret;

	if (!tppq->rq_queued)
		return cnt;

	do {
		ret = tpps_dispatch_insert(tppd->queue, tppq);
		if (ret) {
			cnt++;
			tppd->dispatched--;
		}
	} while (ret && cnt < count);

	return cnt;
}

static int tpps_dispatch_requests(struct request_queue *q, int force)
{
	struct tpps_data *tppd = q->elevator->elevator_data;
	struct tpps_group *tppg, *group_n;
	struct tpps_queue *tppq;
	struct list_head *next;
	int count = 0, total = 0, ret;
	int quota, grp_quota;

	if (!tppd->total_weight)
		return 0;

	quota = q->nr_requests - tppd->rq_in_driver;
	if (quota < MIN_DISPATCH_RQ && !force)
		return 0;

	list_for_each_entry_safe(tppg, group_n, &tppd->group_list, tppd_node) {
		if (!tppg->nr_tppq)
			continue;
		grp_quota = (quota * tppg->pd.blkg->blkcg->cfq_weight
					/ tppd->total_weight) - tppg->rq_in_driver;
		tpps_log_tppg(tppd, tppg,
			"nr:%d, wt:%u total_wt:%u in_driver:%d %d quota:%d grp_quota:%d",
			tppg->nr_tppq, tppg->pd.blkg->blkcg->cfq_weight,
			tppd->total_weight, tppg->rq_in_driver, tppg->rq_queued,
			quota, grp_quota);
		if (grp_quota <= 0 && !force)
			continue;
		BUG_ON(tppg->queue_list.next == &tppg->queue_list);
		if (!tppg->cur_dispatcher)
			tppg->cur_dispatcher = tppg->queue_list.next;
		next = tppg->cur_dispatcher;
		count = 0;
		do {
			tppq = list_entry(next, struct tpps_queue, tppg_node);
			tpps_log_tppq(tppd, tppq, "tppq: %d\n", tppq->rq_queued);
			if (force)
				ret = tpps_dispatch_requests_nr(tppd, tppq, -1);
			else
				ret = tpps_dispatch_requests_nr(tppd, tppq, 1);
			count += ret;
			total += ret;
			next = next->next;
			if (next == &tppg->queue_list)
				next = tppg->queue_list.next;
			if (count >= grp_quota && !force) {
				tppg->cur_dispatcher = next;
				break;
			}
			BUG_ON(tppg->cur_dispatcher == &tppg->queue_list);
		} while (next != tppg->cur_dispatcher);
	}
	return total > 0;
}

static void tpps_kick_queue(struct work_struct *work)
{
	struct tpps_data *tppd =
		container_of(work, struct tpps_data, unplug_work);
	struct request_queue *q = tppd->queue;

	spin_lock_irq(q->queue_lock);
	__blk_run_queue(q);
	spin_unlock_irq(q->queue_lock);
}

static void tpps_init_tppg_base(struct tpps_group *tppg)
{
	INIT_LIST_HEAD(&tppg->tppd_node);
	INIT_LIST_HEAD(&tppg->queue_list);
	tppg->cur_dispatcher = NULL;

}

static int tpps_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct tpps_data *tppd;
	struct tpps_group *tppg;
	int ret;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if(!eq)
		return -ENOMEM;
		
	tppd = kzalloc_node(sizeof(*tppd), GFP_KERNEL, q->node);
	
	if (!tppd) {
		kobject_put(&eq->kobj);		
		return -ENOMEM;
	}

	eq->elevator_data = tppd;
	
	tppd->queue = q;
	spin_lock_irq(q->queue_lock);	
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	

	INIT_LIST_HEAD(&tppd->group_list);

	ret = blkcg_activate_policy(q, &blkcg_policy_tpps);
	if (ret)
		goto out_free;

	/* Init root group */
	tppd->root_group = blkg_to_tppg(q->root_blkg);
	tppg = tppd->root_group;
	tpps_init_tppg_base(tppg);

	/* Give preference to root group over other groups */
	tppg->weight = 2 * TPPS_WEIGHT_DEFAULT;
	tppg->leaf_weight = 2 * TPPS_WEIGHT_DEFAULT;

	INIT_WORK(&tppd->unplug_work, tpps_kick_queue);

	return 0;

out_free:
	kfree(tppd);
	kobject_put(&eq->kobj);
	return ret;
}

static void tpps_exit_queue(struct elevator_queue *e)
{
	struct tpps_data *tppd = e->elevator_data;
	struct request_queue *q = tppd->queue;

	cancel_work_sync(&tppd->unplug_work);

	blkcg_deactivate_policy(q, &blkcg_policy_tpps);
	kfree(tppd->root_group);
	kfree(tppd);
}

static void tpps_activate_request(struct request_queue *q, struct request *rq)
{
	struct tpps_queue *tppq = RQ_TPPQ(rq);
	struct tpps_data *tppd = q->elevator->elevator_data;
	tppd->rq_in_driver++;
	tppq->tppg->rq_in_driver++;
	tpps_log_tppq(tppd, RQ_TPPQ(rq), "activate rq, drv=%d",
						tppd->rq_in_driver);
}

static void tpps_deactivate_request(struct request_queue *q, struct request *rq)
{
	struct tpps_queue *tppq = RQ_TPPQ(rq);
	struct tpps_data *tppd = q->elevator->elevator_data;

	WARN_ON(!tppd->rq_in_driver);
	tppd->rq_in_driver--;
	tppq->tppg->rq_in_driver--;
	tpps_log_tppq(tppd, RQ_TPPQ(rq), "deactivate rq, drv=%d",
						tppd->rq_in_driver);
}

static void tpps_completed_request(struct request_queue *q, struct request *rq)
{
	struct tpps_queue *tppq = RQ_TPPQ(rq);
	struct tpps_data *tppd = tppq->tppd;

	WARN_ON(!tppq);
	WARN_ON(tppq->tppg != RQ_TPPG(rq));

	tpps_log_tppq(tppd, tppq, "complete rqnoidle %d",
			!!(rq->cmd_flags & REQ_NOIDLE));
	WARN_ON(!tppd->rq_in_driver);
	tppd->rq_in_driver--;
	tppq->tppg->rq_in_driver--;
	tppg_stats_update_completion(tppq->tppg,
			rq_start_time_ns(rq), rq_io_start_time_ns(rq), rq->cmd_flags);

	if (!tppd->rq_in_driver)
		tpps_schedule_dispatch(tppd);
}

static void
tpps_merged_request(struct request_queue *q, struct request *rq, int type)
{
	if (type == ELEVATOR_FRONT_MERGE) {
		struct tpps_queue *tppq = RQ_TPPQ(rq);
		list_del_init(&rq->queuelist);
		tppq->rq_queued--;
		tppg_stats_update_io_remove(RQ_TPPG(rq), rq->cmd_flags);
		list_add_tail(&rq->queuelist, &tppq->sort_list);
		tppq->rq_queued++;
		tppg_stats_update_io_add(RQ_TPPG(rq), tppq->tppg, rq->cmd_flags);
	}
}

static void
tpps_merged_requests(struct request_queue *q, struct request *rq,
			struct request *next)
{
	tpps_remove_request(next);
	tppg_stats_update_io_merged(RQ_TPPG(rq), rq->cmd_flags);
}

static void tpps_init_icq(struct io_cq *icq)
{ }

static void tpps_exit_icq(struct io_cq *icq)
{
	struct tpps_io_cq *tic = icq_to_tic(icq);

	if (tic->tppq) {
		tpps_put_queue(tic->tppq);
		tic->tppq = NULL;
	}
}

static struct elevator_type iosched_tpps = {
	.ops = {
		.elevator_merged_fn =		tpps_merged_request,
		.elevator_merge_req_fn =	tpps_merged_requests,
		.elevator_dispatch_fn =		tpps_dispatch_requests,
		.elevator_add_req_fn =		tpps_insert_request,
		.elevator_activate_req_fn = 	tpps_activate_request,
		.elevator_deactivate_req_fn = 	tpps_deactivate_request,
		.elevator_completed_req_fn =	tpps_completed_request,
		.elevator_init_icq_fn =		tpps_init_icq,
		.elevator_exit_icq_fn =		tpps_exit_icq,
		.elevator_set_req_fn =		tpps_set_request,
		.elevator_put_req_fn =		tpps_put_request,
		.elevator_init_fn =		tpps_init_queue,
		.elevator_exit_fn =		tpps_exit_queue,
	},
	.icq_size		= sizeof(struct tpps_io_cq),
	.icq_align		= __alignof__(struct tpps_io_cq),
	.elevator_name	=	"tpps",
	.elevator_owner =	THIS_MODULE,
};

static u64 tppg_prfill_weight_device(struct seq_file *sf,
				     struct blkg_policy_data *pd, int off)
{
	struct tpps_group *tppg = pd_to_tppg(pd);

	if (!tppg->dev_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, tppg->dev_weight);
}

static int tppg_print_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tppg_prfill_weight_device, &blkcg_policy_tpps,
					  0, false);
	return 0;
}

static u64 tppg_prfill_leaf_weight_device(struct seq_file *sf,
					  struct blkg_policy_data *pd, int off)
{
	struct tpps_group *tppg = pd_to_tppg(pd);

	if (!tppg->dev_leaf_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, tppg->dev_leaf_weight);
}

static int tppg_print_leaf_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tppg_prfill_leaf_weight_device, &blkcg_policy_tpps,
					  0, false);
	return 0;
}

static int tppg_print_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->cfq_weight); 
	return 0;
}

static int tppg_print_leaf_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n",  css_to_blkcg(seq_css(sf))->cfq_leaf_weight);
	return 0;
}

			   
static ssize_t __tppg_set_weight_device(struct kernfs_open_file *of,
										char *buf, size_t nbytes, loff_t off,
										bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(of_css(of));	
	struct blkg_conf_ctx ctx;
	struct tpps_group *tppg;
	int ret;

	ret = blkg_conf_prep(blkcg, &blkcg_policy_tpps, buf, &ctx);
	if (ret)
		return ret;

	ret = -EINVAL;
	tppg = blkg_to_tppg(ctx.blkg);
	if (!ctx.v || (ctx.v >= CFQ_WEIGHT_MIN && ctx.v <= CFQ_WEIGHT_MAX)) {
		if (!is_leaf_weight) {
			tppg->dev_weight = ctx.v;
			tppg->new_weight = ctx.v ?: blkcg->cfq_weight;
		} else {
			tppg->dev_leaf_weight = ctx.v;
			tppg->new_leaf_weight = ctx.v ?: blkcg->cfq_leaf_weight;
		}
		ret = 0;
	}

	blkg_conf_finish(&ctx);
	return ret;
}			   

static ssize_t tppg_set_weight_device(struct kernfs_open_file *of,
								  char *buf, size_t nbytes, loff_t off)
{
	return __tppg_set_weight_device(of, buf, nbytes, off, false);
}

static ssize_t tppg_set_leaf_weight_device(struct kernfs_open_file *of,
										   char *buf, size_t nbytes, loff_t off)
{
	return __tppg_set_weight_device(of, buf, nbytes, off, true);
}

static int __tpps_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
							 u64 val, bool is_leaf_weight)
{
//	struct blkcg *blkcg = cgroup_to_blkcg(cgrp);
	struct blkcg *blkcg = css_to_blkcg(css);
		struct blkcg_gq *blkg;

	if (val < CFQ_WEIGHT_MIN || val > CFQ_WEIGHT_MAX)
		return -EINVAL;

	spin_lock_irq(&blkcg->lock);

	if (!is_leaf_weight)
		blkcg->cfq_weight = val;
	else
		blkcg->cfq_leaf_weight = val;

	hlist_for_each_entry(blkg, &blkcg->blkg_list, blkcg_node) {
		struct tpps_group *tppg = blkg_to_tppg(blkg);

		if (!tppg)
			continue;

		if (!is_leaf_weight) {
			if (!tppg->dev_weight)
				tppg->new_weight = blkcg->cfq_weight;
		} else {
			if (!tppg->dev_leaf_weight)
				tppg->new_leaf_weight = blkcg->cfq_leaf_weight;
		}
	}

	spin_unlock_irq(&blkcg->lock);
	return 0;
}

static int tpps_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
							u64 val)
{
	return __tpps_set_weight(css, cft, val, false);
}

static int tpps_set_leaf_weight(struct cgroup_subsys_state *css,
								struct cftype *cft, u64 val)
{
	return __tpps_set_weight(css, cft, val, true);
}

/* offset delta from tppg->stats to tppg->dead_stats */
static const int dead_stats_off_delta = offsetof(struct tpps_group, dead_stats) -
					offsetof(struct tpps_group, stats);

/* to be used by recursive prfill, sums live and dead rwstats recursively */
static struct blkg_rwstat tppg_rwstat_pd_recursive_sum(struct blkg_policy_data *pd,
						       int off)
{
	struct blkg_rwstat a, b;

	a = blkg_rwstat_recursive_sum(pd, off);
	b = blkg_rwstat_recursive_sum(pd, off + dead_stats_off_delta);
	blkg_rwstat_merge(&a, &b);
	return a;
}

/* to be used by recursive prfill, sums live and dead stats recursively */
static u64 tppg_stat_pd_recursive_sum(struct blkg_policy_data *pd, int off)
{
	u64 sum = 0;

	sum += blkg_stat_recursive_sum(pd, off);
	sum += blkg_stat_recursive_sum(pd, off + dead_stats_off_delta);
	return sum;
}

static int tppg_print_stat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_stat,
					  &blkcg_policy_tpps, seq_cft(sf)->private, false);
	return 0;
}

static int tppg_print_rwstat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_rwstat,
					  &blkcg_policy_tpps,  seq_cft(sf)->private, true);
	return 0;
}

static u64 tppg_prfill_stat_recursive(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	u64 sum = tppg_stat_pd_recursive_sum(pd, off);

	return __blkg_prfill_u64(sf, pd, sum);
}

static u64 tppg_prfill_rwstat_recursive(struct seq_file *sf,
					struct blkg_policy_data *pd, int off)
{
	struct blkg_rwstat sum = tppg_rwstat_pd_recursive_sum(pd, off);

	return __blkg_prfill_rwstat(sf, pd, &sum);
}

static int tppg_print_stat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tppg_prfill_stat_recursive,  &blkcg_policy_tpps,
					  seq_cft(sf)->private, false);
	return 0;
}

static int tppg_print_rwstat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tppg_prfill_rwstat_recursive,  &blkcg_policy_tpps,
					  seq_cft(sf)->private, true);
	return 0;
}

static struct cftype tpps_blkcg_files[] = {
	/* on root, weight is mapped to leaf_weight */
	{
		.name = "tpps.weight_device",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = tppg_print_leaf_weight_device,
		.write = tppg_set_leaf_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpps.weight",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = tppg_print_leaf_weight,
		.write_u64 = tpps_set_leaf_weight,
	},

	/* no such mapping necessary for !roots */
	{
		.name = "tpps.weight_device",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = tppg_print_weight_device,
		.write = tppg_set_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpps.weight",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = tppg_print_weight,
		.write_u64 = tpps_set_weight,
	},

	{
		.name = "tpps.leaf_weight_device",
		.seq_show = tppg_print_leaf_weight_device,
		.write = tppg_set_leaf_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpps.leaf_weight",
		.seq_show = tppg_print_leaf_weight,
		.write_u64 = tpps_set_leaf_weight,
	},

	/* statistics, covers only the tasks in the tppg */
	{
		.name = "tpps.time",
		.private = offsetof(struct tpps_group, stats.time),
		.seq_show = tppg_print_stat,
	},
	{
		.name = "tpps.sectors",
		.private = offsetof(struct tpps_group, stats.sectors),
		.seq_show = tppg_print_stat,
	},
	{
		.name = "tpps.io_service_bytes",
		.private = offsetof(struct tpps_group, stats.service_bytes),
		.seq_show = tppg_print_rwstat,
	},
	{
		.name = "tpps.io_serviced",
		.private = offsetof(struct tpps_group, stats.serviced),
		.seq_show= tppg_print_rwstat,
	},
	{
		.name = "tpps.io_service_time",
		.private = offsetof(struct tpps_group, stats.service_time),
		.seq_show = tppg_print_rwstat,
	},
	{
		.name = "tpps.io_wait_time",
		.private = offsetof(struct tpps_group, stats.wait_time),
		.seq_show = tppg_print_rwstat,
	},
	{
		.name = "tpps.io_merged",
		.private = offsetof(struct tpps_group, stats.merged),
		.seq_show = tppg_print_rwstat,
	},
	{
		.name = "tpps.io_queued",
		.private = offsetof(struct tpps_group, stats.queued),
		.seq_show = tppg_print_rwstat,
	},

	/* the same statictics which cover the tppg and its descendants */
	{
		.name = "tpps.time_recursive",
		.private = offsetof(struct tpps_group, stats.time),
		.seq_show = tppg_print_stat_recursive,
	},
	{
		.name = "tpps.sectors_recursive",
		.private = offsetof(struct tpps_group, stats.sectors),
		.seq_show = tppg_print_stat_recursive,
	},
	{
		.name = "tpps.io_service_bytes_recursive",
		.private = offsetof(struct tpps_group, stats.service_bytes),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{
		.name = "tpps.io_serviced_recursive",
		.private = offsetof(struct tpps_group, stats.serviced),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{
		.name = "tpps.io_service_time_recursive",
		.private = offsetof(struct tpps_group, stats.service_time),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{
		.name = "tpps.io_wait_time_recursive",
		.private = offsetof(struct tpps_group, stats.wait_time),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{
		.name = "tpps.io_merged_recursive",
		.private = offsetof(struct tpps_group, stats.merged),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{
		.name = "tpps.io_queued_recursive",
		.private = offsetof(struct tpps_group, stats.queued),
		.seq_show = tppg_print_rwstat_recursive,
	},
	{ }	/* terminate */
};


static void tppg_stats_init(struct tppg_stats *stats)
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
// operation
static void tpps_pd_init(struct blkcg_gq *blkg)
{
	struct tpps_group *tppg = blkg_to_tppg(blkg);

	tpps_init_tppg_base(tppg);
	tppg->weight = blkg->blkcg->cfq_weight;
	tppg->leaf_weight = blkg->blkcg->cfq_leaf_weight;
	// TMP
	tppg_stats_init(&tppg->stats);
	tppg_stats_init(&tppg->dead_stats);
}

static inline struct tpps_group *tppg_parent(struct tpps_group *tppg)
{
	struct blkcg_gq *pblkg = tppg_to_blkg(tppg)->parent;

	return pblkg ? blkg_to_tppg(pblkg) : NULL;
}




static void tppg_stats_reset(struct tppg_stats *stats)
{
	/* queued stats shouldn't be cleared */
	blkg_rwstat_reset(&stats->service_bytes);
	blkg_rwstat_reset(&stats->serviced);
	blkg_rwstat_reset(&stats->merged);
	blkg_rwstat_reset(&stats->service_time);
	blkg_rwstat_reset(&stats->wait_time);
	blkg_stat_reset(&stats->time);
#ifdef CONFIG_DEBUG_BLK_CGROUP //if enabled... error why?
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
static void tppg_stats_merge(struct tppg_stats *to, struct tppg_stats *from)
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

static void tppg_stats_xfer_dead(struct tpps_group *tppg)
{
	struct tpps_group *parent = tppg_parent(tppg);

	lockdep_assert_held(tppg_to_blkg(tppg)->q->queue_lock);

	if (unlikely(!parent))
		return;

	tppg_stats_merge(&parent->dead_stats, &tppg->stats);
	tppg_stats_merge(&parent->dead_stats, &tppg->dead_stats);
	tppg_stats_reset(&tppg->stats);
	tppg_stats_reset(&tppg->dead_stats);
}

static void tpps_pd_offline(struct blkcg_gq *blkg)
{
	struct tpps_group *tppg = blkg_to_tppg(blkg);
	/*
	 * @blkg is going offline and will be ignored by
	 * blkg_[rw]stat_recursive_sum().  Transfer stats to the parent so
	 * that they don't get lost.  If IOs complete after this point, the
	 * stats for them will be lost.  Oh well...
	 */
	tppg_stats_xfer_dead(tppg);

	if (!list_empty(&tppg->tppd_node))
		list_del_init(&tppg->tppd_node);

	//BUG_ON(!list_empty(&(tppg->queue_list)));
}

static void tpps_pd_reset_stats(struct blkcg_gq *blkg)
{
	struct tpps_group *tppg = blkg_to_tppg(blkg);

	tppg_stats_reset(&tppg->stats);
	tppg_stats_reset(&tppg->dead_stats);
}

static struct blkcg_policy blkcg_policy_tpps = {
	.pd_size			= sizeof(struct tpps_group), //policy specific private data size
	.cftypes			= tpps_blkcg_files, // cgroup files for the policy
	/* operations */
	.pd_init_fn			= tpps_pd_init, 
	.pd_offline_fn		= tpps_pd_offline,
	.pd_reset_stats_fn	= tpps_pd_reset_stats,
};

// MEMO:reference to cfq-iosched.c __init
static int __init tpps_init(void)
{
	int ret;

	ret = blkcg_policy_register(&blkcg_policy_tpps); 
	if (ret)
		return ret;

	ret = -ENOMEM;
	tpps_pool = KMEM_CACHE(tpps_queue, 0);
	if (!tpps_pool)
		goto err_pol_unreg;

	ret = elv_register(&iosched_tpps);
	if (ret)
		goto err_free_pool;

	return 0;

err_free_pool:
	kmem_cache_destroy(tpps_pool);
err_pol_unreg:
	blkcg_policy_unregister(&blkcg_policy_tpps);
	return ret;
}

static void __exit tpps_exit(void)
{
	blkcg_policy_unregister(&blkcg_policy_tpps);
	elv_unregister(&iosched_tpps);
	kmem_cache_destroy(tpps_pool);
}

module_init(tpps_init);
module_exit(tpps_exit);

MODULE_AUTHOR("Robin Dong");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tiny Parallel Proportion io Scheduler");
