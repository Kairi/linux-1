/*
 *  TPGS, or Tiny Parallel Group disk Scheduler.
 *
 *  Based on ideas from TPPS io scheduler.
 *
 *  Copyright (C) 2015 Kairi OKUMURA <kairi199088@gmail.com>
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

static struct kmem_cache *tpgs_pool;

struct tpgs_queue {
	/* reference count */
	int ref;
	/* parent tpgs_data */
	struct tpgs_data *tppd;
	/* tpgs_group member */
	struct list_head tpgg_node;
	/* sorted list of pending requests */
	struct list_head sort_list; // reqs
	struct tpgs_group *tpgg;
	pid_t pid;
	int online;
	int rq_queued; // req count
};

struct tpgg_stats {
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
};

/* This is per cgroup per device grouping structure */
struct tpgs_group {
	/* must be the first member */
	struct blkg_policy_data pd;
	
	/* tpgs_data member */
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
	int nr_tpgq;
	int rq_queued;
	int rq_in_driver;

	struct tpgg_stats stats;	/* stats for this tpgg */
	struct tpgg_stats dead_stats;	/* stats pushed from dead children */
};

struct tpgs_io_cq {
	struct io_cq		icq;		/* must be the first member */
	struct tpgs_queue	*tpgq;
	uint64_t			blkcg_serial_nr;	/* the current blkcg ID */
};

struct tpgs_data {
	struct request_queue *queue;
	struct tpgs_group *root_group;

	/* List of tpgs groups being managed on this device*/
	struct list_head group_list;

	unsigned int busy_queues;
	int dispatched;
	int rq_in_driver;

	struct work_struct unplug_work;

	/* Number of groups which are on blkcg->blkg_list */
	unsigned int nr_blkcg_linked_grps;

	unsigned total_weight;
};

static inline struct blkcg_gq *tpgg_to_blkg(struct tpgs_group *tpgg)
{
	return pd_to_blkg(&tpgg->pd);
}

#define tpgs_log_tpgq(tppd, tpgq, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(tpgg_to_blkg((tpgq)->tpgg), __pbuf, sizeof(__pbuf));	\
	blk_add_trace_msg((tppd)->queue, "tpgs%d %s " fmt, (tpgq)->pid, \
			  __pbuf, ##args);				\
} while (0)

#define tpgs_log_tpgg(tppd, tpgg, fmt, args...)	do {			\
	char __pbuf[128];						\
									\
	blkg_path(tpgg_to_blkg(tpgg), __pbuf, sizeof(__pbuf));		\
	blk_add_trace_msg((tppd)->queue, "%s " fmt, __pbuf, ##args);	\
} while (0)
#define tpgs_log(tppd, fmt, args...)	\
	blk_add_trace_msg((tppd)->queue, "tpgs " fmt, ##args)

static inline struct tpgs_io_cq *icq_to_tic(struct io_cq *icq)
{
	/* tic->icq is the first member, %NULL will convert to %NULL */
	return container_of(icq, struct tpgs_io_cq, icq);
}

#define RQ_TIC(rq)	icq_to_tic((rq)->elv.icq)
#define RQ_TPGQ(rq)	(struct tpgs_queue *) ((rq)->elv.priv[0])
#define RQ_TPGG(rq)	(struct tpgs_group *) ((rq)->elv.priv[1])

#define TPGS_WEIGHT_DEFAULT	(500)
#define MIN_DISPATCH_RQ		(8)

static struct blkcg_policy blkcg_policy_tpgs;

static inline struct tpgs_group *pd_to_tpgg(struct blkg_policy_data *pd)
{
	return pd ? container_of(pd, struct tpgs_group, pd) : NULL;
}

static inline struct tpgs_group *blkg_to_tpgg(struct blkcg_gq *blkg)
{
	return pd_to_tpgg(blkg_to_pd(blkg, &blkcg_policy_tpgs));
}

static inline struct tpgs_io_cq *
tpgs_tic_lookup(struct tpgs_data *tppd, struct io_context *ioc)
{
	if (ioc)
		return icq_to_tic(ioc_lookup_icq(ioc, tppd->queue));
	return NULL;
}

static inline struct tpgs_queue *tic_to_tpgq(struct tpgs_io_cq *tic)
{
	return tic->tpgq;
}

static inline void tic_set_tpgq(struct tpgs_io_cq *tic, struct tpgs_queue *tpgq)
{
	tic->tpgq = tpgq;
}

static inline struct tpgs_data *tic_to_tppd(struct tpgs_io_cq *tic)
{
	return tic->icq.q->elevator->elevator_data;
}

static inline void tpgg_get(struct tpgs_group *tpgg)
{
	return blkg_get(tpgg_to_blkg(tpgg));
}

static inline void tpgg_put(struct tpgs_group *tpgg)
{
	return blkg_put(tpgg_to_blkg(tpgg));
}

static inline void tpgg_stats_update_io_add(struct tpgs_group *tpgg,
					    struct tpgs_group *curr_tpgg, int rw)
{
	blkg_rwstat_add(&tpgg->stats.queued, rw, 1);
}

static inline void tpgg_stats_update_io_remove(struct tpgs_group *tpgg, int rw)
{
	blkg_rwstat_add(&tpgg->stats.queued, rw, -1);
}

static inline void tpgg_stats_update_io_merged(struct tpgs_group *tpgg, int rw)
{
	blkg_rwstat_add(&tpgg->stats.merged, rw, 1);
}

static inline void tpgg_stats_update_dispatch(struct tpgs_group *tpgg,
					      uint64_t bytes, int rw)
{
	blkg_stat_add(&tpgg->stats.sectors, bytes >> 9);
	blkg_rwstat_add(&tpgg->stats.serviced, rw, 1);
	blkg_rwstat_add(&tpgg->stats.service_bytes, rw, bytes);
}

static inline void tpgg_stats_update_completion(struct tpgs_group *tpgg,
			uint64_t start_time, uint64_t io_start_time, int rw)
{
	struct tpgg_stats *stats = &tpgg->stats;
	unsigned long long now = sched_clock();

	if (time_after64(now, io_start_time))
		blkg_rwstat_add(&stats->service_time, rw, now - io_start_time);
	if (time_after64(io_start_time, start_time))
		blkg_rwstat_add(&stats->wait_time, rw,
				io_start_time - start_time);
}

static void tpgs_del_queue(struct tpgs_queue *tpgq)
{
	struct tpgs_data *tppd = tpgq->tppd;
	struct tpgs_group *tpgg = tpgq->tpgg;

	if (!list_empty(&tpgq->tpgg_node)) {
		list_del_init(&tpgq->tpgg_node);
		tpgs_log_tpgq(tppd, tpgq, "del queue\n");
		tpgg->cur_dispatcher = NULL;
		tpgq->tpgg = NULL;
	}

	printk("%p nr_tpgq:%d\n", tpgg, tpgg->nr_tpgq);
	BUG_ON(tpgg->nr_tpgq < 1);
	tpgg->nr_tpgq--;
	if (!tpgg->nr_tpgq)
		tppd->total_weight -= tpgg->pd.blkg->blkcg->cfq_weight;

	BUG_ON(!tppd->busy_queues);
	tppd->busy_queues--;
}

/*
 * task holds one reference to the queue, dropped when task exits. each rq
 * in-flight on this queue also holds a reference, dropped when rq is freed.
 *
 * Each tpgs queue took a reference on the parent group. Drop it now.
 * queue lock must be held here.
 */
static void tpgs_put_queue(struct tpgs_queue *tpgq)
{
	struct tpgs_data *tppd = tpgq->tppd;
	struct tpgs_group *tpgg;

	BUG_ON(tpgq->ref <= 0);

	tpgq->ref--;
	if (tpgq->ref)
		return;

	tpgs_log_tpgq(tppd, tpgq, "put_queue");
	BUG_ON(!list_empty(&tpgq->sort_list));
	tpgg = tpgq->tpgg;

	tpgs_del_queue(tpgq);
	kmem_cache_free(tpgs_pool, tpgq);
	tpgg_put(tpgg);
}

static void tpgs_init_tpgq(struct tpgs_data *tppd, struct tpgs_queue *tpgq,
			  pid_t pid)
{
	INIT_LIST_HEAD(&tpgq->tpgg_node);
	INIT_LIST_HEAD(&tpgq->sort_list);

	tpgq->ref = 0;
	tpgq->tppd = tppd;
	tpgq->pid = pid;

}

static void tpgs_link_tpgq_tpgg(struct tpgs_queue *tpgq,
		struct tpgs_group *tpgg)
{
	tpgq->tpgg = tpgg;
	/* tpgq reference on tpgg */
	tpgg_get(tpgg);
}

static struct tpgs_group *tpgs_lookup_create_tpgg(struct tpgs_data *tppd,
						struct blkcg *blkcg)
{
	struct request_queue *q = tppd->queue;
	struct tpgs_group *tpgg = NULL;

	/* avoid lookup for the common case where there's no blkcg */
	if (blkcg == &blkcg_root) {
		tpgg = tppd->root_group;
	} else {
		struct blkcg_gq *blkg;

		blkg = blkg_lookup_create(blkcg, q);
		if (!IS_ERR(blkg))
			tpgg = blkg_to_tpgg(blkg);
	}

	return tpgg;
}

static struct tpgs_queue *
tpgs_find_alloc_queue(struct tpgs_data *tppd, struct tpgs_io_cq* tic, struct bio *bio,
		gfp_t gfp_mask)
{
	struct tpgs_queue *tpgq, *new_tpgq = NULL;
	struct tpgs_group *tpgg;
	struct blkcg *blkcg;

retry:
	rcu_read_lock();

	blkcg = bio_blkcg(bio);
	tpgg = tpgs_lookup_create_tpgg(tppd, blkcg);
	tpgq = tic_to_tpgq(tic);

	if (!tpgq) {
		if (new_tpgq) {
			tpgq = new_tpgq;
			new_tpgq = NULL;
		} else if (gfp_mask & __GFP_WAIT) {
			rcu_read_unlock();
			spin_unlock_irq(tppd->queue->queue_lock);
			new_tpgq = kmem_cache_alloc_node(tpgs_pool,
					gfp_mask | __GFP_ZERO,
					tppd->queue->node);
			spin_lock_irq(tppd->queue->queue_lock);
			if (new_tpgq)
				goto retry;
		} else
			tpgq = kmem_cache_alloc_node(tpgs_pool,
					gfp_mask | __GFP_ZERO,
					tppd->queue->node);

		if (tpgq) {
			tpgs_init_tpgq(tppd, tpgq, current->pid);
			tpgs_link_tpgq_tpgg(tpgq, tpgg);
			tpgs_log_tpgq(tppd, tpgq, "alloced");
		}
	}

	if (new_tpgq)
		kmem_cache_free(tpgs_pool, new_tpgq);

	rcu_read_unlock();
	return tpgq;
}

static struct tpgs_queue *
tpgs_get_queue(struct tpgs_data *tppd, struct tpgs_io_cq *tic, struct bio *bio,
			gfp_t gfp_mask)
{
	struct tpgs_queue *tpgq;

	tpgq = tpgs_find_alloc_queue(tppd, tic, bio, gfp_mask);
	tpgq->ref++;
	return tpgq;
}

/*
 * scheduler run of queue, if there are requests pending and no one in the
 * driver that will restart queueing
 */
static inline void tpgs_schedule_dispatch(struct tpgs_data *tppd)
{
	if (tppd->busy_queues) {
		tpgs_log(tppd, "schedule dispatch");
		kblockd_schedule_work(&tppd->unplug_work);
	}
}

static void check_blkcg_changed(struct tpgs_io_cq *tic, struct bio *bio)
{
	struct tpgs_data *tppd = tic_to_tppd(tic);
	struct tpgs_queue *tpgq;
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

	tpgq = tic_to_tpgq(tic);
	if (tpgq) {
		/*
		 * Drop reference to sync queue. A new sync queue will be
		 * assigned in new group upon arrival of a fresh request.
		 */
		tpgs_log_tpgq(tppd, tpgq, "changed cgroup");
		tic_set_tpgq(tic, NULL);
		tpgs_put_queue(tpgq);
	}

	tic->blkcg_serial_nr = serial_nr;
}

static int
tpgs_set_request(struct request_queue *q, struct request *rq, struct bio *bio,
			gfp_t gfp_mask)
{
	struct tpgs_data *tppd = q->elevator->elevator_data;
	struct tpgs_io_cq *tic = icq_to_tic(rq->elv.icq);
	struct tpgs_queue *tpgq;

	might_sleep_if(gfp_mask & __GFP_WAIT);

	spin_lock_irq(q->queue_lock);

	check_blkcg_changed(tic, bio);

	tpgq = tic_to_tpgq(tic);
	if (!tpgq) {
		tpgq = tpgs_get_queue(tppd, tic, bio, gfp_mask);
		tic_set_tpgq(tic, tpgq);
	}

	tpgq->ref++;
	tpgg_get(tpgq->tpgg);
	rq->elv.priv[0] = tpgq;
	rq->elv.priv[1] = tpgq->tpgg;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

/*
 * queue lock held here
 */
static void tpgs_put_request(struct request *rq)
{
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);

	if (tpgq) {
		WARN_ON(tpgq->tpgg != RQ_TPGG(rq));

		/* Put down rq reference on cfqg */
		tpgg_put(RQ_TPGG(rq));
		rq->elv.priv[0] = NULL;
		rq->elv.priv[1] = NULL;

		tpgs_put_queue(tpgq);
	}
}

static void
tpgs_update_group_weight(struct tpgs_group *tpgg)
{
	if (tpgg->needs_update) {
		tpgg->weight = tpgg->new_weight;
		tpgg->needs_update = false;
	}
}

static void tpgs_add_queue(struct tpgs_data *tppd, struct tpgs_queue *tpgq)
{
	struct tpgs_group *tpgg;

	if (!tpgq->online) {
		tpgq->online = 1;
		tpgg = tpgq->tpgg;
		tpgs_log_tpgq(tppd, tpgq, "add queue");
		tpgg->nr_tpgq++;
		tppd->busy_queues++;
		list_add(&tpgq->tpgg_node, &tpgg->queue_list);
		printk("add tpgq %p to %p\n", tpgq, tpgg);
		tpgs_update_group_weight(tpgg);
		if (tpgg->nr_tpgq <= 1) {
			tppd->total_weight += tpgg->pd.blkg->blkcg->cfq_weight;
			list_add(&tpgg->tppd_node, &tppd->group_list);
			printk("twt:%u, wt:%u %u %d %p\n", tppd->total_weight, tpgg->weight,
					tpgg->pd.blkg->blkcg->cfq_weight,
					tpgg->nr_tpgq,
					tpgg);
		}
	}
}

// registered to elevator_add_req_fn
static void tpgs_insert_request(struct request_queue *q, struct request *rq)
{
	struct tpgs_data *tppd = q->elevator->elevator_data;
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);

	tpgs_log_tpgq(tppd, tpgq, "insert_request");

	list_add_tail(&rq->queuelist, &tpgq->sort_list);
	tpgq->rq_queued++;
	tpgq->tpgg->rq_queued++;
	tppd->dispatched++;
	tpgs_add_queue(tppd, tpgq);
	tpgg_stats_update_io_add(RQ_TPGG(rq), tpgq->tpgg, rq->cmd_flags);
}

static void tpgs_remove_request(struct request *rq)
{
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);

	list_del_init(&rq->queuelist);
	tpgq->rq_queued--;
	tpgq->tpgg->rq_queued--;
	tpgg_stats_update_io_remove(RQ_TPGG(rq), rq->cmd_flags);
}

/*
 * Move request from internal lists to the request queue dispatch list.
 */
static int tpgs_dispatch_insert(struct request_queue *q,
				struct tpgs_queue *tpgq)
{
	struct list_head *rbnext = tpgq->sort_list.next;
	struct request *rq;

	if (rbnext == &tpgq->sort_list)
		return 0;

	rq = rq_entry_fifo(rbnext);
	tpgs_remove_request(rq);
	elv_dispatch_sort(q, rq);
	tpgg_stats_update_dispatch(tpgq->tpgg, blk_rq_bytes(rq), rq->cmd_flags);
	return 1;
}

static int tpgs_dispatch_requests_nr(struct tpgs_data *tppd,
				struct tpgs_queue *tpgq, int count)
{
	int cnt = 0, ret;

	if (!tpgq->rq_queued)
		return cnt;

	do {
		ret = tpgs_dispatch_insert(tppd->queue, tpgq);
		if (ret) {
			cnt++;
			tppd->dispatched--;
		}
	} while (ret && cnt < count);

	return cnt;
}


//registerd to elevator_dispatch_fn
static int tpgs_dispatch_requests(struct request_queue *q, int force)
{
	struct tpgs_data *tppd = q->elevator->elevator_data;
	struct tpgs_group *tpgg, *group_n;
	struct tpgs_queue *tpgq;
	struct list_head *next;
	int count = 0, total = 0, ret;
	int quota, grp_quota;

	if (!tppd->total_weight)
		return 0;

	quota = q->nr_requests - tppd->rq_in_driver;
	if (quota < MIN_DISPATCH_RQ && !force)
		return 0;

	list_for_each_entry_safe(tpgg, group_n, &tppd->group_list, tppd_node) {
		if (!tpgg->nr_tpgq)
			continue;
		grp_quota = (quota * tpgg->pd.blkg->blkcg->cfq_weight
					/ tppd->total_weight) - tpgg->rq_in_driver;
		tpgs_log_tpgg(tppd, tpgg,
			"nr:%d, wt:%u total_wt:%u in_driver:%d %d quota:%d grp_quota:%d",
			tpgg->nr_tpgq, tpgg->pd.blkg->blkcg->cfq_weight,
			tppd->total_weight, tpgg->rq_in_driver, tpgg->rq_queued,
			quota, grp_quota);
		if (grp_quota <= 0 && !force)
			continue;
		BUG_ON(tpgg->queue_list.next == &tpgg->queue_list);
		if (!tpgg->cur_dispatcher)
			tpgg->cur_dispatcher = tpgg->queue_list.next;
		next = tpgg->cur_dispatcher;
		count = 0;
		do {
			tpgq = list_entry(next, struct tpgs_queue, tpgg_node);
			tpgs_log_tpgq(tppd, tpgq, "tpgq: %d\n", tpgq->rq_queued);
			if (force)
				ret = tpgs_dispatch_requests_nr(tppd, tpgq, -1);
			else
				ret = tpgs_dispatch_requests_nr(tppd, tpgq, 1);
			count += ret;
			total += ret;
			next = next->next;
			if (next == &tpgg->queue_list)
				next = tpgg->queue_list.next;
			if (count >= grp_quota && !force) {
				tpgg->cur_dispatcher = next;
				break;
			}
			BUG_ON(tpgg->cur_dispatcher == &tpgg->queue_list);
		} while (next != tpgg->cur_dispatcher);
	}
	return total > 0;
}

static void tpgs_kick_queue(struct work_struct *work)
{
	struct tpgs_data *tppd =
		container_of(work, struct tpgs_data, unplug_work);
	struct request_queue *q = tppd->queue;

	spin_lock_irq(q->queue_lock);
	__blk_run_queue(q);
	spin_unlock_irq(q->queue_lock);
}

static void tpgs_init_tpgg_base(struct tpgs_group *tpgg)
{
	INIT_LIST_HEAD(&tpgg->tppd_node);
	INIT_LIST_HEAD(&tpgg->queue_list);
	tpgg->cur_dispatcher = NULL;

}

static int tpgs_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct tpgs_data *tppd;
	struct tpgs_group *tpgg;
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

	ret = blkcg_activate_policy(q, &blkcg_policy_tpgs);
	if (ret)
		goto out_free;

	/* Init root group */
	tppd->root_group = blkg_to_tpgg(q->root_blkg);
	tpgg = tppd->root_group;
	tpgs_init_tpgg_base(tpgg);

	/* Give preference to root group over other groups */
	tpgg->weight = 2 * TPGS_WEIGHT_DEFAULT;
	tpgg->leaf_weight = 2 * TPGS_WEIGHT_DEFAULT;

	INIT_WORK(&tppd->unplug_work, tpgs_kick_queue);

	return 0;

out_free:
	kfree(tppd);
	kobject_put(&eq->kobj);
	return ret;
}

static void tpgs_exit_queue(struct elevator_queue *e)
{
	struct tpgs_data *tppd = e->elevator_data;
	struct request_queue *q = tppd->queue;

	cancel_work_sync(&tppd->unplug_work);

	blkcg_deactivate_policy(q, &blkcg_policy_tpgs);
	kfree(tppd->root_group);
	kfree(tppd);
}

static void tpgs_activate_request(struct request_queue *q, struct request *rq)
{
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);
	struct tpgs_data *tppd = q->elevator->elevator_data;
	tppd->rq_in_driver++;
	tpgq->tpgg->rq_in_driver++;
	tpgs_log_tpgq(tppd, RQ_TPGQ(rq), "activate rq, drv=%d",
						tppd->rq_in_driver);
}

static void tpgs_deactivate_request(struct request_queue *q, struct request *rq)
{
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);
	struct tpgs_data *tppd = q->elevator->elevator_data;

	WARN_ON(!tppd->rq_in_driver);
	tppd->rq_in_driver--;
	tpgq->tpgg->rq_in_driver--;
	tpgs_log_tpgq(tppd, RQ_TPGQ(rq), "deactivate rq, drv=%d",
						tppd->rq_in_driver);
}

static void tpgs_completed_request(struct request_queue *q, struct request *rq)
{
	struct tpgs_queue *tpgq = RQ_TPGQ(rq);
	struct tpgs_data *tppd = tpgq->tppd;

	WARN_ON(!tpgq);
	WARN_ON(tpgq->tpgg != RQ_TPGG(rq));

	tpgs_log_tpgq(tppd, tpgq, "complete rqnoidle %d",
			!!(rq->cmd_flags & REQ_NOIDLE));
	WARN_ON(!tppd->rq_in_driver);
	tppd->rq_in_driver--;
	tpgq->tpgg->rq_in_driver--;
	tpgg_stats_update_completion(tpgq->tpgg,
			rq_start_time_ns(rq), rq_io_start_time_ns(rq), rq->cmd_flags);

	if (!tppd->rq_in_driver)
		tpgs_schedule_dispatch(tppd);
}

static void
tpgs_merged_request(struct request_queue *q, struct request *rq, int type)
{
	if (type == ELEVATOR_FRONT_MERGE) {
		struct tpgs_queue *tpgq = RQ_TPGQ(rq);
		list_del_init(&rq->queuelist);
		tpgq->rq_queued--;
		tpgg_stats_update_io_remove(RQ_TPGG(rq), rq->cmd_flags);
		list_add_tail(&rq->queuelist, &tpgq->sort_list);
		tpgq->rq_queued++;
		tpgg_stats_update_io_add(RQ_TPGG(rq), tpgq->tpgg, rq->cmd_flags);
	}
}

static void
tpgs_merged_requests(struct request_queue *q, struct request *rq,
			struct request *next)
{
	tpgs_remove_request(next);
	tpgg_stats_update_io_merged(RQ_TPGG(rq), rq->cmd_flags);
}

static void tpgs_init_icq(struct io_cq *icq)
{ }

static void tpgs_exit_icq(struct io_cq *icq)
{
	struct tpgs_io_cq *tic = icq_to_tic(icq);

	if (tic->tpgq) {
		tpgs_put_queue(tic->tpgq);
		tic->tpgq = NULL;
	}
}

static struct elevator_type iosched_tpgs = {
	.ops = {
		.elevator_merged_fn =		tpgs_merged_request,
		.elevator_merge_req_fn =	tpgs_merged_requests,
		.elevator_dispatch_fn =		tpgs_dispatch_requests,
		.elevator_add_req_fn =		tpgs_insert_request,
		.elevator_activate_req_fn = 	tpgs_activate_request,
		.elevator_deactivate_req_fn = 	tpgs_deactivate_request,
		.elevator_completed_req_fn =	tpgs_completed_request,
		.elevator_init_icq_fn =		tpgs_init_icq,
		.elevator_exit_icq_fn =		tpgs_exit_icq,
		.elevator_set_req_fn =		tpgs_set_request,
		.elevator_put_req_fn =		tpgs_put_request,
		.elevator_init_fn =		tpgs_init_queue,
		.elevator_exit_fn =		tpgs_exit_queue,
	},
	.icq_size		= sizeof(struct tpgs_io_cq),
	.icq_align		= __alignof__(struct tpgs_io_cq),
	.elevator_name	=	"tpgs",
	.elevator_owner =	THIS_MODULE,
};

static u64 tpgg_prfill_weight_device(struct seq_file *sf,
				     struct blkg_policy_data *pd, int off)
{
	struct tpgs_group *tpgg = pd_to_tpgg(pd);

	if (!tpgg->dev_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, tpgg->dev_weight);
}

static int tpgg_print_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tpgg_prfill_weight_device, &blkcg_policy_tpgs,
					  0, false);
	return 0;
}

static u64 tpgg_prfill_leaf_weight_device(struct seq_file *sf,
					  struct blkg_policy_data *pd, int off)
{
	struct tpgs_group *tpgg = pd_to_tpgg(pd);

	if (!tpgg->dev_leaf_weight)
		return 0;
	return __blkg_prfill_u64(sf, pd, tpgg->dev_leaf_weight);
}

static int tpgg_print_leaf_weight_device(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tpgg_prfill_leaf_weight_device, &blkcg_policy_tpgs,
					  0, false);
	return 0;
}

static int tpgg_print_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n", css_to_blkcg(seq_css(sf))->cfq_weight); 
	return 0;
}

static int tpgg_print_leaf_weight(struct seq_file *sf, void *v)
{
	seq_printf(sf, "%u\n",  css_to_blkcg(seq_css(sf))->cfq_leaf_weight);
	return 0;
}

			   
static ssize_t __tpgg_set_weight_device(struct kernfs_open_file *of,
										char *buf, size_t nbytes, loff_t off,
										bool is_leaf_weight)
{
	struct blkcg *blkcg = css_to_blkcg(of_css(of));	
	struct blkg_conf_ctx ctx;
	struct tpgs_group *tpgg;
	int ret;

	ret = blkg_conf_prep(blkcg, &blkcg_policy_tpgs, buf, &ctx);
	if (ret)
		return ret;

	ret = -EINVAL;
	tpgg = blkg_to_tpgg(ctx.blkg);
	if (!ctx.v || (ctx.v >= CFQ_WEIGHT_MIN && ctx.v <= CFQ_WEIGHT_MAX)) {
		if (!is_leaf_weight) {
			tpgg->dev_weight = ctx.v;
			tpgg->new_weight = ctx.v ?: blkcg->cfq_weight;
		} else {
			tpgg->dev_leaf_weight = ctx.v;
			tpgg->new_leaf_weight = ctx.v ?: blkcg->cfq_leaf_weight;
		}
		ret = 0;
	}

	blkg_conf_finish(&ctx);
	return ret;
}			   

static ssize_t tpgg_set_weight_device(struct kernfs_open_file *of,
								  char *buf, size_t nbytes, loff_t off)
{
	return __tpgg_set_weight_device(of, buf, nbytes, off, false);
}

static ssize_t tpgg_set_leaf_weight_device(struct kernfs_open_file *of,
										   char *buf, size_t nbytes, loff_t off)
{
	return __tpgg_set_weight_device(of, buf, nbytes, off, true);
}

static int __tpgs_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
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
		struct tpgs_group *tpgg = blkg_to_tpgg(blkg);

		if (!tpgg)
			continue;

		if (!is_leaf_weight) {
			if (!tpgg->dev_weight)
				tpgg->new_weight = blkcg->cfq_weight;
		} else {
			if (!tpgg->dev_leaf_weight)
				tpgg->new_leaf_weight = blkcg->cfq_leaf_weight;
		}
	}

	spin_unlock_irq(&blkcg->lock);
	return 0;
}

static int tpgs_set_weight(struct cgroup_subsys_state *css, struct cftype *cft,
							u64 val)
{
	return __tpgs_set_weight(css, cft, val, false);
}

static int tpgs_set_leaf_weight(struct cgroup_subsys_state *css,
								struct cftype *cft, u64 val)
{
	return __tpgs_set_weight(css, cft, val, true);
}

/* offset delta from tpgg->stats to tpgg->dead_stats */
static const int dead_stats_off_delta = offsetof(struct tpgs_group, dead_stats) -
					offsetof(struct tpgs_group, stats);

/* to be used by recursive prfill, sums live and dead rwstats recursively */
static struct blkg_rwstat tpgg_rwstat_pd_recursive_sum(struct blkg_policy_data *pd,
						       int off)
{
	struct blkg_rwstat a, b;

	a = blkg_rwstat_recursive_sum(pd, off);
	b = blkg_rwstat_recursive_sum(pd, off + dead_stats_off_delta);
	blkg_rwstat_merge(&a, &b);
	return a;
}

/* to be used by recursive prfill, sums live and dead stats recursively */
static u64 tpgg_stat_pd_recursive_sum(struct blkg_policy_data *pd, int off)
{
	u64 sum = 0;

	sum += blkg_stat_recursive_sum(pd, off);
	sum += blkg_stat_recursive_sum(pd, off + dead_stats_off_delta);
	return sum;
}

static int tpgg_print_stat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_stat,
					  &blkcg_policy_tpgs, seq_cft(sf)->private, false);
	return 0;
}

static int tpgg_print_rwstat(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)), blkg_prfill_rwstat,
					  &blkcg_policy_tpgs,  seq_cft(sf)->private, true);
	return 0;
}

static u64 tpgg_prfill_stat_recursive(struct seq_file *sf,
				      struct blkg_policy_data *pd, int off)
{
	u64 sum = tpgg_stat_pd_recursive_sum(pd, off);

	return __blkg_prfill_u64(sf, pd, sum);
}

static u64 tpgg_prfill_rwstat_recursive(struct seq_file *sf,
					struct blkg_policy_data *pd, int off)
{
	struct blkg_rwstat sum = tpgg_rwstat_pd_recursive_sum(pd, off);

	return __blkg_prfill_rwstat(sf, pd, &sum);
}

static int tpgg_print_stat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tpgg_prfill_stat_recursive,  &blkcg_policy_tpgs,
					  seq_cft(sf)->private, false);
	return 0;
}

static int tpgg_print_rwstat_recursive(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
					  tpgg_prfill_rwstat_recursive,  &blkcg_policy_tpgs,
					  seq_cft(sf)->private, true);
	return 0;
}

static struct cftype tpgs_blkcg_files[] = {
	/* on root, weight is mapped to leaf_weight */
	{
		.name = "tpgs.weight_device",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = tpgg_print_leaf_weight_device,
		.write = tpgg_set_leaf_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpgs.weight",
		.flags = CFTYPE_ONLY_ON_ROOT,
		.seq_show = tpgg_print_leaf_weight,
		.write_u64 = tpgs_set_leaf_weight,
	},

	/* no such mapping necessary for !roots */
	{
		.name = "tpgs.weight_device",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = tpgg_print_weight_device,
		.write = tpgg_set_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpgs.weight",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = tpgg_print_weight,
		.write_u64 = tpgs_set_weight,
	},

	{
		.name = "tpgs.leaf_weight_device",
		.seq_show = tpgg_print_leaf_weight_device,
		.write = tpgg_set_leaf_weight_device,
		.max_write_len = 256,
	},
	{
		.name = "tpgs.leaf_weight",
		.seq_show = tpgg_print_leaf_weight,
		.write_u64 = tpgs_set_leaf_weight,
	},

	/* statistics, covers only the tasks in the tpgg */
	{
		.name = "tpgs.time",
		.private = offsetof(struct tpgs_group, stats.time),
		.seq_show = tpgg_print_stat,
	},
	{
		.name = "tpgs.sectors",
		.private = offsetof(struct tpgs_group, stats.sectors),
		.seq_show = tpgg_print_stat,
	},
	{
		.name = "tpgs.io_service_bytes",
		.private = offsetof(struct tpgs_group, stats.service_bytes),
		.seq_show = tpgg_print_rwstat,
	},
	{
		.name = "tpgs.io_serviced",
		.private = offsetof(struct tpgs_group, stats.serviced),
		.seq_show= tpgg_print_rwstat,
	},
	{
		.name = "tpgs.io_service_time",
		.private = offsetof(struct tpgs_group, stats.service_time),
		.seq_show = tpgg_print_rwstat,
	},
	{
		.name = "tpgs.io_wait_time",
		.private = offsetof(struct tpgs_group, stats.wait_time),
		.seq_show = tpgg_print_rwstat,
	},
	{
		.name = "tpgs.io_merged",
		.private = offsetof(struct tpgs_group, stats.merged),
		.seq_show = tpgg_print_rwstat,
	},
	{
		.name = "tpgs.io_queued",
		.private = offsetof(struct tpgs_group, stats.queued),
		.seq_show = tpgg_print_rwstat,
	},

	/* the same statictics which cover the tpgg and its descendants */
	{
		.name = "tpgs.time_recursive",
		.private = offsetof(struct tpgs_group, stats.time),
		.seq_show = tpgg_print_stat_recursive,
	},
	{
		.name = "tpgs.sectors_recursive",
		.private = offsetof(struct tpgs_group, stats.sectors),
		.seq_show = tpgg_print_stat_recursive,
	},
	{
		.name = "tpgs.io_service_bytes_recursive",
		.private = offsetof(struct tpgs_group, stats.service_bytes),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{
		.name = "tpgs.io_serviced_recursive",
		.private = offsetof(struct tpgs_group, stats.serviced),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{
		.name = "tpgs.io_service_time_recursive",
		.private = offsetof(struct tpgs_group, stats.service_time),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{
		.name = "tpgs.io_wait_time_recursive",
		.private = offsetof(struct tpgs_group, stats.wait_time),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{
		.name = "tpgs.io_merged_recursive",
		.private = offsetof(struct tpgs_group, stats.merged),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{
		.name = "tpgs.io_queued_recursive",
		.private = offsetof(struct tpgs_group, stats.queued),
		.seq_show = tpgg_print_rwstat_recursive,
	},
	{ }	/* terminate */
};


static void tpgg_stats_init(struct tpgg_stats *stats)
{
	blkg_rwstat_init(&stats->service_bytes);
	blkg_rwstat_init(&stats->serviced);
	blkg_rwstat_init(&stats->merged);
	blkg_rwstat_init(&stats->service_time);
	blkg_rwstat_init(&stats->wait_time);
	blkg_rwstat_init(&stats->queued);

	blkg_stat_init(&stats->sectors);
	blkg_stat_init(&stats->time);
}
// operation
static void tpgs_pd_init(struct blkcg_gq *blkg)
{
	struct tpgs_group *tpgg = blkg_to_tpgg(blkg);

	tpgs_init_tpgg_base(tpgg);
	tpgg->weight = blkg->blkcg->cfq_weight;
	tpgg->leaf_weight = blkg->blkcg->cfq_leaf_weight;
	// TMP
	tpgg_stats_init(&tpgg->stats);
	tpgg_stats_init(&tpgg->dead_stats);
}

static inline struct tpgs_group *tpgg_parent(struct tpgs_group *tpgg)
{
	struct blkcg_gq *pblkg = tpgg_to_blkg(tpgg)->parent;

	return pblkg ? blkg_to_tpgg(pblkg) : NULL;
}




static void tpgg_stats_reset(struct tpgg_stats *stats)
{
	/* queued stats shouldn't be cleared */
	blkg_rwstat_reset(&stats->service_bytes);
	blkg_rwstat_reset(&stats->serviced);
	blkg_rwstat_reset(&stats->merged);
	blkg_rwstat_reset(&stats->service_time);
	blkg_rwstat_reset(&stats->wait_time);
	blkg_stat_reset(&stats->time);
}

/* @to += @from */
static void tpgg_stats_merge(struct tpgg_stats *to, struct tpgg_stats *from)
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

static void tpgg_stats_xfer_dead(struct tpgs_group *tpgg)
{
	struct tpgs_group *parent = tpgg_parent(tpgg);

	lockdep_assert_held(tpgg_to_blkg(tpgg)->q->queue_lock);

	if (unlikely(!parent))
		return;

	tpgg_stats_merge(&parent->dead_stats, &tpgg->stats);
	tpgg_stats_merge(&parent->dead_stats, &tpgg->dead_stats);
	tpgg_stats_reset(&tpgg->stats);
	tpgg_stats_reset(&tpgg->dead_stats);
}

static void tpgs_pd_offline(struct blkcg_gq *blkg)
{
	struct tpgs_group *tpgg = blkg_to_tpgg(blkg);
	/*
	 * @blkg is going offline and will be ignored by
	 * blkg_[rw]stat_recursive_sum().  Transfer stats to the parent so
	 * that they don't get lost.  If IOs complete after this point, the
	 * stats for them will be lost.  Oh well...
	 */
	tpgg_stats_xfer_dead(tpgg);

	if (!list_empty(&tpgg->tppd_node))
		list_del_init(&tpgg->tppd_node);

	//BUG_ON(!list_empty(&(tpgg->queue_list)));
}

static void tpgs_pd_reset_stats(struct blkcg_gq *blkg)
{
	struct tpgs_group *tpgg = blkg_to_tpgg(blkg);

	tpgg_stats_reset(&tpgg->stats);
	tpgg_stats_reset(&tpgg->dead_stats);
}

static struct blkcg_policy blkcg_policy_tpgs = {
	.pd_size			= sizeof(struct tpgs_group), //policy specific private data size
	.cftypes			= tpgs_blkcg_files, // cgroup files for the policy
	/* operations */
	.pd_init_fn			= tpgs_pd_init, 
	.pd_offline_fn		= tpgs_pd_offline,
	.pd_reset_stats_fn	= tpgs_pd_reset_stats,
};

// MEMO:reference to cfq-iosched.c __init :OK
// register ( blkcg policy -> kmem cache -> elevator )
static int __init tpgs_init(void)
{
	int ret;

	ret = blkcg_policy_register(&blkcg_policy_tpgs); 
	if (ret)
		return ret;

	ret = -ENOMEM;
	tpgs_pool = KMEM_CACHE(tpgs_queue, 0);
	if (!tpgs_pool)
		goto err_pol_unreg;

	ret = elv_register(&iosched_tpgs);
	if (ret)
		goto err_free_pool;

	return 0;

err_free_pool:
	kmem_cache_destroy(tpgs_pool);
err_pol_unreg:
	blkcg_policy_unregister(&blkcg_policy_tpgs);
	return ret;
}

// free ( blkcg policy -> elevator ->  kmem cache )
static void __exit tpgs_exit(void)
{
	blkcg_policy_unregister(&blkcg_policy_tpgs);
	elv_unregister(&iosched_tpgs);
	kmem_cache_destroy(tpgs_pool);
}

module_init(tpgs_init);
module_exit(tpgs_exit);

MODULE_AUTHOR("Kairi OKUMURA");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tiny Parallel Group io Scheduler");
