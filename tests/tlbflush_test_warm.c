/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Benchmark the cycle cost of local and broadcast user TLB invalidations.
 *
 * The broadcast cases are split so remote CPU wakeup/interconnect effects are
 * visible separately from the cost of invalidating entries that are known to be
 * present in remote TLBs.
 *
 * (C) 2026 Sayali Kulkarni <sk@gentwo.org>
 */

#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/uaccess.h>
#include <asm/tlbflush.h>

#include "cycles.h"

#define TEST_COUNT 1000
#define TEST_SIZE PAGE_SIZE

enum flush_op {
	FLUSH_NONE,
	FLUSH_MM,
	FLUSH_PAGE,
};

struct flush_target {
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned long addr;
};

struct sample_stats {
	unsigned long min;
	unsigned long p50;
	unsigned long p90;
	unsigned long p99;
	unsigned long max;
	unsigned long avg;
	int n;
};

struct once_data {
	struct mm_struct *mm;
	unsigned long addr;
	struct completion done;
	int ret;
};

struct worker_data {
	struct mm_struct *mm;
	unsigned long addr;
	atomic_t *phase;
	atomic_t *acks;
	bool sync_hot;
	bool use_mm;
};

struct worker_group {
	struct task_struct **tasks;
	struct worker_data *data;
	atomic_t phase;
	atomic_t acks;
	int nr;
	bool mm_pinned;
};

static int cmp_ulong(const void *a, const void *b)
{
	unsigned long va = *(const unsigned long *)a;
	unsigned long vb = *(const unsigned long *)b;

	return (va > vb) - (va < vb);
}

static unsigned long percentile(const unsigned long *samples, int n, int pct)
{
	return samples[((n - 1) * pct) / 100];
}

static void compute_stats(unsigned long *samples, int n, struct sample_stats *s)
{
	u64 sum = 0;
	int i;

	sort(samples, n, sizeof(*samples), cmp_ulong, NULL);

	for (i = 0; i < n; i++)
		sum += samples[i];

	s->n = n;
	s->min = samples[0];
	s->p50 = percentile(samples, n, 50);
	s->p90 = percentile(samples, n, 90);
	s->p99 = percentile(samples, n, 99);
	s->max = samples[n - 1];
	s->avg = DIV_ROUND_CLOSEST_ULL(sum, n);
}

static unsigned long get_current_freq_khz(void)
{
	struct cpufreq_policy *p = cpufreq_cpu_get(smp_processor_id());
	unsigned long khz = 3000000;

	if (p) {
		khz = p->cur;
		cpufreq_cpu_put(p);
	}

	return khz;
}

static void report_stats(const char *label, const struct sample_stats *s)
{
	unsigned long freq_khz = get_current_freq_khz();
	unsigned long p50_ns = s->p50 * 1000UL * 1000UL / freq_khz;

	pr_info("tlbflush_warm: %-15s n=%d avg=%lu p50=%lu p90=%lu p99=%lu min=%lu max=%lu cycles (~p50 %lu ns @ %lu MHz)\n",
		label, s->n, s->avg, s->p50, s->p90, s->p99, s->min, s->max,
		p50_ns, freq_khz / 1000);
}

static void report_ratio(const char *label, unsigned long num,
			 unsigned long denom)
{
	if (!num || !denom) {
		pr_info("tlbflush_warm: %-15s p50 ratio=unavailable\n",
			label);
		return;
	}

	pr_info("tlbflush_warm: %-15s p50 ratio=%lu.%02lux\n",
		label, num / denom, (num * 100 / denom) % 100);
}

static int touch_once_thread(void *arg)
{
	struct once_data *d = arg;
	u8 val = 0;

	kthread_use_mm(d->mm);
	d->ret = get_user(val, (u8 __user *)d->addr);
	kthread_unuse_mm(d->mm);
	complete(&d->done);

	(void)val;
	return 0;
}

static int force_broadcast_state(struct mm_struct *mm, unsigned long addr)
{
	struct task_struct *task;
	struct once_data data;
	int cpu;

	cpu = cpumask_any_but(cpu_online_mask, smp_processor_id());
	if (cpu >= nr_cpu_ids) {
		pr_warn("tlbflush_warm: no remote CPU; broadcast cases skipped\n");
		return -EINVAL;
	}

	init_completion(&data.done);
	data.mm = mm;
	data.addr = addr;
	data.ret = 0;

	mmgrab(mm);
	task = kthread_create(touch_once_thread, &data, "tlbflush_once");
	if (IS_ERR(task)) {
		mmdrop(mm);
		return PTR_ERR(task);
	}

	kthread_bind(task, cpu);
	wake_up_process(task);
	wait_for_completion(&data.done);
	kthread_stop(task);
	mmdrop(mm);

	if (data.ret)
		return data.ret;

	/*
	 * active_cpu is now permanently MULTIPLE. Clear the TLB entry left by
	 * the remote toucher before measuring the non-hot broadcast cases.
	 */
	flush_tlb_mm(mm);

	return 0;
}

static int remote_worker_thread(void *arg)
{
	struct worker_data *w = arg;
	int seen = 0;
	u8 val = 0;

	if (w->use_mm)
		kthread_use_mm(w->mm);

	while (!kthread_should_stop()) {
		int phase;

		if (!w->sync_hot) {
			cpu_relax();
			continue;
		}

		phase = atomic_read(w->phase);
		if (phase == seen) {
			cpu_relax();
			continue;
		}

		seen = phase;
		if (get_user(val, (u8 __user *)w->addr))
			pr_warn_once("tlbflush_warm: remote get_user faulted\n");
		atomic_inc(w->acks);
	}

	if (w->use_mm)
		kthread_unuse_mm(w->mm);

	(void)val;
	return 0;
}

static int start_workers(struct worker_group *g, struct mm_struct *mm,
			 unsigned long addr, bool sync_hot, bool use_mm)
{
	int cpu;

	memset(g, 0, sizeof(*g));
	atomic_set(&g->phase, 0);
	atomic_set(&g->acks, 0);

	g->tasks = kcalloc(nr_cpu_ids, sizeof(*g->tasks), GFP_KERNEL);
	g->data = kcalloc(nr_cpu_ids, sizeof(*g->data), GFP_KERNEL);
	if (!g->tasks || !g->data) {
		kfree(g->tasks);
		kfree(g->data);
		memset(g, 0, sizeof(*g));
		return -ENOMEM;
	}

	if (use_mm) {
		mmgrab(mm);
		g->mm_pinned = true;
	}

	for_each_online_cpu(cpu) {
		struct task_struct *task;
		struct worker_data *w;

		if (cpu == smp_processor_id())
			continue;

		w = &g->data[g->nr];
		w->mm = mm;
		w->addr = addr;
		w->phase = &g->phase;
		w->acks = &g->acks;
		w->sync_hot = sync_hot;
		w->use_mm = use_mm;

		task = kthread_create(remote_worker_thread, w,
				      sync_hot ? "tlbflush_hot%d" :
						 "tlbflush_awake%d",
				      cpu);
		if (IS_ERR(task))
			continue;

		kthread_bind(task, cpu);
		g->tasks[g->nr] = task;
		g->nr++;
		wake_up_process(task);
	}

	if (!g->nr) {
		pr_warn("tlbflush_warm: no remote workers started\n");
		if (g->mm_pinned)
			mmdrop(mm);
		kfree(g->tasks);
		kfree(g->data);
		memset(g, 0, sizeof(*g));
		return -ENODEV;
	}

	return 0;
}

static void stop_workers(struct worker_group *g)
{
	int i;

	for (i = 0; i < g->nr; i++)
		if (g->tasks[i])
			kthread_stop(g->tasks[i]);

	if (g->mm_pinned)
		mmdrop(g->data[0].mm);

	kfree(g->tasks);
	kfree(g->data);
	memset(g, 0, sizeof(*g));
}

static int sync_remote_hot(struct worker_group *g)
{
	unsigned long deadline;
	int phase;

	if (!g || !g->nr)
		return 0;

	atomic_set(&g->acks, 0);
	smp_wmb();
	phase = atomic_inc_return(&g->phase);

	deadline = jiffies + msecs_to_jiffies(1000);
	while (atomic_read(&g->acks) < g->nr) {
		if (!time_before(jiffies, deadline)) {
			pr_warn("tlbflush_warm: remote hot sync timed out phase=%d acks=%d/%d\n",
				phase, atomic_read(&g->acks), g->nr);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	return 0;
}

static int collect_samples(struct flush_target *target, enum flush_op op,
			   struct worker_group *hot, unsigned long *samples,
			   int count)
{
	unsigned long start, stop;
	int done = 0;
	u8 val = 0;

	if (CYCLES_ENABLE)
		return 0;

	while (done < count) {
		if (hot && sync_remote_hot(hot))
			break;

		if (get_user(val, (u8 __user *)target->addr))
			break;

		barrier();
		isb();
		start = CYCLES;
		switch (op) {
		case FLUSH_NONE:
			barrier();
			break;
		case FLUSH_MM:
			flush_tlb_mm(target->mm);
			break;
		case FLUSH_PAGE:
			flush_tlb_page(target->vma, target->addr);
			break;
		}
		stop = CYCLES;
		samples[done++] = stop - start;
	}
	CYCLES_DISABLE;

	(void)val;
	return done;
}

static int run_case(const char *label, struct flush_target *target,
		    enum flush_op op, struct worker_group *hot,
		    struct sample_stats *stats)
{
	unsigned long *samples;
	int n;

	samples = kcalloc(TEST_COUNT, sizeof(*samples), GFP_KERNEL);
	if (!samples)
		return -ENOMEM;

	n = collect_samples(target, op, hot, samples, TEST_COUNT);
	if (n <= 0) {
		kfree(samples);
		pr_err("tlbflush_warm: no samples collected for %s\n", label);
		return -EIO;
	}

	if (n != TEST_COUNT)
		pr_warn("tlbflush_warm: %s collected only %d/%d samples\n",
			label, n, TEST_COUNT);

	compute_stats(samples, n, stats);
	report_stats(label, stats);
	kfree(samples);

	return n == TEST_COUNT ? 0 : -EIO;
}

static int tlbflush_warm_init(void)
{
	struct sample_stats base = {};
	struct sample_stats local_mm = {};
	struct sample_stats local_page = {};
	struct sample_stats idle_mm = {};
	struct sample_stats idle_page = {};
	struct sample_stats awake_mm = {};
	struct sample_stats awake_page = {};
	struct sample_stats hot_mm = {};
	struct sample_stats hot_page = {};
	struct flush_target target;
	struct worker_group awake;
	struct worker_group hot;
	cpumask_var_t old_mask;
	cpumask_t self_mask;
	unsigned long addr;
	int ret = 0;
	u8 val;

	memset(&awake, 0, sizeof(awake));
	memset(&hot, 0, sizeof(hot));

	if (!current->mm) {
		pr_err("tlbflush_warm: no user mm\n");
		return -EAGAIN;
	}

	if (!alloc_cpumask_var(&old_mask, GFP_KERNEL))
		return -ENOMEM;

	cpumask_copy(old_mask, current->cpus_ptr);
	cpumask_clear(&self_mask);
	cpumask_set_cpu(smp_processor_id(), &self_mask);
	ret = set_cpus_allowed_ptr(current, &self_mask);
	if (ret) {
		pr_err("tlbflush_warm: failed to pin benchmark task: %d\n", ret);
		goto out_affinity;
	}

	addr = vm_mmap(NULL, 0, TEST_SIZE, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, 0);
	if (IS_ERR_VALUE(addr)) {
		pr_err("tlbflush_warm: vm_mmap failed\n");
		ret = -EAGAIN;
		goto out_affinity;
	}

	target.mm = current->mm;
	target.addr = addr;

	mmap_read_lock(target.mm);
	target.vma = find_vma(target.mm, addr);
	mmap_read_unlock(target.mm);
	if (!target.vma) {
		pr_err("tlbflush_warm: find_vma failed\n");
		ret = -EAGAIN;
		goto out_unmap;
	}

	if (get_user(val, (u8 __user *)addr)) {
		pr_err("tlbflush_warm: initial get_user faulted\n");
		ret = -EAGAIN;
		goto out_unmap;
	}
	(void)val;

	pr_info("tlbflush_warm: measuring on CPU%d with %d online CPUs\n",
		smp_processor_id(), num_online_cpus());

	run_case("BASE none", &target, FLUSH_NONE, NULL, &base);
	run_case("LOCAL mm", &target, FLUSH_MM, NULL, &local_mm);
	run_case("LOCAL page", &target, FLUSH_PAGE, NULL, &local_page);

	ret = force_broadcast_state(target.mm, target.addr);
	if (ret) {
		pr_err("tlbflush_warm: failed to force broadcast state: %d\n",
		       ret);
		goto out_unmap;
	}

	run_case("BCAST idle mm", &target, FLUSH_MM, NULL, &idle_mm);
	run_case("BCAST idle page", &target, FLUSH_PAGE, NULL, &idle_page);

	if (!start_workers(&awake, target.mm, target.addr, false, false)) {
		run_case("BCAST awake mm", &target, FLUSH_MM, NULL, &awake_mm);
		run_case("BCAST awake page", &target, FLUSH_PAGE, NULL,
			 &awake_page);
		stop_workers(&awake);
	}

	if (!start_workers(&hot, target.mm, target.addr, true, true)) {
		pr_info("tlbflush_warm: synchronized hot workers=%d\n", hot.nr);
		run_case("BCAST hot mm", &target, FLUSH_MM, &hot, &hot_mm);
		run_case("BCAST hot page", &target, FLUSH_PAGE, &hot,
			 &hot_page);
		stop_workers(&hot);
	}

	report_ratio("idle/local mm", idle_mm.p50, local_mm.p50);
	report_ratio("idle/local page", idle_page.p50, local_page.p50);
	report_ratio("awake/local mm", awake_mm.p50, local_mm.p50);
	report_ratio("awake/local page", awake_page.p50, local_page.p50);
	report_ratio("hot/local mm", hot_mm.p50, local_mm.p50);
	report_ratio("hot/local page", hot_page.p50, local_page.p50);

out_unmap:
	vm_munmap(addr, TEST_SIZE);
out_affinity:
	set_cpus_allowed_ptr(current, old_mask);
	free_cpumask_var(old_mask);

	return ret ?: -EAGAIN;
}

static void tlbflush_warm_exit(void)
{
}

module_init(tlbflush_warm_init);
module_exit(tlbflush_warm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayali Kulkarni <sk@gentwo.org>");
MODULE_DESCRIPTION("TLB flush broadcast benchmark with controlled remote CPU states");
