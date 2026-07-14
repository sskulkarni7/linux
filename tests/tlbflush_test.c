/* tlbflush_test.c
*
* Benchmark: cycle cost of flush_tlb_mm()/flush_tlb_page() when the
* active_cpu optimization takes the local-only fast path versus when
* it must broadcast to all CPUs.
*
* (C) 2026 Sayali Kulkarni <sk@gentwo.org>
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/sched/mm.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/uaccess.h>
#include <asm/tlbflush.h>
#include "cycles.h"

#define TEST_COUNT 1000
#define TEST_SIZE PAGE_SIZE

struct toucher_data {
	struct mm_struct *mm;
	unsigned long addr;
	struct completion attached;
	struct completion release;
};

static int toucher_thread(void *arg)
{
	struct toucher_data *d = arg;
	u8 val;

	kthread_use_mm(d->mm);
	/*
* Genuinely access the page on this (remote) CPU: this makes the mm
* truly active here (flipping active_cpu to MULTIPLE) and leaves a
* real TLB entry behind. get_user() carries the extable fixup, so a
* fault returns an error instead of oopsing.
*/
	if (get_user(val, (u8 __user *)d->addr))
		pr_warn("tlbflush_test: toucher get_user faulted\n");
	complete(&d->attached);
	wait_for_completion(&d->release);
	kthread_unuse_mm(d->mm);
	return 0;
}

static unsigned long time_flush_mm(struct mm_struct *mm, unsigned long addr,
				   int count)
{
	unsigned long start, stop, sum = 0;
	u8 val;
	int i;

	if (CYCLES_ENABLE)
		return 0;

	for (i = 0; i < count; i++) {
		/*
* Re-establish a real TLB entry before every flush; the
* previous iteration's flush emptied it, so without this we
* would just be timing the invalidation of nothing.
*/
		if (get_user(val, (u8 __user *)addr))
			break;
		barrier();
		isb();
		start = CYCLES;
		flush_tlb_mm(mm);
		stop = CYCLES;
		sum += stop - start;
	}
	CYCLES_DISABLE;
	(void)val;

	return (sum + count / 2) / count;
}

static unsigned long time_flush_page(struct vm_area_struct *vma,
				     unsigned long addr, int count)
{
	unsigned long start, stop, sum = 0;
	u8 val;
	int i;

	if (CYCLES_ENABLE)
		return 0;

	for (i = 0; i < count; i++) {
		if (get_user(val, (u8 __user *)addr))
			break;
		barrier();
		isb();
		start = CYCLES;
		flush_tlb_page(vma, addr);
		stop = CYCLES;
		sum += stop - start;
	}
	CYCLES_DISABLE;
	(void)val;

	return (sum + count / 2) / count;
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

static void report_result(const char *label, unsigned long cycles)
{
	unsigned long freq_khz = get_current_freq_khz();
	unsigned long ns = cycles * 1000UL * 1000UL / freq_khz;

	pr_info("tlbflush_test: %-10s %5lu cycles  (~%lu ns @ %lu MHz)\n",
		label, cycles, ns, freq_khz / 1000);
}

static int tlbflush_test_init(void)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	unsigned long addr;
	unsigned long local_mm, bcast_mm, local_page, bcast_page;
	struct toucher_data d;
	struct task_struct *toucher;
	cpumask_t self_mask = CPU_MASK_NONE;
	u8 val;
	int other_cpu;

	if (!mm) {
		pr_err("tlbflush_test: no user mm\n");
		return -EAGAIN;
	}

	cpumask_set_cpu(smp_processor_id(), &self_mask);
	set_cpus_allowed_ptr(current, &self_mask);

	addr = vm_mmap(NULL, 0, TEST_SIZE, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, 0);
	if (addr >= (unsigned long)TASK_SIZE) {
		pr_err("tlbflush_test: vm_mmap failed\n");
		return -EAGAIN;
	}

	mmap_read_lock(mm);
	vma = find_vma(mm, addr);
	mmap_read_unlock(mm);
	if (!vma) {
		pr_err("tlbflush_test: find_vma failed\n");
		vm_munmap(addr, TEST_SIZE);
		return -EAGAIN;
	}

	/*
* Make sure the page is really resident before timing, so the
* get_user() refills inside the loops hit the TLB fill path and not
* the page-fault path.
*/
	if (get_user(val, (u8 __user *)addr)) {
		pr_err("tlbflush_test: initial get_user faulted\n");
		vm_munmap(addr, TEST_SIZE);
		return -EAGAIN;
	}
	(void)val;

	/* ---- Phase 1: mm has only ever run on this CPU ---- */
	local_mm = time_flush_mm(mm, addr, TEST_COUNT);
	local_page = time_flush_page(vma, addr, TEST_COUNT);
	report_result("LOCAL mm", local_mm);
	report_result("LOCAL page", local_page);

	/* ---- Phase 2: force active_cpu -> ACTIVE_CPU_MULTIPLE ---- */
	init_completion(&d.attached);
	init_completion(&d.release);
	d.mm = mm;
	d.addr = addr;
	mmgrab(mm);

	toucher = kthread_create(toucher_thread, &d, "tlbflush_toucher");
	if (IS_ERR(toucher)) {
		pr_err("tlbflush_test: kthread_create failed\n");
		mmdrop(mm);
		goto out_unmap;
	}

	other_cpu = cpumask_any_but(cpu_online_mask, smp_processor_id());
	if (other_cpu < nr_cpu_ids) {
		cpumask_t other_mask = CPU_MASK_NONE;

		cpumask_set_cpu(other_cpu, &other_mask);
		set_cpus_allowed_ptr(toucher, &other_mask);
	} else {
		pr_warn("tlbflush_test: only 1 online CPU, broadcast phase skipped\n");
	}

	wake_up_process(toucher);
	wait_for_completion(&d.attached);

	bcast_mm = time_flush_mm(mm, addr, TEST_COUNT);
	bcast_page = time_flush_page(vma, addr, TEST_COUNT);
	report_result("BCAST mm", bcast_mm);
	report_result("BCAST page", bcast_page);

	complete(&d.release);
	kthread_stop(toucher);
	mmdrop(mm);

	pr_info("tlbflush_test: ratio mm=%lu.%02lux  page=%lu.%02lux\n",
		bcast_mm / (local_mm ?: 1),
		(bcast_mm * 100 / (local_mm ?: 1)) % 100,
		bcast_page / (local_page ?: 1),
		(bcast_page * 100 / (local_page ?: 1)) % 100);

out_unmap:
	vm_munmap(addr, TEST_SIZE);
	return -EAGAIN;
}

static void tlbflush_test_exit(void)
{
}

module_init(tlbflush_test_init);
module_exit(tlbflush_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sayali Kulkarni <sk@gentwo.org>");
MODULE_DESCRIPTION("TLB flush local vs broadcast cycle benchmark");
