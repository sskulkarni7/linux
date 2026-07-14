/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _TLBFLUSH_TEST_CYCLES_H
#define _TLBFLUSH_TEST_CYCLES_H

#include <linux/err.h>
#include <linux/perf_event.h>
#include <linux/printk.h>
#include <linux/smp.h>

static struct perf_event *cycles_event;

static int cycles_enable(void)
{
	struct perf_event_attr attr = {
		.type		= PERF_TYPE_HARDWARE,
		.config		= PERF_COUNT_HW_CPU_CYCLES,
		.size		= sizeof(attr),
		.pinned		= 1,
		.disabled	= 1,
		.exclude_user	= 1,
		.exclude_hv	= 1,
	};

	if (cycles_event)
		return 0;

	cycles_event = perf_event_create_kernel_counter(&attr,
							smp_processor_id(),
							NULL, NULL, NULL);
	if (IS_ERR(cycles_event)) {
		int ret = PTR_ERR(cycles_event);

		cycles_event = NULL;
		pr_err("tlbflush_test: failed to create CPU cycles perf event: %d\n",
		       ret);
		return ret;
	}

	perf_event_enable(cycles_event);

	return 0;
}

static void cycles_disable(void)
{
	if (!cycles_event)
		return;

	perf_event_release_kernel(cycles_event);
	cycles_event = NULL;
}

static u64 cycles_read(void)
{
	u64 value = 0;
	u64 enabled, running;

	if (!cycles_event)
		return 0;

	value = perf_event_read_value(cycles_event, &enabled, &running);
	if (enabled != running)
		pr_warn_once("tlbflush_test: CPU cycles perf event was multiplexed\n");

	return value;
}

#define CYCLES		cycles_read()
#define CYCLES_ENABLE	cycles_enable()
#define CYCLES_DISABLE	cycles_disable()

#endif /* _TLBFLUSH_TEST_CYCLES_H */
