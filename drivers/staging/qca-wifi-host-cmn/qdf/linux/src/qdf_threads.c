/*
 * Copyright (c) 2014-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * DOC: qdf_threads
 * QCA driver framework (QDF) thread APIs
 */

/* Include Files */
#include <qdf_threads.h>
#include <qdf_types.h>
#include <qdf_trace.h>
#include <qdf_module.h>
#include <linux/jiffies.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
#include <linux/sched.h>
#else
#include <linux/sched/signal.h>
#endif /* KERNEL_VERSION(4, 11, 0) */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/stacktrace.h>
#include <qdf_defer.h>

/* Function declarations and documenation */

/**
 *  qdf_sleep() - sleep
 *  @ms_interval : Number of milliseconds to suspend the current thread.
 *  A value of 0 may or may not cause the current thread to yield.
 *
 *  This function suspends the execution of the current thread
 *  until the specified time out interval elapses.
 *
 *  Return: none
 */
void qdf_sleep(uint32_t ms_interval)
{
	if (in_interrupt()) {
		QDF_TRACE(QDF_MODULE_ID_QDF, QDF_TRACE_LEVEL_ERROR,
			  "%s cannot be called from interrupt context!!!",
			  __func__);
		return;
	}
	msleep_interruptible(ms_interval);
}
qdf_export_symbol(qdf_sleep);

/**
 *  qdf_sleep_us() - sleep
 *  @us_interval : Number of microseconds to suspend the current thread.
 *  A value of 0 may or may not cause the current thread to yield.
 *
 *  This function suspends the execution of the current thread
 *  until the specified time out interval elapses.
 *
 *  Return : none
 */
void qdf_sleep_us(uint32_t us_interval)
{
	unsigned long timeout = usecs_to_jiffies(us_interval) + 1;

	if (in_interrupt()) {
		QDF_TRACE(QDF_MODULE_ID_QDF, QDF_TRACE_LEVEL_ERROR,
			  "%s cannot be called from interrupt context!!!",
			  __func__);
		return;
	}

	while (timeout && !signal_pending(current))
		timeout = schedule_timeout_interruptible(timeout);
}
qdf_export_symbol(qdf_sleep_us);

/**
 *  qdf_busy_wait() - busy wait
 *  @us_interval : Number of microseconds to busy wait.
 *
 *  This function places the current thread in busy wait until the specified
 *  time out interval elapses. If the interval is greater than 50us on WM, the
 *  behaviour is undefined.
 *
 *  Return : none
 */
void qdf_busy_wait(uint32_t us_interval)
{
	udelay(us_interval);
}
qdf_export_symbol(qdf_busy_wait);

#if defined(BUILD_DEBUG_VERSION)
#define QDF_PRINT_TRACE_COUNT 32
void qdf_print_thread_trace(qdf_thread_t *thread)
{
	const int spaces = 4;
	struct task_struct *task = (struct task_struct *)thread;
	unsigned long entries[QDF_PRINT_TRACE_COUNT] = {0};
	struct stack_trace trace = {
		.nr_entries = 0,
		.skip = 0,
		.entries = &entries[0],
		.max_entries = QDF_PRINT_TRACE_COUNT,
	};

	save_stack_trace_tsk(task, &trace);
	print_stack_trace(&trace, spaces);
}
#else
void qdf_print_thread_trace(qdf_thread_t *thread) { }
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0) */
qdf_export_symbol(qdf_print_thread_trace);

qdf_thread_t *qdf_get_current_task(void)
{
	return current;
}
qdf_export_symbol(qdf_get_current_task);
