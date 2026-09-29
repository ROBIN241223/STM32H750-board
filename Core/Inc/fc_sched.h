#ifndef FC_SCHED_H
#define FC_SCHED_H

#include <stdbool.h>
#include <stdint.h>

/* PX4-style cooperative rate scheduler ("work queue" flavour):
 *
 * A single high-rate tick source drives the scheduler (FC_Task, 500 Hz).
 * Every module registers a callback with FC_Sched_Add (interval in us) and is
 * invoked when `interval_us` of accumulated time has passed.  Callbacks run
 * cooperatively in the calling task, exactly like PX4 ScheduleOnInterval()
 * callbacks on the work queue: no per-module threads, no preemption between
 * callbacks.
 *
 * run the control loop at its own multiple of the tick, e.g. 2 ms at 500 Hz,
 * a 10 ms module just accumulates ticks.
 */

#define FC_SCHED_MAX_CALLBACKS 8U

typedef void (*fc_sched_callback_t)(void *context, uint32_t elapsed_us);

typedef struct {
    uint32_t interval_us;       /* run every interval_us of elapsed time      */
    fc_sched_callback_t callback;
    void *context;
    uint32_t elapsed;           /* leftover time since last run (us)         */
    uint32_t active;
} fc_sched_entry_t;

typedef struct {
    fc_sched_entry_t entry[FC_SCHED_MAX_CALLBACKS];
    uint32_t count;
    uint32_t initialized;
} fc_sched_t;

void FC_Sched_Init(fc_sched_t *sched);
bool FC_Sched_Add(fc_sched_t *sched, uint32_t interval_us,
                  fc_sched_callback_t callback, void *context);
void FC_Sched_Tick(fc_sched_t *sched, uint32_t dt_us);

#endif