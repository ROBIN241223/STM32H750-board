#include "fc_sched.h"
#include <string.h>

void FC_Sched_Init(fc_sched_t *sched)
{
    if (sched == NULL) {
        return;
    }
    memset(sched, 0, sizeof(*sched));
    for (uint32_t i = 0U; i < FC_SCHED_MAX_CALLBACKS; i++) {
        sched->entry[i].interval_us = 0U;
        sched->entry[i].callback = NULL;
        sched->entry[i].context = NULL;
        sched->entry[i].elapsed = 0U;
        sched->entry[i].active = 0U;
    }
    sched->count = 0U;
    sched->initialized = 1U;
}

bool FC_Sched_Add(fc_sched_t *sched, uint32_t interval_us,
                  fc_sched_callback_t callback, void *context)
{
    if ((sched == NULL) || (callback == NULL) || (interval_us == 0U)) {
        return false;
    }
    if ((sched->initialized == 0U) || (sched->count >= FC_SCHED_MAX_CALLBACKS)) {
        return false;
    }
    uint32_t slot = sched->count;
    sched->entry[slot].interval_us = interval_us;
    sched->entry[slot].callback = callback;
    sched->entry[slot].context = context;
    sched->entry[slot].elapsed = 0U;
    sched->entry[slot].active = 1U;
    sched->count++;
    return true;
}

void FC_Sched_Tick(fc_sched_t *sched, uint32_t dt_us)
{
    if ((sched == NULL) || (sched->initialized == 0U)) {
        return;
    }
    if (dt_us == 0U) {
        return;
    }
    for (uint32_t i = 0U; i < sched->count; i++) {
        fc_sched_entry_t *e = &sched->entry[i];
        if ((e->active == 0U) || (e->callback == NULL)) {
            continue;
        }
        e->elapsed += dt_us;
        if (e->elapsed >= e->interval_us) {
            uint32_t elapsed_us = e->elapsed;
            /* keep the remainder so a 1 Hz callback stays drift-free     */
            e->elapsed %= e->interval_us;
            e->callback(e->context, elapsed_us);
        }
    }
}