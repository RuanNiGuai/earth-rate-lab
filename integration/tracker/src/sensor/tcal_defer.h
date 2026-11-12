/*
 * T-Cal verification deferral policy (pure logic, no Zephyr dependencies).
 *
 * The T-Cal verification window ends with flash writes (sys_write + tcal_save_point)
 * that stall raw TX. While a raw collection session is active those writes must be
 * postponed; the postponement is bounded so T-Cal still runs on a device that
 * records for a long time.
 *
 * Kept free of RTOS/board dependencies so the state machine can be unit tested on
 * the host (see tests/tcal_defer_test.c).
 */
#ifndef SLIMENRF_TCAL_DEFER_H
#define SLIMENRF_TCAL_DEFER_H

#include <stdbool.h>
#include <stdint.h>

#define TCAL_VERIFY_INTERVAL_MS (5 * 60 * 1000)
#define TCAL_DEFER_MAX_MS       (10 * 60 * 1000)

typedef enum {
	TCAL_ACT_NONE = 0, /* nothing due */
	TCAL_ACT_RUN,      /* open the verification window now */
	TCAL_ACT_DEFER,    /* due, but recording is active: postponed */
	TCAL_ACT_SKIP,     /* deferral limit reached: skip this run, rescheduled */
} tcal_action_t;

typedef struct {
	int64_t verify_next_ms; /* 0 = immediately due (first run) */
	int64_t defer_since_ms; /* 0 = not currently deferred */
	uint32_t defer_count;
	uint32_t skip_count;
	bool pending_after_skip; /* a run was skipped; run it as soon as recording stops */
} tcal_defer_state_t;

void tcal_defer_init(tcal_defer_state_t *st);

/* One policy tick. `recording` must come from the explicit collection session
 * state (DATA_COLLECT_ON/OFF), never from observed traffic.
 *
 * Execution contexts: the session flag is written in the ESB receive path and
 * this policy runs in the sensor thread, so the flag is read through a C11 atomic bool
 * and the guard below is a best-effort gate: a flash write that has already
 * started cannot be interrupted (nRF flash erase/write is not abortable). */
/* window_open is the actual tcal_verify_mode flag from calibration.c.
 * While it is true, the running verification owns the only open window;
 * a stalled flush must never cause another RUN. */
tcal_action_t tcal_defer_step(tcal_defer_state_t *st, int64_t now_ms,
                              bool recording, bool window_open);

/* Call only when the active verification window successfully completes its
 * Flash flush. Rebase the next deadline to its real completion time. */
void tcal_defer_window_closed(tcal_defer_state_t *st, int64_t closed_at_ms);

/* Guard for the flash-writing part of a flush: true means "do not write now".
 * This is the same helper the host tests exercise, and calibration.c calls it on
 * the real flush path. */
static inline bool tcal_defer_block_write(bool recording)
{
	return recording;
}

#endif
