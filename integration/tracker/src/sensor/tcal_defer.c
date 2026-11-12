#include "tcal_defer.h"

void tcal_defer_init(tcal_defer_state_t *st)
{
	st->verify_next_ms = 0;
	st->defer_since_ms = 0;
	st->defer_count = 0;
	st->skip_count = 0;
	st->pending_after_skip = false;
}

tcal_action_t tcal_defer_step(tcal_defer_state_t *st, int64_t now_ms,
                              bool recording, bool window_open)
{
    /* The previous verification window may remain open while a recording
     * postpones its flush. Do not start (or schedule) another window until
     * that flush actually completes. */
    if (window_open) {
        return TCAL_ACT_NONE;
    }
	bool due = (now_ms >= st->verify_next_ms) || (st->defer_since_ms != 0);

	if (recording) {
		if (!due) {
			/* Waiting for the next scheduled attempt; a skipped run stays
			 * pending so it can start the moment recording stops. */
			return TCAL_ACT_NONE;
		}
		if (st->defer_since_ms == 0) {
			st->defer_since_ms = now_ms;
			st->defer_count++;
			return TCAL_ACT_DEFER;
		}
		if (now_ms - st->defer_since_ms >= TCAL_DEFER_MAX_MS) {
			/* Bound the deferral: skip this run, but remember that it is still
			 * owed so it runs as soon as recording ends. */
			st->skip_count++;
			st->defer_since_ms = 0;
			st->pending_after_skip = true;
			st->verify_next_ms = now_ms + TCAL_VERIFY_INTERVAL_MS;
			return TCAL_ACT_SKIP;
		}
		return TCAL_ACT_DEFER;
	}

	if (!due && !st->pending_after_skip) {
		return TCAL_ACT_NONE;
	}

	/* Recording stopped: run the deferred (or skipped) verification at once. */
	st->defer_since_ms = 0;
	st->pending_after_skip = false;
	st->verify_next_ms = now_ms + TCAL_VERIFY_INTERVAL_MS;
	return TCAL_ACT_RUN;
}

void tcal_defer_window_closed(tcal_defer_state_t *st, int64_t closed_at_ms)
{
    /* Reschedule relative to the *actual* completion time, not the time
     * the window originally opened. Prevents back-to-back windows when the
     * flush was suspended throughout a long collection session. */
    st->verify_next_ms = closed_at_ms + TCAL_VERIFY_INTERVAL_MS;
    st->defer_since_ms = 0;
    st->pending_after_skip = false;
}
