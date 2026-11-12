/* Host-side unit tests for the T-Cal deferral policy (no Zephyr needed). */
#include <stdio.h>
#include <string.h>
#include "tcal_defer.h"

static int failures;

#define CHECK(cond, msg)                                                              \
	do {                                                                          \
		if (!(cond)) {                                                        \
			printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);       \
			failures++;                                                   \
		} else {                                                              \
			printf("ok  : %s\n", msg);                                    \
		}                                                                     \
	} while (0)

static const int64_t T = 1000000; /* arbitrary base time */

static void test_fresh_runs_once(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	CHECK(tcal_defer_step(&st, T, false, false) == TCAL_ACT_RUN, "fresh state runs immediately");
	CHECK(st.verify_next_ms == T + TCAL_VERIFY_INTERVAL_MS, "next run scheduled +300 s");
	CHECK(tcal_defer_step(&st, T + 1000, false, false) == TCAL_ACT_NONE, "no double run");
}

static void test_recording_before_window(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	CHECK(tcal_defer_step(&st, T, true, false) == TCAL_ACT_DEFER, "recording defers the run");
	CHECK(st.defer_count == 1, "defer counted once");
	CHECK(tcal_defer_step(&st, T + 1000, true, false) == TCAL_ACT_DEFER, "still deferred");
	CHECK(st.defer_count == 1, "defer not re-counted while active");
	CHECK(tcal_defer_step(&st, T + 2000, false, false) == TCAL_ACT_RUN, "runs once recording ends");
	CHECK(tcal_defer_step(&st, T + 3000, false, false) == TCAL_ACT_NONE, "deferred run happens once");
}

static void test_defer_limit(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	CHECK(tcal_defer_step(&st, T, true, false) == TCAL_ACT_DEFER, "defer starts");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS - 1, true, false) == TCAL_ACT_DEFER,
	      "still deferring just before the limit");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false) == TCAL_ACT_SKIP,
	      "skips at the limit");
	CHECK(st.skip_count == 1, "skip counted");
	CHECK(st.defer_since_ms == 0, "defer state cleared after skip");
	CHECK(st.verify_next_ms == T + TCAL_DEFER_MAX_MS + TCAL_VERIFY_INTERVAL_MS,
	      "next attempt rescheduled after skip");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 1000, true, false) == TCAL_ACT_NONE,
	      "not due again right after skip");
}

static void test_skip_then_recur(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	(void)tcal_defer_step(&st, T, true, false);                    /* defer */
	(void)tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false); /* skip */
	int64_t next = st.verify_next_ms;
	CHECK(tcal_defer_step(&st, next, true, false) == TCAL_ACT_DEFER, "defers again in the new cycle");
	CHECK(st.defer_count == 2, "second deferral counted");
	CHECK(st.skip_count == 1, "skip count unchanged");
}

static void test_skip_then_recording_ends_runs_immediately(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	CHECK(tcal_defer_step(&st, T, true, false) == TCAL_ACT_DEFER, "defer before the limit");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false) == TCAL_ACT_SKIP,
	      "skip at the limit");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 1000, false, false) == TCAL_ACT_RUN,
	      "skipped run executes immediately once recording ends");
	CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 2000, false, false) == TCAL_ACT_NONE,
	      "skipped run happens only once");
}

static void test_skip_while_still_recording_waits(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	(void)tcal_defer_step(&st, T, true, false);
	(void)tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false); /* skip */
	int64_t next = st.verify_next_ms;
	CHECK(tcal_defer_step(&st, next - 1000, true, false) == TCAL_ACT_NONE,
	      "still recording after skip: waits for the next schedule");
	CHECK(tcal_defer_step(&st, next, true, false) == TCAL_ACT_DEFER,
	      "next schedule defers again");
	CHECK(st.defer_count == 2, "new cycle counted separately");
	CHECK(st.skip_count == 1, "skip count unchanged by the new cycle");
}

static void test_flush_guard(void)
{
	CHECK(tcal_defer_block_write(true) == true, "flash write blocked while recording");
	CHECK(tcal_defer_block_write(false) == false, "flash write allowed when idle");
}

static void test_restart_clears_state(void)
{
	tcal_defer_state_t st;
	tcal_defer_init(&st);
	(void)tcal_defer_step(&st, T, true, false);
	CHECK(st.defer_since_ms != 0, "precondition: deferring before restart");
	tcal_defer_init(&st);
	CHECK(st.defer_since_ms == 0 && st.defer_count == 0 && st.skip_count == 0,
	      "restart clears deferral state");
	CHECK(tcal_defer_step(&st, T + 5, false, false) == TCAL_ACT_RUN, "runs after restart");
}


/* Integrated timing model: window opens; recording begins while its flush is
 * pending; 30 minutes elapse; recording ends; exactly one original window
 * flushes, and no second window opens until 300 s after actual completion. */
static void test_mid_window_recording_long_flush(void)
{
    tcal_defer_state_t st;
    tcal_defer_init(&st);
    CHECK(tcal_defer_step(&st, T, false, false) == TCAL_ACT_RUN,
          "open verification window before recording starts");
    CHECK(tcal_defer_step(&st, T + 1000, true, true) == TCAL_ACT_NONE,
          "window already open: no new run when recording starts");
    CHECK(tcal_defer_block_write(true),
          "flash flush blocked while recording is active");
    CHECK(tcal_defer_step(&st, T + 1800000, true, true) == TCAL_ACT_NONE,
          "window still open after 30 min: no duplicate run or deferral");
    CHECK(st.defer_count == 0 && st.skip_count == 0,
          "long pending window does not consume extra deferrals");
    CHECK(!tcal_defer_block_write(false),
          "recording ended: pending flush may now complete");
    tcal_defer_window_closed(&st, T + 1800100);
    CHECK(st.verify_next_ms == T + 2100100,
          "next verification rebased to completion +300 s");
    CHECK(tcal_defer_step(&st, T + 1801100, false, false) == TCAL_ACT_NONE,
          "next policy tick does not reopen immediately");
    CHECK(tcal_defer_step(&st, T + 2100099, false, false) == TCAL_ACT_NONE,
          "no second verification before the new deadline");
    CHECK(tcal_defer_step(&st, T + 2100100, false, false) == TCAL_ACT_RUN,
          "exactly one new verification at the new deadline");
}

static void test_open_window_blocks_overdue_reschedule(void)
{
    tcal_defer_state_t st;
    tcal_defer_init(&st);
    CHECK(tcal_defer_step(&st, T, false, false) == TCAL_ACT_RUN,
          "ordinary verification window opens");
    CHECK(tcal_defer_step(&st, T + 600000, false, true) == TCAL_ACT_NONE,
          "even overdue by 10 min, already-open window cannot run twice");
    tcal_defer_window_closed(&st, T + 600050);
    CHECK(tcal_defer_step(&st, T + 601000, false, false) == TCAL_ACT_NONE,
          "completion reschedule suppresses immediate duplicate");
}

static void test_skipped_defer_rebased_after_window_close(void)
{
    tcal_defer_state_t st;
    tcal_defer_init(&st);
    CHECK(tcal_defer_step(&st, T, true, false) == TCAL_ACT_DEFER,
          "recording starts with due verification");
    CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false) == TCAL_ACT_SKIP,
          "run skipped after maximum deferral");
    CHECK(st.pending_after_skip, "pending run remains owed");
    CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 1000, false, false) == TCAL_ACT_RUN,
          "recording end schedules exactly one make-up run");
    CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 180000, false, true)
          == TCAL_ACT_NONE,
          "no duplicate while make-up window is open");
    tcal_defer_window_closed(&st, T + TCAL_DEFER_MAX_MS + 180010);
    CHECK(!st.pending_after_skip && st.defer_since_ms == 0,
          "actual flush completion clears pending/defer state");
    CHECK(tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS + 181000, false, false)
          == TCAL_ACT_NONE,
          "make-up flush is not immediately followed by another window");
}

static void test_window_closure_does_not_zero_counters(void)
{
    tcal_defer_state_t st;
    tcal_defer_init(&st);
    (void)tcal_defer_step(&st, T, true, false);
    (void)tcal_defer_step(&st, T + TCAL_DEFER_MAX_MS, true, false);
    CHECK(st.defer_count == 1 && st.skip_count == 1,
          "precondition: deferral and skipped run counted");
    tcal_defer_window_closed(&st, T + TCAL_DEFER_MAX_MS + 2000);
    CHECK(st.defer_count == 1 && st.skip_count == 1,
          "closing a window preserves lifetime counters");
}

int main(void)
{
	printf("tcal_defer policy tests\n");
	printf("-----------------------\n");
	test_fresh_runs_once();
	test_recording_before_window();
	test_defer_limit();
	test_skip_then_recur();
	test_skip_then_recording_ends_runs_immediately();
	test_skip_while_still_recording_waits();
	test_flush_guard();
	test_restart_clears_state();
    test_mid_window_recording_long_flush();
    test_open_window_blocks_overdue_reschedule();
    test_skipped_defer_rebased_after_window_close();
    test_window_closure_does_not_zero_counters();
	printf("-----------------------\n");
	if (failures) {
		printf("%d FAILURE(S)\n", failures);
		return 1;
	}
	printf("ALL PASS\n");
	return 0;
}
