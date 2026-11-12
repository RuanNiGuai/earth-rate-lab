#include "sensor/f04_summary.h"
#include "sensor/calwire.h"
#include "test_mode.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(test_mode, LOG_LEVEL_INF);

static bool test_mode_active;

bool test_mode_get(void)
{
	return test_mode_active || f04_summary_active() || calwire_active();
}

void test_mode_set(bool enable)
{
	if (test_mode_active == enable)
		return;
	test_mode_active = enable;
	LOG_INF("Test mode %s", enable ? "ENABLED" : "DISABLED");
}
