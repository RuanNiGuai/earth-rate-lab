#include "globals.h"
#include "test_mode.h"
#include "sensor/sensor.h"
#include "sensor/calibration.h"
#include "connection/connection.h"
#include "connection/esb.h"
#include "watchdog.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <hal/nrf_gpio.h>

#include "system.h"
#include "build_defines.h"
#include <math.h>

static struct nvs_fs fs;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION_SIZE FIXED_PARTITION_SIZE(NVS_PARTITION)

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

#if DT_NODE_HAS_PROP(DT_ALIAS(sw0), gpios) // Alternate button if available to use as "reset key"
#define BUTTON_EXISTS true
static void button_thread(void);
K_THREAD_DEFINE(
	button_thread_id,
	1024,
	button_thread,
	NULL,
	NULL,
	NULL,
	6,
	0,
	0
); // TODO: stack increased because of reboot request (to 512) and sensor scan (to 1024)
#else
#pragma message "Button GPIO does not exist"
#endif

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define CLKOUT_NODE DT_NODELABEL(pwmclock)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dock_gpios)
#define DOCK_EXISTS true
static const struct gpio_dt_spec dock = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dock_gpios);
#else
#pragma message "Dock sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, chg_gpios)
#define CHG_EXISTS true
static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, chg_gpios);
#else
#pragma message "Charge sense GPIO does not exist"
#endif
#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, stby_gpios)
#define STBY_EXISTS true
static const struct gpio_dt_spec stby = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, stby_gpios);
#else
#pragma message "Standby sense GPIO does not exist"
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, clk_gpios)
#define CLK_EN_EXISTS true
static const struct gpio_dt_spec clk_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, clk_gpios);
static const struct pwm_dt_spec clk_out = {0};
#elif DT_NODE_HAS_PROP(CLKOUT_NODE, pwms)
#define CLK_OUT_EXISTS true
static const struct pwm_dt_spec clk_out = PWM_DT_SPEC_GET(CLKOUT_NODE);
#else
#pragma message "Clock enable GPIO or clock PWM out does not exist"
static const struct pwm_dt_spec clk_out = {0};
#endif

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

void configure_sense_pins(void)
{
	// Configure dock sense
	bool docked = dock_read();
#if DOCK_EXISTS
	if (docked) {
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_NOPULL); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_HIGH);
	} else {
		nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_PULLUP); // Still works
		nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, dock_gpios), NRF_GPIO_PIN_SENSE_LOW);
	}
	LOG_INF("Configured dock sense");
#endif
	// Configure chgstat sense
	if (!docked) {
		bool ignore_charge_wake = IGNORE_CHARGE_WAKE_ON_VBUS && vbus_read();
		if (ignore_charge_wake) {
			LOG_INF("Skipped charge wake sense while VBUS is present");
		}
#if CHG_EXISTS
		if (!ignore_charge_wake) {
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(
				NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, chg_gpios),
				chg_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW
			);
			LOG_INF("Configured chg sense");
		}
#endif
#if STBY_EXISTS
		if (!ignore_charge_wake) {
			nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios), NRF_GPIO_PIN_PULLUP);
			nrf_gpio_cfg_sense_set(
				NRF_DT_GPIOS_TO_PSEL(ZEPHYR_USER_NODE, stby_gpios),
				stby_read() ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW
			);
			LOG_INF("Configured stby sense");
		}
#endif
	}
	// Configure sw0 sense
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios), NRF_GPIO_PIN_SENSE_LOW);
	LOG_INF("Configured sw0 sense");
#endif
}

static bool nvs_init = false;

static inline bool sys_nvs_init(void)
{
	if (nvs_init) {
		return true;
	}
	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // starting at NVS_PARTITION_OFFSET
	if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
		LOG_ERR("Failed to get page info");
		return false;
	}
	fs.sector_size = info.size; // sector_size equal to the pagesize
	fs.sector_count = 6U;       // 6 sectors
	int err = nvs_mount(&fs);
	if (err) {
		LOG_ERR("Failed to mount NVS, error: %d", err);
		return false;
	}
	nvs_init = true;
	return true;
}

static bool ram_retention_valid = false;
static bool startup_system_off_wake;
static bool startup_reset_reason_unavailable;
static bool wom_resume;
static uint32_t startup_reset_reason;
static bool startup_wom_armed;
static uint8_t startup_fusion_result;

/* Capture before watchdog_early_check clears RESETREAS at PRE_KERNEL_1 priority 1. */
static int sys_capture_wake_reason(void)
{
#ifdef NRF_RESET
#if defined(RESET_RESETREAS_OFF_Msk)
    uint32_t reason=NRF_RESET->RESETREAS;
    startup_reset_reason=reason;
    startup_reset_reason_unavailable=(reason==0);
    startup_system_off_wake=(reason & RESET_RESETREAS_OFF_Msk) &&
        !(reason & (RESET_RESETREAS_DOG_Msk | RESET_RESETREAS_SREQ_Msk | RESET_RESETREAS_RESETPIN_Msk));
#endif
#else
    uint32_t reason=NRF_POWER->RESETREAS;
    startup_reset_reason=reason;
    startup_reset_reason_unavailable=(reason==0);
    startup_system_off_wake=(reason & POWER_RESETREAS_OFF_Msk) &&
        !(reason & (POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_RESETPIN_Msk));
#endif
    return 0;
}
SYS_INIT(sys_capture_wake_reason, PRE_KERNEL_1, 0);

bool sys_is_wom_resume(void) { return wom_resume; }

static void sys_restore_wom_intent(bool valid_ram)
{
    startup_wom_armed=valid_ram && retained->wom_resume_armed;
    /* Some UF2 bootloaders consume RESETREAS before entering the application.
     * A zero reason is usable only with CRC-valid, one-shot WoM intent; explicit
     * reboot/shutdown clear that intent and known reset faults never resume. */
    wom_resume=valid_ram && retained->wom_resume_armed &&
        (startup_system_off_wake || startup_reset_reason_unavailable);
    if(wom_resume && startup_reset_reason_unavailable)
        LOG_INF("WoM resume from retained intent; boot reset reason unavailable");
    retained->wom_resume_armed=false;
    retained_update();
}

uint8_t sys_startup_diagnostic_flags(void)
{
    return 1u | (retained_boot_status() << 1) |
        (startup_wom_armed ? 8u : 0u) | (wom_resume ? 16u : 0u) |
        (startup_fusion_result == 1 ? 32u : 0u) |
        (startup_fusion_result == 2 ? 64u : 0u) |
        (startup_fusion_result == 3 ? 128u : 0u);
}

uint8_t sys_startup_reset_flags(void)
{
    /* nRF52840 wire encoding: OFF, DOG, SREQ, PIN, unavailable. */
    return ((startup_reset_reason & (1u << 16)) ? 1u : 0u) |
        ((startup_reset_reason & 2u) ? 2u : 0u) |
        ((startup_reset_reason & 4u) ? 4u : 0u) |
        ((startup_reset_reason & 1u) ? 8u : 0u) |
        (startup_reset_reason == 0 ? 16u : 0u);
}

void sys_startup_fusion_result(uint8_t result) { startup_fusion_result=result; }

void sys_print_startup_diagnostics(void)
{
    printk("BOOT reset=0x%08x retained=%p crc=%u build=%u armed=%u wom=%u fusion=%u\n",
        startup_reset_reason, (void *)retained,
        retained_boot_status() & 1u, (retained_boot_status() >> 1) & 1u,
        startup_wom_armed, wom_resume, startup_fusion_result);
}

static void sys_validate_gyro_parameters(void)
{
    for (int i=0;i<3;i++) {
        if (!isfinite(retained->gyroSensScale[i]) ||
            retained->gyroSensScale[i] <= 0.5f || retained->gyroSensScale[i] >= 1.5f) {
            retained->gyroSensScale[i]=1.0f;
            LOG_WRN("Invalid stored gyro scale on axis %d; using 1",i);
        }
    }
#if CONFIG_SENSOR_USE_TCAL
    for (int i=0;i<9;i++) {
        if (!isfinite(retained->gyroGsens[i]) || fabsf(retained->gyroGsens[i])>10.0f) {
            memset(retained->gyroGsens,0,sizeof(retained->gyroGsens));
            LOG_WRN("Invalid stored gyro g-sensitivity; disabled");
            break;
        }
    }
#endif
    retained_update();
}

static int sys_retained_init(void)
{
#ifdef NRF_RESET
	bool reset_pin_reset = NRF_RESET->RESETREAS & 0x01;
#else
	bool reset_pin_reset = NRF_POWER->RESETREAS & 0x01;
#endif
	// on most nrf, reset by pin reset will clear retained
	if (!reset_pin_reset) { // if reset reason is not by pin reset, system automatically trusts retained state
		ram_retention_valid = true;
	}
	// All contents of NVS was stored in RAM to not need initializing NVS often
	bool valid_ram=retained_validate();
    sys_restore_wom_intent(valid_ram);
	if (!valid_ram) // Check ram retention
	{
		LOG_WRN("Invalidated RAM");
		if (!sys_nvs_init()) {
			LOG_ERR("NVS init failed during retained init, cannot restore data from flash");
			// Don't try to read from NVS if it failed to init
			// The data will be zero/uninitialized but we can't recover it
			retained_update();
			return 0;
		}
        field_reference_t earth_record;
        sys_read(EARTH_REFERENCE_ID, &earth_record, sizeof(earth_record));
        if (field_reference_valid(&earth_record)) {
            memcpy(retained->earthCalBody, earth_record.body, sizeof(earth_record.body));
            retained->earthCalValid = true;
        }
        /* On old firmware, ID40 may contain a matrix, not the 1-byte axis.
         * nvs_read returns STORED length even with a small destination buffer. */
        uint8_t axis = 0;
        int axis_len = nvs_read(&fs, EARTH_UP_AXIS_ID, &axis, sizeof(axis));
        if (axis_len != (int)sizeof(axis)) {
            int old_len = nvs_read(&fs, LEGACY_EARTH_GSENS_ID, &axis, sizeof(axis));
            if (old_len != (int)sizeof(axis) || axis > 5) {
                axis = 0; /* old matrix overwrote the axis: cannot recover */
                LOG_WRN("Legacy earth up-axis not recoverable, defaulting to +X");
            } else {
                /* Read-only boot compatibility. Migration is explicit. */
                LOG_INF("Legacy earth up-axis ID40 loaded (not migrated)");
            }
        }
        retained->earthUpAxis = axis <= 5 ? axis : 0;
        {
            float pose_q[4] = {0};
            sys_read(POSE_QUAT_ID, pose_q, sizeof(pose_q));
            float qn = pose_q[0]*pose_q[0] + pose_q[1]*pose_q[1] +
                       pose_q[2]*pose_q[2] + pose_q[3]*pose_q[3];
            if (isfinite(qn) && qn > 0.9f && qn < 1.1f) {
                memcpy(retained->poseQuat, pose_q, sizeof(pose_q));
                retained->poseQuatValid = true;
            }
        }
        /* Preserve 10 existing devices' legacy gsens. Validate record length,
         * finite coefficients, and exact NVS readback BEFORE migration. */
        float old_gsens[9] = {0};
        int gs_len = nvs_read(&fs, GYRO_GSENS_ID, retained->gyroGsens,
                              sizeof(retained->gyroGsens));
        bool gs_new_valid = (gs_len == (int)sizeof(retained->gyroGsens));
        if (gs_new_valid) for (int j = 0; j < 9; j++) {
            if (!isfinite(retained->gyroGsens[j]) ||
                fabsf(retained->gyroGsens[j]) > 10.0f) gs_new_valid = false;
        }
        if (!gs_new_valid) {
            memset(retained->gyroGsens, 0, sizeof(retained->gyroGsens));
            int legacy_len = nvs_read(&fs, LEGACY_EARTH_GSENS_ID,
                                      old_gsens, sizeof(old_gsens));
            bool legacy_valid = legacy_len == (int)sizeof(old_gsens);
            if (legacy_valid) for (int j = 0; j < 9; j++) {
                if (!isfinite(old_gsens[j]) || fabsf(old_gsens[j]) > 10.0f)
                    legacy_valid = false;
            }
            if (legacy_valid) {
                memcpy(retained->gyroGsens, old_gsens, sizeof(old_gsens));
                /* Do NOT write NVS during startup. Explicit nvs migrate only. */
                LOG_INF("Legacy gsens ID40 loaded in RAM (not migrated)");
            }
        }
		// read from nvs to retained
		sys_read(PAIRED_ID, &retained->paired_addr, sizeof(retained->paired_addr));
		sys_read(MAIN_SENSOR_DATA_ID, &retained->sensor_data, sizeof(retained->sensor_data));
		sys_read(MAIN_ACCEL_BIAS_ID, &retained->accelBias, sizeof(retained->accelBias));
		sys_read(MAIN_GYRO_BIAS_ID, &retained->gyroBias, sizeof(retained->gyroBias));
		sys_read(MAIN_MAG_BIAS_ID, &retained->magBAinv, sizeof(retained->magBAinv));
		sys_read(MAIN_ACC_6_BIAS_ID, &retained->accBAinv, sizeof(retained->accBAinv));
		sys_read(BATT_STATS_CURVE_ID, &retained->battery_pptt_curve, sizeof(retained->battery_pptt_curve));
		sys_read(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, sizeof(retained->gyroSensScale));
		// If gyroSensScale was never set in NVS (all zeros), restore default values
		if (retained->gyroSensScale[0] == 0.0f &&
		    retained->gyroSensScale[1] == 0.0f &&
		    retained->gyroSensScale[2] == 0.0f) {
			retained->gyroSensScale[0] = 1.0f;
			retained->gyroSensScale[1] = 1.0f;
			retained->gyroSensScale[2] = 1.0f;
		}
#if CONFIG_SENSOR_USE_TCAL
		sys_read(MAIN_GYRO_TEMP_ID, &retained->gyroTemp, sizeof(retained->gyroTemp));
		sys_read(MAIN_GYRO_TCAL_POINTS_ID, &retained->tempCalPoints, sizeof(retained->tempCalPoints));
		sys_read(MAIN_GYRO_TCAL_COEFFS_ID, &retained->tempCalCoeffs, sizeof(retained->tempCalCoeffs));
		// tempCalCorrectionOffset is retained for compatibility only; no longer used.
		sys_read(MAIN_GYRO_TCAL_STATE_ID, &retained->tempCalState, sizeof(retained->tempCalState));
		/* Re-bin supported layouts by stored temperature without startup NVS writes. */
		sensor_tcal_validate_layout();
#endif
		sys_read(RF_CHANNEL_ID, &retained->rf_channel, sizeof(retained->rf_channel));
		sys_read(MAG_ENABLED_ID, &retained->mag_enabled, sizeof(retained->mag_enabled));
		retained_update();
	} else {
		LOG_INF("Validated RAM");
		ram_retention_valid = true;
		// Still need to init NVS for later sys_read/sys_write calls (e.g., battery_tracker)
		sys_nvs_init();
	}
    sys_validate_gyro_parameters();
	return 0;
}

SYS_INIT(sys_retained_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// read from retained
uint8_t reboot_counter_read(void)
{
	if (!ram_retention_valid) // system cannot trust retained state, read from nvs
	{
		sys_nvs_init();
		nvs_read(&fs, RBT_CNT_ID, &retained->reboot_counter, sizeof(retained->reboot_counter));
		retained_update();
	}
	return retained->reboot_counter;
}

// write to retained
void reboot_counter_write(uint8_t reboot_counter)
{
	retained->reboot_counter = reboot_counter;
	if (!ram_retention_valid) // system cannot trust retained state, write to nvs
	{
		sys_nvs_init();
		nvs_write(&fs, RBT_CNT_ID, &retained->reboot_counter, sizeof(retained->reboot_counter));
	}
	retained_update();
}

/* Save only after a successful flash write and exact readback. */
bool sys_earth_reference_save(const float body[3])
{
    field_reference_t record, verify;
    if (!field_reference_make(&record, body) || !sys_nvs_init()) return false;
    if (nvs_write(&fs, EARTH_REFERENCE_ID, &record, sizeof(record)) < 0) return false;
    if (nvs_read(&fs, EARTH_REFERENCE_ID, &verify, sizeof(verify)) != sizeof(verify) ||
        memcmp(&record, &verify, sizeof(record)) || !field_reference_valid(&verify)) return false;
    memcpy(retained->earthCalBody, body, sizeof(record.body));
    retained->earthCalValid = true;
    retained_update();
    return true;
}

bool sys_pose_quat_save(const float q[4])
{
    if (!q || !sys_nvs_init()) return false;
    float n = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!isfinite(n) || n < 0.9f || n > 1.1f) return false;
    if (nvs_write(&fs, POSE_QUAT_ID, q, sizeof(float) * 4) < 0) return false;
    float verify[4];
    if (nvs_read(&fs, POSE_QUAT_ID, verify, sizeof(verify)) != (int)sizeof(verify)) return false;
    if (memcmp(q, verify, sizeof(verify))) return false;
    memcpy(retained->poseQuat, q, sizeof(retained->poseQuat));
    retained->poseQuatValid = true;
    retained_update();
    return true;
}

bool sys_earth_up_axis_save(uint8_t axis)
{
    if (!sys_nvs_init()) return false;
    if (nvs_write(&fs, EARTH_UP_AXIS_ID, &axis, sizeof(axis)) < 0) return false;
    uint8_t verify = 0xFF;
    if (nvs_read(&fs, EARTH_UP_AXIS_ID, &verify, sizeof(verify)) != sizeof(verify)) return false;
    if (verify != axis) return false;
    retained->earthUpAxis = axis;
    retained_update();
    return true;
}

void sys_earth_reference_default(float body[3])
{
    /* Shared +X-up reference for the charging box with its top edge facing
     * north. Derived on 2026-10-29 by booting at the old zero pose and
     * rotating the box to north (quaternion method); norm equals the Earth
     * rate, site latitude about 36.65 deg. */
    static const float base[3] = {0.002493997f, -0.000105918f, 0.003350382f};
    int idx = retained->earthUpAxis / 2;
    int sign = (retained->earthUpAxis & 1) ? -1 : 1;
    if (idx < 0 || idx > 2) { idx = 0; sign = 1; }
    int p = (idx + 1) % 3;
    int q = (idx + 2) % 3;
    body[idx] = base[0] * (float)sign;
    body[p] = base[1];
    body[q] = base[2] * (float)sign;
}

/* Used only from the sensor thread for coherent calibration commits. */
static bool sys_gyro_vector_save(uint16_t id, float *target, const float *values, size_t count)
{
    float verify[9];
    if (!values || !target || count>9 || !sys_nvs_init()) return false;
    size_t bytes=count*sizeof(float);
    if (nvs_write(&fs,id,values,bytes)<0) return false;
    if (nvs_read(&fs,id,verify,bytes)!=(int)bytes || memcmp(values,verify,bytes)) return false;
    memcpy(target,values,bytes);
    retained_update();
    return true;
}

bool sys_gyro_scale_save(const float scales[3])
{
    if (!scales) return false;
    for (int i=0;i<3;i++) {
        if (!isfinite(scales[i]) || scales[i]<=0.5f || scales[i]>=1.5f) return false;
    }
    return sys_gyro_vector_save(MAIN_GYRO_SENS_ID,retained->gyroSensScale,scales,3);
}

bool sys_gyro_gsens_save(const float matrix[9])
{
#if CONFIG_SENSOR_USE_TCAL
    if (!matrix) return false;
    for (int i=0;i<9;i++) {
        if (!isfinite(matrix[i]) || fabsf(matrix[i])>10.0f) return false;
    }
    return sys_gyro_vector_save(GYRO_GSENS_ID,retained->gyroGsens,matrix,9);
#else
    return false;
#endif
}

/* Explicit migration, safe to call from console after verifying NVS capacity.
 * Never attempts to guess missing up-axis from the 36-byte ID40 matrix. */
void sys_nvs_legacy_status(void)
{
    if (!sys_nvs_init()) { printk("NVS not mounted\n"); return; }
    uint8_t temp[36];
    int old=nvs_read(&fs,LEGACY_EARTH_GSENS_ID,temp,sizeof(temp));
    int gs=nvs_read(&fs,GYRO_GSENS_ID,temp,sizeof(temp));
    int ax=nvs_read(&fs,EARTH_UP_AXIS_ID,temp,sizeof(temp));
    printk("NVS legacy ID40 len=%d, gsens ID42 len=%d, up-axis ID43 len=%d\n",old,gs,ax);
    if(old==36 && ax!=1)printk("Legacy ID40 stores gsens, old up-axis is irrecoverable; manual axis verification required\n");
}
void sys_nvs_legacy_migrate(void)
{
    if (!sys_nvs_init()) { printk("NVS not mounted\n"); return; }
    uint8_t buf[36]={0};
    int old_len=nvs_read(&fs,LEGACY_EARTH_GSENS_ID,buf,sizeof(buf));
    int gs_len=nvs_read(&fs,GYRO_GSENS_ID,buf,sizeof(buf));
    if(gs_len==36) {
        printk("NVS gsens ID42 already populated; no overwrite\n");
    } else if(old_len==36) {
        float m[9];
        int got=nvs_read(&fs,LEGACY_EARTH_GSENS_ID,m,sizeof(m));
        bool valid=got==(int)sizeof(m);
        for(int i=0;i<9 && valid;i++)
            if(!isfinite(m[i]) || fabsf(m[i])>10.f)valid=false;
        if(valid) {
            int result=nvs_write(&fs,GYRO_GSENS_ID,m,sizeof(m));
            float verify[9];
            if(result>=0 && nvs_read(&fs,GYRO_GSENS_ID,verify,sizeof(verify))==
                   (int)sizeof(verify) && !memcmp(verify,m,sizeof(verify)))
                printk("NVS gsens ID40 -> ID42 verified; ID40 retained\n");
            else printk("NVS gsens migration FAILED; old ID40 preserved\n");
        } else printk("NVS gsens legacy record invalid; no migration\n");
    }
    int ax_len=nvs_read(&fs,EARTH_UP_AXIS_ID,buf,1);
    if(ax_len==1) {
        printk("NVS up-axis ID43 already populated\n");
    } else if(old_len==1) {
        uint8_t axis=0;
        if(nvs_read(&fs,LEGACY_EARTH_GSENS_ID,&axis,1)==1 && axis<=5) {
            int r=nvs_write(&fs,EARTH_UP_AXIS_ID,&axis,1);
            uint8_t v=255;
            if(r>=0 && nvs_read(&fs,EARTH_UP_AXIS_ID,&v,1)==1 && v==axis)
                printk("NVS up-axis ID40 -> ID43 verified\n");
            else printk("NVS up-axis migration FAILED\n");
        }
    } else {
        printk("NVS up-axis cannot be inferred from legacy matrix; current RAM axis=%u; use existing Earth calibration to save a verified axis\n",retained->earthUpAxis);
    }
}

/* Archive NVS operations MUST ONLY be invoked by the user-activated worker.
 * No archive I/O in SYS_INIT, sensor ISR or first gyro sample. */
#define ARC_ID_MIN 100
#define ARC_ID_MAX 131
#define ARC_IO_MAX 1900
int sys_arc_read(uint16_t id, void *dst, size_t len)
{
    if (!dst || !len || len > ARC_IO_MAX || id < ARC_ID_MIN || id > ARC_ID_MAX)
        return -EINVAL;
    if (!sys_nvs_init()) return -EIO;
    memset(dst, 0, len);
    return (int)nvs_read(&fs, id, dst, len);
}
int sys_arc_write(uint16_t id, const void *src, size_t len)
{
    if (!src || !len || len > ARC_IO_MAX || id < ARC_ID_MIN || id > ARC_ID_MAX)
        return -EINVAL;
    if (!sys_nvs_init()) return -EIO;
    int free = nvs_calc_free_space(&fs);
    if (free < 0) return free;
    /* Preserve one erase sector for other production NVS entries / GC. */
    if ((size_t)free < len + fs.sector_size + 128u) return -ENOSPC;
    int r = (int)nvs_write(&fs, id, src, len);
    if (r < 0) return r;
    /* NVS may return zero if content already matches; verify in caller. */
    return r;
}
void sys_arc_storage_status(void)
{
    if (!sys_nvs_init()) { printk("ARCHIVE NVS mount failed\n"); return; }
    int free = nvs_calc_free_space(&fs);
    int item = nvs_sector_max_data_size(&fs);
    printk("ARCHIVE NVS partition=%u bytes sectors=%u x %u free=%d max_item=%d\n",
        (unsigned)NVS_PARTITION_SIZE, fs.sector_count, fs.sector_size, free, item);
    printk("ARCHIVE namespace=100..131 (no data loaded during startup)\n");
}

// write to retained and nvs
void sys_write(uint16_t id, void *retained_ptr, const void *data, size_t len)
{
	if (!sys_nvs_init()) {
		LOG_ERR("sys_write: NVS init failed, cannot write ID %d", id);
		if (retained_ptr) {
			memcpy(retained_ptr, data, len);
			retained_update();
		}
		return;
	}
	if (retained_ptr) {
		memcpy(retained_ptr, data, len);
	}
	int err = nvs_write(&fs, id, data, len);
	if (err < 0) {
		LOG_ERR("Failed to write to NVS, error: %d", err);
		return;
	}
	if (retained_ptr) {
		retained_update();
	}
}

void sys_read(uint16_t id, void *data, size_t len)
{
	memset(data, 0, len);

	if (!sys_nvs_init()) {
		LOG_ERR("sys_read: NVS init failed, cannot read ID %d", id);
		return;
	}
	int err = nvs_read(&fs, id, data, len);
	if (err < 0) {
		if (err == -ENOENT) // suppress ENOENT
		{
			LOG_DBG("No entry exists for ID %d, read data set to zero", id);
		} else {
			LOG_ERR("Failed to read from NVS, error: %d", err);
			LOG_WRN("Read data set to zero");
		}
		return;
	}
	if ((size_t)err < len) {
		LOG_WRN("Short NVS read for ID %d: got %d bytes, expected %zu", id, err, len);
	}
}

void sys_clear(void)
{

	static bool reset_confirm = false;
	if (!reset_confirm) {
		printk(
			"Resetting NVS and retained will clear all pairing, sensor calibration data, and battery calibration data. "
			"Are you sure?\n"
		);
		reset_confirm = true;
		return;
	}
	printk("Resetting NVS and retained\n");

	sys_nvs_init();
	memset(retained, 0, sizeof(*retained));
	nvs_clear(&fs);
	nvs_init = false;
	reset_confirm = false;

	// Re-initialize fields that need non-zero default values
	retained->gyroSensScale[0] = 1.0f;
	retained->gyroSensScale[1] = 1.0f;
	retained->gyroSensScale[2] = 1.0f;
	retained->build_timestamp = BUILD_TIMESTAMP;
	retained_update();

	LOG_INF("NVS and retained reset");
}

void sys_nvs_stats(void)
{
	if (!sys_nvs_init()) {
		printk("NVS init failed\n");
		return;
	}

	printk("Storage partition: %u bytes\n", NVS_PARTITION_SIZE);
	printk(
		"Allocated NVS: %u * %u = %u bytes\n",
		fs.sector_size,
		fs.sector_count,
		fs.sector_size * fs.sector_count
	);
	printk(
		"NVS free: %d bytes, max item: %d bytes\n",
		nvs_calc_free_space(&fs),
		nvs_sector_max_data_size(&fs)
	);
}

// return 0 if clock applied, -1 if failed (because there is no clk_en or clk_out)
int set_sensor_clock(bool enable, float rate, float *actual_rate)
{
	*actual_rate = 0;
#if CLK_EN_EXISTS
	int ret = gpio_pin_set_dt(&clk_en, enable);
	if (ret) {
		LOG_ERR("CLK_EN GPIO set to %d failed (ret=%d)", enable, ret);
		return ret;
	}
	LOG_INF("CLK_EN GPIO set to %d", enable);
	if (enable) {
		k_msleep(2); // allow external oscillator to stabilize
		*actual_rate = 32768;
	}
	return 0;
#endif
	if (!device_is_ready(clk_out.dev)) {
		if (enable) {
			LOG_WRN("Clock output device not ready");
		}
		return -1;
	}
	int err = pwm_set_dt(&clk_out, PWM_HZ(rate), enable ? PWM_HZ(rate * 2) : 0);
	if (err) {
		LOG_ERR("PWM clock output set failed (err=%d)", err);
		return err;
	}
	if (enable) {
		*actual_rate = rate;
	}
	return 0;
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int64_t press_time = 0;
static int64_t last_press_duration = 0;

static void button_interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bool pressed = button_read();
	int64_t current_time = k_uptime_get();
	if (press_time && !pressed && current_time - press_time > 50) { // debounce
		last_press_duration = current_time - press_time;
	} else if (press_time && pressed) { // unusual press event on button already pressed
		return;
	}
	press_time = pressed ? current_time : 0;
}

static struct gpio_callback button_cb_data;

static int sys_button_init(void)
{
	gpio_pin_configure_dt(&button0, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&button_cb_data, button_interrupt_handler, BIT(button0.pin));
	gpio_add_callback(button0.port, &button_cb_data);
	return 0;
}

SYS_INIT(sys_button_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

bool button_read(void)
{
#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
	return gpio_pin_get_dt(&button0);
#else
	return false;
#endif
}

#if BUTTON_EXISTS // Alternate button if available to use as "reset key"
static void button_thread(void)
{
	int num_presses = 0;
	int64_t last_press = 0;

	/* Register button thread with watchdog */
	watchdog_register_thread(WDT_CHANNEL_BUTTON, 0);

	while (1) {
		if (press_time && k_uptime_get() - press_time > 50) // debounce
		{
			if (!get_status(SYS_STATUS_BUTTON_PRESSED)) {
				set_status(SYS_STATUS_BUTTON_PRESSED, true);
			}
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		}
		if (last_press_duration > 50) // debounce
		{
			if (!get_status(SYS_STATUS_BUTTON_PRESSED)) {
				set_status(SYS_STATUS_BUTTON_PRESSED, true);
			}
			num_presses++;
			LOG_INF("Button pressed %d times", num_presses);
			last_press_duration = 0;
			last_press = k_uptime_get();
			set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_HIGHEST);
		}
		if (last_press && k_uptime_get() - last_press > 1000) {
			LOG_INF("Button was pressed %d times", num_presses);
			last_press = 0;
            /* Release the pressed-button pattern BEFORE starting a report. */
            set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
            if (num_presses == 1) {
				sys_show_status();
			} else if (num_presses == 4) {
                if (sensor_earth_cal_active()) {
                    sensor_earth_cal_cancel();
                } else {
                    /* Explicit four-click action is allowed even in test mode.
                     * Use the immediate path so a pending power request cannot
                     * silently swallow the reboot. */
                    sys_request_system_reboot(true);
                }
			}
#if CONFIG_USER_EXTRA_ACTIONS // TODO: extra actions are default until server can send commands to trackers
			sys_reset_mode(num_presses - 1);
#endif
			num_presses = 0;
			set_status(SYS_STATUS_BUTTON_PRESSED, false);
		}
		if (press_time && k_uptime_get() - press_time > 1000 && button_read()) // Button is being held
		{
			if (sys_user_shutdown()) // held for 5 seconds, reset pairing
			{
				LOG_INF("Pairing requested");
				esb_reset_pair();
				press_time = 0;
				set_status(SYS_STATUS_BUTTON_PRESSED, false); // TODO: is needed?
			}
			else // shutting down or rebooting
			{
				k_thread_abort(button_thread_id);
			}
		}

		/* Feed watchdog at end of each loop iteration */
		watchdog_feed(WDT_CHANNEL_BUTTON);

		k_msleep(20);
	}
}
#endif

static int sys_gpio_init(void)
{
#if DOCK_EXISTS // configure if exists
	gpio_pin_configure_dt(&dock, GPIO_INPUT);
#endif
#if CHG_EXISTS
	gpio_pin_configure_dt(&chg, GPIO_INPUT);
#endif
#if STBY_EXISTS
	gpio_pin_configure_dt(&stby, GPIO_INPUT);
#endif
#if CLK_EN_EXISTS
	gpio_pin_configure_dt(&clk_en, GPIO_OUTPUT);
#endif
#if DCDC_EN_EXISTS
	gpio_pin_configure_dt(&dcdc_en, GPIO_OUTPUT);
#endif
#if LDO_EN_EXISTS
	gpio_pin_configure_dt(&ldo_en, GPIO_OUTPUT);
#endif
	return 0;
}

SYS_INIT(sys_gpio_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool dock_read(void)
{
#if DOCK_EXISTS
	return gpio_pin_get_dt(&dock);
#else
	return false;
#endif
}

bool chg_read(void)
{
#if CHG_EXISTS
	return gpio_pin_get_dt(&chg);
#else
	return false;
#endif
}

bool stby_read(void)
{
#if STBY_EXISTS
	return gpio_pin_get_dt(&stby);
#else
	return false;
#endif
}

int sys_user_shutdown(void)
{
	int64_t start_time = k_uptime_get();
#if USER_SHUTDOWN_ENABLED
	LOG_INF("User shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
#endif
	k_msleep(1500);
	if (button_read()) // If alternate button is available and still pressed, wait for the user to stop pressing the
					   // button
	{
		set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
		bool led_on = false;
		while (button_read()) {
			if (!led_on && k_uptime_get() - start_time > 500) // long pattern starts with led on, so delay pattern a bit
			{
				set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
				led_on = 1;
			}
			if (k_uptime_get() - start_time > 4000) // held for over 5 seconds, cancel shutdown
			{
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				return 1;
			}
			k_msleep(1);
		}
		set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
	}
#if USER_SHUTDOWN_ENABLED
	sys_request_system_off(false);
#else
	sys_request_system_reboot(false);
#endif
	return 0;
}

void sys_command_shutdown(void)
{
	LOG_INF("Command shutdown requested");
	reboot_counter_write(0);
	set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
	k_msleep(1500);
	sys_request_system_off(false);
}

static void sys_status_led_off(struct k_work *work)
{
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
}
static K_WORK_DELAYABLE_DEFINE(sys_status_led_work, sys_status_led_off);

/* One quick press reports device health with the LED:
 * 4 flashes = temperature table complete and Earth direction stored
 * 2 flashes = temperature table complete, no Earth calibration stored
 * 3 flashes = table exists but current temperature lacks nearby samples
 * 1 long flash = no temperature table */
void sys_show_status(void)
{
	int count = retained->tempCalState.count;
	bool complete = sensor_tcal_coverage_ready();
	bool earth = retained->earthCalValid;
	k_work_cancel_delayable(&sys_status_led_work);

	if (complete && earth) {
		set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_HIGHEST);
	} else if (complete) {
		set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
	} else if (count > 0) {
		set_led(SYS_LED_PATTERN_ONESHOT_POWERON, SYS_LED_PRIORITY_HIGHEST);
	} else {
		set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_HIGHEST);
		k_work_schedule(&sys_status_led_work, K_MSEC(700));
	}
	LOG_INF("Status report: tcal=%d points, earthCal=%d", count, (int)earth);
}

void sys_reset_mode(uint8_t mode)
{
	switch (mode) {
	case 1: // two quick presses: on-device Earth calibration
		LOG_INF("Earth calibration requested by button");
		sensor_earth_cal_start();
		break;
	case 2: // Reset mode pairing reset
		LOG_INF("Pairing reset requested");
		esb_reset_pair();
		break;
#if defined(CONFIG_BOARD_STYRIA_MINI_UF2)
	case 4: // five quick presses: on-device gyro scale calibration (auto-finish, 20 turns about X)
		LOG_INF("Gyro scale calibration requested by button (auto-finish, 20 turns, X)");
		sensor_gyro_scale_auto_start();
		break;
	case 5: // six quick presses: guided g-sensitivity pose calibration
		LOG_INF("G-sensitivity pose calibration requested by button");
		sensor_gsens_guide_start();
		break;
#endif
#if DFU_EXISTS // Using DFU bootloader
#if !defined(CONFIG_BOARD_STYRIA_MINI_UF2)
	case 3:
	case 4: // Reset mode DFU
#else
	case 8: // nine quick presses: DFU
#endif
		LOG_INF("DFU requested");
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_UF2_RESET;
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		break;
#if !defined(CONFIG_BOARD_STYRIA_MINI_UF2)
	case 7:
	case 8: // Reset mode DFU OTA
#else
	case 9: // ten quick presses: DFU OTA
#endif
		LOG_INF("DFU OTA requested");
#if ADAFRUIT_BOOTLOADER
		NRF_POWER->GPREGRET = ADAFRUIT_DFU_MAGIC_OTA_RESET;
		sys_request_system_reboot(false);
#endif
#if NRF5_BOOTLOADER
		gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		break;
#endif
	default:
		break;
	}
}
