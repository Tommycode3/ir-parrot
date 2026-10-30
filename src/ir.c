#include "ir.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ir, LOG_LEVEL_INF);

#define IR_PWM_NODE    DT_NODELABEL(pwm0)
#define IR_PWM_CHANNEL 0

static const struct device *const ir_pwm = DEVICE_DT_GET(IR_PWM_NODE);

/* Some transmitter modules light the LED with DAT low: invert the carrier. */
static pwm_flags_t ir_flags;

void ir_set_inverted(bool inverted)
{
	ir_flags = inverted ? PWM_POLARITY_INVERTED : PWM_POLARITY_NORMAL;
	pwm_set(ir_pwm, IR_PWM_CHANNEL, PWM_USEC(26), 0, ir_flags);
}

int ir_init(void)
{
	if (!device_is_ready(ir_pwm)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}
	pwm_set(ir_pwm, IR_PWM_CHANNEL, PWM_USEC(26), 0, ir_flags);
	return 0;
}

static void busy_wait_us(uint32_t us)
{
	k_busy_wait(us);
}

int ir_send_raw(uint32_t carrier_hz, const uint16_t *durations_us, size_t count)
{
	return ir_send_raw_ex(carrier_hz, durations_us, count, busy_wait_us);
}

int ir_send_raw_ex(uint32_t carrier_hz, const uint16_t *durations_us,
		   size_t count, void (*wait_us)(uint32_t us))
{
	if (carrier_hz < 20000U || carrier_hz > 60000U) {
		return -EINVAL;
	}
	if (!durations_us || count < 2U || (count & 1U)) {
		return -EINVAL;
	}

	uint32_t period_ns = 1000000000U / carrier_hz;
	uint32_t pulse_ns  = period_ns / 3U;  /* ~33% duty: better current ratio for IR LED */

	for (size_t i = 0; i < count; i++) {
		bool mark = ((i & 1U) == 0U);
		int err = pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns,
				  mark ? pulse_ns : 0U, ir_flags);
		if (err) {
			LOG_ERR("pwm_set failed at idx %u: %d", (unsigned)i, err);
			pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns, 0U, ir_flags);
			return err;
		}
		wait_us(durations_us[i]);
	}

	pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns, 0U, ir_flags);
	return 0;
}

void ir_pulse(uint32_t on_ms, uint32_t off_ms, uint32_t count)
{
	const uint32_t period_ns = PWM_USEC(26);
	const uint32_t pulse_ns  = period_ns / 3U;

	for (uint32_t i = 0; i < count; i++) {
		pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns, pulse_ns, ir_flags);
		k_msleep(on_ms);
		pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns, 0U, ir_flags);
		k_msleep(off_ms);
	}
}

void ir_level(bool high)
{
	const uint32_t period_ns = PWM_USEC(26);

	/* 100% duty = DAT steady high, 0% = steady low (no carrier). */
	pwm_set(ir_pwm, IR_PWM_CHANNEL, period_ns, high ? period_ns : 0U, ir_flags);
}
