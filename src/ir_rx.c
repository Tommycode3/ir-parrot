#include "ir_rx.h"

#include <errno.h>
#include <stdio.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ir_rx, LOG_LEVEL_INF);

/* IR receiver module (VS1838B / TSOP-style): GND, VCC, OUT. */
#define RX_PWR_PIN 17 /* P0.17 driven high = receiver VCC */
#define RX_OUT_PIN 20 /* P0.20 = demodulated output, active low */

/* Silence longer than this closes a frame (NEC repeats come ~40 ms later). */
#define IR_RX_GAP_US 20000U

#define CYC_PER_US (SystemCoreClock / 1000000U)

static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct gpio_callback rx_cb;

/*
 * Edge timestamps (cycle counter), double-buffered: the ISR fills the
 * active buffer while ir_rx_poll() processes the other one.
 */
#define MAX_EDGES 4096

/* Raw (non-demodulating) sensors toggle at the carrier: intervals this short mean "carrier". */
#define CARRIER_MAX_US 150U
/* In raw mode, a pause longer than this ends a mark. */
#define MARK_GAP_US    300U

/* Hard cap on one capture, so a stuck-low input can't hang the CPU. */
#define CAPTURE_MAX_US 300000U

static uint32_t          edges[2][MAX_EDGES];
static uint8_t           active;
static volatile size_t   ready_count;
static volatile bool     ready;
static volatile uint32_t total_edges;

/* Raw edge intervals of the last frame, for DUMP. */
static uint16_t last_iv[MAX_EDGES];
static size_t   last_iv_len;

static inline uint32_t rx_level(void)
{
	return (NRF_P0->IN >> RX_OUT_PIN) & 1U;
}

static volatile uint32_t trigger_cyc;
static K_SEM_DEFINE(trigger_sem, 0, 1);

/* First falling edge of a frame: timestamp it and hand over to the capture thread. */
static void rx_edge(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	trigger_cyc = DWT->CYCCNT;
	gpio_pin_interrupt_configure(dev, RX_OUT_PIN, GPIO_INT_DISABLE);
	k_sem_give(&trigger_sem);
}

/*
 * Polls the pin in a tight loop (sub-microsecond resolution) until the
 * line has been quiet for IR_RX_GAP_US. Runs as a cooperative thread with
 * interrupts enabled, so USB keeps being serviced; an ISR only adds a few
 * microseconds of jitter. One interrupt per edge can't keep up with
 * sensors that output the carrier itself.
 */
static void capture_thread(void)
{
	const uint32_t gap = IR_RX_GAP_US * CYC_PER_US;
	const uint32_t cap = CAPTURE_MAX_US * CYC_PER_US;

	for (;;) {
		k_sem_take(&trigger_sem, K_FOREVER);

		uint32_t *e = edges[active];
		size_t m = 0;
		uint32_t t0 = trigger_cyc;
		uint32_t last = DWT->CYCCNT;
		uint32_t level = rx_level();

		/* The triggering edge went low; if we're already high again, that mark ended too. */
		e[m++] = t0;
		if (level != 0U) {
			e[m++] = last;
		}

		for (;;) {
			uint32_t now = DWT->CYCCNT;
			uint32_t l = rx_level();

			if (l != level) {
				level = l;
				if (m < MAX_EDGES) {
					e[m++] = now;
				}
				last = now;
			} else if (now - last > gap || now - t0 > cap) {
				break;
			}
		}

		total_edges += m;
		if (m >= 2 && !ready) {
			ready_count = m;
			ready = true;
		}

		/*
		 * This is the highest-priority thread and it busy-polls: give the
		 * rest of the system (console, USB) time to run before re-arming,
		 * otherwise a noisy line would starve everything else.
		 */
		k_msleep(10);
		gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_EDGE_FALLING);
	}
}

K_THREAD_DEFINE(ir_capture_tid, 1024, capture_thread, NULL, NULL, NULL,
		K_PRIO_COOP(0), 0, 0);

int ir_rx_init(void)
{
	int err;

	if (!device_is_ready(gpio0)) {
		return -ENODEV;
	}

	/* Cycle counter at core clock (64 MHz) for microsecond timestamps. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	err = gpio_pin_configure(gpio0, RX_PWR_PIN,
				 GPIO_OUTPUT_HIGH | NRF_GPIO_DRIVE_H0H1);
	if (err) {
		return err;
	}

	err = gpio_pin_configure(gpio0, RX_OUT_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		return err;
	}

	gpio_init_callback(&rx_cb, rx_edge, BIT(RX_OUT_PIN));
	err = gpio_add_callback(gpio0, &rx_cb);
	if (err) {
		return err;
	}

	return gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_EDGE_FALLING);
}

static uint16_t clamp_us(uint32_t cyc)
{
	uint32_t us = cyc / CYC_PER_US;
	return us > UINT16_MAX ? UINT16_MAX : (uint16_t)us;
}

static size_t process_edges(const uint32_t *t, size_t m, uint16_t *out,
			    size_t max, bool *raw);

size_t ir_rx_poll(uint16_t *out, size_t max, bool *raw)
{
	const uint32_t *t;
	size_t m = 0;
	unsigned int key = irq_lock();

	if (ready) {
		t = edges[active];
		m = ready_count;
		active ^= 1U;
		ready = false;
	}

	irq_unlock(key);

	if (m < 2) {
		return 0;
	}
	return process_edges(t, m, out, max, raw);
}

static size_t process_edges(const uint32_t *t, size_t m, uint16_t *out,
			    size_t max, bool *raw)
{
	size_t n = 0;

	uint32_t min_cyc = UINT32_MAX;
	for (size_t i = 1; i < m; i++) {
		min_cyc = MIN(min_cyc, t[i] - t[i - 1]);
		last_iv[i - 1] = clamp_us(t[i] - t[i - 1]);
	}
	last_iv_len = m - 1;
	*raw = min_cyc / CYC_PER_US < CARRIER_MAX_US;

	if (*raw) {
		/* Software demodulation: merge carrier pulses into marks. */
		uint32_t burst_start = t[0];

		for (size_t i = 1; i < m && n + 2 <= max; i++) {
			uint32_t gap = t[i] - t[i - 1];
			if (gap / CYC_PER_US > MARK_GAP_US) {
				out[n++] = clamp_us(t[i - 1] - burst_start);
				out[n++] = clamp_us(gap);
				burst_start = t[i];
			}
		}
		if (n < max) {
			out[n++] = clamp_us(t[m - 1] - burst_start);
		}
	} else {
		/* Demodulated receiver: every edge-to-edge interval is a mark or a space. */
		for (size_t i = 1; i < m && n < max; i++) {
			out[n++] = clamp_us(t[i] - t[i - 1]);
		}
	}

	/* Frame ends on a mark: close it with the gap so the TX side gets mark/space pairs. */
	if ((n & 1U) && n < max) {
		out[n++] = IR_RX_GAP_US;
	} else if (n & 1U) {
		n--;
	}
	return n;
}

static bool near(uint32_t v, uint32_t target)
{
	return v > target * 3U / 4U && v < target * 5U / 4U;
}

/* Pulse-distance bits (NEC/Samsung): fixed mark, short/long space. LSB first. */
static int decode_pd(const uint16_t *d, size_t n, size_t start, uint32_t mark,
		     uint32_t one_space, uint32_t zero_space, uint32_t *val)
{
	int bits = 0;

	*val = 0;
	for (size_t i = start; i + 1 < n && bits < 32; i += 2) {
		if (!near(d[i], mark)) {
			break;
		}
		if (near(d[i + 1], one_space)) {
			*val |= 1UL << bits;
		} else if (!near(d[i + 1], zero_space)) {
			break;
		}
		bits++;
	}
	return bits;
}

/* Sony SIRC: pulse-width bits, mark 1200 = 1, 600 = 0, space 600. LSB first. */
static int decode_sony(const uint16_t *d, size_t n, uint32_t *val)
{
	int bits = 0;

	*val = 0;
	for (size_t i = 2; i < n && bits < 20; i += 2) {
		if (near(d[i], 1200)) {
			*val |= 1UL << bits;
		} else if (!near(d[i], 600)) {
			break;
		}
		bits++;
	}
	return bits;
}

void ir_rx_describe(const uint16_t *d, size_t n, char *out, size_t len)
{
	uint32_t v;
	int bits;

	if (n >= 4 && near(d[0], 9000) && near(d[1], 2250)) {
		snprintf(out, len, "NEC repeat");
		return;
	}
	if (n >= 66 && near(d[0], 9000) && near(d[1], 4500)) {
		bits = decode_pd(d, n, 2, 560, 1690, 560, &v);
		if (bits == 32) {
			uint8_t a = v, na = v >> 8, c = v >> 16, nc = v >> 24;
			snprintf(out, len, "NEC addr=0x%02X%s cmd=0x%02X%s (0x%08X)",
				 a, (uint8_t)~a == na ? "" : "(ext)",
				 c, (uint8_t)~c == nc ? "" : "(!chk)", v);
			return;
		}
	}
	if (n >= 66 && near(d[0], 4500) && near(d[1], 4500)) {
		bits = decode_pd(d, n, 2, 560, 1690, 560, &v);
		if (bits == 32) {
			snprintf(out, len, "Samsung addr=0x%04X cmd=0x%02X (0x%08X)",
				 v & 0xFFFF, (v >> 16) & 0xFF, v);
			return;
		}
	}
	if (n >= 26 && near(d[0], 2400) && near(d[1], 600)) {
		bits = decode_sony(d, n, &v);
		if (bits == 12 || bits == 15 || bits == 20) {
			snprintf(out, len, "Sony%d cmd=0x%02X dev=0x%X (0x%X)",
				 bits, v & 0x7F, v >> 7, v);
			return;
		}
	}
	snprintf(out, len, "unknown protocol, leader %u/%u us, %u durations",
		 n > 0 ? d[0] : 0, n > 1 ? d[1] : 0, (unsigned)n);
}

void ir_rx_stat(int *pwr, int *out, uint32_t *edges)
{
	*pwr = gpio_pin_get(gpio0, RX_PWR_PIN);
	*out = gpio_pin_get(gpio0, RX_OUT_PIN);
	*edges = total_edges;
}

size_t ir_rx_last_raw(const uint16_t **iv)
{
	*iv = last_iv;
	return last_iv_len;
}

/* Loopback: the transmitter samples our own receiver while it sends. */
static uint32_t *lb_edges;
static size_t    lb_count;
static uint32_t  lb_level;

void ir_rx_loopback_begin(void)
{
	gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_DISABLE);
	lb_edges = edges[active];
	lb_count = 0;
	lb_level = rx_level();
}

void ir_rx_loopback_sample(void)
{
	uint32_t l = rx_level();

	if (l != lb_level) {
		lb_level = l;
		if (lb_count < MAX_EDGES) {
			lb_edges[lb_count++] = DWT->CYCCNT;
		}
	}
}

void ir_rx_loopback_wait_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t span = us * CYC_PER_US;

	while (DWT->CYCCNT - start < span) {
		ir_rx_loopback_sample();
	}
}

size_t ir_rx_loopback_end(uint16_t *out, size_t max, bool *raw)
{
	/* Receivers lag the light by ~100-200 us: keep listening for the tail. */
	ir_rx_loopback_wait_us(IR_RX_GAP_US);
	gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_EDGE_FALLING);
	total_edges += lb_count;
	return lb_count >= 2 ? process_edges(lb_edges, lb_count, out, max, raw) : 0;
}

int ir_rx_probe(void)
{
	/*
	 * With the internal pull-up swapped for a pull-down, the pin only reads
	 * high if a powered receiver is actively holding its output high.
	 */
	int level;

	gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_DISABLE);
	gpio_pin_configure(gpio0, RX_OUT_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	k_busy_wait(1000);
	level = gpio_pin_get(gpio0, RX_OUT_PIN);
	gpio_pin_configure(gpio0, RX_OUT_PIN, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure(gpio0, RX_OUT_PIN, GPIO_INT_EDGE_FALLING);
	return level;
}
