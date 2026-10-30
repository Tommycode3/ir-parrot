#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>
#include <zephyr/usb/usb_device.h>
#if defined(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#endif

#include "ir.h"
#include "ir_rx.h"

void crashlog_boot(void);
void crashlog_report(void);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LINE_BUF_SIZE 4096
#define MAX_PULSES    1024

/* Shortest frame worth reporting (leader + a few bits). */
#define MIN_FRAME_DURATIONS 8

/*
 * nice!nano switched VCC pin (feeds the IR transmitter module): P0.13
 * drives the switch. Active low on v2, active high on v1; EXTVCC overrides.
 */
#define EXT_VCC_PIN     13
#define EXT_VCC_ON_LEVEL 0

/* Samsung32 timings (us) and the TV's address byte. */
#define SAMSUNG_ADDR    0x07
#define SAMSUNG_HDR     4500
#define SAMSUNG_BIT     560
#define SAMSUNG_ONE     1690
#define SAMSUNG_ZERO    560
#define SAMSUNG_GAP     46000

/* Adafruit nRF52 bootloader: GPREGRET magic that reboots into UF2 mode. */
#define DFU_MAGIC_UF2_RESET 0x57

typedef void (*reply_fn)(const char *msg);

static uint16_t pulse_buf[MAX_PULSES];

/* Last captured frame, replayed by REPLAY. */
static uint16_t last_rx[IR_RX_MAX_PULSES];
static size_t   last_rx_len;

/* Commands arrive from both USB console and BLE NUS threads. */
static K_MUTEX_DEFINE(cmd_lock);

#if defined(CONFIG_BT)
static char     line_buf[LINE_BUF_SIZE];
static size_t   line_len;

static struct bt_conn *current_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void nus_reply(const char *msg)
{
	if (!current_conn) {
		return;
	}
	int err = bt_nus_send(current_conn, (const uint8_t *)msg, strlen(msg));
	if (err) {
		LOG_WRN("nus send failed: %d", err);
	}
}

#endif /* CONFIG_BT */

static void usb_reply(const char *msg)
{
	printk("%s", msg);
}

static void send_and_reply(uint32_t carrier_hz, const uint16_t *d, size_t n,
			   reply_fn reply)
{
	int err = ir_send_raw(carrier_hz, d, n);
	if (err) {
		char buf[32];
		snprintf(buf, sizeof(buf), "ERR send %d\n", err);
		reply(buf);
	} else {
		reply("OK\n");
	}
}

/* Samsung32: addr, addr, cmd, ~cmd, LSB first, then a stop bit. Returns durations. */
static size_t samsung_frame(uint8_t cmd, uint16_t *d)
{
	uint32_t v = SAMSUNG_ADDR | (SAMSUNG_ADDR << 8) | ((uint32_t)cmd << 16) |
		     ((uint32_t)(uint8_t)~cmd << 24);
	size_t n = 0;

	d[n++] = SAMSUNG_HDR;
	d[n++] = SAMSUNG_HDR;
	for (int i = 0; i < 32; i++) {
		d[n++] = SAMSUNG_BIT;
		d[n++] = (v & BIT(i)) ? SAMSUNG_ONE : SAMSUNG_ZERO;
	}
	d[n++] = SAMSUNG_BIT;
	d[n++] = SAMSUNG_GAP;
	return n;
}

static void print_frame(const uint16_t *d, size_t n, bool raw);

/*
 * Connect the input buffer (with pull-up, so idle pins stay high) on every
 * pin not used by the firmware, so pin scans can see a stray wire.
 * Returns the mask of P0 pins that were left alone.
 */
static uint32_t scan_inputs_enable(void)
{
	/* Skip: P0.13 ext VCC, P0.15 LED, P0.17 RX power, P0.18 reset, P0.20 RX, P0.31 IR TX. */
	const uint32_t skip0 = BIT(13) | BIT(15) | BIT(17) | BIT(18) | BIT(20) | BIT(31);

	for (int p = 0; p < 48; p++) {
		if (p < 32 && (skip0 & BIT(p))) {
			continue;
		}
		volatile uint32_t *cnf = p < 32 ? &NRF_P0->PIN_CNF[p] : &NRF_P1->PIN_CNF[p - 32];

		*cnf = GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos |
		       GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos |
		       GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
	}
	k_msleep(5);
	return skip0 & ~BIT(20);
}

/* IR LED pin, for the bit-banged carrier diagnostic (PWM must be idle). */
#define IR_TX_PIN 31

/* Bit-banged 38 kHz carrier at 1/3 duty, sampling our receiver meanwhile. */
static void bitbang_send(const uint16_t *d, size_t n)
{
	const uint32_t cpu = SystemCoreClock / 1000000U;
	const uint32_t period = SystemCoreClock / 38000U;
	const uint32_t high = period / 3U;

	for (size_t i = 0; i < n; i++) {
		uint32_t start = DWT->CYCCNT;
		uint32_t span = d[i] * cpu;
		bool mark = (i & 1U) == 0U;

		while (DWT->CYCCNT - start < span) {
			if (mark && (DWT->CYCCNT - start) % period < high) {
				NRF_P0->OUTSET = BIT(IR_TX_PIN);
			} else {
				NRF_P0->OUTCLR = BIT(IR_TX_PIN);
			}
			ir_rx_loopback_sample();
		}
	}
	NRF_P0->OUTCLR = BIT(IR_TX_PIN);
}

static void process_line(char *line, reply_fn reply)
{
	char *saveptr = NULL;
	char *cmd = strtok_r(line, " ", &saveptr);
	if (!cmd) {
		reply("ERR empty\n");
		return;
	}

	if (strcmp(cmd, "TX") == 0) {
		char *khz_str = strtok_r(NULL, " ", &saveptr);
		if (!khz_str) {
			reply("ERR no_khz\n");
			return;
		}
		int khz = atoi(khz_str);
		if (khz < 20 || khz > 60) {
			reply("ERR bad_khz\n");
			return;
		}

		size_t n = 0;
		char *tok;
		while ((tok = strtok_r(NULL, " ", &saveptr)) != NULL) {
			if (n >= MAX_PULSES) {
				reply("ERR too_many\n");
				return;
			}
			int v = atoi(tok);
			if (v <= 0 || v > 65535) {
				reply("ERR bad_pulse\n");
				return;
			}
			pulse_buf[n++] = (uint16_t)v;
		}
		if (n < 2) {
			reply("ERR no_pulses\n");
			return;
		}
		if (n & 1U) {
			reply("ERR odd_pulses\n");
			return;
		}

		send_and_reply((uint32_t)khz * 1000U, pulse_buf, n, reply);
	} else if (strcmp(cmd, "REPLAY") == 0) {
		char *khz_str = strtok_r(NULL, " ", &saveptr);
		int khz = khz_str ? atoi(khz_str) : 38;

		if (last_rx_len < 2) {
			reply("ERR nothing_captured\n");
			return;
		}
		send_and_reply((uint32_t)khz * 1000U, last_rx, last_rx_len, reply);
	} else if (strcmp(cmd, "SAMSUNG") == 0) {
		char *arg = strtok_r(NULL, " ", &saveptr);
		char *rep = strtok_r(NULL, " ", &saveptr);
		int times = rep ? atoi(rep) : 1;

		if (!arg) {
			reply("ERR no_cmd\n");
			return;
		}
		size_t n = samsung_frame((uint8_t)strtoul(arg, NULL, 0), pulse_buf);
		for (int i = 0; i < times; i++) {
			int err = ir_send_raw(38000U, pulse_buf, n);
			if (err) {
				char buf[32];
				snprintf(buf, sizeof(buf), "ERR send %d\n", err);
				reply(buf);
				return;
			}
		}
		reply("OK\n");
	} else if (strcmp(cmd, "LOOP") == 0) {
		/* Send a Samsung frame and decode what our own receiver saw. */
		static uint16_t rx[IR_RX_MAX_PULSES];
		char *arg = strtok_r(NULL, " ", &saveptr);
		bool raw;

		if (!arg) {
			reply("ERR no_cmd\n");
			return;
		}
		size_t n = samsung_frame((uint8_t)strtoul(arg, NULL, 0), pulse_buf);

		ir_rx_loopback_begin();
		int err = ir_send_raw_ex(38000U, pulse_buf, n, ir_rx_loopback_wait_us);
		size_t got = ir_rx_loopback_end(rx, ARRAY_SIZE(rx), &raw);

		if (err) {
			reply("ERR send\n");
		} else if (got == 0) {
			reply("LOOP nothing received\n");
		} else {
			print_frame(rx, got, raw);
		}
	} else if (strcmp(cmd, "LOOPBB") == 0) {
		/* Like LOOP but with a CPU bit-banged carrier: isolates the PWM. */
		static uint16_t rx[IR_RX_MAX_PULSES];
		char *arg = strtok_r(NULL, " ", &saveptr);
		bool raw;

		if (!arg) {
			reply("ERR no_cmd\n");
			return;
		}
		size_t n = samsung_frame((uint8_t)strtoul(arg, NULL, 0), pulse_buf);

		NRF_P0->PIN_CNF[IR_TX_PIN] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos |
					     GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos |
					     GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;
		/* An enabled PWM owns its PSEL pin and would override the GPIO. */
		uint32_t pwm_en = NRF_PWM0->ENABLE;

		NRF_PWM0->ENABLE = 0;
		ir_rx_loopback_begin();
		unsigned int key = irq_lock();
		bitbang_send(pulse_buf, n);
		irq_unlock(key);
		NRF_PWM0->ENABLE = pwm_en;
		size_t got = ir_rx_loopback_end(rx, ARRAY_SIZE(rx), &raw);

		if (got == 0) {
			reply("LOOPBB nothing received\n");
		} else {
			print_frame(rx, got, raw);
		}
	} else if (strcmp(cmd, "TXSCAN") == 0) {
		/* Transmit a Samsung frame while counting transitions on every pin. */
		static uint32_t changes[48];
		static uint32_t prev0, prev1;
		char *arg = strtok_r(NULL, " ", &saveptr);
		size_t n = samsung_frame(arg ? (uint8_t)strtoul(arg, NULL, 0) : 0x07, pulse_buf);
		const uint32_t cpu = SystemCoreClock / 1000000U;

		memset(changes, 0, sizeof(changes));
		scan_inputs_enable();
		gpio_pin_interrupt_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 20, GPIO_INT_DISABLE);
		prev0 = NRF_P0->IN;
		prev1 = NRF_P1->IN;

		uint32_t period_ns = 1000000000U / 38000U;
		const struct device *pwm = DEVICE_DT_GET(DT_NODELABEL(pwm0));

		for (size_t i = 0; i <= n; i++) {
			/* One extra round after the frame: listen for the receiver's tail. */
			uint32_t dur = i < n ? pulse_buf[i] : 20000U;
			bool mark = i < n && (i & 1U) == 0U;

			pwm_set(pwm, 0, period_ns, mark ? period_ns / 3U : 0U, 0);
			uint32_t start = DWT->CYCCNT;

			while (DWT->CYCCNT - start < dur * cpu) {
				uint32_t in0 = NRF_P0->IN, in1 = NRF_P1->IN;
				uint32_t d0 = (in0 ^ prev0) & ~BIT(31), d1 = in1 ^ prev1;

				while (d0) {
					changes[__builtin_ctz(d0)]++;
					d0 &= d0 - 1;
				}
				while (d1) {
					changes[32 + __builtin_ctz(d1)]++;
					d1 &= d1 - 1;
				}
				prev0 = in0;
				prev1 = in1;
			}
		}
		gpio_pin_interrupt_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 20, GPIO_INT_EDGE_FALLING);

		printk("TXSCAN P0.IN=0x%08X P1.IN=0x%08X changes:", prev0, prev1);
		for (int p = 0; p < 48; p++) {
			if (changes[p]) {
				printk(" P%d.%02d=%u", p / 32, p % 32, changes[p]);
			}
		}
		printk(" (end)\n");
	} else if (strcmp(cmd, "SCAN") == 0) {
		/*
		 * Find which pin a signal actually arrives on: connect the input
		 * buffer (with pull-up, so idle pins stay high) on every free pin,
		 * poll both ports and count transitions per pin.
		 */
		char *arg = strtok_r(NULL, " ", &saveptr);
		uint32_t secs = arg ? strtoul(arg, NULL, 0) : 15;
		static uint32_t changes[48];
		const uint32_t skip0 = scan_inputs_enable();

		memset(changes, 0, sizeof(changes));

		printk("SCAN %us: press the remote now\n", secs);
		uint32_t prev0 = NRF_P0->IN, prev1 = NRF_P1->IN;
		int64_t end = k_uptime_get() + secs * 1000;

		while (k_uptime_get() < end) {
			uint32_t in0 = NRF_P0->IN, in1 = NRF_P1->IN;
			uint32_t d0 = (in0 ^ prev0) & ~skip0, d1 = in1 ^ prev1;

			while (d0) {
				int b = __builtin_ctz(d0);
				changes[b]++;
				d0 &= d0 - 1;
			}
			while (d1) {
				int b = __builtin_ctz(d1);
				changes[32 + b]++;
				d1 &= d1 - 1;
			}
			prev0 = in0;
			prev1 = in1;
		}

		printk("SCAN result:");
		for (int p = 0; p < 48; p++) {
			if (changes[p]) {
				printk(" P%d.%02d=%u", p / 32, p % 32, changes[p]);
			}
		}
		printk(" (end)\n");
	} else if (strcmp(cmd, "PROBE") == 0) {
		char buf[48];

		snprintf(buf, sizeof(buf), "PROBE out_with_pulldown=%d\n", ir_rx_probe());
		reply(buf);
	} else if (strcmp(cmd, "REGS") == 0) {
		char buf[160];

		snprintf(buf, sizeof(buf),
			 "REGS PWM0.ENABLE=%u PWM0.PSEL0=0x%08X PIN_CNF31=0x%08X OUT31=%u P0.13=%u\n",
			 NRF_PWM0->ENABLE, NRF_PWM0->PSEL.OUT[0], NRF_P0->PIN_CNF[IR_TX_PIN],
			 (NRF_P0->OUT >> IR_TX_PIN) & 1U, (NRF_P0->OUT >> 13) & 1U);
		reply(buf);
	} else if (strcmp(cmd, "PINTEST") == 0) {
		/*
		 * Drive P0.31 as plain GPIO (PWM released) and read back the real
		 * pin level through the input buffer: catches shorts, and blinks
		 * the module's LED without relying on the PWM driver.
		 */
		char *arg = strtok_r(NULL, " ", &saveptr);
		int times = arg ? atoi(arg) : 5;
		uint32_t pwm_en = NRF_PWM0->ENABLE;
		char buf[64];

		NRF_PWM0->ENABLE = 0;
		NRF_P0->PIN_CNF[IR_TX_PIN] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos |
					     GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos |
					     GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;
		for (int i = 0; i < times; i++) {
			NRF_P0->OUTSET = BIT(IR_TX_PIN);
			k_msleep(1000);
			int hi = (NRF_P0->IN >> IR_TX_PIN) & 1U;
			NRF_P0->OUTCLR = BIT(IR_TX_PIN);
			k_msleep(1000);
			int lo = (NRF_P0->IN >> IR_TX_PIN) & 1U;

			snprintf(buf, sizeof(buf), "PINTEST %d: wrote 1 read %d, wrote 0 read %d\n",
				 i + 1, hi, lo);
			reply(buf);
		}
		NRF_PWM0->ENABLE = pwm_en;
	} else if (strcmp(cmd, "SLOWBLINK") == 0) {
		char *arg = strtok_r(NULL, " ", &saveptr);
		int times = arg ? atoi(arg) : 10;

		for (int i = 0; i < times; i++) {
			ir_level(true);
			k_msleep(1000);
			ir_level(false);
			k_msleep(1000);
		}
		reply("OK\n");
	} else if (strcmp(cmd, "BLINK") == 0) {
		/* Visible through a phone camera: 5 x 0.5 s bursts of carrier. */
		ir_pulse(500, 500, 5);
		reply("OK\n");
	} else if (strcmp(cmd, "PWR29") == 0) {
		/* Power the transmitter module from P0.29 (high drive) instead of VCC. */
		char *arg = strtok_r(NULL, " ", &saveptr);
		int on = arg ? atoi(arg) : 1;

		gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 29,
				   (on ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW) | NRF_GPIO_DRIVE_H0H1);
		reply("OK\n");
	} else if (strcmp(cmd, "INV") == 0) {
		char *arg = strtok_r(NULL, " ", &saveptr);

		ir_set_inverted(arg && atoi(arg));
		reply("OK\n");
	} else if (strcmp(cmd, "EXTVCC") == 0) {
		char *arg = strtok_r(NULL, " ", &saveptr);
		int on = arg ? atoi(arg) : 1;

		gpio_pin_set_raw(DEVICE_DT_GET(DT_NODELABEL(gpio0)), EXT_VCC_PIN,
				 on ? EXT_VCC_ON_LEVEL : !EXT_VCC_ON_LEVEL);
		reply("OK\n");
	} else if (strcmp(cmd, "STAT") == 0) {
		int pwr, out;
		uint32_t edges;
		char buf[64];

		ir_rx_stat(&pwr, &out, &edges);
		snprintf(buf, sizeof(buf), "STAT pwr=%d out=%d edges=%u\n", pwr, out, edges);
		reply(buf);
	} else if (strcmp(cmd, "CRASH") == 0) {
		crashlog_report();
	} else if (strcmp(cmd, "DUMP") == 0) {
		const uint16_t *iv;
		size_t n = ir_rx_last_raw(&iv);

		printk("DUMP %u", (unsigned)n);
		for (size_t i = 0; i < n; i++) {
			printk(" %u", iv[i]);
		}
		printk("\n");
	} else if (strcmp(cmd, "DFU") == 0) {
		reply("OK rebooting to UF2 bootloader\n");
		k_msleep(100);
		NRF_POWER->GPREGRET = DFU_MAGIC_UF2_RESET;
		NVIC_SystemReset();
	} else {
		reply("ERR unknown\n");
	}
}

#if defined(CONFIG_BT)
static void nus_received(struct bt_conn *conn, const uint8_t *const data,
			 uint16_t len)
{
	ARG_UNUSED(conn);

	for (uint16_t i = 0; i < len; i++) {
		char c = (char)data[i];
		if (c == '\n' || c == '\r') {
			if (line_len > 0) {
				line_buf[line_len] = '\0';
				k_mutex_lock(&cmd_lock, K_FOREVER);
				process_line(line_buf, nus_reply);
				k_mutex_unlock(&cmd_lock);
				line_len = 0;
			}
		} else if (line_len < LINE_BUF_SIZE - 1) {
			line_buf[line_len++] = c;
		} else {
			line_len = 0;
			nus_reply("ERR overflow\n");
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = nus_received,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	current_conn = bt_conn_ref(conn);
	LOG_INF("Connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("Disconnected (reason %u)", reason);
	line_len = 0;
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

#endif /* CONFIG_BT */

/*
 * USB console input, polled from the main loop. All console output also
 * happens on the main thread: the CDC ACM TX ring buffer is not safe with
 * several concurrent producers (that race crashed the board).
 */
static const struct device *const console_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char   console_buf[LINE_BUF_SIZE];
static size_t console_len;

static void poll_console(void)
{
	unsigned char c;

	while (uart_poll_in(console_uart, &c) == 0) {
		if (c == '\r' || c == '\n') {
			if (console_len > 0) {
				console_buf[console_len] = '\0';
				k_mutex_lock(&cmd_lock, K_FOREVER);
				process_line(console_buf, usb_reply);
				k_mutex_unlock(&cmd_lock);
				console_len = 0;
			}
		} else if (console_len < LINE_BUF_SIZE - 1) {
			console_buf[console_len++] = (char)c;
		} else {
			console_len = 0;
			usb_reply("ERR overflow\n");
		}
	}
}

static void print_frame(const uint16_t *d, size_t n, bool raw)
{
	char desc[96];

	ir_rx_describe(d, n, desc, sizeof(desc));
	printk("\n[IR] %s%s\n", desc, raw ? " [sw-demod]" : "");
	printk("TX 38");
	for (size_t i = 0; i < n; i++) {
		printk(" %u", d[i]);
	}
	printk("\n");
}

int main(void)
{
	int err;

	crashlog_boot();

	/* Blue LED on P0.15 = firmware is running. */
	gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), 15, GPIO_OUTPUT_HIGH);

	/* Switched VCC on, for the IR transmitter module. */
	gpio_pin_configure(DEVICE_DT_GET(DT_NODELABEL(gpio0)), EXT_VCC_PIN,
			   EXT_VCC_ON_LEVEL ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW);

	err = usb_enable(NULL);
	if (err) {
		LOG_ERR("USB enable failed: %d", err);
	}

	err = ir_init();
	if (err) {
		LOG_ERR("IR init failed: %d", err);
		return 0;
	}

	err = ir_rx_init();
	if (err) {
		LOG_ERR("IR RX init failed: %d", err);
		return 0;
	}

#if defined(CONFIG_BT)
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BT enable failed: %d", err);
	} else if ((err = bt_nus_init(&nus_cb)) != 0) {
		LOG_ERR("NUS init failed: %d", err);
	} else if ((err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
					  sd, ARRAY_SIZE(sd))) != 0) {
		LOG_ERR("Advertising start failed: %d", err);
	}
#endif

	printk("IR-Tx ready: point the remote at the receiver. Commands: REPLAY [khz] | TX khz d0 d1 ... | SAMSUNG cmd [n] | LOOP cmd | LOOPBB cmd | REGS | PROBE | SCAN [s] | TXSCAN [cmd] | BLINK | SLOWBLINK [n]  PWR29 0/1 | INV 0/1 | EXTVCC 0/1 | STAT | DUMP | CRASH | DFU\n");

	static uint16_t frame[IR_RX_MAX_PULSES];

	for (;;) {
		bool raw;
		size_t n = ir_rx_poll(frame, ARRAY_SIZE(frame), &raw);
		/* A real frame has a leader plus data; shorter bursts are glitches. */
		if (n >= MIN_FRAME_DURATIONS) {
			k_mutex_lock(&cmd_lock, K_FOREVER);
			memcpy(last_rx, frame, n * sizeof(frame[0]));
			last_rx_len = n;
			k_mutex_unlock(&cmd_lock);
			print_frame(frame, n, raw);
		}
		poll_console();
		k_msleep(5);
	}

	return 0;
}
