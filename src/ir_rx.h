#ifndef IR_RX_H_
#define IR_RX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IR_RX_MAX_PULSES 512

int ir_rx_init(void);

/*
 * If a complete frame has been captured (no edges for IR_RX_GAP_US),
 * copy it into out[] and return the number of durations (always even:
 * mark, space, mark, space, ..., the trailing space is the gap).
 * Returns 0 if no frame is ready. *raw is set when the sensor output the
 * carrier itself (no demodulator) and the frame was demodulated in software.
 */
size_t ir_rx_poll(uint16_t *out, size_t max, bool *raw);

/* Print a decoded summary (NEC / Samsung / Sony / raw) of a frame. */
void ir_rx_describe(const uint16_t *d, size_t n, char *buf, size_t buflen);

/* Diagnostics: power pin state, OUT level (idle = 1), edges seen since boot. */
void ir_rx_stat(int *pwr, int *out, uint32_t *edges);

/* Raw edge-to-edge intervals (us) of the last frame. */
size_t ir_rx_last_raw(const uint16_t **iv);

/*
 * Loopback test: begin() stops the normal capture, wait_us() is passed to
 * ir_send_raw_ex() and records receiver edges while transmitting, end()
 * returns the received frame like ir_rx_poll().
 */
void ir_rx_loopback_begin(void);
void ir_rx_loopback_wait_us(uint32_t us);
void ir_rx_loopback_sample(void);
size_t ir_rx_loopback_end(uint16_t *out, size_t max, bool *raw);

/* 1 if a powered receiver drives OUT high (pin read with pull-down). */
int ir_rx_probe(void);

#endif /* IR_RX_H_ */
