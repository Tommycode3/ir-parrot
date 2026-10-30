#ifndef IR_H_
#define IR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int ir_init(void);

/* Invert the carrier for modules whose LED is on while DAT is low. */
void ir_set_inverted(bool inverted);

/*
 * Send a raw IR sequence.
 *
 * durations_us[0,2,4,...] are mark (carrier on) times in microseconds.
 * durations_us[1,3,5,...] are space (carrier off) times in microseconds.
 * count must be even and >= 2.
 *
 * Blocks until the sequence has been transmitted.
 */
int ir_send_raw(uint32_t carrier_hz, const uint16_t *durations_us, size_t count);

/* Same, with a custom wait for each duration (used by the loopback test). */
int ir_send_raw_ex(uint32_t carrier_hz, const uint16_t *durations_us,
		   size_t count, void (*wait_us)(uint32_t us));

/*
 * Visual heartbeat: pulse the IR LED on/off `count` times so the user can
 * see the board is alive by pointing a phone camera at the LED. Most phone
 * cameras render IR as a faint white/violet glow.
 */
void ir_pulse(uint32_t on_ms, uint32_t off_ms, uint32_t count);

/* Hold DAT steady high or low (diagnostics: visible on the module's LED). */
void ir_level(bool high);

#endif /* IR_H_ */
