/*
 * Crash "black box": the fatal error handler stores reason and faulting PC
 * in RAM that survives a soft reset, so it can be reported on the next boot
 * over USB (no J-Link on this board).
 */
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/fatal.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define CRASH_MAGIC 0xC0FFEE42U

struct crash_info {
	uint32_t magic;
	uint32_t reason;
	uint32_t pc;
	uint32_t lr;
	char     thread[16];
};

static __noinit struct crash_info crash;
static uint32_t boot_resetreas;

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
	const char *name = k_thread_name_get(k_current_get());

	crash.magic = CRASH_MAGIC;
	crash.reason = reason;
	crash.pc = esf ? esf->basic.pc : 0;
	crash.lr = esf ? esf->basic.lr : 0;
	strncpy(crash.thread, name ? name : "?", sizeof(crash.thread) - 1);
	crash.thread[sizeof(crash.thread) - 1] = '\0';

	NVIC_SystemReset();
	CODE_UNREACHABLE;
}

void crashlog_boot(void)
{
	boot_resetreas = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = boot_resetreas; /* write-1-to-clear */
}

void crashlog_report(void)
{
	printk("RESETREAS=0x%08X", boot_resetreas);
	if (crash.magic == CRASH_MAGIC) {
		printk(" CRASH reason=%u pc=0x%08X lr=0x%08X thread=%s\n",
		       crash.reason, crash.pc, crash.lr, crash.thread);
	} else {
		printk(" no crash recorded\n");
	}
}
