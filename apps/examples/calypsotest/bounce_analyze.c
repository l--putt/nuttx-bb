/****************************************************************************
 * Analyze bouncing properties of keypad switches
 *
 * (C) 2011 by Stefan Richter <ichgeh@l--putt.de>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs.h>

#include <stdio.h>
#include <stdint.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <arch/calypso/defines.h>
#include <arch/calypso/memory.h>
#include <arch/calypso/timer.h>


/****************************************************************************
 * HW access
 ****************************************************************************/

#define BASE_ADDR_ARMIO	0xfffe4800
#define ARMIO_REG(x)	((void *)BASE_ADDR_ARMIO + (x))

enum armio_reg {
	LATCH_IN	= 0x00,
	LATCH_OUT	= 0x02,
	IO_CNTL		= 0x04,
	CNTL_REG	= 0x06,
	LOAD_TIM	= 0x08,
	KBR_LATCH_REG	= 0x0a,
	KBC_REG		= 0x0c,
	BUZZ_LIGHT_REG	= 0x0e,
	LIGHT_LEVEL	= 0x10,
	BUZZER_LEVEL	= 0x12,
	GPIO_EVENT_MODE	= 0x14,
	KBD_GPIO_INT	= 0x16,
	KBD_GPIO_MASKIT	= 0x18,
	GPIO_DEBOUNCING	= 0x1a,
	GPIO_LATCH	= 0x1c,
};


/****************************************************************************
 * Debug helpers
 ****************************************************************************/

#if 0

#define ARMIO_LATCH_OUT 0xfffe4802
#define IO_CNTL_REG	0xfffe4804
#define ASIC_CONF_REG	0xfffef008

static void lights_on(void)
{
	uint16_t reg;

	reg = readw(ASIC_CONF_REG);
	/* LCD Set I/O(3) / SA0 to I/O(3) mode */
	reg &= ~( (1 << 12) | (1 << 10) | (1 << 7) | (1 << 1)) ;
	/* don't set function pins to I2C Mode, C155 uses UWire */
	/* TWL3025: Set SPI+RIF RX clock to rising edge */
	reg |= (1 << 13) | (1 << 14);
	writew(reg, ASIC_CONF_REG);

	/* LCD Set I/O(3) to output mode and enable C155 backlight (IO1) */
	/* FIXME: Put the display backlight control to backlight.c */
	reg = readw(IO_CNTL_REG);
	reg &= ~( (1 << 3) | (1 << 1));
	writew(reg, IO_CNTL_REG);

	/* LCD Set I/O(3) output low */
	reg = readw(ARMIO_LATCH_OUT);
	reg &= ~(1 << 3);
	reg |= (1 << 1);
	writew(reg, ARMIO_LATCH_OUT);
}

static void lights_off(void)
{
	uint16_t reg;
	reg = readw(ARMIO_LATCH_OUT);
	reg &= ~(1 << 1);
	writew(reg, ARMIO_LATCH_OUT);
}

#endif

/****************************************************************************
 * ARMIO interrupt handler
 *   posts keypad semaphore
 *   forward GPIO changes
 ****************************************************************************/

static sem_t kbdsem;
static int iqq = 0;

// runs at 13MHz / 32 clock
static uint16_t systimer_cnt;
static uint16_t systimer_cnt_after;

static int kbd_gpio_irq(int irq, uint32_t *regs)
{
	int i;

	/* We'll get into trouble if we post too often */	
	if(iqq++ == 0) {
		sem_post(&kbdsem);
		systimer_cnt = hwtimer_read(2);
	}

	// Turn off column drivers
	writew(0x1f, ARMIO_REG(KBC_REG));
	
	// Wait for propagation
	do {
		i = readw(ARMIO_REG(KBR_LATCH_REG));
	} while(i != 0xffff);

	systimer_cnt_after = hwtimer_read(2);

	/* Restore zeros */
	writew(0, ARMIO_REG(KBC_REG));

	return 0;
}


/****************************************************************************
 * Entry point for test:
 *   setup HW
 *   wait for button event
 *   dump recordings to console
 *   then: loop
 ****************************************************************************/

void bounce_analyze(void)
{
	uint32_t count = 0;
	uint16_t kbd = 0;

	/* Enable ARMIO clock */
	writew(1<<5, ARMIO_REG(CNTL_REG));

	/* Mask GPIO interrupt, unmask keypad interrupt */
	writew(1<<1, ARMIO_REG(KBD_GPIO_MASKIT));

	/* Semaphore; helps leaving IRQ ctx as soon as possible */
	sem_init(&kbdsem, 0, 0);

	/* Drive cols low in idle state such that all buttons cause events */
	writew(0, ARMIO_REG(KBC_REG));

#if 0
	/*
	 * Determine the propagation time for changes on keypad inputs.
	 * systimer_cnt - systimer_cnt_after yields 2 to 3 clock cycles.
	 * Considering overhead, this seems to be a sync with 2 FFs.
	 */

	/*
	 * Attach and enable the interrupt here, otherwise pwr button gets
	 * stuck in IRQ handler
	 */
	irq_attach(IRQ_KEYPAD_GPIO, (xcpt_t)kbd_gpio_irq);
	up_enable_irq(IRQ_KEYPAD_GPIO);

	while(1) {
		sem_wait(&kbdsem);
		printf("%d int's, %d/%d/%d ticks, load_tim %x\n", iqq,
			systimer_cnt, systimer_cnt_after, hwtimer_read(2),
			readw(ARMIO_REG(LOAD_TIM)));
		
		iqq = 0;
		fflush(stdout);
	}
#else
	/*
	 * Continously sample keypad rows. The value should toggle serveral
	 * times before it finally settles. Well, turns out that there seems
	 * to be some kind of debounce magic!?
	 */
	while(1) {
		uint16_t tmp = readw(ARMIO_REG(KBR_LATCH_REG));
		if(tmp != kbd) {
			printf("%04x: %d times\n", kbd, count);
			fflush(stdout);
			kbd = tmp;
			count = 0;
		}
		count++;
	}
#endif
		/*
		for(i=0; i<idx; ) {
			for(j=0; j<24; j++)
				printf("%02x ", rows[i++]);
			printf("\n");
		}
		printf("\n%d int's\n", iqq);
		fflush(stdout);
		*/
}
