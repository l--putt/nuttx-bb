/****************************************************************************
 * arch/arm/src/calypso/calypso_timer.c
 * Calypso DBB internal Timer Driver
 *
 * (C) 2010 by Harald Welte <laforge@gnumonks.org>
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

#include <stdio.h>
#include <stdint.h>
#include <nuttx/arch.h>

#include <arch/calypso/defines.h>
#include <arch/calypso/memory.h>
#include <arch/calypso/timer.h>


#define BASE_ADDR_TIMER		0xfffe3800
#define TIMER2_OFFSET		0x3000

#define TIMER_REG(n, m)		(((n)-1) ? (BASE_ADDR_TIMER + TIMER2_OFFSET + (m)) : (BASE_ADDR_TIMER + (m)))

enum timer_reg {
	CNTL_TIMER	= 0x00,
	LOAD_TIMER	= 0x02,
	READ_TIMER	= 0x04,
};

enum timer_ctl {
	CNTL_START		= (1 << 0),
	CNTL_AUTO_RELOAD	= (1 << 1),
	CNTL_CLOCK_ENABLE	= (1 << 5),
};

/* Regular Timers (1 and 2) */

void hwtimer_enable(int num, int on)
{
	uint8_t ctl;

	if (num < 1 || num > 2) {
		printf("Unknown timer %u\n", num);
		return;
	}

	ctl = readb(TIMER_REG(num, CNTL_TIMER));
	if (on)
		ctl |= CNTL_START|CNTL_CLOCK_ENABLE;
	else
		ctl &= ~CNTL_START;
	writeb(ctl, TIMER_REG(num, CNTL_TIMER));
}

void hwtimer_config(int num, uint8_t pre_scale, int auto_reload)
{
	uint8_t ctl;

	ctl = (pre_scale & 0x7) << 2;
	if (auto_reload)
		ctl |= CNTL_AUTO_RELOAD;

	writeb(ctl, TIMER_REG(num, CNTL_TIMER));
}

void hwtimer_load(int num, uint16_t val)
{
	writew(val, TIMER_REG(num, LOAD_TIMER));
}

uint16_t hwtimer_read(int num)
{
	uint8_t ctl = readb(TIMER_REG(num, CNTL_TIMER));

	/* somehow a read results in an abort */
	if ((ctl & (CNTL_START|CNTL_CLOCK_ENABLE)) != (CNTL_START|CNTL_CLOCK_ENABLE))
		return 0xFFFF;
	return readw(TIMER_REG(num, READ_TIMER));
}

/************************************************************
 * Watchdog Timer
 ************************************************************/

#define BASE_ADDR_WDOG		0xfffff800
#define WDOG_REG(m)		(BASE_ADDR_WDOG + m)

enum wdog_reg {
	WD_CNTL_TIMER	= CNTL_TIMER,
	WD_LOAD_TIMER	= LOAD_TIMER,
	WD_READ_TIMER	= 0x02,
	WD_MODE		= 0x04,
};

enum wdog_ctl {
	WD_CTL_START = (1 << 7),
	WD_CTL_AUTO_RELOAD = (1 << 8)
};

enum wdog_mode {
	WD_MODE_DIS_ARM = 0xF5,
	WD_MODE_DIS_CONFIRM = 0xA0,
	WD_MODE_ENABLE = (1 << 15)
};

#define WD_CTL_PRESCALE(value) (((value)&0x07) << 9)

static void wdog_irq(__unused enum irq_nr nr)
{
	puts("=> WATCHDOG\n");
}

void wdog_enable(int on)
{
	if (on) {
#if 0
		irq_config(IRQ_WATCHDOG, 0, 0, 0);
		irq_register_handler(IRQ_WATCHDOG, &wdog_irq);
		irq_enable(IRQ_WATCHDOG);
		writew(WD_MODE_ENABLE, WDOG_REG(WD_MODE));
#endif
	} else {
		writew(WD_MODE_DIS_ARM, WDOG_REG(WD_MODE));
		writew(WD_MODE_DIS_CONFIRM, WDOG_REG(WD_MODE));
	}
}

void wdog_reset(void)
{
#if 0
	// XXX: this is supposed to reset immediately but does not seem to
	writew(0xF5, WDOG_REG(WD_MODE));
	writew(0xFF, WDOG_REG(WD_MODE));
#else
	// enable watchdog
	writew(WD_MODE_ENABLE, WDOG_REG(WD_MODE));
	// force expiration
	writew(0x0000, WDOG_REG(WD_LOAD_TIMER));
	writew(0x0000, WDOG_REG(WD_LOAD_TIMER));
#endif
}

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * Function:  up_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for
 *   various portions of the systems.
 *
 ************************************************************/

int up_timerisr(int irq, uint32_t *regs)
{
   /* Process timer interrupt */

   sched_process_timer();
   return 0;
}

/************************************************************
 * Function:  up_timerinit
 *
 * Description:
 *   Setup Calypso HW timer 2 to cause system ticks.
 *
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ************************************************************/

void up_timerinit(void)
{
  up_disable_irq(IRQ_SYSTIMER);

  /* The timer runs at 13MHz / 32, i.e. 406.25kHz */
  /* 4062 ticks until expiry yields 100Hz interrupt */
  hwtimer_load(2, 4062);
  hwtimer_config(2, 0, 1);
  hwtimer_enable(2, 1);

  /* Attach and enable the timer interrupt */
  irq_attach(IRQ_SYSTIMER, (xcpt_t)up_timerisr);
  up_enable_irq(IRQ_SYSTIMER);
}

