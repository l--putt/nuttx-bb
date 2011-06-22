/****************************************************************************
 * arch/arm/src/calypso/calypso_irq.c
 * Driver for Calypso IRQ controller
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/calypso/memory.h>

#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BASE_ADDR_IRQ	0xfffffa00
#define BASE_ADDR_IBOOT_EXC	0x0080001C

enum irq_reg {
	IT_REG1		= 0x00,
	IT_REG2		= 0x02,
	MASK_IT_REG1	= 0x08,
	MASK_IT_REG2	= 0x0a,
	IRQ_NUM		= 0x10,
	FIQ_NUM		= 0x12,
	IRQ_CTRL	= 0x14,
};

#define ILR_IRQ(x)	(0x20 + (x*2))
#define IRQ_REG(x)	((void *)BASE_ADDR_IRQ + (x))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;
extern uint32_t _exceptions;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t default_irq_prio[] = {
	[IRQ_WATCHDOG]		= 0xff,
	[IRQ_TIMER1]		= 0xff,
	[IRQ_TIMER2]		= 0xff,
	[IRQ_TSP_RX]		= 0,
	[IRQ_TPU_FRAME]		= 3,
	[IRQ_TPU_PAGE]		= 0xff,
	[IRQ_SIMCARD]		= 0xff,
	[IRQ_UART_MODEM]	= 8,
	[IRQ_KEYPAD_GPIO]	= 4,
	[IRQ_RTC_TIMER]		= 9,
	[IRQ_RTC_ALARM_I2C]	= 10,
	[IRQ_ULPD_GAUGING]	= 2,
	[IRQ_EXTERNAL]		= 12,
	[IRQ_SPI]		= 0xff,
	[IRQ_DMA]		= 0xff,
	[IRQ_API]		= 0xff,
	[IRQ_SIM_DETECT]	= 0,
	[IRQ_EXTERNAL_FIQ]	= 7,
	[IRQ_UART_IRDA]		= 2,
	[IRQ_ULPD_GSM_TIMER]	= 1,
	[IRQ_GEA]		= 0xff,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void _irq_enable(enum irq_nr nr, int enable)
{
	uint16_t *reg = IRQ_REG(MASK_IT_REG1);
	uint16_t val;

	if (nr > 15) {
		reg = IRQ_REG(MASK_IT_REG2);
		nr -= 16;
	}

	val = readw(reg);
	if (enable)
		val &= ~(1 << nr);
	else
		val |= (1 << nr);
	writew(val, reg);
}

static void set_default_priorities(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(default_irq_prio); i++) {
		uint16_t val;
		uint8_t prio = default_irq_prio[i];
		if (prio > 31)
			prio = 31;

		val = readw(IRQ_REG(ILR_IRQ(i)));
		val &= ~(0x1f << 2);
		val |= prio << 2;
		
		/* Make edge mode default. Hopefully causes less trouble */
		val |= 0x02;
		
		writew(val, IRQ_REG(ILR_IRQ(i)));
	}
}

/* Install the exception handlers to where the ROM loader jumps */
static void calypso_exceptions_install(void)
{
	uint32_t *exceptions_dst = (uint32_t *) BASE_ADDR_IBOOT_EXC;
	uint32_t *exceptions_src = &_exceptions;
	int i;

	for (i = 0; i < 7; i++)
		*exceptions_dst++ = *exceptions_src++;

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Setup the IRQ and FIQ controllers
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
	/* Prepare hardware */
	calypso_exceptions_install();
	current_regs = NULL;

	/* Switch to internal ROM */
	calypso_bootrom(1);

	/* set default priorities */
	set_default_priorities();

	/* mask all interrupts off */
	writew(0xffff, IRQ_REG(MASK_IT_REG1));
	writew(0xffff, IRQ_REG(MASK_IT_REG2));

	/* clear all pending interrupts */
	writew(0, IRQ_REG(IT_REG1));
	writew(0, IRQ_REG(IT_REG2));

	/* enable interrupts globally to the ARM core */
#ifndef CONFIG_SUPPRESS_INTERRUPTS
	irqrestore(SVC_MODE | PSR_F_BIT);
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
	if((unsigned)irq < NR_IRQS)
		_irq_enable(irq, 0);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
	if((unsigned)irq < NR_IRQS)
		_irq_enable(irq, 1);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int nr, int prio)
{
	uint16_t val;

	if (prio == -1)
		prio = default_irq_prio[nr];

	if (prio > 31)
		prio = 31;

	val = prio << 2;
/*
	if (edge)
		val |= 0x02;
	if (fiq)
		val |= 0x01;
*/
	writew(val, IRQ_REG(ILR_IRQ(nr)));
	
	return 0; // XXX: what's the return???
}
#endif

/****************************************************************************
 * Entry point for interrupts
 ****************************************************************************/

void up_decodeirq(uint32_t *regs)
{
	uint8_t num, tmp;
	uint32_t *saved_regs;

	/* XXX: What is this???
	 * Passed to but ignored in IRQ handlers
	 * Only valid meaning is apparently non-NULL == IRQ context */
	saved_regs = (uint32_t *)current_regs;
	current_regs = regs;

	/* Detect & deliver the IRQ */
	num = readb(IRQ_REG(IRQ_NUM)) & 0x1f;
	irq_dispatch(num, regs);

	/* Start new IRQ agreement */
	tmp = readb(IRQ_REG(IRQ_CTRL));
	tmp |= 0x01;
	writeb(tmp, IRQ_REG(IRQ_CTRL));

	current_regs = saved_regs;
}

/****************************************************************************
 * Entry point for FIQs
 ****************************************************************************/

void calypso_fiq(void)
{
	uint8_t num, tmp;
	uint32_t *regs;

	/* XXX: What is this???
	 * Passed to but ignored in IRQ handlers
	 * Only valid meaning is apparently non-NULL == IRQ context */
	regs = (uint32_t *)current_regs;
	current_regs = (uint32_t *)&num;

	/* Detect & deliver like an IRQ but we are in FIQ context */
	num = readb(IRQ_REG(FIQ_NUM)) & 0x1f;
	irq_dispatch(num, regs);

	/* Start new FIQ agreement */
	tmp = readb(IRQ_REG(IRQ_CTRL));
	tmp |= 0x02;
	writeb(tmp, IRQ_REG(IRQ_CTRL));

	current_regs = regs;
}
