/****************************************************************************
 * arch/arm/include/calypso/irq.h
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

#ifndef __INCLUDE_NUTTX_IRQ_H
#error "This file should never be included directly! Use <nuttx/irq.h>"
#endif

#ifndef _CALYPSO_IRQ_H
#define _CALYPSO_IRQ_H

#ifndef __ASSEMBLY__

enum irq_nr {
	IRQ_WATCHDOG		= 0,
	IRQ_TIMER1		= 1,
	IRQ_TIMER2		= 2,
	IRQ_TSP_RX		= 3,
	IRQ_TPU_FRAME		= 4,
	IRQ_TPU_PAGE		= 5,
	IRQ_SIMCARD		= 6,
	IRQ_UART_MODEM		= 7,
	IRQ_KEYPAD_GPIO		= 8,
	IRQ_RTC_TIMER		= 9,
	IRQ_RTC_ALARM_I2C	= 10,
	IRQ_ULPD_GAUGING	= 11,
	IRQ_EXTERNAL		= 12,
	IRQ_SPI			= 13,
	IRQ_DMA			= 14,
	IRQ_API			= 15,
	IRQ_SIM_DETECT		= 16,
	IRQ_EXTERNAL_FIQ	= 17,
	IRQ_UART_IRDA		= 18,
	IRQ_ULPD_GSM_TIMER	= 19,
	IRQ_GEA			= 20,
	_NR_IRQS
};

#endif /* __ASSEMBLY__ */

/* Don't use _NR_IRQS!!! Won't work in preprocessor... */
#define NR_IRQS			21

#define IRQ_SYSTIMER		IRQ_TIMER2

#endif /* _CALYPSO_IRQ_H */
