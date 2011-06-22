/****************************************************************************
 * Perform tests on CALYPSO hardware
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <arch/calypso/armio.h>
#include <arch/calypso/memory.h>

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

int user_start(int argc, char *argv[])
{
#if 0
	/* Analyze bouncing properties of keypad switches */
	bounce_analyze();
#elif 1
	/* Test keypad char device */
	FILE *fp;
	char ch;

	calypso_armio();
	calypso_keypad();

	fp = fopen("/dev/keypad", "r");
	while(1) {
		ch = fgetc(fp);
		if(ch == 's')
			lights_on();
		if(ch == 'q')
			lights_off();
		printf("%c\n", ch);
		fflush(stdout);
	}
#endif
}
