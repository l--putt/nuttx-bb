/************************************************************************************
 * arch/arm/src/stm32/stm32_adc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_ADC_H
#define __ARCH_ARM_SRC_STM32_STM32_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_ADC_SR_OFFSET        0x0000  /* ADC status register (32-bit) */
#define STM32_ADC_CR1_OFFSET       0x0004  /* ADC control register 1 (32-bit) */
#define STM32_ADC_CR2_OFFSET       0x0008  /* ADC control register 2 (32-bit) */
#define STM32_ADC_SMPR1_OFFSET     0x000c  /* ADC sample time register 1 (32-bit) */
#define STM32_ADC_SMPR2_OFFSET     0x0010  /* ADC sample time register 2 (32-bit) */
#define STM32_ADC_JOFR1_OFFSET     0x0014  /* ADC injected channel data offset register 1 (32-bit) */
#define STM32_ADC_JOFR2_OFFSET     0x0018  /* ADC injected channel data offset register 2 (32-bit) */
#define STM32_ADC_JOFR3_OFFSET     0x001c  /* ADC injected channel data offset register 3 (32-bit) */
#define STM32_ADC_JOFR4_OFFSET     0x0020  /* ADC injected channel data offset register 4 (32-bit) */
#define STM32_ADC_HTR_OFFSET       0x0024  /* ADC watchdog high threshold register (32-bit) */
#define STM32_ADC_LTR_OFFSET       0x0028  /* ADC watchdog low threshold register (32-bit) */
#define STM32_ADC_SQR1_OFFSET      0x002c  /* ADC regular sequence register 1 (32-bit) */
#define STM32_ADC_SQR2_OFFSET      0x0030  /* ADC regular sequence register 2 (32-bit) */
#define STM32_ADC_SQR3_OFFSET      0x0034  /* ADC regular sequence register 3 (32-bit) */
#define STM32_ADC_JSQR_OFFSET      0x0038  /* ADC injected sequence register (32-bit) */
#define STM32_ADC_JDR1_OFFSET      0x003c  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR2_OFFSET      0x0040  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR3_OFFSET      0x0044  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_JDR4_OFFSET      0x0048  /* ADC injected data register 1 (32-bit) */
#define STM32_ADC_DR_OFFSET        0x004c  /* ADC regular data register (32-bit) */

/* Register Addresses ***************************************************************/

#if STM32_NADC > 0
#  define STM32_ADC1_SR            (STM32_ADC1_BASE+STM32_ADC_SR_OFFSET)
#  define STM32_ADC1_CR1           (STM32_ADC1_BASE+STM32_ADC_CR1_OFFSET)
#  define STM32_ADC1_CR2           (STM32_ADC1_BASE+STM32_ADC_CR2_OFFSET)
#  define STM32_ADC1_SMPR1         (STM32_ADC1_BASE+STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC1_SMPR2         (STM32_ADC1_BASE+STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC1_JOFR1         (STM32_ADC1_BASE+STM32_ADC_JOFR1_OFFSET)
#  define STM32_ADC1_JOFR2         (STM32_ADC1_BASE+STM32_ADC_JOFR2_OFFSET)
#  define STM32_ADC1_JOFR3         (STM32_ADC1_BASE+STM32_ADC_JOFR3_OFFSET)
#  define STM32_ADC1_JOFR4         (STM32_ADC1_BASE+STM32_ADC_JOFR4_OFFSET)
#  define STM32_ADC1_HTR           (STM32_ADC1_BASE+STM32_ADC_HTR_OFFSET)
#  define STM32_ADC1_LTR           (STM32_ADC1_BASE+STM32_ADC_LTR_OFFSET)
#  define STM32_ADC1_SQR1          (STM32_ADC1_BASE+STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC1_SQR2          (STM32_ADC1_BASE+STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC1_SQR3          (STM32_ADC1_BASE+STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC1_JSQR          (STM32_ADC1_BASE+STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC1_JDR1          (STM32_ADC1_BASE+STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC1_JDR2          (STM32_ADC1_BASE+STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC1_JDR3          (STM32_ADC1_BASE+STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC1_JDR4          (STM32_ADC1_BASE+STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC1_DR            (STM32_ADC1_BASE+STM32_ADC_DR_OFFSET)
#endif

#if STM32_NADC > 1
#  define STM32_ADC2_SR            (STM32_ADC2_BASE+STM32_ADC_SR_OFFSET)
#  define STM32_ADC2_CR1           (STM32_ADC2_BASE+STM32_ADC_CR1_OFFSET)
#  define STM32_ADC2_CR2           (STM32_ADC2_BASE+STM32_ADC_CR2_OFFSET)
#  define STM32_ADC2_SMPR1         (STM32_ADC2_BASE+STM32_ADC_SMPR1_OFFSET)
#  define STM32_ADC2_SMPR2         (STM32_ADC2_BASE+STM32_ADC_SMPR2_OFFSET)
#  define STM32_ADC2_JOFR1         (STM32_ADC2_BASE+STM32_ADC_JOFR1_OFFSET)
#  define STM32_ADC2_JOFR2         (STM32_ADC2_BASE+STM32_ADC_JOFR2_OFFSET)
#  define STM32_ADC2_JOFR3         (STM32_ADC2_BASE+STM32_ADC_JOFR3_OFFSET)
#  define STM32_ADC2_JOFR4         (STM32_ADC2_BASE+STM32_ADC_JOFR4_OFFSET)
#  define STM32_ADC2_HTR           (STM32_ADC2_BASE+STM32_ADC_HTR_OFFSET)
#  define STM32_ADC2_LTR           (STM32_ADC2_BASE+STM32_ADC_LTR_OFFSET)
#  define STM32_ADC2_SQR1          (STM32_ADC2_BASE+STM32_ADC_SQR1_OFFSET)
#  define STM32_ADC2_SQR2          (STM32_ADC2_BASE+STM32_ADC_SQR2_OFFSET)
#  define STM32_ADC2_SQR3          (STM32_ADC2_BASE+STM32_ADC_SQR3_OFFSET)
#  define STM32_ADC2_JSQR          (STM32_ADC2_BASE+STM32_ADC_JSQR_OFFSET)
#  define STM32_ADC2_JDR1          (STM32_ADC2_BASE+STM32_ADC_JDR1_OFFSET)
#  define STM32_ADC2_JDR2          (STM32_ADC2_BASE+STM32_ADC_JDR2_OFFSET)
#  define STM32_ADC2_JDR3          (STM32_ADC2_BASE+STM32_ADC_JDR3_OFFSET)
#  define STM32_ADC2_JDR4          (STM32_ADC2_BASE+STM32_ADC_JDR4_OFFSET)
#  define STM32_ADC2_DR            (STM32_ADC2_BASE+STM32_ADC_DR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* ADC status register */

#define ADC_SR_AWD                (1 << 0)  /* Bit 0 : Analog watchdog flag */
#define ADC_SR_EOC                (1 << 1)  /* Bit 1 : End of conversion */
#define ADC_SR_JEOC               (1 << 2)  /* Bit 2 : Injected channel end of conversion */
#define ADC_SR_JSTRT              (1 << 3)  /* Bit 3 : Injected channel Start flag */
#define ADC_SR_STRT               (1 << 4)  /* Bit 4 : Regular channel Start flag */

/* ADC control register 1 */

#define ADC_CR1_AWDCH_SHIFT       (0)       /* Bits 4-0: Analog watchdog channel select bits */
#define ADC_CR1_AWDCH_MASK        (0x1f << ADC_CR1_AWDCH_SHIFT)
#define ADC_CR1_EOCIE             (1 << 5)  /* Bit 5: Interrupt enable for EOC */
#define ADC_CR1_AWDIE             (1 << 6)  /* Bit 6: Analog Watchdog interrupt enable */
#define ADC_CR1_JEOCIE            (1 << 7)  /* Bit 7: Interrupt enable for injected channels */
#define ADC_CR1_SCAN              (1 << 8)  /* Bit 8: Scan mode */
#define ADC_CR1_AWDSGL            (1 << 9)  /* Bit 9: Enable the watchdog on a single channel in scan mode */
#define ADC_CR1_JAUTO             (1 << 10) /* Bit 10: Automatic Injected Group conversion */
#define ADC_CR1_DISCEN            (1 << 11) /* Bit 11: Discontinuous mode on regular channels */
#define ADC_CR1_JDISCEN           (1 << 12) /* Bit 12: Discontinuous mode on injected channels */
#define ADC_CR1_DISCNUM_MASK      (0x07 << ADC_CR1_DISCNUM_SHIFT)
#define ADC_CR1_DISCNUM_SHIFT     (13)      /* Bits 15-13: Discontinuous mode channel count */
#define ADC_CR1_DUALMOD_MASK      (0x0f << ADC_CR1_DUALMOD_SHIFT)
#  define ADC_CR1_IND             (0 << ADC_CR1_DUALMOD_SHIFT) /* 0000: Independent mode */
#  define ADC_CR1_RSIS            (1 << ADC_CR1_DUALMOD_SHIFT) /* 0001: Combined regular simultaneous + injected simultaneous mode */
#  define ADC_CR1_RSAT            (2 << ADC_CR1_DUALMOD_SHIFT) /* 0010: Combined regular simultaneous + alternate trigger mode */
#  define ADC_CR1_ISFI            (3 << ADC_CR1_DUALMOD_SHIFT) /* 0011: Combined injected simultaneous + fast interleaved mode */
#  define ADC_CR1_ISFL            (4 << ADC_CR1_DUALMOD_SHIFT) /* 0100: Combined injected simultaneous + slow Interleaved mode */
#  define ADC_CR1_IS              (5 << ADC_CR1_DUALMOD_SHIFT) /* 0101: Injected simultaneous mode only */
#  define ADC_CR1_RS              (6 << ADC_CR1_DUALMOD_SHIFT) /* 0110: Regular simultaneous mode only */
#  define ADC_CR1_FI              (7 << ADC_CR1_DUALMOD_SHIFT) /* 0111: Fast interleaved mode only */
#  define ADC_CR1_SI              (8 << ADC_CR1_DUALMOD_SHIFT) /* 1000: Slow interleaved mode only */
#  define ADC_CR1_AT              (9 << ADC_CR1_DUALMOD_SHIFT) /* 1001: Alternate trigger mode only */
#define ADC_CR1_DUALMOD_SHIFT     (16)      /* Bits 19-16: Dual mode selection */
#define ADC_CR1_JAWDEN            (1 << 22) /* Bit 22: Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN             (1 << 23) /* Bit 23: Analog watchdog enable on regular channels */

/* ADC control register 2 */

#define ADC_CR2_ADON              (1 << 0)  /* Bit 0: A/D Converter ON / OFF */
#define ADC_CR2_CONT              (1 << 1)  /* Bit 1: Continuous Conversion */
#define ADC_CR2_CAL               (1 << 2)  /* Bit 2: A/D Calibration */
#define ADC_CR2_RSTCAL            (1 << 3)  /* Bit 3: Reset Calibration */
#define ADC_CR2_DMA               (1 << 8)  /* Bit 8: Direct Memory access mode */
#define ADC_CR2_ALIGN             (1 << 11) /* Bit 11: Data Alignment */
#define ADC_CR2_JEXTSEL_SHIFT     (12)      /* Bits 14-12: External event select for injected group */
#define ADC_CR2_JEXTSEL_MASK      (7 << ADC_CR2_JEXTSEL_SHIFT)
#  define ADC_CR2_JEXTSEL_T1TRG0  (0 << ADC_CR2_JEXTSEL_SHIFT) /* 000: Timer 1 TRGO event */
#  define ADC_CR2_JEXTSEL_T1CC4   (1 << ADC_CR2_JEXTSEL_SHIFT) /* 001: Timer 1 CC4 event */
#  define ADC_CR2_JEXTSEL_T2TRG0  (2 << ADC_CR2_JEXTSEL_SHIFT) /* 010: Timer 2 TRGO event */
#  define ADC_CR2_JEXTSEL_T2CC1   (3 << ADC_CR2_JEXTSEL_SHIFT) /* 011: Timer 2 CC1 event */
#  define ADC_CR2_JEXTSEL_T3CC4   (4 << ADC_CR2_JEXTSEL_SHIFT) /* 100: Timer 3 CC4 event */
#  define ADC_CR2_JEXTSEL_T4TRG0  (5 << ADC_CR2_JEXTSEL_SHIFT) /* 101: Timer 4 TRGO event */
#  define ADC_CR2_JEXTSEL_EXTI15  (6 << ADC_CR2_JEXTSEL_SHIFT) /* 110: EXTI line15 */
#  define ADC_CR2_JEXTSEL_SWSTART (7 << ADC_CR2_JEXTSEL_SHIFT) /* 111: JSWSTART */
#define ADC_CR2_JEXTTRIG          (1 << 15) /* Bit 15: External Trigger Conversion mode for injected channels */
#define ADC_CR2_EXTSEL_SHIFT      (17)      /* Bits 19-17: External Event Select for regular group */
#define ADC_CR2_EXTSEL_MASK       (7 << ADC_CR2_EXTSEL_SHIFT)
#  define ADC_CR2_EXTSEL_T1CC1    (0 << ADC_CR2_EXTSEL_SHIFT) /* 000: Timer 1 CC1 event */
#  define ADC_CR2_EXTSEL_T1CC2    (1 << ADC_CR2_EXTSEL_SHIFT) /* 001: Timer 1 CC2 event */
#  define ADC_CR2_EXTSEL_T1CC3    (2 << ADC_CR2_EXTSEL_SHIFT) /* 010: Timer 1 CC3 event */
#  define ADC_CR2_EXTSEL_T2CC2    (3 << ADC_CR2_EXTSEL_SHIFT) /* 011: Timer 2 CC2 event */
#  define ADC_CR2_EXTSEL_T3TRG0   (4 << ADC_CR2_EXTSEL_SHIFT) /* 100: Timer 3 TRGO event */
#  define ADC_CR2_EXTSEL_T4CC4    (5 << ADC_CR2_EXTSEL_SHIFT) /* 101: Timer 4 CC4 event */
#  define ADC_CR2_EXTSEL_EXTI11   (6 << ADC_CR2_EXTSEL_SHIFT) /* 110: EXTI line11 */
#  define ADC_CR2_EXTSEL_SWSTART  (7 << ADC_CR2_EXTSEL_SHIFT) /* 111: SWSTART */
#define ADC_CR2_EXTTRIG           (1 << 20) /* Bit 20: External Trigger Conversion mode for regular channels */
#define ADC_CR2_JSWSTART          (1 << 21) /* Bit 21: Start Conversion of injected channels */
#define ADC_CR2_SWSTART           (1 << 22) /* Bit 22: Start Conversion of regular channels */
#define ADC_CR2_TSVREFE           (1 << 23) /* Bit 23: Temperature Sensor and VREFINT Enable */

/* ADC sample time register 1 */

#define ADC_SMPR1_SMP10_SHIFT     (0)       /* Bits 2-0: Channel 10 Sample time selection */
#define ADC_SMPR1_SMP10_MASK      (7 << ADC_SMPR1_SMP10_SHIFT)
#define ADC_SMPR1_SMP11_SHIFT     (3)       /* Bits 5-3: Channel 11 Sample time selection */
#define ADC_SMPR1_SMP11_MASK      (7 << ADC_SMPR1_SMP11_SHIFT)
#define ADC_SMPR1_SMP12_SHIFT     (6)       /* Bits 8-6: Channel 12 Sample time selection */
#define ADC_SMPR1_SMP12_MASK      (7 << ADC_SMPR1_SMP12_SHIFT)
#define ADC_SMPR1_SMP13_SHIFT     (9)       /* Bits 11-9: Channel 13 Sample time selection */
#define ADC_SMPR1_SMP13_MASK      (7 << ADC_SMPR1_SMP13_SHIFT)
#define ADC_SMPR1_SMP14_SHIFT     (12)      /* Bits 14-12: Channel 14 Sample time selection */
#define ADC_SMPR1_SMP14_MASK      (7 << ADC_SMPR1_SMP14_SHIFT)
#define ADC_SMPR1_SMP15_SHIFT     (15)      /* Bits 17-15: Channel 15 Sample time selection */
#define ADC_SMPR1_SMP15_MASK      (7 << ADC_SMPR1_SMP15_SHIFT)
#define ADC_SMPR1_SMP16_SHIFT     (18)      /* Bits 20-18: Channel 16 Sample time selection */
#define ADC_SMPR1_SMP16_MASK      (7 << ADC_SMPR1_SMP16_SHIFT)
#define ADC_SMPR1_SMP17_SHIFT     (21)      /* Bits 23-21: Channel 17 Sample time selection */
#define ADC_SMPR1_SMP17_MASK      (7 << ADC_SMPR1_SMP17_SHIFT)

#define ADC_SMPR_1p5              0         /* 000: 1.5 cycles */
#define ADC_SMPR_7p5              1         /* 001: 7.5 cycles */
#define ADC_SMPR_13p5             2         /* 010: 13.5 cycles */
#define ADC_SMPR_28p5             3         /* 011: 28.5 cycles */
#define ADC_SMPR_41p5             4         /* 100: 41.5 cycles */
#define ADC_SMPR_55p5             5         /* 101: 55.5 cycles */
#define ADC_SMPR_71p5             6         /* 110: 71.5 cycles */
#define ADC_SMPR_239p5            7         /* 111: 239.5 cycles */

/* ADC sample time register 2 */

#define ADC_SMPR2_SMP0_SHIFT      (0)       /* Bits 2-0: Channel 0 Sample time selection */
#define ADC_SMPR2_SMP0_MASK       (7 << ADC_SMPR1_SMP0_SHIFT)
#define ADC_SMPR2_SMP1_SHIFT      (3)       /* Bits 5-3: Channel 1 Sample time selection */
#define ADC_SMPR2_SMP1_MASK       (7 << ADC_SMPR1_SMP1_SHIFT)
#define ADC_SMPR2_SMP2_SHIFT      (6)       /* Bits 8-6: Channel 2 Sample time selection */
#define ADC_SMPR2_SMP2_MASK       (7 << ADC_SMPR1_SMP2_SHIFT)
#define ADC_SMPR2_SMP3_SHIFT      (9)       /* Bits 11-9: Channel 3 Sample time selection */
#define ADC_SMPR2_SMP3_MASK       (7 << ADC_SMPR1_SMP3_SHIFT)
#define ADC_SMPR2_SMP4_SHIFT      (12)      /* Bits 14-12: Channel 4 Sample time selection */
#define ADC_SMPR2_SMP4_MASK       (7 << ADC_SMPR1_SMP4_SHIFT)
#define ADC_SMPR2_SMP5_SHIFT      (15)      /* Bits 17-15: Channel 5 Sample time selection */
#define ADC_SMPR2_SMP5_MASK       (7 << ADC_SMPR1_SMP5_SHIFT)
#define ADC_SMPR2_SMP6_SHIFT      (18)      /* Bits 20-18: Channel 6 Sample time selection */
#define ADC_SMPR2_SMP6_MASK       (7 << ADC_SMPR1_SMP6_SHIFT)
#define ADC_SMPR2_SMP7_SHIFT      (21)      /* Bits 23-21: Channel 7 Sample time selection */
#define ADC_SMPR2_SMP7_MASK       (7 << ADC_SMPR1_SMP7_SHIFT)
#define ADC_SMPR2_SMP8_SHIFT      (24)      /* Bits 26-24: Channel 8 Sample time selection */
#define ADC_SMPR2_SMP8_MASK       (7 << ADC_SMPR1_SMP8_SHIFT)
#define ADC_SMPR2_SMP9_SHIFT      (27)      /* Bits 29-27: Channel 9 Sample time selection */
#define ADC_SMPR2_SMP9_MASK       (7 << ADC_SMPR1_SMP9_SHIFT)

/* ADC injected channel data offset register 1-4 */

#define ADC_JOFR_SHIFT           (0)        /* Bits 11-0: Data offset for injected channel x */
#define ADC_JOFR_MASK            (0x0fff << ADC_JOFR_SHIFT)

/* ADC watchdog high threshold register */

#define ADC_HTR_SHIFT            (0)        /* Bits 11-0: Analog watchdog high threshold */
#define ADC_HTR_MASK             (0x0fff << ADC_HTR_SHIFT)

/* ADC watchdog low threshold register */

#define ADC_LTR_SHIFT            (0)        /* Bits 11:0: Analog watchdog low threshold */
#define ADC_LTR_MASK             (0x0fff << ADC_LTR_SHIFT)

/* ADC regular sequence register 1 */

#define ADC_SQR1_SQ13_SHIFT     (0)         /* Bits 4-0: 13th conversion in regular sequence */
#define ADC_SQR1_SQ13_MASK      (0x1f << ADC_SQR1_SQ13_SHIFT)
#define ADC_SQR1_SQ14_SHIFT     (5)         /* Bits 9-5: 14th conversion in regular sequence */
#define ADC_SQR1_SQ14_MASK      (0x1f << ADC_SQR1_SQ14_SHIFT)
#define ADC_SQR1_SQ15_SHIFT     (10)        /* Bits 14-10: 15th conversion in regular sequence */
#define ADC_SQR1_SQ15_MASK      (0x1f << ADC_SQR1_SQ15_SHIFT)
#define ADC_SQR1_SQ16_SHIFT     (15)        /* Bits 19-15: 16th conversion in regular sequence */
#define ADC_SQR1_SQ16_MASK      (0x1f << ADC_SQR1_SQ16_SHIFT)
#define ADC_SQR1_L_SHIFT        (20)        /* Bits 23:20 L[3:0]: Regular channel sequence length */
#define ADC_SQR1_L_MASK         (0x0f << ADC_SQR1_L_SHIFT)

/* ADC regular sequence register 2 */

#define ADC_SQR1_SQ7_SHIFT      (0)         /* Bits 4-0: 7th conversion in regular sequence */
#define ADC_SQR2_SQ7_MASK       (0x1f << ADC_SQR2_SQ7_SHIFT)
#define ADC_SQR2_SQ8_SHIFT      (5)         /* Bits 9-5: 8th conversion in regular sequence */
#define ADC_SQR2_SQ8_MASK       (0x1f << ADC_SQR2_SQ8_SHIFT)
#define ADC_SQR2_SQ9_SHIFT      (10)        /* Bits 14-10: 9th conversion in regular sequence */
#define ADC_SQR2_SQ9_MASK       (0x1f << ADC_SQR2_SQ9_SHIFT)
#define ADC_SQR2_SQ10_SHIFT     (15)        /* Bits 19-15: 10th conversion in regular sequence */
#define ADC_SQR2_SQ10_MASK      (0x1f << ADC_SQR2_SQ10_SHIFT)
#define ADC_SQR2_SQ11_SHIFT     (20)        /* Bits 24:20: 11th conversion in regular sequence */
#define ADC_SQR2_SQ11_MASK      (0x1f << ADC_SQR2_SQ11_SHIFT )
#define ADC_SQR2_SQ12_SHIFT     (25)        /* Bits 29:25: 12th conversion in regular sequence */
#define ADC_SQR2_SQ12_MASK      (0x1f << ADC_SQR2_SQ12_SHIFT)

/* ADC regular sequence register 3 */

#define ADC_SQR3_SQ1_SHIFT      (0)         /* Bits 4-0: 1st conversion in regular sequence */
#define ADC_SQR3_SQ1_MASK       (0x1f << ADC_SQR3_SQ1_SHIFT)
#define ADC_SQR3_SQ2_SHIFT      (5)         /* Bits 9-5: 2nd conversion in regular sequence */
#define ADC_SQR3_SQ2_MASK       (0x1f << ADC_SQR3_SQ2_SHIFT)
#define ADC_SQR3_SQ3_SHIFT      (10)        /* Bits 14-10: 3rd conversion in regular sequence */
#define ADC_SQR3_SQ3_MASK       (0x1f << ADC_SQR3_SQ3_SHIFT)
#define ADC_SQR3_SQ4_SHIFT      (15)        /* Bits 19-15: 4th conversion in regular sequence */
#define ADC_SQR3_SQ4_MASK       (0x1f << ADC_SQR3_SQ4_SHIFT)
#define ADC_SQR3_SQ5_SHIFT      (20)        /* Bits 24:20: 5th conversion in regular sequence */
#define ADC_SQR3_SQ5_MASK       (0x1f << ADC_SQR3_SQ5_SHIFT )
#define ADC_SQR3_SQ6_SHIFT      (25)        /* Bits 29:25: 6th conversion in regular sequence */
#define ADC_SQR3_SQ6_MASK       (0x1f << ADC_SQR3_SQ6_SHIFT)

/* ADC injected sequence register */

#define ADC_JSQR_JSQ1_SHIFT     (0)        /* Bits 4:0 JSQ1[4:0]: 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_MASK      (0x1f << ADC_JSQR_JSQ1_SHIFT)
#define ADC_JSQR_JSQ2_SHIFT     (5)        /* Bits 9:5 JSQ2[4:0]: 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_MASK      (0x1f << ADC_JSQR_JSQ2_MASK)
#define ADC_JSQR_JSQ3_SHIFT     (10)       /* Bits 14:10 JSQ3[4:0]: 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_MASK      (0x1f << ADC_JSQR_JSQ3_SHIFT)
#define ADC_JSQR_JSQ4_SHIFT     (15)       /* Bits 19:15 JSQ4[4:0]: 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_MASK      (0x1f << ADC_JSQR_JSQ4_SHIFT)
#define ADC_JSQR_JL_SHIFT       (20)       /* Bits 21:20 JL[1:0]: Injected Sequence length */
#define ADC_JSQR_JL_MASK        (3 << ADC_JSQR_JL_SHIFT)

/* ADC injected data register 1-4 */

#define ADC_JDR_SHIFT           (0)        /* Bits 15-0: Injected data */
#define ADC_JDR_MASK            (0xffff << ADC_JDR_SHIFT)

/* ADC regular data register */

#define ADC_DR_DATA_SHIFT       (0)        /* Bits 15:0 Regular data */
#define ADC_DR_DATA_MASK        (0xffff << yyyy)
#define ADC_DR_ADC2DATA_SHIFT   (16)       /* Bits 31:16: ADC2 data */
#define ADC_DR_ADC2DATA_MASK    (0xffff << yyyy)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_STM32_ADC_H */
