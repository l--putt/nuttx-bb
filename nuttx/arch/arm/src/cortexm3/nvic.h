/************************************************************************************
 * arch/arm/src/cortexm3/nvic.h
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

#ifndef __ARCH_ARM_SRC_COMMON_CORTEXM3_NVIC_H
#define __ARCH_ARM_SRC_COMMON_CORTEXM3_NVIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* NVIC base address ****************************************************************/

#define CORTEXM3_NVIC_BASE              0xe000e000

/* NVIC register offsets ************************************************************/

#define NVIC_INTCTRL_TYPE_OFFSET        0x0004 /* Interrupt controller type */
#define NVIC_SYSTICK_CTRL_OFFSET        0x0010 /* SysTick control and status register */
#define NVIC_SYSTICK_RELOAD_OFFSET      0x0014 /* SysTick reload value register */
#define NVIC_SYSTICK_CURRENT_OFFSET     0x0018 /* SysTick current value register */
#define NVIC_SYSTICK_CALIB_OFFSET       0x001c /* SysTick calibration value register */

#define NVIC_IRQ_ENABLE_OFFSET(n)       (0x0100 + 4*((n) >> 5))
#define NVIC_IRQ0_31_ENABLE_OFFSET      0x0100 /* IRQ 0-31 set enable register */
#define NVIC_IRQ32_63_ENABLE_OFFSET     0x0104 /* IRQ 32-63 set enable register */
#define NVIC_IRQ64_95_ENABLE_OFFSET     0x0108 /* IRQ 64-95 set enable register */
#define NVIC_IRQ96_127_ENABLE_OFFSET    0x010c /* IRQ 96-127 set enable register */
#define NVIC_IRQ128_159_ENABLE_OFFSET   0x0110 /* IRQ 128-159 set enable register */
#define NVIC_IRQ160_191_ENABLE_OFFSET   0x0114 /* IRQ 160-191 set enable register */
#define NVIC_IRQ192_223_ENABLE_OFFSET   0x0118 /* IRQ 192-223 set enable register */
#define NVIC_IRQ224_239_ENABLE_OFFSET   0x011c /* IRQ 224-239 set enable register */

#define NVIC_IRQ_CLEAR_OFFSET(n)        (0x0180 + 4*((n) >> 5))
#define NVIC_IRQ0_31_CLEAR_OFFSET       0x0180 /* IRQ 0-31 clear enable register */
#define NVIC_IRQ32_63_CLEAR_OFFSET      0x0184 /* IRQ 32-63 clear enable register */
#define NVIC_IRQ64_95_CLEAR_OFFSET      0x0188 /* IRQ 64-95 clear enable register */
#define NVIC_IRQ96_127_CLEAR_OFFSET     0x018c /* IRQ 96-127 clear enable register */
#define NVIC_IRQ128_159_CLEAR_OFFSET    0x0190 /* IRQ 128-159 clear enable register */
#define NVIC_IRQ160_191_CLEAR_OFFSET    0x0194 /* IRQ 160-191 clear enable register */
#define NVIC_IRQ192_223_CLEAR_OFFSET    0x0198 /* IRQ 192-223 clear enable register */
#define NVIC_IRQ224_239_CLEAR_OFFSET    0x019c /* IRQ 224-2391 clear enable register */

#define NVIC_IRQ_PEND_OFFSET(n)         (0x0200 + 4*((n) >> 5))
#define NVIC_IRQ0_31_PEND_OFFSET        0x0200 /* IRQ 0-31 set pending register */
#define NVIC_IRQ32_63_PEND_OFFSET       0x0204 /* IRQ 32-63 set pending register */
#define NVIC_IRQ64_95_PEND_OFFSET       0x0208 /* IRQ 64-95 set pending register */
#define NVIC_IRQ96_127_PEND_OFFSET      0x020c /* IRQ 96-127 set pending register */
#define NVIC_IRQ128_159_PEND_OFFSET     0x0210 /* IRQ 128-159 set pending register */
#define NVIC_IRQ160_191_PEND_OFFSET     0x0214 /* IRQ 160-191 set pending register */
#define NVIC_IRQ192_223_PEND_OFFSET     0x0218 /* IRQ 192-2231 set pending register */
#define NVIC_IRQ224_239_PEND_OFFSET     0x021c /* IRQ 224-2391 set pending register */

#define NVIC_IRQ_CLRPEND_OFFSET(n)      (0x0280 + 4*((n) >> 5))
#define NVIC_IRQ0_31_CLRPEND_OFFSET     0x0280 /* IRQ 0-31 clear pending register */
#define NVIC_IRQ32_63_CLRPEND_OFFSET    0x0284 /* IRQ 32-63 clear pending register */
#define NVIC_IRQ64_95_CLRPEND_OFFSET    0x0288 /* IRQ 64-95 clear pending register */
#define NVIC_IRQ96_127_CLRPEND_OFFSET   0x028c /* IRQ 96-127 clear pending register */
#define NVIC_IRQ128_159_CLRPEND_OFFSET  0x0290 /* IRQ 128-159 clear pending register */
#define NVIC_IRQ160_191_CLRPEND_OFFSET  0x0294 /* IRQ 160-191 clear pending register */
#define NVIC_IRQ192_223_CLRPEND_OFFSET  0x0298 /* IRQ 192-223 clear pending register */
#define NVIC_IRQ224_239_CLRPEND_OFFSET  0x029c /* IRQ 224-239 clear pending register */

#define NVIC_IRQ_ACTIVE_OFFSET(n)       (0x0300 + 4*((n) >> 5))
#define NVIC_IRQ0_31_ACTIVE_OFFSET      0x0300 /* IRQ 0-31 active bit register */
#define NVIC_IRQ32_63_ACTIVE_OFFSET     0x0304 /* IRQ 32-63 active bit register */
#define NVIC_IRQ64_95_ACTIVE_OFFSET     0x0308 /* IRQ 64-95 active bit register */
#define NVIC_IRQ96_127_ACTIVE_OFFSET    0x030c /* IRQ 96-127 active bit register */
#define NVIC_IRQ128_159_ACTIVE_OFFSET   0x0310 /* IRQ 128-159 active bit register */
#define NVIC_IRQ160_191_ACTIVE_OFFSET   0x0314 /* IRQ 160-191 active bit register */
#define NVIC_IRQ192_223_ACTIVE_OFFSET   0x0318 /* IRQ 192-223 active bit register */
#define NVIC_IRQ224_239_ACTIVE_OFFSET   0x031c /* IRQ 224-239 active bit register */

#define NVIC_IRQ_PRIORITY_OFFSET(n)     (0x0400 + 4*((n) >> 2))
#define NVIC_IRQ0_3_PRIORITY_OFFSET     0x0400 /* IRQ 0-3 priority register */
#define NVIC_IRQ4_7_PRIORITY_OFFSET     0x0404 /* IRQ 4-7 priority register */
#define NVIC_IRQ8_11_PRIORITY_OFFSET    0x0408 /* IRQ 8-11 priority register */
#define NVIC_IRQ12_15_PRIORITY_OFFSET   0x040c /* IRQ 12-15 priority register */
#define NVIC_IRQ16_19_PRIORITY_OFFSET   0x0410 /* IRQ 16-19 priority register */
#define NVIC_IRQ20_23_PRIORITY_OFFSET   0x0414 /* IRQ 20-23 priority register */
#define NVIC_IRQ24_27_PRIORITY_OFFSET   0x0418 /* IRQ 24-29 priority register */
#define NVIC_IRQ28_31_PRIORITY_OFFSET   0x041c /* IRQ 28-31 priority register */
#define NVIC_IRQ32_35_PRIORITY_OFFSET   0x0420 /* IRQ 32-35 priority register */
#define NVIC_IRQ36_39_PRIORITY_OFFSET   0x0424 /* IRQ 36-39 priority register */
#define NVIC_IRQ40_43_PRIORITY_OFFSET   0x0428 /* IRQ 40-43 priority register */
#define NVIC_IRQ44_47_PRIORITY_OFFSET   0x042c /* IRQ 44-47 priority register */
#define NVIC_IRQ48_51_PRIORITY_OFFSET   0x0430 /* IRQ 48-51 priority register */
#define NVIC_IRQ52_55_PRIORITY_OFFSET   0x0434 /* IRQ 52-55 priority register */
#define NVIC_IRQ56_59_PRIORITY_OFFSET   0x0438 /* IRQ 56-59 priority register */
#define NVIC_IRQ60_63_PRIORITY_OFFSET   0x043c /* IRQ 60-63 priority register */
#define NVIC_IRQ64_67_PRIORITY_OFFSET   0x0440 /* IRQ 64-67 priority register */
#define NVIC_IRQ68_71_PRIORITY_OFFSET   0x0444 /* IRQ 68-71 priority register */
#define NVIC_IRQ72_75_PRIORITY_OFFSET   0x0448 /* IRQ 72-75 priority register */
#define NVIC_IRQ76_79_PRIORITY_OFFSET   0x044c /* IRQ 76-79 priority register */
#define NVIC_IRQ80_83_PRIORITY_OFFSET   0x0450 /* IRQ 80-83 priority register */
#define NVIC_IRQ84_87_PRIORITY_OFFSET   0x0454 /* IRQ 84-87 priority register */
#define NVIC_IRQ88_91_PRIORITY_OFFSET   0x0458 /* IRQ 88-91 priority register */
#define NVIC_IRQ92_95_PRIORITY_OFFSET   0x045c /* IRQ 92-95 priority register */
#define NVIC_IRQ96_99_PRIORITY_OFFSET   0x0460 /* IRQ 96-99 priority register */
#define NVIC_IRQ100_103_PRIORITY_OFFSET 0x0464 /* IRQ 100-103 priority register */
#define NVIC_IRQ104_107_PRIORITY_OFFSET 0x0468 /* IRQ 104-107 priority register */
#define NVIC_IRQ108_111_PRIORITY_OFFSET 0x046c /* IRQ 108-111 priority register */
#define NVIC_IRQ112_115_PRIORITY_OFFSET 0x0470 /* IRQ 112-115 priority register */
#define NVIC_IRQ116_119_PRIORITY_OFFSET 0x0474 /* IRQ 116-119 priority register */
#define NVIC_IRQ120_123_PRIORITY_OFFSET 0x0478 /* IRQ 120-123 priority register */
#define NVIC_IRQ124_127_PRIORITY_OFFSET 0x047c /* IRQ 124-127 priority register */
#define NVIC_IRQ128_131_PRIORITY_OFFSET 0x0480 /* IRQ 128-131 priority register */
#define NVIC_IRQ132_135_PRIORITY_OFFSET 0x0484 /* IRQ 132-135 priority register */
#define NVIC_IRQ136_139_PRIORITY_OFFSET 0x0488 /* IRQ 136-139 priority register */
#define NVIC_IRQ140_143_PRIORITY_OFFSET 0x048c /* IRQ 140-143 priority register */
#define NVIC_IRQ144_147_PRIORITY_OFFSET 0x0490 /* IRQ 144-147 priority register */
#define NVIC_IRQ148_151_PRIORITY_OFFSET 0x0494 /* IRQ 148-151 priority register */
#define NVIC_IRQ152_155_PRIORITY_OFFSET 0x0498 /* IRQ 152-155 priority register */
#define NVIC_IRQ156_159_PRIORITY_OFFSET 0x049c /* IRQ 156-159 priority register */
#define NVIC_IRQ160_163_PRIORITY_OFFSET 0x04a0 /* IRQ 160-163 priority register */
#define NVIC_IRQ164_167_PRIORITY_OFFSET 0x04a4 /* IRQ 164-167 priority register */
#define NVIC_IRQ168_171_PRIORITY_OFFSET 0x04a8 /* IRQ 168-171 priority register */
#define NVIC_IRQ172_175_PRIORITY_OFFSET 0x04ac /* IRQ 172-175 priority register */
#define NVIC_IRQ176_179_PRIORITY_OFFSET 0x04b0 /* IRQ 176-179 priority register */
#define NVIC_IRQ180_183_PRIORITY_OFFSET 0x04b4 /* IRQ 180-183 priority register */
#define NVIC_IRQ184_187_PRIORITY_OFFSET 0x04b8 /* IRQ 184-187 priority register */
#define NVIC_IRQ188_191_PRIORITY_OFFSET 0x04bc /* IRQ 188-191 priority register */
#define NVIC_IRQ192_195_PRIORITY_OFFSET 0x04c0 /* IRQ 192-195 priority register */
#define NVIC_IRQ196_199_PRIORITY_OFFSET 0x04c4 /* IRQ 196-199 priority register */
#define NVIC_IRQ200_203_PRIORITY_OFFSET 0x04c8 /* IRQ 200-203 priority register */
#define NVIC_IRQ204_207_PRIORITY_OFFSET 0x04cc /* IRQ 204-207 priority register */
#define NVIC_IRQ208_211_PRIORITY_OFFSET 0x04d0 /* IRQ 208-211 priority register */
#define NVIC_IRQ212_215_PRIORITY_OFFSET 0x04d4 /* IRQ 212-215 priority register */
#define NVIC_IRQ216_219_PRIORITY_OFFSET 0x04d8 /* IRQ 216-219 priority register */
#define NVIC_IRQ220_223_PRIORITY_OFFSET 0x04dc /* IRQ 220-223 priority register */
#define NVIC_IRQ224_227_PRIORITY_OFFSET 0x04e0 /* IRQ 224-227 priority register */
#define NVIC_IRQ228_231_PRIORITY_OFFSET 0x04e4 /* IRQ 228-231 priority register */
#define NVIC_IRQ232_235_PRIORITY_OFFSET 0x04e8 /* IRQ 232-235 priority register */
#define NVIC_IRQ236_239_PRIORITY_OFFSET 0x04ec /* IRQ 236-239 priority register */

#define NVIC_CPUID_BASE_OFFSET          0x0d00 /* CPUID base register */
#define NVIC_INTCTRL_OFFSET             0x0d04 /* Interrupt control state register */
#define NVIC_VECTAB_OFFSET              0x0d08 /* Vector table offset register */
#define NVIC_AIRC_OFFSET                0x0d0c /* Application interrupt/reset contol registr */
#define NVIC_SYSCON_OFFSET              0x0d10 /* System control register */
#define NVIC_CFGCON_OFFSET              0x0d14 /* Configuration control register */
#define NVIC_SYSH_PRIORITY_OFFSET(n)    (0x0d14 + 4*((n) >> 2))
#define NVIC_SYSH4_7_PRIORITY_OFFSET    0x0d18 /* System handlers 4-7 priority register */
#define NVIC_SYSH8_11_PRIORITY_OFFSET   0x0d1c /* System handler 8-11 priority register */
#define NVIC_SYSH12_15_PRIORITY_OFFSET  0x0d20 /* System handler 12-15 priority register */
#define NVIC_SYSHCON_OFFSET             0x0d24 /* System handler control and state register */
#define NVIC_CFAULTS_OFFSET             0x0d28 /* Configurable fault status register */
#define NVIC_HFAULTS_OFFSET             0x0d2c /* Hard fault status register */
#define NVIC_DFAULTS_OFFSET             0x0d30 /* Debug fault status register */
#define NVIC_MEMMANAGE_ADDR_OFFSET      0x0d34 /* Mem manage address register */
#define NVIC_BFAULT_ADDR_OFFSET         0x0d38 /* Bus fault address register */
#define NVIC_AFAULTS_OFFSET             0x0d3c /* Auxiliary fault status register */
#define NVIC_PFR0_OFFSET                0x0d40 /* Processor feature register 0 */
#define NVIC_PFR1_OFFSET                0x0d44 /* Processor feature register 1 */
#define NVIC_DFR0_OFFSET                0x0d48 /* Debug feature register 0 */
#define NVIC_AFR0_OFFSET                0x0d4c /* Auxiliary feature register 0 */
#define NVIC_MMFR0_OFFSET               0x0d50 /* Memory model feature register 0 */
#define NVIC_MMFR1_OFFSET               0x0d54 /* Memory model feature register 1 */
#define NVIC_MMFR2_OFFSET               0x0d58 /* Memory model feature register 2 */
#define NVIC_MMFR3_OFFSET               0x0d5c /* Memory model feature register 3 */
#define NVIC_ISAR0_OFFSET               0x0d60 /* ISA feature register 0 */
#define NVIC_ISAR1_OFFSET               0x0d64 /* ISA feature register 1 */
#define NVIC_ISAR2_OFFSET               0x0d68 /* ISA feature register 2 */
#define NVIC_ISAR3_OFFSET               0x0d6c /* ISA feature register 3 */
#define NVIC_ISAR4_OFFSET               0x0d70 /* ISA feature register 4 */
#define NVIC_STIR_OFFSET                0x0f00 /* Software trigger interrupt register */
#define NVIC_PID4_OFFSET                0x0fd0 /* Peripheral identification register (PID4) */
#define NVIC_PID5_OFFSET                0x0fd4 /* Peripheral identification register (PID5) */
#define NVIC_PID6_OFFSET                0x0fd8 /* Peripheral identification register (PID6) */
#define NVIC_PID7_OFFSET                0x0fdc /* Peripheral identification register (PID7) */
#define NVIC_PID0_OFFSET                0x0fe0 /* Peripheral identification register bits 7:0 (PID0) */
#define NVIC_PID1_OFFSET                0x0fe4 /* Peripheral identification register bits 15:8 (PID1) */
#define NVIC_PID2_OFFSET                0x0fe8 /* Peripheral identification register bits 23:16 (PID2) */
#define NVIC_PID3_OFFSET                0x0fec /* Peripheral identification register bits 23:16 (PID3) */
#define NVIC_CID0_OFFSET                0x0ff0 /* Component identification register bits 7:0 (CID0) */
#define NVIC_CID1_OFFSET                0x0ff4 /* Component identification register bits 15:8 (CID0) */
#define NVIC_CID2_OFFSET                0x0ff8 /* Component identification register bits 23:16 (CID0) */
#define NVIC_CID3_OFFSET                0x0ffc /* Component identification register bits 23:16 (CID0) */

/* NVIC register addresses **********************************************************/

#define NVIC_INTCTRL_TYPE             (CORTEXM3_NVIC_BASE + NVIC_INTCTRL_TYPE_OFFSET)
#define NVIC_SYSTICK_CTRL             (CORTEXM3_NVIC_BASE + NVIC_SYSTICK_CTRL_OFFSET)
#define NVIC_SYSTICK_RELOAD           (CORTEXM3_NVIC_BASE + NVIC_SYSTICK_RELOAD_OFFSET)
#define NVIC_SYSTICK_CURRENT          (CORTEXM3_NVIC_BASE + NVIC_SYSTICK_CURRENT_OFFSET)
#define NVIC_SYSTICK_CALIB            (CORTEXM3_NVIC_BASE + NVIC_SYSTICK_CALIB_OFFSET)

#define NVIC_IRQ_ENABLE(n)            (CORTEXM3_NVIC_BASE + NVIC_IRQ_ENABLE_OFFSET(n))
#define NVIC_IRQ0_31_ENABLE           (CORTEXM3_NVIC_BASE + NVIC_IRQ0_31_ENABLE_OFFSET)
#define NVIC_IRQ32_63_ENABLE          (CORTEXM3_NVIC_BASE + NVIC_IRQ32_63_ENABLE_OFFSET)
#define NVIC_IRQ64_95_ENABLE          (CORTEXM3_NVIC_BASE + NVIC_IRQ64_95_ENABLE_OFFSET)
#define NVIC_IRQ96_127_ENABLE         (CORTEXM3_NVIC_BASE + NVIC_IRQ96_127_ENABLE_OFFSET)
#define NVIC_IRQ128_159_ENABLE        (CORTEXM3_NVIC_BASE + NVIC_IRQ128_159_ENABLE_OFFSET)
#define NVIC_IRQ160_191_ENABLE        (CORTEXM3_NVIC_BASE + NVIC_IRQ160_191_ENABLE_OFFSET)
#define NVIC_IRQ192_223_ENABLE        (CORTEXM3_NVIC_BASE + NVIC_IRQ192_223_ENABLE_OFFSET)
#define NVIC_IRQ224_239_ENABLE        (CORTEXM3_NVIC_BASE + NVIC_IRQ224_239_ENABLE_OFFSET)

#define NVIC_IRQ_CLEAR(n)             (CORTEXM3_NVIC_BASE + NVIC_IRQ_CLEAR_OFFSET(n))
#define NVIC_IRQ0_31_CLEAR            (CORTEXM3_NVIC_BASE + NVIC_IRQ0_31_CLEAR_OFFSET)
#define NVIC_IRQ32_63_CLEAR           (CORTEXM3_NVIC_BASE + NVIC_IRQ32_63_CLEAR_OFFSET)
#define NVIC_IRQ64_95_CLEAR           (CORTEXM3_NVIC_BASE + NVIC_IRQ64_95_CLEAR_OFFSET)
#define NVIC_IRQ96_127_CLEAR          (CORTEXM3_NVIC_BASE + NVIC_IRQ96_127_CLEAR_OFFSET)
#define NVIC_IRQ128_159_CLEAR         (CORTEXM3_NVIC_BASE + NVIC_IRQ128_159_CLEAR_OFFSET)
#define NVIC_IRQ160_191_CLEAR         (CORTEXM3_NVIC_BASE + NVIC_IRQ160_191_CLEAR_OFFSET)
#define NVIC_IRQ192_223_CLEAR         (CORTEXM3_NVIC_BASE + NVIC_IRQ192_223_CLEAR_OFFSET)
#define NVIC_IRQ224_239_CLEAR         (CORTEXM3_NVIC_BASE + NVIC_IRQ224_239_CLEAR_OFFSET)

#define NVIC_IRQ_PEND(n)              (CORTEXM3_NVIC_BASE + NVIC_IRQ_PEND_OFFSET(n))
#define NVIC_IRQ0_31_PEND             (CORTEXM3_NVIC_BASE + NVIC_IRQ0_31_PEND_OFFSET)
#define NVIC_IRQ32_63_PEND            (CORTEXM3_NVIC_BASE + NVIC_IRQ32_63_PEND_OFFSET)
#define NVIC_IRQ64_95_PEND            (CORTEXM3_NVIC_BASE + NVIC_IRQ64_95_PEND_OFFSET)
#define NVIC_IRQ96_127_PEND           (CORTEXM3_NVIC_BASE + NVIC_IRQ96_127_PEND_OFFSET)
#define NVIC_IRQ128_159_PEND          (CORTEXM3_NVIC_BASE + NVIC_IRQ128_159_PEND_OFFSET)
#define NVIC_IRQ160_191_PEND          (CORTEXM3_NVIC_BASE + NVIC_IRQ160_191_PEND_OFFSET)
#define NVIC_IRQ192_223_PEND          (CORTEXM3_NVIC_BASE + NVIC_IRQ192_223_PEND_OFFSET)
#define NVIC_IRQ224_239_PEND          (CORTEXM3_NVIC_BASE + NVIC_IRQ224_239_PEND_OFFSET)

#define NVIC_IRQ_CLRPEND(n)           (CORTEXM3_NVIC_BASE + NVIC_IRQ_CLRPEND_OFFSET(n))
#define NVIC_IRQ0_31_CLRPEND          (CORTEXM3_NVIC_BASE + NVIC_IRQ0_31_CLRPEND_OFFSET)
#define NVIC_IRQ32_63_CLRPEND         (CORTEXM3_NVIC_BASE + NVIC_IRQ32_63_CLRPEND_OFFSET)
#define NVIC_IRQ64_95_CLRPEND         (CORTEXM3_NVIC_BASE + NVIC_IRQ64_95_CLRPEND_OFFSET)
#define NVIC_IRQ96_127_CLRPEND        (CORTEXM3_NVIC_BASE + NVIC_IRQ96_127_CLRPEND_OFFSET)
#define NVIC_IRQ128_159_CLRPEND       (CORTEXM3_NVIC_BASE + NVIC_IRQ128_159_CLRPEND_OFFSET)
#define NVIC_IRQ160_191_CLRPEND       (CORTEXM3_NVIC_BASE + NVIC_IRQ160_191_CLRPEND_OFFSET)
#define NVIC_IRQ192_223_CLRPEND       (CORTEXM3_NVIC_BASE + NVIC_IRQ192_223_CLRPEND_OFFSET)
#define NVIC_IRQ224_239_CLRPEND       (CORTEXM3_NVIC_BASE + NVIC_IRQ224_239_CLRPEND_OFFSET)

#define NVIC_IRQ_ACTIVE(n)            (CORTEXM3_NVIC_BASE + NVIC_IRQ_ACTIVE_OFFSET(n))
#define NVIC_IRQ0_31_ACTIVE           (CORTEXM3_NVIC_BASE + NVIC_IRQ0_31_ACTIVE_OFFSET)
#define NVIC_IRQ32_63_ACTIVE          (CORTEXM3_NVIC_BASE + NVIC_IRQ32_63_ACTIVE_OFFSET)
#define NVIC_IRQ64_95_ACTIVE          (CORTEXM3_NVIC_BASE + NVIC_IRQ64_95_ACTIVE_OFFSET)
#define NVIC_IRQ96_127_ACTIVE         (CORTEXM3_NVIC_BASE + NVIC_IRQ96_127_ACTIVE_OFFSET)
#define NVIC_IRQ128_159_ACTIVE        (CORTEXM3_NVIC_BASE + NVIC_IRQ128_159_ACTIVE_OFFSET)
#define NVIC_IRQ160_191_ACTIVE        (CORTEXM3_NVIC_BASE + NVIC_IRQ160_191_ACTIVE_OFFSET)
#define NVIC_IRQ192_223_ACTIVE        (CORTEXM3_NVIC_BASE + NVIC_IRQ192_223_ACTIVE_OFFSET)
#define NVIC_IRQ224_239_ACTIVE        (CORTEXM3_NVIC_BASE + NVIC_IRQ224_239_ACTIVE_OFFSET)

#define NVIC_IRQ_PRIORITY(n)          (CORTEXM3_NVIC_BASE + NVIC_IRQ_PRIORITY_OFFSET(n))
#define NVIC_IRQ0_3_PRIORITY          (CORTEXM3_NVIC_BASE + NVIC_IRQ0_3_PRIORITY_OFFSET)
#define NVIC_IRQ4_7_PRIORITY          (CORTEXM3_NVIC_BASE + NVIC_IRQ4_7_PRIORITY_OFFSET)
#define NVIC_IRQ8_11_PRIORITY         (CORTEXM3_NVIC_BASE + NVIC_IRQ8_11_PRIORITY_OFFSET)
#define NVIC_IRQ12_15_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ12_15_PRIORITY_OFFSET)
#define NVIC_IRQ16_19_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ16_19_PRIORITY_OFFSET)
#define NVIC_IRQ20_23_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ20_23_PRIORITY_OFFSET)
#define NVIC_IRQ24_27_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ24_27_PRIORITY_OFFSET)
#define NVIC_IRQ28_31_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ28_31_PRIORITY_OFFSET)
#define NVIC_IRQ32_35_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ32_35_PRIORITY_OFFSET)
#define NVIC_IRQ36_39_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ36_39_PRIORITY_OFFSET)
#define NVIC_IRQ40_43_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ40_43_PRIORITY_OFFSET)
#define NVIC_IRQ44_47_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ44_47_PRIORITY_OFFSET)
#define NVIC_IRQ48_51_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ48_51_PRIORITY_OFFSET)
#define NVIC_IRQ52_55_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ52_55_PRIORITY_OFFSET)
#define NVIC_IRQ56_59_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ56_59_PRIORITY_OFFSET)
#define NVIC_IRQ60_63_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ60_63_PRIORITY_OFFSET)
#define NVIC_IRQ64_67_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ64_67_PRIORITY_OFFSET)
#define NVIC_IRQ68_71_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ68_71_PRIORITY_OFFSET)
#define NVIC_IRQ72_75_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ72_75_PRIORITY_OFFSET)
#define NVIC_IRQ76_79_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ76_79_PRIORITY_OFFSET)
#define NVIC_IRQ80_83_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ80_83_PRIORITY_OFFSET)
#define NVIC_IRQ84_87_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ84_87_PRIORITY_OFFSET)
#define NVIC_IRQ88_91_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ88_91_PRIORITY_OFFSET)
#define NVIC_IRQ92_95_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ92_95_PRIORITY_OFFSET)
#define NVIC_IRQ96_99_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_IRQ96_99_PRIORITY_OFFSET)
#define NVIC_IRQ100_103_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ100_103_PRIORITY_OFFSET)
#define NVIC_IRQ104_107_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ104_107_PRIORITY_OFFSET)
#define NVIC_IRQ108_111_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ108_111_PRIORITY_OFFSET)
#define NVIC_IRQ112_115_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ112_115_PRIORITY_OFFSET)
#define NVIC_IRQ116_119_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ116_119_PRIORITY_OFFSET)
#define NVIC_IRQ120_123_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ120_123_PRIORITY_OFFSET)
#define NVIC_IRQ124_127_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ124_127_PRIORITY_OFFSET)
#define NVIC_IRQ128_131_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ128_131_PRIORITY_OFFSET)
#define NVIC_IRQ132_135_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ132_135_PRIORITY_OFFSET)
#define NVIC_IRQ136_139_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ136_139_PRIORITY_OFFSET)
#define NVIC_IRQ140_143_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ140_143_PRIORITY_OFFSET)
#define NVIC_IRQ144_147_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ144_147_PRIORITY_OFFSET)
#define NVIC_IRQ148_151_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ148_151_PRIORITY_OFFSET)
#define NVIC_IRQ152_155_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ152_155_PRIORITY_OFFSET)
#define NVIC_IRQ156_159_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ156_159_PRIORITY_OFFSET)
#define NVIC_IRQ160_163_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ160_163_PRIORITY_OFFSET)
#define NVIC_IRQ164_167_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ164_167_PRIORITY_OFFSET)
#define NVIC_IRQ168_171_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ168_171_PRIORITY_OFFSET)
#define NVIC_IRQ172_175_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ172_175_PRIORITY_OFFSET)
#define NVIC_IRQ176_179_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ176_179_PRIORITY_OFFSET)
#define NVIC_IRQ180_183_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ180_183_PRIORITY_OFFSET)
#define NVIC_IRQ184_187_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ184_187_PRIORITY_OFFSET)
#define NVIC_IRQ188_191_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ188_191_PRIORITY_OFFSET)
#define NVIC_IRQ192_195_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ192_195_PRIORITY_OFFSET)
#define NVIC_IRQ196_199_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ196_199_PRIORITY_OFFSET)
#define NVIC_IRQ200_203_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ200_203_PRIORITY_OFFSET)
#define NVIC_IRQ204_207_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ204_207_PRIORITY_OFFSET)
#define NVIC_IRQ208_211_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ208_211_PRIORITY_OFFSET)
#define NVIC_IRQ212_215_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ212_215_PRIORITY_OFFSET)
#define NVIC_IRQ216_219_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ216_219_PRIORITY_OFFSET)
#define NVIC_IRQ220_223_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ220_223_PRIORITY_OFFSET)
#define NVIC_IRQ224_227_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ224_227_PRIORITY_OFFSET)
#define NVIC_IRQ228_231_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ228_231_PRIORITY_OFFSET)
#define NVIC_IRQ232_235_PRIORITY      (CORTEXM3_NVIC_BASE + NVIC_IRQ232_235_PRIORITY_OFFSET)

#define NVIC_CPUID_BASE               (CORTEXM3_NVIC_BASE + NVIC_CPUID_BASE_OFFSET)
#define NVIC_INTCTRL                  (CORTEXM3_NVIC_BASE + NVIC_INTCTRL_OFFSET)
#define NVIC_VECTAB                   (CORTEXM3_NVIC_BASE + NVIC_VECTAB_OFFSET)
#define NVIC_AIRC                     (CORTEXM3_NVIC_BASE + NVIC_AIRC_OFFSET)
#define NVIC_SYSCON                   (CORTEXM3_NVIC_BASE + NVIC_SYSCON_OFFSET)
#define NVIC_CFGCON                   (CORTEXM3_NVIC_BASE + NVIC_CFGCON_OFFSET)
#define NVIC_SYSH_PRIORITY(n)         (CORTEXM3_NVIC_BASE + NVIC_SYSH_PRIORITY_OFFSET(n))
#define NVIC_SYSH4_7_PRIORITY         (CORTEXM3_NVIC_BASE + NVIC_SYSH4_7_PRIORITY_OFFSET)
#define NVIC_SYSH8_11_PRIORITY        (CORTEXM3_NVIC_BASE + NVIC_SYSH8_11_PRIORITY_OFFSET)
#define NVIC_SYSH12_15_PRIORITY       (CORTEXM3_NVIC_BASE + NVIC_SYSH12_15_PRIORITY_OFFSET)
#define NVIC_SYSHCON                  (CORTEXM3_NVIC_BASE + NVIC_SYSHCON_OFFSET)
#define NVIC_CFAULTS                  (CORTEXM3_NVIC_BASE + NVIC_CFAULTS_OFFSET)
#define NVIC_HFAULTS                  (CORTEXM3_NVIC_BASE + NVIC_HFAULTS_OFFSET)
#define NVIC_DFAULTS                  (CORTEXM3_NVIC_BASE + NVIC_DFAULTS_OFFSET)
#define NVIC_MEMMANAGE_ADDR           (CORTEXM3_NVIC_BASE + NVIC_MEMMANAGE_ADDR_OFFSET)
#define NVIC_BFAULT_ADDR              (CORTEXM3_NVIC_BASE + NVIC_BFAULT_ADDR_OFFSET)
#define NVIC_AFAULTS                  (CORTEXM3_NVIC_BASE + NVIC_AFAULTS_OFFSET)
#define NVIC_PFR0                     (CORTEXM3_NVIC_BASE + NVIC_PFR0_OFFSET)
#define NVIC_PFR1                     (CORTEXM3_NVIC_BASE + NVIC_PFR1_OFFSET)
#define NVIC_DFR0                     (CORTEXM3_NVIC_BASE + NVIC_DFR0_OFFSET)
#define NVIC_AFR0                     (CORTEXM3_NVIC_BASE + NVIC_AFR0_OFFSET)
#define NVIC_MMFR0                    (CORTEXM3_NVIC_BASE + NVIC_MMFR0_OFFSET)
#define NVIC_MMFR1                    (CORTEXM3_NVIC_BASE + NVIC_MMFR1_OFFSET)
#define NVIC_MMFR2                    (CORTEXM3_NVIC_BASE + NVIC_MMFR2_OFFSET)
#define NVIC_MMFR3                    (CORTEXM3_NVIC_BASE + NVIC_MMFR3_OFFSET)
#define NVIC_ISAR0                    (CORTEXM3_NVIC_BASE + NVIC_ISAR0_OFFSET)
#define NVIC_ISAR1                    (CORTEXM3_NVIC_BASE + NVIC_ISAR1_OFFSET)
#define NVIC_ISAR2                    (CORTEXM3_NVIC_BASE + NVIC_ISAR2_OFFSET)
#define NVIC_ISAR3                    (CORTEXM3_NVIC_BASE + NVIC_ISAR3_OFFSET)
#define NVIC_ISAR4                    (CORTEXM3_NVIC_BASE + NVIC_ISAR4_OFFSET)
#define NVIC_STIR                     (CORTEXM3_NVIC_BASE + NVIC_STIR_OFFSET)
#define NVIC_PID4                     (CORTEXM3_NVIC_BASE + NVIC_PID4_OFFSET)
#define NVIC_PID5                     (CORTEXM3_NVIC_BASE + NVIC_PID5_OFFSET)
#define NVIC_PID6                     (CORTEXM3_NVIC_BASE + NVIC_PID6_OFFSET)
#define NVIC_PID7                     (CORTEXM3_NVIC_BASE + NVIC_PID7_OFFSET)
#define NVIC_PID0                     (CORTEXM3_NVIC_BASE + NVIC_PID0_OFFSET)
#define NVIC_PID1                     (CORTEXM3_NVIC_BASE + NVIC_PID1_OFFSET)
#define NVIC_PID2                     (CORTEXM3_NVIC_BASE + NVIC_PID2_OFFSET)
#define NVIC_PID3                     (CORTEXM3_NVIC_BASE + NVIC_PID3_OFFSET)
#define NVIC_CID0                     (CORTEXM3_NVIC_BASE + NVIC_CID0_OFFSET)
#define NVIC_CID1                     (CORTEXM3_NVIC_BASE + NVIC_CID1_OFFSET)
#define NVIC_CID2                     (CORTEXM3_NVIC_BASE + NVIC_CID2_OFFSET)
#define NVIC_CID3                     (CORTEXM3_NVIC_BASE + NVIC_CID3_OFFSET)

/* NVIC register bit definitions ****************************************************/

/* Interrrupt controller type (INCTCTL_TYPE) */

#define NVIC_INTCTRL_TYPE_INTLINESNUM_SHIFT 0    /* Bits 4-0: Number of interrupt intputs / 32 */
#define NVIC_INTCTRL_TYPE_INTLINESNUM_MASK  (0x1f << NVIC_INTCTRL_TYPE_INTLINESNUM_SHIFT)

/* SysTick control and status register (SYSTICK_CTRL) */

#define NVIC_SYSTICK_CTRL_ENABLE       (1 << 0)  /* Bit 0:  Enable */
#define NVIC_SYSTICK_CTRL_TICKINT      (1 << 1)  /* Bit 1:  Tick interrupt */
#define NVIC_SYSTICK_CTRL_CLKSOURCE    (1 << 2)  /* Bit 2:  Clock source */
#define NVIC_SYSTICK_CTRL_COUNTFLAG    (1 << 16) /* Bit 16: Count Flag */

/* SysTick reload value register (SYSTICK_RELOAD) */

#define NVIC_SYSTICK_RELOAD_SHIFT      0         /* Bits 23-0: Timer reload value */
#define NVIC_SYSTICK_RELOAD_MASK       (0x00ffffff << NVIC_SYSTICK_RELOAD_SHIFT)

/* SysTick current value registe (SYSTICK_CURRENT) */

#define NVIC_SYSTICK_CURRENT_SHIFT     0         /* Bits 23-0: Timer current value */
#define NVIC_SYSTICK_CURRENT_MASK      (0x00ffffff << NVIC_SYSTICK_RELOAD_SHIFT)

/* SysTick calibration value register (SYSTICK_CALIB) */

#define NVIC_SYSTICK_CALIB_TENMS_SHIFT 0        /* Bits 23-0: Calibration value */
#define NVIC_SYSTICK_CALIB_TENMS_MASK  (0x00ffffff << NVIC_SYSTICK_CALIB_TENMS_SHIFT)
#define NVIC_SYSTICK_CALIB_SKEW        (1 << 30) /* Bit 30: Calibration value inexact */
#define NVIC_SYSTICK_CALIB_NOREF       (1 << 31) /* Bit 31: No external reference clock */

/* Interrupt control state register (INTCTRL) */

#define NVIC_INTCTRL_NMIPENDSET        (1 << 31) /* Bit 31: Set pending NMI bit */
#define NVIC_INTCTRL_PENDSVSET         (1 << 28) /* Bit 28: Set pending PendSV bit */
#define NVIC_INTCTRL_PENDSVCLR         (1 << 27) /* Bit 27: Clear pending PendSV bit */
#define NVIC_INTCTRL_PENDSTSET         (1 << 26) /* Bit 26: Set pending SysTick bit */
#define NVIC_INTCTRL_PENDSTCLR         (1 << 25) /* Bit 25: Clear pending SysTick bit */
#define NVIC_INTCTRL_ISPREEMPOT        (1 << 23) /* Bit 23: Pending active next cycle */
#define NVIC_INTCTRL_ISRPENDING        (1 << 22) /* Bit 22: Interrupt pending flag */
#define NVIC_INTCTRL_VECTPENDING_SHIFT 12        /* Bits 21-12: Pending ISR number field */
#define NVIC_INTCTRL_VECTPENDING_MASK  (0x3ff << NVIC_INTCTRL_VECTPENDING_SHIFT)
#define NVIC_INTCTRL_RETTOBASE         (1 << 11) /* Bit 11: no other exceptions pending */
#define NVIC_INTCTRL_VECTACTIVE_SHIFT  0         /* Bits 8-0: Active ISR number */
#define NVIC_INTCTRL_VECTACTIVE_MASK   (0x1ff << NVIC_INTCTRL_VECTACTIVE_SHIFT)

/* System handler 4-7 priority register */

#define NVIC_SYSH_PRIORITY_PR4_SHIFT   0
#define NVIC_SYSH_PRIORITY_PR4_MASK    (0xff << NVIC_SYSH_PRIORITY_PR4_SHIFT)
#define NVIC_SYSH_PRIORITY_PR5_SHIFT   8
#define NVIC_SYSH_PRIORITY_PR5_MASK    (0xff << NVIC_SYSH_PRIORITY_PR5_SHIFT)
#define NVIC_SYSH_PRIORITY_PR6_SHIFT   16
#define NVIC_SYSH_PRIORITY_PR6_MASK    (0xff << NVIC_SYSH_PRIORITY_PR6_SHIFT)
#define NVIC_SYSH_PRIORITY_PR7_SHIFT   24
#define NVIC_SYSH_PRIORITY_PR7_MASK    (0xff << NVIC_SYSH_PRIORITY_PR7_SHIFT)

/* System handler 8-11 priority register */

#define NVIC_SYSH_PRIORITY_PR8_SHIFT 0
#define NVIC_SYSH_PRIORITY_PR8_MASK  (0xff << NVIC_SYSH_PRIORITY_PR8_SHIFT)
#define NVIC_SYSH_PRIORITY_PR9_SHIFT 8
#define NVIC_SYSH_PRIORITY_PR9_MASK  (0xff << NVIC_SYSH_PRIORITY_PR9_SHIFT)
#define NVIC_SYSH_PRIORITY_PR10_SHIFT 16
#define NVIC_SYSH_PRIORITY_PR10_MASK  (0xff << NVIC_SYSH_PRIORITY_PR10_SHIFT)
#define NVIC_SYSH_PRIORITY_PR11_SHIFT 24
#define NVIC_SYSH_PRIORITY_PR11_MASK  (0xff << NVIC_SYSH_PRIORITY_PR11_SHIFT)

/* System handler 12-15 priority register */

#define NVIC_SYSH_PRIORITY_PR12_SHIFT 0
#define NVIC_SYSH_PRIORITY_PR12_MASK  (0xff << NVIC_SYSH_PRIORITY_PR12_SHIFT)
#define NVIC_SYSH_PRIORITY_PR13_SHIFT 8
#define NVIC_SYSH_PRIORITY_PR13_MASK  (0xff << NVIC_SYSH_PRIORITY_PR13_SHIFT)
#define NVIC_SYSH_PRIORITY_PR14_SHIFT 16
#define NVIC_SYSH_PRIORITY_PR14_MASK  (0xff << NVIC_SYSH_PRIORITY_PR14_SHIFT)
#define NVIC_SYSH_PRIORITY_PR15_SHIFT 24
#define NVIC_SYSH_PRIORITY_PR15_MASK  (0xff << NVIC_SYSH_PRIORITY_PR15_SHIFT)

/* System handler control and state register (SYSHCON) */

#define NVIC_SYSHCON_MEMFAULTACT      (1 << 0)  /* Bit 0:  MemManage is active */
#define NVIC_SYSHCON_BUSFAULTACT      (1 << 1)  /* Bit 1:  BusFault is active */
#define NVIC_SYSHCON_USGFAULTACT      (1 << 3)  /* Bit 3:  UsageFault is active */
#define NVIC_SYSHCON_SVCALLACT        (1 << 7)  /* Bit 7:  SVCall is active */
#define NVIC_SYSHCON_MONITORACT       (1 << 8)  /* Bit 8:  Monitor is active */
#define NVIC_SYSHCON_PENDSVACT        (1 << 10) /* Bit 10: PendSV is active */
#define NVIC_SYSHCON_SYSTICKACT       (1 << 11) /* Bit 11: SysTick is active */
#define NVIC_SYSHCON_USGFAULTPENDED   (1 << 12) /* Bit 12: Usage fault is pended */
#define NVIC_SYSHCON_MEMFAULTPENDED   (1 << 13) /* Bit 13: MemManage is pended */
#define NVIC_SYSHCON_BUSFAULTPENDED   (1 << 14) /* Bit 14: BusFault is pended */
#define NVIC_SYSHCON_SVCALLPENDED     (1 << 15) /* Bit 15: SVCall is pended */
#define NVIC_SYSHCON_MEMFAULTENA      (1 << 16) /* Bit 16: MemFault enabled */
#define NVIC_SYSHCON_BUSFAULTENA      (1 << 17) /* Bit 17: BusFault enabled */
#define NVIC_SYSHCON_USGFAULTENA      (1 << 18) /* Bit 18: UsageFault enabled */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_COMMON_CORTEXM3_NVIC_H */
