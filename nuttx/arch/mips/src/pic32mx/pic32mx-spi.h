/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-spi.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_SPI_CON_OFFSET     0x0000 /* SPI control register */
#define PIC32MX_SPI_CONCLR_OFFSET  0x0004 /* SPI control clear register */
#define PIC32MX_SPI_CONSET_OFFSET  0x0008 /* SPI control set register */
#define PIC32MX_SPI_CONINV_OFFSET  0x000c /* SPI control invert register */
#define PIC32MX_SPI_STAT_OFFSET    0x0010 /* SPI status register */
#define PIC32MX_SPI_STATSET_OFFSET 0x0018 /* SPI status set register */
#define PIC32MX_SPI_BUF_OFFSET     0x0020 /* SPI buffer register */
#define PIC32MX_SPI_BRG_OFFSET     0x0030 /* SPI baud rate register */
#define PIC32MX_SPI_BRGCLR_OFFSET  0x0034 /* SPI baud rate clear register */
#define PIC32MX_SPI_BRGSET_OFFSET  0x0038 /* SPI baud rate set register */
#define PIC32MX_SPI_BRGINV_OFFSET  0x003c /* SPI baud rate invert register */

/* Register Addresses *******************************************************/

#define PIC32MX_SPI1_CON           (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CON_OFFSET)
#define PIC32MX_SPI1_CONCLR        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#define PIC32MX_SPI1_CONSET        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#define PIC32MX_SPI1_CONINV        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#define PIC32MX_SPI1_STAT          (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#define PIC32MX_SPI1_STATSET       (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_STATSET_OFFSET)
#define PIC32MX_SPI1_BUF           (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#define PIC32MX_SPI1_BRG           (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#define PIC32MX_SPI1_BRGCLR        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#define PIC32MX_SPI1_BRGSET        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#define PIC32MX_SPI1_BRGINV        (PIC32MX_SPI1_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)

#define PIC32MX_SPI2_CON           (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CON_OFFSET)
#define PIC32MX_SPI2_CONCLR        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONCLR_OFFSET)
#define PIC32MX_SPI2_CONSET        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONSET_OFFSET)
#define PIC32MX_SPI2_CONINV        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_CONINV_OFFSET)
#define PIC32MX_SPI2_STAT          (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_STAT_OFFSET)
#define PIC32MX_SPI2_STATSET       (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_STATSET_OFFSET)
#define PIC32MX_SPI2_BUF           (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BUF_OFFSET)
#define PIC32MX_SPI2_BRG           (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRG_OFFSET)
#define PIC32MX_SPI2_BRGCLR        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGCLR_OFFSET)
#define PIC32MX_SPI2_BRGSET        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGSET_OFFSET)
#define PIC32MX_SPI2_BRGINV        (PIC32MX_SPI2_K1BASE+PIC32MX_SPI_BRGINV_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* SPI control register */

#define SPI_CON_MSTEN              (1 << 5)  /* Bits 5: Master mode enable */
#define SPI_CON_CKP                (1 << 6)  /* Bits 6: Clock polarity select */
#define SPI_CON_SSEN               (1 << 7)  /* Bits 7: Slave select enable (slave mode) */
#define SPI_CON_CKE                (1 << 8)  /* Bits 8: SPI clock edge select */
#define SPI_CON_SMP                (1 << 9)  /* Bits 9: SPI data input sample phase */
#define SPI_CON_MODE_SHIFT         (10)      /* Bits 10-11: 32/16-Bit Communication Select */
#define SPI_CON_MODE_MASK          (3 << SPI_CON_MODE_SHIFT)
#define SPI_CON_MODE32             (1 << 11) /* Bits 11: xx */
#  define SPI_CON_MODE_8BIT        (0 << SPI_CON_MODE_SHIFT) /* 8-bit data width */
#  define SPI_CON_MODE_16BIT       (1 << SPI_CON_MODE_SHIFT) /* 16-bit data width */
#  define SPI_CON_MODE_32BIT       (2 << SPI_CON_MODE_SHIFT) /* 2-bit data width */
#define SPI_CON_DISSDO             (1 << 12) /* Bits 12: Disable SDOx pin */
#define SPI_CON_SIDL               (1 << 13) /* Bits 13: Stop in idle mode */
#define SPI_CON_FRZ                (1 << 14) /* Bits 14: Freeze in debug exception */
#define SPI_CON_ON                 (1 << 15) /* Bits 15: SPI peripheral on */
#define SPI_CON_ENHBUF             (1 << 16) /* Bits 16: Enhanced buffer enable */
#define SPI_CON_SPIFE              (1 << 17) /* Bits 17: Frame sync pulse edge select */
#define SPI_CON_FRMCNT_SHIFT       (24)      /* Bits 24-26: Frame Sync Pulse Counter bits */
#define SPI_CON_FRMCNT_MASK        (7 << SPI_CON_FRMCNT_SHIFT)
#  define SPI_CON_FRMCNT_CHAR1     (0 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse each char */
#  define SPI_CON_FRMCNT_CHAR2     (1 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 2 chars */
#  define SPI_CON_FRMCNT_CHAR4     (2 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 4 chars */
#  define SPI_CON_FRMCNT_CHAR8     (3 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 8 chars */
#  define SPI_CON_FRMCNT_CHAR16    (4 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 16 chars */
#  define SPI_CON_FRMCNT_CHAR32    (5 << SPI_CON_FRMCNT_SHIFT) /* Frame sync pulse every 32 chars */
#define SPI_CON_FRMSYPW            (1 << 27) /* Bits 27: Frame sync pulse width */
#define SPI_CON_MSSEN              (1 << 28) /* Bits 28: Master mode slave select enable */
#define SPI_CON_FRMPOL             (1 << 29) /* Bits 29: Frame sync polarity */
#define SPI_CON_FRMSYNC            (1 << 30) /* Bits 30: Frame sync pulse direction control on SSx pin */
#define SPI_CON_FRMEN              (1 << 31) /* Bits 31: Framed SPI support */

/* SPI status register */

#define SPI_STAT_SPIRBF            (1 << 0)  /* Bits 0: SPI receive buffer full status */
#define SPI_STAT_SPITBF            (1 << 1)  /* Bits 1: SPI transmit buffer full status */
#define SPI_STAT_SPITBE            (1 << 3)  /* Bits 3: SPI transmit buffer empty status */
#define SPI_STAT_SPIRBE            (1 << 5)  /* Bits 5: RX FIFO Empty */
#define SPI_STAT_SPIROV            (1 << 6)  /* Bits 6: Receive overflow flag */
#define SPI_STAT_SRMT              (1 << 7)  /* Bits 6: Shift Register Empty */
#define SPI_STAT_SPITUR            (1 << 6)  /* Bits 8: Transmit under run */
#define SPI_STAT_SPIBUSY           (1 << 11) /* Bits 11: SPI activity status */
#define SPI_STAT_TXBUFELM_SHIFT    (16)      /* Bits 16-20: Transmit Buffer Element Count bits */
#define SPI_STAT_TXBUFELM_MASK     (31 << SPI_STAT_TXBUFELM_SHIFT)
#define SPI_STAT_RXBUFELM_SHIFT    (24)      /* Bits 24-28: Receive Buffer Element Count bits */
#define SPI_STAT_RXBUFELM_MASK     (31 << SPI_STAT_RXBUFELM_SHIFT)

/* SPI buffer register (May be 31-bits wide on some parts) */

#define SPI_BUF_MASK               0x1ff

/* SPI baud rate register (This register holds 32-bits of data with other
 * bit-fields
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_SPI_H */
