/****************************************************************************
 * examples/usbstorage/usbstrg_lpc214x.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Configure and register the LPC214x MMC/SD SPI block driver.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/mmcsd.h>

#include "usbstrg.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_MCU123
#  undef LPC214X_MMCSDSPIPORTNO
#  define LPC214X_MMCSDSPIPORTNO 1
#  undef LPC214X_MMCSDSLOTNO
#  define LPC214X_MMCSDSLOTNO 0
#else
   /* Add configuration for new LPC214x boards here */
#  error "Unrecognized LPC214x board"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbstrg_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int usbstrg_archinitialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  message("usbstrg_archinitialize: Initializing SPI port %d\n",
          LPC214X_MMCSDSPIPORTNO);

  spi = up_spiinitialize(1);
  if (!spi)
    {
      message("usbstrg_archinitialize: Failed to initialize SPI port %d\n",
              LPC214X_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  message("usbstrg_archinitialize: Successfully initialized SPI port %d\n",
          LPC214X_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  message("usbstrg_archinitialize: Binding SPI port %d to MMC/SD slot %d\n",
          LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_EXAMPLES_USBSTRG_DEVMINOR1, LPC214X_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      message("usbstrg_archinitialize: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
              LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO, ret);
      return ret;
    }

  message("usbstrg_archinitialize: Successfuly bound SPI port %d to MMC/SD slot %d\n",
          LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);
  return OK;
}