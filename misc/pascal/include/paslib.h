/***************************************************************************
 * include/paslib.h
 * External Declarations associated with paslib
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 ***************************************************************************/

#ifndef __PASLIB_H
#define __PASLIB_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "keywords.h"
#include "pdefs.h"
#include "pofflib.h"

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* POFF file is always big-endian */

#ifdef CONFIG_BIG_ENDIAN
# undef  CONFIG_POFF_SWAPNEEDED
# define poff16(val) (val)
# define poff32(val) (val)
#else
# define CONFIG_POFF_SWAPNEEDED 1
# define poff16(val) poffSwap16(val)
# define poff32(val) poffSwap32(val)
#endif

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* File name extension helper */

extern boolean extension(const char *inName, const char *ext, char *outName,
			 boolean force_default);

/* Math helpers */

extern sint32  signExtend16(uint16 arg16);
extern sint32  signExtend25(uint32 arg25);

/* Endian-ness helpers */

extern uint16 poffSwap16(uint16 val);
extern uint32 poffSwap32(uint32 val);

/***************************************************************************
 * Global Variables
 ***************************************************************************/

#endif /* __PASLIB_H */

