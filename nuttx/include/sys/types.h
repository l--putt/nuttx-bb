/****************************************************************************
 * sys/types.h
 *
 *   Copyright (C) 2007, 2008, 2009 Gregory Nutt. All rights reserved.
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

#ifndef __SYS_TYPES_H
#define __SYS_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Values for type boolean */

#define TRUE 1
#define FALSE 0

/* NULL is usually defined in stddef.h (which includes this file) */

#ifndef NULL
    /* SDCC is sensitive to NULL pointer type conversions */
#  ifdef SDCC
#    define NULL (0)
#  else
#    define NULL ((void*)0)
#  endif
#endif

/* POSIX-like OS return values: */

#if !defined(__cplusplus)
#  undef  ERROR
#  define ERROR -1
#endif

#undef  OK
#define OK 0

/* HPUX-like MIN/MAX value */

#define PRIOR_RR_MIN      0
#define PRIOR_RR_MAX    255
#define PRIOR_FIFO_MIN    0
#define PRIOR_FIFO_MAX  255
#define PRIOR_OTHER_MIN   0
#define PRIOR_OTHER_MAX 255

/* Scheduling Priorities.  NOTE:  Only the idle task can take
 * the TRUE minimum priority. */

#define SCHED_PRIORITY_MAX     255
#define SCHED_PRIORITY_DEFAULT 100
#define SCHED_PRIORITY_MIN       1
#define SCHED_PRIORITY_IDLE      0

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef float  float32;
#ifndef CONFIG_HAVE_DOUBLE
typedef float  double_t;
typedef float  float64;
#else
typedef double double_t;
typedef double float64;
#endif

/* Misc. scalar types */

/* mode_t is an integer type used for file attributes.  mode_t needs
 * to be at least 16-bits but, in fact must be sizeof(int) because it is
 * pased via varargs.
 */

typedef unsigned int mode_t;

/* size_t is used for sizes of objects.
 * ssize_t is used for a count of bytes or an error indication.
 */

#ifdef CONFIG_SMALL_MEMORY
typedef uint16       size_t;
typedef sint16       ssize_t;
#else
typedef uint32       size_t;
typedef sint32       ssize_t;
#endif

/* uid_t is used for user IDs
 * gid_t is used for group IDs.
 */

typedef sint16       uid_t;
typedef sint16       gid_t;

/* dev_t is used for device IDs */

typedef uint16       dev_t;

/* ino_t is used for file serial numbers */

typedef uint16       ino_t;

/* pid_t is used for process IDs and process group IDs */

typedef int          pid_t;

/* blkcnt_t and off_t are signed integer types.
 *
 *   blkcnt_t is used for file block counts.
 *   off_t is used for file sizes.
 *
 * Hence, both should be independent of processor architecture.
 */

typedef uint32       blkcnt_t;
typedef sint32       off_t;
typedef off_t        fpos_t;

/* blksize_t is a signed integer value used for file block sizes */

typedef sint16       blksize_t;

/* Network related */

typedef unsigned int socklen_t;
typedef uint16       sa_family_t;

/* The type useconds_t shall be an unsigned integer type capable of storing
 * values at least in the range [0, 1000000]. The type suseconds_t shall be
 * a signed integer type capable of storing values at least in the range
 * [-1, 1000000].
 */

typedef uint32       useconds_t;
typedef sint32       suseconds_t;

/* Task entry point */

typedef int (*main_t)(int argc, char *argv[]);

#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#endif /* __SYS_TYPES_H */
