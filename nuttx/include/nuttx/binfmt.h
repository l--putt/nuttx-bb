/****************************************************************************
 * include/nuttx/binfmt.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_BINFMT_H
#define __INCLUDE_NUTTX_BINFMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nxflat.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the file to be loaded */

struct binary_s
{
  /* Provided to the loader */

  FAR const char  *filename;         /* Full path to the binary */
  FAR const char **argv;             /* Argument list */

  /* Provided by the loader (if successful) */

  main_t       entrypt;              /* Entry point into a program module */
  FAR void    *ispace;               /* Memory-mapped, I-space (.text) address */
  FAR struct dspace_s *dspace;       /* Address of the allocated .data/.bss space */
  size_t       isize;                /* Size of the I-space region (needed for munmap) */
  size_t       stacksize;            /* Size of the stack in bytes (unallocated) */
};

/* This describes one binary format handler */

struct binfmt_s
{
  FAR struct binfmt_s *next;             /* Supports a singly-linked list */
  int (*load)(FAR struct binary_s *bin); /* Verify and load binary into memory */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: register_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int register_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: unregister_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int unregister_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: load_module
 *
 * Description:
 *   Load a module into memory and prep it for execution.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns 0 (OK) on success.  On failure, it returns -1 (ERROR) with
 *   errno set appropriately.
 *
 ****************************************************************************/

EXTERN int load_module(const char *filename, FAR struct binary_s *bin);

/****************************************************************************
 * Name: unload_module
 *
 * Description:
 *   Unload a (non-executing) module from memory.  If the module has
 *   been started (via exec_module), calling this will be fatal.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int unload_module(FAR const struct binary_s *bin);

/****************************************************************************
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns the PID of the exec'ed module.  On failure, it.returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

EXTERN int exec_module(FAR const struct binary_s *bin, int priority);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_H */
