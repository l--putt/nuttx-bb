/****************************************************************************
 * examples/ostest/ostest.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __OSTEST_H
#define __OSTEST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The task_create task size can be specified in the defconfig file */

#ifdef CONFIG_EXAMPLES_OSTEST_STACKSIZE
#  define STACKSIZE CONFIG_EXAMPLES_OSTEST_STACKSIZE
#else
#  define STACKSIZE 8192
#endif

/* The number of times to execute the test can be specified in the defconfig
 * file.
 */

#ifndef CONFIG_EXAMPLES_OSTEST_LOOPS
#  define CONFIG_EXAMPLES_OSTEST_LOOPS 1
#endif

/* This is the number of threads that are created in the barrier test.
 * A smaller number should be selected on systems without sufficient memory
 * to start so many threads.
 */

#ifndef CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS
#  define CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS 8
#endif

/* Priority inheritance */

#if defined(CONFIG_DEBUG) && defined(CONFIG_PRIORITY_INHERITANCE) && defined(CONFIG_SEM_PHDEBUG)
#  define dump_nfreeholders(s) printf(s " nfreeholders: %d\n", sem_nfreeholders())
#else
#  define dump_nfreeholders(s)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* dev_null.c ***************************************************************/

extern int dev_null(void);

/* mutex.c ******************************************************************/

extern void mutex_test(void);

/* rmutex.c ******************************************************************/

extern void recursive_mutex_test(void);

/* sem.c ********************************************************************/

extern void sem_test(void);

/* cond.c *******************************************************************/

extern void cond_test(void);

/* mqueue.c *****************************************************************/

extern void mqueue_test(void);

/* timedmqueue.c ************************************************************/

extern void timedmqueue_test(void);

/* cancel.c *****************************************************************/

extern void cancel_test(void);

/* timedwait.c **************************************************************/

extern void timedwait_test(void);

/* sighand.c ****************************************************************/

extern void sighand_test(void);

/* posixtimers.c ************************************************************/

extern void timer_test(void);

/* roundrobin.c *************************************************************/

extern void rr_test(void);

/* barrier.c ****************************************************************/

extern void barrier_test(void);

/* prioinherit.c ************************************************************/

extern void priority_inheritance(void);

/* APIs exported (conditionally) by the OS specifically for testing of
 * priority inheritance
 */

#if defined(CONFIG_DEBUG) && defined(CONFIG_PRIORITY_INHERITANCE) && defined(CONFIG_SEM_PHDEBUG)
extern void sem_enumholders(FAR sem_t *sem);
extern int sem_nfreeholders(void);
#else
#  define sem_enumholders(sem)
#  define sem_nfreeholders()
#endif

#endif /* __OSTEST_H */
