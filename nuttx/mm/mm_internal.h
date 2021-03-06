/************************************************************************
 * mm/mm_internal.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

#ifndef __MM_MM_INTERNAL_H
#define __MM_MM_INTERNAL_H

/************************************************************************
 * Included Files
 ************************************************************************/

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/* These definitions define the characteristics of allocator
 *
 * MM_MIN_SHIFT is used to define MM_MIN_CHUNK.
 * MM_MIN_CHUNK - is the smallest physical chunk that can
 *   be allocated.  It must be at least a large as
 *   sizeof(struct mm_freenode_s).  Larger values may
 *   improve performance slightly, but will waste memory
 *   due to quantization losses.
 *
 * MM_MAX_SHIFT is used to define MM_MAX_CHUNK
 * MM_MAX_CHUNK is the largest, contiguous chunk of memory
 *   that can be allocated.  It can range from 16-bytes to
 *   4Gb.  Larger values of MM_MAX_SHIFT can cause larger
 *   data structure sizes and, perhaps, minor performance
 *   losses.
 */

#ifdef CONFIG_SMALL_MEMORY
# define MM_MIN_SHIFT      4  /* 16 bytes */
# define MM_MAX_SHIFT     15  /* 32 Kb */
#else
# define MM_MIN_SHIFT      4  /* 16 bytes */
# define MM_MAX_SHIFT     22  /*  4 Mb */
#endif

/* All other definitions derive from these two */

#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_MAX_CHUNK     (1 << MM_MAX_SHIFT)
#define MM_NNODES        (MM_MAX_SHIFT - MM_MIN_SHIFT + 1)

#define MM_GRAN_MASK     (MM_MIN_CHUNK-1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

/* An allocated chunk is distinguished from a free chunk by
 * bit 31 of the 'preceding' chunk size.  If set, then this is
 * an allocated chunk.
 */

#ifdef CONFIG_SMALL_MEMORY
# define MM_ALLOC_BIT    0x8000
#else
# define MM_ALLOC_BIT    0x80000000
#endif
#define MM_IS_ALLOCATED(n) \
  ((int)((struct mm_allocnode_s*)(n)->preceding) < 0))

/************************************************************************
 * Public Types
 ************************************************************************/

/* This describes an allocated chunk.  An allocated chunk is
 * distinguished from a free chunk by bit 31 of the 'precding'
 * chunk size.  If set, then this is an allocated chunk.
 */

struct mm_allocnode_s
{
  size_t size;           /* Size of this chunk */
  size_t preceding;      /* Size of the preceding chunk */
};

#ifdef CONFIG_SMALL_MEMORY
# define SIZEOF_MM_ALLOCNODE   4
#else
# define SIZEOF_MM_ALLOCNODE   8
#endif

#define CHECK_ALLOCNODE_SIZE \
  DEBUGASSERT(sizeof(struct mm_allocnode_s) == SIZEOF_MM_ALLOCNODE)

/* This describes a free chunk */

struct mm_freenode_s
{
  size_t size;                     /* Size of this chunk */
  size_t preceding;                /* Size of the preceding chunk */
  FAR struct mm_freenode_s *flink; /* Supports a doubly linked list */
  FAR struct mm_freenode_s *blink;
};

#ifdef CONFIG_SMALL_MEMORY
# define SIZEOF_MM_FREENODE     8
#else
# define SIZEOF_MM_FREENODE    16
#endif
#define CHECK_FREENODE_SIZE    \
  DEBUGASSERT(sizeof(struct mm_freenode_s) == SIZEOF_MM_FREENODE)

/* Normally defined in stdlib.h */

#ifdef MM_TEST
struct mallinfo
{
  int arena;    /* This is the total size of memory allocated
                 * for use by malloc in bytes. */
  int ordblks;  /* This is the number of free (not in use) chunks */
  int mxordblk; /* Size of the largest free (not in use) chunk */
  int uordblks; /* This is the total size of memory occupied by
                 * chunks handed out by malloc. */
  int fordblks; /* This is the total size of memory occupied
                 * by free (not in use) chunks.*/
};
#endif

/************************************************************************
 * Global Variables
 ************************************************************************/

/* This is the size of the heap provided to mm */

extern size_t  g_heapsize;

/* This is the first and last nodes of the heap */

extern FAR struct mm_allocnode_s *g_heapstart[CONFIG_MM_REGIONS];
extern FAR struct mm_allocnode_s *g_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
extern int g_nregions;
#else
# define g_nregions 1
#endif

/* All free nodes are maintained in a doubly linked list.  This
 * array provides some hooks into the list at various points to
 * speed searches for free nodes.
 */

extern FAR struct mm_freenode_s g_nodelist[MM_NNODES];

/************************************************************************
 * Public Function Prototypes
 ************************************************************************/

/* Normally defined in malloc.h */

#ifdef MM_TEST
 extern FAR void *mm_malloc(size_t);
 extern void      mm_free(void*);
 extern FAR void *mm_realloc(void*, size_t);
 extern FAR void *mm_memalign(size_t, size_t);
 extern FAR void *mm_zalloc(size_t);
 extern FAR void *mm_calloc(size_t, size_t);
#ifdef CONFIG_CAN_PASS_STRUCTS
 extern struct mallinfo mallinfo(void);
#else
 extern int       mallinfo(struct mallinfo *info);
#endif
#endif

extern void       mm_shrinkchunk(FAR struct mm_allocnode_s *node,
                                 size_t size);
extern void       mm_addfreechunk(FAR struct mm_freenode_s *node);
extern int        mm_size2ndx(size_t size);
extern void       mm_seminitialize(void);
extern void       mm_takesemaphore(void);
extern void       mm_givesemaphore(void);
#ifdef MM_TEST
 extern int       mm_getsemaphore(void);
#endif

#endif /* __MM_MM_INTERNAL_H */
