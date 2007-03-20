/********************************************************************************
 * fcntl.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************************/

#ifndef __FCNTL_H
#define __FCNTL_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* open flag settings for open() (and related APIs) */

#define O_RDONLY    0x01     /* Open for read access */
#define O_WRONLY    0x02     /* Open for write access */
#define O_RDWR      0x03     /* Open for both read & write access */
#define O_CREAT     0x04     /* Create file/sem/mq object */
#define O_EXCL      0x08     /* Name must not exist when opened  */
#define O_APPEND    0x10     /* Keep contents, append to end */
#define O_TRUNC     0x20     /* Delete contents */
#define O_NONBLOCK  0x40     /* Don't wait for data */
#define O_SYNC      0x80     /* Synchronize output on write */
#define O_DSYNC     OSYNC

#define O_RSYNC     0x00     /* Sychronize input on read */
#define O_ACCMODE   0x00     /* Required by POSIX */
#define O_NOCTTY    0x00     /* Reqired by POSIX */

#define O_RDOK      O_RDONLY /* Not POSIX */
#define O_WROK      O_WRONLY /* Not POSIX */

/* fcntl() commands */

#define F_DUPFD     0  /* Duplicate a file descriptor */
#define F_GETFD     1  /* Read the file descriptor flags */
#define F_GETFL     2  /* Read the file status flags */
#define F_GETLEASE  3  /* Indicas what type of lease is held on fd (linux) */
#define F_GETLK     4  /* Check if we could place a lock */
#define F_GETOWN    5  /* Get the pid receiving  SIGIO and SIGURG signals for fd */
#define F_GETSIG    6  /* Get the signal sent */
#define F_NOTIFY    7  /* Provide notification when directory referred to by fd changes (linux)*/
#define F_SETFD     8  /* Set the file descriptor flags to value */
#define F_SETFL     9  /* Set  the  file status flags to the value */
#define F_SETLEASE  10 /* Set or remove file lease (linux) */
#define F_SETLK     11 /* Acquire or release a lock on range of bytes */
#define F_SETLKW    12 /* Like F_SETLK, but wait for lock to become available */
#define F_SETOWN    13 /* Set pid that will receive SIGIO and SIGURG signals for fd */
#define F_SETSIG    14 /* Set the signal to be sent */

/* close-on-exec flag for F_GETRL and F_SETFL */

#define FD_CLOEXEC  1

/* Arguments to F_SETLEASE */

#define F_RDLCK     0  /* Take out a read lease */
#define F_WRLCK     2  /* Take out a write lease */
#define F_UNLCK     3  /* Remove a lease */

/* These are the notifications that can be received from F_NOTIFY (linux) */

#define DN_ACCESS   0  /* A file was accessed */
#define DN_MODIFY   1  /* A file was modified */
#define DN_CREATE   2  /* A file was created */
#define DN_DELETE   3  /* A file was unlinked */
#define DN_RENAME   4  /* A file was renamed */
#define DN_ATTRIB   5  /* Attributes of a file were changed */

#define S_IFMT      0170000
#define S_IFSOCK    0140000
#define S_IFLNK     0120000
#define S_IFREG     0100000
#define S_IFBLK     0060000
#define S_IFCHR     0020000
#define S_IFDIR     0040000
#define S_IFIFO     0010000
#define S_ISUID     0004000
#define S_ISGID     0002000
#define S_ISVTX     0001000
#define S_IRWXU     0000700
#define S_IRUSR     0000400
#define S_IWUSR     0000200
#define S_IXUSR     0000100
#define S_IRWXG     0000070
#define S_IRGRP     0000040
#define S_IWGRP     0000020
#define S_IXGRP     0000010
#define S_IRWXO     0000007
#define S_IROTH     0000004
#define S_IWOTH     0000002
#define S_IXOTH     0000001

#define S_ISLNK(m)  (((m) & S_IFMT) == S_IFLNK)
#define S_ISREG(m)  (((m) & S_IFMT) == S_IFREG)
#define S_ISDIR(m)  (((m) & S_IFMT) == S_IFDIR)
#define S_ISCHR(m)  (((m) & S_IFMT) == S_IFCHR)
#define S_ISBLK(m)  (((m) & S_IFMT) == S_IFBLK)
#define S_ISFIFO(m) (((m) & S_IFMT) == S_IFIFO)
#define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

/* struct flock is the third argument for F_GETLK, F_SETLK and F_SETLKW */

struct flock
{
  short l_type;    /* Type of lock: F_RDLCK, F_WRLCK, F_UNLCK */
  short l_whence;  /* How to interpret l_start: SEEK_SET, SEEK_CUR, SEEK_END */
  off_t l_start;   /* Starting offset for lock */
  off_t l_len;     /* Number of bytes to lock */
  pid_t l_pid;     /* PID of process blocking our lock (F_GETLK only) */
};

/********************************************************************************
 * Public Variables
 ********************************************************************************/

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* POSIX-like File System Interfaces */

EXTERN int creat(const char *path, mode_t mode);
EXTERN int open(const char *path, int oflag, ...);
EXTERN int fcntl(int fd, int cmd, ...);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FCNTL_H */
