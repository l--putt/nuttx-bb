/****************************************************************************
 * arch/arm/src/calypso/calypso_serial.c
 *
 * (C) 2011 Stefan Richter <ichgeh@l--putt.de>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/fs.h>
#include <nuttx/serial.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "uart.h"
#include <sercomm/sercomm.h>

/* stubs to make serial driver happy */
void sercomm_recvchars(void *a) { }
void sercomm_xmitchars(void *a) { }

/* Stubs to make memory allocator happy */
void cons_puts(void *foo){}
void delay_ms(int ms){}

/************************************************************************************
 * Fileops Prototypes and Structures
 ************************************************************************************/

typedef FAR struct file		file_t;

static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen);
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen);
static int     sc_console_ioctl(file_t *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     sc_console_poll(file_t *filep, FAR struct pollfd *fds, bool setup);
#endif

static const struct file_operations g_sercom_console_ops =
{
	0,			/* open, always opened */
	0,			/* close, stays open */
	sc_console_read,	/* read */
	sc_console_write,	/* write */
	0,			/* seek, not supported */
	sc_console_ioctl,	/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	sc_console_poll		/* poll */
#endif
};

/****************************************************************************
 * Helper functions
 ****************************************************************************/
static FAR uart_dev_t *readdev = NULL;
static struct msgb *recvmsg = NULL;
static void recv_cb(uint8_t dlci, struct msgb *msg)
{
	sem_post(&readdev->recvsem);
	recvmsg = msg;
}

/****************************************************************************
 * Fileops
 ****************************************************************************/

/* XXX: recvmsg is overwritten when multiple msg arrive! */
static ssize_t sc_console_read(file_t *filep, FAR char *buffer, size_t buflen)
{
	size_t len;
	struct msgb *tmp;

	/* Wait until data is received */
	while(recvmsg == NULL) {
		sem_wait(&readdev->recvsem);
	}

	len = recvmsg->len > buflen ? buflen : recvmsg->len;
	memcpy(buffer, msgb_get(recvmsg, len), len);

	if(recvmsg->len == 0) {
		/* prevent inconsistent msg by first invalidating it, then free it */
		tmp = recvmsg;
		recvmsg = NULL;
		msgb_free(tmp);
	}

	return len;
}

/* XXX: redirect to old Osmocom-BB comm/sercomm_cons.c -> 2 buffers */
extern int sercomm_write(void *file, const char *s, const int len);
static ssize_t sc_console_write(file_t *filep, FAR const char *buffer, size_t buflen)
{
	int ret = sercomm_write(filep, buffer, buflen);
	if(ret < 0)
  		return ret;
	else
  		return buflen;
}

/* Forward ioctl to uart driver */
static int sc_console_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR uart_dev_t   *dev   = inode->i_private;

	return dev->ops->ioctl(filep, cmd, arg);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Use sercomm on uart driver, register console driver */
int sercomm_register(FAR const char *path, FAR uart_dev_t *dev)
{
	/* XXX: initialize MODEMUART to be used for sercomm*/
	uart_init(SERCOMM_UART_NR, 1);
	uart_baudrate(SERCOMM_UART_NR, UART_115200);
	readdev = dev;
	sercomm_register_rx_cb(SC_DLCI_LOADER, &recv_cb);
	
	sem_init(&dev->xmit.sem, 0, 1);
	sem_init(&dev->recv.sem, 0, 1);
	sem_init(&dev->closesem, 0, 1);
	sem_init(&dev->xmitsem,  0, 0);
	sem_init(&dev->recvsem,  0, 0);
#ifndef CONFIG_DISABLE_POLL
	sem_init(&dev->pollsem,  0, 1);
#endif

	dbg("Registering %s\n", path);
	return register_driver(path, &g_sercom_console_ops, 0666, NULL);
}
