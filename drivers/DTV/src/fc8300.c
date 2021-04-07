/*****************************************************************************
*	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/device.h>

#include "fc8300.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8300_regs.h"
#include "fc8300_isr.h"
#include "fci_hal.h"

struct ISDBT_INIT_INFO_T *hInit;

struct kobject *dtv_8300_kobj;
extern s32 rssi_value;

#define RING_BUFFER_SIZE	(188 * 320 * 32)

/* GPIO(RESET & INTRRUPT) Setting */
#define FC8300_NAME		"isdbt"

#ifndef CONFIG_OF
#ifdef EVB
#define GPIO_ISDBT_IRQ IRQ_EINT(2)
#define GPIO_ISDBT_PWR_EN EXYNOS4_GPK1(2)
#define GPIO_ISDBT_RST EXYNOS4_GPL2(6)
#else
#define GPIO_ISDBT_IRQ 0x24
#define GPIO_ISDBT_PWR_EN 1
#define GPIO_ISDBT_RST 2
#endif

#else
static int irq_gpio;
static int enable_gpio;
static int reset_gpio;
static int power_en_gpio;
#define GPIO_ISDBT_IRQ		irq_gpio
#define GPIO_ISDBT_PWR_EN	enable_gpio
#define GPIO_ISDBT_RST		reset_gpio
#define GPIO_SYS_PWR_EN     power_en_gpio
#endif
#define FC8300_CHIP_ID 0x8300
#define FC8300_CHIP_ID_REG 0x26

static DEFINE_MUTEX(ringbuffer_lock);
static DEFINE_MUTEX(driver_mode_lock);

static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);

struct ISDBT_OPEN_INFO_T hOpen_Val;
u8 static_ringbuffer[RING_BUFFER_SIZE];
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
u32 bbm_xtal_freq;
#ifndef BBM_I2C_TSIF
static u8 isdbt_isr_sig;

static irqreturn_t isdbt_threaded_irq(int irq, void *dev_id)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)dev_id;

	mutex_lock(&driver_mode_lock);
	isdbt_isr_sig = 1;
	if (driver_mode == ISDBT_POWERON)
		bbm_com_isr(hInit);
	isdbt_isr_sig = 0;
	mutex_unlock(&driver_mode_lock);

	return IRQ_HANDLED;
}

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}
#endif
int isdbt_hw_setting(HANDLE hDevice)
{
	int err;
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	print_log(0, "isdbt_hw_setting\n");

	err = gpio_request(GPIO_SYS_PWR_EN, "dtv_ldo_en_1");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request dtv_ldo_en_1\n");
		goto gpio_ldo_en_1;
	}
	gpio_direction_output(GPIO_SYS_PWR_EN, 0);
	err = gpio_export(GPIO_SYS_PWR_EN, 0);
	if (err)
		print_log(0, "%s: error %d gpio_export for %d\n",
			__func__, err, GPIO_SYS_PWR_EN);
	else {
		err = gpio_export_link(fc8300_misc_device.this_device,
			"dtv_ldo_en_1", GPIO_SYS_PWR_EN);
		if (err)
			print_log(0, "%s: error %d gpio_export for %d\n",
				__func__, err, GPIO_SYS_PWR_EN);
	}

	err = gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);
	err = gpio_export(GPIO_ISDBT_PWR_EN, 0);
	if (err)
		print_log(0, "%s: error %d gpio_export for %d\n",
			__func__, err, GPIO_ISDBT_PWR_EN);
	else {
		err = gpio_export_link(fc8300_misc_device.this_device,
			"isdbt_en", GPIO_ISDBT_PWR_EN);
		if (err)
			print_log(0, "%s: error %d gpio_export for %d\n",
				__func__, err, GPIO_ISDBT_PWR_EN);
	}

	err = gpio_request(GPIO_ISDBT_RST, "isdbt_rst");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_rst\n");
		goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST, 0);

#ifndef BBM_I2C_TSIF
	err = gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
#ifndef EVB
		goto gpio_isdbt_rst;
#endif
	}

	gpio_direction_input(GPIO_ISDBT_IRQ);
#ifdef EVB
	irq_set_irq_type(GPIO_ISDBT_IRQ, IRQF_TRIGGER_FALLING);
	err = request_threaded_irq((GPIO_ISDBT_IRQ), isdbt_irq
	, isdbt_threaded_irq, IRQF_DISABLED | IRQF_TRIGGER_RISING
	, FC8300_NAME, hInit);
#else
	err = request_threaded_irq(gpio_to_irq(GPIO_ISDBT_IRQ), isdbt_irq
	, isdbt_threaded_irq, IRQF_ONESHOT | IRQF_TRIGGER_RISING
	, FC8300_NAME, hInit);
#endif

	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: couldn't request gpio interrupt %d reason(%d)\n"
			, gpio_to_irq(GPIO_ISDBT_IRQ), err);
	goto request_isdbt_irq;
	}
#endif

	return 0;
#ifndef BBM_I2C_TSIF
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ);
#endif
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);
gpio_isdbt_en:
	gpio_free(GPIO_SYS_PWR_EN);
gpio_ldo_en_1:
	return err;
}

/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{

	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_init\n");

	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_SYS_PWR_EN, 1);
	mdelay(5);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	mdelay(5);
	gpio_set_value(GPIO_ISDBT_RST, 1);
	mdelay(5);

	driver_mode = ISDBT_POWERON;
	mutex_unlock(&driver_mode_lock);
}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{

	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_deinit\n");
	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
	gpio_set_value(GPIO_SYS_PWR_EN, 0);
	driver_mode = ISDBT_POWEROFF;
	mutex_unlock(&driver_mode_lock);
}

int data_callback(ulong hDevice, u8 bufid, u8 *data, int len)
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;

	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len)
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);

			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);
			mutex_unlock(&ringbuffer_lock);

		}
	}

	return 0;
}

#ifdef CONFIG_COMPAT
long isdbt_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long) compat_ptr(arg);

	return isdbt_ioctl(filp, cmd, arg);
}
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= isdbt_compat_ioctl,
#endif
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

struct miscdevice fc8300_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FC8300_NAME,
	.fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt open\n");

	hOpen = &hOpen_Val;
	hOpen->buf = &static_ringbuffer[0];
	hOpen->isdbttype = 0;

	if (list_empty(&(hInit->hHead)))
		list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if (hOpen->buf == NULL) {
		print_log(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;

	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}
	if (fci_ringbuffer_empty(cibuf))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}
	mutex_lock(&ringbuffer_lock);
	avail = fci_ringbuffer_avail(cibuf);
	mutex_unlock(&ringbuffer_lock);

	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);

	read_len = fci_ringbuffer_read_user(cibuf, buf, len);

	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt_release\n");
	isdbt_hw_deinit();

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;
	if (!list_empty(&(hInit->hHead)))
		list_del(&(hOpen->hList));
	return 0;
}


#ifndef BBM_I2C_TSIF
void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, DIV_BROADCAST, BBM_BUF_ENABLE, 0x00);

	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;

		msWait(10);
		isr_time++;
	}

}
#endif

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	s32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);
	if (size > sizeof(struct ioctl_info))
		size = sizeof(struct ioctl_info);
	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_reset(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_RESET\n");
#endif
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		res |= bbm_com_probe(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_INIT BBM : %s X : %d, DEV : %s\n"
		, DRIVER_VER, BBM_XTAL_FREQ, DRV_VER);
#endif
		if (res) {
			print_log(hInit, "FC8300 Initialize Fail\n");
			break;
		}
		res |= bbm_com_init(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_INIT res %d\n", res);
#endif
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u8)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u16 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u16)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u32 *)(&info.buff[2]));
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], (u32)info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3)) {
			print_log(hInit, "[FC8300] BULK_READ sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_read(hInit, (u16)info.buff[1]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[1], (u16)info.buff[0], info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BYTE_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u16)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_WORD_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0]
			, (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_LONG_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[1] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3)) {
			print_log(hInit, "[FC8300] BULK_WRITE sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_write(hInit, (u16)info.buff[2]
			, (u16)info.buff[0], (u8 *)(&info.buff[3])
			, info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_BULK_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[2], (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 4))) {
			print_log(hInit
				, "[FC8300] TUNER_R sizeErr A[%d] D[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_read(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			,  (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 4))) {
			print_log(hInit
				, "[FC8300] TUNER_W sizeErr A[%d] D[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_write(hInit, (u16)info.buff[3]
			, (u8)info.buff[0], (u8)info.buff[1]
			, (u8 *)(&info.buff[4]), (u8)info.buff[2]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[3], (u16)info.buff[0], info.buff[4]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			u8 subch;

			err = copy_from_user((void *)&info, (void *)arg, size);

			f_rf = (u32)info.buff[0];
			subch = (u8)info.buff[1];
#ifndef BBM_I2C_TSIF
			isdbt_isr_check(hInit);
#endif
			res = bbm_com_tuner_set_freq(hInit
				, (u16)info.buff[2], f_rf, subch);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_SET_FREQ [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], f_rf, subch);
#endif
#ifndef BBM_I2C_TSIF
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit
				, DIV_BROADCAST, BBM_BUF_ENABLE, 0x01);
#endif
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hInit, (u16)info.buff[2]
			, (u32)info.buff[0], (u32)info.buff[1]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_SELECT [0x%x][%d][0x%x]\n"
		, (u16)info.buff[2], (u32)info.buff[0], (u32)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TS_START:
		hOpen->isdbttype = TS_TYPE;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_START\n");
#endif
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_TS_STOP\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_init();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_ON\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_POWER_OFF\n");
#endif
		break;
	case IOCTL_ISDBT_SCAN_STATUS:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_scan_status(hInit, (u16)info.buff[0]);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_SCAN_STATUS [0x%x]\n"
		, (u16)info.buff[0]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit
			, (u16)info.buff[0], (s32 *)&info.buff[1]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
#ifdef FC8300_DEBUG
		print_log(hInit
		, "[FC8300] IOCTL_ISDBT_TUNER_GET_RSSI [0x%x][%d]\n"
		, (u16)info.buff[0], info.buff[1]);
#endif
		break;

	case IOCTL_ISDBT_DEINIT:
		res = bbm_com_deinit(hInit, DIV_BROADCAST);
#ifdef FC8300_DEBUG
		print_log(hInit, "[FC8300] IOCTL_ISDBT_DEINIT\n");
#endif
		break;

	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

#ifdef CONFIG_OF
static int fc8300_dt_init(void)
{
	struct device_node *np;
	u32 rc;

	np = of_find_compatible_node(NULL, NULL,
		fc8300_match_table[0].compatible);
	if (!np)
		return -ENODEV;

	power_en_gpio = of_get_named_gpio(np, "power-en-gpio", 0);
	if (!gpio_is_valid(power_en_gpio)) {
		print_log(hInit, "isdbt error getting power_en_gpio\n");
		return -EINVAL;
	}

	enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (!gpio_is_valid(enable_gpio)) {
		print_log(hInit, "isdbt error getting enable_gpio\n");
		return -EINVAL;
	}

	reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(reset_gpio)) {
		print_log(hInit, "isdbt error getting reset_gpio\n");
		return -EINVAL;
	}

	irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		print_log(hInit, "isdbt error getting irq_gpio\n");
		return -EINVAL;
	}

	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	rc = of_property_read_u32(np, "bbm-xtal-freq", &bbm_xtal_freq);
	if (rc)
		print_log(hInit, "no dt xtal-freq config, using default\n");

	return 0;
}
#else
static int fc8300_dt_init(void)
{
	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	return 0;
}
#endif

s32 isdbt_chip_id(void)
{
	s32 res;
	u16 addr, data;

	isdbt_hw_init();
	addr = FC8300_CHIP_ID_REG;
	res = bbm_com_word_read(hInit, DIV_BROADCAST, addr, &data);
	if (res) {
		print_log(hInit, "%s reading chip id err %d\n", __func__, res);
		goto errout;
	}

	if (data != FC8300_CHIP_ID) {
		print_log(hInit, "%s wrong chip id %#x\n", __func__, data);
		res = -1;
	} else
		print_log(hInit, "%s reg %#x id %#x\n", __func__, addr, data);

errout:
	isdbt_hw_deinit();
	return res;
}

//add for selftest
static ssize_t dvt_sysfs_enable(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ret = 2;
    int len = 0;
    int res = 0;

    isdbt_hw_init();   //power on

    res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
    res |= bbm_com_probe(hInit, DIV_BROADCAST);

    if (res) {
        printk(KERN_ERR"FC8300 Initialize Fail\n");
    }
    ret  = res;

    res |= bbm_com_init(hInit, DIV_BROADCAST);
    res = bbm_com_tuner_select(hInit, DIV_BROADCAST, 8300, 0x40);

    printk(KERN_ERR"%s reading chip id status %d\n", __func__, res);
    len = sprintf(&(buf[len]),"%d\n", ret);

    return len;
}

static ssize_t dvt_sysfs_disable(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret  = 2;
    int len = 0;

    isdbt_hw_deinit();
    ret = gpio_get_value(GPIO_ISDBT_PWR_EN);
    printk(KERN_ERR"%s ---dvt_sysfs_disable--- %d\n", __func__, ret);
    len = sprintf(&(buf[len]), "%d\n",ret);

    return len;
}

static ssize_t dvt_sysfs_rssi(struct device *dev, struct device_attribute *attr, char *buf)
{
    int tmp = 0;
    int len = 0;
    int res = 0;

    res = bbm_com_tuner_get_rssi(hInit, DIV_BROADCAST, (s32 *)&tmp);
    len = sprintf(&(buf[len]),"%d\n", rssi_value);

    return len;
}

static ssize_t dvt_sysfs_channel(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ISDBT_OPEN_INFO_T *hOpen;
    u32 freq = 473143;
    int res = 2;
    int len = 0;
    hOpen = &hOpen_Val;

#ifndef BBM_I2C_TSIF
    isdbt_isr_check(hInit);
#endif

    res = bbm_com_tuner_set_freq(hInit, DIV_BROADCAST, freq, 0x16);
    printk(KERN_ERR"%s bbm_com_tuner_set_freq---alex- --%d\n", __func__, res);

#ifndef BBM_I2C_TSIF
    mutex_lock(&ringbuffer_lock);
    fci_ringbuffer_flush(&hOpen->RingBuffer);
    mutex_unlock(&ringbuffer_lock);

    res=bbm_com_write(hInit, DIV_BROADCAST, BBM_BUF_ENABLE, 0x01);
    printk(KERN_ERR"%s bbm_com_write---alex- -111111-%d\n", __func__, res);
#endif

    res = bbm_com_scan_status(hInit, DIV_BROADCAST);
    printk(KERN_ERR"[FC8300] ----alex---status : %d\n", res);
    len = sprintf(&(buf[len]),"%d\n", res);

    return len;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, dvt_sysfs_enable, NULL);
static DEVICE_ATTR(disable, S_IRUGO|S_IWUSR, dvt_sysfs_disable, NULL);
static DEVICE_ATTR(rssi, S_IRUGO|S_IWUSR, dvt_sysfs_rssi, NULL);
static DEVICE_ATTR(channel, S_IRUGO|S_IWUSR, dvt_sysfs_channel, NULL);

s32 dvt_mp_proc_init(void)
{
    int ret;

    dtv_8300_kobj = kobject_create_and_add("android_dtv", NULL);
    if (dtv_8300_kobj == NULL)
    {
        printk("%s: DTV selftest subsystem register failed\n", __func__);
        return -1;
    }

    ret = sysfs_create_file(dtv_8300_kobj, &dev_attr_enable.attr);
    if (ret)
    {
        printk("%s: sysfs_create enable failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(dtv_8300_kobj, &dev_attr_disable.attr);
    if (ret) {
        printk("%s: sysfs_create disable failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(dtv_8300_kobj, &dev_attr_rssi.attr);
    if (ret)
    {
        printk("%s: sysfs_create rssi failed\n", __func__);
        return ret;
    }

    ret = sysfs_create_file(dtv_8300_kobj, &dev_attr_channel.attr);
    if (ret)
    {
        printk("%s: sysfs_create channel failed\n", __func__);
        return ret;
    }

    printk("DTV subsystem register succeed!\n");
    return 0;
}

void dvt_test_sysfs_deinit(void)
{
    sysfs_remove_file(dtv_8300_kobj, &dev_attr_enable.attr);
    sysfs_remove_file(dtv_8300_kobj, &dev_attr_disable.attr);
    sysfs_remove_file(dtv_8300_kobj, &dev_attr_rssi.attr);
    sysfs_remove_file(dtv_8300_kobj, &dev_attr_channel.attr);

    kobject_del(dtv_8300_kobj);
}

int isdbt_init(void)
{
	s32 res;

	print_log(NULL, "isdbt_init 20181213\n");

	res = misc_register(&fc8300_misc_device);

	if (res < 0) {
		print_log(NULL, "isdbt init fail : %d\n", res);
		return res;
	}

	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);
	res = fc8300_dt_init();
	if (res) {
		misc_deregister(&fc8300_misc_device);
		return res;
	}
	isdbt_hw_setting(hInit);
#ifndef BBM_I2C_TSIF
	bbm_com_ts_callback_register((ulong)hInit, data_callback);
#endif
#if defined(BBM_I2C_TSIF) || defined(BBM_I2C_SPI)
	res = bbm_com_hostif_select(hInit, BBM_I2C);
#else
	res = bbm_com_hostif_select(hInit, BBM_SPI);
#endif

	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");

	INIT_LIST_HEAD(&(hInit->hHead));

    dvt_mp_proc_init();

	res = isdbt_chip_id();
	if (res)
		goto error_out;

	return 0;
error_out:
	isdbt_exit();
	return -ENODEV;
}

void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit\n");

	isdbt_hw_deinit();

#ifndef BBM_I2C_TSIF
#ifdef EVB
	free_irq((GPIO_ISDBT_IRQ), NULL);
#else
	free_irq(gpio_to_irq(GPIO_ISDBT_IRQ), NULL);
#endif
	gpio_free(GPIO_ISDBT_IRQ);
#endif

	gpio_free(GPIO_ISDBT_RST);
	gpio_free(GPIO_ISDBT_PWR_EN);
	gpio_free(GPIO_SYS_PWR_EN);

#ifndef BBM_I2C_TSIF
	bbm_com_ts_callback_deregister();
#endif

	bbm_com_hostif_deselect(hInit);

	if (hInit != NULL)
		kfree(hInit);

	misc_deregister(&fc8300_misc_device);

    dvt_test_sysfs_deinit();
}

late_initcall(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");

