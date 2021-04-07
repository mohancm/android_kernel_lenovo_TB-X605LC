/*****************************************************************************
*	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300_spi.c

	Description : source of SPI interface

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
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "fci_types.h"
#include "fc8300_regs.h"
#include "fci_oal.h"
#ifdef FEATURE_MTK
#include <mach/mt_spi.h>
#endif

#ifdef EVB
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <mach/dma.h>
#include <plat/s3c64xx-spi.h>
#endif

#define SPI_LEN             0x00 /* or 0x10 */
#define SPI_REG             0x20
#define SPI_THR             0x30
#define SPI_READ            0x40
#define SPI_WRITE           0x00
#define SPI_AINC            0x80

#ifdef EVB
#define DRIVER_NAME "tdmb"
#else
#define DRIVER_NAME "fc8300_spi"
#endif
struct spi_device *fc8300_spi;
static u8 tx_data[32];
static u8 *data_buf;
#ifdef EVB
#define S3C64XX_SPI_SWAP_CFG    0x28

#define S3C64XX_SPI_SWAP_RX_HALF_WORD           (1<<7)
#define S3C64XX_SPI_SWAP_RX_BYTE                (1<<6)
#define S3C64XX_SPI_SWAP_RX_BIT                 (1<<5)
#define S3C64XX_SPI_SWAP_RX_EN                  (1<<4)
#define S3C64XX_SPI_SWAP_TX_HALF_WORD           (1<<3)
#define S3C64XX_SPI_SWAP_TX_BYTE                (1<<2)
#define S3C64XX_SPI_SWAP_TX_BIT                 (1<<1)
#define S3C64XX_SPI_SWAP_TX_EN                  (1<<0)

#define S3C64XX_SPI_SWAP_OFF    0

struct s3c64xx_spi_driver_data {
	void __iomem                    *regs;
	struct clk                      *clk;
	struct clk                      *src_clk;
	struct platform_device          *pdev;
	struct spi_master               *master;
	struct workqueue_struct         *workqueue;
	struct s3c64xx_spi_info  *cntrlr_info;
	struct spi_device               *tgl_spi;
	struct work_struct              work;
	struct list_head                queue;
	spinlock_t                      lock;
	enum dma_ch                     rx_dmach;
	enum dma_ch                     tx_dmach;
	unsigned long                   sfr_start;
	struct completion               xfer_completion;
	unsigned                        state;
	unsigned                        cur_mode, cur_bpw;
	unsigned                        cur_speed;
};

static void spi_swap_set(struct spi_device *spi, int swap_cfg)
{
	struct spi_master *master = spi->master;
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	void __iomem *regs = sdd->regs;

	writel(swap_cfg, regs + S3C64XX_SPI_SWAP_CFG);
}
#endif

static DEFINE_MUTEX(fci_spi_lock);
static int fc8300_spi_probe(struct spi_device *spi)
{
	s32 ret;
#ifdef EVB
	struct s3c64xx_spi_csinfo *spi_csi;
#endif
#ifdef FEATURE_MTK
	struct mt_chip_conf *chip_config;
#endif

	print_log(0, "fc8300_spi_probe\n");

#ifdef FEATURE_MTK
	chip_config = (struct mt_chip_conf *) spi->controller_data;

	chip_config->setuptime = 3;
	chip_config->holdtime = 3;
	chip_config->high_time = 1;
	chip_config->low_time = 2;
	chip_config->cs_idletime = 2;
	chip_config->ulthgh_thrsh = 0;

	chip_config->cpol = 0;
	chip_config->cpha = 0;

	chip_config->rx_mlsb = 1;
	chip_config->tx_mlsb = 1;

	chip_config->tx_endian = 0;
	chip_config->rx_endian = 0;

	chip_config->com_mod = DMA_TRANSFER;
	chip_config->pause = 0;
	chip_config->finish_intr = 1;
	chip_config->deassert = 0;
	chip_config->ulthigh = 0;
	chip_config->tckdly = 2;
#endif
	spi->max_speed_hz = 50000000;
	spi->bits_per_word = 8;
	spi->mode =  SPI_MODE_0;
#ifdef EVB
	spi_csi = spi->controller_data;
	spi->max_speed_hz = 24000000;
#endif
	ret = spi_setup(spi);

	if (ret < 0)
		return ret;

	fc8300_spi = spi;

	return ret;
}

static int fc8300_spi_remove(struct spi_device *spi)
{

	return 0;
}
const struct of_device_id fc8300_match_table[] = {
	{
		.compatible = "fci, isdbt",
	},
	{}
};
static struct spi_driver fc8300_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
		.of_match_table = fc8300_match_table,
	},
	.probe		= fc8300_spi_probe,
	.remove		= fc8300_spi_remove,
};

#ifdef FEATURE_MTK
static struct spi_board_info fc8300_spi_devs[] __initdata = {
	[0] = {
		.modalias = "fc8300_spi",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz	 = 100000,
	},
};
#endif
static int fc8300_spi_write_then_read(struct spi_device *spi
	, u8 *txbuf, u16 tx_length, u8 *rxbuf, u16 rx_length)
{
	s32 res;
	struct spi_message  message;
	struct spi_transfer x;

	if (spi == NULL) {
		print_log(0, "[ERROR] FC8300_SPI Handle Fail...........\n");
		return BBM_NOK;
	}
	spi_message_init(&message);
	memset(&x, 0, sizeof(x));
	spi_message_add_tail(&x, &message);
	memcpy(data_buf, txbuf, tx_length);
	x.tx_buf = data_buf;
	x.rx_buf = data_buf;
	x.len = tx_length + rx_length;
	x.cs_change = 0;
#ifdef EVB
	if (x.len > 188) {
		int i;

		x.bits_per_word = 32;
		for (i = tx_length; i >= 0; i--)
			data_buf[i-1] = txbuf[tx_length - i];
	} else
#endif
	x.bits_per_word = 8;
#ifdef EVB
	if (x.bits_per_word == 8)
		spi_swap_set(spi, S3C64XX_SPI_SWAP_OFF);
	else
		spi_swap_set(spi,
		S3C64XX_SPI_SWAP_RX_EN
		|S3C64XX_SPI_SWAP_RX_BYTE
		|S3C64XX_SPI_SWAP_RX_HALF_WORD);
#endif
	res = spi_sync(spi, &message);

	if (rxbuf != NULL)
		memcpy(rxbuf, x.rx_buf + tx_length, rx_length);

	return res;
}

static s32 spi_bulkread(HANDLE handle, u8 devid,
		u16 addr, u8 command, u8 *data, u16 length)
{
	s32 res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = command | devid;
	tx_data[3] = length & 0xff;

	res = fc8300_spi_write_then_read(fc8300_spi
		, &tx_data[0], 4, data, length);

	if (res) {
		print_log(0, "fc8300_spi_bulkread fail : %d\n", res);
		return BBM_NOK;
	}

	return res;
}

static s32 spi_bulkwrite(HANDLE handle, u8 devid,
		u16 addr, u8 command, u8 *data, u16 length)
{
	int i;
	s32 res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = command | devid;
	tx_data[3] = length & 0xff;

	for (i = 0 ; i < length ; i++)
		tx_data[4+i] = data[i];

	res = fc8300_spi_write_then_read(fc8300_spi
		, &tx_data[0], length+4, NULL, 0);

	if (res) {
		print_log(0, "fc8300_spi_bulkwrite fail : %d\n", res);
		return BBM_NOK;
	}

	return res;
}

static s32 spi_dataread(HANDLE handle, u8 devid,
		u16 addr, u8 command, u8 *data, u32 length)
{
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = command | devid;
	tx_data[3] = length & 0xff;

	res = fc8300_spi_write_then_read(fc8300_spi
		, &tx_data[0], 4, data, length);

	if (res) {
		print_log(0, "fc8300_spi_dataread fail : %d\n", res);
		return BBM_NOK;
	}

	return res;
}


s32 fc8300_spi_init(HANDLE handle, u16 param1, u16 param2)
{
	s32 res = 0;

	print_log(0, "fc8300_spi_init\n");
	if (data_buf == NULL) {
		data_buf = kmalloc(64 * 1024, GFP_DMA | GFP_KERNEL);
		if (!data_buf) {
			print_log(0, "spi rdata_buf kmalloc fail\n");
			return BBM_NOK;
		}
	}
#ifdef FEATURE_MTK
	res = spi_register_board_info(fc8300_spi_devs
		, ARRAY_SIZE(fc8300_spi_devs));
	if (res)
		print_log(0, "spi_register_board_info fail : %d\n", res);
#endif
	res = spi_register_driver(&fc8300_spi_driver);
	if (res) {
		print_log(0, "fc8300_spi register fail : %d\n", res);
		return BBM_NOK;
	}
	return res;
}

s32 fc8300_spi_byteread(HANDLE handle, DEVICEID devid, u16 addr, u8 *data)
{
	s32 res;
	u8 command = SPI_READ;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, (u8) (devid & 0x000f), addr, command,
				data, 1);
	mutex_unlock(&fci_spi_lock);
	return res;
}

s32 fc8300_spi_wordread(HANDLE handle, DEVICEID devid, u16 addr, u16 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, (u8) (devid & 0x000f), addr, command,
				(u8 *)data, 2);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_longread(HANDLE handle, DEVICEID devid, u16 addr, u32 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, (u8) (devid & 0x000f), addr, command,
				(u8 *)data, 4);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_bulkread(HANDLE handle, DEVICEID devid,
		u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkread(handle, (u8) (devid & 0x000f), addr, command,
				data, length);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_bytewrite(HANDLE handle, DEVICEID devid, u16 addr, u8 data)
{
	s32 res;
	u8 command = SPI_WRITE;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, (u8) (devid & 0x000f), addr, command,
				(u8 *)&data, 1);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_wordwrite(HANDLE handle, DEVICEID devid, u16 addr, u16 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, (u8) (devid & 0x000f), addr, command,
				(u8 *)&data, 2);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_longwrite(HANDLE handle, DEVICEID devid, u16 addr, u32 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, (u8) (devid & 0x000f), addr, command,
				(u8 *) &data, 4);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_bulkwrite(HANDLE handle, DEVICEID devid,
		u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&fci_spi_lock);
	res = spi_bulkwrite(handle, (u8) (devid & 0x000f), addr, command,
				data, length);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_dataread(HANDLE handle, DEVICEID devid,
		u16 addr, u8 *data, u32 length)
{
	s32 res;
	u8 command = SPI_READ | SPI_THR;

	mutex_lock(&fci_spi_lock);
	res = spi_dataread(handle, (u8) (devid & 0x000f), addr, command,
				data, length);
	mutex_unlock(&fci_spi_lock);

	return res;
}

s32 fc8300_spi_deinit(HANDLE handle)
{
	spi_unregister_driver(&fc8300_spi_driver);
	return BBM_OK;
}

