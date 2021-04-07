#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include "couloMeter.h"
#include <linux/hqsysfs.h>  //modified by jiahao for M10x-658 @20190403

//add by zym for debug log  
//#define COULOMETER_DEBUG_LOG  //stone open for debug
#ifdef COULOMETER_DEBUG_LOG
#define LOG_INF pr_err     //stone modify for debug
#else
#define LOG_INF pr_debug
#endif

#define CM_SLAVE_ADDR_WRITE   0xBB
#define CM_SLAVE_ADDR_Read    0xAA

#define REG_TEMPERATURE                     0x06
#define REG_VOLTAGE                         0x08
#define REG_CURRENT                         0x14
#define REG_RSOC                            0x2c
#define REG_BLOCKDATAOFFSET                 0x3e
#define REG_BLOCKDATA                       0x40

/* Modify by lichuangchuang for battery debug (8909) SW00131408 20150531 */
int g_power_is_couloMeter = 1;

static struct i2c_board_info __initdata i2c_dev={I2C_BOARD_INFO("couloMeter", 0x55)};
static const struct i2c_device_id cm_i2c_id[] = {{"couloMeter", 0}, {}};
struct i2c_client *cm_iic_client = NULL;
static struct regulator *g_ldo = NULL;

static const struct of_device_id couloMeter_of_match[] = {
	{ .compatible = "qcom,couloMeter", },
	{},
};
MODULE_DEVICE_TABLE(of, couloMeter_of_match);


static int cm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm_i2c_remove(struct i2c_client *client);
static void cm_i2c_shutdown(struct i2c_client *client);

static struct i2c_driver CM_i2c_driver = {
	.driver = {
		.name  = "couloMeter",
		.owner = THIS_MODULE,
		.of_match_table = couloMeter_of_match,
	},
	.id_table  = cm_i2c_id,
	.probe     = cm_i2c_probe,
	.remove    = cm_i2c_remove,
	.shutdown  = cm_i2c_shutdown,
};

/**********************************************************
 *
 *   [Global Variable] 
 *
 *********************************************************/
 /* Modify by lichuangchuang for battery debug (8909) SW00131408 20150603 start */
static struct mutex coulometer_mutex;

/**********************************************************
 *
 *   [I2C Function For Read/Write sn65dsi8x] 
 *
 *********************************************************/
static int __cm_read_byte(u8 reg, int *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(cm_iic_client, reg);
	if (ret < 0) {
		LOG_INF("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}
	else {
		*val = ret;
	}
	return 0;
}

static int cm_read_byte(u8 reg, int *val)
{
	int rc;

	mutex_lock(&coulometer_mutex);
	if (g_ldo) {
		if (regulator_enable(g_ldo) < 0)
			LOG_INF("%s: failed to enable regulator\n", __func__);
	}
	rc = __cm_read_byte(reg, val);
	regulator_disable(g_ldo);
	mutex_unlock(&coulometer_mutex);
	return rc;
}

static int __cm_read_word(u8 reg)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(cm_iic_client, reg);
	if (ret < 0) {
		LOG_INF("i2c read fail: can't read from %02x: %d\n", reg, ret);
	}
	msleep(4);
	return ret;
}

static int cm_read_word(u8 reg)
{
	int rc;

	mutex_lock(&coulometer_mutex);
	if (g_ldo) {
		if (regulator_enable(g_ldo) < 0)
			LOG_INF("%s: failed to enable regulator\n", __func__);
	}
	rc = __cm_read_word(reg);
	regulator_disable(g_ldo);
	mutex_unlock(&coulometer_mutex);
	return rc;
}

static int __cm_write_word(u8 reg, int val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(cm_iic_client, reg, val);
	if (ret < 0) {
		LOG_INF("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
	}
	msleep(4);
	return ret;
}

static int cm_write_word(int reg, int val)
{
	int rc;

	mutex_lock(&coulometer_mutex);
	if (g_ldo) {
		if (regulator_enable(g_ldo) < 0)
			pr_err("%s: failed to enable regulator\n", __func__);
	}
	rc = __cm_write_word(reg, val);
	regulator_disable(g_ldo);
	mutex_unlock(&coulometer_mutex);
	return rc;
}

/**********************************************************
 *
 *   [coulometer function For Read/Write coulometer] 
 *
 *********************************************************/
/**********************************************************
 *[name]      :cm_get_ParameterVersion
 *[return]    :unsigned integer value
 *[desciption]:get battery vendor information.
 *********************************************************/
int cm_get_ParameterVersion()
{
	int ret = -1;
	uint prev = 0;

	// Parameter Rev.
	ret = cm_write_word(REG_BLOCKDATAOFFSET, 0x41f2);
	if (ret < 0)
		return ret;
	ret = cm_read_word(REG_BLOCKDATAOFFSET);
	if (ret != 0x41f2)
		return ret;
	ret = cm_read_word(REG_BLOCKDATA);
	if (ret < 0)
		return ret;
	prev = (uint)(ret & 0xffff);
	ret = prev;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_ParameterVersion prev=0x%x,ret=0x%x\n", prev, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_ID1
 *[return]    :unsigned integer value
 *[desciption]:get battery vendor information.
 *********************************************************/
int cm_get_ID1()
{
	int ret = -1;
	uint id1 = 0;

	// ID information 1
	ret = cm_write_word(REG_BLOCKDATAOFFSET, 0x41f8);
	if (ret < 0)
		return ret;
	ret = cm_read_word(REG_BLOCKDATAOFFSET);
	if (ret != 0x41f8)
		return ret;
	ret = cm_read_word(REG_BLOCKDATA);
	if (ret < 0)
		return ret;
	id1 = (uint)((ret & 0x00ff) << 8) | (uint)((ret & 0xff00) >> 8);
	id1 = id1 << 16;
	ret = cm_read_word(REG_BLOCKDATA + 2);
	if (ret < 0)
		return ret;
	id1 |= (uint)((ret & 0x00ff) << 8) | (uint)((ret & 0xff00) >> 8);
	ret = id1;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_ID1 id1=0x%x,ret=0x%x\n", id1, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_ID2
 *[return]    :unsigned integer value
 *[desciption]:get battery vendor information.
 *********************************************************/
int cm_get_ID2()
{
	int ret = -1;
	uint id2 = 0;

	// ID information 2
	ret = cm_write_word(REG_BLOCKDATAOFFSET, 0x41fc);
	if (ret < 0)
		return ret;
	ret = cm_read_word(REG_BLOCKDATAOFFSET);
	if (ret != 0x41fc)
		return ret;
	ret = cm_read_word(REG_BLOCKDATA);
	if (ret < 0)
		return ret;
	id2 = (uint)((ret & 0x00ff) << 8) | (uint)((ret & 0xff00) >> 8);
	id2 = id2 << 16;
	ret = cm_read_word(REG_BLOCKDATA + 2);
	if (ret < 0)
		return ret;
	id2 |= (uint)((ret & 0x00ff) << 8) | (uint)((ret & 0xff00) >> 8);
	ret = id2;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_ID2 id2=0x%x,ret=0x%x\n", id2, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_Temperature
 *[return]    :battery temperature in units 0.1k 
 *[desciption]:ret*0.1 -273.5
 *********************************************************/
int cm_get_Temperature(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	if (0 == g_power_is_couloMeter)
		return 250;/* Modify by lichuangchuang for battery debug (8909) SW00131408 20150629  */

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x06, &ret_a);
		ret = cm_read_byte(0x07, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	//switch k to T
	/* Modify by lichuangchuang for battery debug (8909) SW00131408 20150629  */
	ret = (ret * 1 - 2735); 
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_Temperature ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_Voltage
 *[return]    :unsigned integer value 
 *[desciption]:the measured cell-pack voltage in mv with a range of 0 to 6000mv
 *********************************************************/
int cm_get_Voltage(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	if (0 == g_power_is_couloMeter)
		return 4000;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x08, &ret_a);
		ret = cm_read_byte(0x09, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_Voltage ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_NominalAvailableCapacity
 *[return]    :battery capacity remaining
 *[desciption]:uncompenstated battery capacity remaining,units are mAh
 *********************************************************/
int cm_get_NominalAvailableCapacity(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x0C, &ret_a);
		ret = cm_read_byte(0x0D, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_NominalAvailableCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_RemainingCapacity
 *[return]    :compensated battery capacity remaining
 *[desciption]:compenstated battery capacity remaining,units are mAh
 *********************************************************/
int cm_get_RemainingCapacity(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x10, &ret_a);
		ret = cm_read_byte(0x11, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_RemainingCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_FullChargeCapacity
 *[return]    :compensated capacity
 *[desciption]:returns the compensated capacity of the battery when fully charged,units are mAh
 *********************************************************/
int cm_get_FullChargeCapacity(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x12, &ret_a);
		ret = cm_read_byte(0x13, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_FullChargeCapacity ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_AverageCurrent
 *[return]    :average current flow through the sense resistor
 *[desciption]:units are mAh
 *********************************************************/
int cm_get_AverageCurrent(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	if (0 == g_power_is_couloMeter)
		return 1900;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x14, &ret_a);
		ret = cm_read_byte(0x15, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;

	if (ret > 32767)
		ret = ret - 65536;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_AverageCurrent ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_AverageTimeToEmpty
 *[return]    :unsigned integer value of the predicted remaining battery life
 *[desciption]:a value of 65535 indicates battery is not being discharged,units are minutes
 *********************************************************/
int cm_get_AverageTimeToEmpty(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x16, &ret_a);
		ret = cm_read_byte(0x17, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_AverageTimeToEmpty ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_AverageTimeToFull
 *[return]    :unsigned integer value of predicated remaining time
 *[desciption]:remaining time until the battery reaches full charge,units are minutes
 *********************************************************/
int cm_get_AverageTimeToFull(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x18, &ret_a);
		ret = cm_read_byte(0x19, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_AverageTimeToFull ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_InternalTemperature
 *[return]    :internal temperature
 *[desciption]:
 *********************************************************/
int cm_get_InternalTemperature(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;
	
	if (0 == g_power_is_couloMeter )
		return 25;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x28, &ret_a);
		ret = cm_read_byte(0x29, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
	ret = (ret * 1 -2735)/10;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_InternalTemperature ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_StateOfCharge
 *[return]    :intege value of the predicted,0%000%
 *[desciption]:
 *********************************************************/
int cm_get_StateOfCharge(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	if (0 == g_power_is_couloMeter)
		return 50;

	while ((ret < 0) && (count < 10)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x2C, &ret_a);
		ret = cm_read_byte(0x2D, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
		LOG_INF("[%s]: read soc count=%d\n", __FUNCTION__, count);
	}

	/*stone add start SOC*/
	if (ret < 0) {
		ret = 50;
		g_power_is_couloMeter = 0;
		LOG_INF("[%s,%d]g_power_is_couloMeter=%d ret=%d\n",
				__FUNCTION__, __LINE__, g_power_is_couloMeter, ret);
	}
	else {
		ret = (ret_b << 8) | ret_a;
		g_power_is_couloMeter = 1;
		LOG_INF("[%s,%d] else g_power_is_couloMeter=%d ret=%d\n",
				__FUNCTION__, __LINE__, g_power_is_couloMeter, ret);
	}
	/*stone  add  end*/

#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_StateOfCharge ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *[name]      :cm_get_StateOfHealth
 *[return]    :unsigned integer value
 *[desciption]:SOH percentage is 0x00 to 0x64,indicating 0 to 100% correspondingly
 *********************************************************/
int cm_get_StateOfHealth(void)
{
	int ret = -1, ret_a = 0, ret_b = 0;
	int count = 0;

	while ((ret < 0) && (count < 3)) {
		//after firstRegister is repeatStart flag,so firstRegister fail,secondRegister must fail
		ret = cm_read_byte(0x2E, &ret_a);
		ret = cm_read_byte(0x2F, &ret_b);
		if (ret < 0) {
			LOG_INF("couloMeter fatal error,+^-^+ ret=%d\n", ret);
			count++;
			mdelay(10);
		}
	}
	ret = (ret_b << 8) | ret_a;
#if defined(COULOMETER_DEBUG_LOG)
	LOG_INF("cm_get_StateOfHealth ret_a=0x%x,ret_b=0x%x,ret=%d\n", ret_a, ret_b, ret);
#endif
	return ret;
}

/**********************************************************
 *
 *   [I2C probe For Read/Write coulometer] 
 *
 *********************************************************/
static int cm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int bat_id;  //modified by jiahao for M10x-658 @20190403
	/* Modify by lichuangchuang for battery debug (8909) SW00131408 20150531 start */
	LOG_INF("++stone [CM_i2c_probe]Start\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	cm_iic_client = client;
	g_ldo = devm_regulator_get(&client->dev, "iic2");
	if (!g_ldo) {
		pr_err("%s: failed to get regulator!\n", __func__);
	}

	LOG_INF("++stone [CM_i2c_probe]Attached!!\n");
	cm_get_ParameterVersion();
        /*modified by jiahao for M10x-658 @20190403 begin*/
	bat_id = cm_get_ID1();
        if (bat_id == 0x21)
            hq_regiser_hw_info(HWID_BATERY, "FMT");
        else
            hq_regiser_hw_info(HWID_BATERY, "JBY");
        /*modified by jiahao for M10x-658 @20190403 end*/
	cm_get_ID2();

	return 0;
}

static int cm_i2c_remove(struct i2c_client *client)
{
	LOG_INF("[CM_i2c_probe]Start\n");
	return 0;
}

static void cm_i2c_shutdown(struct i2c_client *client)
{
	LOG_INF("[CM_i2c_probe]Start\n");
}

static int cm_probe(struct platform_device *pdev)
{
	LOG_INF("+++stone+++_[cm_probe]_Start\n");
	mutex_init(&coulometer_mutex);
	return i2c_add_driver(&CM_i2c_driver);
}

static int cm_remove(struct platform_device *pdev)
{
	LOG_INF("[cm_remove] here we start!\n");
	i2c_del_driver(&CM_i2c_driver);
	return 0;
}

static void cm_shutdown(struct platform_device *pdev)
{
	LOG_INF("[cm_shutdown] here we start!\n");
}

static int cm_suspend(struct platform_device *pdev, pm_message_t state)
{
	LOG_INF("[cm_suspend] here we start!\n");
	return 0;
}

static int cm_resume(struct platform_device *pdev)
{
	LOG_INF("[cm_resume] here we start !\n");
	return 0;
}

static int mt_couloMeter_probe(struct platform_device *dev)    
{
	return 0;
}

//platform devices
static struct platform_driver g_cm_Driver = {
	.probe    = cm_probe,
	.remove   = cm_remove,
	.shutdown = cm_shutdown,
	.suspend  = cm_suspend,
	.resume   = cm_resume,
	.driver = {
		.name = "couloMeter",
	}
};
static struct platform_device g_cm_device = {
	.name = "couloMeter",
	.id = -1,
};

//for porc debugfs
struct platform_device MT_couloMeter_device = {
	.name = "mt_coulometer",
	.id   = -1,
};

static struct platform_driver mt_couloMeter_driver = {
	.probe  = mt_couloMeter_probe,
	.driver = {
		.name = "mt_coulometer",
	},
};

static int __init cm_i2C_init(void)
{
	LOG_INF("couloMeter_i2c_init\n");
	if (platform_device_register(&g_cm_device)) {
		LOG_INF("failed to register couloMeter driver\n");
		return -ENODEV;
	}

	if (platform_driver_register(&g_cm_Driver)) {
		LOG_INF("Failed to register couloMeter driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&MT_couloMeter_device)) {
		LOG_INF("****[mt_couloMeter] Unable to device register\n");
		return -ENODEV;
	}

	if (platform_driver_register(&mt_couloMeter_driver)) {
		LOG_INF("****[mt_couloMeter] Unable to register driver\n");
		return -ENODEV;
	}

	i2c_register_board_info(1, &i2c_dev, 1);
	LOG_INF("+++zym+++ i2c_register_board_info(1,$i2c_dev,1)\n");
	LOG_INF("%s: g_power_is_couloMeter = %d\n", __func__, g_power_is_couloMeter);
	return 0;
}

static void __exit cm_i2C_exit(void)
{
	LOG_INF("couloMeter_i2c_exit\n");
	platform_driver_unregister(&g_cm_Driver);
	platform_driver_unregister(&mt_couloMeter_driver);
}

module_init(cm_i2C_init);
module_exit(cm_i2C_exit);

MODULE_DESCRIPTION("CouloMeter i2c module driver");
MODULE_LICENSE("GPL v2");

