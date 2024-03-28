/*
 * Tiny I2C Chip Driver
 *
 * Copyright (C) 2003 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, version 2 of the License.
 *
 * This driver shows how to create a minimal I2C bus and algorithm driver.
 *
 * Compile this driver with:

	echo "obj-m := tiny_i2c_chip.o" > Makefile
	make -C <path/to/kernel/src> SUBDIRS=$PWD modules
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/delay.h>


#define AHT10_MEAS_SIZE		6

/*
 * Poll intervals (in milliseconds)
 */
#define AHT10_DEFAULT_MIN_POLL_INTERVAL	2000
#define AHT10_MIN_POLL_INTERVAL		2000

/*
 * I2C command delays (in microseconds)
 */
#define AHT10_MEAS_DELAY	80000
#define AHT10_CMD_DELAY		350000
#define AHT10_DELAY_EXTRA	100000

/*
 * Command bytes
 */
#define AHT10_CMD_INIT	0b11100001
#define AHT10_CMD_MEAS	0b10101100
#define AHT10_CMD_RST	0b10111010

/*
 * Flags in the answer byte/command
 */
#define AHT10_CAL_ENABLED	BIT(3)
#define AHT10_BUSY			BIT(7)
#define AHT10_MODE_NOR		(BIT(5) | BIT(6))
#define AHT10_MODE_CYC		BIT(5)
#define AHT10_MODE_CMD		BIT(6)

#define AHT10_MAX_POLL_INTERVAL_LEN	30

struct aht10_data {
	struct i2c_client *client;
	/*
	 * Prevent simultaneous access to the i2c
	 * client and previous_poll_time
	 */
	struct mutex lock;
	ktime_t min_poll_interval;
	ktime_t previous_poll_time;
	int temperature;
	int humidity;
	bool crc8;
	unsigned int meas_size;
} ;

static struct aht10_data *priv_data = NULL;

/**
 * aht10_init() - Initialize an AHT10/AHT20 chip
 * @data: the data associated with this AHT10/AHT20 chip
 * Return: 0 if successful, 1 if not
 */
static int aht10_init(struct aht10_data *data)
{
	const u8 cmd_init[] = {AHT10_CMD_INIT, AHT10_CAL_ENABLED | AHT10_MODE_CYC,
			       0x00};
	int res;
	u8 status;
	struct i2c_client *client = data->client;

	res = i2c_master_send(client, cmd_init, 3);
	if (res < 0)
		return res;

	usleep_range(AHT10_CMD_DELAY, AHT10_CMD_DELAY +
		     AHT10_DELAY_EXTRA);

	res = i2c_master_recv(client, &status, 1);
	if (res != 1)
		return -ENODATA;

	if (status & AHT10_BUSY)
		return -EBUSY;

	return 0;
}

/**
 * aht10_polltime_expired() - check if the minimum poll interval has
 *                                  expired
 * @data: the data containing the time to compare
 * Return: 1 if the minimum poll interval has expired, 0 if not
 */
static int aht10_polltime_expired(struct aht10_data *data)
{
	ktime_t current_time = ktime_get_boottime();
	ktime_t difference = ktime_sub(current_time, data->previous_poll_time);

	return ktime_after(difference, data->min_poll_interval);
}

/**
 * aht10_read_values() - read and parse the raw data from the AHT10/AHT20
 * @data: the struct aht10_data to use for the lock
 * Return: 0 if successful, 1 if not
 */
static int aht10_read_values(struct aht10_data *data)
{
	const u8 cmd_meas[] = {AHT10_CMD_MEAS, 0x33, 0x00};
	u32 temp, hum;
	int res;
	u8 raw_data[AHT10_MEAS_SIZE];
	struct i2c_client *client = data->client;

	mutex_lock(&data->lock);
	if (!aht10_polltime_expired(data)) {
		mutex_unlock(&data->lock);
		return 0;
	}

	res = i2c_master_send(client, cmd_meas, sizeof(cmd_meas));
	if (res < 0) {
		mutex_unlock(&data->lock);
		return res;
	}

	usleep_range(AHT10_MEAS_DELAY, AHT10_MEAS_DELAY + AHT10_DELAY_EXTRA);

	res = i2c_master_recv(client, raw_data, data->meas_size);
	if (res != data->meas_size) {
		mutex_unlock(&data->lock);
		if (res >= 0)
			return -ENODATA;
		return res;
	}

	hum =   ((u32)raw_data[1] << 12u) |
		((u32)raw_data[2] << 4u) |
		((raw_data[3] & 0xF0u) >> 4u);

	temp =  ((u32)(raw_data[3] & 0x0Fu) << 16u) |
		((u32)raw_data[4] << 8u) |
		raw_data[5];

	temp = ((temp * 625) >> 15u) * 10;
	hum = ((hum * 625) >> 16u) * 10;

	data->temperature = (int)temp - 50000;
	data->humidity = hum;
	data->previous_poll_time = ktime_get_boottime();

	mutex_unlock(&data->lock);

	pr_info("temperature: %d", data->temperature);
	pr_info("humidity: %d", data->humidity);
	return 0;
}

static int aht10_probe(struct i2c_client *client,
                              const struct i2c_device_id *id)
{
	int res = 0;

	pr_info("Driver Probe!!!\n");

	priv_data = devm_kzalloc(&client->dev, sizeof(*priv_data), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;
	
	priv_data->client = client;
	priv_data->meas_size = AHT10_MEAS_SIZE;

	priv_data->min_poll_interval = ms_to_ktime(AHT10_DEFAULT_MIN_POLL_INTERVAL);
	
	mutex_init(&priv_data->lock);

	res = aht10_init(priv_data);
	if (res < 0)
		return res;

	res = aht10_read_values(priv_data);
	if (res < 0)
		return res;

	return res;
}

static int aht10_remove(struct i2c_client *client)
{
	pr_info("Driver aht10_remove!!!\n");
	i2c_unregister_device(client);
	return 0;
}

static struct i2c_device_id foo_idtable[] = {
      { "aht10", 0 },
      { }
};

MODULE_DEVICE_TABLE(i2c, foo_idtable);

static struct i2c_driver aht10_driver = {
	.driver = {
		.name   = "aht10",
		.owner = THIS_MODULE,
		// .of_match_table =  , // for device tree
	},
	.id_table       = foo_idtable,
	.probe          = aht10_probe,
	.remove         = aht10_remove,
};

static struct i2c_board_info ath10_i2c_board_info = {
	I2C_BOARD_INFO("aht10", 0x38)
};


static int __init aht10_driver_init(void)
{
    int ret = -1;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client  *client = NULL;

    adapter = i2c_get_adapter(3);    
    if( adapter != NULL )
    {
		struct i2c_msg msg = {
			.addr = 0x38,
			.len = 0,
		};

		if (i2c_transfer(adapter, &msg, 1) != 1) {
			pr_info("No aht10 slave found.");
		}

        client = i2c_new_client_device(adapter, &ath10_i2c_board_info);
        if( client != NULL )
        {
            i2c_add_driver(&aht10_driver);
            ret = 0;
        }

        i2c_put_adapter(adapter);
    }
    
    pr_info("Driver Added!!!\n");
    return ret;
}
 
static void __exit aht10_driver_exit(void)
{
    i2c_del_driver(&aht10_driver);
    pr_info("Driver Removed!!!\n");
}
 
module_init(aht10_driver_init);
module_exit(aht10_driver_exit);


MODULE_AUTHOR("Greg Kroah-Hartman <greg@kroah.com>");
MODULE_DESCRIPTION("AHT10 driver");
MODULE_LICENSE("GPL");