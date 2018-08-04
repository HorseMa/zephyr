/** @file
 *  @brief HRS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>
#include <sensor.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "pid.h"

#define BLE_L2CAP_MTU_DEF           (23)
#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Health Thermometer Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Health Thermometer Measurement packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Health Thermometer Measurement. */
#define MAX30205_TEMP_STEP		0.00390625f

// Health Thermometer Measurement flag bits
#define HTS_MEAS_FLAG_TEMP_UNITS_BIT (0x01 << 0)  /**< Temperature Units flag. */
#define HTS_MEAS_FLAG_TIME_STAMP_BIT (0x01 << 1)  /**< Time Stamp flag. */
#define HTS_MEAS_FLAG_TEMP_TYPE_BIT  (0x01 << 2)  /**< Temperature Type flag. */

#define BLE_UUID_HEALTH_THERMOMETER_SERVICE                      BT_UUID_DECLARE_16(0x1809)     /**< Health Thermometer service UUID. */
#define BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR                    BT_UUID_DECLARE_16(0x2A1C)     /**< Temperature Measurement characteristic UUID. */
#define BLE_UUID_TEMPERATURE_TYPE_CHAR                           BT_UUID_DECLARE_16(0x2A1D)     /**< Temperature Type characteristic UUID. */
#define TEMP_TYPE_AS_CHARACTERISTIC     0                                           /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */
// Temperature Type measurement locations
#define BLE_HTS_TEMP_TYPE_ARMPIT      1
#define BLE_HTS_TEMP_TYPE_BODY        2
#define BLE_HTS_TEMP_TYPE_EAR         3
#define BLE_HTS_TEMP_TYPE_FINGER      4
#define BLE_HTS_TEMP_TYPE_GI_TRACT    5
#define BLE_HTS_TEMP_TYPE_MOUTH       6
#define BLE_HTS_TEMP_TYPE_RECTUM      7
#define BLE_HTS_TEMP_TYPE_TOE         8
#define BLE_HTS_TEMP_TYPE_EAR_DRUM    9

/**@brief Date and Time structure. */
typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
} ble_date_time_t;

typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;

/**@brief Health Thermometer Service measurement structure. This contains a Health Thermometer
 *        measurement. */
typedef struct ble_hts_meas_s
{
    bool                         temp_in_fahr_units;                        /**< True if Temperature is in Fahrenheit units, Celcius otherwise. */
    bool                         time_stamp_present;                        /**< True if Time Stamp is present. */
    bool                         temp_type_present;                         /**< True if Temperature Type is present. */
    ieee_float32_t               temp_in_celcius;                           /**< Temperature Measurement Value (Celcius). */
    ieee_float32_t               temp_in_fahr;                              /**< Temperature Measurement Value (Fahrenheit). */
    ble_date_time_t              time_stamp;                                /**< Time Stamp. */
    uint8_t                      temp_type;                                 /**< Temperature Type. */
} ble_hts_meas_t;

static struct bt_gatt_ccc_cfg hrmc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_hrm;
static u8_t htp_blsc;
static struct device *dev;
extern unsigned long stepCount; /* 步数值 */

static uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

static uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_hrm = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/*static ssize_t read_blsc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, u16_t len, u16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &htp_blsc,
				 sizeof(htp_blsc));
}*/

/* Heart Rate Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BLE_UUID_HEALTH_THERMOMETER_SERVICE),
	BT_GATT_CHARACTERISTIC(BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg, hrmc_ccc_cfg_changed),
	/*BT_GATT_CHARACTERISTIC(BLE_UUID_TEMPERATURE_TYPE_CHAR, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_blsc, NULL, NULL),*/
};

static struct bt_gatt_service htp_svc = BT_GATT_SERVICE(attrs);

u8_t pidrawdata[20];
static u8_t pidrawdatacount = 0;
static int hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
    struct sensor_value temp;
    uint32_t celciusX100;
    static int loop = 0;

    if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return -1;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printk("Cannot read MAX30205 temperature channel\n");
		return -1;
	}
    printk("%d,%d\n",loop ++,temp.val2);
    pidrawdata[pidrawdatacount ++] = temp.val2 / 0x0f;
    if(pidrawdatacount >= 20)
    {
        pidrawdatacount = 0;
        pid_notify();

        p_meas->temp_in_fahr_units = false;
        p_meas->time_stamp_present = false;
        p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

        celciusX100 = (uint32_t)((float)temp.val1 * MAX30205_TEMP_STEP * 100);//3700;//sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
        p_meas->temp_in_celcius.exponent = -2;
        p_meas->temp_in_celcius.mantissa = celciusX100;
        p_meas->temp_in_fahr.exponent    = -2;
        p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
        p_meas->time_stamp               = time_stamp;
        p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

        // update simulated time stamp
        time_stamp.seconds += 27;
        if (time_stamp.seconds > 59)
        {
            time_stamp.seconds -= 60;
            time_stamp.minutes++;
            if (time_stamp.minutes > 59)
            {
                time_stamp.minutes = 0;
            }
        }
        return 0;
    }
    else
    {
        return -1;
    }
}

static uint8_t ble_date_time_encode(const ble_date_time_t * p_date_time,
                                             uint8_t *               p_encoded_data)
{
    uint8_t len = uint16_encode(p_date_time->year, p_encoded_data);

    p_encoded_data[len++] = p_date_time->month;
    p_encoded_data[len++] = p_date_time->day;
    p_encoded_data[len++] = p_date_time->hours;
    p_encoded_data[len++] = p_date_time->minutes;
    p_encoded_data[len++] = p_date_time->seconds;

    return len;
}

static uint8_t hts_measurement_encode(ble_hts_meas_t * p_hts_meas,
                                      uint8_t        * p_encoded_buffer)
{
    uint8_t  flags = 0;
    uint8_t  len   = 1;
    uint32_t encoded_temp;

    // Flags field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;
    }
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
    }

    // Temperature Measurement Value field
    if (p_hts_meas->temp_in_fahr_units)
    {
        flags |= HTS_MEAS_FLAG_TEMP_UNITS_BIT;

        encoded_temp = ((p_hts_meas->temp_in_fahr.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_fahr.mantissa <<  0) & 0x00FFFFFF);
    }
    else
    {
        encoded_temp = ((p_hts_meas->temp_in_celcius.exponent << 24) & 0xFF000000) |
                       ((p_hts_meas->temp_in_celcius.mantissa <<  0) & 0x00FFFFFF);
    }
    len += uint32_encode(encoded_temp, &p_encoded_buffer[len]);
    /**(uint32_t*)&p_encoded_buffer[len] = encoded_temp;
    len += 4;*/

    // Time Stamp field
    if (p_hts_meas->time_stamp_present)
    {
        flags |= HTS_MEAS_FLAG_TIME_STAMP_BIT;
        len   += ble_date_time_encode(&p_hts_meas->time_stamp, &p_encoded_buffer[len]);
    }

    // Temperature Type field
    if (p_hts_meas->temp_type_present)
    {
        flags                  |= HTS_MEAS_FLAG_TEMP_TYPE_BIT;
        p_encoded_buffer[len++] = p_hts_meas->temp_type;
    }

    // Flags field
    p_encoded_buffer[0] = flags;

    return len;
}

void htp_init(u8_t blsc)
{
	dev = device_get_binding("MAX30205");

	if (dev == NULL) {
		printk("Could not get MAX30205 device\n");
		return;
	}

	htp_blsc = blsc;

	bt_gatt_service_register(&htp_svc);
}

void htp_notify(void)
{
	uint8_t encoded_hts_meas[MAX_HTM_LEN];
    uint16_t len;
    ble_hts_meas_t simulated_meas;
    int ret;

    ret = hts_sim_measurement(&simulated_meas);

    /* Heartrate measurements simulation */
	if (!simulate_hrm) {
		return;
	}
    if(ret < 0)
    {
        return;
    }
    //hts_sim_measurement(&simulated_meas);
    len = hts_measurement_encode(&simulated_meas, encoded_hts_meas);

#if 0
	hrm[0] = 0xf8; /* uint8, sensor contact */
	hrm[1] = tempinc.btemperature[3];
	hrm[2] = tempinc.btemperature[2];
	hrm[3] = tempinc.btemperature[1];
	hrm[4] = tempinc.btemperature[0];
#endif
	bt_gatt_notify(NULL, &attrs[1], encoded_hts_meas, len);
}
