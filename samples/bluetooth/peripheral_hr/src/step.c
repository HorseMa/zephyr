/** @file
 *  @brief STEP Service sample
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
#include <device.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "step.h"
#include <sensor.h>
#include <device.h>
#include <math.h>

#ifndef TRUE
#define TRUE     1
#endif
#ifndef FALSE
#define FALSE    0
#endif
#define BT_UUID_STEP                       BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x87, 0x19, 0x86, 0x75)
#define BT_UUID_STEP_MEASUREMENT           BT_UUID_DECLARE_128(0x00, 0x00, 0x8B, 0x98, 0x42, 0xC2, 0x34, 0xA0,0x5C, 0x46, 0x84, 0x38, 0x25, 0x01, 0x86, 0x75)

static struct bt_gatt_ccc_cfg hrmc_ccc_cfg[BT_GATT_CCC_MAX] = {};
static u8_t simulate_hrm;
static u8_t step_blsc;

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 u16_t value)
{
	simulate_hrm = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Heart Rate Service Declaration */
static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(BT_UUID_STEP),
	BT_GATT_CHARACTERISTIC(BT_UUID_STEP_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg, hrmc_ccc_cfg_changed),
};

static struct bt_gatt_service step_svc = BT_GATT_SERVICE(attrs);

u32_t stepcounts = 0,stepcounts_old = 0;
unsigned long Step_Count(float axis0, float axis1, float axis2);

static struct device *mpu6050dev;
void step_init(u8_t blsc)
{
	step_blsc = blsc;
	int ret;

	printk("MPU6050 sample application\n");
	mpu6050dev = device_get_binding("MPU6050");

	if (!mpu6050dev) {
		printk("sensor: device not found.\n");
		return;
	}
	stepcounts = 0;

	ret = bt_gatt_service_register(&step_svc);
	if(ret < 0)
	{
		printk("register step Service err!\n");
	}
}

void step_notify(void)
{
	static u8_t hrm[1];
	struct sensor_value intensity[3];

	/* Heartrate measurements simulation */
	if (!simulate_hrm) {
		return;
	}

	if (sensor_sample_fetch(mpu6050dev)) {
			printk("sensor_sample fetch failed\n");
	}

	sensor_channel_get(mpu6050dev, SENSOR_CHAN_ACCEL_XYZ, intensity);

	/*printk("ambient light intensity without"
			" trigger is %d, %d, %d\n", intensity[0].val1, intensity[1].val1, intensity[2].val1);*/
	stepcounts = Step_Count(intensity[0].val1,intensity[1].val1,intensity[2].val1);
	if(stepcounts == stepcounts_old)
	{
		return;
	}
	else
	{
		hrm[0] = stepcounts;
		stepcounts_old = stepcounts;
		bt_gatt_notify(NULL, &attrs[1], &hrm, sizeof(hrm));
	}
}

#define P_P_DIFF	3 /* 波峰-波谷的差值，即3D阈值 */
#define RISING_EDGE  1 /* 上升沿状态 */
#define FALLING_EDGE 0 /* 下降沿状态 */
#define FAST_WALK_TIME_LIMIT_MS	200 	/* ms */
#define SLOW_WALK_TIME_LIMIT_MS	10000 /* 10s 内没有走一步 结束计步 */
#define STEP_OK 7	/* 7步法则 */

unsigned int lastPos = 0;	/* 旧数据 */
unsigned int newMax = 0, newMin = 0; /* 波峰-波谷 */

bool walkSta = FALSE; /* 获得一次峰值状态 */
bool walkOkSta = FALSE; /* 连续10s内走了7步 有效行走状态 */
bool pSta = RISING_EDGE; /* 3D数据波形状态 */

int64_t lastTime = 0;		/* 上一次 walkSta 的时间 */
unsigned char stepOK = 0; /* 初始计步门限  滤除干扰 */
unsigned long stepCount = 0; /* 步数值 */

/*****************************************************************
** input: 3 axis or angle
** output: step count
** user read:
	3 axis is filter value.
******************************************************************/
unsigned long Step_Count(float axis0, float axis1, float axis2){
	unsigned int nowPos = 0;
	int ppDiff = 0;
	int64_t timeDiff = 0;
	/* 获取3D IMU */
	nowPos = sqrtf(axis0 * axis0 + axis1 * axis1 + axis2 * axis2);
	//nowPos = (unsigned int)powf(sqrtf(axis0) + sqrtf(axis1) + sqrtf(axis2), 0.5);
	//printk("nowPos = %d,lastPos = %d\n",nowPos,lastPos);
	/* 得到波峰和波谷 */
	if((pSta==RISING_EDGE) && (nowPos<=lastPos)){
		pSta = FALLING_EDGE;
		newMax = lastPos;
		walkSta = TRUE;
	}
	else if((pSta==FALLING_EDGE) && (nowPos>lastPos)){
		pSta = FALLING_EDGE;
		newMin = lastPos;
		walkSta = TRUE;
	}
	else{
		walkSta = FALSE;
	}
	/* 更新3D step状态数据 */
	lastPos = nowPos;

	/* 有波峰或波谷 */
	if(walkSta==TRUE){
		walkSta = FALSE;
		ppDiff = newMax - newMin; /* 波峰与波谷的差值 */
		//printk("ppDiff = %d\n",ppDiff);
		if(ppDiff > P_P_DIFF){
			//printk("threhold\n");
			timeDiff = k_uptime_delta(&lastTime);	/* 获取波峰和波谷的时间差 */
			if(timeDiff < FAST_WALK_TIME_LIMIT_MS){  /* 波峰波谷时间差小于200ms的直接去掉 */
				//printk("<200ms\n");
				return stepCount;
			}
			else if(timeDiff > SLOW_WALK_TIME_LIMIT_MS){ /* 波峰波谷时间差大于10s的视为静止 */
				walkOkSta = FALSE;
				stepOK = 0;
				//printk(">10s\n");
				return stepCount;
			}
			stepOK++;
			//if(stepOK>=STEP_OK){ /* 走7步之后更新状态 */
				walkOkSta = TRUE;
			//}
			lastTime = k_uptime_get(); /* 更新时间 */
		}
	}

	if(walkOkSta==TRUE){ /* 满足10s内走7步 */
		stepCount += stepOK;
		stepOK = 0;
		//printk("step count = %d\n",stepCount);
	}
	return stepCount;
}
