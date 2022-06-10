/*
 * mpu9250.c
 *
 *  Created on: May 23, 2022
 *      Author: heemu.lee
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "mpu9250.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

#define PI	3.14159265359f

/* Define the I2C Handler's name according to I2C port in use */
extern I2C_HandleTypeDef hi2c1;

static inline float deg2rad(float x);

MPU9250_Result_t MPU9250_Init(MPU9250_t *MPU9250, MPU9250_Device_t dev, MPU9250_Accel_Scale_t accScale, MPU9250_Gyro_Scale_t gyroScale, MPU9250_Mag_Scale_t magScale)
{
	uint8_t data;
	uint8_t asa[3];
	MPU9250->I2C_Addr = MPU9250_I2C_ADDR | (uint8_t)dev;
	MPU9250->I2C_Addr_Mag = MPU9250_I2C_ADDR_MAG;

	/* Gyro & Acc Device Connection Check */
	if (isMPU9250Ready(&hi2c1, MPU9250->I2C_Addr) != MPU9250_RESULT_OK)
		return MPU9250_RESULT_NC;

	/* Who Am I Check */
	readByte(&hi2c1, MPU9250->I2C_Addr, WHO_AM_I, &data);
	if (data != 0x71)
		return MPU9250_RESULT_NC;

	/* Device Wake Up */
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x00);
	HAL_Delay(100);

	/* Auto-select best (stable) available clock source */
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x01);
	HAL_Delay(200);

	/* DLPF_CFG[2:0] = 001
	 * @Fs = 1000 Hz
	 * @Gyro BW: 41 Hz
	 * @Temperature BW: 42 Hz
	 * @Note: Sensor fusion update rate cannot be higher than (1/5.9ms) = 170 Hz
	 */
	writeByte(&hi2c1, MPU9250->I2C_Addr, CONFIG, 0x03);

	/* Sample Rate = Fs / (1 + SMPLRT_DIV)
	 * @Sample Rate = 1000 / (1 + 4) = 200 Hz
	 */
	writeByte(&hi2c1, MPU9250->I2C_Addr, SMPLRT_DIV, 0x04);

	/* Gyro Configuration */
	readByte(&hi2c1, MPU9250->I2C_Addr, GYRO_CONFIG, &data);
	data &= ~0x02;		/* [1:0] - Clear Fchoice_b[1:0] bits */
	data &= ~0x18;		/* [4:3] - Clear GYRO_FS_SEL[1:0] bits */
	data |= gyroScale;
	writeByte(&hi2c1, MPU9250->I2C_Addr, GYRO_CONFIG, data);

	/* Accel Configuraiton */
	readByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG, &data);
	data &= ~0x18;		/* [4:3] - Clear ACCEL_FS_SEL[1:0] bits */
	data |= accScale;
	writeByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG, data);

	/* Accel_2 Configuration
	 * @Accel BW: 44.8 Hz
	 */
	readByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG_2, &data);
	data &= ~0x0F;		/* Clear ACCEL_CONFIG_2[3:0] Bits */
	data |= 0x03;		/* A_DLPFCFG[2:0] bits are set to 011 */
	writeByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG_2, data);

	/* Interrupt Pin/Bypass Pin Configuration
	 * @INT Pin: push-pull (active high)
	 * Interrupt on raw sensor data ready
	 */
	writeByte(&hi2c1, MPU9250->I2C_Addr, INT_PIN_CFG, 0x22);
	writeByte(&hi2c1, MPU9250->I2C_Addr, INT_ENABLE, 0x01);

	/* Magnetometer Device Connection Check */
	if (isMPU9250Ready(&hi2c1, MPU9250->I2C_Addr_Mag) != MPU9250_RESULT_OK)
		return MPU9250_RESULT_NC;

	/* Magnetometer Power Down */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x00);
	HAL_Delay(10);
	/* Magnetometer Fuse ROM Access Mode ON */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x0F);
	HAL_Delay(10);
	/* read Sensitivity adjust registers */
	readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag, RA_ASAX, asa, 3);
	MPU9250->ASA[0] = asa[0];
	MPU9250->ASA[1] = asa[1];
	MPU9250->ASA[2] = asa[2];
	/* Magnetometer Power Down */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x00);
	HAL_Delay(10);
	/* MODE[3:0] Operation Mode: Continuous Measurement Mode 1
	 * @Mode Configuration
	 * 		- 0010: 8 Hz
	 * 		- 0110: 100 Hz
	 * @BIT Output Bit Setting: 16-bit output
	 */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, (1 << 4) | 2);
	HAL_Delay(10);

	/*
	 * Accelerometer Full Scale: 	±16g
	 * Gyroscope Full Scale:		±2000 degree/s
	 * Magnetometer Full Scale:		±4912 uT
	 */

	/* Accelerometer Resolution Multiplicator: LSB / g 			*/
	switch (accScale) {
		case ACCEL_SCALE_2G:
			MPU9250->accMult = 16834.0f;
			break;
		case ACCEL_SCALE_4G:
			MPU9250->accMult = 8192.0f;
			break;
		case ACCEL_SCALE_8G:
			MPU9250->accMult = 4096.0f;
			break;
		case ACCEL_SCALE_16G:
			MPU9250->accMult = 2048.0f;
			break;
	}

	/* Gyroscope Resolution Multiplicator: LSB / (degree/s) 	*/
	switch (gyroScale) {
		case GYRO_SCALE_250dps:
			MPU9250->gyroMult = 131.0f;
			break;
		case GYRO_SCALE_500dps:
			MPU9250->gyroMult = 65.5f;
			break;
		case GYRO_SCALE_1000dps:
			MPU9250->gyroMult = 32.8f;
			break;
		case GYRO_SCALE_2000dps:
			MPU9250->gyroMult = 16.4f;
			break;
	}

	/* Magnetometer Resolution Multiplicator: LSB / 0.15uT 		*/
	switch (magScale) {
		case MAG_SCALE_14bit:
			MPU9250->magMult = 0.6f;
			break;
		case MAG_SCALE_16bit:
			MPU9250->magMult = 0.15f;
			break;
	}

	/* Temperature Resolution Multiplicator: LSB / degreeC		*/
	MPU9250->tempMult = 333.87f;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadAcc(MPU9250_t *MPU9250)
{
	uint8_t data[6];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, ACCEL_XOUT_H, data, 6);

	MPU9250->acc_raw[0] = ((int16_t)data[0] << 8) | data[1];
	MPU9250->acc_raw[1] = ((int16_t)data[2] << 8) | data[3];
	MPU9250->acc_raw[2] = ((int16_t)data[4] << 8) | data[5];

	MPU9250->acc[0] = (float)MPU9250->acc_raw[0] / MPU9250->accMult;
	MPU9250->acc[1] = (float)MPU9250->acc_raw[1] / MPU9250->accMult;
	MPU9250->acc[2] = (float)MPU9250->acc_raw[2] / MPU9250->accMult;

	//MPU9250->acc[0] = (float)MPU9250->acc_raw[0] * MPU9250->accMult;
	//MPU9250->acc[1] = (float)MPU9250->acc_raw[1] * MPU9250->accMult;
	//MPU9250->acc[2] = (float)MPU9250->acc_raw[2] * MPU9250->accMult;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadGyro(MPU9250_t *MPU9250)
{
	uint8_t data[6];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, GYRO_XOUT_H, data, 6);

	MPU9250->gyro_raw[0] = ((int16_t)data[0] << 8) | data[1];
	MPU9250->gyro_raw[1] = ((int16_t)data[2] << 8) | data[3];
	MPU9250->gyro_raw[2] = ((int16_t)data[4] << 8) | data[5];

	MPU9250->gyro[0] = (float)MPU9250->gyro_raw[0] / MPU9250->gyroMult;
	MPU9250->gyro[1] = (float)MPU9250->gyro_raw[1] / MPU9250->gyroMult;
	MPU9250->gyro[2] = (float)MPU9250->gyro_raw[2] / MPU9250->gyroMult;

	//MPU9250->gyro[0] = (float)MPU9250->gyro_raw[0] * MPU9250->gyroMult;
	//MPU9250->gyro[1] = (float)MPU9250->gyro_raw[1] * MPU9250->gyroMult;
	//MPU9250->gyro[2] = (float)MPU9250->gyro_raw[2] * MPU9250->gyroMult;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadMag(MPU9250_t *MPU9250)
{
	uint8_t data[7];
	uint8_t check;

	/* Check Mag Data Ready Status */
	readByte(&hi2c1, MPU9250->I2C_Addr_Mag, ST1, &check);

	if (check & 0x01)
	{
		readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag, HXL, data, 7);
		/* Check (ST2 Register) If Magnetic Sensor Overflow Occured */
		if (!(data[6] & 0x08))
		{
			MPU9250->mag_raw[0] = ((int16_t)data[1] << 8) | data[0];
			MPU9250->mag_raw[1] = ((int16_t)data[3] << 8) | data[2];
			MPU9250->mag_raw[2] = ((int16_t)data[5] << 8) | data[4];

			//MPU9250->mag[0] = deg2rad((float)MPU9250->mag_raw[0] * MPU9250->magMult);
			//MPU9250->mag[1] = deg2rad((float)MPU9250->mag_raw[1] * MPU9250->magMult);
			//MPU9250->mag[2] = deg2rad((float)MPU9250->mag_raw[2] * MPU9250->magMult);
			MPU9250->mag[0] = ((float)MPU9250->mag_raw[0] / MPU9250->magMult) * ((MPU9250->ASA[0] - 128) * 0.5f / 128 + 1);
			MPU9250->mag[1] = ((float)MPU9250->mag_raw[1] / MPU9250->magMult) * ((MPU9250->ASA[1] - 128) * 0.5f / 128 + 1);
			MPU9250->mag[2] = ((float)MPU9250->mag_raw[2] / MPU9250->magMult) * ((MPU9250->ASA[2] - 128) * 0.5f / 128 + 1);

			return MPU9250_RESULT_OK;
		}
		return MPU9250_RESULT_ERROR;
	}
	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadTemperature(MPU9250_t *MPU9250)
{
	uint8_t data[2];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, TEMP_OUT_H, data, 2);

	MPU9250->temp_raw = ((int16_t)data[0] << 8) | data [1];

	MPU9250->temp = ((float)MPU9250->temp_raw / MPU9250->tempMult) + 21.0f;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_DataReady(MPU9250_t *MPU9250)
{
	uint8_t data;
	readByte(&hi2c1, MPU9250->I2C_Addr, INT_STATUS, &data);
	if (data & 0x01)
		return MPU9250_RESULT_OK;

	return MPU9250_RESULT_ERROR;
}

HAL_StatusTypeDef writeByte(I2C_HandleTypeDef *hi2c1, uint8_t device_addr, uint8_t register_addr, uint8_t data)
{
	uint8_t buffer[2];
	buffer[0] = register_addr;
	buffer[1] = data;

	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t)device_addr, (uint8_t *)buffer, 2, 1000) != HAL_OK)
	{
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
		return HAL_ERROR;
	}
	return HAL_OK;

}

HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c1, uint8_t device_addr, uint8_t register_addr, uint8_t *data)
{
	/* Transmit Register Address */
	//HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
	//HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t)device_addr, &register_addr, 1, 1000) != HAL_OK)
	{
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
			return HAL_ERROR;
		}
	}

	/* Receive Register Data */
	if (HAL_I2C_Master_Receive(hi2c1, (uint16_t)device_addr, data, 1, 1000) != HAL_OK)
	{
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef writeMultiBytes(I2C_HandleTypeDef *hi2c1, uint8_t device_addr, uint8_t register_addr, uint8_t *data, uint16_t count)
{
	if (HAL_I2C_Mem_Write(hi2c1, (uint16_t)device_addr, register_addr, register_addr > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, data, count, 1000) != HAL_OK)
	{
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef readMultiBytes(I2C_HandleTypeDef *hi2c1, uint8_t device_addr, uint8_t register_addr, uint8_t *data, uint16_t count)
{
	/* Transmit Register Address */
	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t)device_addr, &register_addr, 1, 1000) != HAL_OK)
	{
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
		return HAL_ERROR;
	}

	/* Receive Multiple Register Data */
	if (HAL_I2C_Master_Receive(hi2c1, (uint16_t)device_addr, data, count, 1000) != HAL_OK)
	{
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {}
		return HAL_ERROR;
	}
	return HAL_OK;
}

MPU9250_Result_t isMPU9250Ready(I2C_HandleTypeDef *hi2c1, uint8_t device_addr)
{
	/* Checks if device is ready to communicate */
	if (HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)device_addr, 2, 5) != HAL_OK)
		return MPU9250_RESULT_NC;

	return MPU9250_RESULT_OK;
}

static inline float deg2rad(float x)
{
	return ((PI / 180.0f) * x);
}

HAL_StatusTypeDef whoAmI_Check(MPU9250_t *mpu9250)
{
	uint8_t data;
	/* MPU9250 Who Am I Register Check */
	if (readByte(&hi2c1, mpu9250->I2C_Addr, WHO_AM_I, &data) != HAL_OK)
	{
		if (data != 0x71)
			return HAL_ERROR;
	}
	/* AK8963 Who Am I Register Check */
	if (readByte(&hi2c1, mpu9250->I2C_Addr_Mag, WIA, &data) != HAL_OK)
	{
		if (data != 0x48)
			return HAL_ERROR;
	}
	return HAL_OK;
}
/*
int getHeading(int x, int y)
{
  float heading;

  // 대한민국 서울:  -8º4' W = -8.067º W = -0.1408 radian
  heading = 180 * (atan2(y, x) - 0.1408) / PI;

  if (heading < 0) heading += 360;
  return (int)heading;
}

void MPU_ComplementaryFilter(MPU9250_t *MPU9250){
    float accdegx,accdegy,accdegz,acctotvec;

    MPU9250_ReadAcc(MPU9250);
    MPU9250_ReadGyro(MPU9250);
    MPU9250_ReadMag(MPU9250);
    MPU9250_ReadTemperature(MPU9250);

    acctotvec=sqrtf((float)(MPU9250->acc[0]*MPU9250->acc[0]/100
            +MPU9250->acc[1]*MPU9250->acc[1]/100
            +MPU9250->acc[2]*MPU9250->acc[2]/100))*10;
    accdegx=asinf((float)MPU9250->acc[0]/acctotvec)*(57.29577951);
    accdegy=asinf((float)MPU9250->acc[1]/acctotvec)*(57.29577951);
    accdegz=asinf((float)MPU9250->acc[2]/acctotvec)*(57.29577951);

    MPU9250->pitch=(0.5)*(MPU9250->pitch-(MPU9250->gyro[1])*0.01)+(1-0.5)*(accdegx);
    MPU9250->roll=(0.5)*(MPU9250->roll+(MPU9250->gyro[0])*0.01)+(1-0.5)*(accdegy);
    MPU9250->yaw=(0.5)*(MPU9250->yaw+(MPU9250->gyro[2])*0.01)+(1-0.5)*(accdegz);

    MPU9250->heading_angle = getHeading(MPU9250->mag[0], MPU9250->mag[1]);
}
*/




