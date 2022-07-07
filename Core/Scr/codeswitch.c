/*
 * codeswitch.c
 *
 *  Created on: Jun 24, 2022
 *      Author: heemu.lee
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "packet.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

structInit init;

void CarNum_Init(void)
{
	uint8_t ten = 0, ten_1 = 0, ten_2 = 0, ten_4 = 0, ten_8 = 0;
	uint8_t one = 0, one_1 = 0, one_2 = 0, one_4 = 0, one_8 = 0;

	if(HAL_GPIO_ReadPin(ten_1_GPIO_Port,ten_1_Pin)) ten_1 =  0x01;
	if(HAL_GPIO_ReadPin(ten_2_GPIO_Port,ten_2_Pin)) ten_2 =  0x02;
	if(HAL_GPIO_ReadPin(ten_4_GPIO_Port,ten_4_Pin)) ten_4 =  0x04;
	if(HAL_GPIO_ReadPin(ten_8_GPIO_Port,ten_8_Pin)) ten_8 =  0x08;
	ten = ten_1 + ten_2 + ten_4 + ten_8;

	if(HAL_GPIO_ReadPin(one_1_GPIO_Port,one_1_Pin)) one_1 =  0x01;
	if(HAL_GPIO_ReadPin(one_2_GPIO_Port,one_2_Pin)) one_2 =  0x02;
	if(HAL_GPIO_ReadPin(one_4_GPIO_Port,one_4_Pin)) one_4 =  0x04;
	if(HAL_GPIO_ReadPin(one_8_GPIO_Port,one_8_Pin)) one_8 =  0x08;
	one = one_1 + one_2 + one_4 + one_8;

	init.CarNum = ten * 10 + one;
}

