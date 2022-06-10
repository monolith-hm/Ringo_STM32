/*
 * spibuscntl.c
 *
 *  Created on: May 23, 2022
 *      Author: heemu.lee
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "packet.h"

#include <string.h>
#include <stdbool.h>

#define BUS_BUFFER_SIZE		128
#define READ_WAIT() \
	for(;;) \
	{ \
		if ( hdma_spi1_rx.Instance->NDTR != sizeof(busbuffer_rx) ) \
		{ \
			if ( prev_size == hdma_spi1_rx.Instance->NDTR ) \
			{ \
				recv_size = sizeof(busbuffer_rx) - prev_size; \
				spi_dma_stop(recv_size); \
				break; \
			} \
		} \
		prev_size = hdma_spi1_rx.Instance->NDTR; \
		osDelay(1); \
	}

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/

/* USER CODE BEGIN Variables */
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern structLocation location;
extern structRingoMode ringomode;

uint8_t busbuffer_rx[ BUS_BUFFER_SIZE ];
uint8_t busbuffer_tx[ BUS_BUFFER_SIZE + 1 ];
uint8_t const busbuffer_size = BUS_BUFFER_SIZE;
uint8_t prev_size = BUS_BUFFER_SIZE;


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */
void spi_dma_stop( uint8_t size );
void spi_dma_ready( void );
bool bus_reset_check( void );

extern void set_resist_value(uint32_t resist);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* USER CODE BEGIN Application */
void BusControlLoop( void )
{
	uint8_t recv_size;
	uint8_t cmd;
	uint8_t l_udDataLength;
	//uint32_t value;

	if ( hdma_spi1_rx.Instance->NDTR != sizeof(busbuffer_rx) )
	{
		if ( prev_size == hdma_spi1_rx.Instance->NDTR )
		{
			recv_size = busbuffer_size - prev_size;
			spi_dma_stop(recv_size);

			if ( ( busbuffer_rx[0] & 0x01 ) == 0x00 )	// WRITE MODE
			{
				switch(recv_size)
				{
					case 0x03:
						break;

					case 0x08:
						break;
				}
			}
			else	// READ MODE
			{
				switch(recv_size)
				{
				case 0x02:
					cmd = busbuffer_rx[0] & 0xFE;
					l_udDataLength = busbuffer_rx[1];
					if(cmd == SPI_CMD_lOCATION_GET && l_udDataLength == 0)
					{

						busbuffer_tx[0] = 0x06;
						busbuffer_tx[1] = location.positionX & 0xFF;
						busbuffer_tx[2] = (location.positionX >> 8) &0xFF;
						busbuffer_tx[3] = location.positionY & 0xFF;
						busbuffer_tx[4] = (location.positionY >> 8) & 0xFF;
						busbuffer_tx[5] = location.g_fpsi & 0xFF;
						busbuffer_tx[6] = (location.g_fpsi >> 8) & 0xFF;
						/*
						busbuffer_tx[0] = 0x07;
						busbuffer_tx[1] = 1500 & 0xFF;
						busbuffer_tx[2] = (1500 >> 8) &0xFF;
						busbuffer_tx[3] = 2000 & 0xFF;
						busbuffer_tx[4] = (2000 >> 8) & 0xFF;
						busbuffer_tx[5] = 100 & 0xFF;
						busbuffer_tx[6] = (100 >> 8) & 0xFF;
						busbuffer_tx[7] = 100 & 0xFF;
						*/
						prev_size = sizeof(busbuffer_rx);
						spi_dma_ready();

						READ_WAIT();

						break;
					}
					else if(cmd == SPI_CMD_STAUS && l_udDataLength == 0)
					{
						busbuffer_tx[0] = 0x02;
						busbuffer_tx[1] = SPI_CMD_STAUS;
						busbuffer_tx[2] = ringomode.g_udStatusData;
						prev_size = sizeof(busbuffer_rx);
						spi_dma_ready();

						READ_WAIT();

						break;
					}
					break;
				case 0x03:
					cmd = busbuffer_rx[0] & 0xFE;
					l_udDataLength = busbuffer_rx[1];
					if(cmd == SPI_CMD_EVENT && l_udDataLength == 1)
					{
						ringomode.g_udEventData = busbuffer_rx[2];

						busbuffer_tx[0] = 0x02;
						busbuffer_tx[1] = SPI_CMD_EVENT;
						busbuffer_tx[2] = ringomode.g_udEventData;
						prev_size = sizeof(busbuffer_rx);
						spi_dma_ready();

						READ_WAIT();

						break;
					}
					break;
				case 0x06:
					cmd = busbuffer_rx[0] & 0xFE;
					l_udDataLength = busbuffer_rx[1];
					if(cmd == SPI_CMD_REMOTE_SET && l_udDataLength == 4)
					{
						ringomode.g_iVelocity = (busbuffer_rx[2] & 0x0F) + ((busbuffer_rx[3] << 8) & 0xF0);
						ringomode.g_iAngularVelocity = (busbuffer_rx[4] & 0x0F) + ((busbuffer_rx[5] << 8) & 0xF0);

						busbuffer_tx[0] = 0x04;
						busbuffer_tx[1] = ringomode.g_iVelocity &0xFF;
						busbuffer_tx[2] = (ringomode.g_iVelocity >> 8) &0xFF;
						busbuffer_tx[3] = ringomode.g_iAngularVelocity &0xFF;
						busbuffer_tx[4] = (ringomode.g_iAngularVelocity >> 8) &0xFF;
						prev_size = sizeof(busbuffer_rx);
						spi_dma_ready();

						READ_WAIT();

						break;
					}
					break;
				}

			}
			memset(busbuffer_tx, 0x00, busbuffer_size);
			spi_dma_ready( );
		}
		prev_size = hdma_spi1_rx.Instance->NDTR;
	}
}
void spi_dma_stop( uint8_t size )
{
	//uint8_t i;

	HAL_SPI_DMAStop(&hspi1);

	//printf("SPI1 RX:"); for (i=0;i<size;i++) printf(" %02X", busbuffer_rx[i]); printf("\n");
	//printf("SPI1 TX:"); for (i=0;i<size;i++) printf(" %02X", busbuffer_tx[i]); printf("\n");
}

void spi_dma_ready( void )
{
	memset(busbuffer_rx, 0x00, sizeof(busbuffer_rx));

	if ( __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) )
		HAL_SPI_TransmitReceive_DMA(&hspi1, busbuffer_tx, busbuffer_rx, busbuffer_size);
	else
	{
		hspi1.Instance->DR = busbuffer_tx[0];
		HAL_SPI_TransmitReceive_DMA(&hspi1, busbuffer_tx+1, busbuffer_rx, busbuffer_size);
	}
}

bool bus_reset_check( void )
{
	if ( busbuffer_tx[0] == 0x55)
		return true;
	else
		return false;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




