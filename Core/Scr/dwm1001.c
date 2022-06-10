/*
 * dwm1001.c
 *
 *  Created on: May 23, 2022
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

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart3;
extern uart_t uart;
extern void push(uart_t*, uint8_t);
extern uint8_t pop(uart_t*);
extern uint8_t isEmpty(uart_t*);


void init_uart(uart_t* u)
{
  u->head = 0;
  u->tail = 0;
  memset(u->buffer, 0, sizeof(u->buffer));
}

void push(uart_t* u, uint8_t data)
{
  u->buffer[u->head] = data;

  u->head++;

  if (u->head >= MAX_BUFFER_SIZE) {
    u->head = 0;
  }
}

uint8_t pop(uart_t* u)
{
  uint8_t data = u->buffer[u->tail];

  u->tail++;

  if (u->tail >= MAX_BUFFER_SIZE) {
    u->tail = 0;
  }

  return data;
}

uint8_t isEmpty(uart_t* u)
{
  return u->head == u->tail;
}
int ParseUWBStr(uint8_t *strInput, structUWB *pUwb)
{
	int l_rtnVal = 0 ;

	//memcpy(strInput, "DIST,4,AN0,0C2E,0.00,0.00,0.00,2.03,AN1,47A7,5.00,0.00,0.00,5.32,AN2,5B39,0.00,5.00,0.00,6.91,AN3,5B26,5.00,5.00,0.00,8.38,POS,0.12,-1.83,0.92,83",
			//strlen("DIST,4,AN0,0C2E,0.00,0.00,0.00,2.03,AN1,47A7,5.00,0.00,0.00,5.32,AN2,5B39,0.00,5.00,0.00,6.91,AN3,5B26,5.00,5.00,0.00,8.38,POS,0.12,-1.83,0.92,83")) ;

	sscanf((const char*)strInput, "TSID,%d", &(pUwb->l_ucAncNum));

	 if (pUwb->l_ucAncNum == 3)
	    {
	        l_rtnVal = sscanf((const char*)strInput, "TSID,3,AN0,%x,%f,%f,%f,%f,AN1,%x,%f,%f,%f,%f,AN2,%x,%f,%f,%f,%f,POS,%f,%f,%f,%d\r"
	                                                    , &(pUwb->l_udAncID[0]), &(pUwb->l_fAncX[0]), &(pUwb->l_fAncY[0]), &(pUwb->l_fAncZ[0]), &(pUwb->l_fDistFromAnc[0])
	                                                    , &(pUwb->l_udAncID[1]), &(pUwb->l_fAncX[1]), &(pUwb->l_fAncY[1]), &(pUwb->l_fAncZ[1]), &(pUwb->l_fDistFromAnc[1])
	                                                    , &(pUwb->l_udAncID[2]), &(pUwb->l_fAncX[2]), &(pUwb->l_fAncY[2]), &(pUwb->l_fAncZ[2]), &(pUwb->l_fDistFromAnc[2])
	                                                    , &(pUwb->l_fPosX), &(pUwb->l_fPosY), &(pUwb->l_fPosZ), &(pUwb->l_udPosProb));

	    }
	    else  if(pUwb->l_ucAncNum == 4)
	    {
	        l_rtnVal = sscanf((const char*)strInput, "TSID,4,AN0,%x,%f,%f,%f,%f,AN1,%x,%f,%f,%f,%f,AN2,%x,%f,%f,%f,%f,AN3,%x,%f,%f,%f,%f,POS,%f,%f,%f,%d\r"
														, &(pUwb->l_udAncID[0]), &(pUwb->l_fAncX[0]), &(pUwb->l_fAncY[0]), &(pUwb->l_fAncZ[0]), &(pUwb->l_fDistFromAnc[0])
														, &(pUwb->l_udAncID[1]), &(pUwb->l_fAncX[1]), &(pUwb->l_fAncY[1]), &(pUwb->l_fAncZ[1]), &(pUwb->l_fDistFromAnc[1])
														, &(pUwb->l_udAncID[2]), &(pUwb->l_fAncX[2]), &(pUwb->l_fAncY[2]), &(pUwb->l_fAncZ[2]), &(pUwb->l_fDistFromAnc[2])
														, &(pUwb->l_udAncID[3]), &(pUwb->l_fAncX[3]), &(pUwb->l_fAncY[3]), &(pUwb->l_fAncZ[3]), &(pUwb->l_fDistFromAnc[3])
														, &(pUwb->l_fPosX), &(pUwb->l_fPosY), &(pUwb->l_fPosZ), &(pUwb->l_udPosProb));

	    }
	    else
	    {

	        return 0;
	    }




	return l_rtnVal ;
}
