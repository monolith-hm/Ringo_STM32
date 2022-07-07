/*
 * packet.h
 *
 *  Created on: May 23, 2022
 *      Author: heemu.lee
 */
#include <stdint.h>

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

/* ringo spi cmd */
#define SPI_CMD_EVENT			0x10
#define SPI_CMD_STAUS			0x20
#define SPI_CMD_REMOTE_SET		0x30
#define SPI_CMD_lOCATION_GET	0x40
#define SPI_CMD_CARNUM_GET		0x50

/* ringo event data */
#define REMOTE_ON				0x00
#define REMOTE_OFF				0x01
#define MANUAL_ON				0x02
#define MANUAL_OFF				0x03
#define NPC_ON					0x04
#define NPC_OFF					0x05
#define RESUME					0x06
#define SUSPEND					0x07

/* ringo status data */
#define READY					0x00
#define MANUAL					0x01
#define REMOTE					0x02
#define PAUSE					0x03
#define NPC						0x04

/* uwb buffer */
#define MAX_BUFFER_SIZE  	  (255)
#define STREAM_STATUS_READY     (0)
#define STREAM_STATUS_START     (1)

typedef struct {
	uint8_t g_udStatusData;
	uint8_t g_udEventData;
	int16_t g_iVelocity;
	int16_t g_iAngularVelocity;
} structRingoMode;

typedef struct {
    int16_t positionX;
    int16_t positionY;
    float headingAngle;
    int16_t pqf;
    float g_fq[4];
    int16_t g_fpi;
    int16_t g_ftheta;
    int16_t g_fpsi;
    uint8_t chrTemp[30];
} structLocation;

typedef struct{
  uint8_t head;
  uint8_t tail;
  uint8_t buffer[MAX_BUFFER_SIZE];
} uart_t;

typedef struct
{
	int l_ucAncNum;
	int l_udAncID[4];
	float l_fAncX[4];
	float l_fAncY[4];
	float l_fAncZ[4];
	float l_fDistFromAnc[4];
	float l_fPosX;
	float l_fPosY;
	float l_fPosZ;
	int l_udPosProb;
} structUWB;

typedef struct
{
	uint8_t CarNum;
} structInit;
void CarNum_Init(void);

#define UART_BUFFER_SIZE         256
#define DMA_RX_BUFFER_SIZE       256

#endif /* INC_PACKET_H_ */
