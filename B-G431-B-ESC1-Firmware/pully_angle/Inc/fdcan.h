/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

#define	USTOP_RAMP  1
#define	U_POSITION  2
#define	U_VELOCITY 3
#define	U_IQ  4
#define U_RESET 0xFF
#define U_TYPE_DATA_5BYTE 7

// 8bit
#define U_REG_MODE 							(36 << ELT_IDENTIFIER_POS) | TYPE_DATA_8BIT

// 16bit
//#define  U_REG_IQ_      ((108U << ELT_IDENTIFIER_POS) | TYPE_DATA_16BIT)

// 32bit
#define  U_REG_POSITION_UPPER_LIMIT          ((117 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)
#define  U_REG_POSITION_LOWER_LIMIT          ((118 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)
#define  U_REG_SPEED_LIMIT          		 ((119 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)
#define  U_REG_IQ_LIMIT          			 ((120 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)
//#define  U_REG_POSITION_LIMIT          		 ((121 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)
//#define  U_REG_POSITION_LIMIT          		 ((122 << ELT_IDENTIFIER_POS) | TYPE_DATA_32BIT)


#define  MC_OVER_RUN     ((uint16_t)0x0800)

typedef struct CANDATA{
	uint32_t id;
	uint32_t len;
	uint8_t data[8];
} CANDATA;

typedef struct QUEUE{
	CANDATA queue[32];
	uint8_t front;
	uint8_t rear;
}QUEUE;

typedef enum {
    U_TORQUE_MODE = 0,
    U_VELOCITY_MODE,
    U_POSITION_TORQUE_MODE,
    U_POS_VEL_TORQUE_MODE,
} U_Contorol_Mode;

typedef struct MOTOR {
	uint8_t this_id;
	U_Contorol_Mode mode;

	int32_t accelation;
	int32_t velocity;
	int32_t position;
	int32_t torque;

	MCI_State_t prevstate;

	int32_t posLimit[2];
	int32_t velLimit[2];
	int32_t curLimit[2];

	int32_t velocity_ref;
}MOTOR;



/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void InitQueue();
bool isEmpty();
bool isFull();
bool dequeue(CANDATA* data);
uint8_t getQueueSize();
bool FDCAN_Soft_FifoQ(uint32_t id_, uint32_t len_, uint8_t data[8]);
void FDCAN_soft_FifoQ_Send();
bool FDCAN_Soft_FifoQ_Test(uint32_t id_, uint32_t len_, uint32_t data);
void Send_Sensor(uint32_t num);
void FDCAN_RxCallback(void);
void FDCAN_RxCallback2(void);
void U_SetLimit(U_Contorol_Mode mode);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

