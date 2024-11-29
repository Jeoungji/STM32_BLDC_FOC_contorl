/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include <string.h>
#include <stdint.h>
#include "register_interface.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_FilterTypeDef sFilterConfig;

QUEUE q;
uint8_t TxData[8];
uint8_t RxData[12];

extern MOTOR motor;
extern MCI_Handle_t *pMCI[NBR_OF_MOTORS];


/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 4;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	/* Prepare Tx Header*/
	TxHeader.Identifier = 0x1;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType=FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	    Error_Handler();
	}

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterID1 = 0x28;
	sFilterConfig.FilterID2 = 0x50;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) { Error_Handler(); }

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) { Error_Handler(); }

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) { Error_Handler();	}
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void InitQueue() {
	q.front = -1;
	q.rear = -1;
}

bool isEmpty() {
	return (q.front == -1);
}

bool isFull() {
    return (q.rear + 1) % 64 == q.front;
}

bool dequeue(CANDATA* data) {
    if (isEmpty()) {
        return false;
    }
    *data = q.queue[q.front];
    if (q.front == q.rear) {
        q.front = -1;
        q.rear = -1;
    } else {
        q.front = (q.front + 1) % 64;
    }
    return true;
}

uint8_t getQueueSize() {
    if (isEmpty()) {
        return 0;
    }
    if (q.rear >= q.front) {
        return q.rear - q.front + 1;
    } else {
        return (64 - q.front + q.rear + 1) % 64;
    }
}

bool FDCAN_Soft_FifoQ(uint32_t id_, uint32_t len_, uint8_t data[8]) {
	if (id_ < 1 || id_ > 0x7FF) return false;
	if (len_ > 8) return false;
	if (isFull()) return false;

	if (isEmpty()) {
		q.front = 0;
	}
	CANDATA datas;
	datas.id = id_;
	datas.len = len_;
	memcpy(datas.data, data, 8*sizeof(uint8_t));

	q.rear = (q.rear + 1) % 64;
	q.queue[q.rear] = datas;

	return true;
}
bool FDCAN_Soft_FifoQ_Test(uint32_t id_, uint32_t len_, uint32_t data) {
	uint8_t datas[8];
	memcpy(datas, &data, sizeof(data));
	return FDCAN_Soft_FifoQ(id_, len_, datas);
}
void FDCAN_soft_FifoQ_Send() {
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
		CANDATA senddata;
		if (!dequeue(&senddata)) break;
		if (senddata.id < 1 || senddata.id > 0x7FF) break;
		if (senddata.len <= 0 || senddata.len > 8) break;
		TxHeader.Identifier = senddata.id;
		TxHeader.DataLength = senddata.len;
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, senddata.data) != HAL_OK) { Error_Handler(); }
	}
}

void U_SetLimit(U_Contorol_Mode mode) {
	switch (mode) {
	case U_TORQUE_MODE:
		{
			// pMCI[0]->pSTC->TargetFinal = 0;
			motor.velocity = 0;
			break;
		}
	case U_VELOCITY_MODE:
		{
			motor.torque = 0;
			pMCI[0]->pSTC->PISpeed->uLowerLimit = motor.curLimit[0];
			pMCI[0]->pSTC->PISpeed->uUpperLimit = motor.curLimit[1];
			break;
		}
	case U_POSITION_TORQUE_MODE:
		{
			motor.torque = 0;
			motor.velocity = 0;
			pMCI[0]->pPosCtrl->PIDPosRegulator->uLowerLimit = motor.curLimit[0];
			pMCI[0]->pPosCtrl->PIDPosRegulator->uUpperLimit = motor.curLimit[1];
			break;
		}
	case U_POS_VEL_TORQUE_MODE:
		{
			motor.torque = 0;
			motor.velocity = 0;
			pMCI[0]->pPosCtrl->PIDPosRegulator->uLowerLimit = motor.velLimit[0];
			pMCI[0]->pPosCtrl->PIDPosRegulator->uUpperLimit = motor.velLimit[1];
			pMCI[0]->pSTC->PISpeed->uLowerLimit = motor.curLimit[0];
			pMCI[0]->pSTC->PISpeed->uUpperLimit = motor.curLimit[1];
			break;
		}
	default:
	{

	}
	}
}

uint8_t TypeID(uint8_t d) {
	uint8_t result;
	switch (d) {
	case 0x01:
		result = TYPE_DATA_SEG_END;
		break;
	case 0x02:
		result = TYPE_DATA_8BIT;
		break;
	case 0x03:
		result = TYPE_DATA_16BIT;
		break;
	case 0x04:
		result = TYPE_DATA_32BIT;
		break;
	case 0x05:
		result = TYPE_DATA_STRING;
		break;
	case 0x06:
		result = TYPE_DATA_RAW;
		break;
	case 0x07:
		result = TYPE_DATA_FLAG;
		break;
	case 0x08:
		result = TYPE_DATA_SEG_BEG;
		break;
	default:
		result = 0;
	}
	return result;
}

uint8_t SizeID(uint8_t typeID) {
	uint8_t result;
	switch (typeID) {
	case TYPE_DATA_8BIT:
		result = 1;
		break;
	case TYPE_DATA_16BIT:
		result = 2;
		break;
	case TYPE_DATA_32BIT:
		result = 4;
		break;
	default:
		result = 0;
	}
	return result;
}

void USER_Set_register(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size) {

}

void USER_Set(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size) {
	MCI_Handle_t *pMCIN = &Mci[0];
	uint8_t retVal = MCP_CMD_OK;
	*size = 0;
	switch (typeID)
	{
	case 0:
	{
		switch (regID)
		{
		case USTOP_RAMP:
		{
			if (RUN == MCI_GetSTMState(pMCI[0]))
			{
			  MCI_StopRamp(pMCI[0]);
			}
			else
			{
			  /* Nothing to do */
			}
			break;
		}
		case U_RESET:
		{
			FDCAN_Soft_FifoQ(0x04, 1, 1);
			Error_Handler();
			break;
		}
		}
		break;
	}
	case TYPE_DATA_8BIT:
	{
		switch (regID)
		{
		case U_REG_MODE:
		{
			motor.mode = *data;
			U_SetLimit(motor.mode);
			data = data - 3;
			* data &= 0x0F;
			* data |= 0x60;
			*size = 1;
			break;
		}
		}
		break;
	}
	case TYPE_DATA_16BIT:
	{
		uint16_t regdata16 = *(uint16_t *)data;
		break;
	}
	case TYPE_DATA_32BIT:
	{
		uint32_t* regdata32 = (uint32_t *)data;
		switch (regID)
		{
			case U_POSITION:
			{
				float target;
				memcpy(&target, regdata32, sizeof(float));
				MC_ProgramPositionCommandMotor1(target, 0);

				break;
			}
			case U_VELOCITY:
			{
				float target;
				memcpy(&target, regdata32, sizeof(float));

				break;
			}
			case U_IQ:
			{

				break;
			}
			case U_REG_POSITION_LOWER_LIMIT:
			{
				motor.velLimit[0] =  * regdata32;
				break;
			}
			case U_REG_POSITION_UPPER_LIMIT:
			{
				motor.velLimit[1] =  * regdata32;
				break;
			}
			case U_REG_SPEED_LIMIT:
			{
				int16_t regdata16_1 = *(int16_t *)data;
				if (regdata16_1 < -MAX_APPLICATION_SPEED_UNIT)
					regdata16_1 = -MAX_APPLICATION_SPEED_UNIT;
				motor.velLimit[0] =  regdata16_1;

				int16_t regdata16_2 = *(int16_t *)(data+2);
				if (regdata16_1 > MAX_APPLICATION_SPEED_UNIT)
					regdata16_1 = MAX_APPLICATION_SPEED_UNIT;
				motor.velLimit[1] =  regdata16_2;

				U_SetLimit(motor.mode);
				break;
			}
			case U_REG_IQ_LIMIT:
			{
				int16_t regdata16_1 = *(int16_t *)data;
				if (regdata16_1 < -NOMINAL_CURRENT)
					regdata16_1 = -NOMINAL_CURRENT;
				motor.curLimit[0] = (int32_t)regdata16_1;

				int16_t regdata16_2 = *(int16_t *)(data+2);
				if (regdata16_1 > NOMINAL_CURRENT)
					regdata16_1 = NOMINAL_CURRENT;
				motor.curLimit[1] = (int32_t)regdata16_2;

				U_SetLimit(motor.mode);
				break;
			}
			case MC_REG_SPEED_REF:
			{
				int16_t target = *(uint16_t *)data;
				data++; data++;
				uint16_t duration = *(int16_t* )data;
				motor.velocity = (int16_t)target;
				break;
			}
			case MC_REG_CURRENT_REF:
			{
				int16_t target = *(uint16_t *)data;
				data++; data++;
				uint16_t duration = *(int16_t* )data;
				motor.torque = (int16_t)target;
				break;
			}
		}
		break;
	}
	case U_TYPE_DATA_5BYTE:
		{
			switch(regID)
			{

			}
		}
	default:

	}
}

void USER_Get(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size) {
	MCI_Handle_t *pMCIN = &Mci[0];
	uint8_t retVal = MCP_CMD_OK;

	switch (typeID)
	{
	case TYPE_DATA_8BIT:
	{
		*size = 1;
		switch (regID)
		{
		case U_REG_MODE:
		{
			*data = motor.mode;
			break;
		}
		}
		break;
		break;
	}
	case TYPE_DATA_16BIT:
	{
		*size = 2;
		uint16_t regdata16 = *(uint16_t *)data;
		break;
	}
	case TYPE_DATA_32BIT:
	{
		uint32_t* regdata32 = (uint32_t *)data;
		*size = 4;
		switch (regID)
		{
			case MC_REG_SPEED_MEAS:
			{
				float target = MCI_GetAvrgMecSpeed_F(pMCI[0]);
				memcpy(regdata32, &target, *size*sizeof(uint8_t));
				break;
			}
			case MC_REG_I_Q_MEAS:
			{
				float target = MCI_GetIqd_F(pMCI[0]).q;
				memcpy(regdata32, &target, *size*sizeof(uint8_t));
				break;
			}
			case MC_REG_I_D_MEAS:
			{
				float target = MCI_GetIqd_F(pMCI[0]).d;
				memcpy(regdata32, &target, *size*sizeof(uint8_t));
				break;
			}
			case U_REG_SPEED_LIMIT:
			{
				int16_t regdata16_1 = (int16_t)motor.velLimit[0];
				int16_t regdata16_2 = (int16_t)motor.velLimit[1];
				*(int16_t *)(data) = regdata16_1;
				*(int16_t *)(data+2) = regdata16_2;
				break;
			}
			case U_REG_IQ_LIMIT:
			{
				int16_t regdata16_1 = (int16_t)motor.curLimit[0];
				int16_t regdata16_2 = (int16_t)motor.curLimit[0];
				*(int16_t *)(data) = regdata16_1;
				*(int16_t *)(data+2) = regdata16_2;
				break;
			}
		}
		break;
	}
	default:

	}
}

void FDCAN_RxCallback2() {
	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_ERROR) {
		return;
	}

	//FDCAN_Soft_FifoQ_Test(0x89, 1, 1);
	// FDCAN_Soft_FifoQ(0x04, RxHeader.DataLength, RxData);

	if (RxHeader.Identifier != motor.this_id) return;

	uint8_t MCPResponse;
	uint8_t command = RxData[0] >> 4;
	uint8_t typeID = TypeID(RxData[0] & 0x0F);
	uint16_t regID;
	uint8_t *data = &RxData[3];
	uint16_t size;
	uint16_t dataAvailable = size;
	uint8_t textbuf[100] = {0,};

	memcpy(&regID, &RxData[1], 2);

	switch (command)
	    {
		  case 0x00: // Response
		  {
			  RxData[3]= motor.this_id;
			  FDCAN_Soft_FifoQ(motor.this_id-1, 4, RxData);
			  break;
		  }
	      case 0x01: // SET_DATA_ELEMENT
	      {
	        MCPResponse = RI_SetRegisterMotor1(regID, typeID, data, &size, dataAvailable);
	        break;
	      }

	      case 0x02: //GET_DATA_ELEMENT:
	      {
	    	if (typeID == TYPE_DATA_STRING) {
	    		MCPResponse = RI_GetRegisterMotor1(regID, typeID, textbuf, &size, sizeof(textbuf));
	    		for (int8_t cur = 0; cur <= size; cur += 4) {
	    			RxData[3] = size - cur;
					memcpy(&RxData[4], &textbuf[cur], 4);
					FDCAN_Soft_FifoQ(motor.this_id-1, 8, RxData);
	    		}
	    	}
	    	else if (typeID == TYPE_DATA_RAW) {
	    		MCPResponse = RI_GetRegisterMotor1(regID, typeID, textbuf, &size, sizeof(textbuf));
	    		memcpy(&size, textbuf, 2);
	    		for (int16_t cur = 2; cur <= size+2; cur += 3) {
					int16_t dif = size+2 - cur;
					memcpy(&RxData[3], &dif, 2);
					memcpy(&RxData[5], &textbuf[cur], 3);
					FDCAN_Soft_FifoQ(motor.this_id-1, 8, RxData);
				}
	    	}
	    	else {
	    		MCPResponse = RI_GetRegisterMotor1(regID, typeID, &RxData[3], &size, dataAvailable+10);
	    		FDCAN_Soft_FifoQ(motor.this_id-1, 3+size, RxData);
	    	}
	        break;
	      }

	      case 0x03: //START_MOTOR:
	      {
	        MCPResponse = (MCI_StartMotor(pMCI[0]) == true) ? MCP_CMD_OK : MCP_CMD_NOK;
	        RxData[3] = MCPResponse;
	        FDCAN_Soft_FifoQ(motor.this_id-1, 4, RxData);
	        break;
	      }

	      case 0x04: //STOP_MOTOR: /* Todo: Check the pertinance of return value */
	      {
	        (void)MCI_StopMotor(pMCI[0]);
	        MCPResponse = MCP_CMD_OK;
	        RxData[3] = MCPResponse;
	        FDCAN_Soft_FifoQ(motor.this_id-1, 4, RxData);
	        break;
	      }

	      case 0x05: // USER SET
	      {
	    	  USER_Set(regID, typeID, &RxData[3], &size);
	    	  if (size > 0) {
	    		  FDCAN_Soft_FifoQ(motor.this_id-1, 3+size, RxData);
	    	  }
	    	  break;
	      }

	      case 0x06: // USER GET
		  {
			  USER_Get(regID, typeID, &RxData[3], &size);
			  FDCAN_Soft_FifoQ(motor.this_id-1, 3+size, RxData);
			  break;
		  }

	      case 0x07: //FAULT_ACK:
	      {
	        (void)MCI_FaultAcknowledged(pMCI[0]);
	        MCPResponse = MCP_CMD_OK;
	        uint32_t fa = MCI_GetFaultState(pMCI[0]);
		    memcpy(&RxData[3], &fa, 4);
		    RxData[0] = 0x22; RxData[1] = 0x18; RxData[2] = 0x00;
		    FDCAN_Soft_FifoQ(motor.this_id-1, 7, RxData);
	        break;
	      }

/*	      case 0x08: // IQDREF_CLEAR:
	      {
	        MCI_Clear_Iqdref(pMCI[0]);
	        MCPResponse = MCP_CMD_OK;
	        break;
	      }*/

	      default :
	      {
	        MCPResponse = MCP_CMD_UNKNOWN;
	        break;
	      }
	    }
	// FDCAN_Soft_FifoQ_Test(0x87, 1, MCPResponse);
}

/* USER CODE END 1 */
