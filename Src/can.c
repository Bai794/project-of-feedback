/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
uint16_t speed4, speed5, speed6;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN1_Config(void)
{
    CAN_FilterTypeDef CAN_FilterType;
    CAN_FilterType.FilterBank = 0;
    CAN_FilterType.FilterIdHigh = 0xFFFF;
    CAN_FilterType.FilterIdLow = 0xFFFF;
    CAN_FilterType.FilterMaskIdHigh = 0x0000; //(((uint32_t)0x1313<<3)&0xFFFF0000)>>16;
    CAN_FilterType.FilterMaskIdLow = 0x0000;  //(((uint32_t)0x1313<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;
    CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation = ENABLE;
    CAN_FilterType.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterType) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
}
int Can_TxMessage(uint8_t ide, uint32_t id, uint8_t len, uint8_t *data)
{
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    HAL_StatusTypeDef HAL_RetVal;
    uint16_t i = 0;
    if (ide == 0)
    {
        CAN_TxHeader.IDE = CAN_ID_STD; //标准帧
        CAN_TxHeader.StdId = id;
    }
    else
    {
        CAN_TxHeader.IDE = CAN_ID_EXT; //扩展帧
        CAN_TxHeader.ExtId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA; //数据帧,CAN_RTR_REMxx	OTE遥控帧
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        i++;
        if (i > 0xfffe)
            return 1;
    }

    HAL_RetVal = HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, data, &TxMailbox);
    if (HAL_RetVal != HAL_OK)
        return 1;
    return 0;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    HAL_StatusTypeDef HAL_Retval;
    uint8_t Rx_Data[8];
    uint8_t Data_Len = 0;

    //     uint8_t i;
    HAL_Retval = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, Rx_Data);
    memcpy(Rxd_Data, Rx_Data, 8);
    if (HAL_Retval == HAL_OK) //接收数据
    {
        Data_Len = CAN_RxHeader.DLC;
        if (CAN_RxHeader.IDE)
            ID = CAN_RxHeader.ExtId;
        else
            ID = CAN_RxHeader.StdId;

        if ((CAN_RxHeader.IDE == 0) && (Data_Len == 8)) //标准帧、数据帧、数据长度为8
        {

            if (CAN_RxHeader.StdId == 0x1B)
            {

                Real_Current_Value[0] = (Rx_Data[0] << 8) | (Rx_Data[1]);
                Real_Velocity_Value[0] = (Rx_Data[2] << 8) | (Rx_Data[3]);
                Real_Position_Value[0] = ((Rx_Data[4] << 24) | (Rx_Data[5] << 16) | (Rx_Data[6] << 8) | (Rx_Data[7]));
                // printf("id:%d\r\n", Real_Velocity_Value[0]);
            }
            else if (CAN_RxHeader.StdId == 0x2B)
            {
                Real_Current_Value[1] = (Rx_Data[0] << 8) | (Rx_Data[1]);
                Real_Velocity_Value[1] = (Rx_Data[2] << 8) | (Rx_Data[3]);
                Real_Position_Value[1] = ((Rx_Data[4] << 24) | (Rx_Data[5] << 16) | (Rx_Data[6] << 8) | (Rx_Data[7]));
                // printf("%ld\r\n", Real_Position_Value[1]);
            }
            else if (CAN_RxHeader.StdId == 0x3B)
            {
                Real_Current_Value[2] = (Rx_Data[0] << 8) | (Rx_Data[1]);
                Real_Velocity_Value[2] = (Rx_Data[2] << 8) | (Rx_Data[3]);
                Real_Position_Value[2] = ((Rx_Data[4] << 24) | (Rx_Data[5] << 16) | (Rx_Data[6] << 8) | (Rx_Data[7]));
            }
            else if (CAN_RxHeader.StdId == 0x4B)
            {
                Real_Current_Value[3] = (Rx_Data[0] << 8) | (Rx_Data[1]);
                Real_Velocity_Value[3] = (Rx_Data[2] << 8) | (Rx_Data[3]);
                Real_Position_Value[3] = ((Rx_Data[4] << 24) | (Rx_Data[5] << 16) | (Rx_Data[6] << 8) | (Rx_Data[7]));
            }
            else if (CAN_RxHeader.StdId == 0x1F)
            {
                Real_Online[0] = 1;
            }
            else if (CAN_RxHeader.StdId == 0x2F)
            {
                Real_Online[1] = 1;
            }
            else if (CAN_RxHeader.StdId == 0x3F)
            {
                Real_Online[2] = 1;
            }
            else if (CAN_RxHeader.StdId == 0x4F)
            {
                Real_Online[3] = 1;
            }
            else if (CAN_RxHeader.StdId == 0x1C)
            {
                Real_Ctl1_Value[0] = Rx_Data[0];
                Real_Ctl2_Value[0] = Rx_Data[1];
            }
            else if (CAN_RxHeader.StdId == 0x2C)
            {
                Real_Ctl1_Value[1] = Rx_Data[0];
                Real_Ctl2_Value[1] = Rx_Data[1];
            }
            else if (CAN_RxHeader.StdId == 0x3C)
            {
                Real_Ctl1_Value[2] = Rx_Data[0];
                Real_Ctl2_Value[2] = Rx_Data[1];
            }
            else if (CAN_RxHeader.StdId == 0x4C)
            {
                Real_Ctl1_Value[3] = Rx_Data[0];
                Real_Ctl2_Value[3] = Rx_Data[1];
            }
            /**/
            // angle_4,angle_5,angle_6;
            if (Rx_Data[0] == 0x94) //读取角度
            {
                if (ID == 0x144)
                {

                    angle_4 = Rx_Data[7];
                    angle_4 = angle_4 << 8;
                    angle_4 = angle_4 | Rx_Data[6];
                    // printf("angle_4:%ld\r\n\r\n", angle_4);
                }
                if (ID == 0x145)
                {

                    angle_5 = Rx_Data[7];
                    angle_5 = angle_5 << 8;
                    angle_5 = angle_5 | Rx_Data[6];
                    // printf("angle_5:%ld\r\n\r\n", angle_5);
                }
                if (ID == 0x146)
                {
                    angle_6 = Rx_Data[7];
                    angle_6 = angle_6 << 8;
                    angle_6 = angle_6 | Rx_Data[6];
                    //printf("angle_6:%ld\r\n\r\n", angle_6);
                }
            }
            //////////////////////////////////////////////////////////////////
            if (Rx_Data[0] == 0x9c) //读取编码器值
            {
                if (ID == 0x144)
                {

                    encoder_4 = Rx_Data[7];
                    encoder_4 = encoder_4 << 8;
                    encoder_4 = encoder_4 | Rx_Data[6];
                    // printf("encoder_4:%ld\r\n\r\n", encoder_4);
                }
                if (ID == 0x145)
                {

                    encoder_5 = Rx_Data[7];
                    encoder_5 = encoder_5 << 8;
                    encoder_5 = encoder_5 | Rx_Data[6];
                    // printf("encoder_5:%ld\r\n\r\n", encoder_5);
                }
                if (ID == 0x146)
                {

                    encoder_6 = Rx_Data[7];
                    encoder_6 = encoder_6 << 8;
                    encoder_6 = encoder_6 | Rx_Data[6];
                    // printf("encoder_6:%ld\r\n\r\n", encoder_6);
                }
            }
            //            if(ID == 0x144)
            //            {
            //                //printf("CAN_RxHeader.StdId:%x\r\n", CAN_RxHeader.StdId );
            //                if(Rx_Data[0] == 0x9c)//读取当前5电机的温度、电压、转速、编码器位置。
            //                {
            //                    /*
            //                    Rx_Data[6];//编码器位置低字节
            //                    Rx_Data[7];//编码器位置高字节
            //                    */
            //                    //printf("id:%x\r\n", ID);
            //                    encoder_4 =  Rx_Data[7];
            //                    encoder_4 = encoder_4 << 8;
            //                    encoder_4 = encoder_4 | Rx_Data[6];
            //                    //printf("encoder_4:%ld\r\n\r\n", encoder_4);
            //                }
            //            }
            //            if(ID == 0x145)
            //            {
            //                //printf("CAN_RxHeader.StdId:%x\r\n", CAN_RxHeader.StdId );
            //                if(Rx_Data[0] == 0x9c)//读取当前5电机的温度、电压、转速、编码器位置。
            //                {
            //                    /*
            //                    Rx_Data[6];//编码器位置低字节
            //                    Rx_Data[7];//编码器位置高字节
            //                    */
            //                    //printf("id:%x\r\n", ID);
            //                    encoder_5 =  Rx_Data[7];
            //                    encoder_5 = encoder_5 << 8;
            //                    encoder_5 = encoder_5 | Rx_Data[6];
            //                    //printf("encoder_5:%ld\r\n\r\n", encoder_5);
            //                }
            //            }
            //            if(ID == 0x146)
            //            {
            //                //printf("CAN_RxHeader.StdId:%x\r\n", CAN_RxHeader.StdId );
            //                if(Rx_Data[0] == 0x9c)//读取当前5电机的温度、电压、转速、编码器位置。
            //                {
            //                    /*
            //                    Rx_Data[6];//编码器位置低字节
            //                    Rx_Data[7];//编码器位置高字节
            //                    */
            //                    //printf("id:%x\r\n", ID);
            //                    encoder_6 =  Rx_Data[7];
            //                    encoder_6 = encoder_6 << 8;
            //                    encoder_6 = encoder_6 | Rx_Data[6];
            //                    //printf("encoder_6:%ld\r\n\r\n", encoder_6);
            //                }
            //            }
            //            printf("encoder_4:%ld encoder_5:%ld encoder_6:%ld \r\n", encoder_4,encoder_5,encoder_6);
            /**/
        }
    }
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
