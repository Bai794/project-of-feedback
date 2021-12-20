/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dianji.h"
#include "key.h"
#include "ztjs.h"
#define mytrans(a) a / 100.f
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void running(void);
void test(void);
void pattern(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float jiaodu_temp[3];
float jiado2[3];
uint8_t data[8] = {0}; // 参数命令数组
uint8_t key = 0;
int Origin_qc4, Origin_qc5, Origin_qc6;
int x, y, z, fl = 100, gravity1 = 400, gravity2 = 410, gravity3 = 300, rst = 0, flag = 1, status = 0;
long Origin_qc1, Origin_qc2, Origin_qc3, electricity1, electricity2, electricity3; //两次编码器差值计算
unsigned char len = 0;
long dif_qc1 = 0, dif_qc2 = 0, dif_qc3 = 0;
// float real_angle1 = 0, real_angle2 = 0, real_angle3 = 0, dif_angle1 = 0, dif_angle2 = 0, dif_angle3 = 0; //推导真实编码器角度
long real_angle1 = 0, real_angle2 = 0, real_angle3 = 0, dif_angle1 = 0, dif_angle2 = 0, dif_angle3 = 0, angle1 = 0, angle2 = 0, angle3 = 0; //推导真实编码器角度
long rst_qc1, rst_qc2, rst_qc3;                                                                                                             //复位编码器qc
double man[7];
unsigned char flag_tp = 1;
unsigned char mode;
unsigned char flag_tx = 0;
uint32_t ID = 0;
uint16_t encoder_4 = 0, encoder_5 = 0, encoder_6 = 0;
uint16_t angle_4, angle_5, angle_6;
uint8_t Rxd_Data[8]; // can中断接收数组（全局变量）
uint8_t flag1 = 1, flag2 = 1, flag3 = 1;
// unsigned char falg_m1;
// unsigned char falg_m2;
// unsigned char falg_m3;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) // 1s
  {

    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  if (htim->Instance == TIM4) // 20ms
  {

    // running();
    // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  if (htim->Instance == TIM6) // 10ms
  {
    flag_tx++;
    if (flag_tx == 1) //无法同时接收三个数据，只能两个两个发
    {
      Read_Angle_Cmd(4);
    }
    else if (flag_tx == 2)
    {
      Read_Angle_Cmd(6);
    }
    else if (flag_tx == 3)
    {
      flag_tx = 0;
      Read_Angle_Cmd(5);
    }
  }
}

void mysprintf_toHMI()
{
  char x_str[100], t4_str[100];
  uint8_t end_hex[] = {0xff, 0xff, 0xff};
  sprintf(x_str, "t3.txt=\"x:%0.2f y:%0.2f z:%0.2f\"", PM.x0, PM.y0, PM.z0 + 100);
  int num = 0;
  while (x_str[num] != '\0')
  {
    num++;
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)x_str, num, 0xff);
  HAL_UART_Transmit(&huart2, end_hex, 3, 0xffff);

  sprintf(t4_str, "t4.txt=\"roll:%0.2f pitch:%0.2f yaw:%0.2f \"", (angle_6 - Origin_qc6) / 100.0f,
          (angle_5 - Origin_qc5) / 100.0f,
          (angle_4 - Origin_qc4) / 100.0f);
  int num1 = 0;
  while (t4_str[num1] != '\0')
  {
    num1++;
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)t4_str, num1, 0xff);
  HAL_UART_Transmit(&huart2, end_hex, 3, 0xffff);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  printf("-----------------设备初始化中-----------\n");
  HAL_CAN_Start(&hcan);
  CAN1_Config();                                                    //设置滤波
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // Enable interrupts
  printf("-----------------CAN通信初始化成功-----------\n");

  CAN_RoboModule_DRV_Reset(0, 0);                              //对0组所有驱动器进行复位
  HAL_Delay(500);                                              //发送复位指令后的延时必须要有，等待驱动器再次初始化完成
  CAN_RoboModule_DRV_Config(0, 1, 15, 0);                      // 1号驱动器配置为15ms传回一次数据
  HAL_Delay(200);                                              //此处延时为了不让传回数据时候4个不一起传
  CAN_RoboModule_DRV_Config(0, 2, 15, 0);                      // 2号驱动器配置为15ms传回一次数据
  HAL_Delay(200);                                              //此处延时为了不让传回数据时候4个不一起传
  CAN_RoboModule_DRV_Config(0, 3, 15, 0);                      // 3号驱动器配置为15ms传回一次数据
  CAN_RoboModule_DRV_Mode_Choice(0, 0, Current_Position_Mode); // 0组的所有驱动器 都进入电流位置模式
  HAL_Delay(500);                                              //发送模式选择指令后，要等待驱动器进入模式就绪。所以延时也不可以去掉。
  /*********************************************+*************************************/
  Origin_qc1 = Real_Position_Value[0]; //记录初始位置
  Origin_qc2 = Real_Position_Value[1];
  Origin_qc3 = Real_Position_Value[2];
  Read_Angle_Cmd(4);
  HAL_Delay(100);
  Read_Angle_Cmd(6);
  HAL_Delay(100);
  Read_Angle_Cmd(5);
  HAL_Delay(100);
  Origin_qc4 = angle_4;
  Origin_qc5 = angle_5;
  Origin_qc6 = angle_6;
  rst_qc1 = Origin_qc1;
  rst_qc2 = Origin_qc2;
  rst_qc3 = Origin_qc3;
  jiaodu_temp[0] = angle_4 / 100.0f;
  jiaodu_temp[1] = angle_5 / 100.0f;
  jiaodu_temp[2] = angle_6 / 100.0f;
  //控制上面三个电机
  // Close_Clear_Cmd(4);
  // Close_Clear_Cmd(5);
  // Close_Clear_Cmd(6);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // int num_postion1=0,num_postion2=0,num_postion3=0;
  /*************以下while(1)是平台初始化调平的动作 通过光电传感器反馈检测其水平 *****************/
  //		while(1){
  //			 if(M_Z_1 == 1 && M_Z_2 == 1 && M_Z_3 == 1)
  //			 {
  //				 break;
  //			 }
  //			 //printf("%ld  %ld %ld\r\n",Real_Position_Value[0],Real_Position_Value[1],Real_Position_Value[2]);
  //			 if(M_Z_1==1){
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 0, 2000, num_postion1);
  //			 }
  //			 else{
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 0, 2000, num_postion1);
  //				  num_postion1+=5;
  //			 }
  //			  if(M_Z_2==1){
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 1, 2000, num_postion2);
  //			 }
  //			 else{
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 1, 2000, num_postion2);
  //				  num_postion2+=5;
  //			 }//			  if(M_Z_3==1){
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 3, 2000, num_postion3);
  //			 }
  //			 else{
  //				  CAN_RoboModule_DRV_Current_Position_Mode(0, 3, 2000, num_postion3);
  //				  num_postion3+=5;
  //			 }
  //			 HAL_Delay(10);
  //		}
  rst_qc1 = Real_Position_Value[0]; //记录复位水平位置的编码器数量
  rst_qc2 = Real_Position_Value[1];
  rst_qc3 = Real_Position_Value[2];
  HAL_TIM_Base_Start_IT(&htim6); //开启定时器6中断
  printf("-----------------设备初始化成功-----------\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    run();             //位置解算 并通过usart1 发送给上位机vr
    test();            // test();//该函数用于发给pc打印坐标
    mysprintf_toHMI(); //将坐标发送个串口屏
                       // printf("angle_4:%ld  angle_5:%ld angle_6:%ld\r\n\r\n", angle_4,angle_5,angle_6);
    //  Read_Angle_Cmd(4);
    //  HAL_Delay(100);
    //   Read_Angle_Cmd(6);
    // printf("%d  %d  %d %d\r\n",encoder_5,encoder_5,encoder_6,speed4);
    /*****************以下通过按键来选择模式***********************/
    key = KEY_Scan(0); //得到键值
    switch (key)
    {
    case KEY0_PRES:
      flag = !flag;
      mode = 1;
      // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      break;
    case KEY1_PRES:
      //__enable_irq();//打开所有中断
      mode = 2;
      break;
    case WKUP_PRES:
      mode = 3;
      break;
    }
    pattern(); //根据不同的模式给电机发指令
               // zitaijiesuan((angle_6 - Origin_qc6) / 100, (angle_5 - Origin_qc5) / 100, (angle_4 - Origin_qc4) / 100); //上面三个自由度的解算
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void test()
{
  printf("X:%0.2f Y:%0.2f Z:%0.2f \r\n", PM.x0, PM.y0, PM.z0);
  printf("roll:%0.2f pitch :%0.2f yaw:%0.2f \r\n", (angle_6 - Origin_qc6) / 100.0f,
         (angle_5 - Origin_qc5) / 100.0f,
         (angle_4 - Origin_qc4) / 100.0f);
}
void running()
{
  if (flag == 0) //按key_0切换为运行模式，将数值传至上位机，运行阶段
  {

    //得到差值并保存
    dif_qc1 = Real_Position_Value[0] - Origin_qc1;
    // Origin_qc1=Real_Position_Value[0];
    dif_qc2 = Real_Position_Value[1] - Origin_qc2;
    // Origin_qc2= Real_Position_Value[1];
    dif_qc3 = Real_Position_Value[2] - Origin_qc3;
    //	Origin_qc3= Real_Position_Value[2];
    //得到真实角度差
    dif_angle1 = dif_qc1 * 0.18;
    dif_angle2 = dif_qc2 * 0.18;
    dif_angle3 = dif_qc3 * 0.18;
    //计算真实角度
    real_angle1 = dif_angle1;
    real_angle2 = dif_angle2;
    real_angle3 = dif_angle3;
    delta_calcForward(real_angle1 / 13.3 + 31.4, real_angle2 / 13.3 + 31.4, real_angle3 / 13.3 + 31.4);
    DATE();
    printf("%010d\n", str);
  }
}
void pattern()
{
  if (mode == 1)
  {
    map(); //位置和电流的运算 用于补偿重力
    CAN_RoboModule_DRV_Current_Position_Mode(0, 1, gravity1 + 200, Origin_qc1 + 4000);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 2, gravity2 + 200, Origin_qc2 + 4000);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 3, gravity3 + 200, Origin_qc3 + 4000);
  }
  else if (mode == 2)
  { //悬停

    CAN_RoboModule_DRV_Current_Position_Mode(0, 1, 4000, Real_Position_Value[0]);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 2, 4000, Real_Position_Value[0]);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 3, 4000, Real_Position_Value[0]);
  }
  else if (mode == 3)
  { //归位
    CAN_RoboModule_DRV_Current_Position_Mode(0, 1, 4000, rst_qc1);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 2, 4000, rst_qc2);
    CAN_RoboModule_DRV_Current_Position_Mode(0, 3, 4000, rst_qc3);
  }
  else
    return;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
