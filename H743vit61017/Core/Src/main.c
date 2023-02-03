/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "motor.h"
#include "Dataex.h"
#include "control.h"
#include  "math.h"
#include "key.h"
#include "Ctrl_schedule.h"
#include "FC_PID.h"
#include "Inv_Kine.h"
#include "target.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*�ڴ�0x38000000�ϵı�������DMA��������ʹ�� */
ALIGN_32BYTES(__attribute__((section(".RAM_D3"))) uint32_t ADC3_Data[2]);       /*����ͨ��������*/
ALIGN_32BYTES(__attribute__((section(".RAM_D3"))) uint32_t ADC1_Data[6]);       /*����ͨ��������*/
/* DTCNM�ڴ��ȫ�ֱ��� */

float Temp = 0;
float Vrefint = 0;
float adc3In1 = 0;
float Curent1 = 0;
float Curent2 = 0;
float Curent3 = 0;
float Curent4 = 0;
float Curent5 = 0;
float Curent6 = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

float turn_CPU_Temperature(uint32_t Temp_Value);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Position_zeroed_Flag = 0;
uint8_t Key_contrl_flag = 0;
uint8_t target_followed_flag = 0;
uint8_t receive_buff;
uint8_t receive_buff_JY901;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 /*   ���ᴫ��������iic   */
		//�ô���2��JYģ�鷢��ָ��
		void sendcmd(char cmd[])
		{
		char i;
		for(i=0;i<5;i++)
		HAL_UART_Transmit(&huart3,(uint8_t*)cmd+i, 1, 10);
		}
 /*   ���ᴫ��������iic   */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//////////////////
	MX_ADC1_Init();//Ҫ����MX_DMA_Init();���棬����DMA�����ã�����
	
	/************************��������****************************************/

	Position_zeroed_Flag = 0;       //���ԣ��ֶ�У׼λ�ñ�־
	HAL_TIM_Base_Start_IT(&htim6); //�ж�ʱ��ʹ���ṩ����ʱ��
	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);  //����tim12ͨ��1   PB14���pwm
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);  //����tim12ͨ��2   PB15���pwm
	HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);  //����tim13ͨ��1   PA6���pwm
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);  //����tim14ͨ��1   PA7���pwm
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);  //����tim15ͨ��1   PE5���pwm
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);  //����tim15ͨ��2   PE6���pwm
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   //ʹ������������ PE9 PE11
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   //ʹ������������ PA15 PB3
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   //ʹ������������ PB4 PB5
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   //ʹ������������ PD12 PD13     
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   //ʹ������������ PA0 PA1
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);   //ʹ������������ PC6 PC7      

  /////////****�����ж�����*******///////
	HAL_UART_Receive_IT(&huart1 , &receive_buff ,1);
	HAL_UART_Receive_IT(&huart3 , (uint8_t*)&receive_buff_JY901 ,1 );
	
  /////////****ADC+DMA����*******///////
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)ADC3_Data,2);  /* ����DMA�������� */
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_Data,6);  /* ����DMA�������� */
	
	/************************��ʼ��****************************************/
	
	Motor_Init();    //����ǶȲ�����ʼ��
	Control_Init();  //PID������ʼ��
	
  /**********ģ��pid��ʼ��***********/
	fuzzy_controler_Init(&FC_M1);
	fuzzy_controler_Init(&FC_M3);
	fuzzy_controler_Init(&FC_M4);
	fuzzy_controler_Init(&FC_M6);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	/************************�������в�������****************************************/
	
	///***ǰ���Ƕȼ���******/////
	motor_Angle_d_Caculate(3, &Left_InvKine, 0);   //���һ������0~4����4�ֹ켣����
	motor_Angle_d_Caculate(3, &Right_InvKine, 0);
	
	//ǰ����������
	forward_Value(Left_InvKine.angle_M1_d, Left_InvKine.angle_M1_dv,2000);
	forward_Value(Left_InvKine.angle_M2_d, Left_InvKine.angle_M2_dv,3000);
	forward_Value(Left_InvKine.angle_M3_d, Left_InvKine.angle_M3_dv,2000);
	forward_Value(Right_InvKine.angle_M1_d, Right_InvKine.angle_M1_dv,2000);
	forward_Value(Right_InvKine.angle_M2_d, Right_InvKine.angle_M2_dv,3000);
	forward_Value(Right_InvKine.angle_M3_d, Right_InvKine.angle_M3_dv,2000);
	
	//ǰ���������ݷ��͵���λ��
	uint16_t transfer_cnt = 0;
	for(transfer_cnt = 0;transfer_cnt<3000;transfer_cnt++)
	{
		printf("data:%d\n",(int)(Right_InvKine.angle_M1_dv[transfer_cnt]));
	}
	//PID��������
	Pid_Set(&Motor1pid,500,0,0);
	Pid_Set(&Motor2pid,500,0,0);
	Pid_Set(&Motor3pid,500,0,0);
	Pid_Set(&Motor4pid,500,0,0.001);
	Pid_Set(&Motor5pid,500,0,5);
	Pid_Set(&Motor6pid,500,0,0.1);
	


  while(1)
  {
		HAL_Delay(5);	
		
		HAL_UART_Receive_IT(&huart3 , (uint8_t*)&receive_buff_JY901 ,1 );
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
		while(Position_zeroed_Flag == 0)
		{
//			Position_zeroed_Flag = Key_Scan();
			//�����������
      Motor_Adusted_from_PC();
			
			HAL_Delay(5);
			
		}
		if(Position_zeroed_Flag)
		{
//			HAL_Delay(20);
//			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//			HAL_Delay(200);
//			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//			HAL_Delay(200);
//			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//			HAL_Delay(200);
//			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//			HAL_Delay(200);
//			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
//			HAL_Delay(1000);
//			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
//			HAL_Delay(200);
		}	
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





float turn_CPU_Temperature(uint32_t Temp_Value)
{
	
	uint16_t TS_CAL1;
	uint16_t TS_CAL2;
	float Temp_oC = 0;

	/* ���ݲο��ֲ���Ĺ�ʽ�����¶�ֵ */
    TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
    TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);

    Temp_oC = ((110.0f - 30.0f) / (TS_CAL2 - TS_CAL1)) * (Temp_Value - TS_CAL1) + 30.0f;

    return Temp_oC;
}






/****************************���˶�ѧ���� ****************************   
SysTick->VAL = 0;
int start_time=SysTick->VAL;
//���Լ�����Ҫ��ʱ��  int end_time = SysTick->VAL;
int i=0;
for(i=0;i<100;i++)
{
Inv_Kine_Caculator(-3.14+0.02*i,-0.52+0.01*i,-0.52+0.01*i,&Left_InvKine);
printf("%d\n",(int)(Left_InvKine.theta1*1000));
printf("%d\n",(int)(Left_InvKine.theta2*1000));
printf("%d\n",(int)(Left_InvKine.theta3*1000));
}
int end_time = SysTick->VAL;
printf("%d\n",end_time);

***************************** ���˶�ѧ���� **************************/

/**************************** �������˶�ѧ���ɵĹ켣�Է� *************************

uint8_t falg1234;
for(falg1234=0;falg1234<4;falg1234++)
{
	int i;
	motor_Angle_d_Caculate(3, &Left_InvKine, falg1234);
	motor_Angle_d_Caculate(3, &Right_InvKine, falg1234);
	for(i=0;i<3000;i++)
	{
		inv_To_PC();
	}
}
**************************** �������˶�ѧ���ɵĹ켣�Է� *************************/






/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

