/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint16_t u16_ADCScanVal[5];
int sum_ADC=0;
uint32_t red_value=0, green_value=0, blue_value=0;
int red=0, green=0;

uint32_t current_time=0, time_A=0, time_B=0;
uint32_t time=0, duration=0, time_start_feedback=0, duration_feedback=0, duration_feedback_;
long count_A = 0, count_B = 0;
float rpm_A = 0, rpm_B = 0, expected_rpm_A=0, expected_rpm_B=0, duty_cycle_A=0, duty_cycle_B=0, v_A=0, v_B=0;


float error_rpm=0, cum_error_A=0, cum_error_B=0, error_prev_A=0, error_prev_B=0;
float error_filtered_A = 0, error_filtered_B = 0, rate_error = 0, duty_cycle = 0;
float sample_time = 0.05,  T_filter=0.1, feedback_time=0.05; // s
float kp_A=0.47, ki_A=5.26, kd_A=-0.004;
float kp_B=0.47, ki_B=5.26, kd_B=-0.004;
float k1 = 0.05/200*0.5, k2 = 0.1/200*0.5, k3 = 2/200*0.5, vRef=650,wRef=0;
	
int isStarted=0;
int sign = 1;
float e2=0, e2_prev=0, e3=0;
float v=0, w=0, v_fb=0, v_prev=0;
float R=96/2.0, L=240, pi=3.1415;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void color_recognize(){
	for(int i=0; i<3; i++){
			switch (i){
				case 0:
					HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
					HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
					red_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
					HAL_Delay(1);
					break;
				case 1:
					HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
					HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
					blue_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
					HAL_Delay(1);
					break;
				default:
					HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
					HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
					green_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
					HAL_Delay(1);
			}
		}
		if (red_value<800 && green_value>600){
			green=1;
			red=0;
		}
		else if (red_value>800 && green_value<700 && red_value<1000){
			green=0;
			red=1;
		}
		else {
			green=0;
			red=0;
		}
}


// tinh gia tri encoder

int convert_2_sign_int(int counter){
	if (counter >  32767) return counter - 65535;
	else return counter;
}

// tinh rpm
void calculate_rpm(int motor_id){
	current_time = HAL_GetTick();
	if (motor_id==1){
		time_A = current_time-time_A;
		count_A = __HAL_TIM_GET_COUNTER(&htim1);
		count_A = convert_2_sign_int(count_A);
		rpm_A = (float)count_A*1000.0/((float)time_A*374)*60/4;
		v_A = (rpm_A/60.0)*2*R*pi;
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		time_A = HAL_GetTick();
	}
	if (motor_id==2){
		time_B = current_time-time_B;
		count_B = __HAL_TIM_GET_COUNTER(&htim2);
		count_B = convert_2_sign_int(count_B);
		rpm_B = (float)count_B*1000.0/((float)time_B*374)*60/4;
		v_B = (rpm_B/60.0)*2*R*pi;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		
		time_B = HAL_GetTick();
	}
}


float control_motor_A(float rpm_motor, float expected_rpm){
		error_rpm = expected_rpm-rpm_motor;
		cum_error_A += error_rpm*feedback_time;
		rate_error = (error_rpm-error_prev_A)/feedback_time;
//		error_filtered_A = error_filtered_A*(1-feedback_time/T_filter)+(feedback_time/T_filter)*error_rpm;
//		rate_error = (error_rpm-error_filtered_A)/T_filter;
		duty_cycle = kp_A*error_rpm + ki_A*cum_error_A+ kd_A*rate_error;//+ kd*(error_rpm-error_prev) + ki*cum_error+ kd*rate_error
		error_prev_A = error_rpm;
//		
	if (duty_cycle<-100) return -100;
	if (duty_cycle>100) return 100;
	return duty_cycle;
}

float control_motor_B(float rpm_motor, float expected_rpm){
		error_rpm = expected_rpm-rpm_motor;
		cum_error_B += error_rpm*feedback_time;
//		rate_error = (error_rpm-error_prev_B)/feedback_time;
		error_filtered_B = error_filtered_B*(1-feedback_time/T_filter)+(feedback_time/T_filter)*error_rpm;
		rate_error = (error_rpm-error_filtered_B)/T_filter;
		duty_cycle = kp_B*error_rpm + ki_B*cum_error_B+ kd_B*rate_error;//+ kd*(error_rpm-error_prev) + ki*cum_error+ kd*rate_error
		error_prev_B = error_rpm;
//		
	if (duty_cycle<-100) return -100;
	if (duty_cycle>100) return 100;
	return duty_cycle;
}

// 
float compute_e2(){
	sum_ADC=0;
	for (int i=0; i<5;i++){
		sum_ADC+= (u16_ADCScanVal[i]-200);
	}
	if (u16_ADCScanVal[3]>2400 && u16_ADCScanVal[1]>2400)
		e2 = -15;
	else
		e2=((u16_ADCScanVal[0]-u16_ADCScanVal[4])*34+(u16_ADCScanVal[1]-u16_ADCScanVal[3])*17)/(float)sum_ADC;
	return e2;
}


float abs_(float a){
	if(a<0)
		return -a;
	else
		return a;
}

float sign_(float a){
	if (a<0)
		return -1;
	else
		return 1;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	// cam bien mau
	HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 1);
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
	
	// cam bien do line
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)u16_ADCScanVal, 5);
	
	
	// dong co
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	TIM3->CCR1 = 0; 
	TIM3->CCR2 = 0;
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
	// encoder
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_Delay(1000);
	TIM3->CCR1=50;
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
	TIM3->CCR2=50;
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
	HAL_Delay(1000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		duration = 0;
		time = HAL_GetTick();
		color_recognize();
		
		calculate_rpm(1); //motor A
		calculate_rpm(2);
		v_fb = (v_A+v_B)/2;
		
		e2 = compute_e2();
		
		if(v==0)
			e3=0;
		else
			e3=(e2-e2_prev)/(0.5*(v_fb+v_prev)*sample_time);
		
		k1 = 0.05/200*0.5;
    k2 = 0.1/200*0.5;
    k3 = 2/200*0.5;
    
    wRef = -e3; 
    if (abs_(wRef)>pi/2){
        wRef = pi/2*sign_(wRef);
    }
		
    v = vRef*cos(e3);
    w = k2*vRef*e2 + wRef + k3*sin(e3);
    
    if (abs_(v-v_fb)>50)
			v = v_fb + 50*sign_(v-v_fb);
    		
//		expected_rpm_A=50;
//		expected_rpm_B=50;
		expected_rpm_A = 1.0/2*(w*L+2*v)/(pi*2*R)*60; // banh phai
    expected_rpm_B = 1.0/2*(2*v-w*L)/(pi*2*R)*60; // banh trai
		
		duty_cycle_A = control_motor_A(rpm_A, expected_rpm_A);
		duty_cycle_B = control_motor_B(rpm_B, expected_rpm_B);
	
		if (u16_ADCScanVal[0]<1500 && u16_ADCScanVal[1] <1500 && u16_ADCScanVal[2]<1500 && u16_ADCScanVal[3]<1500 && u16_ADCScanVal[4]<1500){
			TIM3->CCR1=0;
			TIM3->CCR2=0;
			HAL_Delay(100000);
			
		}
		if (duty_cycle_A<0){
			HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 1);
			HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
			duty_cycle_A = -duty_cycle_A;
		}
		else{
			HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
			HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
		}
		TIM3->CCR1 = (int)duty_cycle_A;
		
		
		if (duty_cycle_B<0){
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
			HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
			duty_cycle_B = -duty_cycle_B;
		}
		else{
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
			HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
		}
		TIM3->CCR2 = (int)duty_cycle_B;
		
		
		e2_prev = e2;
		v_prev = v_fb;
		duration= HAL_GetTick()-time;
		HAL_Delay((int)(sample_time*1000)-duration);

		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B2_Pin|B1_Pin|A1_Pin|A2_Pin
                          |S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B2_Pin B1_Pin A1_Pin A2_Pin
                           S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B1_Pin|A1_Pin|A2_Pin
                          |S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
