/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//电机
#define COUNTS_PER_REV  1320
#define MAX_PWM         999
//循迹引脚
#define TRACK_PORT      GPIOA
#define TRACK_L1_PIN    GPIO_PIN_2   /* 最左 PA2 */
#define TRACK_L2_PIN    GPIO_PIN_3   /* 左内 PA3 */
#define TRACK_R1_PIN    GPIO_PIN_6   /* 右内 PA6 */
#define TRACK_R2_PIN    GPIO_PIN_7   /* 最右 PA7 */
#define LINE_ACTIVE     GPIO_PIN_SET   /* 压黑线 = 高电平 */
//循迹参数
#define TRACK_BASE_SPEED_RPM  273.0f /* 直道基础转速 rpm (= 60计数档, 与F键一致) */
#define TRACK_KP              45.0f  /* P: 误差 -> 差速rpm (等价旧计数单位KP=10) */
#define TRACK_KD              20.0f  /* D: 误差变化率 -> 差速rpm (等价旧计数单位KD=5) */
#define TRACK_MAX_DIFF_RPM    136.0f /* 左右差速限幅 rpm (等价旧30计数) */
#define TRACK_SLOW_AT         2      /* |误差|>=2 判定为弯道 */
#define TRACK_SLOW_DELTA_RPM  91.0f  /* 弯道降速 rpm (273->182, 即60档->40档) */
//循迹模式状态
volatile uint8_t track_mode     = 1;  /* 1=循迹中, 0=手动(原有F/B/L/R控制) */
volatile int16_t track_err_last = 0;  /* 上一次误差, 丢线时保持转向用 */
volatile int16_t track_err_prev = 0;  /* 上一次误差, PD的D项用 */



typedef struct {
    int16_t target;
    int16_t speed;
    int16_t last_cnt;
    float kp, ki, kd;
    float err, err_last;
    float integ;
    int16_t out;
} MOTOR_PID;

MOTOR_PID ml = {.kp=2.0f, .ki=2.0f, .kd=2.0f};
MOTOR_PID mr = {.kp=2.0f, .ki=2.0f, .kd=2.0f};

typedef enum { STOP=0, FORWARD, BACKWARD, LEFT, RIGHT,TRACK } MOVE_STATE;
volatile MOVE_STATE move_state = STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//电机函数原型
void Motor_Left_Set(int16_t duty);
void Motor_Right_Set(int16_t duty);
void pid_calc(MOTOR_PID *p, TIM_HandleTypeDef *htim);
void Motor_Stop(void);
void Motor_Forward(int16_t speed);
void Motor_Backward(int16_t speed);
void Motor_TurnLeft(int16_t speed);
void Motor_TurnRight(int16_t speed);
void Print_Speed(void);

/* ----【NEW】循迹PD函数原型 ---- */
uint8_t Track_ReadSensors(void);        /* 读四路传感器, 返回位图 */
int16_t Track_CalcError(uint8_t sensors);/* 位图 -> 误差(-3~+3) */
void Track_Update(void);             /* 每10ms: 传感->误差->PD->目标速度 */
void Track_SetMode(uint8_t on);      /* 启动/停止循迹 */
int16_t SpeedToCount(float rpm);        /* 【NEW】rpm -> 每10ms计数值(四舍五入) */
float CountToRpm(int16_t cnt);        /* 【NEW】每10ms计数值 -> rpm */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void Motor_Left_Set(int16_t duty)
{
    if (duty > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        duty = -duty;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    if (duty > MAX_PWM) duty = MAX_PWM;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

void Motor_Right_Set(int16_t duty)
{
    if (duty > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        duty = -duty;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    if (duty > MAX_PWM) duty = MAX_PWM;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);
}

void pid_calc(MOTOR_PID *p, TIM_HandleTypeDef *htim)
{
    // 1. 目标为0 -> 强制停转
    if (p->target == 0) {
        p->out = 0;
        p->integ = 0;
        p->speed = 0;
        p->err = 0;
        return;
    }

    // 2. 差分计数（绝不写回CNT）
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    p->speed = now - p->last_cnt;



    p->last_cnt = now;

   

    // 5. PID
    p->err = p->target - p->speed;
    p->integ += p->err;
    if (p->integ > 500) p->integ = 500;
    if (p->integ < -500) p->integ = -500;

    float d = p->err - p->err_last;
    p->out = (int16_t)(p->kp * p->err + p->ki * p->integ + p->kd * d);
    p->err_last = p->err;

    // 6. 输出限幅
    if (p->out > MAX_PWM) p->out = MAX_PWM;
    if (p->out < -MAX_PWM) p->out = -MAX_PWM;
}

void Motor_Stop(void)
{
    ml.target = 0; mr.target = 0;
    ml.integ = 0;   mr.integ = 0;
    ml.out = 0;     mr.out = 0;
    ml.err = 0;     mr.err = 0;
    ml.err_last = 0;mr.err_last = 0;
    Motor_Left_Set(0);
    Motor_Right_Set(0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // STBY=0
}

void Motor_Forward(int16_t speed)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ml.target = speed; mr.target = speed;
    ml.integ = 0; mr.integ = 0; ml.out = 0; mr.out = 0;
    ml.err = 0; mr.err = 0; ml.err_last = 0; mr.err_last = 0;
}

void Motor_Backward(int16_t speed)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ml.target = -speed; mr.target = -speed;
    ml.integ = 0; mr.integ = 0; ml.out = 0; mr.out = 0;
    ml.err = 0; mr.err = 0; ml.err_last = 0; mr.err_last = 0;
}

void Motor_TurnLeft(int16_t speed)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ml.target = speed / 2; mr.target = speed;
    ml.integ = 0; mr.integ = 0; ml.out = 0; mr.out = 0;
    ml.err = 0; mr.err = 0; ml.err_last = 0; mr.err_last = 0;
}

void Motor_TurnRight(int16_t speed)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    ml.target = speed; mr.target = speed / 2;
    ml.integ = 0; mr.integ = 0; ml.out = 0; mr.out = 0;
    ml.err = 0; mr.err = 0; ml.err_last = 0; mr.err_last = 0;
}

void Print_Speed(void)
{
    float rpm_l = ml.speed * 100.0f * 60.0f / COUNTS_PER_REV;
    float rpm_r = mr.speed * 100.0f * 60.0f / COUNTS_PER_REV;
    printf("State:%d | L:tgt=%d spd=%d rpm=%.1f out=%d | R:tgt=%d spd=%d rpm=%.1f out=%d\r\n",
           move_state,
           ml.target, ml.speed, rpm_l, ml.out,
           mr.target, mr.speed, rpm_r, mr.out);
}

/* ==================== 【NEW】转速 <-> 计数 换算函数 ==================== */
/* 换算公式推导:
   每转 = 1320个计数(减速比30 x 线数11 x 4倍频, 即 COUNTS_PER_REV)
   1分钟 = 60000ms = 6000 个 10ms 周期
   => rpm 对应的每10ms计数值 = rpm x 1320 / 6000
   例: 273rpm -> 273x1320/6000 = 60计数(与你F键的60档一致)          */
int16_t SpeedToCount(float rpm)
{
    float cnt = rpm * (float)COUNTS_PER_REV / 6000.0f;
    return (int16_t)(cnt + (cnt >= 0 ? 0.5f : -0.5f)); /* 四舍五入, 防浮点截断少1 */
}

float CountToRpm(int16_t cnt)
{
    return (float)cnt * 6000.0f / (float)COUNTS_PER_REV;
}

/* ==================== 【NEW】四路循迹PD: 实现 ==================== */
/* 传感器位图约定: bit3=最左PA2, bit2=左内PA3, bit1=右内PA6, bit0=最右PA7
   位=1 表示该路压在黑线上                                          */
/* 权重表: 从左到右 {-3,-1,+1,+3}
   中间两路(L2,R1)同时对正压线时 (-1)+(+1)=0 -> 误差恰为0
   越靠外侧权重越大, 偏得越远误差绝对值越大                          */
static const int8_t TRACK_W[4] = {-3, -1, 1, 3};

uint8_t Track_ReadSensors(void)
{
    uint8_t sensors = 0;
    if (HAL_GPIO_ReadPin(TRACK_PORT, TRACK_L1_PIN) == LINE_ACTIVE) sensors |= 0x08; /* 最左 */   //也就是说检测到黑线就置1
    if (HAL_GPIO_ReadPin(TRACK_PORT, TRACK_L2_PIN) == LINE_ACTIVE) sensors |= 0x04; /* 左内 */   //|表示有1就表示1
	  if (HAL_GPIO_ReadPin(TRACK_PORT, TRACK_R1_PIN) == LINE_ACTIVE) sensors |= 0x02; /* 右内 */   //0x08表示1000   0x04表示0100
	  if (HAL_GPIO_ReadPin(TRACK_PORT, TRACK_R2_PIN) == LINE_ACTIVE) sensors |= 0x01; /* 最右 */   //0x02表示0010   0x01表示0001
    return sensors;
}

int16_t Track_CalcError(uint8_t sensors)   //&表示都是1才为1
{
    int16_t sum = 0;
    uint8_t cnt = 0, i;
    for (i = 0; i < 4; i++) 
	  {
			if (sensors & (uint8_t)(1u << (3 - i))) { sum += TRACK_W[i]; cnt++; }// 用于检测是否为1,1的话就表示识别黑线
    }                                                                      //(1u << (3 - i))表示0001左移三位，变为1000
    if (cnt == 0) 
		{
        return track_err_last;      /* 全白丢线: 保持上次误差, 继续原方向转向 */
    }
    track_err_last = sum / cnt;     /* 归一化加权平均(向零截断), 范围[-3,+3] */
    return track_err_last;
}

void Track_Update(void)
{
    int16_t err = Track_CalcError(Track_ReadSensors());
    int16_t d   = err - track_err_prev;   /* 误差变化率(D项), 丢线时d=0输出平稳 */
    track_err_prev = err;

    float pd = TRACK_KP * err + TRACK_KD * d;   /* 位置式PD -> 左右差速, 单位rpm */
    if (pd >  TRACK_MAX_DIFF_RPM) pd =  TRACK_MAX_DIFF_RPM;  /* 差速限幅 */
    if (pd < -TRACK_MAX_DIFF_RPM) pd = -TRACK_MAX_DIFF_RPM;

    float base = TRACK_BASE_SPEED_RPM;
    if (err >= TRACK_SLOW_AT || err <= -TRACK_SLOW_AT) 
		 {
        base -= TRACK_SLOW_DELTA_RPM;   /* 弯道(误差大)降速, 防止半圆弯甩出去 */
     }
  /* err>0 = 黑线在车右侧 = 车偏左 -> 需右转: 左轮快右轮慢   err>0时PD为正，err<0时PD为负
       换算成底层PID的每10ms计数值后下发, 实际转速由原有电机PID闭环保证 */ 
    int16_t tL = SpeedToCount(base + pd);
    int16_t tR = SpeedToCount(base - pd);
    if (tL < 0) tL = 0;     /* 【审查修复】极端调参保护: 内轮最多停转, 不反打 */
    if (tR < 0) tR = 0;
    ml.target = tR;
		mr.target = tL;//这里反过来，是因为原来电机pid就是反的
}

void Track_SetMode(uint8_t on)
{
    if (on) {
        track_err_last = 0;                         /* 复位误差记忆 */
        track_err_prev = 0;
        ml.integ = 0; mr.integ = 0;                 /* 复位电机PID积分 */
        ml.err = 0; mr.err = 0; ml.err_last = 0; mr.err_last = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); /* STBY=1 使能驱动 */
        move_state = TRACK;
        printf(">> Track On\r\n");
    } else {
        Motor_Stop();               /* 复用原有停转函数(已做STBY=0和复位) */
        move_state = STOP;
        printf(">> Track Off\r\n");
    }
    track_mode = on;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

HAL_TIM_Base_Start_IT(&htim3);

ml.last_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
mr.last_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

Motor_Stop();
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // STBY=1
Track_SetMode (1);


printf("System Ready!\r\n");
printf("Commands: F=Forward B=Backward L=Left R=Right S=Stop\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
  
//    // ===== PID闭环模式 =====
    uint8_t cmd;
    if (HAL_UART_Receive(&huart1, &cmd, 1, 0) == HAL_OK) {
        if (cmd == '\r' || cmd == '\n') continue;
        switch (cmd) {
            case 'F': case 'f': Motor_Forward(60);  move_state = FORWARD;  printf(">> Forward\r\n");  break;
            case 'B': case 'b': Motor_Backward(60); move_state = BACKWARD; printf(">> Backward\r\n"); break;
            case 'L': case 'l': Motor_TurnLeft(60); move_state = LEFT;     printf(">> Left\r\n");     break;
            case 'R': case 'r': Motor_TurnRight(60);move_state = RIGHT;    printf(">> Right\r\n");    break;
					  /* ----【NEW】T键: 启动/停止循迹(切换) ---- */        //按一次变为循迹mode再按则变为STOP
            case 'T': case 't':  Track_SetMode(!track_mode);                                          break;
            case 'S': case 's': Motor_Stop();       move_state = STOP;     printf(">> Stop\r\n");     break;
            default: break;
        }
    }
    HAL_Delay(10);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        static uint8_t cnt = 0;

			   /* ----【NEW】循迹模式下每10ms刷新一次目标速度 ---- */
        if (track_mode) 
					{
            Track_Update();
          }

        pid_calc(&ml, &htim2);
        pid_calc(&mr, &htim4);
        Motor_Left_Set(ml.out);
        Motor_Right_Set(mr.out);

        if (++cnt >= 10) {
            cnt = 0;
            Print_Speed();
        }

    }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
