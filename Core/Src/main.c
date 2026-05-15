/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ir_sensor.h"
#include "CarGo.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_LINE_TRACKING  = 0,  // 正常巡线
    STATE_WAITING_CMD    = 1,  // 发送begin，等待RK3588返回指令
    STATE_POST_JUNCTION  = 2,  // 越过路口状态，防止重复触发黑线
    STATE_FINISHED       = 3   // 到达终点，永久停止
} CarState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_MAX_LEN        20    // 接收缓冲区大小
#define JUNCTION_BLACK     1     // 【重要修复】根据ir_sensor.c，压黑线应当为1
#define RECEIVE_TIMEOUT    5000  // 等待视觉指令的超时时间 (ms)
#define JUNCTION_DEBOUNCE  1     // 连续几次全黑才确认是十字路口 (防抖)
#define POST_JUNCTION_MS   450   // 越过十字路口需要盲开的时间 (根据车速调整)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CarState_t car_state = STATE_LINE_TRACKING;
uint8_t    junction_cnt = 0;
uint32_t   post_junc_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_SendString(const char *str);
uint8_t UART_ReceiveCommand(char *cmd, uint16_t timeout_ms);
uint8_t ExecuteCommand(const char *cmd); // 返回值修改为uint8_t，用于判断是否结束
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MotorInit();
  HAL_TIM_Base_Start_IT(&htim3);
  UART_SendString("Car ready\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    int L1, L2, R2, R1;
//    IR_Read(&L1, &L2, &R2, &R1);  // (left1, left2, right2, right1)

//    // =====================================================
//    // 状态 0: 正常巡线
//    // =====================================================
//    if (car_state == STATE_LINE_TRACKING)
//    {
//        // 判定十字路口：四个传感器全压黑线
//        if (L1 == JUNCTION_BLACK && L2 == JUNCTION_BLACK &&
//            R2 == JUNCTION_BLACK && R1 == JUNCTION_BLACK)
//        {
//            junction_cnt++;
//            if (junction_cnt >= JUNCTION_DEBOUNCE)
//            {
//                // 确认到达路口，立刻停车
//                CarStop();
//                HAL_Delay(200); // 稳定车身
//                
//                // 对应流程图 II：通过串口给RK3588发送开始识别标志
//                UART_SendString("begin\n");
//                
//                car_state = STATE_WAITING_CMD;
//                junction_cnt = 0;
//                continue;
//            }
//        }
//        else
//        {
//            junction_cnt = 0;

//            // 调用你写好的巡线逻辑
//            int track = IR_GetCarState();
//            switch (track)
//            {
//                case 1: CarAhead(90, 90); break;   
//                case 2: Left(30, 80);     break;
//                case 3: Right(80, 30);    break;
//                case 4: Left(80, 80);     break;
//                case 5: Right(80, 80);    break;
//                case 0:
//                default: CarStop(); break;
//            }
//        }
//    }
//    // =====================================================
//    // 状态 1: 对应流程图 III，等待RK3588返回指令
//    // =====================================================
//    else if (car_state == STATE_WAITING_CMD)
//    {
//        char command[CMD_MAX_LEN] = {0};
//        
//        // 阻塞等待接收
//        if (UART_ReceiveCommand(command, RECEIVE_TIMEOUT))
//        {
//            // 执行收到动作，如果返回0代表收到了"stop"
//            if (ExecuteCommand(command) == 0) 
//            {
//                car_state = STATE_FINISHED;
//                continue; // 直接进入结束状态，不再往前冲
//            }
//        }
//        else
//        {
//            // 串口超时没有回传（对应无标识路口，默认直行）
//            ExecuteCommand("stop");
//        }

//        // 对应流程图 IV：动作完成后，需要越过当前路口的黑线，防止死循环检测
//        post_junc_tick = HAL_GetTick();
//        car_state = STATE_POST_JUNCTION;
//    }
//    // =====================================================
//    // 状态 2: 越过路口盲区
//    // =====================================================
//    else if (car_state == STATE_POST_JUNCTION)
//    {
//        CarAhead(80, 80);  // 盲开一小段距离，驶出十字路口黑线区
//        if ((HAL_GetTick() - post_junc_tick) >= POST_JUNCTION_MS)
//        {
//            car_state = STATE_LINE_TRACKING; // 恢复巡线
//        }
//    }
//    // =====================================================
//    // 状态 3: 到达终点
//    // =====================================================
//    else if (car_state == STATE_FINISHED)
//    {
//        CarStop(); // 彻底停止响应
//        // 如果想重新启动，需按下板子上的Reset键
//    }

//    HAL_Delay(10);  // Main loop period
UART_SendString("begin\n");
	HAL_Delay(500);

}
	
  /* USER CODE END 3 */
}

// ... 保持系统自带的 SystemClock_Config() 不动 ...

/* USER CODE BEGIN 4 */

/**
  * @brief  通过USART2发送字符串
  */
void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

/**
  * @brief  阻塞接收直到遇到换行符或超时
  */
uint8_t UART_ReceiveCommand(char *cmd, uint16_t timeout_ms)
{
    memset(cmd, 0, CMD_MAX_LEN);
    uint8_t idx = 0;
    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return 0;  // 超时
        }

        uint8_t ch;
        // 每次等10ms接收1个字节
        if (HAL_UART_Receive(&huart2, &ch, 1, 10) == HAL_OK)
        {
            if (ch == '\n' || ch == '\r')
            {
                cmd[idx] = '\0';
                return 1;  // 成功接收到一条指令
            }
            if (ch >= ' ' && ch <= '~')
            {
                if (idx < CMD_MAX_LEN - 1)
                {
                    cmd[idx++] = ch;
                }
            }
        }
    }
}

/**
  * @brief  执行RK3588发送过来的指令
  * @retval 1: 继续运行 | 0: 收到终点指令，需永久停车
  */
uint8_t ExecuteCommand(const char *cmd)
{
    if (strlen(cmd) == 0) return 1;

    if (strcmp(cmd, "turn_right") == 0)
    {
        Right(80, 80);
        HAL_Delay(600); // 注意：电池满电和亏电时，转90度所需时间不同，请实车微调
    }
    else if (strcmp(cmd, "turn_left") == 0)
    {
        Left(80, 80);
        HAL_Delay(600);
    }
    else if (strcmp(cmd, "go_straight") == 0)
    {
        CarAhead(90, 90);
        HAL_Delay(300);
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        // 收到停止牌命令，到达终点
        CarStop();
        return 0; // 返回0通知主循环彻底结束
    }
    else
    {
        // 收到垃圾数据，保守起见默认直行
        CarAhead(80, 80);
        HAL_Delay(300);
    }
    
    return 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        // User timer callback
    }
}

/* USER CODE END 4 */
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */