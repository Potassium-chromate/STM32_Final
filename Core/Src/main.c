/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Esp8266-01 Test-Function
 * Author          :
 * websites        :https://embeddedsystemshalilgk.wordpress.com
 * Youtube         :https://www.youtube.com/channel/UC8cJpAVnScqDzZ7IFs2tQMw
 ******************************************************************************
 * @attention
 *ESP_Init();this function should be called after testing. ESP_Init() can be called
 *this If hardware and software problems are not encountered.
 *test_AT();first, you must query with the test_AT command.
 *  USART2 -> PC
 *  USART3 ->ESP8266
 *We do the data retrieval part with interrupt.
 * Don't forget to activate the interrupt.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "myprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_RESET,
    STATE_INIT,
    STATE_JOIN_WIFI,
    STATE_CHECK_STATUS,
    STATE_GET_IP,
	STATE_CONNECT_TCP,
    STATE_DONE
} esp8266_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define resp_len 128
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
char resp[resp_len];
volatile int if_resp_callback;
volatile int if_resp_timeout;
volatile int resp_index;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ESP8266_SendCommand(char *cmd)
{
	myprintf(&huart2, "Send command: %s\r\n", cmd);
    myprintf(&huart6, "%s\r\n", cmd);
}

void ESP8266_ReadResponse()
{
    memset(resp, '\0', resp_len);
    if_resp_callback = 0;
    if_resp_timeout = 0;
    resp_index = 0;

    HAL_UART_Receive_IT(&huart6, (uint8_t *)&resp[resp_index], 1);

    uint32_t start_time = HAL_GetTick();

    while (if_resp_callback == 0) {
        if (HAL_GetTick() - start_time > 2000) {
            if_resp_timeout = 1;
            break;
        }
    }
    vTaskDelay(1000);
    myprintf(&huart2, "Received: %s\r\n", resp + 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        if (resp_index < resp_len - 1) {
        	resp_index++;
            // Continue receiving
            HAL_UART_Receive_IT(&huart6, (uint8_t *)&resp[resp_index], 1);

            // Look for end of response
            if (resp[resp_index - 1] == '\n' || if_resp_timeout == 1) {
                resp[resp_index] = '\0'; // Ensure null termination
                if_resp_callback = 1;
                // Don't reset resp_index here to preserve received data
            }
        } else {
            // Buffer full
            resp[resp_len - 1] = '\0';
            if_resp_callback = 1;
        }
    }
}
void showIP_task(void *pvParameters)
{
    esp8266_state_t state = STATE_RESET;
    int retry = 0;

    while (1) {
        switch (state) {
        case STATE_RESET:
            ESP8266_SendCommand("AT+RST");
            ESP8266_ReadResponse();
            if (strstr(resp, "OK")) {
                state = STATE_INIT;
                retry = 0;
            } else if (++retry > 3) {
                myprintf(&huart2, "Reset failed, retrying...\r\n");
                retry = 0;
            }
            break;

        case STATE_INIT:
            ESP8266_SendCommand("AT");  // Test AT startup
            ESP8266_ReadResponse();
            ESP8266_SendCommand("AT+CWMODE=1");  // Set station mode
            ESP8266_ReadResponse();
            state = STATE_JOIN_WIFI;
            break;

        case STATE_JOIN_WIFI:
            ESP8266_SendCommand("AT+CWJAP=\"AndroidAPF7C1\",\"eason901215\"");
            ESP8266_ReadResponse();
            if (strstr(resp + 1, "WIFI CONNECTED") || strstr(resp + 1, "OK")) {
                state = STATE_CHECK_STATUS;
                retry = 0;
            } else if (++retry > 3) {
                myprintf(&huart2, "WiFi join failed, retrying...\r\n");
                state = STATE_RESET;
                retry = 0;
            }
            break;

        case STATE_CHECK_STATUS:
            ESP8266_SendCommand("AT+CIPSTATUS");
            ESP8266_ReadResponse();
            if (strstr(resp + 1, "STATUS:2") || strstr(resp + 1, "STATUS:3") || strstr(resp + 1, "OK")) {
                state = STATE_GET_IP;
            }
            break;

        case STATE_GET_IP:
            ESP8266_SendCommand("AT+CIFSR");
            ESP8266_ReadResponse();
            state = STATE_CONNECT_TCP;
            break;
        case STATE_CONNECT_TCP:
        	ESP8266_SendCommand("AT+CIPMUX=0");
        	ESP8266_ReadResponse();
        	ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"192.168.191.189\",5000");
        	ESP8266_ReadResponse();
        	state = STATE_DONE;
        case STATE_DONE:
            // Optionally poll IP or status repeatedly
            vTaskDelay(5000);  // Poll every 5s
            break;

        default:
            state = STATE_RESET;
            break;
        }

        vTaskDelay(1500);
    }
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  xTaskCreate(showIP_task, "ShowIP", 1024, NULL, 0, NULL);
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WIFI_RST_Pin|WIFI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WIFI_RST_Pin WIFI_EN_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin|WIFI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
