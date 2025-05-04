/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Block Select: 0x00 = Common Register
// RWB: 1 = Write, 0 = Read
// OM: 01 = VDM mode (固定地址模式)
#define W5500_CB_READ   0x00 | (0 << 2) | (0 << 0)  // 0x00
#define W5500_CB_WRITE  0x00 | (1 << 2) | (1 << 0)  // 0x04 | 0x01 = 0x05
// Socket類型定義（對應 W5500 控制暫存器的值）
#define SOCK_STREAM     0x01  // TCP
#define SOCK_DGRAM      0x02  // UDP
#define SOCK_RAW        0x03  // Raw IP
#define SOCK_MACRAW     0x04  // MAC Raw
#define SOCK_PPPOE      0x05  // PPPoE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  uint8_t gateway[4] = {192, 168, 0, 1};
  uint8_t subnet[4]  = {255, 255, 255, 0};
  uint8_t ip[4]      = {192, 168, 0, 20};
  uint8_t mac[6]     = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */                                                   // 等 W5500 啟動穩定

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // RESET 腳拉低（請換成你實際接的腳位）
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // 拉高解除 RESET
  HAL_Delay(300);  // 等 W5500 穩定
  // 參數寫入 W5500 的暫存器
  W5500_WriteRegister(0x0009, W5500_CB_WRITE, mac, 6);     
  W5500_WriteRegister(0x0001, W5500_CB_WRITE, gateway, 4); 
  W5500_WriteRegister(0x0005, W5500_CB_WRITE, subnet, 4);  
  W5500_WriteRegister(0x000F, W5500_CB_WRITE, ip, 4);      
  printf("W5500 IP/GW/Subnet 設定完成\r\n");
  uint8_t check_ip[4];
  W5500_ReadRegister(0x000F, W5500_CB_READ, check_ip, 4);
  printf("W5500 IP: %d.%d.%d.%d\r\n", check_ip[0], check_ip[1], check_ip[2], check_ip[3]);
  // 開啟 UDP Socket 0，綁定 port 5000
  uint8_t sock_num = 0;
  uint16_t port = 5000;
  if (socket(sock_num, SOCK_DGRAM, port) != 0) {
    printf("socket() open failed\r\n");
  } else {
    printf("socket open on port %d\r\n", port);
  }
  uint8_t rx_buf[64];
  uint8_t sender_ip[4];
  uint16_t sender_port;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t raw = HAL_ADC_GetValue(&hadc1);
  float voltage = (float)raw / 4095.0 * 3.3; // 單位：V
  float temperature_C = voltage * 10.0; // LM35 每 10mV = 1°C
  printf("ADC raw: %u, voltage: %.2f V, temperature: %.2f °C\r\n", raw, voltage, temperature_C);
  HAL_Delay(500);
  //--------------------------
  uint8_t version;
  if (W5500_ReadRegister(0x0039, W5500_CB_READ, &version) == 0)
  {
    printf("W5500 VERSIONR: 0x%02X\r\n", version);
  }
  else
  {
    printf("Failed to read VERSIONR.\r\n");
  }
  HAL_Delay(500);
  //--------------------------
  W5500_ReadRegister(0x000F, W5500_CB_READ, check_ip, 4);
  printf("W5500 IP: %d.%d.%d.%d\r\n", check_ip[0], check_ip[1], check_ip[2], check_ip[3]);
  HAL_Delay(500);
  uint8_t phycfgr;
  if (W5500_ReadRegister(0x002E, W5500_CB_READ, &phycfgr) == 0)
  {
    printf("PHYCFGR: 0x%02X\r\n", phycfgr);
    if (phycfgr & 0x01)
      printf("網路已連線 (PHY Link Up)\r\n");
    else
      printf("網路未連線 (PHY Link Down)\r\n");
  }
  else
  {
    printf("無法讀取 PHYCFGR，請檢查 SPI 連接\r\n");
  }
  HAL_Delay(500);
  W5500_Test_ReadVersion();
  HAL_Delay(500);
  printf("準備呼叫 recvfrom()\r\n");
  int len = recvfrom(sock_num, rx_buf, sizeof(rx_buf), sender_ip, &sender_port);
  printf("recvfrom() 回傳長度: %d\r\n", len);

  if (len > 0) {
  printf("收到 %d bytes from %d.%d.%d.%d:%d\r\n", len, sender_ip[0], sender_ip[1], sender_ip[2], sender_ip[3], sender_port);
  printf("準備回傳 echo...\r\n");
  sendto(sock_num, rx_buf, len, sender_ip, sender_port);
  printf("傳輸 %d bytes from %d.%d.%d.%d:%d\r\n", len, sender_ip[0], sender_ip[1], sender_ip[2], sender_ip[3], sender_port);
  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
