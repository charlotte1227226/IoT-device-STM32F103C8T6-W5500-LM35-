/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#define Sn_MR_UDP      0x02

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t W5500_ReadRegister(uint16_t addr, uint8_t control_byte, uint8_t *data_out)
{
    extern SPI_HandleTypeDef hspi1;
    #define W5500_CS_GPIO_Port GPIOA
    #define W5500_CS_Pin       GPIO_PIN_4

    uint8_t tx_buf[3];
    tx_buf[0] = (addr >> 8) & 0xFF;      // Address High Byte
    tx_buf[1] = addr & 0xFF;             // Address Low Byte
    tx_buf[2] = control_byte;            // Control Byte

    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);  // 可以留著稍微穩定
    // 印出送出的封包 (debug 用)
    printf("TX: %02X %02X %02X\n", tx_buf[0], tx_buf[1], tx_buf[2]);
    // 傳送位址 + 控制碼，並檢查是否成功
    if (HAL_SPI_Transmit(&hspi1, tx_buf, 3, HAL_MAX_DELAY) != HAL_OK) {
        printf("TX failed (W5500_ReadRegister)\n");
        HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
        return 1;
    }

    // 接收資料，並檢查是否成功
    if (HAL_SPI_Receive(&hspi1, data_out, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("RX failed (W5500_ReadRegister)\n");
        HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
        return 2;
    }

    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
    return 0;
}

uint8_t W5500_WriteRegister(uint16_t addr, uint8_t control_byte, const uint8_t *data, uint16_t len)
{
    extern SPI_HandleTypeDef hspi1;
    #define W5500_CS_GPIO_Port GPIOA
    #define W5500_CS_Pin       GPIO_PIN_4

    uint8_t tx_buf[3];
    tx_buf[0] = (addr >> 8) & 0xFF;  // Address high byte
    tx_buf[1] = addr & 0xFF;         // Address low byte
    tx_buf[2] = control_byte;        // Control byte
    // ✅ 印出送出的封包 (debug 用)
    printf("TX: %02X %02X %02X\n", tx_buf[0], tx_buf[1], tx_buf[2]);
    // Assert CS (active low)
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);  // Optional short delay

    // 傳送位址與控制位元
    if (HAL_SPI_Transmit(&hspi1, tx_buf, 3, HAL_MAX_DELAY) != HAL_OK) {
        printf("TX failed (addr/control) @ 0x%04X\r\n", addr);
        HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
        return 1;
    }

    // 寫入資料區段
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY) != HAL_OK) {
        printf("TX failed (data payload) @ 0x%04X\r\n", addr);
        HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
        return 2;
    }

    // 釋放 CS
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
    return 0;
}
void W5500_Test_ReadVersion(void)
{
    uint8_t version = 0;

    // VERSIONR 位址：0x0039，BSB = 0x00，RWB=0（Read），OM=VDM=0x00
    uint8_t tx_buf[3] = {0x00, 0x39, 0x00};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS LOW
    HAL_Delay(1);

    if (HAL_SPI_Transmit(&hspi1, tx_buf, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("SPI Transmit failed\n");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        return;
    }

    if (HAL_SPI_Receive(&hspi1, &version, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("SPI Receive failed\n");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        return;
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // CS HIGH

    printf("W5500 VERSIONR: 0x%02X\r\n", version);

    if (version == 0x04)
        printf("SPI & W5500 正常\n");
    else
        printf("SPI 錯誤 或 W5500 無回應\n");
}
#include <string.h>

#define SOCKET_BASE(sn) (0x4000 + (sn * 0x100))

int socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
    uint8_t mode = protocol | flag;

    // 關閉原本的 socket
    uint16_t addr = SOCKET_BASE(sn) + 0x0001 + 1; // Sn_CR
    W5500_WriteRegister(addr, 0x04, (uint8_t[]){0x10}, 1);  // Sn_CR = CLOSE
    HAL_Delay(1);

    // 設定 mode
    addr = SOCKET_BASE(sn) + 0x0000; // Sn_MR
    W5500_WriteRegister(addr, 0x04, &mode, 1);

    // 設定 port
    uint8_t port_bytes[2] = {port >> 8, port & 0xFF};
    addr = SOCKET_BASE(sn) + 0x0004; // Sn_PORT
    W5500_WriteRegister(addr, 0x04, port_bytes, 2);

    // Open command
    addr = SOCKET_BASE(sn) + 0x0001; // Sn_CR
    W5500_WriteRegister(addr, 0x04, (uint8_t[]){0x01}, 1);  // Sn_CR = OPEN
    HAL_Delay(1);

    return sn;
}

int32_t recvfrom(uint8_t sn, uint8_t *buf, uint16_t len, uint8_t *ip, uint16_t *port)
{
    uint8_t header[8];
    uint8_t rx_len_buf[2];
    uint16_t addr = SOCKET_BASE(sn) + 0x0026; // Sn_RX_RSR
    W5500_ReadRegister(addr, 0x00, rx_len_buf);
    uint16_t rx_size = (rx_len_buf[0] << 8) | rx_len_buf[1];

    if (rx_size == 0) return 0;

    // 讀出 8 bytes UDP header
    addr = 0x6000 + (sn * 0x100); // RX buffer 起始位址
    W5500_ReadRegister(addr, 0x18, header); // 0x18: read RX buffer

    ip[0] = header[0]; ip[1] = header[1]; ip[2] = header[2]; ip[3] = header[3];
    *port = (header[4] << 8) | header[5];
    uint16_t data_len = (header[6] << 8) | header[7];

    if (data_len > len) data_len = len;
    W5500_ReadRegister(addr + 8, 0x18, buf); // 接著讀取 payload

    // 更新接收指標 Sn_CR = RECV
    addr = SOCKET_BASE(sn) + 0x0001; // Sn_CR
    W5500_WriteRegister(addr, 0x04, (uint8_t[]){0x40}, 1);  // RECV

    return data_len;
}

int32_t sendto(uint8_t sn, const uint8_t *buf, uint16_t len, const uint8_t *ip, uint16_t port)
{
    uint16_t addr;

    // 寫入目標 IP
    addr = SOCKET_BASE(sn) + 0x000C; // Sn_DIPR
    W5500_WriteRegister(addr, 0x04, (uint8_t *)ip, 4);

    // 寫入目標 port
    uint8_t port_buf[2] = {port >> 8, port & 0xFF};
    addr = SOCKET_BASE(sn) + 0x0010; // Sn_DPORT
    W5500_WriteRegister(addr, 0x04, port_buf, 2);

    // 寫入資料到 TX buffer
    addr = 0x4000 + (sn * 0x100); // TX buffer base
    W5500_WriteRegister(addr, 0x14, (uint8_t *)buf, len); // 0x14 = Write TX buffer

    // 下 SEND 命令
    addr = SOCKET_BASE(sn) + 0x0001; // Sn_CR
    W5500_WriteRegister(addr, 0x04, (uint8_t[]){0x20}, 1); // Sn_CR = SEND

    return len;
}
/* USER CODE END 1 */
