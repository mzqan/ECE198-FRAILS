/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 10 // Size of the sliding window
#define K 2.0 // Threshold multiplier
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1; // Timer for microsecond delays

/* USER CODE BEGIN PV */
float readings[N]; // Sliding window for temperature readings
uint8_t index = 0;
uint8_t count = 0;
bool to_alert;
uint8_t dht11_data[5]; // Buffer to store DHT11 data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void); // Timer initialization prototype

/* USER CODE BEGIN PFP */
float calculate_sma(void);
float calculate_std_dev(float sma);
float update_and_calculate_threshold(float new_reading);
int get_dht11_temperature(void);
void delay_us(uint16_t us);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadByte(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Microsecond delay
 */
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/**
 * @brief Start communication with DHT11
 */
void DHT11_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure DHT11_PIN as output
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET); // Pull LOW
    HAL_Delay(18); // Wait 18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);   // Pull HIGH
    delay_us(20); // Wait 20-40us
}

/**
 * @brief Check response from DHT11
 */
uint8_t DHT11_CheckResponse(void) {
    uint8_t response = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure DHT11_PIN as input
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) { // Pin LOW
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) response = 1; // Pin HIGH
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)); // Wait for LOW
    }
    return response;
}

/**
 * @brief Read a byte from DHT11
 */
uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))); // Wait for HIGH
        delay_us(40); // Wait 40us
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
            byte |= (1 << (7 - i)); // Set bit
        }
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)); // Wait for LOW
    }
    return byte;
}

/**
 * @brief Get temperature from DHT11
 */
int get_dht11_temperature(void) {
    uint8_t i;
    int temperature = -1;

    DHT11_Start();
    if (DHT11_CheckResponse()) {
        for (i = 0; i < 5; i++) {
            dht11_data[i] = DHT11_ReadByte();
        }
        if ((dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3]) == dht11_data[4]) {
            temperature = dht11_data[2]; // Extract temperature
        }
    }

    return temperature;
}

/**
 * @brief Compute the Simple Moving Average (SMA).
 */
float calculate_sma(void) {
    float sum = 0.0;
    for (uint8_t i = 0; i < count; i++) {
        sum += readings[i];
    }
    return sum / count;
}

/**
 * @brief Compute the standard deviation.
 */
float calculate_std_dev(float sma) {
    float variance = 0.0;
    for (uint8_t i = 0; i < count; i++) {
        variance += (readings[i] - sma) * (readings[i] - sma);
    }
    return sqrt(variance / count);
}

/**
 * @brief Update the readings buffer and compute the threshold.
 */
float update_and_calculate_threshold(float new_reading) {
    readings[index] = new_reading;
    index = (index + 1) % N;
    if (count < N) count++;

    float sma = calculate_sma();
    float std_dev = calculate_std_dev(sma);

    return sma + K * std_dev;
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

  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init(); // Initialize timer for microsecond delays

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Ensure LED is off initially
  int temperature = 0;
  float threshold = 0.0;

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    temperature = get_dht11_temperature();
    if (temperature != -1) {
        printf("Temperature: %d°C\n", temperature); // Output via UART
        threshold = update_and_calculate_threshold((float)temperature);

        // Compare current reading to threshold
        if ((float)temperature > threshold) {
            to_alert = true;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Example: Turn on LED
        } else {
            to_alert = false;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LED
        }
    } else {
        printf("Error reading DHT11\n");
    }

    HAL_Delay(2000); // Wait 2 seconds between readings

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
