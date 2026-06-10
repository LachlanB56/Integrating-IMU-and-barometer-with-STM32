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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR 0xD0
#define MPU_REG_PWR 0x6B
#define BMP280_ADDR 0xEC
#define BMP280_REG_PWR 0xF4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//global declaration of BMP compensation constants
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//temperature compensation formula given on datasheet
int32_t BMP280_T_Compensation(int32_t adc_temp){

	int32_t var1, var2, T;
	    var1 = ((((adc_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	    var2 = (((((adc_temp >> 4) - ((int32_t)dig_T1)) *
	              ((adc_temp >> 4) - ((int32_t)dig_T1))) >> 12) *
	            ((int32_t)dig_T3)) >> 14;
	    t_fine = var1 + var2;
	    T = (t_fine * 5 + 128) >> 8;
	    return T;

}
//pressure compensation formula on datasheet
int32_t BMP280_P_Compensation(int32_t adc_P)
{
	int64_t var1, var2, p;
		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)dig_P6;
		var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
		var2 = var2 + (((int64_t)dig_P4)<<35);
		var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
		var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

			if (var1 == 0){
				return 0; // avoid exception caused by division by zero
}

		p = 1048576-adc_P;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);

return (uint32_t)p;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
   HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
   return len;
}

void I2C_BusReset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure PB6 (SCL) and PB7 (SDA) as open-drain outputs, high
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    // Pulse SCL ~10 times to flush any stuck slave
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // Generate a STOP condition: SDA low->high while SCL high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
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
  I2C_BusReset();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // used this for debugging it returns the address of the modules
    printf("Scanning I2C...\r\n");

    for(uint16_t addr = 1; addr < 128; addr++)
    {
        if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
        {
            printf("Found device at 0x%02X\r\n", addr);
        }
    }


  uint8_t who = 0;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR,0x75,I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
    if(status == HAL_OK && who == 0x70){
  	 printf("MPU INITIALIZED SUCCESSFULLY\r\n");
   } else {
  	 printf("MPU FAIL status = %d ID=0x%02X err=0x%08lX\r\n", status, who, HAL_I2C_GetError(&hi2c1));
   }
    status = HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xD0, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
    if (status == HAL_OK && who == 0x58) {
        printf("BMP280 INITIALIZED SUCCESSFULLY\r\n");
    } else {
        printf("BMP280 FAIL: status=%d ID=0x%02X err=0x%08lX\r\n",
               status, who, HAL_I2C_GetError(&hi2c1));
    }
  uint8_t val = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_PWR, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
  uint8_t check = 0xFF;
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);
  printf("PWR_MGMT_1 after wake = 0x%02X\r\n", check);
  val = 0x27;
  HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP280_REG_PWR, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last = HAL_GetTick();
  float filteredpitch = 0.0f;
  float filteredroll = 0.0f;
  //reading BMP280 pressure and temperature compensation formula constants
  uint8_t calib[24];
  HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, calib, 24, 100);
   dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);   // 0x88/0x89, unsigned
   dig_T2 = (int16_t) ((calib[3] << 8) | calib[2]);   // 0x8A/0x8B, signed
   dig_T3 = (int16_t) ((calib[5] << 8) | calib[4]);   // 0x8C/0x8D, signed
   dig_P1 = (uint16_t)((calib[7] << 8) | calib[6]);   // 0x8E/8F  unsigned
   dig_P2 = (int16_t) ((calib[9] << 8) | calib[8]);   // 0x90/91  signed
   dig_P3 = (int16_t) ((calib[11] << 8) | calib[10]); // 0x92/93  signed
   dig_P4 = (int16_t) ((calib[13] << 8) | calib[12]); // 0x94/95  signed
   dig_P5 = (int16_t) ((calib[15] << 8) | calib[14]); // 0x96/97  signed
   dig_P6 = (int16_t) ((calib[17] << 8) | calib[16]); // 0x98/99  signed
   dig_P7 = (int16_t) ((calib[19] << 8) | calib[18]); // 0x9A/9B  signed
   dig_P8 = (int16_t) ((calib[21] << 8) | calib[20]); // 0x9C/9D  signed
   dig_P9 = (int16_t) ((calib[23] << 8) | calib[22]); // 0x9E/9F  signed
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  uint8_t buf[14];
	 	  	  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buf, 14, 100);
	 	  	  //reading bytes from register and combining the high and low values
	 	  	  int16_t ax = (int16_t)(buf[0] << 8 | buf[1]);
	 	  	  int16_t ay = (int16_t)(buf[2] << 8 | buf[3]);
	 	  	  int16_t az = (int16_t)(buf[4] << 8 | buf[5]);
	 	  	  int16_t gx = (int16_t)(buf[8]  << 8 | buf[9]);
	 	  	  int16_t gy = (int16_t)(buf[10] << 8 | buf[11]);
	 	  	  int16_t gz = (int16_t)(buf[12] << 8 | buf[13]);
	 	  	  //using 3d trig formula to find roll and pitch
	 	  	  float pitch = atan2f(ax, sqrtf(ay*ay + az*az))*(180.0 / 3.14159);
	 	  	  float roll = atan2f(ay, sqrtf(ax*ax + az*az))*(180.0 / 3.14159);

	 	  	  uint32_t now = HAL_GetTick();
	 	  	  float dt = (now - last) / 1000.0f;
	 	  	//131 defined in MPU6050 datasheet to convert raw data
	 	  	  float gyroRateX = gx / 131.0;
	 	  	  float gyroRateY = gy / 131.0;


	 	  	  //filter that is a high pass filter for the gyroscope and low pass for accelerometer to reduce noise
	 	  	  filteredpitch = 0.96*(filteredpitch + (gyroRateY*dt)) + 0.04*pitch;
	 	      filteredroll = 0.96*(filteredroll + (gyroRateX*dt)) + 0.04*roll;

	 	      printf("PITCH = %.2f\r\n", filteredpitch);
	 	      printf("ROLL = %.2f\r\n", filteredroll);


	 	  	uint8_t pbuf[6];
	 	  	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, pbuf, 6, 100);
	 	  	// reading bytes from the BMP280 register and combining
	 	  	int32_t raw_press = (int32_t)(((uint32_t)pbuf[0] << 12) | ((uint32_t)pbuf[1] << 4) | ((uint32_t)pbuf[2] >> 4));
	 	  	int32_t raw_temp  = (int32_t)(((uint32_t)pbuf[3] << 12) | ((uint32_t)pbuf[4] << 4) | ((uint32_t)pbuf[5] >> 4));



	 	  	//temperature compensation provided on datasheet
	 	  	int32_t T = BMP280_T_Compensation(raw_temp);
	 	  	int32_t P = BMP280_P_Compensation(raw_press);


	 	  	printf("Temp = %ld.%02ld C  Press = %lu.%02lu hPa\r\n",
	 	  	       T/100, T%100, (P/256)/100, ((P/256)%100));

	 	  	float pressure_hPa = (P / 256.0f) / 100.0f;
	 	  	float altitude = 44330.0f * (1.0f - powf(pressure_hPa / 1013.25f, 0.1903f));
	 	  	printf("Altitude = %.2f m\r\n", altitude);

	 	  	HAL_Delay(100);

	 	  	last = now;



  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
