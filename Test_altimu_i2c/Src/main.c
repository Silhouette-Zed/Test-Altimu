
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
/* USER CODE BEGIN Includes */
#include <math.h>
//#include <conio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* THIS CODE IS WRITTEN FOR ON STM32F407
 * USING 12C1 AT FAST MODE
 * USING USART1 AT 115200 BAUDRATE
 * SCL PB6
 * SDA PB7
 * VIN VCC
 * GND GND
 * SA0 GND
 */

/*** LPS25H Barometer DEFINED USING DATASHEET***/
#define LPS_HIGH_ADDRESS  		0x5D<<1
#define LPS_LOW_ADDRESS  			0x5C<<1
#define WHO_AM_I_REG_LPS   	 	0x0F	//0xBD

#define REF_P_XL_LPS 					0x08
#define REF_P_L_LPS 					0x09
#define REF_P_H_LPS 					0x0A

#define RES_CONF_LPS    			0x10

#define CTRL1_REG_LPS					0x20
#define CTRL2_REG_LPS					0x21
#define CTRL3_REG_LPS					0x22
#define CTRL4_REG_LPS					0x23

#define STATUS_REG_LPS				0x27

#define PRESS_OUT_XL_LPS			0x28
#define PRESS_OUT_L_LPS				0x29
#define PRESS_OUT_H_LPS				0X2A

#define TEMP_OUT_L_LPS    		0X2B
#define TEMP_OUT_H_LPS    		0X2C

#define FIFO_CTRL_LPS					0x2E
#define FIFO_STATUS_LPS				0x2F

#define RPDS_L_LPS						0x39
#define RPDS_H_LPS						0x3A
	
#define LPS25H_INTERRUPT_CFG  0x24
#define LPS25H_INT_SOURCE     0x25
#define LPS25H_THS_P_L        0x30
#define LPS25H_THS_P_H			  0x31

/*** LIS3MDL Magnetometer DEFINED USING DATASHEET***/
#define LIS_HIGH_ADDRESS  		0x1E<<1
#define LIS_LOW_ADDRESS   		0x1C<<1
#define WHO_AM_I_REG_LIS  		0x0F	// 0x3D

#define CTRL_REG1_LIS   			0x20
#define CTRL_REG2_LIS   			0x21
#define CTRL_REG3_LIS   			0x22
#define CTRL_REG4_LIS   			0x23
#define CTRL_REG5_LIS   			0x24

#define STATUS_REG_LIS  			0x27
#define OUT_X_L_LIS     			0x28
#define OUT_X_H_LIS     			0x29
#define OUT_Y_L_LIS     			0x2A
#define OUT_Y_H_LIS     			0x2B
#define OUT_Z_L_LIS     			0x2C
#define OUT_Z_H_LIS     			0x2D
#define TEMP_OUT_L_LIS  			0x2E
#define TEMP_OUT_H_LIS  			0x2F
#define INT_CFG_LIS     			0x30
#define INT_SRC_LIS     			0x31
#define INT_THS_L_LIS   			0x32
#define INT_THS_H_LIS   			0x33

/*** LSM6DS33 Gyro and Accelerometer DEFINED USING DATASHEET***/
#define LSM_HIGH_ADDRESS  		0x6B<<1
#define LSM_LOW_ADDRESS   		0x6A<<1
#define WHO_AM_I_LSM          0x0F	// 0x69

#define FUNC_CFG_ACCESS_LSM   0x01
#define FIFO_CTRL1_LSM        0x06
#define FIFO_CTRL2_LSM        0x07
#define FIFO_CTRL3_LSM        0x08
#define FIFO_CTRL4_LSM        0x09
#define FIFO_CTRL5_LSM        0x0A
#define ORIENT_CFG_G_LSM      0x0B

#define INT1_CTRL_LSM         0x0D
#define INT2_CTRL_LSM         0x0E
#define CTRL1_XL_LSM          0x10
#define CTRL2_G_LSM           0x11
#define CTRL3_C_LSM           0x12
#define CTRL4_C_LSM           0x13
#define CTRL5_C_LSM           0x14
#define CTRL6_C_LSM           0x15
#define CTRL7_G_LSM           0x16
#define CTRL8_XL_LSM          0x17
#define CTRL9_XL_LSM          0x18
#define CTRL10_C_LSM          0x19

#define WAKE_UP_SRC_LSM       0x1B
#define TAP_SRC_LSM           0x1C
#define D6D_SRC_LSM           0x1D
#define STATUS_REG_LSM        0x1E

#define OUT_TEMP_L_LSM        0x20
#define OUT_TEMP_H_LSM        0x21
#define OUTX_L_G_LSM          0x22
#define OUTX_H_G_LSM          0x23
#define OUTY_L_G_LSM          0x24
#define OUTY_H_G_LSM          0x25
#define OUTZ_L_G_LSM          0x26
#define OUTZ_H_G_LSM          0x27
#define OUTX_L_XL_LSM         0x28
#define OUTX_H_XL_LSM         0x29
#define OUTY_L_XL_LSM         0x2A
#define OUTY_H_XL_LSM         0x2B
#define OUTZ_L_XL_LSM         0x2C
#define OUTZ_H_XL_LSM         0x2D

#define FIFO_STATUS1_LSM      0x3A
#define FIFO_STATUS2_LSM      0x3B
#define FIFO_STATUS3_LSM      0x3C
#define FIFO_STATUS4_LSM      0x3D
#define FIFO_DATA_OUT_L_LSM   0x3E
#define FIFO_DATA_OUT_H_LSM   0x3F
#define TIMESTAMP0_REG_LSM    0x40
#define TIMESTAMP1_REG_LSM    0x41
#define TIMESTAMP2_REG_LSM    0x42

#define STEP_TIMESTAMP_L_LSM  0x49
#define STEP_TIMESTAMP_H_LSM  0x4A
#define STEP_COUNTER_L_LSM    0x4B
#define STEP_COUNTER_H_LSM    0x4C

#define FUNC_SRC_LSM         0x53

#define TAP_CFG_LSM           0x58
#define TAP_THS_6D_LSM        0x59
#define INT_DUR2_LSM          0x5A
#define WAKE_UP_THS_LSM       0x5B
#define WAKE_UP_DUR_LSM       0x5C
#define FREE_FALL_LSM         0x5D
#define MD1_CFG_LSM           0x5E
#define MD2_CFG_LSM           0x5F

uint8_t devAdd_LPS = LPS_HIGH_ADDRESS;
uint8_t devAdd_LIS = LIS_HIGH_ADDRESS;
uint8_t devAdd_LSM = LSM_HIGH_ADDRESS;
uint8_t rxBuf[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retval None*/
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
	return ch;
}

/*****************LPS*****************/
void LPS_init() {
	uint8_t rxBuffer[2];
	uint8_t txdata[2];
	// PAGE 24 DATASHEET
	// WHO AM I, 0xBD - 189
	txdata[0] = WHO_AM_I_REG_LPS;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LPS, &txdata[0], 1, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LPS, rxBuffer, 1, 100);   
	HAL_Delay(10);
	printf("Who Am I : 0x%x\n\r",rxBuffer[0]);
	
	// PAGE 24 DATASHEET
	// 0xB0 = 0b10110000
	// PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
	txdata[0] = CTRL1_REG_LPS;
	txdata[1] = 0xB0;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LPS, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LPS, rxBuffer, 2, 100);   
	HAL_Delay(10);
	
}

// reads pressure in millibars (mbar)/hectopascals (hPa)
float readPressure() {
	// PAGE 33 DATASHEET
	// assert MSB to enable register address auto-increment
	uint8_t rxBuffer[3];
	uint8_t register_pointer = PRESS_OUT_XL_LPS | (1<<7);
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LPS, &register_pointer, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LPS, rxBuffer, 3, 100);   
	HAL_Delay(20);
	
	float readVal = (int32_t)(int8_t)rxBuffer[2]<<16 | (uint16_t)rxBuffer[1]<<8 | rxBuffer[0];
	return (float)readVal / 4096;
	
}

// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
float pressureToAltitudeMeters(float pressure_mbar){
	
	return (float)(1 - pow(pressure_mbar / 1013, 0.190263)) * 44330.8;
	
}

float readTemperature() {
	// PAGE 34 DATASHEET
	// assert MSB to enable register address auto-increment
	uint8_t rxBuffer[2];
	uint8_t register_pointer = TEMP_OUT_L_LPS | (1 << 7);
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LPS, &register_pointer, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LPS, rxBuffer, 2, 100);   
	HAL_Delay(20);
	
	float readVal = (int16_t)(rxBuffer[1] << 8 | rxBuffer[0]);
	return (float)(42.5 + (float)readVal / 480);
	
}

/*****************LIS*****************
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS_init() {
	uint8_t rxBuffer[2];
	uint8_t txdata[2];
	// PAGE 24 DATASHEET
	// WHO AM I, 0x3D - 61
	txdata[0] = WHO_AM_I_REG_LIS;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, &txdata[0], 1, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 1, 100);   
	HAL_Delay(10);
	printf("Who Am I : 0x%x\n\r",rxBuffer[0]);
	
	// 0x70 = 0b01110000
	// OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
	txdata[0] = CTRL_REG1_LIS;
	txdata[1] = 0x70;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 2, 100);   
	HAL_Delay(10);

	// 0x00 = 0b00000000
	// FS = 00 (+/- 4 gauss full scale)
	txdata[0] = CTRL_REG2_LIS;
	txdata[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 2, 100);   
	HAL_Delay(10);

	// 0x00 = 0b00000000
	// MD = 00 (continuous-conversion mode)
	txdata[0] = CTRL_REG3_LIS;
	txdata[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 2, 100);   
	HAL_Delay(10);

	// 0x0C = 0b00001100
	// OMZ = 11 (ultra-high-performance mode for Z)
	txdata[0] = CTRL_REG4_LIS;
	txdata[1] = 0x0C;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 2, 100);   
	HAL_Delay(10);
	
}

//read magnometer
void readLIS(){
//	uint16_t *m = malloc(3)
	uint8_t rxBuffer[6];
	uint8_t register_pointer = OUT_X_L_LIS | 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LIS, &register_pointer, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LIS, rxBuffer, 6, 100);   
	HAL_Delay(20);
	uint8_t xlm = rxBuffer[0];
	uint8_t xhm = rxBuffer[1];
	uint8_t ylm = rxBuffer[2];
	uint8_t yhm = rxBuffer[3];
	uint8_t zlm = rxBuffer[4];
	uint8_t zhm = rxBuffer[5];
	
	int16_t m[3];
	m[0] = (int16_t)(xhm<<8 | xlm);
	m[1] = (int16_t)(yhm<<8 | ylm);
	m[2] = (int16_t)(zhm<<8 | zlm);
	printf("Mag - x: %i, y: %i, z:%i\n\r",m[0], m[1], m[2]);
	
}

/*****************LSM*****************
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM_init() {
	uint8_t rxBuffer[2];
	uint8_t txdata[2];
	
	// WHO AM I, 0x69, 105
	txdata[0] = WHO_AM_I_LSM;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, &txdata[0], 1, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 1, 100);   
	HAL_Delay(10);
	printf("Who Am I : 0x%x\n\r", rxBuffer[0]);
	
	
	// Accelerometer
	// 0x80 = 0b10000000
	// ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
	txdata[0] = CTRL1_XL_LSM;
	txdata[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 2, 100);   
	HAL_Delay(10);
	
	
	// Gyro
	// 0x80 = 0b010000000
	// ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
	txdata[0] = CTRL2_G_LSM;
	txdata[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 2, 100);   
	HAL_Delay(10);


	// Common
	// 0x04 = 0b00000100
	// IF_INC = 1 (automatically increment register address)
	txdata[0] = CTRL3_C_LSM;
	txdata[1] = 0x04;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, txdata, 2, 100);
	HAL_Delay(10);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 2, 100);   
	HAL_Delay(10);

}

void readAcc(){
	uint8_t rxBuffer[6];
	uint8_t register_pointer = OUTX_L_XL_LSM;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, &register_pointer, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 6, 100);   
	HAL_Delay(20);
	
	int16_t ax,ay,az;
	ax = (int16_t)(rxBuffer[1] << 8 | rxBuffer[0]);
  ay = (int16_t)(rxBuffer[3] << 8 | rxBuffer[2]);
  az = (int16_t)(rxBuffer[5] << 8 | rxBuffer[4]);
	
	printf("Acc - x: %i, y: %i, z:%i\n\r",ax,ay,az);
	
}

void readGyro(){
	uint8_t rxBuffer[6];
	uint8_t register_pointer = OUTX_L_G_LSM;
	HAL_I2C_Master_Transmit(&hi2c1, devAdd_LSM, &register_pointer, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, devAdd_LSM, rxBuffer, 6, 100);   
	HAL_Delay(20);
	
	int16_t gx,gy,gz;
	gx = (int16_t)(rxBuffer[1] << 8 | rxBuffer[0]);
  gy = (int16_t)(rxBuffer[3] << 8 | rxBuffer[2]);
  gz = (int16_t)(rxBuffer[5] << 8 | rxBuffer[4]);
	
	printf("Gyro - x: %i, y: %i, z:%i\n\r", gx, gy, gz);
	
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	/* Checks if target device is ready for communication. 3 is number of trials, 1000ms is timeout */
	
	// https://github.com/pololu/lps-arduino/blob/master/LPS.cpp
	// https://www.eedblog.com/2018/02/03/eedblog-2-how-to-use-hal-library-i2c-functions/
  // https://electronics.stackexchange.com/questions/244491/addressing-registers-with-i2c-stm32f0-hal-libraries
	
	if (HAL_I2C_IsDeviceReady(&hi2c1, devAdd_LPS, 3, 1000) != HAL_OK) {
		printf("LPS25H IsDeviceReady ERROR!!! \n\r");
		/* Return error */
		return HAL_ERROR;
	}
	HAL_Delay(200);
	LPS_init();	
	
	if (HAL_I2C_IsDeviceReady(&hi2c1, devAdd_LIS, 3, 1000) != HAL_OK) {
		printf("LIS3MDL IsDeviceReady ERROR!!! \n\r");
		/* Return error */
		return HAL_ERROR;
	}
	HAL_Delay(200);
	LIS_init();	
	
	if (HAL_I2C_IsDeviceReady(&hi2c1, devAdd_LIS, 3, 1000) != HAL_OK) {
		printf("LSM6DS33 IsDeviceReady ERROR!!! \n\r");
		/* Return error */
		return HAL_ERROR;
	}
	HAL_Delay(200);
	LSM_init();	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	printf("\n\n\n\r Start while loop!!\n\r");
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	printf("\n\r**********\n\r");
	HAL_Delay(1000);
	/*Measurements from LPS*/
	float pressure = readPressure();
	printf("Pressure - P: %f (mbar)/hectopascals (hPa)\n\r", pressure);
  float altitude = pressureToAltitudeMeters(pressure);
	printf("Altitude - P: %f meters \n\r", altitude);
	float temperature = readTemperature();
	printf("Temperature - P: %f C \n\r", temperature);
	
	/*Measurements from LIS, magnometer*/
	readLIS();

	/*Measurements from LSM6*/
	readAcc();
	readGyro();

	
  /* USER CODE END 3 */

	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE9 
                           PE10 PE11 PE12 PE13 
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA11 PA12 PA13 
                           PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
