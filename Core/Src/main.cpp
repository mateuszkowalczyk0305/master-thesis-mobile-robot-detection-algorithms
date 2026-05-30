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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <motion/robot.hpp>

#include "IrSensors.h"
#include "UltrasonicSensor.h"
#include "RPLidar.h"

#include "DebugData.h"
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /******************************************************/
  /* Motion system initialization */

  /* Motors */
  Motor m1(&htim4, TIM_CHANNEL_3, &htim4, TIM_CHANNEL_4);
  Motor m2(&htim4, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_1);
  Motor m3(&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4);
  Motor m4(&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2);

  /* Wheels */
  Wheel w1(m1);
  Wheel w2(m2);
  Wheel w3(m3);
  Wheel w4(m4);

  /* Robot */
  Robot robot(w1, w2, w3, w4);

  /******************************************************/

  /* IR detection initialization */
  IrSensors irSensors(&hadc1);
  irSensors.startDma();

  /******************************************************/

  /* Ultrasonic detection initialization */
  UltrasonicSensor leftUltrasonic(
      &htim2,
      GPIOB, GPIO_PIN_10,   // TRIG LEFT
      GPIOB, GPIO_PIN_1     // ECHO LEFT
  );

  UltrasonicSensor rightUltrasonic(
      &htim2,
      GPIOB, GPIO_PIN_0,    // TRIG RIGHT
      GPIOB, GPIO_PIN_2     // ECHO RIGHT
  );

  leftUltrasonic.init();
  rightUltrasonic.init();

  leftUltrasonic.registerForExti();
  rightUltrasonic.registerForExti();

  /******************************************************/

  /* LiDAR detection initialization */
  RPLidar lidar(&huart2, &htim3, TIM_CHANNEL_2);

  lidar.init();

  RPLidarPoint lidarPoint;
  RPLidarCluster currentCluster;
  RPLidarCluster bestCluster;
  RPLidarDetectedObject detectedObject;
  RPLidarObjectSector lidarSector;

  robot.forward(1000);

  /******************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  robot.update();
/************************************** IR SENSORS **********************************/
//	  irSensors.update();
//
//	  IrSensors::Sector sector = irSensors.getSector();
//
//	  IrSensors::SensorData leftData =
//		  irSensors.getSensorData(IrSensors::Sensor::Left);
//
//	  IrSensors::SensorData centerData =
//		  irSensors.getSensorData(IrSensors::Sensor::Center);
//
//	  IrSensors::SensorData rightData =
//		  irSensors.getSensorData(IrSensors::Sensor::Right);
//
//	  if (sector == IrSensors::Sector::Center)
//	  {
//		  // Przeciwnik na środku.
//		  // TODO: jedź prosto.
//	  }
//	  else if (sector == IrSensors::Sector::Left)
//	  {
//		  // Przeciwnik po lewej.
//		  // TODO: skręć w lewo.
//	  }
//	  else if (sector == IrSensors::Sector::Right)
//	  {
//		  // Przeciwnik po prawej.
//		  // TODO: skręć w prawo.
//	  }
//	  else if (sector == IrSensors::Sector::FrontWide)
//	  {
//		  // Przeciwnik wykryty szeroko z przodu.
//		  // TODO: atak prosto.
//	  }
//	  else
//	  {
//		  // Brak przeciwnika.
//		  // TODO: szukanie przeciwnika.
//	  }
//
//	  HAL_Delay(40);
///////////////////////////////////////////////////////////
//	    irSensors.update();
//
//	    IrSensors::SensorData leftData =
//	        irSensors.getSensorData(IrSensors::Sensor::Left);
//
//	    IrSensors::SensorData centerData =
//	        irSensors.getSensorData(IrSensors::Sensor::Center);
//
//	    IrSensors::SensorData rightData =
//	        irSensors.getSensorData(IrSensors::Sensor::Right);
//
//	    IrSensors::Sector sector = irSensors.getSector();
//
//	    irDebug.leftRaw = leftData.adcRaw;
//	    irDebug.centerRaw = centerData.adcRaw;
//	    irDebug.rightRaw = rightData.adcRaw;
//
//	    irDebug.leftVoltage = leftData.voltage;
//	    irDebug.centerVoltage = centerData.voltage;
//	    irDebug.rightVoltage = rightData.voltage;
//
//	    irDebug.leftDistance = leftData.distanceCm;
//	    irDebug.centerDistance = centerData.distanceCm;
//	    irDebug.rightDistance = rightData.distanceCm;
//
//	    irDebug.leftFiltered = leftData.filteredCm;
//	    irDebug.centerFiltered = centerData.filteredCm;
//	    irDebug.rightFiltered = rightData.filteredCm;
//
//	    irDebug.leftDetected = leftData.detected ? 1 : 0;
//	    irDebug.centerDetected = centerData.detected ? 1 : 0;
//	    irDebug.rightDetected = rightData.detected ? 1 : 0;
//
//	    irDebug.sector = static_cast<uint8_t>(sector);
//
//	    HAL_Delay(40);
/////////////////////////////////////////////************************** IR SENSORS *********************//





//	  ///////////////////////////////////////////// US METODA ?????///////////////
//	  float leftDistance = 0.0f;
//	  float rightDistance = 0.0f;
//
//	  bool leftOk = leftUltrasonic.measure(leftDistance);
//	  HAL_Delay(25); // przerwa, żeby echo z lewego czujnika wygasło
//
//	  bool rightOk = rightUltrasonic.measure(rightDistance);
//
//	  ultrasonicDebug.leftOk = leftOk ? 1 : 0;
//	  ultrasonicDebug.rightOk = rightOk ? 1 : 0;
//
//	  if (leftOk)
//	  {
//	      ultrasonicDebug.leftDistance = leftDistance;
//	      ultrasonicDebug.leftCounter++;
//	  }
//	  else
//	  {
//	      ultrasonicDebug.leftErrorCounter++;
//	  }
//
//	  if (rightOk)
//	  {
//	      ultrasonicDebug.rightDistance = rightDistance;
//	      ultrasonicDebug.rightCounter++;
//	  }
//	  else
//	  {
//	      ultrasonicDebug.rightErrorCounter++;
//	  }
//
//	  HAL_Delay(100);
///////////////////////////////////////////// US METODA ?????///////////////


	/************************************** LiDAR **********************************/

//	  if (lidar.getPoint(lidarPoint))
//	    {
//	        lidarDebug.angleDeg = lidarPoint.angleDeg;
//	        lidarDebug.distanceMm = lidarPoint.distanceMm;
//	        lidarDebug.quality = lidarPoint.quality;
//	        lidarDebug.startFlag = lidarPoint.startFlag ? 1 : 0;
//	        lidarDebug.valid = lidarPoint.valid ? 1 : 0;
//
//	        if (lidar.isPointInDetectionZone(lidarPoint))
//	        {
//	            lidarDebug.inDetectionZone = 1;
//
//	            lidarDebug.filteredAngleDeg = lidarPoint.angleDeg;
//	            lidarDebug.filteredDistanceMm = lidarPoint.distanceMm;
//	            lidarDebug.filteredQuality = lidarPoint.quality;
//	        }
//	        else
//	        {
//	            lidarDebug.inDetectionZone = 0;
//	        }
//
//	        lidar.processPointForDetection(lidarPoint);
//	    }
//
//	    currentCluster = lidar.getCurrentCluster();
//	    bestCluster = lidar.getBestCluster();
//	    lidar.getDetectedObject(detectedObject);
//	    lidarSector = lidar.getObjectSector();
//
//	    lidarDebug.receivedBytes = lidar.receivedBytes;
//	    lidarDebug.validPoints = lidar.validPoints;
//	    lidarDebug.invalidNodes = lidar.invalidNodes;
//	    lidarDebug.filteredPoints = lidar.filteredPoints;
//
//	    lidarDebug.clustersCreated = lidar.clustersCreated;
//	    lidarDebug.detectedObjects = lidar.detectedObjects;
//
//	    lidarDebug.currentClusterValid = currentCluster.valid ? 1 : 0;
//	    lidarDebug.currentClusterPoints = currentCluster.pointsCount;
//	    lidarDebug.currentClusterStartAngle = currentCluster.startAngleDeg;
//	    lidarDebug.currentClusterEndAngle = currentCluster.endAngleDeg;
//	    lidarDebug.currentClusterCenterAngle = currentCluster.centerAngleDeg;
//	    lidarDebug.currentClusterDistance = currentCluster.averageDistanceMm;
//	    lidarDebug.currentClusterWidth = currentCluster.widthDeg;
//
//	    lidarDebug.bestClusterValid = bestCluster.valid ? 1 : 0;
//	    lidarDebug.bestClusterPoints = bestCluster.pointsCount;
//	    lidarDebug.bestClusterCenterAngle = bestCluster.centerAngleDeg;
//	    lidarDebug.bestClusterDistance = bestCluster.averageDistanceMm;
//	    lidarDebug.bestClusterWidth = bestCluster.widthDeg;
//
//	    lidarDebug.objectDetected = detectedObject.detected ? 1 : 0;
//	    lidarDebug.objectAngle = detectedObject.angleDeg;
//	    lidarDebug.objectDistance = detectedObject.distanceMm;
//	    lidarDebug.objectWidth = detectedObject.clusterWidthDeg;
//	    lidarDebug.objectPoints = detectedObject.pointsCount;
//	    lidarDebug.objectSector = static_cast<uint8_t>(lidarSector);

	/******************************************************************************/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
