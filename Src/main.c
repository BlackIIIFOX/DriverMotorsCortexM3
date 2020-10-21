/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Текущее состояние работы драйвера линейных двигателей. */
enum TypeWork {
  /* Режим инициализации. В данном режиме происходит
  расчет минимальных и максимальных положений кисти и пальцев протеза. */
  InitializationMode,
  
  /* Режим ожидания. Механические действия не исполняются. */
  SleepMode,
  
  /* Режим установки новых положений. */
  SettingPositionMode,
  
  /* Режим ошибки. Работа невозможна. */
  ErrorMode
};

/* Команды, которые протез может принимать по протоколу */
enum ProtocolCommand {
  /* Телеметрия устройства. Протез возвращает позиции пальцев. */
  Telemetry,
  
  /* Запрос установки позиций протеза. */
  SetPositions
};

/* Структура для работы с движущимися частями протеза (пальцами и кистью). 
* Хранит информацию о расчетах энкордера и физическом подключении протеза.
*/
typedef struct {
  /* Текущая позиция пальца в угловом соотношении. */
  uint8_t Position;
  
  /* Ожидаемая позиция пальца в угловом соотношении. */
  uint8_t RequiredPosition;
  
  /* Минимальное значение потенциометра после колибровки. */
  uint16_t MinADCValue;
  
  /* Максимальное значение потенциометра после колибровки. */
  uint16_t MaxADCValue;
  
  /* Значение потенциометра. */
  uint16_t* ADCValue;
  
  /* Структура GPIO к которой подключен вывод мотора для движения вперед
  (Движение вперед - при подаче логической единицы на этот пин мотор будет
  разжимать палец). */
  GPIO_TypeDef* GPIO_MotorForward;
  
  /* Номер пина к которому подключен вывод мотора для движения вперед. */
  uint16_t GPIO_Motor_PinForward;
  
  /* Структура GPIO к которой подключен вывод мотора для движения назад
  (Движение назад - при подаче логической единицы на этот пин мотор будет
  сжимать палец). */
  GPIO_TypeDef* GPIO_MotorBackward;
  
  /* Номер пина к которому подключен вывод мотора для движения назад. */
  uint16_t GPIO_Motor_PinBackward;
} FingerStruct;

/* Позиции всех движущихся частей протеза.*/
typedef struct {
  enum TypeWork CurrentRegime;
  FingerStruct* PointerFinger;
  FingerStruct* MiddleFinger;
  FingerStruct* RingFinder;
  FingerStruct* LittleFinger;
  FingerStruct* ThumbFinger;
  FingerStruct* ThumbFingerFlexion;
} HandStruct;

typedef struct {
  enum ProtocolCommand Command;
  enum TypeWork CurrentRegime;
  uint8_t PointerFingerPosition;
  uint8_t MiddleFingerPosition;
  uint8_t RingFinderPosition;
  uint8_t LittleFingerPosition;
  uint8_t ThumbFingerPosition;
  uint8_t CRC8;
} ProtocolStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char CRC8_TABLE[] = {
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

/* Основные переменные */

/* Основная конфигурация протеза для осуществления движений. 
 * На ее основе работает вся прошивка МК. */
HandStruct* HandConfig;

/* CAN BUS PV Variables */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* UART PV Variables */
uint32_t uartLastReceiveTimeMs;
uint8_t uartPositionProtocol;

/* ADC PV Variables */
volatile uint16_t ADC_Data[6];

/* UART data receive */
ProtocolStruct request;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Объявление функций. */

/** 
* @brief Выполняет инициализацию конфигурации протеза.
* Устанавливает пины, к которым подключены моторы пальцев.
* @retval Структура, в которой содержится конфигурация протеза.
*/
HandStruct* Hand_Init();

/**
* @brief  Выполнение колибровки указанного пальца.
* @param  finger Структура, с набором характеристик пальца.
*/
void CalibrateFinger(FingerStruct* finger);

/** 
* @brief Выполняет инициализацию конфигурации отдельного пальца. Выполняет 
*       установку конфигурации пина в соотвесвтующие поля структуры.
* @param  motorForward Структура GPIO к которой подключен вывод мотора для движения вперед.
*       (Движение вперед - при подаче логической единицы на этот пин мотор будет разжимать палец).
* @param  pinForward Номер пина к которому подключен вывод мотора для движения вперед.
* @param  motorBackward Структура GPIO к которой подключен вывод мотора для движения назад. 
*       (Движение назад - при подаче логической единицы на этот пин мотор будет сжимать палец).
* @param  pinBackward Номер пина к которому подключен вывод мотора для движения вперед.
* @param  ADCValue Указатель на значение АЦП.
* @retval Структура, в которой содержится конфигурация отдельного пальца.
*/
FingerStruct* Finger_Init(GPIO_TypeDef *motorForward, uint16_t pinForward, GPIO_TypeDef *motorBackward, uint16_t pinBackward, uint16_t* ADCValue);

/** 
* @brief Выполняет инициализацию конфигурации отдельного пальца. 
* @retval None
*/
void SendTelemetryByCAN();

/**
  * @brief  CRC8 calculate function
  * @param  data Pointer to data
  * @param  data length
  * @retval crc8
*/
unsigned char CalculateCRC8(unsigned char *data, unsigned int length);

/* Реализация функций. */

void SendTelemetryByCAN(HandStruct* handConfig)
{
  TxData[0] = handConfig->CurrentRegime;
  TxData[1] = handConfig->PointerFinger->Position;
  TxData[2] = handConfig->MiddleFinger->Position;
  TxData[3] = handConfig->RingFinder->Position;
  TxData[4] = handConfig->LittleFinger->Position;
  TxData[5] = handConfig->ThumbFinger->Position;
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

HandStruct* Hand_Init()
{
  HandStruct* newHandConfig = (HandStruct*)malloc(sizeof(HandStruct));
  memset(newHandConfig, 0, sizeof(HandStruct));
  
  // MOTOR 1
  newHandConfig->LittleFinger = Finger_Init(GPIOB, GPIO_PIN_10, 
                                             GPIOB, GPIO_PIN_11, (uint16_t*)&ADC_Data[6]);
  // MOTOR 2
  newHandConfig->RingFinder = Finger_Init(GPIOB, GPIO_PIN_0, 
                                             GPIOB, GPIO_PIN_1, (uint16_t*)&ADC_Data[5]);
  // MOTOR 3
  newHandConfig->MiddleFinger = Finger_Init(GPIOB, GPIO_PIN_3, 
                                             GPIOB, GPIO_PIN_12, (uint16_t*)&ADC_Data[4]);
  // MOTOR 4
  newHandConfig->PointerFinger = Finger_Init(GPIOB, GPIO_PIN_5, 
                                             GPIOB, GPIO_PIN_4, (uint16_t*)&ADC_Data[3]);
  // MOTOR 5
  newHandConfig->ThumbFinger = Finger_Init(GPIOB, GPIO_PIN_6, 
                                             GPIOB, GPIO_PIN_7, (uint16_t*)&ADC_Data[2]);
  // MOTOR 6
  newHandConfig->ThumbFingerFlexion = Finger_Init(GPIOB, GPIO_PIN_8, 
                                             GPIOB, GPIO_PIN_9, (uint16_t*)&ADC_Data[1]);
  
  return newHandConfig;
}

FingerStruct* Finger_Init(GPIO_TypeDef *motorForward, uint16_t pinForward, GPIO_TypeDef *motorBackward, uint16_t pinBackward, uint16_t* ADCValue)
{
  FingerStruct* configFinger = (FingerStruct*)malloc(sizeof(FingerStruct));
  memset(configFinger, 0, sizeof(FingerStruct));
  
  configFinger->GPIO_MotorForward = motorForward;
  configFinger->GPIO_Motor_PinForward = pinForward;
  configFinger->GPIO_MotorBackward = motorBackward;
  configFinger->GPIO_Motor_PinBackward = pinBackward;
  configFinger->ADCValue = ADCValue;
  
  return configFinger;
}

/**
* @brief  Остановка движения мотора.
*/
void Finger_Stop(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_RESET);
}

/**
* @brief  Движение мотора вперед (сжатие).
*/
void Finger_Forward(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_RESET);
}

/**
* @brief  Движение мотора назад (разжатие).
*/
void Finger_Backward(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_SET);
}

/**
* @brief  Выполняет установку положения пальца в указанною позицию.
* @param  finger Структура, с набором характеристик пальца.
*/
void Set_Finger_Position(FingerStruct* finger)
{ //0 - разжато, 180 - сжато
  uint16_t angle = finger->RequiredPosition - finger->Position;
  uint16_t reqPosition = abs(angle) * ((finger->MaxADCValue - finger->MinADCValue) / 180);
  
  if (angle > 0) {
    //Сжать руку на angle градусов
    Finger_Forward(finger);
    while(true) {
      if (*finger->ADCValue <= reqPosition) {
        Finger_Stop(finger);
        
        break;
      }
    }
  } else {
    //Разжать руку на angle градусов
    Finger_Backward(finger);
    while(true) {
      if (*finger->ADCValue >= reqPosition) {
        Finger_Stop(finger);
        
        break;
      }
    }
  }
  
  finger->Position = finger->RequiredPosition;
}

/**
* @brief  Выполняет установку положения пальца в указанною позицию.
* @param  hand Структура, с набором характеристик руки.
*/
/*
void Set_Hand_Position(HandStruct* hand)
{
  uint16_t LittleFingerAngle = hand->LittleFinger->RequiredPosition - hand->LittleFinger->Position;
  uint16_t RingFinderAngle = hand->RingFinder->RequiredPosition - hand->RingFinder->Position;
  uint16_t MiddleFingerAngle = hand->MiddleFinger->RequiredPosition - hand->MiddleFinger->Position;
  uint16_t PointerFingerAngle = hand->PointerFinger->RequiredPosition - hand->PointerFinger->Position;
  uint16_t ThumbFingerFlexionAngle = hand->ThumbFingerFlexion->RequiredPosition - hand->ThumbFingerFlexion->Position;
  uint16_t ThumbFingerAngle = hand->ThumbFinger->RequiredPosition - hand->ThumbFinger->Position;
  
  uint16_t LittleFingerReqPosition = abs(LittleFingerAngle) * ((hand->LittleFinger->MaxADCValue - hand->LittleFinger->MinADCValue) / 180);
  uint16_t RingFinderReqPosition = abs(RingFinderAngle) * ((hand->RingFinder->MaxADCValue - hand->RingFinder->MinADCValue) / 180);
  uint16_t MiddleFingerReqPosition = abs(MiddleFingerAngle) * ((hand->MiddleFinger->MaxADCValue - hand->MiddleFinger->MinADCValue) / 180);
  uint16_t PointerFingerReqPosition = abs(PointerFingerAngle) * ((hand->PointerFinger->MaxADCValue - hand->PointerFinger->MinADCValue) / 180);
  uint16_t ThumbFingerFlexionReqPosition = abs(ThumbFingerFlexionAngle) * ((hand->ThumbFingerFlexion->MaxADCValue - hand->ThumbFingerFlexion->MinADCValue) / 180);
  uint16_t ThumbFingerReqPosition = abs(ThumbFingerAngle) * ((hand->ThumbFinger->MaxADCValue - hand->ThumbFinger->MinADCValue) / 180);
  
  bool LittleFingerSeted = false;
  bool RingFinderSeted = false;
  bool MiddleFingerSeted = false;
  bool PointerFingerSeted = false;
  bool ThumbFingerFlexionSeted = false;
  bool RequiredPositionSeted = false;
  
  
}
*/

/**
* @brief  Выполнение колибровки руки.
* @param  hand Структура, с набором характеристик руки.
*/
void Calibrate(HandStruct* hand)
{
  CalibrateFinger(hand->LittleFinger);
  CalibrateFinger(hand->RingFinder);
  CalibrateFinger(hand->MiddleFinger);
  
  uint16_t PointerFingerADCMaxPosition = *(hand->PointerFinger->ADCValue);
  uint16_t ThumbFingerADCMaxPosition = *(hand->ThumbFinger->ADCValue);
  uint16_t ThumbFingerFlexionADCMaxPosition = *(hand->ThumbFingerFlexion->ADCValue);
  
  uint16_t PointerFingerADCMinPosition;
  uint16_t ThumbFingerADCMinPosition;
  uint16_t ThumbFingerFlexionADCMinPosition;
  
  uint32_t receiveTimeMs = HAL_GetTick();
  
  Finger_Backward(hand->PointerFinger);
  Finger_Backward(hand->ThumbFinger);
  Finger_Backward(hand->ThumbFingerFlexion);
  
  bool PointerFingerCalibrated = false;
  bool ThumbFingerCalibrated = false;
  bool ThumbFingerFlexionCalibrated = false;
  
  uint8_t ConfirmPointerFinger = 100;
  uint8_t ConfirmThumbFinger = 100;
  uint8_t ConfirmThumbFingerFlexion = 100;
  
  //Установка максимальных значений при разжатии пальцев (значениея малы ~100-200)
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (PointerFingerCalibrated && ThumbFingerFlexionCalibrated && ThumbFingerCalibrated) {
        break;
      }
      
      if (!PointerFingerCalibrated) {
        if (ConfirmPointerFinger < 1) {
          PointerFingerADCMaxPosition = *(hand->PointerFinger->ADCValue);
          Finger_Stop(hand->PointerFinger);
          PointerFingerCalibrated = true;
          ConfirmPointerFinger = 100;
        }
        
        if (*(hand->PointerFinger->ADCValue) + 150 < PointerFingerADCMaxPosition) {
          PointerFingerADCMaxPosition = *(hand->PointerFinger->ADCValue);      
        } else {
          ConfirmPointerFinger--;
        }
      }
      
      if (!ThumbFingerCalibrated) {
        if (ConfirmThumbFinger < 1) {
          ThumbFingerADCMaxPosition = *(hand->ThumbFinger->ADCValue);
          Finger_Stop(hand->ThumbFinger);
          ThumbFingerCalibrated = true;
          ConfirmThumbFinger = 100;
        }
        
        if (*(hand->ThumbFinger->ADCValue) + 150 < ThumbFingerADCMaxPosition) {
          ThumbFingerADCMaxPosition = *(hand->ThumbFinger->ADCValue);      
        } else {
          ConfirmThumbFinger--;
        }
      }
      
      if (!ThumbFingerFlexionCalibrated) {
        if (ConfirmThumbFingerFlexion < 1) {
          ThumbFingerFlexionADCMaxPosition = *(hand->ThumbFingerFlexion->ADCValue);
          Finger_Stop(hand->ThumbFingerFlexion);
          ThumbFingerFlexionCalibrated = true;
          ConfirmThumbFingerFlexion = 100;
        }
        
        if (*(hand->ThumbFingerFlexion->ADCValue) + 150 < ThumbFingerFlexionADCMaxPosition) {
          ThumbFingerFlexionADCMaxPosition = *(hand->ThumbFingerFlexion->ADCValue);      
        } else {
          ConfirmThumbFingerFlexion--;
        }
      }      
    }
  }  

  PointerFingerADCMinPosition = PointerFingerADCMaxPosition;
  ThumbFingerADCMinPosition = ThumbFingerADCMaxPosition;
  ThumbFingerFlexionADCMinPosition = ThumbFingerFlexionADCMaxPosition;
  
  Finger_Forward(hand->ThumbFinger);
  
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (ConfirmThumbFinger < 1) {
        ThumbFingerADCMinPosition = *(hand->ThumbFinger->ADCValue);
        Finger_Stop(hand->ThumbFinger);
        
        break;
      }
      
      if (*(hand->ThumbFinger->ADCValue) - 150 > ThumbFingerADCMinPosition) {
        ThumbFingerADCMinPosition = *(hand->ThumbFinger->ADCValue);      
      } else {
        ConfirmThumbFinger--;
      }
    }  
  }
  
  hand->ThumbFinger->MaxADCValue = ThumbFingerADCMaxPosition - 150;
  hand->ThumbFinger->MinADCValue = ThumbFingerADCMinPosition + 150;
  hand->ThumbFinger->Position = 180;
  hand->ThumbFinger->RequiredPosition = 0;
  Set_Finger_Position(hand->ThumbFinger);
  
  Finger_Forward(hand->ThumbFingerFlexion);
  
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (ConfirmThumbFingerFlexion < 1) {
        ThumbFingerFlexionADCMinPosition = *(hand->ThumbFingerFlexion->ADCValue);
        Finger_Stop(hand->ThumbFingerFlexion);
        
        break;
      }
      
      if (*(hand->ThumbFingerFlexion->ADCValue) - 150 > ThumbFingerFlexionADCMinPosition) {
        ThumbFingerFlexionADCMinPosition = *(hand->ThumbFingerFlexion->ADCValue);      
      } else {
        ConfirmThumbFingerFlexion--;
      }
    }  
  }
  
  hand->ThumbFingerFlexion->MaxADCValue = ThumbFingerFlexionADCMaxPosition - 150;
  hand->ThumbFingerFlexion->MinADCValue = ThumbFingerFlexionADCMinPosition + 150;
  hand->ThumbFingerFlexion->Position = 180;
  hand->ThumbFingerFlexion->RequiredPosition = 0;
  Set_Finger_Position(hand->ThumbFingerFlexion);
  
  Finger_Forward(hand->PointerFinger);
  
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (ConfirmPointerFinger < 1) {
        PointerFingerADCMinPosition = *(hand->PointerFinger->ADCValue);
        Finger_Stop(hand->PointerFinger);
        
        break;
      }
      
      if (*(hand->PointerFinger->ADCValue) - 150 > PointerFingerADCMinPosition) {
        PointerFingerADCMinPosition = *(hand->PointerFinger->ADCValue);      
      } else {
        ConfirmPointerFinger--;
      }
    }  
  }
  
  hand->PointerFinger->MaxADCValue = PointerFingerADCMaxPosition - 150;
  hand->PointerFinger->MinADCValue = PointerFingerADCMinPosition + 150;
  hand->PointerFinger->Position = 180;
  hand->PointerFinger->RequiredPosition = 180;
}

void CalibrateFinger(FingerStruct* finger)
{
  uint8_t Confirm = 100;
  uint16_t FingerADCMaxPosition = *(finger->ADCValue);  //Максимальное значение с потенциометра
  uint16_t FingerADCMinPosition;                        //Минимальное значение с потенциометра
  
  uint32_t receiveTimeMs = HAL_GetTick();
    
  Finger_Backward(finger);
  
  //Установка максимального значения при разжатии пальца (значение мало ~100-200)
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (Confirm < 1) {
        FingerADCMaxPosition = *(finger->ADCValue);
        Finger_Stop(finger);
        Confirm = 100;
        
        break;
      }
      
      if (*(finger->ADCValue) + 150 < FingerADCMaxPosition) {
        FingerADCMaxPosition = *(finger->ADCValue);      
      } else {
        Confirm--;
      }
    }  
  }
    
  FingerADCMinPosition = FingerADCMaxPosition;
  Finger_Forward(finger);
  
  //Установка минимального значений при сжатии пальца (значение велико ~3700-4000)
  while(true) {
    uint32_t tmpTimeMs = HAL_GetTick();
    
    if (receiveTimeMs + 4000 < tmpTimeMs)
    {
      receiveTimeMs = tmpTimeMs;
      
      if (Confirm < 1) {
        FingerADCMinPosition = *(finger->ADCValue);
        Finger_Stop(finger);
        
        break;
      }
      
      if (*(finger->ADCValue) - 150 > FingerADCMinPosition) {
        FingerADCMinPosition = *(finger->ADCValue);      
      } else {
        Confirm--;
      }
    }  
  }  
  
  finger->MaxADCValue = FingerADCMaxPosition - 150;
  finger->MinADCValue = FingerADCMinPosition + 150;
  
  finger->Position = 180;
  finger->RequiredPosition = 180;
}

/**
* @brief Выполняет необходимую программную иницилизацию CAN и 
* переменных для работы с ней.
* @retval None
*/
void CAN_Init()
{
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0X0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_CAN_ActivateNotification (&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    Error_Handler();
  }
  
  TxHeader.StdId = 0x3E9;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
  TxData[0] = 0;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;
}

unsigned char CalculateCRC8(unsigned char *data, unsigned int length)
{
  unsigned char result = 0;
  for (int i = 0; i < length; i++)
  {
    result = CRC8_TABLE[result ^ data[i]];
  }
  
  return result;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  HandConfig = Hand_Init();
  
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data,6);
  // HAL_ADC_Stop_DMA(&hadc1);
  
  CAN_Init();
  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
  
  uartLastReceiveTimeMs = 0;
  
  HAL_UART_Receive_IT(&huart1, ((uint8_t*)&request), 1);
  
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HandConfig->CurrentRegime = InitializationMode;
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  //Calibrate(HandConfig);
  
  HandConfig->CurrentRegime = SleepMode;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    Finger_Forward(HandConfig->LittleFinger);
    Finger_Forward(HandConfig->MiddleFinger);
    Finger_Forward(HandConfig->PointerFinger);
    Finger_Forward(HandConfig->RingFinder);
    Finger_Forward(HandConfig->ThumbFinger);
    HAL_Delay(2000);
    Finger_Backward(HandConfig->LittleFinger);
    Finger_Backward(HandConfig->MiddleFinger);
    Finger_Backward(HandConfig->PointerFinger);
    Finger_Backward(HandConfig->RingFinder);
    Finger_Backward(HandConfig->ThumbFinger);
    HAL_Delay(2000);
    Finger_Stop(HandConfig->LittleFinger);
    Finger_Stop(HandConfig->MiddleFinger);
    Finger_Stop(HandConfig->PointerFinger);
    Finger_Stop(HandConfig->RingFinder);
    Finger_Stop(HandConfig->ThumbFinger);
    HAL_Delay(2000);
    
    //for(int i=0;i<6;i++)
    //{
      //float u;
      //char str[9];
      //u =((float)ADC_Data[i])*3/4096;//занесём результат преобразований в переменную
      //sprintf(str,"%.2fv",u);//преобразуем результат в строку
      //sprintf(str, "%d", ADC_Data[i]);//преобразуем результат в строку
      
      //UART_Transmit_Text(str);
      //UART_Transmit_Text("\n");
      //break;
      // UART_Transmit_Text(" ");
    //}
    
    //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  char text[20];
  sprintf(text, "%d,%d\n", HAL_GetTick(), ADC_Data[0]);
  HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text), 0xFFFF);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  
  uint32_t receiveTimeMs = HAL_GetTick();
  
  if (uartPositionProtocol != 0 && abs(receiveTimeMs - uartLastReceiveTimeMs) > 500)
  {
    uartLastReceiveTimeMs = receiveTimeMs;
    request.Command = (enum ProtocolCommand)(*(((uint8_t*)&request) + uartPositionProtocol));
    uartPositionProtocol = 1;
    HAL_UART_Receive_IT(&huart1, ((uint8_t*)((uint8_t*)(&request) + uartPositionProtocol)), 1);
    return;
  }
  
  uartLastReceiveTimeMs = receiveTimeMs;
  
  if (uartPositionProtocol == 7)
  {
    uartPositionProtocol = 0;
    unsigned char realCRC8 = CalculateCRC8(((unsigned char*)&request), 7);
    if (false && realCRC8 != request.CRC8)
    {
      // Ожидаем следующие 8 байт и обнуляем позицию
      HAL_UART_Receive_IT(&huart1, ((uint8_t*)&request), 1);
      return;
    }
    
    // Если это команда установки, то обновляем требуемые позиции.
    if (request.Command == SetPositions) {
      HandConfig->LittleFinger->RequiredPosition = request.LittleFingerPosition;
      HandConfig->MiddleFinger->RequiredPosition = request.MiddleFingerPosition;
      HandConfig->PointerFinger->RequiredPosition = request.PointerFingerPosition;
      HandConfig->RingFinder->RequiredPosition = request.RingFinderPosition;
      HandConfig->ThumbFinger->RequiredPosition = request.ThumbFingerPosition;
    }
    
    // Отправляем телеметрию на каждый запрос в любом случае.
    ProtocolStruct response;
    response.Command = Telemetry;
    response.CurrentRegime = HandConfig->CurrentRegime;
    response.LittleFingerPosition = HandConfig->LittleFinger->Position;
    response.MiddleFingerPosition = HandConfig->MiddleFinger->Position;
    response.PointerFingerPosition = HandConfig->PointerFinger->Position;
    response.RingFinderPosition = HandConfig->RingFinder->Position;
    response.ThumbFingerPosition = HandConfig->ThumbFinger->Position;
    response.CRC8 = CalculateCRC8(((unsigned char*)&response), 7);
    HAL_UART_Transmit(&huart1, ((uint8_t*)&response), 8, 0xFFFF);
  }
  else
  {
    uartPositionProtocol++;
  }
  
  
  // Ожидаем следующие 8 байт
  HAL_UART_Receive_IT(&huart1, ((uint8_t*)((uint8_t*)(&request) + uartPositionProtocol)), 1);
}

void CAN_HandlePackage(uint8_t* dataPackage)
{
  if (dataPackage[0] == 0x04)
  {
    // Запрос установки новых положений.
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
  HandConfig->CurrentRegime = ErrorMode;
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
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
