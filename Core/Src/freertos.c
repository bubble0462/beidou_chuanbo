/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_MODE_RD = 0,
  APP_MODE_RN
} app_mode_t;

typedef enum
{
  APP_UART_SOURCE_BT = 0,
  APP_UART_SOURCE_RD,
  APP_UART_SOURCE_GNSS
} app_uart_source_t;

typedef struct
{
  uint8_t source;
  uint16_t length;
  uint8_t data[255U + 1U];
} app_line_msg_t;

typedef struct
{
  uint8_t debounceState;
  uint16_t holdTicks;
} app_power_key_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_MAX_LINE_SIZE              255U
#define APP_BT_QUEUE_DEPTH             4U
#define APP_MODULE_QUEUE_DEPTH         6U
#define APP_POWER_OFF_TICKS            300U
#define APP_DEBOUNCE_MS                80U
#define APP_CONTROL_TASK_DELAY_MS      20U
#define APP_RD_MODE_SETTLE_MS          300U

#define APP_BD2_TEXT_MAX_LEN           24U
#define APP_BD2_HEX_MAX_LEN            (APP_BD2_TEXT_MAX_LEN * 2U)
#define APP_BD2_FRAME_MAX_LEN          96U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const uint8_t kCardQueryCommand[] = "$CCICR,0,00*68\r\n";
static const uint8_t kBd2TxrEnableCommand[] = "$CCRMO,TXR,2,1*21\r\n";

/* 北二短报文目标地址：0362746 */
static const char kBd2TargetCardId[] = "0362746";

/* 默认发送内容 */
static const char kBd2SosText[] = "hello";

static volatile app_mode_t g_currentMode = APP_MODE_RD;
static uint8_t g_runtimeStarted = 0U;
static uint32_t g_lastSafeTick = 0U;
static uint32_t g_lastSosTick = 0U;

static uint8_t g_btRxByte = 0U;
static uint8_t g_rdRxByte = 0U;
static uint8_t g_gnssRxByte = 0U;

static uint8_t g_btRxBuffer[APP_MAX_LINE_SIZE + 1U];
static uint8_t g_rdRxBuffer[APP_MAX_LINE_SIZE + 1U];
static uint8_t g_gnssRxBuffer[APP_MAX_LINE_SIZE + 1U];
static volatile uint16_t g_btRxLength = 0U;
static volatile uint16_t g_rdRxLength = 0U;
static volatile uint16_t g_gnssRxLength = 0U;

static app_power_key_t g_powerKey = {0};
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for btTask */
osThreadId_t btTaskHandle;
const osThreadAttr_t btTask_attributes = {
  .name = "btTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for moduleTask */
osThreadId_t moduleTaskHandle;
const osThreadAttr_t moduleTask_attributes = {
  .name = "moduleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for btQueue */
osMessageQueueId_t btQueueHandle;
const osMessageQueueAttr_t btQueue_attributes = {
  .name = "btQueue"
};

/* Definitions for moduleQueue */
osMessageQueueId_t moduleQueueHandle;
const osMessageQueueAttr_t moduleQueue_attributes = {
  .name = "moduleQueue"
};

/* Definitions for rdModeSemaphore */
osSemaphoreId_t rdModeSemaphoreHandle;
const osSemaphoreAttr_t rdModeSemaphore_attributes = {
  .name = "rdModeSemaphore"
};

/* Definitions for rnModeSemaphore */
osSemaphoreId_t rnModeSemaphoreHandle;
const osSemaphoreAttr_t rnModeSemaphore_attributes = {
  .name = "rnModeSemaphore"
};

/* Definitions for safeSemaphore */
osSemaphoreId_t safeSemaphoreHandle;
const osSemaphoreAttr_t safeSemaphore_attributes = {
  .name = "safeSemaphore"
};

/* Definitions for sosSemaphore */
osSemaphoreId_t sosSemaphoreHandle;
const osSemaphoreAttr_t sosSemaphore_attributes = {
  .name = "sosSemaphore"
};

/* Definitions for btTxMutex */
osMutexId_t btTxMutexHandle;
const osMutexAttr_t btTxMutex_attributes = {
  .name = "btTxMutex"
};

/* Definitions for rdTxMutex */
osMutexId_t rdTxMutexHandle;
const osMutexAttr_t rdTxMutex_attributes = {
  .name = "rdTxMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartBtTask(void *argument);
void StartModuleTask(void *argument);

static void App_StartRuntime(void);
static void App_HandleBtMessage(const app_line_msg_t *msg);
static void App_HandleModuleMessage(const app_line_msg_t *msg);
static void App_SwitchMode(app_mode_t mode);
static void App_HandlePowerKeyScan(void);
static void App_ResetRxState(app_uart_source_t source);

static void App_SendBt(const uint8_t *data, uint16_t length);
static void App_SendBtText(const char *text);
static void App_SendRd(const uint8_t *data, uint16_t length);

static void App_HandleSosAction(void);
static void App_RestartRx(UART_HandleTypeDef *huart);
static void App_ProcessRxByteFromIsr(app_uart_source_t source, uint8_t byte);
static void App_QueueLineFromIsr(osMessageQueueId_t queue, app_uart_source_t source,
                                 uint8_t *buffer, volatile uint16_t *length);

static size_t App_NormalizeCommand(const app_line_msg_t *msg, char *command, size_t commandSize);
static uint8_t App_CommandEquals(const char *left, const char *right);

static uint8_t App_TryDecodeBdTci(const app_line_msg_t *msg, uint8_t *decoded, uint16_t *decodedLength);
static uint8_t App_TryDecodeBdTxr(const app_line_msg_t *msg, uint8_t *decoded, uint16_t *decodedLength);
static uint8_t App_IsGnrmcSentence(const app_line_msg_t *msg);

/* 北二短报文组帧 */
static uint8_t App_BuildBd2ShortMessage(const char *targetCardId,
                                        const char *text,
                                        uint8_t *frame,
                                        uint16_t *frameLength);
static uint8_t App_EncodeHexText(const char *text, char *hexBuffer, size_t hexBufferSize);
static uint8_t App_CalcXorChecksum(const char *text);
static int8_t App_HexNibble(char value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Create the mutex(es) */
  btTxMutexHandle = osMutexNew(&btTxMutex_attributes);
  rdTxMutexHandle = osMutexNew(&rdTxMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  rdModeSemaphoreHandle = osSemaphoreNew(1U, 0U, &rdModeSemaphore_attributes);
  rnModeSemaphoreHandle = osSemaphoreNew(1U, 0U, &rnModeSemaphore_attributes);
  safeSemaphoreHandle = osSemaphoreNew(1U, 0U, &safeSemaphore_attributes);
  sosSemaphoreHandle = osSemaphoreNew(1U, 0U, &sosSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  btQueueHandle = osMessageQueueNew(APP_BT_QUEUE_DEPTH, sizeof(app_line_msg_t), &btQueue_attributes);
  moduleQueueHandle = osMessageQueueNew(APP_MODULE_QUEUE_DEPTH, sizeof(app_line_msg_t), &moduleQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  btTaskHandle = osThreadNew(StartBtTask, NULL, &btTask_attributes);
  moduleTaskHandle = osThreadNew(StartModuleTask, NULL, &moduleTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  (void)argument;
  App_StartRuntime();

  for (;;)
  {
    if ((rdModeSemaphoreHandle != NULL) && (osSemaphoreAcquire(rdModeSemaphoreHandle, 0U) == osOK))
    {
      App_SwitchMode(APP_MODE_RD);
    }

    if ((rnModeSemaphoreHandle != NULL) && (osSemaphoreAcquire(rnModeSemaphoreHandle, 0U) == osOK))
    {
      App_SwitchMode(APP_MODE_RN);
    }

    if ((sosSemaphoreHandle != NULL) && (osSemaphoreAcquire(sosSemaphoreHandle, 0U) == osOK))
    {
      App_HandleSosAction();
    }

    if ((safeSemaphoreHandle != NULL) && (osSemaphoreAcquire(safeSemaphoreHandle, 0U) == osOK))
    {
      App_SendBtText("SAFE\r\n");
    }

    osDelay(APP_CONTROL_TASK_DELAY_MS);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartBtTask(void *argument)
{
  app_line_msg_t message;

  (void)argument;

  for (;;)
  {
    if ((btQueueHandle != NULL) && (osMessageQueueGet(btQueueHandle, &message, NULL, osWaitForever) == osOK))
    {
      App_HandleBtMessage(&message);
    }
  }
}

void StartModuleTask(void *argument)
{
  app_line_msg_t message;

  (void)argument;

  for (;;)
  {
    if ((moduleQueueHandle != NULL) && (osMessageQueueGet(moduleQueueHandle, &message, NULL, osWaitForever) == osOK))
    {
      if ((message.source == APP_UART_SOURCE_RD) && (g_currentMode == APP_MODE_RD))
      {
        App_HandleModuleMessage(&message);
      }
      else if ((message.source == APP_UART_SOURCE_GNSS) &&
               (g_currentMode == APP_MODE_RN) &&
               (App_IsGnrmcSentence(&message) != 0U))
      {
        App_HandleModuleMessage(&message);
      }
    }
  }
}

static void App_StartRuntime(void)
{
  if (g_runtimeStarted != 0U)
  {
    return;
  }

  g_runtimeStarted = 1U;

  HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_SET);

  App_ResetRxState(APP_UART_SOURCE_BT);
  App_ResetRxState(APP_UART_SOURCE_RD);
  App_ResetRxState(APP_UART_SOURCE_GNSS);

  if (rdModeSemaphoreHandle != NULL)
  {
    while (osSemaphoreAcquire(rdModeSemaphoreHandle, 0U) == osOK) {}
  }

  if (rnModeSemaphoreHandle != NULL)
  {
    while (osSemaphoreAcquire(rnModeSemaphoreHandle, 0U) == osOK) {}
  }

  if (safeSemaphoreHandle != NULL)
  {
    while (osSemaphoreAcquire(safeSemaphoreHandle, 0U) == osOK) {}
  }

  if (sosSemaphoreHandle != NULL)
  {
    while (osSemaphoreAcquire(sosSemaphoreHandle, 0U) == osOK) {}
  }

  (void)HAL_TIM_Base_Start_IT(&htim16);
  App_RestartRx(&hlpuart1);
  App_RestartRx(&huart1);
  App_RestartRx(&huart3);

  g_currentMode = APP_MODE_RN;
  App_SwitchMode(APP_MODE_RD);
  App_SendBtText("System ready. Default mode: RD\r\n");
}

void App_ControlInit(void)
{
  App_StartRuntime();
}

void App_UartRxCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == LPUART1)
  {
    App_ProcessRxByteFromIsr(APP_UART_SOURCE_BT, g_btRxByte);
    App_RestartRx(&hlpuart1);
  }
  else if (huart->Instance == USART1)
  {
    if (g_currentMode == APP_MODE_RD)
    {
      App_ProcessRxByteFromIsr(APP_UART_SOURCE_RD, g_rdRxByte);
    }
    else
    {
      App_ResetRxState(APP_UART_SOURCE_RD);
    }

    App_RestartRx(&huart1);
  }
  else if (huart->Instance == USART3)
  {
    if (g_currentMode == APP_MODE_RN)
    {
      App_ProcessRxByteFromIsr(APP_UART_SOURCE_GNSS, g_gnssRxByte);
    }
    else
    {
      App_ResetRxState(APP_UART_SOURCE_GNSS);
    }

    App_RestartRx(&huart3);
  }
}

void App_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == LPUART1)
  {
    App_ResetRxState(APP_UART_SOURCE_BT);
    App_RestartRx(&hlpuart1);
  }
  else if (huart->Instance == USART1)
  {
    App_ResetRxState(APP_UART_SOURCE_RD);
    App_RestartRx(&huart1);
  }
  else if (huart->Instance == USART3)
  {
    App_ResetRxState(APP_UART_SOURCE_GNSS);
    App_RestartRx(&huart3);
  }
}

void App_GpioExtiCallback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();

  if ((GPIO_Pin == SAFE_Pin) &&
      (HAL_GPIO_ReadPin(SAFE_GPIO_Port, SAFE_Pin) == GPIO_PIN_RESET) &&
      ((now - g_lastSafeTick) >= APP_DEBOUNCE_MS) &&
      (safeSemaphoreHandle != NULL))
  {
    g_lastSafeTick = now;
    (void)osSemaphoreRelease(safeSemaphoreHandle);
  }
  else if ((GPIO_Pin == SOS_Pin) &&
           (HAL_GPIO_ReadPin(SOS_GPIO_Port, SOS_Pin) == GPIO_PIN_RESET) &&
           ((now - g_lastSosTick) >= APP_DEBOUNCE_MS) &&
           (sosSemaphoreHandle != NULL))
  {
    g_lastSosTick = now;
    (void)osSemaphoreRelease(sosSemaphoreHandle);
  }
}

void App_TimerElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM16))
  {
    App_HandlePowerKeyScan();
  }
}

static void App_HandleBtMessage(const app_line_msg_t *msg)
{
  char command[APP_MAX_LINE_SIZE + 1U];

  if (msg == NULL)
  {
    return;
  }

  if (App_NormalizeCommand(msg, command, sizeof(command)) == 0U)
  {
    return;
  }

  if (App_CommandEquals(command, "rd mode") != 0U)
  {
    if (rdModeSemaphoreHandle != NULL)
    {
      (void)osSemaphoreRelease(rdModeSemaphoreHandle);
    }
    return;
  }

  if (App_CommandEquals(command, "rn mode") != 0U)
  {
    if (rnModeSemaphoreHandle != NULL)
    {
      (void)osSemaphoreRelease(rnModeSemaphoreHandle);
    }
    return;
  }

  if (App_CommandEquals(command, "cardword") != 0U)
  {
    if (g_currentMode == APP_MODE_RD)
    {
      App_SendRd(kCardQueryCommand, (uint16_t)(sizeof(kCardQueryCommand) - 1U));
    }
    else
    {
      App_SendBtText("cardword is only available in RD mode\r\n");
    }
    return;
  }

  /* 蓝牙发 sos，也直接走北二短报文发给 0362746 */
  if (App_CommandEquals(command, "sos") != 0U)
  {
    App_HandleSosAction();
    return;
  }

  if (g_currentMode == APP_MODE_RD)
  {
    App_SendRd(msg->data, msg->length);
  }
}

static void App_HandleModuleMessage(const app_line_msg_t *msg)
{
  if ((msg == NULL) || (msg->length == 0U))
  {
    return;
  }

  App_SendBt(msg->data, msg->length);
}

static void App_SwitchMode(app_mode_t mode)
{
  app_mode_t previousMode = g_currentMode;

  g_currentMode = mode;
  App_ResetRxState(APP_UART_SOURCE_RD);
  App_ResetRxState(APP_UART_SOURCE_GNSS);

  if (mode == APP_MODE_RD)
  {
    HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENRD_GPIO_Port, ENRD_Pin, GPIO_PIN_SET);

    if (previousMode == APP_MODE_RD)
    {
      App_SendBtText("Already in RD mode\r\n");
    }
    else
    {
      App_SendBtText("Switched to RD mode\r\n");
    }

    osDelay(APP_RD_MODE_SETTLE_MS);
    App_SendRd(kBd2TxrEnableCommand, (uint16_t)(sizeof(kBd2TxrEnableCommand) - 1U));
    App_SendBtText("RD TXR output enabled\r\n");
  }
  else
  {
    HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENRD_GPIO_Port, ENRD_Pin, GPIO_PIN_RESET);

    if (previousMode == APP_MODE_RN)
    {
      App_SendBtText("Already in RN mode\r\n");
    }
    else
    {
      App_SendBtText("Switched to RN mode\r\n");
    }
  }
}

static void App_HandlePowerKeyScan(void)
{
  GPIO_PinState keyState = HAL_GPIO_ReadPin(P_SWITCH_GPIO_Port, P_SWITCH_Pin);

  switch (g_powerKey.debounceState)
  {
    case 0U:
      if (keyState == GPIO_PIN_RESET)
      {
        g_powerKey.debounceState = 1U;
      }
      break;

    case 1U:
      if (keyState == GPIO_PIN_RESET)
      {
        g_powerKey.debounceState = 2U;
        g_powerKey.holdTicks = 0U;
        HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_SET);
      }
      else
      {
        g_powerKey.debounceState = 0U;
      }
      break;

    case 2U:
      if (keyState == GPIO_PIN_SET)
      {
        HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_SET);
        g_powerKey.debounceState = 0U;
        g_powerKey.holdTicks = 0U;
      }
      else
      {
        if (g_powerKey.holdTicks < APP_POWER_OFF_TICKS)
        {
          g_powerKey.holdTicks++;
          HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_SET);
        }
        else
        {
          HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_RESET);
        }
      }
      break;

    default:
      g_powerKey.debounceState = 0U;
      g_powerKey.holdTicks = 0U;
      break;
  }
}

static void App_ResetRxState(app_uart_source_t source)
{
  switch (source)
  {
    case APP_UART_SOURCE_BT:
      g_btRxLength = 0U;
      memset(g_btRxBuffer, 0, sizeof(g_btRxBuffer));
      break;

    case APP_UART_SOURCE_RD:
      g_rdRxLength = 0U;
      memset(g_rdRxBuffer, 0, sizeof(g_rdRxBuffer));
      break;

    case APP_UART_SOURCE_GNSS:
      g_gnssRxLength = 0U;
      memset(g_gnssRxBuffer, 0, sizeof(g_gnssRxBuffer));
      break;

    default:
      break;
  }
}

static void App_SendBt(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (btTxMutexHandle == NULL))
  {
    return;
  }

  if (osMutexAcquire(btTxMutexHandle, osWaitForever) == osOK)
  {
    (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, length, HAL_MAX_DELAY);
    (void)osMutexRelease(btTxMutexHandle);
  }
}

static void App_SendBtText(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  App_SendBt((const uint8_t *)text, (uint16_t)strlen(text));
}

static void App_SendRd(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (rdTxMutexHandle == NULL))
  {
    return;
  }

  if (osMutexAcquire(rdTxMutexHandle, osWaitForever) == osOK)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, length, HAL_MAX_DELAY);
    (void)osMutexRelease(rdTxMutexHandle);
  }
}

static void App_HandleSosAction(void)
{
  uint8_t frame[APP_BD2_FRAME_MAX_LEN];
  uint16_t frameLength = 0U;

  if (g_currentMode != APP_MODE_RD)
  {
    App_SendBtText("hello test requires RD mode\r\n");
    return;
  }

  if (App_BuildBd2ShortMessage(kBd2TargetCardId, kBd2SosText, frame, &frameLength) == 0U)
  {
    App_SendBtText("BD2 frame build failed\r\n");
    return;
  }

  App_SendRd(frame, frameLength);

  App_SendBtText("BD2 message sent to 0362746\r\n");
  App_SendBt(frame, frameLength);
}

static void App_RestartRx(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == LPUART1)
  {
    (void)HAL_UART_Receive_IT(&hlpuart1, &g_btRxByte, 1U);
  }
  else if (huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &g_rdRxByte, 1U);
  }
  else if (huart->Instance == USART3)
  {
    (void)HAL_UART_Receive_IT(&huart3, &g_gnssRxByte, 1U);
  }
}

static void App_ProcessRxByteFromIsr(app_uart_source_t source, uint8_t byte)
{
  switch (source)
  {
    case APP_UART_SOURCE_BT:
      if (g_btRxLength < APP_MAX_LINE_SIZE)
      {
        g_btRxBuffer[g_btRxLength++] = byte;
      }
      else
      {
        App_QueueLineFromIsr(btQueueHandle, APP_UART_SOURCE_BT, g_btRxBuffer, &g_btRxLength);
        g_btRxBuffer[g_btRxLength++] = byte;
      }

      if (byte == '\n')
      {
        App_QueueLineFromIsr(btQueueHandle, APP_UART_SOURCE_BT, g_btRxBuffer, &g_btRxLength);
      }
      break;

    case APP_UART_SOURCE_RD:
      if (g_rdRxLength < APP_MAX_LINE_SIZE)
      {
        g_rdRxBuffer[g_rdRxLength++] = byte;
      }
      else
      {
        App_QueueLineFromIsr(moduleQueueHandle, APP_UART_SOURCE_RD, g_rdRxBuffer, &g_rdRxLength);
        g_rdRxBuffer[g_rdRxLength++] = byte;
      }

      if (byte == '\n')
      {
        App_QueueLineFromIsr(moduleQueueHandle, APP_UART_SOURCE_RD, g_rdRxBuffer, &g_rdRxLength);
      }
      break;

    case APP_UART_SOURCE_GNSS:
      if (g_gnssRxLength < APP_MAX_LINE_SIZE)
      {
        g_gnssRxBuffer[g_gnssRxLength++] = byte;
      }
      else
      {
        App_QueueLineFromIsr(moduleQueueHandle, APP_UART_SOURCE_GNSS, g_gnssRxBuffer, &g_gnssRxLength);
        g_gnssRxBuffer[g_gnssRxLength++] = byte;
      }

      if (byte == '\n')
      {
        App_QueueLineFromIsr(moduleQueueHandle, APP_UART_SOURCE_GNSS, g_gnssRxBuffer, &g_gnssRxLength);
      }
      break;

    default:
      break;
  }
}

static void App_QueueLineFromIsr(osMessageQueueId_t queue, app_uart_source_t source,
                                 uint8_t *buffer, volatile uint16_t *length)
{
  app_line_msg_t message;
  uint16_t copyLength = 0U;

  if ((queue == NULL) || (buffer == NULL) || (length == NULL) || (*length == 0U))
  {
    return;
  }

  memset(&message, 0, sizeof(message));

  copyLength = *length;
  if (copyLength > APP_MAX_LINE_SIZE)
  {
    copyLength = APP_MAX_LINE_SIZE;
  }

  message.source = (uint8_t)source;
  message.length = copyLength;
  memcpy(message.data, buffer, copyLength);
  message.data[copyLength] = '\0';

  (void)osMessageQueuePut(queue, &message, 0U, 0U);

  *length = 0U;
  memset(buffer, 0, APP_MAX_LINE_SIZE + 1U);
}

static size_t App_NormalizeCommand(const app_line_msg_t *msg, char *command, size_t commandSize)
{
  size_t start = 0U;
  size_t end = 0U;
  size_t out = 0U;
  size_t i = 0U;
  char current = 0;

  if ((msg == NULL) || (command == NULL) || (commandSize == 0U))
  {
    return 0U;
  }

  end = msg->length;

  while ((start < end) &&
         ((msg->data[start] == ' ') || (msg->data[start] == '\t') ||
          (msg->data[start] == '\r') || (msg->data[start] == '\n')))
  {
    start++;
  }

  while ((end > start) &&
         ((msg->data[end - 1U] == ' ') || (msg->data[end - 1U] == '\t') ||
          (msg->data[end - 1U] == '\r') || (msg->data[end - 1U] == '\n')))
  {
    end--;
  }

  for (i = start; (i < end) && (out < (commandSize - 1U)); i++)
  {
    current = (char)msg->data[i];
    if ((current >= 'A') && (current <= 'Z'))
    {
      current = (char)(current - 'A' + 'a');
    }
    command[out++] = current;
  }

  command[out] = '\0';
  return out;
}

static uint8_t App_CommandEquals(const char *left, const char *right)
{
  if ((left == NULL) || (right == NULL))
  {
    return 0U;
  }

  return (strcmp(left, right) == 0) ? 1U : 0U;
}

static uint8_t App_TryDecodeBdTci(const app_line_msg_t *msg, uint8_t *decoded, uint16_t *decodedLength)
{
  const char *payload = NULL;
  uint16_t commaCount = 0U;
  uint16_t beginIndex = 0U;
  uint16_t endIndex = 0U;
  uint16_t i = 0U;
  uint16_t out = 0U;
  uint8_t codingOffset = 0U;
  int8_t high = 0;
  int8_t low = 0;

  if ((msg == NULL) || (decoded == NULL) || (decodedLength == NULL))
  {
    return 0U;
  }

  *decodedLength = 0U;
  payload = strstr((const char *)msg->data, "BDTCI");
  if (payload == NULL)
  {
    return 0U;
  }

  for (i = 0U; payload[i] != '\0'; i++)
  {
    if (payload[i] == ',')
    {
      commaCount++;

      if ((commaCount == 5U) && (payload[i + 1U] == '3'))
      {
        codingOffset = 1U;
      }

      if (commaCount == 7U)
      {
        beginIndex = i + 1U;
      }
    }
    else if (payload[i] == '*')
    {
      endIndex = i;
      break;
    }
  }

  if ((beginIndex == 0U) || (endIndex <= beginIndex))
  {
    return 0U;
  }

  for (i = (uint16_t)(beginIndex + codingOffset);
       (i + 1U) < endIndex && (out < APP_MAX_LINE_SIZE);
       i = (uint16_t)(i + 2U))
  {
    high = App_HexNibble(payload[i]);
    low = App_HexNibble(payload[i + 1U]);
    if ((high < 0) || (low < 0))
    {
      return 0U;
    }

    decoded[out++] = (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
  }

  *decodedLength = out;
  return (out > 0U) ? 1U : 0U;
}

static uint8_t App_TryDecodeBdTxr(const app_line_msg_t *msg, uint8_t *decoded, uint16_t *decodedLength)
{
  const char *payload = NULL;
  const char *asterisk = NULL;
  const char *lastComma = NULL;
  const char *cursor = NULL;
  uint16_t out = 0U;
  int8_t high = 0;
  int8_t low = 0;

  if ((msg == NULL) || (decoded == NULL) || (decodedLength == NULL))
  {
    return 0U;
  }

  *decodedLength = 0U;
  payload = strstr((const char *)msg->data, "BDTXR");
  if (payload == NULL)
  {
    return 0U;
  }

  asterisk = strchr(payload, '*');
  if (asterisk == NULL)
  {
    return 0U;
  }

  for (cursor = asterisk; cursor > payload; cursor--)
  {
    if (*(cursor - 1) == ',')
    {
      lastComma = cursor - 1;
      break;
    }
  }

  if ((lastComma == NULL) || ((lastComma + 1) >= asterisk) ||
      ((((size_t)(asterisk - (lastComma + 1))) % 2U) != 0U))
  {
    return 0U;
  }

  for (cursor = (lastComma + 1); ((cursor + 1) < asterisk) && (out < APP_MAX_LINE_SIZE); cursor += 2)
  {
    high = App_HexNibble(*cursor);
    low = App_HexNibble(*(cursor + 1));
    if ((high < 0) || (low < 0))
    {
      return 0U;
    }

    decoded[out++] = (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
  }

  *decodedLength = out;
  return (out > 0U) ? 1U : 0U;
}

static uint8_t App_IsGnrmcSentence(const app_line_msg_t *msg)
{
  const char *sentence = NULL;

  if ((msg == NULL) || (msg->length == 0U))
  {
    return 0U;
  }

  sentence = (const char *)msg->data;
  while ((*sentence == ' ') || (*sentence == '\r') || (*sentence == '\n'))
  {
    sentence++;
  }

  if (*sentence == '$')
  {
    sentence++;
  }

  if ((sentence[0] != '\0') &&
      (sentence[1] != '\0') &&
      ((sentence[2] == 'R') || (sentence[2] == 'r')) &&
      ((sentence[3] == 'M') || (sentence[3] == 'm')) &&
      ((sentence[4] == 'C') || (sentence[4] == 'c')))
  {
    return 1U;
  }

  if (strstr((const char *)msg->data, "RMC,") != NULL)
  {
    return 1U;
  }

  return 0U;
}

/*
 * 北二短报文协议组帧：
 * 最终格式：
 *   $CCTXA,<目标卡号>,1,1,<ASCII转HEX文本>*CS\r\n
 * 例如：
 *   $CCTXA,0362746,1,1,534F53*XX\r\n
 */
static uint8_t App_BuildBd2ShortMessage(const char *targetCardId,
                                        const char *text,
                                        uint8_t *frame,
                                        uint16_t *frameLength)
{
  char hexPayload[APP_BD2_HEX_MAX_LEN + 1U];
  char body[APP_BD2_FRAME_MAX_LEN];
  int written = 0;
  uint8_t checksum = 0U;

  if ((targetCardId == NULL) || (text == NULL) || (frame == NULL) || (frameLength == NULL))
  {
    return 0U;
  }

  if (App_EncodeHexText(text, hexPayload, sizeof(hexPayload)) == 0U)
  {
    return 0U;
  }

  written = snprintf(body, sizeof(body), "CCTXA,%s,1,1,%s", targetCardId, hexPayload);
  if ((written <= 0) || ((size_t)written >= sizeof(body)))
  {
    return 0U;
  }

  checksum = App_CalcXorChecksum(body);

  written = snprintf((char *)frame, APP_BD2_FRAME_MAX_LEN, "$%s*%02X\r\n", body, checksum);
  if ((written <= 0) || ((size_t)written >= APP_BD2_FRAME_MAX_LEN))
  {
    return 0U;
  }

  *frameLength = (uint16_t)written;
  return 1U;
}

static uint8_t App_EncodeHexText(const char *text, char *hexBuffer, size_t hexBufferSize)
{
  size_t textLength = 0U;
  size_t i = 0U;
  int written = 0;

  if ((text == NULL) || (hexBuffer == NULL) || (hexBufferSize == 0U))
  {
    return 0U;
  }

  textLength = strlen(text);
  if ((textLength == 0U) ||
      (textLength > APP_BD2_TEXT_MAX_LEN) ||
      ((textLength * 2U + 1U) > hexBufferSize))
  {
    return 0U;
  }

  for (i = 0U; i < textLength; i++)
  {
    written = snprintf(&hexBuffer[i * 2U],
                       hexBufferSize - i * 2U,
                       "%02X",
                       (unsigned char)text[i]);
    if (written != 2)
    {
      return 0U;
    }
  }

  hexBuffer[textLength * 2U] = '\0';
  return 1U;
}

static uint8_t App_CalcXorChecksum(const char *text)
{
  uint8_t checksum = 0U;

  if (text == NULL)
  {
    return 0U;
  }

  while (*text != '\0')
  {
    checksum ^= (uint8_t)(*text);
    text++;
  }

  return checksum;
}

static int8_t App_HexNibble(char value)
{
  if ((value >= '0') && (value <= '9'))
  {
    return (int8_t)(value - '0');
  }

  if ((value >= 'A') && (value <= 'F'))
  {
    return (int8_t)(value - 'A' + 10);
  }

  if ((value >= 'a') && (value <= 'f'))
  {
    return (int8_t)(value - 'a' + 10);
  }

  return -1;
}
/* USER CODE END Application */


