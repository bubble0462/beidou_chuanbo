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
#include "app_location_report.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "fence_manager.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
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
#define APP_POWER_OFF_TICKS            180U
#define APP_DEBOUNCE_MS                80U
#define APP_CONTROL_TASK_DELAY_MS      20U
#define APP_RD_MODE_SETTLE_MS          300U
#define APP_RN_RD_READY_DELAY_MS       2000U

#define APP_BD2_TEXT_MAX_LEN           APP_LOCATION_REPORT_TEXT_MAX_LEN
#define APP_BD2_HEX_MAX_LEN            (APP_BD2_TEXT_MAX_LEN * 2U)
#define APP_BD2_FRAME_MAX_LEN          220U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const uint8_t kCardQueryCommand[] = "$CCICR,0,00*68\r\n";
static const uint8_t kBd2TxrEnableCommand[] = "$CCRMO,TXR,2,1*21\r\n";

/* 北斗短报文卡号存储（可通过蓝牙设置） */
static char g_selfCardId[16] = "0365966";
char g_targetCardId[16] = "0362746";

/* 预设常用消息 */
typedef struct
{
  const char *name;
  const char *text;
} app_preset_msg_t;

static const app_preset_msg_t kPresetMessages[] =
{
  {"hello", "hello"},
  {"sos",   "SOS"},
  {"help",  "help"},
  {"ok",    "ok"},
  {"test",  "test"},
};
static const uint8_t kPresetMessageCount = sizeof(kPresetMessages) / sizeof(kPresetMessages[0]);

static volatile app_mode_t g_currentMode = APP_MODE_RD;
static uint8_t g_runtimeStarted = 0U;
static uint32_t g_lastSafeTick = 0U;
static uint32_t g_lastSosTick = 0U;
static app_location_reporter_t g_locationReporter;

/* 循环发送状态 */
#define APP_LOOP_SEND_MSG_MAX_LEN 32U
static uint8_t g_loopSendEnabled = 0U;
static uint32_t g_loopSendIntervalMs = 120000UL;
static uint32_t g_loopSendLastTick = 0U;
static char g_loopSendMessage[APP_LOOP_SEND_MSG_MAX_LEN + 1U] = "SAFE";

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for moduleTask */
osThreadId_t moduleTaskHandle;
const osThreadAttr_t moduleTask_attributes = {
  .name = "moduleTask",
  .stack_size = 512 * 4,
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
void App_SendBtText(const char *text);
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

/* 卡号设置与消息发送 */
static uint8_t App_SetCardId(char *dest, size_t destSize, const char *src);
uint8_t App_SendBd2Message(const char *targetCardId, const char *text);
static uint8_t App_SendRnLocationReport(const char *text);
static const char *App_FindPresetMessage(const char *name);
static void App_HandleSendCommand(const char *command);
static void App_HandleReportCommand(const char *command);
static void App_HandleLoopCommand(const char *command);
static void App_HandleFenceSetCommand(const char *params);
static void App_HandleFenceDelCommand(const char *params);
static uint8_t App_HandleCompactFenceCmd(const char *text);
static uint8_t App_IsCardIdValid(const char *cardId);
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

    if ((g_loopSendEnabled != 0U) &&
        (g_targetCardId[0] != '\0') &&
        ((uint32_t)(HAL_GetTick() - g_loopSendLastTick) >= g_loopSendIntervalMs))
    {
      g_loopSendLastTick = HAL_GetTick();
      if (App_SendBd2Message(g_targetCardId, g_loopSendMessage) != 0U)
      {
        char buf[64];
        (void)snprintf(buf, sizeof(buf), "Loop sent [%s] to %s\r\n",
                       g_loopSendMessage, g_targetCardId);
        App_SendBtText(buf);
      }
      else
      {
        App_SendBtText("Loop send failed\r\n");
      }
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
  char reportPacket[APP_LOCATION_REPORT_TEXT_MAX_LEN + 1U];
  char diagBuf[80];
  static uint32_t s_gnssLogTick = 0U;

  (void)argument;

  for (;;)
  {
    if ((moduleQueueHandle != NULL) && (osMessageQueueGet(moduleQueueHandle, &message, NULL, osWaitForever) == osOK))
    {
      if ((message.source == APP_UART_SOURCE_RD) && (g_currentMode == APP_MODE_RD))
      {
        App_HandleModuleMessage(&message);
      }
      else if ((message.source == APP_UART_SOURCE_RD) && (g_currentMode == APP_MODE_RN))
      {
        /* RN模式下透传RD原始数据到蓝牙，便于查看BDFKI等反馈 */
        App_HandleModuleMessage(&message);
      }
      else if ((message.source == APP_UART_SOURCE_GNSS) &&
               (g_currentMode == APP_MODE_RN) &&
               (App_IsGnrmcSentence(&message) != 0U))
      {
        uint32_t nowTick = HAL_GetTick();
        app_location_point_t point;

        /* 解析RMC用于电子围栏检测 */
        if (AppLocationReport_ParseRmc((const char *)message.data, &point) != 0U)
        {
          fence_check_location(point.lat_nmea, point.lat_dir,
                               point.lon_nmea, point.lon_dir);
        }

        App_HandleModuleMessage(&message);
        if (AppLocationReport_ProcessRmc(&g_locationReporter,
                                         (const char *)message.data,
                                         nowTick,
                                         reportPacket,
                                         sizeof(reportPacket)) != 0U)
        {
          if (App_SendRnLocationReport(reportPacket) == 0U)
          {
            App_SendBtText("RN report send failed\r\n");
          }
        }
        else if ((uint32_t)(nowTick - s_gnssLogTick) >= 30000UL)
        {
          /* 每30秒输出一次状态，方便诊断：pts=0说明无定位，elapsed<interval说明时间未到 */
          s_gnssLogTick = nowTick;
          uint32_t elapsedS = (uint32_t)(nowTick - g_locationReporter.last_report_ms) / 1000U;
          uint32_t intervalS = g_locationReporter.interval_ms / 1000U;
          (void)snprintf(diagBuf, sizeof(diagBuf),
                         "GNSS: pts=%u el=%lus iv=%lus\r\n",
                         (unsigned)g_locationReporter.point_count,
                         (unsigned long)elapsedS,
                         (unsigned long)intervalS);
          App_SendBtText(diagBuf);
        }
      }
    }
  }
}

/**
 * @brief  Read battery voltage and report via Bluetooth
 * @note   3600mV = 0%, 4100mV = 100%, linear interpolation, clamped
 */
static void App_ReportBattery(void)
{
  uint32_t vbat_mV = App_ReadVbat_mV();
  uint8_t pct = 0U;
  if (vbat_mV >= 4100U)
  {
    pct = 100U;
  }
  else if (vbat_mV > 3600U)
  {
    pct = (uint8_t)((vbat_mV - 3600U) * 100U / 500U);
  }
  else
  {
    pct = 0U;
  }
  char buf[48];
  (void)snprintf(buf, sizeof(buf), "Battery: %lumV %u%%\r\n",
                 (unsigned long)vbat_mV, (unsigned)pct);
  App_SendBtText(buf);
}

static void App_StartRuntime(void)
{
  if (g_runtimeStarted != 0U)
  {
    return;
  }

  g_runtimeStarted = 1U;

  HAL_GPIO_WritePin(POWER_KEEP_GPIO_Port, POWER_KEEP_Pin, GPIO_PIN_SET);

  /* Wait for power supply to stabilize after POWER_KEEP latch */
  osDelay(500U);

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
  AppLocationReport_Init(&g_locationReporter, HAL_GetTick());
  App_RestartRx(&hlpuart1);
  App_RestartRx(&huart1);
  App_RestartRx(&huart3);

  g_currentMode = APP_MODE_RN;
  App_SwitchMode(APP_MODE_RD);
  App_SendBtText("System ready. Default mode: RD\r\n");

  /* Report battery status after power stabilized */
  App_ReportBattery();
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
      /* RN模式下保留RD数据接收，用于查看BDFKI发送反馈和调试 */
      App_ProcessRxByteFromIsr(APP_UART_SOURCE_RD, g_rdRxByte);
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

  /* 目标卡号设置: target <卡号> */
  if ((App_CommandEquals(command, "report get") != 0U) ||
      (App_CommandEquals(command, "report off") != 0U) ||
      (strncmp(command, "report time ", 12) == 0) ||
      (strncmp(command, "report sec ", 11) == 0) ||
      (strncmp(command, "report dist ", 12) == 0))
  {
    App_HandleReportCommand(command);
    return;
  }

  if ((App_CommandEquals(command, "loop on") != 0U) ||
      (App_CommandEquals(command, "loop off") != 0U) ||
      (App_CommandEquals(command, "loop get") != 0U) ||
      (strncmp(command, "loop on ", 8) == 0) ||
      (strncmp(command, "loop time ", 10) == 0))
  {
    App_HandleLoopCommand(command);
    return;
  }

  if ((strncmp(command, "target ", 7) == 0) && (strlen(command) > 7U))
  {
    if (App_SetCardId(g_targetCardId, sizeof(g_targetCardId), command + 7) != 0U)
    {
      char buf[48];
      (void)snprintf(buf, sizeof(buf), "Target card set: %s\r\n", g_targetCardId);
      App_SendBtText(buf);
    }
    else
    {
      App_SendBtText("Invalid target card ID\r\n");
    }
    return;
  }

  /* 自身卡号设置: self <卡号> */
  if ((strncmp(command, "self ", 5) == 0) && (strlen(command) > 5U))
  {
    if (App_SetCardId(g_selfCardId, sizeof(g_selfCardId), command + 5) != 0U)
    {
      char buf[48];
      (void)snprintf(buf, sizeof(buf), "Self card set: %s\r\n", g_selfCardId);
      App_SendBtText(buf);
    }
    else
    {
      App_SendBtText("Invalid self card ID\r\n");
    }
    return;
  }

  /* 查询目标卡号 */
  if (App_CommandEquals(command, "get target") != 0U)
  {
    char buf[48];
    if (g_targetCardId[0] != '\0')
    {
      (void)snprintf(buf, sizeof(buf), "Target card: %s\r\n", g_targetCardId);
    }
    else
    {
      (void)snprintf(buf, sizeof(buf), "Target card: not set\r\n");
    }
    App_SendBtText(buf);
    return;
  }

  /* 查询自身卡号 */
  if (App_CommandEquals(command, "get self") != 0U)
  {
    char buf[48];
    if (g_selfCardId[0] != '\0')
    {
      (void)snprintf(buf, sizeof(buf), "Self card: %s\r\n", g_selfCardId);
    }
    else
    {
      (void)snprintf(buf, sizeof(buf), "Self card: not set\r\n");
    }
    App_SendBtText(buf);
    return;
  }

  /* 发送自定义内容: send <内容> */
  if ((strncmp(command, "send ", 5) == 0) && (strlen(command) > 5U))
  {
    App_HandleSendCommand(command + 5);
    return;
  }

  /* SOS / 一键报平安 -> 发送 SOS 给目标卡号 */
  if (App_CommandEquals(command, "sos") != 0U)
  {
    App_HandleSosAction();
    return;
  }

  if (App_CommandEquals(command, "battery") != 0U)
  {
    App_ReportBattery();
    return;
  }

  /* 围栏命令 */
  if (strncmp(command, "set_fence:", 10) == 0)
  {
    App_HandleFenceSetCommand(command + 10);
    return;
  }

  if (strncmp(command, "del_fence:", 10) == 0)
  {
    App_HandleFenceDelCommand(command + 10);
    return;
  }

  if (App_CommandEquals(command, "list_fence") != 0U)
  {
    fence_list_all();
    return;
  }

  /* 预设消息快捷命令 */
  {
    const char *preset = App_FindPresetMessage(command);
    if (preset != NULL)
    {
      App_HandleSendCommand(preset);
      return;
    }
  }

  if (g_currentMode == APP_MODE_RD)
  {
    App_SendRd(msg->data, msg->length);
  }
}

static void App_HandleModuleMessage(const app_line_msg_t *msg)
{
  uint8_t decoded[256];
  uint16_t decodedLen = 0U;

  if ((msg == NULL) || (msg->length == 0U))
  {
    return;
  }

  /* 尝试解码北斗短报文，如果是岸基围栏配置则额外转发给App */
  if ((App_TryDecodeBdTci(msg, decoded, &decodedLen) != 0U) ||
      (App_TryDecodeBdTxr(msg, decoded, &decodedLen) != 0U))
  {
    decoded[decodedLen] = '\0';

    if (strncmp((const char *)decoded, "SET_FENCE:", 10) == 0)
    {
      /* 旧格式兼容 */
      char buf[256];
      (void)snprintf(buf, sizeof(buf), "FENCE_CFG:%s\r\n", (const char *)decoded + 10);
      App_SendBtText(buf);
    }
    else if (App_HandleCompactFenceCmd((const char *)decoded) != 0U)
    {
      /* 新紧凑格式 FC/FP/FQ 已在函数内部处理 */
    }
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
    HAL_GPIO_WritePin(EN_5V_GPIO_Port, EN_5V_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENRD_GPIO_Port, ENRD_Pin, GPIO_PIN_SET);

    if (previousMode == APP_MODE_RN)
    {
      App_SendBtText("Already in RN mode\r\n");
    }
    else
    {
      App_SendBtText("Switched to RN mode\r\n");
      osDelay(APP_RN_RD_READY_DELAY_MS);
      App_SendBtText("RN RD standby ready\r\n");
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

void App_SendBtText(const char *text)
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
  if (g_currentMode != APP_MODE_RD)
  {
    App_SendBtText("SOS requires RD mode\r\n");
    return;
  }

  if (g_targetCardId[0] == '\0')
  {
    App_SendBtText("Target card not set\r\n");
    return;
  }

  if (App_SendBd2Message(g_targetCardId, "SOS") == 0U)
  {
    App_SendBtText("BD2 SOS send failed\r\n");
    return;
  }

  {
    char buf[64];
    (void)snprintf(buf, sizeof(buf), "BD2 SOS sent to %s\r\n", g_targetCardId);
    App_SendBtText(buf);
  }
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

/* -------------------------------------------------------------------------- */
/* 卡号设置与消息发送辅助函数                                                  */
/* -------------------------------------------------------------------------- */

static uint8_t App_IsCardIdValid(const char *cardId)
{
  size_t len = 0U;
  size_t i = 0U;

  if ((cardId == NULL) || (cardId[0] == '\0'))
  {
    return 0U;
  }

  len = strlen(cardId);
  if ((len < 4U) || (len > 12U))
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    if ((cardId[i] < '0') || (cardId[i] > '9'))
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t App_SetCardId(char *dest, size_t destSize, const char *src)
{
  size_t len = 0U;

  if ((dest == NULL) || (destSize == 0U) || (src == NULL))
  {
    return 0U;
  }

  len = strlen(src);
  if ((len == 0U) || (len >= destSize))
  {
    return 0U;
  }

  if (App_IsCardIdValid(src) == 0U)
  {
    return 0U;
  }

  (void)strncpy(dest, src, destSize - 1U);
  dest[destSize - 1U] = '\0';
  return 1U;
}

static const char *App_FindPresetMessage(const char *name)
{
  if (name == NULL)
  {
    return NULL;
  }

  for (uint8_t i = 0U; i < kPresetMessageCount; i++)
  {
    if (strcmp(name, kPresetMessages[i].name) == 0)
    {
      return kPresetMessages[i].text;
    }
  }

  return NULL;
}

uint8_t App_SendBd2Message(const char *targetCardId, const char *text)
{
  uint8_t frame[APP_BD2_FRAME_MAX_LEN];
  uint16_t frameLength = 0U;
  uint8_t result = 0U;

  if ((targetCardId == NULL) || (text == NULL))
  {
    return 0U;
  }

  result = App_BuildBd2ShortMessage(targetCardId, text, frame, &frameLength);
  if (result != 0U)
  {
    App_SendRd(frame, frameLength);
    App_SendBt(frame, frameLength);
    App_SendBtText("\r\n");
  }

  return result;
}

static uint8_t App_SendRnLocationReport(const char *text)
{
  uint8_t frame[APP_BD2_FRAME_MAX_LEN];
  uint16_t frameLength = 0U;

  if ((text == NULL) || (text[0] == '\0'))
  {
    return 0U;
  }

  if (g_currentMode != APP_MODE_RN)
  {
    return 0U;
  }

  if (g_targetCardId[0] == '\0')
  {
    App_SendBtText("Location report target not set\r\n");
    return 0U;
  }

  if (App_BuildBd2ShortMessage(g_targetCardId, text, frame, &frameLength) == 0U)
  {
    App_SendBtText("Location report build failed\r\n");
    return 0U;
  }

  App_SendBtText("RN report frame: ");
  App_SendBt(frame, frameLength);
  App_SendBtText("\r\n");
  App_SendRd(frame, frameLength);

  App_ResetRxState(APP_UART_SOURCE_RD);
  App_RestartRx(&huart1);
  App_RestartRx(&huart3);

  App_SendBtText("RN report sent: ");
  App_SendBtText(text);
  App_SendBtText("\r\n");
  return 1U;
}

static void App_HandleSendCommand(const char *text)
{
  const char *message = NULL;

  if ((text == NULL) || (text[0] == '\0'))
  {
    App_SendBtText("Empty message\r\n");
    return;
  }

  if (g_currentMode != APP_MODE_RD)
  {
    App_SendBtText("Send requires RD mode\r\n");
    return;
  }

  if (g_targetCardId[0] == '\0')
  {
    App_SendBtText("Target card not set\r\n");
    return;
  }

  /* 先尝试匹配预设消息 */
  message = App_FindPresetMessage(text);
  if (message == NULL)
  {
    message = text;
  }

  if (App_SendBd2Message(g_targetCardId, message) == 0U)
  {
    App_SendBtText("BD2 message build failed\r\n");
    return;
  }

  {
    char buf[80];
    (void)snprintf(buf, sizeof(buf), "BD2 sent [%s] to %s\r\n", message, g_targetCardId);
    App_SendBtText(buf);
  }
}

static void App_HandleReportCommand(const char *command)
{
  char buf[96];
  char *end = NULL;
  unsigned long value = 0UL;
  uint32_t now = HAL_GetTick();

  if (command == NULL)
  {
    return;
  }

  if (App_CommandEquals(command, "report get") != 0U)
  {
    uint32_t elapsed_s = (uint32_t)(now - g_locationReporter.last_report_ms) / 1000U;
    uint32_t interval_s = g_locationReporter.interval_ms / 1000U;
    AppLocationReport_FormatStatus(&g_locationReporter, buf, sizeof(buf));
    App_SendBtText(buf);
    (void)snprintf(buf, sizeof(buf), "Points: %u, elapsed: %lus, interval: %lus\r\n",
                   (unsigned)g_locationReporter.point_count,
                   (unsigned long)elapsed_s,
                   (unsigned long)interval_s);
    App_SendBtText(buf);
    return;
  }

  if (App_CommandEquals(command, "report off") != 0U)
  {
    (void)AppLocationReport_SetDefault(&g_locationReporter, now);
    App_SendBtText("Report default: 2 min\r\n");
    return;
  }

  if (strncmp(command, "report time ", 12) == 0)
  {
    value = strtoul(command + 12, &end, 10);
    if ((end == (command + 12)) || (*end != '\0') ||
        (value > 65535UL) ||
        (AppLocationReport_SetTimeMinutes(&g_locationReporter, (uint16_t)value, now) == 0U))
    {
      App_SendBtText("Invalid report time, must be >= 1 min\r\n");
      return;
    }

    (void)snprintf(buf, sizeof(buf), "Report time set: %lu min\r\n", value);
    App_SendBtText(buf);
    return;
  }

  if (strncmp(command, "report sec ", 11) == 0)
  {
    value = strtoul(command + 11, &end, 10);
    if ((end == (command + 11)) || (*end != '\0') ||
        (value > 4294967UL) ||
        (AppLocationReport_SetTimeSeconds(&g_locationReporter, (uint32_t)value, now) == 0U))
    {
      App_SendBtText("Invalid report sec, must be >= 60 s\r\n");
      return;
    }

    (void)snprintf(buf, sizeof(buf), "Report time set: %lu s\r\n", value);
    App_SendBtText(buf);
    return;
  }

  if (strncmp(command, "report dist ", 12) == 0)
  {
    value = strtoul(command + 12, &end, 10);
    if ((end == (command + 12)) || (*end != '\0') ||
        (value > 4294967295UL) ||
        (AppLocationReport_SetDistanceMeters(&g_locationReporter, (uint32_t)value, now) == 0U))
    {
      App_SendBtText("Invalid report distance, must be >= 500 m\r\n");
      return;
    }

    (void)snprintf(buf, sizeof(buf), "Report distance set: %lu m\r\n", value);
    App_SendBtText(buf);
    return;
  }

  App_SendBtText("Unknown report command\r\n");
}

static void App_HandleLoopCommand(const char *command)
{
  char buf[80];
  char *end = NULL;
  unsigned long value = 0UL;

  if (command == NULL)
  {
    return;
  }

  if (App_CommandEquals(command, "loop off") != 0U)
  {
    g_loopSendEnabled = 0U;
    App_SendBtText("Loop send disabled\r\n");
    return;
  }

  if (App_CommandEquals(command, "loop get") != 0U)
  {
    if (g_loopSendEnabled != 0U)
    {
      (void)snprintf(buf, sizeof(buf), "Loop: on, msg: %s, interval: %lu min\r\n",
                     g_loopSendMessage, (unsigned long)(g_loopSendIntervalMs / 60000UL));
    }
    else
    {
      (void)snprintf(buf, sizeof(buf), "Loop: off\r\n");
    }
    App_SendBtText(buf);
    return;
  }

  if (strncmp(command, "loop time ", 10) == 0)
  {
    value = strtoul(command + 10, &end, 10);
    if ((end == (command + 10)) || (*end != '\0') ||
        (value < 1UL) || (value > 65535UL))
    {
      App_SendBtText("Invalid loop time, must be >= 1 min\r\n");
      return;
    }

    g_loopSendIntervalMs = (uint32_t)value * 60000UL;
    (void)snprintf(buf, sizeof(buf), "Loop interval: %lu min\r\n", value);
    App_SendBtText(buf);
    return;
  }

  if ((strncmp(command, "loop on ", 8) == 0) && (strlen(command) > 8U))
  {
    const char *msg = command + 8;
    size_t msgLen = strlen(msg);

    if ((msgLen == 0U) || (msgLen > APP_LOOP_SEND_MSG_MAX_LEN))
    {
      App_SendBtText("Loop message too long\r\n");
      return;
    }

    (void)strncpy(g_loopSendMessage, msg, sizeof(g_loopSendMessage) - 1U);
    g_loopSendMessage[sizeof(g_loopSendMessage) - 1U] = '\0';
    g_loopSendEnabled = 1U;
    g_loopSendLastTick = HAL_GetTick();
    (void)snprintf(buf, sizeof(buf), "Loop send enabled: %s\r\n", g_loopSendMessage);
    App_SendBtText(buf);
    return;
  }

  if (App_CommandEquals(command, "loop on") != 0U)
  {
    g_loopSendEnabled = 1U;
    g_loopSendLastTick = HAL_GetTick();
    (void)snprintf(buf, sizeof(buf), "Loop send enabled: %s\r\n", g_loopSendMessage);
    App_SendBtText(buf);
    return;
  }

  App_SendBtText("Unknown loop command\r\n");
}

/* ==========================================================
 * 围栏命令处理
 * ========================================================== */

static void App_HandleFenceSetCommand(const char *params)
{
  char buf[64];

  if ((params == NULL) || (params[0] == '\0'))
  {
    App_SendBtText("Invalid fence params\r\n");
    return;
  }

  /* 圆形: set_fence:c,id,lat,lng,radius */
  if ((params[0] == 'c') && (params[1] == ','))
  {
    uint32_t id = 0U;
    double lat = 0.0;
    double lng = 0.0;
    double radius = 0.0;

    if (sscanf(params + 2, "%lu,%lf,%lf,%lf", &id, &lat, &lng, &radius) == 4)
    {
      fence_add_circle(id, lat, lng, radius);
      (void)snprintf(buf, sizeof(buf), "Fence set: C ID=%lu\r\n", id);
      App_SendBtText(buf);
    }
    else
    {
      App_SendBtText("Invalid circle fence params\r\n");
    }
    return;
  }

  /* 多边形: set_fence:p,id,vertex_count,lat1,lng1,lat2,lng2,... */
  if ((params[0] == 'p') && (params[1] == ','))
  {
    uint32_t id = 0U;
    int vertex_count = 0;
    double vertices[FENCE_MAX_VERTICES][2];
    const char *p = params + 2;
    int parsed = 0;
    int i;

    if (sscanf(p, "%lu,%d", &id, &vertex_count) != 2)
    {
      App_SendBtText("Invalid polygon fence params\r\n");
      return;
    }

    if ((vertex_count < 3) || (vertex_count > FENCE_MAX_VERTICES))
    {
      App_SendBtText("Invalid vertex count\r\n");
      return;
    }

    /* 跳过 id,vertex_count 部分，定位到第一个坐标 */
    {
      int comma_count = 0;
      while ((*p != '\0') && (comma_count < 2))
      {
        if (*p == ',')
        {
          comma_count++;
        }
        p++;
      }
    }

    for (i = 0; (i < vertex_count) && (*p != '\0'); i++)
    {
      double lat = 0.0;
      double lng = 0.0;
      int n = 0;

      n = sscanf(p, "%lf,%lf", &lat, &lng);
      if (n != 2)
      {
        break;
      }
      vertices[i][0] = lng;  /* 经度 */
      vertices[i][1] = lat;  /* 纬度 */
      parsed++;

      /* 跳过这对坐标 */
      {
        int comma_count = 0;
        while ((*p != '\0') && (comma_count < 2))
        {
          if (*p == ',')
          {
            comma_count++;
          }
          p++;
        }
      }
    }

    if (parsed == vertex_count)
    {
      fence_add_polygon(id, (const double (*)[2])vertices, vertex_count);
      (void)snprintf(buf, sizeof(buf), "Fence set: P ID=%lu N=%d\r\n", id, vertex_count);
      App_SendBtText(buf);
    }
    else
    {
      App_SendBtText("Invalid polygon vertex data\r\n");
    }
    return;
  }

  App_SendBtText("Unknown fence type, use c or p\r\n");
}

static void App_HandleFenceDelCommand(const char *params)
{
  char buf[48];
  uint32_t id = 0U;

  if ((params == NULL) || (params[0] == '\0'))
  {
    App_SendBtText("Invalid fence ID\r\n");
    return;
  }

  id = strtoul(params, NULL, 10);
  fence_remove(id);
  (void)snprintf(buf, sizeof(buf), "Fence removed: ID=%" PRIu32 "\r\n", id);
  App_SendBtText(buf);
}

/* ==========================================================
 * 辅助函数：将6位纬度raw补零为9位(NMEA 5dp)，7位经度raw补零为10位
 * 然后调用 fence_nmea_to_decimal_deg 转换
 * ========================================================== */
static double App_PaddedNmeaToDec(uint32_t raw, int digits, char dir)
{
  /* 补零到标准 NMEA 位数（纬度9位，经度10位） */
  int target = (dir != '\0') ? 9 : 10; /* 不使用 dir 区分，用 digits */
  (void)target;
  (void)digits;

  /* 纬度补3个0(6→9位)，经度补3个0(7→10位) */
  raw = raw * 1000U;

  return fence_nmea_to_decimal_deg(raw, dir);
}

/* ==========================================================
 * 从逗号分隔字符串中解析下一个整数字段，返回指针偏移
 * ========================================================== */
static const char *App_ParseNextInt(const char *p, int32_t *out)
{
  char *end = NULL;
  long val = strtol(p, &end, 10);
  if (end == p)
  {
    return NULL;
  }
  *out = (int32_t)val;
  /* 跳过逗号 */
  if (*end == ',')
  {
    end++;
  }
  return (const char *)end;
}

static const char *App_ParseNextUint(const char *p, uint32_t *out)
{
  char *end = NULL;
  unsigned long val = strtoul(p, &end, 10);
  if (end == p)
  {
    return NULL;
  }
  *out = (uint32_t)val;
  if (*end == ',')
  {
    end++;
  }
  return (const char *)end;
}

/* ==========================================================
 * 紧凑围栏命令解析（FC/FP/FQ）
 * 由 App_HandleModuleMessage 中解码短报文后调用
 * 返回 1 表示已处理，0 表示不匹配
 * ========================================================== */
static uint8_t App_HandleCompactFenceCmd(const char *text)
{
  char bt_buf[256];
  const char *p;

  if ((text == NULL) || (text[0] != 'F'))
  {
    return 0U;
  }

  /* ---- FC: 圆形围栏 ---- FC<id>,<lat6>,<lng7>,<radius> */
  if ((text[1] == 'C') && (text[2] >= '0') && (text[2] <= '9'))
  {
    uint32_t id = 0U;
    uint32_t lat_raw = 0U;
    uint32_t lng_raw = 0U;
    uint32_t radius = 0U;

    p = &text[2];
    p = App_ParseNextUint(p, &id);
    if (p == NULL) { return 0U; }
    p = App_ParseNextUint(p, &lat_raw);
    if (p == NULL) { return 0U; }
    p = App_ParseNextUint(p, &lng_raw);
    if (p == NULL) { return 0U; }
    p = App_ParseNextUint(p, &radius);
    if (p == NULL) { return 0U; }

    double lat = App_PaddedNmeaToDec(lat_raw, 6, 'N');
    double lng = App_PaddedNmeaToDec(lng_raw, 7, 'E');

    fence_add_circle(id, lat, lng, (double)radius);

    /* 蓝牙转发标准 FENCE_CFG 格式给App */
    (void)snprintf(bt_buf, sizeof(bt_buf),
                   "FENCE_CFG:C,%" PRIu32 ",%.5f,%.5f,%.1f\r\n",
                   id, lat, lng, (double)radius);
    App_SendBtText(bt_buf);
    return 1U;
  }

  /* ---- FP: 多边形绝对坐标 ---- FP<id>,<lat6>,<lng7>,<lat6>,<lng7>,... */
  if ((text[1] == 'P') && (text[2] >= '0') && (text[2] <= '9'))
  {
    uint32_t id = 0U;
    double vertices[FENCE_MAX_VERTICES][2];
    int count = 0;

    p = &text[2];
    p = App_ParseNextUint(p, &id);
    if (p == NULL) { return 0U; }

    /* 交替解析纬度6位+经度7位 */
    while ((*p != '\0') && (*p != '\r') && (*p != '\n') && (count < FENCE_MAX_VERTICES))
    {
      uint32_t lat_raw = 0U;
      uint32_t lng_raw = 0U;
      const char *next;

      next = App_ParseNextUint(p, &lat_raw);
      if (next == NULL) { break; }
      p = next;

      next = App_ParseNextUint(p, &lng_raw);
      if (next == NULL) { break; }
      p = next;

      vertices[count][0] = App_PaddedNmeaToDec(lng_raw, 7, 'E'); /* 经度 */
      vertices[count][1] = App_PaddedNmeaToDec(lat_raw, 6, 'N'); /* 纬度 */
      count++;
    }

    if (count >= 3)
    {
      int i;
      fence_add_polygon(id, (const double (*)[2])vertices, count);

      /* 蓝牙转发标准 FENCE_CFG:P 格式给App */
      int offset = snprintf(bt_buf, sizeof(bt_buf),
                            "FENCE_CFG:P,%" PRIu32 ",%d", id, count);
      for (i = 0; (i < count) && (offset < 240); i++)
      {
        offset += snprintf(bt_buf + offset, sizeof(bt_buf) - (size_t)offset,
                           ",%.5f,%.5f", vertices[i][1], vertices[i][0]);
      }
      bt_buf[offset++] = '\r';
      bt_buf[offset++] = '\n';
      bt_buf[offset] = '\0';
      App_SendBtText(bt_buf);
      return 1U;
    }
    return 0U;
  }

  /* ---- FQ: 多边形偏移编码 ---- FQ<id>,<lat6>,<lng7>,<dlat>,<dlng>,... */
  if ((text[1] == 'Q') && (text[2] >= '0') && (text[2] <= '9'))
  {
    uint32_t id = 0U;
    uint32_t lat0_raw = 0U;
    uint32_t lng0_raw = 0U;
    double vertices[FENCE_MAX_VERTICES][2];
    int count = 0;

    p = &text[2];
    p = App_ParseNextUint(p, &id);
    if (p == NULL) { return 0U; }
    p = App_ParseNextUint(p, &lat0_raw);
    if (p == NULL) { return 0U; }
    p = App_ParseNextUint(p, &lng0_raw);
    if (p == NULL) { return 0U; }

    /* 基准点（第1个顶点） */
    double base_lat = App_PaddedNmeaToDec(lat0_raw, 6, 'N');
    double base_lng = App_PaddedNmeaToDec(lng0_raw, 7, 'E');
    vertices[0][0] = base_lng; /* 经度 */
    vertices[0][1] = base_lat; /* 纬度 */
    count = 1;

    /* 后续顶点为偏移量（单位 0.001° ≈ 111m） */
    while ((*p != '\0') && (*p != '\r') && (*p != '\n') && (count < FENCE_MAX_VERTICES))
    {
      int32_t dlat = 0;
      int32_t dlng = 0;
      const char *next;

      next = App_ParseNextInt(p, &dlat);
      if (next == NULL) { break; }
      p = next;

      next = App_ParseNextInt(p, &dlng);
      if (next == NULL) { break; }
      p = next;

      vertices[count][1] = base_lat + ((double)dlat * 0.001); /* 纬度 */
      vertices[count][0] = base_lng + ((double)dlng * 0.001); /* 经度 */
      count++;
    }

    if (count >= 3)
    {
      int i;
      fence_add_polygon(id, (const double (*)[2])vertices, count);

      /* 蓝牙转发展开后的标准 FENCE_CFG:P 格式 */
      int offset = snprintf(bt_buf, sizeof(bt_buf),
                            "FENCE_CFG:P,%" PRIu32 ",%d", id, count);
      for (i = 0; (i < count) && (offset < 240); i++)
      {
        offset += snprintf(bt_buf + offset, sizeof(bt_buf) - (size_t)offset,
                           ",%.5f,%.5f", vertices[i][1], vertices[i][0]);
      }
      bt_buf[offset++] = '\r';
      bt_buf[offset++] = '\n';
      bt_buf[offset] = '\0';
      App_SendBtText(bt_buf);
      return 1U;
    }
    return 0U;
  }

  return 0U;
}
/* USER CODE END Application */


