/**
 ******************************************************************************
 * @file    bvopus_per_app.c
 * @author  SRA-A&SP
 * @version V 1.1.0
 * @date    19-Dic-2019
 * @brief   Bluevoice Opus peripheral application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "ble.h"
#include "svc/Inc/bvopus_service_stm.h"  
#include "stm32_seq.h"
#include "app_ble.h"
#include "cube_hal.h"
#include "bvopus_service_stm.h"

/** @addtogroup BV_OPUS_APP
  * @{
  */

/** @addtogroup BV_OPUS_PERIPHERAL
  * @{
  */


/** @defgroup BV_OPUS_PER_Private_Typedef  BV_OPUS_PER_Private_Typedef
  * @{
  */
typedef struct
{
  BVOPUS_PER_APP_Status_t status;               /*!< Specifies the status of the BlueVoice app. */

  uint16_t connHandle;                          /*!< Connection handle. */

  BV_OPUS_ProfileHandle_t BV_handle_CEN;        /*!< handle of the BlueVoice service exported by the Central node. */    

}BVOPUS_Per_Context_t;

typedef enum
{
  GATT_PROC_MTU_UPDATE,
  GATT_PROC_DISC_PRIMARY_SERVICES,
  GATT_PROC_DISC_CHAR,
} GattProcId_t;

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_Private_Defines BV_OPUS_PER_Private_Defines
  * @{
  */
/* Private defines -----------------------------------------------------------*/

#define AUDIO_CHANNELS_OUT                              (uint16_t) (0x01)                               /*!< Output channels number.*/
#define AUDIO_OUT_SAMPLING_FREQUENCY                    (uint16_t) (SAMPLING_FREQ_16000)		/*!< Audio USB output sampling frequency.*/
#define AUDIO_OUT_MS                                    (20)  						/*!< Number of ms of Audio buffered in Rx before USB.*/

#define LED_ON_TIMEOUT                                  (0.010*1000*1000/CFG_TS_TICK_VAL) /**< 10ms */

/* Opus configuration that must be sent to the mobile application */
#define BV_OPUS_CONF_CMD                                (uint8_t)0x0B

#define BV_OPUS_FRAME_SIZE_2_5                          (uint8_t)0x20
#define BV_OPUS_FRAME_SIZE_5                            (uint8_t)0x21
#define BV_OPUS_FRAME_SIZE_10                           (uint8_t)0x22
#define BV_OPUS_FRAME_SIZE_20                           (uint8_t)0x23
#define BV_OPUS_FRAME_SIZE_40                           (uint8_t)0x24
#define BV_OPUS_FRAME_SIZE_60                           (uint8_t)0x25

#define BV_OPUS_SAMPLING_FREQ_8                         (uint8_t)0x30
#define BV_OPUS_SAMPLING_FREQ_16                        (uint8_t)0x31
#define BV_OPUS_SAMPLING_FREQ_24                        (uint8_t)0x32
#define BV_OPUS_SAMPLING_FREQ_48                        (uint8_t)0x33

#define BV_OPUS_CHANNELS_1                              (uint8_t)0x40
#define BV_OPUS_CHANNELS_2                              (uint8_t)0x41

#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_Private_Variables BV_OPUS_PER_Private_Variables
  * @{
  */

/* Global variables ----------------------------------------------------------*/

uint16_t PDM_Buffer[((((AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY)/1000) * MAX_DECIMATION_FACTOR) / 16)  * N_MS_PER_INTERRUPT];        
uint16_t PCM_Buffer[((AUDIO_IN_CHANNELS * AUDIO_IN_SAMPLING_FREQUENCY)/1000) * N_MS_PER_INTERRUPT];                                          
CCA02M2_AUDIO_Init_t MicParams;

/*!< PCM data to be streamed via USB are stored here.*/
static uint16_t PCM_Buffer_USB[(AUDIO_OUT_SAMPLING_FREQUENCY/1000) * AUDIO_OUT_MS];     

USBD_HandleTypeDef hUSBDDevice;
extern USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops;

uint8_t wait_serv_char_discovery = 0;
uint8_t service_discovered = 0;
uint8_t char_audio_discovered = 0;
uint8_t char_ctrl_discovered = 0;
volatile uint16_t RX_MTU = 0;

uint8_t Led_timer_ID;
static uint32_t led_counter = 0;
static uint32_t alarm_led_counter = 0;

static BVOPUS_Per_Context_t BVOPUS_Per_Context;
BV_OPUS_Status status;
OPUS_IF_ENC_ConfigTypeDef EncConfigOpus;                                        /*!< BlueVoice opus encode configuration.*/
OPUS_IF_DEC_ConfigTypeDef DecConfigOpus;                                        /*!< BlueVoice opus decode configuration.*/

/**
  * @}
  */
  
/* Private function prototypes -----------------------------------------------*/

static SVCCTL_EvtAckStatus_t Event_Handler(void *Event);
static void GattProcReq(GattProcId_t GattProcId);
void AudioProcess(void);
void SendData(void);
void BV_APP_PER_GATTNotification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value);
BV_OPUS_Status BV_APP_PER_Send_Opus_Configuration(void);
static void Led_manager(void);
BV_OPUS_Status BVOPUS_PER_CodecConfiguration(void);
BV_OPUS_Status BluevoiceOPUS_EnableAudio_Notif(void);
BV_OPUS_Status BluevoiceOPUS_DisableAudio_Notif(void);
BV_OPUS_Status BluevoiceOPUS_EnableCtrl_Notif(void);
BV_OPUS_Status BluevoiceOPUS_DisableCtrl_Notif(void);
void USB_Initialization(void);
void Error_Handler(void);

/* Functions Definition ----------------------------------------------------------*/

/**
 * @brief  Bluevoice Opus service initialization.
 * @param  None
 * @retval None
 */
void BVOPUS_PER_Init(void)
{    
  /* Register the event handler to the BLE controller */
  SVCCTL_RegisterCltHandler(Event_Handler);
  
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SEND_DATA_ID, UTIL_SEQ_RFU, SendData);
  
  USB_Initialization();
    
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_IN_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AUDIO_IN_SAMPLING_FREQUENCY;
  MicParams.Volume = AUDIO_VOLUME_INPUT;
  
  CCA02M2_AUDIO_IN_Init(CCA02M2_AUDIO_INSTANCE, &MicParams);
  CCA02M2_AUDIO_IN_Record(CCA02M2_AUDIO_INSTANCE, (uint8_t *) PDM_Buffer, AUDIO_IN_BUFFER_SIZE);
  CCA02M2_AUDIO_IN_Pause(CCA02M2_AUDIO_INSTANCE);
   
  BVOPUS_Per_Context.status = BV_STATUS_INITIALIZED; 
  led_counter = 0;
  
  HW_TS_Create(CFG_TimLedID_isr, &Led_timer_ID, hw_ts_Repeated, Led_manager);
  HW_TS_Start(Led_timer_ID, (uint32_t)LED_ON_TIMEOUT);
 
  BV_OPUS_Status status = BVOPUS_PER_CodecConfiguration();
  if(status != BV_OPUS_SUCCESS)
  {
    Error_Handler();
  }
}

/**
 * @brief  BLE Link ready notification.
 * @param  None
 * @retval None
 */
void BVOPUS_App_LinkReadyNotification(void)
{ 
  GattProcReq(GATT_PROC_MTU_UPDATE);
  
  GattProcReq(GATT_PROC_DISC_PRIMARY_SERVICES);
  
  if(service_discovered)
  {
    while(!(char_audio_discovered && char_ctrl_discovered))
    {
      GattProcReq(GATT_PROC_DISC_CHAR);
    }
  }
   
  BluevoiceOPUS_EnableCtrl_Notif();
  
  BVOPUS_Per_Context.status = BV_STATUS_READY;
}

 
/**
 * @brief  It performs the requested Gatt procedure
 * @param  Gatt procedure ID
 * @retval None
 */
static void GattProcReq(GattProcId_t GattProcId)
{
  tBleStatus status;
  uint8_t ret;
  
  switch(GattProcId)
  {
    case GATT_PROC_MTU_UPDATE:
    {         
      ret = aci_gatt_exchange_config(BVOPUS_Per_Context.connHandle);
      if(ret != BLE_STATUS_SUCCESS)
      {
        Error_Handler();
      }         
      UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
    }
    break;

    case GATT_PROC_DISC_PRIMARY_SERVICES:
    {
      BVOPUS_Per_Context.status = BV_STATUS_SERVICE_DISC;
      status = aci_gatt_disc_all_primary_services(BVOPUS_Per_Context.connHandle);
      if (status != BLE_STATUS_SUCCESS)
      {
        Error_Handler();
      }
      UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
    }
    break;

    case GATT_PROC_DISC_CHAR:
    {
      BVOPUS_Per_Context.status = BV_STATUS_CHAR_DISC;
      status = aci_gatt_disc_all_char_of_service(BVOPUS_Per_Context.connHandle, BVOPUS_Per_Context.BV_handle_CEN.ServiceHandle, BVOPUS_Per_Context.BV_handle_CEN.ServiceEndHandle);
      if (status != BLE_STATUS_SUCCESS)
      {
        Error_Handler();
      }
      UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
    } 
    break;

    default:
      break;
  }
  return;
}

/**
* @brief  Called when a Start or Stop streaming event occurs.
* @param  Evt_code: start or stop streaming event.
* @retval None.
*/
void BVOPUS_STM_APP_Notification(BVOPUS_STM_evt_code_t Evt_code)
{
  switch(Evt_code)
  {
    case BVOPUS_STM_START_STREAMING_EVT:
      {
        BV_APP_PER_Send_Opus_Configuration();
        
        if (BVOPUS_Per_Context.status == BV_STATUS_READY)
        {  
          BVOPUS_Per_Context.status = BV_STATUS_STREAMING;
          led_counter = 0;
          /* Audio acquisition is resumed */
          CCA02M2_AUDIO_IN_Resume(CCA02M2_AUDIO_INSTANCE);
        }
        else if (BVOPUS_Per_Context.status == BV_STATUS_RECEIVING)
        {
          BVOPUS_Per_Context.status = BV_STATUS_FULLDUPLEX;
          led_counter = 0;
          /* Audio acquisition is resumed */
          CCA02M2_AUDIO_IN_Resume(CCA02M2_AUDIO_INSTANCE);
        }
      }
      break;
    case BVOPUS_STM_STOP_STREAMING_EVT:
      {
        if (BVOPUS_Per_Context.status == BV_STATUS_STREAMING)
        {
          BVOPUS_Per_Context.status = BV_STATUS_READY;
          led_counter = 0;
          /* Audio acquisition is paused */
          CCA02M2_AUDIO_IN_Pause(CCA02M2_AUDIO_INSTANCE);
        }
        else if (BVOPUS_Per_Context.status == BV_STATUS_FULLDUPLEX)
        {
          BVOPUS_Per_Context.status = BV_STATUS_RECEIVING;
          led_counter = 0;
          /* Audio acquisition is paused */
          CCA02M2_AUDIO_IN_Pause(CCA02M2_AUDIO_INSTANCE);
        }
      }
      break;
  }
}

/**
* @brief  This function manages the sw1 button event.
* @param  None.
* @retval None
*/
void Button_Trigger_Received(void)
{
  if((BVOPUS_Per_Context.status == BV_STATUS_READY)||(BVOPUS_Per_Context.status==BV_STATUS_RECEIVING))
  {
    status = BluevoiceOPUS_SendEnableNotifReq();
    
    if(status != BV_OPUS_SUCCESS)
    {
      Error_Handler();
    }
  }
  else if((BVOPUS_Per_Context.status == BV_STATUS_STREAMING)||(BVOPUS_Per_Context.status==BV_STATUS_FULLDUPLEX))
  {
    status = BluevoiceOPUS_SendDisableNotifReq();
    
    if(status != BV_OPUS_SUCCESS)
    {
      Error_Handler();
    }
  }
}

/** @defgroup BV_OPUS_PER_Private_Functions BV_OPUS_PER_Private_Functions
  * @{
  */
  
/**
 * @brief  Opus Encode and Decode configuration
 * @param  None
 * @retval BV_OPUS_Status: Value indicating success or error code.
 */
BV_OPUS_Status BVOPUS_PER_CodecConfiguration(void)
{
  EncConfigOpus.application = OPUS_APPLICATION_VOIP;
	
#if defined ( __ICCARM__ ) /*!< IAR Compiler */	
  EncConfigOpus.bitrate = 24000;
#elif defined( __GNUC__ )	
	EncConfigOpus.bitrate = 24000;
#elif defined ( __CC_ARM ) /*!< Keil Compiler */
	EncConfigOpus.bitrate = 12000;
#endif	
  EncConfigOpus.channels = AUDIO_IN_CHANNELS;
  EncConfigOpus.complexity = 0;
  EncConfigOpus.ms_frame = N_MS_PER_INTERRUPT;
  EncConfigOpus.sample_freq = AUDIO_IN_SAMPLING_FREQUENCY;
 
  BV_OPUS_Status status = BVOPUS_CodecEncInit(&EncConfigOpus);
  if(status != BV_OPUS_SUCCESS)
  {
    return status;
  }

#if defined ( __ICCARM__ ) /*!< IAR Compiler */	  
  DecConfigOpus.bitrate = 24000;
#elif defined( __GNUC__ )	
  DecConfigOpus.bitrate = 24000;	
#elif defined ( __CC_ARM ) /*!< Keil Compiler */	
	DecConfigOpus.bitrate = 12000;	
#endif	
  DecConfigOpus.channels = AUDIO_CHANNELS_OUT;
  DecConfigOpus.ms_frame = AUDIO_OUT_MS;
  DecConfigOpus.sample_freq = AUDIO_OUT_SAMPLING_FREQUENCY;
  
  status = BVOPUS_CodecDecInit(&DecConfigOpus);
  if(status != BV_OPUS_SUCCESS)
  {
    return BV_OPUS_ERROR;
  }

  return BV_OPUS_SUCCESS;  
}

/**
  * @brief  This function send is called when user button is pressed.
  * @param  None.
  * @retval BlueVoice status.
  */
BV_OPUS_Status BV_APP_PER_Send_Opus_Configuration(void)
{
  uint8_t data[4];
  
  data[0] = BV_OPUS_CONF_CMD;
 
  switch((int)EncConfigOpus.ms_frame)
  {
    case 5:
    {
      data[1] = BV_OPUS_FRAME_SIZE_5;
    }
    break;
    case 10:
    {
      data[1] = BV_OPUS_FRAME_SIZE_10;
    }
    break;
    case 20:
    {
      data[1] = BV_OPUS_FRAME_SIZE_20;
    }
    break;
    case 40:
    {
      data[1] = BV_OPUS_FRAME_SIZE_40;
    }
    break;
    case 60:
    {
      data[1] = BV_OPUS_FRAME_SIZE_60;
    }
    break;
  }
  
  switch(EncConfigOpus.sample_freq)
  {
    case 8000:
    {
      data[2] = BV_OPUS_SAMPLING_FREQ_8;
    }
    break;
    case 16000:
    {
      data[2] = BV_OPUS_SAMPLING_FREQ_16;
    }
    break;
    case 24000:
    {
      data[2] = BV_OPUS_SAMPLING_FREQ_24;
    }
    break;
    case 48000:
    {
      data[2] = BV_OPUS_SAMPLING_FREQ_48;
    }
    break;
  }
    
  if(EncConfigOpus.channels == 1)
  {
    data[3] = BV_OPUS_CHANNELS_1;
  }
  else if(EncConfigOpus.channels == 2)
  {
    data[3] = BV_OPUS_CHANNELS_2;
  }  
  
  return BluevoiceOPUS_SendCtrlData(data, 4);
}

/**
 * @brief  USB Audio output initialization
 * @param  None
 * @retval None
 */
void USB_Initialization(void)
{
  /* Initialize USB descriptor basing on channels number and sampling frequency */
  USBD_AUDIO_Init_Microphone_Descriptor(&hUSBDDevice, AUDIO_OUT_SAMPLING_FREQUENCY, AUDIO_CHANNELS_OUT);
  /* Init Device Library */
  USBD_Init(&hUSBDDevice, &AUDIO_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&hUSBDDevice, &USBD_AUDIO);
  /* Add Interface callbacks for AUDIO Class */
  USBD_AUDIO_RegisterInterface(&hUSBDDevice, &USBD_AUDIO_fops);
  /* Start Device Process */
  USBD_Start(&hUSBDDevice);
}

/**
 * @brief  Status Led management
 * @param  None
 * @retval None
 */
static void Led_manager(void)
{
  if(alarm_led_counter<200)
  {
    alarm_led_counter++;
    if(alarm_led_counter==200)
    {
      BSP_LED_Off(LED_RED);
    }
  } 

  led_counter++;
  switch(BVOPUS_Per_Context.status)
  {
    case BV_STATUS_UNITIALIZED:
      {
            if(led_counter==20)
            {
              led_counter = 0;
              BSP_LED_On(LED_GREEN);
            }
      }
      break;
    case BV_STATUS_INITIALIZED:
      {
        if(led_counter==60)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_GREEN);
        }
      }
      break;
    case BV_STATUS_SERVICE_DISC:
      {
        BSP_LED_Off(LED_GREEN);
        if(led_counter==30)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break; 
    case BV_STATUS_CHAR_DISC:
      {
        if(led_counter==30)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break;
    case BV_STATUS_ENABLE_NOTIF:
      {
        if(led_counter==30)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break;
    case BV_STATUS_READY:
      {
        if(led_counter==30)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break;
    case BV_STATUS_STREAMING:
      {
        if(led_counter==10)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break;  
    case BV_STATUS_FULLDUPLEX:
      {
        if(led_counter==5)
        {
          led_counter = 0;
          BSP_LED_Toggle(LED_BLUE);
        }
      }
      break;
    case BV_STATUS_RECEIVING:
      {
        BSP_LED_On(LED_BLUE);
      }
      break;           
  }
}

/**
* @brief  Audio Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
* @brief  Audio Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void CCA02M2_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
* @brief  Audio processing, called when N_MS_PER_INTERRUPT ms of PDM data are available.
* @param  none
* @retval None
*/
void AudioProcess(void)
{   
  CCA02M2_AUDIO_IN_PDMToPCM(CCA02M2_AUDIO_INSTANCE, (uint16_t * )PDM_Buffer, PCM_Buffer);  
  
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SEND_DATA_ID, CFG_SCH_PRIO_0);
}

/**
* @brief  Encode and send an audio frame.
* @param  none
* @retval None
*/
void SendData(void)
{
  int opus_err;
  status = BluevoiceOPUS_SendAudioData((uint8_t*) PCM_Buffer, &opus_err);
  
  if(status != BV_OPUS_SUCCESS)
  {
    BSP_LED_On(LED_RED);
    alarm_led_counter =0;
  }
}

/**
* @brief  Called when a connection or disconnection event occurs.
* @param  pNotification: a connection event
* @retval None
*/
void APP_BLE_Notification(APP_BLE_ConnHandle_Not_evt_t *pNotification)
{
  switch(pNotification->Evt_Opcode)
  {
    case BV_CONN_HANDLE_EVT :
      {
        BVOPUS_Per_Context.connHandle = pNotification->ConnectionHandle;
        led_counter = 0;
      }
      break;

    case BV_DISCON_HANDLE_EVT :
      {
        BVOPUS_Per_Context.connHandle =  0x00;     
        BVOPUS_Per_Context.status = BV_STATUS_INITIALIZED;
        BVOPUS_Per_Context.BV_handle_CEN.CharAudioHandle = 0;
        BVOPUS_Per_Context.BV_handle_CEN.CharCtrlHandle = 0;
        BVOPUS_Per_Context.BV_handle_CEN.ServiceEndHandle = 0;
        BVOPUS_Per_Context.BV_handle_CEN.ServiceHandle = 0;
        service_discovered = 0;
        char_audio_discovered = 0;
        char_ctrl_discovered = 0;
        RX_MTU = 0;
        CCA02M2_AUDIO_IN_Pause(CCA02M2_AUDIO_INSTANCE);
        led_counter = 0;
        BSP_LED_Off(LED_BLUE);
      }
      break;

    default:
      break;
  }

  return;
}

/**
  * @brief  This function is called to enable audio notifications on the connected module.
  * @param  None.
  * @retval BV_OPUS_Status: Value indicating success or error code.
  */
BV_OPUS_Status BluevoiceOPUS_EnableAudio_Notif(void)
{
  uint8_t client_char_conf_data[] = { 0x01, 0x00 }; 
  uint32_t timeout_cnt = 0;

  while (aci_gatt_write_char_desc(BVOPUS_Per_Context.connHandle,
                                          BVOPUS_Per_Context.BV_handle_CEN.CharAudioHandle + 2, 2,
                                          client_char_conf_data) == BLE_STATUS_NOT_ALLOWED)
  {
    /* Radio is busy */
    /*-------------------------------------------------------*/
    timeout_cnt++;
    if (timeout_cnt >= 0xff)
    {             
      return BV_OPUS_TIMEOUT;
    }
    /*-------------------------------------------------------*/
  }
  return BV_OPUS_SUCCESS;
}

/**
  * @brief  This function is called to disable audio notifications on the connected module.
  * @param  None.
  * @retval BV_OPUS_Status: Value indicating success or error code.
  */
BV_OPUS_Status BluevoiceOPUS_DisableAudio_Notif(void)
{
  uint8_t client_char_conf_data[] = { 0x00, 0x00 }; 
  uint32_t timeout_cnt = 0;

  while (aci_gatt_write_char_desc(BVOPUS_Per_Context.connHandle,
                                          BVOPUS_Per_Context.BV_handle_CEN.CharAudioHandle + 2, 2,
                                          client_char_conf_data) == BLE_STATUS_NOT_ALLOWED)
  {
    /* Radio is busy */
    /*-------------------------------------------------------*/
    timeout_cnt++;
    if (timeout_cnt >= 0xff)
    {             
      return BV_OPUS_TIMEOUT;
    }
    /*-------------------------------------------------------*/
  }
  return BV_OPUS_SUCCESS;
}

/**
  * @brief  This function is called to enable control notifications on the connected module.
  * @param  None.
  * @retval BV_OPUS_Status: Value indicating success or error code.
  */
BV_OPUS_Status BluevoiceOPUS_EnableCtrl_Notif(void)
{
  uint8_t client_char_conf_data[] = { 0x01, 0x00 }; 
  uint32_t timeout_cnt = 0;

  while (aci_gatt_write_char_desc(BVOPUS_Per_Context.connHandle,
                                          BVOPUS_Per_Context.BV_handle_CEN.CharCtrlHandle + 2, 2,
                                          client_char_conf_data) == BLE_STATUS_NOT_ALLOWED)
  {
    /* Radio is busy */
    /*-------------------------------------------------------*/
    timeout_cnt++;
    if (timeout_cnt >= 0xff)
    {             
      return BV_OPUS_TIMEOUT;
    }
    /*-------------------------------------------------------*/
  }
  return BV_OPUS_SUCCESS;
}

/**
  * @brief  This function is called to disable control notifications on the connected module.
  * @param  None.
  * @retval BV_OPUS_Status: Value indicating success or error code.
  */
BV_OPUS_Status BluevoiceOPUS_DisableCtrl_Notif(void)
{
  uint8_t client_char_conf_data[] = { 0x00, 0x00 }; 
  uint32_t timeout_cnt = 0;

  while (aci_gatt_write_char_desc(BVOPUS_Per_Context.connHandle,
                                          BVOPUS_Per_Context.BV_handle_CEN.CharCtrlHandle + 2, 2,
                                          client_char_conf_data) == BLE_STATUS_NOT_ALLOWED)
  {
    /* Radio is busy */
    /*-------------------------------------------------------*/
    timeout_cnt++;
    if (timeout_cnt >= 0xff)
    {             
      return BV_OPUS_TIMEOUT;
    }
    /*-------------------------------------------------------*/
  }
  return BV_OPUS_SUCCESS;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not.
 */
static SVCCTL_EvtAckStatus_t Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blue_aci *blue_evt;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case EVT_VENDOR:
    {
      blue_evt = (evt_blue_aci*)event_pckt->data;
      switch(blue_evt->ecode)
      {
        case EVT_BLUE_ATT_EXCHANGE_MTU_RESP:
        {
          aci_att_exchange_mtu_resp_event_rp0 *pr = (void*)blue_evt->data;
          RX_MTU = pr->Server_RX_MTU;
          BluevoiceOPUS_SetMaxDataLength(RX_MTU);
        }
        break;

        case EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP:
        {
          uint8_t idx = 0;
          aci_att_read_by_group_type_resp_event_rp0 *pr = (void*) blue_evt->data;
          if(pr->Attribute_Data_Length == 20)
          {
            while(pr->Data_Length > 0)
            {
              if(memcmp(&pr->Attribute_Data_List[idx + 4], bvopus_service_uuid, 16)==0)
              {
                BVOPUS_Per_Context.BV_handle_CEN.ServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                BVOPUS_Per_Context.BV_handle_CEN.ServiceEndHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx + 2]);
                service_discovered = 1;
              }
              idx = idx + pr->Attribute_Data_Length;
              pr->Data_Length = pr->Data_Length - pr->Attribute_Data_Length;
            }
          }      
        }
        break;
        
        case EVT_BLUE_ATT_READ_BY_TYPE_RESP:
        {
          uint8_t idx = 0;
          aci_att_read_by_type_resp_event_rp0 *pr = (void*) blue_evt->data;
          if(pr->Handle_Value_Pair_Length == 21)
          {
            while(pr->Data_Length > 0)
            {
              if(memcmp(&pr->Handle_Value_Pair_Data[idx + 5], bvopus_audio_char_uuid, 16)==0)
              {
                BVOPUS_Per_Context.BV_handle_CEN.CharAudioHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                char_audio_discovered = 1;
              }
              if(memcmp(&pr->Handle_Value_Pair_Data[idx + 5], bvopus_ctrl_char_uuid, 16)==0)
              {
                BVOPUS_Per_Context.BV_handle_CEN.CharCtrlHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                char_ctrl_discovered = 1;
              }
              idx = idx + pr->Handle_Value_Pair_Length;
              pr->Data_Length = pr->Data_Length - pr->Handle_Value_Pair_Length;
            }
          }  
        }
        break;
        
        case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        {      
          if(BVOPUS_Per_Context.status == BV_STATUS_INITIALIZED)
          {
            led_counter = 0;
            UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
          }
          else if(BVOPUS_Per_Context.status == BV_STATUS_SERVICE_DISC)
          {  
            led_counter = 0;
            UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
          }
          else if(BVOPUS_Per_Context.status == BV_STATUS_CHAR_DISC)
          {
            led_counter = 0;
            UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_GATT_PROC_COMPLETE);
          }     
        }
        break;
        
        case EVT_BLUE_GATT_NOTIFICATION:
        {
          aci_gatt_notification_event_rp0 *pr = (void*)blue_evt->data;                   
          BV_APP_PER_GATTNotification_CB(pr->Attribute_Handle, pr->Attribute_Value_Length, pr->Attribute_Value);         
        }
        break;
        default:
          break;
      }
    }
    break; 

    default:
      break;
  }
  
  return(return_value);
}

/**
* @brief  This function is called when there is a notification from the sever.
* @param  attr_handle Handle of the attribute
* @param  attr_len    Length of attribute value in the notification
* @param  attr_value  Attribute value in the notification
* @retval None
*/
void BV_APP_PER_GATTNotification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{ 
  if(attr_handle == BVOPUS_Per_Context.BV_handle_CEN.CharAudioHandle+1)
  {  
    uint16_t PCMSamples = 0;
    int opur_error;
  
    status = BluevoiceOPUS_ParseData(attr_value, attr_len, (uint8_t *) PCM_Buffer_USB, &PCMSamples, &opur_error);
    
    if(status == BV_OPUS_SUCCESS)
    {
      if (PCMSamples > 0) 
      { 
        Send_Audio_to_USB((int16_t *)PCM_Buffer_USB, PCMSamples);  
      }
    } 
  }
  else if(attr_handle == BVOPUS_Per_Context.BV_handle_CEN.CharCtrlHandle + 1)
  {
    if(attr_value[0] == BV_OPUS_CONTROL)
    {     
      switch(attr_value[1])
      {
      case BV_OPUS_ENABLE_NOTIF_REQ:
        {
          status = BluevoiceOPUS_EnableAudio_Notif();       
        
          if(status != BV_OPUS_SUCCESS)
          {
            Error_Handler();
          }        
          
          if(BVOPUS_Per_Context.status == BV_STATUS_READY)
          {
            BVOPUS_Per_Context.status = BV_STATUS_RECEIVING;
            led_counter = 0;
          }     
          else if(BVOPUS_Per_Context.status == BV_STATUS_STREAMING)
          {
            BVOPUS_Per_Context.status = BV_STATUS_FULLDUPLEX;
            led_counter = 0;
          }        
        }
        break;
      case BV_OPUS_DISABLE_NOTIF_REQ:
        {
          status = BluevoiceOPUS_DisableAudio_Notif();
     
          if(status != BV_OPUS_SUCCESS)
          {
            Error_Handler();
          }
          
          if(BVOPUS_Per_Context.status == BV_STATUS_RECEIVING)
          {
            BVOPUS_Per_Context.status = BV_STATUS_READY;
            led_counter = 0;
          }
          else if(BVOPUS_Per_Context.status == BV_STATUS_FULLDUPLEX)
          {
            BVOPUS_Per_Context.status = BV_STATUS_STREAMING;
            led_counter = 0;
          }      
        }
        break;
      }
    }   
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None.
  * @retval None
  */
void Error_Handler(void)
{
  BSP_LED_On(LED_RED);
  while(1);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
