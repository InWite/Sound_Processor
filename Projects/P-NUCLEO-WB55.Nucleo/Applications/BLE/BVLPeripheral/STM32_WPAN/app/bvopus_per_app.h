
/**
 ******************************************************************************
 * @file    bvopus_per_app.h
 * @author  SRA-A&SP
 * @version V 1.1.0
 * @date    19-Dic-2019
 * @brief   Header for bvopus_per_app.c module
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BVOPUS_PER_APP_H
#define __BVOPUS_PER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BV_OPUS_APP BV_OPUS_APP
  * @{
  */

/** @defgroup BV_OPUS_PERIPHERAL BV_OPUS_PERIPHERAL
  * @{
  */

/** @defgroup BV_OPUS_PER_Exported_Types BV_OPUS_PER_Exported_Types
  * @{
  */
 
/**
  * @brief Connection notification code
  */
typedef enum
{
  BV_CONN_HANDLE_EVT,
  BV_DISCON_HANDLE_EVT,
} APP_BLE_Opcode_Notification_evt_t;

/**
  * @brief Connection notification
  */
typedef struct
{
  APP_BLE_Opcode_Notification_evt_t     Evt_Opcode;
  uint16_t                              ConnectionHandle;
}APP_BLE_ConnHandle_Not_evt_t;

/**
  * @brief Audio Sampling Frequency enumerator
  */
typedef enum
{
  SAMPLING_FREQ_8000 = 8000,  
  SAMPLING_FREQ_16000 = 16000
}OUTPUT_FREQ_TypeDef;

/** 
* @brief BlueVoice OPUS profile status.
*/
typedef enum
{
  BV_STATUS_UNITIALIZED = 0x00,                 /*!< BlueVoice Profile is not initialized.*/
  BV_STATUS_INITIALIZED = 0x10,                 /*!< BlueVoice Profile is initialized.*/
  BV_STATUS_SERVICE_DISC = 0x21,                /*!< Service discovery procedure.*/  
  BV_STATUS_CHAR_DISC = 0x22,                   /*!< Audio and control characteristic discovery procedure.*/ 
  BV_STATUS_ENABLE_NOTIF = 0x23,                /*!< Enable notification discovered.*/ 
  BV_STATUS_READY = 0x30,                       /*!< BlueVoice channel is ready and idle.*/
  BV_STATUS_STREAMING = 0x40,                   /*!< BlueVoice device is streaming data.*/
  BV_STATUS_RECEIVING = 0x50,                   /*!< BlueVoice device is receiving data.*/
  BV_STATUS_FULLDUPLEX = 0x60                   /*!< BlueVoice device is in full-duplex mode.*/   
} BVOPUS_PER_APP_Status_t;

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_Exported_Defines BV_OPUS_PER_Exported_Defines
  * @{
  */

/*Comment this define if you want to configure and start acquisition 
depending on USB functionalities implemented by user*/
#define DISABLE_USB_DRIVEN_ACQUISITION

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_Exported_Variables BV_OPUS_PER_Exported_Variables
  * @{
  */

extern uint16_t PDM_Buffer[];  

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_Exported_Functions BV_OPUS_PER_Exported_Functions
  * @{
  */
  
/**
 * @brief  Bluevoice Opus service initialization.
 * @param  None
 * @retval None
 */
void BVOPUS_PER_Init( void );

/**
* @brief  Called when a connection or disconnection event occurs.
* @param  pNotification: a connection event
* @retval None
*/
void APP_BLE_Notification( APP_BLE_ConnHandle_Not_evt_t *pNotification );

/**
 * @brief  BLE Link ready notification.
 * @param  None
 * @retval None
 */
void BVOPUS_App_LinkReadyNotification(void);

/**
* @brief  This function manages the sw1 button event.
* @param  None.
* @retval None
*/
void Button_Trigger_Received( void );


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /*__BVOPUS_PER_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
