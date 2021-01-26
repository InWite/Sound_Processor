
/**
  ******************************************************************************
  * @file    app_ble.h
  * @author  SRA-A&SP
  * @version V 1.1.0
  * @date    19-Dic-2019
  * @brief   Header for ble application
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
#ifndef __APP_BLE_H
#define __APP_BLE_H

#ifdef __cplusplusl
extern "C" {
#endif

  /* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"
  
  /* Exported types ------------------------------------------------------------*/
typedef enum
{
  APP_BLE_IDLE,
  APP_BLE_CONNECTED,
} APP_BLE_ConnStatus_t;  

/**
 * @brief Application status
 */
typedef enum 
{
  APP_SUCCESS = 0x00,
  APP_ERROR = 0x10
} APP_Status;


/* Exported functions ------------------------------------------------------- */

void APP_BLE_Init( void );
void APP_BLE_UserEvtRx( void * pPayload );
void APP_BLE_StatusNot( HCI_TL_CmdStatus_t status );
void APP_BLE_Key_Button1_Action(void);

#ifdef __cplusplus
}
#endif

#endif /*__APP_BLE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
