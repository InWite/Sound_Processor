/**
 ******************************************************************************
 * @file    app_ble.c
 * @author  SRA-A&SP
 * @version V 1.1.0
 * @date    19-Dic-2019
 * @brief   BLE Application
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"
#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "bvopus_per_app.h"

  
/** @addtogroup BV_OPUS_APP
  * @{
  */

/** @addtogroup BV_OPUS_PERIPHERAL
  * @{
  */

/** @defgroup BV_OPUS_PER_BLE BV_OPUS_PER_BLE
  * @{
  */


/** @defgroup BV_OPUS_PER_BLE_Private_Typedef  BV_OPUS_PER_BLE_Private_Typedef
  * @{
  */
   
/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /* IO capability of the device */
  uint8_t ioCapability;

  /* Authentication requirement of the device, Man In the Middle protection required? */
  uint8_t mitm_mode;

  /* bonding mode of the device */
  uint8_t bonding_mode;

  /* Flag to tell whether OOB data has to be used during the pairing process */
  uint8_t OOB_Data_Present;

  /* OOB data to be used in the pairing process if OOB_Data_Present is set to TRUE */
  uint8_t OOB_Data[16];

  /* this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey */
  uint8_t Use_Fixed_Pin;

  /* minimum encryption key size requirement */
  uint8_t encryptionKeySizeMin;

  /* maximum encryption key size requirement */
  uint8_t encryptionKeySizeMax;

  /* fixed pin to be used in the pairing process if Use_Fixed_Pin is set to 1 */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;

} tSecurityParams;

/**
 * global context contains the variables common to all services
 */
typedef struct _tBLEProfileGlobalContext
{
  /* security requirements of the host */
  tSecurityParams bleSecurityParam;

  /* gap service handle */
  uint16_t gapServiceHandle;

  /* device name characteristic handle */
  uint16_t devNameCharHandle;

  /* appearance characteristic handle */
  uint16_t appearanceCharHandle;

  /* connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF */
  uint16_t connectionHandle;

} BleAppContext_t;

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_BLE_Private_Defines BV_OPUS_PER_BLE_Private_Defines
  * @{
  */

#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define BD_ADDR_SIZE_LOCAL    6

/**
  * @}
  */

/** @defgroup BV_OPUS_PER_BLE_Private_Variables BV_OPUS_PER_BLE_Private_Variables
  * @{
  */

  
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];
const char BV_PER_NAME[APPBLE_GAP_DEVICE_NAME_LENGTH] = {'B','V','L','-','W','B','1'};   
static BleAppContext_t BleApplicationContext;

/**
  * @}
  */
  
/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx( void * pPayload );
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress( void );
static void Advertise_Request( void );
static void LinkConfiguration(void);

/** @defgroup BV_OPUS_PER_BLE_Functions_Definition BV_OPUS_PER_BLE_Functions_Definition
  * @{
  */
  
/**
* @brief  This function manages BLE initialization.
* @param  None.
* @retval None
*/
void APP_BLE_Init( void )
{
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
    0,                                  /** BleBufferSize not used */
    CFG_BLE_NUM_GATT_ATTRIBUTES,
    CFG_BLE_NUM_GATT_SERVICES,
    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
    CFG_BLE_NUM_LINK,
    CFG_BLE_DATA_LENGTH_EXTENSION,
    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
    CFG_BLE_MBLOCK_COUNT,
    CFG_BLE_MAX_ATT_MTU,
    CFG_BLE_SLAVE_SCA,
    CFG_BLE_MASTER_SCA,
    CFG_BLE_LSE_SOURCE,
    CFG_BLE_MAX_CONN_EVENT_LENGTH,
    CFG_BLE_HSE_STARTUP_TIME,
    CFG_BLE_VITERBI_MODE,
    CFG_BLE_LL_ONLY,
    0}
  };
  
  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /* Do not allow stanby in the application */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /* Register the hci transport layer to handle BLE User Asynchronous Events */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /* Starts the BLE Stack on CPU2 */
  SHCI_C2_BLE_Init( &ble_init_cmd_packet );

  /* Initialization of HCI & GATT & GAP layer */
  Ble_Hci_Gap_Gatt_Init();

  /* Initialization of the BLE Services */
  SVCCTL_Init();

  /* From here, all initialization are BLE application specific */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_START_ADV_ID, UTIL_SEQ_RFU, Advertise_Request);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_LINK_CONFIG_ID, UTIL_SEQ_RFU, LinkConfiguration);

  /* Initialize BlueVoice Application */
  BVOPUS_PER_Init();

  UTIL_SEQ_SetTask( 1<<CFG_TASK_START_ADV_ID, CFG_SCH_PRIO_1);

  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;
  hci_le_connection_complete_event_rp0 * connection_complete_event;
  hci_disconnection_complete_event_rp0 *cc = (void *) event_pckt->data;
  
  switch (event_pckt->evt)
  {
    case EVT_DISCONN_COMPLETE:

      if (cc->Connection_Handle == BleApplicationContext.connectionHandle)
      {
        BleApplicationContext.connectionHandle = 0;
        APP_BLE_ConnHandle_Not_evt_t BVOPUS_APP_event;
        BVOPUS_APP_event.Evt_Opcode = BV_DISCON_HANDLE_EVT;
        BVOPUS_APP_event.ConnectionHandle = 0;
        APP_BLE_Notification(&BVOPUS_APP_event);
        UTIL_SEQ_SetTask( 1<<CFG_TASK_START_ADV_ID, CFG_SCH_PRIO_1);
      }
      break; /* EVT_DISCONN_COMPLETE */

    case EVT_LE_META_EVENT:
    {
      meta_evt = (evt_le_meta_event*) event_pckt->data;

      switch (meta_evt->subevent)
      {
        case EVT_LE_CONN_COMPLETE:
          
          /* The connection is done */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;
          BleApplicationContext.connectionHandle = connection_complete_event->Connection_Handle;

          /* Connection interval parameters update request in order to be compatible with Android and iOS*/
          aci_l2cap_connection_parameter_update_req(connection_complete_event->Connection_Handle,
                                                        8   /* interval_min*/,
                                                        17  /* interval_max */,
                                                        0   /* slave_latency */,
                                                        400 /*timeout_multiplier*/);
          
          APP_BLE_ConnHandle_Not_evt_t BVOPUS_APP_event;
          BVOPUS_APP_event.Evt_Opcode = BV_CONN_HANDLE_EVT;
          BVOPUS_APP_event.ConnectionHandle = BleApplicationContext.connectionHandle;
          APP_BLE_Notification(&BVOPUS_APP_event);

          UTIL_SEQ_SetTask(1 << CFG_TASK_LINK_CONFIG_ID, CFG_SCH_PRIO_1);
          break; /* HCI_EVT_LE_CONN_COMPLETE */
          
        default:
          break;
          
      break; /* HCI_EVT_LE_META_EVENT */
     }
      default:
      break;
   }
  }
  return (SVCCTL_UserEvtFlowEnable);
}


/**
  * @brief  This function is called when the button 1 has been pressed.
  * @param  None
  * @retval None.
  */
void APP_BLE_Key_Button1_Action(void)
{
  Button_Trigger_Received();
}


/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

/**
  * @brief  This function initializes GAP and GATT level.
  * @param  None
  * @retval None.
  */
static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t index;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint16_t appearance[1] = { CFG_GAP_APPEARANCE }; /* Generic Heart Rate Sensor */
  
  /* Initialize HCI layer */
  
  /* HCI Reset to synchronise BLE Stack */
   hci_reset();
  
   /* Write the BD Address */
  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, (uint8_t*) bd_addr);

  /* Set TX Power to 0dBm */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  
  /* Initialize GATT interface */
  aci_gatt_init();

  aci_gap_init(GAP_PERIPHERAL_ROLE, 0, APPBLE_GAP_DEVICE_NAME_LENGTH, &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

  aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(BV_PER_NAME), (uint8_t *) BV_PER_NAME);

  aci_gatt_update_char_value(gap_service_handle, gap_appearance_char_handle, 0, 2, (uint8_t *)&appearance);

  /* Initialize IO capability */
  BleApplicationContext.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.bleSecurityParam.ioCapability);

  /* Initialize authentication */
  BleApplicationContext.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.bleSecurityParam.OOB_Data_Present = 0;
  BleApplicationContext.bleSecurityParam.encryptionKeySizeMin = 8;
  BleApplicationContext.bleSecurityParam.encryptionKeySizeMax = 16;
  BleApplicationContext.bleSecurityParam.Use_Fixed_Pin = 0;
  BleApplicationContext.bleSecurityParam.Fixed_Pin = 123456;
  BleApplicationContext.bleSecurityParam.bonding_mode = 1;
  for (index = 0; index < 16; index++)
  {
    BleApplicationContext.bleSecurityParam.OOB_Data[index] = (uint8_t) index;
  }

  aci_gap_set_authentication_requirement(BleApplicationContext.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.bleSecurityParam.mitm_mode,
                                         0,
                                         0,
                                         BleApplicationContext.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.bleSecurityParam.Fixed_Pin,
                                         0
                                         );

  /* Initialize whitelist */
  if (BleApplicationContext.bleSecurityParam.bonding_mode)
  {
    aci_gap_configure_whitelist();
  }
}


/**
  * @brief  This function is called to set advertise mode.
  * @param  None
  * @retval None.
  */
static void Advertise_Request( void )
{
  const char local_name[] =
  {
    AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'V', 'L', '-', 'W', 'B', '1' 
  };
  
  uint8_t manuf_data[20] = {
  2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
  8,0x09, 'B', 'V', 'L', '-', 'W', 'B', '1', // Complete Name
  7,0xFF,0x01, /* SDK version */
         0x80, /* NUCLEO-Board */
         0x00,
         0x00,
         0x00, 
         0x00
  };

  /* disable scan response */
  hci_le_set_scan_response_data(0, NULL);

  aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name),(uint8_t*) &local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(20, manuf_data);

}

/**
  * @brief  This function requests a connection with the peripheral node.
  * @param  None.
  * @retval None.
  */
static void LinkConfiguration( void )
{
  BVOPUS_App_LinkReadyNotification();
}

/**
  * @brief  This function returns the BLE address.
  * @param  None.
  * @retval BLE address.
  */
const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)( (udn & 0x00FF0000) >> 16 );
    bd_addr_udn[3] = (uint8_t)device_id;
    bd_addr_udn[4] = (uint8_t)(company_id & 0x000000FF);;
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }
  }
  return bd_addr;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_1);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload; 

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;
  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);

      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
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
