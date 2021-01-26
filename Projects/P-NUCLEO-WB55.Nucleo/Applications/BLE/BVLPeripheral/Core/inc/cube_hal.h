/**
  ******************************************************************************
  * @file    cube_hal.h
  * @date    30-Jul-2019
  * @brief   CUBE includes file
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

#ifndef _CUBE_HAL_H_
#define _CUBE_HAL_H_

/* Includes ------------------------------------------------------------------*/
#define USB_IRQHandler OTG_FS_IRQHandler
#include "stm32wbxx_hal.h"
#include "stm32wbxx.h"
#include "stm32wbxx_it.h"
#include "stm32wbxx_nucleo.h"
#include "app_common.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio_in.h"
#include "usbd_audio_if.h"    
#include "bvopus_per_app.h"
#include "cca02m2_audio.h"


#endif //_CUBE_HAL_H_
