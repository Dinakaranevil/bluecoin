/**
******************************************************************************
* @file    BlueCoin_conf.h
* @author  SRA
* @version v2.4.0
* @date    23-Apr-2021
* @brief   This file contains definitions for the MEMS components bus interfaces
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUECOIN_CONF_H__
#define __BLUECOIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "BlueCoin_bus.h"
#include "BlueCoin_errno.h"
  
/*Uncomment this define if you want to configure and start acquisition 
independentrly from USB functionalities*/
#define DISABLE_USB_DRIVEN_ACQUISITION
  
#define AUDIO_CHANNELS 				4
#define AUDIO_SAMPLING_FREQUENCY 		16000  
#define BSP_AUDIO_IN_INSTANCE                   0U   /* Define the audio peripheral used: 0U = I2S */  
#define BSP_AUDIO_OUT_INSTANCE                  0U   /* Define the audio peripheral used: 0U = SAI */  
#define AUDIO_VOLUME_INPUT                      64U
#define AUDIO_VOLUME_OUTPUT                     50U
#define BSP_AUDIO_IN_IT_PRIORITY                6U

#define USE_MOTION_SENSOR_LSM6DSM_0             1U
#define USE_MOTION_SENSOR_LSM303AGR_ACC_0       0U
#define USE_MOTION_SENSOR_LSM303AGR_MAG_0       1U
#define USE_ENV_SENSOR_LPS22HB_0                1U

#ifdef __cplusplus
}
#endif

#endif /* __BLUECOIN_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

