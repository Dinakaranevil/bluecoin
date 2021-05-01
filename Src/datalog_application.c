/**
******************************************************************************
* @file    datalog_application.c
* @author  SRA
* @version v2.4.0
* @date    23-Apr-2021
* @brief   This file provides a set of functions to handle the datalog
*          application.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "datalog_application.h"
#include "main.h"
#include "usbd_cdc_interface.h"
#include "string.h"
#include "BlueCoin.h"
#include <math.h>
    
/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

FRESULT res;                                          /* FatFs function common result code */
uint32_t byteswritten, bytesread;                     /* File write/read counts */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

/* Private function prototypes -----------------------------------------------*/
static void MX_DataLogTerminal_Init(void);  

volatile uint8_t SD_Log_Enabled = 0;

char newLine[] = "\r\n";

extern void *VL53L0X_0_handler;
extern volatile uint8_t no_VL53L0X;


/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      while(1)
      {
        BSP_LED_On(LED1);
        HAL_Delay(500);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
      }
    }
  }
}
  
/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  char header[] = "T [ms],AccX [mg],AccY [mg],AccZ [mg],GyroX [mdps],GyroY [mdps],GyroZ [mdps],MagX [mgauss],MagY [mgauss],MagZ [mgauss],P [mB],T [°C],\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30] = {0};
  
  sprintf(file_name, "%s%.3d%s", "BlueCoin_Log_N", sdcard_file_counter, ".csv");
  sdcard_file_counter++;

  HAL_Delay(100);

  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    return 0;
  }
  
  if(f_write(&MyFile, (const void*)&header, sizeof(header)-1, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}

uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size)
{
  uint32_t byteswritten;
  
  if(f_write(&MyFile, s, size, (void *)&byteswritten) != FR_OK)
  {
    return 0;
  }
  return 1;
}


/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  f_close(&MyFile);
}

void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&MyFile, (const void*)&newLine, 2, (void *)&byteswritten);
}

 
int32_t getSensorsData( T_SensorsData *mptr)
{
  int32_t ret = BSP_ERROR_NONE;
  mptr->ms_counter = HAL_GetTick();
  uint8_t proxStatus, pDataready;
  
  /* Get Data from Sensors */  
  if ( BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_ACCELERO, &mptr->acc ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->acc.x = 0;
    mptr->acc.y = 0;
    mptr->acc.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_GYRO, &mptr->gyro ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->gyro.x = 0;
    mptr->gyro.y = 0;
    mptr->gyro.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_MOTION_SENSOR_GetAxes(LSM303AGR_MAG_0, MOTION_MAGNETO, &mptr->mag ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->mag.x = 0;
    mptr->mag.y = 0;
    mptr->mag.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE, &mptr->pressure ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->pressure = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  if ( BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE, &mptr->temperature ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->temperature = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  
  BSP_PROX_IsInitialized( VL53L0X_0_handler, &proxStatus );
  if(proxStatus == 1)
  {
    BSP_PROX_Start_Measurement(VL53L0X_0_handler);
    do
    {
      BSP_PROX_Get_Measurement_DataReady( VL53L0X_0_handler, &pDataready);
    }
    while(!pDataready);
    BSP_PROX_Get_Range(VL53L0X_0_handler, &mptr->range);
  }
  return ret;
}


void MX_X_CUBE_MEMS1_Init(void)
{
  MX_DataLogTerminal_Init();
}


/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_DataLogTerminal_Init(void)
{
  BSP_MOTION_SENSOR_Init(LSM303AGR_MAG_0, MOTION_MAGNETO);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM303AGR_MAG_0, MOTION_MAGNETO, LSM303AGR_MAG_ODR);
  BSP_MOTION_SENSOR_SetFullScale(LSM303AGR_MAG_0, MOTION_MAGNETO, LSM303AGR_MAG_FS);
  
  BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_ACCELERO | MOTION_GYRO);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_ACCELERO, LSM6DSM_ACC_ODR);
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSM_0, MOTION_ACCELERO, LSM6DSM_ACC_FS);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_GYRO, LSM6DSM_GYRO_ODR);
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSM_0, MOTION_GYRO, LSM6DSM_GYRO_FS);
  
  BSP_ENV_SENSOR_Init(LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);
  BSP_ENV_SENSOR_SetOutputDataRate(LPS22HB_0, ENV_TEMPERATURE, LPS22HB_ODR);
  BSP_ENV_SENSOR_SetOutputDataRate(LPS22HB_0, ENV_PRESSURE, LPS22HB_ODR);
  
  if (BSP_PROX_Init( VL53L0X_0, &VL53L0X_0_handler) == COMPONENT_ERROR)
  {
    no_VL53L0X=1;
  }
  if(!no_VL53L0X)
  {
    BSP_PROX_Sensor_Enable( VL53L0X_0_handler );
    BSP_PROX_Set_DeviceMode( VL53L0X_0_handler, CONTINUOUS );
    BSP_PROX_Set_RangingProfile(VL53L0X_0_handler, VL53L0X_PROFILE_LONG_RANGE);
  }
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
