#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_led.h"
#include "OSAL_Timers.h"
#include  "bcomdef.h"
#include "peripheral.h"
#include "central.h"
#include "hal_oid.h"


volatile static uint8 oidState = OID_POWER_OFF;
/* enable oid
  * 1) power up the OID sensor
  * 2) reset   the oid
  */
/***************************************************************************************************
 * @fn      halOidInit
 *
 * @brief   
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalOidInit(void) 
{
  HAL_CONFIG_IO_OUTPUT(HAL_OID_RST_PORT,HAL_OID_RST_PIN,1);  
  HAL_CONFIG_IO_OUTPUT(HAL_OID_POWER_PORT,HAL_OID_POWER_PIN,0); 
  HAL_CONFIG_IO_OUTPUT(HAL_OID_WAKEUP_PORT, HAL_OID_WAKEUP_PIN, 0);  
//  HAL_CONFIG_IO_OUTPUT(HAL_OID_WAKEUP_PORT, HAL_OID_WAKEUP_PIN, 1 );  

  halOidPower(0);
}
/***************************************************************************************************
 * @fn      halOidPower
 *
 * @brief   
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void halOidPower(uint8 on)  
{
  if(1 == on) 
  {
    HAL_IO_SET_HIGH(HAL_OID_POWER_PORT, HAL_OID_POWER_PIN);  
//    osal_start_timerEx (Hal_TaskID, HAL_OID_EVENT, 1000);
    HAL_IO_SET_HIGH(HAL_OID_RST_PORT,HAL_OID_RST_PIN);  
    HAL_IO_SET_HIGH(HAL_OID_WAKEUP_PORT, HAL_OID_WAKEUP_PIN);
    oidState = OID_POWER_ON;
  } 
  else 
  {
    oidState = OID_POWER_OFF;
    HAL_IO_SET_LOW(HAL_OID_WAKEUP_PORT, HAL_OID_WAKEUP_PIN);
    HAL_IO_SET_LOW(HAL_OID_RST_PORT,HAL_OID_RST_PIN);  
    HAL_IO_SET_LOW(HAL_OID_POWER_PORT, HAL_OID_POWER_PIN);
  }
}

uint8 getOidState(void)
{
  return oidState;
}
