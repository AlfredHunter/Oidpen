#ifndef HAL_OID_H
#define HAL_OID_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "hal_types.h"
#include "hal_board.h"

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/
#define DEVICE_ADDRESS   0X78
#define OID_CODE         0X88

typedef enum {
  OID_POWER_OFF = 0x00,
  OID_POWER_ON  = 0x01
} oidState_t;
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/* PowerOn OID Sensor 
  * 1) power up the OID sensor
  * 2) reset the processor
  */

extern void halOidPower(uint8 on) ;
extern void HalOidInit(void);
extern oidState_t getOidState(void);

#ifdef __cplusplus
}
#endif


#endif 
