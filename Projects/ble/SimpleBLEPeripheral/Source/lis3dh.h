#ifndef LIS3DH_H
#define LIS3DH_H

/* Includes ------------------------------------------------------------------*/

#include "lis3dh_driver.h"
#include "hal_types.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
// Simple BLE Peripheral Task Events
#define LIS_START_DEVICE_EVT                              0x0001
#define LIS_PERIODIC_EVT                                  0x0002

#define LIS_X                   0x01
#define LIS_Y                   0x02
#define LIS_Z                   0x04
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the LIS Application
 */
extern void gSensorApp_Init( uint8 task_id );

/*
* Task Event Processor for the LIS Application
 */
extern uint16 gSensorApp_ProcessEvent( uint8 task_id, uint16 events );



extern void HalLis3dhInit(void);

/************************************************************************************************/


#endif
