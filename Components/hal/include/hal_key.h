/**************************************************************************************************
  Filename:       hal_key.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the KEY Service.


  Copyright 2005-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

#define HAL_KEY_SW_1            0x01  // Key
#define HAL_SENSOR_SW_IN1       0x02  // Sensor Int1
#define HAL_SENSOR_SW_IN2       0x03  // Sensor Int2
#define HAL_CHG_STATUS_SW       0x04
#define HAL_USB_DETECT_SW       0x10
   
/* key is at P0.3 */
#define HAL_KEY_SW_1_PORT     P0
#define HAL_KEY_SW_1_BIT      BV(3)
#define HAL_KEY_SW_1_SEL      P0SEL
#define HAL_KEY_SW_1_DIR      P0DIR

/* Sensor IN1 is at P0.2 */
#define HAL_SENSOR_SW_IN1_PORT     P0
#define HAL_SENSOR_SW_IN1_BIT      BV(2)
#define HAL_SENSOR_SW_IN1_SEL      P0SEL
#define HAL_SENSOR_SW_IN1_DIR      P0DIR
  
/* Sensor IN2 is at P0.1 */
#define HAL_SENSOR_SW_IN2_PORT     P0
#define HAL_SENSOR_SW_IN2_BIT      BV(1)
#define HAL_SENSOR_SW_IN2_SEL      P0SEL
#define HAL_SENSOR_SW_IN2_DIR      P0DIR

/* CHG Status is at P2.0 */
#define HAL_CHG_STATUS_SW_PORT     P2
#define HAL_CHG_STATUS_SW_BIT      BV(0)
#define HAL_CHG_STATUS_SW_SEL      P2SEL
#define HAL_CHG_STATUS_SW_DIR      P2DIR
   
/* USB detect is at P0.0 */
#define HAL_USB_DETECT_SW_PORT     P0
#define HAL_USB_DETECT_SW_BIT      BV(0)
#define HAL_USB_DETECT_SW_SEL      P0SEL
#define HAL_USB_DETECT_SW_DIR      P0DIR
   
#define HAL_KEY_DEBOUNCE_VALUE  25
#define HAL_KEY_LONG_PUSH_VALUE  5000
#define HAL_KEY_LONG_PUSH (HAL_KEY_LONG_PUSH_VALUE/HAL_KEY_DEBOUNCE_VALUE)
   
enum {
	HAL_KEY_EVENT_DOWN,
	HAL_KEY_EVENT_UP,
        HAL_KEY_EVENT_SHORT,
	HAL_KEY_EVENT_LONG,
	HAL_KEY_EVENT_MAX,
	HAL_KEY_EVENT_INVALID
};

enum {
        HAL_CHARGE_EVENT_OFF,
        HAL_CHARGE_EVENT_CHARGING,
        HAL_CHARGE_EVENT_FINISHED,
        HAL_CHARGE_EVENT_INVALID
};
/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8 keys, uint8 state);

/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_KeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8 HalKeyRead( void);

/*
 * Enter sleep mode, store important values
 */
extern void HalKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern uint8 HalKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern	void processKey(void);


/*
 * This is for internal used by hal_sleep
 */
extern bool HalKeyPressed( void );

extern uint8 hal_key_keys(void);                                           

extern uint8 hal_key_int_keys(void);

/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
