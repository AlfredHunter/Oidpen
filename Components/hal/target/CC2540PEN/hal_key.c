/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2013-09-20 11:53:10 -0700 (Fri, 20 Sep 2013) $
  Revision:       $Revision: 35401 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "hal_interrupt.h"

#ifdef BRIAN_HW_TEST
#include "hal_led.h"
#endif

#include "hal_oid.h"

#include "osal.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1



/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_1_IF P1IF
#define HAL_KEY_CPU_PORT_2_IF P2IF


#define HAL_KEY_SW_1_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT  BV(1) /* P0IEN - P0.0 enable/disable bit */
#define HAL_KEY_SW_1_IENBIT   BV(4) /* Mask bit for all of Port_0 */

#define HAL_KEY_SW_1_PXIFG    P1IFG /* Interrupt flag at source */

#define HAL_KEY_SW_1_EDGEBIT  BV(0)



/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

typedef void (*EVT_FUNC_PTR)(void);

/**************************************************************************************************
 *                                        LOCAL VARIABLES
 **************************************************************************************************/

static const digioConfig pinKey = {HAL_SENSOR_INT1_PORT,
                                   HAL_SENSOR_INT1_PIN,
                                   BV(HAL_SENSOR_INT1_PIN),
                                   HAL_DIGIO_INPUT, 0};
static void HalKeyISR(void);
static EVT_FUNC_PTR key_event_tbl[HAL_KEY_EVENT_MAX] = {0};

//static uint8 keyValue = MCU_IO_TRISTATE;
//static int counter = 0;

//static uint8 keyStatus = HAL_KEY_EVENT_INVALID;

static uint8 chargeStatus = HAL_CHARGE_EVENT_INVALID;
/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);
uint8 HalKeyIntEnable();
uint8 HalKeyIntConnect(uint8 event, EVT_FUNC_PTR func);
void longPressHandle(void);
void shortPressHandle(void);

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
	//HAL_CONFIG_IO_INPUT(HAL_KEY_PORT, HAL_KEY_PIN, MCU_IO_TRISTATE);
        HAL_CONFIG_IO_INPUT(HAL_USB_DETECT_PORT,HAL_MIC_INSERT_PIN,MCU_IO_PULLDOWN);
        HAL_CONFIG_IO_INPUT(HAL_SENSOR_INT1_PORT, HAL_SENSOR_INT1_PIN, MCU_IO_TRISTATE);
        HAL_CONFIG_IO_INPUT(HAL_SENSOR_INT2_PORT, HAL_SENSOR_INT2_PIN, MCU_IO_TRISTATE);
        
	halDigioConfig(&pinKey);
	halDigioIntSetEdge(&pinKey, HAL_DIGIO_INT_RISING_EDGE);
	halDigioIntConnect(&pinKey,&HalKeyISR);
//        HalKeyIntConnect(HAL_KEY_EVENT_LONG, &longPressHandle);
//        HalKeyIntConnect(HAL_KEY_EVENT_SHORT, &shortPressHandle);
	HalKeyIntEnable();
        HalKeyConfigured = FALSE;
}
#if 0
void longPressHandle(void)
{
  HAL_SHAKER_ON();
  HalLedSet(HAL_LED_R, HAL_LED_MODE_ON);
}

void shortPressHandle(void)
{
#ifdef BRIAN_HW_TEST
   static uint8 led_test = 0;
   led_test ++;

   if( led_test == 1 )
   {
     HalLedSet(HAL_LED_B, HAL_LED_MODE_BLINK);
	 HAL_SHAKER_OFF();
   }
   else if( led_test == 2 )
   {
     HalLedSet(HAL_LED_R, HAL_LED_MODE_BLINK);
   }
   else if( led_test == 3 )
   {
     HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK);
   }
   else
   {
     led_test = 0;
	 HAL_SHAKER_ON();
   }
#endif
}
#endif

/**************************************************************************************************
 * @fn      HalKeyIntConnect
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
uint8 HalKeyIntConnect(uint8 event, EVT_FUNC_PTR func)
{
	istate_t key;
	HAL_INT_LOCK(key);

	switch(event) 
	{
	    case HAL_KEY_EVENT_DOWN:
		key_event_tbl[HAL_KEY_EVENT_DOWN] = func;
		break;
	    case HAL_KEY_EVENT_UP:
		key_event_tbl[HAL_KEY_EVENT_UP] = func;
		break;
            case HAL_KEY_EVENT_SHORT:
		key_event_tbl[HAL_KEY_EVENT_SHORT] = func;
                break;
	    case HAL_KEY_EVENT_LONG:
		key_event_tbl[HAL_KEY_EVENT_LONG] = func;
                break;
	    default:
		HAL_INT_UNLOCK(key); return FAILURE;
	}
	HAL_INT_UNLOCK(key);
	return SUCCESS;
}
/**************************************************************************************************
 * @fn      HalKeyIntEnable
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
uint8 HalKeyIntEnable()
{
	halDigioIntEnable(&pinKey);
	return SUCCESS;
}
/**************************************************************************************************
 * @fn      HalKeyIntDisable
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyIntDisable(void)
{
	halDigioIntDisable(&pinKey);
}
/**************************************************************************************************
 * @fn      HalKeyIntClear
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyIntClear(void)
{
	halDigioIntClear(&pinKey);
}
/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
uint8 HalKeyRead() 
{
  uint8 keys = 0;

  if (!(HAL_KEY_SW_1_PORT & HAL_KEY_SW_1_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_1;
  }
  if ((HAL_SENSOR_SW_IN1_PORT & HAL_SENSOR_SW_IN1_BIT))    /* Key is active low */
  {
    keys |= HAL_SENSOR_SW_IN1;
  }
  if ((HAL_SENSOR_SW_IN2_PORT & HAL_SENSOR_SW_IN2_BIT))    /* Key is active low */
  {
    keys |= HAL_SENSOR_SW_IN2;
  }
  if ((HAL_CHG_STATUS_SW_PORT & HAL_CHG_STATUS_SW_BIT))    /* Key is active LOW */
  {
    keys |= HAL_CHG_STATUS_SW;
  }
  if (!(HAL_USB_DETECT_SW_PORT & HAL_USB_DETECT_SW_BIT))    /* Key is active low */
  {
    keys |= HAL_USB_DETECT_SW;
  }
  return keys;
//	return HAL_IO_GET(HAL_KEY_PORT, HAL_KEY_PIN);
}
/**************************************************************************************************
 * @fn      HalKeyISR
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
static void HalKeyISR(void) 
{
  HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK);
   #if  0 //def BRIAN_HW_TEST
   static uint8 led_test = 0;
   led_test ++;

   if( led_test == 1 )
   {
     HalLedSet(HAL_LED_B, HAL_LED_MODE_BLINK);
	 HAL_SHAKER_OFF();
   }
   else if( led_test == 2 )
   {
     HalLedSet(HAL_LED_R, HAL_LED_MODE_BLINK);
   }
   else if( led_test == 3 )
   {
     HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK);
   }
   else
   {
     led_test = 0;
	 HAL_SHAKER_ON();
   }
   #endif
   
//   osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);
//   osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
}
/**************************************************************************************************
 * @fn      processKey
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void processKey(void)
{
  uint8 keys = 0;
//  uint8 notify = 0;
//  uint8 event_id = HAL_KEY_EVENT_INVALID;
  
  keys = HalKeyRead();
#if 1

  /* If interrupts are not enabled, previous key status and current key status
  * are compared to find out if a key has changed status.
  */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      /* Exit - since no keys have changed */
      return;
    }
    else
    {
//      notify = 1;
    }
  }
  else
  {
    /* Key interrupt handled here */
    if (keys)
    {
//      notify = 1;
    }
  }
#endif
#if 0
  counter++;
  //long push
  if( (keys & HAL_KEY_SW_1) && (counter > HAL_KEY_LONG_PUSH) ) 
  {
    counter = 0;
    event_id = HAL_KEY_EVENT_LONG;
    keyStatus = HAL_KEY_EVENT_LONG;
  }
  //short push
  if( (keys & HAL_KEY_SW_1) && counter >= 2 )
  {
    event_id = HAL_KEY_EVENT_DOWN;
  }
  
  if( !(keys & HAL_KEY_SW_1) && (counter >=2 && counter < HAL_KEY_LONG_PUSH))
  {
    counter = 0;
    event_id = HAL_KEY_EVENT_SHORT;
  }
  
  if( !(keys & HAL_KEY_SW_1) && (event_id == HAL_KEY_EVENT_INVALID) )
  {
    counter = 0;
    if( keyStatus == HAL_KEY_EVENT_LONG )
    {
      HalLedSet(HAL_LED_R, HAL_LED_MODE_OFF);
    }
    event_id = HAL_KEY_EVENT_UP;
  }
  if( event_id != HAL_KEY_EVENT_INVALID )
  {
    if(key_event_tbl[event_id]) key_event_tbl[event_id]();
  }
#endif  
 
  if( keys & HAL_USB_DETECT_SW ) // exit power on
  {
    if( HAL_CHARGE_EVENT_FINISHED == chargeStatus)
    {
      HalLedSet(HAL_LED_G, HAL_LED_MODE_ON);
    }
    else
    {
      chargeStatus = HAL_CHARGE_EVENT_CHARGING;
      if( HAL_CHARGE_EVENT_CHARGING == chargeStatus )
      {
        HalLedSet(HAL_LED_R, HAL_LED_MODE_ON);
      }
    }
  }
  else if( !(keys & HAL_USB_DETECT_SW )) // exit power off
  {
    chargeStatus = HAL_CHARGE_EVENT_OFF;
    if( HAL_CHARGE_EVENT_OFF == chargeStatus )
    {
      HalLedSet(HAL_LED_R, HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_G, HAL_LED_MODE_OFF);
    }
  }
  if( keys & HAL_CHG_STATUS_SW ) // charge finished
  {
/*    chargeStatus = HAL_CHARGE_EVENT_FINISHED;
    if( HAL_CHARGE_EVENT_FINISHED == chargeStatus )
    {
      HalLedSet(HAL_LED_R, HAL_LED_MODE_OFF);
      HalLedSet(HAL_LED_G, HAL_LED_MODE_ON);
    }*/
  }
  /* Store the current keys for comparation next time */
  halKeySavedKeys = keys;
}
/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback)
{
    /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;
  /* Register the callback fucntion */
  //pHalKeyProcessFunction = cback;
  /* Determine if interrupt is enable or not */
  if (Hal_KeyIntEnable)
  {
    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else
  {
   osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }
  /* Key now is configured */
  HalKeyConfigured = TRUE;
#if (defined HAL_WD) && (HAL_WD == TRUE)
    osal_set_event(Hal_TaskID, HAL_WD_EVENT);
#endif
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}






#else

void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}

#endif
/**************************************************************************************************
**************************************************************************************************/
