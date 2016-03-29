/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : example_main.c
* Author             : MSH Application Team
* Author             : Fabio Tota
* Revision           : $Revision: 1.5 $
* Date               : $Date: 16/06/2011 12:19:08 $
* Description        : Example main file for MKI109V1 board
* HISTORY:
* Date        | Modification                                | Author
* 16/06/2011  | Initial Revision                            | Fabio Tota

********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "lis3dh.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_oid.h"
#include "hal_interrupt.h"
#include "hal_sleep.h"
#include "hal_drivers.h"
#include "lis3dh_driver.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// How often to perform periodic event
#define LIS_PERIODIC_EVT_PERIOD                   100
#define LIS_POSITION_NOT_CHANGE_COUNT             (5000/LIS_PERIODIC_EVT_PERIOD)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8 response;
AccAxesRaw_t Accdata;
static AccAxesRaw_t oldAccdata;
uint8 Test_response_Who;

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 gSensorApp_TaskID;   // Task ID for internal task/event processing
volatile static uint8 timer_count = 0;
static uint16 old_xData = 0;
/* Extern variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void init_gsensor_interrupt( void );
static void Hal_Sensor_ISR(void);
/* Private functions ---------------------------------------------------------*/

//define for example1 or example2
//#define __EXAMPLE1__H 
//#define __EXAMPLE2__H 

/*******************************************************************************
* Function Name  : HalLis3dhInit.
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/


uint8 position=0, old_position=0;

void HalLis3dhInit(void)
{
  //Initialize your hardware here
  HalLis3dhSelect();
  initAcc();
#if 0
  do{
  response = GetWHO_AM_I(&Test_response_Who);
  } while(response == MEMS_ERROR );

  do{
  response = SetODR(ODR_10Hz);
  } while(response == MEMS_ERROR );
  //set PowerMode 
  do{
  response = SetMode(NORMAL);
  } while(response == MEMS_ERROR );  
  //set Fullscale
  do{
  response = SetFullScale(FULLSCALE_2);
  } while(response == MEMS_ERROR );
  //set axis Enable
  do{
  response = SetAxis(X_ENABLE | Y_ENABLE | Z_ENABLE);
  } while(response == MEMS_ERROR );
  do{
  response = SetInt1Pin(I1_INT1_ON_PIN_INT1_ENABLE);
  } while(response == MEMS_ERROR );  
  do{
  response = Int1LatchEnable(MEMS_DISABLE);
  } while(response == MEMS_ERROR );
  //set Interrupt Threshold 
  do{
  response = SetInt1Threshold(8);
  } while(response == MEMS_ERROR );
  do{
  response = SetInt1Duration(50);
  } while(response == MEMS_ERROR );
     //set Interrupt configuration (all enabled)
  do{
  response = SetIntConfiguration(INT1_ZLIE_ENABLE |
                                 INT1_YLIE_ENABLE |  INT1_XLIE_ENABLE ); 
  } while(response == MEMS_ERROR );  
#endif
/***********************************************************************************************/  
/******Example 1******/ 
#ifdef __EXAMPLE1__H 
  while(1)
  {
    //get Acceleration Raw data  
    response = GetAccAxesRaw(&Accdata);
    if(response==1)
	{ 
      old_position = position;
    }
  }
#endif /* __EXAMPLE1__H  */ 
 
/**********************************************************************************************/
/******Example 2******/
#ifdef __EXAMPLE2__H
   //configure Mems Sensor
   //set Interrupt Threshold 
   response = SetInt1Threshold(20);
   if(response==1)
   {
   }
   //set Interrupt configuration (all enabled)
   response = SetIntConfiguration(INT1_ZHIE_ENABLE | INT1_ZLIE_ENABLE |
                                  INT1_YHIE_ENABLE | INT1_YLIE_ENABLE |
                                  INT1_XHIE_ENABLE | INT1_XLIE_ENABLE ); 
  if(response==1)
  {
  }
  //set Interrupt Mode
  response = SetIntMode(INT_MODE_6D_POSITION);
  if(response==1)
  {
  }

  while(1)
  {
    //get 6D Position
    response = Get6DPosition(&position);
    if((response==1) && (old_position!=position))
    {
      switch (position)
      {
        case UP_SX:   sprintf((char*)buffer,"\n\rposition = UP_SX  \n\r\0");   break;
        case UP_DX:   sprintf((char*)buffer,"\n\rposition = UP_DX  \n\r\0");   break;
        case DW_SX:   sprintf((char*)buffer,"\n\rposition = DW_SX  \n\r\0");   break;              
        case DW_DX:   sprintf((char*)buffer,"\n\rposition = DW_DX  \n\r\0");   break; 
        case TOP:     sprintf((char*)buffer,"\n\rposition = TOP    \n\r\0");   break; 
        case BOTTOM:  sprintf((char*)buffer,"\n\rposition = BOTTOM \n\r\0");   break; 
        default:      sprintf((char*)buffer,"\n\rposition = unknown\n\r\0");   break;
      }
      //function for MKI109V1 board    
      old_position = position;
    }
  }
#endif /*__EXAMPLE2__H */ 
  init_gsensor_interrupt();
/************************************************************************************************/
} // end main

void resetLis3dTimerCount(void)
{
  timer_count = 0;
}

static void performPeriodicTask( void )
{
    
#if 0   
    //get 6D Position
    response = Get6DPosition(&position);
    if((response==1) && (old_position!=position))
    {
      switch (position)
      {
        case UP_SX:            
          if(SUCCESS == OnBoard_Send_gSensors(LIS_X, 0x0001));
          break;
        case UP_DX:    
          if(SUCCESS == OnBoard_Send_gSensors(LIS_X, 0x0002));
          break;
        case DW_SX:    
          if(SUCCESS == OnBoard_Send_gSensors(LIS_Y, 0x0001));
          break;              
        case DW_DX:    
          if(SUCCESS == OnBoard_Send_gSensors(LIS_Y, 0x0002));
          break; 
        case TOP:      
          if(SUCCESS == OnBoard_Send_gSensors(LIS_Z, 0x0001));
          break; 
        case BOTTOM:   
          if(SUCCESS == OnBoard_Send_gSensors(LIS_Z, 0x0002));
          break; 
        default:       break;
      }

      //function for MKI109V1 board    
      old_position = position;
      HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK); 
    }
#endif
    timer_count ++ ;
    if(timer_count > LIS_POSITION_NOT_CHANGE_COUNT) //5s
    {
      timer_count = 0;
      if(OID_POWER_ON == getOidState())
      {
        halOidPower(0);
        HalLedSet(HAL_LED_R, HAL_LED_MODE_BLINK); 
#if defined HAL_SLEEP
        osal_start_timerEx( Hal_TaskID, HAL_SLEEP_EVENT, 2000 );
//        SysPowerMode(3);
#endif
      }
    }
#if 1
    //get Acceleration Raw data  
    response = GetAccAxesRaw(&Accdata);
    
    if( response==1 )
    { 
      osal_memcpy(&oldAccdata, &Accdata, sizeof(AccAxesRaw_t));
//      old_position = position;
      
      if( (( Accdata.AXIS_X & 0xff00 ) != old_xData) && \
          (( Accdata.AXIS_X & 0xff00 ) != 0x0000) && \
          (( Accdata.AXIS_X & 0xff00 ) != 0xFF00) )
      {
        if( OID_POWER_OFF == getOidState())
        {
          halOidPower(1);
          HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK); 
        }
        timer_count = 0;
      }
      old_xData = ( Accdata.AXIS_X & 0xff00 );
      
      if(SUCCESS == OnBoard_Send_gSensors(LIS_X, SWAP_UINT16(Accdata.AXIS_X)));
//      if(SUCCESS == OnBoard_Send_gSensors(LIS_Y, SWAP_UINT16(Accdata.AXIS_Y)));
//      if(SUCCESS == OnBoard_Send_gSensors(LIS_Z, SWAP_UINT16(Accdata.AXIS_Z)));     
    }
#endif
}

void gSensorApp_Init( uint8 task_id )
{
    gSensorApp_TaskID = task_id;
    
    osal_set_event( gSensorApp_TaskID, LIS_START_DEVICE_EVT );
}

uint16 gSensorApp_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & LIS_START_DEVICE_EVT )
  {
    // Start the Device
    HalLis3dhInit();
    // Set timer for first periodic event
    osal_start_timerEx( gSensorApp_TaskID, LIS_PERIODIC_EVT, LIS_PERIODIC_EVT_PERIOD );
    return ( events ^ LIS_START_DEVICE_EVT );
  }
  
  if ( events & LIS_PERIODIC_EVT )
  {
    // Restart timer
    if ( LIS_PERIODIC_EVT )
    {
      osal_start_timerEx( gSensorApp_TaskID, LIS_PERIODIC_EVT, LIS_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ LIS_PERIODIC_EVT);
  }
    // Discard unknown events
  return 0;
}

static const digioConfig pinKey[2] = {
  { HAL_SENSOR_INT1_PORT,
    HAL_SENSOR_INT1_PIN,
    BV( HAL_SENSOR_INT1_PIN ),
    HAL_DIGIO_INPUT, 0 },
  { HAL_SENSOR_INT2_PORT,
    HAL_SENSOR_INT2_PIN,
    BV( HAL_SENSOR_INT2_PIN ),
    HAL_DIGIO_INPUT, 0 } 
};

static void init_gsensor_interrupt( void )
{
  halDigioConfig( &pinKey[0] );
  halDigioIntSetEdge( &pinKey[0], HAL_DIGIO_INT_FALLING_EDGE );
  halDigioIntConnect( &pinKey[0], &Hal_Sensor_ISR );
  halDigioIntEnable(&pinKey[0]);
}


static void Hal_Sensor_ISR(void)
{
//  CLEAR_SLEEP_MODE();
#if defined HAL_SLEEP
  SysPowerMode(4);
#endif
  HalLedSet(HAL_LED_B, HAL_LED_MODE_BLINK);
}
