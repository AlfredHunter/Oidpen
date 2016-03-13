/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "lis3dh.h"
#include "battservice.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define DEFAULT_BATT_CRITICAL_LEVEL  6 
// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED//GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
//static uint8 scanRspData[] =
uint8 scanRspData[] =
{
  // complete name
  0x0f,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x4d,   // 'M'
  0x65,   // 'e'
  0x6e,   // 'n'
  0x75,   // 'u'
  0x4f,   // 'O'
  0x72,   // 'r'
  0x64,   // 'd'
  0x65,   // 'e'
  0x72,   // 'r'
  0x50,   // 'P'
  0x65,   // 'e'
  0x6e,   // 'n'

  0x55,   // ''
  0x55,   // ''

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_gSensors( uint8 xyz, int16 value );
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
//static void simpleProfileChangeCB( uint8 paramID );
static void BattCB(uint8 event);

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif
static  void simpleBLEPeripheral_Oid( uint16 oid );

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

#if 0
// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
#endif
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;//Ê¹ÄÜÇëÇó¸üÐÂ
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;//×îÐ¡Á¬½Ó¼ä¸ô0.1ms    Í¨ÐÅ¼äÏ¶ Ã¿¸ö¼ä¸ô1.25msÎª»ù±¾µ¥Î»£¬×îÐ¡6µ¥Î»7.5mS£¬×î´ó3200µ¥Î»4.0S¡£
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;//×î´óÁ¬½Ó¼ä¸ô 1ms
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;//´Ó»úÑÓÊ± ±íÊ¾Peripheral¿ÉÒÔÁ¬ÐøºöÂÔµÄÁ¬½ÓÊý£¬×î´ó²»ÄÜ³¬¹ý499¸ö£¬×î³¤²»ÄÜ³¬¹ý32S¡£
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;//10s,¶¨Òå¸Ã²ÎÊý±íÊ¾ÔÚ¸ÃÊ±¼ä¶ÎÄÚ½¨Á¢µÄÁ¬½Ó²»³É¹¦ ¼à¹Ü³¬Ê± 2¸ö³É¹¦Á¬½ÓÊÂ¼þÖ®¼äµÄ×î´ó¼ä¸ô

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );//µ±Ö÷»úÉ¨Ãèµ½¹ã²¥ºó»á·¢³öÉ¨ÃèÇëÇó£¬´Ó»ú¾Í·¢»Ø¸ÃÊý¾Ýµ½Ö÷»ú
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );//¹ã²¥²ÎÊý

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );//GAP GATT·þÎñÆ÷²ÎÊýÉèÖÃ

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, 5 );//±£³Ö¹ã²¥Ê±¼ä unit for second£¬Ä¬ÈÏ180S
  }

  // Setup the GAP Bond Manager//GAP °ó¶¨¹ÜÀíÆ÷ÉèÖÃ
  {
    uint32 passkey = 0; // passkey "000000"// passkey "000000"£¬ÃÜÔ¿
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;//Åä¶ÔÄ£Ê½£¬ÅäÖÃ³ÉµÈ´ýÖ÷»úµÄÅä¶ÔÇëÇó
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values  ¶ÔÏà¹ØÊýÖµ×ö³õÊ¼»¯
  {

    uint16 charValue4 = 4;
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
  }

  RegisterForOids(simpleBLEPeripheral_TaskID);
  RegisterFor_gSensors(simpleBLEPeripheral_TaskID);

    //µç³Ø·þÎñ  
  {     
  
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;  
  
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8 ), &critical );/* ÉèÖÃÄ¬ÈÏÁÙ½çµçÁ¿ */  
  
  }  
  
  Batt_AddService();  /* Ìí¼Óµç³Ø·þÎñ */  
  
  Batt_Register(BattCB); /* ×¢²áµç³Ø·þÎñµÄÓ¦ÓÃ»Øµ÷º¯Êý */  

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD


  // Register callback with SimpleGATTprofile
  //VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  
  /*¼ì²éÊÇ·ñÓÐÏµÍ³ÏûÏ¢ÈÎÎñ,ÓÐÔò¶¨ÒåÒ»¸öÏûÏ¢Ö¸Õë*/
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
     /*¼ì²éÊÇ·ñ´ÓÏûÏ¢¶ÓÁÐÖÐÊÕµ½Êý¾Ý*/
    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      /*´¦ÀíÈÎÎñÐÅÏ¢*/
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
       /*ÊÍ·ÅÏûÏ¢µÄ»º´æ¿Õ¼ä*/
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    /*·µ»ØÎ´´¦ÀíµÄÈÎÎñ±êÖ¾*/
    return (events ^ SYS_EVENT_MSG);
  }

   /*¼ì²éÊÇ·ñÓÐÆô¶¯ÉèÎñÈÎÎñ*/
  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
     /*Æô¶¯Éè±¸,À¨ºÅÄÚÎª»Øµ÷º¯Êý,À´ÉèÖÃÒªÏÔÊ¾µÄÐÅÏ¢»ò²Ù×÷*/
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
     /*Æô¶¯°ó¶¨¹ÜÀíº¯Êý,´¦ÀíÈÏÖ¤ÐÅÏ¢ºÍ×¢²áÈÎÎñÐÅÏ¢*/
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    /*ÉèÖÃ¶¨Ê±Ê±¼ä,µ½Ê±ºóÖÜÆÚÊÂ¼þµÄÈÎÎñid±»ÖÃÆð*/
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD ); 
    /*·µ»ØÎ´´¦ÀíµÄÈÎÎñ±êÖ¾*/
    return ( events ^ SBP_START_DEVICE_EVT );
  }


  /*¼ì²éÊÇ·ñÓÐÖÜÆÚÈÎÎñÊÂ¼þ*/
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    /*Èç¹ûÓÐÖÜÆÚÈÎÎñÊÂ¼þ*/
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      /*ÉèÖÃ¶¨Ê±Ê±¼ä*/
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
 
    // Perform periodic application task
    /*´¦ÀíÖÜÆÚÊÂ¼þÖÐµÄ´¦Àí¹¤×÷*/
    performPeriodicTask();
    /*·µ»ØÎ´´¦ÀíµÄÈÎÎñ±êÖ¾*/
    return (events ^ SBP_PERIODIC_EVT);
  }
	



  // Discard unknown events
  return 0;
}

static void performPeriodicTask( void )  
{  
  if ( gapProfileState == GAPROLE_CONNECTED )  
  {  
    // perform battery level check  
    Batt_MeasLevel( );  
  }  
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case OID_CHANGE:
        simpleBLEPeripheral_Oid( ((oid_t *)pMsg)->oid );
    break;
    case GENSOR_CHANGE:
        simpleBLEPeripheral_gSensors( ((gSensor_t *)pMsg)->xyz, ((gSensor_t *)pMsg)->value );
    break;
    default:
    // do nothing
    break;
  }
}

static void simpleBLEPeripheral_Oid( uint16 oid )
{
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4,2, &oid);
}

static void simpleBLEPeripheral_gSensors( uint8 xyz, int16 value )
{
  uint8 AXIS = xyz;

  if ( AXIS & LIS_X )
  {
      AXIS &= ~LIS_X;
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1,2, &value);
  }
  if ( AXIS & LIS_Y )
  {
      AXIS &= ~LIS_Y;
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,2, &value);
  }
  if ( AXIS & LIS_Z )
  {
      AXIS &= ~LIS_Z;
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3,2, &value);
  }
}
/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_R, HAL_LED_MODE_BLINK);
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_B, HAL_LED_MODE_FLASH);
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_B, HAL_LED_MODE_OFF );
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
        HalLedSet(HAL_LED_G, HAL_LED_MODE_BLINK);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_B, HAL_LED_MODE_OFF );
          HalLedSet(HAL_LED_R, HAL_LED_MODE_5HZ_FLASH);
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_B, HAL_LED_MODE_OFF );
          HalLedSet(HAL_LED_R, HAL_LED_MODE_BLINK);
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLedSet(HAL_LED_R, HAL_LED_MODE_10HZ_FLASH);
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

static void BattCB(uint8 event)  
{  
  if (event == BATT_LEVEL_NOTI_ENABLED)  
  {  
    if (gapProfileState == GAPROLE_CONNECTED)  
    {         
       //HalLedSet(HAL_LED_G, HAL_LED_MODE_10HZ_FLASH);
    }   
  }  
  
  else if (event == BATT_LEVEL_NOTI_DISABLED)  
  {  
     //HalLedSet(HAL_LED_B, HAL_LED_MODE_10HZ_FLASH);
  }  
}

/*********************************************************************
*********************************************************************/
