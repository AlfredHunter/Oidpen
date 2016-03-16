/***********************************************************************************
* INCLUDES
*/
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"
#include "hal_interrupt.h"




/***********************************************************************************
* LOCAL VARIABLES
*/
static ISR_FUNC_PTR port0_isr_tbl[8] = {0};
static ISR_FUNC_PTR port1_isr_tbl[8] = {0};
static ISR_FUNC_PTR port2_isr_tbl[5] = {0};


#define PICTL_P0ICON_BM        BV(0)
#define PICTL_P1ICONL_BM        BV(1)
#define PICTL_P1ICONH_BM        BV(2)
#define PICTL_P2ICON_BM         BV(3)
  


uint8 halDigioConfig(const digioConfig *p)
{
	register const uint8 bitmask  = p->pin_bm;

	    // Sanity check
    if ((bitmask == 0) || (bitmask != (uint8)BV(p->pin)))
    {
        return(HAL_DIGIO_ERROR);
    }

	switch(p->port)
    {
    case 0: P0SEL &= ~bitmask; 
            if (p->dir == HAL_DIGIO_OUTPUT)
            {
                if (p->initval == 1)
                {
                  P0 |= bitmask;  
                }
                else
                {
                  P0 &= ~bitmask;
                }
                P0DIR |= bitmask;
             }
             else // input
             {
                P0DIR &= ~bitmask;
             }
            break;
     case 1: P1SEL &= ~bitmask; 
            if (p->dir == HAL_DIGIO_OUTPUT)
            {
                if (p->initval == 1)
                {
                  P1 |= bitmask;  
                }
                else
                {
                  P1 &= ~bitmask;
                }
                P1DIR |= bitmask;
             }
             else // input
             {
                P1DIR &= ~bitmask;
             }
            break;
     case 2: P2SEL &= ~bitmask; 
            if (p->dir == HAL_DIGIO_OUTPUT)
            {
                if (p->initval == 1)
                {
                  P2 |= bitmask;  
                }
                else
                {
                  P2 &= ~bitmask;
                }
                P2DIR |= bitmask;
             }
             else // input
             {
                P2DIR &= ~bitmask;
             }
            break;
    //case 1: P1SEL &= ~bitmask; out = &P1OUT; dir = &P1DIR; break;
    //case 2: P2SEL &= ~bitmask; out = &P2OUT; dir = &P2DIR; break;
    default: return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}



/***********************************************************************************
* @fn      halDigioSet
*
* @brief   Set output pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioSet(const digioConfig* p)
{
   if (p->dir == HAL_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
        case 0: P0 |= p->pin_bm; break;
        case 1: P1 |= p->pin_bm; break;
        case 2: P2 |= p->pin_bm; break;
        default: return(HAL_DIGIO_ERROR);
        }
        return(HAL_DIGIO_OK);
    }
    return(HAL_DIGIO_ERROR);
}


/***********************************************************************************
* @fn      halDigioClear
*
* @brief   Clear output pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioClear(const digioConfig* p)
{
   if (p->dir == HAL_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
        case 0: P0 &= ~p->pin_bm; break;
        case 1: P1 &= ~p->pin_bm; break;
        case 2: P2 &= ~p->pin_bm; break;
        default: return(HAL_DIGIO_ERROR);
        }
        return(HAL_DIGIO_OK);
    }
    return(HAL_DIGIO_ERROR);
}


/***********************************************************************************
* @fn      halDigioToggle
*
* @brief   Toggle output pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioToggle(const digioConfig* p)
{
  if (p->dir == HAL_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
        case 0: P0 ^= p->pin_bm; break;
        case 1: P1 ^= p->pin_bm; break;
        case 2: P2 ^= p->pin_bm; break;
        default: return(HAL_DIGIO_ERROR);
        }
        return(HAL_DIGIO_OK);
    }
    return(HAL_DIGIO_ERROR);
}


/***********************************************************************************
* @fn      halDigioGet
*
* @brief   Get value on input pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioGet(const digioConfig* p)
{
    if (p->dir == HAL_DIGIO_INPUT)
    {
        switch (p->port)
        {
        case 0: return (P0 & p->pin_bm ? 1 : 0);
        case 1: return (P1 & p->pin_bm ? 1 : 0);
        case 2: return (P2 & p->pin_bm ? 1 : 0);
        default: break;
        }
    }
    return(HAL_DIGIO_ERROR);
}


/***********************************************************************************
* @fn      halDigioIntConnect
*
* @brief   Connect function to IO interrupt
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*          ISR_FUNC_PTR func - pointer to function
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioIntConnect(const digioConfig *p, ISR_FUNC_PTR func)
{
  istate_t key;
    HAL_INT_LOCK(key);
    switch (p->port)
    {
    case 0: port0_isr_tbl[p->pin] = func; break;
    case 1: port1_isr_tbl[p->pin] = func; break;
    case 2: port2_isr_tbl[p->pin] = func; break;
    default: HAL_INT_UNLOCK(key); return(HAL_DIGIO_ERROR);
    }
    halDigioIntClear(p);
    HAL_INT_UNLOCK(key);
    return(HAL_DIGIO_OK);
}


/***********************************************************************************
* @fn      halDigioIntEnable
*
* @brief   Enable interrupt on IO pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioIntEnable(const digioConfig *p)
{
     switch (p->port)
    {

    case 0: 
      IEN1 |=  0x20;    // set P0IE bit
      P0IEN |= p->pin_bm;
      break;
    case 1:
      IEN2 |= 0x10;    // set P1IE bit
      P1IEN |= p->pin_bm;
      break;
    case 2:
      IEN2 |= 0x02;     // set P2IE bit
      P2IEN |= p->pin_bm; 
      break;
    default: 
      return(HAL_DIGIO_ERROR);
    }

    return(HAL_DIGIO_OK);
}


/***********************************************************************************
* @fn      halDigioIntDisable
*
* @brief   Disable interrupt on IO pin
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioIntDisable(const digioConfig *p)
{
   switch (p->port)
    {

    case 0: 
      P0IEN &= ~p->pin_bm;
      break;
    case 1:
      P1IEN &= ~p->pin_bm;
      break;
    case 2:
      P2IEN &= ~p->pin_bm;
      break;
    default: 
      return(HAL_DIGIO_ERROR);
    }

    return(HAL_DIGIO_OK);
}


/***********************************************************************************
* @fn      halDigioIntClear
*
* @brief   Clear interrupt flag
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/
uint8 halDigioIntClear(const digioConfig *p)
{
    switch (p->port)
    {
    case 0: P0IFG &= ~p->pin_bm; break;
    case 1: P1IFG &= ~p->pin_bm; break;
    case 2: P2IFG &= ~p->pin_bm; break;
    default: return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}


/***********************************************************************************
* @fn      halDigioIntSetEdge
*
* @brief   Set edge for IO interrupt
*
* @param   digioConfig* p - pointer to configuration structure for IO pin
*          edge - HAL_DIGIO_INT_FALLING_EDGE or HAL_DIGIO_INT_RISING_EDGE
*
* @return  uint8 - HAL_DIGIO_ERROR or HAL_DIGIO_OK
*/

// Comment: all pins on port are configured at same time
uint8 halDigioIntSetEdge(const digioConfig *p, uint8 edge)
{
  switch(edge)
    {
    case HAL_DIGIO_INT_FALLING_EDGE:
        switch(p->port)
        {
        case 0: PICTL |= PICTL_P0ICON_BM; // set P0ICON high
                break;
        case 1: 
			if(p->pin < 4) {
				PICTL |= PICTL_P1ICONL_BM; // set P1ICON high
            } else {
            	PICTL |= PICTL_P1ICONH_BM; // set P1ICON high
            }
            break;
        case 2: PICTL |= PICTL_P2ICON_BM; // set P2ICON high
                break;
        default: return(HAL_DIGIO_ERROR);
        }
        break;

    case HAL_DIGIO_INT_RISING_EDGE:
        switch(p->port)
        {
        case 0: PICTL &= ~PICTL_P0ICON_BM; // set P0ICON low
                break;
        case 1:
			if(p->pin < 4) {
				PICTL &= ~PICTL_P1ICONL_BM; // set P0ICON low
            }  else {
				PICTL &= ~ PICTL_P1ICONH_BM;
			}
			break;
        case 2: PICTL &= ~PICTL_P2ICON_BM; // set P0ICON low
                break;
        default: return(HAL_DIGIO_ERROR);
        }
        break;

    default:
        return(HAL_DIGIO_ERROR);
    }
    return(HAL_DIGIO_OK);
}


/***********************************************************************************
* @fn      port0_ISR
*
* @brief   ISR framework for P0 digio interrupt
*
* @param   none
*
* @return  none
*/
HAL_ISR_FUNCTION(port0_ISR,P0INT_VECTOR)
{
    register uint8 i;
    P0IF = 0;
     HAL_ENTER_ISR();
    if (P0IFG)
    {
        for (i = 0; i < 8; i++)
        {
            register const uint8 pinmask = 1 << i;
            if (P0IFG & pinmask) {
                if (port0_isr_tbl[i] != 0) {
                (*port0_isr_tbl[i])();
                }
                P0IFG &= ~pinmask;
            }
        }
        //__low_power_mode_off_on_exit();
    }
     HAL_EXIT_ISR();
}

/***********************************************************************************
* @fn      port1_ISR
*
* @brief   ISR framework for P0 digio interrupt
*
* @param   none
*
* @return  none
*/
HAL_ISR_FUNCTION(port1_ISR,P1INT_VECTOR)
{
    register uint8 i;
    P1IF = 0;
     HAL_ENTER_ISR();
	 //handle SPI interrupt asap
	 if(P1IFG & 0x10) {
	 	port1_isr_tbl[4]();
		P1IFG &= ~BV(4);
	 }
	 
    if (P1IFG)
    {
        for (i = 0; i < 8; i++)
        {
            register const uint8 pinmask = 1 << i;
            if (P1IFG & pinmask) {
              
                if (port1_isr_tbl[i] != 0) {
                    (*port1_isr_tbl[i])();
                }
                P1IFG &= ~pinmask;
            }
        }
        //__low_power_mode_off_on_exit();
    }
     HAL_EXIT_ISR();
}

/***********************************************************************************
* @fn      port2_ISR
*
* @brief   ISR framework for P2 digio interrupt
*
* @param   none
*
* @return  none
*/

HAL_ISR_FUNCTION(port2_ISR,P2INT_VECTOR)
{
    register uint8 i;
    P2IF = 0;
    if (P2IFG)
    {
        for (i = 0; i < 5; i++)
        {
            register const uint8 pinmask = 1 << i;
            if (P2IFG & pinmask) {
                if (port2_isr_tbl[i] != 0) {
                    (*port2_isr_tbl[i])();
                }
                P2IFG &= ~pinmask;
            }
        }
        //__low_power_mode_off_on_exit();
    }
}

#if 0
HAL_ISR_FUNCTION( halI2CIsr, I2C_VECTOR )
{
  HAL_ENTER_ISR();

  /*
    Clear the CPU interrupt flag for Port_2
    PxIFG has to be cleared before PxIF
    Notes: P2_1 and P2_2 are debug lines.
  */
  P2IFG = 0;
  P2IF    = 0;

  HAL_EXIT_ISR();
}

#endif


