//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 44, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT3x Sample Code (V1.0)
// File      :  system.h (V1.0)
// Author    :  RFU
// Date      :  16-Jun-2014
// Controller:  STM32F100RB
// IDE       :  µVision V4.71.2.0
// Compiler  :  Armcc
// Brief     :  System functions, global definitions
//==============================================================================

#ifndef SYSTEM_H
#define SYSTEM_H

//-- Includes ------------------------------------------------------------------
//#include "stm32f10x.h"             // controller register definitions
#include "typedefs.h"              // type definitions

//-- Enumerations --------------------------------------------------------------
// Error codes
typedef enum{
  NO_ERROR       = 0x00, // no error
  ACK_ERROR      = 0x01, // no acknowledgment error
  CHECKSUM_ERROR = 0x02, // checksum mismatch error
  TIMEOUT_ERROR  = 0x04, // timeout error
  PARM_ERROR     = 0x80, // parameter out of range error
}etError;

//==============================================================================
void SystemInit(void);
//==============================================================================
// Initializes the system
//------------------------------------------------------------------------------

//==============================================================================
void DelayMicroSeconds(u32t nbrOfUs);
//==============================================================================
// Wait function for small delays.
//------------------------------------------------------------------------------
// input:  nbrOfUs   wait x times approx. one micro second (fcpu = 8MHz)
// return: -
// remark: smallest delay is approx. 15us due to function call

#endif
