//#############################################################################
// $Copyright:
// Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//! \file  solutions/universal_motorcontrol_lab/common/include/sys_main.h
//! \brief  header file to be included in all labs
//!

#ifndef SYS_MAIN_H
#define SYS_MAIN_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup SYS MAIN
//! @{
//
//*****************************************************************************

// modules
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#if defined(DAC128S_ENABLE)
#include "dac128s085.h"
#endif  // DAC128S_ENABLE


#include "motor_common.h"
#include "motor1_drive.h"


#include "datalogIF.h"
#include "cpu_time.h"

#define LED_BLINK_FREQ_Hz           (0.5f)       // 1Hz
#define POWER_RELAY_WAIT_TIME_ms    (1000)       // 1s

//------------------------------------------------------------------------
typedef struct _SYSTEM_Vars_t_
{
    float32_t speedRef_Hz;

    uint32_t mainLoopCnt;
    uint32_t timerCnt_1min;

    uint16_t timerBase_1ms;
    uint16_t timerCnt_5ms;
    uint16_t timerCnt_1s;

    uint16_t powerRelayWaitTime_ms;

    uint16_t counterLEDC;       //!< Counter used to divide down a timer base for
                                //!< visually blinking an LED

    uint16_t counterLEDB;       //!< Counter used to divide down a timer base for
                                //!< visually blinking an LED

    uint16_t timeWaitLEDB;      //

    uint16_t            estLibVersion;
    Board_Kit_e         boardKit;
    Project_Config_e    projectConfig;
    EST_Type_e          estType;
    CURRENTSEN_Type_e   currentSenseType;

    bool flagEnableSystem;
}SYSTEM_Vars_t;

extern volatile SYSTEM_Vars_t systemVars;



#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
extern HAL_PWMDACData_t pwmDACData;
  // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

#if defined(DAC128S_ENABLE)
extern DAC128S_Handle   dac128sHandle;
extern DAC128S_Obj      dac128s;
#endif  // DAC128S_ENABLE


#ifdef CPUTIME_ENABLE
extern CPU_TIME_Obj     cpuTime;
extern CPU_TIME_Handle  cpuTimeHandle;
#endif  // CPUTIME_ENABLE

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of SYS_MAIN_H definition
