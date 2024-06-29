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


//! \file   /solutions/universal_motorcontrol_lab/common/include/communication.h
//!
//! \brief  header file to be included in all labs
//!         support for universal motor control lab with F28002x/F28003x/F280013x
//!


#ifndef COMMUNICATION_H
#define COMMUNICATION_H


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
//! \defgroup COMMUNICATION COMMUNICATION
//! @{
//
//*****************************************************************************

// Included Files
#include "driverlib.h"
#include "device.h"

// platforms
#include "hal.h"

//==============================================================================

// the defines

// CAN
#define MSG_DATA_LENGTH             8
#define TX_MSG_OBJ_ID               1
#define RX_MSG_OBJ_ID               2

#define CAN_IF2                     2

//! \brief Defines the CANCOM object
//!
typedef struct _CANCOM_Obj_
{
    float32_t   speedConv_sf;
    float32_t   speedInv_sf;
    float32_t   currentConv_sf;
    float32_t   currentInv_sf;

    float32_t   speedRef_Hz;
    float32_t   speedSet_Hz;
    float32_t   speedRx_Hz;
    float32_t   IqRx_A;

    int16_t     txMsgData[8];
    int16_t     rxMsgData[8];

    uint16_t    txMsgCount;       // for debug
    uint16_t    rxMsgCount;       // for debug
    uint16_t    errorFlag;        // for debug

    uint16_t    waitTimeCnt;
    uint16_t    waitTimeDelay;

    MOTOR_Status_e motorStateRx;

    bool    flagRxDone;
    bool    flagTxDone;

    bool    flagCmdTxRun;
    bool    flagCmdRxRun;
}CANCOM_Obj;

//! \brief Defines the CANCOM handle
//!
typedef struct _CANCOM_Obj_ *CANCOM_Handle;

extern volatile CANCOM_Obj canComVars;

extern HAL_Handle   halHandle;

//! \brief     The ISR
//! \param[in] N/A
extern __interrupt void canaISR(void);

//! \brief      Initializes CAN
//! \param[in]  N/A
extern void initCANCOM(HAL_Handle handle);

//! \brief     Sets up the CANA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCAN(HAL_Handle handle);

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

#endif // end of COMMUNICATION_H defines
