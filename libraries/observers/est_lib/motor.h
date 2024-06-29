//##############################################################################
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
//##############################################################################

//------------------------------------------------------------------------------
//! MotorControl SDK
//!
//! \file   libraries/motor/include/motor.h for software library
//! \brief  Contains motor related definitions
//!
//------------------------------------------------------------------------------

#ifndef MOTOR_H
#define MOTOR_H

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
//! \defgroup MOTOR_LIB MOTOR_LIB
//! @{
//
//*****************************************************************************

// the includes

#include "types.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//
//! \brief Enumeration for the motor types
//!
typedef enum
{
  MOTOR_TYPE_INDUCTION = 0,   //!< induction
  MOTOR_TYPE_PM               //!< permanent magnet
} MOTOR_Type_e;

//
//! \brief Enumeration for the braking Mode
//
typedef enum
{
    FREE_STOP_MODE           = 0,       //!< Free stop mode without braking
    HARDSWITCH_BRAKE_MODE    = 1,       //!< Hard switch braking mode
    FORCESTOP_BRAKE_MODE     = 2,       //!< Force alignment braking mode
    DYNAMIC_BRAKE_MODE       = 3        //!< N/A, Dynamic braking mode
} Braking_Mode_e;

typedef Braking_Mode_e BRAKE_Mode_e;

//
//! \brief Enumeration for the control mode
//
typedef enum
{
    OPERATE_MODE_SPEED  = 0,        //!< Speed closed-loop running mode
    OPERATE_MODE_TORQUE = 1,        //!< Torque control running mode
    OPERATE_MODE_POWER  = 2,        //!< Constant power running mode
    OPERATE_MODE_VQCTRL = 3,        //!< Vq direction control
    OPERATE_MODE_SCALAR = 4         //!< V/f scalar control
} Operation_Mode_e;

typedef Operation_Mode_e OPERATE_Mode_e;

//! \brief Enumeration for the current sampling mode
//
typedef enum
{
    SAMPLE_MODE_DCSS2  = 0,         //!< dclink_ss2
    SAMPLE_MODE_DCSS4  = 1,         //!< dclink_ss4
    SAMPLE_MODE_DCLINK = 2,         //!< dclink_ss
    SAMPLE_MODE_3LSR   = 3,         //!< three_shunt
    SAMPLE_MODE_3INL   = 4,         //!< inline
    SAMPLE_MODE_SDFM   = 5          //!< sdfm
} SAMPLE_Mode_e;

//
//! \brief Enumeration for the Flying Start Mode
//
typedef enum
{
    FLYINGSTART_MODE_HALT    = 0,       //!< Halt Mode
    FLYINGSTART_MODE_STANDBY = 1        //!< Standby Mode
} FlyingStart_Mode_e;

//! \brief Enumeration for the current sampling mode
//
typedef enum
{
    LSC_TYPE_THREE_SHUNT  = 0,         //!< three_shunt
    LSC_TYPE_TWO_SHUNT    = 1,         //!< two_shunt
    LSC_TYPE_SINGLE_SHUNT = 2,         //!< dclink_ss
    LSC_TYPE_INLINE_SHUNT = 3          //!< inline 3/2 shunt
} CurrentShunt_Type_e;

//! \brief Enumeration for the RsOnline mode
//
typedef enum
{
    RSONLINE_CONTINUE     = 0,         //!< RsOnline Continue
    RSONLINE_INTERVAL     = 1          //!< RsOnline Interval
} RsOnline_Mode_e;

//! \brief Enumeration for the current direction mode
//
typedef enum
{
    CS_DIR_POSTIVE     = 0,              //!< Positive input direction
    CS_DIR_NEGATIVE    = 1               //!< Negative input direction
} CurrentSense_Dir_e;

//! \brief Enumeration for the current direction mode
//
typedef enum
{
    STARTUP_FREE     = 0,              //!< Startup mode
    STARTUP_HFI      = 1               //!< Startup mode
} Startup_Mode_e;


#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_ESMO_ONLY     = 1,     //!< the estimator is only ESMO
    SLEST_TYPE_FAST_ESMO     = 2      //!< the estimator is FAST and ESMO
} Estimator_Type_e;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_ENC_ONLY      = 1,     //!< the estimator is only ENC
    SLEST_TYPE_FAST_ENC      = 2      //!< the estimator is FAST and ENC
} Estimator_Type_e;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_HALL_ONLY     = 1,     //!< the estimator is only HALL
    SLEST_TYPE_FAST_HALL     = 2      //!< the estimator is FAST and HALL
} Estimator_Type_e;
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_ESMO_ONLY     = 0,     //!< the estimator is only ESMO
    SLEST_TYPE_ENC_ONLY      = 1,     //!< the estimator is only ENC
    SLEST_TYPE_ESMO_ENC      = 2      //!< the estimator is ESMO and ENC
} Estimator_Type_e;
#elif defined(MOTOR1_FAST)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_ESMO_ONLY     = 1,     //!< the estimator is only ESMO
    SLEST_TYPE_FAST_ESMO     = 2      //!< the estimator is FAST and ESMO
} Estimator_Type_e;
#elif defined(MOTOR1_ESMO)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_ESMO_ONLY     = 1,     //!< the estimator is only ESMO
    SLEST_TYPE_FAST_ESMO     = 2      //!< the estimator is FAST and ESMO
} Estimator_Type_e;
#elif defined(MOTOR1_ENC)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_ENC_ONLY      = 1,     //!< the estimator is only ENC
    SLEST_TYPE_FAST_ENC      = 2      //!< the estimator is FAST and ENC
} Estimator_Type_e;
#elif defined(MOTOR1_HALL)
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0,     //!< the estimator is only FAST
    SLEST_TYPE_HALL_ONLY     = 1,     //!< the estimator is only HALL
    SLEST_TYPE_FAST_HALL     = 2      //!< the estimator is FAST and HALL
} Estimator_Type_e;
#else
//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    SLEST_TYPE_FAST_ONLY     = 0     //!< the estimator is only FAST
} Estimator_Type_e;
#endif

// State machine typedef for motor running status
typedef enum
{
    MOTOR_STOP_IDLE      = 0,       // 0x00
    MOTOR_FAULT_STOP     = 1,       // 0x01
    MOTOR_BRAKE_STOP     = 2,       // 0x02
    MOTOR_NORM_STOP      = 3,       // 0x03
    MOTOR_CHARGE         = 4,       // 0x04
    MOTOR_SEEK_POS       = 5,       // 0x05
    MOTOR_ALIGNMENT      = 6,       // 0x06
    MOTOR_IPD_HFI        = 7,       // 0x07
    MOTOR_OL_START       = 8,       // 0x08
    MOTOR_CL_RUNNING     = 9,       // 0x09
    MOTOR_CTRL_RUN       = 10,      // 0x0A
    MOTOR_FWC_RUN        = 11,      // 0x0B
    MOTOR_BRAKE_RUN      = 12,      // 0x0C
    MOTOR_ID_START       = 13,      // 0x0D
    MOTOR_ID_RUN         = 14,      // 0x0E
    MOTOR_ID_DONE        = 15,      // 0x0F
    MOTOR_RS_START       = 16,      // 0x10
    MOTOR_RS_RUN         = 17,      // 0x11
    MOTOR_RS_DONE        = 18,      // 0x12
    MOTOR_PRMS_STORE     = 19,      // 0x13, only do this at motor stop
    MOTOR_CODE_UPDATE    = 20       // 0x14, only do this at motor stop
} MOTOR_Status_e;

//! \brief Enumeration for the motor drive control state
//
typedef enum
{
    MOTOR_CTRL_IDLE_STATE   = 0,    //!< (0000), Idle state, no any response
    MOTOR_CTRL_FREE_STOP    = 1,    //!< (0001), Stop the motor with free mode
    MOTOR_CTRL_BRAKE_STOP   = 2,    //!< (0010), Stop the motor with braking mode
    MOTOR_CTRL_URGENT_STOP  = 3,    //!< (0011), urgent stop
    MOTOR_CTRL_SPEED_CW     = 4,    //!< (0100), CW spin with speed closed-loop
    MOTOR_CTRL_SPEED_CCW    = 5,    //!< (0101), CCW spin with speed closed-loop
    MOTOR_CTRL_TORQUE_CW    = 6,    //!< (0110), CW spin with torque control
    MOTOR_CTRL_TORQUE_CCW   = 7,    //!< (0111), CCW spin with torque control
    MOTOR_CTRL_POWER_CW     = 8,    //!< (1000), CW spin with power control
    MOTOR_CTRL_POWER_CCW    = 9,    //!< (1001), CCW spin with power control
    MOTOR_CTRL_ID_AUTO      = 10,   //!< (1010), Motor parameters identification
    MOTOR_CTRL_ID_SET       = 11,   //!< (1011), Motor parameters identification
    MOTOR_CTRL_RS_AUTO      = 12,   //!< (1100), Rs recalculation
    MOTOR_CTRL_RS_SET       = 13,   //!< (1101), Rs recalculation
    MOTOR_CTRL_PRMS_RESET   = 14,   //!< (1110), Reset control parameters
    MOTOR_CTRL_DEBUG_CHECK  = 15,   //!< (1111), debug monitor
    MOTOR_CTRL_PRMS_STORE   = 0x40, //!< (0100), only do this at motor stop
    MOTOR_CTRL_CODE_UPDATE  = 0x80  //!< (1000), only do this at motor stop
} MOTOR_CtrlMode_e;

//! \brief Defines the motor parameters
//!
typedef struct _MOTOR_Params_
{
  MOTOR_Type_e type;               //!< Defines the motor type

  uint16_t     numPolePairs;       //!< Defines the number of pole pairs

  float32_t    Lmag_H;             //!< Defines the magnetizing inductance, H
  float32_t    Ls_d_H;             //!< Defines the direct stator inductance, H
  float32_t    Ls_q_H;             //!< Defines the quadrature stator inductance, H

  float32_t    Rr_d_Ohm;           //!< Defines the direct rotor resistance, Ohm
  float32_t    Rr_q_Ohm;           //!< Defines the quadrature rotor resistance, Ohm

  float32_t    Rs_d_Ohm;           //!< Defines the direct stator resistance, Ohm
  float32_t    Rs_q_Ohm;           //!< Defines the quadrature stator resistance, Ohm

  float32_t    ratedFlux_Wb;       //!< Defines the rated flux, Wb
} MOTOR_Params;

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

#endif // end of MOTOR_LIB_H definition





