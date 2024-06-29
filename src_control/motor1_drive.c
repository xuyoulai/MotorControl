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


//! \file   /solutions/universal_motorcontrol_lab/common/source/motor1_drive.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM boards
//!

//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"

#pragma CODE_SECTION(motor1CtrlISR, ".TI.ramfunc");
#pragma INTERRUPT(motor1CtrlISR, {HP});

// the globals

//!< the hardware abstraction layer object to motor control
volatile MOTOR_Handle motorHandle_M1;
#pragma DATA_SECTION(motorHandle_M1,"foc_data");

volatile MOTOR_Vars_t motorVars_M1;
#pragma DATA_SECTION(motorVars_M1, "foc_data");

MOTOR_SetVars_t motorSetVars_M1;
#pragma DATA_SECTION(motorSetVars_M1, "foc_data");

HAL_MTR_Obj    halMtr_M1;
#pragma DATA_SECTION(halMtr_M1, "foc_data");

#if defined(MOTOR1_FAST)
//!< the voltage Clarke transform object
CLARKE_Obj    clarke_V_M1;
#pragma DATA_SECTION(clarke_V_M1, "foc_data");
#endif  // MOTOR1_FAST

//!< the current Clarke transform object
CLARKE_Obj    clarke_I_M1;
#pragma DATA_SECTION(clarke_I_M1, "foc_data");

//!< the inverse Park transform object
IPARK_Obj     ipark_V_M1;
#pragma DATA_SECTION(ipark_V_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_I_M1;
#pragma DATA_SECTION(park_I_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_V_M1;
#pragma DATA_SECTION(park_V_M1, "foc_data");

//!< the Id PI controller object
PI_Obj        pi_Id_M1;
#pragma DATA_SECTION(pi_Id_M1, "foc_data");

//!< the Iq PI controller object
PI_Obj        pi_Iq_M1;
#pragma DATA_SECTION(pi_Iq_M1, "foc_data");

//!< the speed PI controller object
PI_Obj        pi_spd_M1;
#pragma DATA_SECTION(pi_spd_M1, "foc_data");

//!< the space vector generator object
SVGEN_Obj     svgen_M1;
#pragma DATA_SECTION(svgen_M1, "foc_data");

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
SVGENCURRENT_Obj svgencurrent_M1;
#pragma DATA_SECTION(svgencurrent_M1, "foc_data");
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
TRAJ_Obj     traj_spd_M1;
#pragma DATA_SECTION(traj_spd_M1, "foc_data");

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
PI_Obj       pi_fwc_M1;
#pragma DATA_SECTION(pi_fwc_M1, "foc_data");
#endif  // MOTOR1_FWC

#if defined(MOTOR1_ISBLDC)
//!< the isbldc object
ISBLDC_Obj isbldc_M1;
#pragma DATA_SECTION(isbldc_M1, "foc_data");

//!< the rimpulse object
RIMPULSE_Obj rimpulse_M1;
#pragma DATA_SECTION(rimpulse_M1, "foc_data");

//!< the mod6cnt object
MOD6CNT_Obj mod6cnt_M1;
#pragma DATA_SECTION(mod6cnt_M1, "foc_data");

//!< the bldc object
BLDC_Obj bldc_M1;
#pragma DATA_SECTION(bldc_M1, "foc_data");
#else // !MOTOR1_ISBLDC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen_M1;
#pragma DATA_SECTION(angleGen_M1, "foc_data");
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
VS_FREQ_Obj    VsFreq_M1;
#pragma DATA_SECTION(VsFreq_M1, "foc_data");
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#endif  // !MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
//!< the handle for the enc object
ENC_Obj enc_M1;
#pragma DATA_SECTION(enc_M1, "foc_data");

//!< the handle for the speedcalc object
SPDCALC_Obj speedcalc_M1;
#pragma DATA_SECTION(speedcalc_M1, "foc_data");
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
//!< the handle for the enc object
HALL_Obj hall_M1;
#pragma DATA_SECTION(hall_M1, "foc_data");

// Enable MOTOR1_HALL_CAL pre-defined symbols, run the motor with FAST for angle
// calibration
// Copy hall_M1.thetaCalBuf[] to hallAngleBuf[]
// 1->1, 2->2, 3->3, 4->4, 5->5, 6->6, 6->0
// Disable MOTOR1_HALL_CAL pre-defined symbols after calibration for normal operation
                                    // 6           1              2
                                    // 3           4              5
                                    // 6
#if (USER_MOTOR1 == Teknic_M2310PLN04K)
const float32_t hallAngleBuf[7] = { 2.60931063f,  -0.45987016f, -2.57219672f, \
                                    -1.50797582f, 1.59862518f , 0.580176473f,
                                    2.60931063f };

#elif (USER_MOTOR1 == Anaheim_BLY172S_24V)
const float32_t hallAngleBuf[7] = { -1.41421735f,  1.75656128f, -2.48391223f, \
                                    2.76515913f,  -0.460148782f, 0.606459916f,
                                    -1.41421735f };
#elif (USER_MOTOR1 == Anaheim_BLWS235D)
const float32_t hallAngleBuf[7] = { 1.64448488f,  -1.54361129f,  0.548367858f, \
                                   -0.390248626f,  2.67842388f, -2.52673817f,
                                    1.64448488f };
#elif (USER_MOTOR1 == Tool_Makita_GFD01)
const float32_t hallAngleBuf[7] = { -2.71645141f,  0.399317622f, 2.47442961f,  \
                                    1.47019732f, -1.67825437f, -0.643157125f, \
                                    -2.71645141f };
#else   // !Teknic_M2310PLN04K | !Anaheim_BLY172S_24V | !Anaheim_BLWS235D | !Tool_Makita_GFD01
#error Not a right hall angle buffer for this project, need to do hall calibration
#endif  //

#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
//!< the speedfr object
SPDFR_Obj spdfr_M1;
#pragma DATA_SECTION(spdfr_M1, "foc_data");

//!< the esmo object
ESMO_Obj   esmo_M1;
#pragma DATA_SECTION(esmo_M1, "foc_data");
#endif  // MOTOR1_ESMO
#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
MTPA_Obj     mtpa_M1;
#pragma DATA_SECTION(mtpa_M1, "foc_data");
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)
//!< the single-shunt current reconstruction object
DCLINK_SS_Obj    dclink_M1;

#pragma DATA_SECTION(dclink_M1, "foc_data");
#endif // MOTOR1_DCLINKSS

#if defined(MOTOR1_VOLRECT)
//!< the voltage reconstruct object
VOLREC_Obj volrec_M1;
#pragma DATA_SECTION(volrec_M1, "foc_data");
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_FILTERIS)
//!< first order current filter object
FILTER_FO_Obj    filterIs_M1[3];

#pragma DATA_SECTION(filterIs_M1, "foc_data");
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
//!< first order voltage filter object
FILTER_FO_Obj    filterVs_M1[3];

#pragma DATA_SECTION(filterVs_M1, "foc_data");
#endif  // MOTOR1_FILTERVS

#if defined(BSXL8316RT_REVA) && defined(OFFSET_CORRECTION)
MATH_Vec3 I_correct_A;

#pragma DATA_SECTION(I_correct_A, "foc_data");
#endif  // BSXL8316RT_REVA & OFFSET_CORRECTION

#if defined(BENCHMARK_TEST)
BMTEST_Vars_t bmarkTestVars;

#pragma DATA_SECTION(bmarkTestVars, "foc_data");
#endif  // BENCHMARK_TEST


// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->motorNum = MTR_1;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setParams_priv(obj->userParamsHandle);

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle);

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle, obj->userParamsHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;

    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;

    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;
    objSets->overModulation = USER_M1_MAX_VS_MAG_PU;

    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;

    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;
    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;

    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;
    objSets->IsFailedChekSet_A = USER_M1_FAULT_CHECK_CURRENT_A;

    objSets->maxPeakCurrent_A = USER_M1_ADC_FULL_SCALE_CURRENT_A * 0.475f;
    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->currentInv_sf = USER_M1_CURRENT_INV_SF;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;
    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;

    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;
    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;
    objSets->restartTimesSet = USER_M1_START_TIMES_SET;


    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    obj->adcData.current_sf = objUser->current_sf * USER_M1_SIGN_CURRENT_SF;

    obj->adcData.voltage_sf = objUser->voltage_sf;
    obj->adcData.dcBusvoltage_sf = objUser->voltage_sf;

    obj->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
    obj->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
    obj->speedFlyingStart_Hz = USER_MOTOR1_SPEED_FS_Hz;

    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
    obj->accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;

    obj->VsRef_pu = 0.98f * USER_M1_MAX_VS_MAG_PU;
    obj->VsRef_V =
            0.98f * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    obj->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    obj->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    obj->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    obj->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    obj->angleDelayed_sf = 0.5f * MATH_TWO_PI * USER_M1_CTRL_PERIOD_sec;

    obj->anglePhaseAdj_rad = MATH_PI * 0.001f;

    obj->power_sf = MATH_TWO_PI / USER_MOTOR1_NUM_POLE_PAIRS;
    obj->VIrmsIsrScale = objUser->ctrlFreq_Hz;

    obj->stopWaitTimeCnt = 0;
    obj->flagEnableRestart = false;

    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;
    obj->operateMode = OPERATE_MODE_SPEED;
//  obj->operateMode = OPERATE_MODE_SCALAR;
    obj->flyingStartTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.5f); // 0.5s
    obj->flyingStartMode = FLYINGSTART_MODE_HALT;

    if(objUser->flag_bypassMotorId == true)
    {
#if defined(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_COM_C;
#else  // !(MOTOR1_DCLINKSS)
        obj->svmMode = SVM_MIN_C;
#endif  // !(MOTOR1_DCLINKSS)
        obj->flagEnableFWC = true;
    }
    else
    {
        obj->svmMode = SVM_COM_C;
        obj->flagEnableFWC = false;
    }

    obj->flagEnableForceAngle = true;

    // true - enables flying start, false - disables flying start
    obj->flagEnableFlyingStart = true;

    // true - enables SSIPD start, false - disables SSIPD
    obj->flagEnableSSIPD = false;

    obj->flagEnableSpeedCtrl = true;
    obj->flagEnableCurrentCtrl = true;

    obj->IsSet_A = 0.0f;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);      // 2.0s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);   // 1.0s
    obj->startupTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);    // 2.0s

#if defined(MOTOR1_ISBLDC)
    obj->estimatorMode = ESTIMATOR_MODE_BINT;

    obj->flagEnableAlignment = true;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = true;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

//    obj->estimatorMode = ESTIMATOR_MODE_FAST;
    obj->estimatorMode = ESTIMATOR_MODE_HALL;

    obj->flagEnableAlignment = true;
#elif defined(MOTOR1_FAST)
    obj->estState = EST_STATE_IDLE;
    obj->trajState = EST_TRAJ_STATE_IDLE;

    obj->estimatorMode = ESTIMATOR_MODE_FAST;

    obj->flagEnableAlignment = false;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
#elif defined(MOTOR1_ESMO)
    obj->estimatorMode = ESTIMATOR_MODE_ESMO;

    obj->flagEnableAlignment = true;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
#elif defined(MOTOR1_ENC)
    obj->estimatorMode = ESTIMATOR_MODE_ENC;

    obj->flagEnableAlignment = true;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
#elif defined(MOTOR1_HALL)
    obj->estimatorMode = ESTIMATOR_MODE_HALL;

    obj->flagEnableAlignment = true;
#else   // Not select algorithm
#error Not select a right estimator for this project
#endif  // !MOTOR1_ISBLDC

    obj->speed_int_Hz = 0.0f;
    obj->speed_Hz = 0.0f;

    obj->speedAbs_Hz = 0.0f;
    obj->speedFilter_Hz = 0.0f;

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));

    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));

    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)
    obj->dclinkHandle = DCLINK_SS_init(&dclink_M1, sizeof(dclink_M1));

    DCLINK_SS_setInitialConditions(obj->dclinkHandle,
                                   HAL_getTimeBasePeriod(obj->halMtrHandle), 0.5f);

    //disable full sampling
//    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, false);     // default
    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, true);    // test, not recommend in most cases

    //enable sequence control
//    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, false);  // default
    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, true); // test, not recommend in most cases

    // Tdt  =  55 ns (Dead-time between top and bottom switch)
    // Tpd  = 140 ns (Gate driver propagation delay)
    // Tr   = 136 ns (Rise time of amplifier including power switches turn on time)
    // Ts   = 800 ns (Settling time of amplifier)
    // Ts&h = 100 ns (ADC sample&holder = 1+(9)+2 = 12 SYSCLK)
    // T_MinAVDuration = Tdt+Tr+Tpd+Ts+Ts&h
    //                 = 55+140+136+800+100 = 1231(ns) => 148 SYSCLK cycles
    // T_SampleDelay   = Tdt+Tpd+Tr+Ts
    //                 = 55+140+136+800     = 1131(ns) => 136 SYSCLK cycles
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, USER_M1_DCLINKSS_MIN_DURATION);
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, USER_M1_DCLINKSS_SAMPLE_DELAY);
#endif   // MOTOR1_DCLINKSS

#ifdef MOTOR1_VOLRECT
    // initialize the Voltage reconstruction
    obj->volrecHandle = VOLREC_init(&volrec_M1, sizeof(volrec_M1));

    // configure the Voltage reconstruction
    VOLREC_setParams(obj->volrecHandle,
                     objUser->voltageFilterPole_rps,
                     objUser->ctrlFreq_Hz);

    VOLREC_disableFlagEnableSf(obj->volrecHandle);
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_ESMO)
    // initialize the esmo
    obj->esmoHandle = ESMO_init(&esmo_M1, sizeof(esmo_M1));

    // set parameters for ESMO controller
    ESMO_setKslideParams(obj->esmoHandle,
                         USER_MOTOR1_KSLIDE_MAX, USER_MOTOR1_KSLIDE_MIN);

    ESMO_setPLLParams(obj->esmoHandle, USER_MOTOR1_PLL_KP_MAX,
                      USER_MOTOR1_PLL_KP_MIN, USER_MOTOR1_PLL_KP_SF);

    ESMO_setPLLKi(obj->esmoHandle, USER_MOTOR1_PLL_KI);   // Optional

    ESMO_setBEMFThreshold(obj->esmoHandle, USER_MOTOR1_BEMF_THRESHOLD);
    ESMO_setOffsetCoef(obj->esmoHandle, USER_MOTOR1_THETA_OFFSET_SF);
    ESMO_setBEMFKslfFreq(obj->esmoHandle, USER_MOTOR1_BEMF_KSLF_FC_SF);
    ESMO_setSpeedFilterFreq(obj->esmoHandle, USER_MOTOR1_SPEED_LPF_FC_Hz);

    // set the ESMO controller parameters
    ESMO_setParams(obj->esmoHandle, obj->userParamsHandle);

    // initialize the spdfr
    obj->spdfrHandle = SPDFR_init(&spdfr_M1, sizeof(spdfr_M1));

    // set the spdfr parameters
    SPDFR_setParams(obj->spdfrHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;
#endif  //MOTOR1_ESMO

#if defined(MOTOR1_ISBLDC)
    // initialize the ISBLDC handle
    obj->isbldcHandle = ISBLDC_init(&isbldc_M1, sizeof(isbldc_M1));

    // set the ISBLDC controller parameters
    ISBLDC_setParams(obj->isbldcHandle, obj->userParamsHandle,
                     USER_MOTOR1_ISBLDC_INT_MAX, USER_MOTOR1_ISBLDC_INT_MIN);

    // initialize the RIMPULSE handle
    obj->rimpulseHandle = RIMPULSE_init(&rimpulse_M1, sizeof(rimpulse_M1));

    // set the RIMPULSE controller parameters
    RIMPULSE_setParams(obj->rimpulseHandle, obj->userParamsHandle,
                       USER_MOTOR1_RAMP_START_Hz, USER_MOTOR1_RAMP_END_Hz,
                       USER_MOTOR1_RAMP_DELAY);

    // initialize the MOD6CNT handle
    obj->mod6cntHandle = MOD6CNT_init(&mod6cnt_M1, sizeof(mod6cnt_M1));

    // sets up the MOD6CNT controller parameters
    MOD6CNT_setMaximumCount(obj->mod6cntHandle, 6);

    // initialize the BLDC handle
    obj->bldcHandle = &bldc_M1;

    // sets up initialization value for startup
    obj->bldcHandle->IdcRefSet = 1.0f;           // 1.0A
    obj->bldcHandle->IdcStart = USER_MOTOR1_ISBLDC_I_START_A;       // 0.5A

#if (DMC_BUILDLEVEL == DMC_LEVEL_2)
    obj->bldcHandle->pwmDutySet = 0.10f;         // 10% duty
    obj->bldcHandle->pwmDutyStart = USER_MOTOR1_ISBLDC_DUTY_START;  // 10% duty
#endif  // DMC_LEVEL_2

    obj->bldcHandle->commSampleDelay = 3;
#else  // !MOTOR1_ISBLDC
#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));

    ANGLE_GEN_setParams(obj->angleGenHandle, objUser->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = obj->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));

    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objUser->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_Hz);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_Hz, USER_MOTOR1_FREQ_HIGH_Hz,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)
#endif  // !MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
    // initialize the enc handle
    obj->encHandle = ENC_init(&enc_M1, sizeof(enc_M1));

    // set the ENC controller parameters
    ENC_setQEPHandle(obj->encHandle, MTR1_QEP_BASE);
    ENC_setParams(obj->encHandle, obj->userParamsHandle);

#if !defined(_F280013x)
    ENC_setHallGPIO(obj->encHandle, MTR1_HALL_U_GPIO,
                    MTR1_HALL_V_GPIO, MTR1_HALL_W_GPIO);
#endif  // !_F280013x

    // initialize the apll handle
    obj->spdcalcHandle = SPDCALC_init(&speedcalc_M1, sizeof(speedcalc_M1));

    // set the SPEEDCALC controller parameters
    SPDCALC_setParams(obj->spdcalcHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_ENC


#if defined(MOTOR1_HALL)
    // initialize the hall handle
    obj->hallHandle = HALL_init(&hall_M1, sizeof(hall_M1));

    // set the HALL controller parameters
    HALL_setParams(obj->hallHandle, obj->userParamsHandle);
    HALL_setAngleBuf(obj->hallHandle, &hallAngleBuf[0]);
    HALL_setAngleDelta_rad(obj->hallHandle, USER_MOTOR1_HALL_DELTA_rad);
    HALL_setGPIOs(obj->hallHandle,
                  MTR1_HALL_U_GPIO, MTR1_HALL_V_GPIO, MTR1_HALL_W_GPIO);

    obj->frswPos_sf = 1.0f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_HALL

#if defined(MOTOR1_FAST)
    // initialize the Clarke modules
    obj->clarkeHandle_V = CLARKE_init(&clarke_V_M1, sizeof(clarke_V_M1));

    // set the Clarke parameters
    setupClarke_V(obj->clarkeHandle_V, objUser->numVoltageSensors);
#endif // MOTOR1_FAST

    // initialize the Clarke modules
    obj->clarkeHandle_I = CLARKE_init(&clarke_I_M1, sizeof(clarke_I_M1));

    // set the Clarke parameters
    setupClarke_I(obj->clarkeHandle_I, objUser->numCurrentSensors);

    // initialize the inverse Park module
    obj->iparkHandle_V = IPARK_init(&ipark_V_M1, sizeof(ipark_V_M1));

    // initialize the Park module
    obj->parkHandle_I = PARK_init(&park_I_M1, sizeof(park_I_M1));

    // initialize the Park module
    obj->parkHandle_V = PARK_init(&park_V_M1, sizeof(park_V_M1));

    // initialize the PI controllers
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setMinValue(obj->trajHandle_spd, -objUser->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objUser->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objUser->maxAccel_Hzps * objUser->ctrlPeriod_sec));

    // initialize the space vector generator module
    obj->svgenHandle = SVGEN_init(&svgen_M1, sizeof(svgen_M1));

#if !defined(MOTOR1_ISBLDC)
    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);
#endif  //! MOTOR1_ISBLDC

#if !defined(MOTOR1_DCLINKSS)   // 2/3-shunt
    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.01f, 0.01f, 0.14f);
#else   // MOTOR1_DCLINKSS
    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.0f, 0.0f, 0.0f);
#endif  // MOTOR1_DCLINKSS

#if defined(MOTOR1_FILTERIS)
    obj->flagEnableFilterIs = true;

    // assign the current filter handle (low pass filter)
    obj->filterHandle_Is[0] = FILTER_FO_init((void *)(&filterIs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[1] = FILTER_FO_init((void *)(&filterIs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[2] = FILTER_FO_init((void *)(&filterIs_M1[2]), sizeof(FILTER_FO_Obj));

    obj->filterIsPole_rps = USER_M1_IS_FILTER_POLE_rps;     //

    float32_t beta_lp_Is = obj->filterIsPole_rps * objUser->ctrlPeriod_sec;

    float32_t a1_Is = (beta_lp_Is - (float32_t)2.0f) / (beta_lp_Is + (float32_t)2.0f);
    float32_t b0_Is = beta_lp_Is / (beta_lp_Is + (float32_t)2.0f);
    float32_t b1_Is = b0_Is;

    // set filter coefficients for current filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[0], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[0], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[1], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[1], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[2], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[2], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    obj->flagEnableFilterVs = true;

    // assign the voltage filter handle (low pass filter)
    obj->filterHandle_Vs[0] = FILTER_FO_init((void *)(&filterVs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[1] = FILTER_FO_init((void *)(&filterVs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[2] = FILTER_FO_init((void *)(&filterVs_M1[2]), sizeof(FILTER_FO_Obj));

    obj->filterVsPole_rps = USER_M1_VS_FILTER_POLE_rps;

    float32_t beta_lp_Vs = obj->filterVsPole_rps * objUser->ctrlPeriod_sec;

    float32_t a1_Vs = (beta_lp_Vs - (float32_t)2.0f) / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b0_Vs = beta_lp_Vs / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b1_Vs = b0_Vs;

    // set filter coefficients for voltage filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[0], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[0], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[1], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[1], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[2], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[2], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgencurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));

    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM

#if defined(MOTOR1_FAST)
    // initialize the estimator
    obj->estHandle = EST_initEst(MTR_1);

    // set the default estimator parameters
    EST_setParams(obj->estHandle, obj->userParamsHandle);
    EST_setFlag_enableForceAngle(obj->estHandle, obj->flagEnableForceAngle);
    EST_setFlag_enableRsRecalc(obj->estHandle, obj->flagEnableRsRecalc);

    // set the scale factor for high frequency low inductance motor
    EST_setOneOverFluxGain_sf(obj->estHandle,
                              obj->userParamsHandle, USER_M1_EST_FLUX_HF_SF);
    EST_setFreqLFP_sf(obj->estHandle,
                      obj->userParamsHandle, USER_M1_EST_FREQ_HF_SF);
    EST_setBemf_sf(obj->estHandle,
                   obj->userParamsHandle, USER_M1_EST_BEMF_HF_SF);

#if defined(MOTOR1_LS_CAL)
    objSets->Ls_d_comp_H = EST_getLs_d_H(obj->estHandle);
    objSets->Ls_q_comp_H = EST_getLs_q_H(obj->estHandle);

    objSets->Ls_d_Icomp_coef = USER_MOTOR1_Ls_d_COMP_COEF / obj->maxCurrent_A;
    objSets->Ls_q_Icomp_coef = USER_MOTOR1_Ls_q_COMP_COEF / obj->maxCurrent_A;

    objSets->Ls_min_H = objSets->Ls_d_comp_H * USER_MOTOR1_Ls_MIN_NUM_COEF;

    obj->flagEnableLsUpdate = false;
#endif // MOTOR1_LS_CAL

    // for Rs re-calculation
    obj->flagEnableRsRecalc = false;

    // if motor is an induction motor, configure default state of PowerWarp
    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
    {
        EST_setFlag_enablePowerWarp(obj->estHandle, obj->flagEnablePowerWarp);
        EST_setFlag_bypassLockRotor(obj->estHandle, obj->flagBypassLockRotor);
    }

    // for Rs online calibration
    obj->flagRsOnLineContinue = false;
    obj->flagStartRsOnLine = false;

    objSets->RsOnlineWaitTimeSet = 6000;  //USER_MOTOR1_RSONLINE_WAIT_TIME;
    objSets->RsOnlineWorkTimeSet = 24000; //USER_MOTOR1_RSONLINE_WORK_TIME;
#endif // MOTOR1_FAST


#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = false;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE

#if defined(MOTOR1_RPM_CMD)
    obj->flagCmdRpmOrHz = false;     // the speed command is rpm
    obj->rpm2Hz_sf = objUser->motor_numPolePairs / 60.0f;
    obj->hz2Rpm_sf = 60.0f / objUser->motor_numPolePairs;
#endif  // MOTOR1_RPM_CMD


    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

#if defined(MOTOR1_PI_TUNE)
    // set the coefficient of the controllers gains
    setupControllerSF(handle);
#endif      // MOTOR1_PI_TUNE

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

#if defined(BENCHMARK_TEST)
    bmarkTestVars.recordDataCount = 0;
    bmarkTestVars.recordTicksSet = 15;
#endif  // BENCHMARK_TEST

    return;
}   // end of initMotor1CtrlParameters() function


void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // calculate motor protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

#if defined(MOTOR1_DCLINKSS)
    HAL_MTR_Obj *objHal = (HAL_MTR_Obj *)(obj->halMtrHandle);

    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, 5);

    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, 5);
#endif  // MOTOR1_DCLINKSS

        // Offsets in phase current sensing
#if defined(MOTOR1_ISBLDC)
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#elif defined(MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                              MTR1_IDC3_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                              MTR1_IDC4_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                              USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                              USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                              USER_M1_IC_OFFSET_AD);

    obj->adcData.offset_I_ad.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->adcData.offset_I_ad.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->adcData.offset_I_ad.value[2]  = USER_M1_IC_OFFSET_AD;
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
    // Offsets in phase voltage sensing
    obj->adcData.offset_V_sf.value[0]  = USER_M1_VA_OFFSET_SF;
    obj->adcData.offset_V_sf.value[1]  = USER_M1_VB_OFFSET_SF;
    obj->adcData.offset_V_sf.value[2]  = USER_M1_VC_OFFSET_SF;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->adcData.current_sf;

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
        float32_t invVdcbus;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

        uint16_t offsetCnt;

        DEVICE_DELAY_US(2.0f);      // delay 2us

#if defined(MOTOR1_ISBLDC)
        uint16_t period = EPWM_getTimeBasePeriod(MTR1_PWM_U_BASE);
        EPWM_setCounterCompareValue(MTR1_PWM_U_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));
        EPWM_setCounterCompareValue(MTR1_PWM_V_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));
        EPWM_setCounterCompareValue(MTR1_PWM_W_BASE,
                                    EPWM_COUNTER_COMPARE_A, (period >> 1));

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);
#elif defined(MOTOR1_DCLINKSS)
        HAL_setOffsetTrigger(obj->halMtrHandle);

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);

        obj->adcData.offset_Idc_ad  = USER_M1_IDC_OFFSET_AD * USER_M1_CURRENT_SF;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

#else  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);

        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * obj->adcData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == false);

            HAL_readMtr1ADCData(&obj->adcData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
#if defined(MOTOR1_ISBLDC)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                         (obj->adcData.Idc1_A.value[0] + obj->adcData.Idc1_A.value[1]) * 0.5f * offsetK2;
#elif defined(MOTOR1_DCLINKSS)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                               0.25f * offsetK2 *(obj->adcData.Idc1_A.value[0] +
                                                  obj->adcData.Idc1_A.value[1] +
                                                  obj->adcData.Idc2_A.value[0] +
                                                  obj->adcData.Idc2_A.value[1]);
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
                obj->adcData.offset_I_ad.value[0] =
                        offsetK1 * obj->adcData.offset_I_ad.value[0] +
                        obj->adcData.I_A.value[0] * offsetK2;

                obj->adcData.offset_I_ad.value[1] =
                        offsetK1 * obj->adcData.offset_I_ad.value[1] +
                        obj->adcData.I_A.value[1] * offsetK2;

                obj->adcData.offset_I_ad.value[2] =
                        offsetK1 * obj->adcData.offset_I_ad.value[2] +
                        obj->adcData.I_A.value[2] * offsetK2;
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
                invVdcbus = 1.0f / obj->adcData.VdcBus_V;

                // Offsets in phase voltage sensing
                obj->adcData.offset_V_sf.value[0] =
                         offsetK1 * obj->adcData.offset_V_sf.value[0] +
                         (invVdcbus * obj->adcData.V_V.value[0]) * offsetK2;

                obj->adcData.offset_V_sf.value[1] =
                         offsetK1 * obj->adcData.offset_V_sf.value[1] +
                         (invVdcbus * obj->adcData.V_V.value[1]) * offsetK2;

                obj->adcData.offset_V_sf.value[2] =
                         offsetK1 * obj->adcData.offset_V_sf.value[2] +
                         (invVdcbus * obj->adcData.V_V.value[2]) * offsetK2;
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC
            }
            else if(offsetCnt <= 1000)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

#if defined(MOTOR1_ISBLDC)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ISBLDC_setVabcOffset(obj->isbldcHandle, &obj->adcData.offset_V_sf);
#elif defined(MOTOR1_DCLINKSS)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * invCurrentSf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * invCurrentSf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[2]);
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    }   // flagEnableOffsetCalc = true

#if defined(MOTOR1_ISBLDC)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#elif defined(MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#else // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_I_ad.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#endif // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
    if( (obj->adcData.offset_V_sf.value[0] > USER_M1_VA_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[0] < USER_M1_VA_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[1] > USER_M1_VB_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[1] < USER_M1_VB_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }

    if( (obj->adcData.offset_V_sf.value[2] > USER_M1_VC_OFFSET_SF_MAX) ||
        (obj->adcData.offset_V_sf.value[2] < USER_M1_VC_OFFSET_SF_MIN) )
    {
        obj->faultMtrNow.bit.voltageOffset = 1;
    }
#endif  // MOTOR1_FAST ||  MOTOR1_ISBLDC

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
            (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = false;
    }

    return;
} // end of runMotor1OffsetsCalculation() function



void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagClearFaults == true)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = false;
    }

    if(obj->flagEnableRunAndIdentify == true)
    {
        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == true)
            {
                obj->flagRunIdentAndOnLine = false;
                obj->motorState = MOTOR_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == false)
                {
                    obj->flagEnableRunAndIdentify = false;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = false;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == false) &&
                (obj->stopWaitTimeCnt == 0))
        {
            restartMotorControl(handle);
        }
    }
    // if(obj->flagEnableRunAndIdentify == false)
    else if(obj->flagRunIdentAndOnLine == true)
    {
        stopMotorControl(handle);

        if(obj->flagEnableFlyingStart == false)
        {
            obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
        }
        else
        {
            obj->stopWaitTimeCnt = 0;
        }
    }
    else
    {
    }

#if defined(MOTOR1_FAST)
    // enable or disable bypassLockRotor flag
    if((objUser->motor_type == MOTOR_TYPE_INDUCTION)
        && (obj->flagMotorIdentified == true))
    {
        EST_setFlag_bypassLockRotor(obj->estHandle,
                                    obj->flagBypassLockRotor);
    }
#endif // MOTOR1_FAST

    if(obj->flagRunIdentAndOnLine == true)
    {
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
        {
#if defined(MOTOR1_FAST)
            // enable the estimator
            EST_enable(obj->estHandle);

            // enable the trajectory generator
            EST_enableTraj(obj->estHandle);
#endif // MOTOR1_FAST

            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }

#if defined(MOTOR1_FAST)
        if(obj->flagMotorIdentified == true)
#endif  // MOTOR1_FAST
        {


            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

        #if defined(MOTOR1_FAST)
            // enable or disable force angle
            EST_setFlag_enableForceAngle(obj->estHandle,
                                         obj->flagEnableForceAngle);

            // enable or disable stator resistance (Rs) re-calculation
            EST_setFlag_enableRsRecalc(obj->estHandle,
                                       obj->flagEnableRsRecalc);
        #endif  // MOTOR1_FAST

            // Sets the target speed for the speed trajectory
        #if defined(MOTOR1_ESMO)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_ISBLDC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                obj->speed_int_Hz = obj->speedForce_Hz * obj->direction;
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_ENC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_HALL)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_FAST)
            TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
        #else   // !MOTOR1_ESMO && !MOTOR1_FAST
        #error No select a right estimator for motor_1 control
        #endif  // MOTOR1_ESMO || MOTOR1_FAST

            if((fabsf(obj->speed_Hz) > obj->speedStart_Hz) ||
                    (obj->motorState == MOTOR_CTRL_RUN))
            {
                //  Sets the acceleration / deceleration for the speed trajectory
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationMax_Hzps * objUser->ctrlPeriod_sec));

#if defined(MOTOR1_FAST) && defined(MOTOR1_LS_CAL)
                if(obj->flagEnableLsUpdate ==  true)
                {
                    // Calculate the Ld and Lq which reduce with current
                    objSets->Ls_d_comp_H = objUser->motor_Ls_d_H * (1.0f - obj->Is_A * objSets->Ls_d_Icomp_coef);
                    objSets->Ls_q_comp_H = objUser->motor_Ls_q_H * (1.0f - obj->Is_A * objSets->Ls_q_Icomp_coef);

                    if(objSets->Ls_d_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_d_comp_H = objSets->Ls_min_H;
                    }

                    if(objSets->Ls_q_comp_H < objSets->Ls_min_H)
                    {
                        objSets->Ls_q_comp_H = objSets->Ls_min_H;
                    }

                    // Update the Ld and Lq for motor control
                    EST_setLs_d_H(obj->estHandle, objSets->Ls_d_comp_H);
                    EST_setLs_q_H(obj->estHandle, objSets->Ls_q_comp_H);
                }
#endif //MOTOR1_FAST & MOTOR1_LS_CAL

            #if defined(MOTOR1_ISBLDC)
                ISBLDC_updateThresholdInt(obj->isbldcHandle, obj->speed_int_Hz);
                #if (DMC_BUILDLEVEL >= DMC_LEVEL_4)
                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);
                #else
                PI_setMinMax(obj->piHandle_spd, -1.0f, 1.0f);
                #endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3
            #else  // !MOTOR1_ISBLDC
                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);

                SVGEN_setMode(obj->svgenHandle, obj->svmMode);
            #endif  // !MOTOR1_ISBLDC

                if(obj->motorState == MOTOR_CL_RUNNING)
                {
                    obj->stateRunTimeCnt++;

                    if(obj->stateRunTimeCnt == obj->startupTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->motorState = MOTOR_CTRL_RUN;
                    }
                }
            }
            else
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationStart_Hzps * objUser->ctrlPeriod_sec));

            #if defined(MOTOR1_ISBLDC)
                #if (DMC_BUILDLEVEL >= DMC_LEVEL_4)
                if(obj->speed_int_Hz > 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
                #else   // (DMC_BUILDLEVEL < DMC_LEVEL_3)
                PI_setMinMax(obj->piHandle_spd, -1.0f, 1.0f);
                #endif  // DMC_BUILDLEVEL < DMC_LEVEL_3
            #else  // !MOTOR1_ISBLDC
                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
            #endif  // !MOTOR1_ISBLDC
            }
        }

        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
    else
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }

#if defined(MOTOR1_FAST)
    // check the trajectory generator
    if(EST_isTrajError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else
    {
        // update the trajectory generator state
        EST_updateTrajState(obj->estHandle);
    }


    // check the estimator
    if(EST_isError(obj->estHandle) == true)
    {
        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);
    }
    else
    {
        bool flagEstStateChanged = false;

        float32_t Id_target_A = EST_getIntValue_Id_A(obj->estHandle);

        if(obj->flagMotorIdentified == true)
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, 0.0f);
        }
        else
        {
            flagEstStateChanged = EST_updateState(obj->estHandle, Id_target_A);
        }

        if(flagEstStateChanged == true)
        {
            // configure the trajectory generator, enter once every state
            EST_configureTraj(obj->estHandle);

            if(obj->flagMotorIdentified == false)
            {
                // configure the controllers, enter once every state
                EST_configureTrajState(obj->estHandle, obj->userParamsHandle,
                                       obj->piHandle_spd,
                                       obj->piHandle_Id, obj->piHandle_Iq);
            }

            if(objUser->flag_bypassMotorId == false)
            {
                if((EST_isLockRotor(obj->estHandle) == true) ||
                        ( (EST_isMotorIdentified(obj->estHandle) == true)
                                  && (EST_isIdle(obj->estHandle) == true) ) )
                {
                    if(EST_isMotorIdentified(obj->estHandle) == true)
                    {
                        obj->flagMotorIdentified = true;

                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;

                        // disable the estimator
                        EST_disable(obj->estHandle);

                        // enable the trajectory generator
                        EST_disableTraj(obj->estHandle);
                    }

                    if(objUser->motor_type == MOTOR_TYPE_INDUCTION)
                    {
                        // clear the flag
                        obj->flagRunIdentAndOnLine = false;
                        obj->flagEnableRunAndIdentify = false;
                    }
                }
            }   // objUser->flag_bypassMotorId = false
        }
    }

    obj->flagMotorIdentified = EST_isMotorIdentified(obj->estHandle);

#else   // !MOTOR1_FAST
    obj->flagMotorIdentified = true;
#endif //  !MOTOR1_FAST

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagSetupController == true)
        {
            // update the controller
            updateControllers(handle);
        }
        else
        {
            obj->flagSetupController = true;

            setupControllers(handle);
        }
    }


#if defined(MOTOR1_FAST)
    // run Rs online
    runRsOnLine(handle);
#endif // MOTOR1_FAST


    // update the global variables
    updateGlobalVariables(handle);

#if defined(MOTOR1_ESMO)
    if(obj->motorState >= MOTOR_CTRL_RUN)
    {
        ESMO_updateFilterParams(obj->esmoHandle);
        ESMO_updatePLLParams(obj->esmoHandle);
    }
#endif  // MOTOR2_ESMO

    return;
}   // end of the runMotor1Control() function

__interrupt void motor1CtrlISR(void)
{

    motorVars_M1.ISRCount++;


    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);


    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();

    // read the ADC data with offsets
    HAL_readMtr1ADCData(&obj->adcData);

#if defined(BSXL8316RT_REVA) && defined(OFFSET_CORRECTION)
    // CSA Offset correction
    I_correct_A.value[0] = 0.995832f * obj->adcData.I_A.value[0] -
                           0.028199f * obj->adcData.I_A.value[1] -
                           0.014988f * obj->adcData.I_A.value[2];

    I_correct_A.value[1] = 0.037737f * obj->adcData.I_A.value[0] +
                           1.007723f * obj->adcData.I_A.value[1] -
                           0.033757f * obj->adcData.I_A.value[2];

    I_correct_A.value[2] = 0.009226f * obj->adcData.I_A.value[0] +
                           0.029805f * obj->adcData.I_A.value[1] +
                           1.003268f * obj->adcData.I_A.value[2];

    obj->adcData.I_A.value[0] = I_correct_A.value[0];
    obj->adcData.I_A.value[1] = I_correct_A.value[1];
    obj->adcData.I_A.value[2] = I_correct_A.value[2];

#endif  // BSXL8316RT_REVA & OFFSET_CORRECTION
//------------------------------------------------------------------------------
//**!!! ISBLDC only supports one direction rotation (speed_ref = positive) !!
#if defined(MOTOR1_ISBLDC)
    obj->bldcHandle->IdcIn =
            (obj->adcData.Idc1_A.value[0] + obj->adcData.Idc1_A.value[1]) * 0.5f;

    if(obj->bldcHandle->commSampleCount >= obj->bldcHandle->commSampleDelay)
    {
        obj->bldcHandle->IdcInFilter = (obj->bldcHandle->IdcInBuff[2] + \
                                        obj->bldcHandle->IdcInBuff[1] + \
                                        obj->bldcHandle->IdcInBuff[0] + \
                                        obj->bldcHandle->IdcIn) * 0.25f;

        obj->bldcHandle->IdcInBuff[2] = obj->bldcHandle->IdcInBuff[1];
        obj->bldcHandle->IdcInBuff[1] = obj->bldcHandle->IdcInBuff[0];
        obj->bldcHandle->IdcInBuff[0] = obj->bldcHandle->IdcIn;
    }

    if(obj->bldcHandle->commTrigFlag == true)
    {
        obj->bldcHandle->commSampleCount = 0;
    }

    obj->bldcHandle->commSampleCount++;

    if((obj->flagRunIdentAndOnLine == true) && (obj->motorState >= MOTOR_CL_RUNNING))
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    }

    MOD6CNT_setFlagCount(obj->mod6cntHandle, obj->bldcHandle->commTrigFlag);

    MOD6CNT_run(obj->mod6cntHandle, obj->speed_int_Hz);

    obj->bldcHandle->commState = MOD6CNT_getCounter(obj->mod6cntHandle);

    ISBLDC_setCommState(obj->isbldcHandle, obj->bldcHandle->commState);

    ISBLDC_run(obj->isbldcHandle, &obj->adcData.V_V);

    obj->speedINT_Hz = ISBLDC_getSpeedINT(obj->isbldcHandle);
    obj->speed_Hz = obj->speedINT_Hz * obj->direction;

#if (DMC_BUILDLEVEL == DMC_LEVEL_1)
    RIMPULSE_run(obj->rimpulseHandle);
    obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);

    obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutySet;
#endif  // DMC_LEVEL_1

#if (DMC_BUILDLEVEL == DMC_LEVEL_2)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutySet;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        RIMPULSE_run(obj->rimpulseHandle);

        obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;
        obj->enableSpeedCtrl = false;

        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else
        {
            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->stateRunTimeCnt = 0;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }
#endif  // DMC_LEVEL_2

#if (DMC_BUILDLEVEL == DMC_LEVEL_3)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);

            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->motorState = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
                obj->speed_int_Hz = obj->speedINT_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedINT_Hz);
                PI_setUi(obj->piHandle_spd, obj->bldcHandle->pwmDuty);
            }
        }
        else
        {
            RIMPULSE_run(obj->rimpulseHandle);

            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

            obj->enableSpeedCtrl = false;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->pwmDuty = obj->bldcHandle->pwmDutyStart;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }

    // run the speed controller
    if(obj->enableSpeedCtrl == true)
    {
        obj->counterSpeed++;

        if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
        {
            obj->counterSpeed = 0;

            PI_run(obj->piHandle_spd, obj->speed_int_Hz,
                   obj->speed_Hz, (float32_t*)&obj->bldcHandle->pwmDuty);
        }
    }
#endif  // DMC_LEVEL_3

#if (DMC_BUILDLEVEL == DMC_LEVEL_4)
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        if(RIMPULSE_getRmpDoneFlag(obj->rimpulseHandle) == true)
        {
            obj->bldcHandle->commTrigFlag = ISBLDC_getCommTrigFlag(obj->isbldcHandle);
            obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;

            obj->stateRunTimeCnt++;

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                obj->motorState = MOTOR_CL_RUNNING;
                obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
                obj->speed_int_Hz = obj->speedINT_Hz;

                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedINT_Hz);
                PI_setUi(obj->piHandle_spd, obj->bldcHandle->IdcRef);
            }
        }
        else
        {
            RIMPULSE_run(obj->rimpulseHandle);

            obj->bldcHandle->commTrigFlag = RIMPULSE_getTrigFlag(obj->rimpulseHandle);
            obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;

            obj->enableSpeedCtrl = false;
            obj->stateRunTimeCnt = 0;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->bldcHandle->commTrigFlag = false;
        obj->enableSpeedCtrl = false;

        MOD6CNT_setCounter(obj->mod6cntHandle, 0);

        obj->bldcHandle->IdcRef = obj->bldcHandle->IdcStart;
        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;

        obj->stateRunTimeCnt++;

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;
        }
    }

    // run the speed controller
    if(obj->enableSpeedCtrl == true)
    {
        obj->counterSpeed++;

        if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
        {
            obj->counterSpeed = 0;

            PI_run(obj->piHandle_spd, fabsf(obj->speed_int_Hz),
                   fabsf(obj->speed_Hz), (float32_t*)&obj->bldcHandle->IdcRef);
        }
    }

    if(obj->enableCurrentCtrl == true)
    {
        PI_run(obj->piHandle_Iq, obj->bldcHandle->IdcRef,
               obj->bldcHandle->IdcInFilter, &obj->bldcHandle->pwmDuty);
    }
#endif  // DMC_LEVEL_4

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        HAL_writePWMDataBLDC(obj->halMtrHandle,
                         obj->bldcHandle->pwmDuty, obj->bldcHandle->commState);
    }
    // defined(MOTOR1_ISBLDC)
//------------------------------------------------------------------------------
// 180-degree Sinusoidal Sensorless-FOC or Sensored-FOC
//******************************************************************************
#else  // !MOTOR1_ISBLDC
#if defined(MOTOR1_DCLINKSS)
    // run single-shunt current reconstruction
    DCLINK_SS_runCurrentReconstruction(obj->dclinkHandle,
                                     &obj->adcData.Idc1_A, &obj->adcData.Idc2_A);

    obj->sector = DCLINK_SS_getSector1(obj->dclinkHandle);

    obj->adcData.I_A.value[0] = DCLINK_SS_getIa(obj->dclinkHandle);
    obj->adcData.I_A.value[1] = DCLINK_SS_getIb(obj->dclinkHandle);
    obj->adcData.I_A.value[2] = DCLINK_SS_getIc(obj->dclinkHandle);

#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == true)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS
#else // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == true)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_OVM)
    // Over Modulation Supporting, run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(obj->svgencurrentHandle,
                                 &obj->adcData.I_A, &obj->adcDataPrev);
#endif  // MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)    // (OK<->OK)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT
    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // MOTOR1_VOLRECT

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->motorState = MOTOR_CTRL_RUN;

        // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);
    }
    else if(obj->flagMotorIdentified == true)
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    // run the FAST estimator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->estInputData.Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleEST_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            obj->motorState = MOTOR_CL_RUNNING;

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(fabsf(obj->estInputData.speed_ref_Hz) >= obj->speedForce_Hz)
            {
                TRAJ_setIntValue(obj->trajHandle_spd, obj->estInputData.speed_ref_Hz);

                if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
                {
                    obj->motorState = MOTOR_CL_RUNNING;

                    EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#ifdef MOTOR1_VOLRECT
    if(obj->motorState == MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(obj->speedAbs_Hz >= obj->speedForce_Hz)
        {
            obj->motorState = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

    // End of MOTOR1_FAST && MOTOR1_ESMO
//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->motorState = MOTOR_CTRL_RUN;

        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    // Runs the FAST estimator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // Runs encoder
    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;

            if((ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE) ||
                    ((obj->stateRunTimeCnt > obj->forceRunTimeDelay)))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                    (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);

                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        obj->speed_Hz = obj->speedENC_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleENC_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
            {
                obj->motorState = MOTOR_CL_RUNNING;
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

#ifdef BRAKE_ENABLE
    if(obj->flagEnableBraking == true)
    {
        if(obj->motorState != MOTOR_BRAKE_STOP)
        {
            obj->motorState = MOTOR_BRAKE_STOP;

            if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
            {
                // enable the braking mode PWM with
                // turning-on three low side, turn off three high side
                HAL_enableBrakePWM(obj->halMtrHandle);
            }
            else if(obj->brakingMode == FORCESTOP_BRAKE_MODE)
            {
                obj->angleBrake_rad = obj->angleFOC_rad;
                PI_setRefValue(obj->piHandle_spd, 0.0f);
                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }

        if(obj->brakingMode == FORCESTOP_BRAKE_MODE)
        {
            // compute the sin/cos phasor
            obj->angleBrake_rad = obj->angleBrake_rad;

            obj->IsRef_A = obj->brakingCurrent_A;
            obj->Idq_out_A.value[1] = obj->brakingCurrent_A;

            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = true;
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }
    }
#endif  // BRAKE_ENABLE

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST && MOTOR1_ENC
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;

    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG


    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);

    // run the encoder
    ENC_inline_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(fabsf(obj->speed_int_Hz) >= obj->speedForce_Hz)
            {
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

                if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
                {
                    obj->motorState = MOTOR_CL_RUNNING;
                    obj->stateRunTimeCnt = 0;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ESMO_setAnglePu(obj->esmoHandle, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->motorState = MOTOR_OL_START;
                obj->stateRunTimeCnt = 0;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        obj->speed_Hz = obj->speedENC_Hz;

        if(obj->motorState >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleENC_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleENC_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
            {
                obj->motorState = MOTOR_CL_RUNNING;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ESMO && MOTOR1_ENC

//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    // sensorless-FOC
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);

    if(obj->flagMotorIdentified == true)
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }
    else if(EST_isEnabled(obj->estHandle) == true)  // run identification
    {
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0f);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);

        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);

        obj->motorState = MOTOR_CTRL_RUN;
    }

    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;


    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;


    // Hall Sensor
    HALL_setTimeStamp(obj->hallHandle, HAL_calcCAPCount(obj->halMtrHandle));
    HALL_run(obj->hallHandle, obj->speed_int_Hz);
    obj->angleHall_rad = HALL_getAngle_rad(obj->hallHandle);
    obj->speedHall_Hz = HALL_getSpeed_Hz(obj->hallHandle);


    #ifdef HALL_CAL
    HALL_calibrateIndexAngle(obj->hallHandle, obj->angleEST_rad);
    #endif  // MOTOR1_HALL_CAL

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    // Running state
    if(obj->estimatorMode == ESTIMATOR_MODE_FAST)
    {
        obj->speed_Hz = obj->speedEST_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleEST_rad;
            obj->motorState = MOTOR_CL_RUNNING;
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;

            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_HALL)
    {
        obj->speed_Hz = obj->speedHall_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleHall_rad;
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleHall_rad;
            obj->enableSpeedCtrl = false;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            obj->motorState = MOTOR_CL_RUNNING;
            PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == false))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = false;

            obj->stateRunTimeCnt++;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST && MOTOR1_HALL

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);
    obj->speed_Hz = obj->speedPLL_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->anglePLL_rad;

        ESMO_updateKslide(obj->esmoHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(fabsf(obj->speed_int_Hz) >= obj->speedForce_Hz)
        {
            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->motorState = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
            }
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ESMO

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ENC)
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speed_Hz = obj->speedENC_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleENC_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ENC
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#elif defined(MOTOR1_FAST)
    // sensorless-FOC
    MATH_Vec2 phasor;

#if ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT))
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // ((DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT))

#if defined(MOTOR1_VOLRECT)
    VOLREC_run(obj->volrecHandle, obj->adcData.VdcBus_V,
               &(obj->pwmData.Vabc_pu), &(obj->estInputData.Vab_V));
#else  // !MOTOR1_VOLRECT
    // remove offsets
    obj->adcData.V_V.value[0] -=
            obj->adcData.offset_V_sf.value[0] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[1] -=
            obj->adcData.offset_V_sf.value[1] * obj->adcData.VdcBus_V;

    obj->adcData.V_V.value[2] -=
            obj->adcData.offset_V_sf.value[2] * obj->adcData.VdcBus_V;

#if defined(MOTOR1_FILTERVS)
    // run first order filters for voltage sensing
    obj->adcVs_V.value[0] = FILTER_FO_run(obj->filterHandle_Vs[0], obj->adcData.V_V.value[0]);
    obj->adcVs_V.value[1] = FILTER_FO_run(obj->filterHandle_Vs[1], obj->adcData.V_V.value[1]);
    obj->adcVs_V.value[2] = FILTER_FO_run(obj->filterHandle_Vs[2], obj->adcData.V_V.value[2]);

    if(obj->flagEnableFilterVs == true)
    {
        obj->adcData.V_V.value[0] = obj->adcVs_V.value[0];
        obj->adcData.V_V.value[1] = obj->adcVs_V.value[1];
        obj->adcData.V_V.value[2] = obj->adcVs_V.value[2];
    }
#endif  // MOTOR1_FILTERVS

    // run Clarke transform on voltage
    CLARKE_run(obj->clarkeHandle_V,
               &obj->adcData.V_V, &obj->estInputData.Vab_V);
#endif  // MOTOR1_VOLRECT

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->estInputData.Iab_A);


    // store the input data into a buffer
    obj->estInputData.dcBus_V = obj->adcData.VdcBus_V;


    // configure the trajectory generator
    EST_run(obj->estHandle, &obj->estInputData, &obj->estOutputData);

    // compute angle with delay compensation
    obj->angleESTCOMP_rad =
            objUser->angleDelayed_sf_sec * obj->estOutputData.fm_lp_rps;

    obj->angleEST_rad =
            MATH_incrAngle(obj->estOutputData.angle_rad, obj->angleESTCOMP_rad);

    obj->speedEST_Hz = EST_getFm_lp_Hz(obj->estHandle);
    obj->speed_Hz = obj->speedEST_Hz;


    if(((EST_isMotorIdentified(obj->estHandle) == false) ||
            (EST_getState(obj->estHandle) == EST_STATE_RS)) &&
            (EST_isEnabled(obj->estHandle) == true))
    {
        obj->Idq_out_A.value[0] = 0.0f;
        obj->motorState = MOTOR_CTRL_RUN;

        // run identification or Rs Recalibration
        // setup the trajectory generator
        EST_setupTrajState(obj->estHandle,
                           obj->Idq_out_A.value[1],
                           obj->speedRef_Hz,
                           0.0);

        // run the trajectories
        EST_runTraj(obj->estHandle);

        obj->IdRated_A = EST_getIntValue_Id_A(obj->estHandle);

        // store the input data into a buffer
        obj->estInputData.speed_ref_Hz = EST_getIntValue_spd_Hz(obj->estHandle);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;

        obj->enableSpeedCtrl = EST_doSpeedCtrl(obj->estHandle);
        obj->enableCurrentCtrl = EST_doCurrentCtrl(obj->estHandle);
    }
    else if(obj->flagMotorIdentified == true)   // Normal Running
    {
        if(obj->flagRunIdentAndOnLine == true)
        {
            obj->counterTrajSpeed++;

            if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
            {
                // clear counter
                obj->counterTrajSpeed = 0;

                // run a trajectory for speed reference,
                // so the reference changes with a ramp instead of a step
                TRAJ_run(obj->trajHandle_spd);
            }

            obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
            obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;

            // get Id reference for Rs OnLine
            obj->IdRated_A = EST_getIdRated_A(obj->estHandle);
        }
        else
        {
            obj->enableSpeedCtrl = false;
            obj->enableCurrentCtrl = false;
        }

        obj->estInputData.speed_ref_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
        obj->speed_int_Hz = obj->estInputData.speed_ref_Hz;
    }

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->oneOverDcBus_invV = obj->estOutputData.oneOverDcBus_invV;

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleEST_rad;
        obj->motorState = MOTOR_CL_RUNNING;
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;
            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);
            PI_setUi(obj->piHandle_spd, obj->alignCurrent_A);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        obj->angleFOC_rad = obj->angleEST_rad;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#ifdef MOTOR1_VOLRECT
    if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(obj->speedAbs_Hz >= obj->speedForce_Hz)
        {
            obj->motorState = MOTOR_CL_RUNNING;

            EST_setAngle_rad(obj->estHandle, obj->angleFOC_rad);

            PI_setUi(obj->piHandle_spd, obj->startCurrent_A * 0.5f);
        }
    }
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->estInputData.Iab_A),
             (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_FAST
//------------------------------------------------------------------------------

#elif defined(MOTOR1_HALL)
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // run Clarke transform on current
    CLARKE_run(obj->clarkeHandle_I, &obj->adcData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    HALL_setTimeStamp(obj->hallHandle, HAL_calcCAPCount(obj->halMtrHandle));
    HALL_run(obj->hallHandle, obj->speed_int_Hz);
    obj->angleHall_rad = HALL_getAngle_rad(obj->hallHandle);
    obj->speedHall_Hz = HALL_getSpeed_Hz(obj->hallHandle);

    obj->speed_Hz = obj->speedHall_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        obj->motorState = MOTOR_CL_RUNNING;
        PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = false;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;

                obj->IsRef_A = 0.0f;
                obj->Idq_out_A.value[0] = 0.0f;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_HALL

//------------------------------------------------------------------------------
#else   // No Any Estimator
#error Not select a right estimator for this project
#endif  // (ESTIMATOR)

#if defined(MOTOR1_RPM_CMD)
    // convert the feedback speed to rpm
    obj->speed_rpm = obj->speed_Hz * obj->hz2Rpm_sf;

    if(obj->flagCmdRpmOrHz == false)
    {
        obj->speedRef_rpm = obj->speedRef_Hz * obj->hz2Rpm_sf;
    }
    else
    {
        obj->speedRef_Hz = obj->speedRef_rpm * obj->rpm2Hz_sf;
    }
#endif  // MOTOR1_RPM_CMD

//---------- Common Speed and Current Loop for all observers -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

#if defined(SFRA_ENABLE)

    if(sfraCollectStart == true)
    {
        collectSFRA(motorHandle_M1);    // Collect noise feedback from loop
    }

    //  SFRA injection
    injectSFRA();                   // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = true;       // enable SFRA data collection
#endif  // SFRA_ENABLE

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        obj->speed_reg_Hz = obj->speed_Hz;

        if(obj->enableSpeedCtrl == true)
        {
            obj->Is_ffwd_A = 0.0f;


#if defined(SFRA_ENABLE)
            PI_run_series(obj->piHandle_spd,
                   (obj->speed_int_Hz + sfraNoiseSpd), obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#else     // !SFRA_ENABLE
            PI_run_series(obj->piHandle_spd,
                   obj->speed_int_Hz, obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#endif  // !SFRA_ENABLE
        }
        else if((obj->motorState >= MOTOR_CL_RUNNING) &&
                (obj->flagMotorIdentified == true))
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }
    }
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == true) || (obj->flagEnableMTPA == true))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableMTPA == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad = MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A;
#endif  // STEP_RP_EN

#if defined(MOTOR1_FAST)
    // update Id reference for Rs OnLine
    EST_updateId_ref_A(obj->estHandle, &obj->IdqRef_A.value[0]);
#endif  // MOTOR1_FAST

#if !defined(STEP_RP_EN)
#if defined(MOTOR1_VIBCOMP)
    // get the Iq reference value plus vibration compensation
    obj->IdqRef_A.value[1] = Idq_out_A.value[1] +
            VIB_COMP_inline_run(vibCompHandle, angleFOCM1_rad, Idq_in_A.value[1]);
#else
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
#endif  // MOTOR1_VIBCOMP
#else   // STEP_RP_EN
    if(GRAPH_getBufferMode(&stepRPVars) != GRAPH_STEP_RP_TORQUE)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
    }
    else
    {
        PI_setUi(obj->piHandle_spd, obj->IdqRef_A.value[1]);
    }
#endif  // STEP_RP_EN


#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == true)
    {
        obj->Vdq_ffwd_V.value[0] = 0.0f;
        obj->Vdq_ffwd_V.value[1] = 0.0f;




        // Maximum voltage output
        obj->VsMax_V = objUser->maxVsMag_pu * obj->adcData.VdcBus_V;
        PI_setMinMax(obj->piHandle_Id, -obj->VsMax_V, obj->VsMax_V);

#if defined(SFRA_ENABLE)
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      (obj->IdqRef_A.value[0] + sfraNoiseId), obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, (obj->IdqRef_A.value[1] + sfraNoiseIq),
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);

#else     // !SFRA_ENABLE
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, obj->IdqRef_A.value[1],
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#endif  // !SFRA_ENABLE


#if defined(MOTOR1_FAST)
        // set the Id reference value in the estimator
        EST_setId_ref_A(obj->estHandle, obj->IdqRef_A.value[0]);
        EST_setIq_ref_A(obj->estHandle, obj->IdqRef_A.value[1]);
#endif  // MOTOR1_FAST
    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(PHASE_ADJ_EN)
    if(obj->flagPhaseAdjustEnable == true)
    {
        obj->angleFOCAdj_rad =
                MATH_incrAngle(obj->angleFOC_rad, obj->anglePhaseAdj_rad);

        // compute the sin/cos phasor
        phasor.value[0] = __cos(obj->angleFOCAdj_rad);
        phasor.value[1] = __sin(obj->angleFOCAdj_rad);
    }
    else
    {
        obj->angleFOCAdj_rad = obj->angleFOC_rad;
    }

#if defined(MOTOR1_FAST)
    EST_getEab_V(obj->estHandle, &obj->Eab_V);
#endif  // MOTOR1_FAST
#endif  // PHASE_ADJ_EN

    // set the phasor in the inverse Park transform
    IPARK_setPhasor(obj->iparkHandle_V, &phasor);

    // run the inverse Park module
    IPARK_run(obj->iparkHandle_V,
              &obj->Vdq_out_V, &obj->Vab_out_V);

    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle,
                obj->oneOverDcBus_invV);

    // run the space vector generator (SVGEN) module
    SVGEN_run(obj->svgenHandle,
              &obj->Vab_out_V, &(obj->pwmData.Vabc_pu));

#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }

#if defined(MOTOR1_DCLINKSS)
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    // revise PWM compare(CMPA/B) values for shifting switching pattern
    // and, update SOC trigger point
    HAL_runSingleShuntCompensation(obj->halMtrHandle, obj->dclinkHandle,
                         &obj->Vab_out_V, &obj->pwmData, obj->adcData.VdcBus_V);
#else   // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    else
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgencurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgencurrentHandle);
    obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(obj->halMtrHandle,
                   &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);
//------------------------------------------------------------------------------
#endif  // !MOTOR1_ISBLDC

#if defined(BENCHMARK_TEST)
    recordSpeedData(motorHandle_M1);
#endif  // BENCHMARK_TEST

#if defined(STEP_RP_EN)
    // Collect predefined data into arrays
    GRAPH_updateBuffer(&stepRPVars);
#endif  // STEP_RP_EN




#if defined(EPWMDAC_MODE)
    // connect inputs of the PWMDAC module.
    HAL_writePWMDACData(halHandle, &pwmDACData);
#endif  // EPWMDAC_MODE

#if defined(DATALOGF2_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
    }
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
        HAL_trigDMAforDLOG(halHandle, 2);
        HAL_trigDMAforDLOG(halHandle, 3);
    }
#endif  // DATALOGF4_EN || DATALOGF2_EN

#if defined(DAC128S_ENABLE)
#if defined(_F280013x) || defined(_F280015x)
#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
    if(HAL_getSelectionSPICS(motorHandle_M1->halMtrHandle) == SPI_CS_DAC)
    {
        // Write the variables data value to DAC128S085
        DAC128S_writeData(dac128sHandle);
    }
#else   // !(BSXL8323RS_REVA | BSXL8353RS_REVA | BSXL8316RT_REVA)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif   // !(BSXL8323RS_REVA | BSXL8353RS_REVA | BSXL8316RT_REVA)
#elif defined(_F28002x) || defined(_F28003x)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif  // !(F280013x | F280015x | F28002x | F28003x)
#endif  // DAC128S_ENABLE


    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
