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


//! \file   /solutions/universal_motorcontrol_lab/common/source/sys_main.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM boards
//!
//
// dcsm_security_tool guidance refer to: http://www.ti.com/lit/pdf/spracp8
//
//

// include the related header files
//
#include "user.h"
#include "sys_settings.h"
#include "sys_main.h"


volatile SYSTEM_Vars_t systemVars;
#pragma DATA_SECTION(systemVars,"sys_data");

#ifdef CPUTIME_ENABLE
// define CPU time for performance test
CPU_TIME_Obj     cpuTime;
CPU_TIME_Handle  cpuTimeHandle;
#pragma DATA_SECTION(cpuTime,"sys_data");
#pragma DATA_SECTION(cpuTimeHandle,"sys_data");
#endif  // CPUTIME_ENABLE


#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
HAL_PWMDACData_t pwmDACData;
#pragma DATA_SECTION(pwmDACData,"sys_data");
  // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");

#define DAC_SCALE_SET       (4096.0f)     // 12bit
#endif  // DAC128S_ENABLE



#if defined(SFRA_ENABLE)
float32_t   sfraNoiseId;
float32_t   sfraNoiseIq;
float32_t   sfraNoiseSpd;
float32_t   sfraNoiseOut;
float32_t   sfraNoiseFdb;
SFRA_TEST_e sfraTestLoop;        //speedLoop;
bool        sfraCollectStart;

#pragma DATA_SECTION(sfraNoiseId, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseIq, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseSpd, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseOut, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseFdb, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraTestLoop, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraCollectStart, "SFRA_F32_Data");
#endif  // SFRA_ENABLE

// **************************************************************************
// the functions
// !!! Please make sure that you had gone through the user guide, and follow the
// !!! guide to set up the kit and load the right code
void main(void)
{

    // Clear memory for system and controller
    // The variables must be assigned to these sector if need to be cleared to zero
    HAL_clearDataRAM((void *)loadStart_est_data, (uint16_t)loadSize_est_data);
    HAL_clearDataRAM((void *)loadStart_user_data, (uint16_t)loadSize_user_data);
    HAL_clearDataRAM((void *)loadStart_hal_data, (uint16_t)loadSize_hal_data);
    HAL_clearDataRAM((void *)loadStart_foc_data, (uint16_t)loadSize_foc_data);
    HAL_clearDataRAM((void *)loadStart_sys_data, (uint16_t)loadSize_sys_data);
    HAL_clearDataRAM((void *)loadStart_vibc_data, (uint16_t)loadSize_vibc_data);
    HAL_clearDataRAM((void *)loadStart_datalog_data, (uint16_t)loadSize_datalog_data);
    HAL_clearDataRAM((void *)loadStart_SFRA_F32_Data, (uint16_t)loadSize_SFRA_F32_Data);

#if defined(SYSCONFIG_EN)
    systemVars.projectConfig = PRJ_DEV_SYSCONFIG;
#else
    systemVars.projectConfig = PRJ_NON_SYSCONFIG;
#endif  // SYSCONFIG_EN

#if defined(HVMTRPFC_REV1P1)
    systemVars.boardKit = BOARD_HVMTRPFC_REV1P1;    // HVMTRPFC_REV1P1
#elif defined(DRV8329AEVM_REVA)
    systemVars.boardKit = BOARD_DRV8329AEVM_REVA;    // DRV8329AEVM_REVA
#elif defined(BSXL8323RS_REVA)
    systemVars.boardKit = BOARD_BSXL8323RS_REVA;    // BSXL8323RS_REVA
#elif defined(BSXL8323RH_REVB)
    systemVars.boardKit = BOARD_BSXL8323RH_REVB;    // BSXL8323RH_REVB
#elif defined(BSXL8353RS_REVA)
    systemVars.boardKit = BOARD_BSXL8353RS_REVA;    // BSXL8353RS_REVA
#elif defined(BSXL3PHGAN_REVA)
    systemVars.boardKit = BOARD_BSXL3PHGAN_REVA;    // BSXL3PHGAN_REVA
#elif defined(BSXL8316RT_REVA)
    systemVars.boardKit = BOARD_BSXL8316RT_REVA;    // BSXL8316RT_REVA
#else
#error Not select a right board for this project
#endif

#if defined(MOTOR1_ISBLDC) && (defined(MOTOR1_FAST) || \
    defined(MOTOR1_ESMO) || defined(MOTOR1_ENC) || defined(MOTOR1_HALL))
#error ISBLDC can't work with other estimaor simultaneously
#elif defined(MOTOR1_ENC) && defined(MOTOR1_HALL)
#error Can't support ENC and HALL simultaneously
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_HALL)
#error Can't support ESMO and HALL simultaneously
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_FAST_ENC;     // the estimator is FAST and ENC
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    systemVars.estType = EST_TYPE_FAST_ESMO;    // the estimator is FAST and ESMO
#elif defined(MOTOR1_FAST) && defined(MOTOR1_RESL)
    systemVars.estType = EST_TYPE_FAST_RESL;    // the estimator is FAST and RESOLVER
#elif defined(MOTOR1_FAST) && defined(MOTOR1_PSCOS)
    systemVars.estType = EST_TYPE_FAST_PSCOS;    // the estimator is FAST and SIN/COS Encoder
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    systemVars.estType = EST_TYPE_FAST_HALL;    // the estimator is FAST and HALL
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ESMO_ENC;     // the estimator is ESMO and ENC
#elif defined(MOTOR1_FAST)
    systemVars.estType = EST_TYPE_FAST;         // the estimator is only FAST
#elif defined(MOTOR1_ESMO)
    systemVars.estType = EST_TYPE_ESMO;         // the estimator is only ESMO
#elif defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ENC;          // the estimator is only ENC
#elif defined(MOTOR1_HALL)
    systemVars.estType = EST_TYPE_HALL;         // the estimator is only HALL
#elif defined(MOTOR1_ISBLDC)
    systemVars.estType = EST_TYPE_ISBLDC;       // the estimator is only ISBLDC
#else
#error Not select a right estimator for this project
#endif  // MOTOR1_FAST->MOTOR1_ENC

#if defined(MOTOR1_FAST)
    systemVars.estLibVersion = EST_getFASTVersion();   // gets FAST version
#endif  // MOTOR1_FAST


#if defined(MOTOR1_DCLINKSS) || defined(MOTOR1_ISBLDC)
    systemVars.currentSenseType = CURSEN_TYPE_SINGLE_SHUNT;
#elif defined(BSXL3PHGAN_REVA)
    systemVars.currentSenseType = CURSEN_TYPE_INLINE_SHUNT;
#else
    systemVars.currentSenseType = CURSEN_TYPE_THREE_SHUNT;
#endif  // Current Sense Type

#if defined(MOTOR1_HALL) && defined(_F280013x)
#error HALL sensors based FOC is not supported on this device
#endif  // MOTOR1_HALL & _F280013x

#if defined(MOTOR1_DCLINKSS) && defined(DRV8329AEVM_REVA)
// This kit supports single shunt
#elif defined(MOTOR1_DCLINKSS)
#error This kit doesn't support single shunt
// Only modificated BSXL8323RS_REVA and BSXL8323RS_REVA support single shunt
#endif  // MOTOR1_DCLINKSS

#if defined(MOTOR1_ISBLDC) && defined(DRV8329AEVM_REVA)
// This kit supports IS-BLDC
#elif defined(MOTOR1_ISBLDC)
#error This kit doesn't support InstaSPIN-BLDC
// Only modificated BSXL8323RS_REVA and BSXL8323RS_REVA support instaspin-bldc
#endif  // MOTOR1_ISBLDC

#if defined(DATALOGF2_EN) && defined(STEP_RP_EN)
#error DATALOG and GRAPH_STEP_RESPONSE can't be used simultaneously on this device
#endif  // DATALOGF2_EN && STEP_RP_EN

#if defined(MOTOR1_ISBLDC) && defined(MOTOR1_DCLINKSS)
#error Don't need to enable single shunt pre-define name if use instaspin-bldc
#endif  // MOTOR1_ISBLDC & MOTOR1_DCLINKSS

#if (defined(MOTOR1_SSIPD) || defined(MOTOR1_OVM)) && defined(MOTOR1_DCLINKSS)
#error Don't enable SSIPD and OVM if enable single shunt
#endif  // (MOTOR1_SSIPD | MOTOR1_OVM) & (MOTOR1_DCLINKSS

#if defined(MOTOR1_ISBLDC) && (defined(MOTOR1_OVM) || defined(MOTOR1_FWC) || \
        defined(MOTOR1_MTPA) || defined(MOTOR1_SSIPD))
#error Don't need to enable these functions if use instaspin-bldc
#endif  // MOTOR1_ISBLDC & (MOTOR1_OVM | MOTOR1_FWC | MOTOR1_MTPA | MOTOR1_SSIPD)

// ** above codes are only for checking the settings, not occupy the memory

    // Initialize device clock and peripherals
    Device_init();                  // call the function in device.c

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();              // call the function in device.c

    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();         // call the function in driverlib.lib

    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();    // call the function in driverlib.lib

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC/PWM interrupts for control
    // enable interrupts to trig DMA
    HAL_enableCtrlInts(halHandle);

    // set the control parameters for motor 1

    motorHandle_M1 = (MOTOR_Handle)(&motorVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.flagEnableRunAndIdentify = false;

    motorVars_M1.speedRef_Hz = 60.0f;       // Hz
    motorVars_M1.speedRef_rpm = 600.0f;     // rpm

    // false - enables identification, true - disables identification
    userParams_M1.flag_bypassMotorId = true;  //    false;   //

    initMotor1Handles(motorHandle_M1);
    initMotor1CtrlParameters(motorHandle_M1);

    // setup the GPIOs
    HAL_setupGPIOs(halHandle);

    // set up gate driver after completed GPIO configuration
    motorVars_M1.faultMtrNow.bit.gateDriver =
            HAL_MTR_setGateDriver(motorHandle_M1->halMtrHandle);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

#if defined(CMD_POT_EN)
    setExtCmdPotParams(motorHandle_M1);
#endif  // CMD_POT_EN

#if defined(MOTOR1_HALL) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
    setExtCmdCapParams(motorHandle_M1);
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
    setExtCmdSwitchParams(motorHandle_M1);
#endif  //CMD_SWITCH_EN

#ifdef CPUTIME_ENABLE
    // initialize the CPU usage module
    cpuTimeHandle = CPU_TIME_init(&cpuTime, sizeof(cpuTime));
    CPU_TIME_reset(cpuTimeHandle);
    CPU_TIME_setCtrlPeriod(cpuTimeHandle, HAL_getTimeBasePeriod(motorHandle_M1->halMtrHandle));
#endif  // CPUTIME_ENABLE


#if defined(EPWMDAC_MODE)
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &motorVars_M1.angleEST_rad;             // PWMDAC1
//    pwmDACData.ptrData[0] = &motorVars_M1.anglePLL_rad;             // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleENC_rad;             // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleHall_rad;            // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleGen_rad;             // PWMDAC2
    pwmDACData.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];     // PWMDAC2
//    pwmDACData.ptrData[1] = &motorVars_M1.speedAbs_Hz;            // PWMDAC2
//    pwmDACData.ptrData[1] = &motorVars_M1.speedAbs_Hz;            // PWMDAC3
//    pwmDACData.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];     // PWMDAC3
//    pwmDACData.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];     // PWMDAC4

    pwmDACData.offset[0] = 0.5f;    // PWMDAC1
    pwmDACData.offset[1] = 0.5f;    // PWMDAC2
//    pwmDACData.offset[1] = 0.0f;    // PWMDAC2
//    pwmDACData.offset[1] = 0.0f;    // PWMDAC3
//    pwmDACData.offset[2] = 0.5f;    // PWMDAC3
//    pwmDACData.offset[3] = 0.5f;    // PWMDAC4

    pwmDACData.gain[0] = 1.0f / MATH_TWO_PI;                          // PWMDAC1
//    pwmDACData.gain[1] = 1.0f / MATH_TWO_PI;                        // PWMDAC2
    pwmDACData.gain[1] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;     // PWMDAC2
//    pwmDACData.gain[1] = 1.0f / USER_MOTOR1_FREQ_MAX_Hz;              // PWMDAC2
//    pwmDACData.gain[2] = 1.0f / USER_MOTOR1_FREQ_MAX_Hz;              // PWMDAC3
//    pwmDACData.gain[2] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;   // PWMDAC3
//    pwmDACData.gain[3] = 2.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;     // PWMDAC4
#endif  // EPWMDAC_MODE

#if defined(DATALOGF2_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.speed_Hz;
//    datalogObj->iptr[0] = &resl_M1.sin_os;
//    datalogObj->iptr[1] = &resl_M1.cos_os;
//    datalogObj->iptr[0] = &isbldc_M1.bemfInt;
//    datalogObj->iptr[1] = &isbldc_M1.VintPhase;
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);
    HAL_setupDMAforDLOG(halHandle, 2, &datalogBuff3[0], &datalogBuff3[1]);
    HAL_setupDMAforDLOG(halHandle, 3, &datalogBuff4[0], &datalogBuff4[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[1];
    datalogObj->iptr[2] = &motorVars_M1.adcData.I_A.value[2];
    datalogObj->iptr[3] = &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
    datalogObj->iptr[2] = &motorVars_M1.adcData.V_V.value[2];
    datalogObj->iptr[3] = &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.angleEST_rad;
    datalogObj->iptr[2] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[3] = &motorVars_M1.adcData.V_V.value[0];
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#endif  // DATALOGI4_EN


#if defined(DAC128S_ENABLE)
    // initialize the DAC128S
    dac128sHandle = DAC128S_init(&dac128s);

#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
#if defined(_F280013x) || defined(_F280015x)    // DRV and DAC share SPIA
    // switch the SPI_STE pin for DRV device
    HAL_switchSPICS(motorHandle_M1->halMtrHandle);

    DEVICE_DELAY_US(1.0f);      // delay 1.0us

    // setup SPI for DAC128S
//    DAC128S_setupSPI(dac128sHandle);
    DAC128S_setupSPIBR(dac128sHandle, DACS_SPI_BITRATE);
#else   // !(_F280013x | F280015x)
    // setup SPI for DAC128S
    DAC128S_setupSPI(dac128sHandle);
#endif  // !(F280013x | F280015x)
#else   // !(BSXL8323RS_REVA | BSXL8353RS_REVA | BSXL8316RT_REVA)
    // setup SPI for DAC128S
    DAC128S_setupSPI(dac128sHandle);
#endif  // !(BSXL8323RS_REVA | BSXL8353RS_REVA | BSXL8316RT_REVA)



// The following settings are for output the values of different variables
// in each build level for debug. The User can select one of these groups in
// different build level as commented note

// DAC_LEVEL4_ISBLDC, DAC_LEVEL4_DCLINK, DAC_LEVEL4_VIBCOMP,
// DAC_LEVEL2_MOTOR1_VS, DAC_LEVEL2_MOTOR1_IS, DAC_LEVEL_MOTOR1_FAST,
// DAC_LEVEL4_FAST_ESMO, DAC_LEVEL4_FAST_ENC, DAC_LEVEL4_FAST_HALL
// DAC_LEVEL4_FAST, DAC_LEVEL4_FAST_ENC, DAC_LEVEL4_ENC, DAC_LEVEL4_HALL
// DAC_LEVEL4_PHADJ,

#if defined(MOTOR1_ISBLDC)
#define DAC_LEVEL4_ISBLDC               // define the DAC level
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
#define DAC_LEVEL4_FAST_ESMO            // define the DAC level
#else   // !MOTOR1_RESL && !MOTOR1_PSCOS && !MOTOR1_PSCOS
#define DAC_LEVEL_MOTOR1_FAST            // define the DAC level
#endif      // !MOTOR1_RESL && !MOTOR1_PSCOS

#if defined(DAC_LEVEL4_ISBLDC)
    dac128s.ptrData[0] = &isbldc_M1.VintPhase;              // CH_A
    dac128s.ptrData[1] = &isbldc_M1.bemfInt;                // CH_B
    dac128s.ptrData[2] = &isbldc_M1.Vabcn.value[0];         // CH_C
    dac128s.ptrData[3] = &isbldc_M1.Vabcn.value[1];         // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[1] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_DCLINK)
    // Build_Level_2, verify the current sampling value
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcIs_A.value[0];            // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST_ESMO)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.anglePLL_rad;                // CH_B
#if defined(ESMO_DEBUG)
    dac128s.ptrData[2] = &esmo_M1.thetaElec_rad;                    // CH_C
#else   //!ESMO_DEBUG
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
#endif  //!ESMO_DEBUG
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
#if defined(ESMO_DEBUG)
    dac128s.gain[2] = DAC_SCALE_SET / MATH_TWO_PI;
#else   //!ESMO_DEBUG
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
#endif  //!ESMO_DEBUG
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL_MOTOR1_FAST)
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST_ESMO)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_B
    dac128s.ptrData[1] = &motorVars_M1.anglePLL_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_D
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_E

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST_ENC)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.angleENC_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST_HALL)
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.angleHall_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_FAST)
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL4_PHADJ)
    dac128s.ptrData[0] = &motorVars_M1.Vab_out_V.value[0];          // CH_A
    dac128s.ptrData[1] = &motorVars_M1.estInputData.Iab_A.value[0]; // CH_B
    dac128s.ptrData[2] = &motorVars_M1.Eab_V.value[0];              // CH_C
    dac128s.ptrData[3] = &motorVars_M1.Eab_V.value[1];             // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[1] = DAC_SCALE_SET * 4.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL3_MOTOR1_FAST)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleGen_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[0];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[2] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL2_MOTOR1_IS)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_LEVEL2_MOTOR1_VS)
    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleEST_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.V_V.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.V_V.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)


    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE


#if defined(SFRA_ENABLE)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, USER_M1_ISR_FREQ_Hz);

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;
    sfraNoiseOut = 0.0f;
    sfraNoiseFdb = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = false;
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
    GRAPH_init(&stepRPVars,
               &motorVars_M1.speedRef_Hz, &motorVars_M1.speed_Hz,
               &motorVars_M1.IdqRef_A.value[0], &motorVars_M1.Idq_in_A.value[0],
               &motorVars_M1.IdqRef_A.value[1], &motorVars_M1.Idq_in_A.value[1]);
#endif  // STEP_RP_EN

    systemVars.flagEnableSystem = true;

#if defined(CMD_CAN_EN)
    // initialize the CANCOM
    initCANCOM(halHandle);

    motorVars_M1.cmdCAN.speedSet_Hz = 40.0f;

    motorVars_M1.cmdCAN.flagEnableCmd = false;
    motorVars_M1.cmdCAN.flagEnableSyncLead = false;
#endif // CMD_CAN_EN

    motorVars_M1.flagEnableOffsetCalc = true;

    // run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

#if defined(MOTOR1_RESL)
    motorVars_M1.flagEnableRESLAdcOffsetCalc = true;

    // run offset calibration for resolver 1
    runResolver1OffsetsCalculation(motorHandle_M1);
#endif  // MOTOR1_RESL


    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    motorVars_M1.flagInitializeDone = true;

    while(systemVars.flagEnableSystem == true)
    {
        // loop while the enable system flag is true
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            // toggle status LED on controller board
            systemVars.counterLEDC++;

            if(systemVars.counterLEDC > (uint16_t)(LED_BLINK_FREQ_Hz * 1000))
            {
                HAL_toggleGPIO(halHandle, HAL_GPIO_LED1C);     // Toggle on the LED

                systemVars.counterLEDC = 0;
            }

            if(motorVars_M1.motorState >= MOTOR_CL_RUNNING)
            {
                systemVars.timeWaitLEDB =
                        (uint16_t)(40000.0f / (fabsf(motorVars_M1.speed_Hz) + 20.0f));

                // toggle status LED on inverter board if have
                systemVars.counterLEDB++;

                if(systemVars.counterLEDB > systemVars.timeWaitLEDB)
                {
                    HAL_toggleGPIO(halHandle, HAL_GPIO_LED1B);     // Toggle on the LED

                    systemVars.counterLEDB = 0;
                }
            }
            else
            {
                HAL_setGPIOHigh(halHandle, HAL_GPIO_LED1B);     // Turn on the LED
            }

            systemVars.timerBase_1ms++;

            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
                    break;
                case 2:
                    calculateRMSData(motorHandle_M1);
                    break;
                case 3:
#if defined(MOTOR1_PI_TUNE)
                    // Tune the gains of the controllers
                    tuneControllerGains(motorHandle_M1);
#endif      // MOTOR1_PI_TUNE
                    break;
                case 4:     // calculate motor protection value
                    calcMotorOverCurrentThreshold(motorHandle_M1);
                    break;
                case 5:     // system control
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;
                    break;
            }

#if defined(CMD_CAN_EN)
            updateCANCmdFreq(motorHandle_M1);

            if((motorVars_M1.cmdCAN.flagEnableCmd == true) && (motorVars_M1.faultMtrUse.all == 0))
            {
                canComVars.flagCmdTxRun = motorVars_M1.cmdCAN.flagCmdRun;
                canComVars.speedSet_Hz = motorVars_M1.cmdCAN.speedSet_Hz;

                if(motorVars_M1.cmdCAN.flagEnableSyncLead == true)
                {
                    motorVars_M1.flagEnableRunAndIdentify = motorVars_M1.cmdCAN.flagCmdRun;
                    motorVars_M1.speedRef_Hz = motorVars_M1.cmdCAN.speedSet_Hz;
                }
                else
                {
                    motorVars_M1.flagEnableRunAndIdentify = canComVars.flagCmdRxRun;
                    motorVars_M1.speedRef_Hz = canComVars.speedRef_Hz;
                }
            }
#endif // CMD_CAN_EN

#if defined(CMD_POT_EN)
            updateExtCmdPotFreq(motorHandle_M1);
#endif  // CMD_POT_EN

#if defined(MOTOR1_HALL) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
            updateExtCmdCapFreq(motorHandle_M1,
                                HAL_calcCAPCount(motorHandle_M1->halMtrHandle));
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
            updateCmdSwitch(motorHandle_M1);
#endif  //CMD_SWITCH_EN

#if defined(SFRA_ENABLE)
            // SFRA test
            SFRA_F32_runBackgroundTask(&sfra1);
            SFRA_GUI_runSerialHostComms(&sfra1);
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
            // Generate Step response
            GRAPH_generateStepResponse(&stepRPVars);
#endif  // STEP_RP_EN

#ifdef CPUTIME_ENABLE
            CPU_TIME_calcCPUWidthRatio(cpuTimeHandle);
#endif  // CPUTIME_ENABLE

        }       // 1ms Timer

#if defined(CMD_SWITCH_EN)
        outputCmdState(motorHandle_M1);
#endif  //CMD_SWITCH_EN

        // runs control for motor 1
        runMotor1Control(motorHandle_M1);

        // Read/Write the registers of DRV device
#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
#if defined(_F280013x) || defined(_F280015x)    // DRV and DAC share SPIA
#if defined(DAC128S_ENABLE)
        if(HAL_getDRVFlagWR(motorHandle_M1->halMtrHandle) == true)
        {
            if(HAL_getSelectionSPICS(motorHandle_M1->halMtrHandle) != SPI_CS_DRV)
            {
                // switch the SPI_STE pin for DRV device
                HAL_switchSPICS(motorHandle_M1->halMtrHandle);

                DEVICE_DELAY_US(1.0f);      // delay 1.0us

                // setup the spi for drv8323/drv8353/drv8316
                HAL_setupSPI(motorHandle_M1->halMtrHandle);

                DEVICE_DELAY_US(1.0f);      // delay 1.0us
            }

            HAL_writeDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
            HAL_readDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
        }
        else if(HAL_getSelectionSPICS(motorHandle_M1->halMtrHandle) != SPI_CS_DAC)
        {
            // switch the SPI_STE pin for DRV device
            HAL_switchSPICS(motorHandle_M1->halMtrHandle);

            DEVICE_DELAY_US(1.0f);      // delay 1.0us

            // setup SPI for DAC128S
            DAC128S_setupSPIBR(dac128sHandle, DACS_SPI_BITRATE);

            DEVICE_DELAY_US(1.0f);      // delay 1.0us
        }
#else  // !DAC128S_ENABLE
        HAL_writeDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
        HAL_readDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
#endif  // !DAC128S_ENABLE
#elif defined(_F28002x) || defined(_F28003x)
        HAL_writeDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
        HAL_readDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
#else
#error This lab doesn't support thses devices, you need to change some files
#endif  // !(F280013x | F280015x | F28002x | F28003x)
#endif  // BSXL8323RS_REVA | BSXL8353RS_REVA | BSXL8316RT_REVA

    } // end of while() loop

    // disable the PWM
    HAL_disablePWM(motorHandle_M1->halMtrHandle);

} // end of main() function

//
//-- end of this file ----------------------------------------------------------
//
