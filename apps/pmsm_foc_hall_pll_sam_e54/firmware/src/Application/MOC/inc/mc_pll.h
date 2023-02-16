/*******************************************************************************
 Board services Header file 

  Company:
    Microchip Technology Inc.

  File Name:
    board_services.c

  Summary:
    This file contains all the functions related to board services

  Description:
    This file contains all the functions related to board services
 
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef MCPLL_H_    // Guards against multiple inclusion
#define MCPLL_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_generic_libraries.h"
#include "mc_data_manager.h"


/*******************************************************************************
 * Constants
 ******************************************************************************/

/*
 * Motor per phase resistance 
 */
 #define CONFIG_MotorPerPhaseResistanceInOhm  (float)MOTOR_PER_PHASE_RESISTANCE
/*
 * Number of rotor position module instances 
 */
#define  CONFIG_MotorPerPhaseInductanceInHenry  (float)MOTOR_PER_PHASE_INDUCTANCE
/*
 * Number of rotor position module instances 
 */
#define CONFIG_MotorBemfConstInVrmsPerKrpm  MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH

/*
 * Fast loop time in seconds 
 */
#define CONFIG_AlgorithmCycleTimeInSec   (float)(1.0f/(float)PWM_FREQ ) 
 

/*
 * Open loop end speed in RPM
 */
#define CONFIG_SpeedThresholdForCloseLoopInRpm       (float)(500.0f)


/*
 * Q-axis back EMF filter parameter 
 */
#define CONFIG_EdFilterParameter      (float)(0.5)

/*
 * D-axis back EMF filter parameter 
 */
#define CONFIG_EqFilterParameter  (float)(0.5)

/*
 * Open loop end speed in RPM
 */
#define CONFIG_SpeedFilterParameter        (float)(0.0053)

/*******************************************************************************
 User defined data-types
 *******************************************************************************/

typedef struct
{
    volatile float * ialpha;
    volatile float * ibeta;
    volatile float * ualpha;
    volatile float * ubeta;
    volatile float * umax;
}tmcPll_InputPorts_s;

typedef struct
{
    float  * elecAngle;
    float  * elecSpeed;
    float  * backEMF;
}tmcPll_OutputPorts_s;

typedef struct
{
    float  Rs;
    float  Ls;
    float  Ke;
    float Wrmin;
    float Ts;
    float EdqFilterBandwidth;
    float WrFilterBandwidth;
}tmcPll_UserParameters_s;

typedef struct
{
    /* Instance identifier */
    uint8_t Id;
    
    /* Input ports */
    tmcPll_InputPorts_s inPort;
    
    /* Output ports */
    tmcPll_OutputPorts_s outPort;
    
    /* User Parameters */
    tmcPll_UserParameters_s userParam;
    
}tmcPll_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcPll_ModuleData_s mcPllI_ModuleData_gds;

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
__STATIC_INLINE void mcPllI_InputPortsSet( tmcPll_InputPorts_s * const pInputPort )
{
    pInputPort->ualpha = &mcMocI_AlphaAxisVoltage_gdf32;
    pInputPort->ubeta = &mcMocI_BetaAxisVoltage_gdf32;
    pInputPort->umax     = &mcVolI_UacPeak_gdf32;
    pInputPort->ialpha  = &mcMocI_AlphaAxisCurrent_gdf32;
    pInputPort->ibeta  = &mcMocI_BetaAxisCurrent_gdf32;
}

__STATIC_INLINE void mcPllI_OutputPortsSet( tmcPll_OutputPorts_s * const pOutputPort )
{
    pOutputPort->elecAngle = &mcPllI_ElectricalAngle_gdf32;
    pOutputPort->elecSpeed = &mcPllI_ElectricalSpeed_gdf32;
}

__STATIC_INLINE void mcPllI_UserParametersSet( tmcPll_UserParameters_s * const pUserParam )
{
    pUserParam->Ke = CONFIG_MotorBemfConstInVrmsPerKrpm;
    pUserParam->Ls = CONFIG_MotorPerPhaseInductanceInHenry;
    pUserParam->Rs = CONFIG_MotorPerPhaseResistanceInOhm;
    pUserParam->Ts = CONFIG_AlgorithmCycleTimeInSec;
    pUserParam->Wrmin = CONFIG_SpeedThresholdForCloseLoopInRpm;
    pUserParam->WrFilterBandwidth = CONFIG_SpeedFilterParameter;
    pUserParam->EdqFilterBandwidth = CONFIG_EdFilterParameter;
}

/*******************************************************************************
 Interface functions 
 *******************************************************************************/

/*! \brief Rotor position calculation reset
 * 
 * Details.
 * Rotor position calculation reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcPllI_PosCalInit(tmcPll_ModuleData_s * const module);


/*! \brief Rotor position calculation 
 * 
 * Details.
 * Rotor position calculation 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcPllI_PosCalRun( tmcPll_ModuleData_s * const module);

/*! \brief Rotor position calculation reset
 * 
 * Details.
 * Rotor position calculation reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcPllI_PosCalReset(tmcPll_ModuleData_s * const module);

#endif //MCRPO_H_

/**
 End of File
*/