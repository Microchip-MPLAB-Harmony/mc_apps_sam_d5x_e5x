/**
 * @file mc_rotor_position_parameters.c
 *
 * @brief 
 *    Rotor Position Parameters
 *
 * @Company 
 *    Microchip Technology Inc.
 *
 * @Summary
 *    Source file containing data structures and function prototypes for rotor position.
 *
 * @Description
 *    This file defines the data structures and function prototypes used for
 *    managing rotor position in motor control applications.
 */

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "mc_rotor_position_parameters.h"

/*******************************************************************************
 Configuration parameters
 *******************************************************************************/

/*******************************************************************************
 Private data-types
 *******************************************************************************/
 
/*******************************************************************************
 Constants
 *******************************************************************************/

/*******************************************************************************
 Function macros
 *******************************************************************************/

/*******************************************************************************
 Private variables
 *******************************************************************************/
tmcRpe_Parameters_s  mcRpeI_Parameters_gds;
tmcTune_HFITuning_s  mcRpeI_TuningSettings_gds;

/*******************************************************************************
 Interface Variables
 *******************************************************************************/
/** 
 * @brief Set rotor position estimation parameters
 *
 * @details 
 *   This function sets the parameters for rotor position estimation module.
 *
 * @param[in] pParameters Pointer to the parameters structure.
 * @return None
 */
void mcRpeI_ParametersSet(tmcRpe_Parameters_s * const pParameters )
{
    /** Soft switch parameters */
    pParameters->lowerThreshold = (float32_t)0.1;
    pParameters->upperThreshold = (float32_t)0.2;

    pParameters->pMotorParameters = &mcMotI_PMSM_gds;

    /** HFI parameters */
    pParameters->hfiParameters.pulseAmplitude = (float32_t)5;

    /** IPC parameters */
    pParameters->ipcParameters.pulseAmplitude = (float32_t)5;
    pParameters->ipcParameters.pulseDuty = (float32_t)0.0005;
    pParameters->ipcParameters.pulseDuration = (float32_t)0.005;

    /** HFI Tracking loop parameters */
    pParameters->hfiTrackingLoop.foInHertz = (float32_t)20;
    pParameters->hfiTrackingLoop.Keps = (float32_t)1.2;

    /** BEMF Tracking loop parameters */
    pParameters->bemfTrackingLoop.foInHertz = (float32_t)20;
    pParameters->bemfTrackingLoop.Keps = (float32_t)1;

    /** Sampling time */
    pParameters->dt = (float32_t)(0.00005);

    /** Mechanical Gear Ratio */
    pParameters->gearRatio = (uint8_t)2;
}
