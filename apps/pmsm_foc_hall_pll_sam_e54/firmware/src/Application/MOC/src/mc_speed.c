/*******************************************************************************
 Function coordinator source file 

  Company:
    Microchip Technology Inc.

  File Name:
    function_coordinator.c

  Summary:
    This file contains all the functions related to function coordination

  Description:
    This file contains implementation of function coordination
 
 *******************************************************************************/

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


/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include "mc_speed.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/
#define CONSTANT_RpmToRadPerSec    (float)(0.1047197551196)
#define CONSTANT_RpmToElecRadPerSec    (float)(CONSTANT_RpmToRadPerSec * CONFIG_PolePairs)

/*******************************************************************************
 Private data-types 
 *******************************************************************************/
typedef struct
{
    float absSpeFilt;
 }tmcSpe_StateVariables_s;

typedef struct
{
    float speFiltParam;
    float adcUnitsToSpeed;
    float minSpeedinRpm;
    float maxSpeedinRpm;
}tmcSpe_Parameters_s;

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcSpe_StateVariables_s mcSpe_StateVariables_mds;
static tmcSpe_Parameters_s mcSpe_Parameters_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcSpe_ModuleData_s mcSpeI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/
__STATIC_INLINE void mcSpe_SpeedRampControl( float * const pInput, const float ramp, const float Final )
{
    if( ( *pInput + ramp ) <= Final )
    {
        *pInput += ramp;
    }
    else if( ( *pInput - ramp ) >= Final )
    {
        *pInput -= ramp;
    }
    else 
    {
        *pInput = Final;
    }
}

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
/*! \brief Current measurement initialization
 * 
 * Details.
 * Current measurement initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcSpeI_SpeedMeasurementInit( tmcSpe_ModuleData_s * const module )
{
    tmcSpe_Parameters_s * pParam;
    pParam = &mcSpe_Parameters_mds;
     
    /* Set input ports */
    mcSpeI_InputPortsSet(&module->dInputPorts);
    
    /* Set output ports */
    mcSpeI_OutputPortsSet(&module->dOutputPorts);
    
    /* Set user parameters  */
    mcSpeI_UserParametersSet(&module->dUserParam);
    
    /* Calculate user dependent intermediate constant values */
    
    pParam->adcUnitsToSpeed = module->dUserParam.adcUnitsToSpeed;
    pParam->maxSpeedinRpm = module->dUserParam.maxSpeedinRpm * CONSTANT_RpmToElecRadPerSec;
    pParam->minSpeedinRpm = module->dUserParam.minSpeedinRpm * CONSTANT_RpmToElecRadPerSec;
    pParam->speFiltParam = module->dUserParam.speFiltParam;
}

/*! \brief Current measurement run
 * 
 * Details.
 * Current measurement run
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcSpeI_SpeedMeasurementRun(tmcSpe_ModuleData_s * const module)
{
    float absCmdSpeed;
    tmcSpe_InputPorts_s * pInput;
    tmcSpe_StateVariables_s * pState;
    tmcSpe_Parameters_s * pParam;
    
    pInput = &module->dInputPorts;
    pState = &mcSpe_StateVariables_mds;
    pParam = &mcSpe_Parameters_mds;
    
    if( 1u == *pInput->runCommand )
    {
        *module->dOutputPorts.runStatus  = 1u;
                
        /* Calculate command speed */
        absCmdSpeed =(float)(*pInput->speAdcInput) * pParam->adcUnitsToSpeed;
      
        /* Ramp control of reference speed  */
        mcSpe_SpeedRampControl(&pState->absSpeFilt, 10.0f,  absCmdSpeed );

        /* Impose upper and lower limits*/
        if( pParam->maxSpeedinRpm <  pState->absSpeFilt )
        {
             pState->absSpeFilt = pParam->maxSpeedinRpm;
        }
        else if( pParam->minSpeedinRpm >  pState->absSpeFilt )
        {
             pState->absSpeFilt = pParam->minSpeedinRpm;
        }
        else
        {
            /* Do nothing */
        }
        *module->dOutputPorts.cmdSpeed = (*pInput->directionFlag) * absCmdSpeed;
        *module->dOutputPorts.cmdSpeedFilt =  (*pInput->directionFlag) * pState->absSpeFilt;
    }
    else 
    {
        mcSpe_SpeedRampControl(&pState->absSpeFilt, 20.0f,  0.0f );
        
        if(fabsf(pState->absSpeFilt ) < 10.0f )
        {
             *module->dOutputPorts.runStatus  = 0u;
        }
    }
    
}

/*! \brief Current measurement reset
 * 
 * Details.
 * Current measurement reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcSpeI_SpeedMeasurementReset(tmcSpe_ModuleData_s * const module)
{
    tmcSpe_StateVariables_s * pState;
    
    pState = &mcSpe_StateVariables_mds;
    
    pState->absSpeFilt = 0.0f;
}
