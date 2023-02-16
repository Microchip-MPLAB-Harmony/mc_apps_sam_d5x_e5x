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
#include "mc_rotor_position.h"
#include <math.h>

/*******************************************************************************
 * Constants 
 *******************************************************************************/


/*******************************************************************************
 Private data-types 
 *******************************************************************************/
typedef struct
{
    float sine;
    float cosine;
    float angle;
}tmcRpo_StateVariables_s;


typedef struct
{
    float filterDrift;
    float filterParam;
}tmcRpo_Parameters_s;
/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcRpo_StateVariables_s mcRpo_StateVariables_mds;
static tmcRpo_Parameters_s mcRpo_Parameters_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcRpo_ModuleData_s mcRpoI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
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
void mcRpoI_PosCalInit(tmcRpo_ModuleData_s * const module)
{
    /* Set input ports */
    mcRpoI_InputPortsSet( &module->dInput);
    
    /* Set output ports */
    mcRpoI_OutputPortsSet(&module->dOutput);
    
    /* Update parameters */
    mcRpo_Parameters_mds.filterDrift = 1.22f;
    mcRpo_Parameters_mds.filterParam = 0.005f;
}

/*! \brief Rotor position calculation trigger
 * 
 * Details.
 * Rotor position calculation trigger
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcRpoI_PosCalTrigger( tmcRpo_ModuleData_s * const module )
{
    /* Re-initialize hall data */
     mcHall_HallDataInit();
}

/*! \brief Rotor position smoothing
 * 
 * Details.
 * Rotor position smoothing
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcRpo_RotorPositionSmoothing( tmcRpo_ModuleData_s * const module )
{   
    tmcMocI_PHASOR_s phasor;
    tmcRpo_StateVariables_s * pState;
    tmcRpo_Parameters_s * pParam;
    
    pState = &mcRpo_StateVariables_mds;
    pParam = &mcRpo_Parameters_mds;
    
    phasor.angle = *module->dOutput.elecAngle;
    mcLib_SinCosGen(&phasor);
    
    pState->sine = pState->sine + pParam->filterParam * ( phasor.Sin - pState->sine );
    pState->cosine = pState->cosine + pParam->filterParam * (phasor.Cos - pState->cosine );
    
   pState->angle = atan2f( pState->sine,  pState->cosine ) + pParam->filterDrift;
    
    if( pState->angle > CONSTANT_2Pi )
    {
        pState->angle -= CONSTANT_2Pi;
    }
    else if( pState->angle < 0.0f )
    {
         pState->angle += CONSTANT_2Pi;
    }
    else
    {
        /* Dummy branch for MISRAC compliance*/
    }
   
   /* Update rotor position angle */
    *module->dOutput.elecAngle = pState->angle;
}

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

void mcRpoI_PosCalRun(tmcRpo_ModuleData_s * const module)
{
    float temp;
    tmcHall_OutputBuffer_s  outBuffer;
        
    mcHallI_HallProcessBufferRead(&outBuffer);
    
    if ( outBuffer.elecSpeed < 100.0f )
    {
        *module->dOutput.elecAngle = outBuffer.meanAngle;
    }
    else 
    {         
        temp =  outBuffer.elecSpeed  * (float)(*module->dInput.tcHallToAdcIsr )  * RL_TCO_TS;
       
        if(temp > CONSTANT_1PiBy3 ) 
        {
            temp = CONSTANT_1PiBy3; 
        }
       
        *module->dOutput.elecAngle = outBuffer.baseAngle + outBuffer.directionFlag * temp;
    }
    
    if(  *module->dOutput.elecAngle > CONSTANT_2Pi )
    {
         *module->dOutput.elecAngle -= CONSTANT_2Pi;
    }
    else if(  *module->dOutput.elecAngle < 0.0f )
    {
          *module->dOutput.elecAngle += CONSTANT_2Pi;
    }
    else
    {
        /* Dummy branch for MISRAC compliance*/
    }
    
    /* Low pass filter */
    temp = outBuffer.elecSpeed * outBuffer.directionFlag;
    *module->dOutput.elecSpeed = (RL_LPF_COEFF_1 * (*module->dOutput.elecSpeed ) ) + (RL_LPF_COEFF_2 * temp );    
 
#ifdef ENABLE_POSITION_SMOOTHENING
    mcRpo_RotorPositionSmoothing(module);
#endif
}


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
void mcRpoI_PosCalReset(tmcRpo_ModuleData_s * const module)
{
    
}