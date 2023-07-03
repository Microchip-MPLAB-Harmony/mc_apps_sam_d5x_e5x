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
#include "mc_current.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/


/*******************************************************************************
 Private data-types 
 *******************************************************************************/
typedef struct
{
    uint16_t sampleCount;
    uint16_t iaAdcOffset;
    uint16_t ibAdcOffset;
    uint32_t iaAdcSum;
    uint32_t ibAdcSum;
}tmcCur_StateVariables_s;

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcCur_StateVariables_s mcCur_StateVariables_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcCur_ModuleData_s mcCurI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/


/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/*! \brief Current Calculation initialization
 * 
 * Details.
 * Current Calculation initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcCurI_CurrentMeasurementInit(tmcCur_ModuleData_s * const module)
{
    /* Set input ports */
    mcCurI_InputPortsSet(&module->dInputPorts);
    
    /* Set output ports */
    mcCurI_OutputPortsSet(&module->dOutputPorts);
    
    /* Set user parameters  */
    mcCurI_UserParametersSet(&module->dUserParam);
    
    /* Calculate user dependent intermediate constant values */
}

/*! \brief Offset Calibration
 * 
 * Details.
 * Offset Calibration
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
tStd_ReturnType_e mcCurI_OffsetCalibarationRun( void)
{
    tStd_ReturnType_e status = returnType_Running;
    
    mcCur_StateVariables_mds.sampleCount++;
    if(mcCur_StateVariables_mds.sampleCount <= 4096U)
    {
        mcCur_StateVariables_mds.iaAdcSum += (uint32_t)mcBseI_IaAdcInput_gds16;    
        mcCur_StateVariables_mds.ibAdcSum += (uint32_t)mcBseI_IbAdcInput_gds16;
    }
    else
    {
        mcCur_StateVariables_mds.iaAdcOffset = (uint16_t)(mcCur_StateVariables_mds.iaAdcSum >>12U);
        mcCur_StateVariables_mds.ibAdcOffset = (uint16_t)(mcCur_StateVariables_mds.ibAdcSum >>12U);     
        status = returnType_Passed; 
    }
    return status;
}

/*! \brief Current measurement
 * 
 * Details.
 * Current measurement
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcCurI_CurrentMeasurementRun(tmcCur_ModuleData_s * const module)
{  
    float temp;
    
    /* Phase A current calculation */
    temp =  (float)((float)mcCur_StateVariables_mds.iaAdcOffset - (float)mcBseI_IaAdcInput_gds16) ;
    *module->dOutputPorts.iA =  temp * ADC_CURRENT_SCALE ;
    
    /* Phase B current calculation */
    temp = (float)((float)mcCur_StateVariables_mds.ibAdcOffset - (float)mcBseI_IbAdcInput_gds16) ;
    *module->dOutputPorts.iB =  temp * ADC_CURRENT_SCALE ;
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
void mcCurI_CurrentMeasurementReset(tmcCur_ModuleData_s * const module )
{
    tmcCur_StateVariables_s * pState;
    
    pState = &mcCur_StateVariables_mds;
    
    pState->sampleCount  = 0U;
    pState->iaAdcOffset = 0U;
    pState->ibAdcOffset = 0U;
    pState->iaAdcSum      = 0U;
    pState->ibAdcOffset  = 0U;
}