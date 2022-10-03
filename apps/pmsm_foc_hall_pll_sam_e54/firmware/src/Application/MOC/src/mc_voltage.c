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
#include "mc_voltage.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/
#define CONSTANT_OneBySqrt3     		(float)0.5773502691   

/*******************************************************************************
 Private data-types 
 *******************************************************************************/
typedef struct _tmcVol_StateVariables_s
{
    float udcFilt;
}tmcVol_StateVariables_s;

typedef struct _tmcVol_Parameters_s
{
    float udcFiltParam;
    float adcUnitsToVolts;
}tmcVol_Parameters_s;

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcVol_StateVariables_s mcVol_StateVariables_mds;
static tmcVol_Parameters_s mcVol_Parameters_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcVol_ModuleData_s mcVolI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/


/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/*! \brief Voltage Calculation initialization
 * 
 * Details.
 * Voltage Calculation initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcVolI_VoltageMeasurementInit(tmcVol_ModuleData_s * const module)
{
    /* Set input ports */
    mcVolI_InputPortsSet(&module->dInputPorts);
    
    /* Set output ports */
    mcVolI_OutputPortsSet(&module->dOutputPorts);
    
    /* Set user parameters  */
    mcVolI_UserParametersSet(&module->dUserParam);
    
    /* Calculate user dependent intermediate constant values */
    mcVol_Parameters_mds.udcFiltParam = module->dUserParam.udcFiltParam;
    mcVol_Parameters_mds.adcUnitsToVolts = module->dUserParam.adcUnitsToVolts;
    
}

/*! \brief Voltage measurement
 * 
 * Details.
 * Voltage measurement
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcVolI_VoltageMeasurementRun(tmcVol_ModuleData_s * const module)
{ 
    float udc;
    tmcVol_InputPorts_s * pInput;
    tmcVol_StateVariables_s * pState;
    tmcVol_Parameters_s * pParam;
    
    pInput = &module->dInputPorts;
    pState = &mcVol_StateVariables_mds;
    pParam = &mcVol_Parameters_mds;
    
    /* Calculate DC bus voltage */ 
    udc = (float)(*pInput->udcAdcInput * module->dUserParam.adcUnitsToVolts );
    *module->dOutputPorts.udc = udc;
    
    /* Filter DC bus voltage */
    pState->udcFilt = pState->udcFilt + pParam->udcFiltParam * ( udc - pState->udcFilt );
    *module->dOutputPorts.udcFilt = pState->udcFilt;
    *module->dOutputPorts.uacPeak = (float)( pState->udcFilt  * CONSTANT_OneBySqrt3 );
}

/*! \brief Voltage measurement reset 
 * 
 * Details.
 * Voltage measurement reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcVolI_VoltageMeasurementReset(tmcVol_ModuleData_s * const module )
{
    tmcVol_StateVariables_s * pState;
    
    pState = &mcVol_StateVariables_mds;
    
    pState->udcFilt = 0.0f;
}