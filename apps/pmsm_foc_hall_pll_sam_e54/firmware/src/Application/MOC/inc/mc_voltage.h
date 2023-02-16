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

#ifndef MCVOL_H   // Guards against multiple inclusion
#define MCVOL_H

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_data_manager.h"

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/
#define CONFIG_CurrentModuleInstances 2u

/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct 
{
    int16_t * udcAdcInput;
}tmcVol_InputPorts_s;

typedef struct
{
    float udcFiltParam;
    float adcUnitsToVolts;
}tmcVol_UserParameters_s;

typedef struct
{
    float * udc;
    float * udcFilt;
    float * uacPeak;
}tmcVol_OutputPorts_s;

typedef struct
{
    uint8_t                                    index;
    tmcVol_InputPorts_s             dInputPorts;
    tmcVol_OutputPorts_s           dOutputPorts;
    tmcVol_UserParameters_s    dUserParam;
}tmcVol_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcVol_ModuleData_s mcVolI_ModuleData_gds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
/*! \brief Set input ports 
 * 
 * Details.
 * Set input ports 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
__STATIC_INLINE void mcVolI_InputPortsSet(  tmcVol_InputPorts_s * const pPorts )
{
    pPorts->udcAdcInput = &mcBseI_UdcAdcInput_gds16;
}

/*! \brief Set output ports 
 * 
 * Details.
 * Set output ports 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
__STATIC_INLINE void mcVolI_OutputPortsSet( tmcVol_OutputPorts_s * const pPorts )
{
    pPorts->udc = &mcVolI_Udc_gdf32;
    pPorts->udcFilt = &mcVolI_UdcFilt_gdf32;
    pPorts->uacPeak = &mcVolI_UacPeak_gdf32;
}

/*! \brief Set user parameters 
 * 
 * Details.
 * Set user parameters 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
__STATIC_INLINE void mcVolI_UserParametersSet( tmcVol_UserParameters_s * const pPorts )
{
    pPorts->udcFiltParam = 0.05f;
    pPorts->adcUnitsToVolts = VOLTAGE_ADC_TO_PHY_RATIO;
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
void mcVolI_VoltageMeasurementInit( tmcVol_ModuleData_s * const module );

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
void mcVolI_VoltageMeasurementRun(tmcVol_ModuleData_s * const module);

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
void mcVolI_VoltageMeasurementReset(tmcVol_ModuleData_s * const module);

#endif //MCVOL_H_

/**
 End of File
*/