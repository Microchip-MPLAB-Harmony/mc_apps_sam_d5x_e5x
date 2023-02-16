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

#ifndef MCSPE_H_    // Guards against multiple inclusion
#define MCSPE_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_generic_libraries.h"
#include "mc_data_manager.h"
#include "mc_motor_control.h"

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/
#define CONFIG_PolePairs    (float)5.0f
/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
    uint8_t * runCommand;
    int16_t * speAdcInput;
    float * directionFlag;
}tmcSpe_InputPorts_s;

typedef struct
{
    float speFiltParam;
    float adcUnitsToSpeed;
    float minSpeedinRpm;
    float maxSpeedinRpm;
}tmcSpe_UserParameters_s;

typedef struct
{
    float * cmdSpeed;
    float * cmdSpeedFilt;
    uint8_t * runStatus;
}tmcSpe_OutputPorts_s;

typedef struct
{
    uint8_t                                     index;
    tmcSpe_InputPorts_s             dInputPorts;
    tmcSpe_OutputPorts_s           dOutputPorts;
    tmcSpe_UserParameters_s    dUserParam;
}tmcSpe_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcSpe_ModuleData_s mcSpeI_ModuleData_gds;

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
__STATIC_INLINE void mcSpeI_InputPortsSet(  tmcSpe_InputPorts_s * const pPorts )
{
    pPorts->speAdcInput = &mcBseI_UpotAdcInput_gds16;
    pPorts->directionFlag = &(mcMocI_MotorControl_gds.Flags.direction);
    pPorts->runCommand = &(mcMocI_MotorControl_gds.runCommand);
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
__STATIC_INLINE void mcSpeI_OutputPortsSet( tmcSpe_OutputPorts_s * const pPorts )
{
    pPorts->cmdSpeed = &mcSpeI_CommandSpeed_gdf32;
    pPorts->cmdSpeedFilt = &mcSpeI_CommandSpeedFilt_gdf32;
    pPorts->runStatus = &(mcMocI_MotorControl_gds.runStatus);
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
__STATIC_INLINE void mcSpeI_UserParametersSet( tmcSpe_UserParameters_s * const pPorts )
{
    pPorts->speFiltParam = 0.05f;
    pPorts->adcUnitsToSpeed = POT_ADC_COUNT_FW_SPEED_RATIO;
    pPorts->minSpeedinRpm = 10.0f;
    pPorts->maxSpeedinRpm = 3000.0f;
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
void mcSpeI_SpeedMeasurementInit( tmcSpe_ModuleData_s * const module );

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
void mcSpeI_SpeedMeasurementRun(tmcSpe_ModuleData_s * const module);

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
void mcSpeI_SpeedMeasurementReset(tmcSpe_ModuleData_s * const module);

#endif //MCSPE_H_

/**
 End of File
*/