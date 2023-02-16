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

#ifndef MCRPO_H_    // Guards against multiple inclusion
#define MCRPO_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "mc_hall.h"
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_generic_libraries.h"
#include "mc_data_manager.h"


/*******************************************************************************
 Default module parameters 
 ******************************************************************************/
#undef ENABLE_POSITION_SMOOTHENING

/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
    uint32_t * tcHallToAdcIsr;
}tmcRpo_InputPorts_s;

typedef struct
{
    float * elecAngle;
    float * elecSpeed;
}tmcRpo_OutputPorts_s;

typedef struct
{
    float * elecAngle;
    float * elecSpeed;
}tmcRpo_UserParameters_s;

typedef struct
{
    uint8_t index;
    tmcRpo_InputPorts_s           dInput;
    tmcRpo_OutputPorts_s         dOutput;
    tmcRpo_UserParameters_s dParam;
}tmcRpo_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcRpo_ModuleData_s mcRpoI_ModuleData_gds;

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
__STATIC_INLINE void mcRpoI_InputPortsSet( tmcRpo_InputPorts_s * const pInputPort )
{
    pInputPort->tcHallToAdcIsr = &mcFcoI_HallToAdcISRTimerCount_gdu32;
}

__STATIC_INLINE void mcRpoI_OutputPortsSet( tmcRpo_OutputPorts_s * const pOutputPort )
{
    pOutputPort->elecAngle = &mcRpoI_ElectricalAngle_gdf32;
    pOutputPort->elecSpeed = &mcRpoI_ElectricalSpeed_gdf32;
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
void mcRpoI_PosCalInit(tmcRpo_ModuleData_s * const module);

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
void mcRpoI_PosCalTrigger( tmcRpo_ModuleData_s * const module );

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
void mcRpoI_PosCalRun( tmcRpo_ModuleData_s * const module);

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
void mcRpoI_PosCalReset(tmcRpo_ModuleData_s * const module);

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
void mcRpo_RotorPositionSmoothing( tmcRpo_ModuleData_s * const module );

#endif //MCRPO_H_

/**
 End of File
*/