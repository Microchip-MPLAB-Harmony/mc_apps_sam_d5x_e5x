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

#ifndef MCFCO_H_    // Guards against multiple inclusion
#define MCFCO_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "mc_generic_libraries.h"
#include "mc_data_manager.h"
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_rotor_position.h"
#include "device_services.h"
#include "board_services.h"
#include "X2CScope.h"
#include "X2CScopeCommunication.h"
#include "mc_current.h"
#include "mc_motor_control.h"
#include "mc_speed.h"
#include "mc_data_manager.h"
#include "mc_voltage.h"
#include "mc_pll.h"

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/


/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
    
}tmcFco_InputPorts_s;

typedef struct
{
    uint32_t * tcHallToAdcIsr;
    uint32_t * tcHallIsr;
    uint32_t * tcHallToHallIsr;
}tmcFco_OutputPorts_s;

typedef struct
{

}tmcFco_UserParameters_s;

typedef struct
{
    uint8_t index;
    tmcFco_InputPorts_s           dInput;
    tmcFco_OutputPorts_s         dOutput;
    tmcFco_UserParameters_s dParam;
}tmcFco_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcFco_ModuleData_s mcFcoI_ModuleData_gds;

/*******************************************************************************
 Interface functions 
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
__STATIC_INLINE void mcFcoI_InputPortsSet(  tmcFco_InputPorts_s * const pPorts )
{
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
__STATIC_INLINE void mcFcoI_OutputPortsSet( tmcFco_OutputPorts_s * const pPorts )
{
    pPorts->tcHallIsr = &mcFcoI_HallISRTimerCount_gdu32;
    pPorts->tcHallToAdcIsr = &mcFcoI_HallToAdcISRTimerCount_gdu32;
    pPorts->tcHallToHallIsr = &mcFcoI_HallToHallISRTimerCount_gdu32;
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
__STATIC_INLINE void mcFcoI_UserParametersSet( tmcFco_UserParameters_s * const pPorts )
{
}

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
/*! \brief Application initialization
 * 
 * Details.
 * Application initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_SoftwareModuleInit( void);

/*! \brief ADC Calibration
 * 
 * Details.
 * ADC Calibration
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_AdcCalibrationTasks (ADC_STATUS status, uintptr_t context);


/*! \brief ADC Conversion complete ISR
 * 
 * Details.
 * ADC Conversion complete ISR
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_AdcInterruptTasks(ADC_STATUS status, uintptr_t context);

/*! \brief Hall Event ISR
 * 
 * Details.
 * Hall Event ISR
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_HallEventISR ( PDEC_HALL_STATUS status, uintptr_t context );

/*! \brief Function Coordinator initialization 
 * 
 * Details.
 * Function Coordinator initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_FunctionCoordinatorInit(tmcFco_ModuleData_s * const module);

#endif //MCBSE_H_

/**
 End of File
*/