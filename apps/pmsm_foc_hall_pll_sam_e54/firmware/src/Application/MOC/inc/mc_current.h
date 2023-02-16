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

#ifndef MCCUR_H_    // Guards against multiple inclusion
#define MCCUR_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "definitions.h"
#include "mc_userparams.h"
#include "device_services.h"
#include "mc_data_manager.h"
#include "mc_types.h"

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/
#define CONFIG_CurrentModuleInstances 2u

/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
    int16_t * iaAdcInput;
    int16_t * ibAdcInput;
}tmcCur_InputPorts_s;

typedef struct
{
    float iOffsetMax;
    float iOffsetMin;
}tmcCur_UserParameters_s;

typedef struct
{
    float * iA;
    float * iB;
    float * iC;
}tmcCur_OutputPorts_s;

typedef struct
{
    uint8_t                                    index;
    tmcCur_InputPorts_s             dInputPorts;
    tmcCur_OutputPorts_s           dOutputPorts;
    tmcCur_UserParameters_s    dUserParam;
}tmcCur_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcCur_ModuleData_s mcCurI_ModuleData_gds;

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
__STATIC_INLINE void mcCurI_InputPortsSet(  tmcCur_InputPorts_s * const pPorts )
{
    pPorts->iaAdcInput = NULL;
    pPorts->ibAdcInput = NULL;
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
__STATIC_INLINE void mcCurI_OutputPortsSet( tmcCur_OutputPorts_s * const pPorts )
{
    pPorts->iA = &mcCurI_Ia_gdf32;
    pPorts->iB = &mcCurI_Ib_gdf32;
    pPorts->iC = &mcCurI_Ic_gdf32;;
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
__STATIC_INLINE void mcCurI_UserParametersSet( tmcCur_UserParameters_s * const pPorts )
{
    //pPorts->iOffsetMax = 1900.0f;
    pPorts->iOffsetMax = 2200.0f;
}


/*******************************************************************************
 Interface functions 
 *******************************************************************************/

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
tStd_ReturnType_e mcCurI_OffsetCalibarationRun( void);

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
void mcCurI_CurrentMeasurementInit( tmcCur_ModuleData_s * const module );

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
void mcCurI_CurrentMeasurementRun(tmcCur_ModuleData_s * const module);

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
void mcCurI_CurrentMeasurementReset(tmcCur_ModuleData_s * const module);

#endif //MCCUR_H_

/**
 End of File
*/