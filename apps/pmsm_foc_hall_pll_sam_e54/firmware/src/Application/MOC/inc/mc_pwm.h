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

#ifndef MCPWM_H_    // Guards against multiple inclusion
#define MCPWM_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "mc_generic_libraries.h"
#include "definitions.h"
#include "mc_userparams.h"

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/


/*******************************************************************************
 User defined data-types
 *******************************************************************************/

/*******************************************************************************
 Interface variables 
 *******************************************************************************/


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
void mcPwmI_SpaceVectorModulationInit( tmcMocI_SVPWM_s * const svParam);

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
void mcPwmI_SpaceVectorModulationRun(tmcMocI_AB_s *alphabetaParam, tmcMocI_SVPWM_s *svParam);

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
void mcPwmI_SpaceVectorModulationReset( tmcMocI_SVPWM_s * const svParam);


#endif //MCBSE_H_

/**
 End of File
*/