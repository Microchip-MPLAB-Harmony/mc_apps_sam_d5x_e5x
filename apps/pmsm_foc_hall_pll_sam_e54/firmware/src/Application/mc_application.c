/*******************************************************************************
 Application source file 

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
* Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries.
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
#include "mc_application.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/


/*******************************************************************************
 Private data-types 
 *******************************************************************************/


/*******************************************************************************
 Private variables 
 *******************************************************************************/

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcApp_ControlFlags_s     mcAppI_ControlFlags_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/*! \brief Set Motor speed
 * 
 * Details.
 * Set motor speed
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_MotorSpeedSet( const int16_t rpm )
{
    /* Get direction */
    
    /* Get value */
}

/*! \brief Start motor
 * 
 * Details.
 * Start motor
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_MotorStart( void )
{
      mcMocI_MotorStart(&mcMocI_MotorControl_gds);
}

/*! \brief Stop motor
 * 
 * Details.
 * Stop motor
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_MotorStop( void )
{
      mcMocI_MotorStop(&mcMocI_MotorControl_gds);
}

/*! \brief Start/Stop motor
 * 
 * Details.
 * Start/stop motor
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_MotorStartStop( void )
{
    mcMocI_MotorStartToggle(&mcMocI_MotorControl_gds);
}

/*! \brief Start/Stop motor
 * 
 * Details.
 * Start/stop motor
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcAppI_MotorDirectionToggle( void )
{
     mcMocI_MotorDirectionToggle(&mcMocI_MotorControl_gds);
}