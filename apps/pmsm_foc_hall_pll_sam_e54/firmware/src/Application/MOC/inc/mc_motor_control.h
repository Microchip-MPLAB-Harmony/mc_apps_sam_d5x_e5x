/*******************************************************************************
 Motor Control Application Header File

  File Name:
    mc_app.h

  Summary:
 Motor Control Application Variable and Function declarations.

  Description:
    This file contains the declarations for Motor Control application specific 
 * variables and functions (excluding variables and functions used by Motor Control
 * Library

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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
//DOM-IGNORE-END

#ifndef MCMOC_H    /* Guard against multiple inclusion */
#define MCMOC_H

#include <stddef.h>
#include <math.h>
#include "definitions.h"
#include "mc_pwm.h"
#include "mc_current.h"
#include "board_services.h"
#include "mc_generic_libraries.h"
#include "mc_userparams.h"
#include "mc_data_manager.h"
#include "mc_rotor_position.h"

/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
    float * iA;
    float * iB;
    float * iC;
    float * uDC;
    float * sensedAngle;
    float * senselessAngle;
    float * sensedSpeed;
    float * senselessSpeed;
    float * cmdSpeed;
}tmcMocI_InputPorts_s;

typedef struct
{
    float * uA;
    float * uB;
    float * uC;
    float * dA;
    float * dB;
    float * dC;   
}tmcMocI_OutputPorts_s; 

/*******************************************************************************
 Constants 
 *******************************************************************************/
/* D axis current control loop coefficients */
#define     CONFIG_IdControllerKp          (float)( 0.3   )              
#define     CONFIG_IdControllerKi           (float)( 0.05)         
#define     CONFIG_IdControllerKc          (float)( 0.5  )               
#define     CONFIG_IdControllerYmax      (float)( 0.999   )          

/* Q axis current control loop coefficients */
#define     CONFIG_IqControllerKp          (float)( 0.3  )              
#define     CONFIG_IqControllerKi           (float)( 0.05)         
#define     CONFIG_IqControllerKc          (float)( 0.5  )               
#define     CONFIG_IqControllerYmax      (float)( 0.999   )  

/* Speed control loop coefficients */
#define     CONFIG_SpeedControllerKp          (float)( 0.00040   )              
#define     CONFIG_SpeedControllerKi           (float)( 0.00000010)         
#define     CONFIG_SpeedControllerKc          (float)( 0.5  )               
#define     CONFIG_SpeedControllerYmax      (float)( MAX_MOTOR_CURRENT   )  

/*******************************************************************************
 * Default Ports
 *******************************************************************************/

/*******************************************************************************
 Interface variables
 *******************************************************************************/
extern tmcMocI_MotorControl_s     mcMocI_MotorControl_gds;

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
__STATIC_INLINE void mcMocI_InputPortsSet( tmcMocI_InputPorts_s * const pInputPorts )
{
    pInputPorts->iA = &mcCurI_Ia_gdf32;
    pInputPorts->iB = &mcCurI_Ib_gdf32;
    pInputPorts->iC = &mcCurI_Ic_gdf32;
    pInputPorts->uDC  = NULL;
    pInputPorts->sensedAngle = &mcRpoI_ElectricalAngle_gdf32;
    pInputPorts->sensedSpeed = &mcRpoI_ElectricalSpeed_gdf32;
    pInputPorts->senselessAngle = &mcPllI_ElectricalAngle_gdf32;
    pInputPorts->senselessSpeed = &mcPllI_ElectricalSpeed_gdf32;
    pInputPorts->cmdSpeed = &mcSpeI_CommandSpeedFilt_gdf32;
}


/*******************************************************************************
 Interface functions 
 *******************************************************************************/

/*! \brief Motor Control initialization tasks 
 * 
 * Details.
 * Motor Control initialization tasks 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlInit(tmcMocI_MotorControl_s * const pControl );

/*! \brief Motor control tasks
 * 
 * Details.
 * Motor control tasks 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlRun( tmcMocI_MotorControl_s * const pState );

/*! \brief Motor control reset 
 * 
 * Details.
 * Motor control reset 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlReset( tmcMocI_MotorControl_s * const pState );

/*! \brief Motor start
 * 
 * Details.
 * Motor start
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStart(tmcMocI_MotorControl_s * const pState);

/*! \brief Motor stop
 * 
 * Details.
 * Motor stop
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStop(tmcMocI_MotorControl_s * const pState);

/*! \brief Motor start toggle
 * 
 * Details.
 * Motor Toggle
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStartToggle(tmcMocI_MotorControl_s * const pState);

/*! \brief Motor direction change
 * 
 * Details.
 * Motor direction change
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorDirectionToggle(tmcMocI_MotorControl_s * const pState);


#endif /* MCMOC_H */

/* *****************************************************************************
 End of File
 */
