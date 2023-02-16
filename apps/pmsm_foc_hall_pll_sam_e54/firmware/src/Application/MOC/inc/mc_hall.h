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

#ifndef MCHALL__H_    // Guards against multiple inclusion
#define MCHALL__H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>
#include "definitions.h"
#include "mc_generic_libraries.h"
#include "mc_userparams.h"
#include "mc_data_manager.h"

/*******************************************************************************
 Constants 
 ******************************************************************************/
/* HALL constants */
#define CONSTANT_1PiBy6 (float)(1.0f / 6.0f * (float)M_PI)
#define CONSTANT_PiBy2 (float)(0.5f * (float)M_PI)
#define CONSTANT_5PiBy6 (float)(5.0f / 6.0f * (float)M_PI)
#define CONSTANT_7PiBy6 (float)(7.0f / 6.0f * (float)M_PI)
#define CONSTANT_3PiBy2 (float)(1.5f * (float)M_PI)
#define CONSTANT_11PiBy6 (float)(11.0f / 6.0f * (float)M_PI)
#define CONSTANT_1PiBy3 (float)(1.0f / 3.0f * (float)M_PI)
#define CONSTANT_2PiBy3 (float)(2.0 / 3.0f * (float)M_PI)
#define CONSTANT_Pi (float)(M_PI)
#define CONSTANT_4PiBy3 (float)(4.0f / 3.0f * (float)M_PI)
#define CONSTANT_5PiBy3 (float)(5.0f / 3.0f * (float)M_PI)
#define CONSTANT_2Pi (float)(2.0f * (float)M_PI)
#define CONSTANT_Dummy  (0.0f)

/*******************************************************************************
 User defined data-types
 *******************************************************************************/

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
typedef struct
{
    uint32_t * tcHallToHallIsr;
}tmcHall_InputPorts_s;

typedef struct
{
    float baseAngle;
    float boundAngle;
    float meanAngle;
    float directionFlag;
    float elecSpeed;    
}tmcHall_OutputBuffer_s;

typedef struct
{
    
}tmcHall_OutputPorts_s;

typedef struct 
{

}tmcHall_UserParameters_s;

typedef struct
{
    uint8_t index;
    tmcHall_InputPorts_s           dInput;
    tmcHall_OutputPorts_s         dOutput;
    tmcHall_UserParameters_s dParam;
}tmcHall_ModuleData_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcHall_ModuleData_s mcHallI_ModuleData_gds;

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
__STATIC_INLINE void mcHallI_InputPortsSet( tmcHall_InputPorts_s * const pInputPort )
{
    pInputPort->tcHallToHallIsr = &mcFcoI_HallToHallISRTimerCount_gdu32;
}

__STATIC_INLINE void mcHallI_OutputPortsSet( tmcHall_OutputPorts_s * const pOutputPort )
{
}

/*******************************************************************************
 Interface functions 
 *******************************************************************************/
/*! \brief Hall signal processing buffer read
 * 
 * Details.
 * Hall signal processing buffer read
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHallI_HallProcessBufferRead( tmcHall_OutputBuffer_s * const pOutput  );

/*! \brief Hall signal processing initialization
 * 
 * Details.
 * Hall signal processing initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHallI_HallSignalProcessInit( tmcHall_ModuleData_s * const pModule );

/*! \brief Hall signal processing run
 * 
 * Details.
 * Hall signal processing run
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHallI_HallSignalProcessRun( tmcHall_ModuleData_s * const pModule);

void mcHall_HallDataInit(void );

uint8_t mcHallI_HallPatternGet( void );


#endif //MCHALL_H_

/**
 End of File
*/