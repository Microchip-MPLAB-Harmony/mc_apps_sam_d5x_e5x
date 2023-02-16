/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/
 
//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "X2CScope.h"
#include "X2CScopeCommunication.h"
#include "mc_motor_control.h"
#include "mc_userparams.h"
#include "mc_generic_libraries.h"
#include "mc_pwm.h"
#include "mc_application.h"
#include "mc_error_handler.h"
#include "mc_speed.h"
#include "board_services.h"
#include "mc_function_coordinator.h"
#include "mc_hall.h"

static uintptr_t dummyforMisra;
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    mcBse_DelayState_mds.period = (uint16_t)((float)DELAY_10MS_COUNT);
    ADC0_CallbackRegister((ADC_CALLBACK) mcFcoI_AdcCalibrationTasks, (uintptr_t)dummyforMisra);
    EIC_CallbackRegister ((EIC_PIN)EIC_PIN_2, (EIC_CALLBACK) OC_FAULT_ISR,(uintptr_t)dummyforMisra);
    PDEC_HALLCallbackRegister((PDEC_HALL_CALLBACK)mcFcoI_HallEventISR, (uintptr_t)dummyforMisra);
    
    PDEC_HALLStart();  /* Start PDEC in HALL or QDEC mode. */
    TCC0_PWMStart(); 
    ADC0_Enable();
    X2CScope_Init();
    mcBseI_InverterDisable();
    
    mcFcoI_SoftwareModuleInit();

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
               
        /* Button scan */
        mcBseI_ButtonScan();
        
        /* X2C Communication */
        X2CScope_Communicate();
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

