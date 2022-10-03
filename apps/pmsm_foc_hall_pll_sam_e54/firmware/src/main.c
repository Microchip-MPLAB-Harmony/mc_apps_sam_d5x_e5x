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


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    mcBse_DelayState_mds.period = DELAY_10MS_COUNT;
    ADC0_CallbackRegister((ADC_CALLBACK) mcFcoI_AdcCalibrationTasks, (uintptr_t)NULL);
    EIC_CallbackRegister ((EIC_PIN)EIC_PIN_2, (EIC_CALLBACK) OC_FAULT_ISR,(uintptr_t)NULL);
    PDEC_HALLCallbackRegister((PDEC_HALL_CALLBACK)mcFcoI_HallEventISR, (uintptr_t)NULL);
    
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

