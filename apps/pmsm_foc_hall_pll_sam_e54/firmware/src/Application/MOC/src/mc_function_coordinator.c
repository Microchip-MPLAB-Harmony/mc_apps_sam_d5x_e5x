/*******************************************************************************
 Function coordinator source file 

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


/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include "mc_function_coordinator.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/


/*******************************************************************************
 Private data-types 
 *******************************************************************************/

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static uintptr_t dummyforMisra;
/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcFco_ModuleData_s mcFcoI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/


/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

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
void mcFcoI_FunctionCoordinatorInit(tmcFco_ModuleData_s * const module)
{
    /* Set input ports */
    mcFcoI_InputPortsSet(&module->dInput);
    
    /* Set output ports */
    mcFcoI_OutputPortsSet(&module->dOutput);
    
    /* Set user parameters  */
    mcFcoI_UserParametersSet(&module->dParam);
    
    /* Calculate user dependent intermediate constant values */
}

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
void mcFcoI_SoftwareModuleInit( void)
{
    /* Function coordinator initialization */
    mcFcoI_FunctionCoordinatorInit(&mcFcoI_ModuleData_gds);
            
    /* Motor control module initialization */
    mcMocI_MotorControlInit(&mcMocI_MotorControl_gds);
    
    /* Speed measurement module initialization */
    mcSpeI_SpeedMeasurementInit( &mcSpeI_ModuleData_gds);
    
    /* Current measurement initialization */
    mcCurI_CurrentMeasurementInit(&mcCurI_ModuleData_gds);
    
    /* Hall signal processing module initialization  */
    mcHallI_HallSignalProcessInit(&mcHallI_ModuleData_gds);
        
     /* Initialize sensor-less position estimation   */
    mcRpoI_PosCalInit(&mcRpoI_ModuleData_gds);
    
     /* Voltage measurement module initialization */
     mcVolI_VoltageMeasurementInit(&mcVolI_ModuleData_gds);
    
}

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
void mcFcoI_AdcCalibrationTasks (ADC_STATUS status, uintptr_t context)
{   
    /* Current measurement */
    mcBseI_IaAdcInput_gds16 = mcBseI_IaAdcInputGet( );
    mcBseI_IbAdcInput_gds16 = mcBseI_IbAdcInputGet( );
    
    if( returnType_Passed == mcCurI_OffsetCalibarationRun() )
    {
        ADC0_Disable();
        ADC0_CallbackRegister((ADC_CALLBACK) mcFcoI_AdcInterruptTasks, (uintptr_t)dummyforMisra);
        ADC0_Enable();
    }
}

/*! \brief ADC interrupt tasks
 * 
 * Details.
 * ADC interrupt tasks
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcFcoI_AdcInterruptTasks(ADC_STATUS status, uintptr_t context)
{
  
    tmcFco_OutputPorts_s * pOutput;
    pOutput = &mcFcoI_ModuleData_gds.dOutput;
    
    /* Disable HALL ISR */
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
           
    /* Calculate time elapsed between last hall and ADC ISR */
    *pOutput->tcHallToAdcIsr = TC2_Timer32bitCounterGet() - ( *pOutput->tcHallIsr );
       
    /* Rotor position measurement */
    mcRpoI_PosCalRun(&mcRpoI_ModuleData_gds); 

    /* Enable HALL ISR */
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
    
    /* Current measurement */
    mcBseI_IaAdcInput_gds16 = mcBseI_IaAdcInputGet( );
    mcBseI_IbAdcInput_gds16 = mcBseI_IbAdcInputGet( );
      
    mcCurI_CurrentMeasurementRun(&mcCurI_ModuleData_gds);
    
    mcBseI_ClearAdcInterruptFlag();
    mcBseI_DisableAdcInterrupt();
    
    /* Clear all interrupt flags */
    mcBseI_ClearAdcInterruptFlag();
    mcBseI_DisableAdcInterrupt();
    
    /* Select the next ADC channel for conversion */
    mcBseI_UdcAdcInputChannelSet();
    mcBseI_UpotAdcInputChannelSet();
    
    mcBseI_TriggerAdcConversion();
      
    mcMocI_MotorControlRun(&mcMocI_MotorControl_gds);
    
    while(ADC0_REGS->ADC_INTFLAG != ADC_INTFLAG_RESRDY_Msk)
    {
        /*Wait for ADC conversion complete*/
    }           
    /* Read the ADC result value */
    mcBseI_UdcAdcInput_gds16 = mcBseI_UdcAdcInputGet();
    mcBseI_UpotAdcInput_gds16 = mcBseI_UpotAdcInputGet( );
    
    /* Voltage measurement */
    mcVolI_VoltageMeasurementRun(&mcVolI_ModuleData_gds);
  
    /* Select the next ADC channel for conversion */
    mcBseI_IaAdcInputChannelSet();
    mcBseI_IbAdcInputChannelSet();
      
    mcBseI_ClearAdcInterruptFlag();
    mcBseI_EnableAdcInterrupt();
             
    X2CScope_Update();
    mcBse_DelayState_mds.count++; 
}


void mcFcoI_HallEventISR ( PDEC_HALL_STATUS status, uintptr_t context )
{ 
    if( 1u ==  mcMocI_MotorControl_gds.runStatus )
    {
        uint32_t capture_val_last;
       tmcFco_OutputPorts_s * pOutput;

       pOutput = &mcFcoI_ModuleData_gds.dOutput;

       /* Determine time elapsed between two HALL events */
       capture_val_last = *pOutput->tcHallIsr;
      *pOutput->tcHallIsr = TC2_Capture32bitChannel0Get();
       if(0xFFFFFFFFLU == *pOutput->tcHallIsr)
       {
          *pOutput->tcHallIsr = TC2_Capture32bitChannel0Get();
       }

       /* Time elapsed between two hall events */
      *pOutput->tcHallToHallIsr =  *pOutput->tcHallIsr - capture_val_last;

       /* Rotor position and speed calculation from HALL sensor input */
       mcHallI_HallSignalProcessRun(&mcHallI_ModuleData_gds);   
    }
}

