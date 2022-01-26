/*******************************************************************************
 Hardware abstraction functions 

  Company:
    Microchip Technology Inc.

  File Name:
    mc_hardware_abstraction.c

  Summary:
   Hardware abstraction layer 

  Description:
  Hardware abstraction layer 
 
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
#include <stdint.h>
#include "mc_hardware_abstraction.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcHal_ButtonState_s     mcHal_PressButtonState_mds;

/*******************************************************************************
 Private variables 
 *******************************************************************************/

/*******************************************************************************
 Interface variables 
 *******************************************************************************/

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Set PWM Channels of the voltage source inverter
 * 
 * Details.
 * Set PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmSet( tmcHal_InverterInstanceId_e Id, uint16_t * duty )
{
    if( Id == 0u )
    {

            TCC0_PWM24bitDutySet(TCC0_CHANNEL0, duty[0] );
        TCC0_PWM24bitDutySet(TCC0_CHANNEL1, duty[1] );
        TCC0_PWM24bitDutySet(TCC0_CHANNEL2, duty[2] );
    }
    else 
     {
         
     }
}


/*! \brief Disable PWM Channels of the voltage source inverter
 * 
 * Details.
 * Disable PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmDisable(void)
{
   /*Override all PWM outputs to low*/
    while ((TCC0_REGS->TCC_SYNCBUSY & (TCC_SYNCBUSY_PATT_Msk)) == TCC_SYNCBUSY_PATT_Msk)
    {
        /* Wait for sync */
    }
    TCC0_REGS->TCC_PATT = 0x00FF;

}

/*! \brief Enable PWM Channels of the voltage source inverter
 * 
 * Details.
 * Enable PWM Channels of the voltage source inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_VoltageSourceInverterPwmEnable(void)
{
    /*Disable PWM override*/
    while ((TCC0_REGS->TCC_SYNCBUSY & (TCC_SYNCBUSY_PATT_Msk)) == TCC_SYNCBUSY_PATT_Msk)
    {
        /* Wait for sync */
    }
    TCC0_REGS->TCC_PATT = 0x0000;
   
}

/*! \brief Get voltage signal from ADC port
 * 
 * Details.
 * Get voltage signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_VoltageSignalGet( void )
{
    return ADC1_ConversionResultGet();
}


/*! \brief Get phase A current from ADC port 
 * 
 * Details.
 * Get phase A current from ADC port 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PhaseACurrentSignalGet( uint8_t Id )
{
    if ( 0u == Id)
    {
        return ADC0_ConversionResultGet();
    }
    else 
    {
        return 0;
    }
}



/*! \brief Get voltage signal from ADC port
 * 
 * Details.
 * Get voltage signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PhaseBCurrentSignalGet( uint8_t Id )
{
    return ADC1_ConversionResultGet();   
   
}



/*! \brief Get potentiometer signal from ADC port
 * 
 * Details.
 * Get potentiometer signal from ADC port
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PotentiometerSignalGet( void )
{
    return ADC0_ConversionResultGet();
}


/*! \brief Button Polling 
 * 
 * Details.
 * Button polling 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHal_ButtonResponse( const tmcHal_ButtonState_e  buttonState,  void (*buttonFunction)(void) )
{
    switch(mcHal_PressButtonState_mds.state)
    {
        case buttonStateMachine_Ready:
        {
            if( buttonState_Pressed == buttonState )
            {
                buttonFunction();
                mcHal_PressButtonState_mds.debounceCounter = 0;
                mcHal_PressButtonState_mds.state = buttonStateMachine_Wait;
            }
        }
        break;

        case buttonStateMachine_Wait:
        {
            if( SW_DEBOUNCE_DLY_500MS <= mcHal_PressButtonState_mds.debounceCounter)
            {
                mcHal_PressButtonState_mds.state = buttonStateMachine_Ready;
                mcHal_PressButtonState_mds.debounceCounter = 0;
            }
            else
            {
                mcHal_PressButtonState_mds.debounceCounter++;
            }
        }
        break;
        default:
        {
              /* Should never come here */
        }
    }
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptDisable( void )
{
    ADC0_InterruptsDisable( ADC_STATUS_RESRDY );
} 

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptEnable( void )
{
    ADC0_InterruptsEnable( ADC_STATUS_RESRDY );  
} 


/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcInterruptClear( void )
{
    ADC0_InterruptsClear(ADC_STATUS_MASK);
}


/*! \brief ADC Enable
 * 
 * Details.
 * ADC Enable
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_ADCEnable( void )
{
    ADC0_Enable();
}

/*! \brief ADC callback function
 * 
 * Details.
 * ADC callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_AdcCallBackRegister(ADC_CALLBACK  callback, uintptr_t context)
{
    ADC0_CallbackRegister( callback, context);
}


/*! \brief PWM timer Start
 * 
 * Details.
 * PWM timer Start
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmTimerStart( void )
{
    TCC0_PWMStart( );  
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptDisable( void )
{
    NVIC_DisableIRQ(TCC0_OTHER_IRQn);
} 

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptEnable( void )
{
    NVIC_EnableIRQ(TCC0_OTHER_IRQn);  
} 


/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmInterruptClear( void )
{
    NVIC_ClearPendingIRQ(TCC0_OTHER_IRQn);
}

/*! \brief Get analog signals from ADC peripheral
 * 
 * Details.
 * Get analog signals from ADC peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmEnable( void )
{
   
}

/*! \brief Get PWM period
 * 
 * Details.
 * Get PWM period
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
uint16_t mcHalI_PwmPeriodGet( void )
{
    return TCC0_PWM24bitPeriodGet( );
}

/*! \brief PWM callback function
 * 
 * Details.
 * PWM callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_PwmCallbackRegister( TCC_CALLBACK  callback, uintptr_t context )
{
    TCC0_PWMCallbackRegister( callback, (uintptr_t)context);

}

/*! \brief Read Group 01 signal 
 * 
 * Details.
 * Read Group 01 signal
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group01SignalRead( uint16_t Id  )
{
    mcHalI_PhaseACurrent_gdu16 = mcHalI_PhaseACurrentSignalGet( Id );
    mcHalI_PhaseBCurrent_gdu16  = mcHalI_PhaseBCurrentSignalGet( Id );

}

/*! \brief Read Group 02 signal 
 * 
 * Details.
 * Read Group 02 signal
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group02SignalRead( uint16_t Id  )
{
    mcHalI_DcBusVoltage_gdu16  = mcHalI_VoltageSignalGet();
    mcHalI_Potentiometer_gdu16 = mcHalI_PotentiometerSignalGet();
    
}

/*! \brief Re-assign ADC channels for  Group 01 signals and enable hardware trigger 
 * 
 * Details.
 * Re-assign ADC channels for  Group 01 signals and enable hardware trigger
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group01SignalHardwareTrigger( uint16_t Id  )
{
    /* Select channels for Group 01 signals */
    ADC0_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
    ADC1_ChannelSelect( ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
    
    /* Enable hardware trigger */
    ADC0_InterruptsClear(ADC_STATUS_MASK);
    ADC0_InterruptsEnable( ADC_STATUS_RESRDY );
}

/*! \brief Re-assign ADC channels for  Group 02 signals and enable software trigger 
 * 
 * Details.
 * Re-assign ADC channels for  Group 02 signals and enable software trigger 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_Group02SignalSoftwareTrigger( uint16_t Id  )
{
    /* Select channels for Group 02 signals */
    ADC0_ChannelSelect( ADC_POSINPUT_AIN6, ADC_NEGINPUT_GND);
    ADC1_ChannelSelect( ADC_POSINPUT_AIN14, ADC_NEGINPUT_GND);
    
    /* Enable software  trigger */
    ADC0_InterruptsClear(ADC_STATUS_MASK);
    ADC0_InterruptsDisable( ADC_STATUS_RESRDY );
    ADC0_ConversionStart(); 


}

/*! \brief Start the encoder peripheral
 * 
 * Details.
 * Start the encoder peripheral
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_EncoderStart( uint16_t Id  )
{
    PDEC_QDECStart();
}

/*! \brief Get the encoder count
 * 
 * Details.
 * Get the encoder count
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int32_t mcHalI_EncoderPositionGet( uint16_t Id  )
{
        return PDEC_QDECPositionGet();
}


