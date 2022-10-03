/*******************************************************************************
 Board services Source file 

  Company:
    Microchip Technology Inc.

  File Name:
    board_services.c

  Summary:
    This file contains all the functions related to board services

  Description:
    This file contains all the functions related to board services
 
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
#include "definitions.h"
#include "mc_userparams.h"
#include "mc_motor_control.h"
#include "mc_userparams.h"
#include "mc_generic_libraries.h"
#include "board_services.h"
#include "mc_pwm.h"
#include "mc_error_handler.h"
#include "mc_speed.h"
#include "mc_application.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/


#define CONFIG_ButtonScanIntervalInSec      (float)(0.00005f)
#define CONFIG_ButtonDebounceInSec          (float)(0.5f)
#define CONSTANT_ButtonDebounceLoopCount    (float)( CONFIG_ButtonDebounceInSec / CONFIG_ButtonScanIntervalInSec )



/*******************************************************************************
 Private data-types 
 *******************************************************************************/


/*******************************************************************************
 Private variables 
 *******************************************************************************/
tmcBse_DelayGeneration_s mcBse_DelayState_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcBse_ButtonState_s    button_S2_data;
tmcBse_ButtonState_s    button_S3_data;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
/*! \brief Disable inverter
 * 
 * Details.
 * Disable inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_InverterDisable( void )
{
    TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t)PWM_PERIOD_COUNT>>1);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t)PWM_PERIOD_COUNT>>1);
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t)PWM_PERIOD_COUNT>>1);
    
    /*Override all PWM outputs to low*/
    TCC0_PWMPatternSet((TCC_PATT_PGE0_Msk|TCC_PATT_PGE1_Msk|TCC_PATT_PGE2_Msk
            |TCC_PATT_PGE4_Msk|TCC_PATT_PGE5_Msk|TCC_PATT_PGE6_Msk),
            (TCC_PATT_PGE0(0)|TCC_PATT_PGE1(0)|TCC_PATT_PGE2(0)|TCC_PATT_PGE4(0)
            |TCC_PATT_PGE5(0)|TCC_PATT_PGE6(0)));
}

/*! \brief Enable inverter
 * 
 * Details.
 * Enable inverter
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_InverterEnable( void )
{
    /*Disable PWM override*/
    TCC0_PWMPatternSet(0x00,0x00);
}

/*! \brief Board services initialization 
 * 
 * Details.
 * Board services initialization  
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_BoardServicesInit( void )
{
//    mcBse_DelayState_mds.period = DELAY_10MS_COUNT;
//    ADC0_CallbackRegister((ADC_CALLBACK) ADC_CALIB_ISR, (uintptr_t)NULL);
//    EIC_CallbackRegister ((EIC_PIN)EIC_PIN_2, (EIC_CALLBACK) OC_FAULT_ISR,(uintptr_t)NULL);
//    PDEC_HALLCallbackRegister((PDEC_HALL_CALLBACK)HALL_jump_ISR, (uintptr_t)NULL);
//    
//    PDEC_HALLStart();  /* Start PDEC in HALL or QDEC mode. */
//    TCC0_PWMStart(); 
//    ADC0_Enable();
//    X2CScope_Init();
//    PWM_Output_Disable();
}

/*! \brief Current control initialization function 
 * 
 * Details.
 *  Current control initialization function 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_ButtonResponse(tmcBse_ButtonState_s * pButton, void (*function)(void))
{
    switch(pButton->buttonState)
    {
        case buttonState_Pressed: 
        {
           if(pButton->inputValue == 0u)
           {
                function();
                pButton->debounceCount = 0u;
                pButton->buttonState = buttonState_Released;
            }
        }
        break;
        
        case buttonState_Released: 
        {
            pButton->debounceCount++;
//            if(pButton->debounceCount >= CONSTANT_ButtonDebounceLoopCount )
            if(pButton->debounceCount >= SW_DEBOUNCE_DLY_500MS )
            {
                pButton->debounceCount = 0u;
                pButton->buttonState = buttonState_Pressed;
            }
        }
        break;
        
        default:
        {
            /* Should not come here */
        }
        break;
    }
}



/*! \brief Button scan function
 * 
 * Details.
 * Button scan function 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */

void mcBseI_ButtonScan(void)
{
    if(mcBse_DelayState_mds.count>mcBse_DelayState_mds.period)
    {
    #ifdef MCLV2
        button_S2_data.inputValue = BTN_START_STOP_Get();
    #endif
        mcBseI_ButtonResponse(&button_S2_data, &mcAppI_MotorStartStop);
        button_S3_data.inputValue = BTN_DIR_TGL_Get();
        mcBseI_ButtonResponse(&button_S3_data, &mcAppI_MotorDirectionToggle);
        mcSpeI_SpeedMeasurementRun(&mcSpeI_ModuleData_gds);
        mcBse_DelayState_mds.count = 0;    
    }
    else
    {
           
    }
}

void mcBseI_InverterDutySet( tmcMocI_SVPWM_s *duty)
{
   TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) duty->dPWM_A );
   TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) duty->dPWM_B );
   TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) duty->dPWM_C );
}

/*! \brief Set phase A current channel
 * 
 * Details.
 * Set phase A current channel
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_IaAdcInputChannelSet( void )
{
    ADC0_ChannelSelect(ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND);
}

/*! \brief Get phase A current input 
 * 
 * Details.
 * Get phase A current input 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int16_t mcBseI_IaAdcInputGet( void )
{
    return (int16_t)ADC0_ConversionResultGet();
}

/*! \brief Set phase B current channel
 * 
 * Details.
 * Set phase B current channel
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_IbAdcInputChannelSet( void )
{
    ADC1_ChannelSelect(ADC_POSINPUT_AIN0, ADC_NEGINPUT_GND); 
}

/*! \brief Get phase B current input 
 * 
 * Details.
 * Get phase B current input 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int16_t mcBseI_IbAdcInputGet( void )
{
    return (int16_t)ADC1_ConversionResultGet();
}

/*! \brief Set DC bus voltage channel
 * 
 * Details.
 * Set DC bus voltage channel
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_UdcAdcInputChannelSet( void )
{
      ADC1_ChannelSelect(ADC_POSINPUT_AIN14, ADC_NEGINPUT_GND);
}

/*! \brief Get DC bus voltage input 
 * 
 * Details.
 * Get DC bus voltage input 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int16_t mcBseI_UdcAdcInputGet( void )
{
    return (int16_t)ADC1_ConversionResultGet();
}

/*! \brief Set potentiometer channel
 * 
 * Details.
 * Set potentiometer channel
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_UpotAdcInputChannelSet( void )
{
    ADC0_ChannelSelect(ADC_POSINPUT_AIN6, ADC_NEGINPUT_GND);
}

/*! \brief Get potentiometer input 
 * 
 * Details.
 * Get potentiometer input 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
int16_t mcBseI_UpotAdcInputGet( void )
{
    return (int16_t)ADC0_ConversionResultGet();
}

/*! \brief Trigger ADC conversion
 * 
 * Details.
 * Trigger ADC conversion
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_TriggerAdcConversion( void )
{
    ADC0_ConversionStart();
}


/*! \brief Wait for  ADC conversion complete
 * 
 * Details.
 * Wait for  ADC conversion complete
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_WaitAdcConversion( void )
{
    while(ADC0_REGS->ADC_INTFLAG != ADC_INTFLAG_RESRDY_Msk);
}

/*! \brief Disable ADC interrupt 
 * 
 * Details.
 * Disable ADC interrupt 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_DisableAdcInterrupt( void )
{
    ADC0_InterruptsDisable(ADC_INTFLAG_RESRDY_Msk );
}

/*! \brief Enable ADC interrupt 
 * 
 * Details.
 * Enable ADC interrupt 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_EnableAdcInterrupt( void )
{
    ADC0_InterruptsEnable(ADC_INTFLAG_RESRDY_Msk );
}

/*! \brief Clear ADC interrupt flags
 * 
 * Details.
 * Clear ADC interrupt flags 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_ClearAdcInterruptFlag( void )
{
    ADC0_InterruptsClear( ADC_INTFLAG_Msk );
}