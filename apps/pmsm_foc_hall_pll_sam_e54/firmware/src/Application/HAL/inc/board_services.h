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

#ifndef MCBSE_H_    // Guards against multiple inclusion
#define MCBSE_H_

/*******************************************************************************
 Header files inclusions
 ******************************************************************************/
#include <stddef.h>

/*******************************************************************************
 Default module parameters 
 ******************************************************************************/


/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef enum _tmcHal_ButtonState_e
{
    buttonState_Pressed,
    buttonState_Released           
}tmcBse_ButtonState_e;

typedef struct _tmcHal_ButtonState_s
{
    tmcBse_ButtonState_e buttonState;
    uint32_t inputValue;
    uint32_t debounceCount;
}tmcBse_ButtonState_s;


typedef struct _tmcBse_DelayGeneration_s
{
    uint16_t count;
    uint16_t period;
}tmcBse_DelayGeneration_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
extern tmcBse_DelayGeneration_s mcBse_DelayState_mds;
/*******************************************************************************
 Interface functions 
 *******************************************************************************/
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
void mcBseI_BoardServicesInit( void );

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
void mcBseI_ButtonResponse(tmcBse_ButtonState_s * pButton, void (*function)(void));

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
void mcBseI_ButtonScan(void);

/*! \brief Set inverter duty
 * 
 * Details.
 * Set inverter duty
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcBseI_InverterDutySet( tmcMocI_SVPWM_s *duty);

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
void mcBseI_InverterDisable( void );

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
void mcBseI_InverterEnable( void );

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
void mcBseI_IaAdcInputChannelSet( void );

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
int16_t mcBseI_IaAdcInputGet( void );

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
void mcBseI_IbAdcInputChannelSet( void );

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
int16_t mcBseI_IbAdcInputGet( void );

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
void mcBseI_UdcAdcInputChannelSet( void );

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
int16_t mcBseI_UdcAdcInputGet( void );

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
void mcBseI_UpotAdcInputChannelSet( void );

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
int16_t mcBseI_UpotAdcInputGet( void );

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
void mcBseI_TriggerAdcConversion( void );

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
void mcBseI_WaitAdcConversion( void );



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
void mcBseI_DisableAdcInterrupt( void );

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
void mcBseI_EnableAdcInterrupt( void );

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
void mcBseI_ClearAdcInterruptFlag( void );

#endif //MCBSE_H_

/**
 End of File
*/
