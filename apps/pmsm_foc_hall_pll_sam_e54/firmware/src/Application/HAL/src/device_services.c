/*******************************************************************************
 Device services Source file 

  Company:
    Microchip Technology Inc.

  File Name:
    device_services.c

  Summary:
    This file contains all the functions related to device services

  Description:
    This file contains implementation of to device services
 
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
#include "device_services.h"

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


/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Timer callback function 
 * 
 * Details.
 * Timer callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_HallCallbackFunction(void)
{
    
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
void mcHalI_AdcCallbackFunction(void)
{
    
}

/*! \brief External interrupt callback function 
 * 
 * Details.
 * External interrupt callback function
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_EicCallbackFunction(void)
{
    
}

/*! \brief Get hall pattern 
 * 
 * Details.
 * Get hall pattern 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcHalI_HallPatternGet(void)
{
    
}


/* Get the current timer counter value */
uint32_t TC2_Timer32bitCounterGet( void )
{
    /* Write command to force COUNT register read synchronization */
    TC2_REGS->COUNT32.TC_CTRLBSET |= TC_CTRLBSET_CMD_READSYNC;

    while((TC2_REGS->COUNT32.TC_SYNCBUSY & TC_SYNCBUSY_CTRLB_Msk) == TC_SYNCBUSY_CTRLB_Msk)
    {
        /* Wait for Write Synchronization */
    }

    while((TC2_REGS->COUNT32.TC_CTRLBSET & TC_CTRLBSET_CMD_Msk) != 0)
    {
        /* Wait for CMD to become zero */
    }
    
    /* Read current count value */
    return TC2_REGS->COUNT32.TC_COUNT;

}

void mcDseI_PeripheralsInit( void )
{
   
}

void mcDseI_AdcSoftwareTrigger( void )
{
    
}
