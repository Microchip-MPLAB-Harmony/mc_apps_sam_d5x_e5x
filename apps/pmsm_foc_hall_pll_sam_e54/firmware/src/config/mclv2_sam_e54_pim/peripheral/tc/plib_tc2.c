/*******************************************************************************
  Timer/Counter(TC2) PLIB

  Company
    Microchip Technology Inc.

  File Name
    plib_TC2.c

  Summary
    TC2 PLIB Implementation File.

  Description
    This file defines the interface to the TC peripheral library. This
    library provides access to and control of the associated peripheral
    instance.

  Remarks:
    None.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/* This section lists the other files that are included in this file.
*/
#include "interrupts.h"
#include "plib_tc2.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: TC2 Implementation
// *****************************************************************************
// *****************************************************************************

void TC2_CaptureInitialize( void )
{
    /* Reset TC */
    TC2_REGS->COUNT32.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC2_REGS->COUNT32.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode, prescaler, standby & on demand mode */
    TC2_REGS->COUNT32.TC_CTRLA = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC
                                  | TC_CTRLA_CAPTEN0_Msk  ;


    TC2_REGS->COUNT32.TC_EVCTRL = (uint16_t)(TC_EVCTRL_EVACT_STAMP | TC_EVCTRL_TCEI_Msk);

    /* Clear all interrupt flags */
    TC2_REGS->COUNT32.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;

}


void TC2_CaptureStart( void )
{
    /* Enable TC */
    TC2_REGS->COUNT32.TC_CTRLA |= TC_CTRLA_ENABLE_Msk;

    while((TC2_REGS->COUNT32.TC_SYNCBUSY & TC_SYNCBUSY_ENABLE_Msk) == TC_SYNCBUSY_ENABLE_Msk)
    {
        /* Wait for Write Synchronization */
    }
}

void TC2_CaptureStop( void )
{
    /* Disable TC */
    TC2_REGS->COUNT32.TC_CTRLA &= ~TC_CTRLA_ENABLE_Msk;

    while((TC2_REGS->COUNT32.TC_SYNCBUSY & TC_SYNCBUSY_ENABLE_Msk) == TC_SYNCBUSY_ENABLE_Msk)
    {
        /* Wait for Write Synchronization */
    }
}

uint32_t TC2_CaptureFrequencyGet( void )
{
    return (uint32_t)(30000000U);
}

void TC2_CaptureCommandSet(TC_COMMAND command)
{
    TC2_REGS->COUNT32.TC_CTRLBSET = (uint8_t)((uint32_t)command << TC_CTRLBSET_CMD_Pos);
    while((TC2_REGS->COUNT32.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
}


uint32_t TC2_Capture32bitChannel0Get( void )
{
    return TC2_REGS->COUNT32.TC_CC[0];
}

uint32_t TC2_Capture32bitChannel1Get( void )
{
    return TC2_REGS->COUNT32.TC_CC[1];
}


TC_CAPTURE_STATUS TC2_CaptureStatusGet(void)
{
    TC_CAPTURE_STATUS capture_status;
    capture_status =  ((TC_CAPTURE_STATUS)(TC2_REGS->COUNT32.TC_INTFLAG) & (TC_CAPTURE_STATUS)TC_CAPTURE_STATUS_MSK);
    TC2_REGS->COUNT32.TC_INTFLAG = (uint8_t)capture_status;
    return capture_status;
}
