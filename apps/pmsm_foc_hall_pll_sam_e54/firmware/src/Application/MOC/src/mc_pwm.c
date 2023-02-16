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
#include "mc_pwm.h"


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
void mcPwm_CalcTimes(tmcMocI_SVPWM_s *svParam)
{
    svParam->T1 = svParam->ts * svParam->T1;
    svParam->T2 = svParam->ts * svParam->T2;
    svParam->Tc = (svParam->ts- svParam->T1 - svParam->T2)/2.0f;
    svParam->Tb = svParam->Tc + svParam->T2;
    svParam->Ta = svParam->Tb + svParam->T1;    
}  
/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
void mcPwmI_SpaceVectorModulationInit( tmcMocI_SVPWM_s * const svParam)
{
    /* Initialize space vector modulator */
    svParam->ts = (float)MAX_DUTY;
    svParam->dPWM_A = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
    svParam->dPWM_B = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
    svParam->dPWM_C = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
}

void mcPwmI_SpaceVectorModulationRun(tmcMocI_AB_s *alphabetaParam, tmcMocI_SVPWM_s *svParam)
{

    //Modified inverse clarke transform which allows using instantaneous phase 
    // value to be used directly to calculate vector times.
    svParam->ua = alphabetaParam->beta;
    svParam->ub = (-alphabetaParam->beta/2.0f + SQRT3_BY2 * alphabetaParam->alpha);
    svParam->uc = (-alphabetaParam->beta/2.0f - SQRT3_BY2 * alphabetaParam->alpha);     
    
    if( svParam->ua >= 0.0f )
    {       
		// (xx1)
        if( svParam->ub >= 0.0f )
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
           svParam->T1 = svParam->ub;
            svParam->T2 = svParam->ua;
            mcPwm_CalcTimes(svParam);
            svParam->dPWM_A = svParam->Ta;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Tc;
        }
        else
        {            
            // (x01)
            if( svParam->uc >= 0.0f )
            {
                // Sector 5: (1,0,1)  120-180 degrees
               svParam->T1 = svParam->ua;
               svParam->T2 = svParam->uc;
                mcPwm_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tc;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tb;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                svParam->T1 = -svParam->ub;
                svParam->T2 = -svParam->uc;
                mcPwm_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tc;
            }
        }
    }
    else
    {
        // (xx0)
        if( svParam->ub >= 0.0f )
        {
			// (x10)
            if( svParam->uc >= 0.0f )
            {
                // Sector 6: (1,1,0)  240-300 degrees
                svParam->T1 = svParam->uc;
                svParam->T2 = svParam->ub;
                mcPwm_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Tc;
                svParam->dPWM_C = svParam->Ta;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                svParam->T1 = -svParam->uc;
                svParam->T2 = -svParam->ua;
                mcPwm_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Ta;
                svParam->dPWM_B = svParam->Tc;
                svParam-> dPWM_C = svParam->Tb;
            }
        }
        else
        {            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            svParam->T1 = -svParam->ua;
            svParam->T2 = -svParam->ub;
            mcPwm_CalcTimes(svParam);
            svParam->dPWM_A = svParam->Tc;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Ta;

        }
    }
}

void mcPwmI_SpaceVectorModulationReset( tmcMocI_SVPWM_s * const svParam)
{
    /* Initialize space vector modulator */
    svParam->ts = (float)MAX_DUTY;
    svParam->dPWM_A = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
    svParam->dPWM_B = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
    svParam->dPWM_C = (float)((uint32_t)PWM_HALF_PERIOD_COUNT);
}