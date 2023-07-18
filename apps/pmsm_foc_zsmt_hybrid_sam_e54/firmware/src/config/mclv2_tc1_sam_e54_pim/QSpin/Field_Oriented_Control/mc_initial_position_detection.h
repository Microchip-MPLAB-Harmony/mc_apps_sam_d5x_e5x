/*******************************************************************************
  Initial position detection header file

  File Name:
    mc_initial_position_detection.h

  Summary:
    Initial position detection header file
 
  Description:
    Initial position detection header file

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

#ifndef MCIPD_H
#define MCIPD_H

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/*******************************************************************************
 Header inclusions
 *******************************************************************************/
#include "mc_types.h"
#include "mc_current_calculation.h"
#include "mc_voltage_measurement.h"
#include "mc_hardware_abstraction.h"
#include "mc_key_manager.h"

/*******************************************************************************
 Configuration macros
 *******************************************************************************/

/*******************************************************************************
 User defined data-types
 *******************************************************************************/
typedef struct
{
   float32_t iA;
   float32_t iB;
   float32_t iC;
   float32_t uBus;
}tmcIpd_Input_s;

typedef struct
{
  float32_t uPulse;
  float32_t tPulse;
  float32_t tPeriod;
  float32_t fsInHertz;
  uint16_t  yPeriodCount;
}tmcIpd_Parameters_s;

typedef struct
{
   int16_t duty[3u];
   float32_t phi;
   bool ready;
}tmcIpd_Output_s;

typedef struct
{
   tmcIpd_Input_s dInput;
   tmcIpd_Parameters_s dParameter;
   tmcIpd_Output_s dOutput;
   void * pStatePointer;
}tmcIpd_ModuleData_s;

/*******************************************************************************
 Interface variables
 *******************************************************************************/
extern tmcIpd_ModuleData_s mcIpdI_ModuleData_gds;

/*******************************************************************************
 Static functions
 *******************************************************************************/
/*! \brief Read input ports
 *
 * Details.
 * Read input ports
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
__STATIC_INLINE void mcIpdI_InputPortsRead( tmcIpd_Input_s * const pInput )
{
     pInput->iA = mcCurI_ModuleData_gds.dOutput.iABC.a;
     pInput->iB = mcCurI_ModuleData_gds.dOutput.iABC.b;
     pInput->iC = mcCurI_ModuleData_gds.dOutput.iABC.c;
     
     pInput->uBus = mcVolI_ModuleData_gds.dOutput.uBus;
}

/*! \brief Read input ports
 *
 * Details.
 * Read input ports
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
__STATIC_INLINE void mcIpdI_OutputPortsWrite( tmcIpd_Output_s * const pOutput )
{
    mcPwmI_Duty_gau16[0u] =  pOutput->duty[0u];
    mcPwmI_Duty_gau16[1u] =  pOutput->duty[1u];
    mcPwmI_Duty_gau16[2u] =  pOutput->duty[2u];
}


/*! \brief Set module parameters
 *
 * Details.
 * Set module parameters
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
__STATIC_INLINE void mcIpdI_ParametersSet( tmcIpd_Parameters_s * const pParameter )
{
    pParameter->uPulse = (float32_t)(5);
    pParameter->tPulse = (float32_t)(0.0002);
    pParameter->tPeriod = (float32_t)(0.0055);
    pParameter->fsInHertz = (float32_t)(20000u);
    pParameter->yPeriodCount = mcHalI_PwmPeriodGet();
}

/*******************************************************************************
 Interface functions
 *******************************************************************************/

/*! \brief Initial position detection enable
 *
 * Details.
 * Initial position detection enable
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
void mcIpdI_InitialPositionDetectEnable(tmcIpd_ModuleData_s * const pModule);

/*! \brief Initial position detection disable
 *
 * Details.
 * Initial position detection disable
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
void mcIpdI_InitialPositionDetectDisable(tmcIpd_ModuleData_s * const pModule);

/*! \brief Initial position detection initialization
 *
 * Details.
 * Initial position detection initialization
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
void mcIpdI_InitialPositionDetectInit(tmcIpd_ModuleData_s * const pModule);


/*! \brief Initial position detection
 *
 * Details.
 * Initial position detection
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
void mcIpdI_InitialPositionDetect(tmcIpd_ModuleData_s * const pModule);

/*! \brief Initial position detection reset
 *
 * Details.
 * Initial position detection reset
 *
 * @param[in]: Input structure
 * @param[in/out]: State structure
 * @param[out]: Output structure
 * @return: None
 */
void mcIpdI_InitialPositionDetectReset(tmcIpd_ModuleData_s * const pModule);


#endif

#ifdef __cplusplus
}
#endif // MCIPD_H
