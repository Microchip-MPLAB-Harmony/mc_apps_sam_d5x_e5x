
/*******************************************************************************
  Motor Control Library Interface Header

  Company:
    Microchip Technology Inc. 

  File Name:  
    mc_Lib.h

  Summary:
    Motor Control Library Header File.

  Description:
    This file describes the macros, structures and APIs used by Motor Control Library for SAMD5x 
*******************************************************************************/


// DOM-IGNORE-BEGIN   
/*******************************************************************************
Copyright (c) <2018> released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#include <stdint.h>
#include "math.h"
#include "mc_userparams.h"

#ifndef _MCLIB_H
#define _MCLIB_H
// DOM-IGNORE-END

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

/*******************************************************************************
 Constants
 *******************************************************************************/
#define 	SQRT3_BY2     			(float)0.866025403788
#define 	ONE_BY_SQRT3     		(float)0.5773502691

#define 	ANGLE_2PI              	(2*M_PI) 
#define 	TABLE_SIZE      		256 
#define 	ANGLE_STEP      		(float)((float)ANGLE_2PI/(float)TABLE_SIZE) 
#define     ONE_BY_ANGLE_STEP       (float)(1/ANGLE_STEP)
        
#define     TC_HALL_FIFO_LEN        6u       
        
                
/*******************************************************************************
 User defined data-types 
 *******************************************************************************/
typedef enum _tmcMoc_ControlStates_s
{
    ControlState_Idle,  
    ControlState_HallTune,
    ControlState_CurrentLoopTune,
    ControlState_OpenLoop,
    ControlState_SixStep,
    ControlState_SensoredFoc,
    ControlState_SensorlessFoc
}tmcMoc_ControlStates_s;

typedef struct
{
    uint16_t state;
    uint16_t isSttChanged;
    uint32_t cnt32_1;
} tagState;

typedef struct 
{
    float   alpha; 
    float   beta;

} tmcMocI_AB_s;

//Structure containing component values for 2 Phase Rotating Reference Frame
typedef struct 
{
    float   d;     // D axis component in 2 Phase Rotating Frame 
    float   q;      // Q axis component  in 2 Phase Rotating Frame

} tmcMocI_DQ_s;

//Structure containing component values for 3 Phase Stationary Reference Frame
typedef struct 
{
    float   a; 
    float   b; 
    float   c; 
} tmcMocI_ABC_s;

//Structure containing variables used for Sine & Cosine calculation 
typedef struct 
{
    float   angle;
    float   Sin; 
    float   Cos;
} tmcMocI_PHASOR_s;

typedef struct 
{
    float   Yi; 
    float   Kp;    
    float   Ki;    
    float   Kc;   
    float   Ymax;
    float   Ymin;
    float   reference; 
    float   feedback;
    float   Yout;
    float   error;
} tmcMocI_PI_s;

//Structure containing variables used in calculating Space Vector PWM Duty cycles
typedef struct 
{
    float   ts;  
    float   ua;     
    float   ub; 
    float   uc;
    float   T1; 
    float   T2;
    float   Ta;
    float   Tb;  
    float   Tc; 
    float   dPWM_A;
    float   dPWM_B;
    float   dPWM_C;
} tmcMocI_SVPWM_s;

typedef struct _tmcMoc_ControlFlags_s
{
    uint8_t start;
    uint8_t openLoop;
    float direction;
}tmcMoc_ControlFlags_s;

typedef struct _mcMoc_MotorControl_s
{
    uint8_t runStatus;
    uint8_t runCommand;
    uint8_t stateMachine;
    float sensedAngle;
    float sensorlessAngle;
    float sensedSpeed;
    float sensorlessSpeed;
    tmcMoc_ControlFlags_s Flags;
    tmcMocI_ABC_s iABC;
    tmcMocI_ABC_s uABC;
    tmcMocI_AB_s  iAB;
    tmcMocI_AB_s  uAB;
    tmcMocI_DQ_s  iDQ;
    tmcMocI_DQ_s  uDQ;
    tmcMocI_PI_s  iDPI;
    tmcMocI_PI_s  iQPI;
    tmcMocI_PI_s  nQPI;
    tmcMocI_SVPWM_s  ySVPWM; 
    tmcMocI_PHASOR_s tPHASOR;
}tmcMocI_MotorControl_s;

/*******************************************************************************
 Interface functions
 *******************************************************************************/
/*! \brief Clarke Transformation
 * 
 * Details.
 * Clarke Transformation
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_ClarkeTransform(tmcMocI_ABC_s *abcParam, tmcMocI_AB_s *alphabetaParam);

/*! \brief Park Transformation
 * 
 * Details.
 * Park Transformation
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_ParkTransform(tmcMocI_AB_s *alphabetaParam , tmcMocI_PHASOR_s *scParam, tmcMocI_DQ_s *dqParam);

/*! \brief Inverse Park Transformation
 * 
 * Details.
 * Inverse Park Transformation
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_InvParkTransform(tmcMocI_DQ_s *dqParam, tmcMocI_PHASOR_s *scParam,tmcMocI_AB_s *alphabetaParam);

/*! \brief Control parameters initialization
 * 
 * Details.
 * Control parameters initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcApp_InitControlParameters(void);

/*! \brief PI controller initialization
 * 
 * Details.
 * PI controller initialization
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_InitPI( tmcMocI_PI_s *pParam);

/*! \brief PI controller
 * 
 * Details.
 * PI controller
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_PiControlRun( tmcMocI_PI_s *pParam);

/*! \brief Sine and cosine calculation
 * 
 * Details.
 * Sine and cosine calculation
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcLib_SinCosGen(tmcMocI_PHASOR_s *scParam);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // #ifndef _MC_LIB_H
/*******************************************************************************
 End of File
*/
