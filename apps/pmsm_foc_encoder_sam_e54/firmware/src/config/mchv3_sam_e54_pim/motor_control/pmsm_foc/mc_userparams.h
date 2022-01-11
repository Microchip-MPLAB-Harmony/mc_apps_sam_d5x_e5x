 /*******************************************************************************
 User Parameters interface file

  Company:
    Microchip Technology Inc.

  File Name:
    mc_userparams.h

  Summary:
    Motor control parameters interface

  Description:
    This file contains the motor parameters and hardware board parameters
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _USER_HEADER
#define _USER_HEADER

#include "mc_pmsm_foc_common.h"
/***********************************************************************************************/
/*                    include files                                                            */
/***********************************************************************************************/

/***********************************************************************************************/
/* USER CONFIGURABLE PARAMETERS - START                                                        */
/***********************************************************************************************/

/***********************************************************************************************/
/* Algorithm Configuration parameters                                                          */
/***********************************************************************************************/
#define BOARD                            (MCHV3)                            
#define POSITION_FEEDBACK                (SENSORED_ENCODER)

#define CONTROL_LOOP                     (SPEED_LOOP)
#define FIELD_WEAKENING                  (DISABLED)  /* If enabled - Field weakening */
#define ALIGNMENT_METHOD                 (Q_AXIS)  /* alignment method  */


#define CURRENT_MEASUREMENT              (DUAL_SHUNT)  /* Current measurement shunts */

#define POTENTIOMETER_INPUT_ENABLED       ENABLED

#define SPEED_REF_RPM                     (float)1000   /* Speed Ref */

/***********************************************************************************************/
/* Motor Configuration Parameters */
/***********************************************************************************************/
#define MOTOR_PER_PHASE_RESISTANCE                          ((float)1.39)
#define MOTOR_PER_PHASE_INDUCTANCE                          ((float)0.00253)
#define MOTOR_BEMF_CONST_V_PEAK_LL_KRPM_MECH                ((float)44.380001)
#define NUM_POLE_PAIRS                                      ((float)5)
#define RATED_SPEED_RPM                                     ((float)3000)
#define MAX_SPEED_RPM                                       ((float)5000)
#define MAX_MOTOR_CURRENT                                   ((float)4)
#define MOTOR_CONNECTION                                    (STAR)
#define ENCODER_PULSES_PER_REV                              ((float)2500)

/*****************************************************************************/
/* Control Parameters                                                        */
/*****************************************************************************/

/* Motor Start-up configuration parameters */
#define LOCK_TIME_IN_SEC                (2)   /* Startup - Rotor alignment time */

#define OPEN_LOOP_END_SPEED_RPM         (500) /* Startup - Control loop switches to close loop at this speed */
#define OPEN_LOOP_RAMP_TIME_IN_SEC      (5)   /* Startup - Time to reach OPEN_LOOP_END_SPEED_RPM in seconds */

#define Q_CURRENT_REF_OPENLOOP          (0.4) /* Startup - Motor start to ramp up in current control mode */


/* Current ramp parameters for open loop to close loop transition  */
#define Q_CURRENT_OPENLOOP_STEP                    ((float)0.001)
#define CLOSING_LOOP_TIME_COUNTS                   (uint32_t)( Q_CURRENT_REF_OPENLOOP / Q_CURRENT_OPENLOOP_STEP)

/* Field weakening - Limit for -ve Idref */
#if(FIELD_WEAKENING == ENABLED)
#define MAX_FW_NEGATIVE_ID_REF              (float)(-2.5)
#endif

/******************************************************************************/
/* PI controllers tuning values - */

/********* D Control Loop Coefficients ****************************************/
#define     D_CURRCNTR_PTERM           (float)(0.02)
#define     D_CURRCNTR_ITERM           (float)(0.000099)
#define     D_CURRCNTR_CTERM           (float)(0.5)
#define     D_CURRCNTR_OUTMAX          (float)(0.98)

/******** Q Control Loop Coefficients ****************************************/
#define     Q_CURRCNTR_PTERM           (float)(0.02)
#define     Q_CURRCNTR_ITERM           (float)(0.000099)
#define     Q_CURRCNTR_CTERM           (float)(0.5)
#define     Q_CURRCNTR_OUTMAX          (float)(0.98)

/******* Velocity Control Loop Coefficients **********************************/
#define     SPEEDCNTR_PTERM            (float)(0.005)
#define     SPEEDCNTR_ITERM            (float)(0.000002 * 0.01f)
#define     SPEEDCNTR_CTERM            (float)(0.5)
#define     SPEEDCNTR_OUTMAX           (float)(4)


/* First order low pass Filter constants used inside the project             */
#define KFILTER_ESDQ                   (float)((float)0.122)
#define KFILTER_BEMF_AMPLITUDE         (float)((float)4000/(float)32767)
#define KFILTER_VELESTIM               (float)((float)0.122)
#define KFILTER_POT                    (float)((float)250/(float)32767)

/***********************************************************************************************/
/* Driver board configuration Parameters */
/***********************************************************************************************/

#define MAX_CURRENT                                         ((float)(16.40159)) /* Max current as per above calculations */
#define MAX_ADC_COUNT                                       (float)4095     /*  ADC Resolution*/
#define MAX_ADC_INPUT_VOLTAGE                               (float)3.3      /* volts */

#define DC_BUS_VOLTAGE                                      (float)400     /* Volts */
#define DCBUS_SENSE_RATIO                                   (float)0.00766    /* Voltage divider ratio R2/(R2 + R1) */

#define STATOR_VOLTAGE_LIMIT                                (float)(0.98)   /* In percentage */

/***********************************************************************************************/
/* Peripheral Configuration parameters */
/***********************************************************************************************/

/** PWM frequency in Hz */
#define PWM_FREQUENCY                     (20000U)

/**********************************************************************************************/

/***********************************************************************************************/
/* USER CONFIGURABLE PARAMETERS - END                                                          */
/***********************************************************************************************/

#endif
