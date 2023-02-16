/*******************************************************************************
 * User Parameters 

  File Name:
    userparams.h

 Summary:
    Header file which defines Motor Specific and Board Specific constants 

  Description:
    This file contains the motor and board specific constants. It also defines
 * switches which allows algorithm to be run in debug modes like Open Loop Mode,
 * Torque mode, etc. 

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef USERPARAMS_H
#define USERPARAMS_H

#define     PWM_FREQ                                            20000           // PWM Frequency in Hz
#define     DELAY_MS                                            (float)10  // Delay in milliseconds after which Speed Ramp loop is executed
#define     SW_DEBOUNCE_DLY_MS                                  (float)500  // Switch debounce delay in mS


#define LEADSHINE

#undef MCLV2
#define MCHV3


#ifdef MCLV2
//===============================================================
// Following parameters for MCLV-2 board
// Gain of opamp = 15
// shunt resistor = 0.025 ohms
// DC offset = 1.65V
// max current = x
// (x * 0.025 * 15) + 1.65V = 3.3V
// x = 4.4Amps
#define     MAX_BOARD_CURRENT                                   (float)(4.4)            // Max Board Current in A
#define     DCBUS_SENSE_TOP_RESISTOR                            (float)30.0             // DC Bus voltage Divider - Top Side Resistor in Kohm
#define     DCBUS_SENSE_BOTTOM_RESISTOR                         (float)2.0              // DC Bus voltage Divider - Bottom Side Resistor in Kohm
#endif

#ifdef MCHV3
//===============================================================
// Following parameters for MCHV-3 board
// Gain of opamp = 10.06
// shunt resistor = 0.01 ohms
// DC offset = 1.65V
// max current = x
// (x * 0.01 * 10.06) + 1.65V = 3.3V
// x = 16.4Amps
#define     MAX_BOARD_CURRENT                                   (float)(16.4)           // Max Board Current in A
#define     DCBUS_SENSE_TOP_RESISTOR                            (float)285              // DC Bus voltage Divider - Top Side Resistor in Kohm
#define     DCBUS_SENSE_BOTTOM_RESISTOR                         (float)2.2              // DC Bus voltage Divider - Bottom Side Resistor in Kohm
#endif

#ifdef LEADSHINE
//---------------Motor Specifications : High Voltage Leadshine Motor : EL5-M0400-1-24// 
#define     MOTOR_PER_PHASE_RESISTANCE                          ((float)1.39)			// Per Phase Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE                          ((float)0.00253)		// Per Phase Inductance in Henrys
#define     BEMF_CNST_Vpk_PH_Line_Line_KRPM_MECH   (float)44.38			// Back EMF Constant in Vpeak(L-L)/KRPM 
#define     NOPOLESPAIRS                                        (float)5                       // Number of Pole Pairs of the PMSM Motor      
#define     STAR_CONNECTED_MOTOR                                1                       // 1 - Motor is Star Connected, 0 - Motor is Delta Connected
#define     NOMINAL_SPEED_RPM                                   (float)3000             // Nominal Rated Speed of the Motor - Value in RPM
#define     FW_SPEED_RPM                                        (float)5000             // Maximum Speed of the Motor in Flux Weakening Mode - Value in RPM
#define     ENCODER_PULSES_PER_REV                              ((float)10000)          // Motor Startup Behavior Configuration
#define     RPM_TO_ELEC_RDPS                             (float)((2.0f*M_PI*NOPOLESPAIRS)/(60.0f))
//--------------Motor Startup Behavior Configuration----------//
#define     ALIGN_TIME_IN_SEC                                   2                       // Duration of Motor Alignment in seconds
#define     OPENLOOP_RAMP_TIME_IN_SEC                           3                       // Ramp time to reach from 0 to Open Loop Speed in seconds
#define     OPENLOOP_END_SPEED_RPM                              500                     // Speed at which the motor switches from open loop to closed loop in RPM
#define     CLOSEDLOOP_RAMP_RATE_RPM_SEC                        200                     // Closed Loop Speed Ramp rate in Rev/min/Sec
#define     ALIGN_Q_CURRENT_REF                                 0.4f                     // Maximum Torque Reference during Motor Alignment in A
#define     OPENLOOP_Q_CURRENT_REF                              0.4f                     // Maximum Torque Reference during Open Loop Mode in A
#define     TORQUE_MODE_MAX_CUR                                 0.4f                     // Maximum Torque Mode Current Reference in A
#define     MAX_MOTOR_CURRENT                                   (float)(4.4)            // Maximum Motor Current in A

/* PI controllers tuning values - */
/********* D Control Loop Coefficients ****************************************/
#define     D_CURRCNTR_PTERM                                    ((float)0.05)
#define     D_CURRCNTR_ITERM                                    ((float)0.003)
#define     D_CURRCNTR_CTERM                                    ((float)0.999)
#define     D_CURRCNTR_OUTMAX                                   ((float)0.999)

/******** Q Control Loop Coefficients ****************************************/
#define     Q_CURRCNTR_PTERM                                    ((float)0.05)
#define     Q_CURRCNTR_ITERM                                    ((float)0.003)
#define     Q_CURRCNTR_CTERM                                    ((float)0.999)
#define     Q_CURRCNTR_OUTMAX                                   ((float)0.999)

/******* Speed Control Loop Coefficients **********************************/
#define     SPEEDCNTR_PTERM                                     ((float)0.005)
#define     SPEEDCNTR_ITERM                                     ((float)0.000005)
#define     SPEEDCNTR_CTERM                                     ((float)(0.999))
#define     SPEEDCNTR_OUTMAX                                    ((float)MAX_MOTOR_CURRENT)

//*** Position Control Loop Coefficients *****
#define     POSCNTR_PTERM                                      (float)(0.1)
#define     POSCNTR_ITERM                                      (float)(0.000000)
#define     POSCNTR_CTERM                                      (float)(0.0)
#define     POSCNTR_OUTMAX                                     (float)(1000.0f*RPM_TO_ELEC_RDPS)

#endif 



// <editor-fold defaultstate="collapsed" desc=" Derived Macros from Motor Control Board Specifications, Motor Specifications and Motor Dyanmics">

#define     PWM_PERIOD_COUNT                                    3000
#define     PWM_HALF_PERIOD_COUNT                               PWM_PERIOD_COUNT>>1
#define     MAX_MOTOR_CURRENT_SQUARED                           (float)((float)MAX_MOTOR_CURRENT*(float)MAX_MOTOR_CURRENT)
#define     VREF_DAC_VALUE                                      (int) 2048
#define     ADC_CURRENT_SCALE                                   (float)(MAX_BOARD_CURRENT/(float)2048)
#define     CURRENT_LIMIT_CMP_REF                               (int)(((float)2048*(MAX_MOTOR_CURRENT/MAX_BOARD_CURRENT))+VREF_DAC_VALUE)
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PER_PHASE_INDUCTANCE/(2*M_PI)))	
#define     MAX_ADC_COUNT                                       (float)4095     // for 12-bit ADC
#define     MAX_ADC_INPUT_VOLTAGE                               (float)3.3      // volts
#define     DCBUS_SENSE_RATIO                                   (float)(DCBUS_SENSE_BOTTOM_RESISTOR/(DCBUS_SENSE_BOTTOM_RESISTOR + DCBUS_SENSE_TOP_RESISTOR))
#define     VOLTAGE_ADC_TO_PHY_RATIO                            (float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUS_SENSE_RATIO))
#define     SINGLE_ELEC_ROT_RADS_PER_SEC                        (float)(2*M_PI)
#define     MAX_DUTY                                            (PWM_PERIOD_COUNT)
#define     LOOPTIME_SEC                                        (float)(1.0f/((float)PWM_FREQ))           // PWM Period - 50 uSec, 20Khz PWM
#define     COUNT_FOR_ALIGN_TIME                                (float)((float)ALIGN_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     ALIGN_CURRENT_STEP                                  (float)(2.0f*ALIGN_Q_CURRENT_REF/COUNT_FOR_ALIGN_TIME) // Current reference during aligning is ramped up for 50% of align time.#define     OPENLOOP_END_SPEED_RPS                              ((float)OPENLOOP_END_SPEED_RPM/60)
#define     OPENLOOP_END_SPEED_RPS                              ((float)OPENLOOP_END_SPEED_RPM/60.0f)
#define     OPNLP_END_SPEED_RDPS_MECH                (float)(OPENLOOP_END_SPEED_RPS * SINGLE_ELEC_ROT_RADS_PER_SEC)
#define     OPNLP_END_SPEED_RDPS_ELEC                (float)(OPNLP_END_SPEED_RDPS_MECH * NOPOLESPAIRS)
#define     OPNLP_END_SPEED_RDPS_ELEC_IN_LOOPTIME    (float)(OPNLP_END_SPEED_RDPS_ELEC * LOOPTIME_SEC)
#define     OPENLOOP_RAMPSPEED_INCREASERATE                     (float)(OPNLP_END_SPEED_RDPS_ELEC_IN_LOOPTIME/(OPENLOOP_RAMP_TIME_IN_SEC/LOOPTIME_SEC))
#define     CLOSEDLOOP_RAMP_RATE_RPS_SEC                        ((float)CLOSEDLOOP_RAMP_RATE_RPM_SEC/60.0f) // CLosed Loop  Speed Ramp rate in Rev/sec^2 
#define     CLLP_RMP_RT_RDPS2_MECH             (float)(CLOSEDLOOP_RAMP_RATE_RPS_SEC*2.0f*M_PIf) // CLosed Loop  Speed Ramp Rate in Mechanical Radians/Sec^2
#define     CLLP_RMP_RT_RDPS2_ELEC             (float)(CLLP_RMP_RT_RDPS2_MECH*NOPOLESPAIRS) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define     CLOSEDLOOP_SPEED_RAMP_RATE_DELTA                    (float)(CLLP_RMP_RT_RDPS2_ELEC*DELAY_MS*0.001f) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define     CLOSEDLOOP_SPEED_HYSTERESIS                         (float)(5*CLOSEDLOOP_SPEED_RAMP_RATE_DELTA)
#define     NOMINAL_SPEED_RDPS_ELEC                      (float)(((NOMINAL_SPEED_RPM/60.0f)*2.0f*M_PI)*NOPOLESPAIRS)   // Value in Electrical Radians/Sec

#if(STAR_CONNECTED_MOTOR==1)
    #define     BEMF_CNST_Vpk_PH_RPM_MECH    (float)((BEMF_CNST_Vpk_PH_Line_Line_KRPM_MECH/1.732f)/1000.0f)
#else
    #define     BEMF_CNST_Vpk_PH_RPM_MECH    (float)((BEMF_CNST_Vpk_PH_Line_Line_KRPM_MECH)/1000)
#endif 
#define     BEMF_CNST_Vpk_PH_RPS_MECH        (float)(BEMF_CNST_Vpk_PH_RPM_MECH * 60.0f)
#define     BEMF_CNST_Vpk_PH_RDPS_MECH (float)(BEMF_CNST_Vpk_PH_RPS_MECH/(2.0f*M_PI))
#define     BEMF_CNST_Vpk_PH_RDPS_ELEC (float)(BEMF_CNST_Vpk_PH_RDPS_MECH/NOPOLESPAIRS)
#define     INVKFi_BELOW_BASE_SPEED                              (float)(1/BEMF_CNST_Vpk_PH_RDPS_ELEC)
#define     DELAY_10MS_COUNT                                     (float)((float)PWM_FREQ*DELAY_MS*(float)0.001)
#define     SW_DEBOUNCE_DLY_500MS                                (uint32_t)(SW_DEBOUNCE_DLY_MS/DELAY_MS)  // Switch debounce duration in multiple of 10mS
#define     FW_SPEED_RDPS_ELEC                           (float)(((FW_SPEED_RPM/60.0f)*2.0f*M_PI)*NOPOLESPAIRS)
#define     MAX_NORM                                            (float) 0.95
#define     MAX_NORM_SQ                                         (float) (MAX_NORM*MAX_NORM)
#define     TORQUE_MODE_POT_ADC_RATIO                           (float) (TORQUE_MODE_MAX_CUR/MAX_ADC_COUNT)


#define     POT_ADC_COUNT_FW_SPEED_RATIO                       (float)(NOMINAL_SPEED_RDPS_ELEC/MAX_ADC_COUNT)


#define 	DECIMATE_NOMINAL_SPEED                              ((NOMINAL_SPEED_RPM *(M_PI/30))*NOPOLESPAIRS/10)




//--------------PDEC Configuration----------//
#define ENCODER_PULSES_PER_EREV                      (uint16_t)((uint16_t)ENCODER_PULSES_PER_REV/(uint16_t)NOPOLESPAIRS)
#define QDEC_RC                                       65536u
#define QDEC_UPPER_THRESHOLD                          49151u   
#define QDEC_LOWER_THRESHOLD                          16384u  
#define QDEC_OVERFLOW                                (uint16_t)(QDEC_RC % ENCODER_PULSES_PER_EREV) 
#define QDEC_UNDERFLOW                               (uint16_t)(ENCODER_PULSES_PER_EREV - QDEC_OVERFLOW)
#define FAST_LOOP_TIME_SEC                           (float)(1.0f/(float)PWM_FREQ)        /* Always runs in sync with PWM    */
#define SLOW_LOOP_FACTOR                             10U  
#define SLOW_LOOP_TIME_SEC                           (float)(FAST_LOOP_TIME_SEC * (float)SLOW_LOOP_FACTOR) /* 100 times slower than Fast Loop */
#define KFILTER_POT                    (float)((float)50/(float)32767) 
#endif
// </editor-fold>

