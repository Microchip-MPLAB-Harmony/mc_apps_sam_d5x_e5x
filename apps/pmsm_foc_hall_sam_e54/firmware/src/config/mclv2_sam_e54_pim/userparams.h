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

#include <math.h>

#define     PWM_CLK                                             (120000000UL)   // PWM Peripheral Input Clock Frequency in Hz
#define     PWM_FREQ                                            20000U           // PWM Frequency in Hz
#define     DELAY_MS                                            10.0f  // Delay in milliseconds after which Speed Ramp loop is executed
#define     SW_DEBOUNCE_DLY_MS                                  500.0f  // Switch debounce delay in mS

#define LONG_HURST

#define MCLV2

/*Define macro TORQUE_MODE to run the motor in TORQUE Control Mode
 Undefine macro TORQUE_MODE to run the motor in Speed Control Mode*/
#undef TORQUE_MODE


/*Define macro ENABLE_FLUX_WEAKENING to enable Flux Weakening
 Undefine macro ENABLE_FLUX_WEAKENING to enable Flux Weakening*/
#undef ENABLE_FLUX_WEAKENING

#define     KFILTER_IDREF               (float)((float)(10)/(float)32767)               // Flux Weakening mode - D axis Current Reference Filter

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

#ifdef LONG_HURST
//---------------Motor Specifications : Hurst Motor With Quadrature Encoder : DMA0204024B101 (a.k.a. "Long" Hurst Motor)
#define     MOTOR_PER_PHASE_RESISTANCE                          ((float)0.285)			// Per Phase Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE                          ((float)0.00032)		// Per Phase Inductance in Henrys
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH   (float)6.6				// Back EMF Constant in Vpeak(L-L)/KRPM 
#define     NOPOLESPAIRS                                        5.0f                       // Number of Pole Pairs of the PMSM Motor        
#define     STAR_CONNECTED_MOTOR                                1                       // 1 - Motor is Star Connected, 0 - Motor is Delta Connected
#define     NOMINAL_SPEED_RPM                                   (float)2804             // Nominal Rated Speed of the Motor - Value in RPM
#define     FW_SPEED_RPM                                        (float)3500             // Maximum Speed of the Motor in Flux Weakening Mode - Value in RPM
//--------------Motor Startup Behavior Configuration----------//
#define     ALIGN_TIME_IN_SEC                                   2                       // Duration of Motor Alignment in seconds
#define     OPENLOOP_RAMP_TIME_IN_SEC                           3                       // Ramp time to reach from 0 to Open Loop Speed in seconds
#define     OPENLOOP_END_SPEED_RPM                              500                     // Speed at which the motor switches from open loop to closed loop in RPM

#define     CLOSEDLOOP_RAMP_RATE_RPM_SEC                        500                     // Closed Loop Speed Ramp rate in Rev/min/Sec
#define     ALIGN_D_CURRENT_REF                                 0.4                     // Maximum Torque Reference during Motor Alignment in A
#define     OPENLOOP_D_CURRENT_REF                              0.4                     // Maximum Torque Reference during Open Loop Mode in A
#define     MAX_FW_NEGATIVE_ID_REF                              (float)(-2.0)           // Maximum negative D axis reference current (in A) during Flux Weakening
#define     TORQUE_MODE_MAX_CUR                                 0.4                     // Maximum Torque Mode Current Reference in A
#define     MAX_MOTOR_CURRENT                                   (float)(4.4)            // Maximum Motor Current in A
#define     WINDMILL_TIME_SEC                                   0.5                       // Duration of Motor Windmilling in seconds
#define     WINDMILL_START_Q_AXIS_REF                           0.4
#define     REGEN_BRAKE_CURRENT_REF                             0.4
#define     PASSIVE_BRAKE_TIME_IN_SEC                           2
#define     D_CURRENT_REF_FALL_TIME_SEC                         (float)(1.0)            // D axis Current Reference Fall Time in Seconds
#define     REGEN_BRAKE_CURRENT_RAMP_TIME_SEC                   (float)(1.0)
#define     MIN_WM_SPEED_IN_RPM                                 (float)(200)
/* PI controllers tuning values - */

//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM                                    0.08f                    // D axis Proportional Gain
#define     D_CURRCNTR_ITERM                                    (0.0008f)                // D axis Integral Gain
#define     D_CURRCNTR_CTERM                                    0.5f                     // D axis Anti-Windup Gain
#define     D_CURRCNTR_OUTMAX                                   0.999f                   // D axis PI Controller Maximum Output - Max D axis Voltage (Normalized)    

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM                                    0.08f                    // Q axis Proportional Gain
#define     Q_CURRCNTR_ITERM                                    (0.0008f)                // Q axis Integral Gain
#define     Q_CURRCNTR_CTERM                                    0.5f                     // Q axis Anti-Windup Gain
#define     Q_CURRCNTR_OUTMAX                                   0.999f                   // Q axis PI Controller Maximum Output - Max D axis Voltage (Normalized)    
//*** Speed Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM                                     (0.0040f)                 // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM                                     (0.00000010f)            // Speed Loop Integral Gain
#define     SPEEDCNTR_CTERM                                     0.5f                     // Speed Loop Anti-Windup Gain
#define     SPEEDCNTR_OUTMAX                                    MAX_MOTOR_CURRENT       // Speed Loop PI Controller Maximum Output - Max Q axis Current Reference in A

/* motor dependent HALL constants */
#define     RL_30_HALL_JUMP_CW  0x0405U
#define     RL_30_HALL_JUMP_CCW  0x0504U
#define     RL_90_HALL_JUMP_CW  0x0501U
#define     RL_90_HALL_JUMP_CCW  0x0105U
#define     RL_150_HALL_JUMP_CW  0x0103U
#define     RL_150_HALL_JUMP_CCW  0x0301U
#define     RL_210_HALL_JUMP_CW  0x0302U
#define     RL_210_HALL_JUMP_CCW  0x0203U
#define     RL_270_HALL_JUMP_CW  0x0206U
#define     RL_270_HALL_JUMP_CCW  0x0602U
#define     RL_330_HALL_JUMP_CW  0x0604U
#define     RL_330_HALL_JUMP_CCW  0x0406U
#endif

/* HALL constants */
#define RL_ONE_OVER_SIX_PI (float)(1.0 / 6.0 * M_PI)
#define RL_ONE_OVER_TWO_PI (float)(0.5 * M_PI)
#define RL_FIVE_OVER_SIX_PI (float)(5.0 / 6.0 * M_PI)
#define RL_SEVEN_OVER_SIX_PI (float)(7.0 / 6.0 * M_PI)
#define RL_THREE_OVER_TWO_PI (float)(1.5 * M_PI)
#define RL_ELEVEN_OVER_SIX_PI (float)(11.0 / 6.0 * M_PI)
/**/
#define RL_ONE_OVER_THREE_PI (float)(1.0 / 3.0 * M_PI)
#define RL_TWO_OVER_THREE_PI (float)(2.0 / 3.0 * M_PI)
#define RL_PI (float)(M_PI)
#define RL_FOUR_OVER_THREE_PI (float)(4.0 / 3.0 * M_PI)
#define RL_FIVE_OVER_THREE_PI (float)(5.0 / 3.0 * M_PI)
#define RL_TWO_PI (float)(2.0 * M_PI)
#define RL_DUMM (0.0f)
/**/
#define RL_TC0_FREQ (30000000.0f)
#define RL_TCO_TS (float)(1.0 / RL_TC0_FREQ)
#define RL_2PI_TC0_FREQ (float)(RL_TWO_PI * RL_TC0_FREQ)
#define NOMINAL_ELE_VEL_RAD_PER_SEC (float)(NOMINAL_SPEED_RPM * (float)NOPOLESPAIRS * RL_TWO_PI / 60.0f)
#define CTC_ELE_VEL_1_HIGH_BOUND (float)(NOMINAL_ELE_VEL_RAD_PER_SEC * 0.1f)
#define CTC_ELE_VEL_1_LOW_BOUND (float)(NOMINAL_ELE_VEL_RAD_PER_SEC * 0.05f)
#define CTC_ELE_VEL_2_HIGH_BOUND (float)(NOMINAL_ELE_VEL_RAD_PER_SEC * 0.6f)
#define CTC_ELE_VEL_2_LOW_BOUND (float)(NOMINAL_ELE_VEL_RAD_PER_SEC * 0.4f)
/**/
#define RL_LPF_BW 200.0f
#define MC_ISR_T (float)(1.0f / (float)PWM_FREQ)
#define RL_LPF_COEFF_2 (float)(RL_LPF_BW * MC_ISR_T)
#define RL_LPF_COEFF_1 (float)(1.0 - RL_LPF_COEFF_2)

// <editor-fold defaultstate="collapsed" desc=" Derived Macros from Motor Control Board Specifications, Motor Specifications and Motor Dyanmics">

#define     PWM_PERIOD_COUNT                                    (((PWM_CLK/PWM_FREQ)/2U))
#define     PWM_HALF_PERIOD_COUNT                               PWM_PERIOD_COUNT>>1U
#define     MAX_MOTOR_CURRENT_SQUARED                           (float)((float)MAX_MOTOR_CURRENT*(float)MAX_MOTOR_CURRENT)
#define     VREF_DAC_VALUE                                      (int) 2048
#define     ADC_CURRENT_SCALE                                   (float)(MAX_BOARD_CURRENT/(float)2048)
#define     CURRENT_LIMIT_CMP_REF                               (int)(((float)2048*(MAX_MOTOR_CURRENT/MAX_BOARD_CURRENT))+VREF_DAC_VALUE)
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PER_PHASE_INDUCTANCE/(2*M_PI)))	
#define     MAX_ADC_COUNT                                       (float)4095     // for 12-bit ADC
#define     MAX_ADC_INPUT_VOLTAGE                               (float)3.3      // volts
#define     DCBUS_SENSE_RATIO                                   (float)(DCBUS_SENSE_BOTTOM_RESISTOR/(DCBUS_SENSE_BOTTOM_RESISTOR + DCBUS_SENSE_TOP_RESISTOR))
#define     VOLTAGE_ADC_TO_PHY_RATIO                            (float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUS_SENSE_RATIO))
#define     SINGLE_ELEC_ROT_RADS_PER_SEC                        (float)(2.0f*M_PI)
#define     MAX_DUTY                                            (PWM_PERIOD_COUNT)
#define     LOOPTIME_SEC                                        (float)(1/((float)PWM_FREQ))           // PWM Period - 50 uSec, 20Khz PWM
#define     COUNT_FOR_ALIGN_TIME                                (unsigned int)((float)ALIGN_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     COUNT_FOR_WINDMILLING_TIME                          (unsigned int)((float)WINDMILL_TIME_SEC/(float)LOOPTIME_SEC)
#define     COUNT_FOR_PASSIVE_BRAKE_TIME                        (unsigned int)((float)PASSIVE_BRAKE_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     ALIGN_CURRENT_STEP                                  (float)(2*ALIGN_D_CURRENT_REF/COUNT_FOR_ALIGN_TIME) // Current reference during aligning is ramped up for 50% of align time.
#define     OPENLOOP_END_SPEED_RPS                              ((float)OPENLOOP_END_SPEED_RPM/60.0f)
#define     OPNLP_END_SPEED_RDPS_MECH                (float)(OPENLOOP_END_SPEED_RPS * SINGLE_ELEC_ROT_RADS_PER_SEC)
#define     OPNLP_END_SPEED_RDPS_ELEC                (float)(OPNLP_END_SPEED_RDPS_MECH * NOPOLESPAIRS)
#define     OPNLP_END_SPEED_RDPS_ELEC_IN_LOOPTIME    (float)(OPNLP_END_SPEED_RDPS_ELEC * LOOPTIME_SEC)
#define     OPENLOOP_RAMPSPEED_INCREASERATE                     (float)(OPNLP_END_SPEED_RDPS_ELEC_IN_LOOPTIME/(OPENLOOP_RAMP_TIME_IN_SEC/LOOPTIME_SEC))
#define     CLOSEDLOOP_RAMP_RATE_RPS_SEC                        ((float)CLOSEDLOOP_RAMP_RATE_RPM_SEC/60.0f) // CLosed Loop  Speed Ramp rate in Rev/sec^2 
#define     CLLP_RMP_RT_RDPS2_MECH             (float)(CLOSEDLOOP_RAMP_RATE_RPS_SEC*2.0f*M_PI) // CLosed Loop  Speed Ramp Rate in Mechanical Radians/Sec^2
#define     CLLP_RMP_RT_RDPS2_ELEC             (float)(CLLP_RMP_RT_RDPS2_MECH*NOPOLESPAIRS) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define     CLOSEDLOOP_SPEED_RAMP_RATE_DELTA                    (float)(CLLP_RMP_RT_RDPS2_ELEC*DELAY_MS*0.001f) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define     CLOSEDLOOP_SPEED_HYSTERESIS                         (float)(5.0f*CLOSEDLOOP_SPEED_RAMP_RATE_DELTA)
#define     NOMINAL_SPEED_RAD_PER_SEC_ELEC                      (float)(((NOMINAL_SPEED_RPM/60.0f)*2.0f*M_PI)*NOPOLESPAIRS)   // Value in Electrical Radians/Sec
#define     MIN_WM_SPEED_SPEED_ELEC_RAD_PER_SEC                 (float)(((MIN_WM_SPEED_IN_RPM/60.0f)*2.0f*M_PI)*NOPOLESPAIRS)    // Value in Electrical Radians/Sec
#if(STAR_CONNECTED_MOTOR==1)
    #define     BEMF_CNST_Vpk_PH_RPM_MECH    (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH/1.732)/1000)
#else
    #define     BEMF_CNST_Vpk_PH_RPM_MECH    (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH)/1000)
#endif 
#define     BEMF_CNST_Vpk_PH_RPS_MECH        (float)(BEMF_CNST_Vpk_PH_RPM_MECH * 60)
#define     BEMF_CNST_Vpk_PH_RAD_PER_SEC_MECH (float)(BEMF_CNST_Vpk_PH_RPS_MECH/(2*M_PI))
#define     BEMF_CNST_Vpk_PH_RAD_PER_SEC_ELEC (float)(BEMF_CNST_Vpk_PH_RAD_PER_SEC_MECH/NOPOLESPAIRS)
#define     INVKFi_BELOW_BASE_SPEED                              (float)(1/BEMF_CNST_Vpk_PH_RAD_PER_SEC_ELEC)
#define     DELAY_10MS_COUNT                                     (float)((float)PWM_FREQ*DELAY_MS*0.001f)
#define     SW_DEBOUNCE_DLY_500MS                                (float)(SW_DEBOUNCE_DLY_MS/DELAY_MS)  // Switch debounce duration in multiple of 10mS
#define     RHO_OFFSET_ELEC_RAD                                  (float)(RHO_OFFSET_ELEC_DEG*M_PI/180)
#define     MIN_RHO_OFFSET_ELEC_RAD                              (float)(RHO_OFFSET_ELEC_RAD/PWM_FREQ)

#define     FW_SPEED_RAD_PER_SEC_ELEC                           (float)(((FW_SPEED_RPM/60)*2*M_PI)*NOPOLESPAIRS)
#define     MAX_NORM                                            (float) 0.95
#define     MAX_NORM_SQ                                         (float) (MAX_NORM*MAX_NORM)
#define     TORQUE_MODE_POT_ADC_RATIO                           (float) (TORQUE_MODE_MAX_CUR/MAX_ADC_COUNT)

#ifdef ENABLE_FLUX_WEAKENING
    #define     POT_ADC_COUNT_FW_SPEED_RATIO                    (float)(FW_SPEED_RAD_PER_SEC_ELEC/MAX_ADC_COUNT)
#else
    #define     POT_ADC_COUNT_FW_SPEED_RATIO                    (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC/MAX_ADC_COUNT)
#endif

#define 	DECIMATE_NOMINAL_SPEED                              ((NOMINAL_SPEED_RPM *(M_PI/30))*NOPOLESPAIRS/10)
#ifdef      CURPI_TUN
    #define CPT_CNT_VAL   (float)(CUR_STEP_TIM*PWM_FREQ)
#endif 

#define     D_CURRENT_REF_STEP                  (float)(ALIGN_D_CURRENT_REF/(D_CURRENT_REF_FALL_TIME_SEC*PWM_FREQ))
#define     REGEN_BRAKE_CURRENT_STEP            (float)(REGEN_BRAKE_CURRENT_REF/(REGEN_BRAKE_CURRENT_RAMP_TIME_SEC*PWM_FREQ))


#endif
 
// </editor-fold>

