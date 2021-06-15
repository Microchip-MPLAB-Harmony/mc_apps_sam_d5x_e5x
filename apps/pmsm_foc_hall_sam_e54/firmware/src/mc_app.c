/*******************************************************************************
 Motor Control Application Source File

  File Name:
    mc_app.c

  Summary:
 Motor Control Application Variable and Function definitions.

  Description:
    This file contains the variable initializations and function definitions for
 *  specific to Motor Control Application (excluding variables and functions 
 *  defined by Motor ControlLibrary
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#include "userparams.h"
#include <math.h>
#include "mc_app.h"
#include "mc_Lib.h"
#include "definitions.h"


#define STT_HALL_ISR_INI 0u
#define STT_HALL_ISR_1ST_JUMP 1u
#define STT_HALL_ISR_ONE_JUMP_LOW_SPEED 2u
#define STT_HALL_ISR_ONE_JUMP_HIGH_SPEED 3u
#define STT_HALL_ISR_SIX_JUMP 4u

#ifdef LONG_HURST
float tblHALLangle[8u] = {
    RL_DUMM,  /* 0: dummy */
    RL_TWO_OVER_THREE_PI,  /* 1: 120 degree */
    RL_FOUR_OVER_THREE_PI,  /* 2: 240 degree */
    RL_PI,  /* 3: 180 degree */
    0.0f,  /* 4: 0 degree */
    RL_ONE_OVER_THREE_PI,  /* 5: 60 degree */
    RL_FIVE_OVER_THREE_PI,  /* 6: 300 degree */
    RL_DUMM  /* 7: dummy */
};
#endif

tagHALLdata                         HALLdata;
mcParam_PIController     			mcApp_Q_PIParam;      // Parameters for Q axis Current PI Controller 
mcParam_PIController     			mcApp_D_PIParam;      // Parameters for D axis Current PI Controller 
mcParam_PIController     			mcApp_Speed_PIParam;  // Parameters for Speed PI Controller 
mcParam_FOC							mcApp_focParam;       // Parameters related to Field Oriented Control
mcParam_SinCos					    mcApp_SincosParam;    // Parameters related to Sine/Cosine calculator
mcParam_SVPWM 						mcApp_SVGenParam;     // Parameters related to Space Vector PWM
mcParam_ControlRef 					mcApp_ControlParam;   // Parameters related to Current and Speed references

mcParam_AlphaBeta                   mcApp_I_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current values
mcParam_AlphaBeta                   mcApp_IRef_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current Reference values
mcParam_DQ                          mcApp_I_DQParam;// D and Q axis (2 Phase Rotating Frame) current values
mcParam_DQ                          mcApp_IRef_DQParam; // D and Q axis (2 Phase Rotating Frame) Current Reference Values
mcParam_ABC                         mcApp_I_ABCParam; // A,B,C axis (3 Phase Stationary Frame) current values
mcParam_AlphaBeta                   mcApp_V_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis voltage values
mcParam_DQ                          mcApp_V_DQParam;// D and Q axis (2 Phase Rotating Frame) voltage values
motor_status_t                      mcApp_motorState;
delay_gen_t                         delay_10ms;

short        						potReading;
int16_t                             phaseCurrentA;
int16_t                             phaseCurrentB;
float								DoControl_Temp1, DoControl_Temp2;

float                               d_intermediate;
mcParam_DQ                          I_dq_old,I_dq_new;
float                               ol_angle,cl_angle;
uint16_t calibration_sample_count = 0x0000U;
uint16_t adc_0_offset = 0;
uint16_t adc_1_offset = 0;
uint32_t adc_0_sum = 0;
uint32_t adc_1_sum = 0;
uint32_t curpi_counter = 0;
char    state_count = 0;
uint32_t capture_val;
uint32_t capture_val_last;
uint32_t tcMCisrVal;


void mcApp_SpeedRamp()
{
        if(mcApp_motorState.motorDirection ==0)
        {
            mcApp_ControlParam.VelInput = (float)((float)potReading * POT_ADC_COUNT_FW_SPEED_RATIO);
        }
        else
        {
             mcApp_ControlParam.VelInput = (float)((float)-potReading * POT_ADC_COUNT_FW_SPEED_RATIO); 
        }
        mcApp_ControlParam.Diff =  mcApp_ControlParam.VelInput - mcApp_ControlParam.VelRef;
    
      
       //Speed Rate Limiter implementation.    
        if(mcApp_ControlParam.Diff >=CLOSEDLOOP_SPEED_HYSTERESIS)
        {
             mcApp_ControlParam.VelRef+=CLOSEDLOOP_SPEED_RAMP_RATE_DELTA;   
        }
        else if(mcApp_ControlParam.Diff <=-CLOSEDLOOP_SPEED_HYSTERESIS)
        {
             mcApp_ControlParam.VelRef-=CLOSEDLOOP_SPEED_RAMP_RATE_DELTA; 
        }
        else
        {
             mcApp_ControlParam.VelRef = mcApp_ControlParam.VelInput;
        }
        
         if(mcApp_motorState.motorDirection ==0)
        {
            if(mcApp_ControlParam.VelRef < OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC)
            {
                mcApp_ControlParam.VelRef = OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
            }
            else
            {
                
            }
        }
        else
        {
            if(mcApp_ControlParam.VelRef > -OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC)
            {
                mcApp_ControlParam.VelRef = -OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
            } 
            else
            {
                
            }
        }	
        
}

void PWM_Output_Disable()
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

void PWM_Output_Enable()
{
    TCC0_PWMPatternSet(0x00,0x00);/*Disable PWM override*/
}

inline void dirHALLget(tagHALLdata * ptData)
{
    uint16_t tmp1;
    
    tmp1 = ptData->combHALLvalue;
    switch(tmp1){
        case RL_30_HALL_JUMP_CW:
        case RL_90_HALL_JUMP_CW:            
        case RL_150_HALL_JUMP_CW:            
        case RL_210_HALL_JUMP_CW:            
        case RL_270_HALL_JUMP_CW:            
        case RL_330_HALL_JUMP_CW:
            ptData->dir = 1;  /* Rotor angle is increasing. */
            break;
        case RL_30_HALL_JUMP_CCW:
        case RL_90_HALL_JUMP_CCW:
        case RL_150_HALL_JUMP_CCW:
        case RL_210_HALL_JUMP_CCW:
        case RL_270_HALL_JUMP_CCW:
        case RL_330_HALL_JUMP_CCW:
            ptData->dir = -1;  /* Rotor angle is decreasing. */        
            break;
        default:
            break;
    }
}

inline void nextAngleHALLcal(tagHALLdata * ptData)
{
    float tmp;
    
    tmp = ptData->angleHALLjump + ((float)(ptData->dir) * RL_ONE_OVER_THREE_PI);
    
    if(RL_TWO_PI < tmp){
        ptData->nextAngleHALLjump = tmp - RL_TWO_PI;
    } else if(0.0f > tmp){
        ptData->nextAngleHALLjump = tmp + RL_TWO_PI;
    } else{
        ptData->nextAngleHALLjump = tmp;
    }
}

inline float angleHALLjumpGet(tagHALLdata * ptData)
{
    uint16_t tmp1;
    float tmp2 = 0.0f;
    
    tmp1 = ptData->combHALLvalue;
    switch(tmp1){
        case RL_30_HALL_JUMP_CW:
        case RL_30_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ONE_OVER_SIX_PI;
            break;
        case RL_90_HALL_JUMP_CW:
        case RL_90_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ONE_OVER_TWO_PI;
            break;
        case RL_150_HALL_JUMP_CW:
        case RL_150_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_FIVE_OVER_SIX_PI;
            break;
        case RL_210_HALL_JUMP_CW:
        case RL_210_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_SEVEN_OVER_SIX_PI;
            break;
        case RL_270_HALL_JUMP_CW:
        case RL_270_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_THREE_OVER_TWO_PI;
            break;
        case RL_330_HALL_JUMP_CW:
        case RL_330_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ELEVEN_OVER_SIX_PI;
            break;
        default:
            ptData->numEffectiveHALLjump = 0u;  /* not an effective HALL jump */
            break;
    }
    
    if(0x1000u < ptData->numEffectiveHALLjump){
        ptData->numEffectiveHALLjump = 0x1000u;
    }
    
    return tmp2;
}

inline void velocityCalOneJump(tagHALLdata * ptData)
{
    if(0u == ptData->tcHALLisr){
        return;
    }
    
    ptData->eleVelocity = ptData->dAngleHALLjump / (float)ptData->tcHALLisr * RL_TC0_FREQ;
}

inline void velocityCalSixJump(tagHALLdata * ptData)
{
    if(0u == ptData->tcSum){
        return;
    }
    
    ptData->eleVelocity = (float)(ptData->dir) * RL_2PI_TC0_FREQ / (float)ptData->tcSum;  
}

inline void velocityManipulate(tagHALLdata * ptData)
{
    /* Calculate absolute value. */
    if(0.0f > ptData->eleVelocity){
        ptData->absEleVel = -1.0f * ptData->eleVelocity;
    } else{
        ptData->absEleVel = ptData->eleVelocity;
    }   
}

void PORT_HALLcodeGet(tagHALLdata * ptData)
{
    ptData->HALLa = PORT_PinRead(PORT_PIN_PC16);
    ptData->HALLb = PORT_PinRead(PORT_PIN_PC17);  
    ptData->HALLc = PORT_PinRead(PORT_PIN_PC18);   
    ptData->HALLvalue = ((ptData->HALLc << 2) | (ptData->HALLb << 1) | ptData->HALLa) & 0x7u;
}

void resetHALLdata(tagHALLdata * ptData)
{
    uint16_t cnt;
    tagState * ptStt = &(ptData->sttData);    
 
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
    
    for(cnt = 0u; cnt<TC_HALL_FIFO_LEN; cnt++){
        ptData->tcFIFO[cnt] = 0ul;
    }
    ptData->cntTCfifo = 0u;
    ptData->tcSum = 0u;
    PORT_HALLcodeGet(&HALLdata);
    ptData->angleHALL = tblHALLangle[ptData->HALLvalue];
    ptData->estimatedRotorAngle = ptData->angleHALL;
    ptData->dir = 0u;    
    ptData->numEffectiveHALLjump = 0u;
    ptData->tcHALLisr = 0lu;
    ptData->eleVelocity = 0.0f;
    ptData->offsetAngleBound = 0.0f;
    ptData->offsetAngle = 0.0f;
    ptData->eleVelocity = 0.0f;
    ptData->eleVelocityFil = 0.0f;
    ptStt->state = STT_HALL_ISR_INI;
    ptStt->isSttChanged = 0u;
    ptStt->cnt32_1 = 0u;
   
    capture_val_last = 0u;
    capture_val = 0u;
    
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
}

inline void tcFIFOmaintain(tagHALLdata * ptData)
{
    uint16_t * cnt = &(ptData->cntTCfifo);
    
    ptData->tcSum += ptData->tcHALLisr;
    (*cnt)++;
    if(TC_HALL_FIFO_LEN <= *cnt){
        *cnt = 0u;
    }
    ptData->tcSum -= ptData->tcFIFO[*cnt];
    ptData->tcFIFO[*cnt] = ptData->tcHALLisr;
}

void HALL_jump_ISR (PDEC_HALL_STATUS status, uintptr_t context)
{
    float tmp1;
    tagState * ptStt = &(HALLdata.sttData);
    
    capture_val_last = capture_val;
    capture_val = TC2_Capture32bitChannel0Get();
    if(0xFFFFFFFFlu == capture_val){
        capture_val = TC2_Capture32bitChannel0Get();
    }
    HALLdata.tcHALLisr = capture_val - capture_val_last;

    tcFIFOmaintain(&HALLdata);
    HALLdata.lastHALLvalue = HALLdata.HALLvalue;
#if 1  /* Result from PDEC maybe the last HALL value, not the current one. */
    HALLdata.HALLvalue = PDEC_HALLPatternGet();
#else
    PORT_HALLcodeGet(&HALLdata);
#endif
    if (HALLdata.HALLvalue != 0 || HALLdata.HALLvalue != 7)
    {
    HALLdata.combHALLvalue = (HALLdata.lastHALLvalue << 8) + (HALLdata.HALLvalue);        
    tmp1 = angleHALLjumpGet(&HALLdata);
    
    if(0u != HALLdata.numEffectiveHALLjump){
        HALLdata.lastAngleHALLjump = HALLdata.angleHALLjump;
        HALLdata.angleHALLjump = tmp1;  
        tmp1 = HALLdata.angleHALLjump - HALLdata.lastAngleHALLjump;
        if(-RL_PI > tmp1){
            HALLdata.dAngleHALLjump = tmp1 + RL_TWO_PI;
        }else if(RL_PI < tmp1){
            HALLdata.dAngleHALLjump = tmp1 - RL_TWO_PI;
        }else{
            HALLdata.dAngleHALLjump = tmp1;
        }       
    }

    switch(ptStt->state){
        case STT_HALL_ISR_INI: 
            HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
            ptStt->state = STT_HALL_ISR_1ST_JUMP;
            break;
        case STT_HALL_ISR_1ST_JUMP:
            if(2u <= HALLdata.numEffectiveHALLjump){
                HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
                dirHALLget(&HALLdata);
                HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
                velocityCalOneJump(&HALLdata);
                velocityManipulate(&HALLdata);
                ptStt->state = STT_HALL_ISR_ONE_JUMP_LOW_SPEED;
            }            
            break;
        case STT_HALL_ISR_ONE_JUMP_LOW_SPEED:  /* Calculate velocity by neighbored HALL jumps, without angle estimation. */
            HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
            dirHALLget(&HALLdata);
            HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
            velocityCalOneJump(&HALLdata);
            velocityManipulate(&HALLdata);
            if((7u < HALLdata.numEffectiveHALLjump) && (CTC_ELE_VEL_1_HIGH_BOUND < HALLdata.absEleVel)){
                ptStt->state = STT_HALL_ISR_ONE_JUMP_HIGH_SPEED;
            }
            break;
        case STT_HALL_ISR_ONE_JUMP_HIGH_SPEED:  /* Calculate velocity by neighbored HALL jumps, with angle estimation. */
            dirHALLget(&HALLdata);
            HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
            velocityCalOneJump(&HALLdata);
            velocityManipulate(&HALLdata);
            if(CTC_ELE_VEL_2_HIGH_BOUND < HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_SIX_JUMP;
            } else if(CTC_ELE_VEL_1_LOW_BOUND > HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_ONE_JUMP_LOW_SPEED;
            }
            break;            
        case STT_HALL_ISR_SIX_JUMP:
            velocityCalSixJump(&HALLdata); 
            velocityManipulate(&HALLdata);
            if(CTC_ELE_VEL_2_LOW_BOUND > HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_ONE_JUMP_HIGH_SPEED;
            }
            break;
        default:
            break;
    }
    }   
    HALLdata.test32_1++;
    
}

/* This ISR calibrates zero crossing point for Phase U and Phase V currents*/
void ADC_CALIB_ISR (ADC_STATUS status, uintptr_t context)
{
    X2CScope_Update();
    calibration_sample_count++;
    if(calibration_sample_count <= 4096)
    {
        adc_0_sum += ADC0_ConversionResultGet();    
        adc_1_sum += ADC1_ConversionResultGet();
    }
    else
    {
        adc_0_offset = adc_0_sum>>12;
        adc_1_offset = adc_1_sum>>12;
        ADC0_Disable();
        ADC0_CallbackRegister((ADC_CALLBACK) mcApp_ADCISRTasks, (uintptr_t)NULL);
        ADC0_Enable();
    }
 
}

inline void latchHALLdata(tagHALLdata * ptData)
{
    ptData->angleHALLjumpLth = ptData->angleHALLjump;
    ptData->eleVelocityLth = ptData->eleVelocity;
    ptData->dirLth = ptData->dir;
    ptData->stateLth = ptData->sttData.state;
}

inline void lowSpeedCorrect(tagHALLdata * ptData)
{
    float vel, tcMCisr, tcHALLisr;
    
    if((STT_HALL_ISR_ONE_JUMP_LOW_SPEED != ptData->stateLth) || (0u == ptData->tcMCisr)){
        return;
    }
    
    vel = ptData->eleVelocityLth;
    tcMCisr = (float)(ptData->tcMCisr);
    tcHALLisr = (float)(ptData->tcHALLisr);
    
    if(tcMCisr > tcHALLisr){
        ptData->eleVelocityLth = vel * tcHALLisr / tcMCisr;
    }    
}

inline void angleCal(tagHALLdata * ptData)
{
    float tmp;
    
    tmp = ptData->eleVelocityLth * (float)ptData->tcMCisr * RL_TCO_TS;
    
    switch(ptData->dirLth){
        case 1:
            if(tmp < ptData->offsetAngleBound){
                ptData->offsetAngle = tmp;
            } else {
                ptData->offsetAngle = ptData->offsetAngleBound;
            }
            break;
        case -1:
            if(tmp > ptData->offsetAngleBound){
                ptData->offsetAngle = tmp;
            } else {
                ptData->offsetAngle = ptData->offsetAngleBound;
            }            
            break;
        case 0:
            ptData->offsetAngle = 0.0f;
            break;
        default:           
            break;
    }
       
    tmp = ptData->angleHALLjumpLth + ptData->offsetAngle;
    
    if(RL_TWO_PI < tmp){
        ptData->estimatedRotorAngle = tmp - RL_TWO_PI;
    } else if(0.0f > tmp){
        ptData->estimatedRotorAngle = tmp + RL_TWO_PI;
    } else{
        ptData->estimatedRotorAngle = tmp;
    } 
}

inline void rotorAngleEstimate(tagHALLdata * ptData)
{
    switch(ptData->stateLth){
        case STT_HALL_ISR_INI: 
        case STT_HALL_ISR_1ST_JUMP:
        case STT_HALL_ISR_ONE_JUMP_LOW_SPEED:
            /* angleHALL is not latched, it is read once and only once here. */
            ptData->estimatedRotorAngle = ptData->angleHALL;
            break;
        case STT_HALL_ISR_ONE_JUMP_HIGH_SPEED: 
        case STT_HALL_ISR_SIX_JUMP:
            angleCal(ptData);
            break;
        default:
            break;
    }
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


// *****************************************************************************
// *****************************************************************************
// Section: MC ADC ISR TASKS
// *****************************************************************************
// *****************************************************************************
void mcApp_ADCISRTasks(ADC_STATUS status, uintptr_t context)
{
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
    /* TC2 works in capture mode. 
     * Note that with TC2 in capture mode, TC2's counter is updated by hardware.
     * So, the HALLdata and TC2-counter being read here is not guaranteed to be
     * perfectly synchronized. For example, a HALL-jump occurs right after 
     * "NVIC_DisableIRQ(PDEC_OTHER_IRQn)", then TC2-counter is read as almost 
     * zero, meanwhile HALLdata is not updated to the latest value yet.
     */
    tcMCisrVal = TC2_Timer32bitCounterGet();
    HALLdata.tcMCisr = tcMCisrVal - capture_val;

    latchHALLdata(&HALLdata);    
#if 0
    lowSpeedCorrect(&HALLdata);
#endif
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
    rotorAngleEstimate(&HALLdata);
    /* LPF */
    HALLdata.eleVelocityFil = (RL_LPF_COEFF_1 * HALLdata.eleVelocityFil) + (RL_LPF_COEFF_2 * HALLdata.eleVelocityLth);    
    
	X2CScope_Update();

    phaseCurrentA = (int16_t)ADC0_ConversionResultGet() - (int16_t)adc_0_offset;// Phase Current A measured using ADC0
    phaseCurrentB = (int16_t)ADC1_ConversionResultGet() - (int16_t)adc_1_offset;// Phase Current B measured using ADC4
    
    /* Clear all interrupt flags */
       ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;

       ADC0_REGS->ADC_INTENCLR = ADC_INTFLAG_RESRDY_Msk;// Disable ADC interrupt
    /* select the next channel */
	/* select the next ADC channel for conversion */
    ADC0_ChannelSelect(ADC_POSINPUT_AIN6,ADC_NEGINPUT_GND); // Potentiometer to ADC0
    ADC1_ChannelSelect(ADC_POSINPUT_AIN14,ADC_NEGINPUT_GND); // DC Bus Voltage to ADC1
    ADC0_REGS->ADC_SWTRIG |= ADC_SWTRIG_START_Msk; 
    
   
    
    if(mcApp_motorState.focStart)
    {
        mcApp_I_ABCParam.a = (float)phaseCurrentA*ADC_CURRENT_SCALE * (-1); 
        mcApp_I_ABCParam.b = (float)phaseCurrentB*ADC_CURRENT_SCALE * (-1);

        mcLib_ClarkeTransform(&mcApp_I_ABCParam, &mcApp_I_AlphaBetaParam);



        mcLib_ParkTransform(&mcApp_I_AlphaBetaParam, &mcApp_SincosParam, 
                            &mcApp_I_DQParam);

        switch (mcApp_motorState.focStateMachine)
        {
            case CLOSEDLOOP_FOC:
            {
                 mcApp_ControlParam.AssertActiveVector = 1;
                /* Use the rotor angle */          
                mcApp_SincosParam.Angle = HALLdata.estimatedRotorAngle;


                //if TORQUE MODE skip the speed and Field Weakening controller               
        #ifndef	TORQUE_MODE
                // Execute the velocity control loop

                mcApp_Speed_PIParam.qInMeas = HALLdata.eleVelocityFil;

                mcApp_Speed_PIParam.qInRef  = mcApp_ControlParam.VelRef;
                mcLib_CalcPI(&mcApp_Speed_PIParam);
                mcApp_IRef_DQParam.q= mcApp_Speed_PIParam.qOut;

        #ifdef ENABLE_FLUX_WEAKENING	
                // Implement Field Weakening if Speed input is greater than the base speed of the motor
                if(mcApp_motorState.motorDirection == 0)
                {
                    if(mcApp_ControlParam.VelRef > NOMINAL_SPEED_RAD_PER_SEC_ELEC)
                    {
                        mcApp_focParam.Vds = mcApp_V_DQParam.d*mcApp_V_DQParam.d;

                        if(mcApp_focParam.Vds>MAX_NORM_SQ)
                        {
                            mcApp_focParam.Vds = MAX_NORM_SQ;
                        }

                        mcApp_focParam.Vqs = sqrtf(MAX_NORM_SQ-mcApp_focParam.Vds);
                        mcApp_focParam.VqRefVoltage = mcApp_focParam.MaxPhaseVoltage*mcApp_focParam.Vqs;

                         //Calculating Flux Weakening value of Id, Id_flux_Weakening = (Vqref- Rs*Iq - BEMF)/omega*Ls

                        mcApp_ControlParam.Id_FW_Raw = (mcApp_focParam.VqRefVoltage - (MOTOR_PER_PHASE_RESISTANCE * mcApp_IRef_DQParam.q) 
                                                -(mcApp_ControlParam.VelRef  * MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC))/(mcApp_ControlParam.VelRef  * MOTOR_PER_PHASE_INDUCTANCE);

                        mcApp_ControlParam.Id_FW_Filtered = mcApp_ControlParam.Id_FW_Filtered +
                                          ((mcApp_ControlParam.Id_FW_Raw - mcApp_ControlParam.Id_FW_Filtered) * mcApp_ControlParam.qKfilterIdRef) ;	

                        mcApp_IRef_DQParam.d= mcApp_ControlParam.Id_FW_Filtered;
                          //Limit Id such that MAX_FW_NEGATIVE_ID_REF < Id < 0
                        if(mcApp_IRef_DQParam.d> 0)
                            mcApp_IRef_DQParam.d= 0; 

                        if(mcApp_IRef_DQParam.d< MAX_FW_NEGATIVE_ID_REF)
                            mcApp_IRef_DQParam.d= MAX_FW_NEGATIVE_ID_REF;

                        // Limit Q axis current such that sqrtf(Id^2 +Iq^2) <= MAX_MOTOR_CURRENT
                        mcApp_ControlParam.Iqmax = sqrtf((MAX_MOTOR_CURRENT_SQUARED) - (mcApp_IRef_DQParam.d*mcApp_IRef_DQParam.d));	
                    }
                    else
                    {
                        mcApp_IRef_DQParam.d= 0;
                        mcApp_ControlParam.Iqmax = MAX_MOTOR_CURRENT;
                        mcApp_ControlParam.Id_FW_Filtered = 0;
                        mcApp_ControlParam.Id_FW_Raw = 0;
                    }
                }
                else
                {
                    if(mcApp_ControlParam.VelRef < -NOMINAL_SPEED_RAD_PER_SEC_ELEC )
                    {
                        mcApp_focParam.Vds = mcApp_V_DQParam.d*mcApp_V_DQParam.d;

                        if(mcApp_focParam.Vds>MAX_NORM_SQ)
                        {
                        mcApp_focParam.Vds = MAX_NORM_SQ;
                        }

                        mcApp_focParam.Vqs = sqrtf(MAX_NORM_SQ-mcApp_focParam.Vds);
                        mcApp_focParam.VqRefVoltage = -mcApp_focParam.MaxPhaseVoltage*mcApp_focParam.Vqs;

                         //Calculating Flux Weakening value of Id, Id_flux_Weakening = (Vqref- Rs*Iq - BEMF)/omega*Ls

                        mcApp_ControlParam.Id_FW_Raw = (mcApp_focParam.VqRefVoltage - (MOTOR_PER_PHASE_RESISTANCE * mcApp_IRef_DQParam.q) 
                                                -(mcApp_ControlParam.VelRef  * MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC))/(mcApp_ControlParam.VelRef  * MOTOR_PER_PHASE_INDUCTANCE);

                        mcApp_ControlParam.Id_FW_Filtered = mcApp_ControlParam.Id_FW_Filtered +
                                          ((mcApp_ControlParam.Id_FW_Raw - mcApp_ControlParam.Id_FW_Filtered) * mcApp_ControlParam.qKfilterIdRef) ;										
                        mcApp_IRef_DQParam.d= mcApp_ControlParam.Id_FW_Filtered;						
                          //Limit Id such that MAX_FW_NEGATIVE_ID_REF < Id < 0
                        if(mcApp_IRef_DQParam.d> 0)
                            mcApp_IRef_DQParam.d= 0; 

                        if(mcApp_IRef_DQParam.d< MAX_FW_NEGATIVE_ID_REF)
                            mcApp_IRef_DQParam.d= MAX_FW_NEGATIVE_ID_REF;

                        // Limit Q axis current such that sqrtf(Id^2 +Iq^2) <= MAX_MOTOR_CURRENT
                        mcApp_ControlParam.Iqmax = -sqrtf((MAX_MOTOR_CURRENT_SQUARED) - (mcApp_IRef_DQParam.d*mcApp_IRef_DQParam.d));	
                    }
                    else
                    {
                        mcApp_IRef_DQParam.d= 0;
                        mcApp_ControlParam.Iqmax = -MAX_MOTOR_CURRENT;
                        mcApp_ControlParam.Id_FW_Filtered = 0;
                    }
                }


        #endif 	

        #else
        if(mcApp_motorState.motorDirection == 0)
        {
                mcApp_IRef_DQParam.q= (float)((float)potReading * TORQUE_MODE_POT_ADC_RATIO); // During torque mode, Iq = Potentiometer provides torque reference in terms of current, Id = 0
                mcApp_IRef_DQParam.d= 0;
                mcApp_ControlParam.Iqmax = TORQUE_MODE_MAX_CUR;
        }
        else
        {
                mcApp_IRef_DQParam.q= (float)((float)-potReading * TORQUE_MODE_POT_ADC_RATIO); // During torque mode, Iq = Potentiometer provides torque reference in terms of current, Id = 0
                mcApp_IRef_DQParam.d= 0;
                mcApp_ControlParam.Iqmax = -TORQUE_MODE_MAX_CUR;
        }
        #endif  // endif for TORQUE_MODE


            break;
            }
         }


            // PI control for D
            mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;          // This is in Amps
            mcApp_D_PIParam.qInRef  = mcApp_IRef_DQParam.d;      // This is in Amps
            mcLib_CalcPI(&mcApp_D_PIParam);
            mcApp_V_DQParam.d    =  mcApp_D_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)

            // dynamic d-q adjustment
            // with d component priority
            // vq=sqrt (vs^2 - vd^2)
            // limit vq maximum to the one resulting from the calculation above
            DoControl_Temp2 = mcApp_D_PIParam.qOut * mcApp_D_PIParam.qOut;
            DoControl_Temp1 = MAX_NORM_SQ - DoControl_Temp2;
            mcApp_Q_PIParam.qOutMax = sqrtf(DoControl_Temp1);
            mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;        

            if(mcApp_motorState.motorDirection == 0)
            {
                //Limit Q axis current
                if(mcApp_IRef_DQParam.q>mcApp_ControlParam.Iqmax)
                {
                    mcApp_IRef_DQParam.q= mcApp_ControlParam.Iqmax;
                }
                else
                {

                }
            }
            else
            {
                //Limit Q axis current
                if(mcApp_IRef_DQParam.q<mcApp_ControlParam.Iqmax)
                {
                    mcApp_IRef_DQParam.q= mcApp_ControlParam.Iqmax;
                }   
                else
                {

                }
                }
            // PI control for Q
            mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;          // This is in Amps
            mcApp_Q_PIParam.qInRef  = mcApp_IRef_DQParam.q;      // This is in Amps
            mcLib_CalcPI(&mcApp_Q_PIParam);
            mcApp_V_DQParam.q    =  mcApp_Q_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)   

            mcLib_SinCosGen(&mcApp_SincosParam);

            mcLib_InvParkTransform(&mcApp_V_DQParam,&mcApp_SincosParam, &mcApp_V_AlphaBetaParam);

            mcLib_SVPWMGen(&mcApp_V_AlphaBetaParam , &mcApp_SVGenParam);
            if(mcApp_ControlParam.AssertActiveVector == 1)
            {
                TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) mcApp_SVGenParam.dPWM_A );
                TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) mcApp_SVGenParam.dPWM_B );
                TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) mcApp_SVGenParam.dPWM_C );
            }
            else
            {
                TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );
                TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );
                TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );
            }

        }
        else
        {
            TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );
            TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );
            TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );

        }
        while(ADC0_REGS->ADC_INTFLAG != ADC_INTFLAG_RESRDY_Msk);
                       
        /* Read the ADC result value */
        mcApp_focParam.DCBusVoltage = ((float)ADC1_ConversionResultGet())* VOLTAGE_ADC_TO_PHY_RATIO; // Reads and translates to actual bus voltage
		potReading = ADC0_ConversionResultGet();
  
        /* select the next ADC channel for conversion */
        ADC0_ChannelSelect(ADC_POSINPUT_AIN0,ADC_NEGINPUT_GND); // Phase U to ADC0
        ADC1_ChannelSelect(ADC_POSINPUT_AIN0,ADC_NEGINPUT_GND); // Phase V to ADC1
        ADC0_REGS->ADC_INTENSET = ADC_INTFLAG_RESRDY_Msk;// Enable ADC interrupt
        /* Clear all interrupt flags */
        ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;
      
        mcApp_focParam.MaxPhaseVoltage = (float)(mcApp_focParam.DCBusVoltage*ONE_BY_SQRT3);     
        delay_10ms.count++; 
}


// *****************************************************************************
// *****************************************************************************
// Section: MC PI Controller Routines
// *****************************************************************************
// *****************************************************************************

void mcApp_InitControlParameters(void)
{
	// PI D Term     
    mcApp_D_PIParam.qKp = D_CURRCNTR_PTERM;       
    mcApp_D_PIParam.qKi = D_CURRCNTR_ITERM;              
    mcApp_D_PIParam.qKc = D_CURRCNTR_CTERM;
    mcApp_D_PIParam.qdSum = 0;
    mcApp_D_PIParam.qOutMax = D_CURRCNTR_OUTMAX;
    mcApp_D_PIParam.qOutMin = -mcApp_D_PIParam.qOutMax;

    mcLib_InitPI(&mcApp_D_PIParam);

    // PI Q Term 
    mcApp_Q_PIParam.qKp = Q_CURRCNTR_PTERM;    
    mcApp_Q_PIParam.qKi = Q_CURRCNTR_ITERM;
    mcApp_Q_PIParam.qKc = Q_CURRCNTR_CTERM;
    mcApp_Q_PIParam.qdSum = 0;
    mcApp_Q_PIParam.qOutMax = Q_CURRCNTR_OUTMAX;
    mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;
    if(mcApp_motorState.motorDirection == 0)
    {
        mcApp_ControlParam.Iqmax = MAX_MOTOR_CURRENT;
    }
    else
    {
       mcApp_ControlParam.Iqmax = -MAX_MOTOR_CURRENT; 
    }
    mcLib_InitPI(&mcApp_Q_PIParam);

    // PI Qref Term
    mcApp_Speed_PIParam.qKp = SPEEDCNTR_PTERM;       
    mcApp_Speed_PIParam.qKi = SPEEDCNTR_ITERM;       
    mcApp_Speed_PIParam.qKc = SPEEDCNTR_CTERM;  
    mcApp_Speed_PIParam.qdSum = 0;
    mcApp_Speed_PIParam.qOutMax = SPEEDCNTR_OUTMAX;   
    mcApp_Speed_PIParam.qOutMin = -mcApp_Speed_PIParam.qOutMax;

    mcLib_InitPI(&mcApp_Speed_PIParam);
	
    mcApp_ControlParam.qKfilterIdRef = KFILTER_IDREF;
	
	
	return;
}

void mcApp_motorStart()
{
    mcApp_InitControlParameters();

    resetHALLdata(&HALLdata);  /* Initialize HALL data. */
    mcApp_IRef_DQParam.d= 0;
    mcApp_IRef_DQParam.q= 0;
    mcApp_ControlParam.Id_FW_Filtered = 0;
    mcApp_ControlParam.VelInput = 0;
    mcApp_ControlParam.VelRef = 0;
    mcApp_motorState.focStateMachine = CLOSEDLOOP_FOC;

    mcApp_SincosParam.Angle = 0;
	mcApp_SVGenParam.PWMPeriod = (float)MAX_DUTY;
    mcApp_motorState.focStart = 1;
    TC2_CaptureStart();
    TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );
    TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );
    TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );
    PWM_Output_Enable();   
}

void mcApp_motorStop()
{
    mcApp_motorState.focStart = 0;
    mcApp_IRef_DQParam.d= 0;
    mcApp_IRef_DQParam.q= 0;
    mcApp_I_DQParam.d = 0;
    mcApp_I_DQParam.q = 0;
    state_count = 0;
    PWM_Output_Disable(); 
    TC2_CaptureStop();
}

void mcApp_motorStartToggle()
{
    mcApp_motorState.motorStart = !mcApp_motorState.motorStart;
    if(mcApp_motorState.motorStart == 1)
    {
        mcApp_motorStart();
    }
    else
    {
        mcApp_motorStop();
    }
}

void mcApp_motorDirectionToggle()
{
    if(mcApp_motorState.motorStart == 0)
    {
        /*Change Motor Direction Only when Motor is Stationary*/
        mcApp_motorState.motorDirection = !mcApp_motorState.motorDirection;
        LED2_Direction_Toggle();
    }
    else
    {
        mcApp_motorState.motorDirection = mcApp_motorState.motorDirection;
    }
}



void OC_FAULT_ISR(uintptr_t context)
{
    mcApp_motorStop();
    mcApp_motorState.motorStart = 0;
    mcApp_motorState.focStart = 0;
    LED1_OC_FAULT_Set();
    while(1)
    {
        X2CScope_Communicate();
        X2CScope_Update();
    }
    
}