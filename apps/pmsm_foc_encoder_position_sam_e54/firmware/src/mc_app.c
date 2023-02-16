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


static mcParam_PIController     			mcApp_Q_PIParam;      // Parameters for Q axis Current PI Controller 
static mcParam_PIController     			mcApp_D_PIParam;      // Parameters for D axis Current PI Controller 
static mcParam_PIController     			mcApp_Speed_PIParam;  // Parameters for Speed PI Controller 
static mcParam_PIController                mcApp_Position_PIParam;//Parameters for Position PI Controller
static mcParam_FOC							mcApp_focParam;       // Parameters related to Field Oriented Control
mcParam_SinCos					    mcApp_SincosParam;    // Parameters related to Sine/Cosine calculator
mcParam_SVPWM 						mcApp_SVGenParam;     // Parameters related to Space Vector PWM
mcParam_ControlRef 					mcApp_ControlParam;   // Parameters related to Current and Speed references
static mcParam_AlphaBeta                   mcApp_I_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current values
static mcParam_DQ                          mcApp_I_DQParam;// D and Q axis (2 Phase Rotating Frame) current values
static mcParam_ABC                         mcApp_I_ABCParam; // A,B,C axis (3 Phase Stationary Frame) current values
static mcParam_AlphaBeta                   mcApp_V_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis voltage values
static mcParam_DQ                          mcApp_V_DQParam;// D and Q axis (2 Phase Rotating Frame) voltage values
static MCAPP_POSITION_CALC                 gPositionCalc;
motor_status_t                      mcApp_motorState;
delay_gen_t                         delay_10ms;
float 								OpenLoop_Ramp_Angle_Rads_Per_Sec = 0.0f; 	// ramp angle variable for initial ramp 
unsigned int 						Align_Counter = 0U; 				// lock variable for initial ramp 
static short        						potReading;
static int16_t                             phaseCurrentA;
static int16_t                             phaseCurrentB;
static float								DoControl_Temp1, DoControl_Temp2;
static float                               speed_ref_filtered = 0.0f;
static uint16_t                            slow_loop_count = 0U;
static int16_t                             pos_count_diff = 0;
static float                               speed_elec_rad_per_sec = 0.0f;
static float                               potPosition = 0.0f;
static float                               position_filtered=0.0f;
static uint8_t                             fixed_pot=1U;



static uintptr_t dummyforMisra;

static uint16_t calibration_sample_count = 0x0000U;
static uint16_t adc_0_offset = 0;
static uint16_t adc_1_offset = 0;
static uint32_t adc_0_sum = 0;
static uint32_t adc_1_sum = 0;

#ifdef MCHV3 
static int16_t  pot_adc;
#endif


void PWM_Output_Disable(void)
{
    uint8_t status=1U;
    bool pwm_flag;
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t)PWM_PERIOD_COUNT>>1));
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t)PWM_PERIOD_COUNT>>1));
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t)PWM_PERIOD_COUNT>>1));
    
    /*Override all PWM outputs to low*/
    status |= (uint8_t)(pwm_flag=TCC0_PWMPatternSet((TCC_PATT_PGE0_Msk|TCC_PATT_PGE1_Msk|TCC_PATT_PGE2_Msk
            |TCC_PATT_PGE4_Msk|TCC_PATT_PGE5_Msk|TCC_PATT_PGE6_Msk),
            (TCC_PATT_PGE0(0)|TCC_PATT_PGE1(0)|TCC_PATT_PGE2(0)|TCC_PATT_PGE4(0)
            |TCC_PATT_PGE5(0)|TCC_PATT_PGE6(0))));
	if (status!=1U){
        /* Error log*/
    }    
}

void PWM_Output_Enable(void)
{    
    uint8_t status=1U;
    bool pwm_flag;
    status |= (uint8_t)(pwm_flag=TCC0_PWMPatternSet(0x00,0x00));/*Disable PWM override*/
    if (status!=1U){
        /* Error log*/
    } 
}

/* This ISR calibrates zero crossing point for Phase U and Phase V currents*/
void ADC_CALIB_ISR (ADC_STATUS status, uintptr_t context)
{
    X2CScope_Update();
    calibration_sample_count++;
    if(calibration_sample_count <= 4096U)
    {
        adc_0_sum += ADC0_ConversionResultGet();    
        adc_1_sum += ADC1_ConversionResultGet();
    }
    else
    {
        adc_0_offset = (uint16_t)(adc_0_sum>>12U);
        adc_1_offset = (uint16_t)(adc_1_sum>>12U);
        ADC0_Disable();
        ADC0_CallbackRegister((ADC_CALLBACK) mcApp_ADCISRTasks, (uintptr_t)dummyforMisra);
        ADC0_Enable();
    }
 
}
// *****************************************************************************
// *****************************************************************************
// Section: MC ADC ISR TASKS
// *****************************************************************************
// *****************************************************************************
void mcApp_ADCISRTasks(ADC_STATUS status, uintptr_t context)
{
    uint8_t pwm_status=1U;
    bool pwm_flag; 
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
    
   
    
    if(mcApp_motorState.focStart==1U)
    {
        
     mcApp_I_ABCParam.a = (float)phaseCurrentA*ADC_CURRENT_SCALE * (-1.0f); 
     mcApp_I_ABCParam.b = (float)phaseCurrentB*ADC_CURRENT_SCALE * (-1.0f);
    
     mcLib_ClarkeTransform(&mcApp_I_ABCParam, &mcApp_I_AlphaBetaParam);    
    
     mcLib_ParkTransform(&mcApp_I_AlphaBetaParam, &mcApp_SincosParam, 
                        &mcApp_I_DQParam);

     switch (mcApp_motorState.focStateMachine)
     {
        
        case ALIGN:
        {
       
         if (Align_Counter < (unsigned int)COUNT_FOR_ALIGN_TIME)
            {
                     
            Align_Counter++;
            mcApp_motorState.focStateMachine = ALIGN;
            mcApp_SincosParam.Angle = (float)(M_PI);
            }
         else if (Align_Counter < 2U*(unsigned int)COUNT_FOR_ALIGN_TIME)
            {
            Align_Counter++;
            mcApp_motorState.focStateMachine = ALIGN;
            mcApp_SincosParam.Angle = (float)(3.0f*(float)M_PI_2); 
            }
         else 
           {
              /*start PDEC timer*/
            PDEC_QDECStart();
            
            gPositionCalc.QDECcntZ = 0u;
            gPositionCalc.prev_position_count=0;
            gPositionCalc.posCompensation = 0u;
            speed_ref_filtered=0.0f;
            mcApp_SincosParam.Angle = 0.0f;
            mcApp_motorState.focStateMachine = CLOSEDLOOP_FOC;
            potPosition = 0.0f;
            position_filtered =0.0f;
            mcApp_Position_PIParam.qInMeas = 0.0f;
            speed_elec_rad_per_sec = 0.0f;
            
            }
            /* Align stator reference frame to alpha-beta axis*/
         
            if(mcApp_ControlParam.IqRef > ALIGN_Q_CURRENT_REF)
            {
                mcApp_ControlParam.IqRef = (float)ALIGN_Q_CURRENT_REF;
            }
            else
            {
                mcApp_ControlParam.IqRef += (float)ALIGN_CURRENT_STEP;
            }

        mcApp_ControlParam.IdRef = 0.0f;
        break;
        }
          
        case CLOSEDLOOP_FOC:
        {
       
            gPositionCalc.QDECcnt = PDEC_QDECPositionGet();
            if((gPositionCalc.QDECcnt>QDEC_UPPER_THRESHOLD) && (gPositionCalc.QDECcntZ<QDEC_LOWER_THRESHOLD))
            {
                gPositionCalc.posCompensation += QDEC_UNDERFLOW;
            } 
            else if((gPositionCalc.QDECcntZ>QDEC_UPPER_THRESHOLD) && (gPositionCalc.QDECcnt<QDEC_LOWER_THRESHOLD))
            {
            gPositionCalc.posCompensation += QDEC_OVERFLOW;           
            } 
            else{
                /* Dummy branch for MISRAC compliance*/
            } 
            
            gPositionCalc.posCompensation = gPositionCalc.posCompensation % ENCODER_PULSES_PER_EREV;
            gPositionCalc.posCntTmp = (uint32_t)((uint32_t)gPositionCalc.QDECcnt + (uint32_t)gPositionCalc.posCompensation);  
            gPositionCalc.posCnt = (uint16_t)(gPositionCalc.posCntTmp % ENCODER_PULSES_PER_EREV);
            mcApp_SincosParam.Angle = ((float)gPositionCalc.posCnt) * (2.0f * (float)M_PI / (float)ENCODER_PULSES_PER_EREV);
            gPositionCalc.QDECcntZ = gPositionCalc.QDECcnt;
            
            if(mcApp_SincosParam.Angle > (2.0f*(float)M_PI))
            {
              mcApp_SincosParam.Angle = mcApp_SincosParam.Angle - (2.0f*(float)M_PI);
            }
            else if (mcApp_SincosParam.Angle < 0.0f)
            {
              mcApp_SincosParam.Angle = mcApp_SincosParam.Angle + (2.0f*(float)M_PI); 
            }
            else
            {
               //mcApp_SincosParam.Angle = mcApp_SincosParam.Angle;
            }
                         
        
        mcApp_Position_PIParam.qInMeas = (float)((int16_t)PDEC_QDECPositionGet());    
        mcLib_CalcPI(&mcApp_Position_PIParam);
        mcApp_Speed_PIParam.qInRef = mcApp_Position_PIParam.qOut;        
        
        if (slow_loop_count >= SLOW_LOOP_FACTOR)
        {
        slow_loop_count = 0;
            /* Speed Calculation from Encoder */
        gPositionCalc.present_position_count = (int16_t)(PDEC_QDECPositionGet());
        pos_count_diff = gPositionCalc.present_position_count - gPositionCalc.prev_position_count;
        speed_elec_rad_per_sec = ((float)pos_count_diff * 2.0f*(float)M_PI)/((float)ENCODER_PULSES_PER_EREV *SLOW_LOOP_TIME_SEC );
        gPositionCalc.prev_position_count = gPositionCalc.present_position_count;
        }
            // Execute the velocity control loop

        mcApp_Speed_PIParam.qInMeas = speed_elec_rad_per_sec;
     	mcLib_CalcPI(&mcApp_Speed_PIParam);
    	mcApp_ControlParam.IqRef = mcApp_Speed_PIParam.qOut;
        mcApp_ControlParam.IdRef = 0.0f;
        
        break;
        }
         default:
             /* Undefined state: Should never come here */
             break;
     }
    

        // PI control for D
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;          // This is in Amps
        mcApp_D_PIParam.qInRef  = mcApp_ControlParam.IdRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d    =  mcApp_D_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)

        // dynamic d-q adjustment
        // with d component priority
        // vq=sqrt (vs^2 - vd^2)
        // limit vq maximum to the one resulting from the calculation above
        DoControl_Temp2 = mcApp_D_PIParam.qOut * mcApp_D_PIParam.qOut;
        DoControl_Temp1 = MAX_NORM_SQ - DoControl_Temp2;
        if(DoControl_Temp1>=0.0f){
        mcApp_Q_PIParam.qOutMax = sqrtf(DoControl_Temp1);}
        else{
          mcApp_Q_PIParam.qOutMax = sqrtf(-DoControl_Temp1);
        }
        
        mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax; 
        
        if(mcApp_ControlParam.IqRef>mcApp_ControlParam.IqRefmax)
        {
            //Limit Q axis current
            mcApp_ControlParam.IqRef = mcApp_ControlParam.IqRefmax;
        }
	

        // PI control for Q
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;          // This is in Amps
        mcApp_Q_PIParam.qInRef  = mcApp_ControlParam.IqRef;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q    =  mcApp_Q_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)   

        mcLib_SinCosGen(&mcApp_SincosParam);

        mcLib_InvParkTransform(&mcApp_V_DQParam,&mcApp_SincosParam, &mcApp_V_AlphaBetaParam);
         
        mcLib_SVPWMGen(&mcApp_V_AlphaBetaParam , &mcApp_SVGenParam);
        
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) mcApp_SVGenParam.dPWM_A ));
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) mcApp_SVGenParam.dPWM_B ));
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) mcApp_SVGenParam.dPWM_C ));

        
    }
    else
    {
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT ));
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT ));
        pwm_status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT ));
    
    }
        while(ADC0_REGS->ADC_INTFLAG != ADC_INTFLAG_RESRDY_Msk)
        {
            /* wait till ADC interrupt*/
        }
                       
        /* Read the ADC result value */
        mcApp_focParam.DCBusVoltage = (float)((float)((uint32_t)ADC1_ConversionResultGet())* VOLTAGE_ADC_TO_PHY_RATIO); // Reads and translates to actual bus voltage
		potReading =(int16_t) ADC0_ConversionResultGet();
        potReading-=2047;
        
        #ifdef MCLV2
         mcApp_Position_PIParam.qInRef  = (float) ((float)potReading*(3.0f*(float)ENCODER_PULSES_PER_REV/4096.0f));
        #endif
        
        #ifdef MCHV3         

        if(fixed_pot == 1U)
        {
          mcApp_Position_PIParam.qInRef  =  (float)((float)potReading*(6.0f*(float)ENCODER_PULSES_PER_REV/4096.0f));  
  
          if (mcApp_Position_PIParam.qInRef > 0.0f && mcApp_Position_PIParam.qInRef < 2000.0f)
          {
             mcApp_Position_PIParam.qInRef = 2000.0f; 
          }
          else if (mcApp_Position_PIParam.qInRef < 0.0f && mcApp_Position_PIParam.qInRef > -2000.0f)
          {
             mcApp_Position_PIParam.qInRef = -2000.0f; 
          }
          else
          {
          //   mcApp_Position_PIParam.qInRef = mcApp_Position_PIParam.qInRef; 
          }
          pot_adc = potReading;
          fixed_pot = 0;          
        }
        #endif
        /* select the next ADC channel for conversion */
        ADC0_ChannelSelect(ADC_POSINPUT_AIN0,ADC_NEGINPUT_GND); // Phase U to ADC0
        ADC1_ChannelSelect(ADC_POSINPUT_AIN0,ADC_NEGINPUT_GND); // Phase V to ADC1
        ADC0_REGS->ADC_INTENSET = ADC_INTFLAG_RESRDY_Msk;// Enable ADC interrupt
        /* Clear all interrupt flags */
        ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;
      
        mcApp_focParam.MaxPhaseVoltage = (float)(mcApp_focParam.DCBusVoltage*ONE_BY_SQRT3);     
        delay_10ms.count++; 
        slow_loop_count++;
        
    if (pwm_status!=1U){
        /* Error log*/
    } 
        
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
    mcApp_D_PIParam.qdSum = 0.0f;
    mcApp_D_PIParam.qOutMax = D_CURRCNTR_OUTMAX;
    mcApp_D_PIParam.qOutMin = -mcApp_D_PIParam.qOutMax;

    mcLib_InitPI(&mcApp_D_PIParam);

    // PI Q Term 
    mcApp_Q_PIParam.qKp = Q_CURRCNTR_PTERM;    
    mcApp_Q_PIParam.qKi = Q_CURRCNTR_ITERM;
    mcApp_Q_PIParam.qKc = Q_CURRCNTR_CTERM;
    mcApp_Q_PIParam.qdSum = 0.0f;
    mcApp_Q_PIParam.qOutMax = Q_CURRCNTR_OUTMAX;
    mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;
 
    mcApp_ControlParam.IqRefmax = MAX_MOTOR_CURRENT;
 
    mcLib_InitPI(&mcApp_Q_PIParam);

    // PI Qref Term
    mcApp_Speed_PIParam.qKp = SPEEDCNTR_PTERM;       
    mcApp_Speed_PIParam.qKi = SPEEDCNTR_ITERM;       
    mcApp_Speed_PIParam.qKc = SPEEDCNTR_CTERM;  
    mcApp_Speed_PIParam.qdSum = 0.0f;
    mcApp_Speed_PIParam.qOutMax = SPEEDCNTR_OUTMAX;   
    mcApp_Speed_PIParam.qOutMin = -mcApp_Speed_PIParam.qOutMax;

    mcLib_InitPI(&mcApp_Speed_PIParam);
    // PI Position Term
    mcApp_Position_PIParam.qKp = POSCNTR_PTERM;       
    mcApp_Position_PIParam.qKi = POSCNTR_ITERM;       
    mcApp_Position_PIParam.qKc = POSCNTR_CTERM;  
    mcApp_Position_PIParam.qdSum = 0.0f;
    mcApp_Position_PIParam.qOutMax = POSCNTR_OUTMAX;   
    mcApp_Position_PIParam.qOutMin = -mcApp_Position_PIParam.qOutMax;

    mcLib_InitPI(&mcApp_Position_PIParam);   
	
	return;
}



void mcApp_motorStart(void)
{
    uint8_t status=1u;
    bool pwm_flag;
    PDEC_QDECInitialize();
    mcApp_InitControlParameters();
    mcApp_ControlParam.IdRef = 0.0f;
    mcApp_ControlParam.IqRef = 0.0f;
    mcApp_ControlParam.VelInput = 0.0f;
    mcApp_ControlParam.VelRef = 0.0f;
    mcApp_motorState.focStateMachine = ALIGN;
    mcApp_SincosParam.Angle = 0.0f;
	mcApp_SVGenParam.PWMPeriod = (float)MAX_DUTY;
    Align_Counter = 0;
    mcApp_motorState.focStateMachine = ALIGN;
    mcApp_motorState.focStart = 1;
    
    gPositionCalc.elec_rotation_count = 0;
    gPositionCalc.prev_position_count = 0;
    gPositionCalc.present_position_count = 0;
    speed_ref_filtered = 0.0f;
    potPosition = 0.0f;
    position_filtered =0.0f;
    mcApp_Position_PIParam.qInMeas = 0.0f;
    
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT ));
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT ));
    status |= (uint8_t)(pwm_flag=TCC0_PWM24bitDutySet(TCC0_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT ));
    PWM_Output_Enable();
    fixed_pot =1;
	if (status!=1U){
        /* Error log*/
    }
}

void mcApp_motorStop(void)
{
    mcApp_motorState.focStart = 0U;
    mcApp_ControlParam.IdRef = 0.0f;
    mcApp_ControlParam.IqRef = 0.0f;
    mcApp_I_DQParam.d = 0.0f;
    mcApp_I_DQParam.q = 0.0f;
            
    PWM_Output_Disable(); 
}

void mcApp_motorStartToggle(void)
{
    mcApp_motorState.motorStart = (uint8_t)(!(bool)mcApp_motorState.motorStart);
    if(mcApp_motorState.motorStart == 1U)
    {
        mcApp_motorStart();
    }
    else
    {
        mcApp_motorStop();
    }
}





void __NO_RETURN OC_FAULT_ISR(uintptr_t context)
{
    mcApp_motorStop();
    mcApp_motorState.motorStart = 0;
    mcApp_motorState.focStart = 0;
    LED1_OC_FAULT_Set();
    while(true)
    {
    	/* Fault Operation */
    }
    
}