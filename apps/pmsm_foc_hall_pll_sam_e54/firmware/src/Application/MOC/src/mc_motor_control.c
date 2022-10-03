/*******************************************************************************
 Motor Control Application Source File

  File Name:
    mc_motor_control.c

  Summary:
 Motor Control Application Variable and Function definitions.

  Description:
    This file contains the variable initializations and function definitions for
 *  specific to Motor Control Application (excluding variables and functions 
 *  defined by Motor ControlLibrary
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

/*******************************************************************************
 Header inclusions
 *******************************************************************************/
#include "mc_motor_control.h"
#include "mc_hall.h"
#include "mc_pll.h"

/*******************************************************************************
 Constants
 *******************************************************************************/
#define CONSTANT_3PiBy2 (float)(1.5 * M_PI)
#define CONSTANT_2Pi  (float)(2.0 * M_PI)

#define CONFIG_AlignmentTime   (float)(0.5f)
#define CONSTANT_AlignmentCounts  (float)( CONFIG_AlignmentTime * PWM_FREQ )

#define CONFIG_RampTime   (float)(5.0f)
#define CONSTANT_RampCounts  (float)( CONFIG_RampTime * PWM_FREQ )

#define CONFIG_OpenLoopStabSpeedInRpm    (float)(500.0f)
#define CONSTANT_RpmToRadPerSec   (float)( CONSTANT_2Pi  * NOPOLESPAIRS / 60.0f )
#define CONFIG_OpenLoopStabSpeedInRadPerLoop   (float)(CONSTANT_RpmToRadPerSec * CONFIG_OpenLoopStabSpeedInRpm/ PWM_FREQ )

#define CONSTANT_OpenLoopAccelRate  (float)(CONFIG_OpenLoopStabSpeedInRadPerLoop / CONSTANT_RampCounts )

#define CONFIG_OpenLoopCurrent  (float)(0.4f)

#define CONFIG_HoldTimeCount  (uint32_t)( 2.0f * PWM_FREQ )


#define CONFIG_AngleStepSize  (float)( 10.0f )
#define CONSTANT_AngleStepSize  (float)( CONSTANT_Pi * CONFIG_AngleStepSize / 180.0f  )
#define CONSTANT_NumberOfSamples  (uint8_t)( ( CONSTANT_2Pi/ CONSTANT_AngleStepSize ) + 0.5 )

#define CONSTANT_PulsePeriodInLoopCount   (uint32_t)(0.5f * PWM_FREQ )
#define CONSTANT_PulseDutyInLoopCount   (uint32_t)(CONSTANT_PulsePeriodInLoopCount >> 1u  )
#define CONSTANT_PulseMaximumPeak    (float)( 0.4f)
#define CONSTANT_PulseMinimumPeak    (float)( 0.2f)

#define CONFIG_SixStepEnableSpeed (float)(200.0f)
#define CONFIG_SixStepDisableSpeed (float)(300.0f)
#define CONFIG_SensorlessFocDisableSpeed (float)(600.0f)
#define CONFIG_SensorlessFocEnableSpeed (float)(700.0f)


#define CONSTANT_SixStepEnableSpeed  (float)( CONFIG_SixStepEnableSpeed  * CONSTANT_RpmToRadPerSec )
#define CONSTANT_SixStepUpperThreshold  (float)( CONFIG_SixStepDisableSpeed  * CONSTANT_RpmToRadPerSec )
#define CONSTANT_SensorlessFocDisableSpeed  (float)( CONFIG_SensorlessFocDisableSpeed  * CONSTANT_RpmToRadPerSec )
#define CONSTANT_SensorlessFocEnableSpeed  (float)( CONFIG_SensorlessFocEnableSpeed  * CONSTANT_RpmToRadPerSec )


const uint16_t TABLE_ClockwisePattern[8u] = {
    0x0077, /*Invalid Hall Code - Disable all PWM*/
    0x1073, /*1: 120 degree */
    0x2076, /*2: 240 degree */
    0x2073, /*3: 180 degree */
    0x4075, /*4: 0 degree   */
    0x1075, /*5: 60 degree  */
    0x4076, /*6: 300 degree */
    0x0077  /*7: Invalid Hall Code - Disable all PWM*/            
};

const uint16_t TABLE_CounterClockwisePattern[8u] = {
    0x0077, /*Invalid Hall Code - Disable all PWM*/
    0x4076, /*1: 120 degree */
    0x1075, /*2: 240 degree */
    0x4075, /*3: 180 degree */
    0x2073, /*4: 0 degree   */
    0x2076, /*5: 60 degree  */
    0x1073, /*6: 300 degree */
    0x0077  /*7: Invalid Hall Code - Disable all PWM*/            
};


/*******************************************************************************
 User defined data-types 
 *******************************************************************************/
typedef enum _tmcMocI_OpenLoopStates_e
{
    openLoopState_Align,
    openLoopState_Ramp,
    openLoopState_Stab
}tmcMocI_OpenLoopStates_e;

typedef struct _tmcMocI_OpenLoopControl_s
{
    tmcMocI_OpenLoopStates_e state;
    float rampRate;
    uint32_t  track;
    float openLoopAngle;
}tmcMocI_OpenLoopControl_s;


typedef struct _tmcMocI_AnglePatternPair_s
{
    float setAngle;
    uint8_t pattern;
}tmcMocI_AnglePatternPair_s;

typedef struct _tmcMocI_HallSensorTuning_s
{
    uint32_t track;
    tmcMocI_AnglePatternPair_s pairs[50u];
}tmcMocI_HallSensorTuning_s;

typedef struct _tmcMoc_CurrentLoopTuning_s
{
    uint32_t zTrack;
    
}tmcMoc_CurrentLoopTuning_s;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcMocI_MotorControl_s     mcMocI_MotorControl_gds;
tmcMocI_InputPorts_s      mcMocI_InputPorts_gds;
tmcMocI_OpenLoopControl_s mcMocI_OpenLoopControlData_gds;
tmcMocI_HallSensorTuning_s mcMocI_HallSensorTuning_gds;
tmcMoc_CurrentLoopTuning_s mcMoc_CurrentLoopTuning_mds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

/*! \brief Read motor control inputs  
 * 
 * Details.
 * Read motor control inputs  
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
__STATIC_INLINE void mcMoc_MotorControlInputsRead( tmcMocI_MotorControl_s * const pControl, 
                                                   const tmcMocI_InputPorts_s * const pInput )
{
    /* Read phase currents */
    pControl->iABC.a = *pInput->iA ;
    pControl->iABC.b = *pInput->iB ;
   
    /* Read rotor position angle */
    pControl->sensedAngle = *pInput->sensedAngle;
    pControl->sensorlessAngle = *pInput->senselessAngle;
    
    /* Read DC link voltage */
    
    /* Read command speed */
    pControl->nQPI.reference = *pInput->cmdSpeed;
    
    /* Read electrical speed of the motor */
    pControl->sensedSpeed = *pInput->sensedSpeed;
    pControl->sensorlessSpeed = *pInput->senselessSpeed;
    
}


void mcMocI_OpenLoopControlReset( tmcMocI_MotorControl_s * const pControl  )
{
    
}

void mcMocI_OpenLoopControlOverride( tmcMocI_MotorControl_s * const pControl  )
{
    tmcMocI_OpenLoopControl_s * pState;
    
    pState = &mcMocI_OpenLoopControlData_gds;
    
    pState->track++;
    
    switch( pState->state)
    {
        case openLoopState_Align:
        {
            if(  pState->track < CONSTANT_AlignmentCounts )
            {
                pState->openLoopAngle = CONSTANT_3PiBy2;
                pControl->nQPI.Yout = CONFIG_OpenLoopCurrent;
            }
            else
            {
                pState->state = openLoopState_Ramp;
                pState->track = 0u;
            }
        }
        break;
        
        case openLoopState_Ramp:
        {
            if(  pState->track < CONSTANT_RampCounts )
            {
               pState->rampRate += CONSTANT_OpenLoopAccelRate;
               pState->openLoopAngle += pState->rampRate;
               pControl->nQPI.Yout = CONFIG_OpenLoopCurrent;
            }
            else
            {
                pState->state = openLoopState_Stab;
                pState->track = 0u;
            }
        }
        break;
        
        case openLoopState_Stab:
        {
             pState->openLoopAngle += pState->rampRate;
             pState->track = 0u;
             pControl->nQPI.Yout = CONFIG_OpenLoopCurrent;
        }
        break;
    }    
    
    if(  pState->openLoopAngle > CONSTANT_2Pi )
    {
         pState->openLoopAngle -= CONSTANT_2Pi;
    }
    else  if( pState->openLoopAngle < 0.0f )
    {
         pState->openLoopAngle += CONSTANT_2Pi;
    }
    pControl->tPHASOR.angle = pState->openLoopAngle;
}

void mcMocI_HallSensorTuning( tmcMocI_MotorControl_s * const pControl )
{  
    static uint8_t zCounter;
    tmcMocI_HallSensorTuning_s * pState;
    
    pState = &mcMocI_HallSensorTuning_gds;   
    pState->track++;
    
    if( zCounter < CONSTANT_NumberOfSamples )
    {
        if( pState->track < CONFIG_HoldTimeCount )
        {
             pControl->iDPI.reference  = CONFIG_OpenLoopCurrent;
             pControl->tPHASOR.angle= zCounter * CONSTANT_AngleStepSize;
             pState->pairs[zCounter].setAngle = pControl->tPHASOR.angle;
             pState->pairs[zCounter].pattern = mcHallI_HallPatternGet();
        }
        else 
        {
             zCounter++;
             pState->track = 0u;
        }
    }
    else
    {
        zCounter = CONSTANT_NumberOfSamples;
    }
            
}

void mcMocI_CurrentLoopTuning( tmcMocI_MotorControl_s * const pControl )
{  
    tmcMoc_CurrentLoopTuning_s * pState;
    
    pState = &mcMoc_CurrentLoopTuning_mds;   
    pState->zTrack++;
    
    if(  pState->zTrack < CONSTANT_PulseDutyInLoopCount )
    {
        pControl->nQPI.Yout  = CONSTANT_PulseMaximumPeak;
    }
    else if( pState->zTrack < CONSTANT_PulsePeriodInLoopCount)
    {
         pControl->nQPI.Yout  = CONSTANT_PulseMinimumPeak;
    }      
    else 
    {
        pState->zTrack = 0u;
    }
}

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Motor Control initialization tasks 
 * 
 * Details.
 * Motor Control initialization tasks 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlInit(tmcMocI_MotorControl_s * const pControl )
{
    /* Set input ports */
    mcMocI_InputPortsSet(&mcMocI_InputPorts_gds);
    
    /* Set motor control state to IDLE */
    mcMocI_MotorControl_gds.stateMachine = ControlState_Idle;
        
    /* Initialize D-axis controller */      
    pControl->iDPI.Kp = CONFIG_IdControllerKp;       
    pControl->iDPI.Ki = CONFIG_IdControllerKi;              
    pControl->iDPI.Kc = CONFIG_IdControllerKc;
    pControl->iDPI.Yi = 0;
    pControl->iDPI.Ymax = CONFIG_IdControllerYmax;
    pControl->iDPI.Ymin = -mcMocI_MotorControl_gds.iDPI.Ymax;

    mcLib_InitPI(&pControl->iDPI);

    /* Initialize Q-axis controller */  
    pControl->iQPI.Kp = CONFIG_IqControllerKp;    
    pControl->iQPI.Ki = CONFIG_IqControllerKi;
    pControl->iQPI.Kc = CONFIG_IqControllerKc;
    pControl->iQPI.Yi = 0;
    pControl->iQPI.Ymax = CONFIG_IqControllerYmax;
    pControl->iQPI.Ymin = -mcMocI_MotorControl_gds.iQPI.Ymax;
    mcLib_InitPI(&mcMocI_MotorControl_gds.iQPI);

    /* Initialize speed controller */ 
    pControl->nQPI.Kp = CONFIG_SpeedControllerKp;       
    pControl->nQPI.Ki = CONFIG_SpeedControllerKi;       
    pControl->nQPI.Kc = CONFIG_SpeedControllerKc;  
    pControl->nQPI.Yi = 0;
    pControl->nQPI.Ymax = CONFIG_SpeedControllerYmax;   
    pControl->nQPI.Ymin = -mcMocI_MotorControl_gds.nQPI.Ymax;

    mcLib_InitPI(&mcMocI_MotorControl_gds.nQPI);
    	   
    /* Initialize phase angle  */
    pControl->tPHASOR.angle = 0;
    
    /* Initialize space vector modulator */
    mcPwmI_SpaceVectorModulationInit( &pControl->ySVPWM);
        
    /* Rotor position calculation module initialization */
     mcPllI_RotorPositionCalculationInit(&mcPllI_ModuleData_gds);
    
    /* Initialize default control Flags */
    pControl->Flags.direction = 1.0f;
    pControl->Flags.openLoop = 0u;
           
    return;   
}

/*! \brief Block commutation 
 * 
 * Details.
 * Block commutation 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_SixStepCommutation( tmcMocI_MotorControl_s * const pState )
{
    uint8_t pattern;
    tmcMocI_SVPWM_s duty;

    tmcMocI_PI_s * nQPI;
    nQPI  = &pState->nQPI;
    
    /* Get HALL pattern */
    pattern = mcHallI_HallPatternGet();
             
    /* Determine modulation index */
    if( pState->Flags.direction == 1u )
    {
        TCC0_REGS->TCC_PATTBUF =(uint16_t)(TABLE_ClockwisePattern[pattern]);
    }
    else
    {
        TCC0_REGS->TCC_PATTBUF =(uint16_t)(TABLE_CounterClockwisePattern[pattern]);
    }
     
    /* Run speed controller */            
     mcLib_PiControlRun(nQPI);
     
     /* Update Q-axis controller integral value */
     pState->iQPI.Yi =  nQPI->Yout;
     pState->iQPI.Yout =  nQPI->Yout;
     pState->uDQ.q =  nQPI->Yout;
     
     /* Update D-axis controller integral value */
     pState->iDPI.Yi =  0.0f;
     pState->iDPI.Yout =  0.0f;
     pState->uDQ.d = 0.0f;

     
     mcLib_SinCosGen(&pState->tPHASOR);
     mcLib_InvParkTransform(&pState->uDQ, &pState->tPHASOR, &pState->uAB);
          
     duty.dPWM_A = duty.dPWM_B = duty.dPWM_C =  (uint32_t)( pState->ySVPWM.ts *  pState->Flags.direction * nQPI->Yout );   

     /* Set inverter duty cycles */
     mcBseI_InverterDutySet(&duty);
}


/*! \brief Space vector commutation 
 * 
 * Details.
 * Space vector commutation 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_FieldOrientedControl( tmcMocI_MotorControl_s * const pState )
{
    float temp;
    
    tmcMocI_PI_s * iDPI;
    tmcMocI_PI_s * iQPI;
    tmcMocI_PI_s * nQPI;
   

    iDPI  = &pState->iDPI;
    iQPI  = &pState->iQPI;
    
     /* Run speed controller */    
     nQPI  = &pState->nQPI;
     nQPI->feedback = pState->sensedSpeed;
     mcLib_PiControlRun(nQPI);
  
    /* D-axis controller */
    iDPI->feedback = pState->iDQ.d;         
    iDPI->reference  = 0.0f; 
    mcLib_PiControlRun(iDPI);
    pState->uDQ.d    =  iDPI->Yout;  

    /* Q-axis controller */
    temp = MAX_NORM_SQ - (iDPI->Yout * iDPI->Yout);
    iQPI->Ymax = sqrtf(temp);
    iQPI->Ymin = -iQPI->Ymax;  
    iQPI->reference = pState->nQPI.Yout;
    iQPI->feedback = pState->iDQ.q;         
   
    mcLib_PiControlRun(iQPI);
    pState->uDQ.q = iQPI->Yout;

    /* Inverse Park transformation */
    mcLib_SinCosGen(&pState->tPHASOR);
    mcLib_InvParkTransform(&pState->uDQ, &pState->tPHASOR, &pState->uAB);

    /* Space vector modulation */
    mcPwmI_SpaceVectorModulationRun(&pState->uAB , &pState->ySVPWM);
    
    /* Set inverter duty cycles */
    mcBseI_InverterDutySet(&pState->ySVPWM);
}

/*! \brief Motor control tasks
 * 
 * Details.
 * Motor control tasks 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlRun( tmcMocI_MotorControl_s * const pState )
{
    if( pState->runStatus)
    {
        /* Read motor control inputs  */
        mcMoc_MotorControlInputsRead(pState, &mcMocI_InputPorts_gds);

        /* Clark Transformation */
        mcLib_ClarkeTransform(&pState->iABC, &pState->iAB);

        /* Park Transformation */
        mcLib_ParkTransform(&pState->iAB, &pState->tPHASOR, &pState->iDQ);
        
        /* Sensor-less rotor position estimation */
        mcPllI_RotorPositionCalculationRun(&mcPllI_ModuleData_gds); 

        /* Motor control state machines */
        switch (pState->stateMachine)
        {
            case ControlState_Idle:
            {

            }
            break;

            case ControlState_HallTune:
            {
                 mcMocI_HallSensorTuning(pState);
            }
            break;

            case ControlState_OpenLoop:
            {
                /* Override space vector angle */
                mcMocI_OpenLoopControlOverride(pState);

            }
             break;
             
            case ControlState_CurrentLoopTune:
            {
                mcMocI_CurrentLoopTuning(pState);
            }
            break;

            case ControlState_SixStep:
            {  
                pState->tPHASOR.angle = pState->sensedAngle;
                pState->nQPI.feedback = pState->sensedSpeed;
                
                mcMocI_SixStepCommutation(pState);

                if( pState->sensedSpeed > CONSTANT_SixStepUpperThreshold )
                {
                    pState->stateMachine = ControlState_SensoredFoc;
                }
            }
            break;
            
            case ControlState_SensoredFoc:
            {              
                TCC0_REGS->TCC_PATTBUF = 0x0000; 
                pState->tPHASOR.angle = pState->sensedAngle;
                pState->nQPI.feedback = pState->sensedSpeed;
               
                mcMocI_FieldOrientedControl(pState);
                
                if( pState->sensedSpeed > CONSTANT_SensorlessFocEnableSpeed )
                {
                    pState->stateMachine = ControlState_SensorlessFoc;
                }
                else if ( pState->sensedSpeed < CONSTANT_SixStepEnableSpeed )
                {
                    pState->stateMachine = ControlState_SixStep;
                }
                else
                {
                    /* Stay in this state */
                }
            }
            break;
                       
            case ControlState_SensorlessFoc:
            {
                TCC0_REGS->TCC_PATTBUF = 0x0000; 
                pState->tPHASOR.angle = pState->sensorlessAngle;
                pState->nQPI.feedback = pState->sensorlessSpeed;
                mcMocI_FieldOrientedControl(pState);
                
                if( pState->sensorlessSpeed < CONSTANT_SensorlessFocDisableSpeed )
                {
                    pState->stateMachine = ControlState_SensoredFoc;
                }
                else
                {
                   /* Stay in this state */
                }
            }
            break;
            
            default:
            {

            }
            break;
        }
    }
    else 
    {
       mcMocI_MotorControlReset(pState);
    }
    
     mcMocI_AlphaAxisVoltage_gdf32 = pState->uAB.alpha;
     mcMocI_BetaAxisVoltage_gdf32 = pState->uAB.beta;
     mcMocI_AlphaAxisCurrent_gdf32 =  pState->iAB.alpha;
     mcMocI_BetaAxisCurrent_gdf32 = pState->iAB.beta;
}

/*! \brief Motor control reset 
 * 
 * Details.
 * Motor control reset 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorControlReset( tmcMocI_MotorControl_s * const pState )
{
     /* Set control state to IDLE*/
        pState->stateMachine = ControlState_Idle;

        /* Reset Q axis current controller */
        mcLib_InitPI(&mcMocI_MotorControl_gds.iQPI);

        /* Reset D axis current controller */
        mcLib_InitPI(&mcMocI_MotorControl_gds.iDPI);

        /* Reset speed controller */
        mcLib_InitPI(&mcMocI_MotorControl_gds.nQPI);
        
        /* Reset PWM modulator */
        mcPwmI_SpaceVectorModulationReset( &mcMocI_MotorControl_gds.ySVPWM);
        
        /*  Reset PLL based sensor-less estimation  */
        mcPllI_RotorPositionCalculationReset(&mcPllI_ModuleData_gds); 

        /* Disable inverter  */
        mcBseI_InverterDisable(); 

        /* Stop TC2 capture  */
        TC2_CaptureStop();
        
        mcHall_HallDataInit();
        
        mcFcoI_HallToAdcISRTimerCount_gdu32 = 0u;
}

/*! \brief Motor start
 * 
 * Details.
 * Motor start
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStart(tmcMocI_MotorControl_s * const pState)
{   
    if( pState->runStatus != 1u )
    {
        /* Set run status to 1 */
        pState->runCommand = 1u;

        if( pState->Flags.openLoop == 0u )
        {
            /* Set motor control state to CLOSE LOOP */
            pState->stateMachine = ControlState_SixStep;
        }
        else 
        {
            /* Set motor control state to OPEN LOOP*/
            pState->stateMachine = ControlState_OpenLoop;
        }
        /* Hall sensor initialize */
        mcRpoI_RotorPositionCalculationTrigger(&mcRpoI_ModuleData_gds);

        /* Start TC2 capture */
        TC2_CaptureStart();

        /* Enable PWM output */
        mcBseI_InverterEnable();   
    }
}

/*! \brief Motor stop
 * 
 * Details.
 * Motor stop
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStop(tmcMocI_MotorControl_s * const pState)
{
    /* Set run status to 0 */
    pState->runCommand = 0u;
}

/*! \brief Motor start toggle
 * 
 * Details.
 * Motor Toggle
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorStartToggle(tmcMocI_MotorControl_s * const pState)
{
    pState->Flags.start = !pState->Flags.start;
    if( pState->Flags.start)
    {
        mcMocI_MotorStart(pState);
    }
    else
    {
        mcMocI_MotorStop(pState);
    }
}

/*! \brief Motor direction change
 * 
 * Details.
 * Motor direction change
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcMocI_MotorDirectionToggle(tmcMocI_MotorControl_s * const pState)
{
    if(pState->Flags.start == 0)
    {
        /*Change Motor Direction Only when Motor is Stationary*/
        pState->Flags.direction = -pState->Flags.direction;
        LED2_Direction_Toggle();
    }
}