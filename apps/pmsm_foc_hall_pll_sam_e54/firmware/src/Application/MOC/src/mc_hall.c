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
#include "mc_hall.h"

/*******************************************************************************
 * Constants 
 *******************************************************************************/
#ifdef LONG_HURST
#define CONSTANT_HallOffsetInDegree (float)(-30.0f )
#define CONSTANT_HallOffsetInRad  (float)( CONSTANT_Pi * CONSTANT_HallOffsetInDegree/ 180.0f )
static const float TABLE_HallJumpAngle[8u] = 
{
    /* Invalid: 000  */  CONSTANT_Dummy,
    /* Sector III: 001 */   CONSTANT_2PiBy3,
    /* Sector V :010 */   CONSTANT_4PiBy3,
    /* Sector IV : 011 */    CONSTANT_Pi,
    /* Sector I  : 100 */   0.0f,
    /* Sector II :  101 */    CONSTANT_1PiBy3,
    /* Sector VI  :110 */    CONSTANT_5PiBy3,
    /* Invalid: 111 */     CONSTANT_Dummy 
};
#endif

/*******************************************************************************
 Private data-types 
 *******************************************************************************/

typedef struct
{
    uint8_t pattern;
    float angle;
    float jumpAngle;
    float jumpAngleLast;
    int16_t jumpDirection;
    float electricalSpeed;
    float electricalSpeedFilt;
    uint32_t tcHallToHallIsr;
    uint32_t tcFIFO[TC_HALL_FIFO_LEN];
    uint32_t tcSum;
    uint16_t cntTCfifo;
    uint8_t noOfJumps;    
 }mcHall_StateVariable_s;
 
/*******************************************************************************
 Private variables 
 *******************************************************************************/
static mcHall_StateVariable_s mcHall_StateVariables_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcHall_ModuleData_s mcHallI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/

static void mcHallI_WrapFromMinusPiToPi( float * const angle )
{
    if( *angle > CONSTANT_Pi )
    {
        *angle -= CONSTANT_2Pi;
    }
    else if(*angle < -CONSTANT_Pi)
    {
        *angle += CONSTANT_2Pi;
    }
    else
    {
        /* Do nothing */
    }
}

static void mcHall_ElectricalSpeedCalculate( mcHall_StateVariable_s * const pState )
{
    uint16_t * cnt = &(pState->cntTCfifo);
    
    pState->noOfJumps++;
    pState->tcSum += pState->tcHallToHallIsr;
    (*cnt)++;
    if(TC_HALL_FIFO_LEN <= *cnt)
    {
        *cnt = 0u;
    }
    pState->tcSum -= pState->tcFIFO[*cnt];
    pState->tcFIFO[*cnt] = pState->tcHallToHallIsr;
   
    if( TC_HALL_FIFO_LEN > pState->noOfJumps )
    {
        /* Calculate electrical  speed based on one jump */
        pState->electricalSpeed = CONSTANT_1PiBy3 / (float)pState->tcHallToHallIsr * RL_TC0_FREQ;
    }
    else 
    {
        /* Calculate electrical  speed based on complete revolution */
        pState->electricalSpeed = CONSTANT_2Pi / (float)pState->tcSum * RL_TC0_FREQ;
        pState->noOfJumps = TC_HALL_FIFO_LEN;
    }
    
    /* Calculate filtered speed of the motor */
    pState->electricalSpeedFilt = pState->electricalSpeedFilt + 0.005f * ( pState->electricalSpeed - pState->electricalSpeedFilt);
}

/*******************************************************************************
 Interface Functions 
 *******************************************************************************/
void mcHallI_HallSignalProcessInit( tmcHall_ModuleData_s * const pModule )
{
    /* Set input ports */
    mcHallI_InputPortsSet( &pModule->dInput);
    
    /* Set output ports */
    mcHallI_OutputPortsSet(&pModule->dOutput);
}

static float Offset = 0.0f;
void mcHallI_HallSignalProcessRun( tmcHall_ModuleData_s * const pModule )
{
    float delta;
        
    mcHall_StateVariable_s * pState;
       
    pState = &mcHall_StateVariables_mds;
    
    /* Calculate speed from elapsed time */
    pState->tcHallToHallIsr = *pModule->dInput.tcHallToHallIsr;
    mcHall_ElectricalSpeedCalculate( pState);
    
    /* Get HALL pattern */
    pState->pattern =  PDEC_HALLPatternGet();
       
    /* Get HALL jump angle and direction from table */
    pState->jumpAngle = TABLE_HallJumpAngle[pState->pattern] + Offset;
    
    if( pState->jumpAngle > CONSTANT_2Pi )
    {
        pState->jumpAngle -= CONSTANT_2Pi;
    }
    else if( pState->jumpAngle < 0.0f )
    {
         pState->jumpAngle += CONSTANT_2Pi;
    }
    else
    {
        /* Dummy branch for MISRAC compliance*/
    }
    
    /* Determine HALL direction */
    delta = pState->jumpAngle - pState->jumpAngleLast;
    mcHallI_WrapFromMinusPiToPi(&delta);
       
    if( delta > 0.0f )
    {
        pState->jumpDirection = 1;
    }
    else 
    {
        pState->jumpDirection = -1;
    }   
    
    /* Set last jump angle for next cycle */
    pState->jumpAngleLast = pState->jumpAngle;
   
}


void mcHallI_HallProcessBufferRead( tmcHall_OutputBuffer_s * const pOutput )
{       
    mcHall_StateVariable_s * pState;   
    pState = &mcHall_StateVariables_mds;
    
     /* Update output buffer */
    pOutput->baseAngle = pState->jumpAngle;
    pOutput->directionFlag = (float)pState->jumpDirection ;
    pOutput->meanAngle = pState->jumpAngle + (float)pState->jumpDirection * CONSTANT_1PiBy6;
    pOutput->boundAngle = pState->jumpAngle + (float)pState->jumpDirection * CONSTANT_1PiBy3;
    pOutput->elecSpeed = pState->electricalSpeed;
}


static void mcHall_HallPatternRead( mcHall_StateVariable_s * const pState )
{
    uint8_t A, B, C;
    bool port_status;
    A = (uint8_t)(port_status = PORT_PinRead(PORT_PIN_PC16));
    B = (uint8_t)(port_status = PORT_PinRead(PORT_PIN_PC17));  
    C = (uint8_t)(port_status = PORT_PinRead(PORT_PIN_PC18));   
    
    pState->pattern = ( ( C << 2U) | (B << 1U) | A ) & 0x7u;
}

void mcHall_HallDataInit(void )
{
    uint16_t zCount;
    
    mcHall_StateVariable_s * pState;
      
    pState = &mcHall_StateVariables_mds;
   
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
    
    for(zCount = 0u; zCount<TC_HALL_FIFO_LEN; zCount++)
    {
        pState->tcFIFO[zCount] = 0U;
    }
    pState->cntTCfifo = 0u;
    pState->tcSum = 0u;
    mcHall_HallPatternRead(pState);
    pState->jumpAngle = TABLE_HallJumpAngle[pState->pattern];
    pState->angle = pState->jumpAngle;
    pState->jumpDirection = 0;    
    pState->noOfJumps = 0u;
    pState->tcHallToHallIsr = 0u;
    pState->electricalSpeed = 0.0f;
    pState->electricalSpeedFilt = 0.0f;
      
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
}


uint8_t mcHallI_HallPatternGet( void )
{   
    return mcHall_StateVariables_mds.pattern;
}

