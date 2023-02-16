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
#include "mc_pll.h"
#include <math.h>

/*******************************************************************************
 * Constants 
 *******************************************************************************/
/*
 * Zero Boundary
 */
#define EPSILON   1E-31

/*
 * Constant value for PI
 */
#define CONSTANT_Pi        (float)3.14159265358979323846

/*
 * Constant value for 2PI
 */
#define CONSTANT_2Pi        (float)( 2.0f * CONSTANT_Pi )

/*
 * Constant value for root 3
 */
#define CONSTANT_squareRootOf3              ((float)1.732)

/*
 * Back EMF constant conversion factor from Vpeak / kRPM to Vpeak-s/rad
 */
#define  CONSTANT_vPeakPerKrpmTovPeakSecPerRad (float)( 3.0f  / ( 100.0f * CONSTANT_squareRootOf3 * CONSTANT_Pi * NOPOLESPAIRS ))

/*
 * RPM to electrical rad/s conversion factor
 */
#define  CONSTANT_mechRpmToElecRadPerSec  (float)( CONSTANT_2Pi * NOPOLESPAIRS / 30.0f ) 

extern void mcLib_WrapAngleTo2Pi( float * const angle );
/*******************************************************************************
 Private data-types 
 *******************************************************************************/
typedef struct
{
    float  ealpha;
    float  ebeta;
    float  Ed;
    float  Eq;
    float  ualpha;
    float  ubeta;
    float ialpha;
    float ibeta;
    float  theta;
    float  Wre;
    float  accel;
}tmcPll_StateVariables_s;

typedef struct
{
    float   dLsByDt;
    float   Rs;
    float   oneByKe;
    float   EdqFilterParam;
    float   WrFilterParam;
    float   Wrmin;
    float   Ts;
}tmcPll_Parameters_s;

/*******************************************************************************
 Private variables 
 *******************************************************************************/
static tmcPll_StateVariables_s mcPll_StateVariables_mds;
static tmcPll_Parameters_s mcPll_Parameters_mds;

/*******************************************************************************
 Interface variables 
 *******************************************************************************/
tmcPll_ModuleData_s mcPllI_ModuleData_gds;

/*******************************************************************************
 Private Functions 
 *******************************************************************************/
static inline void mcPll_EulerFilter( float new, float * old, float filterParam )
{
    *old += ( new - ( *old ) ) * filterParam;
 }

void mcLib_WrapAngleTo2Pi( float * const angle )
{
    if(*angle >= CONSTANT_2Pi )
    {
        *angle -= CONSTANT_2Pi;
    }
    else if( 0.0f > *angle )
    {
        *angle += CONSTANT_2Pi;
    }
    else
    {
       /* Do nothing */
    }
}


/*******************************************************************************
 Interface Functions 
 *******************************************************************************/

/*! \brief Rotor position calculation reset
 * 
 * Details.
 * Rotor position calculation reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcPllI_PosCalInit(tmcPll_ModuleData_s * const module)
{
    float Ke;
    
    tmcPll_Parameters_s * pParam;
    
    /* Set input ports */
    mcPllI_InputPortsSet( &module->inPort);
    
    /* Set output ports */
    mcPllI_OutputPortsSet(&module->outPort);
    
    /* Update parameters */
     mcPllI_UserParametersSet(&module->userParam);
     
    /* Update and calculate independent and dependent parameters respectively */
    pParam = &mcPll_Parameters_mds;
    pParam->Rs = module->userParam.Rs;
    pParam->Wrmin = CONSTANT_mechRpmToElecRadPerSec * CONFIG_SpeedThresholdForCloseLoopInRpm;
    pParam->Ts = module->userParam.Ts;
    
    pParam->dLsByDt  = module->userParam.Ls / module->userParam.Ts;
    
    Ke = CONSTANT_vPeakPerKrpmTovPeakSecPerRad * module->userParam.Ke;
    pParam->oneByKe  = 1.0f / Ke;
    pParam->EdqFilterParam = module->userParam.EdqFilterBandwidth;
    pParam->WrFilterParam = module->userParam.WrFilterBandwidth;
}


/*! \brief Rotor position calculation 
 * 
 * Details.
 * Rotor position calculation 
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
     static tmcMocI_PHASOR_s phasor;

void mcPllI_PosCalRun(tmcPll_ModuleData_s * const module)
{


    tmcPll_InputPorts_s * pInput;
    tmcPll_OutputPorts_s * pOutput;
    tmcPll_Parameters_s * pParam;
    tmcPll_StateVariables_s * pState;
    
    pInput = &module->inPort;
    pOutput = &module->outPort;
    pParam = &mcPll_Parameters_mds;
    pState = &mcPll_StateVariables_mds;
    float ealpha_temp, ebeta_temp;
    float Wre_temp, Ed_temp, Eq_temp;
//#if (ENABLE == FIELD_WEAKENING )
    float esSquare;
//#endif
    
    /* Calculate back EMF along alpha and beta axis */
    ealpha_temp  =   pState->ialpha - (*pInput->ialpha );
    ealpha_temp *=  pParam->dLsByDt;
    ealpha_temp -= (*pInput->ialpha * pParam->Rs );
    ealpha_temp += pState->ualpha; 
    mcPll_EulerFilter( ealpha_temp, &pState->ealpha, 1.0f );
    
    ebeta_temp  =   pState->ibeta - (*pInput->ibeta );
    ebeta_temp *=  pParam->dLsByDt;
    ebeta_temp -= (*pInput->ibeta * pParam->Rs );
    ebeta_temp +=  pState->ubeta; 
    mcPll_EulerFilter( ebeta_temp, &pState->ebeta, 1.0f );

//#if (ENABLE == FIELD_WEAKENING )
    /* Calculate BEMF for field weakening*/
    esSquare =  ( pState->ebeta * pState->ebeta ) +  ( pState->ealpha * pState->ealpha );
    if (esSquare>=0.0f){
      *pOutput->backEMF = (float)sqrt(esSquare);
    }else{
      *pOutput->backEMF = (float)sqrt(-esSquare);  
    }
//#endif
    
    /* Determine back EMF along direct and quadrature axis using estimated angle */
    phasor.angle =  pState->theta;
    mcLib_SinCosGen(&phasor );
    
    Ed_temp  =     pState->ealpha * phasor.Cos;
    Ed_temp +=  ( pState->ebeta * phasor.Sin );
    mcPll_EulerFilter( Ed_temp, &pState->Ed, pParam->EdqFilterParam);
    
    Eq_temp  =    -pState->ealpha * phasor.Sin;
    Eq_temp +=  ( pState->ebeta * phasor.Cos );
    mcPll_EulerFilter( Eq_temp, &pState->Eq, pParam->EdqFilterParam);
     
     /* Determine speed  */
    if( pState->Eq  > 0.0f )
    {
         Wre_temp  = pState->Eq -  pState->Ed;
    }
    else
    {
         Wre_temp  = pState->Eq + pState->Ed;
    }
    
    Wre_temp *= pParam->oneByKe;
    mcPll_EulerFilter( Wre_temp, &pState->Wre, pParam->WrFilterParam);
               
    /*Determine phase angle */
    pState->theta += ( pParam->Ts * pState->Wre );
    mcLib_WrapAngleTo2Pi( &pState->theta );
    
    /* Update state variables for next cycle calculation */
    pState->ualpha  = (*pInput->ualpha ) * (*pInput->umax );
    pState->ubeta  =  (*pInput->ubeta ) * (*pInput->umax );
    pState->ialpha  = *pInput->ialpha;
    pState->ibeta   = *pInput->ibeta;
      
    /* Update output ports */
    *pOutput->elecAngle = pState->theta;
    *pOutput->elecSpeed = pState->Wre;
}


/*! \brief Rotor position calculation reset
 * 
 * Details.
 * Rotor position calculation reset
 * 
 * @param[in]: 
 * @param[in/out]:
 * @param[out]:
 * @return:
 */
void mcPllI_PosCalReset(tmcPll_ModuleData_s * const module)
{
    tmcPll_StateVariables_s * pState;
    pState = &mcPll_StateVariables_mds;
    
    /* Reset state variables */
    pState->Ed = 0.0f;
    pState->Eq = 0.0f;
    pState->ealpha = 0.0f;
    pState->ebeta = 0.0f;
    pState->theta = 0.0f;
    pState->Wre = 0.0f;
    pState->accel = 0.0f;
    pState->ualpha = 0.0f;
    pState->ubeta = 0.0f;
    pState->ialpha = 0.0f;
    pState->ibeta = 0.0f;
    
    /* Reset output ports */
    *module->outPort.elecAngle = 0.0f;
     *module->outPort.elecSpeed = 0.0f;  
     *module->outPort.backEMF = 0.0f;  
}
