/*******************************************************************************
 Motor Control Library Filee - PLL Estimator

  Company:
    Microchip Technology Inc.

  File Name:
    mclib_generic_float.c

  Summary:
    This file contains the motor control algorithm functions.

  Description:
    This file contains the motor control algorithm functions like clarke transform,
    park transform. This library is implemented with float data type. 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

#include "mc_Lib.h"
#include <math.h>


static const float sineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Sine Table">
{
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f,
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f
};
// </editor-fold>
static const float cosineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Cosine Table">
{
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f,
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: MC PLL Estimator Routines
// *****************************************************************************
// *****************************************************************************
void mcLib_PLLEstimator(mcParam_PLLEstimator *pllestimatorParam,
                        mcParam_SinCos *scParam, mcParam_FOC *focParam, 
                        mcParam_AlphaBeta *I_alphabetaParam,
                        mcParam_AlphaBeta *V_alphabetaParam )
{
	float tempqVelEstim;
    mcParam_DQ BEMF_DQParam;
    mcParam_AlphaBeta BEMF_AlphaBetaParam;
    
    if(pllestimatorParam->qVelEstim < 0.0f)
    {
        tempqVelEstim = pllestimatorParam->qVelEstim * (-1.0f);    
    }
    else
    {
        tempqVelEstim = pllestimatorParam->qVelEstim;
    }
    
	pllestimatorParam->qDIalpha	=	(I_alphabetaParam->alpha-pllestimatorParam->qLastIalpha);
    pllestimatorParam->qVIndalpha = (pllestimatorParam->qLsDt * pllestimatorParam->qDIalpha);
    pllestimatorParam->qDIbeta	=	(I_alphabetaParam->beta-pllestimatorParam->qLastIbeta);
    pllestimatorParam->qVIndbeta= (pllestimatorParam->qLsDt * pllestimatorParam->qDIbeta);
    
    // Update LastIalpha and LastIbeta
    pllestimatorParam->qLastIalpha	=	I_alphabetaParam->alpha;
    pllestimatorParam->qLastIbeta 	=	I_alphabetaParam->beta;
    
    // Stator voltage equations
 	pllestimatorParam->qEsa = BEMF_AlphaBetaParam.alpha = pllestimatorParam->qLastValpha -
							((pllestimatorParam->qRs  * I_alphabetaParam->alpha))
							- pllestimatorParam->qVIndalpha;
							
 	pllestimatorParam->qEsb = BEMF_AlphaBetaParam.beta	= 	pllestimatorParam->qLastVbeta -
							((pllestimatorParam->qRs  * I_alphabetaParam->beta ))
							- pllestimatorParam->qVIndbeta;

    
//    // Update LastValpha and LastVbeta
	pllestimatorParam->qLastValpha = (focParam->MaxPhaseVoltage * V_alphabetaParam->alpha);
	pllestimatorParam->qLastVbeta = (focParam->MaxPhaseVoltage * V_alphabetaParam->beta);

    // Calculate Sin(Rho) and Cos(Rho)
    scParam->Angle 	=	pllestimatorParam->qRho;// + pllestimatorParam->RhoOffset; 


  
	mcLib_SinCosGen(scParam);

//    // Translate Back EMF (Alpha,beta)  ESA, ESB to Back EMF(D,Q) ESD, ESQ using Park Transform. 

    mcLib_ParkTransform(&BEMF_AlphaBetaParam , scParam, &BEMF_DQParam);
    
     pllestimatorParam->qEsd = BEMF_DQParam.d;
     pllestimatorParam->qEsq = BEMF_DQParam.q;
    // Filter first order for Esd and Esq
	pllestimatorParam->qEsdf			= pllestimatorParam->qEsdf+
							  ((pllestimatorParam->qEsd - pllestimatorParam->qEsdf) * pllestimatorParam->qKfilterEsdq) ;

	pllestimatorParam->qEsqf			= pllestimatorParam->qEsqf+
							  ((pllestimatorParam->qEsq - pllestimatorParam->qEsqf) * pllestimatorParam->qKfilterEsdq) ;


  			  
      
	if (tempqVelEstim>pllestimatorParam->qDecimate_Nominal_Speed)
    {
    	if(pllestimatorParam->qEsqf>0.0f)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf + pllestimatorParam->qEsdf)));
    	}
    } 
	else // if est speed<10% => condition VelRef<>0
    {
    	if(pllestimatorParam->qVelEstim>0.0f)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf+ pllestimatorParam->qEsdf))) ;
    	}
    }
     // The estimated speed is a filter value of the above calculated OmegaMr. The filter implementation
    // is the same as for BEMF d-q components filtering
	pllestimatorParam->qVelEstim = (pllestimatorParam->qVelEstim+
						( (pllestimatorParam->qOmegaMr-pllestimatorParam->qVelEstim)*pllestimatorParam->qVelEstimFilterK ));
	
	// The integral of the angle is the estimated angle */
	pllestimatorParam->qRho	= 	pllestimatorParam->qRho+
							(pllestimatorParam->qOmegaMr)*(pllestimatorParam->qDeltaT);
    
    if(pllestimatorParam->qRho >= ANGLE_2PI)
    {
        pllestimatorParam->qRho = pllestimatorParam->qRho - ANGLE_2PI;   
    }

     if(pllestimatorParam->qRho <= 0.0f)
     {
        pllestimatorParam->qRho = pllestimatorParam->qRho + ANGLE_2PI; 
     }
    

   
}





void mcLib_InitPI( mcParam_PIController *pParam)
{
    pParam->qdSum = 0.0f;
    pParam->qOut = 0.0f;
}

void mcLib_CalcPI( mcParam_PIController *pParam)
{
    float Err;
    float U;
    float Exc;
    
    Err  = pParam->qInRef - pParam->qInMeas;
    pParam->qErr =  Err; 
    U  = pParam->qdSum + pParam->qKp * Err;
   
    if( U > pParam->qOutMax )
    {
        pParam->qOut = pParam->qOutMax;
    }    
    else if( U < pParam->qOutMin )
    {
        pParam->qOut = pParam->qOutMin;
    }
    else        
    {
        pParam->qOut = U;  
    }
     
    Exc = U - pParam->qOut;
    pParam->qdSum = pParam->qdSum + pParam->qKi * Err - pParam->qKc * Exc;

}

// *****************************************************************************
// *****************************************************************************
// Section: MC Sine Cosine Functions
// *****************************************************************************
// *****************************************************************************

void mcLib_SinCosGen(mcParam_SinCos *scParam)
{
   
    // Since we are using "float", it is not possible to get an index of array
    // directly. Almost every time, we will need to do interpolation, as per
    // following equation: -
    // y = y0 + (y1 - y0)*((x - x0)/(x1 - x0))
    
    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, y0, y1, temp;
    
    // Software check to ensure  0 <= Angle < 2*PI
    if(scParam->Angle <  0.0f)
    {
        scParam->Angle = scParam->Angle + ANGLE_2PI; 
    }
    if(scParam->Angle >= ANGLE_2PI)
    {
        scParam->Angle = scParam->Angle - ANGLE_2PI; 
    }
    
    y0_Index = (uint32_t)((float)(scParam->Angle/ANGLE_STEP));
    
	//Added this condition which detects if y0_Index is >=256.
    //Earlier the only check was for y0_IndexNext. 
    //We observed y0_Index > = 256 when the code to reverse the direction of the motor was added
    if(y0_Index>=TABLE_SIZE)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
      //  x0 = ANGLE_2PI;
        temp = 0.0f;
    }
    else
    {
        y0_IndexNext = y0_Index + 1U;
        if(y0_IndexNext >= TABLE_SIZE )
        {
            y0_IndexNext = 0;
        }
        else
        {

        }

        x0 = ((float)y0_Index * ANGLE_STEP);  
    
    
    // Since below calculation is same for sin & cosine, we can do it once and reuse
    
	temp = ((scParam->Angle - x0)*ONE_BY_ANGLE_STEP);
    }
    
	// Find Sine now
    y0 = sineTable[y0_Index];
    y1 = sineTable[y0_IndexNext];     
   scParam->Sin = y0 + ((y1 - y0)*temp);
	
    // Find Cosine now
    y0 = cosineTable[y0_Index];
    y1 = cosineTable[y0_IndexNext];
    scParam->Cos = y0 + ((y1 - y0)*temp);
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Space Vector Modulation Routines
// *****************************************************************************
// *****************************************************************************



void mcLib_SVPWMGen(mcParam_AlphaBeta *alphabetaParam, mcParam_SVPWM *svParam)
{

    //Modified inverse clarke transform which allows using instantaneous phase 
    // value to be used directly to calculate vector times.
    svParam->Vr1 = alphabetaParam->beta;
    svParam->Vr2 = (-alphabetaParam->beta/2.0f + SQRT3_BY2 * alphabetaParam->alpha);
    svParam->Vr3 = (-alphabetaParam->beta/2.0f - SQRT3_BY2 * alphabetaParam->alpha);     
    
    if( svParam->Vr1 >= 0.0f )
    {       
		// (xx1)
        if( svParam->Vr2 >= 0.0f )
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
           svParam->T1 = svParam->Vr2;
            svParam->T2 = svParam->Vr1;
            mcLib_CalcTimes(svParam);
            svParam->dPWM_A = svParam->Ta;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Tc;
        }
        else
        {            
            // (x01)
            if( svParam->Vr3 >= 0.0f )
            {
                // Sector 5: (1,0,1)  120-180 degrees
               svParam->T1 = svParam->Vr1;
               svParam->T2 = svParam->Vr3;
                mcLib_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tc;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tb;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                svParam->T1 = -svParam->Vr2;
                svParam->T2 = -svParam->Vr3;
                mcLib_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tc;
            }
        }
    }
    else
    {
        // (xx0)
        if( svParam->Vr2 >= 0.0f )
        {
			// (x10)
            if( svParam->Vr3 >= 0.0f )
            {
                // Sector 6: (1,1,0)  240-300 degrees
                svParam->T1 = svParam->Vr3;
                svParam->T2 = svParam->Vr2;
                mcLib_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Tc;
                svParam->dPWM_C = svParam->Ta;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                svParam->T1 = -svParam->Vr3;
                svParam->T2 = -svParam->Vr1;
                mcLib_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Ta;
                svParam->dPWM_B = svParam->Tc;
                svParam-> dPWM_C = svParam->Tb;
            }
        }
        else
        {            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            svParam->T1 = -svParam->Vr1;
            svParam->T2 = -svParam->Vr2;
            mcLib_CalcTimes(svParam);
            svParam->dPWM_A = svParam->Tc;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Ta;

        }
    }
}

void mcLib_CalcTimes(mcParam_SVPWM *svParam)
{
    svParam->T1 = svParam->PWMPeriod * svParam->T1;
    svParam->T2 = svParam->PWMPeriod * svParam->T2;
    svParam->Tc = (svParam->PWMPeriod- svParam->T1 - svParam->T2)/2.0f;
    svParam->Tb = svParam->Tc + svParam->T2;
    svParam->Ta = svParam->Tb + svParam->T1;    
}  




