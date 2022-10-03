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

/*******************************************************************************
 Header inclusions
 *******************************************************************************/
#include "mc_generic_libraries.h"

/*******************************************************************************
 Constants
 *******************************************************************************/
const float sineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Sine Table">
{
    0,
    0.024541,
    0.049068,
    0.073565,
    0.098017,
    0.122411,
    0.14673,
    0.170962,
    0.19509,
    0.219101,
    0.24298,
    0.266713,
    0.290285,
    0.313682,
    0.33689,
    0.359895,
    0.382683,
    0.405241,
    0.427555,
    0.449611,
    0.471397,
    0.492898,
    0.514103,
    0.534998,
    0.55557,
    0.575808,
    0.595699,
    0.615232,
    0.634393,
    0.653173,
    0.671559,
    0.689541,
    0.707107,
    0.724247,
    0.740951,
    0.757209,
    0.77301,
    0.788346,
    0.803208,
    0.817585,
    0.83147,
    0.844854,
    0.857729,
    0.870087,
    0.881921,
    0.893224,
    0.903989,
    0.91421,
    0.92388,
    0.932993,
    0.941544,
    0.949528,
    0.95694,
    0.963776,
    0.970031,
    0.975702,
    0.980785,
    0.985278,
    0.989177,
    0.99248,
    0.995185,
    0.99729,
    0.998795,
    0.999699,
    1,
    0.999699,
    0.998795,
    0.99729,
    0.995185,
    0.99248,
    0.989177,
    0.985278,
    0.980785,
    0.975702,
    0.970031,
    0.963776,
    0.95694,
    0.949528,
    0.941544,
    0.932993,
    0.92388,
    0.91421,
    0.903989,
    0.893224,
    0.881921,
    0.870087,
    0.857729,
    0.844854,
    0.83147,
    0.817585,
    0.803208,
    0.788346,
    0.77301,
    0.757209,
    0.740951,
    0.724247,
    0.707107,
    0.689541,
    0.671559,
    0.653173,
    0.634393,
    0.615232,
    0.595699,
    0.575808,
    0.55557,
    0.534998,
    0.514103,
    0.492898,
    0.471397,
    0.449611,
    0.427555,
    0.405241,
    0.382683,
    0.359895,
    0.33689,
    0.313682,
    0.290285,
    0.266713,
    0.24298,
    0.219101,
    0.19509,
    0.170962,
    0.14673,
    0.122411,
    0.098017,
    0.073565,
    0.049068,
    0.024541,
    0,
    -0.024541,
    -0.049068,
    -0.073565,
    -0.098017,
    -0.122411,
    -0.14673,
    -0.170962,
    -0.19509,
    -0.219101,
    -0.24298,
    -0.266713,
    -0.290285,
    -0.313682,
    -0.33689,
    -0.359895,
    -0.382683,
    -0.405241,
    -0.427555,
    -0.449611,
    -0.471397,
    -0.492898,
    -0.514103,
    -0.534998,
    -0.55557,
    -0.575808,
    -0.595699,
    -0.615232,
    -0.634393,
    -0.653173,
    -0.671559,
    -0.689541,
    -0.707107,
    -0.724247,
    -0.740951,
    -0.757209,
    -0.77301,
    -0.788346,
    -0.803208,
    -0.817585,
    -0.83147,
    -0.844854,
    -0.857729,
    -0.870087,
    -0.881921,
    -0.893224,
    -0.903989,
    -0.91421,
    -0.92388,
    -0.932993,
    -0.941544,
    -0.949528,
    -0.95694,
    -0.963776,
    -0.970031,
    -0.975702,
    -0.980785,
    -0.985278,
    -0.989177,
    -0.99248,
    -0.995185,
    -0.99729,
    -0.998795,
    -0.999699,
    -1,
    -0.999699,
    -0.998795,
    -0.99729,
    -0.995185,
    -0.99248,
    -0.989177,
    -0.985278,
    -0.980785,
    -0.975702,
    -0.970031,
    -0.963776,
    -0.95694,
    -0.949528,
    -0.941544,
    -0.932993,
    -0.92388,
    -0.91421,
    -0.903989,
    -0.893224,
    -0.881921,
    -0.870087,
    -0.857729,
    -0.844854,
    -0.83147,
    -0.817585,
    -0.803208,
    -0.788346,
    -0.77301,
    -0.757209,
    -0.740951,
    -0.724247,
    -0.707107,
    -0.689541,
    -0.671559,
    -0.653173,
    -0.634393,
    -0.615232,
    -0.595699,
    -0.575808,
    -0.55557,
    -0.534998,
    -0.514103,
    -0.492898,
    -0.471397,
    -0.449611,
    -0.427555,
    -0.405241,
    -0.382683,
    -0.359895,
    -0.33689,
    -0.313682,
    -0.290285,
    -0.266713,
    -0.24298,
    -0.219101,
    -0.19509,
    -0.170962,
    -0.14673,
    -0.122411,
    -0.098017,
    -0.073565,
    -0.049068,
    -0.024541
};

// </editor-fold>
const float cosineTable[TABLE_SIZE] = 
// <editor-fold defaultstate="collapsed" desc="Cosine Table">
    {
    1,
    0.999699,
    0.998795,
    0.99729,
    0.995185,
    0.99248,
    0.989177,
    0.985278,
    0.980785,
    0.975702,
    0.970031,
    0.963776,
    0.95694,
    0.949528,
    0.941544,
    0.932993,
    0.92388,
    0.91421,
    0.903989,
    0.893224,
    0.881921,
    0.870087,
    0.857729,
    0.844854,
    0.83147,
    0.817585,
    0.803208,
    0.788346,
    0.77301,
    0.757209,
    0.740951,
    0.724247,
    0.707107,
    0.689541,
    0.671559,
    0.653173,
    0.634393,
    0.615232,
    0.595699,
    0.575808,
    0.55557,
    0.534998,
    0.514103,
    0.492898,
    0.471397,
    0.449611,
    0.427555,
    0.405241,
    0.382683,
    0.359895,
    0.33689,
    0.313682,
    0.290285,
    0.266713,
    0.24298,
    0.219101,
    0.19509,
    0.170962,
    0.14673,
    0.122411,
    0.098017,
    0.073565,
    0.049068,
    0.024541,
    0,
    -0.024541,
    -0.049068,
    -0.073565,
    -0.098017,
    -0.122411,
    -0.14673,
    -0.170962,
    -0.19509,
    -0.219101,
    -0.24298,
    -0.266713,
    -0.290285,
    -0.313682,
    -0.33689,
    -0.359895,
    -0.382683,
    -0.405241,
    -0.427555,
    -0.449611,
    -0.471397,
    -0.492898,
    -0.514103,
    -0.534998,
    -0.55557,
    -0.575808,
    -0.595699,
    -0.615232,
    -0.634393,
    -0.653173,
    -0.671559,
    -0.689541,
    -0.707107,
    -0.724247,
    -0.740951,
    -0.757209,
    -0.77301,
    -0.788346,
    -0.803208,
    -0.817585,
    -0.83147,
    -0.844854,
    -0.857729,
    -0.870087,
    -0.881921,
    -0.893224,
    -0.903989,
    -0.91421,
    -0.92388,
    -0.932993,
    -0.941544,
    -0.949528,
    -0.95694,
    -0.963776,
    -0.970031,
    -0.975702,
    -0.980785,
    -0.985278,
    -0.989177,
    -0.99248,
    -0.995185,
    -0.99729,
    -0.998795,
    -0.999699,
    -1,
    -0.999699,
    -0.998795,
    -0.99729,
    -0.995185,
    -0.99248,
    -0.989177,
    -0.985278,
    -0.980785,
    -0.975702,
    -0.970031,
    -0.963776,
    -0.95694,
    -0.949528,
    -0.941544,
    -0.932993,
    -0.92388,
    -0.91421,
    -0.903989,
    -0.893224,
    -0.881921,
    -0.870087,
    -0.857729,
    -0.844854,
    -0.83147,
    -0.817585,
    -0.803208,
    -0.788346,
    -0.77301,
    -0.757209,
    -0.740951,
    -0.724247,
    -0.707107,
    -0.689541,
    -0.671559,
    -0.653173,
    -0.634393,
    -0.615232,
    -0.595699,
    -0.575808,
    -0.55557,
    -0.534998,
    -0.514103,
    -0.492898,
    -0.471397,
    -0.449611,
    -0.427555,
    -0.405241,
    -0.382683,
    -0.359895,
    -0.33689,
    -0.313682,
    -0.290285,
    -0.266713,
    -0.24298,
    -0.219101,
    -0.19509,
    -0.170962,
    -0.14673,
    -0.122411,
    -0.098017,
    -0.073565,
    -0.049068,
    -0.024541,
    0,
    0.024541,
    0.049068,
    0.073565,
    0.098017,
    0.122411,
    0.14673,
    0.170962,
    0.19509,
    0.219101,
    0.24298,
    0.266713,
    0.290285,
    0.313682,
    0.33689,
    0.359895,
    0.382683,
    0.405241,
    0.427555,
    0.449611,
    0.471397,
    0.492898,
    0.514103,
    0.534998,
    0.55557,
    0.575808,
    0.595699,
    0.615232,
    0.634393,
    0.653173,
    0.671559,
    0.689541,
    0.707107,
    0.724247,
    0.740951,
    0.757209,
    0.77301,
    0.788346,
    0.803208,
    0.817585,
    0.83147,
    0.844854,
    0.857729,
    0.870087,
    0.881921,
    0.893224,
    0.903989,
    0.91421,
    0.92388,
    0.932993,
    0.941544,
    0.949528,
    0.95694,
    0.963776,
    0.970031,
    0.975702,
    0.980785,
    0.985278,
    0.989177,
    0.99248,
    0.995185,
    0.99729,
    0.998795,
    0.999699
};
// </editor-fold>


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
void mcLib_ClarkeTransform(tmcMocI_ABC_s *abcParam, tmcMocI_AB_s *alphabetaParam)
{
    alphabetaParam->alpha = abcParam->a;
    alphabetaParam->beta = (abcParam->a * ONE_BY_SQRT3) + (abcParam->b * 2 * ONE_BY_SQRT3);
}

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
void mcLib_ParkTransform(tmcMocI_AB_s *alphabetaParam , tmcMocI_PHASOR_s *scParam, tmcMocI_DQ_s *dqParam)
{
    dqParam->d =  alphabetaParam->alpha*scParam->Cos + alphabetaParam->beta*scParam->Sin;
    dqParam->q = -alphabetaParam->alpha*scParam->Sin + alphabetaParam->beta*scParam->Cos;
}

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
void mcLib_InvParkTransform(tmcMocI_DQ_s *dqParam, tmcMocI_PHASOR_s *scParam,tmcMocI_AB_s *alphabetaParam)
{
    alphabetaParam->alpha =  dqParam->d*scParam->Cos - dqParam->q*scParam->Sin;
    alphabetaParam->beta  =  dqParam->d*scParam->Sin + dqParam->q*scParam->Cos;       
}

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
void mcLib_InitPI( tmcMocI_PI_s *pParam)
{
    pParam->reference = 0.0f;
    pParam->feedback = 0.0f;
    pParam->Yi = 0;
    pParam->Yout = 0;
}

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
void mcLib_PiControlRun( tmcMocI_PI_s *pParam)
{
    float Err;
    float U;
    float Exc;
    
    Err  = pParam->reference - pParam->feedback;
    pParam->error =  Err; 
    U  = pParam->Yi + pParam->Kp * Err;
   
    if( U > pParam->Ymax )
    {
        pParam->Yout = pParam->Ymax;
    }    
    else if( U < pParam->Ymin )
    {
        pParam->Yout = pParam->Ymin;
    }
    else        
    {
        pParam->Yout = U;  
    }
     
    Exc = U - pParam->Yout;
    pParam->Yi = pParam->Yi + pParam->Ki * Err - pParam->Kc * Exc;

}

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
void mcLib_SinCosGen(tmcMocI_PHASOR_s *scParam)
{   
    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, y0, y1, temp;
    
    // Software check to ensure  0 <= Angle < 2*PI
     if(scParam->angle <  0) 
        scParam->angle = scParam->angle + ANGLE_2PI; 
    if(scParam->angle >= ANGLE_2PI)
        scParam->angle = scParam->angle - ANGLE_2PI; 
    
    y0_Index = (uint32_t)(scParam->angle/ANGLE_STEP);
    
    if(y0_Index>=TABLE_SIZE)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
        x0 = ANGLE_2PI;
        temp = 0;
    }
    else
    {
        y0_IndexNext = y0_Index + 1;
        if(y0_IndexNext >= TABLE_SIZE )
        {
            y0_IndexNext = 0;
        }
        else
        {

        }

        x0 = (y0_Index * ANGLE_STEP);  
    
        
	temp = ((scParam->angle - x0)*ONE_BY_ANGLE_STEP);
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

