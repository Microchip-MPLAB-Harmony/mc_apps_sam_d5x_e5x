/* ************************************************************************** */
/** Data manager header file

  @Company
 Microchip Technology Pvt. Ltd

  @File Name
    data_manager.h

  @Summary
  Data management APIs

  @Description
    Data management APIs
 */
/* ************************************************************************** */

#ifndef _DAM_H    /* Guard against multiple inclusion */
#define _DAM_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "stddef.h"
#include "definitions.h"


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */



    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************
    
     // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************
    extern int16_t mcBseI_IaAdcInput_gds16;
    extern int16_t mcBseI_IbAdcInput_gds16;
    extern int16_t mcBseI_UdcAdcInput_gds16;
    extern int16_t mcBseI_UpotAdcInput_gds16;

    extern float mcCurI_Ia_gdf32;
    extern float mcCurI_Ib_gdf32;
    extern float mcCurI_Ic_gdf32;
    
    extern float mcVolI_Udc_gdf32;
    extern float mcVolI_UdcFilt_gdf32;
    extern float mcVolI_UacPeak_gdf32;
    
    extern float mcSpeI_CommandSpeed_gdf32;
    extern float mcSpeI_CommandSpeedFilt_gdf32;
    
    extern float mcRpoI_ElectricalSpeed_gdf32;
    extern float mcRpoI_ElectricalAngle_gdf32;
    extern float mcPllI_ElectricalSpeed_gdf32;
    extern float mcPllI_ElectricalAngle_gdf32;

    extern uint32_t mcFcoI_HallToAdcISRTimerCount_gdu32;
    extern uint32_t mcFcoI_HallToHallISRTimerCount_gdu32;
    extern uint32_t mcFcoI_HallISRTimerCount_gdu32;
    
    extern float mcMocI_AlphaAxisVoltage_gdf32;
    extern float mcMocI_BetaAxisVoltage_gdf32;
    extern float mcMocI_AlphaAxisCurrent_gdf32;
    extern float mcMocI_BetaAxisCurrent_gdf32;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************



    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
