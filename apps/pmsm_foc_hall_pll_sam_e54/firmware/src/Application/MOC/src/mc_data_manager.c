/* ************************************************************************** */
/** Data Manager

  @Company
 Microchip Technology Pvt. Ltd

  @File Name
    data_manager.c

  @Summary
 This file contains data and APIs for data management

  @Description
    This file contains data and APIs for data management
 */
/* ************************************************************************** */

/*******************************************************************************
Headers inclusions
*******************************************************************************/
#include "mc_data_manager.h"


/*******************************************************************************
 * Interface variables 
*******************************************************************************/
int16_t mcBseI_IaAdcInput_gds16;
int16_t mcBseI_IbAdcInput_gds16;
int16_t mcBseI_UdcAdcInput_gds16;
int16_t mcBseI_UpotAdcInput_gds16;

float mcCurI_Ia_gdf32;
float mcCurI_Ib_gdf32;
float mcCurI_Ic_gdf32;

float mcVolI_Udc_gdf32;
float mcVolI_UdcFilt_gdf32;
float mcVolI_UacPeak_gdf32;

float mcSpeI_CommandSpeed_gdf32;
float mcSpeI_CommandSpeedFilt_gdf32;

float mcRpoI_ElectricalSpeed_gdf32;
float mcRpoI_ElectricalAngle_gdf32;
float mcPllI_ElectricalSpeed_gdf32;
float mcPllI_ElectricalAngle_gdf32;


uint32_t mcFcoI_HallToAdcISRTimerCount_gdu32;
uint32_t mcFcoI_HallToHallISRTimerCount_gdu32;
uint32_t mcFcoI_HallISRTimerCount_gdu32;

float mcMocI_AlphaAxisVoltage_gdf32;
float mcMocI_BetaAxisVoltage_gdf32;
float mcMocI_AlphaAxisCurrent_gdf32;
float mcMocI_BetaAxisCurrent_gdf32;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */


/* *****************************************************************************
 End of File
 */
