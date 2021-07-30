---
parent: Motor Control Application Examples for SAM D5x/E5x family
title: PMSM FOC using Hall Sensor
has_children: false
has_toc: false
---

[![MCHP](https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png)](https://www.microchip.com)

# PMSM FOC Using Hall Sensor

This motor control example project shows how to control the Permanent Magnet Synchronous Motor (PMSM) using hall sensor  based Field Oriented Control (FOC) on SAME54 Micro-controller.

## Description
The permanent magnet synchronous motors ( PMSM ) are widely used in various industries due to its high power density, smaller size and high efficiency. The Field oriented control is one of the most popular control mechanisms for the PMSM motor for applications which requires high dynamic performance.

This example shows how to configure motor control peripherals ADC, PDEC and TCC for the control operation. The control strategy is the sensored FOC, in which rotor position is determined by the Hall Sensor. Position is obtained using hall sensor and speed is calculated from the position.

Waveforms and variables can be monitored at runtime using X2CScope. 

Key features enabled in this project are:

- Dual shunt current measurement
- Speed control loop
- Torque control loop
- Field Weakening


## MHC Project Configurations

![MHC Project Graph](images/hall_foc_same54_project_graph.jpg)


- **ADC0-ADC1**: 

   ADC0 and ADC1 are setup to operate in Master - Slave mode with ADC0 acting as a Master

   Both ADCs convert single ended inputs. Phase U current is sampled and converted by ADC0 and Phase V current is sampled and converted by ADC1

   Both ADCs are hardware triggered simultaneously by an event generated from TCC0 at the end of each PWM cycle

   Conversion Ready interrupt is generated by ADC0. Since both ADCs are triggered simultaneously and have the same resolution and sampling time, both ADCs complete conversion at the same time

- **TCC0**: 

    This peripheral is used to generated three phase synchronous PWM waveforms. Fault functionality is also enabled to switch off the output waveforms asynchronously.

- **PDEC Peripheral**:

    It is used to decode the rotor position and speed from hall sensor signals. PDEC is configured in Hall mode.

- **TC2 - TC3**:

    This peripheral is configured in 32-bit time-stamp capture mode, to capture the time between two hall edges.
    Capture happens on PDEC velocity event input.

- **EIC**:

    External Interrupt Controller detects a hardware over-current fault input and generates a non-recoverable fault event for TCC0, thereby shutting down the PWM in the event of an over-current fault

- **EVSYS**:

    Event System acts as an intermediary between event generator and event users

    Event generated by the TCC0 when the counter reaches TOP, is used by the ADC0 as a hardware trigger source via the Event System

    Event generated by the EIC upon over-current fault, is used by the TCC0 as a non-recoverable fault event via Event System

    Event generated by the PDEC when hall input changes, is used by the TC2 as a hardware event to capture the counter value via Event System

- **SERCOM2**:

    SERCOM2 is configured in USART mode and is set to operate at 115200 bps

    This USART channel is used by the X2CScope plugin to plot or watch global variables in run-time. Refer to X2C Scope Plugin section for more details on how to install and use the X2CScope

## Control Algorithm

This section briefly explains the FOC control algorithm, software design and implementation. 

Field Oriented Control is the technique used to achieve the decoupled control of torque and flux. This is done by transforming the stator current quantities (phase currents) from stationary reference frame to torque and flux producing currents components in rotating reference frame using mathematical transformations. The Field Oriented Control is done as follows: 

1. Measure the motor phase currents. 
2. Transform them into the two phase system (a, b) using the Clarke transformation. 
3. Calculate the rotor position angle. 
4. Transform stator currents into the d,q-coordinate system using the Park transformation. 
5. Speed is controlled by speed PI controllers.
6. The stator current torque (iq) and flux (id) producing components are controlled separately by the corresponding PI controllers. 
7. The output stator voltage space vector is transformed back from the d,q-coordinate system into the two phase system fixed with the stator by the Inverse Park transformation. 
8. Using the space vector modulation, the three-phase output voltage is generated. 

**Hall Sensor based Position and Speed Measurement** :

Rotor position and speed are determined using hall sensor. Three hall sensors mounted on the motor 120 degress apart. PDEC captures the pin status of the three hall sensor inputs and generates interrupt upon change in any of the hall sensor input. Hall sensor pattern determines the position of the rotor. Hall sensors give position at 60 degree resolution. This position is interpolated to get the absolute position required for FOC.
Speed is calculated by measuring the time between two hall edges by TC peripheral. TC captures the counter value on PDEC velocity event. 

The following block diagram shows the software realization of the FOC algorithm.

![block_diagram](images/hall_foc_same54_block_diagram.jpg)

## Software Design

In the software, the PMSM FOC task is run in ADC interrupt. Hall interrupt calculates angle from hall pattern and saves the time between two hall events. 

### INITIALIZE
In this state, following tasks are performed:
1. Initialization and configuration of motor control peripherals for generation of periodic ADC triggers and ADC conversion interrupt
2. Current Offset measurement and calibration
3. Initialize PI controller parameters for speed and current control loops
4. Enables ADC interrupt and Hall interrupt

### START
Control waits for the switch press.

### Run
In this state, the motor starts spinning in closed loop. No field alignment is used for hall based FOC. The below flow chart and the timing diagram shows the tasks performed in run state:

![state_machine](images/hall_foc_same54_control_flow.jpg)

![hall_isr](images/hall_same54_hall_isr.jpg)

Hall interrupt has higher priority than ADC interrupt. 

## Development Kits

### MCLV2 with ATSAME54 PIM
#### Downloading and building the application

To clone or download this application from Github, go to the [main page of this repository](https://github.com/Microchip-MPLAB-Harmony/mc_apps_sam_d5x_e5x) and then click **Clone** button to clone this repository or download as zip file.
This content can also be downloaded using content manager by following these [instructions](https://github.com/Microchip-MPLAB-Harmony/contentmanager/wiki).

Path of the application within the repository is **apps/pmsm_foc_hall_sam_e54** .

To build the application, refer to the following table and open the project using its IDE.

| Project Name      | Description                                    | Demo User Guide |
| ----------------- | ---------------------------------------------- | --------------- |
| mclv2_sam_e54_pim.X | MPLABX project for MCLV2 board with ATSAME54 PIM | [Hardware Setup and Running The Application on MCLV2 with ATSAME54 PIM](../docs/mclv2_atsame54_pim_hall.md) |
||||


