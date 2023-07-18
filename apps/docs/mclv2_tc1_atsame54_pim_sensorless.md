---
parent: Hardware Setup
title: MCLV2 Development Board Setup for Sensorless Mode
has_children: false
has_toc: false
---

# MCLV2 Development Board
## Setting up the hardware

The following table shows the target hardware for the application projects.

| Project Name| Hardware |
|:---------|:---------:|
| mclv2_sam_e54_pim.X |<br>[MCLV2 Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2)<br>[ATSAME54 Plug-in module](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/MA320207)<br>[EBM-PAPST ECI 6320-K1-B00 motor](https://ebmpapst.se/sv/dat/media_manager/news/696/news-files/Ext_38717.pdf) |

### Modify the [MCLV2 Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2)
- Modify the MCLV2 development board for higher current capability- TC1 modifications. Refer the [MCLV2 Development Board modification guide](http://ww1.microchip.com/downloads/en/DeviceDoc/motorBench-mclv2-modifications-guidelines.2.285.pdf) for more details.
  

### Setting up [TC1 modified MCLV2 Development Board](https://www.microchip.com/DevelopmentTools/ProductDetails/DM330021-2)

- Mount the ATSAME54 Motor Control Plug In Module on U9 header. 

    ![PIM Install](images/mclv2/same54_mclv2_pim.jpg)

- Place the "External Opamp Configuration" Matrix board at J14

    ![External OPAMP](images/mclv2/same54_mclv2_ext_opamp.jpg)

- Motor Connections: 
    - Phase U - M1 
    - Phase V - M2 
    - Phase W - M3

    ![Motor Connections](images/mclv2/pll_same70_mclv2_motor_connection.png)

- Jumper Settings: 
    - JP1 - Curr, JP2 - Curr, JP3 - Curr 
    - In order to use RS232 port for X2CScope Communication JP4 - UART, JP5 - UART 
    - In order to use USB port for X2CScope Communication JP4 - USB, JP5 - USB

    ![jumper Settings](images/mclv2/pll_same70_mclv2_jumper_settings.png)


- Power the board with a 24V DC supply using J2 or BP1-BP2. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. 

    ![jumper Settings](images/mclv2/pll_same70_power_mclv2.png)

- Complete Setup

    ![Setup](images/mclv2/tc1_board.jpg)

## Running the Application

1. Build and Program the application with ````src/firmware.hex````using Microchip's IPE.
2. Press switch S2 to start the motor
3. Vary potentiometer to change the speed of the motor
4. Press switch S2 to stop the motor
5. Press switch S2 again to start the motor
6. Monitor graphs on X2C Scope

Refer to the following tables for switch and LED details:

| Switch | Description |
|------|----------------|
| Switch S2 | To start or stop the motor |


| LED D17 Status | Description |
|------|----------------|
| OFF  | No fault  |
| ON   | Fault is detected  |