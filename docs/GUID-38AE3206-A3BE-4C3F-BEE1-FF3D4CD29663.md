# MCHV3 Development Board - Setup and Running 'Position Control of PMSM using Quadrature Encoder Application'
**Setting up the hardware**

The following table shows the target hardware for the application projects.

| Project Name| Hardware |
|:---------|:---------:|
| mchv3_sam_e54_pim.X |<br>[MCHV3 Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3)<br>[ATSAME54 Plug-in module](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/MA320207)<br>[Leadshine EL5-M0400-1-24 Motor](https://www.microchip.com/developmenttools/ProductDetails/AC300025) <br>[Isolated Embedded Debugger Interface](https://www.microchip.com/DevelopmentTools/ProductDetails/AC320202) |
|||

**Setting up [MCHV3 Development Board](https://www.microchip.com/developmenttools/ProductDetails/dm330023-3)**

- Mount the ATSAME54 Motor Control Plug In Module on U9 header. 

    ![PIM Install](GUID-EF24FB48-4A06-4B92-AB17-3EC80AA27275-low.jpg)

- Place the "PFC - External Opamp Configuration" Matrix board at J4.

    ![External OPAMP](GUID-AD0DF98E-86F7-43D3-9B3F-355D31893645-low.jpg)

- Motor Connections: 
    - Phase U - M1 
    - Phase V - M2 
    - Phase W - M3

    ![Motor Connections](GUID-7E3BFBA0-F563-4326-89D5-3FA74B288F39-low.png)

- Encoder Connections:
    - A+ - HA
    - B+ - HB
    - +5V - +5V
    - GND - G

    ![Encoder Connections](GUID-E0ECA475-FCFC-400D-9B8F-824220158553-low.png)

- Jumper Settings: 
    - J11 - VAC ( Short Pin 3 - 4)
    - J12 - IA ( Short Pin 1 - 2)
    - J13 - IB ( Short Pin 1 - 2)
    - J14 - Fault_IP/IBUS ( Short Pin 1 - 2)


- Power the board with (110V/220V) AC mains. For additional safety, it is recommended to use a current limited power supply while testing this software demonstration on a non-default hardware and motor. 

    ![jumper Settings](GUID-94349583-0C3E-42D2-ABE3-1F0C9AD1C98F-low.png)

- Installing Isolated Embedded Debugger
Default programmer or debugger daughter card shipped with the MCHV3 board cannot program or debug SAM series MCU and therefore, it needs to be replaced with an Isolated Embedded Debugger Interface for MCHV.

    ![Isolated EDBG](GUID-61ECD694-5820-410A-8D03-FAC567F64BC5-low.png)

- Complete Setup

    ![Setup](GUID-F636A3BA-9FEB-46B9-8FF3-5A092A99F26C-low.jpg)

**Running the Application**

1. Vary the POT (position reference) before PUSHBUTTON press for 'motor start'
2. Press PUSHBUTTON for 'motor start'. Observe the position control response. Then press PUSHBUTTON for 'motor stop'
3. Vary the POT again for different position reference
4. Press PUSHBUTTON for 'motor start'. Observe the position control response. Then press PUSHBUTTON for 'motor stop'
5. Continue the steps above for different reference positions

Refer to the following tables for switch and LED details:

| Switch | Description |
|------|----------------|
| PUSHBUTTON | To start or stop the motor |
||

| LED D2 Status | Description |
|------|----------------|
| OFF  | No fault  |
| ON   | Fault is detected  |
||