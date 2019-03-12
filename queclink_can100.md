# eTrans Queclink CAN100
## Introduction
The CAN100 accessory is designed to gather logistic information from vehicles and use it for vehicle monitoring systems. It has a 
RS-232 output which is not dependent from any car or model. It works with CAN and J1708. Some of the decoded variables are:
* Vehicle distance.
* Fuel level.
* Fuel consumption.
* Engine speed.
* Engine temperatures.
* State of doors, locks, indicators on the dashboard.
* Supports driver identification based on digital tachograph.

## Technical specification

|Item|Spec|
|:---|---:|
|Box size|69x49.5x18 mm|
|Supply voltage|7 to 32 Vdc|
|Supply current|11 mA|
|Supply current in sleep|max 1mA|
|Operating temperature|-40 to 80 °C|

The package comes with the CAN100 device, a 4 pin connector cable for power supply and serial port, a 6 pin connector for the can bus (j1939 I believe), a 8 pi connector for the J1708 protocol, and an optional CAN click (contactless CAN conection).
## Pinout
![pinout img](https://github.com/gopimn/gopimn_style/blob/img/eTrans_can100_pinout.png)
The mode of the device is indicated by the LED on top of the case, the button in fron of the panel runs synchronization and awakes the the device.

* CANH(S2-1) and CANL(S2-4) are the lines of the CAN bus.
* J1708 A(S3-4) and B(S3-8) are the CAN lines for J1708.
* SUPPLY POWER(S1-3) and GROUND(S1-4) are the main power supply.
* TX(S1-1) and RX(S1-2) are the RS-232 UART connections. 
* Ignition output(S2-2) reads when the vehicle's ignition is on from the CAN bus. Active level with max 10 mA.
* "Active" output(S2-3), is a negative output that indicates the status of the buses in the vehicle. Tied to GND when data is present, high impedance when the vehicle is in low power mode. Max 100 mA. 
* Output 3(S3-3) is a cofigurable negative output with max 100 mA.
* Inputs 1(S2-5),2(S2-6) and 3(S3-7) can be configures as analog (12 bit) or digital.

## Installation

When you connect the power on, the LED flashes green onece per secod if the information is recived from the bus or once per 4 seconds if the vehicle is on sleep mode or improperly conected.

If the vehicle goes to sleep mode, the devices also goes into low power mode. The LED is turned off when this happens.

If the device recieves information from any of the buses, it returns to normal operation and activate the "active" output (tie to GND). The sleep mode can also be ended pressing the button in the front of the device. Vehicle, and the LED blinks every 4 sec, the device is not properly connected.

**If ignition is turned on in the vehicle, and the LED blinks every 4 sec, the device is not properly connected.**

## Synchronization

The CAN buses can vary from specific models and manufacturers signifiantly. The CAN100 can recognize each type of CAN-bus and adjust it automatically.

If an unconfigured CAN100 is connected for the first time it synchronizes after the power is supplied, make shure the ignition is connected.

If you want to reconfigure the device:
1. Connect the power of the device, LED lights red.
2. Press the button on the front ogthe panel (hold it while connecting the power supply)
3. After a few seconds, the LED will light green, then release the button. After starting the device, the sync LED blinks red, After several seconds the syncronization is done and:
     * If the green LED lights, the vehicle is succesfully syncronized. Turn the power supply off and on after 5 secpnds.
     * If the LED flashes alternating green/red it means an invalid connection to the bus. make shure the CAN wires are not swapped and the ignition is turned on. If these conditions are met, then the devices is not connected to any bus.
     * If the red LED ligts, the CAN bus connectios is correct but the vehicle is not supported.

The CAN bus synchronization may also be done trough the serial port. 

### Firmware Upgrade

Before installing make shure that the latest firmware is installed on the device. CAN100 firmware can be updated trough the serial port.

## GV300 example

The GV300 can communicate with CAN100 by RS-232 protocol. This type of CAN100 is named CAN100_STD (for the GV65 the device is called CAN100_INV). The following table shows the connections:
<table>
   <tr>
    <td colspan="2" align="center">GV300</td>
    <td></td>
    <td colspan="2" align="center">CAN100_STD</td>
  </tr>
  <tr>
    <td>Pin No.</td>
    <td>Pin Name</td>
    <td align="center">Connection</td>
    <td>Pin Name</td>
    <td>Pin No.</td>
  </tr>
  <tr>
    <td>4</td>
    <td>RXD</td>
    <td align="center"><----></td>
    <td align="right">TX</td>
    <td align="right">S1-1</td>
  </tr>
  <tr>
    <td>5</td>
    <td>TXD</td>
    <td align="center"><----></td>
    <td align="right">RX</td>
    <td align="right">S1-2</td>
  </tr>
  <tr>
    <td>11</td>
    <td>Power</td>
    <td align="center"><----></td>
    <td align="right">Power supply</td>
    <td align="right">S1-3</td>
  </tr>
  <tr>
    <td>6</td>
    <td>Ground</td>
    <td align="center"><----></td>
    <td align="right">Ground</td>
    <td align="right">S1-4</td>
  </tr>
</table>

Remember that this way you are powering the CAN100 from the GV300. If you are powering the device from other source, don't connect the power line.
## References
The main reference of this document is [CAN100 User Guide V1.01](https://drive.google.com/open?id=1tS-P5NAi1Ux6r_UjPbvylLiTXeWuQ86A).

For a detailed way to synchronize and verify the device, check the [CAN100 Synchro and Verifying V2.1](https://drive.google.com/open?id=1VqohfdTpn7xUJhko-kuwn3CuAHL6UF-b) docummentation.

[CAN100 supported car models V2.7](https://drive.google.com/open?id=1V3fXr-EIT2Gz8c4_OnbVT3Jj3m5fjY98) has the vehicle models available and their parameters.

[CAN100 supported machines 2015-03-20](https://drive.google.com/open?id=1wKen_dIr94m4pK54fqXjam4E7VBp2CAn) has the same, but for machinery.


