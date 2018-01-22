# RAK813-BreakBoard <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/substance.jpg" width=30% height=30% />

### chip: RAK813(nRF52832+SX1276) + GPS + HT + MEMS
#### Support IAR8.11/Keil5
#### Bace on nRF5_SDK_14.0.0 and semtech LoRaWAN1.0.2
The document you can download in the RAK Document Center:
http://www.rakwireless.com/en/download/RAK813%20BreakBoard/Software%20Development

---
## Development environment
#### 1. Install nRFgo Stdio
  * Navigate to [Nordic official website](http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832) or [RAK Data Center](http://www.rakwireless.com/en/download/RAK813%20BreakBoard/Tools)
  * Download and install the tool <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/nRFgo%20Stdio%20start.png)
  
#### 2. Install j-link driver
  * Navigate to https://www.segger.com/downloads/jlink
  * Click “Click for downloads” under “J-Link Software and Documentation Pack” <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/download%20j-link%20driver.png)
  * Download the appropriate package for your OS
  * Accept the License Agreement
  * Run the installation program with default configurations

#### 3.  Download Bluetooth protocol station
  * Connect the j-link and RAK813 BreakBoard <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/j-link%20Connect.png)
  * Open the nRFgo Stdio, Select the "nRF5X Programming" <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/nRFgo%20Stdio%20programming.png)
  * "Erase all", and Select the "Program SoftDevice" <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/erase%20all.png)
  * Select the `../nRF_Lib/components/softdevice/s132/hex/s132_nrf52_5.0.0_softdevice.hex` file,and then "Program",Waiting for a while will prompt programming success <br />
![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/download%20firmware.png)

#### 4. Program with Keil5
  * Navigate to http://www.keil.com/
  * Download and install the Keil5
  * Download the `../Keil5/NordicSemiconductor.nRF_DeviceFamilyPack.8.14.1.pack` ,Installed nRF52832 compiler environment for Keil5
  * Use Keil5 to Open the `../Keil5/rak813_breakboard.uvprojx` file,Now you can start writing your own program
  * Click on the top left corner ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil%20buide.png) to "Buide",Click on the top left corner ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil%20download.png) to "Download" <br />
  ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil.png)

