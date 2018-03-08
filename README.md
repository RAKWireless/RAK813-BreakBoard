# RAK813-BreakBoard <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/substance.jpg" width=50% height=50% /> <br />
### chip: RAK813(nRF52832+SX1276) + GPS + HT + MEMS
#### Support IAR8.11/Keil5
#### Bace on nRF5_SDK_14.2.0 and semtech LoRaWAN1.0.2
The document you can download in the RAK Document Center:
http://www.rakwireless.com/en/download/RAK813%20BreakBoard/Software%20Development

---
## Development environment
#### 1. Install nRFgo Stdio
  * Navigate to [Nordic official website](http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832) or [RAK Data Center](http://www.rakwireless.com/en/download/RAK813%20BreakBoard/Tools)
  * Download and install the tool <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/nRFgo%20Stdio%20start.png" width=80% height=80% /> <br />
  
#### 2. Install j-link driver
  * Navigate to https://www.segger.com/downloads/jlink
  * Click “Click for downloads” under “J-Link Software and Documentation Pack” <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/download%20j-link%20driver.png" width=80% height=80% />  <br />
  * Download the appropriate package for your OS
  * Accept the License Agreement
  * Run the installation program with default configurations

#### 3.  Download Bluetooth protocol station
  * Connect the j-link and RAK813 BreakBoard <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/j-link%20Connect.png" width=80% height=80% />  <br />
  * Open the nRFgo Stdio, Select the "nRF5X Programming" <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/nRFgo%20Stdio%20programming.png" width=80% height=80% />  <br />
  * "Erase all", and Select the "Program SoftDevice" <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/erase%20all.png" width=80% height=80% />  <br />
  * Select the `../nRF_Lib/components/softdevice/s132/hex/s132_nrf52_5.0.0_softdevice.hex` file,and then "Program",Waiting for a while will prompt programming success <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/download%20firmware.png" width=80% height=80% />  <br />

#### 4. Program with Keil5
  * Navigate to http://www.keil.com/
  * Download and install the Keil5
  * Download the `../Keil5/NordicSemiconductor.nRF_DeviceFamilyPack.8.14.1.pack` ,Installed nRF52832 compiler environment for Keil5
  * Use Keil5 to Open the `../Keil5/rak813_breakboard.uvprojx` file,Now you can start writing your own program
  * Click on the top left corner ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil%20buide.png) to "Buide",Click on the top left corner ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil%20download.png) to "Download" <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/keil.png" width=80% height=80% />  <br />
  
#### 5. Program with IAR 
  * Navigate to https://www.iar.com/
  * Download and install the IAR (Note:the best version is 8.11)
  * Use IAR to Open the `../IAR8.11/rak813_breakboard.eww` file,Now you can start writing your own program, Click on the above ![](https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/IAR%20make.png) to "Make"  <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/IAR%20program.png" width=80% height=80% />  <br />
  * Click "Project"->"Download"->"Download Activities Application" to download  <br />
<img src="https://raw.githubusercontent.com/RAKWireless/RAK813-BreakBoard/master/Doc/img/IAR%20download.png" width=80% height=80% />  <br />

