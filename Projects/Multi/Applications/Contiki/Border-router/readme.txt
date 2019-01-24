-------------------------------------------------------------------------------
-                    (C) COPYRIGHT 2017 STMicroelectronics                    -
- File:    readme.txt                                                         -
- Author:  ST Central Labs                                                    -
- Date:    15-April-2018                                                      -
- Version: V3.0.0                                                             -

-----------
Border-router/readme.txt
-----------
This sample application shows how to use SPIRIT1/S2-LP radio and Contiki OS to configure a node as "RPL Border Router".

To test this application you need:
- one STM32 Nucleo board with its X-NUCLEO-IDS01A4 or X-NUCLEO-IDS01A5 or (X-NUCLEO-S2868A1 or X-NUCLEO-S2915A1) expansion board
- a host PC running Windows or Linux

On the STM32 Nucleo board this sample application for Border-router must be running. This board must be connected through a mini USB cable to the PC.
Another STM32 Nucleo board with its X-NUCLEO-IDS01A4 or X-NUCLEO-IDS01A5 or (X-NUCLEO-S2868A1 or X-NUCLEO-S2915A1) expansion board(running one of the Udp-sender or Udp-receiver sample applicatioins included in this package) can be used to test the functionality of the Border-router by pinging its ipv6 address from the host PC.

Be sure to select the right X-NUCLEO-IDS01A4 or X-NUCLEO-IDS01A5 or (X-NUCLEO-S2868A1 or X-NUCLEO-S2915A1) expansion board in the code by setting the symbol X-NUCLEO-IDS01A4 for the 868MHz bandwidth, and X-NUCLEO-IDS01A5 for the 915MHz bandwidth

-----------
Hardware and Software environment
-----------
  - This example runs on STM32 Nucleo devices with X-NUCLEO-IDS01A4 or X-NUCLEO-IDS01A5 STM32 expansion board
  - This example has been tested with STMicroelectronics:
    - NUCLEO-L152RE RevC board
    - NUCLEO-F401RE RevC board
    and can be easily tailored to any other supported device and development board.

-----------
How to use it?
-----------    
In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open IAR toolchain (see the release note for detailed information about the version).
   Alternatively you can use the Keil uVision toolchain (see the release note for detailed information about the version).
   Alternatively you can use the System Workbench for STM32 (see the release note for detailed information about the version).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package. 
   
 - IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is   
   suggested to update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version.
   Please refer to the readme.txt file in the Applications directory for details.
   
_______________________________________________________________________________
- (C) COPYRIGHT 2016 STMicroelectronics                   ****END OF FILE**** -
