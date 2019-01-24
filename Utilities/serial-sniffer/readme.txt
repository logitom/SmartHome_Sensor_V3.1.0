-------------------------------------------------------------------------------
-                    (C) COPYRIGHT 2018 STMicroelectronics                    -
- File:    readme.txt                                                         -
- Author:  ST Central Labs                                                    -
- Date:    01-June-2018                                                       -
- Version: V3.0.0                                                             -


This is a simple sniffer. The sniffer runs a device that is connected
to a local USB/serial port and prints out sniffed packets as
pcap-compatible data that can be fed directly into Wireshark. Allows
real-time sniffing.

Prerequisites:
- Cygwin
- Perl
- Whireshark

How to use the sniffer
Once the board is flashed with the proper binary, open a cygwin shell in this folder
(Utilities/serial-sniffer) and call this command:

 ./serialdump-windows.exe -b115200 /dev/ttySX  | ./convert-to-binary | [PATH_TO_]/wireshark.exe -k -i -


sudo serialdump-linux -b115200 /dev/ttyACM0|./convert-to-binary|wireshark -k -i - 
 
(mind the trailing dash!)
ttySx is dependent on which device number your Nucleo board will take.
whireshark.exe must be in path or you must provide the full path.

_______________________________________________________________________________
- (C) COPYRIGHT 2018 STMicroelectronics                   ****END OF FILE**** -