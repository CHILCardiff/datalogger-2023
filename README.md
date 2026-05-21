# CHIL datalogger firmware (v2023-2)
This repository contains the firmware for the CHIL Cryoegg datalogger (version 2023, modified to include the reassurance display).

## Enable receiver channel 2
In order to enable Channel 2 of the receiver, there is a modification that must be made to the 'variant.cpp' file which corresponds to the Adafruit Feather M0 board used in the datalogger.

To find the correct "variant.cpp" file either:

a.) navigate to your user directory (i.e. C:/Users/janedoe/) and then find the file: '.platformio\packages\framework-arduino-samd-adafruit\variants\feather_m0\variant.cpp'.

b.) within VSCode, right click an instance of "sercom5" and select "Go to Definition".
    From here, find the variant.cpp file.

 Then identify and comment out the lines below:
 
```C
Uart Serial5( &sercom5, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
...
void SERCOM5_Handler()
{
Serial5.IrqHandler();
}
```