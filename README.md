# 1-Wire (® Maxim Integrated) Slave Library for ATTiny13 AVR microcontroller

It is based OneWireSlave v1.0 by Alexander Gordeyev
It is based on Jim's Studt OneWire library v2.0

## Features
- Basic protocol handling, reading/writing timeslots, 
- Search ROM command (may be disabled)
- Match ROM command  
- Write ROM (like iButton) command (may be disabled)
- Very small size: from 200 bytes (only byte receive & send)
- CRC8 function written in assembler (based on Arduino AVR function by Maxim Integrated)
- EEPROM can be used as ROM