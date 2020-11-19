#define F_CPU 9600000UL

//#define OW_DISABLE_SEARCH
//#define OW_DISABLE_WRITE_ID
#define OW_FLEX_ADDRESS // if this defined - the address of slave save in EEPROM
#define OW_PIN PB3
#ifndef OW_FLEX_ADDRESS
#define OW_ADDR 0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xBB // if not OW_FLEX_ADDRESS - the fix address of slave save in sketch
#endif

#include "OneWireSlave.h"

#define WRITE_STATUS 0x55
#define READ_STATUS  0xAA

#define LED_BIT _BV(PB4)
#define INPUT1_BIT _BV(PB1)
#define INPUT2_BIT _BV(PB2)

int main(){
    uint8_t buf[5]; 
    OneWireSlave wire;

    wire.setRom();
    PORTB = 0xFF;
    DDRB |= LED_BIT; // OUTPUT
    while(1){
        if (!wire.waitForRequest()) continue;
        if (!wire.recvData(buf,5)) continue;
        uint8_t crc = OneWireSlave::crc8(buf,4);
        if (crc != buf[4]) continue;
        if (buf[0] == WRITE_STATUS){
            wire.send(crc);
            if (buf[3]) PORTB &= ~LED_BIT; else PORTB |= LED_BIT;
        }
        else if (buf[0] == READ_STATUS){
            buf[3] = (PINB & (INPUT1_BIT | INPUT2_BIT)) >> 1;
            buf[4] = OneWireSlave::crc8(buf,4);
            wire.sendData(buf, 5);
        }
    }
}