/*
OneWireSlave for ATTINY13 v1.0 by Alexey Bogdan (aka SrFatCat)
It is based OneWireSlave v1.0 by Alexander Gordeyev
It is based on Jim's Studt OneWire library v2.0
*/

#ifndef ONEWIRESLAVE_H
#define ONEWIRESLAVE_H

//#include <inttypes.h>
#ifndef OW_PIN
#error "Define OW_PIN first!"
#endif
extern "C" {
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#ifdef OW_FLEX_ADDRESS
#include <avr/eeprom.h>
#endif
}

#define DIRECT_READ         (( PINB & (1<<OW_PIN) ) ? 1 : 0) 
#define DIRECT_MODE_INPUT   (DDRB &= ~(1<<OW_PIN))
#define DIRECT_MODE_OUTPUT  (DDRB |= (1<<OW_PIN))
#define DIRECT_WRITE_LOW    (PORTB &=  ~(1<<OW_PIN))
#define DIRECT_WRITE_HIGH   (PORTB |= (1<<OW_PIN))

#define TIMESLOT_WAIT_RETRY_COUNT ( ((120L) * (F_CPU / 1000L)) / 1000L ) / 10L

class OneWireSlave {
  private:
    bool recvAndProcessCmd();
    bool waitTimeSlot();
    static __inline__ uint8_t _crc_update(uint8_t __crc, uint8_t __data);
#ifdef OW_FLEX_ADDRESS
#ifdef OW_ADDR
#error "Don't define OW_ADDR in FLEX_ADDRESS mode!"
#endif 
    uint8_t rom[8];   
#else
#ifndef OW_ADDR
#error "Define OW_ADDR in this mode!"
#endif
    uint8_t rom[8] = {OW_ADDR};
#endif        
  public:
    void setRom();
    bool waitForRequest();
    bool waitReset();
#ifndef OW_DISABLE_SEARCH    
    bool search();
#endif  
    bool sendData(uint8_t buf[], uint8_t data_len);
    bool recvData(uint8_t buf[], uint8_t data_len);
    bool send(uint8_t v);
    uint8_t recv(bool &);
    bool sendBit(uint8_t v);
    uint8_t recvBit(bool &);
    static uint8_t crc8(uint8_t *addr, uint8_t len);
};

void OneWireSlave::setRom() {
#ifdef OW_FLEX_ADDRESS
    for (uint8_t i=0; i<7; i++) this->rom[i] = eeprom_read_byte((const uint8_t*)i);
#endif   
    this->rom[7] = crc8(this->rom, 7);

}

bool OneWireSlave::waitForRequest() {
    for (;;) {
        if (!waitReset() ) continue;
        return recvAndProcessCmd();
    }
}

bool OneWireSlave::recvAndProcessCmd() {
    uint8_t addr[8];

    for (;;) {
      bool err;  
      uint8_t cmd = recv(err);
      if (err) return false;
      if (cmd == 0xCC) return true; 
#ifndef OW_DISABLE_SEARCH         
      if (cmd == 0xF0) {
          search();
          return false;
      }
#endif      
      else if (cmd == 0x33){
            if (!sendData(rom, 8)) return false;
      }
#if defined(OW_FLEX_ADDRESS) && !defined(OW_DISABLE_WRITE_ID)
      else if (cmd == 0x55 || cmd == 0xD5) {
            if (!recvData(addr, 8)) return false;
            if (cmd == 0xD5) { 
              const uint8_t crc = crc8(addr,7); 
              if (crc == addr[7]) send(crc); else return false;
            }
            for (int i=0; i<8; i++){
                if (cmd == 0x55) {if (rom[i] != addr[i]) return false;}
                else eeprom_write_byte((uint8_t*)i, rom[i] = addr[i]);
            }
            if (cmd == 0xD5) continue; else return true;
      }  
#else
      else if (cmd == 0x55 ) {
            if (!recvData(addr, 8)) return false;
             for (int i=0; i<8; i++){
                if (rom[i] != addr[i]) return false;
            }
            return true;
      }  
#endif
    }
}

#ifndef OW_DISABLE_SEARCH
bool OneWireSlave::search() {
    uint8_t bitmask;
    uint8_t bit_send, bit_recv, crc = 0;

    for (int i=0; i<8; i++) {
        for (bitmask = 0x01; bitmask; bitmask <<= 1) {
            bit_send = (bitmask & rom[i])?1:0;
            sendBit(bit_send);
            sendBit(!bit_send);
            bool err;
            bit_recv = recvBit(err);
            if (err) return false;
            if (bit_recv != bit_send)
                return false;
        }
    }
    return true;
}
#endif // OW_DISABLE_SEARCH

bool OneWireSlave::waitReset(){
    DIRECT_MODE_INPUT;
    while( DIRECT_READ ); // ждем 0
    // Замер длительности 0
    uint8_t micros = 0;
    while(!DIRECT_READ){
      _delay_us(2);
      if (++micros >197) return false;  // 394мкс, вместо 540
    }
    if ( micros < 174) return false; //176 через раз. Итого 350мкс, вместо 480
     _delay_us(30);
    //presence - сигнал присутствия
    DIRECT_WRITE_LOW;
    DIRECT_MODE_OUTPUT;
    _delay_us(120);
    DIRECT_MODE_INPUT;
    _delay_us(275);
    if(!DIRECT_READ) return false;
    return true;
}

bool OneWireSlave::sendData(uint8_t buf[], uint8_t len) {
    for (int i=0; i<len; i++) {
        if (!send(buf[i])) return false;
    }
    return true;
}

bool OneWireSlave::recvData(uint8_t buf[], uint8_t len) {    
    bool err;
    for (int i=0; i<len; i++) {
        buf[i] = recv(err);
        if (err) return false;
    }
    return true;
}

bool OneWireSlave::send(uint8_t v) {
    for (uint8_t bitmask = 0x01; bitmask; bitmask <<= 1)
        if (!sendBit((bitmask & v)?1:0)) return false;
    return true;        
}

uint8_t OneWireSlave::recv(bool &err) {
    uint8_t r = 0;
    for (uint8_t bitmask = 0x01; bitmask ; bitmask <<= 1){
        if (recvBit(err))
            r |= bitmask;
        if (err) break;    
    }
    return r;
}

bool OneWireSlave::sendBit(uint8_t v) {
    // cli();
    DIRECT_MODE_INPUT;
    if (!waitTimeSlot() ) {
        // sei();
        return false;
    }
    if (v & 1)
        _delay_us(30);
    else {
        // cli();
        DIRECT_WRITE_LOW;
        DIRECT_MODE_OUTPUT;
        _delay_us(30);
        DIRECT_WRITE_HIGH;
        // sei();
    }
    // sei();
    return true;
}

uint8_t OneWireSlave::recvBit(bool &err) {
    uint8_t r;
    err = false;
    // cli();
    DIRECT_MODE_INPUT;
    if (!waitTimeSlot() ) {
        err = true;
        // sei();
        return 0;
    }
    _delay_us(30);
    r = DIRECT_READ;
    // sei();
    return r;
}

bool OneWireSlave::waitTimeSlot() {
    uint16_t retries;

    retries = TIMESLOT_WAIT_RETRY_COUNT;
    while ( !DIRECT_READ)
        if (--retries == 0)
            return false;
    retries = TIMESLOT_WAIT_RETRY_COUNT;
    while ( DIRECT_READ)
        if (--retries == 0)
            return false;
    return true;
}

uint8_t OneWireSlave::crc8(uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;
    
    while (len--) {
        crc = _crc_update(crc, *addr++);
    }
    return crc;
}

uint8_t OneWireSlave::_crc_update(uint8_t __crc, uint8_t __data){
	uint8_t __i, __pattern;
	__asm__ __volatile__ (
		"	eor	%0, %4" "\n\t"
		"	ldi	%1, 8" "\n\t"
		"	ldi	%2, 0x8C" "\n\t"
		"1:	lsr	%0" "\n\t"
		"	brcc	2f" "\n\t"
		"	eor	%0, %2" "\n\t"
		"2:	dec	%1" "\n\t"
		"	brne	1b" "\n\t"
		: "=r" (__crc), "=d" (__i), "=d" (__pattern)
		: "0" (__crc), "r" (__data));
	return __crc;
} 


#endif
