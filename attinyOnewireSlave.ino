// 1.2 MHz (default) built in resonator
#define F_CPU 9600000UL

#include <avr/io.h>
#include <util/delay.h>

#define LED_BIT _BV(PB4)
#define BUTTON_BIT _BV(PB3)

#define ONEWIRE_PORTReg     PORTB
#define ONEWIRE_DDRReg      DDRB
#define ONEWIRE_PINReg      PINB
#define ONEWIRE_PIN         PB3

#include "sourcesru.h"

uint8_t rom[]={0x01, 0xC8, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x08, 0x10, 0x3F};

uint8_t OneWire_waitReset(){
    OneWire_setPinAsInput;
    while( OneWire_readPin )  {} // ждем 0
    // Замер длительности 0
    uint8_t micros = 0;
    while(!OneWire_readPin){
      _delay_us(2);
      if (++micros >197) return 0;  // 394мкс, вместо 540
    }
    if ( micros < 175 ) return 0; //176 через раз. Итого 350мкс, вместо 480
     _delay_us(30);
    //presence - сигнал присутствия
    OneWire_writePinLOW;
    OneWire_setPinAsOutput;
    _delay_us(120);
    OneWire_setPinAsInput;
    _delay_us(300 - 25);
    if(!OneWire_readPin) return 0;
    return 1;
}

uint8_t OneWire_waitTimeSlot() {
    retries = TIMESLOT_WAIT_RETRY_COUNT;
    while (!OneWire_readPin)
        if (--retries == 0)
            return 0;
    retries = TIMESLOT_WAIT_RETRY_COUNT;
    while ( OneWire_readPin)
        if (--retries == 0)
            return 0;
    return 1;
}

uint8_t p_cmd = 0;

int main()
{
  DDRB |= LED_BIT; // OUTPUT
  //DDRB &= ~BUTTON_BIT; // INPUT
  PORTB &= ~LED_BIT; // LOW
//  _delay_ms(250);

  while (1)
  {
        // uint8_t r = 0;
        // OneWire_setPinAsInput;
        // PORTB |= LED_BIT; // HIGH
        // while(OneWire_readPin){_delay_us(2); if (++r == 125) break;}
        // PORTB &= ~LED_BIT; // LOW
        // r= 0;
        // while(OneWire_readPin){_delay_us(2); if (++r == 125) break;}
    cli();
    if (OneWire_waitReset()){
        OneWire_writeByte(p_cmd++);
        // uint8_t cmd = onewire_read();
        // if (cmd == 0xCC) {
        //   PORTB |= LED_BIT; // HIGH
        //   if (cmd == 0 || cmd ==0xFF || p_cmd !=cmd ) _delay_ms(1000); else _delay_ms(100);
        //   PORTB &= ~LED_BIT; // LOW
        //   p_cmd = cmd;
        //   // _delay_ms(250);
        // }
    }
    sei();
  }
}

//if(PINB & BUTTON_BIT)

uint8_t onewire_read_bit() {
  OneWire_writePinLOW;
  OneWire_setPinAsOutput;
  _delay_us(2); // Длительность низкого уровня, минимум 1 мкс
  OneWire_setPinAsInput;
  _delay_us(8); // Пауза до момента сэмплирования, всего не более 15 мкс
  uint8_t r = OneWire_readPin;
  _delay_us(80); // Ожидание до следующего тайм-слота, минимум 60 мкс с начала низкого уровня
  return r;
}

// Читает один байт, переданный устройством, младший бит вперёд, возвращает прочитанное значение
uint8_t onewire_read() {
  uint8_t r = 0;
  for (uint8_t p = 8; p; p--) {
    r >>= 1;
    if (onewire_read_bit())
      r |= 0x80;
  }
  return r;
}

void onewire_send(uint8_t b) {
  for (uint8_t p = 8; p; p--) {
    onewire_send_bit(b & 1);
    b >>= 1;
  }
}

void onewire_send_bit(uint8_t bit) {
  OneWire_writePinLOW;
  OneWire_setPinAsOutput;
  if (bit) {
    _delay_us(5); // Низкий импульс, от 1 до 15 мкс (с учётом времени восстановления уровня)
    OneWire_setPinAsInput;
    _delay_us(90); // Ожидание до завершения таймслота (не менее 60 мкс)
  } else {
    _delay_us(90); // Высокий уровень на весь таймслот (не менее 60 мкс, не более 120 мкс)
    OneWire_setPinAsInput;
    _delay_us(5); // Время восстановления высокого уровеня на шине + 1 мс (минимум)
  }
}
