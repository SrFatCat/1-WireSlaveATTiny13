/*
 * OneWire.c
 *
 * Проект, демонстрирующий работу с 1-wire сетью в режиме "мастера".
 * Реализует опрос датчиков температуры DS18B20, DS18S20, DS1822
 *
 * Author: Погребняк Дмитрий, г. Самара, 2013
 *
 * Помещённый здесь код является свободным. Т.е. допускается его свободное использование для любых целей, включая коммерческие, при условии указания ссылки на автора (Погребняк Дмитрий, http://aterlux.ru/).
 */ 


#define F_CPU 9600000UL // Объявление частоты микроконтроллера для макросов _delay_us

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define ONEWIRE_PORT PORTB
#define ONEWIRE_DDR DDRB
#define ONEWIRE_PIN PINB
#define ONEWIRE_PIN_NUM PB0

// Устанавливает низкий уровень на шине 1-wire
inline void onewire_low() {
  ONEWIRE_DDR |= _BV(ONEWIRE_PIN_NUM);
}

// "Отпускает" шину 1-wire
inline void onewire_high() {
  ONEWIRE_DDR &= ~_BV(ONEWIRE_PIN_NUM);
}

// Читает значение уровня на шине 1-wire
inline uint8_t onewire_level() {
  return ONEWIRE_PIN & _BV(ONEWIRE_PIN_NUM);
}


// Определения и функции ниже нужны только если требуется "сильный" подтягивающий резистор
#define ONEWIRE_STRONG_DDR DDRB
#define ONEWIRE_STRONG_PORT PORTB
#define ONEWIRE_STRONG_PIN_NUM PB1

// включение "сильной" подтяжки
void onewire_strong_enable() {
  // Для исключения низкого уровня на шине, сначала изменяется регистр значения
  ONEWIRE_STRONG_PORT |= _BV(ONEWIRE_STRONG_PIN_NUM);
  // затем - регистр направления
  ONEWIRE_STRONG_DDR |= _BV(ONEWIRE_STRONG_PIN_NUM); 
}

// отключение "сильной" подтяжки
void onewire_strong_disable() {
  // Для исключения низкого уровня на шине, сначала изменяется регистр направления
  ONEWIRE_STRONG_DDR &= ~_BV(ONEWIRE_STRONG_PIN_NUM); 
  // затем - регистр значения
  ONEWIRE_STRONG_PORT &= ~_BV(ONEWIRE_STRONG_PIN_NUM);
}


// Выдаёт импульс сброса (reset), ожидает ответный импульс присутствия.
// Если импульс присутствия получен, дожидается его завершения и возвращает 1, иначе возвращает 0 
uint8_t onewire_reset() 
{
  onewire_low();
  _delay_us(640); // Пауза 480..960 мкс
  onewire_high();
  _delay_us(2); // Время необходимое подтягивающему резистору, чтобы вернуть высокий уровень на шине
  // Ждём не менее 60 мс до появления импульса присутствия;
  for (uint8_t c = 80; c; c--) {
    if (!onewire_level()) {
      // Если обнаружен импульс присутствия, ждём его окончания
      while (!onewire_level()) { } // Ждём конца сигнала присутствия
      return 1;
    }
    _delay_us(1);
  }
  return 0;
}


// Отправляет один бит
// bit отправляемое значение, 0 - ноль, любое другое значение - единица
void onewire_send_bit(uint8_t bit) {
  onewire_low();
  if (bit) {
    _delay_us(5); // Низкий импульс, от 1 до 15 мкс (с учётом времени восстановления уровня)
    onewire_high();
    _delay_us(90); // Ожидание до завершения таймслота (не менее 60 мкс)
  } else {
    _delay_us(90); // Высокий уровень на весь таймслот (не менее 60 мкс, не более 120 мкс)
    onewire_high();
    _delay_us(5); // Время восстановления высокого уровеня на шине + 1 мс (минимум)
  }
}

// Отправляет один байт, восемь подряд бит, младший бит вперёд
// b - отправляемое значение
void onewire_send(uint8_t b) {
  for (uint8_t p = 8; p; p--) {
    onewire_send_bit(b & 1);
    b >>= 1;
  }
}

// читает значение бита, передаваемое уйстройством.
// Возвращает 0 - если передан 0, отличное от нуля значение - если передана единица
uint8_t onewire_read_bit() {
  onewire_low();
  _delay_us(2); // Длительность низкого уровня, минимум 1 мкс
  onewire_high();
  _delay_us(8); // Пауза до момента сэмплирования, всего не более 15 мкс
  uint8_t r = onewire_level();
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

// Обновляет значение контольной суммы crc применением всех бит байта b.
// Возвращает обновлённое значение контрольной суммы
uint8_t onewire_crc_update(uint8_t crc, uint8_t b) {
//  return pgm_read_byte(&onewire_crc_table[crc ^ b]);
  for (uint8_t p = 8; p; p--) {
    crc = ((crc ^ b) & 1) ? (crc >> 1) ^ 0b10001100 : (crc >> 1);
    b >>= 1;
  }
  return crc;
}


// Выполняет последовательность инициализации (reset + ожидает импульс присутствия)
// Если импульс присутствия получен, выполняет команду SKIP ROM
// Возвращает 1, если импульс присутствия получен, 0 - если нет
uint8_t onewire_skip() {
  if (!onewire_reset())
    return 0;
  onewire_send(0xCC);
  return 1;
}

// Выполняет последовательность инициализации (reset + ожидает импульс присутствия)
// Если импульс присутствия получен, выполняет команду READ ROM, затем читает 8-байтовый код устройства
//    и сохраняет его в буфер по указателю buf, начиная с младшего байта
// Возвращает 1, если код прочитан, 0 - если импульс присутствия не получен
uint8_t onewire_read_rom(uint8_t * buf) {
  if (!onewire_reset())
    return 0; 
  onewire_send(0x33);
  for (uint8_t p = 0; p < 8; p++) {
    *(buf++) = onewire_read();
  }
  return 1;
}

// Выполняет последовательность инициализации (reset + ожидает импульс присутствия)
// Если импульс присутствия получен, выполняет команду MATCH ROM, и пересылает 8-байтовый код 
//   по указателю data (младший байт вперёд)
// Возвращает 1, если импульс присутствия получен, 0 - если нет
uint8_t onewire_match(uint8_t * data) {
  if (!onewire_reset())
    return 0;
  onewire_send(0x55);
  for (uint8_t p = 0; p < 8; p++) {
    onewire_send(*(data++));
  }
  return 1;
}

// Переменные для хранения промежуточного результата поиска
uint8_t onewire_enum[8]; // найденный восьмибайтовый адрес 
uint8_t onewire_enum_fork_bit; // последний нулевой бит, где была неоднозначность (нумеруя с единицы)

// Инициализирует процедуру поиска адресов устройств
void onewire_enum_init() {
  for (uint8_t p = 0; p < 8; p++) {
    onewire_enum[p] = 0;
  }      
  onewire_enum_fork_bit = 65; // правее правого
}

// Перечисляет устройства на шине 1-wire и получает очередной адрес.
// Возвращает указатель на буфер, содержащий восьмибайтовое значение адреса, либо NULL, если поиск завешён
uint8_t * onewire_enum_next() {
  if (!onewire_enum_fork_bit) { // Если на предыдущем шаге уже не было разногласий
    return 0; // то просто выходим ничего не возвращая
  }
  if (!onewire_reset()) {
    return 0;
  }  
  uint8_t bp = 8;
  uint8_t * pprev = &onewire_enum[0];
  uint8_t prev = *pprev;
  uint8_t next = 0;
  
  uint8_t p = 1;
  onewire_send(0xF0);
  uint8_t newfork = 0;
  for(;;) {
    uint8_t not0 = onewire_read_bit();
    uint8_t not1 = onewire_read_bit();
    if (!not0) { // Если присутствует в адресах бит ноль
      if (!not1) { // Но также присустствует бит 1 (вилка)
        if (p < onewire_enum_fork_bit) { // Если мы левее прошлого правого конфликтного бита, 
          if (prev & 1) {
            next |= 0x80; // то копируем значение бита из прошлого прохода
          } else {
            newfork = p; // если ноль, то запомним конфликтное место
          }          
        } else if (p == onewire_enum_fork_bit) {
          next |= 0x80; // если на этом месте в прошлый раз был правый конфликт с нулём, выведем 1
        } else {
          newfork = p; // правее - передаём ноль и запоминаем конфликтное место
        }        
      } // в противном случае идём, выбирая ноль в адресе
    } else {
      if (!not1) { // Присутствует единица
        next |= 0x80;
      } else { // Нет ни нулей ни единиц - ошибочная ситуация
        return 0;
      }
    }
    onewire_send_bit(next & 0x80);
    bp--;
    if (!bp) {
      *pprev = next;
      if (p >= 64)
        break;
      next = 0;
      pprev++;
      prev = *pprev;
      bp = 8;
    } else {
      if (p >= 64)
        break;
      prev >>= 1;
      next >>= 1;
    }
    p++;
  }
  onewire_enum_fork_bit = newfork;
  return &onewire_enum[0];
}

// Выполняет инициализацию (сброс) и, если импульс присутствия получен,
// выполняет MATCH_ROM для последнего найденного методом onewire_enum_next адреса
// Возвращает 0, если импульс присутствия не был получен, 1 - в ином случае
uint8_t onewire_match_last() {
  return onewire_match(&onewire_enum[0]);
}


// //////////////////// РАБОТА С UART ////////////////////

// // Отправляет один байт в UART
// void uart_write(uint8_t data) {
//   while (!(UCSRA & (1 << UDRE))); // Ожидание освобождения буфера отправки
//   UDR = data; // записывает байт в буфер, отправка начинается автоматически.
// }

// // Отсылает по UART одну цифру, являющуюся результатом деления нацело dig на sub.
// // Возвращает остаток этого деления
// uint16_t uart_digit(uint16_t dig, uint16_t sub) {
//   char c = '0';
//   while (dig >= sub) {
//     dig -= sub;
//     c++;
//   }
//   uart_write(c);
//   return dig;
// }

// // Отсылает в UART десятичное представление числа с фиксированной точкой, 
// // где дробная часть представлена младшими 4 разрядами
// void uart_num(int16_t num) {
//   uint16_t unum; // число без знака
//   if (num < 0) { // отрицательное число. Отсылает знак
//     unum = -num;
//     uart_write('-'); 
//   } else {
//     unum = num;
//   }
//   uint16_t snum =  unum >> 4; // отбрасывает дробную часть
//   if (snum >= 10) {
//     if (snum >= 100) {
//       if (snum >= 1000) {
//         snum = uart_digit(snum, 1000); // 4й разряд
//       }
//       snum = uart_digit(snum, 100); // 3й разряд
//     }
//     snum = uart_digit(snum, 10); // 2й разряд
//   }
//   uart_digit(snum, 1); // 1й разряд
//   uart_write('.'); // десятичный разделитель
//   uart_digit((((uint8_t)(unum & 0x0F)) * 10) >> 4, 1); // дробная часть
// }

// // Отсылает в UART шестнадцатиричное двузначное представление числа
// void uart_hex(uint8_t hexdig) {
//   uart_write((hexdig >> 4) + (((hexdig >> 4) >= 10) ? ('A' - 10) : '0'));
//   uart_write((hexdig & 0x0F) + (((hexdig & 0x0F) >= 10) ? ('A' - 10) : '0'));
// }

// // Отсылает в UART символы перевода строки
// void uart_newline() {
//   uart_write('\r');
//   uart_write('\n');
// }

int main(void)
{
  // Инициализация портов
  ONEWIRE_PORT &= ~_BV(ONEWIRE_PIN_NUM);
  onewire_high();
  onewire_strong_disable();
  
  // Инициализация UART
//   UCSRA &= ~(1 << U2X); // одинарная скорость передачи
//   UCSRB = (1 << TXEN); // Включает передатчик USART
//   UCSRC = 0b1000110; // Асинхронный режим, контроль чётности - отключен, 1 стоп бит, 8 бит данных
//   UBRRH = 0;
//   UBRRL = 51; // 9600 бод
  
  while(1)
  {
    if (onewire_skip()) { // Если у нас на шине кто-то присутствует,...
      onewire_send(0x44); // ...запускается замер температуры на всех термодатчиках
//      onewire_strong_enable(); // Включается усиленная подтяжка
      _delay_ms(900); // Минимум на 750 мс.
//      onewire_strong_disable(); // Обязательно выключить подтяжку для дальнейшего обмена
      
      onewire_enum_init(); // Начало перечисления устройств
      for(;;) {
        uint8_t * p = onewire_enum_next(); // Очередной адрес
        if (!p) 
          break;
        // Вывод шестнадцатиричной записи адреса в UART и рассчёт CRC
        uint8_t d = *(p++);
        uint8_t crc = 0;
        uint8_t family_code = d; // Сохранение первого байта (код семейства)
        for (uint8_t i = 0; i < 8; i++) {
        //   uart_hex(d);
        //   uart_write(' ');
          crc = onewire_crc_update(crc, d);
          d = *(p++);
        }
        if (crc) {
          // в итоге должен получиться ноль. Если не так, вывод сообщения об ошибке
        //   uart_write('C');
        //   uart_write('R');
        //   uart_write('C');
        } else {
          if ((family_code == 0x28) || (family_code == 0x22) || (family_code == 0x10)) { 
            // Если код семейства соответствует одному из известных... 
            // 0x10 - DS18S20, 0x28 - DS18B20, 0x22 - DS1822
            // проведём запрос scratchpad'а, считая по ходу crc
            onewire_send(0xBE); 
            uint8_t scratchpad[8];
            crc = 0;
            for (uint8_t i = 0; i < 8; i++) {
              uint8_t b = onewire_read();
              scratchpad[i] = b;
              crc = onewire_crc_update(crc, b);
            }
            if (onewire_read() != crc) {
              // Если контрольная сумма скретчпада не совпала.
            //   uart_write('~');
            //   uart_write('C');
            //   uart_write('R');
            //   uart_write('C');
            //   uart_hex(crc);
            } else {
              int16_t t = (scratchpad[1] << 8) | scratchpad[0];
              if (family_code == 0x10) { // 0x10 - DS18S20 
                // у DS18S20 значение температуры хранит 1 разряд в дробной части.
                // повысить точность показаний можно считав байт 6 (COUNT_REMAIN) из scratchpad
                t <<= 3;
                if (scratchpad[7] == 0x10) { // проверка на всякий случай
                  t &= 0xFFF0;
                  t += 12 - scratchpad[6];
                }
              } // для DS18B20 DS1822 значение по умолчанию 4 бита в дробной части
              // Вывод температуры
            //   uart_num(t);
            }            
          } else {
            // Неизвестное устройство
            // uart_write('?');
          }
        }
        // uart_newline();
      }      
    //   uart_write('.');
    } else {
    //   uart_write('-');
    }
    // uart_newline();
    _delay_ms(2000); // небольшая задержка
  }
}