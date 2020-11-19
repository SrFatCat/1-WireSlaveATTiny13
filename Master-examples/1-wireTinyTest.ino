#ifdef ESP8266
#define DEBUG_PRINT(...) Serial.printf( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...) 
#define yield() 
#endif // ESP8266

#include <OneWire.h>

OneWire  wire(D1);

uint8_t addr[8];

void printHexArray(char *txt, uint8_t *arr, uint8_t len){
    DEBUG_PRINT(txt);
    for (int i =0; i<len; i++) DEBUG_PRINT("%02X ", arr[i]);
    DEBUG_PRINT("\n");    

}

uint32_t setOutput(bool state, uint32_t timeOut){
    const uint32_t t = millis();
    uint8_t buf[5] = {0x55, 0x01, 0x07, state ? 0x01 : 0x00, 0}; // byte #0 is WRITE_STATUS (like ds2406) byte #1-2 magic number :) (any), byte #3 for new value slave output, byte #4 CRC
    buf[4] = OneWire::crc8(buf, 4);
    uint8_t rd = 0;
    do {
        if (millis() - t > timeOut) return 999999;
        if (wire.reset()) {
            wire.select(addr);
            wire.write_bytes(buf, 5);
            delayMicroseconds(15);
            rd = wire.read();
        }
        yield();
    }while (rd != buf[4]);
    return millis() - t;
}

uint32_t getInput(uint32_t timeOut){
    const uint32_t t = millis();
    uint8_t buf[5] = {0xAA, 0x01, 0x07, 0x00, 0}; // byte #0 is READ_STATUS (like ds2406) byte #1-2 magic number :) (any), byte #3 - result for slave input value, byte #4 CRC 
    buf[4] = OneWire::crc8(buf, 4);
    uint8_t crc = 0;
    do {
        if (millis() - t > timeOut) return 999999;
        if (wire.reset()) {
            wire.select(addr);
            wire.write_bytes(buf, 5);
            delayMicroseconds(30);
            wire.read_bytes(buf,5);
            crc = OneWire::crc8(buf, 4);
        }
        yield();
    }while (crc != buf[4]);
    return millis() - t;
}

void writeAddr(uint32_t timeOut){
    const uint32_t t = millis();
    uint8_t crc = 0;
    uint8_t buf[] = {0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xBB}; // new ADDR for slave 7 bytes & CRC
    do {
        if (millis() - t > timeOut) return;
        if (wire.reset()) {
            wire.write(0xD5); // command for write new address in slave (like rewritable iButton)
            wire.write_bytes(buf, 8);
            delayMicroseconds(20);
            crc = wire.read();
        }
        yield();
    } while (crc != buf[7]);

}

void setup(){
    Serial.begin(115200);
    DEBUG_PRINT("\n\n\n Start test \n\n");
}


void loop(){
    wire.reset_search();
    while(wire.search(addr)) {
        if ( OneWire::crc8( addr, 7) != addr[7]) {
            DEBUG_PRINT("CRC is not valid!\n");
            delay(1000);
            return;
        }
        DEBUG_PRINT("[%i] Search complete...\n", millis());
        printHexArray("ROM= ", addr, 8);
        if (addr[0] == 0xFF && addr[7] == 0x14){ // if empty adress (0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0x14) from slave
            DEBUG_PRINT("Write new addr...\n");
            writeAddr(1000);
            break;
        }
        DEBUG_PRINT("setOutput for %ld ms\n", setOutput(true, 1000));
        delay(500);
        DEBUG_PRINT("setOutput for %ld ms\n", setOutput(false, 1000));
        DEBUG_PRINT("getInput for %ld ms\n", getInput(100));
        delay(500);
    }
    delay(500);
}
