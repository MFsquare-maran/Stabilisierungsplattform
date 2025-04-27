#include "DS1820.h"

DS1820::DS1820(PinName pin) : _dataPin(pin) {
    _dataPin.output();
    _dataPin = 1; // Setze den Bus auf High
}

bool DS1820::begin() {
    return reset(); // Prüfen, ob der Sensor antwortet
}

float DS1820::readTemperature() {
    reset();

    writeByte(0xCC); // Skip ROM
    writeByte(0x44); // Start Temperaturmessung

    // Warte, bis die Temperaturmessung fertig ist
    // Optional: kann auch durch Lesen des Busy-Bits erfolgen, hier einfach Polling:
    while (!isConversionDone()) {
       ThisThread::sleep_for(1ms);
    }

    reset();

    writeByte(0xCC); // Skip ROM
    writeByte(0xBE); // Lese Scratchpad

    uint8_t lsb = readByte();
    uint8_t msb = readByte();

    int16_t rawTemp = (msb << 8) | lsb;

    return rawTemp / 16.0f;
}

bool DS1820::reset() {
    _dataPin.output();
    _dataPin = 0;
    wait_us(480); // Reset-Puls

    _dataPin = 1;
    _dataPin.input();
    wait_us(70); // Warten auf Präsenz-Puls

    bool presence = (_dataPin == 0);
    wait_us(410); // Restliche Recovery-Zeit

    return presence;
}

void DS1820::writeByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        _dataPin.output();
        _dataPin = 0;
        wait_us(2);

        if (byte & 0x01) {
            _dataPin = 1;
        }

        wait_us(60); // Haltezeit für das Bit
        _dataPin = 1;
        wait_us(1); // Recovery-Time
        byte >>= 1;
    }
}

uint8_t DS1820::readByte() {
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        _dataPin.output();
        _dataPin = 0;
        wait_us(2);

        _dataPin = 1;
        _dataPin.input();
        wait_us(10);

        if (_dataPin) {
            value |= (1 << i);
        }

        wait_us(50); // Slot-Zeit beenden
    }
    return value;
}

bool DS1820::isConversionDone() {
    _dataPin.output();
    _dataPin = 1;
    _dataPin.input();
    return _dataPin.read();
}
