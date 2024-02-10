#include "dayz221lib.h"
#include <Arduino.h>

int lineSensorsPins[4] = {8, 9, 11, 12};
LineSensors sensors(4, lineSensorsPins);

void setup() {
    Serial.begin(9600);
}

void loop() {
    int bin = sensors.getBin();         // получить двоичное число, где каждый бит - показания датчика (пример: 0b0011 - линия на 3 и 4 датчиках)
    Serial.println(bin);   
    
    switch (bin) {                      // пример обработки данных с датчиков 1 способом
        case 0b0001: /* обработка события */ break;
        case 0b0011: /* обработка события */ break;
        case 0b0010: /* обработка события */ break;
        case 0b0110: /* обработка события */ break;
        case 0b0100: /* обработка события */ break;
        case 0b1100: /* обработка события */ break;
        case 0b1000: /* обработка события */ break;
    }

    bool values[4];
    sensors.getArray(values);           // записать показания с датчиков в массив (лучше использовать 1 метод)
    for (auto a : values) {
        Serial.print(a);
        Serial.print(" ");
    }
    Serial.println("\n");
}