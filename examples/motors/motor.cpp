#include <Arduino.h>
#include "dayz221lib.h"

Motor left(false), right(true); // инициализация моторов, false - левый, true - правый

void setup() {
    left.setK(0.6);         // Коэффициент. Подобрать для левого или правого, если машинка едет криво
    right.setSpeed(255);    // Задать скорость для правого мотора (от -255 до 255)
    left.setSpeed(-255);    // Задать скорость для левого мотора (от -255 до 255)
}

void loop() {}