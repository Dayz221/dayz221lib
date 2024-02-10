#include "dayz221lib.h"
#include <Arduino.h>

Motor leftMotor(0);     // инициализация левого мотора
Motor rightMotor(1);    // инициализация правого мотора
NikiMotors motors(&leftMotor, &rightMotor, 200, 3000);   // указатели на левый и правый моторы, время одного оборота (НА СКОРОСТИ 200!!!!)

void setup() {

}

void loop() {
    motors.move(200);       // двигаться вперед со скоростью 200 [-255, 255]
    delay(1000);

    motors.rotate(90);      // повернуться на 90 градусов против часовой стрелки
    
    motors.move(200, 255);  // двигаться вперед со скоростями двигателей 200, 255
    delay(1000);

    motors.stop();          // остановить робота
}