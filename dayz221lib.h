#include <Arduino.h>


#define left_pwm 6
#define left_dir 7
#define right_pwm 5
#define right_dir 4


class Motor {
private:
    int directionPin, pwmPin;
    float k;
public:
    Motor(int directionPin, int pwmPin, float k = 1.0);
    Motor(bool isRight = false, float k = 1.0);
    void setK(float k);
    void setSpeed(int speed);
};


class NikiMotors {
private:
    Motor* leftMotor;
    Motor* rightMotor;
    uint32_t timeOfOneRotate;
    int initialSpeed;
public:
    NikiMotors(Motor* leftMotor, Motor* rightMotor, int initialSpeed = 100, uint32_t timeOfOneRotate = 1000);
    void setTimeOfOneRotate(uint32_t time);
    void rotate(int degs, int speed = 0);
    void move(int leftSpeed, int rightSpeed);
    void move(int speed);
    void stop();
};


class Sonic {
private:
    int echo, trig;
public:
    Sonic(int echo, int trig);
    float getDist();
};


class LineSensor {
private:
    int pin;
public:
    LineSensor(int pin);
    bool get();
};

class LineSensors {
private:
    int count;
    LineSensor** sensors;

public:
    LineSensors(int leftSensor, int rightSensor);
    LineSensors(int count, int* pins);
    int getBin();
    void getArray(bool* arr);
    float getError();
    void debug();
};

class LineFollower {
private:
    NikiMotors* motors;
    LineSensors* sensors;

public:
    LineFollower(NikiMotors* motors, LineSensors* sensors);
    void follow(int speed, float k = 1.0);
    void stop();
};


class Timer {
private:
    uint32_t last;
public:
    bool get(uint32_t delta);
};


class PID {
private:
    float kp, ki, kd, minOut, maxOut;
    float prev_i, prev_err;
    float *input;
    float *output;
    float *setpoint;
    float dt;

public:
    PID(float* input, float* output, float* setpoint, float kp, float ki, float kd, float dt, float minOut, float maxOut);
    void compute();
};