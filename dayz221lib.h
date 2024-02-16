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
public:
    int initialSpeed;
    NikiMotors(Motor* leftMotor, Motor* rightMotor, int initialSpeed = 100, uint32_t timeOfOneRotate = 1000);
    void setTimeOfOneRotate(uint32_t time);
    void rotate(int degs, int speed = 0);
    void moveMilliseconds(int speed, uint32_t time);
    void moveMilliseconds(int leftSpeed, int rightSpeed, uint32_t time);
    void move(int leftSpeed, int rightSpeed);
    void move(int speed);
    void stop();
    Motor* operator[] (int id);
};


class Sonic {
private:
    int echo, trig;
public:
    Sonic(int echo, int trig);
    float getDist();
};

class IRSensor {
private:
    int pin;
public:
    IRSensor(int pin);
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
    int last_err;
    LineSensor** sensors;

public:
    int count;
    LineSensors(int leftSensor, int rightSensor);
    LineSensors(int count, int* pins);
    int getBin();
    void getArray(bool* arr);
    float getError();
    int getError4();
    void debug();
    int getCount();
    LineSensor* operator[] (int id);
};


class LineFollower {
private:
    NikiMotors* motors;
    LineSensors* sensors;

public:
    LineFollower(NikiMotors* motors, LineSensors* sensors);
    void follow(int speed, float k = 1.0);
    uint32_t followUntilLineEnd(int speed, uint32_t deltaTime = 0);
    uint32_t followUntilCrossroad(int speed, uint32_t deltaTime = 0);
    void followMilliseconds(int speed, uint32_t time);
    uint32_t moveUntilLine(int speed, uint32_t deltaTime = 0);
    uint32_t moveUntilLine(int leftSpeed, int rightSpeed, uint32_t deltaTime = 0);
    uint32_t rotateUntilLine(int speed, bool isRightHanded = true);
    void lineCalibrate(int cnt_of_lines_per_one_rotate);
    // void objectCalibrate();
    void stop();
};


class WallFollower {
private:
    NikiMotors* motors;
    LineSensors* sensors;
    IRSensor* sonic;

public:
    WallFollower(NikiMotors* motors, LineSensors* sensors, IRSensor* sonic);
    void followWall(int speed, float distance, float k = 1);
    uint32_t followUntilLine(int speed, float distance, float k, uint32_t deltaTime = 0);
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


#define  C0 16.35
#define Db0 17.32
#define D0  18.35
#define Eb0 19.45
#define E0  20.60
#define F0  21.83
#define Gb0 23.12
#define G0  24.50
#define Ab0 25.96
#define LA0 27.50
#define Bb0 29.14
#define B0  30.87
#define C1  32.70
#define Db1 34.65
#define D1  36.71
#define Eb1 38.89
#define E1  41.20
#define F1  43.65
#define Gb1 46.25
#define G1  49.00
#define Ab1 51.91
#define LA1 55.00
#define Bb1 58.27
#define B1  61.74
#define C2  65.41
#define Db2 69.30
#define D2  73.42
#define Eb2 77.78
#define E2  82.41
#define F2  87.31
#define Gb2 92.50
#define G2  98.00
#define Ab2 103.83
#define LA2 110.00
#define Bb2 116.54
#define B2  123.47
#define C3  130.81
#define Db3 138.59
#define D3  146.83
#define Eb3 155.56
#define E3  164.81
#define F3  174.61
#define Gb3 185.00
#define G3  196.00
#define Ab3 207.65
#define LA3 220.00
#define Bb3 233.08
#define B3  246.94
#define C4  261.63
#define Db4 277.18
#define D4  293.66
#define Eb4 311.13
#define E4  329.63
#define F4  349.23
#define Gb4 369.99
#define G4  392.00
#define Ab4 415.30
#define LA4 440.00
#define Bb4 466.16
#define B4  493.88
#define C5  523.25
#define Db5 554.37
#define D5  587.33
#define Eb5 622.25
#define E5  659.26
#define F5  698.46
#define Gb5 739.99
#define G5  783.99
#define Ab5 830.61
#define LA5 880.00
#define Bb5 932.33
#define B5  987.77
#define C6  1046.50
#define Db6 1108.73
#define D6  1174.66
#define Eb6 1244.51
#define E6  1318.51
#define F6  1396.91
#define Gb6 1479.98
#define G6  1567.98
#define Ab6 1661.22
#define LA6 1760.00
#define Bb6 1864.66
#define B6  1975.53
#define C7  2093.00
#define Db7 2217.46
#define D7  2349.32
#define Eb7 2489.02
#define E7  2637.02
#define F7  2793.83
#define Gb7 2959.96
#define G7  3135.96
#define Ab7 3322.44
#define LA7 3520.01
#define Bb7 3729.31
#define B7  3951.07
#define C8  4186.01
#define Db8 4434.92
#define D8  4698.64
#define Eb8 4978.03
// DURATION OF THE NOTES
#define BPM 120    //  you can change this value changing all the others
#define H 2*Q //half 2/4
#define Q 60000/BPM //quarter 1/4
#define E Q/2   //eighth 1/8
#define S Q/4 // sixteenth 1/16
#define W 4*Q // whole 4/4

void END();