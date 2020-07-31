/********** PID **********/
#include <PID_v1.h>

double kP = 24;
double kI = 350;
double kD = 1.2;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup

/********** radio **********/
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

RF24 radio(10, 4);    // CE, CSN
const byte address[6] = "00001";
float angleV = 0, turnV = 0; // values from remote
float controlValues[2]; // array to receive both data values

/********** L298N **********/
#include <L298N.h>

// Pin definition
const unsigned int EN_A = 3;
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// motor speeds
int speedLeft = 0;
int speedRight = 0;

/********** MPU **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
long velocity;

float trimPot;
float trimAngle;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/********** SETUP **********/
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);
    
    pinMode(LED_PIN, OUTPUT);

    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
}

void loop() {
  
  // only read remote data if remote is available, otherwise balance normally
  if (radio.available()) {
    radio.read(&controlValues, sizeof(controlValues)); // read values of array
    
    angleV = controlValues[0]; // assign array values to control variables
    turnV = controlValues[1];
  } 
  
  trimPot = analogRead(A3)*-1; // read dial on robot and *-1 for direction

  trimAngle = (trimPot/100) + 5 + angleV; // adjust to degrees

  if (IMUdataReady == 1) {
    readAngles();
  }

  pitch = (ypr[1] * 180/M_PI); // adjust to degrees

  if (abs(turnV) < 15) { // turnV threshold
    turnV = 0;
  }

  if (abs(angleV) < .17) { // angleV threshold
    angleV = 0;
  }

  // PID vars
  setpoint = trimAngle + angleV; 
  input = pitch;

  pid.Compute();

  // set motor speed with adjusted turn values
  speedLeft = output -  turnV;
  speedRight = output + turnV;
  
  if (pitch > 25 || pitch < -25) { // angle threshold
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 
  } else {
    leftMotor.setSpeed(abs(speedLeft));
    rightMotor.setSpeed(abs(speedRight));
  }

  // move motors
  if (speedLeft < 0) { 
    leftMotor.forward();
  } else {
    leftMotor.backward();
  }
  
  if (speedRight < 0) { 
    rightMotor.forward();
  } else {
    rightMotor.backward();
  }
  
  // print some control info
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print(" , trimPot: ");
  Serial.print(trimPot);
  Serial.print(" , angleV: ");
  Serial.print(angleV);
  Serial.print(" , turnV: ");
  Serial.println(turnV);

}
