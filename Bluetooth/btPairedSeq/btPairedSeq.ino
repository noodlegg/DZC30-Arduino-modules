// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69);  // <-- use for AD0 high
#define RPIN 6        // pin 6 for red
#define BPIN 5        // pin 5 for blue
#define GPIN 3        // pin 3 for green

int tickRate;
int selectPin;
int pinCounter;
int accelCounter;
bool prevNegative = false;          // tracks whether previous value was negative
int temp = 0;                       // for testing purposes
int colorOutput[3] = {0, 0, 255};   // actual color output array in [R, G, B]
int seq = 0;
int seq2 = 0;
 
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2             // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13                  // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(RPIN, OUTPUT);
    pinMode(GPIN, OUTPUT);
    pinMode(BPIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // fade color every 10 packets
        if (tickRate/10 >= 1 && seq2 < 5) {
          displayColor(colorOutput, true);
        }

        // only save results every 20 packets
        if (tickRate/20 >= 1) { 
          // display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.println(aaReal.z);
  
          tickRate = 0;

          // Negative aaReal.z implies upwards acceleration? (lifting the foot)
          // If the next aaReal.z is then positive, then the foot has moved back downwards supposedly
          // Idle aaReal.z hovers around the range of ~1100
          if (aaReal.z < 0) {
            prevNegative = true;
            temp = aaReal.z;
          } else if (prevNegative && temp + 1000 < aaReal.z) {
            stepColor();
            seq2++;
            prevNegative = false;
          } else {
            prevNegative = false;
          }
          if (seq < 5) {
            walkingColor();
          } else if (seq < 10) {
            walkingColor2();
          } else if (seq < 15) {
            walkingColor3();
          } else if (seq < 20) {
            walkingColor4();
          } else {
            if (seq2 < 5) { // take 3 steps and then double tap
              walkingColor5();
            } else {
              whiteColor();
            }
          }
          seq++;
        }
        tickRate++;
    }
}

// color that gets displayed upon detecting a step
void stepColor() {
  colorOutput[2] = 255;
  displayColor(colorOutput, false);
}

// color values are currently for testing
void walkingColor() {
  colorOutput[1] = random(40, 80);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, true);
}
void walkingColor2() {
  colorOutput[1] = random(90, 120);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, true);
}
void walkingColor3() {
  colorOutput[1] = random(120, 150);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, true);
}
void walkingColor4() {
  colorOutput[1] = random(150, 200);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, true);
}
void walkingColor5() {
  colorOutput[1] = random(240, 255);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, true);
}
void whiteColor() {
  colorOutput[0] = 255;
  colorOutput[1] = 255;
  colorOutput[2] = 255;
  displayColor(colorOutput, false);
}

// color values are currently for testing
void runningColor() {
  colorOutput[1] = random(200, 255);
  colorOutput[0] = colorOutput[1] - random(10, 30);
  displayColor(colorOutput, false);
}

void displayColor(int color[3], bool fade) {
  for (int i = 0; i < 3; i++) {
    cycleRGBPins(pinCounter);
    pinCounter++;
    if (fade) {
      color[i] = color[i] - random(2, 5);
      color[2] -= 5;  // to make blue fade faster
      if (color[i] < 0) {
        color[i] = 0;
      }
      analogWrite(selectPin, color[i]);
    } else {
      analogWrite(selectPin, color[i]);
      //Serial.print(selectPin);
      //Serial.print(" is set to value: ");
      //Serial.println(color[i]);
    }
  }
}

// After calling this function, do "pinCount++" to cycle through the pins
void cycleRGBPins(int pinCounter) {
  int pin = pinCounter % 3;
  switch (pin) {
    case (0):
      selectPin = RPIN;
      break;
    case (1):
      selectPin = GPIN;
      break;
    case (2):
      selectPin = BPIN;
      break;
  }
}
