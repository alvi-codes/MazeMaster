//NANO esp32 communication script - CORRECT RECEIVED VALUES

//STEERING CODE WORKING - STEERING LEFT - STEPPER MOTORS FACING OUTWARDS

//final design working code PID values: KP: 160, KD: 20, KI: 0
#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "PID.h"

//#define buttonPin1 10
//#define buttonPin2 11

#define pinBit1 10
#define pinBit2 11
#define pinBit3 12

// PORTB, bit 1, PB1
#define MOT_A_STEP  9
#define MOT_A_DIR   8

// PORTD, bit 7, PD7
#define MOT_B_STEP  7
#define MOT_B_DIR   6

#define INTERRUPT_PIN 2

#define PPR   1600
// #define TICKS_PER_SECOND  40000 // 40kHz
#define TICKS_PER_SECOND  50000 // 50kHz
#define PULSE_WIDTH 1

#define MAX_ACCEL (200)
#define ANGLE_Kp  160.0 //160 (working) 450.0
#define ANGLE_Kd  20.0 //20 (working) 30.0
#define ANGLE_Ki  0 //0 (working)

#define VELOCITY_Kp  0.007 //0.007 (working)
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005

#define WARMUP_DELAY_US (5000000UL)

#define ANGLE_SET_POINT (0.0 * DEG_TO_RAD)

#define OUTPUT_READABLE_YAWPITCHROLL
// #define COUNT_LOOP
// #define LOGGING_ENABLED

#define NANO_BLE

/* BLE communication-related params */
#define MAX_PACKET_SIZE 96
#define DIVISOR 10000.0

char packet[MAX_PACKET_SIZE];
uint8_t packet_size = 0;

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;  
uint8_t fifoBuffer[64];

//int buttonState1 = 0;  // variable for reading the pushbutton status
//int buttonState2 = 0;

int bit1 = 0;
int bit2 = 0;
int bit3 = 0;

int prebit1 = 0;
int prebit2 = 0;
int prebit3 = 0;

int ignore = 0;

/*
int roverState1 = 0; //forward
int roverState2 = 0; //backward
int roverState3 = 0; //left
int roverState4 = 0; //right
int roverState5 = 0; //idle

//int prevbuttonState1 = 0;
//int prevbuttonState2 = 0;

int prevroverState1 = 0;
int prevroverState2 = 0;
int prevroverState3 = 0;
int prevroverState4 = 0;
int prevroverState5 = 0;
*/

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

float pid_settings[6] = {
  ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
  VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki
};

float joystick[2] = {0.0f, 0.0f};
float ref_velocity = 0.0f;
float ref_steering = 0.0f;
float steering = 0.0f;

volatile unsigned long currentTickLeft = 0UL;
volatile unsigned long currentTickRight = 0UL;
volatile unsigned long ticksPerPulseLeft = UINT64_MAX;
volatile unsigned long ticksPerPulseRight = UINT64_MAX;
volatile float accel = 0.0;
volatile float velocity = 0.0;

//volatile bool steeringFlag = false;     // steering flag
//volatile bool speedFlag = false;     // speed flag

volatile bool steeringLeftFlag = false;     // steering flag
volatile bool steeringRightFlag = false;     // steering flag
volatile bool speedFlagBackward = false;     // speed flag
volatile bool speedFlag = false;
volatile int steerDirection = 0;
volatile int roverState = 0;
volatile int preroverState = 0;


bool isBalancing = false;

float angle = 0.0;
int n = 1;
float targetAngle = ANGLE_SET_POINT;

float targetVelocity = 0.0;

unsigned long lastUpdateMicros = 0;

void send_float_array(float *a, uint8_t size);
void parse_float_array(char *p, uint8_t p_size, float *dest);
void parse_settings(char *p, uint8_t p_size);
void parse_control(char *p, uint8_t p_size);
void handle_packet(char *p, uint8_t p_size);

void initMPU() {
  const int16_t accel_offset[3] = { -1262, -307, 1897 };
  const int16_t gyro_offset[3] = { 23, -41, 49 };

  Wire.begin();
  Wire.setClock(1000000UL);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    while(1) {}
  }

  mpu.dmpInitialize();

  mpu.setXGyroOffset(gyro_offset[0]);
  mpu.setYGyroOffset(gyro_offset[1]);
  mpu.setZGyroOffset(gyro_offset[2]);
  mpu.setXAccelOffset(accel_offset[0]);
  mpu.setYAccelOffset(accel_offset[1]);
  mpu.setZAccelOffset(accel_offset[2]);

  mpu.setDMPEnabled(true);
  //So when interrupt pin goes from LOW to HIGH then dmpDataReady func is called meaning data is ready to be read from the MPU.
  //Interrupt goes form low to high when timer is equal to comparison value (ocra).
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
}

void setTimer1(int ocra) {  
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0  
  
  // ocra = 16MHz / prescaler / desired_f - 1
  OCR1A = ocra;
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11);  // set prescaler to 8  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
}

void setTimers() {
  cli();
  // setTimer1(49); // 40kHz
  setTimer1(39); // 39 (working) 50kHz
  sei();
}

void initMotors() {
  pinMode(MOT_A_DIR, OUTPUT);
  pinMode(MOT_A_STEP, OUTPUT);
  pinMode(MOT_B_DIR, OUTPUT);
  pinMode(MOT_B_STEP, OUTPUT);
  digitalWrite(MOT_A_STEP, LOW);
  digitalWrite(MOT_B_STEP, LOW);

  //Initialize button:
//  pinMode(buttonPin1, INPUT);
//  pinMode(buttonPin2, INPUT);

  pinMode(pinBit1, INPUT);
  pinMode(pinBit2, INPUT);
  pinMode(pinBit3, INPUT);
  
}

void log(unsigned long nowMicros) {  
  static unsigned long timestamp = micros();  
  if (nowMicros - timestamp < 10000 /* 100Hz */) {
    return;
  }
  Serial.print("a0:");
  Serial.print(targetAngle * RAD_TO_DEG, 4);
  Serial.print("\ta:");
  Serial.print(angle * RAD_TO_DEG, 4);
  Serial.print("\tv:");
  Serial.print(velocity, 4);
  Serial.print("\tu:");
  Serial.println(accel, 4);  
  timestamp = nowMicros;   
}

bool mpuUpdate() {
  if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuInterrupt = false;
    return true;
  }
  return false;
}

unsigned long getTicksPerPulse(float velocity) {
  if (abs(velocity) < 1e-3) {
    // TODO: disable motor
    return UINT64_MAX;
  } else {
    return (uint64_t)(2.0 * PI * TICKS_PER_SECOND / (abs(velocity) * PPR)) - PULSE_WIDTH;
  }
}

void updateVelocity(unsigned long nowMicros) {
  // static unsigned long counter = 0;
  // static unsigned long sum = 0;

  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 100 /* 10kHz */) {
    return;
  }

  // sum += (nowMicros - timestamp);
  // counter++;
  // if (counter >= 1000) {
  //   Serial.println(((float)(sum)) / counter);
  //   counter = 0;
  //   sum = 0;
  // }

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  velocity += accel * dt;
  
  float leftVelocity = velocity + steerDirection * steering;
  float rightVelocity = velocity - steerDirection * steering;
  ticksPerPulseLeft = getTicksPerPulse(leftVelocity);
  ticksPerPulseRight = getTicksPerPulse(rightVelocity);
  
  if (leftVelocity > 0) {
    digitalWrite(MOT_A_DIR, HIGH);    
  } else {
    digitalWrite(MOT_A_DIR, LOW);
  }

  if (rightVelocity > 0) {
    digitalWrite(MOT_B_DIR, LOW);  
  } else {
    digitalWrite(MOT_B_DIR, HIGH);  
  }

  timestamp = nowMicros;
}

void updateControl(unsigned long nowMicros) {
  /* Wait until IMU filter will settle */
  if (nowMicros < WARMUP_DELAY_US) {
    return;
  }

  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 1000 /* 1kHz */) {
    return;
  }
  if (!mpuUpdate()) {
    return;
  }
  angle = ypr[1];
  //Serial.print("Input angle is: ");
  //Serial.println(angle);

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;

  if (abs(angle - targetAngle) < PI / 18) {
    isBalancing = true;
  }

  if (abs(angle - targetAngle) > PI / 4) {
    isBalancing = false;
    accel = 0.0;
    velocity = 0.0;    
  }

  if (!isBalancing) {
    return;
  }
  targetAngle = -velocityPID.getControl(velocity, dt);
  anglePID.setTarget(targetAngle);

  accel = 0.99 * anglePID.getControl(angle, dt) + 0.01 * accel; 
  //accel = anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  timestamp = nowMicros;
}

int inputState (int bit1_, int bit2_, int bit3_) {
  
  int state;
  
  if((!bit3_)&&(!bit2_)&&(!bit1_)){ // idle state is 000
      state = 0; 
      return state;
   }

   if((!bit3_)&&(!bit2_)&&(bit1_)){ //forward state is 001
      state = 1;
      return state;
   }

    if((!bit3_)&&(bit2_)&&(!bit1_)){ // backward state is 010
      state = 2;
      return state;
    }

    if((!bit3_)&&(bit2_)&&(bit1_)){ //left state is 011
      state = 3;
      return state;
    }

    if((bit3_)&&(!bit2_)&&(!bit1_)){ // right state is 100
      state = 4;
      return state;
    }

    return 5;
}

void setup() {
  Serial.begin(115200);
  #ifdef NANO_BLE
    Serial.print("AT+BAUD=4\r\n");
  #endif
  Serial.print("SETUP FUNC!");
  initMPU();
  setTimers();
  initMotors();
}

void loop() { 

 

//  buttonState1 = digitalRead(buttonPin1);
//  Serial.print("Button 1 state is: ");
//  Serial.println(buttonState1);
//
//  buttonState2 = digitalRead(buttonPin2);
//  Serial.print("Button 2 state is: ");
//  Serial.println(buttonState2);
//
//  Serial.print("Previous Button 1 State is: ");
//  Serial.println(prevbuttonState1);
//
//  Serial.print("Previous Button 2 State is: ");
//  Serial.println(prevbuttonState2);

  bit1 = digitalRead(pinBit1);
  bit2 = digitalRead(pinBit2);
  bit3 = digitalRead(pinBit3);

  roverState = inputState(bit1, bit2, bit3);


  if (roverState != preroverState) {
    
    if(((bit1) || (bit2) || (bit3)) && ((!prebit1) && (!prebit2) && (!prebit3))) {
  
      if (ignore==0) { //as long as ignore is 0 then output
       
        Serial.print("The rover state is: ");
        Serial.println(roverState); 
        Serial.print("BITS RECEIVED: ");
        Serial.print(bit3);
        Serial.print(bit2);
        Serial.println(bit1);
  
        if(roverState == 1) { //ignore the next 2 iterations - FORWARD
          Serial.println("GO FORWARD!");
          ignore = 2;
          speedFlag = true;
          steeringLeftFlag = false;
          steeringRightFlag = false;
        }
  
        if(roverState == 4) { // - RIGHT
          Serial.println("GO RIGHT!");
          ignore = 1;
          steeringRightFlag = true; 
          steeringLeftFlag = false;
          speedFlag = false;
        }
  
        if(roverState == 3) { // - LEFT
          Serial.println("GO LEFT!");
          ignore = 0;
          steeringLeftFlag = true; //so steer in position
          steeringRightFlag = false;
          speedFlag = false;
        }
      }
  
      else {
        ignore--;
      }
    }
    
    if((!bit3)&&(!bit2)&&(!bit1)){ // idle state is 000
      speedFlag = false;
      steeringLeftFlag = false;
      steeringRightFlag = false;
    }

    /*

    queue.push(roverState);

    //push into queue, check queue first value if 1 then forward, 4 then right, 3 then left. reset queue. evaluate queue when 

    prerover
    preroverState1 = roverState;

    if(roverState is 

    //Create queue, store values in queue, if i have 1 then expect 4, 3. and empty
    */ 
  }

  /*
  if (roverState != preroverState) {
    
    if((!bit3)&&(!bit2)&&(!bit1)){ // idle state is 000
      speedFlag = false;
      steeringLeftFlag = false;
      steeringRightFlag = false;
    }
   else if((!bit3)&&(!bit2)&&(bit1)){ // forward state is 001
      speedFlag = true;
      steeringLeftFlag = false;
      steeringRightFlag = false;
    }
   else if((!bit3)&&(bit2)&&(!bit1)){ // backward state is 010

      roverState1 = 0;
      roverState2 = 1;
      roverState3 = 0;
      roverState4 = 0;
      roverState5 = 0;

    }
    else if((!bit3)&&(bit2)&&(bit1)){ // left state is 011
      steeringLeftFlag = true; //so steer in position
      steeringRightFlag = false;
      speedFlag = false;
    }
    else if((bit3)&&(!bit2)&&(!bit1)){ // right state is 100
      steeringRightFlag = true; 
      steeringLeftFlag = false;
      speedFlag = false;
    }
   
    
  }
  */

  preroverState = roverState;

  prebit1 = bit1;
  prebit2 = bit2;
  prebit3 = bit3;
  
  unsigned long now = micros();
  unsigned long nowMillis = millis();

  //if (buttonState1 == HIGH && prevbuttonState1 == HIGH && ref_velocity < 0.0001) { //so steer if only the buttonState1 is HIGH
    
//  if (buttonState1 == HIGH && prevbuttonState1 == HIGH && ref_steering < 0.01) {
//    steeringFlag = true; 
//  }
//  
//  if (buttonState2 == HIGH && prevbuttonState2 == LOW) {
//    speedFlag = true;
//  }

//  prevbuttonState1 = buttonState1;
//  prevbuttonState2 = buttonState2; 

  /*
  if(nowMillis > (5000*n)) { 
    steeringFlag = true;
    n++; 
  }
  */
  
  updateVelocity(now);

  //Serial.print("LOOP FUNC!");
  
  updateControl(now);
  #ifdef LOGGING_ENABLED
    log(now);
  #endif

  #ifdef COUNT_LOOP
    static unsigned long last_ts = micros();
    static unsigned long  counter = 0;

    counter++;
    if (now - last_ts >= 1000000) {
      Serial.println(counter);
      counter = 0;
      last_ts = now;
    }
  #endif

  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n') {
        continue;
    } else if (c == '\r') {
        handle_packet(packet, packet_size);
        packet_size = 0;
    } else {
        packet[packet_size++] = (uint8_t) c;
    }
  }

  if(steeringLeftFlag == true) {
    ref_steering = 0.15;
    steeringLeftFlag = false;
    steerDirection = -1;
  }

  if(steeringRightFlag == true) {
    ref_steering = 0.15;
    steeringLeftFlag = false;
    steerDirection = 1;
  }

  if(speedFlag == true) {
      ref_velocity = 40.0;
      //targetVelocity = 10.0;
      speedFlag = false;
  }
  
  static const float a = 0.99;
  targetVelocity = a * targetVelocity + (1.0 - a) * ref_velocity;
  ref_velocity = a * ref_velocity;
  
  velocityPID.setTarget(targetVelocity);

  steering = a * steering + (1.0 - a) * ref_steering;
  ref_steering = a * ref_steering;

}

/**
 * Stepper control interrupt handler
 */
ISR(TIMER1_COMPA_vect) {
  if (currentTickLeft >= ticksPerPulseLeft) {
    currentTickLeft = 0;
  }

  if (currentTickLeft == 0) {
    PORTD |= _BV(PD7); // digitalWrite(MOT_B_STEP, HIGH);
  } else if (currentTickLeft == PULSE_WIDTH) {
    PORTD &= ~_BV(PD7); // digitalWrite(MOT_B_STEP, LOW);
  }
  
  currentTickLeft++;

  if (currentTickRight >= ticksPerPulseRight) {
    currentTickRight = 0;
  }

  if (currentTickRight == 0) {
    PORTB |= _BV(PB1); // digitalWrite(MOT_A_STEP, HIGH);
  } else if (currentTickRight == PULSE_WIDTH) {
    PORTB &= ~_BV(PB1); // digitalWrite(MOT_A_STEP, LOW);    
  }
  
  currentTickRight++;
}

void send_float_array(float *a, uint8_t size) {
    for (int i = 0; i < size; i++) {
        Serial.print((long)(a[i] * DIVISOR));
        if (i < size - 1) {
            Serial.print(';');
        }
    }
    Serial.print("\r\n");
}

void parse_float_array(char *p, uint8_t p_size, float *dest) {
    char buf[16];
    long value;
    uint8_t buf_size = 0;
    uint8_t index = 0;
    for (uint8_t i = 0; i < p_size; i++) {
        if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-') {
            buf[buf_size++] = p[i];
        } else if (p[i] == ';') {
            buf[buf_size] = '\0';
            buf_size = 0;
            value = atol(buf);
            dest[index++] = ((float)value) / DIVISOR;
        }
    }
    buf[buf_size] = '\0';
    value = atol(buf);
    dest[index] = ((float)value) / DIVISOR;
}

void parse_settings(char *p, uint8_t p_size) {
    parse_float_array(p, p_size, pid_settings);
    anglePID.setSettings(pid_settings[0], pid_settings[1], pid_settings[2]);
    velocityPID.setSettings(pid_settings[3], pid_settings[4], pid_settings[5]);
}

void parse_control(char *p, uint8_t p_size) {
    parse_float_array(p, p_size, joystick);
    ref_velocity = joystick[0];
    ref_steering = joystick[1];
}

void handle_packet(char *p, uint8_t p_size) {
    switch (p[0]) {
        case 'r':
            send_float_array(pid_settings, 6);
            break;
        case 's':
            parse_settings(&p[1], p_size - 1);
            send_float_array(pid_settings, 6);
            break;
        case 'c':
            parse_control(&p[1], p_size - 1);
            // send_float_array(joystick, 2);
            break;
    }
}
