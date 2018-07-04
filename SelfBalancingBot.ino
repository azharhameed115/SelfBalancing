#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
int l1 = 6, l2 = 7, r1 = 5, r2 = 4;//Motor Pins IN1, IN2, IN3, IN4 from L298N Driver
double kp = 20, kd = 0.014, ki = 28;
float sampleTime = 0.005, targetAngle = - 0.5;
MPU6050 mpu; int in;
int16_t accY, accZ, gyroX;
volatile int mSpeed, gRate;
volatile float aAngle, gAngle, presAngle, prevAngle = 0, error, errorSum = 0;
void motors(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed >= 0) {
    analogWrite(l1, leftMotorSpeed);
    digitalWrite(l2, LOW);
  }
  else {
    analogWrite(l1, 255 + leftMotorSpeed);
    digitalWrite(l2, HIGH);
  }
  if (rightMotorSpeed >= 0) {
    analogWrite(r1, rightMotorSpeed);
    digitalWrite(r2, LOW);
  }
  else {
    analogWrite(r1, 255 + rightMotorSpeed);
    digitalWrite(r2, HIGH);
  }
}
void init_PID() {
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = 9999;// set compare match register to set sample time 5ms
  TCCR1B |= (1 << WGM12);// turn on CTC mode
  TCCR1B |= (1 << CS11);// Set CS11 bit for prescaling by 8
  TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  init_PID();
  in=0;
}
void loop() {
  if (in == 1)
  {
    accY = mpu.getAccelerationY();
    accZ = mpu.getAccelerationZ();
    gyroX = mpu.getRotationX();
    mSpeed = constrain(mSpeed, -255, 255);
    motors(mSpeed, mSpeed);
  }
  if (in == 0)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
  }
}
ISR(TIMER1_COMPA_vect) {
  if (Serial.read() == 0)
    in = 0;
  if (Serial.read() == 1)
    in = 1;
  aAngle = (atan2(accY, accZ) * RAD_TO_DEG) - 18;
  gRate = map(gyroX, -32768, 32767, -250, 250);
  gAngle = (float)gRate * sampleTime;
  presAngle = 0.9934 * (prevAngle + gAngle) + 0.0066 * aAngle;
  error = presAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  mSpeed = kp * (error) + ki * (errorSum) * sampleTime - kd * (presAngle - prevAngle) / sampleTime;
  prevAngle = presAngle;
}
