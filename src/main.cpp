#include <Arduino.h>

// TODO:  LoRa communication codes
// TODO:  GPS reading codes
// TODO:  Compass reading codes
// TODO:  IMU reading codes
// TODO:  IMU + Compass fusion
// TODO:  Motor control codes
// TODO:  RC remote control
// TODO:  GPS guiding codes
// TODO:  GS communication codes

#define M_1_PINB GPIO_NUM_4  // Backward pin
#define M_1_PINF GPIO_NUM_16 // Forward pin
#define M_1_PINP GPIO_NUM_17 // PWM pin

#define M_2_PINB GPIO_NUM_18 // Backward pin
#define M_2_PINF GPIO_NUM_19 // Forward pin
#define M_2_PINP GPIO_NUM_21 // PWM pin

#define M_3_PINB GPIO_NUM_26 // Backward pin
#define M_3_PINF GPIO_NUM_27 // Forward pin
#define M_3_PINP GPIO_NUM_13 // PWM pin

#define M_4_PINB GPIO_NUM_32 // Backward pin
#define M_4_PINF GPIO_NUM_33 // Forward pin
#define M_4_PINP GPIO_NUM_25 // PWM pin

#define PWM_FREQ 5000
#define PWM_CHN_M1 0
#define PWM_CHN_M2 1
#define PWM_CHN_M3 2
#define PWM_CHN_M4 3
#define PWM_RES 8
#define PCT_TO_PWM(x) ((int)x * ((1 << PWM_RES) - 1) / 100) // bitlik conzunurluk ve yuzdelik hizin duty cycle degerine donusum

#define M1_INDEX 0
#define M2_INDEX 1
#define M3_INDEX 2
#define M4_INDEX 3
#define MOTORS_COUNT 4

extern void motorBackward(int motorIndex, int speed);
extern void motorForward(int motorIndex, int speed);
extern void motorStop(int motorIndex);
extern void carForward(int speed);
extern void carBackward(int speed);
extern void stopCar();

int motorFpins[MOTORS_COUNT] = {M_1_PINF,
                              M_2_PINF,
                              M_3_PINF,
                              M_4_PINF};
int motorBpins[MOTORS_COUNT] = {M_1_PINB,
                              M_2_PINB,
                              M_3_PINB,
                              M_4_PINB};
int motorPpins[MOTORS_COUNT] = {M_1_PINP,
                              M_2_PINP,
                              M_3_PINP,
                              M_4_PINP};

void setup()
{
  Serial.begin(115200);
  for (size_t i = 0; i < MOTORS_COUNT; i++)
  {
    pinMode(motorFpins[i], OUTPUT);
    pinMode(motorBpins[i], OUTPUT);
    ledcAttachPin(motorPpins[i], i);
    ledcSetup(i, PWM_FREQ, PWM_RES);
  }
    
}

void loop()
{ 

}

void motorBackward(int motorIndex, int speed)
{
  digitalWrite(motorBpins[motorIndex],HIGH);
  digitalWrite(motorFpins[motorIndex],LOW);
  ledcWrite(motorIndex, PCT_TO_PWM(speed));
}
void motorForward(int motorIndex, int speed)
{
  digitalWrite(motorBpins[motorIndex],LOW);
  digitalWrite(motorFpins[motorIndex],HIGH);
  ledcWrite(motorIndex, PCT_TO_PWM(speed));
}
void motorStop(int motorIndex)
{
  digitalWrite(motorBpins[motorIndex],HIGH);
  digitalWrite(motorFpins[motorIndex],HIGH);
  ledcWrite(motorIndex, 0);
}

void carForward(int speed)
{
  stopCar();
  for (size_t i = 0; i < MOTORS_COUNT; i++)
  {
    ledcWrite(i, PCT_TO_PWM(speed));
  }
  digitalWrite(motorBpins[M1_INDEX], LOW);
  digitalWrite(motorBpins[M2_INDEX], LOW);
  digitalWrite(motorBpins[M3_INDEX], LOW);
  digitalWrite(motorBpins[M4_INDEX], LOW);
}

void carBackward(int speed)
{
  stopCar();
  for (size_t i = 0; i < MOTORS_COUNT; i++)
  {
    ledcWrite(i, PCT_TO_PWM(speed));
  }
  digitalWrite(motorFpins[M1_INDEX], LOW);
  digitalWrite(motorFpins[M2_INDEX], LOW);
  digitalWrite(motorFpins[M3_INDEX], LOW);
  digitalWrite(motorFpins[M4_INDEX], LOW);
}

void stopCar()
{
  for (size_t i = 0; i < MOTORS_COUNT; i++)
  {
    motorStop(i);
  } 
}