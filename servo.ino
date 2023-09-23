// Servo control via I2C bus version 0.2
// Copyright Andy Joel and Preston&District Model Railway Club
// Documentation here:
// http://www.prestonanddistrictmrs.org.uk/articles/point-control-with-servos/coding-the-arduino-for-the-n-gauge-layout/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define CONTROLLER_COUNT  2
#define CONTROLLER_START  0x40
#define TIME_FACTOR 1.0

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

Adafruit_PWMServoDriver pwms[CONTROLLER_COUNT];
int servo_count;
int switch_count;
unsigned long previous_time;


struct SwitchData {
  int pin;
  int servo;
  bool turnOn;
};

SwitchData switches[] = {
  {5, 1, true},
  {6, 1, false},
  {5, 2, true},
  {6, 2, false},
  {7, 3, true},
  {8, 3, false},
  {9, 4, true},
  {10, 4, false},
  {11, 5, true},
  {12, 5, false},
};


struct ServoData {
  int cluster;  // 0 - 5 or so
  int number; // 0 - 15
  int off_angle;   // all angles in hundredths of a degree
  int on_angle;    // use ints to make comparsons easier
  int current_angle;
  int target_angle;
  int speed;
};

ServoData servos[] = {
  {0, 0, 0, 18000, 0, 18000, 1},
  {0, 1, 1000, 12000, 12000, 1000, 3}, // this one does not like to go beyond 120deg!
  {0, 3, 0, 9000, 9000, 0, 1},
  {1, 1, 0, 18000, 0, 18000, 1},
  {1, 2, 0, 18000, 0, 18000, 4},
  {1, 3, 0, 9000, 0, 9000, 4},
};


void setAngle(ServoData servo) {
  int pulselen = servo.current_angle * (SERVOMAX - SERVOMIN) / 18000 + SERVOMIN;
  if (pulselen > SERVOMAX) {
    Serial.print("ERROR: Too high!!!");
    return;
  }
  if (pulselen < SERVOMIN) {
    Serial.print("ERROR: Too low!!!");
    return;
  }
  pwms[servo.cluster].setPWM(servo.number, 0, pulselen);
}




void setup() {
  servo_count = sizeof(servos)/sizeof(servos[0]);
  switch_count = sizeof(switches)/sizeof(switches[0]);

  Serial.begin(9600);

  for (int i = 0; i < switch_count; i++) {
    pinMode(switches[i].pin, INPUT_PULLUP);           // set pin to input
  }

  for (int i = 0; i < CONTROLLER_COUNT; i++) {
    pwms[i] = Adafruit_PWMServoDriver(CONTROLLER_START + i);
    pwms[i].begin();
    pwms[i].setOscillatorFrequency(27000000);
    pwms[i].setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  }

  delay(100);
  
  //while (!Serial); // wait for output to fire up before sending. But what if it is not connected???
  //Serial.println("Found " + String(servo_count) + " servos!");  // does not appear
  previous_time = millis();
}



void loop() {
  // HANDLE TIME
  unsigned long now_time = millis();
  unsigned long elapsed = now_time - previous_time;
  previous_time = now_time;


  // HANDLE INPUTS
  for (int i = 0; i < switch_count; i++) {
    if (digitalRead(switches[i].pin) == LOW) {
      ServoData servo = servos[switches[i].servo];
      if (switches[i].turnOn) {
        servos[switches[i].servo].target_angle = servo.on_angle;
      }
      else {
        servos[switches[i].servo].target_angle = servo.off_angle;
      }
      Serial.println("servo_count:" + String(servo_count) + " (" + String(servos[5].target_angle) + ", " + String(servos[5].current_angle) + ")");
    }
  }


  // HANDLE SERVOS
  for (int i = 0; i < servo_count; i++) {
    int diff = servos[i].current_angle - servos[i].target_angle;
    if (diff == 0) {
      //Serial.print(".");
      continue;
    }
    // increment is how much the servo can change, given its speed and the elapsed time
    // We are taking a float, a long and an int, and converting to an int, but hopefully okay!
    // The calculation will be done as a float because it starts with a float, and converted on assignment
    int increment = TIME_FACTOR * elapsed * servos[i].speed;
    // diff is then capped at that
    if (diff > 0) {
      if (diff > servos[i].speed) diff = increment;  // cap at speed
      servos[i].current_angle -= diff;
      //Serial.print("-");
    }
    else {
      if (diff < servos[i].speed) diff = increment;  // cap at speed
      servos[i].current_angle += diff;
      //Serial.print("+");
    }
    setAngle(servos[i]);
  }
 
}
