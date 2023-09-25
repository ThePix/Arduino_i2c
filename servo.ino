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
unsigned long previous_time;


struct SwitchData {
  int pin;      // the pin the switch is on
  int servo;    // Number of the sefvo in the list servo_configs below
                // Note that they count from zero!
  bool turnOn;  // If true, will set the servo to the "on" position
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





class Servo {
  int cluster;  // 0 - 5 or so
  int number; // 0 - 15
  int speed;
  int target_angle;
  int current_angle;
  int off_angle;
  int on_angle;

  public:
	Servo(int _cluster, int _number, int _speed, int _off_angle, int _on_angle) {
    cluster = _cluster;
    number = _number;
    speed = _speed;
    off_angle = _off_angle * 100;
    target_angle = _off_angle * 100;
    current_angle = _off_angle * 100;
    on_angle = _on_angle * 100;
  }

  void set(bool turnOn) {
    target_angle = turnOn ? on_angle : off_angle;
  }

  void adjust(int elapsed) {
    int diff = current_angle - target_angle;
    if (diff == 0) return;

    int increment = elapsed * speed;
    // diff is then capped at that
    if (diff > 0) {
      if (diff > speed) diff = increment;  // cap at speed
      current_angle -= diff;
    }
    else {
      if (diff < speed) diff = increment;  // cap at speed
      current_angle += diff;
    }
    update();    
  }

  void update() {
    int pulselen = current_angle * (SERVOMAX - SERVOMIN) / 18000 + SERVOMIN;
    if (pulselen > SERVOMAX) {
      Serial.print("ERROR: Too high!!!");
      return;
    }
    if (pulselen < SERVOMIN) {
      Serial.print("ERROR: Too low!!!");
      return;
    }
    pwms[cluster].setPWM(number, 0, pulselen);
  }

  String status() {
    return String(current_angle) + "/" + String(target_angle);
  }

  private:
};



Servo servos[] = {
  Servo(0, 0, 1, 0, 180),
  Servo(0, 1, 3, 10, 120), // this one does not like to go beyond 120deg!
  Servo(0, 3, 1, 0, 90),
  Servo(1, 1, 1, 0, 180),
  Servo(1, 2, 4, 0, 180),
  Servo(1, 3, 4, 0, 90),
};




const int servo_count = sizeof(servos)/sizeof(servos[0]);
const int switch_count = sizeof(switches)/sizeof(switches[0]);





void setup() {

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

  for (int i = 0; i < servo_count; i++) {
    servos[i].update();
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
  int increment = TIME_FACTOR * elapsed;


  // HANDLE INPUTS
  for (int i = 0; i < switch_count; i++) {
    if (digitalRead(switches[i].pin) == LOW) {
      servos[switches[i].servo].set(switches[i].turnOn);
      Serial.println("switch:" + String(i) + " (" + servos[5].status() + ")");
    }
  }


  // HANDLE SERVOS
  for (int i = 0; i < servo_count; i++) {
    servos[i].adjust(increment);
  }
 
}
