//DFRobot.com
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
  
LiquidCrystal_I2C lcd(0x20,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display



struct I2CBoardGroup {
  int start_address;
  int number;
  String name;
};


I2CBoardGroup i2c_board_groups[] = {
  {0x20, 1,"LCD"},
  {0x40, 2,"Servo"},
  //{0x20, 1,"LCD display"},

};



void setup()
{
  Serial.begin(57600);




  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
  lcd.home();
  
  lcd.setCursor(0, 1);
  lcd.print("   Hello world    ");
  lcd.setCursor(0, 2);
  lcd.print(" arduinos!   ");
//  lcd.clear();

}




void loop()
{

  String result = "";
  for (int i = 0; i < sizeof(i2c_board_groups)/sizeof(i2c_board_groups[0]); i++) {
    for (int j = 0; j < i2c_board_groups[i].number; j++) {
      int address = i2c_board_groups[i].start_address + j;
      Wire.beginTransmission(address);
      int error = Wire.endTransmission();
      if (error != 0) {
        result += " ";
        result += String(address);
      }
    }
  }
  if (result == "") {
    lcd.setCursor(0, 2);
    lcd.print("All I2C modules connected");
  }
  else {
    lcd.setCursor(0, 2);
    lcd.print("No I2C:" + result);
  }
  

  String s = "Up time ";
  s += String(millis() / 1000);
  s += " s";

  lcd.setCursor(0, 3);
  lcd.print(s);
  delay(1000);
}

