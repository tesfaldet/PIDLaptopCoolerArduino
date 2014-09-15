#include <LiquidCrystal.h>

char buffer[4];
char errorByte;
int temperature = 0;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
boolean bufferComplete = false;


void setup() {
  lcd.clear();
  lcd.begin(16, 2); // set up the LCD's number of rows and columns:
  Serial.begin(9600);
}

void loop() {
  if (bufferComplete) {
    lcd.setCursor(0, 0);
    if (temperature == 58) {
    lcd.print(temperature);
    delay(500);
    lcd.clear();
    }
    bufferComplete = false;
  }
}


void serialEvent() {
  Serial.readBytes(buffer, 4);
  if (buffer[0] == (int)'a') {
    int tens = (buffer[1] - 48) * 10;
    int ones = (buffer[2] - 48);
    temperature = tens + ones;
    bufferComplete = true;
  } 
  else {
    while (Serial.available() && errorByte < 97) {
      errorByte = Serial.read();
    }
  }
}

