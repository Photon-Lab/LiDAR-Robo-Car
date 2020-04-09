#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

int result;
int motor;

void setup() {
  {
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  sensor.startContinuous(100);

  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  pinMode(6, OUTPUT);
}

}

void loop() {
  result = sensor.readRangeContinuousMillimeters();
  //Serial.print(result);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  
  if(result<=1200) {digitalWrite(4, HIGH);}
    else {digitalWrite(4, LOW);}

  motor = digitalRead(5);
  digitalWrite(6, motor);
  digitalWrite(7, LOW);

  Serial.print(motor);
}
