#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

int result;

void setup() {
  {
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  sensor.startContinuous(100);

  pinMode(4, OUTPUT);
}

}

void loop() {
  result = sensor.readRangeContinuousMillimeters();
  Serial.print(result);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();

  Serial.print(result);
  
  if(result>=50) {digitalWrite(4, HIGH);}
    else {digitalWrite(4, LOW);}
}
