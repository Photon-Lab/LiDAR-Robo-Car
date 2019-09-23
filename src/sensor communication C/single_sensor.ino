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

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(100);

  pinMode(4, OUTPUT);
}

}

void loop() {
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();

  result = sensor.readRangeContinuousMillimeters();

  Serial.print(result);
  
  if(result>=50) {digitalWrite(4, HIGH);}
    else {digitalWrite(4, LOW);}
}
