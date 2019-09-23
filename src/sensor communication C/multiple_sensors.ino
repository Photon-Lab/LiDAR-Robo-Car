#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

int r1;
int r2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);

  delay(500);

  digitalWrite(2, HIGH);
  Serial.println("sensor1");
  sensor1.init();

  delay(100);

  sensor1.setAddress((uint8_t)01);

  digitalWrite(3, HIGH);
  Serial.println("sensor2");
  sensor2.init();

  sensor2.setAddress((uint8_t)02);

  sensor1.startContinuous();
  sensor2.startContinuous();
}

void loop() {
  r1=sensor1.readRangeContinuousMillimeters();
  Serial.print(r1);
  Serial.print(" ");

  if(r1>50){digitalWrite(6, HIGH);}
    else{digitalWrite(6, LOW);}

  r2=sensor2.readRangeContinuousMillimeters();
  Serial.print(r2);
  Serial.print(" ");

  if(r2>50){digitalWrite(7, HIGH);}
    else{digitalWrite(7, LOW);}

  delay(100);
}
