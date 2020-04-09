#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

int r1;
int r2;
int r3;
int r4;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  delay(500);

  digitalWrite(2, HIGH);
  Serial.println("sensor1");
  sensor1.init();

  delay(100);

  sensor1.setAddress((uint8_t)01);

  digitalWrite(3, HIGH);
  Serial.println("sensor2");
  sensor2.init();

  delay(100);

  sensor2.setAddress((uint8_t)02);

  digitalWrite(4, HIGH);
  Serial.println("sensor3");
  sensor3.init();

  delay(100);

  sensor3.setAddress((uint8_t)03);

  digitalWrite(5, HIGH);
  Serial.println("sensor4");
  sensor4.init();

  delay(100);

  sensor4.setAddress((uint8_t)04);

  delay(100);

  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
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

  r3=sensor3.readRangeContinuousMillimeters();
  Serial.print(r3);
  Serial.print(" ");

  if(r3>50){digitalWrite(8, HIGH);}
    else{digitalWrite(8, LOW);}

  r4=sensor4.readRangeContinuousMillimeters();
  Serial.print(r4);
  Serial.print(" ");

  if(r4>50){digitalWrite(9, HIGH);}
    else{digitalWrite(9, LOW);}

  delay(100);
}
