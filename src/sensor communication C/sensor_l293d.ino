#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

int result;
int motor1;
int motor2;
int turn = 1;
int n;

void setup() {
  {
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  sensor.startContinuous(100);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  /*pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);*/
  
}

}

void loop() {
  result = sensor.readRangeContinuousMillimeters();
  Serial.print(result);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();

  motor1 = 1;
  motor2 = 1;
  n = 1000;

  if(result<=200 && turn == 1) {
    /*digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);

    delay(500);*/
           
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);

    
    /*analogWrite(A3, 100);
    analogWrite(A2, 0);
    analogWrite(A1, 100);
    analogWrite(A0, 0);*/

    delay(n);

    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);

    delay(1.1*n);

    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);

    delay(2*n);

    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);

    delay(1.1*n);

    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);

    delay(n);

    turn = 0;
    }

  digitalWrite(4, motor1);
  digitalWrite(5, LOW);
  digitalWrite(6, motor2);
  digitalWrite(7, LOW);
  digitalWrite(8, motor1);
  digitalWrite(9, LOW);
  digitalWrite(10, motor2);
  digitalWrite(11, LOW);

  /*analogWrite(A3, 255);
  analogWrite(A2, 0);
  analogWrite(A1, 255);
  analogWrite(A0, 0);*/
}
