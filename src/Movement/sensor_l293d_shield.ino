#include <Wire.h>
#include <VL53L0X.h>
#include <AFMotor.h>

VL53L0X sensor;

int result;
int turn, turning;
int top, fast, slow;
int a, b, c, n;

AF_DCMotor motor1(4, MOTOR34_64KHZ);
AF_DCMotor motor2(3, MOTOR34_64KHZ);
AF_DCMotor motor3(1, MOTOR12_64KHZ);
AF_DCMotor motor4(2, MOTOR12_64KHZ);

void setup() {
  {
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

  sensor.startContinuous(100);

  turn = 1;
  top = 255;
  fast = 127;
  slow = 63;
  n = 15;

  motor1.setSpeed(fast);
  motor2.setSpeed(fast);
  motor3.setSpeed(fast);
  motor4.setSpeed(fast);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

}

void loop() {
  result = sensor.readRangeContinuousMillimeters();
  Serial.print(result);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();

  if(!turning){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  }
  
  if(result<=300 && turn == 1) {
    turning = 1;
  }
  
  if(turning) {
    if(a!=n && b!=2*n) {
      motor1.setSpeed(top);
      motor2.setSpeed(slow);
      motor3.setSpeed(top);
      motor4.setSpeed(slow);
      a++;
      }

   if(a==n && b!=2*n){
     motor1.setSpeed(slow);
     motor2.setSpeed(top);
     motor3.setSpeed(slow);
     motor4.setSpeed(top);
     b++;
   }

   if(a!=0 && b==2*n){
      motor1.setSpeed(top);
      motor2.setSpeed(slow);
      motor3.setSpeed(top);
      motor4.setSpeed(slow);
      a--;
    }
   
    if(a==0 && b==2*n) {
      turn = 0;
      turning = 0;
      motor1.setSpeed(fast);
      motor2.setSpeed(fast);
      motor3.setSpeed(fast);
      motor4.setSpeed(fast);
    }
  }
}
