#include <Wire.h>
#include <VL53L0X.h>
#include <L293D.h>
#include <LedControl.h>
#include <PID_v1.h>
#include <PCF8574.h>

#define MAX 0.6

#define pi 3.1415926535897932384626433832795

PCF8574 expander;

VL53L0X sensor1;    // sensors
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;
VL53L0X sensor6;
VL53L0X sensor7;
VL53L0X sensor8;
VL53L0X sensor9;
VL53L0X sensor10;
VL53L0X sensor11;
VL53L0X sensor12;

#define nwf 1
#define fwn 2

int r[13];
int a[13];
int b[13];
int c[13];
int i[13];
int m[13];
int x[13];
int y[13];

double sensor_angle[13];
double sensor_distance[13];

double sensor_orientation[13];

double x_mapping[13];
double y_mapping[13];

L293D motor_left(11, 12, A0);    // motor driver
L293D motor_right(10, 8, 9);

double power_left = MAX;
double power_right = MAX;

int forward, left, right, finish;
int last_forward, last_left, last_right, wait;

#define LEFT 1    // wheel encoders
#define RIGHT 0

long coder[2] = {
  0, 0
};
int lastSpeed[2] = {
  0, 0
};

int coder_left_a, coder_right_a, coder_left_d, coder_right_d;

int left_speed, right_speed;
double angle;
double distance;

double x1, y1, x0, y0;

double Setpoint, Input, Output;    // PID controller

double power;

PID myPID(&Input, &Output, &Setpoint, 0.05, 0.01, 0.001,P_ON_M, DIRECT);

int q, ct;

double angle1, angle2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  expander.begin(0x20);

  expander.pinMode(0, OUTPUT);
  expander.pinMode(1, OUTPUT);
  expander.pinMode(2, OUTPUT);
  expander.pinMode(3, OUTPUT);
  expander.pinMode(4, OUTPUT);
  expander.pinMode(5, OUTPUT);
  expander.pinMode(6, OUTPUT);
  expander.pinMode(7, OUTPUT);

  pinMode(4, OUTPUT);    // sensors
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  expander.digitalWrite(0, LOW);
  expander.digitalWrite(1, LOW);
  expander.digitalWrite(2, LOW);
  expander.digitalWrite(3, LOW);
  expander.digitalWrite(4, LOW);
  expander.digitalWrite(5, LOW);
  expander.digitalWrite(6, LOW);
  expander.digitalWrite(7, LOW);

  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  expander.digitalWrite(0, HIGH);
  Serial.println("sensor1");
  sensor1.init();

  delay(100);

  sensor1.setAddress((uint8_t)1);

  expander.digitalWrite(1, HIGH);
  Serial.println("sensor2");
  sensor2.init();

  delay(100);

  sensor2.setAddress((uint8_t)2);

  expander.digitalWrite(2, HIGH);
  Serial.println("sensor3");
  sensor3.init();

  delay(100);

  sensor3.setAddress((uint8_t)3);

  expander.digitalWrite(3, HIGH);
  Serial.println("sensor4");
  sensor4.init();

  delay(100);

  sensor4.setAddress((uint8_t)4);

  expander.digitalWrite(4, HIGH);
  Serial.println("sensor5");
  sensor5.init();

  delay(100);

  sensor5.setAddress((uint8_t)5);

  expander.digitalWrite(5, HIGH);
  Serial.println("sensor6");
  sensor6.init();

  delay(100);

  sensor6.setAddress((uint8_t)6);
  
  expander.digitalWrite(6, HIGH);
  Serial.println("sensor7");
  sensor7.init();

  delay(100);

  sensor7.setAddress((uint8_t)7);

  expander.digitalWrite(7, HIGH);
  Serial.println("sensor8");
  sensor8.init();

  delay(100);

  sensor8.setAddress((uint8_t)8);

  digitalWrite(4, HIGH);
  Serial.println("sensor9");
  sensor9.init();

  delay(100);

  sensor9.setAddress((uint8_t)9);

  digitalWrite(5, HIGH);
  Serial.println("sensor10");
  sensor10.init();

  delay(100);

  sensor10.setAddress((uint8_t)10);

  digitalWrite(6, HIGH);
  Serial.println("sensor11");
  sensor11.init();

  delay(100);

  sensor11.setAddress((uint8_t)11);

  digitalWrite(7, HIGH);
  Serial.println("sensor12");
  sensor12.init();

  delay(100);

  sensor12.setAddress((uint8_t)12);

  sensor1.startContinuous(100);
  sensor2.startContinuous(100);
  sensor3.startContinuous(100);
  sensor4.startContinuous(100);
  sensor5.startContinuous(100);
  sensor6.startContinuous(100);
  sensor7.startContinuous(100);
  sensor8.startContinuous(100);
  sensor9.startContinuous(100);
  sensor10.startContinuous(100);
  sensor11.startContinuous(100);
  sensor12.startContinuous(100);

  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    // wheel encoders
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);

  myPID.SetOutputLimits(0, 2 * MAX);
  
  Input = coder[LEFT] - coder[RIGHT];
  Setpoint = 0;
  Output = MAX;
  
  myPID.SetMode(AUTOMATIC);

  sensor_angle[1] = 10;
  sensor_distance[1] = 65;
  sensor_angle[2] = -10;
  sensor_distance[2] = 65;
  sensor_angle[3] = 35;
  sensor_distance[3] = 50;
  sensor_angle[4] = -35;
  sensor_distance[4] = 50;
  sensor_angle[5] = 90;
  sensor_distance[5] = 25;
  sensor_angle[6] = -90;
  sensor_distance[6] = 25;
  sensor_angle[7] = 160;
  sensor_distance[7] = 80;
  sensor_angle[8] = -160;
  sensor_distance[8] = 80;
  sensor_angle[9] = 170;
  sensor_distance[9] = 115;
  sensor_angle[10] = -170;
  sensor_distance[10] = 115;
  sensor_angle[11] = 172.5;
  sensor_distance[11] = 130;
  sensor_angle[12] = -172.5;
  sensor_distance[12] = 130;

  sensor_orientation[1] = 0;
  sensor_orientation[2] = 0;
  sensor_orientation[3] = 20;
  sensor_orientation[4] = -20;
  sensor_orientation[5] = 45;
  sensor_orientation[6] = -45;
  sensor_orientation[7] = 90;
  sensor_orientation[8] = -90;
  sensor_orientation[9] = 135;
  sensor_orientation[10] = -135;
  sensor_orientation[11] = 175;
  sensor_orientation[12] = -175;

  ct = millis();
}

void loop() {

  r[1] = sensor1.readRangeContinuousMillimeters();

  r[2] = sensor2.readRangeContinuousMillimeters();
  
  r[3] = sensor3.readRangeContinuousMillimeters();
  
  r[4] = sensor4.readRangeContinuousMillimeters();
  
  r[5] = sensor5.readRangeContinuousMillimeters();
  
  r[6] = sensor6.readRangeContinuousMillimeters();
  
  r[7] = sensor7.readRangeContinuousMillimeters();
  
  r[8] = sensor8.readRangeContinuousMillimeters();
  
  r[9] = sensor9.readRangeContinuousMillimeters();
  
  r[10] = sensor10.readRangeContinuousMillimeters();
  
  r[11] = sensor11.readRangeContinuousMillimeters();
  
  r[12] = sensor12.readRangeContinuousMillimeters();

  for(int n = 1; n<=12; n++){
    if (r[n] < 400) a[n]++;
    else a[n] = 0;
    if (a[n] > nwf) b[n] = fwn;
    if (b[n] > 0) {
      i[n] = 1;
      b[n]--;
    }
    else i[n] = 0;

    m[n] = r[n];
  }

  /*if (!i[1] && !i[2] && !i[3] && !i[4]) goForward();

  if (i[1] || i[3]) goRight();

  if (i[2] || i[4]) goLeft();*/

  static unsigned long timer = 0;                //print manager timer

  left_speed = coder[LEFT] - lastSpeed[LEFT];
  right_speed = coder[RIGHT] - lastSpeed[RIGHT];
  lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
  lastSpeed[RIGHT] = coder[RIGHT];
  timer = millis();

  Input = coder[LEFT] - coder[RIGHT];
  myPID.Compute();
  power = Output;

  if(power <= MAX) {power_left = power;
    power_right = MAX;
  }
  if(power > MAX) {power_right = 2 * MAX - power;
    power_left = MAX;
  }

  /*if(last_forward != forward || last_left != left || last_right != right) wait = 0;

  if(wait>0) {
    motor_left.set(0.0);
    motor_right.set(0.0);
    wait--;
  }
  else { if(forward) goForward();
    if(left) goLeft();
    if(right) goRight();
    }

  last_forward = forward;
  last_left = left;
  last_right = right;*/

  q = millis() - ct;

  /*if(q > 0 && q < 2000) goForward();
  if(q > 2000 && q < 5000) Wait();
  if(q > 5000 && q < 5750) goLeft();
  if(q > 5750 && q < 8750) Wait();
  if(q > 8750 && q < 10750) goForward();
  if(q > 10750 && q < 13750) Wait();
  if(q > 13750 && q < 14500) goRight();
  if(q > 14500 && q < 17500) Wait();
  if(q > 17500 && q < 19500) goForward();
  if(q > 19500) Stop();

  angle = angle + 4.39 * (left - right) * (left_speed + right_speed) / 2;

  if(forward) distance = 5.1 * (coder[LEFT] - coder_left_d + coder[RIGHT] - coder_right_d) / 2;
  else {
    distance = 0;
    }
    coder_left_d = coder[LEFT];
    coder_right_d = coder[RIGHT];

  if(q < 10000) angle1 = angle;
  if(q < 19000) angle2 = angle;*/

  angle = -40.0;

  x1 = x0 - distance * sin(angle * pi / 180);
  y1 = y0 + distance * cos(angle * pi / 180);

  x0 = x1;
  y0 = y1;
  
  for(int n = 1; n <= 4; n++) {
    x[n] = x1 - sensor_distance[n] * sin((angle + sensor_angle[n]) * pi / 180);
    y[n] = y1 + sensor_distance[n] * cos((angle + sensor_angle[n]) * pi / 180);

    x_mapping[n] = x[n] - m[n] * sin((angle + sensor_angle[n] + sensor_orientation[n]) * pi / 180);
    y_mapping[n] = y[n] + m[n] * cos((angle + sensor_angle[n] + sensor_orientation[n]) * pi / 180);

    if(r[n] > 0 && r[n] < 8190 && q < 5000){
      Serial.print(x_mapping[n]);
      Serial.print(" ");
      Serial.print(y_mapping[n]);
      Serial.print(" ");
      Serial.println(0.0);
    }
  }
  /*Serial.print(angle1);
  Serial.print(" ");
  Serial.println(angle2);*/
}

void LwheelSpeed()
{
  coder[LEFT] ++;    // count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  coder[RIGHT] ++;   // count the right wheel encoder interrupts
}

void goForward()
{
  if (!finish) {
    motor_left.set(power_left);
    motor_right.set(power_right);
    forward = 1;
    left = 0;
    right = 0;
  }
}

void goLeft()
{
  if (!finish) {
    motor_left.set(-power_left);
    motor_right.set(power_right);
    forward = 0;
    left = 1;
    right = 0;
  }
}

void goRight()
{
  if (!finish) {
    motor_left.set(power_left);
    motor_right.set(-power_right);
    forward = 0;
    left = 0;
    right = 1;
  }
}

void Wait()
{
  if (!finish) {
    motor_left.set(0.0);
    motor_right.set(0.0);
    //forward = 0;
    //left = 0;
    //right = 0;
  }
}

void Stop()
{
  motor_left.set(0.0);
  motor_right.set(0.0);
  forward = 0;
  left = 0;
  right = 0;
  finish = 1;
}
