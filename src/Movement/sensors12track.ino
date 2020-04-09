#include <Wire.h>
#include <VL53L0X.h>
#include <L293D.h>
#include <LedControl.h>
#include <PID_v1.h>
#include <PCF8574.h>

PCF8574 expander;

#define N 100

#define pi 3.1415926535897932384626433832795

VL53L0X sensor1;    // SENSORS
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

#define nwf 1    // ignores erroneous 'near' readings
#define fwn 2    // ignores erroneous 'far' readings

int r[13];    // variables for interpreting sensor readings
int a[13];
int b[13];
int c[13];
int d[13];
int e[13];
int f[13];
int i[13];
int j[13];
int k[13];
int l[13];

L293D motor_left(11, 12, A0);    // MOTOR DRIVER
L293D motor_right(10, 8, 9);

#define MAX 0.6    // maximum motor power
#define MIN 0.3 
double power = MAX;
double power_left = MAX;
double power_right = MAX;

#define LEFT 1    // WHEEL ENCODERS
#define RIGHT 0

long coder[2] = {
  0, 0
};
int lastSpeed[2] = {
  0, 0
};

int left_speed, right_speed;

double Setpoint, Input, Output;    // PID CONTROLLER

PID myPID(&Input, &Output, &Setpoint, 0.001, 0, 0.0001, P_ON_M, DIRECT);

int DIN = A1;    // LED MATRIX
int CS =  A2;
int CLK = A3;

byte three[8] = {0x3c,0x04,0x04,0x3c,0x04,0x04,0x3c,0x00};
byte two[8] = {0x3c,0x04,0x04,0x3c,0x20,0x20,0x3c,0x00};
byte one[8] = {0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00};
byte x[8] = {0xff,0x81,0xbd,0xa5,0xa5,0xbd,0x81,0xff};

LedControl lc = LedControl(DIN, CLK, CS, 0);

int error;

int r5, r6;

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

  sensor1.setAddress((uint8_t)1);

  expander.digitalWrite(1, HIGH);
  Serial.println("sensor2");
  sensor2.init();

  sensor2.setAddress((uint8_t)2);

  expander.digitalWrite(2, HIGH);
  Serial.println("sensor3");
  sensor3.init();

  sensor3.setAddress((uint8_t)3);

  expander.digitalWrite(3, HIGH);
  Serial.println("sensor4");
  sensor4.init();

  sensor4.setAddress((uint8_t)4);

  expander.digitalWrite(4, HIGH);
  Serial.println("sensor5");
  sensor5.init();

  sensor5.setAddress((uint8_t)5);

  expander.digitalWrite(5, HIGH);
  Serial.println("sensor6");
  sensor6.init();

  sensor6.setAddress((uint8_t)6);
  
  expander.digitalWrite(6, HIGH);
  Serial.println("sensor7");
  sensor7.init();

  sensor7.setAddress((uint8_t)7);

  expander.digitalWrite(7, HIGH);
  Serial.println("sensor8");
  sensor8.init();

  sensor8.setAddress((uint8_t)8);

  digitalWrite(4, HIGH);
  Serial.println("sensor9");
  sensor9.init();

  sensor9.setAddress((uint8_t)9);

  digitalWrite(5, HIGH);
  Serial.println("sensor10");
  sensor10.init();

  sensor10.setAddress((uint8_t)10);

  digitalWrite(6, HIGH);
  Serial.println("sensor11");
  sensor11.init();

  sensor11.setAddress((uint8_t)11);

  digitalWrite(7, HIGH);
  Serial.println("sensor12");
  sensor12.init();

  sensor12.setAddress((uint8_t)12);

  sensor1.startContinuous(N);
  sensor2.startContinuous(N);
  sensor3.startContinuous(N);
  sensor4.startContinuous(N);
  sensor5.startContinuous(N);
  sensor6.startContinuous(N);
  sensor7.startContinuous(N);
  sensor8.startContinuous(N);
  sensor9.startContinuous(N);
  sensor10.startContinuous(N);
  sensor11.startContinuous(N);
  sensor12.startContinuous(N);

  attachInterrupt(digitalPinToInterrupt(3), LwheelSpeed, CHANGE);    // wheel encoders
  attachInterrupt(digitalPinToInterrupt(2), RwheelSpeed, CHANGE);

  myPID.SetOutputLimits(-MAX, MAX);
  //myPID.SetOutputLimits(MIN, 2.0 - MIN);
  
  Input = r[5] - r[6];
  Setpoint = 0;
  Output = 0;
  
  myPID.SetMode(AUTOMATIC);

  lc.shutdown(0, false);   // LED matrix
  lc.setIntensity(0, 15);
  lc.clearDisplay(0);

  printByte(three);
  delay(1000);
  printByte(two);
  delay(1000);
  printByte(one);
  delay(1000);
  lc.clearDisplay(0);
}

void loop() {

  r[1] = sensor1.readRangeContinuousMillimeters();

  r[2] = sensor2.readRangeContinuousMillimeters();

  r[3] = sensor3.readRangeContinuousMillimeters();

  r[4] = sensor4.readRangeContinuousMillimeters();

  r5 = sensor5.readRangeContinuousMillimeters();

  r6 = sensor6.readRangeContinuousMillimeters();

  r[7] = sensor7.readRangeContinuousMillimeters();

  r[8] = sensor8.readRangeContinuousMillimeters();
  
  r[9] = sensor9.readRangeContinuousMillimeters();
  
  r[10] = sensor10.readRangeContinuousMillimeters();
  
  r[11] = sensor11.readRangeContinuousMillimeters();
  
  r[12] = sensor12.readRangeContinuousMillimeters();

  if(r5 > 1000) r[5] = 1000;
    else r[5] = r5;
  if(r6 > 1000) r[6] = 1000;
    else r[6] = r6;

  for(int n = 1; n<=12; n++){
    if (r[n] < 300 && r[n] > 0) a[n]++;
    else a[n] = 0;
    if (a[n] > nwf) b[n] = fwn;
    if (b[n] > 0) {
      i[n] = 1;
      b[n]--;
    }
    else i[n] = 0;
  
    if (r[n] >= 300 && r[n] < 500) c[n]++;
    else c[n] = 0;
    if (c[n] > nwf) d[n] = fwn;
    if (d[n] > 0) {
      j[n] = 1;
      d[n]--;
    }
    else j[n] = 0;
  
    if (r[n] >= 500 && r[n] < 700) e[n]++;
    else e[n] = 0;
    if (e[n] > nwf) f[n] = fwn;
    if (f[n] > 0) {
      k[n] = 1;
      f[n]--;
    }
    else k[n] = 0;

    l[n] = !i[n] && !j[n] && !k[n];
  }

  /*if (r[1] == -1 || r[2] == -1 || r[3] == -1 || r[4] == -1) {
    lc.setRow(0, 0, B11111111);
    lc.setRow(0, 1, B10000001);
    lc.setRow(0, 2, B10111101);
    lc.setRow(0, 3, B10100101);
    lc.setRow(0, 4, B10100101);
    lc.setRow(0, 5, B10111101);
    lc.setRow(0, 6, B10000001);
    lc.setRow(0, 7, B11111111);
    error = 1;
  }

  if (!error) {
    if (i[1]) {
      lc.setLed(0, 0, 3, 0);
      lc.setLed(0, 1, 3, 0);
      lc.setLed(0, 2, 3, 1);
    }

    if (j[1]) {
      lc.setLed(0, 0, 3, 0);
      lc.setLed(0, 1, 3, 1);
      lc.setLed(0, 2, 3, 0);
    }

    if (k[1]) {
      lc.setLed(0, 0, 3, 1);
      lc.setLed(0, 1, 3, 0);
      lc.setLed(0, 2, 3, 0);
    }

    if (!i[1] && !j[1] && !k[1]) {
      lc.setLed(0, 0, 3, 0);
      lc.setLed(0, 1, 3, 0);
      lc.setLed(0, 2, 3, 0);
    }

    if (i[2]) {
      lc.setLed(0, 0, 4, 0);
      lc.setLed(0, 1, 4, 0);
      lc.setLed(0, 2, 4, 1);
    }

    if (j[2]) {
      lc.setLed(0, 0, 4, 0);
      lc.setLed(0, 1, 4, 1);
      lc.setLed(0, 2, 4, 0);
    }

    if (k[2]) {
      lc.setLed(0, 0, 4, 1);
      lc.setLed(0, 1, 4, 0);
      lc.setLed(0, 2, 4, 0);
    }

    if (!i[2] && !j[2] && !k[2]) {
      lc.setLed(0, 0, 4, 0);
      lc.setLed(0, 1, 4, 0);
      lc.setLed(0, 2, 4, 0);
    }

    if (i[3]) {
      lc.setLed(0, 0, 1, 0);
      lc.setLed(0, 1, 2, 0);
    }

    if (j[3]) {
      lc.setLed(0, 0, 1, 0);
      lc.setLed(0, 1, 2, 1);
    }

    if (k[3]) {
      lc.setLed(0, 0, 1, 1);
      lc.setLed(0, 1, 2, 0);
    }

    if (!i[3] && !j[3] && !k[3]) {
      lc.setLed(0, 0, 1, 0);
      lc.setLed(0, 1, 2, 0);
    }

    if (i[4]) {
      lc.setLed(0, 0, 6, 0);
      lc.setLed(0, 1, 5, 0);
    }

    if (j[4]) {
      lc.setLed(0, 0, 6, 0);
      lc.setLed(0, 1, 5, 1);
    }

    if (k[4]) {
      lc.setLed(0, 0, 6, 1);
      lc.setLed(0, 1, 5, 0);
    }

    if (!i[4] && !j[4] && !k[4]) {
      lc.setLed(0, 0, 6, 0);
      lc.setLed(0, 1, 5, 0);
    }

    if (i[5]) {
      lc.setLed(0, 0, 0, 0);
      lc.setLed(0, 1, 0, 0);
      lc.setLed(0, 1, 1, 0);
      lc.setLed(0, 2, 1, 0);
    }

    if (j[5]) {
      lc.setLed(0, 0, 0, 0);
      lc.setLed(0, 1, 0, 0);
      lc.setLed(0, 1, 1, 1);
      lc.setLed(0, 2, 1, 1);
    }

    if (k[5]) {
      lc.setLed(0, 0, 0, 1);
      lc.setLed(0, 1, 0, 1);
      lc.setLed(0, 1, 1, 0);
      lc.setLed(0, 2, 1, 0);
    }

    if (!i[5] && !j[5] && !k[5]) {
      lc.setLed(0, 0, 0, 0);
      lc.setLed(0, 1, 0, 0);
      lc.setLed(0, 1, 1, 0);
      lc.setLed(0, 2, 1, 0);
    }

    if (i[6]) {
      lc.setLed(0, 0, 7, 0);
      lc.setLed(0, 1, 7, 0);
      lc.setLed(0, 1, 6, 0);
      lc.setLed(0, 2, 6, 0);
    }

    if (j[6]) {
      lc.setLed(0, 0, 7, 0);
      lc.setLed(0, 1, 7, 0);
      lc.setLed(0, 1, 6, 1);
      lc.setLed(0, 2, 6, 1);
    }

    if (k[6]) {
      lc.setLed(0, 0, 7, 1);
      lc.setLed(0, 1, 7, 1);
      lc.setLed(0, 1, 6, 0);
      lc.setLed(0, 2, 6, 0);
    }

    if (!i[6] && !j[6] && !k[6]) {
      lc.setLed(0, 0, 7, 0);
      lc.setLed(0, 1, 7, 0);
      lc.setLed(0, 1, 6, 0);
      lc.setLed(0, 2, 6, 0);
    }

    if (i[7]) {
      lc.setLed(0, 4, 0, 0);
      lc.setLed(0, 5, 0, 0);
      lc.setLed(0, 4, 1, 0);
      lc.setLed(0, 4, 2, 1);
    }

    if (j[7]) {
      lc.setLed(0, 4, 0, 0);
      lc.setLed(0, 5, 0, 0);
      lc.setLed(0, 4, 1, 1);
      lc.setLed(0, 4, 2, 0);
    }

    if (k[7]) {
      lc.setLed(0, 4, 0, 1);
      lc.setLed(0, 5, 0, 1);
      lc.setLed(0, 4, 1, 0);
      lc.setLed(0, 4, 2, 0);
    }

    if (!i[7] && !j[7] && !k[7]) {
      lc.setLed(0, 4, 0, 0);
      lc.setLed(0, 5, 0, 0);
      lc.setLed(0, 4, 1, 0);
      lc.setLed(0, 4, 2, 0);
    }

    if (i[8]) {
      lc.setLed(0, 4, 7, 0);
      lc.setLed(0, 5, 7, 0);
      lc.setLed(0, 4, 6, 0);
      lc.setLed(0, 4, 5, 1);
    }

    if (j[8]) {
      lc.setLed(0, 4, 7, 0);
      lc.setLed(0, 5, 7, 0);
      lc.setLed(0, 4, 6, 1);
      lc.setLed(0, 4, 5, 0);
    }

    if (k[8]) {
      lc.setLed(0, 4, 7, 1);
      lc.setLed(0, 5, 7, 1);
      lc.setLed(0, 4, 6, 0);
      lc.setLed(0, 4, 5, 0);
    }

    if (!i[8] && !j[8] && !k[8]) {
      lc.setLed(0, 4, 7, 0);
      lc.setLed(0, 5, 7, 0);
      lc.setLed(0, 4, 6, 0);
      lc.setLed(0, 4, 5, 0);
    }

    if (i[9]) {
      lc.setLed(0, 6, 0, 0);
      lc.setLed(0, 7, 0, 0);
      lc.setLed(0, 6, 1, 0);
      lc.setLed(0, 5, 2, 1);
    }

    if (j[9]) {
      lc.setLed(0, 6, 0, 0);
      lc.setLed(0, 7, 0, 0);
      lc.setLed(0, 6, 1, 1);
      lc.setLed(0, 5, 2, 0);
    }

    if (k[9]) {
      lc.setLed(0, 6, 0, 1);
      lc.setLed(0, 7, 0, 1);
      lc.setLed(0, 6, 1, 0);
      lc.setLed(0, 5, 2, 0);
    }

    if (!i[9] && !j[9] && !k[9]) {
      lc.setLed(0, 6, 0, 0);
      lc.setLed(0, 7, 0, 0);
      lc.setLed(0, 6, 1, 0);
      lc.setLed(0, 5, 2, 0);
    }

    if (i[10]) {
      lc.setLed(0, 6, 7, 0);
      lc.setLed(0, 7, 7, 0);
      lc.setLed(0, 6, 6, 0);
      lc.setLed(0, 5, 5, 1);
    }

    if (j[10]) {
      lc.setLed(0, 6, 7, 0);
      lc.setLed(0, 7, 7, 0);
      lc.setLed(0, 6, 6, 1);
      lc.setLed(0, 5, 5, 0);
    }

    if (k[10]) {
      lc.setLed(0, 6, 7, 1);
      lc.setLed(0, 7, 7, 1);
      lc.setLed(0, 6, 6, 0);
      lc.setLed(0, 5, 5, 0);
    }

    if (!i[10] && !j[10] && !k[10]) {
      lc.setLed(0, 6, 7, 0);
      lc.setLed(0, 7, 7, 0);
      lc.setLed(0, 6, 6, 0);
      lc.setLed(0, 5, 5, 0);
    }

    if (i[11]) {
      lc.setLed(0, 7, 1, 0);
      lc.setLed(0, 7, 2, 0);
      lc.setLed(0, 7, 3, 0);
      lc.setLed(0, 6, 2, 0);
      lc.setLed(0, 6, 3, 0);
      lc.setLed(0, 5, 3, 1);
    }

    if (j[11]) {
      lc.setLed(0, 7, 1, 0);
      lc.setLed(0, 7, 2, 0);
      lc.setLed(0, 7, 3, 0);
      lc.setLed(0, 6, 2, 1);
      lc.setLed(0, 6, 3, 1);
      lc.setLed(0, 5, 3, 0);
    }

    if (k[11]) {
      lc.setLed(0, 7, 1, 1);
      lc.setLed(0, 7, 2, 1);
      lc.setLed(0, 7, 3, 1);
      lc.setLed(0, 6, 2, 0);
      lc.setLed(0, 6, 3, 0);
      lc.setLed(0, 5, 3, 0);
    }

    if (!i[11] && !j[11] && !k[11]) {
      lc.setLed(0, 7, 1, 0);
      lc.setLed(0, 7, 2, 0);
      lc.setLed(0, 7, 3, 0);
      lc.setLed(0, 6, 2, 0);
      lc.setLed(0, 6, 3, 0);
      lc.setLed(0, 5, 3, 0);
    }

    if (i[12]) {
      lc.setLed(0, 7, 6, 0);
      lc.setLed(0, 7, 5, 0);
      lc.setLed(0, 7, 4, 0);
      lc.setLed(0, 6, 5, 0);
      lc.setLed(0, 6, 4, 0);
      lc.setLed(0, 5, 4, 1);
    }

    if (j[12]) {
      lc.setLed(0, 7, 6, 0);
      lc.setLed(0, 7, 5, 0);
      lc.setLed(0, 7, 4, 0);
      lc.setLed(0, 6, 5, 1);
      lc.setLed(0, 6, 4, 1);
      lc.setLed(0, 5, 4, 0);
    }

    if (k[12]) {
      lc.setLed(0, 7, 6, 1);
      lc.setLed(0, 7, 5, 1);
      lc.setLed(0, 7, 4, 1);
      lc.setLed(0, 6, 5, 0);
      lc.setLed(0, 6, 4, 0);
      lc.setLed(0, 5, 4, 0);
    }

    if (!i[12] && !j[12] && !k[12]) {
      lc.setLed(0, 7, 6, 0);
      lc.setLed(0, 7, 5, 0);
      lc.setLed(0, 7, 4, 0);
      lc.setLed(0, 6, 5, 0);
      lc.setLed(0, 6, 4, 0);
      lc.setLed(0, 5, 4, 0);
    }

    if(k[1] || k[3]) lc.setLed(0, 0, 2, 1);
    else lc.setLed(0, 0, 2, 0);
    
    if(k[2] || k[4]) lc.setLed(0, 0, 5, 1);
    else lc.setLed(0, 0, 5, 0);
    
    if(i[3] || i[5]) lc.setLed(0, 2, 2, 1);
    else lc.setLed(0, 2, 2, 0);
    
    if(i[4] || i[6]) lc.setLed(0, 2, 5, 1);
    else lc.setLed(0, 2, 5, 0);

    if(j[7] || j[9]) lc.setLed(0, 5, 1, 1);
    else lc.setLed(0, 5, 1, 0);

    if(j[8] || j[10]) lc.setLed(0, 5, 6, 1);
    else lc.setLed(0, 5, 6, 0);
  }

  if (!l[1] && !l[2] && !l[3] && !l[4] && !l[5] && !l[6] && !l[7] && !l[8] && !l[9] && !l[10] && !l[11] && !l[12]) lc.clearDisplay(0);*/

static unsigned long timer = 0;                //print manager timer

  left_speed = coder[LEFT] - lastSpeed[LEFT];
  right_speed = coder[RIGHT] - lastSpeed[RIGHT];
  lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
  lastSpeed[RIGHT] = coder[RIGHT];

  Input = r[5] - r[6];
  myPID.Compute();
  //if(r[5] < 50 && r[6] < 50) power = Output;
  //else power = 0.0;
  power = Output;

  if(power <= 0) {power_left = MAX + power;
    power_right = MAX;
  }
  if(power > 0) {power_right = MAX - power;
    power_left = MAX;
  }

  //motor_left.set(power_left);
  //motor_right.set(0.79 * power_right);

  //motor_left.set(0.3);
  //motor_right.set(0.3);

  if((r[5] > 500) && (r[6] > 500)){
    power_left = MAX;
    power_right = MAX;
    }

  if((r[1] < r[2]) && (r[1] < 300)){
    power_left = MAX;
    power_right = 0;
    }

  if((r[1] > r[2]) && (r[2] < 300)){
    power_left = 0;
    power_right = MAX;
    }

  motor_left.set(power_left);
  motor_right.set(power_right);

  /*if(finish){
    lc.setRow(0, 0, B00011000);
    lc.setRow(0, 1, B00100100);
    lc.setRow(0, 2, B00100000);
    lc.setRow(0, 3, B00010000);
    lc.setRow(0, 4, B00001000);
    lc.setRow(0, 5, B00000100);
    lc.setRow(0, 6, B00100100);
    lc.setRow(0, 7, B00011000);
  }*/

  if(timer < millis()){
    /*Serial.print(r[1]);
    Serial.print(" ");
    Serial.print(r[2]);
    Serial.print(" ");
    Serial.print(r[3]);
    Serial.print(" ");
    Serial.print(r[4]);
    Serial.print(" ");*/
    Serial.print("Coder value: ");
    Serial.print(left_speed);
    //Serial.print(coder[LEFT]);
    Serial.print("[Left Wheel] ");
    Serial.print(right_speed);
    //Serial.print(coder[RIGHT]);
    Serial.print("[Right Wheel]");
    Serial.print(" ");
    Serial.print(r[5]);
    Serial.print(" ");
    Serial.print(r[6]);
    Serial.print(" ");
    Serial.print(Output);
    Serial.println();
    /*Serial.print(" ");
    Serial.print(r[7]);
    Serial.print(" ");
    Serial.print(r[8]);
    Serial.print(" ");
    Serial.print(r[9]);
    Serial.print(" ");
    Serial.print(r[10]);
    Serial.print(" ");
    Serial.print(r[11]);
    Serial.print(" ");
    Serial.print(r[12]);
    Serial.println(" ");*/
    timer = millis() + 1000;
    }
}

void LwheelSpeed()
{
  coder[LEFT] ++;    // count the left wheel encoder interrupts
}

void RwheelSpeed()
{
  coder[RIGHT] ++;   // count the right wheel encoder interrupts
}

void printByte(byte character [])
{
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    lc.setRow(0, i, character[i]);
  }
}
