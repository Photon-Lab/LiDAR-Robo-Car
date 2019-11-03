#include <Wire.h>
#include <VL53L0X.h>
#include <L293D.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

int r1;
int r2;
int r3;
int r4;
int r5;

L293D motor_left(10, 22, 23);
L293D motor_right(11, 52, 53);

int a, b, c, a1, b1, c1, i1, i2, i3, t, n, n4, n5;
double power;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);

  digitalWrite(24, LOW);
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
  digitalWrite(28, LOW);

  delay(500);

  digitalWrite(24, HIGH);
  Serial.println("sensor1");
  sensor1.init();

  delay(100);

  sensor1.setAddress((uint8_t)01);

  digitalWrite(25, HIGH);
  Serial.println("sensor2");
  sensor2.init();

  delay(100);

  sensor2.setAddress((uint8_t)02);

  digitalWrite(26, HIGH);
  Serial.println("sensor3");
  sensor3.init();

  delay(100);

  sensor3.setAddress((uint8_t)03);

  digitalWrite(27, HIGH);
  Serial.println("sensor4");
  sensor4.init();

  delay(100);

  sensor4.setAddress((uint8_t)04);

  digitalWrite(28, HIGH);
  Serial.println("sensor5");
  sensor5.init();

  delay(100);

  sensor1.startContinuous(100);
  sensor2.startContinuous(100);
  sensor3.startContinuous(100);
  sensor4.startContinuous(100);
  sensor5.startContinuous(100);

  power = 1.0;
}

void loop() {
  r1=sensor1.readRangeContinuousMillimeters();
  Serial.print(r1);
  Serial.print(" ");

  r2=sensor2.readRangeContinuousMillimeters();
  Serial.print(r2);
  Serial.print(" ");

  r3=sensor3.readRangeContinuousMillimeters();
  Serial.print(r3);
  Serial.print(" ");
  
  r4=sensor4.readRangeContinuousMillimeters();
  Serial.print(r4);
  Serial.print(" ");

  r5=sensor5.readRangeContinuousMillimeters();
  Serial.print(r5);
  Serial.print(" ");

  Serial.println();

  if(r1<300) a1++;
  
  else a1 = 0;

  if(a1>1) a = 1;
  
  if(a>0) {i1 = 1;
    a--;
    }
    
  else i1 = 0;
    
  if(r2<300) b1++;
  
  else b1 = 0;

  if(b1>1) b = 1;
  
  if(b>0) {i2 = 1;
    b--;
    }
    
  else i2 = 0;
  
  if(r3<300) c1++;

  else c1 = 0;

  if(c1>1) c = 1;
  
  if(c>0) {i3 = 1;
    c--;
    }
    
  else i3 = 0;

  if(!i1 && !i2 && !i3) {motor_left.set(power);
  motor_right.set(power);
  }

  if(i1) {motor_left.set(power);
  motor_right.set(-power);
  }
  
  if(i2) {motor_left.set(power);
  motor_right.set(-power);
  }
  
  if(i3) {motor_left.set(-power);
  motor_right.set(power);
  }
  
  if(((motor_left.get() == power && motor_right.get() == -power) || (motor_left.get() == -power && motor_right.get() == power)) && n == 0) t++;
  else {
    n = t;
    t = 0;
    }

  if(r4<300 && n>0) n4 = n;

  if(n4>0) {motor_left.set(-power);
    motor_right.set(power);
    n4--;
    }
    
  else n = 0;

  if(r5<300 && n>0) n5 = n;

  if(n5>0) {motor_left.set(power);
    motor_right.set(-power);
    n5--;
    }

  else n = 0;
}
