#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 

ros::NodeHandle  nh;
Servo servo1;
Servo servo2;

double pos1 = 30;
double pos2 = 100;
int direc1 = 1;
int direc2 = -1;
int j = 0;
int i = 0;

void setup() {
  nh.initNode();
  
  servo1.attach(8); 
  servo2.attach(9);

  // verifying connection with swimmer

}

void loop() {
  if (pos1 == 130) {
    direc1 = -1;
  } else if (pos1 == 20) {
    direc1 = 1;
  }
  pos1 += direc1;

  if (pos2 == 130) {
    direc2 = -1;
  } else if (pos2 == 20) {
    direc2 = 1;
  }
  pos2 += direc2;

  servo2.write(pos2);
  servo1.write(pos1 + 30);
  

  delay(5);
}
// servo 1 range: 30 - 175
// servo 2 range: 0 - 155