#define ROSSERIAL_ARDUINO_TCP //(need to comment out when connected to teensy via USB)

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <noodlebot/swimmer_command.h> 
#include <noodlebot/swimmer_info.h> 
#include <noodlebot/swimmer_reset.h> 
#include "global.h"
#include "wifi.h"

/////////////////// VARIABLES ///////////////////

// motors
Servo servo1;
Servo servo2;
int joint1_pin = 28; // these might change if you use different teensy servo pins
int joint2_pin = 29; // these might change if you use different teensy servo pins

// variables for integration for joints
float accel1 = 0;
float accel2 = 0;

float vel1 = 0;
float newvel1;
float pos1;
float newpos1;

float vel2 = 0;
float newvel2;
float pos2;
float newpos2;

float min_joint_angle=10; 
float max_joint_angle=170; 
float max_joint_vel=60/0.18; // servo max speed is 60degrees/0.18sec @ 5V
unsigned long last_hedgehog = millis();

// swimmer location info
float new_x;
float new_y;
float new_ang;
float new_xvel;
float new_yvel;
float new_angvel;
int64_t new_pos_timestamp;

float prev_x;
float prev_y;
float prev_ang;
float prev_xvel = 0;
float prev_yvel = 0;
float prev_angvel = 0;
int64_t prev_pos_timestamp;

// declaring time step
float dt = 0.1; // s
unsigned short interval = dt*1000; // millis

// flags
bool reset_in_progress = false;
bool ep_in_progress = false;
bool use_hedgehog = true; // set to false if you want to use april tags

/////////////////// ROS ///////////////////

// function for extracting information sent from ROS command
void servo_cb( const noodlebot::swimmer_command& cb_msg){  
  accel1 = cb_msg.joint1;
  accel2 = cb_msg.joint2;
  ep_in_progress = true;
  Serial.println("Servo callback msg received");
}

// publisher for reset to python
void servo_reset(const noodlebot::swimmer_reset& reset_msg) {
  dt = reset_msg.dt; 
  min_joint_angle = reset_msg.min_joint_angle + 90;
  max_joint_angle = reset_msg.max_joint_angle + 90;
  interval = dt*1000; 

  newpos1 = reset_msg.joint1pos + 90;
  newpos2 = reset_msg.joint2pos + 90;
  pos1 = servo1.read();
  pos2 = servo2.read();
  reset_in_progress = true;
  Serial.println("reset command received");
}

void episode_done(const std_msgs::Empty& ep_msg) {
  ep_in_progress = false;
}

// accelerations from ROS command
ros::Subscriber<noodlebot::swimmer_command> sub("servo_cb", servo_cb);

// reset pos published from test_env.py
ros::Subscriber<noodlebot::swimmer_reset> subreset("servo_reset", servo_reset);

// receives msg when epsiode is finished
ros::Subscriber<std_msgs::Empty> ep_sub("episode_done", episode_done);

// ros node that uses WiFi
ros::NodeHandle_<WiFiHardware> nh;

// return the joint pose & vel and swimmer pose & vel
noodlebot::swimmer_info joint_swimmer_info;
ros::Publisher pub("joint_swimmer_info", &joint_swimmer_info);

// publishes when done with setup
std_msgs::Empty empty_msgs;
ros::Publisher pub_setup("setup_info", &empty_msgs);
std_msgs::String string_msgs;
ros::Publisher pub_error("error", &string_msgs);

bool build_joint_msg(bool reset) {
  if (reset){
    // joints
    vel1 = 0;
    vel2 = 0;
    newvel1 = 0;
    newvel2 = 0;
    // states
    new_xvel = 0.;
    new_yvel = 0.;
    new_angvel = 0.;
  } else {
    float max_dist = 300; // mm
    new_x = constrain(new_x,prev_x - max_dist, prev_x + max_dist);
    new_y = constrain(new_y,prev_y - max_dist, prev_y + max_dist);

    // vel
    new_xvel = constrain(new_xvel,-1.5,1.5);
    new_yvel = constrain(new_yvel,-1.5,1.5);
    new_angvel = constrain(new_angvel,-180,180);
  }

  joint_swimmer_info.joint1pos = newpos1 - 90;
  joint_swimmer_info.joint2pos = newpos2 - 90;

  joint_swimmer_info.joint1vel = newvel1;
  joint_swimmer_info.joint2vel = newvel2;
  
  if (use_hedgehog){
    // convert hedgehog pose mm -> m
    joint_swimmer_info.swimmer_x = new_x/1000;
    joint_swimmer_info.swimmer_y = new_y/1000;
    // convert millidegrees -> degrees
    joint_swimmer_info.swimmer_direction = new_ang/1000;

    joint_swimmer_info.swimmervel_x = new_xvel;
    joint_swimmer_info.swimmervel_y = new_yvel;
    joint_swimmer_info.swimmervel_ang = new_angvel;

    joint_swimmer_info.hedgehog_count = hedgehog_count;
  }
  return check_msg();
}

bool check_msg() {
  return (isnan(joint_swimmer_info.joint1pos) || isnan(joint_swimmer_info.joint1vel) ||
          isnan(joint_swimmer_info.joint2pos) || isnan(joint_swimmer_info.joint2vel) ||
          isnan(joint_swimmer_info.swimmer_x) || isnan(joint_swimmer_info.swimmer_y) || isnan(joint_swimmer_info.swimmer_direction) ||
          isnan(joint_swimmer_info.swimmervel_x) || isnan(joint_swimmer_info.swimmervel_y) || isnan(joint_swimmer_info.swimmervel_ang));     
}

float angle_wrap(float angle){
  return (float)(((int)(angle + 180*1000) % (360*1000)) - (180*1000));
}

/////////////////// SERVOS ///////////////////

void reset_update() {
  if (!servo1.attached()){
    servo1.attach(joint1_pin); 
  }
  if (!servo2.attached()){
    servo2.attach(joint2_pin);  
  }
  
  Serial.println("running reset update");

  if (newpos1 != pos1) {
    pos1 += constrain(newpos1 - pos1,-50,50);
    servo1.write(pos1);
  } 
  if (newpos2 != pos2) {
    pos2 += constrain(newpos2 - pos2,-50,50);
    servo2.write(pos2);
    }
  delay(100);
} 

// function for providing new pos based on acceleration
void update_control(){
  newpos1 = pos1 + vel1*dt;
  newvel1 = constrain(vel1 + accel1*dt,-max_joint_vel,max_joint_vel);

  newpos2 = pos2 + vel2*dt;
  newvel2 = constrain(vel2 + accel2*dt,-max_joint_vel,max_joint_vel);
}

// function for clipping pos based on joint limits
void check_limits(){
    if (newpos1 >= max_joint_angle && newvel1 > 0.) {
      newvel1 = 0.;
    } else if (newpos1 <= min_joint_angle && newvel1 < 0.){
      newvel1 = 0.;
    }
    newpos1 = constrain(newpos1,min_joint_angle,max_joint_angle);
    vel1 = newvel1;
    pos1 = newpos1;

    if (newpos2 >= max_joint_angle && newvel2 > 0.) {
      newvel2 = 0.;
    } else if (newpos2 <= min_joint_angle && newvel2 < 0.){
      newvel2 = 0.;
    }
    newpos2 = constrain(newpos2,min_joint_angle,max_joint_angle);
    vel2 = newvel2;
    pos2 = newpos2;
}

/////////////////// SWIMMER LOCATION ///////////////////

void get_hedgehog_states(){
  if (use_hedgehog){
    process_hedgehog();
    if (hedgehog_pos_updated){
      last_hedgehog = millis();
      //// POSITION ////
      new_x = hedgehog_x;
      new_y = hedgehog_y;
      // rotate & angle wrapping (decidegrees -> millidegrees)
      new_ang = hedgehog_paired_heading*100;
      new_ang = angle_wrap(new_ang-90*1000);

      //// TIMING (millis) ////
      new_pos_timestamp = hedgehog_pos_timestamp;
      hedgehog_count += 1;
      float tmp_dt = (float)(new_pos_timestamp - prev_pos_timestamp);

      //// VELOCITY ////
      // in mm/millis = m/s
      new_xvel = (hedgehog_x - prev_x) / tmp_dt; 
      new_yvel = (hedgehog_y - prev_y) / tmp_dt;

      // angle wrapping (millidegrees/millis = deg/s)
      if (abs(new_ang - prev_ang) > 180*1000){
        float tmp_new = (float)((int)(new_ang) % (360*1000));
        float tmp_prev = (float)((int)(prev_ang) % (360*1000));
        new_angvel = (tmp_new - tmp_prev) / tmp_dt;
      } else {
        new_angvel = (new_ang - prev_ang) / tmp_dt;
      }
      hedgehog_pos_updated=false;
    }
  } else{
    hedgehog_count = 100;
  }
}

void update_variables() {
  // updating variables
  prev_x = new_x;
  prev_y = new_y;
  
  prev_xvel = new_xvel;
  prev_yvel = new_yvel;

  prev_ang = new_ang;
  prev_angvel = new_angvel;

  prev_pos_timestamp = new_pos_timestamp;
}

/////////////////// MAIN ///////////////////

void setup(){
  // setting up the connection with WiFi module
  // // start the Serial communication 
  Serial.begin(57600); // debugging port
  Serial1.begin(115200); // wifi port
  Serial2.begin(500000); // hedgehog transmits data on 500 kbps
   
  Serial.println("starting setup");
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true); // stay here
  }
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("You're connected to the network");

  nh.initNode();
  nh.setSpinTimeout(10);
  nh.subscribe(sub);
  nh.subscribe(subreset);
  nh.subscribe(ep_sub);
  nh.advertise(pub);
  nh.advertise(pub_setup);
  nh.advertise(pub_error);
  
  // setting up connection with the servos
  servo1.attach(joint1_pin);
  servo2.attach(joint2_pin);

  // test to make sure the motors are connected properly
  servo1.write(90);
  servo2.write(90);
  delay(500);
  servo1.write(70);
  servo2.write(70);
  delay(500);
  servo1.write(90);
  servo2.write(90);
}

void loop() { 
  bool err = false;
  unsigned long previousMillis = millis();
  nh.spinOnce();
  while (!nh.connected()){
      nh.spinOnce();
      if ( millis() - previousMillis > 5000){ // if not connected for 5 seconds, restart
          Serial.println("lost connection to ROS for 5 seconds, restarting");
          delay(100);
          SCB_AIRCR = 0x05FA0004; 
      }
  }
  if (reset_in_progress) {
    reset_update();

    if (newpos1 == pos1 && newpos2 == pos2) {
      if (use_hedgehog){
        hedgehog_count = 0;
        while (hedgehog_count < 4){
          get_hedgehog_states();
          nh.spinOnce();
          if ((hedgehog_count == 0) && (millis() - previousMillis > 1000)) { // if not connected, break and send error message
            reset_in_progress = false;
            string_msgs.data = "timed out waiting for hedgehog";
            pub_error.publish(&string_msgs);
            Serial.println("timed out waiting for hedgehog");
            err = true;
            break; 
          }
        }
      }
      if (!err){
        err = build_joint_msg(true);
        if (err) {
          string_msgs.data = "error formatting joint message";
          pub_error.publish(&string_msgs);
          Serial.println("message type error");
        } else {
          pub.publish(&joint_swimmer_info);
          Serial.println("reset done");
          reset_in_progress = false;
          update_variables();
        }
      }
      nh.spinOnce();
      delay(250);    
      hedgehog_count = 0;
    }
  } else if (ep_in_progress) {
      update_control();       // updating position and velocity based on the acceleration values
      
      // send to servos
      if (!servo1.attached()){
        servo1.attach(joint1_pin);
      }
      if (!servo2.attached()){
        servo2.attach(joint2_pin);  
      }
      
      unsigned long progress = millis() - previousMillis;
      while (progress <= interval) {
        long angle1 = constrain(map(progress, 0, interval, pos1, newpos1),min_joint_angle,max_joint_angle);
        long angle2 = constrain(map(progress, 0, interval, pos2, newpos2),min_joint_angle,max_joint_angle);
        servo1.write(angle1);
        servo2.write(angle2);
        get_hedgehog_states();
        progress = millis() - previousMillis;
      }

      // publishing position/velocities
      check_limits();         // making sure pose is within servo limits
      if (hedgehog_count > 0){
        err = build_joint_msg(false);
        hedgehog_count = 0;

        // checking for nan values in the message
        if (err) {
          string_msgs.data = "error formatting joint message";
          pub_error.publish(&string_msgs);
          Serial.println("message type error:");
          Serial.print("joint1pos ");
          Serial.println(joint_swimmer_info.joint1pos);
          Serial.print("joint2pos "); 
          Serial.println(joint_swimmer_info.joint2pos); 
          Serial.print("joint1vel ");
          Serial.println(joint_swimmer_info.joint1vel);
          Serial.print("joint2vel "); 
          Serial.println(joint_swimmer_info.joint2vel); 
          Serial.print("swimmer_x "); 
          Serial.println(joint_swimmer_info.swimmer_x); 
          Serial.print("swimmer_y "); 
          Serial.println(joint_swimmer_info.swimmer_y); 
          Serial.print("swimmer_direction "); 
          Serial.println(joint_swimmer_info.swimmer_direction); 
          Serial.print("swimmervel_x ");
          Serial.println(joint_swimmer_info.swimmervel_x);
          Serial.print("swimmervel_y ");
          Serial.println(joint_swimmer_info.swimmervel_y);
          Serial.print("swimmervel_ang ");
          Serial.println(joint_swimmer_info.swimmervel_ang);
        } else {
          pub.publish(&joint_swimmer_info);
          update_variables();
        }
      } else if ((use_hedgehog) && (millis()-last_hedgehog > 1000)){ // if not connected, break and send error message
        // ep_in_progress = false;
        string_msgs.data = "timed out waiting for hedgehog";
        pub_error.publish(&string_msgs);
        Serial.println("timed out waiting for hedgehog");
      }
      nh.spinOnce();
    } else {
      // stop servos whenever we're not using them
      servo1.detach(); 
      servo2.detach();
      pub_setup.publish(&empty_msgs);
      nh.spinOnce();
      delay(250);
    }
}
