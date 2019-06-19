/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
  SOLabot LiuYC (2018)
*/

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h> 
#define USE_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nodeHandle;
// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)
// ackermann ateering: outer_wheel_max=23 degree; inner_wheel_max=32 degree 
const int minSteering = 60 ; 
const int maxSteering = 120 ;
const int neutralThrottle = 1500 ;

const int lbbacThrottle = 1345 ;
const int upbacThrottle = 1390 ; // min. to move backward with load
const int lbforThrottle = 1540 ; // min. to move forward with load
const int upforThrottle = 1570 ; // 1560 = 1.5m/s
const int forthrustpwm = 1595;  // upforThrottle +25
const int bacthrustpwm = 1320;  // lbbacThrottle -25

const int minThrottle = 1300 ;
const int maxThrottle = 1700 ;
// This RELAY was closed when digitalWrite at LOW 
const int RelayPin = 8 ;
// Set the maximum speed (m/s) and steering angle (rad) for solabot
const double min_linear = -1.5 ;
const double max_linear = 1.5 ;
const double min_angular = -0.32 ; //CCW
const double max_angular = 0.32 ; //CW
int lastpwm = 0;
unsigned long lastMillis_f = 0;
unsigned long lastMillis_b = 0;
int check = 0;
unsigned long checkMillis = 0;

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback ( const geometry_msgs::Twist&  twistMsg )
{
  
  int steeringAngle = fmap(twistMsg.angular.z, min_angular, max_angular, minSteering, maxSteering) ;
  // The following could be useful for debugging
  //str_msg.data= steeringAngle ;
  //chatter.publish(&str_msg);
  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) { 
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    steeringAngle = maxSteering ;
  }
  steeringServo.write(steeringAngle) ;
  
 
  int escCommand ;  
 // (suspended for now, we have new gear)
 // for loaded forward, need more torque to start  
  if(lastpwm <= neutralThrottle && twistMsg.linear.x > 0){
    lastMillis_f = millis();
    escCommand = forthrustpwm;
  }
  
  if (lastpwm >= neutralThrottle && twistMsg.linear.x < 0){
    lastMillis_b = millis();
    escCommand = bacthrustpwm;
  } 
  
  if(millis()-lastMillis_f <= 250 && twistMsg.linear.x > 0){
      escCommand = forthrustpwm;
      check = 2;
    }  
  // for backward motion, need to activate throttle twice 
  /*else */
  
  /*  
  if (twistMsg.linear.x > 0 && millis()-lastMillis_f < 1050 ){
    
    checkMillis = millis()-lastMillis_f;
    
    if(millis()-lastMillis_f <= 250 && millis()-lastMillis_f > 0 ){
      escCommand = neutralThrottle + 10;
    }
    else if(millis()-lastMillis_f <= 500 && millis()-lastMillis_f > 250 ){
      escCommand = forthrustpwm;
    }  
    else if(millis()-lastMillis_f <= 750 && millis()-lastMillis_f > 500 ){
      escCommand = neutralThrottle + 10;
    }  
    else if(millis()-lastMillis_f <= 1050 && millis()-lastMillis_f > 750 ){
      escCommand = forthrustpwm;
    }  
  }
  */
  
  
  else if (twistMsg.linear.x < 0 && millis()-lastMillis_b < 1050 ){
    
    checkMillis = millis()-lastMillis_b;
    
    if(millis()-lastMillis_b <= 250 && millis()-lastMillis_b > 0 ){
      escCommand = neutralThrottle-10;
    }
    else if(millis()-lastMillis_b <= 500 && millis()-lastMillis_b > 250 ){
      escCommand = bacthrustpwm;
    }  
    else if(millis()-lastMillis_b <= 750 && millis()-lastMillis_b > 500 ){
      escCommand = neutralThrottle-10;
    }  
    else if(millis()-lastMillis_b <= 1050 && millis()-lastMillis_b > 750 ){
      escCommand = bacthrustpwm;
    }
  }
  
   else{
    
    checkMillis = 0;
    
    if (twistMsg.linear.x > 0.0) {
        escCommand = (int)fmap(twistMsg.linear.x, 0, max_linear, lbforThrottle, upforThrottle) ;
    } 
    else if (twistMsg.linear.x == 0){
        escCommand = neutralThrottle ;
        check = 0;
    }
    else {
        escCommand = (int)fmap(twistMsg.linear.x, min_linear, 0, lbbacThrottle, upbacThrottle) ;
    }
   }  
  
  // ESC forward is between -1.5 and 1.5
  
  
  /*
  if (twistMsg.linear.x >= 0.0) {
       escCommand = (int)fmap(twistMsg.linear.x, 0, max_linear, neutralThrottle, maxThrottle) ;
    } 
  else {
      escCommand = (int)fmap(twistMsg.linear.x, min_linear, 0, minThrottle, neutralThrottle) ;
    }
  
  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) { 
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  */
    
  // The following could be useful for debugging
   str_msg.data= escCommand ;
   chatter.publish(&str_msg);
  
  lastpwm = escCommand;
  electronicSpeedController.writeMicroseconds(escCommand) ;
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
 
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;

void setup(){
 pinMode(13, OUTPUT);
 // Use Relay as safety measure
 pinMode(RelayPin, OUTPUT);
 digitalWrite(RelayPin, HIGH);
 
 Serial.begin(115200) ;
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  
  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(10); // ESC is on pin 10
  //Initialize Steering and ESC setting
  //Steering centered is 90, throttle at neutral is 90
  steeringServo.write(90) ;
  electronicSpeedController.writeMicroseconds(neutralThrottle) ;
  delay(1000) ;
  
}

void loop(){
  
  if(nodeHandle.connected()){
    digitalWrite(RelayPin,LOW);
  }
  else{
    digitalWrite(RelayPin,HIGH);
  }
  
  
  nodeHandle.spinOnce();
  delay(1);
   
}



