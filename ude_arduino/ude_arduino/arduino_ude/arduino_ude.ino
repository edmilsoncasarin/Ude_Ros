/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include "ude_arduino/ArmJointState.h"
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Joint 1
#define JOINT4_STEP_PIN    36
#define JOINT4_DIR_PIN     34
#define E1_ENABLE_PIN      30

// Joint 2
#define JOINT3_STEP_PIN    46
#define JOINT3_DIR_PIN     48
#define Z_ENABLE_PIN       62

// Joint 3
#define JOINT2_STEP_PIN    60
#define JOINT2_DIR_PIN     61
#define Y_ENABLE_PIN       56

// Joint 4
#define JOINT1_STEP_PIN    54
#define JOINT1_DIR_PIN     55
#define X_ENABLE_PIN       38

// Joint 5 
//#define E0_STEP_PIN        26
//#define E0_DIR_PIN         28
//#define E0_ENABLE_PIN      24

AccelStepper joint1(1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2(1, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3(1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4(1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
//AccelStepper joint5(1, E0_STEP_PIN, E0_DIR_PIN);

Servo gripper;
MultiStepper steppers;

int joint_step[5];
int joint_status = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

//instantiate publisher (for debugging purposes)
//ros::Publisher steps("joint_steps_feedback",&msg);

void arm_cb(const ude_arduino::ArmJointState& arm_steps){
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  //joint_step[4] = arm_steps.position5;
 // joint_step[5] = arm_steps.position6; //gripper position <0-180>
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}

//instantiate subscribers
ros::Subscriber<ude_arduino::ArmJointState> arm_sub("joint_steps",arm_cb); //subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  Serial.begin(250000);
  nh.getHardware()->setBaud(250000);
      
  
  pinMode(13,OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(E1_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(E1_ENABLE_PIN, LOW);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  //nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(300);
  joint2.setMaxSpeed(500);
  joint3.setMaxSpeed(300);
  joint4.setMaxSpeed(300);
  //joint5.setMaxSpeed(1000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  //steppers.addStepper(joint5);

  // Configure gripper servo
  gripper.attach(11);
  
  digitalWrite(13, 1); //toggle led
}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[4];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = -joint_step[1]; 
    positions[2] = joint_step[2]; 
    positions[3] = joint_step[3]; 
   // positions[4] = -joint_step[4]; 

    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    gripper.write(joint_step[4]);  // move gripper after manipulator reaches goal   
  }
  digitalWrite(13, HIGH-digitalRead(13)); //toggle led
  joint_status = 0;
  
  nh.spinOnce();
  delay(1);
  
}
