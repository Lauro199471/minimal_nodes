/*
 * Blink LED in Raspberry Pi 3
 *
 * Pin: GPIO17(Pin 11)
 *
*/
#include <wiringPi.h>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>
#include <ads1115.h>

using namespace std;

#define MY_BASE 2222
#define JOY_RX 0
#define JOY_RY 1
#define POT 2
#define TEMPERATURE 3

int main (int argc, char **argv)
{
  int joy_x, joy_y, pot_val, temp_val;

  printf("Raspberry Pi wiringPi ADC1115 test\n");

  // *** ROS Stuff ****
  ros::init(argc,argv,"minimal_ads1115"); //name this node
  ros::NodeHandle n; // need this to establish communications with our new node
  // ******************

  // Always initialise wiringPi. Use wiringPiSys() if you don't need
  //	(or want) to run as root
  //wiringPiSetupSys();
  wiringPiSetupGpio(); // Initalize Pi GPIO

   ads1115Setup (MY_BASE, 0x48);

   while(1) {
     joy_x = analogRead (MY_BASE + JOY_RX);
     joy_y = analogRead (MY_BASE + JOY_RY);
     pot_val = analogRead (MY_BASE + POT);
     temp_val = analogRead (MY_BASE + TEMPERATURE);
     printf("joy_x: %d\tjoy_y: %d\npot_val: %d\ntemp_val: %d\n", joy_x, joy_y, pot_val, temp_val);
     sleep(1);
 }


  return 0;
}
