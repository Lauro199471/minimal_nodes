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
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3

int main (int argc, char **argv)
{
  int adc0, adc1, adc2, adc3;

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

  while(ros::ok())
  {
    adc0 = analogRead (MY_BASE + ADC_CHANNEL_0);
    adc1 = analogRead (MY_BASE + ADC_CHANNEL_1);
    adc2 = analogRead (MY_BASE + ADC_CHANNEL_2);
    adc3 = analogRead (MY_BASE + ADC_CHANNEL_3);
    printf("Channel 0: %d\nChannel 1: %d\nChannel 2: %d\nChannel 3: %d\n\n\n", adc0, adc1, adc2, adc3);
  }
  return 0;
}
