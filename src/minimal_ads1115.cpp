/**********************************************************************;
* Project           : Simple ADC I2C
*
* Program name      : minimal_ads1115.cpp
*
* Engineer          : Lauro Cabral
*
* Date created      : May 24 , 2018
*
* Purpose           : Get Analog values
*
* Revision History  : 0.1
*
* Notes             :
*                     VCC : 5V
*                     GND  : GND
*                     ADDR:
*                          Address 0x48: GND
*                          Address 0x49: 5V
*                          Address 0x4A: SDA
*                          Address 0x4B: SCL
**********************************************************************/

#include <wiringPi.h>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <ads1115.h>

// My Custom Message
#include "minimal_nodes/ads1115_sensorVal.h"

using namespace std;

#define MY_BASE 100
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
  ros::Publisher ads1115_pub = n.advertise<minimal_nodes::ads1115_sensorVal>("ads1115_values" , 1000);
  // ******************


  // Always initialise wiringPi. Use wiringPiSys() if you don't need
  //	(or want) to run as root
  //wiringPiSetupSys();
  wiringPiSetupGpio(); // Initalize Pi GPIO

  ads1115Setup (MY_BASE, 0x48);

  ros::Rate r(1000); // 1000 khz for spin (Industy Standard)

  while(ros::ok())
  {

    adc0 = analogRead (MY_BASE + ADC_CHANNEL_0);
    adc1 = analogRead (MY_BASE + ADC_CHANNEL_1);
    adc2 = analogRead (MY_BASE + ADC_CHANNEL_2);
    adc3 = analogRead (MY_BASE + ADC_CHANNEL_3);
    printf("Channel 0: %d\nChannel 1: %d\nChannel 2: %d\nChannel 3: %d\n\n\n", adc0, adc1, adc2, adc3);

    // This is a message object. You stuff it with data, and then publish it.
    minimal_nodes::ads1115_sensorVal ads1115_msb_obj;
    ads1115_msb_obj.analog_values.push_back(adc0);
    ads1115_msb_obj.analog_values.push_back(adc1);
    ads1115_msb_obj.analog_values.push_back(adc2);
    ads1115_msb_obj.analog_values.push_back(adc3);
    ads1115_pub.publish(ads1115_msb_obj);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
