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

using namespace std;

#define one_sec 1000

int main (int argc, char **argv)
{
  int gpiopin = 17;
  std_msgs::String msg;
  std::stringstream ss;

  printf("Raspberry Pi wiringPi blink test\n");

  // *** ROS Stuff ****
  ros::init(argc,argv,"minimal_blink"); //name this node
  ros::NodeHandle n; // need this to establish communications with our new node
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("LED_STATE", 1000);
  // ******************

  // Always initialise wiringPi. Use wiringPiSys() if you don't need
  //	(or want) to run as root
  //wiringPiSetupSys();
  wiringPiSetupGpio(); // Initalize Pi GPIO

  pinMode(gpiopin, OUTPUT);

  while(ros::ok())
  {

    // On State
    delay(one_sec);
    printf("LED On\n");
    digitalWrite(gpiopin, 1);
    ss.str("");
    ss << "ON";
    msg.data = ss.str();
    chatter_pub.publish(msg);

    // Off State
    delay(one_sec);
    printf("LED Off\n");
    digitalWrite(gpiopin, 0);
    ss.str("");
    ss << "OFF";
    msg.data = ss.str();
    chatter_pub.publish(msg);

  }

  digitalWrite(gpiopin, 0);
  cout << "\nFinished\n";
  return 0;
}
