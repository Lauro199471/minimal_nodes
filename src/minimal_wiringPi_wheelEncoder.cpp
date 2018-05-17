/*
 * Wheel Encoder in Raspberry Pi 3
 *
 * Encoder Pin: CLK --> A Clk
 *              DT  --> B Clk
 *              SW --> PushButton
 *
*/
#include <wiringPi.h>
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>

using namespace std;

#define RoAPin 0
#define RoBPin 1

static volatile int globalCounter = 0;

unsigned char flag;
unsigned char Last_RoB_Status;
unsigned char Current_RoB_Status;

void rotaryDeal(void)
{
    Last_RoB_Status = digitalRead(RoBPin);
    while(!digitalRead(RoAPin))
    {
        Current_RoB_Status = digitalRead(RoBPin);
        flag = 1;
    }

    if(flag == 1)
    {
        flag = 0;
        if((Last_RoB_Status == 0) && (Current_RoB_Status == 1))
            globalCounter++;
        if((Last_RoB_Status == 1) && (Current_RoB_Status == 0))
            globalCounter--;
    }
}

int main (int argc, char **argv)
{
  int gpiopin = 17;
  std::stringstream ss;

  printf("Raspberry Pi wiringPi Wheel Encoder test\n");

  // *** ROS Stuff ****
  ros::init(argc,argv,"wheel_encoder"); //name this node
  ros::NodeHandle n; // need this to establish communications with our new node
  // ******************

  // Always initialise wiringPi. Use wiringPiSys() if you don't need
  //	(or want) to run as root
  //wiringPiSetupSys();
  wiringPiSetupGpio(); // Initalize Pi GPIO

  pinMode(RoAPin, OUTPUT);
  pinMode(RoBPin, OUTPUT);

  while(ros::ok())
  {
    rotaryDeal();
    cout << "Counter: " << globalCounter << endl;
  }
  return 0;
}
