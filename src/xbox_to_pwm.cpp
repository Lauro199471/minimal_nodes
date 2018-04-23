#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <wiringPi.h>

using namespace std;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  cout << "Right Trigger: " << (msg->axes[2] + 1) * 500 << endl;
}

int main(int argc, char **argv)
{

  int gpiopin = 18;
  int pwmClock = 1920;
  int pwmRange = 200;

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joy", 1000, chatterCallback);
 // ros::spin();

  wiringPiSetup();
  pinMode(gpiopin,OUTPUT);

  pwmSetMode(PWM_MODE_MS);

  //clock at 50Hz (20ms tick)
  pwmSetClock(pwmClock);
  pwmSetRange(pwmRange); //range at 200 ticks (20ms)


  while(ros::ok()) // Ctrl-C Handler
  {
    pwmWrite(gpiopin, 15);  //15 (1.5ms)
  }

  pwmWrite(gpiopin,0);

  cout << "Finished" << endl;
  return 0;
}
