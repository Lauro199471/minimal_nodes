#include<ros/ros.h>   // Include ROS Library
#include <wiringPi.h> // Include wiringPi Library
#include <iostream>
#include <softPwm.h>
#include <std_msgs/UInt16.h>

using namespace std;

int pos = 150;
double eq_pos = 0;
void myCallback(const std_msgs::UInt16& message_holder)
{
  cout << "I heard: " << message_holder.data << endl;
  pos = message_holder.data;
}

int main(int argc, char **argv)
{
  cout << "Raspberry Pi wiringPi test program\n";




  ros::init(argc,argv,"minimal_pwm"); //name this node
  // when this compiled code is run, ROS will recognize it as a node called "minimal_wiringPi"

  ros::NodeHandle n; // need this to establish communications with our new node
  ros::Subscriber my_subscriber_object= n.subscribe("Servo_POS",1,myCallback);

  wiringPiSetupGpio(); // Initalize Pi

  pinMode (12, PWM_OUTPUT) ;
  pwmSetMode (PWM_MODE_MS);

  //pwmFrequency in Hz = 19.2e6 Hz / pwmClock / pwmRange.
  // 50Hz ---> 20ms per cycle. 20ms / 2000 units = 0.01ms per unit
   pwmSetRange (2000);
   pwmSetClock (192);


  pwmWrite(12,pos); // 1.5 ms (0 degrees) 150 * .01ms = 1.5ms
  delay(2000);
  pwmWrite(12,250); // 2.0 ms (90 degrees)

  ros::Rate r(10); // 10 hz
   while(ros::ok())
   {
     eq_pos = (0.963*pos) + 74.9;
     pwmWrite(12,eq_pos);
     ros::spinOnce();
     r.sleep();
   }

  pwmWrite(12,0); // 0 * .01ms = 0ms

  return 0;
}




