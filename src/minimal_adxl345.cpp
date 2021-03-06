/**********************************************************************;
* Project           : Simple Starter Code
*
* Program name      : minimal_adxl345.cpp
*
* Engineer          : Lauro Cabral
*
* Date created      : May 25 , 2018
*
* Purpose           : Simple code for the ADXL345 sensor
*
* Revision History  : 0.1
*
* Notes             :
*                     GND  : GND
*                     VCC  : 3.3V
*                     SCL  : Pin 5
*                     SDA  : Pin 3
**********************************************************************/

// Header file for input output functions
#include<iostream>
#include <wiringPiI2C.h>
#include <ros/ros.h>
#include <wiringPi.h>
#include <math.h>

using namespace std;

#define  DevAddr  0x53  //device address

struct acc_dat{
  int x;
  int y;
  int z;
};

void adxl345_init(int fd)
{
  wiringPiI2CWriteReg8(fd, 0x31, 0x0b);
  wiringPiI2CWriteReg8(fd, 0x2d, 0x08);
  wiringPiI2CWriteReg8(fd, 0x2e, 0x00);
  wiringPiI2CWriteReg8(fd, 0x1e, 0x00);
  wiringPiI2CWriteReg8(fd, 0x1f, 0x00);
  wiringPiI2CWriteReg8(fd, 0x20, 0x00);

  wiringPiI2CWriteReg8(fd, 0x21, 0x00);
  wiringPiI2CWriteReg8(fd, 0x22, 0x00);
  wiringPiI2CWriteReg8(fd, 0x23, 0x00);

  wiringPiI2CWriteReg8(fd, 0x24, 0x01);
  wiringPiI2CWriteReg8(fd, 0x25, 0x0f);
  wiringPiI2CWriteReg8(fd, 0x26, 0x2b);
  wiringPiI2CWriteReg8(fd, 0x27, 0x00);

  wiringPiI2CWriteReg8(fd, 0x28, 0x09);
  wiringPiI2CWriteReg8(fd, 0x29, 0xff);
  wiringPiI2CWriteReg8(fd, 0x2a, 0x80);
  wiringPiI2CWriteReg8(fd, 0x2c, 0x0a);
  wiringPiI2CWriteReg8(fd, 0x2f, 0x00);
  wiringPiI2CWriteReg8(fd, 0x38, 0x9f);
}

struct acc_dat adxl345_read_xyz(int fd)
{
  char x0, y0, z0, x1, y1, z1;
  struct acc_dat acc_xyz;

  x0 = 0xff - wiringPiI2CReadReg8(fd, 0x32);
  x1 = 0xff - wiringPiI2CReadReg8(fd, 0x33);

  y0 = 0xff - wiringPiI2CReadReg8(fd, 0x34);
  y1 = 0xff - wiringPiI2CReadReg8(fd, 0x35);

  z0 = 0xff - wiringPiI2CReadReg8(fd, 0x36);
  z1 = 0xff - wiringPiI2CReadReg8(fd, 0x37);

  acc_xyz.x = (int)(x1 << 8) + (int)x0;
  acc_xyz.y = (int)(y1 << 8) + (int)y0;
  acc_xyz.z = (int)(z1 << 8) + (int)z0;

  return acc_xyz;
}

int main(int argc, char **argv)
{
  int fd;
  double angle_x = 0.0;
  double angle_y = 0.0;
  double angle_z = 0.0;

  struct acc_dat acc_xyz;

  // *** ROS Stuff ****
  ros::init(argc,argv,"minimal_adxl345"); //name this node
  ros::NodeHandle n; // need this to establish communications with our new node
  // ******************

  fd = wiringPiI2CSetup(DevAddr);

  if(-1 == fd)
  {
    cout << "I2C device setup error" << endl;
  }

  adxl345_init(fd);

  ros::Rate r(1); // 1khz for spin
  while(ros::ok())
  {
    acc_xyz = adxl345_read_xyz(fd);

    angle_x = atan( acc_xyz.x / (sqrt((acc_xyz.y * acc_xyz.y) + (acc_xyz.z * acc_xyz.z))));
    angle_y = atan( acc_xyz.y / (sqrt((acc_xyz.x * acc_xyz.x) + (acc_xyz.z * acc_xyz.z))));
    angle_z = atan( sqrt((acc_xyz.x * acc_xyz.x) + (acc_xyz.y * acc_xyz.y)) / acc_xyz.z);

    angle_x = angle_x * 180;
    angle_x = angle_x / 3.141592;

    angle_y = angle_y * 180;
    angle_y = angle_y / 3.141592;

    angle_z = angle_z * 180;
    angle_z = angle_z / 3.141592;

    printf("x: %d  y: %d  z: %d\n", acc_xyz.x, acc_xyz.y, acc_xyz.z);
    printf("Dx: %3f  Dy: %3f  Dz: %3f\n", angle_x , angle_y , angle_z);

    r.sleep();
  }

  return 0;
}
