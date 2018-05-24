#include <ros/ros.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>

#include "colors.h"
#include "pca9685.h"

#define PIN_BASE 100
#define MAX_PWM 4096
#define HERTZ 50

using namespace std;

int main(int argc, char **argv)
{
  int i = 0, j = 0;
  ros::init(argc, argv, "minimal_pca9685");
  ros::NodeHandle nh;
  cout << ANSI_COLOR_GREEN << "\nPCA9685 LED\n" << ANSI_COLOR_RESET;

  int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  pca9685PWMReset(fd);

  while(ros::ok())
  {
    for (j = 0; j < 5; j++)
    {
      for (i = 0; i < MAX_PWM; i += 32)
      {
        pwmWrite(PIN_BASE + 16, i);
        delay(4);
      }

      for (i = 0; i < MAX_PWM; i += 32)
      {
        pwmWrite(PIN_BASE + 16, MAX_PWM - i);
        delay(4);
      }
    }

    pwmWrite(PIN_BASE + 16, 0);
    delay(500);

    for (j = 0; j < 5; j++)
    {
      for (i = 0; i < 16; i++)
      {
        pwmWrite(PIN_BASE + i, MAX_PWM);
        delay(20);
      }
      for (i = 0; i < 16; i++)
      {
        pwmWrite(PIN_BASE + i, 0);
        delay(20);
      }
    }
    pwmWrite(PIN_BASE + 16, 0);
    delay(500);
  }
  return 0;
}
