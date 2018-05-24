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
#define NUM_PINS 16 // PCA9685 has 16 pwm pins
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
    // Make ALL pins dim down and up
    cout << ANSI_COLOR_RED << "ALL PINS" << ANSI_COLOR_RESET << endl;
    for (j = 0; j < 20; j++)
    {
      for (i = 0; i < MAX_PWM; i += 16) // 16 because its too slow to light up and dim
      {
        pwmWrite(PIN_BASE + NUM_PINS, i);
        delay(1);
      }

      for (i = 0; i < MAX_PWM; i += 16)
      {
        pwmWrite(PIN_BASE + NUM_PINS, MAX_PWM - i);
        delay(1);
      }
    }

    pwmWrite(PIN_BASE + NUM_PINS, 0);
    delay(500);

    // Make Individual Pins turn off and on
    cout << ANSI_COLOR_MAGENTA << "ONE BY ONE" << ANSI_COLOR_RESET << endl;
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
