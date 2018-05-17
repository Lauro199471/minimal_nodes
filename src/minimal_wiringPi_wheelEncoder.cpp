#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <ros/ros.h>

#define  RoAPin    0
#define  RoBPin    1
#define  RoSPin    2

static volatile int globalCounter = 0 ;

unsigned char flag;
unsigned char Last_RoB_Status;
unsigned char Current_RoB_Status;

void rotaryDeal(void)
{
    Last_RoB_Status = digitalRead(RoBPin);

    while(!digitalRead(RoAPin) && ros::ok()){
        Current_RoB_Status = digitalRead(RoBPin);
        flag = 1;
    }

    if(flag == 1){
        flag = 0;
        if((Last_RoB_Status == 0)&&(Current_RoB_Status == 1)){
            globalCounter ++;
            printf("globalCounter : %d\n",globalCounter);
        }
        if((Last_RoB_Status == 1)&&(Current_RoB_Status == 0)){
            globalCounter --;
            printf("globalCounter : %d\n",globalCounter);
        }

    }
}

void rotaryClear(void)
{
    if(digitalRead(RoSPin) == 0)
    {
        globalCounter = 0;
        printf("globalCounter : %d\n",globalCounter);
        delay(1000);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"minimal_wheel_encoder"); //name this node
    ros::NodeHandle n; // need this to establish communications with our new node

    if(wiringPiSetup() < 0){
        fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
        return 1;
    }

    pinMode(RoAPin, INPUT);
    pinMode(RoBPin, INPUT);
    pinMode(RoSPin, INPUT);

    pullUpDnControl(RoSPin, PUD_UP);

    while(ros::ok()){
        rotaryDeal();
        rotaryClear();
    }

    return 0;
}
