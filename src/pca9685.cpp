/**
 * @file pca9685.c
 * @author Patrick Kramer
 * @copyright MIT License
 * @date Dec 28 2017
 * @brief Implementation of the control functions for the PC9685 Library
 *
 * This library requires the PC9685_Init() function to be called to set the global i2c_fd file-handle; i2c_fd is used by
 * both the PC9685_WriteRegister and PC9685_ReadRegister functions.
 * TODO: Implement a new method to allow multiple file-handles; this should be a c++ object.
 *
 * The remaining functions in the libraray exist to narrowly implement tasks using the register read/write functions.
 */


#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "pca9685.h"

#define tcOFF       "\x1b[31m"
#define tcON        "\x1b[32m"
#define tcRESET     "\x1b[0m"

static uint8_t i2c_fd;

uint8_t PCA9685_init(uint8_t i2cAddress, uint8_t i2cSpeed) {

    uint8_t status;

    status = wiringPiSetup();
    if (status != 0) {
        return ((uint8_t) -1);
    }

    i2c_fd = wiringPiI2CSetup(i2cAddress);
}

void PCA9685_WriteRegister(uint8_t address, uint8_t value) {

    wiringPiI2CWriteReg8(i2c_fd, address, value);
}

uint8_t PCA9685_ReadRegister(uint8_t address) {

    return wiringPiI2CReadReg8(i2c_fd, address);
}


void PCA9685_PrintStatus() {

    PCA9685_MODE1_t mode1;
    PCA9685_MODE2_t mode2;

    mode1.raw = PCA9685_ReadRegister(PCA9685_MODE1);
    mode2.raw = PCA9685_ReadRegister(PCA9685_MODE2);

    printf("\nRESET\t"tcRESET);

    if (mode1.fields.extClk == 1) {
        printf(tcON"EXTCLK\t"tcRESET);
    } else {
        printf(tcOFF"EXTCLK\t"tcRESET);
    }

    if (mode1.fields.autoIncrement == 1) {
        printf(tcON"AUTO\t"tcRESET);
    } else {
        printf(tcOFF"AUTO\t"tcRESET);
    }

    if (mode1.fields.sleep == 1) {
        printf(tcON"SLEEP\t"tcRESET);
    } else {
        printf(tcOFF"SLEEP\t"tcRESET);
    }

    if (mode1.fields.sub1 == 1) {
        printf(tcON"SUB1\t"tcRESET);
    } else {
        printf(tcOFF"SUB1\t"tcRESET);
    }

    if (mode1.fields.sub2 == 1) {
        printf(tcON"SUB2\t"tcRESET);
    } else {
        printf(tcOFF"SUB2\t"tcRESET);
    }

    if (mode1.fields.sub3 == 1) {
        printf(tcON"SUB3\t"tcRESET);
    } else {
        printf(tcOFF"SUB3\t"tcRESET);
    }

    if (mode1.fields.allCall == 1) {
        printf(tcON"ALLCALL\n"tcRESET);
    } else {
        printf(tcOFF"ALLCALL\n"tcRESET);
    }

    if (mode2.fields.invert == 1) {
        printf(tcON"INVRT\t"tcRESET);
    } else {
        printf(tcOFF"INVRT\t"tcRESET);
    }

    if (mode2.fields.outputChange == 1) {
        printf(tcON"OCH\t"tcRESET);
    } else {
        printf(tcOFF"OCH\t"tcRESET);
    }

    if (mode2.fields.outputDrive == 1) {
        printf(tcON"OUTDRV\t"tcRESET);
    } else {
        printf(tcOFF"OUTDRV\t"tcRESET);
    }

    if (mode2.fields.outputNegation == 1) {
        printf(tcON"OUTNE\n"tcRESET);
    } else {
        printf(tcOFF"OUTNE\n"tcRESET);
    }
}

void PCA9685_SetPWM(PCA9685_Channel_e channel, uint16_t dutyCycle) {

    PCA9685_ChannelRegisters_t registers;
    PCA9685_LED_OFF_H_t OFF_H;
    PCA9685_LED_OFF_L_t OFF_L;
    PCA9685_LED_ON_H_t ON_H;
    PCA9685_LED_ON_L_t ON_L;

    OFF_H.raw = 0;
    OFF_L.raw = 0;
    ON_H.raw = 0;
    ON_L.raw = 0;

    switch(channel) {
        case ch0:
            registers.OFF_H = PCA9685_LED0_OFF_H;
            registers.OFF_L = PCA9685_LED0_OFF_L;
            registers.ON_H = PCA9685_LED0_ON_H;
            registers.ON_L = PCA9685_LED0_ON_L;
            break;

        case ch1:
            registers.OFF_H = PCA9685_LED1_OFF_H;
            registers.OFF_L = PCA9685_LED1_OFF_L;
            registers.ON_H = PCA9685_LED1_ON_H;
            registers.ON_L = PCA9685_LED1_ON_L;
            break;

        case ch2:
            registers.OFF_H = PCA9685_LED2_OFF_H;
            registers.OFF_L = PCA9685_LED2_OFF_L;
            registers.ON_H = PCA9685_LED2_ON_H;
            registers.ON_L = PCA9685_LED2_ON_L;
            break;

        case ch3:
            registers.OFF_H = PCA9685_LED3_OFF_H;
            registers.OFF_L = PCA9685_LED3_OFF_L;
            registers.ON_H = PCA9685_LED3_ON_H;
            registers.ON_L = PCA9685_LED3_ON_L;
            break;

        case ch4:
            registers.OFF_H = PCA9685_LED4_OFF_H;
            registers.OFF_L = PCA9685_LED4_OFF_L;
            registers.ON_H = PCA9685_LED4_ON_H;
            registers.ON_L = PCA9685_LED4_ON_L;
            break;

        case ch5:
            registers.OFF_H = PCA9685_LED5_OFF_H;
            registers.OFF_L = PCA9685_LED5_OFF_L;
            registers.ON_H = PCA9685_LED5_ON_H;
            registers.ON_L = PCA9685_LED5_ON_L;
            break;

        case ch6:
            registers.OFF_H = PCA9685_LED6_OFF_H;
            registers.OFF_L = PCA9685_LED6_OFF_L;
            registers.ON_H = PCA9685_LED6_ON_H;
            registers.ON_L = PCA9685_LED6_ON_L;
            break;

        case ch7:
            registers.OFF_H = PCA9685_LED7_OFF_H;
            registers.OFF_L = PCA9685_LED7_OFF_L;
            registers.ON_H = PCA9685_LED7_ON_H;
            registers.ON_L = PCA9685_LED7_ON_L;
            break;

        case ch8:
            registers.OFF_H = PCA9685_LED8_OFF_H;
            registers.OFF_L = PCA9685_LED8_OFF_L;
            registers.ON_H = PCA9685_LED8_ON_H;
            registers.ON_L = PCA9685_LED8_ON_L;
            break;

        case ch9:
            registers.OFF_H = PCA9685_LED9_OFF_H;
            registers.OFF_L = PCA9685_LED9_OFF_L;
            registers.ON_H = PCA9685_LED9_ON_H;
            registers.ON_L = PCA9685_LED9_ON_L;
            break;

        case ch10:
            registers.OFF_H = PCA9685_LED10_OFF_H;
            registers.OFF_L = PCA9685_LED10_OFF_L;
            registers.ON_H = PCA9685_LED10_ON_H;
            registers.ON_L = PCA9685_LED10_ON_L;
            break;

        case ch11:
            registers.OFF_H = PCA9685_LED11_OFF_H;
            registers.OFF_L = PCA9685_LED11_OFF_L;
            registers.ON_H = PCA9685_LED11_ON_H;
            registers.ON_L = PCA9685_LED11_ON_L;
            break;

        case ch12:
            registers.OFF_H = PCA9685_LED12_OFF_H;
            registers.OFF_L = PCA9685_LED12_OFF_L;
            registers.ON_H = PCA9685_LED12_ON_H;
            registers.ON_L = PCA9685_LED12_ON_L;
            break;

        case ch13:
            registers.OFF_H = PCA9685_LED13_OFF_H;
            registers.OFF_L = PCA9685_LED13_OFF_L;
            registers.ON_H = PCA9685_LED13_ON_H;
            registers.ON_L = PCA9685_LED13_ON_L;
            break;

        case ch14:
            registers.OFF_H = PCA9685_LED14_OFF_H;
            registers.OFF_L = PCA9685_LED14_OFF_L;
            registers.ON_H = PCA9685_LED14_ON_H;
            registers.ON_L = PCA9685_LED14_ON_L;
            break;

        case ch15:
            registers.OFF_H = PCA9685_LED15_OFF_H;
            registers.OFF_L = PCA9685_LED15_OFF_L;
            registers.ON_H = PCA9685_LED15_ON_H;
            registers.ON_L = PCA9685_LED15_ON_L;
            break;

        case all:
            registers.OFF_H = PCA9685_ALL_LED_OFF_H;
            registers.OFF_L = PCA9685_ALL_LED_OFF_L;
            registers.ON_H = PCA9685_ALL_LED_ON_H;
            registers.ON_L = PCA9685_ALL_LED_ON_L;
            break;
    }

    //algorithm is time-on to time-off (delay is defined when time-on >0)

    if (dutyCycle == 0) {
        OFF_H.fields.fullOff = 1;
        ON_H.fields.fullOn = 0;
    } else if (dutyCycle >= 4096) {
        OFF_H.fields.fullOff = 0;
        ON_H.fields.fullOn = 1;
    } else {
        OFF_L.fields.value = (dutyCycle & 0x0FF);
        OFF_H.fields.value = (dutyCycle & 0xF00) >> 8;
        ON_L.fields.value = 0;
        ON_H.fields.value = 0;
    }

    printf("%d\t%x\t%x\t%x\t%x\t\n", dutyCycle, OFF_H.raw, OFF_L.raw, ON_H.raw, ON_L.raw);

    PCA9685_WriteRegister(registers.OFF_H, OFF_H.raw);
    PCA9685_WriteRegister(registers.OFF_L, OFF_L.raw);
    PCA9685_WriteRegister(registers.ON_H, ON_H.raw);
    PCA9685_WriteRegister(registers.ON_L, ON_L.raw);



}

void PCA9685_SetFrequency(uint8_t frequency) {

}

void PCA9685_Restart() {

    wiringPiI2CWrite (i2c_fd, 0x00);
    wiringPiI2CWrite (i2c_fd, 0x06);

    printf("Reset\n");
}
