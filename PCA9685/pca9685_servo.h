/* I2C bus Functions */
#ifndef _PCA9685_H
#define _PCA9685_H

#include <stdio.h>
#include <stm32f1xx.h>

// Setup registers
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define PCA9685_adrr 0x80
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

// Define first LED and all LED. We calculate the rest
// Define first LED and all LED. We calculate the rest
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define LEDALL_ON_L 0xFA

#define PIN_ALL 16

void PCA9685_Reset(void);
void PCA9685_Go(void);
void SetPWMFreq(float freq);
void setServo(uint32_t num,uint32_t off);
void PCA9685_write(uint8_t startAddress, uint8_t buffer);
extern uint16_t calculate_PWM(uint16_t angle);
extern uint8_t PCA9685_read(uint8_t startAddress);

#endif
