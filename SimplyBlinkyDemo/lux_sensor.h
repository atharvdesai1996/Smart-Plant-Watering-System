/********************************
 * lux_sensor.h
 *
 *  Created on: , 2018
 *  Author: Hunter Hedges
 *
 ********************************/

#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#include <gpio.h>
#include <time.h>
#include "stdint.h"
#include "interrupt.h"
#include <printf.h>

/* Variable for storing lux value returned from VEML7700 */
float lux;
// 7 bit slave address = 0x10
#define VEML7700_ADDR 0x10
// According 8 bit the bus address is then 0010 0000 = 20h for write and 0010 0001 = 21h for read

void I2C_init(void);
void configure_sensor(void);
void I2C_write(uint8_t slaveAddress, uint8_t *data, uint8_t length);
void read_light_level(void);
void get_veml700_value(void);


#endif
