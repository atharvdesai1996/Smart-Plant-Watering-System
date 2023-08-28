/********************************
 * lux_sensor.h
 *
 *  Created on: , 2018
 *  Author: Hunter Hedges
 *
 ********************************/

#ifndef LUX_SENSOR_POLL_H_
#define LUX_SENSOR_POLL_H_

#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
#include <gpio.h>
#include <time.h>
#include "stdint.h"
#include "interrupt.h"
#include <printf.h>
#include <msp.h>
#include "eusci.h"

/* Variable for storing lux value returned from VEML7700 */
float lux;
// 7 bit slave address = 0x10
#define VEML7700_ADDR 0x10
#define VEML_ADDR_8BIT_WRITE 0x20
#define VEML_ADDR_8BIT_READ 0x21
// According 8 bit the bus address is then 0010 0000 = 20h for write and 0010 0001 = 21h for read
#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ 1


uint8_t poll_interruptTX_flag(void);
void I2C_init_poll(void);
void configure_sensor_poll(void);
void I2C_write_poll(uint8_t slaveAddress, uint8_t *data, uint8_t length);
void read_light_level_poll(void);
void get_veml700_value_poll(void);

#endif
