/*
 * IMU.h
 *
 *  Created on: Mar 13, 2024
 *      Author: masondev
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

/**
 * init_IMU with default params
 * mode = IMU
 * range = +/- 4Gs
 * unit = m/s^2
 */
void init_IMU(I2C_HandleTypeDef *hi2c);

/**
 * init_IMU with custom parameters ( not really supported tbh)
 * mode = 	4 bits controlling op mode --> 0b1000 for IMU
 * range:	2 bits for accel. range, 2, 4, 8, 16
 * unit:	1 bit for unit, 0 = m/s^2, 1 = mg
 * not sure why i made this function, other functions depend on default values
 */
void init_IMU_custom(I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t range, uint8_t unit);

/**
 * effective aliases for those annoyin HAL master transmit functions
 */
uint8_t transmit_buf(I2C_HandleTypeDef *hi2c, uint8_t* buf, uint16_t bytes);
uint8_t read_to_buf(I2C_HandleTypeDef *hi2c, uint8_t subAddr, uint8_t* buf, uint16_t bytes);

/**
 * check_ret() prints to terminal if there is an issue with i2c communication
 * return type: uint8_t bool (1 or 0)
 * 		true if no error
 * 		false if error
 */
uint8_t check_ret(HAL_StatusTypeDef ret);

// modifies 3 element array of int16s: x, y, z, components of lin acc vec
void lin_acc_vec_raw(I2C_HandleTypeDef *hi2c, int16_t* vec);
// individual components
int16_t x_lin_acc_raw(I2C_HandleTypeDef *hi2c);
int16_t y_lin_acc_raw(I2C_HandleTypeDef *hi2c);
int16_t z_lin_acc_raw(I2C_HandleTypeDef *hi2c);

// modifies 3 element array of floats: x, y, z, components of lin acc vec
void lin_acc_vec(I2C_HandleTypeDef *hi2c, float* vec);
// individual components
float x_lin_acc(I2C_HandleTypeDef *hi2c);
float y_lin_acc(I2C_HandleTypeDef *hi2c);
float z_lin_acc(I2C_HandleTypeDef *hi2c);

// modifies 3 element array of int16s: x, y, z, components of gravity vector
void grav_vec_raw(I2C_HandleTypeDef *hi2c, int16_t* vec);
// individual components
int16_t x_grav_raw(I2C_HandleTypeDef *hi2c);
int16_t y_grav_raw(I2C_HandleTypeDef *hi2c);
int16_t z_grav_raw(I2C_HandleTypeDef *hi2c);

// modifies 3 element array of floats: x, y, z, components of gravity vector
void grav_vec(I2C_HandleTypeDef *hi2c, float* vec);
// individual components
float x_grav(I2C_HandleTypeDef *hi2c);
float y_grav(I2C_HandleTypeDef *hi2c);
float z_grav(I2C_HandleTypeDef *hi2c);

#endif /* INC_IMU_H_ */
