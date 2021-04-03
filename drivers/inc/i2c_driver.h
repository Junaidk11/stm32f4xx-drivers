/*
 * i2c_driver.h
 *
 *  Created on: Apr 3, 2021
 *      Author: junaidkhan
 */

#ifndef DRIVERS_INC_I2C_DRIVER_H_
#define DRIVERS_INC_I2C_DRIVER_H_

#include "stm32f40xx.h" /*  Include device specific header file. */

/*
 * 						An I2C configuration structure.
 * 		 	This structure holds the configuration settings of the I2Cx module.
 * 			  Place the configuration Structure inside the handle structure.
 */

typedef struct{


	/* I2C User Configurable Items are listed below - minimum  required for achieving an I2C communication protocol */








}I2C_Config_t;


/*
 * 				A Handle Structure for an I2C peripheral.
 * 		Holds I2Cx configuration and Base address of the I2C being configured - i.e. I2C1, I2C2, or I2C3
 */

typedef struct{

	I2C_RegDef_t *pI2Cx_BASEADDR;  /* Holds the Base address of the desired I2C peripheral (I2C1/I2C2/I2C3), the Base addresses defined in device header as MACROS, I2C1, I2C2, and I2C3 */
	I2C_Config_t I2C_Config; 	   /* Holds the I2C configuration information - filled out by the user before calling using any of the API functions defined in this file. */

}I2C_Handle_t;

#endif /* DRIVERS_INC_I2C_DRIVER_H_ */
