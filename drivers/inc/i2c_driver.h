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

	uint32_t I2C_SCLSpeed;  	 // I2C Serial Clock Speed
	uint8_t  I2C_DeviceAddress;  //	The device address  (7-bit address, hence uint8_t used) is only needed if the I2C peripheral is in slave mode.
	uint8_t  I2C_ACKControl;     // Enable the automatic Acknowledgment (for every byte sent/received) for the module -> By default it is not enabled.
	uint8_t  I2C_FMDutyCylce; 	 // The Duty Cycle if the I2C is in master mode and is in Fast Mode I2C - i.e. Communication speed can be set between (100-400] KHz by specifying a duty cycle

}I2C_Config_t;


/*
 * 				A Handle Structure for an I2C peripheral.
 * 		Holds I2Cx configuration and Base address of the I2C being configured - i.e. I2C1, I2C2, or I2C3
 */

typedef struct{

	I2C_RegDef_t *pI2Cx_BASEADDR;  /* Holds the Base address of the desired I2C peripheral (I2C1/I2C2/I2C3), the Base addresses defined in device header as MACROS, I2C1, I2C2, and I2C3 */
	I2C_Config_t I2C_Config; 	   /* Holds the I2C configuration information - filled out by the user before calling using any of the API functions defined in this file. */

}I2C_Handle_t;


/*
 *     I2C driver specific MACROs.
 *
 *          These macros are only relevant to I2C driver &
 *          	they are to be used when configuring an I2C Module parameter shown in I2C_Config_t struct above.
 *
 */


/*
 * @I2C_SCLSpeed
 *   // Possible I2C Device Speeds
 *   	SM = Standard/Normal Mode: <=100KHz
 *   	FM = Fast Mode:  (100KHz, 400KHz]
 */
#define I2C_SCL_SPEED_SM_HZ			100000
#define I2C_SCL_SPEED_FM_4K_HZ		400000
#define I2C_SCL_SPEED_FM_2K_HZ		200000

/*
 * @I2C_DeviceAddress -> There are no values for this. This is initialized by the user and has to be a 7-bit value.
 */

/*
 * @I2C_ACKControl
 * 			By Default the automatic Acknowledgment is disabled -> ACK bit field defined in CR1 register of I2C is '0' by default
 *
 */

#define I2C_ACK_ENABLE 		1
#define I2C_ACK_DISABLE		0


/*
 * @I2C_FMDutyCycle
 * 	The Fast Mode I2C can have a Duty Cycle, there are only two possible values for this according to user manual in the CCR register DUTY field
 * 		DUTY could be either '2' or '16/9'
 * 			For a Duty of 2 = Clear DUTY Field, i.e. DUTY = 0
 * 			For a Duty of 16/9 = Set DUTY Field, i.e. DUTY = 1
 */

#define I2C_FM_DUTY_2		0
#define I2C_Fm_DUTY_16_9	1


#endif /* DRIVERS_INC_I2C_DRIVER_H_ */
