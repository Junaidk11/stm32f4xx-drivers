/*
 * i2c_driver.c
 *
 *  Created on: Apr 3, 2021
 *      Author: junaidkhan
 */

#include "i2c_driver.h"

/*********************************************************************
 * @fn      		  - I2C_ClockControl
 *
 * @brief             - This API function will be used to enable clock access to one of the I2C peripherals.
 *
 * @param[in]         -  a pointer to the base address of the SPI peripheral we want to configure - I2C_RegDef_t -> points to the base-address of the I2Cx (either I2C1, I2C2, or I2C3)
 * @param[in]         -  a variable to indicate if you wish to enable or disable the port. Define a Generic macro in MCU header file, use it - Use MACRO: Enable or Disable.
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -

 */
void I2C_ClockControl(I2C_RegDef_t *pI2Cx,uint8_t enable_disable ){

		/*
		 *  Check if the enable_disable argument is enable or disable, and then use the
		 *  clock enable macros that we have written in the MCU specific file.
		 */
		if(enable_disable == ENABLE){

			if(pI2Cx == I2C1){
				I2C1_PERIPH_CLOCK_EN(); // I2C1 enable clock access MACRO defined in MCU specific header file.
			}else if(pI2Cx == I2C2){
				I2C2_PERIPH_CLOCK_EN();
			}else if(pI2Cx == I2C3){
				I2C3_PERIPH_CLOCK_EN();
			}
		}else if(enable_disable == DISABLE){
			if(pI2Cx == I2C1){
				I2C1_PERIPH_CLOCK_DI(); // I2C1 disable clock access MACRO defined in MCU specific header file.
			}else if(pI2Cx == I2C2){
				I2C2_PERIPH_CLOCK_DI();
			}else if(pI2Cx == I2C3){
				I2C3_PERIPH_CLOCK_DI();
			}
		}
}

