/*
 * gpio_driver.h
 *
 *  Created on: May 18, 2020
 *      Author: Junai
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f40xx.h" /*  Include device specific header file. */

/*
 * 						A GPIO pin configuration structure.
 * 		 			This structure holds the configuration settings of the pin.
 * 			 Place the configuration Structure inside the handle structure.
 */

typedef struct{

	uint8_t PinNumber;				/* Pin number is b/w 0-15 */
	uint8_t PinMode;
	uint8_t PinSpeed;				/* Pin's Slew rate:  How fast does pin change state? */
	uint8_t PinPuPdControl;  		/* Pin's Internal Pull-up or Pull-down Control */
	uint8_t PinOType; 				/* Pin Output type: Open-drain configuration/Push-pull Configuration */
	uint8_t PinAltFunMode; 			/* Pin's alternate Functionality Mode */
}GPIO_PinConfig_t;


/*
 * 				A Handle Structure for a GPIO pin.
 * 			Holds pin's configuration and Base address of the Port being configured.
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx_BASEADDR;   /* Holds the Base address of the desired Port (A-K), the Base addresses defined in device header as MACROS, GPIOA, GPIOB... */
	GPIO_PinConfig_t PinConfig; 	  /* Holds the pin configuration information. */

}GPIO_Handle_t;


/*
 * 					APIs supported by this GPIO driver.
 *
 */


/*
 *  		GPIO Port Clock Access
 */

void GPIO_ClockControl(void);

/*
 * 			GPIO Port Init and Deinit
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 *   		Data Read & Write
 */

void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);
/*
 *			GPIO Interrupt Configuration & Handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_GPIO_DRIVER_H_ */
