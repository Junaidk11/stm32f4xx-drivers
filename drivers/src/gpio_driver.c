/*
 * gpio_driver.c
 *
 *  Created on: May 18, 2020
 *      Author: Junaid
 */

#include "gpio_driver.h" /* include its header file. */


/*
 *  		GPIO Port Clock Access
 *
 *  	This API function will be used to enable clock access to one of the GPIO Ports.
 *  	Therefore: The arguments of this function should be:
 *  		1) a pointer to the base address of the port we want to configure - GPIO_RegDef_t -> points to the base-address of the port
 *  		2) a variable to indicate if you wish to enable or disable the port. Define a Generic macro in MCU header file, use it - Use MACRO: Enable or Disable.
 *
 */
/*********************************************************************
 * @fn      		  - GPIO_ClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */


void GPIO_ClockControl(GPIO_RegDef_t *pGPIO_PORT,uint8_t enable_disable ){

	/*
	 *  Check if the enable_disable arugment is enable or disable, and then use the
	 *  clock enable macros that we have written in the MCU specific file.
	 */
	if(enable_disable == ENABLE){

		if(pGPIO_PORT == GPIOA){
			GPIOA_PERIPH_CLOCK_EN(); // GPIOA enable clock access MACRO defined in MCU specific header file.
		}else if(pGPIO_PORT == GPIOB){
			GPIOB_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOC){
			GPIOC_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOD){
			GPIOD_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOE){
			GPIOE_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOF){
			GPIOF_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOG){
			GPIOG_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOH){
			GPIOH_PERIPH_CLOCK_EN();
		}else if(pGPIO_PORT == GPIOI){
			GPIOI_PERIPH_CLOCK_EN();
		}
	}else if(enable_disable == DISABLE){
		if(pGPIO_PORT == GPIOA){
			GPIOA_PERIPH_CLOCK_DI(); // GPIOA disable clock access MACRO defined in MCU specific header file.
		}else if(pGPIO_PORT == GPIOB){
			GPIOB_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOC){
			GPIOC_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOD){
			GPIOD_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOE){
			GPIOE_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOF){
			GPIOF_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOG){
			GPIOG_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOH){
			GPIOH_PERIPH_CLOCK_DI();
		}else if(pGPIO_PORT == GPIOI){
			GPIOI_PERIPH_CLOCK_DI();
		}
	}
}

/*
 * 			GPIO Port Init and Deinit
 *
 * 			For Initialization of GPIO port register:
 * 		This API function will configure the pin that you wish to use.
 * 		The Arguments of this function should be:
 * 		1) Pointer to the GPIO handle; GPIO_Handle_t *
 *
 * 			For Reseting the GPIO port registers:
 * 		You can use RCC's AHB1 Peripheral Reset Register.
* 		This way, you don't have to reset each every register of the Port.
* 		The Argument of this function should be:
* 		1) The Base address of your GPIO port - GPIO_RegDef_t*
 *
 */

/*********************************************************************
 * @fn      		  -  GPIO_Init
 *
 * @brief             - This function is used to configure the GPIO port and pin.
 *
 * @param[in]         - Holds pin's configuration and Base address of the Port being configured.
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
		
	uint8_t temp = 0;   // Use this temp variable to store the value that will be assigned to the register of the specific GPIO Port

	if(pGPIOHandle->PinConfig.PinMode <= GPIO_PIN_ANALOG_MODE){ // i.e. the GPIO pin mode selected is a non-interrupt mode

				// 1. configure the mode of GPIO pin
//			The info filled by the user before calling the API function.
								// |
				temp = (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber)); // GPIO's MODE register has 2 bits dedicated to each pin of the PORT, hence the need for '2' for shifting to the right pin.

				// Clear the desired bits before setting them.
//														  | the 0x3 is because we're clearing 2 bits, which in decimal is a 3. Also, the left shift argument is to clear the respective field.
				pGPIOHandle->pGPIOx_BASEADDR->MODER &= ~(0x3 << (2 * pGPIOHandle->PinConfig.PinNumber));
//			Grabs the Physical Memory address dedicated to Mode register of your desired GPIO Port
								// |
				pGPIOHandle->pGPIOx_BASEADDR->MODER |= temp; 		// Assign temp value to the MODER register using the base-address of your PORT.
	}else {

		// Do this for if the selected pin mode is one of the Interrupt Modes.

		if(pGPIOHandle->PinConfig.PinMode == GPIO_PIN_INPUT_FALLING_EDGE_INTERRUPT_MODE){
			// 1. Configure the Falling Edge Trigger Selection Register  
			  EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber); 

			// Clear the Corresponding Rising Edge Trigger Selection Register - Just to be safe.   
			  EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber); 

		}else if (pGPIOHandle->PinConfig.PinMode == GPIO_PIN_INPUT_RISING_EDGE_INTERRUPT_MODE){
			// 1. Configure the Rising Edge Trigger Selection Register  
			  EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber); 

			// Clear the Corresponding Falling Edge Trigger Selection Register - Just to be safe.   
			  EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber); 

		}else if(pGPIOHandle->PinConfig.PinMode == GPIO_PIN_INPUT_RISING_FALLING_INTERRUPT_MODE){

			// 1. Configure both Rising Edge Trigger & Falling Edge Trigger Selection Register 
			  EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber); 

			// Clear the Corresponding Falling Edge Trigger Selection Register - Just to be safe.   
			  EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber); 	  
		}
		 	// 2. Select the GPIO port using the SYSCFG_EXTICR register.

			// There are 4 EXTI Control registers(EXTI0-EXTI4)- Integer division of Pin Number by 5, this will give you the EXTICR register to configure 
			// Each EXTIx assigned 4 bits - to find which one to configure in the respective EXTICRx register, find remainder of Pin NUmber / 5 
			
				uint8_t temp1;
				uint8_t temp2; 

				// Gives the value of EXTICRx Register 
				temp1 = (pGPIOHandle->PinConfig.PinMode) / 5; 
				// Gives the value of which field in EXTICRx to configure
				temp2 = (pGPIOHandle->PinConfig.PinMode) % 5; 

				uint8_t portCode = GPIO_BASEADDR_TO_PORTCODE(pGPIOHandle->pGPIOx_BASEADDR); // This macro will return the portCode corresponding to the baseaddress it receives.
				SYSCFG_PERIPH_CLOCK_EN();   												// Enable clock access to System Configuration Peripheral before you configure its register
				//								The starting position of the field
				//										|
				SYSCFG->EXTICR[temp1] |= portCode << (temp2 * 4); 
				
			// 3. Enable the respective EXTI line to allow interrupts to be send to the Processor via NVIC - using Interrupt Mask Register
		
			EXTI->IMR |= (1<< pGPIOHandle->PinConfig.PinNumber); // This will Enable the EXTI line corresponding to the Pin number. 
			
	}

	
	// 2. Configure slew rate of the GPIO pin
	temp = 0; 			// Reset temp, can use temp for next register's value.
	temp = (pGPIOHandle->PinConfig.PinSpeed << (2 * pGPIOHandle->PinConfig.PinNumber)); // Set the value to be assigned to the Speed register.

	// Clear the bits before setting them
	//											| two bits dedicated for each pin.
	pGPIOHandle->pGPIOx_BASEADDR->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->PinConfig.PinNumber));

	// Setting the bits after clearing them first.
	pGPIOHandle->pGPIOx_BASEADDR->OSPEEDR |= temp; 		// Assign the value temp to the speed register of your specific GPIO port.

	// 3. Configure Pull-up/Pull-down resistor settings of the pin - to avoid floating state
	temp = 0;     		// Rest temp
	temp = (pGPIOHandle->PinConfig.PinPuPdControl << (2 * pGPIOHandle->PinConfig.PinNumber));

	// Clear the bits before setting them.
	pGPIOHandle->pGPIOx_BASEADDR->PUPDR &= ~(0x3 << (2* pGPIOHandle->PinConfig.PinNumber));
	// Set the bits
	pGPIOHandle->pGPIOx_BASEADDR->PUPDR |= temp;

	// 4. configure the output type of the pin
	temp = 0;
	temp = (pGPIOHandle->PinConfig.PinOType << (pGPIOHandle->PinConfig.PinNumber)); // The Output type register has 1 bit dedicated for each pin of the port.

	// Clear the bits before setting them.
	//										   | 1 bit field for each pin - check the Reference manual
	pGPIOHandle->pGPIOx_BASEADDR->OTYPER &= ~(0x1 << (pGPIOHandle->PinConfig.PinNumber));
	// Set the bits
	pGPIOHandle->pGPIOx_BASEADDR->OTYPER |= temp;

	// 5. configue the alternate functionality of the GPIO pin.

	if(pGPIOHandle->PinConfig.PinMode == GPIO_PIN_ALTFUNC_MODE){ // Only configure the Alternate Functionality if the user has set the Pin mode to be in Alternate Function mode, else skip this part.

				// There are two register's dedicated for selecting Alternate functionality, four bit for each pin.
				// ALFL = alternate functionalit low for pins 0-7  = 		AFR[0]
				// ALFH = alternate functionality high for pins 8-15 =		AFR[1]
				// Each Alternate Functionality Register (AFR) has 4 bits dedicated to each pin.
				// AFR[0] is dedicated to pins 0-7, and each pin gets 4 bits to decide its functionality
				// AFR[1] is dedicated to pin  8-15, and each pin gets 4 bits to decide its functionality.
				// The value that will be set in the 4 bits is placed by the user in the PinAltFunMode field of the GPIO_PinConfig_t structure.

			// First we need to decide which AFR register to use, this will decided using the PinNumber field set by the user in the GPIO_PinConfig_t structure.
			// Since, each pin is given 4 bits, and each AFR register has 8 pins dedicated, integer division of PinNumber by 8, will give the dedicated AFR register for the pin.
			uint8_t temp1, temp2; // reset temp  //You can also use uint8_t instead of uint32_t - how?? AFR is 32 bits long.
			temp1 = (pGPIOHandle->PinConfig.PinNumber) / 8;

			// Now, to find the field of the dedicated AFR register to configure, you find the remainder of the PinNumber divided by 8, and shift the value in PinAltFunMode of field by 4 times that value. (4 times b/c each pin has 4 bits dedicated to it, in each AFR register)
			temp2 = (pGPIOHandle->PinConfig.PinNumber) % 8;

			// Clear the bits before setting them.
			//											   | 4-bit field, all 4 bits in Decimal number = 15 == F
			pGPIOHandle->pGPIOx_BASEADDR->AFR[temp1] &= ~(0xF << (4 *(temp2)));
			// Set the bits
			// Now configure the physical address dedicated to setting the alternate functionality mode.
			pGPIOHandle->pGPIOx_BASEADDR->AFR[temp1] |= (pGPIOHandle->PinConfig.PinAltFunMode << (4 *(temp2)));

/*  Alternate way of doing it.
			if(temp == 1){  //PinNumber belongs to AFR[1] register

				pGPIOHandle->pGPIOx_BASEADDR->AFR[1] |= (pGPIOHandle->PinConfig.PinAltFunMode << (4 *(pGPIOHandle->PinConfig.PinNumber % 8)));
			}else if(temp==0){  // PinNumbr belong to AFR[0] register
				pGPIOHandle->pGPIOx_BASEADDR->AFR[1] |= (pGPIOHandle->PinConfig.PinAltFunMode << (4 *(pGPIOHandle->PinConfig.PinNumber % 8)));

			}*/

	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This API is used to reset the GPIO registers.
 *
 * @param[in]         - Base address of your GPIO port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Use RCC peripheral register to reset all the GPIO registers instead of reseting all registers manually.
 * 						Also, you need to set the bit in the RCC before you clear it for resetting.

*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIO_PORT){

			if(pGPIO_PORT == GPIOA){
				GPIOA_REG_RESET();				// GPIO port A register reset MACRO defined in MCU specific header file.
			}else if(pGPIO_PORT == GPIOB){
				GPIOB_REG_RESET();
			}else if(pGPIO_PORT == GPIOC){
				GPIOC_REG_RESET();
			}else if(pGPIO_PORT == GPIOD){
				GPIOD_REG_RESET();
			}else if(pGPIO_PORT == GPIOE){
				GPIOE_REG_RESET();
			}else if(pGPIO_PORT == GPIOF){
				GPIOF_REG_RESET();
			}else if(pGPIO_PORT == GPIOG){
				GPIOG_REG_RESET();
			}else if(pGPIO_PORT == GPIOH){
				GPIOH_REG_RESET();
			}else if(pGPIO_PORT == GPIOI){
				GPIOI_REG_RESET();
			}

}


/*
 *   		Data Read & Write
 */

 /* Arguments: To read a pin, you need GPIO Base address & pin number. The returned value will either be 0 or 1, uint8_t is the closest data type to return. */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Reading the value of an input pin.
 *
 * @param[in]         -Base address of the GPIO port the input is connected to.
 * @param[in]         -The pin you want to read
 * @param[in]         -
 *
 * @return            -0 or 1
 *
 * @Note              -

*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber){
	uint8_t value;
//					Grab the Input Data Register, followed by leftshifting the desired bit to the LSB position, followed by masking the remaining bits from Bit 0 - Bit 31, followed by typecasting the uint32_t  as uint8_t, to avoid compiler error.
//							|
	value = (uint8_t) ((pGPIO_PORT->IDR >> pinNumber) & 0x00000001);
	return value;
}

/* Arguments: To read a PORT, you need GPIO Base address. The returned value will be 16 bits long, each PORT has 16 pins. uint16_t is the closest data type to return. */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Reading the entire port data
 *
 * @param[in]         - Base address of the Port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 16 bit value with each bit corresponding to the data on the respective pin.
 *
 * @Note              - Each port as 16 pins

*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO_PORT){
	uint16_t value;

	value = (uint16_t) pGPIO_PORT->IDR;
	return value;
}

/* Arguments: To write to a pin, you need GPIO Base address, pin number & the value (use Generic Macros defined in MCU header file.) you want to write to the pin.*/

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Writing 1 or 0 to the respective pinNumber of the GPIO port.
 *
 * @param[in]         -GPIO port base address
 * @param[in]         -The respective pin number you wish to set or clear
 * @param[in]         -GPIO_PIN_SET or GPIO_PIN_CLEAR from @GPIO_PIN_SET_CLEAR_MACROS
 *
 * @return            -none
 *
 * @Note              -

*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber, uint8_t value){

	if(value == GPIO_PIN_SET){
		pGPIO_PORT->ODR |= (1 << pinNumber);
	}else if(value == GPIO_PIN_CLEAR){
		pGPIO_PORT->ODR &= ~(1 << pinNumber);
	}
}

/* Arguments: To write to a PORT, you need GPIO Base address & the value you want to write to the  port- value has to 16 bits long, each Port has 16 pins. */

/*********************************************************************
 * @fn      		  -GPIO_WriteToOutputPort
 *
 * @brief             -Writing 16 bit value to the entire GPIO port
 *
 * @param[in]         -Base address of the PORT you want to write to.
 * @param[in]         -The 16-bit value you want to set at the port.
 * @param[in]         -
 *
 * @return            -none
 *
 * @Note              -

*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO_PORT, uint16_t Value){
			pGPIO_PORT->ODR = Value;
}

/* Arguments: To toggle a specific output pin, you only need the Base address of the GPIO port & the pin number you wish to toggle. */

/*********************************************************************
 * @fn      		  -GPIO_ToggleOutputPin
 *
 * @brief             -Toggling a GPIO pin
 *
 * @param[in]         -Base address of the GPIO port
 * @param[in]         -The pin you wish to toggle
 * @param[in]         -
 *
 * @return            -none
 *
 * @Note              -

*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO_PORT, uint8_t pinNumber){
		pGPIO_PORT->ODR ^= (1 << pinNumber);
}


/*
 *			GPIO Interrupt Configuration & Handling
 */

/* Arguments: You need IRQ number, the interrupt priority, and variable to hold enable or disable command. */

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - This function is used to configure the Processor side of interrupt configuration and handling. Use this function to configure the IRQs and enable them. 
 * 						 By default, all IRQs are disabled. Therefore, for the processor to accept an interrupt from a peripheral, the assigned IRQ number to that peripheral should be 
 * 						 configured in the NVIC. 
 * @param[in]         - the IRQ numebr coresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - Priority of the interrupt.
 * @param[in]         - ENABLE or DISABLE 
 *
 * @return            - none
 *
 * @Note              -

*/
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t enable_disable){
	if( enable_disable == ENABLE){
		if(IRQNumber <= 31){

			// Program the NVIC_ISER0 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			(*NVIC_ISER0) |= (1 << IRQNumber); 
		
		}else if (IRQNumber > 31 && IRQNumber < 64){  // Interrupt lines  from 32 - 63

			// Program the NVIC_ISER1 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			// 32 % 32 = 0 --> bit 0 of the NVIC_ISER1 register, 33 % 32 == 1 --> bit 1 of the NVIC_ISER1 register...
			(*NVIC_ISER1) |= (1 << (IRQNumber % 32)); 

		}else if(IRQNumber >64 && IRQNumber < 96){   // IRQ lines from 64 - 96 

			// Program the NVIC_ISER2 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			// 64 % 64 = 0 --> bit 0 of the NVIC_ISER2 register, 65 % 64 == 1 --> bit 1 of the NVIC_ISER2 register...
			(*NVIC_ISER2) |= (1 << (IRQNumber % 64)); 
		}
		/*   You can add the rest of the NVIC_ISERx registers, there are a total of 8 NVIC_ISER registers. */ 
	}else if (enable_disable == DISABLE){
		if(IRQNumber <= 31){

			// Program the NVIC_ICER0 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			(*NVIC_ICER0) |= (1 << IRQNumber); 

		}else if (IRQNumber > 31 && IRQNumber < 64){  // Interrupt lines  from 32 - 63
			// Program the NVIC_ICER1 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			// 32 % 32 = 0 --> bit 0 of the NVIC_ICER1 register, 33 % 32 == 1 --> bit 1 of the NVIC_ICER1 register...
			(*NVIC_ICER1) |=  (1 << (IRQNumber % 32)); 

		}else if(IRQNumber >64 && IRQNumber < 96){   // IRQ lines from 64 - 96 

			// Program the NVIC_ICER2 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
			// 64 % 64 = 0 --> bit 0 of the NVIC_ICER2 register, 65 % 64 == 1 --> bit 1 of the NVIC_ICER2 register...
			(*NVIC_ICER2) |= (1 << (IRQNumber % 64)); 
		}

		/*   You can add the rest of the NVIC_ICERx registers, there are a total of 8 NVIC_ISER registers. */ 
	}

}

/*  For Interrupt handling, this API only needs to know the pin number that needs interrupt servicing. */

/*********************************************************************
 * @fn      		  - GPIO_IRQ_Priority_Config
 *
 * @brief             - This function can be used to configure the priority of the given IRQ number. 
 *
 * @param[in]         - the IRQ numebr coresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - The priority of the IRQNumber
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -

*/
void GPIO_IRQ_Priority_Config (uint8_t IRQNumber, uint8_t IRQPriority){

	//1. Find the IPRx register assigned to the IRQNumber
	uint8_t iprx = IRQNumber /4; 
	//2. Find the section of the iprx register assigned to the IRQNumber
	uint8_t iprx_Section = IRQNumber % 4; 

	//3. Set the IRQPriority in the respective iprx register using the NVIC_IPRR baseaddr defined in the device specific header file. 
	//				Each NVIC_IPRx register is 32-bits wide, iprx is an 8-bit value - 1 byte, to get to the NVIC_IPRx register corresponding to iprx, you need to add 4 bytes for each register till you reach NVIC_IPRx register corresponding to iprx.  	
	//							|					Multiply by 8 because each section has 8 bits.
	//							|								|
	//*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPriority << 8 * iprx_Section);  --> Explained in Notes. 
	
	//		To get to the corrected section of the iprx register
	//								|			To fill to the Top 4 bits of the section, as the bottom 4 bits are N.A		
	//													|
	uint8_t shift_amount = (8 * iprx_Section) + (8 - NO_PR_BITS_IMPLEMENTED); 
	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPriority << shift_amount); 
}
/*  For Interrupt handling, this API only needs to know the pin number that needs interrupt servicing. */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

*/
void GPIO_IRQHandling(uint8_t pinNumber){

}
