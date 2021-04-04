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
 * @param[in]         -  a pointer to the base address of the I2C peripheral we want to configure - I2C_RegDef_t -> points to the base-address of the I2Cx (either I2C1, I2C2, or I2C3)
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


/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - Resets the I2Cx registers
 *
 * @param[in]         - The Base address of your I2C module - I2C_RegDef_t*
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  None
 *
 * @Note              -  You can use RCC's AHB1 Peripheral Reset Register.
   		                 This way, you don't have to reset each every register of the respective I2Cx.

 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

    if(pI2Cx == I2C1){
        I2C1_REG_RESET();
    }else if(pI2Cx == I2C2){
        I2C2_REG_RESET();
    }else if(pI2Cx == I2C3){
        I2C3_REG_RESET();
    }
}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - I2C Peripheral Enable/Disable API
 *
 * @param[in]         - Base address of the I2Cx
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  None
 *
 * @Note              - None
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t Enable_Disable){

    if(Enable_Disable == ENABLE){
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }else if(Enable_Disable == DISABLE){
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

/*********************************************************************
 * @fn      		  - I2C_IRQ_Interrupt_Config
 *
 * @brief             - This function is used to configure the Processor side of interrupt configuration and handling. Use this function to configure the IRQs and enable them.
 * 						 By default, all IRQs are disabled. Therefore, for the processor to accept an interrupt from a peripheral, the assigned IRQ number to that peripheral should be
 * 						 configured in the NVIC.
 * @param[in]         - The IRQ number corresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - Priority of the interrupt.
 * @param[in]         - ENABLE or DISABLE
 *
 * @return            - None
 *
 * @Note              - None

*/
void I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t enable_disable){
		if( enable_disable == ENABLE){
			if(IRQNumber <= 31){

				// Program the NVIC_ISER0 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
				(*NVIC_ISER0) |= (1 << IRQNumber);

			}else if (IRQNumber > 31 && IRQNumber < 64){  // Interrupt lines  from 32 - 63

				// Program the NVIC_ISER1 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
				// 32 % 32 = 0 --> bit 0 of the NVIC_ISER1 register, 33 % 32 == 1 --> bit 1 of the NVIC_ISER1 register...
				(*NVIC_ISER1) |= (1 << (IRQNumber % 32));

			}else if(IRQNumber >64 && IRQNumber < 96){   // IRQ lines from 64 - 96; I2C1, I2C2 and I2C3 Event and Error Interrupt lines are in this range

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

			}else if(IRQNumber >64 && IRQNumber < 96){   // IRQ lines from 64 - 96; I2C1, I2C2 and I2C3 Event and Error Interrupt lines are in this range

				// Program the NVIC_ICER2 Register of the NVIC Controller - Registers Defined in the Cortex-M4 Generic User Guide, under the NVIC Section
				// 64 % 64 = 0 --> bit 0 of the NVIC_ICER2 register, 65 % 64 == 1 --> bit 1 of the NVIC_ICER2 register...
				(*NVIC_ICER2) |= (1 << (IRQNumber % 64));
			}

			/*   You can add the rest of the NVIC_ICERx registers, there are a total of 8 NVIC_ISER registers. */
		}
}

/*********************************************************************
 * @fn      		  - I2C_IRQ_Priority_Config
 *
 * @brief             - This function can be used to configure the priority of the given IRQ number.
 *
 * @param[in]         - the IRQ number corresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - The priority of the IRQNumber
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              - None

*/
void I2C_IRQ_Priority_Config (uint8_t IRQNumber, uint8_t IRQPriority){

	//1. Find the IPRx register assigned to the IRQNumber
	uint8_t iprx = IRQNumber /4;
	//2. Find the section of the iprx register assigned to the IRQNumber
	uint8_t iprx_Section = IRQNumber % 4;

	//3. Set the IRQPriority in the respective iprx register using the NVIC_IPRR BaseAddr defined in the device specific header file.
	//				Each NVIC_IPRx register is 32-bits wide, iprx is an 8-bit value - 1 byte, to get to the NVIC_IPRx register corresponding to iprx, you need to add 4 bytes for each register till you reach NVIC_IPRx register corresponding to iprx.
	//							|					Multiply by 8 because each section has 8 bits.
	//							|								|
	//*(NVIC_IPR_BASEADDR + (iprx * 4 )) |= (IRQPriority << 8 * iprx_Section);  --> Explained in Notes; Updated: don't need the 4, because NVIC_IPR_BASEADDR is a 32-bit pointer, incrementing by 1 will point to the next 32-bit register.

	//		To get to the corrected section of the iprx register
	//								|			To fill to the Top 4 bits of the section, as the bottom 4 bits are N.A
	//													|
	uint8_t shift_amount = (8 * iprx_Section) + (8 - NO_PR_BITS_IMPLEMENTED);


	*(NVIC_IPR_BASEADDR + (iprx)) |= (IRQPriority << shift_amount);

	// Removed the *4 from statement above b/c the NVIC_IPR_BASEADDR is defined as a 32-bit pointer (in the device header file) as ((__vo uint32_t*)0xE000E400),
	// Therefore, an increment by 1, will move the pointer to ((__vo uint32_t*)0xE000E404), i.e. increment of 4 bytes (4*8 = 32 bits, i.e. next register). Therefore, you don't need to multiply by 4.

}
