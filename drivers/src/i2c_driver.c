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
 * @fn      		  - RCC_GetPLLOutputClock
 *
 * @brief             - This function will be used to calculate PLL Clock Frequency
 *
 * @param[in]         -  None
 *
 * @return            -  uint32_t PLL Clock Frequency
 *
 * @Note              -
 *
 */
uint32_t RCC_GetPLLOutputClock(void){
	// Add body if using a PLL as clock source
	return 0;
}
/*********************************************************************
 * @fn      		  - RCC_GetPClK1Value
 *
 * @brief             - This function will be used to return the APB1 Peripheral Clock Val
 *
 * @param[in]         -  None
 *
 * @return            -  uint32_t APB1 Clock Frequency
 *
 * @Note              -  This function is used in the I2C_Init to configure the 'FREQ' field of CR2 register.
 * 							The function refers to RCC module in the Reference manual - Page 216 -> The clock tree.
 * 							Follow the trace from the APBx Peripheral Clocks to the source. Thats how you calculate it.
 * 								Step 1: Determine system clock source:  HSI? HSE? PLL? -> Check Bits 2 and 3 of CFGR Register of RCC Module
 * 								Step 2: Determine the AHB prescaler value (check clock tree) -> the value can be found from HPRE field of CFGR register of RCC
 * 								Step 3: Determine the APB1 Prescaler value(check clock tree) -> Value can be from the PPRE1 field of the CFGR register of RCC
 *
 */

uint32_t RCC_GetPClK1Value(void){

	uint32_t peripheral_clock1, systemClk_Hz;

	uint8_t clockSource, prescaler_AHB, prescaler_APB1;

	// ++ Step 1
	// Left shift CFGR register by 2 bits to get bit 2 to bit 0 position -> masking the result with 0x3, will clear all bits except bit 0 and bit 1, the bits you want.
	clockSource = ((RCC->RCC_CFGR >> 2) & 0x3);

	if(clockSource==0){
		// System Clock source is HSI Oscillator
		systemClk_Hz = 16000000;

	}else if(clockSource == 1){
		// System Clock source is HSE Oscillator
		systemClk_Hz = 8000000;

	}else if(clockSource == 2){
		// System Clock source is PLL
		systemClk_Hz = RCC_GetPLLOutputClock(); // Need to calculate this if you're using a PLL as clock source
	}

	// -- Step 1

	// ++ Step 2

	// Left shift CFGR register by 4 bits to get bit 4 to bit 0 position -> masking the result with 0xF (1111), will clear all bits except bit 0-3, the bits you want(the HPRE Field)
	prescaler_AHB =(RCC->RCC_CFGR>>4) & 0xF;

	// According to Reference manual, the HPRE Field, if < 8; system clock not divided, if >8; system clock divided based on information on pg 230 of Reference manual
	if(prescaler_AHB < 8){

		// System clock not divided, so systemClk_Hz remains as determined previously
		prescaler_AHB = 1; // I.e. systemClk_Hz is not divided.

	}else if(prescaler_AHB >= 8){

		// Create an Array that holds the AHB Prescalar possible values based on the value retrieved in prescaler_AHB
		uint16_t AHB_Prescalar[8] = {2,4,8,16,64,128,256,512};

		prescaler_AHB = AHB_Prescaler[prescaler_AHB-8]; // If prescaler_AHB = 8, then we get value 2 from index 0; if it is 9, then we get value 4 from index 1 etc. (from pg 230 of reference manual

	}

	// Updated systemClk_Hz based on determined AHB Prescaler
	systemClk_Hz /= prescaler_AHB;

	// -- Step 2

	// ++ Step 3

	// Left shift CFGR register by 10 bits to get bit 10 to bit 0 position -> masking the result with 0x7 (111), will clear all bits except bit 0-2, the bits you want(the PPRE1 Field)
	prescaler_APB1 = ((RCC->RCC_CFGR>>10) & 0x7);

	// According to Reference manual, the PPRE1 Field, if < 4; system clock is not divided, if >4; system clock divided based on information on pg 229 of Reference manual

	if(prescaler_APB1 < 4){
		// System clock not divided, so systemClk_Hz remains as updated in Step 2
		prescaler_APB1 = 1;

	}else if(prescaler_APB1>=4){
		// Create an Array that holds the APB1 Prescalar possible values based on the value retrieved in prescaler_APB1
		 uint16_t APB1_Prescalar[8] = {2,4,8,16};

		 prescaler_APB1 = APB1_Prescalar[prescaler_APB1-4]; // If prescaler_APB1 = 4, then we get value 2 from index 0; if it is 9, then we get value 4 from index 1 etc. (from pg 229 of reference manual
	}

	// Updated systemClk_Hz based on determined APB1 Prescaler
	systemClk_Hz /= prescaler_APB1;



	// Now, based on the clock tree in pg 216, we have calculated the Clock frequency of peripherals connected to APB1 bus.
	peripheral_clock1 = systemClk_Hz;

	return peripheral_clock1;

}


/*********************************************************************
 * @fn      		  -  I2C_Init
 *
 * @brief             -  Initializes I2Cx registers
 *
 * @param[in]         -  Pointer to the I2C handle; I2C_Handle_t *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -
 * 						Generic Steps for Initializing any I2C Module; The following configurations steps are to be done when the Peripheral is DISABLED in the Control Register
 * 								step1: Configure the Mode of the I2C -> Standard Mode or Fast Mode? Pick one
 * 								step2: Configure the speed of the serial clock (SCL); Remember the faster Serial Clock, the short the I2C bus length should be;
 * 								step3: Configure the device address (this is only applicable if the I2C module is being used on a slave device)
 * 								step4: Enable the Automatic Acknowledgment (ACKing) - ST has this disabled by default, maybe not be the case for other MCUs - just check
 * 								step5: Configure the rise time for the I2C pins; i.e. the slew rate has to be configured according to I2C specification, the slew rate is the time take
 * 									   the i2c pins (SDA and SCL) to go from ground to VCC. You need to configure this according to the I2C specification.
 *								Step6: Enable the peripheral iff you have configured all the steps above. Enable the peripheral from the CR1 register's 'PE' bit field.
 *
 *					   Expanding on 'Step 2: Configuring the Speed of the Serial Clock(SCL)'
 *					 	The Serial Clock to be generated for the I2C module is configured using the Bus frequency. The APB bus supplies clock to the
 *					 		  I2C modules of STM32F40x as they're all connected to the APB bus. The APB frequency is used by the I2C Hardware to derive
 *					 		  the various timings according to i2c specification.
 *
 *					 		  The APB bus frequency is stored in FREQ field of CR2 register. In STM32F4xx, the APB frequency and Clock Control Register Settings (CCR)
 *					 		  are used to generate the various frequencies of the I2C serial clock, i.e. in Standard mode up to 100kHz of SCL and in Fast Mode up to 400kHz of SCL
 *
 *						The CCR Register settings are different depending on the desired Serial Clock frequency (SCL). E.g. given below.
 *
 *						E.g. 1)
 *
 *								In Standard Mode, generate a 100 kHz SCL frequency, given that APB1 clock (Peripheral clock) is 16MHz.
 *									Steps:
 *										1) Configure the mode in CCR Register - the 'F_S' bit field of CCR Register (15th bit will be cleared to indicate Standard Mode I2C)
 *										2) Write peripheral clock value into FREQ field of CR2 register of I2C
 *										3) Calculate and program CCR value in the CCR Field of the CCR register based on the following equations (provided in the reference manual in
 *											CCR Register description):
 *
 *											t_high_SCL_seconds = CCR * t_peripheral_clock_seconds,
 *											t_low_SCL_seconds = CCR * t_peripheral_clock_Seconds,
 *
 *											SCL_pulse_seconds = t_high_SCL_seconds + t_low_SCL_seconds;
 *
 *											In step 1,program 'F_S' field of CCR register with '0' because we're running I2C in standard mode, as given in the information. In Step 2,
 *											you program FREQ field of CR2 with '16' in binary because we're given APB bus frequency is 16MHz. Step 3 is: So, for a desired 100kHz SCL,
 *											the pulse is 10 microseconds. With a 50% Duty cycle(i.e. t_high_scl will equal t_low_scl), the high-time of the pulse is 5 microseconds.
 *											Given that the APB bus (to which the I2C peripheral is hanging to) has a frequency of 16Mhz - this is 62.5 nanoseconds. Therefore, using
 *											t_high_SCL_seconds equation above, the CCR is (5/62.5)*1000 = 80, which is 0x50 in HEX, this is the value you program into CCR field of
 *											the CCR register in step 3.
 *
 *											In I2C, the 'Duty Cycle' is used to define the t_high_SCL_seconds and t_low_SCL_seconds of the SCL pulse. I.e. the duty cycle is used
 *											to define SCL pulse's high and low times. For a SM mode I2C, according to the I2C specifications the high time of SCL has to be minimum 4.7
 *											microseconds and low time of SCL has to be minimum of 4.0 microseconds. There are minimums for FM of I2C as well, check the I2C specification doc.
 *
 *											Main point is, the I2C SCL clock in Fast Mode can have 2 possible configurations of SCL pulse, 1) t_low_SCL_seconds = 2*t_high_SCL_seconds or
 *											2) t_low_SCL_seconds = 1.8*t_high_SCL_seconds -> the desired can be set by programming the 'DUTY' field of the CCR register of I2C. With the DUTY field
 *											set to '1', the I2C SCL of STM32F4xx can achieve SCL clock frequencies of 400KHz. With DUTY field not set, the t_low_SCL_seconds will be twice that
 *											of the t_high_SCL_seconds.
 *
*					E.g. 2)
*
*							In Fast Mode, generate a 200KHz SCL frequency, given that APB1 clock (peripheral clock) is 16MHz.
*								Steps:
 *										1) Configure the mode in CCR Register - the 'F_S' bit field of CCR Register (15th bit will be cleared to indicate Standard Mode I2C)
 *										2) Select the DUTY cycle of the Fast Mode SCL in CCR Register
 *										2) Write peripheral clock value into FREQ field of CR2 register of I2C
 *										3) Calculate and program CCR value in the CCR Field of the CCR register based on the following equations (provided in the reference manual in
 *											CCR Register description):
 *
 *											if DUTY = 0:
 *												t_high_SCL_seconds = CCR * t_peripheral_clock_seconds,
 *										 		t_low_SCL_seconds = 2*CCR * t_peripheral_clock_Seconds
 *										 	if DUTy = 1: (to reach 400 KHz SCL frequency)
 *										 		t_high_SCL_seconds = 9*CCR * t_peripheral_clock_seconds,
 *										 		t_low_SCL_seconds = 16*CCR * t_peripheral_clock_Seconds
 *
 *										So, for Step 3 in example above, we select DUTY = 0:
 *
 *											t_high_SCL_seconds + t_low_SCL_seconds = 5 microseconds
 *											5 microseconds  = 3*CCR * t_peripheral_clock_seconds,
 *											5 microseconds  = 3*CCR * 62.5 nanoseconds,
 *											Therefore,
 *												CCR = (5/3*62.5) * 1000 = 26 -> this is the value you program in CCR register in step 3.
 *
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg =0;  // A place holder for the all the bits that need to be configured for the 32-bit registers.

	// Configure the Automatic ACK bit in the CR1 Register based on the configurations given by the Application layer in I2C_Handle_t *pI2CHandle.

	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl) << 10;  // The 10th-bit of the CR1 register is programmed for ACKing





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
