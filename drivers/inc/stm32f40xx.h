/*
 * stm32f0xx.h
 *
 *  Created on: May 17, 2020
 *      Author: Junai
 */

#ifndef INC_STM32F40XX_H_
#define INC_STM32F40XX_H_


#include <stdint.h>  /* The short hand data type 'uint32_t' is defined in this file, which is why you need to include it. */
/*
 *  The registers of the peripheral's are volatile in nature, their values can change at any time.
 *  The volatile keyword tells the compiler to not use a copy of the variable (when optimizing the code).
 */

#define __vo 		volatile

/*
 *  Using C MACROS to define these memories.
 *
 *  By default, the compiler treats all numbers as Signed. But, we know addresses can't be
 *  negative, which is why we place 'U' after the value, to tell the compiler to treat that number
 *  as unsigned. Also, you can tell the compiler that the number is unsigned by typecasting the number
 *  with (uint32_t)0x08000000 == 0x08000000U
 */

/*
 *  Base addresses of Flash & SRAM memory
 *
 *  Syntax: DRV_FLASH_BASEADDR
 *  		HAL_FLASH_BASEADDR
 *
 *  The suffix can be used to denote which layer does the MACRO belong to --> Coding style for bigger projects
 *  Where a different layer uses the same name for a MACRO. We ignore it here. But its good to know about programming
 *  practices.
 *
 */
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U 	 //  SRAM1 Size = 112 KB
#define SRAM2_BASEADDR 			0x2001C000U		 //  SRAM2 Size = 16  KB, right after SRAM1
#define ROM_BASEADDR			0x1FFF0000U	     // This is the system memory.
#define SRAM					SRAM1_BASEADDR   // This is the main Memory.

/*
 *  AHBx & APBx Bus Peripheral Base Addresses
 */

#define PERIPH_BASEADDR 		0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 	0x40010000U         /*PERIPH_BASEADDR + 0x0010000U */
#define AHB1PERIPH_BASEADDR		0x40020000U         /*PERIPH_BASEADDR + 0x0020000U  */
#define AHB2PERIPH_BASEADDR		0540000000U         /*PERIPH_BASEADDR + 0x1000000U  */


/*
 * Base Addresses of peripherals connected to the AHB1 bus.
 * Syntax: (AHB1PERIPH_BASEADDR + OFFSET); Find offset from Memory Map of Device
 * Read from the Memory map Section of your Device's Reference Manual.
 */

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base Addresses of peripherals connected to the APB1 bus.
 * Syntax: (APB1PERIPH_BASEADDR + OFFSET); Find offset from Memory Map of Device
 * Read from the Memory map Section of your Device's Reference Manual.
 */
#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base Addresses of peripherals connected to the APB2 bus.
 * Syntax: (APB2PERIPH_BASEADDR + OFFSET); Find offset from Memory Map of Device
 * Read from the Memory map Section of your Device's Reference Manual.
 */
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800)


/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For GPIO
 */

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */


/*
 * 			A C Structure for GPIO PORT.
 *
 * 	Note: Create a pointer of type GPIO_RegDef_t and assign it the address of the GPIO port you want to configure.
 * 		e.g. GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *)0x40020000 ; where 0x4002000 is the base address of GPIOA; Type cast is used to convert 0x40020000 from a VALUE to a MEMORY ADDRESS.
 */
typedef struct{
	__vo uint32_t MODER; 						/* Address offset: 0x00 */
	__vo uint32_t OTYPER; 						/* Address offset: 0x04 */
	__vo uint32_t OSPEEDR; 						/* Address offset: 0x08 */
	__vo uint32_t PUPDR; 						/* Address offset: 0x0C */
	__vo uint32_t IDR; 							/* Address offset: 0x10 */
	__vo uint32_t ODR; 							/* Address offset: 0x14 */
	__vo uint32_t BSRR; 							/* Address offset: 0x18 */
	__vo uint32_t LCKR; 							/* Address offset: 0x1C */
	__vo uint32_t AFR[2]; 						/* Address offset: 0x20 for AFR[0] --> AFRL, 0x24 for AFR[1] --> AFRH*/
}GPIO_RegDef_t;


/*
 *  			Peripheral Definitions Type casted to their C Structure holding their register definitions.
 */

#define GPIOA  			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE  			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF  			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG  			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH  			((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI  			((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ  			((GPIO_RegDef_t*) GPIOJ_BASEADDR)
#define GPIOK  			((GPIO_RegDef_t*) GPIOK_BASEADDR)

/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For RCC
 *  				RCC = Reset and Clock Control Peripheral
 *  				Need this peripheral to enable & disable clock access to the Peripherals.
 */

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */

typedef struct{

	__vo uint32_t RCC_CR; 				/* Clock control Register   					Address offset: 0x00 */
	__vo uint32_t RCC_PLLCFGR; 			/* PLL  Configuration Register   				Address offset: 0x04 */
	__vo uint32_t RCC_CFGR; 			/* RCC Clock configuration Register  			Address offset: 0x08 */
	__vo uint32_t RCC_CIR; 				/* RCC Clock interrupt Register   				Address offset: 0x0C */

	__vo uint32_t RCC_AHB1RSTR; 		/* AHB1 Peripheral Reset Register   			Address offset: 0x10 */
	__vo uint32_t RCC_AHB2RSTR; 		/* AHB2 Peripheral Reset Register  				Address offset: 0x14 */
	__vo uint32_t RCC_AHB3RSTR; 		/* AHB3 Peripheral Reset Register  			    Address offset: 0x18 */
		 uint32_t RESERVED0; 			/* Reserved 									Address offset: 0x1C */
	__vo uint32_t RCC_APB1RSTR; 		/* APB1 Peripheral Reset Register  				Address offset: 0x20 */
	__vo uint32_t RCC_APB2RSTR; 		/* APB2 Peripheral Reset Register  				Address offset: 0x24 */
		 uint32_t RESERVED1[2]; 			/* Two Reserved registers 						Address offset: 0x28-0x2C */

	__vo uint32_t RCC_AHB1ENR; 			/* AHB1 Peripheral Clock Register   			Address offset: 0x30 */
	__vo uint32_t RCC_AHB2ENR; 			/* AHB2 Peripheral Clock Enable  Register   	Address offset: 0x34 */
	__vo uint32_t RCC_AHB3ENR; 			/* AHB3 Peripheral Clock Enable  Register   	Address offset: 0x38 */
		 uint32_t RESERVED3; 			/* Reserved 									Address offset: 0x3C */

	__vo uint32_t RCC_AP1ENR; 			/* APB1 Peripheral Clock Enable  Register   	Address offset: 0x40 */
	__vo uint32_t RCC_AP2ENR; 			/* APB2 Peripheral Clock Enable  Register   	Address offset: 0x44 */
	 	 uint32_t RESERVED4[2]; 		/* Two Reserved registers 						Address offset: 0x48-0x4C */

	__vo uint32_t RCC_AHB1LPENR; 		/* AHB1 Peripheral Clock enable in low power mode Register   Address offset: 0x50 */
	__vo uint32_t RCC_AHB2LPENR; 		/* AHB2 Peripheral Clock enable in low power mode Register   Address offset: 0x54 */
	__vo uint32_t RCC_AHB3LPENR; 		/* AHB3 Peripheral Clock enable in low power mode Register   Address offset: 0x58 */
	 	 uint32_t RESERVED5; 			/* Reserved 									Address offset: 0x5C */

	__vo uint32_t RCC_APB1LPENR; 		/* APB1 Peripheral Clock enable in low power mode Register   Address offset: 0x60 */
	__vo uint32_t RCC_APB2LPENR; 		/* APB2 Peripheral Clock enable in low power mode Register   Address offset: 0x64 */
	 	 uint32_t RESERVED6[2]; 		/* Two Reserved registers 						Address offset: 0x68-0x6C */


	__vo uint32_t RCC_BDCR; 			/* Backup Domain control Register   			Address offset: 0x70 */
	__vo uint32_t RCC_CSR; 				/* Clock control & Status Register   			Address offset: 0x74 */
		 uint32_t RESERVED7[2]; 		/* Two Reserved registers 						Address offset: 0x78-0x7C */

	__vo uint32_t RCC_SSCGR; 			/* Spread spectrum clock generation Register   	Address offset: 0x80 */
	__vo uint32_t RCC_PLLI2SCFGR; 		/* PLL I2S configuration Register   			Address offset: 0x84 */
	__vo uint32_t RCC_PLLSAICFGR; 		/* PLL Configuration Register   				Address offset: 0x88 */
	__vo uint32_t RCC_DCKCFGR; 			/* Dedicated Clock Configuration Register   	Address offset: 0x8C */

}RCC_RegDef_t;


/*
 *  			Peripheral Definitions Type casted to their C Structure holding their register definitions.
 */

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)


/*
 *  		Creating MACROS that can be used to enable clock access to the Peripherals.
 */

/*   Clock Enable Macros for GPIOx Peripherals  */

#define GPIOA_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 0))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 0.  */
#define GPIOB_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 1))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 1.  */
#define GPIOC_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 2))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 2.  */
#define GPIOD_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 3))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 3.  */
#define GPIOE_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 4))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 4.  */
#define GPIOF_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 5))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 5.  */
#define GPIOG_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 6))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 6.  */
#define GPIOH_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 7))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 7.  */
#define GPIOI_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 8))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 8.  */
#define GPIOJ_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 9))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 9.  */
#define GPIOK_PERIPH_CLOCK_EN()   		(RCC->RCC_AHB1ENR |= (1 << 10)) /* Dereferencing AHB1 Peripheral Clock Enable Register, and setting it's bit 10.  */

/*   Clock Enable Macros for I2Cx Peripherals  */
#define I2C1_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 21))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 21.  */
#define I2C2_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 22))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 22.  */
#define I2C3_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 23))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 23.  */


/*   Clock Enable Macros for SPIx Peripherals  */
#define SPI1_PERIPH_CLOCK_EN()   		(RCC->RCC_APB2ENR |= (1 << 12))  /* Dereferencing APB2 Peripheral Clock Enable Register, and setting it's bit 12.  */
#define SPI2_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 14))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 14.  */
#define SPI3_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 15))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 15.  */

/*   Clock Enable Macros for USARTx Peripherals  */
#define USART1_PERIPH_CLOCK_EN()   		(RCC->RCC_APB2ENR |= (1 << 4))  /* Dereferencing APB2 Peripheral Clock Enable Register, and setting it's bit 4.  */
#define USART2_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 17))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 17.  */
#define USART3_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 18))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 18.  */
#define USART6_PERIPH_CLOCK_EN()   		(RCC->RCC_APB2ENR |= (1 << 5))  /* Dereferencing APB2 Peripheral Clock Enable Register, and setting it's bit 5.  */

/*   Clock Enable Macros for UARTx Peripherals  */
#define UART4_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 19))  /* Dereferencing APB2 Peripheral Clock Enable Register, and setting it's bit 19.  */
#define UART5_PERIPH_CLOCK_EN()   		(RCC->RCC_APB1ENR |= (1 << 20))  /* Dereferencing APB1 Peripheral Clock Enable Register, and setting it's bit 20.  */



/*   Clock Enable Macros for SYSCFG Peripheral  */
#define SYSCFG_PERIPH_CLOCK_EN()   		(RCC->RCC_APB2ENR |= (1 << 14))  /* Dereferencing APB2 Peripheral Clock Enable Register, and setting it's bit 14.  */




/*
 *  		Creating MACROS that can be used to disable clock access to the Peripherals.
 */

/*   Clock Disable Macros for GPIOx Peripherals  */

#define GPIOA_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 0))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 0.  */
#define GPIOB_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 1))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 1.  */
#define GPIOC_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 2))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 2.  */
#define GPIOD_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 3))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 3.  */
#define GPIOE_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 4))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 4.  */
#define GPIOF_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 5))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 5.  */
#define GPIOG_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 6))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 6.  */
#define GPIOH_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 7))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 7.  */
#define GPIOI_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 8))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 8.  */
#define GPIOJ_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 9))  /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 9.  */
#define GPIOK_PERIPH_CLOCK_DI()   		(RCC->RCC_AHB1ENR &= ~ (1 << 10)) /* Dereferencing AHB1 Peripheral Clock Enable Register, and clearing it's bit 10.  */

/*   Clock Disable Macros for I2Cx Peripherals  */
#define I2C1_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 21))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 21.  */
#define I2C2_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 22))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 22.  */
#define I2C3_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 23))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 23.  */


/*   Clock Disable Macros for SPIx Peripherals  */
#define SPI1_PERIPH_CLOCK_DI()   		(RCC->RCC_APB2ENR &= ~ (1 << 12))  /* Dereferencing APB2 Peripheral Clock Enable Register, and clearing it's bit 12.  */
#define SPI2_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 14))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 14.  */
#define SPI3_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 15))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 15.  */

/*   Clock Disable Macros for USARTx Peripherals  */
#define USART1_PERIPH_CLOCK_DI()   		(RCC->RCC_APB2ENR &= ~ (1 << 4))  /* Dereferencing APB2 Peripheral Clock Enable Register, and clearing it's bit 4.  */
#define USART2_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 17))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 17.  */
#define USART3_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 18))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 18.  */
#define USART6_PERIPH_CLOCK_DI()   		(RCC->RCC_APB2ENR &= ~ (1 << 5))  /* Dereferencing APB2 Peripheral Clock Enable Register, and clearing it's bit 5.  */

/*   Clock Disable Macros for UARTx Peripherals  */
#define UART4_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 19))  /* Dereferencing APB2 Peripheral Clock Enable Register, and clearing it's bit 19.  */
#define UART5_PERIPH_CLOCK_DI()   		(RCC->RCC_APB1ENR &= ~ (1 << 20))  /* Dereferencing APB1 Peripheral Clock Enable Register, and clearing it's bit 20.  */


/*   Clock Disable Macros for SYSCFG Peripheral  */
#define SYSCFG_PERIPH_CLOCK_DI()   		(RCC->RCC_APB2ENR &= ~ (1 << 14))  /* Dereferencing APB2 Peripheral Clock Enable Register, and clearing it's bit 14.  */


/*      GENERIC MACROS				 */

#define ENABLE 					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE

#endif /* INC_STM32F40XX_H_ */