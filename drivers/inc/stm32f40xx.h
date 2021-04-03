/*
 * stm32f0xx.h
 *
 *  Created on: May 17, 2020
 *      Author: Junaid
 */

#ifndef INC_STM32F40XX_H_
#define INC_STM32F40XX_H_

#include <stddef.h>  /* Need this library to get access to NULL definition. */
#include <stdint.h>  /* The short hand data type 'uint32_t' is defined in this file, which is why you need to include it. */
/*
 *  The registers of the peripheral's are volatile in nature, their values can change at any time.
 *  The volatile keyword tells the compiler to not use a copy of the variable (when optimizing the code).
 */

#define __vo 		volatile
#define __weak		__attribute__((weak)) // Use this MACRO to define weak function definitions, which allows application layer definition to overwrite the driver layer definition.

/*
 *  Using C MACROS to define these memories.
 *
 *  By default, the compiler treats all numbers as Signed. But, we know addresses can't be
 *  negative, which is why we place 'U' after the value, to tell the compiler to treat that number
 *  as unsigned. Also, you can tell the compiler that the number is unsigned by type-casting the number
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
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)


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

	__vo uint32_t RCC_APB1ENR; 			/* APB1 Peripheral Clock Enable  Register   	Address offset: 0x40 */
	__vo uint32_t RCC_APB2ENR; 			/* APB2 Peripheral Clock Enable  Register   	Address offset: 0x44 */
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
 *  			Peripheral Definition Type casted to their C Structure holding their register definitions.
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
#define FLAG_SET				SET
#define FLAG_RESET				RESET


/*   GPIO Port Register Reset MACROS */
//		1) Setting the bit in the RCC Peripheral Reset Register first   2) Clearing the bit in the RCC Peripheral Reset Register
//										|												|
#define GPIOA_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()      do{  (RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For EXTI
 *  				EXTI = External Interrupt/Event Controller 
 *  				Need this peripheral for Edge Detection, enable & disable interrupt delivery to the processor via NVIC.
 * 					NVIC = Nested Vector Interrupt Controller
 */	

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */

typedef struct{

	__vo uint32_t IMR; 		/* Interrupt Mask Register. */
	__vo uint32_t EMR; 		/* Event Mask Register. */
	__vo uint32_t RTSR;		/* Rising Trigger Selection Register. */ 
	__vo uint32_t FTSR;		/* Falling Trigger Selection Register. */ 
	__vo uint32_t SWIER;	/* Software Interrupt Event  Register. */
	__vo uint32_t PR; 		/* Pending Register. */

}EXTI_RegDef_t;

/*
 *  			Peripheral Definition Type casted to their C Structure holding their register definitions.
 */

#define EXTI 		((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For SYSCFG
 *  				SYSCFG = System Configuration Controller 
 *  				Need this peripheral for selecting the GPIO port corresponding to the pin configured to deliver interrupt to the Processor via NVIC
 * 					NVIC = Nested Vector Interrupt Controller
 */	

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */

typedef struct{
	__vo uint32_t MEMRMP;   		/* Memory Remap Register. */																/* Address offset: 0x00 */
	__vo uint32_t PMC;   			/* Peripheral Mode Configuration */															/* Address offset: 0x04 */
	__vo uint32_t EXTICR[4];   		/* External Interrupt Configuration Register 1-4  */ 										/* Address offset: 0x08 - 0x14 */
		 uint32_t RESERVED[2]; 		/* According to Reference Manual - these are reserved  - always look at Address Offset!*/   /* Address offset: 0x18-0x1C */
	__vo uint32_t CMPCR;    		/* Compensation Cell Control*/																/* Address offset: 0x20 */

}SYSCFG_RegDef_t; 

/*
 *  			Peripheral Definition Type casted to their C Structure holding their register definitions.
 */
#define SYSCFG   ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)


/**
 *  
 * 		Interrupt Request Numbers(IRQs)
 * 			These numbers are MCU specific - refer to Reference Manual. 
 * 				For STM32f407-Discovery - Interrupt & Exception Vector Table is under section Interrupts & Events  @IRQNumbers
 */

#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1			7 
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10	    40




/**
 * 
 * 		Processor Register Base Addresses are usually defined at the Start of the MCU specific header file - but for now we can keep it here. 
 * 
 * 
 **/

/**
 *   			Nested Vector Interrupt Controller Register Base Address - Referenced from Cortex-M4 Generic User Guide. 
 */


// Interrupt Enable Registers of the NVIC 

#define 	NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define 	NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define 	NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define 	NVIC_ISER3			((__vo uint32_t*)0xE000E10C)
#define 	NVIC_ISER4			((__vo uint32_t*)0xE000E110)
#define 	NVIC_ISER5			((__vo uint32_t*)0xE000E114)
#define 	NVIC_ISER6			((__vo uint32_t*)0xE000E118)
#define 	NVIC_ISER7			((__vo uint32_t*)0xE000E11C)


// Interrupt Disable Registers of the NVIC 

#define 	NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define 	NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define 	NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define 	NVIC_ICER3			((__vo uint32_t*)0xE000E18C)
#define 	NVIC_ICER4			((__vo uint32_t*)0xE000E190)
#define 	NVIC_ICER5			((__vo uint32_t*)0xE000E194)
#define 	NVIC_ICER6			((__vo uint32_t*)0xE000E198)
#define 	NVIC_ICER7			((__vo uint32_t*)0xE000E19C)

// Interrutp Priorty Registers Base address - NVIC_IPR0

#define NVIC_IPR_BASEADDR   	((__vo uint32_t*)0xE000E400)    // Address of NVIC_IPR0 register.

/**
 * 				There are 60 Interrupt Priority Registers, you can define all 60 as commented out below,
 * 				Or, you can define the baseaddress (NVIC_IPR0) register and add the offset of
 * 				size depending of the IRQ Number /4, because each NVIC_IPRx register has 4 IRQs assigned to it. 
 *
#define NVIC_IPR0 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR1 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR2 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR3 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR4 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR5 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR6 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR7 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR8				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR9 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR10 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR11 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR12 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR13 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR14 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR15 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR16 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR17 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR18 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR19 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR20 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR21 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR22 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR23 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR24 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR25 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR26 				((__vo uint32_t*)0xE000E400)
#define NVIC_IPR27 				((__vo uint32_t*)0xE000E400)
*/


/** 
 * 					Number of Priority bits implemented for STM devices. Each MCU has its own rules for this- refer to Generic Device Manual 
 * 								Note: Number of priority bits is specific to the MCU. E.g. Some TI MCU's this value is 3. 
*/

#define NO_PR_BITS_IMPLEMENTED		4

/**
 * 					NVIC Priority Possibilities Macro definition
 */

#define NVIC_PRIORITY_0 		0
#define NVIC_PRIORITY_1 		1 
#define NVIC_PRIORITY_2 		2 
#define NVIC_PRIORITY_3 		3 
#define NVIC_PRIORITY_4 		4 
#define NVIC_PRIORITY_5 		5 
#define NVIC_PRIORITY_6 		6 
#define NVIC_PRIORITY_7 		7 
#define NVIC_PRIORITY_8 		8 
#define NVIC_PRIORITY_9 		9 
#define NVIC_PRIORITY_10 		10 
#define NVIC_PRIORITY_11 		11 
#define NVIC_PRIORITY_12 		12 
#define NVIC_PRIORITY_13 		13 
#define NVIC_PRIORITY_14 		14 
#define NVIC_PRIORITY_15 		15 


/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For SPI
 *  				SPI = Serial Peripheral Interface
 */	

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */

/*
 * 			A C Structure for SPI Register Definitions
 *
 * 	Note: Create a pointer of type SPI_RegDef_t and assign it the address of the SPI module you want to configure, SPI1, SPI2, or SPI3
 * 		e.g. SPI_RegDef_t *pSPI1 = (SPI_RegDef_t *)0x40013000 ; where 0x40013000 is the base address of SPI1; Type cast is used to convert 0x40013000 from a VALUE to a MEMORY ADDRESS.
 */

typedef struct
{
	__vo uint32_t CR1; 			/* Control Register 1 */     			/* Offset: 0x00 */ 
	__vo uint32_t CR2; 			/* Control Register 2 */     			/* Offset: 0x04 */ 	
	__vo uint32_t SR;  			/* Status Register */     				/* Offset: 0x08 */ 
	__vo uint32_t DR;  			/* Data Register */     				/* Offset: 0x0C */
	__vo uint32_t CRCPR; 		/* CRC Polynomial Register */      		/* Offset: 0x10 */
	__vo uint32_t RXCRCR; 		/* RX CRC Register */      				/* Offset: 0x14 */
	__vo uint32_t TXCRCR; 		/* TX CRC Register */      				/* Offset: 0x18 */
	__vo uint32_t I2SCFGR; 		/* I2S Configuration Register */      	/* Offset: 0x1C */
	__vo uint32_t I2SPR; 		/* I2S Prescaler Register */      		/* Offset: 0x20 */
	
}SPI_RegDef_t;

/*
 *  			Peripheral Definitions Type casted to their C Structure holding their register definitions.
 */

#define SPI1 	((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2    ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 	((SPI_RegDef_t *)SPI3_BASEADDR)


/**
 *  SPI Peripheral Register Bit Position Definition
 *   	SYNTAX -->  PERIPHERALNAME_REGISTERNAME_BITFIELDNAME
 */

	/* SPI Control Register 1 */

#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE    15


	/* SPI Control Register 2 */

#define SPI_CR2_RXDMAEN		0 
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7 

	/* SPI Status Register  */

#define SPI_SR_RXNE 		0
#define SPI_SR_TXE 			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR 			3
#define SPI_SR_CRCERR 		4
#define SPI_SR_MODF    		5
#define SPI_SR_OVR 			6
#define SPI_SR_BSY 			7
#define SPI_SR_FRE 			8


/*   SPIx Register Reset MACROS */
//			1) Setting the bit in the RCC Peripheral Reset Register first    2) Clearing the bit in the RCC Peripheral Reset Register
//										|											|
#define SPI1_REG_RESET()      do{  (RCC->RCC_APB2RSTR |= (1 << 12)); (RCC->RCC_APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()      do{  (RCC->RCC_APB1RSTR |= (1 << 14)); (RCC->RCC_APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()      do{  (RCC->RCC_APB1RSTR |= (1 << 15)); (RCC->RCC_APB1RSTR &= ~(1 << 15)); }while(0)



/**
 *
 * 		Interrupt Request Numbers(IRQs)
 * 			These numbers are MCU specific - refer to Reference Manual.
 * 				For STM32f407-Discovery - Interrupt & Exception Vector Table is under section Interrupts & Events  @IRQNumbers
 */

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51


/*
 *  Possible SPI Application Events -> Used in SPI ISR handling to call application when SPI has completed its assigned task (Transmission or reception of data).
 */

#define SPI_EVENT_TX_CMPLT  	1
#define SPI_EVENT_RX_CMPLT 		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR 		4

/*
 *  				PERIPHERAL REGISTER DEFINITION STRUCTURE - For I2C
 *  				I2C = inter-integrated circuit
 *
 *  				General Steps followed for I2C (or any module) Driver Development:
 *  					   1 -Add Base address of Bus that has I2C Peripherals connected to them - Done above
 *  					   2 -Create a structure to Hold I2C register definition - Done below
 *  					   3 -Create MACROS to define I2C1, I2C2, I2C3 peripheral registers. - Done below
 *  					   4 -Create MACROS to enable/Disable clock access to I2C1, I2C2 and I2C3 peripherals -  Done above
 *  					   5 -Create MACROS to define bit position fields for I2C Peripheral Registers - Done below
 */

/*
 * Note: Registers of a Peripheral are specific to the MCU. Refer to the Reference Manual of Device.
 *
 */

/*
 * 			A C Structure for I2C Register Definitions
 *
 * 	Note: Create a pointer of type I2C_RegDef_t and assign it the address of the I2C module you want to configure, I2C1, I2C2, or I2C3
 * 		e.g. I2C_RegDef_t *pI2C1 = (I2C_RegDef_t *)0x40005400 ; where 0x4000500 is the base address of I2C1; Type cast is used to convert 0x4000500 from a VALUE to a MEMORY ADDRESS.
 */

typedef struct
{
	__vo uint32_t CR1; 			/* Control Register 1 */     			/* Offset: 0x00 */
	__vo uint32_t CR2; 			/* Control Register 2 */     			/* Offset: 0x04 */
	__vo uint32_t OAR1;  	    /* Own Address Register 1 */     		/* Offset: 0x08 */
	__vo uint32_t OAR2;  	    /* Own Address Register 2 */     		/* Offset: 0x0C */
	__vo uint32_t DR; 			/* Data Register */      				/* Offset: 0x10 */
	__vo uint32_t SR1; 			/* Status Register 1 */      			/* Offset: 0x14 */
	__vo uint32_t SR2; 			/* Status Register 2 */      			/* Offset: 0x18 */
	__vo uint32_t CCR; 			/* Clock Control Register */      		/* Offset: 0x1C */
	__vo uint32_t TRISE; 		/* TRISE Register */      				/* Offset: 0x20 */
	__vo uint32_t FLTR; 		/* FLTR Register */      				/* Offset: 0x24 */

}I2C_RegDef_t;


/*
 *  			Peripheral Definitions Type casted to their C Structure holding their register definitions.
 */

#define I2C1 	((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2    ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 	((I2C_RegDef_t *)I2C3_BASEADDR)

/**
 * 	 I2C Peripheral Register Bit Position Definition
 *   	SYNTAX -->  PERIPHERALNAME_REGISTERNAME_BITFIELDNAME
 */

/*  I2C Control Register 1 */

#define I2C_CR1_PE 				0 			// Peripheral Enable
#define I2C_CR1_SMBUS 			1 			// SMBUS Mode
#define I2C_CR1_SMBTYPE 		3			// SMBUS Type
#define I2C_CR1_ENARP 			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH 		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10          // Acknowledge Enable
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


/*  I2C Control Register 2 */

#define I2C_CR2_FREQ		    	0 			// Peripheral Clock Frequency - Bits 0:5
#define I2C_CR2_ITERREN 			8 			// Error Interrupt Enable
#define I2C_CR2_ITEVTEN 			9			// Event Interrupt Enable
#define I2C_CR2_ITBUFEN 			10			// Buffer Interrupt Enable
#define I2C_CR2_DMAEN		    	11			// DMA Requests Enable
#define I2C_CR2_LAST				12			// DMA Last Transfer

/* 	Status Register 1 */
/* 	Status Register 2 */

#endif /* INC_STM32F40XX_H_ */
