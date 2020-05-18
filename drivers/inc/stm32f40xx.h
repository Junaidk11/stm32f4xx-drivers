/*
 * stm32f0xx.h
 *
 *  Created on: May 17, 2020
 *      Author: Junai
 */

#ifndef INC_STM32F40XX_H_
#define INC_STM32F40XX_H_

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


#endif /* INC_STM32F40XX_H_ */
