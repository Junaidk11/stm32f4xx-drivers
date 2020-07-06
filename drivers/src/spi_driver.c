/*
 * spi_driver.c
 *
 *  Created on: July 5, 2020
 *      Author: Junaid
 */

#include "spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_ClockControl
 *
 * @brief             - This API function will be used to enable clock access to one of the SPI peripherals.
 *
 * @param[in]         -  a pointer to the base address of the SPI peripheral we want to configure - SPI_RegDef_t -> points to the base-address of the SPIx (either SPI1, SPI2, or SPI3)
 * @param[in]         -  a variable to indicate if you wish to enable or disable the port. Define a Generic macro in MCU header file, use it - Use MACRO: Enable or Disable.
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  

 */

void SPI_ClockControl(SPI_RegDef_t *pSPIx,uint8_t enable_disable ){
    
    /*
	 *  Check if the enable_disable arugment is enable or disable, and then use the
	 *  clock enable macros that we have written in the MCU specific file.
	 */
	if(enable_disable == ENABLE){

		if(pSPIx == SPI1){
			SPI1_PERIPH_CLOCK_EN(); // SPI1 enable clock access MACRO defined in MCU specific header file.
		}else if(pSPIx == SPI2){
			SPI2_PERIPH_CLOCK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PERIPH_CLOCK_EN();
		}
	}else if(enable_disable == DISABLE){
		if(pSPIx == SPI1){
			SPI1_PERIPH_CLOCK_DI(); // SPI1 disable clock access MACRO defined in MCU specific header file.
		}else if(pSPIx == SPI2){
			SPI2_PERIPH_CLOCK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PERIPH_CLOCK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  -  SPI_Init
 *
 * @brief             -  Initializes SPIx registers
 *
 * @param[in]         -  Pointer to the SPI handle; SPI_Handle_t *
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  

 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Resets the SPIx registers
 *
 * @param[in]         - The Base address of your SPI module - SPI_RegDef_t*
 * @param[in]         - 
 * @param[in]         -
 *
 * @return            -  None
 *
 * @Note              -  You can use RCC's AHB1 Peripheral Reset Register.
   		                 This way, you don't have to reset each every register of the respective SPIx. 

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Data Write
 *
 * @param[in]         - Base address of the SPI peripheral -> SPI1/SPI2/SPI3
 * @param[in]         - Pointer to buffer holding the data to be sent
 * @param[in]         - Length of the data to Send 
 *
 * @return            -  None 
 *
 * @Note              -  uint32_t is a standared for defining data length. 

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLength);

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Data Read
 *
 * @param[in]         - Base address of the SPI peripheral -> SPI1/SPI2/SPI3
 * @param[in]         - Pointer to buffer holding the data to be sent
 * @param[in]         - Length of the data to Receive
 *
 *
 * @return            -  None
 *
 * @Note              -  uint32_t is a standared for defining data length.

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLength);

/*********************************************************************
 * @fn      		  - SPI_IRQ_Interrupt_Config
 *
 * @brief             - This function is used to configure the Processor side of interrupt configuration and handling. Use this function to configure the IRQs and enable them. 
 * 						 By default, all IRQs are disabled. Therefore, for the processor to accept an interrupt from a peripheral, the assigned IRQ number to that peripheral should be 
 * 						 configured in the NVIC. 
 * @param[in]         - the IRQ numebr coresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - Priority of the interrupt.
 * @param[in]         - ENABLE or DISABLE 
 *
 * @return            - None
 *
 * @Note              - None

*/
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t enable_disable);

/*********************************************************************
 * @fn      		  - SPI_IRQ_Priority_Config
 *
 * @brief             - This function can be used to configure the priority of the given IRQ number. 
 *
 * @param[in]         - the IRQ numebr coresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - The priority of the IRQNumber
 * @param[in]         -
 *
 * @return            - None
 *
 * @Note              - None

*/
void SPI_IRQ_Priority_Config (uint8_t IRQNumber, uint8_t IRQPriority);


/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - Application ISR handler calls this API function to service the interrupt caused by a SPI peripheral
 *
 * @param[in]         - SPIx Handle structure, it has the baseAddress and the Registers of the SPIx that needs servicing
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - None

*/
void SPI_IRQHandling(SPI_Handle_t *pHandle);
