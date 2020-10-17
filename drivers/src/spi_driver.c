/*
 * spi_driver.c
 *
 *  Created on: July 5, 2020
 *      Author: Junaid
 */

#include "spi_driver.h"

/*
 *  The following are helper functions to the ISR handling of SPI.
 *  	These shouldn't be accessible by the application. The use of static makes that happen.
 *  		Static keyword only allows these functions to be called by any function in this source file  -> spi_driver.c
 *
 */

static void spi_txe_interrupt_handler(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pHandle);

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
void SPI_Init(SPI_Handle_t *pSPIHandle){

    // Configure SPI_CR1 register first. 

    uint32_t tempReg = 0; // Set all the bits corresponding to CR1 register, followed by assigned this value to the CR1 register. 

    // 1. Configure Device Mode
    
    tempReg |= ((pSPIHandle->SPIConfig.DeviceMode) << SPI_CR1_MSTR); // '2' Because bit 2 of CR1 is used for configuring Master or Slave mode. 

    /* Not very clear --> Need to REVIEW spi communication type configurations */ 

    // 2. Configure Bus Configuration, i.e. SPI communication type
    if(pSPIHandle->SPIConfig.BusConfig == SPI_IN_FULL_DUPLEX_MODE)
    {
        // Set BIDIMODE bit of CR1 as '0' -> Will configure 2-line unidirection data mode
        // resulting in separate line for RX and TX
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
        
        // RXONLY bit is by default = '0' which means Receive and Transmit.
    }else if (pSPIHandle->SPIConfig.BusConfig == SPI_IN_HALF_DUPLEX_MODE){
        // Set BIDIMODE bit of CR1 as '1' -> will configure 1-line bidirection data mode, same line for RX and TX 
        tempReg |= (1 << SPI_CR1_BIDIMODE); 

    }else if (pSPIHandle->SPIConfig.BusConfig == SPI_IN_SIMPLEX_TX_ONLY){

        // Clear BIDIMODE bit of CR1 to '0' -> 2-line bidirection data mode, different line for RX & TX
        tempReg &= ~(1<<SPI_CR1_BIDIMODE);
        // Clear RXONLY bit to '0', basically configuring Full-duplex mode with Transmission ONLY, and ignoring Receive 
        tempReg &= ~(1<<SPI_CR1_RXONLY);
        

    }else if(pSPIHandle->SPIConfig.BusConfig == SPI_IN_SIMPLEX_RX_ONLY){
        // Clear BIDIMODE bit of CR1 to '0' -> 2-line unidirectional data mode, i.e. Full duplex SPI communication type
        tempReg &= ~(1<<SPI_CR1_BIDIMODE); 
        // Set RXONLY bit to force clock generation,i.e. if the device is master, you want to only RECEIVE.
        tempReg |= (1<<SPI_CR1_RXONLY);
    }

    // 3. Configure Serial Clock but setting Baudrate. 
    tempReg |= ((pSPIHandle->SPIConfig.SclkSpeed) << SPI_CR1_BR);

    // 4. Configure Data Frame Format
    tempReg |= ((pSPIHandle->SPIConfig.DataFrameFormat) << SPI_CR1_DFF); 

    // 5. Configure ClockPolarity (CPOL)
    tempReg |= ((pSPIHandle->SPIConfig.ClockPolarity) << SPI_CR1_CPOL); 

    // 6. Configure ClockPhase (CPHA)
    tempReg |= ((pSPIHandle->SPIConfig.ClockPhase) << SPI_CR1_CPHA); 

    // 7. Configure Software Slave Management 
    tempReg |= ((pSPIHandle->SPIConfig.SlaveManagementType) << SPI_CR1_SSM); 

    // Assign tempReg to your SPIx's CR1 register.
    pSPIHandle->pSPIx_BASEADDR->CR1 = tempReg;

}

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
void SPI_DeInit(SPI_RegDef_t *pSPIx){

    if(pSPIx == SPI1){
        SPI1_REG_RESET();

    }else if(pSPIx == SPI2){
        SPI2_REG_RESET();
    }else if(pSPIx == SPI3){
        SPI3_REG_RESET();
    }
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - SPI Peripheral Enable/Disable API
 *
 * @param[in]         - Base address of the SPIx
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         - 
 *
 * @return            -  None 
 *
 * @Note              - None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable){

    if(Enable_Disable == ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }else if(Enable_Disable == DISABLE){
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - SPI NSS Enable/Disable API
 *
 * @param[in]         - Base address of the SPIx
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         - 
 *
 * @return            -  None 
 *
 * @Note              - None
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable){

    if(Enable_Disable==ENABLE){
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }else if(Enable_Disable==DISABLE){
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}
 
/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - SPI SPOE Enable/Disable API
 *
 * @param[in]         - Base address of the SPIx
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         - 
 *
 * @return            -  None 
 *
 * @Note              - None
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable){

    if(Enable_Disable==ENABLE){
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }else if(Enable_Disable == DISABLE){
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}
/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - Check if the requested flag is set or not.
 *
 * @param[in]         - Base address of the SPIx
 * @param[in]         - Masked flag bit field 
 * @param[in]         - 
 *
 * @return            -  Either '1' or '0' 
 *
 * @Note              - None
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

    if(pSPIx->SR & FlagName){
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Data Write
 *
 * @param[in]         - Base address of the SPI peripheral -> SPI1/SPI2/SPI3
 * @param[in]         - Pointer to buffer holding the data to be sent
 * @param[in]         - Length of the data to Send in Bytes
 *
 * @return            -  None 
 *
 * @Note              -  uint32_t is a standared for defining data length. 
 *                       This is a "Blocking" implementation of SPI send, you wait till TXE is set, before you can push data into the Tx Buffer.
 *                        It is also called Polling method, because we're waiting till Transmit buffer gets empty, the function will stay there.                        
 *                        There are problems with this method of implementation, what if something wrong with the hardware and the TXE flag is NEVER set, then the system 
                        will be stuck here forever, this when you need a WATCHDOG module to reset the system if it becomes non-responsive for a certain time period. 
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLength){

    while(DataLength > 0){
        //1. Wait until TXE is set
                    // while( !(pSPIx->SR & (1 << 1)) ); // Checking if TXE flag is set in the Status Register, implement the condition using a function defined in this source. 
        while(! SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG));  // Same as above statement, but a much cleaner method of implementation. If TXNE, you stay here, if empty, you push data into Tx Buffer
    
        //2. Check DFF bit CR1 to determine how many bytes to upload in the DR, which will push the data bytes to the Tx Buffer
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){

            // If bit is set, then DFF = 16-bit. You need to upload 2 bytes of data into the DR register. 
            //             The type casting here will convert the 8-bit pointer to a 16-bit pointer, allowing to dereference 2-bytes of consecutive data. Without the uint16_t* typecast, you would be dereferencing a byte of data. 
            //                  |
            pSPIx->DR = *((uint16_t *)pTxBuffer);
             
            // Pushed 2 bytes of Data into Tx Buffer, so reduced length by 2 bytes.
            DataLength--; 
            DataLength--; 
            // Move pointer 2 bytes ahead
            (uint16_t *)pTxBuffer++; // This will make the pointer point to the start of the 16-bits to send.

        }else
        {
            // DFF = 8-bit, you need to upload a byte at a time. 
            pSPIx->DR = *(pTxBuffer); // Don't need typecasting as pointer is of 8-bit type. 
            DataLength--; 
            pTxBuffer++; 
        }
        
    }

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Data Read
 *
 * @param[in]         - Base address of the SPI peripheral -> SPI1/SPI2/SPI3
 * @param[in]         - Pointer to buffer holding the data to be sent
 * @param[in]         - Length of the data to Receive in bytes.
 *
 *
 * @return            -  None
 *
 * @Note              -  uint32_t is a standared for defining data length.

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLength){

     while(DataLength > 0){
        //1. Wait until RXNE is set, which RX Buffer in the SPI block is not Empty, there is new data available 
                    // while( !(pSPIx->SR & (0 << 1)) ); // Checking if RXNE flag is set in the Status Register, implement the condition using a function defined in this source. 

    	 while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );  // Same as above statement, but a much cleaner method of implementation.
        
        //2. Check DFF bit CR1 to determine how many bytes to Download/grab/read from the DR, which will push the data bytes from the SPI RX buffer to the RX buffer of the program
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){

            // If bit is set, then DFF = 16-bit. You need to upload 2 bytes of data into the DR register. 
            //             The type casting here will convert the 8-bit pointer to a 16-bit pointer, allowing to dereference 2-bytes of consecutive data - in this case you will download/read 16-bits of data from SPI's RX buffer. Without the uint16_t* typecast, you would be storing a byte of data.
            //                  |
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
             
            // Pushed 2 bytes of Data into Rx Buffer, so reduced length by 2 bytes.
            DataLength--; 
            DataLength--; 
            // Move pointer 2 bytes ahead
            (uint16_t *)pRxBuffer++; // This will make the pointer point to the start of the 16-bits to send.

        }else
        {
            // DFF = 8-bit, you need to read a byte at a time from the SPI's RX buffer 
            *(pRxBuffer) = pSPIx->DR; // Don't need type-casting as pointer is of 8-bit type.
            DataLength--; 
            pRxBuffer++; 
        }
        
    }

}

/*********************************************************************
 * @fn      		  - SPI_IRQ_Interrupt_Config
 *
 * @brief             - This function is used to configure the Processor side of interrupt configuration and handling. Use this function to configure the IRQs and enable them. 
 * 						 By default, all IRQs are disabled. Therefore, for the processor to accept an interrupt from a peripheral, the assigned IRQ number to that peripheral should be 
 * 						 configured in the NVIC. 
 * @param[in]         - the IRQ number corresponding to the interrupt you want to configure on the processor side from @IRQNumbers defined in MCU header file
 * @param[in]         - Priority of the interrupt.
 * @param[in]         - ENABLE or DISABLE 
 *
 * @return            - None
 *
 * @Note              - None

*/
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t enable_disable){
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

/*********************************************************************
 * @fn      		  - SPI_IRQ_Priority_Config
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
void SPI_IRQ_Priority_Config (uint8_t IRQNumber, uint8_t IRQPriority){

	//1. Find the IPRx register assigned to the IRQNumber
	uint8_t iprx = IRQNumber /4;
	//2. Find the section of the iprx register assigned to the IRQNumber
	uint8_t iprx_Section = IRQNumber % 4;

	//3. Set the IRQPriority in the respective iprx register using the NVIC_IPRR baseaddr defined in the device specific header file.
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

/*********************************************************************
 * @fn      		  - SPI_SendData_UsingInterrupt
 *
 * @brief             - Data write function using Interrupts  (IT = Interrupt
 *
 * @param[in]         - Pointer to the SPI Handler of the SPI module to use for sending data
 * @param[in]         - Pointer to buffer that contains data to be transmitted
 * @param[in]         - Length of the buffer
 *
 * @return            - State of the SPI module:
 * 							If SPI already in TX state, the new transmission request will not be registered and you will return SPI_BUSY_TX.
 *
 *
 * @Note              - This API is a non-blocking implementation of the SPI_SendData function.
 * 						This API DOES not transmit the data, it does the following:
 * 							1) Stores the data buffer location and its length in some global variables
 * 							2) Marks the SPI module in the given handle as BUSY -> to avoid other parts of the application code trying to access this SPI module until its completed its transmission
 * 						  	3) Enable the TXEIE(Transmit Interrupt Enable) control bit to get interrupts whenever the TXE flag is set in Status Register of the SPI.
 *
 * 						The ACTUAL Data transmission is handled by the ISR Handler.
 *
*/
uint8_t SPI_SendData_UsingInterrupts(SPI_Handle_t  *pSPIHandle, uint8_t *pTxBuffer, uint32_t DataLength){


	    // First check if the SPI has not been registered for a transmission -> i.e. if SPI already in TX state, do not register another transmission request.

		uint8_t state = pSPIHandle->txState;

		if(state != SPI_BUSY_IN_TX){

			// 1. Save the TxBuffer location and size information in the SPI Handler.
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->txLen = DataLength;

			// 2. Set the state of the SPI module to busy with TX.
			pSPIHandle->txState = SPI_BUSY_IN_TX; // This will block any other part of the application trying to use this SPI for transmission.

			// 3. Enable the TX Interrupt (TXEIE) -> this will allow interrupts to occur whenever the TX flag in the Status Register is set.
					// From Reference manual -> TXEIE field is 7th bit the CR2 register.
			pSPIHandle->pSPIx_BASEADDR->CR2 |= (1 << SPI_CR2_TXEIE);

			// 4. Data Transmission will be handled by the ISR handler.
		}
		return state;
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData_UsingInterrupt
 *
 * @brief             - Data receive function using Interrupts  (IT = Interrupt
 *
 * @param[in]         - Pointer to the SPI Handler of the SPI module to use for receiving data
 * @param[in]         - Pointer to buffer that will stored received data
 * @param[in]         - Length of the buffer
 *
 * @return            - State of the SPI module:
 * 							If SPI already in RX state, the new receive request will not be registered and you will return SPI_BUSY_RX.
 * 								i.e. Can't request the SPI module to receive when it hasn't received the data from the previous request.
 *
 *
 * @Note              - This API is a non-blocking implementation of the SPI_ReceiveData function.
 * 						This API DOES not transmit the data, it does the following:
 * 							1) Stores the receive buffer location and its length in some global variables
 * 							2) Marks the SPI module in the given handle as BUSY -> to avoid other parts of the application code trying to use this SPI module until its completed receiving the data.
 * 						  	3) Enable the RXEIE(Transmit Interrupt Enable) control bit to get interrupts whenever the RXE flag is set in Status Register of the SPI.
 *
 * 						The ACTUAL Data receiving is handled by the ISR Handler.
 *
*/
uint8_t SPI_ReceiveData_UsingInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t DataLength){

    // First check if the SPI has not already been registered for receiving data -> i.e. if SPI already in RX state, do not register another receive request.

	uint8_t state = pSPIHandle->rxState;

	if(state != SPI_BUSY_IN_RX){

		// 1. Save the RxBuffer location and size information in the SPI Handler.
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = DataLength;

		// 2. Set the state of the SPI module to busy with RX.
		pSPIHandle->rxState = SPI_BUSY_IN_RX; // This will block any other part of the application trying to use this SPI for receiving.

		// 3. Enable the RX Interrupt (RXNEIE) -> this will allow interrupts to occur whenever the RX flag in the Status Register is set.
				// From Reference manual -> RXNEIE field is 6th bit the CR2 register.
		pSPIHandle->pSPIx_BASEADDR->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. Data Receiving will be handled by the ISR handler.
	}
	return state;
}


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
void SPI_IRQHandling(SPI_Handle_t *pHandle){



	// 1. Check SR of the SPI module for the respective Interrupt.
	// 2. Check if the respective interrupt was enabled by the user.
	uint8_t temp1, temp2;

	// Checking for Transmission interrupt and if Transmission interrupt was enabled by the user.
	temp1 = pHandle->pSPIx_BASEADDR->SR & (1 << SPI_SR_TXE); // Extract status of TX; if TXE = 1, interrupt caused by transmission of data;
	temp2 = pHandle->pSPIx_BASEADDR->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){  // If the interrupt was caused by the transmission AND if user enabled transmission interrupt, HANDLE it in the if block below

		// Handle TXE
		spi_txe_interrupt_handler(pHandle); // Helper function, not available to the application.

	}

	// Checking for Reception interrupt and if reception interrupt was enabled by the user.
	temp1 = pHandle->pSPIx_BASEADDR->SR & (1 << SPI_SR_RXNE); // Extract status of RX; if RXNE = 1, interrupt caused by reception of data;
	temp2 = pHandle->pSPIx_BASEADDR->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){  // If the interrupt was caused by the reception AND if user enabled reception interrupt, HANDLE it in the if block below

		// Handle RXNE
		spi_rxne_interrupt_handler(pHandle); // Helper function, not available to the application.

	}

	/*
	 *  There are several other ERROR flags that can be handled in a similar way -> only implementing OVR error flag for showing how to handle it.
	 */
	// Checking for overrun flag and if error interrupt was enabled by the user.
	temp1 = pHandle->pSPIx_BASEADDR->SR & (1 << SPI_SR_OVR); // Extract status of OVR bit in the SR register. If OVR set, then OVERRUN error has occurred
	temp2 = pHandle->pSPIx_BASEADDR->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){  // If the interrupt was caused by OVERRUN flag AND if user enabled error interrupts, HANDLE it in the if block below

		// Handle OVR error interrupt
		spi_ovr_err_interrupt_handler(pHandle); // Helper function, not available to the application.
	}
}

/*
 *  Helper functions for  SPI_IRQHandling
 */


/*********************************************************************
 * @fn      		  - spi_txe_interrupt_handler
 *
 * @brief             - Application ISR handler calls this API function to service the interrupt caused by transmission flag.
 *
 * @param[in]         - SPIx Handle structure, it has the baseAddress and the Registers of the SPIx that needs servicing
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - The operations in this helper function are similar to the SPI_SendData blocking API. The only
 * 						 	difference is that the TXBuffer and length are accessed using the SPI Handler.
 *
 * 						In this helper function, each time it is called it does the following:
 * 							Sends the number of data bytes based on the configuration (i.e. 2 bytes or 1 byte)
 * 							Decrease the length of the data by the number of bytes it transmits.
 * 							Checks if the new length is 0 -> which indicates all data bytes sent.
 * 								In this case, the helper function closes the SPI transmission (disabling tx interrupts).
 * 										resets txBuffer and length in the handler,
 * 											and informs the application layer.
*/
static void spi_txe_interrupt_handler(SPI_Handle_t *pHandle){


		 // Check DFF bit CR1 to determine how many bytes to upload in the DR, which will push the data bytes to the Tx Buffer
		if(pHandle->pSPIx_BASEADDR->CR1 & (1 << SPI_CR1_DFF)){

			// If bit is set, then DFF = 16-bit. You need to upload 2 bytes of data into the DR register.
			//             The type casting here will convert the 8-bit pointer to a 16-bit pointer, allowing to dereference 2-bytes of consecutive data. Without the uint16_t* typecast, you would be dereferencing a byte of data.
			//                  |
			pHandle->pSPIx_BASEADDR->DR = *((uint16_t *)pHandle->pTxBuffer);

			// Pushed 2 bytes of Data into Tx Buffer, so reduced length by 2 bytes.
			pHandle->txLen--;
			pHandle->txLen--;
			// Move pointer 2 bytes ahead
			(uint16_t *)pHandle->pTxBuffer++; // This will make the pointer point to the start of the 16-bits to send.

		}else
		{
			// DFF = 8-bit, you need to upload a byte at a time.
			pHandle->pSPIx_BASEADDR->DR = *(pHandle->pTxBuffer); // Don't need type-casting as pointer is of 8-bit type.
			pHandle->txLen--;
			pHandle->pTxBuffer++;
		}

		// After transmitting data, check if length of is 0, which would indicate that we've sent all the bytes in the txBuffer. Now, we close the SPI transmission and inform the application layer about this.
			// In that case, you first disable TX interrupts first, which would stop TX interrupts from occurring. Followed by clearing txBuffer pointer. Finally, tell the application that SPI is ready for next transmission request.
		if(!pHandle->txLen){

			// Disable TX interrupts from being occurring as you don't have any more bytes to send.
			pHandle->pSPIx_BASEADDR->CR2 &= ~(1 << SPI_CR2_TXEIE);

			// Clear the TX buffers and set length to 0
			pHandle->pTxBuffer = NULL;
			pHandle->txLen =0;

			// Set SPI as ready for next Transmission request.
			pHandle->txState = SPI_READY;

			// Let the application know that you've sent the data that was given for transmission
				// SPI_ApplicationEventCallBack() can be implemented in the application layer to respond to when the SPI assigned task has been completed by the SPI module.
			SPI_ApplicationEventCallBack(pHandle, SPI_EVENT_TX_CMPLT);
		}

}

/*********************************************************************
 * @fn      		  - spi_rxne_interrupt_handler
 *
 * @brief             - Application ISR handler calls this API function to service the interrupt caused reception flag. (RX flag)
 *
 * @param[in]         - SPIx Handle structure, it has the baseAddress and the Registers of the SPIx that needs servicing
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - The operations in this helper function are similar to the SPI_ReceiveData blocking API. The only
 * 						 	difference is that the TXBuffer and length are accessed using the SPI Handler.
 *
 * 						In this helper function, each time it is called it does the following:
 * 							Receives the number of data bytes based on the configuration (i.e. 2 bytes or 1 byte)
 * 							Decrease the length of the buffer by the number of bytes it receives.
 * 							Checks if the new length is 0 -> which indicates all data bytes received.
 * 								In this case, the helper function closes the SPI reception (disabling rx interrupts).
 * 										resets rxBuffer and length in the handler,
 * 											and informs the application layer.


*/
static void spi_rxne_interrupt_handler(SPI_Handle_t *pHandle){

	//Check DFF bit CR1 to determine how many bytes to Download/grab/read from the DR, which will push the data bytes from the SPI RX buffer to the RX buffer of the program
	if(pHandle->pSPIx_BASEADDR->CR1 & (1 << SPI_CR1_DFF)){

		// If bit is set, then DFF = 16-bit. You need to upload 2 bytes of data into the DR register.
		//             The type casting here will convert the 8-bit pointer to a 16-bit pointer, allowing to dereference 2-bytes of consecutive data - in this case you will download/read 16-bits of data from SPI's RX buffer. Without the uint16_t* typecast, you would be storing a byte of data.
		//                  |
		*((uint16_t *)pHandle->pRxBuffer) = pHandle->pSPIx_BASEADDR->DR;

		// Pushed 2 bytes of Data into Rx Buffer, so reduced length by 2 bytes.
		pHandle->rxLen--;
		pHandle->rxLen--;
		// Move pointer 2 bytes ahead
		(uint16_t *)pHandle->pRxBuffer++; // This will make the pointer point to the start of the 16-bits to send.

	}else
	{
		// DFF = 8-bit, you need to read a byte at a time from the SPI's RX buffer
		*(pHandle->pRxBuffer) = pHandle->pSPIx_BASEADDR->DR; // Don't need type-casting as pointer is of 8-bit type.
		pHandle->rxLen--;
		pHandle->pRxBuffer++;
	}


	// After receiving data, check if length of is 0, which would indicate that we've received all the bytes in the rxBuffer. Now, we close the SPI reception and inform the application layer about this.
		// In that case, you first disable RX interrupts first, which would stop RX interrupts from occurring. Followed by clearing rxBuffer pointer. Finally, tell the application that SPI is ready for next reception request.
	if(!pHandle->rxLen){

		// Disable RX interrupts from being occurring as you don't have any more bytes to send.
		pHandle->pSPIx_BASEADDR->CR2 &= ~(1 << SPI_CR2_RXNEIE);

		// Clear the TX buffers and set length to 0
		pHandle->pRxBuffer = NULL;
		pHandle->rxLen =0;

		// Set SPI as ready for next reception request.
		pHandle->rxState = SPI_READY;

		// Let the application know that you've received all the data that was expected from the reception
			// SPI_ApplicationEventCallBack() can be implemented in the application layer to respond to when the SPI assigned task has been completed by the SPI module.
		SPI_ApplicationEventCallBack(pHandle, SPI_EVENT_RX_CMPLT);
	}
}

/*********************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handler
 *
 * @brief             - Application ISR handler calls this API function to service the interrupt caused RX overrun flag.
 *
 * @param[in]         - SPIx Handle structure, it has the baseAddress and the Registers of the SPIx that needs servicing
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - None

*/
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pHandle);
