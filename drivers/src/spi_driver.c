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
 * @return            -  None 
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
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG));  // Same as above statement, but a much cleaner method of implementation. 
    
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
