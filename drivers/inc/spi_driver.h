/*
 * spi_driver.h
 *
 *  Created on: July 5, 2020
 *      Author: Junaid
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f40xx.h" /*  Include device specific header file. */

/*
 * 						A SPI configuration structure.
 * 		 	This structure holds the configuration settings of the SPIx module.
 * 			  Place the configuration Structure inside the handle structure.
 */

typedef struct{

    uint8_t DeviceMode;         /* Slave mode or Master mode? */
    uint8_t BusConfig;          /* Full duplex? Half-duplex? Simple? -> communication types */
    uint8_t SclkSpeed;          /* Communication speed? */
    uint8_t DataFrameFormat;    /* 8-bit or 16-bit shift register? */
    uint8_t ClockPolarity;      /* Idle state clock is LOW or HIGH? */
    uint8_t ClockPhase;         /* Data capture edge is the LEADING edge or the TRAILING edge of the SPI CLOCK? */
    uint8_t SlaveManagementType;    /* Software slave management ( used when single slave) or Hardware slave management (used when multiple slaves)? */

}SPI_Config_t;


/*
 * 				A Handle Structure for a SPI.
 * 		Holds SPIx configuration and Base address of the SPI being configured - i.e. SPI1, SPI2, or SPI3
 */

typedef struct{

    SPI_RegDef_t *pSPIx_BASEADDR;  /* Holds the Base address of the desired SPI peripheral (SPI1/SPI2/SPI3), the Base addresses defined in device header as MACROS, SPI1, SPI2, and SPI3 */
    SPI_Config_t SPIConfig; /* Holds the SPI configuration information - filled out by the user before calling using any of the API functions defined in this file. */

    /* The following additions were made to the Data Handle DS for Non-blocking implementation of Sending and Receiving Data */

	uint8_t *pTxBuffer;  /* Points to the Data buffer to send over SPI -> Pointer will be used by the SPI ISR code. */
	uint8_t *pRxBuffer;  /* Points to a buffer to use when receiving data over SPI -> Pointer will be used by the SPI ISR code. */
	uint8_t txLen; 	     /* Store length of Transmit buffer here. */
	uint8_t rxLen; 		 /* Store length of Receive buffer here. */
	uint8_t txState;     /* Store TX state here. */
	uint8_t rxState; 	 /* Store RX state here. */

}SPI_Handle_t;

/*
 *     SPI driver specific MACROs  - 
 *          these macros are only relevant to SPI driver
 *
 */


/**
 *  @DeviceMode
 */

#define SPI_IN_SLAVE_MODE      0      /* By default, SPIx is in Slave Mode */ 
#define SPI_IN_MASTER_MODE     1


/**
 *  @BusConfig 
 */
#define SPI_IN_FULL_DUPLEX_MODE  0   /* By Default, SPI communication type is Full duplex */ 
#define SPI_IN_HALF_DUPLEX_MODE  1
#define SPI_IN_SIMPLEX_RX_ONLY   2
#define SPI_IN_SIMPLEX_TX_ONLY   3


/**
 *  @SclkSpeed -> Serial Clock Speed, controlled by BaudRate bits of the Control Register
 *  SPI_PERIPHERAL_CLOCK_DIVx, where x = Internal prescaler value
 */ 
#define SPI_PERIPHERAL_CLOCK_DIV2   0 /* By default, SPI serial clock is Peripheral clock /2, i.e. SPI internal prescaler minimum value = 2 */ 
#define SPI_PERIPHERAL_CLOCK_DIV4   1
#define SPI_PERIPHERAL_CLOCK_DIV8   2
#define SPI_PERIPHERAL_CLOCK_DIV16  3
#define SPI_PERIPHERAL_CLOCK_DIV32  4
#define SPI_PERIPHERAL_CLOCK_DIV64  5
#define SPI_PERIPHERAL_CLOCK_DIV128 6
#define SPI_PERIPHERAL_CLOCK_DIV256 7

/**
 * @DataFrameFormat 
 */

#define SPI_DATAFRAME_8BITS 0           /* By Default, SPI communication Data Frame is 8-bits. */ 
#define SPI_DATAFRAME_16BIT 1

/**
 * @ClockPolarity
 */
#define SPI_SCLK_LOW    0               /* By Default, SPI Serial Clock is LOW at idle state. */ 
#define SPI_SCLK_HIGH   1 

/**
 * @ClockPhase
 */
#define SPI_DATA_SAMPLED_ON_LEADING_EDGE    0               /* By Default, Data Sampled on First edge of SCLK.  */ 
#define SPI_DATA_SAMPLED_ON_TRAILING_EDGE   1 


/**
 * @SlaveManagementType
 */
#define SPI_SOFTWARE_SLAVE_MANAGEMENT_DI    0           /* By Default, Software Slave Management is Disabled, i.e. SlaveManagementType = Hardware Slave Management. */ 
#define SPI_SOFTWARE_SLAVE_MANAGEMENT_EN    1

/**
 *  Masked bit fields of the Status Register, to be used for FlagName in SPI_GetFlagStatus function. 
 */ 

 //                             Using Bit Field defined in the MCU specific header file to get the masking detail for the respective flag defined in the Status Register of SPI
 //                               | 
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG        (1 << SPI_SR_TXE) 
#define SPI_CHSIDE_FLAG     (1 << SPI_SR_CHSIDE) 
#define SPI_UDR_FLAG        (1 << SPI_SR_UDR) 
#define SPI_CRCERR_FLAG     (1 << SPI_SR_CRCERR) 
#define SPI_MODF_FLAG       (1 << SPI_SR_MODF) 
#define SPI_OVR_FLAG        (1 << SPI_SR_OVR) 
#define SPI_BUSY_FLAG       (1 << SPI_SR_BSY) 
#define SPI_FRE_FLAG        (1 << SPI_SR_FRE)



/*
 *  Possible SPI Application States
 *		An SPI module can be in any one of these states.
 */
#define SPI_READY 			  0
#define SPI_BUSY_IN_RX		  1
#define SPI_BUSY_IN_TX		  2

/*
 * 					APIs supported by this SPI driver.
 *
 */


void SPI_ClockControl(SPI_RegDef_t *pSPIx,uint8_t enable_disable );
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/**
 *      SPI Peripheral Enable/Disable API
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable);

/**
 * 
 *         SPI NSS Enable/Disable API
 */ 
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable);

/**
 * 
 *         SPI SPOE Enable/Disable API
 */ 
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t Enable_Disable);


/*
 *   		Data Read & Write 
 * 
 *      For Communication protocols, there are two types of Data Read & Write APIs:
 *              Non-blocking & Blocking APIs. 
 *          Blocking -> Polling method for Data Read and Write 
 *          Non-Blocking -> Interrutp Method for Data Read and Write
 *              
 */

        /* Blocking Method APIs */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLength);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLength);


		/* Non-Blocking Method APIs */
void SPI_SendData_UsingInterrupts(SPI_Handle_t  *pSPIHandle, uint8_t *pTxBuffer, uint32_t DataLength);
void SPI_ReceiveData_UsingInterrupt(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t DataLength);


/**
 *  Get SPI Flag Status 
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 *			SPI Interrupt Configuration & Handling
 */
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t enable_disable);
void SPI_IRQ_Priority_Config (uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);




#endif /* INC_SPI_DRIVER_H_ */
