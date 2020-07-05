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

}SPI_Handle_t;

#endif /* INC_SPI_DRIVER_H_ */