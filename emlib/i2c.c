#include <stdio.h>
#include <stdbool.h>
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"

#include "FreeRTOS.h"
#include "semphr.h"

static uint8_t device_addr;

SemaphoreHandle_t xSemaphore = NULL;

void BSP_I2C_Init(uint8_t addr) {

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	CMU_ClockEnable(cmuClock_I2C1, true);
	GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
	I2C1->ROUTE = I2C_ROUTE_SDAPEN |
		I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC0;
	I2C_Init(I2C1, &i2cInit);

	xSemaphore = xSemaphoreCreateMutex();

	device_addr = addr;
}

/**
 * @brief Write register using default I2C bus
 * @param reg register to write
 * @param data data to write
 * @return true on success
 */
bool I2C_WriteRegister(uint8_t reg, uint8_t data) {
	I2C_TransferReturn_TypeDef I2C_Status;
	bool ret_value = false;

	I2C_TransferSeq_TypeDef seq;
	uint8_t dataW[2];

	if( xSemaphore != NULL )
	    {
	        /* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
	        if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
	        {
	            /* We were able to obtain the semaphore and can now access the
	            shared resource. */

	        	seq.addr = device_addr;
				seq.flags = I2C_FLAG_WRITE;

				/* Register to write: 0x67 ( INT_FLAT )*/
				dataW[0] = reg;
				dataW[1] = data;

				seq.buf[0].data = dataW;
				seq.buf[0].len = 2;
				I2C_Status = I2C_TransferInit(I2C1, &seq);

				while (I2C_Status == i2cTransferInProgress) {
					I2C_Status = I2C_Transfer(I2C1);
				}

				if (I2C_Status != i2cTransferDone) {
					ret_value = false;
				}
				else {
					ret_value = true;
				}

	            /* We have finished accessing the shared resource.  Release the
	            semaphore. */
	            xSemaphoreGive( xSemaphore );
	        }
	        else
	        {
	            /* We could not obtain the semaphore and can therefore not access
	            the shared resource safely. */
	        }
	    }

	return ret_value;
}

/**
 * @brief Read register from I2C device
 * @param reg Register to read
 * @param val Value read
 * @return true on success
 */
bool I2C_ReadRegister(uint8_t reg, uint16_t* val) {
	I2C_TransferReturn_TypeDef I2C_Status;
	I2C_TransferSeq_TypeDef seq;
	uint8_t data[2];

	bool ret_value = false;

	if( xSemaphore != NULL )
	    {
	        /* See if we can obtain the semaphore.  If the semaphore is not
	        available wait 10 ticks to see if it becomes free. */
	        if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
	        {
	            /* We were able to obtain the semaphore and can now access the
	            shared resource. */

	        	seq.addr = device_addr;
				seq.flags = I2C_FLAG_WRITE_READ;

				seq.buf[0].data = &reg;
				seq.buf[0].len = 1;
				seq.buf[1].data = data;
				seq.buf[1].len = 1;

				I2C_Status = I2C_TransferInit(I2C1, &seq);

				while (I2C_Status == i2cTransferInProgress) {
					I2C_Status = I2C_Transfer(I2C1);
				}

				if (I2C_Status != i2cTransferDone) {
					ret_value = false;
				}
				else {
					ret_value = true;
				}

				*val = data[0];

	            /* We have finished accessing the shared resource.  Release the
	            semaphore. */
	            xSemaphoreGive( xSemaphore );
	        }
	        else
	        {
	            /* We could not obtain the semaphore and can therefore not access
	            the shared resource safely. */
	        }
	    }

	return ret_value;
}

bool I2C_Test() {
	uint16_t data;
	uint16_t co2 = 0;

	BSP_I2C_Init(0xB6);

	if(!I2C_ReadRegister(0x20, &data)) { // HW_ID
		printf("Error de lectura.\n");
	}

	printf("HW_ID: 0x%02X\n", data);
	if (data == 0x81) printf("HW_ID Correcte.\n");
	else printf("HW_ID Incorrecte.\n");

	if (!I2C_ReadRegister(0x00, &data)) { // STATUS
			printf("Error de lectura.\n");
	}

	printf("STATUS: 0x%02X\n", data);
	if (data == 0x10) {
		printf("STATUS Correcte.\n");

		if (!I2C_ReadRegister(0x02, &co2)) { // ALG_RESULT_DATA
			printf("Error de lectura.\n");
		}

		printf("CO2 Level: 0x%04X\n", co2);
	}
	else {
		printf("STATUS Incorrecte. Hi ha algun error.\n");
		if (!I2C_ReadRegister(0xE0, &data)) { // ERROR_ID
			printf("Error de lectura.\n");
		}

		printf("Error ID: 0x%02X\n", data);
	}

	// if(data & 0b00000000) {
	//	 printf("There is an error.\n");
	//	 printf("I2C: 0x%02X\n", data);
	// }
	return true;
}

bool Operate(uint8_t reg, uint8_t data) {
	return true;
}

bool Send(uint8_t reg, uint8_t data) {
	return true;
}
