/*
 * bms_hardware.cpp
 *
 *  Created on: Nov 18, 2024
 *      Author: tjges
 */
/*!
  ADBMS181x hardware library
@verbatim
  This library contains all of the hardware dependant functions used by the bms
  code
@endverbatim

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2017 Linear Technology Corp. (LTC)
*/
//#include <Arduino.h>
#include <stdint.h>
#include "bms_hardware.h"
#include "stm32h7xx_hal.h"
//#include "Linduino.h"
//#include "LT_SPI.h"
//#include <SPI.h>

extern uint8_t BMS_CS;

void cs_low(uint8_t pin)
{
	if(BMS_CS) { // 1 = HV C7
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	} else { // LV A15
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}

void cs_high(uint8_t pin)
{
	if(BMS_CS) { // 1 = HV C7
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	} else { // LV A15
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	}
}

//void delay_u(uint16_t micro)
//{
//  delayMicroseconds(micro);
//}

void delay_m(uint16_t milli)
{
  HAL_Delay(milli);
}

/*
Writes an array of bytes out of the SPI port
*/
extern SPI_HandleTypeDef hspi1;
#define STD_Timeout                 ((uint32_t)0x0062)  /* Standard timeout of 100ms */

void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
	HAL_SPI_Transmit(&hspi1, data, len, STD_Timeout);
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
//	HAL_SPI_TransmitReceive(&hspi1, tx_Data, rx_data, rx_len, STD_Timeout);
	//Werkte niet
//	NVIC_DisableIRQ(FDCAN2_IT0_IRQn);

	HAL_SPI_Transmit(&hspi1, tx_Data, tx_len, STD_Timeout);
	HAL_SPI_Receive(&hspi1, rx_data, rx_len, STD_Timeout);

//	NVIC_EnableIRQ(FDCAN2_IT0_IRQn);


//  for (uint8_t i = 0; i < tx_len; i++)
//  {
//    SPI.transfer(tx_Data[i]);
//  }

//  for (uint8_t i = 0; i < rx_len; i++)
//  {
//
//    rx_data[i] = (uint8_t)SPI.transfer(0xFF);
//  }

}


uint8_t spi_read_byte(uint8_t tx_dat)
{
  uint8_t txdata = tx_dat;
  uint8_t rxdata;
//  HAL_SPI_Receive(&hspi1, &data, 1, STD_Timeout);
  HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, STD_Timeout);
//  data = (uint8_t)SPI.transfer(0xFF);
  return(rxdata);
}
