/*
 * PTC_FDCAN.h
 *
 *  Created on: Jan 21, 2025
 *      Author: larss
 */

#ifndef INC_PTC_FDCAN_H_
#define INC_PTC_FDCAN_H_

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* IVT-S CAN IDs */
#define IVT_COMMAND_CANID ((uint16_t)0x411)
#define IVT_RESPONSE_CANID ((uint16_t)0x511)
#define IVT_RESULTI_CANID ((uint16_t)0x521)
#define IVT_RESULTU1_CANID ((uint16_t)0x522)
#define IVT_RESULTU2_CANID ((uint16_t)0x523)
#define IVT_RESULTU3_CANID ((uint16_t)0x524)
#define IMD_INFO_CANID ((uint16_t)0x37)

/* Private variables --------------------------------------------------------*/
/* IVT-S CAN Command Constants */
static const uint8_t IVT_STOP_CMD[8]          = {0x34, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // Stop Mode
static const uint8_t IVT_CONFIG_CURRENT_CMD[8] = {0x20, 0x02, 0x07, 0xD0, 0x00, 0x00, 0x00, 0x00}; // Cyclic Current 2000ms
static const uint8_t IVT_CONFIG_U1_CMD[8]    = {0x21, 0x02, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00}; // Disable U1
static const uint8_t IVT_CONFIG_U2_CMD[8]    = {0x22, 0x02, 0x07, 0xD0, 0x00, 0x00, 0x00, 0x00}; // Disable U2
static const uint8_t IVT_CONFIG_U3_CMD[8]    = {0x23, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Disable U3
static const uint8_t IVT_STORE_CMD[8]         = {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Store Config
static const uint8_t IVT_START_CMD[8]         = {0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // Start Mode

/* Function prototypes -------------------------------------------------------*/
extern void Error_Handler(void);
void PTC_FDCAN_Setup(FDCAN_HandleTypeDef *hfdcan);
void General_FDCAN_Setup(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef PTC_FDCAN_SendMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t canID, uint32_t dataLength, const uint8_t *TxData);
void PTC_SendCyclicMessage(void);
void PTC_ControlCodeHandler(uint8_t controlCode);

/* IMD & Isabellenhutte functions */
void IVT_SetConfiguration(void);
void IVT_SendCommand(FDCAN_HandleTypeDef *hfdcan, const uint8_t *data);
void IMD_InfoGeneralHandler(uint8_t *RxData);

#endif /* INC_PTC_FDCAN_H_ */
