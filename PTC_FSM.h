/*
 * PTC_FSM.h
 *
 *  Created on: Jan 20, 2025
 *      Author: larss
 */

#ifndef INC_PTC_FSM_H_
#define INC_PTC_FSM_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* In- & output pins definitions for relays */
// Output pins for setting relays
#define HVPLUS_RELAY_PIN		GPIO_PIN_3  // GPIOC
#define HVMINUS_RELAY_PIN		GPIO_PIN_1  // GPIOB
#define PRECHARGE_RELAY_PIN		GPIO_PIN_2  // GPIOF
#define DISCHARGE_RELAY_PIN		GPIO_PIN_1  // GPIOF
#define BMS_LATCH_PIN			GPIO_PIN_0  // GPIOF

// Input pins for reading out relays, Discharge has no readout contact
#define HVPLUS_RELAY_READOUT	GPIO_PIN_2  // GPIOC
#define HVMINUS_RELAY_READOUT	GPIO_PIN_10 // GPIOF
#define PRECHARGE_RELAY_READOUT	GPIO_PIN_9  // GPIOE
#define SDC_RELAY_READOUT		GPIO_PIN_0  // GPIOG
#define IMD_LATCH_READOUT		GPIO_PIN_3  // GPIOA
#define BMS_LATCH_READOUT		GPIO_PIN_0  // GPIOC

/* Pins for High Voltage Active Lights & LEDs */
#define HVAL_RED  				GPIO_PIN_2  // GPIOB
#define HVAL_GREEN  			GPIO_PIN_3	// GPIOB
#define LED_R					GPIO_PIN_14 // GPIOD
#define LED_G					GPIO_PIN_15 // GPIOD

/* Private typedef -----------------------------------------------------------*/

/* Powertrain Controller States */
typedef enum
{
	IDLE_State,
	PreCharge_State,
	HV_ON,
	Fail_State,
	Discharge_State
} PTC_State;

/* Powertrain error codes */
typedef enum {
	ERROR_NONE              = 0x00,
	ERROR_SDC_TRIGGERED     = 0x01,  // Bit 0
	ERROR_IMD_TRIGGERED     = 0x02,  // Bit 1
	ERROR_CONTACT_MISMATCH  = 0x04,  // Bit 2
	ERROR_TIMER_FAILURE     = 0x08,  // Bit 3
	ERROR_FDCAN_FAILED      = 0x10,  // Bit 4
	ERROR_OVERVOLTAGE       = 0x20,   // Bit 5
	ERROR_BMS_FAIL			= 0x40   // Bit 6
} ErrorCode_t;

/* Private variables ---------------------------------------------------------*/

extern volatile uint8_t buttonPressedFlag;
extern volatile PTC_State currentState;
extern TIM_HandleTypeDef htim6;

/* Function prototypes -------------------------------------------------------*/

/* General functions*/
void Set_LEDs(GPIO_PinState stateLED3, GPIO_PinState stateLED2, GPIO_PinState stateLED1);
void DisableTimer(TIM_HandleTypeDef *htim);
void PTC_UpdateFSM(void);
void PTC_CheckRelayContacts(GPIO_PinState expectedHVPlus,
							GPIO_PinState expectedHVMinus,
							GPIO_PinState expectedPrecharge,
							GPIO_PinState expectedSDC);

/* State Handler Functions */
void IDLE_State_Handler(void);
void PreCharge_State_Handler(void);
void HV_ON_Handler(void);
void Fail_State_Handler(void);
void Discharge_State_Handler(void);

/* PTC error handler functions */
void PTC_ErrorCheck(void);
void RaiseError(ErrorCode_t error);
void ClearError(ErrorCode_t error);

#endif /* INC_PTC_FSM_H_ */
