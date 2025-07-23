/*
 * PTC_FSM.c
 *
 *  Created on: Jan 20, 2025
 *      Author: larss
 */

#include "PTC_FSM.h"
#include "main.h"
#include "PTC_FDCAN.h"

volatile PTC_State currentState = IDLE_State;
uint32_t relayComparisonCounter = 0;
volatile uint8_t errorStatus; // Global error status variable
volatile uint8_t failMessageSent = 0;

// Precharge variables
static uint8_t prechargeTimerStarted = 0;  // 0 = Not started, 1 = Started
static int32_t prechargeTrigger;
volatile uint8_t prechargeTriggerOK = 0;

// External variables
extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim14;
extern int32_t vDCLink;
extern int32_t vBatt;
extern uint8_t *LV_prot_state;
extern uint8_t *HV_prot_state;
extern uint8_t *HV_fail_reason;
extern uint8_t *LV_fail_reason;


void PTC_UpdateFSM(void)
{
//	relayComparisonCounter++;
	PTC_ErrorCheck(); // Error check for BMS, IMD & SDC errors

	switch( currentState )
	{
	  case IDLE_State:
		// Code for IDLE state
		IDLE_State_Handler();
		break;

	  case PreCharge_State:
		// Code for Precharge state
		PreCharge_State_Handler();
		break;

	  case HV_ON:
		// Code for HV On state
		HV_ON_Handler();
		break;

	  case Discharge_State:
		// Code for Discharge State
		Discharge_State_Handler();
		break;

	  case Fail_State:
		// Code for Failure state
		Fail_State_Handler();
		break;

	  default:
		currentState = Fail_State;
		break;
	}
}

/* Powertrain State Handler functions -----------------------------------------*/

/**
  * @brief Handles IDLE State: LV is ON. HVAL should indicate <60 V,
  * all relays output pins low. Input pins should mirror output pins, if not --> Error
  * IDLE STATE IS ONLY LEFT THROUGH AN ERROR OR COMMAND FROM MAIN PCB
  * @param None
  * @retval None
  */
void IDLE_State_Handler(void)
{
	// '000'
	Set_LEDs(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);

	// In IDLE, all relays are reset (i.e. not activated)
	// The discharge relay is normally closed.
	HAL_GPIO_WritePin(GPIOC, HVPLUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HVMINUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, PRECHARGE_RELAY_PIN|DISCHARGE_RELAY_PIN, GPIO_PIN_RESET);

	GPIO_PinState expectedHVPlus    = GPIO_PIN_RESET;
	GPIO_PinState expectedHVMinus   = GPIO_PIN_RESET;
	GPIO_PinState expectedPrecharge = GPIO_PIN_RESET;
	GPIO_PinState expectedSDC       = GPIO_PIN_SET;

	// Check if readout of relays match the settings
	PTC_CheckRelayContacts(expectedHVPlus, expectedHVMinus, expectedPrecharge, expectedSDC);

//	if (buttonPressedFlag)
//	{
//		buttonPressedFlag = 0;	// Reset button flag
//		currentState = PreCharge_State;
//	}
}

/**
  * @brief Handles Pre Charge State: LV is ON. HV-, discharge & precharge output pins high,
  * other relay output pins low. Monitors V BUS Isabellenhütte. Timer for HVB voltage
  *  Input pins should mirror output pins, if not --> Error
  * @param None
  * @retval None
  */
void PreCharge_State_Handler(void)
{
	// '110'
	Set_LEDs(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);

	// PreCharge: HV+ is reset (off), while HV-, Precharge and Discharge are set (on).
	HAL_GPIO_WritePin(GPIOC, HVPLUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HVMINUS_RELAY_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PRECHARGE_RELAY_PIN|DISCHARGE_RELAY_PIN, GPIO_PIN_SET);

	// Check if readout of relays match the settings
	GPIO_PinState expectedHVPlus    = GPIO_PIN_RESET;
	GPIO_PinState expectedSDC       = GPIO_PIN_RESET;
	GPIO_PinState expectedHVMinus   = GPIO_PIN_SET;
	GPIO_PinState expectedPrecharge = GPIO_PIN_SET;
	PTC_CheckRelayContacts(expectedHVPlus, expectedHVMinus, expectedPrecharge, expectedSDC);

	if (!prechargeTimerStarted) // Check if Precharge timer is not started already
	{
		HAL_TIM_Base_Start_IT(&htim6); // Start timer for Precharge
		prechargeTimerStarted = 1;     // Set timer started flag
	}

	// Wait until there is a valid DC link voltage measurement
	if(prechargeTriggerOK >= 5)
	{
		if (vDCLink < 60000)
		{
			currentState = Fail_State;
//			printf("Precharge failed: vBatt < 60V\r\n");
//			printf("V_batt: %ld mV\r\n", vBatt);
			return;
		}
		prechargeTrigger = (95 * vDCLink) / 100;
//		prechargeTrigger = 0;
//		printf("Trigger: %ld mV\r\n", prechargeTrigger);
//		printf("V_batt: %ld mV\r\n", vBatt);
//		printf("V_bus: %ld mV\r\n", vDCLink);
//		printf("\r\n");

		if (vBatt >= prechargeTrigger) // Check if HV is at 95% of the total HV
		{
			currentState = HV_ON; // Precharge completed --> go to HV ON state
			prechargeTriggerOK = 0;
//			printf("Precharge SUCCESS!\r\n");
			return;
		}
	}

//	if (buttonPressedFlag)
//	{
//		buttonPressedFlag = 0;	// Reset button flag
//		currentState = HV_ON;
//	}
}

/**
  * @brief Handles HV ON State: LV is ON. HV-, & discharge relay output pins high,
  * other relay output pins low. HVAL should indicate >60V & contacts closed
  *  Input pins should mirror output pins, if not --> Error
  * @param None
  * @retval None
  */
void HV_ON_Handler(void)
{
	DisableTimer(&htim6); 		  // Stop the Precharge timer
	prechargeTimerStarted = 0;    // Reset Precharge timer started flag

	// '010'
	Set_LEDs(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);

	// HV On: HV+, HV- and Discharge relays are set, Precharge is reset.
	HAL_GPIO_WritePin(GPIOC, HVPLUS_RELAY_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, HVMINUS_RELAY_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, DISCHARGE_RELAY_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PRECHARGE_RELAY_PIN, GPIO_PIN_RESET);

	GPIO_PinState expectedHVPlus    = GPIO_PIN_SET;
	GPIO_PinState expectedHVMinus   = GPIO_PIN_SET;
	GPIO_PinState expectedPrecharge = GPIO_PIN_RESET;
	GPIO_PinState expectedSDC       = GPIO_PIN_RESET;

	// Check if readout of relays match the settings
	PTC_CheckRelayContacts(expectedHVPlus, expectedHVMinus, expectedPrecharge, expectedSDC);

	// If HV is below 50V --> assume power is off and go from FAIL to IDLE
	if (vDCLink <= 50000)
	{
		currentState = Fail_State;
//		printf("DC Link fail: vDCLink < 50V\r\n");
		return;
	}


//	TODO Implement charging mode as below
//	LVBatt_chargeFlag = Get From HV BMS ;
//	if (LVBatt_chargeFlag && (current < threshold))
//	{
//		HAL_GPIO_WritePin(GPIOX, chargerRelayPin, GPIO_PIN_SET);
//		Send CAN charging message;
//	}
//	else
//	{
//		HAL_GPIO_WritePin(GPIOX, chargerRelayPin, GPIO_PIN_RESET);
//	}

	if (buttonPressedFlag)
	{
		 buttonPressedFlag = 0;	// Reset button flag
		 currentState = Discharge_State;
	}
}

/**
  * @brief Handles Discharge State: LV is ON. HV- & HV+ output pins low,
  * other relay output pins low. HVAL should indicate >60V and contact open
  *  Input pins should mirror output pins, if not --> Error
  * @param None
  * @retval None
  */
void Discharge_State_Handler(void)
{
	// '110'
	Set_LEDs(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET);

	// Discharge: All relays are reset.
	HAL_GPIO_WritePin(GPIOC, HVPLUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HVMINUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, PRECHARGE_RELAY_PIN|DISCHARGE_RELAY_PIN, GPIO_PIN_RESET);

	GPIO_PinState expectedHVPlus    = GPIO_PIN_RESET;
	GPIO_PinState expectedHVMinus   = GPIO_PIN_RESET;
	GPIO_PinState expectedPrecharge = GPIO_PIN_RESET;
	GPIO_PinState expectedSDC       = GPIO_PIN_RESET;

	// Check if readout of relays match the settings
	PTC_CheckRelayContacts(expectedHVPlus, expectedHVMinus, expectedPrecharge, expectedSDC);

	if (vDCLink < 60000) // HV is below discharge threshold
	{
		currentState = IDLE_State;
		return;
	}

//	if (buttonPressedFlag)
//	{
//		 buttonPressedFlag = 0; // Reset button flag
//		 RaiseError(ERROR_SDC_TRIGGERED); // FOR TESTING/DEBUGGING
//		 currentState = Fail_State;
//	}
}

/**
  * @brief Handles FAIL State: All relay output pins low. Send FAIL code to main PCB.
  * ONLY LEAVE FAIL STATE IF ALL CRITICAL ERROR FLAGS ARE CLEARED
  * @param None
  * @retval None
  */
void Fail_State_Handler(void)
{
	DisableTimer(&htim6); // Stop the Precharge timer
	prechargeTimerStarted = 0;    // Reset Precharge timer started flag
	prechargeTriggerOK = 0;

	// '100'
	Set_LEDs(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);

	// Fail: all relays are reset.
	HAL_GPIO_WritePin(GPIOC, HVPLUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, HVMINUS_RELAY_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, PRECHARGE_RELAY_PIN|DISCHARGE_RELAY_PIN, GPIO_PIN_RESET);

	GPIO_PinState expectedSDC       = GPIO_PIN_RESET;
	GPIO_PinState expectedHVPlus    = GPIO_PIN_RESET;
	GPIO_PinState expectedHVMinus   = GPIO_PIN_RESET;
	GPIO_PinState expectedPrecharge = GPIO_PIN_RESET;

	// Check if readout of relays match the settings
	PTC_CheckRelayContacts(expectedHVPlus, expectedHVMinus, expectedPrecharge, expectedSDC);

	if (errorStatus & ERROR_FDCAN_FAILED)
	{
		HAL_TIM_Base_Stop_IT(&htim14); // Stop timer for cyclic CAN sends
		HAL_FDCAN_Stop(&hfdcan2);
		if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
		{
			Error_Handler();
		}
//		if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
//		{
//			Error_Handler();
//		}
		General_FDCAN_Setup(&hfdcan2);
		ClearError(ERROR_FDCAN_FAILED);
		HAL_TIM_Base_Start_IT(&htim14); // Start timer for cyclic CAN sends
	}

	// Send fail message to main PCB if it's not sent yet
	if (!failMessageSent)
	{
		uint8_t txDataErrorMsg[3] = {0};
		txDataErrorMsg[0] = errorStatus;
		txDataErrorMsg[1] = *LV_prot_state;
		txDataErrorMsg[2] = *HV_prot_state;

		PTC_FDCAN_SendMessage(&hfdcan2, 0x33, FDCAN_DLC_BYTES_2, txDataErrorMsg);
		failMessageSent = 1;  // set fail message flag
	}

	// Check if errors are cleared
	if (errorStatus == ERROR_NONE)
	{
		currentState = IDLE_State;
		failMessageSent = 0;  // reset fail message flag
//		printf("Error status 0 --> IDLE\r\n");
		return;
	}

//	if (buttonPressedFlag)
//	{
//		 ClearError(0xFF); // Clear all error flags
//		 buttonPressedFlag = 0;	// Reset button flag
//		 currentState = IDLE_State;
//	}
}

/**
  * @brief  Checks the relay contacts against expected states.
  * @param  expectedHVPlus: Expected state for HV+ relay (GPIO_PIN_SET or GPIO_PIN_RESET)
  * @param  expectedHVMinus: Expected state for HV- relay
  * @param  expectedPrecharge: Expected state for Precharge relay
  * @param  expectedDischarge: Expected state for Discharge relay
  * @param  expectedSDC: Expected state for SDC relay
  * @retval None. In case of a mismatch, you can call an error handler or log the issue.
  */
void PTC_CheckRelayContacts(GPIO_PinState expectedHVPlus,
							GPIO_PinState expectedHVMinus,
							GPIO_PinState expectedPrecharge,
							GPIO_PinState expectedSDC)
{
	if (relayComparisonCounter >= 100)
	{
		// Read actual states from GPIOF (the readout pins)
		GPIO_PinState actualHVPlus    = HAL_GPIO_ReadPin(GPIOC, HVPLUS_RELAY_READOUT);
		GPIO_PinState actualHVMinus   = HAL_GPIO_ReadPin(GPIOF, HVMINUS_RELAY_READOUT);
		GPIO_PinState actualPrecharge = HAL_GPIO_ReadPin(GPIOE, PRECHARGE_RELAY_READOUT);
		GPIO_PinState actualSDC       = HAL_GPIO_ReadPin(GPIOG, SDC_RELAY_READOUT);

		// Check each relay. If there is a mismatch, enter Error Handler.
		if (actualHVPlus != expectedHVPlus)
		{
			printf("Error: HV+ relay contact mismatch!\r\n");
			RaiseError(ERROR_CONTACT_MISMATCH);
		}
		if (actualHVMinus != expectedHVMinus)
		{
			printf("Error: HV- relay contact mismatch!\r\n");
			RaiseError(ERROR_CONTACT_MISMATCH);
		}
		if (actualPrecharge != expectedPrecharge)
		{
			printf("Error: Precharge relay contact mismatch!\r\n");
			RaiseError(ERROR_CONTACT_MISMATCH);
		}
//		if (actualSDC != expectedSDC)
//		{
//			printf("Error: SDC relay contact mismatch!\r\n");
//			RaiseError(ERROR_CONTACT_MISMATCH);
//		}
		relayComparisonCounter = 0;
	}
}

void PTC_ErrorCheck(void)
{
	if(errorStatus != ERROR_NONE) // If any error is present --> Fail State
	{
		currentState = Fail_State;
	}

	if (((*LV_prot_state)&0b11110000) || ((*HV_prot_state)&0b11110000) || *HV_fail_reason || *LV_fail_reason) // If BMS has any error --> Raise error
	{
		RaiseError(ERROR_BMS_FAIL);
	}
	else
	{
		ClearError(ERROR_BMS_FAIL); // No BMS failures --> Clear BMS error flag
	}

	if(HAL_GPIO_ReadPin(GPIOG, SDC_RELAY_READOUT) == GPIO_PIN_RESET) // SDC latch
	{
		if(currentState != Fail_State)
		{
			RaiseError(ERROR_SDC_TRIGGERED);
		}
	}
	else
	{
		// SDC is not triggered – if previously an SDC error was raised, clear it.
		if (errorStatus & ERROR_SDC_TRIGGERED)
		{
			ClearError(ERROR_SDC_TRIGGERED);
		}
	}
}

/* Set bit corresponding to error */
void RaiseError(ErrorCode_t error)
{
	errorStatus |= error;
	currentState = Fail_State;
}

/* Clear bit corresponding to error that is solved */
void ClearError(ErrorCode_t error)
{
    errorStatus &= ~error;  // Clear the specified error bit(s)
}

/* Function to reset the precharge timer */
void DisableTimer(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0); // Reset the timer counter
	HAL_TIM_Base_Stop_IT(htim);    // Stop the timer interrupt
}

///* Powertrain Controller State representations on LEDs */
void Set_LEDs(GPIO_PinState stateLED3, GPIO_PinState stateLED2, GPIO_PinState stateLED1)
{
//	// Set or Reset LED1
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, stateLED1);
//	// Set or Reset LED2
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, stateLED2);
//	// Set or Reset LED3
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, stateLED3);
//	HAL_GPIO_WritePin(GPIOD, LED_R, stateLED3);
//	HAL_GPIO_WritePin(GPIOD, LED_G, stateLED2);
}
