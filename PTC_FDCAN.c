/*
 ******************************************************************************
 * @file           : PTC_FDCAN.c
 * @brief          : Contains functions for handling CAN communications in PTC
 * 					- Filters are set up for CAN IDs expected from the PT & General CAN bus
 * 					- Any unexpected CAN ID is ignored
 * 					- PT bus uses hfdcan1 & General bus uses hfdcan2
 * 					- PT bus Baudrate is 500kbit/s & General bus Baudrate is 1Mbit/s
 * 					- RX FIFO0 is for PT bus, RX FIFO1 is for General bus
 * 					- Reception of CAN frames is detected through interrupts
 * 					- PTC & BMS update messages are sent at 10Hz
 ******************************************************************************
 *  Created on: Jan 21, 2025
 *      Author: Lars Schuddeboom
 */

// TODO fdcan2 polarity naar low zetten --> B8

#include "PTC_FDCAN.h"
#include "PTC_FSM.h"
#include "BMS_funcs.h"
#include "BMS_ADBMS1818.h"

/* Variables -------------------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;	// CAN module for PT bus
extern FDCAN_HandleTypeDef hfdcan2; // CAN module for General bus
extern volatile PTC_State currentState;
extern uint8_t prechargeTriggerOK;
extern volatile uint8_t errorStatus;
extern TIM_HandleTypeDef htim14;
extern COMP_HandleTypeDef hcomp1; //HVAL_RED

// Variables for LV BMS Logs
extern uint8_t *LV_slavesNumber;
extern cell_asic *LV_slaves;
extern BMSmodule *LV_modules;
extern uint8_t *LV_fail_reason;

// Variables for HV BMS Logs
extern uint8_t *HV_slavesNumber;
extern cell_asic *HV_slaves;
extern BMSmodule *HV_modules;
extern uint8_t *HV_fail_reason;

// Variables for IMD and Isabellenhutte updates
uint8_t IVT_commandReceivedFlag = 0;
uint8_t IVT_configComplete = 0;
int32_t IVT_Current;
int32_t vDCLink; //U1 of IVT-s
int32_t vBatt;  // U2 of IVT-S
int32_t voltageU3_mV;
uint16_t isolationResistance_kOhm;
uint8_t isolationStatus_kOhm;
uint8_t IMD_measurementCounter;
uint16_t IMD_warnings;
uint8_t IMD_deviceActivity;

// General variables for CAN
volatile HAL_StatusTypeDef IVT_StartupFlag = HAL_OK;
FDCAN_FilterTypeDef			sFilterConfig;
FDCAN_RxHeaderTypeDef 		RxHeader;
FDCAN_TxHeaderTypeDef 		TxHeader;
uint8_t TxData[8] = {0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xFF};
uint8_t RxData1[8];
uint8_t RxData2[1];
uint8_t CAN_errorCounter = 0;

// BMS variables
uint8_t BMS_resetFlag = 0;

uint16_t HV_tMax = 0;
uint16_t HV_tMin = 0;


/* Functions -------------------------------------------------------------------------*/

/**
  * @brief  Sets filters, notifications & starts FDCAN controller for PT CAN Bus
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  */
void PTC_FDCAN_Setup(FDCAN_HandleTypeDef *hfdcan)
{
	// Filter for IVT-S U1 voltage
	sFilterConfig.IdType 		 = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex  = 0;
	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 	 = IVT_RESULTU1_CANID;  // IVT-S U1 voltage CAN ID
	sFilterConfig.FilterID2    = 0x7FF;  // Accept all bits
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for IVT-S U2 voltage data
	sFilterConfig.FilterIndex  = 1;
	sFilterConfig.FilterID1    = IVT_RESULTU2_CANID; // IVT-S U2 voltage CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for IVT-S U3 voltage data
	sFilterConfig.FilterIndex  = 2;
	sFilterConfig.FilterID1    = IVT_RESULTU3_CANID; // IVT-S U3 voltage CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for IVT-S Current data
	sFilterConfig.FilterIndex  = 3;
	sFilterConfig.FilterID1    = IVT_RESULTI_CANID; // IVT-S current CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for command acknowledgement
	sFilterConfig.FilterIndex  = 4;
	sFilterConfig.FilterID1    = IVT_RESPONSE_CANID; // IVT-S Command ACK CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for ISO175 General Info
	sFilterConfig.FilterIndex  = 5;  // Filter for ISO175 General Info
	sFilterConfig.FilterID1    = IMD_INFO_CANID; // ISO175 General Info CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	sFilterConfig.IdType 		= FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex   = 6;
	sFilterConfig.FilterID1		= 0x18FF01F4;
	sFilterConfig.FilterID2 	= 0x1FFFFFFF;
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Activate Notifications
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
	  Error_Handler();
	}

	/* Start FDCAN controller */
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
	{
	  Error_Handler();
	}
}

/**
  * @brief  Sets filters, notifications & starts FDCAN controller for General CAN Bus
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  */
void General_FDCAN_Setup(FDCAN_HandleTypeDef *hfdcan)
{
	// Filter for main FSM update
	sFilterConfig.IdType 		 = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex  = 0;
	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 	 = 0x190;
	sFilterConfig.FilterID2    = 0x7FF;  // Accept all bits
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Filter for Powertrain control message
	sFilterConfig.FilterIndex  = 1;
	sFilterConfig.FilterID1    = 0x1AB; // PT Control CAN ID
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	// Activate Notifications
	if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
	{
	  Error_Handler();
	}

	/* Start FDCAN controller */
	if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
	{
	  Error_Handler();
	}
}

/**
  * @brief  Rx FIFO 0 callback for the PT CAN Bus.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
	      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData1) != HAL_OK)
	      {
	          Error_Handler();
	      }

	      // Process data based on CAN ID
	      switch (RxHeader.Identifier)
	      {
	      case IVT_RESPONSE_CANID:
				if (RxData1[0] == 0xB4) // Check if command is acknowledged
				{
					if (RxData1[1] == 0x01 && RxData1[2] == 0x01) // Start command
					{
						IVT_commandReceivedFlag = 1;
					}
					if (RxData1[1] == 0x00 && RxData1[2] == 0x01) // Stop command
					{
						IVT_commandReceivedFlag = 1;
					}
				}

				if ((RxData1[0] == 0xA0) || (RxData1[0] == 0xA1) || (RxData1[0] == 0xB2)
					 || (RxData1[0] == 0xA2) || (RxData1[0] == 0xA3))
				{
					IVT_commandReceivedFlag = 1;
				}
				break;

		  case IVT_RESULTI_CANID: // IVT-S Current data: Data Byte 0 (DB0) = 0x00
			  // Current data is stored in bytes 2-5
			  IVT_Current = ((RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5]);
//			  printf("Current: %ld mA\r\n", IVT_Current);
			  break;

		  case IVT_RESULTU1_CANID: // IVT-S Voltage data (U1): Data Byte 0 (DB0) = 0x01
			  // Voltage U1 data is stored in bytes 2-5
			  vDCLink = ((RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5]);
			  // If the state is Precharge,
			  if ( currentState == PreCharge_State
				   && (prechargeTriggerOK < 5))
			  {
				  prechargeTriggerOK++;
			  }
//			  printf("V_DCLink: %ld mV\r\n", vDCLink);
			  break;

		  case IVT_RESULTU2_CANID: // IVT-S Voltage data (U2): Data Byte 0 (DB0) = 0x02
			  // Voltage U2 data is stored in bytes 2-5
//			  vBatt = (RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5];
//
//			  // If the state is Precharge,
//			  if ( currentState == PreCharge_State
//				   && (prechargeTriggerOK < 5))
//			  {
//				  prechargeTriggerOK++;
//			  }
//			  printf("V_batt: %ld mV\r\n", vBatt);
			  break;

		  case IVT_RESULTU3_CANID: // IVT-S Voltage data (U3): Data Byte 0 (DB0) = 0x03
			  // Voltage U3 data is stored in bytes 2-5
//			  voltageU3_mV = (RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5];
//			  printf("IVT-S Voltage Data (U3): %ld mV\r\n", voltageU3_mV);
			  vBatt = (RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5];


//			  printf("V_batt: %ld mV\r\n", vBatt);
			  break;

		  case 0x37: // ISO175 General Info
			  IMD_InfoGeneralHandler(RxData1);
			  break;

		  case 0x18FF01F4: // ISO175 General info (J1939)
			  IMD_InfoGeneralHandler(RxData1);

		  default: // Unexpected CAN IDs get ignored
			  break;
	      }
//	      printf("\r\n");
	 }
}

/**
  * @brief  Rx FIFO 1 callback for the General Bus.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData2) != HAL_OK)
		{
			Error_Handler();
		}

		// Process data based on CAN ID
		switch (RxHeader.Identifier)
		{
		case 0x190: // Main PCB FSM update ID: Used to control PTC state
			uint8_t PTC_controlCode = RxData2[0];
			PTC_ControlCodeHandler(PTC_controlCode); // Handle received control code
//			printf("Control code received: %02X\r\n", PTC_controlCode);
			break;

		case 0x1AB: // Reset from precharge fail & BMS: overvoltage, undervoltage, overtemp., undertemp.
			if ((RxData2[0] == 1) && (errorStatus == ERROR_PRECHARGE_FAILURE))
			{
				ClearError(ERROR_PRECHARGE_FAILURE);
				ClearError(ERROR_FDCAN_FAILED);
			}

			if (errorStatus & ERROR_BMS_FAIL)
			{
				BMS_resetFlag = 1;
			}

		default: // Unexpected CAN IDs get ignored
			break;
		}
	}
}

/**
  * @brief  Handler for PTC control commands
  * @param  controlCode 8 bits indicating which state to transition to
  */
void PTC_ControlCodeHandler(uint8_t controlCode)
{
	switch (controlCode)
	{
	case 0x04: // Turn HV On. --- HV can only be turned on in IDLE ---
		if (currentState == IDLE_State)
		{
			currentState = PreCharge_State; // Go to precharge state
		}
		// else {} --> TBD: can be a message saying it's not possible, or just nothing
		break;

	case 0x0B: // Turn HV Off. --- HV can only be turned off in HV On ---
		if ((currentState == HV_ON) || (currentState == PreCharge_State))
		{
			currentState = Discharge_State;
		}
		break;

	case 0x03: // Precharge reset
		if (errorStatus == ERROR_PRECHARGE_FAILURE)
		{
			ClearError(ERROR_PRECHARGE_FAILURE);
			ClearError(ERROR_FDCAN_FAILED);
		}
		break;

//	case 0xBB:
//		if (errorStatus & ERROR_BMS_FAIL)
//		{
//			BMS_resetFlag = 1;
//		}
//		break;

	default: // Unexpected control codes are ignored
		break;
	}
}

/**
 * @brief Sends a CAN message via the FDCAN TX FIFO.
 * @param hfdcan: Pointer to the FDCAN handle (e.g., &hfdcan1).
 * @param canID: CAN ID of the message (standard 11-bit).
 * @param dataLength: Length of the message data in bytes (0 to 8 for Classic CAN).
 * @param data: Pointer to the message data array (up to 8 bytes for Classic CAN).
 * @retval HAL_StatusTypeDef: Returns HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef PTC_FDCAN_SendMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t canID, uint32_t dataLength, const uint8_t *TxData)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    // Ensure dataLength is valid for Classic CAN
    if (dataLength > FDCAN_DLC_BYTES_8)
    {
        return HAL_ERROR; // Invalid length
    }

    // Configure the CAN message header
    TxHeader.Identifier = canID;                          // Set the CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;                  // Use standard 11-bit CAN ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;              // Data frame
    TxHeader.DataLength = dataLength;             		  // DLC encoded in the higher bits
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;      // Error state active
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;               // No bit rate switching
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                // Classic CAN format
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     // No Tx event FIFO
    TxHeader.MessageMarker = 0;                           // No marker

    // Add the message to the transmit FIFO
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
    {
        // Transmission failed
        return HAL_ERROR;
    }

    return HAL_OK; // Transmission succeeded
}


/**
 * @brief Sends cyclic log messages to general CAN Bus at 10 Hz
 * 		  Log messages are: PTC logs, BMS log 1 & BMS log 2
 */
void PTC_SendCyclicMessage(void)
{
	/* PTC FSM Logging message */
	// TODO state of charge, voltage of all cells connected in series,
	//      >25% of all cell temps
	uint8_t txData_PTCLog1[5] = {0};
	txData_PTCLog1[0] = (uint8_t)currentState;

//	txData_PTCLog1[0] = 1;
//	txData_PTCLog1[1] = 2;
//	txData_PTCLog1[2] = (uint8_t)((IMD_warnings & 0xFF00) >> 8);
//	txData_PTCLog1[3] = (uint8_t)(IMD_warnings & 0x00FF);
//	txData_PTCLog1[4] = (uint8_t)errorStatus;

	// Check HVALs states
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
	{
		txData_PTCLog1[1] += 1;
	}
	if (HAL_COMP_GetOutputLevel(&hcomp1) != 0)
	{
		txData_PTCLog1[1] += 2;
	}
	txData_PTCLog1[2] = (uint8_t)((IMD_warnings & 0xFF00) >> 8);
	txData_PTCLog1[3] = (uint8_t)(IMD_warnings & 0x00FF);
	txData_PTCLog1[4] = (uint8_t)errorStatus;
	txData_PTCLog1[5] = *LV_fail_reason;
	txData_PTCLog1[6] = *HV_fail_reason;

	if (PTC_FDCAN_SendMessage(&hfdcan2, 0x4E3, FDCAN_DLC_BYTES_5, txData_PTCLog1) != HAL_OK)
	{
//		printf("send failed\r\n");
//		FDCAN_ProtocolStatusTypeDef protocolStatus;
//		HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocolStatus);
//		if (protocolStatus.BusOff)
//		{
//			HAL_TIM_Base_Stop_IT(&htim14); // Stop timer for cyclic CAN sends
//			RaiseError(ERROR_FDCAN_FAILED);
//			return;
//		}
	}
	else
	{
//		ClearError(ERROR_FDCAN_FAILED);
	}


	/* PTC Log 2: [Iso res, Iso res, I_bus(mA), I_bus(mA), I_bus(mA), I_bus(mA)*/
	uint8_t txData_PTCLog2[6] = {0};
	txData_PTCLog2[0] = (uint8_t)((isolationResistance_kOhm & 0xFF00) >> 8);
	txData_PTCLog2[1] = (uint8_t)(isolationResistance_kOhm & 0x00FF);
	txData_PTCLog2[2] = (uint8_t)((IVT_Current & 0xFF000000) >> 24); // highest byte
	txData_PTCLog2[3] = (uint8_t)((IVT_Current & 0x00FF0000) >> 16); // second highest byte
	txData_PTCLog2[4] = (uint8_t)((IVT_Current & 0x0000FF00) >> 8); // second lowest byte
	txData_PTCLog2[5] = (uint8_t)(IVT_Current & 0x000000FF); // lowest byte
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E4, FDCAN_DLC_BYTES_6, txData_PTCLog2);

	/* HV BMS LOGS */
	uint16_t HV_vMax = 0;
	uint16_t HV_vMin = 0;
	uint16_t HV_vTot = 0;
	int16_t HV_current = 0;
	getLogValues(*HV_slavesNumber, HV_slaves, HV_modules, &HV_vMax, &HV_vMin, &HV_vTot, &HV_tMin, &HV_tMax, &HV_current);

	/* HV BMS Logging message 1: vMax | vMax | vMin | vMin | tMax | tMax | tMin | tMin */
	uint8_t txData_HVBMSLog1[7] = {0};
	txData_HVBMSLog1[0] = (uint8_t)((HV_vMax & 0xFF00) >> 8);
	txData_HVBMSLog1[1] = (uint8_t)(HV_vMax & 0x00FF);
	txData_HVBMSLog1[2] = (uint8_t)((HV_vMin & 0xFF00) >> 8);
	txData_HVBMSLog1[3] = (uint8_t)(HV_vMin & 0x00FF);
	txData_HVBMSLog1[4] = (uint8_t)((HV_tMax & 0xFF00) >> 8);
	txData_HVBMSLog1[5] = (uint8_t)(HV_tMax & 0x00FF);
	txData_HVBMSLog1[6] = (uint8_t)((HV_tMin & 0xFF00) >> 8);
	txData_HVBMSLog1[7] = (uint8_t)(HV_tMin & 0x00FF);
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E5, FDCAN_DLC_BYTES_8, txData_HVBMSLog1);

	/* HV BMS Logging message 2: vTot | vTot | I_pack | I_pack | V_DClink | V_DClink | V_DCLink | V_DCLink */
	uint8_t txData_HVBMSLog2[7] = {0};
	txData_HVBMSLog2[0] = (uint8_t)((HV_vTot & 0xFF00) >> 8);
	txData_HVBMSLog2[1] = (uint8_t)(HV_vTot & 0x00FF);
	txData_HVBMSLog2[2] = (uint8_t)((HV_current & 0xFF00) >> 8);
	txData_HVBMSLog2[3] = (uint8_t)(HV_current & 0x00FF);
	txData_HVBMSLog2[4] = (uint8_t)((vDCLink & 0xFF000000) >> 24); // highest byte
	txData_HVBMSLog2[5] = (uint8_t)((vDCLink & 0x00FF0000) >> 16); // second highest byte
	txData_HVBMSLog2[6] = (uint8_t)((vDCLink & 0x0000FF00) >> 8); // second lowest byte
	txData_HVBMSLog2[7] = (uint8_t)(vDCLink & 0x000000FF); // lowest byte
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E6, FDCAN_DLC_BYTES_8, txData_HVBMSLog2);

	/* LV BMS LOGGING */
	uint16_t LV_vMax = 0;
	uint16_t LV_vMin = 0;
	uint16_t LV_tMax = 0;
	uint16_t LV_tMin = 0;
	uint16_t LV_vTot = 0;
	int16_t LV_current = 0;
	getLogValues(*LV_slavesNumber, LV_slaves, LV_modules, &LV_vMax, &LV_vMin, &LV_vTot, &LV_tMin, &LV_tMax, &LV_current);

	/* LV BMS Logging message 1: vMax | vMax | vMin | vMin | tMax | tMax | tMin | tMin */
	uint8_t txData_LVBMSLog1[7] = {0};
	txData_LVBMSLog1[0] = (uint8_t)((LV_vMax & 0xFF00) >> 8);
	txData_LVBMSLog1[1] = (uint8_t)(LV_vMax & 0x00FF);
	txData_LVBMSLog1[2] = (uint8_t)((LV_vMin & 0xFF00) >> 8);
	txData_LVBMSLog1[3] = (uint8_t)(LV_vMin & 0x00FF);
	txData_LVBMSLog1[4] = (uint8_t)((LV_tMax & 0xFF00) >> 8);
	txData_LVBMSLog1[5] = (uint8_t)(LV_tMax & 0x00FF);
	txData_LVBMSLog1[6] = (uint8_t)((LV_tMin & 0xFF00) >> 8);
	txData_LVBMSLog1[7] = (uint8_t)(LV_tMin & 0x00FF);
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E7, FDCAN_DLC_BYTES_8, txData_LVBMSLog1);

	/* LV BMS Logging message 2: vTot | vTot | I_pack | I_pack */
	uint8_t txData_LVBMSLog2[4] = {0};
	txData_LVBMSLog2[0] = (uint8_t)((LV_vTot & 0xFF00) >> 8);
	txData_LVBMSLog2[1] = (uint8_t)(LV_vTot & 0x00FF);
	txData_LVBMSLog2[2] = (uint8_t)((LV_current & 0xFF00) >> 8);
	txData_LVBMSLog2[3] = (uint8_t)(LV_current & 0x00FF);
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E8, FDCAN_DLC_BYTES_4, txData_LVBMSLog2);
}


void IMD_InfoGeneralHandler(uint8_t *RxData)
{
	// Decode IMD info
	isolationResistance_kOhm = RxData1[0] | (RxData1[1] << 8);
	isolationStatus_kOhm = RxData1[2];
	IMD_measurementCounter = RxData1[3];
	IMD_warnings =  RxData1[4] | (RxData1[5] << 8);
	IMD_deviceActivity = RxData1[6];
//	printf("%u\r\n", IMD_warnings);

	// Check if any error is raised by the IMD
	// TODO implement specific error checks
	if (IMD_warnings != 0x0000)
	{
		RaiseError(ERROR_IMD_TRIGGERED);
	}
	else
	{
		ClearError(ERROR_IMD_TRIGGERED);
	}


//	printf("ISO175 General Info:\r\n");
//	printf("	Isolation Resistance: %d kOhm\r\n", isolationResistance_kOhm);
//	printf("    Isolation Status: 0x%02X\r\n", isolationStatus_kOhm);
//	printf("  	Measurement Counter: %d\r\n", IMD_measurementCounter);
//	printf("  	Warnings: 0x%04X\r\n", IMD_warnings);
//	printf("  	Device Activity: 0x%02X\r\n", IMD_deviceActivity);
}


/**
  * @brief Sends CAN command for IVT-S
  * @param data: pointer to command message data array
  */
void IVT_SendCommand(FDCAN_HandleTypeDef *hfdcan, const uint8_t *data)
{
	if (PTC_FDCAN_SendMessage(hfdcan, IVT_COMMAND_CANID, FDCAN_DLC_BYTES_8, data) == HAL_OK)
	{
		printf("Command sent\r\n");
	}

	else
	{
		printf("Failed to send command.\r\n");
	}
}

/**
 * @brief Sets configuration for IVT results.
 * 	Change the constants in 'PTC_FDCAN.h' for different configurations.
 */

void IVT_SetConfiguration(void)
{

	uint32_t startTime;
	// Configure IVT. Take response delays into account, if response time is too long --> raise CAN error
	IVT_SendCommand(&hfdcan1, IVT_STOP_CMD); // Stop measurement to configure results
	startTime = HAL_GetTick();
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_CONFIG_CURRENT_CMD); // Configure current result command
	startTime = HAL_GetTick();
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_CONFIG_U1_CMD); // Configure voltage U1 command
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_CONFIG_U2_CMD); // Configure voltage U2 command
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_CONFIG_U3_CMD); // Configure voltage U3 command
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_STORE_CMD); // Store config results command
	while(!(IVT_commandReceivedFlag && ((HAL_GetTick() - startTime) >= 2)))
	{
		if ((HAL_GetTick() - startTime) > 1000)
		{
			RaiseError(ERROR_FDCAN_FAILED);
			return;
		}
	}
	IVT_commandReceivedFlag = 0; // Reset flag

	IVT_SendCommand(&hfdcan1, IVT_START_CMD); // Start command
	IVT_commandReceivedFlag = 0;
	printf("IVT-S config success\r\n");
}
