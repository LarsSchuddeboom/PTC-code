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

#include "PTC_FDCAN.h"
#include "PTC_FSM.h"
#include "BMS_funcs.h"
#include "BMS_ADBMS1818.h"

/* Variables -------------------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;	// CAN module for PT bus
extern FDCAN_HandleTypeDef hfdcan2; // CAN module for General bus
extern uint8_t prechargeTriggerOK;
extern volatile uint8_t errorStatus;

// Variables for BMS Logs
extern uint8_t slavesNumber;
extern cell_asic *slaves;
extern BMSmodule *modules;
extern uint8_t ISenseSlave;

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
uint8_t RxData2[8];


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
	sFilterConfig.FilterID1    = 0x193; // PT Control CAN ID
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
//			  printf("IVT-S Current Data: %ld mA\r\n", IVT_Current);
			  break;

		  case IVT_RESULTU1_CANID: // IVT-S Voltage data (U1): Data Byte 0 (DB0) = 0x01
			  // Voltage U1 data is stored in bytes 2-5
			  vDCLink = ((RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5]);

//			  printf("V_DCLink: %ld mV\r\n", vDCLink);
			  break;

		  case IVT_RESULTU2_CANID: // IVT-S Voltage data (U2): Data Byte 0 (DB0) = 0x02
			  // Voltage U2 data is stored in bytes 2-5
			  vBatt = (RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5];

			  // If the state is Precharge,
			  if ( currentState == PreCharge_State
				   && (prechargeTriggerOK < 3))
			  {
				  prechargeTriggerOK++;
			  }
//			  printf("V_batt: %ld mV\r\n", vBatt);
			  break;

		  case IVT_RESULTU3_CANID: // IVT-S Voltage data (U3): Data Byte 0 (DB0) = 0x03
			  // Voltage U3 data is stored in bytes 2-5
			  voltageU3_mV = (RxData1[2] << 24) | (RxData1[3] << 16) | (RxData1[4] << 8) | RxData1[5];
//			  printf("IVT-S Voltage Data (U3): %ld mV\r\n", voltageU3_mV);
			  break;

		  case IMD_INFO_CANID: // ISO175 General Info
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
			uint8_t PTC_controlCode = RxData2[7];
			PTC_ControlCodeHandler(PTC_controlCode); // Handle received control code
			printf("TEST!: %02X\r\n", PTC_controlCode);
			break;

		case 0x193: // PT control ID: Control code is in byte 0
			// TODO Handling for any extra messages Sensecon wants to send
			printf("CLEARED PRECHARGE FAILURE\r\n");
			ClearError(ERROR_TIMER_FAILURE);
			break;

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
		if (currentState == HV_ON)
		{
			currentState = Discharge_State;
		}
		break;

	case 0x03: // Precharge reset
		if (errorStatus == ERROR_TIMER_FAILURE)
		{
			ClearError(ERROR_TIMER_FAILURE);
		}
		break;

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
	// TODO handling for if SendMessage fails
	uint8_t txData_PTCLog[2] = {0};
	txData_PTCLog[0] = (uint8_t)currentState;
//	GPIO_PinState HVALR_state = HAL_GPIO_ReadPin(GPIOB, HVAL_RED);
//	GPIO_PinState HVALG_state = HAL_GPIO_ReadPin(GPIOB, HVAL_GREEN);
//	txData_PTCLog[1] = (HVALR_state + HVALG_state);
	if (PTC_FDCAN_SendMessage(&hfdcan2, 0x4E3, FDCAN_DLC_BYTES_2, txData_PTCLog) != HAL_OK)
	{
//		printf("send failed\r\n");
	}
	else
	{
//		printf("SEND SUCCESS\r\n");
	}

	/* Get BMS data */
	uint8_t txData_BMSLog1[8] = {0};
	uint16_t vMax = 0;
	uint16_t vMin = 0;
	uint16_t vTot = 0;
	uint16_t tMin = 0;
	uint16_t tMax = 0;
	int16_t current = 0;
//	int16_t vDCLink_2B = (int16_t)(vDCLink / 2);
	getLogValues(slavesNumber, slaves, modules, ISenseSlave, &vMax, &vMin, &vTot, &tMin, &tMax, &current);

//	printf("HVB BMS Data:
//			  	  - slavesNumber = %u
//			  	  - slaves       = %d
//			  	  - modules      = %d
//			  	  - ISenseSlave  = %d
//			  	  - V_highest    = %u
//			  	  - V_lowest     = %u
//			  	  - V_pack       = %u
//	              - I_pack       = %u
//	              - V_DCLink     = %ld
//			  	  - tMin         = %u
//			      - tMax         = %u",
//	        slavesNumber, slaves, modules, ISenseSlave, vMax, vMin, vTot, current, vDCLink, tMin, tMax)
//	printf("\r\n\r\n")
	/* BMS Logging message 1: vMax | vMax | vMin | vMin | tMax | tMax | tMin | tMin */
	txData_BMSLog1[0] = (uint8_t)((vMax & 0xFF00) >> 8);
	txData_BMSLog1[1] = (uint8_t)(vMax & 0x00FF);
	txData_BMSLog1[2] = (uint8_t)((vMin & 0xFF00) >> 8);
	txData_BMSLog1[3] = (uint8_t)(vMin & 0x00FF);
	txData_BMSLog1[4] = (uint8_t)((tMax & 0xFF00) >> 8);
	txData_BMSLog1[5] = (uint8_t)(tMax & 0x00FF);
	txData_BMSLog1[6] = (uint8_t)((tMin & 0xFF00) >> 8);
	txData_BMSLog1[7] = (uint8_t)(tMin & 0x00FF);
	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E4, FDCAN_DLC_BYTES_8, txData_BMSLog1);

	/* BMS Logging message 2: vTot | vTot | I_pack | I_pack | V_DClink | V_DClink */
	uint8_t txData_BMSLog2[7] = {0};
	txData_BMSLog2[0] = (uint8_t)((vTot & 0xFF00) >> 8);
	txData_BMSLog2[1] = (uint8_t)(vTot & 0x00FF);
	txData_BMSLog2[2] = (uint8_t)((current & 0xFF00) >> 8);
	txData_BMSLog2[3] = (uint8_t)(vTot & 0x00FF);
	txData_BMSLog2[4] = (uint8_t)((vDCLink & 0xFF000000) >> 24); // highest byte
	txData_BMSLog2[5] = (uint8_t)((vDCLink & 0x00FF0000) >> 16); // second highest byte
	txData_BMSLog2[6] = (uint8_t)((vDCLink & 0x0000FF00) >> 8); // second lowest byte
	txData_BMSLog2[7] = (uint8_t)(vDCLink & 0x000000FF); // lowest byte

	PTC_FDCAN_SendMessage(&hfdcan2, 0x4E5, FDCAN_DLC_BYTES_6, txData_BMSLog2);
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
}
