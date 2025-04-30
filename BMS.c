// BMS FSM DH09. Nils Kauffmann
// TODO uitleg

#include "BMS.h"
#include "BMS_soc.h"


//#define BSM_uart_debug

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5; // DO NOT USE HERE
extern TIM_HandleTypeDef htim13; // OCP



extern UART_HandleTypeDef huart3;
extern FDCAN_HandleTypeDef hfdcan1;

uint8_t BMSstate = state_init;

BMSconfig bmsconf;
uint8_t slavesNumber;
cell_asic *slaves;
uint8_t ISenseSlave;

BMSmodule *modules;

uint8_t ntc = 0;

bool CAN_tx = false; // Packet ready

bool ovpuvp_prot = false;
bool ovpuvp_cooldown = false;

bool otp_prot = false;
bool ocp_prot = false;

uint16_t failed_can = 0;
uint8_t failed_isospi = 0;
uint8_t to_events = 0;

int64_t coulomb_count;

uint8_t fail_reason = 0;
uint8_t prot_state = 0;


//E.19 !!!!!!!!!!
//E.130 ID !!!!!!



/*
E.19
It must be possible to display all cell voltages and measured temperatures, e.g.
by connecting a laptop.
*/


// TODO:
/*
	- Balanceren
    - WDT
*/


/*
Timer 1 TIM1 used to detect timeout
Sysclock = 64MHz
Prescale = 64; 1MHz
Timeout = 20ms = 20000 count
*/

/*
Timer 2 TIM2 used for UVP/OVP 500ms lim
Prescale = 64000; 1kHz
Timeout = 500 count
*/

/*
Timer 3 TIM3 used for UVP/OVP cooldown
Prescale = 64000; 1kHz
Timeout = 100 count = 100ms
*/

/*
Timer 4 TIM4 used for OTP 1000ms lim
Prescale = 64000; 1kHz
Timeout = 1000 count = 1000ms
*/

/*
Timer 13 TIM13 used for OCP 500ms lim
Prescale = 64000; 1kHz
Timeout = 500 count
*/

void init_state() {
    hvRelayOpen();

    bmsconf.slavesCount = 1;
    bmsconf.OVPT = 42000;
    bmsconf.UVPT = 25000;
    bmsconf.current_offset = 30264;
    bmsconf.OT_th = 6500; // 60C
    bmsconf.UT_th = 23000; // 0C
    bmsconf.OCP_d_th = -40; // Steps of 20mA
    bmsconf.OCP_c_th = 10;


    coulomb_count = 0;

    slaves = malloc(bmsconf.slavesCount * sizeof(cell_asic));
    modules = malloc(bmsconf.slavesCount * sizeof(BMSmodule));


    // Cells per slave

    modules[0].cellNumber = 7;
    modules[0].tempNumber = 5;



    ADBMS1818_init_reg_limits(bmsconf.slavesCount, slaves); // Prepare structs

    // Count number of slaves in daisy chain
    slavesNumber = countSlaves(bmsconf.slavesCount, slaves);

    if(slavesNumber != bmsconf.slavesCount) {
    	BMSstate = state_fail;
    	fail_reason = 1;
    	return;
    }

    ISenseSlave = findISenseSlave(slavesNumber, slaves);
    if(ISenseSlave == -1) { // Not found
    	BMSstate = state_fail;
    	fail_reason = 2;
    	return;
    }


    // Setup config, put it in the struct and write it to the slaves.
    bool gpio_set[] = {true, true, true, true, true}; // All GPIO high
    bool dcc_set[] = {false, false, false, false, false, false, false, false, false, false, false, false}; // No discharge
    bool dcto_set[] = {true, true, true, true}; // Discharge timeout setting
    bool ps_set[] = {false, false}; // Digtal redundancy setting

    for(uint8_t i = 0; i < slavesNumber; i++) { // Apply to all slaves
		ADBMS1818_set_cfgr(i, slaves, true, false, gpio_set, dcc_set, dcto_set, bmsconf.UVPT, bmsconf.OVPT); // Config A
		ADBMS1818_set_cfgrb(i, slaves, false, false, ps_set, gpio_set, dcc_set); // Config B
    }

    ADBMS1818_wrcfg(slavesNumber, slaves); // Write these settings
    ADBMS1818_wrcfgb(slavesNumber, slaves);


    // Tegen mux fail?
    ADBMS1818_adcvax(MD_7KHZ_3KHZ, 0);
    (void)ADBMS1818_pollAdc();


    ADBMS1818_diagn(); // MUX selftest
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdstat(2, slavesNumber, slaves); // Read result
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].stat.mux_fail[0]){
        	BMSstate = state_fail;
        	fail_reason = 3;
        	return;
        }
    }


    // NB deze waarden zijn anders voor andere snelheden

    // 7khz ST 01  0x9555 p.36 Datasheet
    ADBMS1818_cvst(0b10, 0b01); // cell ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdcv(1, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].cells.c_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 4;
        	return;
        }
    }


    ADBMS1818_axst(0b10, 0b01); // AUX ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdaux(1, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].aux.a_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 5;
        	return;
        }
    }


    ADBMS1818_statst(0b10, 0b01); // STAT ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdstat(1, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].stat.stat_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 6;
        	return;
        }
    }


    // Start all timers and comparators
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim5); // No comparator
	HAL_TIM_Base_Start(&htim13);
	HAL_TIM_OC_Start(&htim13, TIM_CHANNEL_1);

	// Start FSM
    BMSstate = state_wait;
    ADBMS1818_adcvax(MD_7KHZ_3KHZ, 0);

    return;
}


bool wait_counter_on = false;
uint16_t convTime = 0;

void wait_state() {
    // Timer for measurement, don't wait forever
    if(!wait_counter_on) { // Timer is off, turn on
    	wait_counter_on = 1;
        __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset timer
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1);
    } else { // Counter is running and waiting for conversion
        if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1)) { // Has timer ran out?
            // Timeout
        	BMSstate = state_conv_to;
        	wait_counter_on = 0;
        	return;
        }
    }

    uint8_t poll = ADBMS1818_pladc();

    if(poll != 0x00) { // Conversion done
		BMSstate = state_read_start;
		to_events = 0;
		wait_counter_on = 0; // counter done
		convTime = __HAL_TIM_GET_COUNTER(&htim1);
        return;
    }

    // Still waiting
    BMSstate = state_wait;
    return;
}



void read_start_state() {
	(void)ADBMS1818_rdaux(1, slavesNumber, slaves);
	(void)ADBMS1818_rdcv(0, slavesNumber, slaves);
	// Pec return value from function is only of the last slave read


	// Test for PEC errors in read values
	bool pec_fail = 0;
	for(uint8_t i = 0; i < slavesNumber; i++) {
		if(slaves[i].cells.pec_match[0]|slaves[i].cells.pec_match[1]|slaves[i].cells.pec_match[2]|slaves[i].cells.pec_match[3]|slaves[i].cells.pec_match[4]|slaves[i].cells.pec_match[5]) {
			failed_isospi++;
			pec_fail = 1;
			continue;
		}
	}

	if(!pec_fail) { // If voltage did not fail maybe aux did?
		for(uint8_t i = 0; i < slavesNumber; i++) {
			if(slaves[i].aux.pec_match[0]) { // Only GPIO 1 and 2 are used
				failed_isospi++;
				pec_fail = 1;
				continue;
			}
		}
	}

	// Start new measurement anyway
    setNTC((ntc+1)%16, slavesNumber, slaves); // Set the next NTC. The variable is only updated below but this facilitates
    // the processing of the data just received
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 1); // ~10ms


    if(!pec_fail) { // No PEC fail

    	failed_isospi = 0; // Reset counter of consecutive errors

		// Save current. Cell voltages can stay in the struct
		modules[ISenseSlave].current = (int32_t)slaves[ISenseSlave].aux.a_codes[0] - bmsconf.current_offset;


		// Save new temperature measurement
		for(uint8_t i = 0; i < slavesNumber; i++) {
			modules[i].temperature[ntc] = slaves[i].aux.a_codes[1];
		}



		// OVP UVP routine
		// ovpuvp_prot indicates that an event ha been recorded before. A timer has been started.
		// If the timer overflows, the contacts are opened. If no detection is made, but the bit
		// is high, a cooldown timer is started before resetting the first timer. In this way this
		// avoids a single glitch from resetting the timer.

		// OVP UVP protection
		uint8_t protRet = checkOvpUvp(slavesNumber, slaves, modules);
		prot_state = (prot_state & 0b11111100) | protRet; // return 0-3; Update present flag, keep others
		if(protRet) { // over/under
			if(!ovpuvp_prot) { // First detection?
				ovpuvp_prot = true; // Start counter for 500ms
				__HAL_TIM_SET_COUNTER(&htim2, 0); // reset counter
				__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);

			} else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1)) { // Timer already running. Ran out?
				// 500ms passed; Fault;
				hvRelayOpen();
				prot_state |= protRet<<4;
			}

		} else if (ovpuvp_prot) { // No present OVP/UVP. Previous OVP/UVP?
			if(!ovpuvp_cooldown) { // Yes. Is the cooldown timer already running?
				__HAL_TIM_SET_COUNTER(&htim3, 0); // No, start it
				__HAL_TIM_CLEAR_FLAG(&htim3, TIM_IT_CC1);
				ovpuvp_cooldown = 1;

			} else if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1)) { // Cooldown finished?
				ovpuvp_prot = false; // Then reset OVP/UVP
				ovpuvp_cooldown = false;
			}
		}



		// Over temperature protection routine. Is any temperature of any sensor above the threshold?
		// Start timer and check for timeout. No cooldown timer for the temperature.

		// OTP
		bool otp_check = 0;
		for(uint8_t i = 0; i < slavesNumber; i++) {
			for(uint8_t j = 0; j < modules[i].tempNumber; j++) {
				if(modules[i].temperature[j] < bmsconf.OT_th) {
					otp_check = true; // Are there any OT?
				} else if(modules[i].temperature[j] > bmsconf.UT_th) {
					otp_check = true; // Are there any UT?
				}
			}
		}

		if(otp_check) {
			prot_state = prot_state | 0b100;
			if(!otp_prot) { // First time?
				otp_prot = true;
				__HAL_TIM_SET_COUNTER(&htim4, 0); // Start counter for 1000ms
				__HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_CC1);
			} else if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_CC1)) { // Fault persisted
				// xTP fault
				hvRelayOpen();
				prot_state |= 0b100<<4;
			}
		} else { // No xTP
			otp_prot = false;
			prot_state = prot_state & 0b11111011;
		}





		// Over current protection routine.
		// moet 500ms tijd krijgen

		if((modules[ISenseSlave].current > bmsconf.OCP_c_th) | (modules[ISenseSlave].current < bmsconf.OCP_d_th)) {
			prot_state = prot_state | 0b1000;
			if(!ocp_prot) { // First time?
				ocp_prot = true;
				__HAL_TIM_SET_COUNTER(&htim13, 0); // Start counter for 1000ms
				__HAL_TIM_CLEAR_FLAG(&htim13, TIM_IT_CC1);
			} else if (__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_CC1)) { // Fault persisted
				// OCP fault
				hvRelayOpen();
				prot_state |= 0b1000<<4;
			}
		} else { // No xTP
			ocp_prot = false;
			prot_state = prot_state & 0b11110111;
		}



		(void)update_soc_fsm(); // A second FSM trying to keep track of the SOC

		ntc = (ntc+1)%16; // This measurement has already been started, but now update this for processing when it has finished.

		if(!(prot_state & 0b00001111)) { // Currently no errors
			if(HAL_GPIO_ReadPin(GPIOG, (uint16_t)1<<3)) { // Is the reset button pressed?
				prot_state = 0;
				hvRelayRelease();
			}
		}

		BMSstate = state_form_pack;

    }

	if(failed_isospi > 3) { // Could have come from OVP UVP
		BMSstate = state_fail;
		fail_reason = 7;
		return;
	}

	BMSstate = state_wait;


    return;
}


//C.7; 500ms update

/*
For every BMS (High-Voltage and Low-Voltage):
– Pack Voltage
– Maximum cell voltage
– Minimum cell voltage
– Maximum cell temperature
– Minimum cell temperature
– Pack current
*/

//

void form_pack_state() {
    // Format nog te kiezen; In overleg met S&C
    // Max 8B per frame

    FDCAN_TxHeaderTypeDef header;
    header.Identifier = 500;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0x00;

    for(uint8_t i =0; i < 4; i++) {
        header.Identifier = 500 + i;
        uint8_t candata[8];
        for(uint8_t j = 0; j < 4; j++) {
            candata[j<<1] = (uint8_t)(modules[0].temperature[j+(i<<2)] >> 8);
            candata[(j<<1)+1] = (uint8_t)modules[0].temperature[j+(i<<2)];
        }
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, candata);
    }

    BMSstate = state_wait;

    return;
}



void fail_state() {
	hvRelayOpen();
	BMSstate = state_fail;
	return;
}




// Conversion timeout.
void conv_to_state() {
	to_events++;
	if (to_events > 3) {
		BMSstate = state_fail;
		fail_reason = 8;
	}
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 1); // ~10ms
    return;
}




uint8_t BMS_FSM_update() {
    switch (BMSstate)
    {
    case state_wait:
        wait_state();
        break;

    case state_read_start:
        read_start_state();
        break;

    case state_form_pack:
        form_pack_state();
        break;

    case state_init:
        init_state();
        break;

    case state_fail:
		fail_state();
		break;

    case state_conv_to:
		conv_to_state();
		break;

    default:
    	hvRelayOpen();
    	BMSstate = state_fail;
        break;
    }

    return BMSstate;
}
