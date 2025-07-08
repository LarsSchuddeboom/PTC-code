// BMS FSM DH09. Nils Kauffmann
// TODO uitleg

#include "BMS.h"
#include "BMS_soc.h"

static uint8_t BMSstate = state_init;

static BMSconfig bmsconf;
static uint8_t slavesNumber;
static cell_asic *slaves;
static uint8_t ISenseSlave;

static BMSmodule *modules;

static uint8_t ntc = 0;
static uint8_t new_ntc;

static bool CAN_tx = false; // Packet ready

static bool ovpuvp_prot = false;
static bool ovpuvp_cooldown = false;

static bool otp_prot = false;
static bool ocp_prot = false;

static uint16_t failed_can = 0;
static uint8_t failed_isospi[3] = {0, 0, 0};
static uint8_t to_events = 0;

static int64_t coulomb_count;

static uint8_t fail_reason = 0;
static uint8_t prot_state = 0;

static uint8_t cv_counter = 0;


// TIMER
static uint32_t old_time[5];
static uint32_t counted_time[5];

#define TO_timer 0
#define OVP_timer 1
#define OVP_CD_timer 2
#define OTP_timer 3
#define OCP_timer 4

#define updateCounter(num) counted_time[num] = (__HAL_TIM_GET_COUNTER(&htim2) - old_time[num]);

#define resetCounter(num) counted_time[num] = 0; \
	old_time[num] = __HAL_TIM_GET_COUNTER(&htim2);



extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
extern FDCAN_HandleTypeDef hfdcan1;



// For external access
uint8_t *LV_slavesNumber = &slavesNumber;
cell_asic *LV_slaves;
BMSmodule *LV_modules;
uint8_t *LV_fail_reason = &fail_reason;
uint8_t *LV_prot_state = &prot_state;



// Initialisation state of the BMS FSM.
void init_state() {

	// Battery configuration
    bmsconf.slavesCount = 1;
    bmsconf.OVPT = 42000;
    bmsconf.UVPT = 25000;
    bmsconf.current_offset = 30090;
    bmsconf.OT_th = 6500; // 60C
    bmsconf.UT_th = 23000; // 0C

    // OCP configuration
    bmsconf.OCP_d_th = -28000; // Voor LV met 100x: 1000b per A
    bmsconf.OCP_c_th = 5000;


    // TODO save and read old SoC
    coulomb_count = 0;

    // Malloc structures
    slaves = malloc(bmsconf.slavesCount * sizeof(cell_asic));
    modules = malloc(bmsconf.slavesCount * sizeof(BMSmodule));


    LV_slaves = slaves;
    LV_modules = modules;



    // Setup of each module
    for (uint8_t i = 0; i < bmsconf.slavesCount; i++) {
		modules[i].cellNumber = 7;
		modules[i].tempNumber = 10;
    }


    // Prepare structs
    ADBMS1818_init_reg_limits(bmsconf.slavesCount, slaves);


    // Count number of slaves in daisy chain
    (void)countSlaves(bmsconf.slavesCount, slaves);
    HAL_Delay(1); // wake time
    slavesNumber = countSlaves(bmsconf.slavesCount, slaves);


    if(slavesNumber != bmsconf.slavesCount) {
    	BMSstate = state_fail;
    	fail_reason = 1;
    	return;
    }

    ISenseSlave = findISenseSlave(slavesNumber, slaves);
    if(ISenseSlave == 255) { // Not found
    	BMSstate = state_fail;
    	fail_reason = 2;
    	return;
    }


    // Setup config, put it in the struct and write it to the slaves.
    bool gpio_set[] = {true, true, true, true, true}; // All GPIO high
    bool dcc_set[] = {false, false, false, false, false, false, false, false, false, false, false, false}; // No discharge
    bool dcto_set[] = {false, false, false, true}; // Discharge timeout setting 1 min
    bool ps_set[] = {false, false}; // Digtal redundancy setting


    for(uint8_t i = 0; i < slavesNumber; i++) { // Apply to all slaves
		ADBMS1818_set_cfgr(i, slaves, true, true, gpio_set, dcc_set, dcto_set, bmsconf.UVPT, bmsconf.OVPT); // Config A
		ADBMS1818_set_cfgrb(i, slaves, false, false, ps_set, gpio_set, dcc_set); // Config B
    }

    ADBMS1818_wrcfg(slavesNumber, slaves); // Write these settings
    ADBMS1818_wrcfgb(slavesNumber, slaves);




    // Tegen mux fail?
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 0); // 5ms
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


    ADBMS1818_adcvax(MD_422HZ_1KHZ, 0); // 5ms
    (void)ADBMS1818_pollAdc();

    // NB deze waarden zijn anders voor andere snelheden


    // 7khz ST 01  0x9555 p.36 Datasheet
    ADBMS1818_cvst(0b00, 0b01); // cell ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdcv(0, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].cells.c_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 4;
        	return;
        }
    }


    ADBMS1818_axst(0b00, 0b01); // AUX ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdaux(0, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].aux.a_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 5;
        	return;
        }
    }


    ADBMS1818_statst(0b00, 0b01); // STAT ADC test
    (void)ADBMS1818_pollAdc(); // Blocking wait
    ADBMS1818_rdstat(0, slavesNumber, slaves);
    for(int i = 0; i < slavesNumber; i++) {
        if(slaves[i].stat.stat_codes[0] != 0x9555){
        	BMSstate = state_fail;
        	fail_reason = 6;
        	return;
        }
    }


    // Start timer

    HAL_TIM_Base_Start(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2 , 0);

    resetCounter(TO_timer)
    resetCounter(OVP_timer)
	resetCounter(OVP_CD_timer)
	resetCounter(OTP_timer)
	resetCounter(OCP_timer)


	// Start FSM
	setNTC(ntc, slavesNumber, slaves);
    BMSstate = state_wait;
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 0); // 5ms

    return;
}




// Wait state. Master polls slaves for end of conversion
//bool wait_counter_on = false;
//uint16_t convTime = 0;

void wait_state() {
    // Timer for measurement, don't wait forever
//    if(!wait_counter_on) { // Timer is off, turn on
//    	wait_counter_on = 1;
//        __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset timer
//        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1);
//    } else { // Counter is running and waiting for conversion
//        if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1)) { // Has timer ran out?
//            // Timeout
//        	BMSstate = state_conv_to;
//        	wait_counter_on = 0;
//        	return;
//        }
//    }

	updateCounter(TO_timer);
	if (counted_time[TO_timer] > 15) {
		BMSstate = state_conv_to;
		return;
	}

	// Converstion complete?
    uint8_t poll = ADBMS1818_pladc();

    if(poll != 0x00) { // Conversion done
		BMSstate = state_read_start;
		to_events = 0;
        return;
    }

    // Still waiting
    BMSstate = state_wait;
    return;
}




// Read converted voltages, start new measurement, process read results
void read_start_state() {
	(void)ADBMS1818_rdaux(1, slavesNumber, slaves);
	(void)ADBMS1818_rdaux(4, slavesNumber, slaves);
	if(cv_counter >= 100) {
		cv_counter = 0;
		(void)ADBMS1818_rdcv(0, slavesNumber, slaves);
	} else {
		cv_counter++;
	}
	(void)ADBMS1818_rdstat(2, slavesNumber, slaves);
	// Pec return value from function is only of the last slave read


	// Test for PEC errors in voltage values, return only was for last byte
	uint8_t pec_fail = 0;
	if(cv_counter == 0) { // just read cell voltages
		for(uint8_t i = 0; i < slavesNumber; i++) {
			if(slaves[i].cells.pec_match[0]|slaves[i].cells.pec_match[1]|slaves[i].cells.pec_match[2]|slaves[i].cells.pec_match[3]|slaves[i].cells.pec_match[4]|slaves[i].cells.pec_match[5]) {
				pec_fail = 0b1;
				failed_isospi[0]++;
				continue;
			}
		}
	}

	// same for temp + current
	for(uint8_t i = 0; i < slavesNumber; i++) {
		if(slaves[i].aux.pec_match[0]) { // Only GPIO 1 and 2 are used
			pec_fail |= 0b10;
			failed_isospi[1]++;
			continue;
		}
	}

	// ovp uvp
	for(uint8_t i = 0; i < slavesNumber; i++) {
		if(slaves[i].aux.pec_match[3] || slaves[i].stat.pec_match[1]) {
			pec_fail |= 0b100;
			failed_isospi[2]++;
			continue;
		}
	}

	if(!(pec_fail & 0b1)) {
		failed_isospi[0] = 0;
	}
	if(!(pec_fail & 0b10)) {
		failed_isospi[1] = 0;
	}
	if(!(pec_fail & 0b100)) {
		failed_isospi[2] = 0;
	}


	// Start new measurement in any case
	new_ntc = (ntc+1)%16;
    setNTC(new_ntc, slavesNumber, slaves); // Set the next NTC. The variable is only updated below but this facilitates processing of the previous NTC
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 0); // ~5ms
    resetCounter(TO_timer); // Reset timeout timer




    if(!(pec_fail & 0b10)) {
		// Save current. Cell voltages can stay in the struct
    	for(uint8_t i = 0; i < slavesNumber; i++) {
    		modules[i].current = (int32_t)slaves[ISenseSlave].aux.a_codes[0] - bmsconf.current_offset;
    	}

		// Save new temperature measurement
		for(uint8_t i = 0; i < slavesNumber; i++) {
			modules[i].temperature[ntc] = slaves[i].aux.a_codes[1];
		}
    }




	// OVP UVP routine
    if(!(pec_fail & 0b100)) {
		// ovpuvp_prot indicates that an event ha been recorded before. A timer has been started.
		// If the timer overflows, the contacts are opened. If no detection is made, but the bit
		// is high, a cooldown timer is started before resetting the first timer. In this way this
		// avoids a single glitch from resetting the timer.

		// OVP UVP protection
		uint8_t protRet = checkOvpUvp(slavesNumber, slaves, modules);
		prot_state = (prot_state & 0b11111100) | protRet; // return 0-3; Update present flag, keep others

		if(protRet) { // over/under
			updateCounter(OVP_timer);
			ovpuvp_cooldown = false;

			if(!ovpuvp_prot) { // First detection?
				ovpuvp_prot = true; // Start counter for 500ms
				resetCounter(OVP_timer);

			} else if (counted_time[OVP_timer] > 500) { // Timer already running. Ran out?
				// 500ms passed; Fault;
				lvRelayOpen();
				prot_state |= protRet<<4;
			}

		} else if (ovpuvp_prot) { // No present OVP/UVP. Previous OVP/UVP?
			updateCounter(OVP_CD_timer);

			if(!ovpuvp_cooldown) { // Yes previous. Is the cooldown timer already running?
				// No cooldown yet
				ovpuvp_cooldown = true;
				resetCounter(OVP_CD_timer);

			} else if (counted_time[OVP_CD_timer] > 100) { // Cooldown finished?
				ovpuvp_prot = false; // Then reset OVP/UVP
				ovpuvp_cooldown = false;
			}
		}
    }




    // OTP OCP
    if(!(pec_fail & 0b10)) {
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
			updateCounter(OTP_timer);

			if(!otp_prot) { // First time?
				otp_prot = true;
				resetCounter(OTP_timer);

			} else if (counted_time[OTP_timer] > 1000) { // Fault persisted
				// xTP fault
				lvRelayOpen();
				prot_state |= 0b100<<4;
			}

		} else { // No xTP No cooldown here
			otp_prot = false;
			prot_state = prot_state & 0b11111011;
		}


		// Overcurrent protection routine.
		// moet 500ms tijd krijgen

		if((modules[ISenseSlave].current > bmsconf.OCP_c_th) | (modules[ISenseSlave].current < bmsconf.OCP_d_th)) {
			prot_state = prot_state | 0b1000;
			updateCounter(OCP_timer);

			if(!ocp_prot) { // First time?
				ocp_prot = true;
				resetCounter(OCP_timer);

			} else if (counted_time[OCP_timer] > 500) { // Fault persisted
				// OCP fault
				lvRelayOpen();
				prot_state |= 0b1000<<4;
			}

		} else { // No xTP
			ocp_prot = false;
			prot_state = prot_state & 0b11110111;
		}
    }


	//(void)update_soc_fsm(slavesNumber, slaves, ISenseSlave, modules, &coulomb_count); // A second FSM trying to keep track of the SOC


	if(!(prot_state & 0b00001111)) { // There was a latched error, but currently no errors
//			prot_state = 0;
			lvRelayRelease();
	}


    ntc = new_ntc; // update NTC, conversion started before

	if((failed_isospi[0] > 5) | (failed_isospi[1] > 5) | (failed_isospi[2] > 5)) { // Could have come from OVP UVP
		BMSstate = state_fail;
		fail_reason = 7;
	} else {
		BMSstate = state_wait;
	}


    return;
}


void form_pack_state() {

    BMSstate = state_wait;

    return;
}



void fail_state() {
	lvRelayOpen();
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

	resetCounter(TO_timer);
    ADBMS1818_adcvax(MD_422HZ_1KHZ, 0); // ~5ms
    return;
}




uint8_t LV_BMS_FSM_update() {
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
    	lvRelayOpen();
    	BMSstate = state_fail;
        break;
    }


    return BMSstate;
}
