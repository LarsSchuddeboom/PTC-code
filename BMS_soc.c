// BMS SOC + balancing FSM DH09. Nils Kauffmann
// Charging is positive, discharging negative


// TODO uitleg
// TODO balaneren

#include "BMS_soc.h"



/*
Code gebruite ooit:

extern uint8_t slavesNumber;
extern cell_asic *slaves;
extern uint8_t ISenseSlave;
extern BMSmodule *modules;
extern int64_t coulomb_count;

uint8_t slavesNumber, cell_asic *slaves, int8_t ISenseSlave, BMSmodule *modules, int64_t *coulomb_count
slavesNumber, slaves, ISenseSlave, modules, coulomb_count

Nu als arg naar de SOC functie

*/




/*
Timer 5 for SOC uses;
Tick of 100us = 6400
No comparator
*/
extern TIM_HandleTypeDef htim5;

// Coulomb count: 10mA * 100us = 1uC; 3P 3000mAh = 11kC = 11G count



// FSM state
uint8_t soc_state = state_soc_idle;


// All values in 20mA / 100us increments, equal to resolution

#define idle_to_dis_TH -100
#define idle_to_dis_time 2000

#define idle_to_ch_TH 100
#define idle_to_ch_time 2000


#define dis_to_idle_TH 100 // +- 100mA
#define dis_to_idle_time 2000

#define dis_to_ch_TH 100
#define dis_to_ch_time 2000


#define ch_to_dis_TH -100
#define ch_to_dis_time 2000

#define ch_to_idle_TH 100 // +- 100mA
#define ch_to_idle_time 2000



uint16_t balance_counter = 0;


uint32_t idle_to_dis_count = 0;
uint32_t idle_to_ch_count = 0;

void soc_idle_state(uint8_t slavesNumber, cell_asic *slaves, int8_t ISenseSlave, BMSmodule *modules, int64_t *coulomb_count) {
	if(modules[ISenseSlave].current < idle_to_dis_TH) {
		idle_to_dis_count += __HAL_TIM_GET_COUNTER(&htim5);
		if (idle_to_dis_count > idle_to_dis_time) { // Waited long enough
			soc_state = state_soc_dis; // Now idle
		}
		idle_to_ch_count = 0;
	} else if (modules[ISenseSlave].current > idle_to_ch_TH) {
		idle_to_ch_count += __HAL_TIM_GET_COUNTER(&htim5);
		if (idle_to_ch_count > idle_to_ch_time) { // Waited long enough
			soc_state = state_soc_ch; // Now idle
		}
		idle_to_dis_count = 0;
	} else {
		idle_to_ch_count = 0;
		idle_to_dis_count = 0;
	}

	return;
}





uint32_t dis_to_idle_count = 0;
uint32_t dis_to_ch_count = 0;

void soc_dis_state(uint8_t slavesNumber, cell_asic *slaves, int8_t ISenseSlave, BMSmodule *modules, int64_t *coulomb_count) {
	*coulomb_count += (int32_t)__HAL_TIM_GET_COUNTER(&htim5) * modules[ISenseSlave].current;


	if(modules[ISenseSlave].current < dis_to_idle_TH && modules[ISenseSlave].current > -dis_to_idle_TH) { // Current between +- idleTH
		dis_to_idle_count += __HAL_TIM_GET_COUNTER(&htim5); // Count the time spent. NB TIM5 reset at the end of the loop
		if (dis_to_idle_count > dis_to_idle_time) { // Waited long enough
			soc_state = state_soc_idle; // Now idle
		}
		dis_to_ch_count = 0;

	} else if (modules[ISenseSlave].current > dis_to_ch_TH) { // Now charging
		dis_to_ch_count += __HAL_TIM_GET_COUNTER(&htim5);
		if (dis_to_ch_count > dis_to_ch_time) { // Waited long enough
			soc_state = state_soc_ch; // Now charging
		}
		dis_to_idle_count = 0; // Now charging so reset counter

	} else {
		dis_to_ch_count = 0;
		dis_to_idle_count = 0;
	}
	return;
}





uint32_t ch_to_idle_count = 0;
uint32_t ch_to_dis_count = 0;

void soc_ch_state(uint8_t slavesNumber, cell_asic *slaves, int8_t ISenseSlave, BMSmodule *modules, int64_t *coulomb_count) {
	*coulomb_count += (int32_t)__HAL_TIM_GET_COUNTER(&htim5) * modules[ISenseSlave].current;

	if(modules[ISenseSlave].current < ch_to_idle_TH && modules[ISenseSlave].current > -ch_to_idle_TH) { // Current between +- idleTH
		ch_to_idle_count += __HAL_TIM_GET_COUNTER(&htim5); // Count the time spent. NB TIM5 reset at the end of the loop
		if (ch_to_idle_count > ch_to_idle_time) { // Waited long enough
			soc_state = state_soc_idle; // Now idle
		}
		ch_to_dis_count = 0;

	} else if (modules[ISenseSlave].current < ch_to_dis_TH) { // Now discharging
		ch_to_dis_count += __HAL_TIM_GET_COUNTER(&htim5);
		if (ch_to_dis_count > ch_to_dis_time) { // Waited long enough
			soc_state = state_soc_dis; // Now discharging
		}
		ch_to_idle_count = 0; // Now charging so reset counter

	} else {
		ch_to_idle_count = 0;
		ch_to_dis_count = 0;
	}




	// NB timeout is hier van belang. onder de drempel gaat DCC niet van zelf uit
	if(balance_counter == 500) {
		balance_counter = 0;

		for(uint8_t slave = 0; slave < slavesNumber; slave++) {
			uint16_t maxCell = 0;
			uint16_t minCell = -1;
			uint32_t sumCell = 0;
			for(uint8_t i = 0; i < modules[slave].cellNumber; i++) {
				if(slaves[slave].cells.c_codes[i] > maxCell) {
					maxCell = slaves[slave].cells.c_codes[i];
				}
				if(slaves[slave].cells.c_codes[i] < minCell) {
					minCell = slaves[slave].cells.c_codes[i];
				}
				sumCell += slaves[slave].cells.c_codes[i];
			}

			uint16_t avgCell = sumCell / modules[slave].cellNumber;

			// Start balancing when one of the cells nears the end
			bool dcca[12] = {false, false, false, false, false, false, false, false, false, false, false, false};
			bool dccb[7] = {false, false, false, false, false, false, false};;
			dccb[0] = false; // GPIO9

			if(maxCell > 40000) {
				for(uint8_t i = 0; i < modules[slave].cellNumber; i++) {
					if(slaves[slave].cells.c_codes[i] > (minCell + 100)) { // 10mV above min
						// start balance
						modules[slave].balance[i] = true;
					}

					if(slaves[slave].cells.c_codes[i] < (avgCell - 40)) { // 4mV below mean
						// stop balance
						modules[slave].balance[i] = false;
					}

					if(i < 12) {
						dcca[i] = modules[slave].balance[i];
					} else {
						dccb[i+1] = modules[slave].balance[i];
					}
				}
			} else {
				for(uint8_t i = 0; i < modules[slave].cellNumber; i++) {
					modules[slave].balance[i] = false;
				}
			}

			ADBMS181x_set_cfgr_dis(slave, slaves, dcca);
			ADBMS1818_set_cfgrb_dcc_b(slave, slaves, dccb);
		}

		ADBMS1818_wrcfg(slavesNumber, slaves);
		ADBMS1818_wrcfgb(slavesNumber, slaves);

	} else {
		balance_counter++;
	}


	return;
}




uint8_t update_soc_fsm(uint8_t slavesNumber, cell_asic *slaves, int8_t ISenseSlave, BMSmodule *modules, int64_t *coulomb_count) {
	switch(soc_state) {
		case state_soc_idle:
			soc_idle_state(slavesNumber, slaves, ISenseSlave, modules, coulomb_count);
			break;

		case state_soc_dis:
			soc_dis_state(slavesNumber, slaves, ISenseSlave, modules, coulomb_count);
			break;


		case state_soc_ch:
			soc_ch_state(slavesNumber, slaves, ISenseSlave, modules, coulomb_count);
			break;
	}


	__HAL_TIM_SET_COUNTER(&htim5, 0);
	return soc_state;
}
