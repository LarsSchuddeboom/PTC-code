// BMS SOC + balancing FSM DH09. Nils Kauffmann
// Charging is positive, discharging negative


// TODO uitleg
// TODO balaneren

#include "BMS_soc.h"


extern uint8_t slavesNumber;
extern cell_asic *slaves;
extern uint8_t ISenseSlave;
extern BMSmodule *modules;
/*
Timer 5 for SOC uses;
Tick of 100us = 6400
No comparator
*/
extern TIM_HandleTypeDef htim5;

// Coulomb count: 10mA * 100us = 1uC; 3P 3000mAh = 11kC = 11G count
extern int64_t coulomb_count;


// FSM state
uint8_t soc_state = state_soc_idle;


// All values in 20mA / 100us increments, equal to resolution

#define idle_to_dis_TH -10
#define idle_to_dis_time 2000

#define idle_to_ch_TH 10
#define idle_to_ch_time 2000


#define dis_to_idle_TH 10 // +- 100mA
#define dis_to_idle_time 2000

#define dis_to_ch_TH 10
#define dis_to_ch_time 2000


#define ch_to_dis_TH -10
#define ch_to_dis_time 2000

#define ch_to_idle_TH 10 // +- 100mA
#define ch_to_idle_time 2000





uint32_t idle_to_dis_count = 0;
uint32_t idle_to_ch_count = 0;

void soc_idle_state() {
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

void soc_dis_state() {
	coulomb_count += (int32_t)__HAL_TIM_GET_COUNTER(&htim5) * modules[ISenseSlave].current;


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

void soc_ch_state() {
	coulomb_count += (int32_t)__HAL_TIM_GET_COUNTER(&htim5) * modules[ISenseSlave].current;

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
	return;
}




uint8_t update_soc_fsm() {
	switch(soc_state) {
		case state_soc_idle:
			soc_idle_state();
			break;

		case state_soc_dis:
			soc_dis_state();
			break;


		case state_soc_ch:
			soc_ch_state();
			break;
	}


	__HAL_TIM_SET_COUNTER(&htim5, 0);
	return soc_state;
}
