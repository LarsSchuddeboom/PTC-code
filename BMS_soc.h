/*
 * BMS_soc.h
 *
 *  Created on: Jan 3, 2025
 *      Author: nils
 */

#ifndef INC_SOC_H_
#define INC_SOC_H_

#include "stdint.h"
#include "BMS_funcs.h"
#include "main.h"


#define state_soc_idle 1
#define state_soc_dis 2
#define state_soc_ch 3

void soc_idle_state();
void soc_dis_state();
void soc_ch_state();

uint8_t update_soc_fsm();


#endif /* INC_SOC_H_ */
