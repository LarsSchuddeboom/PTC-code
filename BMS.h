// The header file for BMS functions and structs


#ifndef __BMSH__
#define __BMSH__

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "BMS_ADBMS1818.h"
#include "BMS_funcs.h"
#include "BMS_soc.h"


#define state_wait 1
#define state_CAN_tx 2
#define state_read_start 3
#define state_form_pack 4
#define state_init 5

#define state_fail 6
#define state_conv_to 7



void init_state();
void wait_state();
void read_start_state();
void form_pack_state();

void fail_state();
void conv_to_state();


uint8_t BMS_FSM_update();




#endif
