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
#define state_OW_wait_state 8
#define state_OW_conv 9




static void init_state();
static void wait_state();
static void read_start_state();
static void form_pack_state();

static void fail_state();
static void conv_to_state();
static void OW_conv_state();
static void OW_wait_state();


uint8_t HV_BMS_FSM_update();
uint8_t LV_BMS_FSM_update();



#endif

