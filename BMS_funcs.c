#include "BMS_funcs.h"
#include "main.h"

void hvRelayOpen() {
    // DOE IETS
	HAL_GPIO_WritePin(GPIOE, (uint16_t)1<<14, GPIO_PIN_RESET);
	return;
}

void hvRelayRelease() {
	HAL_GPIO_WritePin(GPIOE, (uint16_t)1<<14, GPIO_PIN_SET);
	return;
}



void commFail() {
    // na max keer nog niet gelukt
	printf("Comm fail\r\n");
}



// In Aux D en Stat B staan OV UV flags
//

uint8_t checkOvpUvp(uint8_t total_ic, cell_asic *ic, BMSmodule *modules){

    ADBMS1818_rdaux(4, total_ic, ic); // read AuxD
    ADBMS1818_rdstat(0, total_ic, ic); // read StatB

    uint8_t ov = 0;
    uint8_t uv = 0;

    for(int i = 0; i < total_ic; i++) {
    	if(ic[i].aux.pec_match[3] || ic[i].stat.pec_match[1]) {
    		// PEC error
			failed_isospi++; // Count
			return 0; // Return no error, but with the cooldown timer this is no problem.
    	}

    	for(uint8_t j = 0; j < modules[i].cellNumber; j++) {
    		if(j < 4) { // 0-3
    			if(ic[i].stat.flags[0] & (0b1 << (2*j))) {
    				uv++;
    			}
    			if(ic[i].stat.flags[0] & (0b10 << (2*j))) {
    			    ov++;
    			}
    		}

    		if(j > 3 && j < 8) { // 4-7
				if(ic[i].stat.flags[1] & (0b1 << (2*j))) {
					uv++;
				}
				if(ic[i].stat.flags[1] & (0b10 << (2*j))) {
					ov++;
				}
			}

    		if(j > 7 && j < 12) { // 8-11
				if(ic[i].stat.flags[2] & (0b1 << (2*j))) {
					uv++;
				}
				if(ic[i].stat.flags[2] & (0b10 << (2*j))) {
					ov++;
				}
			}

    		if(j > 11 && j < 18) { // 12-17
				if(ic[i].aux.a_codes[11] & (0b1 << (2*j))) {
					uv++;
				}
				if(ic[i].aux.a_codes[11] & (0b10 << (2*j))) {
					ov++;
				}
			}
    	}
    }


    if(ov & uv) {
		return 3;
	} else if (ov) {
		return 1;
	} else if (uv) {
		return 2;
	} else {
		return 0;
	}
}

uint8_t findISenseSlave(uint8_t total_ic, cell_asic *ic) {
	ADBMS1818_adcvax(MD_7KHZ_3KHZ, 1);
	(void)ADBMS1818_pollAdc();

	(void)ADBMS1818_rdaux(0, total_ic, ic);


    for(uint8_t i = 0; i < total_ic; i++) {
			if(ic[i].aux.a_codes[0] < 31000 && ic[i].aux.a_codes[0] > 29000) {
				return i;
			}
        return -1;
    }
    return -1;
}


uint8_t countSlaves(uint8_t expected, cell_asic *slaves) {
    for(int i = expected; i > 0; i--) {
        for(int j = 0; j < maxTrials; j++){
            if(ADBMS1818_rdcfg(i, slaves) == 0){return i;} // return 0 = no PEC failure
        }
    }
    return 0;
}


uint8_t verifyConfig() {
    cell_asic readOut[32];


    return 0;

}


void setNTC(uint8_t ntc, uint8_t total_ic, cell_asic *ic) {
    bool ntc_mux[] = {ntc&0b1000, ntc&0b100, ntc&0b10, ntc&0b1};

    for(int i = 0; i < total_ic; i++) {
    	ADBMS1818_set_cfgrb_gpio_b(i, ic, ntc_mux);
    }

    ADBMS1818_wrcfgb(total_ic, ic);

    return;
}

// For C.7
void getLogValues(uint8_t total_ic, cell_asic *ic, BMSmodule *modules, uint8_t ISenseSlave, uint16_t *vMax, uint16_t *vMin, uint16_t *vTot, uint16_t *tMin, uint16_t *tMax, int16_t *current) {
	*vMax = 0;
	*vMin = -1;
	*vTot = 0;
	*tMin = 0;
	*tMax = -1;
	*current = ic[ISenseSlave].aux.a_codes[0];

	uint32_t sum = 0;

	for(uint8_t i = 0; i < total_ic; i++) {

		for(uint8_t cell = 0; cell < modules[i].cellNumber; cell++) {
			if(ic[i].cells.c_codes[cell] > *vMax) {
				*vMax = ic[i].cells.c_codes[cell];
			}
			if(ic[i].cells.c_codes[cell] < *vMin) {
				*vMin = ic[i].cells.c_codes[cell];
			}
			sum += ic[i].cells.c_codes[cell];
		}

		for(uint8_t ntc = 0; ntc < modules[i].tempNumber; ntc++) {
			if(modules[i].temperature[ntc] < *tMax) { // NB, smaller value = higher temperature
				*tMax = modules[i].temperature[ntc];
			}
			if(modules[i].temperature[ntc] > *tMin) { // NB, higher value = lower temperature
				*tMin = modules[i].temperature[ntc];
			}
		}
	}

	// sum can be 18 * 65000
	// BMS for 650V, 6500000
	// To make 16bit work divide by 100
	// Shift seven loses 127 steps, 12,7mV over whole module. Acceptable
	*vTot += (uint16_t)(sum >> 7);

	return;
}
