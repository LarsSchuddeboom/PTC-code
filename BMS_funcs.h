// The header file for BMS functions and structs


#ifndef __BMSFU__
#define __BMSFU__

#include "BMS_ADBMS1818.h"

#define maxTrials 3

void hvRelayOpen();
void hvRelayRelease();
void commFail();


typedef struct {
    uint8_t slavesCount;
    uint16_t OVPT;
    uint16_t UVPT;
    int32_t current_offset;
    int32_t OCP_d_th;
    int32_t OCP_c_th;
    uint16_t OT_th;
    uint16_t UT_th;
} BMSconfig;

typedef struct
{
	uint8_t cellNumber;
	uint8_t tempNumber;
	uint8_t chainPos;

	uint16_t CV[18];
	uint16_t SOC[18];
	uint16_t temperature[16];
	int32_t current;

} BMSmodule;



uint8_t checkOvpUvp(uint8_t total_ic, cell_asic *ic, BMSmodule *modules);

uint8_t findISenseSlave(uint8_t total_ic, cell_asic *ic);

uint8_t countSlaves(uint8_t expected, cell_asic *slaves);

void setNTC(uint8_t ntc, uint8_t total_ic, cell_asic *ic);

uint8_t verifyConfig();

void getLogValues(uint8_t total_ic, cell_asic *ic, BMSmodule *modules, uint8_t ISenseSlave, uint16_t *vMax, uint16_t *vMin, uint16_t *vTot, uint16_t *tMin, uint16_t *tMax, int16_t *current);

extern uint8_t failed_isospi;




#define tryComm(f)\
    for(int i = 0; i < maxTrials; i++){\
        if(f == 0){break;}\
        if(i == (maxTrials - 1)) {\
            commFail();\
        }\
    }

#endif


