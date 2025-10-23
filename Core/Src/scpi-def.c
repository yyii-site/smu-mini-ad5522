/*-
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2018, Jan Breuer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi-def.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  SCPI parser test
 *
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

#include "math.h"
#include "cmsis_os.h"
#include "scpi/types.h"
#include "semphr.h"
#include "ad5522.h"


extern uint32_t ad7190_val[4];
extern handle_AD5522 h_PMU;
extern void AD5522_in(void);
extern void AD5522_out(void);

static uint8_t current2Range(double value) {
    if (value <= 5e-6) {
        return PMU_DAC_SCALEID_5UA;
    } else if (value <= 20e-6) {
        return PMU_DAC_SCALEID_20UA;
    }  else if (value <= 200e-6) {
        return PMU_DAC_SCALEID_200UA;
    }  else if (value <= 2e-3) {
        return PMU_DAC_SCALEID_2MA;
    }  else if (value <= 80e-3) {
        return PMU_DAC_SCALEID_EXT;
    }  else {
        printf("Current set out of range\r\n");
        return PMU_DAC_SCALEID_EXT;
    }
}

static double range2Current(uint32_t IscaleID) {
	switch(IscaleID)
	{
		case PMU_DAC_SCALEID_5UA:
			return 5e-6;
		case PMU_DAC_SCALEID_20UA:
			return 20e-6;
		case PMU_DAC_SCALEID_200UA:
			return 200e-6;
		case PMU_DAC_SCALEID_2MA:
			return 2e-3;
		case PMU_DAC_SCALEID_EXT:
			return 80e-3;
	}
    printf("Current out of range\r\n");
    return 20e-6;
}

static uint32_t chToChannel(uint8_t ch_i) {
    if (ch_i==0) {
        return PMU_CH_0;
    } else if (ch_i==1) {
        return PMU_CH_1;
    } else if (ch_i==2) {
        return PMU_CH_2;
    } else if (ch_i==3) {
        return PMU_CH_3;
    }
    return PMU_CH_0;
}

static scpi_result_t smu_setFunction(uint8_t ch_i, scpi_t * context) {
    char buffer[64];
    size_t copy_len;
    double param2;
    uint32_t i_range;
    uint32_t channel = chToChannel(ch_i);

    // 模式
    if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, TRUE)) {
        buffer[0] = '\0';
    }

     /* read second paraeter if present */
    if (!SCPI_ParamDouble(context, &param2, FALSE)) {
        i_range = h_PMU.i_range[ch_i];
    } else {
        i_range = current2Range(param2);
    }

    printf("ch0:func ***%s, %d***\r\n", buffer, i_range);

    if (strstr(buffer, "HIZMV")) {
        printf("HIZMV\n");
        AD5522_in();
        AD5522_StartHiZMV(&h_PMU, channel);
        AD5522_out();
    }
    else if (strstr(buffer, "FVMI")) {
        printf("FVMI\n");
        AD5522_in();
        AD5522_StartFVMI(&h_PMU, channel, i_range);
        AD5522_out();
    }
    else if (strstr(buffer, "FIMV")) {
        printf("FIMV\n");
        AD5522_in();
        AD5522_StartFIMV(&h_PMU, channel, i_range);
        AD5522_out();
    }
    else if (strstr(buffer, "FVMV")) {
        printf("FVMV\n");
        AD5522_in();
        AD5522_StartFVMV(&h_PMU, channel, i_range);
        AD5522_out();
    }
    else if (strstr(buffer, "FIMI")) {
        printf("FIMI\n");
        AD5522_in();
        AD5522_StartFIMI(&h_PMU, channel, i_range);
        AD5522_out();
    }
    else {
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

//:CHANnel0:FUNCtion "FVMI",0.01
static scpi_result_t SMU_CHANnel0Function(scpi_t * context) {
    return smu_setFunction(0, context);
}
static scpi_result_t SMU_CHANnel1Function(scpi_t * context) {
    return smu_setFunction(1, context);
}
static scpi_result_t SMU_CHANnel2Function(scpi_t * context) {
    return smu_setFunction(2, context);
}
static scpi_result_t SMU_CHANnel3Function(scpi_t * context) {
    return smu_setFunction(3, context);
}

static void readSmuFunction(uint8_t ch_i, char *mode, uint8_t modeSize) {
    char force[5] = {0};
    char measure[5] = {0};

    if (ch_i >= 4) {
        return;
    }

    uint32_t pmu = h_PMU.reg_pmu[ch_i];
    
    uint32_t buf = pmu&(0x11<<19);
    if (buf == PMU_PMUREG_FVCI) {
        strcpy(force, "FV");
    } else if (buf == PMU_PMUREG_FICV) {
        strcpy(force, "FI");
    } else if (buf == PMU_PMUREG_HZV) {
        strcpy(force, "HZV");
    } else if (buf == PMU_PMUREG_HZI){
        strcpy(force, "HZI");
    };

    buf = pmu&(0x11<<13);
    if (buf == PMU_PMUREG_MEAS_I) {
        strcpy(measure, "MI");
    } else if (buf == PMU_PMUREG_MEAS_V) {
        strcpy(measure, "MV");
    } else if (buf == PMU_PMUREG_MEAS_TEMP) {
        strcpy(measure, "MT");
    } else if (buf == PMU_PMUREG_MEAS_HZ){
        strcpy(measure, "MHZ");
    };

    snprintf(mode, modeSize, "%s%s", force, measure);
}

static scpi_result_t sum_getFunction(uint8_t ch_i, scpi_t *context) {
    printf("ch%d:func?\r\n", ch_i);
    char model[10] = {0};
    readSmuFunction(ch_i, model, sizeof(model));
    SCPI_ResultText(context, model);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0FunctionQ(scpi_t * context) {
    return sum_getFunction(0, context);
}
static scpi_result_t SMU_CHANnel1FunctionQ(scpi_t * context) {
    return sum_getFunction(1, context);
}
static scpi_result_t SMU_CHANnel2FunctionQ(scpi_t * context) {
    return sum_getFunction(2, context);
}
static scpi_result_t SMU_CHANnel3FunctionQ(scpi_t * context) {
    return sum_getFunction(3, context);
}

static scpi_result_t smu_getCurrentRange(uint8_t ch_i, scpi_t * context) {
    printf("ch%d:Curr:Range?\r\n", ch_i);

    double range = range2Current(h_PMU.i_range[ch_i]);
    SCPI_ResultDouble(context, range);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentRangeQ(scpi_t * context) {
    return smu_getCurrentRange(0, context);
}
static scpi_result_t SMU_CHANnel1CurrentRangeQ(scpi_t * context) {
    return smu_getCurrentRange(1, context);
}
static scpi_result_t SMU_CHANnel2CurrentRangeQ(scpi_t * context) {
    return smu_getCurrentRange(2, context);
}
static scpi_result_t SMU_CHANnel3CurrentRangeQ(scpi_t * context) {
    return smu_getCurrentRange(3, context);
}

static scpi_result_t smu_setVoltageLevel(uint8_t ch_i, scpi_t * context) {
    uint32_t channel = chToChannel(ch_i);
    double param1;
    printf("ch%d:Vol:Level\r\n", ch_i);

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    if (param1 > 11.5) {
        printf("CHANnel%dVoltage out of range\r\n", ch_i);
        return SCPI_RES_ERR;
    } else if (param1 < -11.5) {
        printf("CHANnel%dVoltage out of range\r\n", ch_i);
        return SCPI_RES_ERR;
    } else {
        AD5522_in();
        AD5522_SetOutputVoltage_float(&h_PMU, channel, param1);
        AD5522_out();
    }

    return SCPI_RES_OK;
}

static scpi_result_t smu_getVoltageLevel(uint8_t ch_i, scpi_t * context) {
    printf("ch%d:Vol:Level?\r\n", ch_i);
    uint32_t V_level = h_PMU.reg_DAC_FIN_V[ch_i][AD5522_DAC_REG_X1];
    float vref = h_PMU.vref;
    double Vlevel = (V_level-32768.0)/pow(2,16)*vref*4.5;

    SCPI_ResultDouble(context, Vlevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageLevel(scpi_t * context) {
    return smu_setVoltageLevel(0, context);
}
static scpi_result_t SMU_CHANnel1VoltageLevel(scpi_t * context) {
    return smu_setVoltageLevel(1, context);
}
static scpi_result_t SMU_CHANnel2VoltageLevel(scpi_t * context) {
    return smu_setVoltageLevel(2, context);
}
static scpi_result_t SMU_CHANnel3VoltageLevel(scpi_t * context) {
    return smu_setVoltageLevel(3, context);
}

static scpi_result_t SMU_CHANnel0VoltageLevelQ(scpi_t * context) {
    return smu_getVoltageLevel(0, context);
}
static scpi_result_t SMU_CHANnel1VoltageLevelQ(scpi_t * context) {
    return smu_getVoltageLevel(1, context);
}
static scpi_result_t SMU_CHANnel2VoltageLevelQ(scpi_t * context) {
    return smu_getVoltageLevel(2, context);
}
static scpi_result_t SMU_CHANnel3VoltageLevelQ(scpi_t * context) {
    return smu_getVoltageLevel(3, context);
}

static scpi_result_t smu_setCurrentLevel(uint8_t ch_i, scpi_t * context) {
    uint32_t channel = chToChannel(ch_i);
    double param1;
    printf("ch%d:Curr:Level\r\n", ch_i);

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    if (param1 < 0.08)
    {
        AD5522_in();
        AD5522_SetOutputCurrent_float(&h_PMU, channel, param1);
        AD5522_out();
    }

    return SCPI_RES_OK;
}

static scpi_result_t smu_getCurrentLevel(uint8_t ch_i, scpi_t * context) {
    printf("ch%d:Curr:Level?\r\n", ch_i);

    uint32_t I_level = h_PMU.reg_DAC_FIN_I[ch_i][h_PMU.i_range[ch_i]][AD5522_DAC_REG_X1];
    float MI_gain = 10;
    float vref = h_PMU.vref;
    float Rsense = h_PMU.Rsense[ch_i];
    double Ilevel = 4.5 * vref * ((I_level - 32768.0)/pow(2,16))/(Rsense*MI_gain);

    SCPI_ResultDouble(context, Ilevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentLevel(scpi_t * context) {
    return smu_setCurrentLevel(0, context);
}
static scpi_result_t SMU_CHANnel1CurrentLevel(scpi_t * context) {
    return smu_setCurrentLevel(1, context);
}
static scpi_result_t SMU_CHANnel2CurrentLevel(scpi_t * context) {
    return smu_setCurrentLevel(2, context);
}
static scpi_result_t SMU_CHANnel3CurrentLevel(scpi_t * context) {
    return smu_setCurrentLevel(3, context);
}

static scpi_result_t SMU_CHANnel0CurrentLevelQ(scpi_t * context) {
    return smu_getCurrentLevel(0, context);
}
static scpi_result_t SMU_CHANnel1CurrentLevelQ(scpi_t * context) {
    return smu_getCurrentLevel(1, context);
}
static scpi_result_t SMU_CHANnel2CurrentLevelQ(scpi_t * context) {
    return smu_getCurrentLevel(2, context);
}
static scpi_result_t SMU_CHANnel3CurrentLevelQ(scpi_t * context) {
    return smu_getCurrentLevel(3, context);
}

static scpi_result_t smu_setVoltageProtection(uint8_t ch_i, scpi_t * context, uint8_t isUpper) {
    uint32_t channel = chToChannel(ch_i);
    double param1;
    printf("ch%d:Vol:%s\r\n", ch_i, isUpper ? "Upper" : "Lower");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    float vref  = h_PMU.vref;
    double V_level;

    V_level=((1.0*param1)/4.5/vref)*pow(2,16)+32768;
    V_level = V_level>65535?65535:V_level;
    V_level = V_level<0?0:V_level;

    AD5522_in();
    if (isUpper) {
        AD5522_SetClamp(&h_PMU, channel,
            h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
            (uint16_t)V_level,
            h_PMU.i_range[ch_i]);
    } else {
        
        AD5522_SetClamp(&h_PMU, channel,
            h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
            (uint16_t)V_level,
            h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
            h_PMU.i_range[ch_i]);
    }
    AD5522_out();

    return SCPI_RES_OK;
}

static scpi_result_t smu_getVoltageProtection(uint8_t ch_i, scpi_t * context, uint8_t isUpper) {
    printf("ch%d:Vol:%s?\r\n", ch_i, isUpper ? "Upper" : "Lower");
    uint32_t V_level;
    if (isUpper) {
        V_level = h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1];
    } else {
        V_level = h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1];
    }
    float vref = h_PMU.vref;
    double Vlevel = (V_level-32768.0)/pow(2,16)*vref*4.5;

    SCPI_ResultDouble(context, Vlevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageProtectionUpper(scpi_t * context) {
    return smu_setVoltageProtection(0, context, 1);
}
static scpi_result_t SMU_CHANnel1VoltageProtectionUpper(scpi_t * context) {
    return smu_setVoltageProtection(1, context, 1);
}
static scpi_result_t SMU_CHANnel2VoltageProtectionUpper(scpi_t * context) {
    return smu_setVoltageProtection(2, context, 1);
}
static scpi_result_t SMU_CHANnel3VoltageProtectionUpper(scpi_t * context) {
    return smu_setVoltageProtection(3, context, 1);
}

static scpi_result_t SMU_CHANnel0VoltageProtectionUpperQ(scpi_t * context) {
    return smu_getVoltageProtection(0, context, 1);
}
static scpi_result_t SMU_CHANnel1VoltageProtectionUpperQ(scpi_t * context) {
    return smu_getVoltageProtection(1, context, 1);
}
static scpi_result_t SMU_CHANnel2VoltageProtectionUpperQ(scpi_t * context) {
    return smu_getVoltageProtection(2, context, 1);
}
static scpi_result_t SMU_CHANnel3VoltageProtectionUpperQ(scpi_t * context) {
    return smu_getVoltageProtection(3, context, 1);
}

static scpi_result_t SMU_CHANnel0VoltageProtectionLower(scpi_t * context) {
    return smu_setVoltageProtection(0, context, 0);
}
static scpi_result_t SMU_CHANnel1VoltageProtectionLower(scpi_t * context) {
    return smu_setVoltageProtection(1, context, 0);
}
static scpi_result_t SMU_CHANnel2VoltageProtectionLower(scpi_t * context) {
    return smu_setVoltageProtection(2, context, 0);
}
static scpi_result_t SMU_CHANnel3VoltageProtectionLower(scpi_t * context) {
    return smu_setVoltageProtection(3, context, 0);
}

static scpi_result_t SMU_CHANnel0VoltageProtectionLowerQ(scpi_t * context) {
    return smu_getVoltageProtection(0, context, 0);
}
static scpi_result_t SMU_CHANnel1VoltageProtectionLowerQ(scpi_t * context) {
    return smu_getVoltageProtection(1, context, 0);
}
static scpi_result_t SMU_CHANnel2VoltageProtectionLowerQ(scpi_t * context) {
    return smu_getVoltageProtection(2, context, 0);
}
static scpi_result_t SMU_CHANnel3VoltageProtectionLowerQ(scpi_t * context) {
    return smu_getVoltageProtection(3, context, 0);
}

static scpi_result_t smu_setCurrentProtection(uint8_t ch_i, scpi_t * context, uint8_t isUpper) {
    uint32_t channel = chToChannel(ch_i);
    double param1;
    printf("ch%d:Curr:%s\r\n", ch_i, isUpper ? "Upper" : "Lower");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    float vref  = h_PMU.vref;
    float MI_gain = 10;
    float Rsense = h_PMU.Rsense[ch_i];
    double I_level;

    I_level=((param1*Rsense*MI_gain)/4.5/vref)*pow(2,16) + 32768;
    I_level = I_level>65535?65535:I_level;
    I_level = I_level<0?0:I_level;

    AD5522_in();
    if (isUpper) {
        AD5522_SetClamp(&h_PMU, channel,
            h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
            (uint16_t)I_level,
            h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
            h_PMU.i_range[ch_i]);
    } else {
        AD5522_SetClamp(&h_PMU, channel,
            (uint16_t)I_level,
            h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
            h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
            h_PMU.i_range[ch_i]);
    }
    AD5522_out();

    return SCPI_RES_OK;
}

static scpi_result_t smu_getCurrentProtection(uint8_t ch_i, scpi_t * context, uint8_t isUpper) {
    printf("ch%d:Curr:%s?\r\n", ch_i, isUpper ? "Upper" : "Lower");
    uint32_t I_level;
    if (isUpper) {
        I_level = h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1];
    } else {
        I_level = h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1];
    }
    float MI_gain = 10;
    float vref = h_PMU.vref;
    float Rsense = h_PMU.Rsense[ch_i];
    double Ilevel = (I_level-32768.0)/pow(2,16)*vref*4.5/MI_gain/Rsense;

    SCPI_ResultDouble(context, Ilevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentProtectionUpper(scpi_t * context) {
    return smu_setCurrentProtection(0, context, 1);
}
static scpi_result_t SMU_CHANnel1CurrentProtectionUpper(scpi_t * context) {
    return smu_setCurrentProtection(1, context, 1);
}
static scpi_result_t SMU_CHANnel2CurrentProtectionUpper(scpi_t * context) {
    return smu_setCurrentProtection(2, context, 1);
}
static scpi_result_t SMU_CHANnel3CurrentProtectionUpper(scpi_t * context) {
    return smu_setCurrentProtection(3, context, 1);
}

static scpi_result_t SMU_CHANnel0CurrentProtectionUpperQ(scpi_t * context) {
    return smu_getCurrentProtection(0, context, 1);
}
static scpi_result_t SMU_CHANnel1CurrentProtectionUpperQ(scpi_t * context) {
    return smu_getCurrentProtection(1, context, 1);
}
static scpi_result_t SMU_CHANnel2CurrentProtectionUpperQ(scpi_t * context) {
    return smu_getCurrentProtection(2, context, 1);
}
static scpi_result_t SMU_CHANnel3CurrentProtectionUpperQ(scpi_t * context) {
    return smu_getCurrentProtection(3, context, 1);
}

static scpi_result_t SMU_CHANnel0CurrentProtectionLower(scpi_t * context) {
    return smu_setCurrentProtection(0, context, 0);
}
static scpi_result_t SMU_CHANnel1CurrentProtectionLower(scpi_t * context) {
    return smu_setCurrentProtection(1, context, 0);
}
static scpi_result_t SMU_CHANnel2CurrentProtectionLower(scpi_t * context) {
    return smu_setCurrentProtection(2, context, 0);
}
static scpi_result_t SMU_CHANnel3CurrentProtectionLower(scpi_t * context) {
    return smu_setCurrentProtection(3, context, 0);
}

static scpi_result_t SMU_CHANnel0CurrentProtectionLowerQ(scpi_t * context) {
    return smu_getCurrentProtection(0, context, 0);
}
static scpi_result_t SMU_CHANnel1CurrentProtectionLowerQ(scpi_t * context) {
    return smu_getCurrentProtection(1, context, 0);
}
static scpi_result_t SMU_CHANnel2CurrentProtectionLowerQ(scpi_t * context) {
    return smu_getCurrentProtection(2, context, 0);
}
static scpi_result_t SMU_CHANnel3CurrentProtectionLowerQ(scpi_t * context) {
    return smu_getCurrentProtection(3, context, 0);
}

static scpi_result_t SMU_FetchQ(scpi_t * context) {
    char msg[128] = {0};
    char model[4][10] = {0};
    printf("ch0:FetchQ?\r\n");
    // float MI_gain = 10;
    // float vref = h_PMU.vref;
	// float Rsense = h_PMU.Rsense;

    readSmuFunction(0, model[0], 10);
    readSmuFunction(1, model[1], 10);
    readSmuFunction(2, model[2], 10);
    readSmuFunction(3, model[3], 10);

    sprintf(msg, "%s%lu,%s%lu,%s%lu,%s%lu", model[0], ad7190_val[0], model[1], ad7190_val[1], model[2], ad7190_val[2], model[3], ad7190_val[3]);
    SCPI_ResultText(context, msg);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_Calibrate(scpi_t * context) {
    printf(":Calibrate\r\n");

    AD5522_in();
    AD5522_Calibrate(&h_PMU);
    AD5522_out();
    return SCPI_RES_OK;
}

static scpi_result_t DMM_MeasureVoltageDcQ(scpi_t * context) {
    scpi_number_t param1, param2;
    char bf[15];
    printf("meas:volt:dc\r\n"); /* debug command name */

    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
        /* do something, if parameter not present */
    }

    /* read second paraeter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
        /* do something, if parameter not present */
    }


    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);

    SCPI_ResultDouble(context, 0);

    return SCPI_RES_OK;
}

static scpi_result_t DMM_MeasureVoltageAcQ(scpi_t * context) {
    scpi_number_t param1, param2;
    char bf[15];
    printf("meas:volt:ac\r\n"); /* debug command name */

    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
        /* do something, if parameter not present */
    }

    /* read second paraeter if present */
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
        /* do something, if parameter not present */
    }

    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);

    SCPI_ResultDouble(context, 0);

    return SCPI_RES_OK;
}

static scpi_result_t DMM_ConfigureVoltageDc(scpi_t * context) {
    double param1, param2;
    printf("conf:volt:dc\r\n"); /* debug command name */

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    /* read second paraeter if present */
    if (!SCPI_ParamDouble(context, &param2, FALSE)) {
        /* do something, if parameter not present */
    }

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Bool(scpi_t * context) {
    scpi_bool_t param1;
    printf("TEST:BOOL\r\n"); /* debug command name */

    if (!SCPI_ParamBool(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    return SCPI_RES_OK;
}

scpi_choice_def_t trigger_source[] = {
    {"BUS", 5},
    {"IMMediate", 6},
    {"EXTernal", 7},
    SCPI_CHOICE_LIST_END /* termination of option list */
};

static scpi_result_t TEST_ChoiceQ(scpi_t * context) {

    int32_t param;
    const char * name;

    if (!SCPI_ParamChoice(context, trigger_source, &param, TRUE)) {
        return SCPI_RES_ERR;
    }

    SCPI_ChoiceToName(trigger_source, param, &name);

    SCPI_ResultInt32(context, param);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Numbers(scpi_t * context) {
    int32_t numbers[2];

    SCPI_CommandNumbers(context, numbers, 2, 1);

    return SCPI_RES_OK;
}

static scpi_result_t TEST_Text(scpi_t * context) {
    char buffer[100];
    size_t copy_len;

    if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, FALSE)) {
        buffer[0] = '\0';
        return SCPI_RES_ERR;
    }

    return SCPI_RES_OK;
}

static scpi_result_t TEST_ArbQ(scpi_t * context) {
    const char * data;
    size_t len;

    if (SCPI_ParamArbitraryBlock(context, &data, &len, FALSE)) {
        SCPI_ResultArbitraryBlock(context, data, len);
    }

    return SCPI_RES_OK;
}

struct _scpi_channel_value_t {
    int32_t row;
    int32_t col;
};
typedef struct _scpi_channel_value_t scpi_channel_value_t;

/**
 * @brief
 * parses lists
 * channel numbers > 0.
 * no checks yet.
 * valid: (@1), (@3!1:1!3), ...
 * (@1!1:3!2) would be 1!1, 1!2, 2!1, 2!2, 3!1, 3!2.
 * (@3!1:1!3) would be 3!1, 3!2, 3!3, 2!1, 2!2, 2!3, ... 1!3.
 *
 * @param channel_list channel list, compare to SCPI99 Vol 1 Ch. 8.3.2
 */
static scpi_result_t TEST_Chanlst(scpi_t *context) {
    scpi_parameter_t channel_list_param;
#define MAXROW 2    /* maximum number of rows */
#define MAXCOL 6    /* maximum number of columns */
#define MAXDIM 2    /* maximum number of dimensions */
    scpi_channel_value_t array[MAXROW * MAXCOL]; /* array which holds values in order (2D) */
    size_t chanlst_idx; /* index for channel list */
    size_t arr_idx = 0; /* index for array */
    size_t n, m = 1; /* counters for row (n) and columns (m) */

    /* get channel list */
    if (SCPI_Parameter(context, &channel_list_param, TRUE)) {
        scpi_expr_result_t res;
        scpi_bool_t is_range;
        int32_t values_from[MAXDIM];
        int32_t values_to[MAXDIM];
        size_t dimensions;

        bool for_stop_row = FALSE; /* true if iteration for rows has to stop */
        bool for_stop_col = FALSE; /* true if iteration for columns has to stop */
        int32_t dir_row = 1; /* direction of counter for rows, +/-1 */
        int32_t dir_col = 1; /* direction of counter for columns, +/-1 */

        /* the next statement is valid usage and it gets only real number of dimensions for the first item (index 0) */
        if (!SCPI_ExprChannelListEntry(context, &channel_list_param, 0, &is_range, NULL, NULL, 0, &dimensions)) {
            chanlst_idx = 0; /* call first index */
            arr_idx = 0; /* set arr_idx to 0 */
            do { /* if valid, iterate over channel_list_param index while res == valid (do-while cause we have to do it once) */
                res = SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions);
                if (is_range == FALSE) { /* still can have multiple dimensions */
                    if (dimensions == 1) {
                        /* here we have our values
                         * row == values_from[0]
                         * col == 0 (fixed number)
                         * call a function or something */
                        array[arr_idx].row = values_from[0];
                        array[arr_idx].col = 0;
                    } else if (dimensions == 2) {
                        /* here we have our values
                         * row == values_fom[0]
                         * col == values_from[1]
                         * call a function or something */
                        array[arr_idx].row = values_from[0];
                        array[arr_idx].col = values_from[1];
                    } else {
                        return SCPI_RES_ERR;
                    }
                    arr_idx++; /* inkrement array where we want to save our values to, not neccessary otherwise */
                    if (arr_idx >= MAXROW * MAXCOL) {
                        return SCPI_RES_ERR;
                    }
                } else if (is_range == TRUE) {
                    if (values_from[0] > values_to[0]) {
                        dir_row = -1; /* we have to decrement from values_from */
                    } else { /* if (values_from[0] < values_to[0]) */
                        dir_row = +1; /* default, we increment from values_from */
                    }

                    /* iterating over rows, do it once -> set for_stop_row = false
                     * needed if there is channel list index isn't at end yet */
                    for_stop_row = FALSE;
                    for (n = values_from[0]; for_stop_row == FALSE; n += dir_row) {
                        /* usual case for ranges, 2 dimensions */
                        if (dimensions == 2) {
                            if (values_from[1] > values_to[1]) {
                                dir_col = -1;
                            } else if (values_from[1] < values_to[1]) {
                                dir_col = +1;
                            }
                            /* iterating over columns, do it at least once -> set for_stop_col = false
                             * needed if there is channel list index isn't at end yet */
                            for_stop_col = FALSE;
                            for (m = values_from[1]; for_stop_col == FALSE; m += dir_col) {
                                /* here we have our values
                                 * row == n
                                 * col == m
                                 * call a function or something */
                                array[arr_idx].row = n;
                                array[arr_idx].col = m;
                                arr_idx++;
                                if (arr_idx >= MAXROW * MAXCOL) {
                                    return SCPI_RES_ERR;
                                }
                                if (m == (size_t)values_to[1]) {
                                    /* endpoint reached, stop column for-loop */
                                    for_stop_col = TRUE;
                                }
                            }
                            /* special case for range, example: (@2!1) */
                        } else if (dimensions == 1) {
                            /* here we have values
                             * row == n
                             * col == 0 (fixed number)
                             * call function or sth. */
                            array[arr_idx].row = n;
                            array[arr_idx].col = 0;
                            arr_idx++;
                            if (arr_idx >= MAXROW * MAXCOL) {
                                return SCPI_RES_ERR;
                            }
                        }
                        if (n == (size_t)values_to[0]) {
                            /* endpoint reached, stop row for-loop */
                            for_stop_row = TRUE;
                        }
                    }


                } else {
                    return SCPI_RES_ERR;
                }
                /* increase index */
                chanlst_idx++;
            } while (SCPI_EXPR_OK == SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions));
            /* while checks, whether incremented index is valid */
        }
        /* do something at the end if needed */
        /* array[arr_idx].row = 0; */
        /* array[arr_idx].col = 0; */
    }

    {
        size_t i;
        printf("TEST_Chanlst: ");
        for (i = 0; i< arr_idx; i++) {
            // printf("%d!%d, ", array[i].row, array[i].col);
            ;
        }
        printf("\r\n");
    }
    return SCPI_RES_OK;
}

/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t * context) {

    SCPI_ResultInt32(context, 0);

    return SCPI_RES_OK;
}

const scpi_command_t scpi_commands[] = {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = SCPI_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = My_CoreTstQ,},
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    /* {.pattern = "STATus:OPERation?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:EVENt?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:CONDition?", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle", .callback = scpi_stub_callback,}, */
    /* {.pattern = "STATus:OPERation:ENABle?", .callback = scpi_stub_callback,}, */

    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    /* {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,}, */
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},

    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    /* SMU */
    //指令发送错误的话，软件会进入断言停止进一步执行
    {.pattern = ":CHANnel0:FUNCtion", .callback = SMU_CHANnel0Function,},    //:CHANnel0:FUNCtion "HIZMV"  字符串最后需要增加0x0A，调试软件先输入字符串，再勾选十六进制发送，此时可增加 0A结束符，最后发送。
    {.pattern = ":CHANnel0:FUNCtion?", .callback = SMU_CHANnel0FunctionQ,},  //:CHANnel0:FUNCtion?
    {.pattern = ":CHANnel0:CURRent:RANGe?", .callback = SMU_CHANnel0CurrentRangeQ,},
    {.pattern = ":CHANnel0:VOLTage:LEVel",  .callback = SMU_CHANnel0VoltageLevel,},
    {.pattern = ":CHANnel0:VOLTage:LEVel?", .callback = SMU_CHANnel0VoltageLevelQ,},
    {.pattern = ":CHANnel0:CURRent:LEVel",  .callback = SMU_CHANnel0CurrentLevel,},
    {.pattern = ":CHANnel0:CURRent:LEVel?", .callback = SMU_CHANnel0CurrentLevelQ,},
    {.pattern = ":CHANnel0:VOLTage:PROTection:UPPer",  .callback = SMU_CHANnel0VoltageProtectionUpper,},
    {.pattern = ":CHANnel0:VOLTage:PROTection:UPPer?", .callback = SMU_CHANnel0VoltageProtectionUpperQ,},
    {.pattern = ":CHANnel0:VOLTage:PROTection:LOWer",  .callback = SMU_CHANnel0VoltageProtectionLower,},
    {.pattern = ":CHANnel0:VOLTage:PROTection:LOWer?", .callback = SMU_CHANnel0VoltageProtectionLowerQ,},
    {.pattern = ":CHANnel0:CURRent:PROTection:UPPer",  .callback = SMU_CHANnel0CurrentProtectionUpper,},
    {.pattern = ":CHANnel0:CURRent:PROTection:UPPer?", .callback = SMU_CHANnel0CurrentProtectionUpperQ,},
    {.pattern = ":CHANnel0:CURRent:PROTection:LOWer",  .callback = SMU_CHANnel0CurrentProtectionLower,},
    {.pattern = ":CHANnel0:CURRent:PROTection:LOWer?", .callback = SMU_CHANnel0CurrentProtectionLowerQ,},

    {.pattern = ":CHANnel1:FUNCtion", .callback = SMU_CHANnel1Function,},
    {.pattern = ":CHANnel1:FUNCtion?", .callback = SMU_CHANnel1FunctionQ,},
    {.pattern = ":CHANnel1:CURRent:RANGe?", .callback = SMU_CHANnel1CurrentRangeQ,},
    {.pattern = ":CHANnel1:VOLTage:LEVel",  .callback = SMU_CHANnel1VoltageLevel,},
    {.pattern = ":CHANnel1:VOLTage:LEVel?", .callback = SMU_CHANnel1VoltageLevelQ,},
    {.pattern = ":CHANnel1:CURRent:LEVel",  .callback = SMU_CHANnel1CurrentLevel,},
    {.pattern = ":CHANnel1:CURRent:LEVel?", .callback = SMU_CHANnel1CurrentLevelQ,},
    {.pattern = ":CHANnel1:VOLTage:PROTection:UPPer",  .callback = SMU_CHANnel1VoltageProtectionUpper,},
    {.pattern = ":CHANnel1:VOLTage:PROTection:UPPer?", .callback = SMU_CHANnel1VoltageProtectionUpperQ,},
    {.pattern = ":CHANnel1:VOLTage:PROTection:LOWer",  .callback = SMU_CHANnel1VoltageProtectionLower,},
    {.pattern = ":CHANnel1:VOLTage:PROTection:LOWer?", .callback = SMU_CHANnel1VoltageProtectionLowerQ,},
    {.pattern = ":CHANnel1:CURRent:PROTection:UPPer",  .callback = SMU_CHANnel1CurrentProtectionUpper,},
    {.pattern = ":CHANnel1:CURRent:PROTection:UPPer?", .callback = SMU_CHANnel1CurrentProtectionUpperQ,},
    {.pattern = ":CHANnel1:CURRent:PROTection:LOWer",  .callback = SMU_CHANnel1CurrentProtectionLower,},
    {.pattern = ":CHANnel1:CURRent:PROTection:LOWer?", .callback = SMU_CHANnel1CurrentProtectionLowerQ,},

    {.pattern = ":CHANnel2:FUNCtion", .callback = SMU_CHANnel2Function,},
    {.pattern = ":CHANnel2:FUNCtion?", .callback = SMU_CHANnel2FunctionQ,},
    {.pattern = ":CHANnel2:CURRent:RANGe?", .callback = SMU_CHANnel2CurrentRangeQ,},
    {.pattern = ":CHANnel2:VOLTage:LEVel",  .callback = SMU_CHANnel2VoltageLevel,},
    {.pattern = ":CHANnel2:VOLTage:LEVel?", .callback = SMU_CHANnel2VoltageLevelQ,},
    {.pattern = ":CHANnel2:CURRent:LEVel",  .callback = SMU_CHANnel2CurrentLevel,},
    {.pattern = ":CHANnel2:CURRent:LEVel?", .callback = SMU_CHANnel2CurrentLevelQ,},
    {.pattern = ":CHANnel2:VOLTage:PROTection:UPPer",  .callback = SMU_CHANnel2VoltageProtectionUpper,},
    {.pattern = ":CHANnel2:VOLTage:PROTection:UPPer?", .callback = SMU_CHANnel2VoltageProtectionUpperQ,},
    {.pattern = ":CHANnel2:VOLTage:PROTection:LOWer",  .callback = SMU_CHANnel2VoltageProtectionLower,},
    {.pattern = ":CHANnel2:VOLTage:PROTection:LOWer?", .callback = SMU_CHANnel2VoltageProtectionLowerQ,},
    {.pattern = ":CHANnel2:CURRent:PROTection:UPPer",  .callback = SMU_CHANnel2CurrentProtectionUpper,},
    {.pattern = ":CHANnel2:CURRent:PROTection:UPPer?", .callback = SMU_CHANnel2CurrentProtectionUpperQ,},
    {.pattern = ":CHANnel2:CURRent:PROTection:LOWer",  .callback = SMU_CHANnel2CurrentProtectionLower,},
    {.pattern = ":CHANnel2:CURRent:PROTection:LOWer?", .callback = SMU_CHANnel2CurrentProtectionLowerQ,},

    {.pattern = ":CHANnel3:FUNCtion", .callback = SMU_CHANnel3Function,},
    {.pattern = ":CHANnel3:FUNCtion?", .callback = SMU_CHANnel3FunctionQ,},
    {.pattern = ":CHANnel3:CURRent:RANGe?", .callback = SMU_CHANnel3CurrentRangeQ,},
    {.pattern = ":CHANnel3:VOLTage:LEVel",  .callback = SMU_CHANnel3VoltageLevel,},
    {.pattern = ":CHANnel3:VOLTage:LEVel?", .callback = SMU_CHANnel3VoltageLevelQ,},
    {.pattern = ":CHANnel3:CURRent:LEVel",  .callback = SMU_CHANnel3CurrentLevel,},
    {.pattern = ":CHANnel3:CURRent:LEVel?", .callback = SMU_CHANnel3CurrentLevelQ,},
    {.pattern = ":CHANnel3:VOLTage:PROTection:UPPer",  .callback = SMU_CHANnel3VoltageProtectionUpper,},
    {.pattern = ":CHANnel3:VOLTage:PROTection:UPPer?", .callback = SMU_CHANnel3VoltageProtectionUpperQ,},
    {.pattern = ":CHANnel3:VOLTage:PROTection:LOWer",  .callback = SMU_CHANnel3VoltageProtectionLower,},
    {.pattern = ":CHANnel3:VOLTage:PROTection:LOWer?", .callback = SMU_CHANnel3VoltageProtectionLowerQ,},
    {.pattern = ":CHANnel3:CURRent:PROTection:UPPer",  .callback = SMU_CHANnel3CurrentProtectionUpper,},
    {.pattern = ":CHANnel3:CURRent:PROTection:UPPer?", .callback = SMU_CHANnel3CurrentProtectionUpperQ,},
    {.pattern = ":CHANnel3:CURRent:PROTection:LOWer",  .callback = SMU_CHANnel3CurrentProtectionLower,},
    {.pattern = ":CHANnel3:CURRent:PROTection:LOWer?", .callback = SMU_CHANnel3CurrentProtectionLowerQ,},

    {.pattern = "FETCh?", .callback = SMU_FetchQ,},
    {.pattern = "Calibrate", .callback = SMU_Calibrate,},

    {.pattern = "MEASure:VOLTage:DC?", .callback = DMM_MeasureVoltageDcQ,},
    {.pattern = "CONFigure:VOLTage:DC", .callback = DMM_ConfigureVoltageDc,},
    {.pattern = "MEASure:VOLTage:DC:RATio?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:VOLTage:AC?", .callback = DMM_MeasureVoltageAcQ,},
    {.pattern = "MEASure:CURRent:DC?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:CURRent:AC?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:RESistance?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:FRESistance?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:FREQuency?", .callback = SCPI_StubQ,},
    {.pattern = "MEASure:PERiod?", .callback = SCPI_StubQ,},

    {.pattern = "SYSTem:COMMunication:TCPIP:CONTROL?", .callback = SCPI_SystemCommTcpipControlQ,},

    {.pattern = "TEST:BOOL", .callback = TEST_Bool,},
    {.pattern = "TEST:CHOice?", .callback = TEST_ChoiceQ,},
    {.pattern = "TEST#:NUMbers#", .callback = TEST_Numbers,},
    {.pattern = "TEST:TEXT", .callback = TEST_Text,},
    {.pattern = "TEST:ARBitrary?", .callback = TEST_ArbQ,},
    {.pattern = "TEST:CHANnellist", .callback = TEST_Chanlst,},

    SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;
