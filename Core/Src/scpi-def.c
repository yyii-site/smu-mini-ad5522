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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"

#include "math.h"
#include "cmsis_os.h"
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

static scpi_result_t SMU_CHANnel0Output(scpi_t * context) {
    uint32_t channel = PMU_CH_0;
    scpi_bool_t param1;
    printf("ch0:output\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamBool(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    // PMU_PMUREG_CH_EN
    if (false == param1) {
        AD5522_in();
        AD5522_StartHiZMV(&h_PMU, channel);
        AD5522_out();
    }

    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0OutputQ(scpi_t * context) {
    printf("ch0:output?\r\n"); /* debug command name */

    SCPI_ResultInt(context, 1);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0Function(scpi_t * context) {
    uint32_t channel = PMU_CH_0;
    char buffer[100];
    size_t copy_len;
    
    // 模式
    if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, TRUE)) {
        buffer[0] = '\0';
    }

    // printf("ch0:func ***%s***\r\n", buffer);  // 打印格式化字符串时会造成错误

    if (strstr(buffer, "HIZMV") != NULL) {
        printf("HIZMV\n");
        AD5522_in();
        AD5522_StartHiZMV(&h_PMU, channel);
        AD5522_out();
    }
    else if (strstr(buffer, "FVMI") != NULL) {
        printf("FVMI\n");
        AD5522_in();
        AD5522_StartFVMI(&h_PMU, channel, h_PMU.i_range);  //TODO 为什么所有的通道只有一个电流通道？
        AD5522_out();
    }
    else if (strstr(buffer, "FIMV") != NULL) {
        printf("FIMV\n");
        AD5522_in();
        AD5522_StartFIMV(&h_PMU, channel, h_PMU.i_range);
        AD5522_out();
    }
    else if (strstr(buffer, "FVMV") != NULL) {
        printf("FVMV\n");
        AD5522_in();
        AD5522_StartFVMV(&h_PMU, channel, h_PMU.i_range);
        AD5522_out();
    }
    else if (strstr(buffer, "FIMI") != NULL) {
        printf("FIMI\n");
        AD5522_in();
        AD5522_StartFIMI(&h_PMU, channel, h_PMU.i_range);
        AD5522_out();
    }
    else {
        return SCPI_RES_ERR;
    }

    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0FunctionQ(scpi_t * context) {
    uint8_t channel = 0;
    char force[5] = {0};
    char measure[5] = {0};
    printf("ch0:func?\r\n");
    uint32_t pmu = h_PMU.reg_pmu[channel];
    
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

    char result[10] = {0};
    snprintf(result, sizeof(result), "%s%s", force, measure);
    SCPI_ResultText(context, result);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentRange(scpi_t * context) {
    uint32_t channel = PMU_CH_0;
    double param1;
    // uint8_t I_range;
    printf("ch0:Curr:Range\r\n"); /* debug command name */

    /* read first parameter if present */
    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    // I_range = current2Range(param1);
    // AD5522_StartFVMI;

    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentRangeQ(scpi_t * context) {
    printf("ch0:Curr:Range?\r\n");

    double range = range2Current(h_PMU.i_range);
    SCPI_ResultDouble(context, range);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageLevel(scpi_t * context) {
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Vol:Level\r\n");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }

    if (param1 > 11.5) {
        printf("CHANnel0Voltage out of range\r\n");
        return SCPI_RES_ERR;
    } else if (param1 < -11.5) {
        printf("CHANnel0Voltage out of range\r\n");
        return SCPI_RES_ERR;
    } else {
        AD5522_in();
        AD5522_SetOutputVoltage_float(&h_PMU, channel, param1);
        AD5522_out();
    }

    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageLevelQ(scpi_t * context) {
    uint8_t ch_i = 0;
    printf("ch0:Vol:Level?\r\n");
    uint32_t V_level = h_PMU.reg_DAC_FIN_V[ch_i][AD5522_DAC_REG_X1];
    float vref = h_PMU.vref;
    double Vlevel = (V_level-32768.0)/pow(2,16)*vref*4.5;

    SCPI_ResultDouble(context, Vlevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentLevel(scpi_t * context) {
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Curr:Level\r\n");

    /* read first parameter if present */
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

static scpi_result_t SMU_CHANnel0CurrentLevelQ(scpi_t * context) {
    uint8_t ch_i = 0;
    printf("ch0:Curr:Level?\r\n");

    uint32_t I_level = h_PMU.reg_DAC_FIN_I[ch_i][h_PMU.i_range][AD5522_DAC_REG_X1];
	float MI_gain = 10;
    float vref = h_PMU.vref;
	float Rsense = h_PMU.Rsense;
    double Ilevel = 4.5 * vref * ((I_level - 32768.0)/pow(2,16))/(Rsense*MI_gain);

    SCPI_ResultDouble(context, Ilevel);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageProtectionUpper(scpi_t * context) {
    uint8_t ch_i = 0;
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Vol:Upper\r\n");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    float vref  = h_PMU.vref;
    double V_high = param1;
    double Vhigh;
    Vhigh=((1.0*V_high)/4.5/vref)*pow(2,16)+32768;
	Vhigh = Vhigh>65535?65535:Vhigh;
	Vhigh = Vhigh<0?0:Vhigh;
    
    AD5522_in();
    AD5522_SetClamp(&h_PMU, channel,
        h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
        (uint16_t)Vhigh,
        h_PMU.i_range);
    AD5522_out();
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageProtectionUpperQ(scpi_t * context) {
    uint8_t ch_i = 0;
    printf("ch0:Vol:UpperQ?\r\n");
    uint32_t Vhigh = h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1];
    float vref = h_PMU.vref;
    double v_level = (Vhigh-32768.0)/pow(2,16)*vref*4.5;

    SCPI_ResultDouble(context, v_level);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageProtectionLower(scpi_t * context) {
    uint8_t ch_i = 0;
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Vol:Lower\r\n");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    float vref  = h_PMU.vref;
    double V_low = param1;
    double Vlow;
    Vlow=((1.0*V_low)/4.5/vref)*pow(2,16)+32768;
	Vlow = Vlow>65535?65535:Vlow;
	Vlow = Vlow<0?0:Vlow;
    
    AD5522_in();
    AD5522_SetClamp(&h_PMU, channel,
        h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
        (uint16_t)Vlow,
        h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
        h_PMU.i_range);
    AD5522_out();
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0VoltageProtectionLowerQ(scpi_t * context) {
    uint8_t ch_i = 0;
    printf("ch0:Vol:LowerQ?\r\n");
    uint32_t Vlow = h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1];
    float vref = h_PMU.vref;
    double v_level = (Vlow-32768.0)/pow(2,16)*vref*4.5;

    SCPI_ResultDouble(context, v_level);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentProtectionUpper(scpi_t * context) {
    uint8_t ch_i = 0;
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Curr:Upper\r\n");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
	float MI_gain = 10;
	float vref  = h_PMU.vref;
	float Rsense = h_PMU.Rsense;
    double I_high = param1;
    double Ihigh;
    Ihigh=((I_high*Rsense*MI_gain)/4.5/vref)*pow(2,16) + 32768;
	Ihigh = Ihigh>65535?65535:Ihigh;
	Ihigh = Ihigh<0?0:Ihigh;
    
    AD5522_in();
    AD5522_SetClamp(&h_PMU, channel,
        h_PMU.reg_DAC_CLL_I[ch_i][AD5522_DAC_REG_X1],
        (uint16_t)Ihigh,
        h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
        h_PMU.i_range);
    AD5522_out();
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentProtectionUpperQ(scpi_t * context) {
	uint8_t ch_i = 0;
    printf("ch0:Curr:UpperQ?\r\n");
    uint32_t Ihigh = h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1];
	float MI_gain = 10;
    float vref = h_PMU.vref;
	float Rsense = h_PMU.Rsense;
    double i_level = (Ihigh-32768)*1.0/pow(2,16)*vref*4.5/MI_gain/Rsense;

    SCPI_ResultDouble(context, i_level);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentProtectionLower(scpi_t * context) {
    uint8_t ch_i = 0;
    uint32_t channel = PMU_CH_0;
    double param1;
    printf("ch0:Curr:Lower\r\n");

    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
        return SCPI_RES_ERR;
    }
    float MI_gain = 10;
	float vref  = h_PMU.vref;
	float Rsense = h_PMU.Rsense;
    double I_low = param1;
    double Ilow;
    Ilow=((I_low*Rsense*MI_gain)/4.5/vref)*pow(2,16)+32768;
	Ilow = Ilow>65535?65535:Ilow;
	Ilow = Ilow<0?0:Ilow;
    
    AD5522_in();
    AD5522_SetClamp(&h_PMU, channel,
		(uint16_t)I_low,
        h_PMU.reg_DAC_CLH_I[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1],
        h_PMU.reg_DAC_CLH_V[ch_i][AD5522_DAC_REG_X1],
        h_PMU.i_range);
    AD5522_out();
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0CurrentProtectionLowerQ(scpi_t * context) {
    uint8_t ch_i = 0;
    printf("ch0:Curr:LowerQ?\r\n");
    uint32_t Ilow = h_PMU.reg_DAC_CLL_V[ch_i][AD5522_DAC_REG_X1];
    float MI_gain = 10;
    float vref = h_PMU.vref;
	float Rsense = h_PMU.Rsense;
    double i_level = (Ilow-32768.0)/pow(2,16)*vref*4.5/MI_gain/Rsense;

    SCPI_ResultDouble(context, i_level);
    return SCPI_RES_OK;
}

static scpi_result_t SMU_CHANnel0FetchQ(scpi_t * context) {
    char msg[64];
    printf("ch0:FetchQ?\r\n");
    // float MI_gain = 10;
    // float vref = h_PMU.vref;
	// float Rsense = h_PMU.Rsense;

    double buf0 = ad7190_val[0] * 0.1;
    double buf1 = ad7190_val[1] * 0.1;
    double buf2 = ad7190_val[2] * 0.1;
    double buf3 = ad7190_val[3] * 0.1;

    sprintf(msg, "v0:%E,v1:%E,v2:%E,v3:%E", buf0, buf1, buf2, buf3);
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

    /* read first parameter if present */
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

    /* read first parameter if present */
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

    /* read first parameter if present */
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

    /* read first parameter if present */
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
    {.pattern = ":CHANnel0:OUTPut", .callback = SMU_CHANnel0Output,},
    {.pattern = ":CHANnel0:OUTPut?", .callback = SMU_CHANnel0OutputQ,},
    {.pattern = ":CHANnel0:FUNCtion", .callback = SMU_CHANnel0Function,},    //:CHANnel0:FUNCtion "HIZMV"  字符串最后需要增加0x0A，调试软件先输入字符串，再勾选十六进制发送，此时可增加 0A结束符，最后发送。
    {.pattern = ":CHANnel0:FUNCtion?", .callback = SMU_CHANnel0FunctionQ,},  //:CHANnel0:FUNCtion?
    {.pattern = ":CHANnel0:CURRent:RANGe",  .callback = SMU_CHANnel0CurrentRange,},
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
    {.pattern = ":CHANnel0:FETCh?", .callback = SMU_CHANnel0FetchQ,},
    {.pattern = ":Calibrate", .callback = SMU_Calibrate,},

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
