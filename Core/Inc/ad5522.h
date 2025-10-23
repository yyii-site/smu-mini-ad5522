#include "stm32f1xx_hal.h"

#define PMU_REG_RD (0x01<<28)

#define PMU_MODE_SYSREG (0x00 << 22)
#define PMU_MODE_GAINREG (0x01 << 22)
#define PMU_MODE_OFFSETREG (0x02 << 22)
#define PMU_MODE_DATAREG (0x03 << 22)

#define PMU_CH_0 (0x01 << 24)
#define PMU_CH_1 (0x01 << 25)
#define PMU_CH_2 (0x01 << 26)
#define PMU_CH_3 (0x01 << 27)

#define PMU_SYSREG_CL0 (0x01 << 18)
#define PMU_SYSREG_CL1 (0x01 << 19)
#define PMU_SYSREG_CL2 (0x01 << 20)
#define PMU_SYSREG_CL3 (0x01 << 21)

#define PMU_SYSREG_CPOLH0 (0x01 << 14)
#define PMU_SYSREG_CPOLH1 (0x01 << 15)
#define PMU_SYSREG_CPOLH2 (0x01 << 16)
#define PMU_SYSREG_CPOLH3 (0x01 << 17)

#define PMU_SYSREG_CPBIAS 0x01 << 13
/*DUTGND per channel enable. The GUARDINx/DUTGNDx pins are shared pins that can be configured to 
enable a DUTGND per PMU channel or a guard input per PMU channel. Setting this bit to 1 enables 
DUTGND per channel. In this mode, the pin functions as a DUTGND pin on a per-channel basis. 
The guard inputs are disconnected from this pin and instead are connected directly to the MEASVHx line by an internal connection. 
The default power-on condition is GUARDINx.
*/
#define PMU_SYSREG_DUTGND (0x01 << 12)
#define PMU_SYSREG_GUARDALM (0x01 << 11)
#define PMU_SYSREG_CLAMPALM (0x01 << 10)
#define PMU_SYSREG_INT10K (0x01 << 9)
#define PMU_SYSREG_GUARDEN (0x01 << 8)
#define PMU_SYSREG_GAIN1 (0x01 << 7)
#define PMU_SYSREG_GAIN0 (0x01 << 6)
#define PMU_SYSREG_TMPEN (0x01 << 5)
#define PMU_SYSREG_TMP1 (0x01 << 4)
#define PMU_SYSREG_TMP0 (0x01 << 3)
#define PMU_SYSREG_LATCH (0x01 << 2)

#define PMU_PMUREG_CH_EN (0x01 << 21)
#define PMU_PMUREG_FVCI (0x00<<19)
#define PMU_PMUREG_FICV (0x01<<19)
#define PMU_PMUREG_HZV (0x02<<19)
#define PMU_PMUREG_HZI (0x03<<19)
#define PMU_PMUREG_C2 (0x01 << 17)
#define PMU_PMUREG_C1 (0x01 << 16)
#define PMU_PMUREG_C0 (0x01 << 15)
#define PMU_PMUREG_MEAS_I (0x00 <<13)
#define PMU_PMUREG_MEAS_V (0x01 <<13)
#define PMU_PMUREG_MEAS_TEMP (0x02 <<13)
#define PMU_PMUREG_MEAS_HZ (0x03 <<13)
#define PMU_PMUREG_FIN (0x01 << 12)
#define PMU_PMUREG_SF0 (0x01 << 11)
#define PMU_PMUREG_SS0 (0x01 << 10)
#define PMU_PMUREG_CL (0x01 << 9)
#define PMU_PMUREG_CPOLH (0x01 << 8)
#define PMU_PMUREG_COMPV (0x01 << 7)
#define PMU_PMUREG_CLEAR (0x01 << 6)

#define PMU_DACREG_ADDR_OFFSET (0x00<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_5UA_M (0x08<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_5UA_C (0x08<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_5UA_X1 (0x08<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_20UA_M (0x09<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_20UA_C (0x09<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_20UA_X1 (0x09<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_200UA_M (0x0A<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_200UA_C (0x0A<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_200UA_X1 (0x0A<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_2MA_M (0x0B<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_2MA_C (0x0B<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_2MA_X1 (0x0B<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_EXTC_M (0x0C<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_EXTC_C (0x0C<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_EXTC_X1 (0x0C<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_VOL_M (0x0D<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_VOL_C (0x0D<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_VOL_X1 (0x0D<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLL_I_M (0x14<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLL_I_C (0x14<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLL_I_X1 (0x14<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLL_V_M (0x15<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLL_V_C (0x15<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLL_V_X1 (0x15<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLH_I_M (0x1C<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLH_I_C (0x1C<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLH_I_X1 (0x1C<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLH_V_M (0x1D<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLH_V_C (0x1D<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLH_V_X1 (0x1D<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_5UA_M (0x20<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_5UA_C (0x20<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_5UA_X1 (0x20<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_20UA_M (0x21<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_20UA_C (0x21<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_20UA_X1 (0x21<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_200UA_M (0x22<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_200UA_C (0x22<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_200UA_X1 (0x22<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_2MA_M (0x23<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_2MA_C (0x23<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_2MA_X1 (0x23<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_EXTC_M (0x24<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_EXTC_C (0x24<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_EXTC_X1 (0x24<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPL_VOL_M (0x25<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPL_VOL_C (0x25<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPL_VOL_X1 (0x25<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_5UA_M (0x28<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_5UA_C (0x28<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_5UA_X1 (0x28<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_20UA_M (0x29<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_20UA_C (0x29<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_20UA_X1 (0x29<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_200UA_M (0x2A<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_200UA_C (0x2A<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_200UA_X1 (0x2A<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_2MA_M (0x2B<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_2MA_C (0x2B<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_2MA_X1 (0x2B<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_EXTC_M (0x2C<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_EXTC_C (0x2C<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_EXTC_X1 (0x2C<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CPH_VOL_M (0x2D<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CPH_VOL_C (0x2D<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CPH_VOL_X1 (0x2D<<16|PMU_MODE_DATAREG)

#define AD5522_DAC_REG_M 0x00
#define AD5522_DAC_REG_C 0x01
#define AD5522_DAC_REG_X1 0x02

#define PMU_DAC_SCALEID_5UA 0x00
#define PMU_DAC_SCALEID_20UA 0x01
#define PMU_DAC_SCALEID_200UA 0x02
#define PMU_DAC_SCALEID_2MA 0x03
#define PMU_DAC_SCALEID_EXT 0x04

#define AD5522_STATE_IDLE 0x01
#define AD5522_STATE_FVMI 0x02
#define AD5522_STATE_FIMV 0x04
#define AD5522_STATE_CLL_ARM 0x08
#define AD5522_STATE_CLH_ARM 0x10
#define AD5522_STATE_CPL_ARM 0x20
#define AD5522_STATE_CPH_ARM 0x40
typedef struct
{
	__IO uint32_t reg_DAC_offset[4]; 
	__IO uint32_t reg_DAC_FIN_I[4][5][3];  // [CH0 CH1 CH2 CH3] [5uA 20uA 200uA 2mA 80mA] [X1  M  C]
	__IO uint32_t reg_DAC_FIN_V[4][3]; 
	__IO uint32_t reg_DAC_CLL_I[4][3]; 
	__IO uint32_t reg_DAC_CLL_V[4][3]; 
	__IO uint32_t reg_DAC_CLH_I[4][3]; 
	__IO uint32_t reg_DAC_CLH_V[4][3];
	__IO uint32_t reg_DAC_CPL_I[4][5][3]; 
	__IO uint32_t reg_DAC_CPL_V[4][3]; 	
	__IO uint32_t reg_DAC_CPH_I[4][5][3]; 
	__IO uint32_t reg_DAC_CPH_V[4][3]; 	
	__IO uint32_t v_measure[4];
	__IO uint32_t i_range[4];
	__IO uint32_t state[4];
	__IO uint32_t reg_sys;
	__IO uint32_t reg_pmu[4];
	__IO uint32_t reg_cmp;
	__IO uint32_t reg_alarm;
	float vref;
	uint16_t DAC_offset;
	uint16_t M_common;
	uint16_t C_common;
	float Rsense[4];
	SPI_HandleTypeDef* hspi;
}handle_AD5522;
int AD5522_init(handle_AD5522* h, SPI_HandleTypeDef* hspi,float vref);
int AD5522_WriteReg(handle_AD5522* h,__IO uint32_t cmd);
int AD5522_ReadReg(handle_AD5522* h,__IO uint32_t cmd,__IO uint32_t *rst);
int AD5522_SetSystemControl(handle_AD5522* h,__IO uint32_t cmd);
int AD5522_SetPMU(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t cmd);
int AD5522_SetClamp(handle_AD5522* h,__IO uint32_t channel,__IO uint16_t I_low,__IO uint16_t I_high,__IO uint16_t V_low,__IO uint16_t V_high,__IO uint8_t I_range);
int AD5522_SetClamp_float(handle_AD5522* h,__IO uint32_t channel,__IO float I_low,__IO float I_high,__IO float V_low,__IO float V_high,__IO uint8_t I_range);
int AD5522_Calibrate(handle_AD5522* h);
int AD5522_Vmeasure(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t* volt);
int AD5522_StartFV_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_StartFI_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_StartHiZMV(handle_AD5522* h,__IO uint32_t channel);
int AD5522_StartFVMI(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_StartFIMV(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_StartFVMV(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_StartFIMI(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range);
int AD5522_SetOutputVoltage(handle_AD5522* h,__IO uint32_t channel,__IO uint16_t voltage);
int AD5522_SetOutputCurrent(handle_AD5522* h,__IO uint32_t channel,__IO uint16_t current);
int AD5522_SetOutputVoltage_float(handle_AD5522* h,__IO uint32_t channel,__IO double voltage);
int AD5522_SetOutputCurrent_float(handle_AD5522* h,__IO uint32_t channel,__IO double current);
double AD5522_ValueMapper(double input,double input_low,double input_high,double output_low,double output_high);
//#define PMU_SPI_CS_DELAY for(__IO uint8_t cnt=0;cnt<10;cnt++);
#define SMU_SPI_CS_DELAY __asm volatile ("nop");