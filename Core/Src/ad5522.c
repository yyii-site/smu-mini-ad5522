#include "AD5522.h"
#include "main.h"
#include "math.h"

void SetRsense(handle_AD5522* h,uint32_t IscaleID)
{
	switch(IscaleID)
	{
		case PMU_DAC_SCALEID_5UA:
			h->Rsense = 200e3;
			break;
		case PMU_DAC_SCALEID_20UA:
			h->Rsense = 50e3;
			break;
		case PMU_DAC_SCALEID_200UA:
			h->Rsense = 5e3;
			break;
		case PMU_DAC_SCALEID_2MA:
			h->Rsense = 500;
			break;
		case PMU_DAC_SCALEID_EXT:
			h->Rsense = 100;
			break;
	}
}

int AD5522_init(handle_AD5522* h, SPI_HandleTypeDef* hspi,float vref)
{
	h->hspi = hspi;
	h->vref = vref;
	uint32_t cmd=0;
	h->DAC_offset = 0xA492;//24940;// 38750;
	h->M_common = 65535; //2.0/2.1*65535;
	h->C_common = 32768; //34768;//50400;
	//cmd|=PMU_SYSREG_CL0|PMU_SYSREG_CL1|PMU_SYSREG_CL2|PMU_SYSREG_CL3;
	cmd|=PMU_SYSREG_GAIN1|PMU_SYSREG_TMPEN; //Sel  Output Gain to 10 (0-4.5V output)
	//cmd|=PMU_SYSREG_INT10K;
	AD5522_SetSystemControl(h,cmd);
	
	cmd=0;
	AD5522_SetPMU(h,PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3,cmd);
	return 0;
}

uint32_t toLittleEndian(uint32_t value) {
    return ((value >> 24) & 0x000000FF) | // 移动到低位
           ((value >> 8) & 0x0000FF00) |  // 移动到中间
           ((value << 8) & 0x00FF0000) |  // 移动到高位
           ((value << 24) & 0xFF000000);   // 移动到最高位
}

int AD5522_WriteReg(handle_AD5522* h,__IO uint32_t cmd)
{
	// cmd = cmd<<3;  //无需此操作
	uint32_t cmd_l = toLittleEndian(cmd);
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,0);
	SMU_SPI_CS_DELAY
	int resp = HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd_l,4,1000);
	SMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,1);
	return resp;
}

int AD5522_ReadReg(handle_AD5522* h,__IO uint32_t cmd, __IO uint32_t* rst)
{
	uint32_t cmd_l = toLittleEndian(cmd);
	uint32_t cmd_nop = toLittleEndian(0xFFFFFF); // NOP
	uint32_t rst_l = 0;
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,0);
	SMU_SPI_CS_DELAY
	HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd_l,4,1000);
	SMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,1);
	SMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,0);
	SMU_SPI_CS_DELAY
	int resp=HAL_SPI_TransmitReceive(h->hspi,(uint8_t*)&cmd_nop,(uint8_t*)&rst_l,3,1000);
	SMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(SMU_CS_GPIO_Port,SMU_CS_Pin,1);

	*rst = toLittleEndian(rst_l);

	*rst=*rst>>8; //shift 32bit SPI to 24 bit readout
	return resp;
}

int AD5522_SetSystemControl(handle_AD5522* h,__IO uint32_t cmd)
{
	h->reg_sys = PMU_MODE_SYSREG|cmd; 
	AD5522_WriteReg(h,cmd);
	return 0;
}

int AD5522_SetPMU(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t cmd)
{
	if(channel!=0)
	{
		for(int i=0;i<4;i++)
		{
			if((channel&(PMU_CH_0<<i))!=0)
			{
				h->reg_pmu[i] = cmd; 
			}
		}
		AD5522_WriteReg(h,channel|cmd);
		return 0;
	}
	return -1;
}

int AD5522_SetClamp(handle_AD5522* h,__IO uint32_t channel,__IO uint16_t I_low,__IO uint16_t I_high,__IO uint16_t V_low,__IO uint16_t V_high,__IO uint8_t I_range)
{
	I_range&=0x07;
	h->i_range=I_range;
	//set I_range
	SetRsense(h,I_range);
	//Check input integraty
	if((V_low>=V_high)|(I_low>=I_high))
	{
		return -1;
	}

	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_V_X1|V_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_V_X1|V_high);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_I_X1|I_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_I_X1|I_high);
	
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			//cmd|=PMU_PMUREG_HZI;
			cmd|=PMU_PMUREG_CL;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
			h->reg_DAC_CLL_V[i][AD5522_DAC_REG_X1] = V_low; 
			h->reg_DAC_CLH_V[i][AD5522_DAC_REG_X1] = V_high; 
			h->reg_DAC_CLL_I[i][AD5522_DAC_REG_X1] = I_low; 
			h->reg_DAC_CLH_I[i][AD5522_DAC_REG_X1] = I_high; 
		}
	}

	
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CL;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}

int AD5522_SetClamp_float(handle_AD5522* h,__IO uint32_t channel,__IO float I_low,__IO float I_high,__IO float V_low,__IO float V_high,__IO uint8_t I_range)
{
	I_range&=0x07;
	h->i_range=I_range;
	//set I_range
	SetRsense(h,I_range);
	
	float vref  = h->vref;
	double Ilow,Ihigh,Vlow,Vhigh;
	
	Vlow=((1.0*V_low)/4.5/vref)*pow(2,16)+32768;
	Vlow = Vlow>65535?65535:Vlow;
	Vlow = Vlow<0?0:Vlow;
	
	Vhigh=((1.0*V_high)/4.5/vref)*pow(2,16)+32768;
	Vhigh = Vhigh>65535?65535:Vhigh;
	Vhigh = Vhigh<0?0:Vhigh;

	float MI_gain = 5;
	float Rsense = h->Rsense;
	//FI = 4.5 * vref * ((value - 32768)/2^16)/(Rsense*MI_amplifier_Gain)
	Ilow=((I_low*Rsense*MI_gain)/4.5/vref)*pow(2,16) + 32768;
	Ilow = Ilow>65535?65535:Ilow;
	Ilow = Ilow<0?0:Ilow;
	
	Ihigh=((I_high*Rsense*MI_gain)/4.5/vref)*pow(2,16) + 32768;
	Ihigh = Ihigh>65535?65535:Ihigh;
	Ihigh = Ihigh<0?0:Ihigh;
	
	AD5522_SetClamp(h,channel,Ilow,Ihigh,Vlow,Vhigh,I_range);
	return 0;
}

int AD5522_Calibrate(handle_AD5522* h)
{
	// reset all DAC M/C registers
	uint16_t DAC_offset = h->DAC_offset;
	uint16_t M_common = h->M_common;
	uint16_t C_common = h->C_common;
	for(int i=0;i<4;i++)
	{
		uint32_t channel  = (PMU_CH_0<<i);
		
		uint16_t value = DAC_offset;
		h->reg_DAC_offset[i] = value;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_OFFSET|value);
		
		//X = ((M+1)/2^16 * x1) + (C-2^n-1)
		value = M_common;
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M|value);

		value = C_common;
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_C] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_C|value);
		
		//FI = 4.5 * vref * ((value - 32768)/2^16)/(R*M)
		value = M_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_5UA][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_5UA_M|value);
		value = C_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_5UA][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_5UA_C|value);

		value = M_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_20UA][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_20UA_M|value);
		value = C_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_20UA][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_20UA_C|value);
		
		value = M_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_200UA][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_200UA_M|value);
		value = C_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_200UA][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_200UA_C|value);
		
		value = M_common;//(1.0/1.67)*65536;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_2MA][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_2MA_M|value);
		value = C_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_2MA][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_2MA_C|value);
		
		value = M_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_EXT][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_EXTC_M|value);
		value = C_common;
		h->reg_DAC_FIN_I[i][PMU_DAC_SCALEID_EXT][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_EXTC_C|value);
		
		//FI = 4.5 * vref * ((value - 32768)/2^16)/(R*M)
		value = (2/2.1)*65536;
		h->reg_DAC_CLL_I[i][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_I_M|value);
		value = 38000;
		h->reg_DAC_CLL_I[i][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_I_C|value);
		
		//FV = 4.5 * vref * ((value - 32768)/2^16) -(3.5*vref*(offset/2^16)) + DUTGND	
		value = (2/2.1)*65536;
		h->reg_DAC_CLL_V[i][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_V_M|value);
		value = C_common;//50400;
		h->reg_DAC_CLL_V[i][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_V_C|value);
		
		value = (2/2.1)*65536;
		h->reg_DAC_CLH_I[i][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_I_M|value);
		value = 30200;
		h->reg_DAC_CLH_I[i][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_I_C|value);
		
		value = (1.975/2.1)*65536;
		h->reg_DAC_CLH_V[i][AD5522_DAC_REG_M] = value;  
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_V_M|value);
		value = C_common;//49400;
		h->reg_DAC_CLH_V[i][AD5522_DAC_REG_C] = value; 
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_V_C|value);

	}
	return 0;
}
int AD5522_Vmeasure(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t* volt);
int AD5522_StartFV_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	return 0;
}
int AD5522_StartFI_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	return 0;
}

int AD5522_StartHiZMV(handle_AD5522* h,__IO uint32_t channel)
{
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_HZI|PMU_PMUREG_MEAS_V;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}

int AD5522_StartFVMI(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	I_range&=0x07;
	h->i_range=I_range;
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	
	//set I_range
	SetRsense(h,I_range);
	
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=PMU_PMUREG_SF0|PMU_PMUREG_SS0|PMU_PMUREG_CL|PMU_PMUREG_CPOLH|PMU_PMUREG_COMPV; // Keep only those settings
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|(I_range<<15);
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
int AD5522_StartFIMV(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	I_range&=0x07;
	h->i_range=I_range;
	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	
	//set I_range
	SetRsense(h,I_range);
	
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=PMU_PMUREG_SF0|PMU_PMUREG_SS0|PMU_PMUREG_CL|PMU_PMUREG_CPOLH|PMU_PMUREG_COMPV; // Keep only those settings
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FICV|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|(I_range<<15);
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}

int AD5522_StartFVMV(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;
	//set I_range
	SetRsense(h,I_range);
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=PMU_PMUREG_SF0|PMU_PMUREG_SS0|PMU_PMUREG_CL|PMU_PMUREG_CPOLH|PMU_PMUREG_COMPV; // Keep only those settings
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|(I_range<<15);
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
int AD5522_StartFIMI(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;

	//set I_range
	SetRsense(h,I_range);
	
	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=PMU_PMUREG_SF0|PMU_PMUREG_SS0|PMU_PMUREG_CL|PMU_PMUREG_CPOLH|PMU_PMUREG_COMPV; // Keep only those settings
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FICV|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|(I_range<<15);
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
// X1 = (M+1)/2^n * X1 + C - 2^n-1
int AD5522_SetOutputVoltage(handle_AD5522* h,uint32_t channel,uint16_t voltage)
{
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			h->reg_DAC_FIN_V[i][AD5522_DAC_REG_X1] = voltage;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		}
	}
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_X1|voltage);
	return 0;
}
int AD5522_SetOutputCurrent(handle_AD5522* h,uint32_t channel,uint16_t current)
{
	uint32_t reg_base=0x08; //base for current 5uA current DAC(base offset)
	reg_base=(reg_base+h->i_range)<<16; //get base addr for I dac of the selected I range
	reg_base|=PMU_MODE_DATAREG;
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			h->reg_DAC_FIN_I[i][h->i_range][AD5522_DAC_REG_X1] = current;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		}
	}
	AD5522_WriteReg(h,channel|reg_base|current);
	return 0;
}

int AD5522_SetOutputVoltage_float(handle_AD5522* h,__IO uint32_t channel,__IO double voltage)
{
	float vref  = h->vref;
	double v_level;
	//FV = 4.5 * vref * ((value - 32768)/2^16) -(3.5*vref*(offset/2^16)) + DUTGND
	v_level=(1.0*(voltage)/4.5/vref)*pow(2,16)+32768;
	v_level = v_level>65535?65535:v_level;
	v_level = v_level<0?0:v_level;
	AD5522_SetOutputVoltage(h,channel,(uint16_t) v_level);
	return 0;
}
int AD5522_SetOutputCurrent_float(handle_AD5522* h,__IO uint32_t channel,__IO double current)
{
	float vref  = h->vref;
	double i_level;
	float MI_gain = 5;
	float Rsense = h->Rsense;
	//FI = 4.5 * vref * ((value - 32768)/2^16)/(Rsense*MI_amplifier_Gain)
	i_level=2*((1.0*current*Rsense*MI_gain)/4.5/vref)*pow(2,16) + 32768;
	i_level = i_level>65535?65535:i_level;
	i_level = i_level<0?0:i_level;
	AD5522_SetOutputCurrent(h,channel,(uint16_t) i_level);
	return 0;
}

double AD5522_ValueMapper(double input,double input_low,double input_high,double output_low,double output_high)
{
	double gain = (output_high-output_low)/(input_high-input_low);
	double offset = output_low;
	return input*gain+offset;
}