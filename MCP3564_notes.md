


----

## Polling Example

Delay(50) required for polling each channel because t_conv = 33ms
using config:


polling:

```c
	  uint32_t adc_lsb[4];
	  float adc_volt[4];

	  MCP3561_Channels(&hspi1, MCP3561_MUX_CH0, MCP3561_MUX_CH1);
	  HAL_Delay(50);
	  adc_lsb[0] = MCP3561_ReadADCData(&hspi1);
	  adc_volt[0] = ((float)adc_lsb[0])*2*VREF_2V5_CALIBRATED / ((float)0xffffff);
	  //printf("CH1 \t%d \t%.5f\n", (int)adc_val, adc_volts);

	  MCP3561_Channels(&hspi1, MCP3561_MUX_CH2, MCP3561_MUX_CH3);
	  HAL_Delay(50);
	  adc_lsb[1] = MCP3561_ReadADCData(&hspi1);
	  adc_volt[1] = ((float)adc_lsb[1])*2*VREF_2V5_CALIBRATED / ((float)0xffffff);
	  //printf("CH2 \t%d %.5f V\n", (int)adc_val, adc_volts);

	  MCP3561_Channels(&hspi1, MCP3561_MUX_CH4, MCP3561_MUX_CH5);
	  HAL_Delay(50);
	  adc_lsb[2] = MCP3561_ReadADCData(&hspi1);
	  adc_volt[2] = ((float)adc_lsb[2])*2*VREF_2V5_CALIBRATED / ((float)0xffffff);
	  //printf("CH3 \t%d %.5f V\n", (int)adc_val, adc_volts);

	  MCP3561_Channels(&hspi1, MCP3561_MUX_CH6, MCP3561_MUX_CH7);
	  HAL_Delay(50);
	  adc_lsb[3] = MCP3561_ReadADCData(&hspi1);
	  adc_volt[3] = ((float)adc_lsb[3])*2*VREF_2V5_CALIBRATED / ((float)0xffffff);
```



