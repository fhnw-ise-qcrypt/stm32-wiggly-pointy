


----

## Polling Example

Delay(50) required for polling each channel because t_conv = 33ms
using config:

```c
	// be careful with the bitwise or operator "|"
	cmd[0]  = MCP3561_CONFIG0_WRITE;
	cmd[1]  = MCP3561_CONFIG0_CLK_SEL_EXT;   // clock selection
	cmd[1] |= MCP3561_CONFIG0_ADC_MODE_CONV; // standby or converting
	cmd[1] |= MCP3561_CONFIG0_CS_SEL_NONE;   // input current
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG1_WRITE;
	cmd[1]  = MCP3561_CONFIG1_OSR_4096;       // over sampling rate
	cmd[1] |= MCP3561_CONFIG1_AMCLK_DIV8;    // sampling clock prescaler
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG2_WRITE;
	cmd[1]  = MCP3561_CONFIG2_BOOST_x1;   // Boost
	cmd[1] |= MCP3561_CONFIG2_GAIN_x1;    // Gain
	cmd[1] |= MCP3561_CONFIG2_AZ_MUX_OFF; // offset cancellation algorithm
	cmd[1] += 3; // last two bits must always be '11'
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG3_WRITE;
	cmd[1]  = MCP3561_CONFIG3_CONV_MODE_CONTINUOUS; // conversion mode
	cmd[1] |= MCP3561_CONFIG3_DATA_FORMAT_24BIT;    // SPI output data format, (32 and 24 bit available)
	cmd[1] |= MCP3561_CONFIG3_CRCCOM_OFF;           // CRC
	cmd[1] |= MCP3561_CONFIG3_GAINCAL_OFF;          // gain calibration
	cmd[1] |= MCP3561_CONFIG3_OFFCAL_OFF;           // offset calibration
	_MCP3561_write(hspi, cmd, 2);

	cmd[0]  = MCP3561_IRQ_WRITE;
	cmd[1]  = MCP3561_IRQ_MODE_IRQ_HIGH;  // IRQ default pin state
	cmd[1] |= MCP3561_IRQ_FASTCMD_ON;     // fast commands
	cmd[1] |= MCP3561_IRQ_STP_ON;         // start of conversion IRQ
	_MCP3561_write(hspi, cmd, 2);
```

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



