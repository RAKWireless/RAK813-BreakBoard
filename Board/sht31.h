#ifndef __SHT31_H__
#define __SHT31_H__

enum privateConstants
  {
    SHT31_ADDRESS_ADR_PIN_LOW = 0x45,
    SHT31_ADDRESS_ARD_PIN_FLOATING = 0x44,
    SHT31_CMD_READ_STATUS = 0xF32D,
    SHT31_CMD_MEAS_POLLING_H = 0x2400,
    SHT31_CMD_MEAS_POLLING_L = 0x2416,
  };
	

#endif