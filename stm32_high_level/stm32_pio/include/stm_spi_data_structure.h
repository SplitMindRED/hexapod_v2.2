#ifndef stm_spi_data_structure_h
#define stm_spi_data_structure_h

#include "stdint.h"

//####### SPI FLAGS #########
//0 - OE
#define FLAG_OE            (1 << 0)

#define START_BYTE_1       0xE7
#define START_BYTE_2       0x18
#define STOP_BYTE          0x18

extern struct Master_output
{
   // uint8_t servo[18];
   uint16_t servo[18];

   uint8_t flags;
}master_output;

extern struct Master_input
{
   int16_t AcX;
   int16_t AcY;
   int16_t AcZ;

   int16_t GyX;
   int16_t GyY;
   int16_t GyZ;

   int16_t MaX;
   int16_t MaY;
   int16_t MaZ;

   int16_t INA1_Ch1;
   int16_t INA1_Ch2;
   int16_t INA1_Ch3;

   int16_t INA2_Ch1;
   int16_t INA2_Ch2;
   int16_t INA2_Ch3;

   int16_t INA3_Ch1;
   int16_t INA3_Ch2;
   int16_t INA3_Ch3;

   int16_t INA4_Ch1;
   int16_t INA4_Ch2;
   int16_t INA4_Ch3;

   int16_t INA5_Ch1;
   int16_t INA5_Ch2;
   int16_t INA5_Ch3;

   int16_t INA6_Ch1;
   int16_t INA6_Ch2;
   int16_t INA6_Ch3;
}master_input;

#endif
