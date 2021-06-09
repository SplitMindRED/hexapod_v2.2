#include "splitmind_stm32f103_lib.h"
#include "hexapod.h"

// #define SHOW_IMU

// #define PRINT_DATA
#ifdef PRINT_DATA
#define PRINT_AC
#define PRINT_GY
//#define PRINT_MA
// #define PRINT_INA_1
// #define PRINT_INA_2
// #define PRINT_INA_3
// #define PRINT_INA_4
// #define PRINT_INA_5
// #define PRINT_INA_6
#endif // PRINT_DATA

#ifndef PRINT_DATA
#define PRINT_DATA_PLOT
#endif // !PRINT_DATA

#ifdef PRINT_DATA_PLOT
// #define PRINT_PLOT_AC
// #define PRINT_PLOT_GY
// #define PRINT_PLOT_LEG_0
// #define PRINT_PLOT_LEG_1
// #define PRINT_PLOT_LEG_2
// #define PRINT_PLOT_LEG_3
// #define PRINT_PLOT_LEG_4
#define PRINT_PLOT_LEG_5
#endif // PRINT_DATA

// #define LED_ERROR          B3
// #define LED_WARNING        B4
// #define LED_BT_CONNECTION  B5

void test()
{
   // Enable slave
   SPI_EnableSlave();
   // Write command to slave to turn on LED blinking
   SPI_Transfer((uint8_t)'1');
   // Write command to slave for asking LED blinking status
   SPI_Transfer((uint8_t)'?');

   // Disable slave
   SPI_DisableSlave();

   delay(500);

   // Enable slave
   SPI_EnableSlave();
   // Write command to slave to turn off LED blinking
   SPI_Transfer((uint8_t)'0');
   // Write command to slave for asking LED blinking status
   SPI_Transfer((uint8_t)'?');

   // Disable slave
   SPI_DisableSlave();

   delay(500);
}

void transferFrame()
{
   SoftTimer_ms timeout;
   timeout.start_time = system_time;
   timeout.delay = 21;

   //   uint8_t buffer[54];
   uint8_t* p_master_input = (uint8_t*)&master_input_raw;
   uint8_t* p_master_output = (uint8_t*)&master_output;

   //eval control sum for output
   master_output.sum = evalSum(p_master_output, sizeof(master_output) - 1);

   //wait for 0
   while (digitalRead(PORT_A, 8) != 0)
   {
      //if (checkTimer(&timeout))
      //{
      //   UART1_println_str("TIMEOUT 0!!!");
      //   return;
      //}
   }

   // Enable slave
   SPI_EnableSlave();

   SPI_Transfer(START_BYTE_1);

   //wait for 1
   timeout.start_time = system_time;
   while (digitalRead(PORT_A, 8) != 1)
   {
      if (checkTimer(&timeout))
      {
         UART1_println_str("TIMEOUT 1!!!");
         SPI_DisableSlave();
         return;
      }
   }

   //wait for 0
   timeout.start_time = system_time;
   while (digitalRead(PORT_A, 8) != 0)
   {
      if (checkTimer(&timeout))
      {
         UART1_println_str("TIMEOUT 2!!!");
         SPI_DisableSlave();
         return;
      }
   }

   SPI_Transfer(START_BYTE_2);

   timeout.start_time = system_time;
   while (digitalRead(PORT_A, 8) != 1)
   {
      if (checkTimer(&timeout))
      {
         UART1_println_str("TIMEOUT 3!!!");
         SPI_DisableSlave();
         return;
      }
   }

   for (uint8_t i = 0; i < sizeof(master_input); i++)
   {
      //wait for 0
      timeout.start_time = system_time;
      while (digitalRead(PORT_A, 8) != 0)
      {
         if (checkTimer(&timeout))
         {
            UART1_print_str("TIMEOUT 4!!! ");
            UART1_println(i);
            SPI_DisableSlave();
            return;
         }
      }

      //MOSI
      if (i < sizeof(master_output))
      {
         SPI_Transfer(*p_master_output++);
      }
      else
      {
         SPI_Transfer(i);
      }

      //MISO
      //buffer[i] = SPI1->DR;
      *p_master_input++ = SPI1->DR;

      //wait for 1
      timeout.start_time = system_time;
      while (digitalRead(PORT_A, 8) != 1)
      {
         if (checkTimer(&timeout))
         {
            UART1_print_str("TIMEOUT 5!!! ");
            UART1_println(i);
            SPI_DisableSlave();
            return;
         }
      }
   }

   // Disable slave
   SPI_DisableSlave();

   if (checkSum(master_input_raw.sum, (uint8_t*)&master_input_raw, sizeof(master_input_raw) - 1) == 1)
   {
      master_input = master_input_raw;
   }
   else
   {
      UART1_println_str("Checksum FAIL");
   }

   //for (uint8_t i = 0; i < 54; i++)
   //{
   //   UART1_print_str("master ");
   //   UART1_print(i);
   //   UART1_print_str(": ");
   //   UART1_println(buffer[i]);
   //}
}

void evalImuAngles()
{
   AcX = (double)master_input.AcX / 2048.0;
   AcY = (double)master_input.AcY / 2048.0;
   AcZ = ((double)master_input.AcZ + 2048.0) / 2048.0;

   GyX += (double)master_input.GyX / 16.4 * (20.0 / 1000.0);
   GyY += (double)master_input.GyY / 16.4 * (20.0 / 1000.0);
   GyZ += (double)master_input.GyZ / 16.4 * (20.0 / 1000.0);

#ifdef SHOW_IMU
   UART1_print_str("AcX: ");
   UART1_print_div(AcX);
   UART1_print_str(" AcY: ");
   UART1_print_div(AcY);
   UART1_print_str(" AcZ: ");
   UART1_print_div(AcZ);
   UART1_print_str(" GyX: ");
   UART1_print_div(GyX);
   UART1_print_str(" GyY: ");
   UART1_print_div(GyY);
   UART1_print_str(" GyZ: ");
   UART1_println_div(GyZ);
#endif

   if (AcY > 0 && AcZ < 0)
   {
      RadX = 1.57079 - atan(AcZ / AcY);
   }
   else if (AcY < 0 && AcZ < 0)
   {
      RadX = -1.57079 - atan(AcZ / AcY);
   }
   else RadX = atan(AcY / AcZ);

   if (AcX < 0 && AcZ < 0)
   {
      RadY = 1.57079 + atan(AcZ / AcX);
   }
   else if (AcX > 0 && AcZ < 0)
   {
      RadY = -1.57079 + atan(AcZ / AcX);
   }
   else
   {
      RadY = -atan(AcX / AcZ);
   }

   // GradX = RadX * 180.0 / pi;
   // GradY = RadY * 180.0 / pi;

   //mpu has different axis directions from my axis convention
   // double tmp = 0;
   // tmp = RadX;
   // RadX = -RadY;
   // RadY = tmp;

   RadX = -GyY * DEG_TO_RAD;
   RadY = GyX * DEG_TO_RAD;
   RadZ = GyZ * DEG_TO_RAD;

   GradX = RadX * 180.0 / pi;
   GradY = RadY * 180.0 / pi;

   // UART1_print_str("Qx: ");
   // UART1_print_div(GradX);
   // UART1_print_str(" Qy: ");
   // UART1_println_div(GradY);
}

void printInputData()
{
#ifdef PRINT_DATA

#ifdef PRINT_AC
   UART1_print_str("AcX: ");
   UART1_print(master_input.AcX);
   UART1_print_str(" AcY: ");
   UART1_print(master_input.AcY);
   UART1_print_str(" AcZ: ");
   UART1_print(master_input.AcZ);
   UART1_print_str(" ");
#endif // PRINT_AC

#ifdef PRINT_GY
   UART1_print_str("GyX: ");
   UART1_print(master_input.GyX);
   UART1_print_str(" GyY: ");
   UART1_print(master_input.GyY);
   UART1_print_str(" GyZ: ");
   UART1_print(master_input.GyZ);
   UART1_print_str(" ");
#endif // PRINT_GY

#ifdef PRINT_MA
   UART1_print_str("MaX: ");
   UART1_print(master_input.MaX);
   UART1_print_str(" MaY: ");
   UART1_print(master_input.MaY);
   UART1_print_str(" MaZ: ");
   UART1_print(master_input.MaZ);
   UART1_print_str(" ");
#endif // PRINT_MA

#ifdef PRINT_INA_1
   UART1_print_str("1_Ch1: ");
   UART1_print(master_input.INA1_Ch1);
   UART1_print_str(" 1_Ch2: ");
   UART1_print(master_input.INA1_Ch2);
   UART1_print_str(" 1_Ch3: ");
   UART1_print(master_input.INA1_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_1

#ifdef PRINT_INA_2
   UART1_print_str("2_Ch1: ");
   UART1_print(master_input.INA2_Ch1);
   UART1_print_str(" 2_Ch2: ");
   UART1_print(master_input.INA2_Ch2);
   UART1_print_str(" 2_Ch3: ");
   UART1_print(master_input.INA2_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_2

#ifdef PRINT_INA_3
   UART1_print_str("3_Ch1: ");
   UART1_print(master_input.INA3_Ch1);
   UART1_print_str(" 3_Ch2: ");
   UART1_print(master_input.INA3_Ch2);
   UART1_print_str(" 3_Ch3: ");
   UART1_print(master_input.INA3_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_3

#ifdef PRINT_INA_4
   UART1_print_str("4_Ch1: ");
   UART1_print(master_input.INA4_Ch1);
   UART1_print_str(" 4_Ch2: ");
   UART1_print(master_input.INA4_Ch2);
   UART1_print_str(" 4_Ch3: ");
   UART1_print(master_input.INA4_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_4

#ifdef PRINT_INA_5
   UART1_print_str("5_Ch1: ");
   UART1_print(master_input.INA5_Ch1);
   UART1_print_str(" 5_Ch2: ");
   UART1_print(master_input.INA5_Ch2);
   UART1_print_str(" 5_Ch3: ");
   UART1_print(master_input.INA5_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_5

#ifdef PRINT_INA_6
   UART1_print_str("6_Ch1: ");
   UART1_print(master_input.INA6_Ch1);
   UART1_print_str(" 6_Ch2: ");
   UART1_print(master_input.INA6_Ch2);
   UART1_print_str(" 6_Ch3: ");
   UART1_print(master_input.INA6_Ch3);
   UART1_print_str(" ");
#endif // PRINT_INA_6

   UART1_println_str(" ");

#endif // PRINT_DATA

#ifdef PRINT_DATA_PLOT

#ifdef PRINT_PLOT_LEG_0
   // UART1_print(master_input.INA1_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA1_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA1_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(servo_current[0]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[1]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[2]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_0


#ifdef PRINT_PLOT_LEG_1
   // UART1_print(master_input.INA2_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA2_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA2_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(servo_current[3]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[4]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[5]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_1


#ifdef PRINT_PLOT_LEG_2
   // UART1_print(master_input.INA3_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA3_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA3_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(servo_current[6]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[7]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[8]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_2


#ifdef PRINT_PLOT_LEG_3
   // UART1_print(master_input.INA4_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA4_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA4_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(servo_current[9]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[10]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[11]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_3


#ifdef PRINT_PLOT_LEG_4
   // UART1_print(master_input.INA5_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA5_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA5_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(servo_current[12]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[13]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[14]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_4


#ifdef PRINT_PLOT_LEG_5
   // UART1_print(master_input.INA6_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA6_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA6_Ch3);

   UART1_print_div(servo_current[15]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[16]);
   UART1_print_str(", ");
   UART1_print_div(servo_current[17]);
   UART1_print_str(", ");
#endif // PRINT_PLOT_LEG_5

   UART1_println_str(" ");

#endif // PRINT_DATA_PLOT

}

void setup()
{
   systemTimeInit();

   //UART1_init(2000000);		//arduino com port
   UART1_init(250000);		//monitor pro port
   UART1_println_str("Setup...");

   SPI_init();

   //LEDs
   // pinMode(PORT_B, 3, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);
   // pinMode(PORT_B, 4, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);
   // pinMode(PORT_B, 5, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);

   // //test leds
   // digitalWrite(PORT_B, 3, 1);
   // digitalWrite(PORT_B, 4, 1);
   // digitalWrite(PORT_B, 5, 1);

   //onboard led
   pinMode(PORT_C, 13, OUTPUT_2, OUTPUT_GPO_PUSH_PULL);

   delay(1000);

   //digitalWrite(PORT_B, 3, 0);
   //digitalWrite(PORT_B, 4, 0);
   //digitalWrite(PORT_B, 5, 0);

   // Disable SPI slave device
   SPI_DisableSlave();

   // for (uint8_t i = 0; i < 18; i++)
   // {
   //    master_output.servo[i] = i * 2;
   //    UART1_print_str("Servo ");
   //    UART1_print(i);
   //    UART1_print_str(" : ");
   //    UART1_println(master_output.servo[i]);
   // }

   EXTI_init();

   __enable_irq();

   hexapodInit((uint8_t*)&master_output);

   // const uint8_t a = sizeof(master_input);
   // const uint8_t b = sizeof(master_output);

   UART1_print_str("master input size: ");
   UART1_print(sizeof(master_input));
   UART1_print_str(" master output size: ");
   UART1_println(sizeof(master_output));

   UART1_println_str("Done!");
}

int main(void)
{
   setup();

   SoftTimer_ms timer1;
   timer1.start_time = system_time;
   timer1.delay = 0;

   while (1)
   {
      if (checkTimer(&timer1))
      {
         // servoManualControl();

         convertFlySkyData();
         switchMode();

         //unsigned long time1 = system_time;
         transferFrame();
         evalCurrent();
         //unsigned long time2 = system_time;

         //time1 = system_time;
         printInputData();
         evalImuAngles();

         //time2 = system_time;
         // convertPPMtoAngle();

         //UART1_print_str("T tr: ");
         //UART1_println(time2 - time1);
      }
   }
}
