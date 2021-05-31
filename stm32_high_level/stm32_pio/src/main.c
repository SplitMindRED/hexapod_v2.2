#include "splitmind_stm32f103_lib.h"
#include "hexapod.h"

#define PRINT_DATA
#ifdef PRINT_DATA
#define PRINT_AC
#define PRINT_GY
//#define PRINT_MA
#define PRINT_INA_1
#define PRINT_INA_2
#define PRINT_INA_3
#define PRINT_INA_4
#define PRINT_INA_5
#define PRINT_INA_6
#endif // PRINT_DATA

#ifndef PRINT_DATA
#define PRINT_DATA_PLOT
#endif // !PRINT_DATA

#ifdef PRINT_DATA_PLOT
#define PRINT_PLOT_INA_1
//#define PRINT_PLOT_INA_2
//#define PRINT_PLOT_INA_3
//#define PRINT_PLOT_INA_4
//#define PRINT_PLOT_INA_5
//#define PRINT_PLOT_INA_6
#endif // PRINT_DATA

// #define LED_ERROR          B3
// #define LED_WARNING        B4
// #define LED_BT_CONNECTION  B5

static double c1 = 0, c2 = 0, c3 = 0;

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
   timeout.delay = 15;

   //   uint8_t buffer[54];
   uint8_t* p = (uint8_t*)&master_input;

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
         SPI_Transfer(master_output.servo[i]);
      }
      else
      {
         SPI_Transfer(i);
      }

      //MISO
      //buffer[i] = SPI1->DR;
      *p++ = SPI1->DR;

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

   //for (uint8_t i = 0; i < 54; i++)
   //{
   //   UART1_print_str("master ");
   //   UART1_print(i);
   //   UART1_print_str(": ");
   //   UART1_println(buffer[i]);
   //}
}

void evalCurrent()
{
   c1 = (double)master_input.INA1_Ch1 * 5 / 100;
   c2 = (double)master_input.INA1_Ch2 * 5 / 100;
   c3 = (double)master_input.INA1_Ch3 * 5 / 100;
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

#ifdef PRINT_PLOT_INA_1
   // UART1_print(master_input.INA1_Ch1);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA1_Ch2);
   // UART1_print_str(", ");
   // UART1_print(master_input.INA1_Ch3);
   // UART1_print_str(", ");

   UART1_print_div(c1);
   UART1_print_str(", ");
   UART1_print_div(c2);
   UART1_print_str(", ");
   UART1_print_div(c3);
   UART1_print_str(", ");
#endif // PRINT_PLOT_INA_1

#ifdef PRINT_PLOT_INA_2
   UART1_print(master_input.INA2_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA2_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA2_Ch3);
   UART1_print_str(", ");
#endif // PRINT_PLOT_INA_2

#ifdef PRINT_PLOT_INA_3
   UART1_print(master_input.INA3_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA3_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA3_Ch3);
   UART1_print_str(", ");
#endif // PRINT_PLOT_INA_3

#ifdef PRINT_PLOT_INA_4
   UART1_print(master_input.INA4_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA4_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA4_Ch3);
   UART1_print_str(", ");
#endif // PRINT_PLOT_INA_4

#ifdef PRINT_PLOT_INA_5
   UART1_print(master_input.INA5_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA5_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA5_Ch3);
   UART1_print_str(", ");
#endif // PRINT_PLOT_INA_5

#ifdef PRINT_PLOT_INA_6
   UART1_print(master_input.INA6_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA6_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA6_Ch3);
#endif // PRINT_PLOT_INA_6

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
         // printInputData();

         // if (c2 > 250)
         // {
         //    delay(1000);
         //    stop_flag = 1;
         // }

         //time2 = system_time;
         // convertPPMtoAngle();

         //UART1_print_str("T tr: ");
         //UART1_println(time2 - time1);
      }
   }
}
