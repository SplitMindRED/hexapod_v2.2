#include "splitmind_stm32f103_lib.h"
#include "hexapod.h"

//#define PRINT_DATA
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

#define START_BYTE_1       0xE7
#define START_BYTE_2       0x18
#define STOP_BYTE          0x18

// #define LED_ERROR          B3
// #define LED_WARNING        B4
// #define LED_BT_CONNECTION  B5

struct Master_output
{
   uint8_t servo[18];
}static master_output;

struct Master_input
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
}static master_input;

unsigned long current_interruption_time = 0;
uint16_t delta_interruption_time = 0;
uint8_t channel_counter = 0;
bool start_package = false;
uint16_t channel[6] = { 0, 0, 0, 0, 0, 0 };
bool servo_enable = false;

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
   UART1_print(master_input.INA1_Ch1);
   UART1_print_str(", ");
   UART1_print(master_input.INA1_Ch2);
   UART1_print_str(", ");
   UART1_print(master_input.INA1_Ch3);
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

void EXTI_init(void)
{
   //clock for PA0 pin and alternate function
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
   RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

   //GPIO PA0 pin initialization
   GPIO_InitTypeDef gpio_cfg;
   GPIO_StructInit(&gpio_cfg);

   gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
   gpio_cfg.GPIO_Pin = GPIO_Pin_11;    //change pin here
   GPIO_Init(GPIOA, &gpio_cfg);

   //connecting alternate funcion of PA0 pin to external interruptions
   AFIO->EXTICR[2] &= ~(AFIO_EXTICR3_EXTI11_PA);      //change EXTICR[group num] and pin

   //some preparations...
   EXTI->RTSR |= EXTI_RTSR_TR11;    //pin
   EXTI->FTSR |= EXTI_RTSR_TR11;    //pin
   EXTI->PR = EXTI_RTSR_TR11;       //pin
   EXTI->IMR |= EXTI_RTSR_TR11;     //pin


   //enable external interruptions of pin 0
   NVIC_EnableIRQ(EXTI15_10_IRQn);  //particular EXTI group for pin
}

//particular EXTI group for pin
void EXTI15_10_IRQHandler(void)//
{
   //if FRONT
   if (GPIOA->IDR & (1 << 11))
   {
      //get time
      current_interruption_time = system_time;
   }
   else
   {
      //if END -> evaluate delta
      delta_interruption_time = system_time - current_interruption_time;

      //if we have found start of package after 9100 mcs
      if (start_package == true)
      {
         //consistently store channel values to array 
         channel[channel_counter] = delta_interruption_time;
         channel_counter++;
         if (channel_counter == 6)
         {
            //when package is fully gathered
            start_package = false;
         }
         else if (channel_counter == 5)
         {
            //for emergency stop check 4 channel with new value
            if (channel[4] > 1100 && servo_enable == 1)
            {
               servo_enable = 0;
               UART1_println_str("Servo enable");
            }
            else if (channel[4] < 1100 && servo_enable == 0)
            {
               servo_enable = 1;
               UART1_println_str("Servo disable");
            }
            // digitalWrite(PORT_A, 12, servo_enable); //1-off 0-on     
            digitalWrite(PORT_C, 13, servo_enable); //1-led off, 0-led on				
         }
      }
      else if (delta_interruption_time > 5000)
      {
         //if long HIGH level detected -> we found start of the package
         start_package = true;
         channel_counter = 0;
      }
   }

   //reset interruption flag
   EXTI->PR = EXTI_PR_PR11;
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

   UART1_println_str("Done!");
}

void convertPPMtoAngle()
{
   //Vx = map(channel[0], 600, 1600, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);

   //float servo0 = channel[0] * 0.18 - 108;
   //float servo1 = channel[1] * 0.18 - 108;
   //float servo2 = channel[2] * 0.18 - 108;

   master_output.servo[0] = map(channel[0], 600, 1600, 30, 150);
   master_output.servo[9] = map(channel[2], 600, 1600, 30, 150);

   //float servo0 = map(channel[0], 600, 1600, 0, 700);


   //UART1_print_str("ppm: ");
   //UART1_print(channel[0]);
   //UART1_print_str(" angle: ");
   //UART1_println(master_output.servo[0]);
   //UART1_print_str(" angle float: ");
   //UART1_println(servo0);
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
         //unsigned long time1 = system_time;
         transferFrame();
         //unsigned long time2 = system_time;

         //time1 = system_time;
         // printInputData();
         //time2 = system_time;
         convertPPMtoAngle();

         //UART1_print_str("T tr: ");
         //UART1_println(time2 - time1);
      }
   }
}
