#include <Arduino.h>
#include <time_utils.h>
#include <spi_data_structure.h>
#include <MPU9250.h>
#include <INA3221.h>
#include <PCA9685.h>
#include <TCA9548A.h>
#include <Wire.h>

//ON/OFF debug info
#define SETUP_INFO
#define SERVO_PERIOD          500  //ms
#define SERVO_STEP_PERIOD     20   //ms
#define LOOP_PERIOD           20000    //mcs

#define DQ_MAX                1    //grad/step (10 grad per 20 ms)

//SPI PINS
#define SS_PIN                10
#define MOSI_PIN              11 
#define MISO_PIN              12 
#define SCK_PIN               13
#define ACK_PIN               9
#define OE_1                  3
#define OE_2                  4

//I2C pins
#define I2C_SCL               A5
#define I2C_SDA               A4

//MPU9250
#define MPU9250_ADDRESS       0x68

//SPI protocol
#define START_BYTE_1          0xE7
#define START_BYTE_2          0x18
#define FLAG_OE              (1 << 0)

//TCA9548A
#define TCA9548A_RESET_PIN    2
#define TCA9548A_ADDRESS      0x70
#define SWITCH_PCA9685        1
#define SWITCH_MPU9250        4

#define TCA9548A_SWITCH_0     0  //Servo board 1: INA_1, INA_2
#define TCA9548A_SWITCH_1     1  //Servo board 1: PCA_1, INA_3
#define TCA9548A_SWITCH_2     2  //Servo board 2: INA_4, INA_5
#define TCA9548A_SWITCH_3     3  //Servo board 2: PCA_2, INA_6
#define TCA9548A_SWITCH_4     4  //MPU9250

//PCA9685
#define PCA9685_ADDRESS_1     0x40
#define PCA9685_ADDRESS_2     0x40

//INA3221                     
#define INA3221_ADDRESS_1     0x41
#define INA3221_ADDRESS_2     0x42
#define INA3221_ADDRESS_3     0x43
#define INA3221_ADDRESS_4     0x41
#define INA3221_ADDRESS_5     0x42
#define INA3221_ADDRESS_6     0x43


//Frequences
#define I2C_CLOCK_FREQ        400000
#define UART_FREQUENCY        250000

PCA9685 pwm1(PCA9685_ADDRESS_1);
PCA9685 pwm2(PCA9685_ADDRESS_2);
INA3221 ina1(INA3221_ADDRESS_1);
INA3221 ina2(INA3221_ADDRESS_2);
INA3221 ina3(INA3221_ADDRESS_3);
INA3221 ina4(INA3221_ADDRESS_4);
INA3221 ina5(INA3221_ADDRESS_5);
INA3221 ina6(INA3221_ADDRESS_6);

TCA9548A tca(TCA9548A_ADDRESS, TCA9548A_RESET_PIN);
MPU9250 mpu(MPU9250_ADDRESS);
SoftwareTimer timer_servo(MILS);
SoftwareTimer timer_step_servo(MILS);
SoftwareTimer timer_main_loop(MICS);
Stopwatch watch_servo(MICS);
Stopwatch watch1(MICS);
Stopwatch watch2(MICS);
Stopwatch watch3(MICS);

SlaveInput slave_input;
SlaveOutput slave_output;

unsigned long system_time_mil = 0;
unsigned long system_time_mic = 0;
unsigned long stopwatch = 0;

bool is_OE = true;

void test()
{
   static uint8_t a[2];

   while (!(SPSR & (1 << SPIF)));
   a[0] = SPDR;

   while (!(SPSR & (1 << SPIF)));
   a[1] = SPDR;

   Serial.print(a[0]);
   Serial.print(" ");
   Serial.println(a[1]);
}

void test1()
{
   while (!(SPSR & (1 << SPIF)));

   Serial.print("Byte: ");
   Serial.println(SPDR);
}

void transferFrame()
{
   uint8_t* p_slave_output = (uint8_t*)&slave_output;
   uint8_t* p_slave_input = (uint8_t*)&slave_input;
   uint8_t tmp = 0;

   //ready for next byte
   digitalWrite(ACK_PIN, 0);

   //check if new byte arrived
   while (!(SPSR & (1 << SPIF)));

   //if (SPSR & (1 << SPIF))
   //{
   //byte recieved
   digitalWrite(ACK_PIN, 1);

   //check if it is first start byte
   if (SPDR == START_BYTE_1)
   {
      //ready for next byte
      digitalWrite(ACK_PIN, 0);

      //wait for incoming byte
      while (!(SPSR & (1 << SPIF)));
      //byte recieved
      digitalWrite(ACK_PIN, 1);

      //check if it is second start byte
      if (SPDR == START_BYTE_2)
      {
         for (uint16_t byte_counter = 0; byte_counter < 54; byte_counter++)
         {
            if (byte_counter < sizeof(slave_output))
            {
               SPDR = *p_slave_output++;
            }

            digitalWrite(ACK_PIN, 0);
            //wait for incoming byte
            while (!(SPSR & (1 << SPIF)));
            //byte recieved
            digitalWrite(ACK_PIN, 1);

            tmp = SPDR;

            if (byte_counter < sizeof(slave_input))
            {
               *p_slave_input++ = tmp;
            }
         }

         //static uint16_t errors = 0;
         //for (uint16_t byte_counter = 0; byte_counter < 54; byte_counter++)
         //{
         //   if (buffer[byte_counter] != byte_counter)
         //   {
         //      errors++;                  
         //   }
         //   buffer[byte_counter] = 0;
         //   //Serial.print("slave ");
         //   //Serial.print(byte_counter);
         //   //Serial.print(": ");
         //   //Serial.println(buffer[byte_counter]);
         //}
         //if (errors > 0)
         //{
         //   Serial.print("err ");
         //   Serial.println(errors);
         //}

         return;
      }

      Serial.println("TRASH!!!!!!!!!!!");
   }

   //byte recieved
   //digitalWrite(ACK_PIN, 1);
}

void printInputData()
{
   for (uint8_t servo_num = 0; servo_num < 18; servo_num++)
   {
      Serial.print("servo ");
      Serial.print(servo_num);
      Serial.print(": ");
      Serial.print(slave_input.servo[servo_num]);
      Serial.print(" ");
      Serial.println(map(slave_input.servo[servo_num], 287, 942, 15, 165));
   }

   Serial.print("OE: ");
   Serial.println(is_OE);
}

void printOutputData()
{
   Serial.print("AcX = "); Serial.print(slave_output.AcX);
   Serial.print(" | AcY = "); Serial.print(slave_output.AcY);
   Serial.print(" | AcZ = "); Serial.print(slave_output.AcZ);
   Serial.print(" | GyX = "); Serial.print(slave_output.GyX);
   Serial.print(" | GyY = "); Serial.print(slave_output.GyY);
   Serial.print(" | GyZ = "); Serial.print(slave_output.GyZ);

   Serial.print(" | 1_1 = "); Serial.print(slave_output.INA1_Ch1);
   Serial.print(" | 1_2 = "); Serial.print(slave_output.INA1_Ch2);
   Serial.print(" | 1_3 = "); Serial.print(slave_output.INA1_Ch3);

   Serial.print(" | 2_1 = "); Serial.print(slave_output.INA2_Ch1);
   Serial.print(" | 2_2 = "); Serial.print(slave_output.INA2_Ch2);
   Serial.print(" | 2_3 = "); Serial.print(slave_output.INA2_Ch3);

   Serial.print(" | 3_1 = "); Serial.print(slave_output.INA3_Ch1);
   Serial.print(" | 3_2 = "); Serial.print(slave_output.INA3_Ch2);
   Serial.print(" | 3_3 = "); Serial.print(slave_output.INA3_Ch3);

   Serial.print(" | 4_1 = "); Serial.print(slave_output.INA4_Ch1);
   Serial.print(" | 4_2 = "); Serial.print(slave_output.INA4_Ch2);
   Serial.print(" | 4_3 = "); Serial.print(slave_output.INA4_Ch3);

   Serial.print(" | 5_1 = "); Serial.print(slave_output.INA5_Ch1);
   Serial.print(" | 5_2 = "); Serial.print(slave_output.INA5_Ch2);
   Serial.print(" | 5_3 = "); Serial.print(slave_output.INA5_Ch3);

   Serial.print(" | 6_1 = "); Serial.print(slave_output.INA6_Ch1);
   Serial.print(" | 6_2 = "); Serial.print(slave_output.INA6_Ch2);
   Serial.print(" | 6_3 = "); Serial.println(slave_output.INA6_Ch3);
}

void checkData()
{
   static uint8_t counter = 0;

   for (uint8_t byte_counter = 0; byte_counter < 18; byte_counter++)
   {
      if (slave_input.servo[byte_counter] != byte_counter * 2)
      {
         counter++;
      }
      slave_input.servo[byte_counter] = 0;
   }

   Serial.print("Error counter: ");
   Serial.println(counter);
}

void AllServoTest()
{
   // �������� ������������ �� �������� � ��������
   for (uint16_t i = 0; i < 16; i++)
   {
      pwm1.setServoAngle(i, 180);
   }
   Serial.println("Max");
   delay(500);

   // �������� ������������ �� ��������� � �������
   for (uint16_t i = 0; i < 16; i++)
   {
      pwm1.setServoAngle(i, 0);
   }
   Serial.println("Min");
   delay(500);
}

void servoTest()
{
   //uint8_t pause = 10;

   //for (uint16_t i = 0; i < 180; i++)
   //{
   //   pwm1.setServoAngle(8, i);
   //   INA3221_getCurrent();
   //   //delay(pause);
   //}
   //delay(500);

   //for (uint16_t i = 180; i > 0; i--)
   //{
   //   pwm1.setServoAngle(8, i);
   //   INA3221_getCurrent();
   //   //delay(pause);
   //}
   //delay(500);

   static bool flag = 0;

   if (timer_servo.check())
   {
      watch_servo.start();
      flag = !flag;
      pwm1.setServoAngle(8, 180 * flag);
      pwm1.setServoAngle(7, 180 * flag);
      pwm1.setServoAngle(6, 180 * flag);
      watch_servo.stop();

      Serial.print("dt in micros: ");
      Serial.print(watch_servo.getDelta());

      Serial.print(" servo cmd time: ");
      Serial.println(timer_servo.time_start);
   }
}

void servoBurstTest()
{
   static bool flag = 0;
   uint8_t angle_array[18];

   if (timer_servo.check())
   {
      watch_servo.start();
      flag = !flag;
      for (uint8_t i = 0; i < 18; i++)
      {
         //angle_array[i] = 180 * flag;
         angle_array[i] = 150 * flag + 30 * !flag;
      }

      //for (uint8_t i = 0; i < 9; i++)
      //{
      //   angle_array[i] = slave_input.servo[0];
      //}
      //for (uint8_t i = 9; i < 18; i++)
      //{
      //   angle_array[i] = slave_input.servo[9];
      //}

      tca.selectLine(TCA9548A_SWITCH_1);
      pwm1.burstSetServoAngles(angle_array);

      tca.selectLine(TCA9548A_SWITCH_3);
      pwm2.burstSetServoAngles(angle_array + 9);
      watch_servo.stop();

      //Serial.print("dt in micros: ");
      //Serial.print(watch_servo.getDelta());

      //Serial.print(" servo cmd time: ");
      //Serial.println(timer_servo.time_start);
   }
}

void servoStep()
{
   static uint8_t angle_array[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
   // static bool direction = true;

   if (timer_step_servo.check())
   {
      //watch_servo.start();

      //if (angle_array[0] < 150 && direction == true)
      //{
      //   angle_array[0] += DQ_MAX;

      //   if (angle_array[0] > 150)
      //   {
      //      angle_array[0] = 150;
      //   }
      //}
      //else if (angle_array[0] >= 150 && direction == true)
      //{
      //   direction = false;
      //}

      //if (angle_array[0] > 30 && direction == false)
      //{
      //   angle_array[0] -= DQ_MAX;

      //   if (angle_array[0] < 30)
      //   {
      //      angle_array[0] = 30;
      //   }
      //}
      //else if (angle_array[0] <= 30 && direction == false)
      //{
      //   direction = true;
      //   angle_array[0] += DQ_MAX;
      //}

      angle_array[0] = slave_input.servo[0];
      angle_array[1] = 90;
      angle_array[2] = 90;

      tca.selectLine(TCA9548A_SWITCH_1);
      pwm1.burstSetServoAngles(angle_array);

      //tca.selectLine(TCA9548A_SWITCH_3);
      //pwm2.burstSetServoAngles(angle_array + 9);
      //watch_servo.stop();

      Serial.print("ang 0: ");
      Serial.println(angle_array[0]);
   }
}

void servoControl()
{
   if (slave_input.flags & FLAG_OE)
   {
      //off
      is_OE = true;
      // Serial.println("Servo disable");
   }
   else
   {
      //on
      is_OE = false;
      // Serial.println("Servo enable");
   }

   digitalWrite(OE_1, is_OE);
   digitalWrite(OE_2, is_OE);

   // static uint8_t servo_angles_1[9] = { 90, 90, 90, 90, 90, 90, 90, 90, 90 };
   // static uint8_t servo_angles_2[9] = { 90, 90, 90, 90, 90, 90, 90, 90, 90 };
   static uint16_t servo_pwm_1[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
   static uint16_t servo_pwm_2[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

   // servo_angles_1[0] = slave_input.servo[11];
   // servo_angles_1[1] = 180 - slave_input.servo[10];
   // servo_angles_1[2] = slave_input.servo[9];
   // servo_angles_1[3] = 180 - slave_input.servo[8];
   // servo_angles_1[4] = slave_input.servo[7];
   // servo_angles_1[5] = slave_input.servo[6];
   // servo_angles_1[6] = 180 - slave_input.servo[5];
   // servo_angles_1[7] = slave_input.servo[4];
   // servo_angles_1[8] = slave_input.servo[3];

   // servo_angles_2[0] = slave_input.servo[0];
   // servo_angles_2[1] = slave_input.servo[1];
   // servo_angles_2[2] = 180 - slave_input.servo[2];
   // servo_angles_2[3] = slave_input.servo[17];
   // servo_angles_2[4] = 180 - slave_input.servo[16];
   // servo_angles_2[5] = slave_input.servo[15];
   // servo_angles_2[6] = slave_input.servo[14];
   // servo_angles_2[7] = 180 - slave_input.servo[13];
   // servo_angles_2[8] = slave_input.servo[12];

   servo_pwm_1[1] = slave_input.servo[11];
   servo_pwm_1[3] = SERVOMAX + SERVOMIN - slave_input.servo[10];
   servo_pwm_1[5] = slave_input.servo[9];
   servo_pwm_1[7] = SERVOMAX + SERVOMIN - slave_input.servo[8];
   servo_pwm_1[9] = slave_input.servo[7];
   servo_pwm_1[11] = slave_input.servo[6];
   servo_pwm_1[13] = SERVOMAX + SERVOMIN - slave_input.servo[5];
   servo_pwm_1[15] = slave_input.servo[4];
   servo_pwm_1[17] = slave_input.servo[3];

   servo_pwm_2[1] = slave_input.servo[0];
   servo_pwm_2[3] = slave_input.servo[1];
   servo_pwm_2[5] = SERVOMAX + SERVOMIN - slave_input.servo[2];
   servo_pwm_2[7] = slave_input.servo[17];
   servo_pwm_2[9] = SERVOMAX + SERVOMIN - slave_input.servo[16];
   servo_pwm_2[11] = slave_input.servo[15];
   servo_pwm_2[13] = slave_input.servo[14];
   servo_pwm_2[15] = SERVOMAX + SERVOMIN - slave_input.servo[13];
   servo_pwm_2[17] = slave_input.servo[12];

   if (is_OE == false)
   {
      tca.selectLine(TCA9548A_SWITCH_1);
      // pwm1.burstSetServoAngles((uint8_t*)&servo_angles_1);
      pwm1.burstSetPWM((uint16_t*)&servo_pwm_1);

      tca.selectLine(TCA9548A_SWITCH_3);
      // pwm2.burstSetServoAngles((uint8_t*)&servo_angles_2);
      pwm2.burstSetPWM((uint16_t*)&servo_pwm_2);
   }
}

void setup(void)
{
   system_time_mic = micros();
   system_time_mil = millis();

   Serial.begin(UART_FREQUENCY);

#ifdef SETUP_INFO
   Serial.println("Setup...");
#endif

   //pins for output enble of PCA9685
   pinMode(OE_1, OUTPUT);
   pinMode(OE_2, OUTPUT);

   digitalWrite(OE_1, is_OE);
   digitalWrite(OE_2, is_OE);

   //pins for SPI
   pinMode(MOSI_PIN, INPUT);
   pinMode(MISO_PIN, OUTPUT);
   pinMode(SCK_PIN, INPUT);
   pinMode(SS_PIN, INPUT);

   //pin for slave acknowledge
   pinMode(ACK_PIN, OUTPUT);

   Wire.begin();
   Wire.setClock(I2C_CLOCK_FREQ);

   tca.selectLine(SWITCH_MPU9250);
   mpu.init();
   delay(1);

   //servo board 1------------------------
   tca.selectLine(TCA9548A_SWITCH_0);
   ina1.init();
   ina2.init();

   tca.selectLine(TCA9548A_SWITCH_1);
   ina3.init();
   pwm1.init();
   //-------------------------------------

   //servo board 2------------------------
   tca.selectLine(TCA9548A_SWITCH_2);
   ina4.init();
   ina5.init();

   tca.selectLine(TCA9548A_SWITCH_3);
   ina6.init();
   pwm2.init();
   //-------------------------------------

   timer_servo.time_start = 0;
   timer_servo.delay = SERVO_PERIOD;

   timer_step_servo.time_start = 0;
   timer_step_servo.delay = SERVO_STEP_PERIOD;

   timer_main_loop.time_start = 0;
   timer_main_loop.delay = LOOP_PERIOD;

   if (sizeof(slave_input) > sizeof(slave_output))
   {
      bigger_struct_size = sizeof(slave_input);
   }
   else
   {
      bigger_struct_size = sizeof(slave_output);
   }
   Serial.print("Bigger struct size: ");
   Serial.println(bigger_struct_size);

   //SPI enable
   SPCR = (1 << SPE);

   slave_output.AcX = 1;
   slave_output.AcY = 2;
   slave_output.AcZ = 3;

   slave_output.GyX = 4;
   slave_output.GyY = 5;
   slave_output.GyZ = 6;

   slave_output.MaX = 7;
   slave_output.MaY = 8;
   slave_output.MaZ = 9;

   slave_output.INA1_Ch1 = 1;
   slave_output.INA1_Ch2 = 2;
   slave_output.INA1_Ch3 = 3;

   slave_output.INA2_Ch1 = 4;
   slave_output.INA2_Ch2 = 5;
   slave_output.INA2_Ch3 = 6;

   slave_output.INA3_Ch1 = 7;
   slave_output.INA3_Ch2 = 8;
   slave_output.INA3_Ch3 = 9;

   slave_output.INA4_Ch1 = 10;
   slave_output.INA4_Ch2 = 11;
   slave_output.INA4_Ch3 = 12;

   slave_output.INA5_Ch1 = 13;
   slave_output.INA5_Ch2 = 14;
   slave_output.INA5_Ch3 = 15;

   slave_output.INA6_Ch1 = 16;
   slave_output.INA6_Ch2 = 17;
   slave_output.INA6_Ch3 = 18;

   stopwatch = micros();

   //uint16_t a = 0xc180;
   //uint16_t b = a;
   //b &= ~(1 << 15);
   //Serial.print(" b: ");
   //Serial.println(b);

   //b = b - 1;
   //Serial.print(" b: ");
   //Serial.println(b);

   //b = ~b;
   //Serial.print(" b: ");
   //Serial.println(b);

   //b = b >> 3;
   //Serial.print(" b: ");
   //Serial.println(b);

   //b = b * 40 / 1000;

   //Serial.print("a: ");
   //Serial.print(a);
   //Serial.print(" b: ");
   //Serial.println(b);

   //int16_t r = 0xc180;
   //int16_t t = (uint8_t)r | ((uint8_t)(r >> 8)<<8);
   //int16_t t2;

   //uint8_t* p = (uint8_t*)&t2;
   //*p++ = (uint8_t)r;
   //*p = (uint8_t)(r >> 8);


   //Serial.print("r: ");
   //Serial.print(r);
   //Serial.print(" t: ");
   //Serial.print(t);
   //Serial.print(" t2: ");
   //Serial.println(t2);

#ifdef SETUP_INFO
   Serial.print("Time init in micros: ");
   Serial.println(stopwatch - system_time_mic);

   Serial.println("Done!");
#endif
}

void loop(void)
{
   //AllServoTest();
   //servoTest();
   //servoBurstTest();
   // servoStep();

   if (timer_main_loop.check())
   {
      watch1.time_stop = watch1.time_start;
      watch1.start();
      tca.selectLine(TCA9548A_SWITCH_0);
      ina1.getCurrent(((uint8_t*)&slave_output) + 18, 0);
      ina2.getCurrent(((uint8_t*)&slave_output) + 18 + 6, 0);

      tca.selectLine(TCA9548A_SWITCH_1);
      ina3.getCurrent(((uint8_t*)&slave_output) + 18 + 12, 0);

      tca.selectLine(TCA9548A_SWITCH_2);
      ina4.getCurrent(((uint8_t*)&slave_output) + 18 + 18, 0);
      ina5.getCurrent(((uint8_t*)&slave_output) + 18 + 24, 0);

      tca.selectLine(TCA9548A_SWITCH_3);
      ina6.getCurrent(((uint8_t*)&slave_output) + 18 + 30, 1);
      // watch1.stop();

      // watch2.start();
      tca.selectLine(SWITCH_MPU9250);
      mpu.getData((uint8_t*)&slave_output, 14);
      // watch2.stop();

      // watch3.start();
      transferFrame();
      // watch3.stop();

      servoControl();

      printInputData();
      //printOutputData();

      //Serial.print("t: ");
      //Serial.print(micros());

      //Serial.print(" cur: ");
      //Serial.print(watch1.getDelta());

      //Serial.print(" mpu: ");
      //Serial.print(watch2.getDelta());

      //Serial.print(" tr: ");
      //Serial.println(watch3.getDelta());

      Serial.print("t_st: ");
      Serial.print(watch1.time_start);
      Serial.print(": ");
      Serial.println(watch1.time_start - watch1.time_stop);
   }
}
