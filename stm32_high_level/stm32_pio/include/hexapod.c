#include "hexapod.h"

uint16_t servomin = 0;
uint16_t servomax = 0;

//local variables
unsigned long current_interruption_time = 0;
uint16_t delta_interruption_time = 0;
uint8_t channel_counter = 0;
bool start_package = false;
uint16_t channel[6] = { 0, 0, 0, 0, 0, 0 };
bool servo_enable = true;

float Vx = 0, Vy = 0, Vz = 0;
float Wz = 0;
float input_roll = 0, input_pitch = 0, input_yaw = 0;
float current_roll = 0, current_pitch = 0, current_yaw = 0;
double RadX = 0, RadY = 0, RadZ = 0, GradX = 0, GradY = 0, GradZ = 0;
double AcX = 0, AcY = 0, AcZ = 0, GyX = 0, GyY = 0, GyZ = 0;
double servo_current[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//geometry variables--------------------------
// const uint8_t OA = 37;
// const float AB = 44;
// const float BC = 83;

const float OA = 41.5;		//ver 2
const float AB = 44;
// const float BC = 81.32;
const float BC = 85;
double pi = 3.14159;
double q0rad, q1rad, q2rad, Qrad, Q0rad;
double q0, q1, q2;
////--------------------------------------------

struct Legs Leg[6];

//local start points AS iS
int16_t local_start_point[6][3] =
{
    {X_OFFSET,      -Y_OFFSET, -STARTHEIGHT},
    {X_OFFSET + 30,  0,        -STARTHEIGHT},
    {X_OFFSET,       Y_OFFSET, -STARTHEIGHT},
    {-X_OFFSET,      Y_OFFSET, -STARTHEIGHT},
    {-X_OFFSET - 30, 0,        -STARTHEIGHT},
    {-X_OFFSET,     -Y_OFFSET, -STARTHEIGHT},
};

int16_t local_stabilization_point[6][3]
{
   {X_OFFSET,      -Y_OFFSET, -STARTHEIGHT},
   {X_OFFSET + 30,  0,        -STARTHEIGHT},
   {X_OFFSET,       Y_OFFSET, -STARTHEIGHT},
   {-X_OFFSET,      Y_OFFSET, -STARTHEIGHT},
   {-X_OFFSET - 30, 0,        -STARTHEIGHT},
   {-X_OFFSET,     -Y_OFFSET, -STARTHEIGHT},
};


//coordinates translation to leg systems
float leg_translation[6][3] =
{
    {X_TRANSLATION,     -Y_TRANSLATION, 0},
    {X_TRANSLATION_MID,  0,             0},
    {X_TRANSLATION,      Y_TRANSLATION, 0},
    {-X_TRANSLATION,     Y_TRANSLATION, 0},
    {-X_TRANSLATION_MID, 0,             0},
    {-X_TRANSLATION,    -Y_TRANSLATION, 0},
};

float diameter = DIAMETER;                   //60 mm amplitude in step

float k = 0;
float dH = DELTAHEIGHT;
float H = STARTHEIGHT;

//array for storing evaluated angles 
//(in hexapod v2.2 this is pointer to master_output structure)
uint8_t* p_angle_array;

unsigned long next_time = 0;

void hexapodInit(uint8_t* l_p_angle_array)
{
   uint16_t a_tmp = 1000000 / PWM_FREQ;
   float c_tmp = (float)a_tmp / 4096;

   servomin = (700.0 / c_tmp);
   servomax = (2300.0 / c_tmp);

   p_angle_array = l_p_angle_array;

   for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
   {
      Leg[leg_num].start_x = local_start_point[leg_num][0];
      Leg[leg_num].start_y = local_start_point[leg_num][1];
      Leg[leg_num].start_z = local_start_point[leg_num][2];

      if (leg_num == 1 || leg_num == 4)
      {
         Leg[leg_num].tip_radius = X_TRANSLATION_MID + X_OFFSET + 30;
      }
      else
      {
         Leg[leg_num].tip_radius = sqrt((X_TRANSLATION + X_OFFSET) * (X_TRANSLATION + X_OFFSET) + (Y_TRANSLATION + Y_OFFSET) * (Y_TRANSLATION + Y_OFFSET));
      }

      switch (leg_num)
      {
      case 0:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = +X_OFFSET;
         Leg[leg_num].tf_step_footprint_y = -Y_OFFSET;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = +X_TRANSLATION;
         Leg[leg_num].tf_leg_link_y = -Y_TRANSLATION;
         break;

      case 1:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = +X_OFFSET + 30;
         Leg[leg_num].tf_step_footprint_y = 0;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = X_TRANSLATION_MID;
         Leg[leg_num].tf_leg_link_y = 0;
         break;

      case 2:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = +X_OFFSET;
         Leg[leg_num].tf_step_footprint_y = +Y_OFFSET;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = +X_TRANSLATION;
         Leg[leg_num].tf_leg_link_y = +Y_TRANSLATION;
         break;

      case 3:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = -X_OFFSET;
         Leg[leg_num].tf_step_footprint_y = +Y_OFFSET;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = -X_TRANSLATION;
         Leg[leg_num].tf_leg_link_y = +Y_TRANSLATION;
         break;

      case 4:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = -X_OFFSET - 30;
         Leg[leg_num].tf_step_footprint_y = 0;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = -X_TRANSLATION_MID;
         Leg[leg_num].tf_leg_link_y = 0;
         break;

      case 5:
         //tf from step footprint to leg link
         Leg[leg_num].tf_step_footprint_x = -X_OFFSET;
         Leg[leg_num].tf_step_footprint_y = -Y_OFFSET;

         //tf from leg link to base link
         Leg[leg_num].tf_leg_link_x = -X_TRANSLATION;
         Leg[leg_num].tf_leg_link_y = -Y_TRANSLATION;
         break;
      }

      //set all legs start position
      moveLeg(leg_num, Leg[leg_num].start_x, Leg[leg_num].start_y, Leg[leg_num].start_z);
   }
}

void moveLeg(uint8_t leg_num, double x, double y, double z)
{
   findAngles(leg_num, x, y, z);

   // setServoAngle(leg_num * 3, (uint8_t)Leg[leg_num].q0);
   // setServoAngle(leg_num * 3 + 1, (uint8_t)Leg[leg_num].q1);
   // setServoAngle(leg_num * 3 + 2, (uint8_t)Leg[leg_num].q2);

   setServoAngle(leg_num * 3, evalPWM(Leg[leg_num].q0));
   setServoAngle(leg_num * 3 + 1, evalPWM(Leg[leg_num].q1));
   setServoAngle(leg_num * 3 + 2, evalPWM(Leg[leg_num].q2));

   // UART1_print_str("s0: ");
   // UART1_print_div(Leg[leg_num].q0);
   // UART1_print_str(" s1: ");
   // UART1_print_div(Leg[leg_num].q1);
   // UART1_print_str(" s2: ");
   // UART1_println_div(Leg[leg_num].q2);

   Leg[leg_num].current_x = x;
   Leg[leg_num].current_y = y;
   Leg[leg_num].current_z = z;
}

uint16_t evalPWM(float angle)
{
   return (uint16_t)(servomin + (servomax - servomin) * (angle - 15.0) / 150.0); //mg90d
}

uint8_t evalSum(uint8_t* p_array, uint8_t size)
{
   uint8_t result = 0;

   for (uint8_t i = 0; i < size; i++)
   {
      result = result + *p_array;
      p_array++;
   }

   return result;
}

void evalCurrent(void)
{
   servo_current[0] = (double)master_input.INA4_Ch3 * 5 / 100;
   servo_current[1] = (double)master_input.INA4_Ch2 * 5 / 100;
   servo_current[2] = (double)master_input.INA4_Ch1 * 5 / 100;

   servo_current[3] = (double)master_input.INA3_Ch1 * 5 / 100;
   servo_current[4] = (double)master_input.INA3_Ch2 * 5 / 100;
   servo_current[5] = (double)master_input.INA3_Ch3 * 5 / 100;

   servo_current[6] = (double)master_input.INA2_Ch1 * 5 / 100;
   servo_current[7] = (double)master_input.INA2_Ch2 * 5 / 100;
   servo_current[8] = (double)master_input.INA2_Ch3 * 5 / 100;

   servo_current[9] = (double)master_input.INA1_Ch1 * 5 / 100;
   servo_current[10] = (double)master_input.INA1_Ch2 * 5 / 100;
   servo_current[11] = (double)master_input.INA1_Ch3 * 5 / 100;

   servo_current[12] = (double)master_input.INA6_Ch1 * 5 / 100;
   servo_current[13] = (double)master_input.INA6_Ch2 * 5 / 100;
   servo_current[14] = (double)master_input.INA6_Ch3 * 5 / 100;

   servo_current[15] = (double)master_input.INA5_Ch1 * 5 / 100;
   servo_current[16] = (double)master_input.INA5_Ch2 * 5 / 100;
   servo_current[17] = (double)master_input.INA5_Ch3 * 5 / 100;
}

bool checkSum(uint8_t source_sum, uint8_t* p_array, uint8_t size)
{
   uint8_t new_sum = 0;

   new_sum = evalSum(p_array, size);

   if (source_sum == new_sum)
   {
      return true;
   }
   else
   {
      return false;
   }
}

void setServoAngle(uint8_t servo_num, uint16_t PWM)
{
   //put angle to master output structure
   // master_output.servo[servo_num] = Q;

   //from grad to pwm
   master_output.servo[servo_num] = PWM;
}

void servoManualControl(void)
{
   for (int i = 0; i < 18; i += 3)
   {
      // uint8_t servo0 = map(channel[0], 600, 1600, 15, 165);
      // uint8_t servo1 = map(channel[1], 600, 1600, 15, 165);
      // uint8_t servo2 = map(channel[2], 600, 1600, 15, 165);

      float servo0 = map(channel[0], 600, 1600, servomin, servomax);
      float servo1 = map(channel[1], 600, 1600, servomin, servomax);
      float servo2 = map(channel[2], 600, 1600, servomin, servomax);

      // UART1_print_str("s0: ");
      // UART1_print_div(servo0);
      // UART1_print_str(" s1: ");
      // UART1_print_div(servo1);
      // UART1_print_str(" s2: ");
      // UART1_println_div(servo2);

      setServoAngle(i, servo0);
      setServoAngle(i + 1, servo1);
      setServoAngle(i + 2, servo2);
   }
}

void switchMode(void)
{
   //SWC switch mode
   if (channel[5] > 1300)                                //low
   {
      // servoManualControl();
      stabilizationMode(1);
   }
   else if (channel[5] < 1200 && channel[5] > 900)       //mid
   {
      //heightTest(H);
      rotateBody(input_roll, input_pitch, input_yaw, 1);

      // new_version();
      // legManualControl(2);

      // moveLeg(0, 110, -110, 0);
      // moveLeg(1, 155, 0, 0);
      // moveLeg(2, 110, 110, 0);
      // moveLeg(3, -110, 110, 0);
      // moveLeg(4, -155, 0, 0);
      // moveLeg(5, -110, -110, 0);
   }
   else if (channel[5] < 700)                            //high
   {
      // hexapodMove();
      // newVersion();
      // senseTest();
      senseTest1();
   }
}

void convertFlySkyData(void)
{
   Vx = map(channel[0], 600, 1600, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);

   if (fabs(Vx) < 20)
   {
      Vx = 0;
   }

   Vy = map(channel[1], 600, 1600, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);

   if (fabs(Vy) < 20)
   {
      Vy = 0;
   }

   Wz = map(channel[3], 600, 1600, MAX_VEL_ANGULAR, -MAX_VEL_ANGULAR);

   //if (system_time >= next_time)
   //{
   //	UART1_print_str("Wz: ");
   //	UART1_println_div(Wz);
   //}	

   H = map(channel[2], 600, 1600, 70, 110);
   dH = 30 * H * 1.1 / 70;

   k = 4 * dH / (diameter * diameter);

   input_roll = map(channel[0], 600, 1600, 0.35, -0.35);    //right -
   input_pitch = map(channel[1], 600, 1600, 0.35, -0.35);   //up +
   input_yaw = map(channel[3], 600, 1600, 0.35, -0.35);     //right -

   if (fabs(input_roll) < 0.02)
   {
      input_roll = 0;
   }

   //servo off when OE low    
   if (servo_enable == true)
   {
      master_output.flags |= FLAG_OE;
   }
   else
   {
      master_output.flags &= ~FLAG_OE;
   }

#ifdef SHOW_RPY
   UART1_print_str("roll: ");
   UART1_print_div(input_roll);
   UART1_print_str(" pitch: ");
   UART1_print_div(input_pitch);
   UART1_print_str(" yaw: ");
   UART1_println_div(input_yaw);
#endif

   // Vx = map(channel[0], 600, 1600, -MAX_VEL_LINEAR, MAX_VEL_LINEAR);

   // float servo0 = channel[0] * 0.18 - 108;
   // float servo1 = channel[1] * 0.18 - 108;
   // float servo2 = channel[2] * 0.18 - 108;

   // master_output.servo[0] = map(channel[0], 600, 1600, 30, 150);
   // master_output.servo[9] = map(channel[2], 600, 1600, 30, 150);

   // float servo0 = map(channel[0], 600, 1600, 0, 700);

   // UART1_print_str("ppm: ");
   // UART1_print(channel[0]);
   // UART1_print_str(" angle: ");
   // UART1_println(master_output.servo[0]);
   // UART1_print_str(" angle float: ");
   // UART1_println(servo0);
}

void stabilizationMode(bool is_move)
{
   static double qx = 0;
   static double qy = 0;
   static double qz = 0;

   static double qx_sum = 0;
   static double qy_sum = 0;
   static double qz_sum = 0;

   static const uint8_t avr = 10;
   static bool is_arr_full = false;

   static double qx_arr[avr];
   static double qy_arr[avr];
   static double qz_arr[avr];

   static uint16_t counter = 0;

   // UART1_print_str("RadX: ");
   // UART1_print_div(RadX);
   // UART1_print_str(" RadX: ");
   // UART1_println_div(RadY);

   //first filling
   if (is_arr_full == false)
   {
      qx_arr[counter] = RadX;
      qy_arr[counter] = RadY;
      qz_arr[counter] = RadZ;

      counter++;

      if (counter == avr)
      {
         is_arr_full = true;
         counter = 0;

         for (uint16_t i = 0; i < avr; i++)
         {
            qx_sum += qx_arr[i];
            qy_sum += qy_arr[i];
            qz_sum += qz_arr[i];
         }

         qx = qx_sum / (double)avr;
         qy = qy_sum / (double)avr;
         qz = qz_sum / (double)avr;

         // UART1_print_str("QavrX: ");
         // UART1_print_div(qx);
         // UART1_print_str(" QavrY: ");
         // UART1_print_div(qy);
         // UART1_print_str(" QavrZ: ");
         // UART1_print_div(qz);
         // UART1_print_str(" RadX: ");
         // UART1_print_div(RadX);
         // UART1_print_str(" RadY: ");
         // UART1_print_div(RadY);
         // UART1_print_str(" RadZ: ");
         // UART1_println_div(RadZ);

         // rotateBody(-qx, qy, qz, is_move);
         rotateBody(-qx, qy, 0.0, is_move);

         qx_sum -= qx_arr[counter];
         qy_sum -= qy_arr[counter];
         qz_sum -= qz_arr[counter];
      }
   }
   else
   {
      qx_arr[counter] = RadX;
      qy_arr[counter] = RadY;
      qz_arr[counter] = RadZ;

      qx_sum += qx_arr[counter];
      qy_sum += qy_arr[counter];
      qz_sum += qz_arr[counter];

      qx = qx_sum / (double)avr;
      qy = qy_sum / (double)avr;
      qz = qz_sum / (double)avr;

      // UART1_print_str("QavrX: ");
      // UART1_print_div(qx);
      // UART1_print_str(" QavrY: ");
      // UART1_print_div(qy);
      // UART1_print_str(" QavrZ: ");
      // UART1_print_div(qz);
      // UART1_print_str(" RadX: ");
      // UART1_print_div(RadX);
      // UART1_print_str(" RadY: ");
      // UART1_print_div(RadY);
      // UART1_print_str(" RadZ: ");
      // UART1_println_div(RadZ);

      // rotateBody(-qx, qy, qz, is_move);
      rotateBody(-qx, qy, 0.0, is_move);

      counter++;

      if (counter == avr)
      {
         counter = 0;
      }

      qx_sum -= qx_arr[counter];
      qy_sum -= qy_arr[counter];
      qz_sum -= qz_arr[counter];
   }
}

void findAngles(uint8_t leg_num, double x, double y, double z)
{
   double p = 0.0;
   //double OC = 0.0;
   double AC = 0.0;

   p = sqrt(x * x + y * y);
   //OC = sqrt(p * p + z * z);
   AC = sqrt((p - OA) * (p - OA) + z * z);

   Qrad = atan(z / (p - OA));
   Q0rad = acos((AB * AB + AC * AC - BC * BC) / (2 * AB * AC));

   if (x == 0)
   {
      q0rad = 0;
   }
   else
   {
      //right side
      if (leg_num <= 2)
      {
         q0rad = atan2(y, x);
      }
      //left side
      else
      {
         if (x < 0)
         {
            q0rad = atan(y / x) + pi;
         }
         else
         {
            if (leg_num == 5)
            {
               q0rad = -atan(y / x) + pi; //FIX THIS SHIT!
            }
            else
            {
               q0rad = atan(y / x);
            }
         }
      }
   }

   switch (leg_num)
   {
   case 0:
      q0 = q0rad * 180 / pi + 135;		//back right
      break;

   case 1:
      q0 = q0rad * 180 / pi + 90;		//mid right
      break;

   case 2:
      q0 = q0rad * 180 / pi + 45;		//front right
      break;

   case 3:
      q0 = q0rad * 180 / pi - 45;		//front left
      break;

   case 4:
      q0 = q0rad * 180 / pi - 90;		//mid left
      break;

   case 5:
      q0 = q0rad * 180 / pi - 135;		//back left
      break;
   }

   //q0 = q0rad * 180 / pi + 45;		//front right
   //q0 = q0rad * 180 / pi + 96;		//mid right
   //q0 = q0rad * 180 / pi + 135;	//back right
   //q0 = q0rad * 180 / pi - 45;		//front left
   //q0 = q0rad * 180 / pi - 90;		//mid left
   //q0 = q0rad * 180 / pi - 135;	//back left

   q1rad = Qrad + Q0rad;
   q1 = q1rad * 180 / pi + 90;

   q2rad = acos((AB * AB + BC * BC - AC * AC) / (2 * AB * BC));
   q2 = q2rad * 180 / pi;

   Leg[leg_num].q0 = q0;
   Leg[leg_num].q1 = q1;
   Leg[leg_num].q2 = q2;
}

void rotatePoint(double* x, double* y, double q)
{
   double x0 = *x;
   double y0 = *y;

   *x = x0 * cos(q) - y0 * sin(q);
   *y = x0 * sin(q) + y0 * cos(q);
}

double getAngle(double x, double y)
{
   double q = 0.0;

   if (x == 0)
   {
      //90
      if (y >= 0)
      {
         q = pi / 2;
      }
      //-90
      else
      {
         q = -pi / 2;
      }
   }
   else if (x < 0)
   {
      //-90 ... -180
      if (y < 0)
      {
         q = -pi + atan(y / x);
      }
      //90 ... 180
      else
      {
         q = pi + atan(y / x);
      }
   }
   //0 ... 90
   else
   {
      q = atan(y / x);
   }

   return q;
}

bool phaseControl(uint8_t leg_num)
{
   float X, Y;
   float x, y, r;

   X = Leg[leg_num].current_x;
   Y = Leg[leg_num].current_y;

   //parameters in circle formula (x+x0)^2 + (y+y0)^2 = r^2
   //circle with center (1,1) -> (x-1)^2 + (y-1)^2 = r^2
   switch (leg_num)
   {
   case 0:
      x = X - X_OFFSET;
      y = Y + Y_OFFSET;
      break;

   case 1:
      x = X - X_OFFSET - 30;
      y = Y;
      break;

   case 2:
      x = X - X_OFFSET;
      y = Y - Y_OFFSET;
      break;

   case 3:
      x = X + X_OFFSET;
      y = Y - Y_OFFSET;
      break;

   case 4:
      x = X + X_OFFSET + 30;
      y = Y;
      break;

   case 5:
      x = X + X_OFFSET;
      y = Y + Y_OFFSET;
      break;
   }

   r = diameter / 2;

   //check if current point is outside of circle
   if (x * x + y * y >= r * r)
   {
      Leg[leg_num].phase = !Leg[leg_num].phase;
   }

   return Leg[leg_num].phase;
}

void rotateBody(double Qx, double Qy, double Qz, bool is_move)
{
   //leg tips's coordinates in central CS (coordinate system)
   float p_base[6][3];
   float p_base_new[6][3];
   float temp_x[6];
   float p_delta[6][3];

   //right back leg
   //X
   p_base[0][0] = local_start_point[0][0] + X_TRANSLATION;
   //Y
   p_base[0][1] = local_start_point[0][1] - Y_TRANSLATION;
   //Z
   p_base[0][2] = local_start_point[0][2];

   //right middle leg
   //X
   p_base[1][0] = local_start_point[1][0] + X_TRANSLATION_MID;
   //Y
   p_base[1][1] = local_start_point[1][1];
   //Z
   p_base[1][2] = local_start_point[1][2];

   //right front leg
   //X
   p_base[2][0] = local_start_point[2][0] + X_TRANSLATION;
   //Y
   p_base[2][1] = local_start_point[2][1] + Y_TRANSLATION;
   //Z
   p_base[2][2] = local_start_point[2][2];

   //left front leg
   //X
   p_base[3][0] = local_start_point[3][0] - X_TRANSLATION;
   //Y
   p_base[3][1] = local_start_point[3][1] + Y_TRANSLATION;
   //Z
   p_base[3][2] = local_start_point[3][2];

   //left middle leg
   //X
   p_base[4][0] = local_start_point[4][0] - X_TRANSLATION_MID;
   //Y
   p_base[4][1] = local_start_point[4][1];
   //Z
   p_base[4][2] = local_start_point[4][2];

   //left back leg
   //X
   p_base[5][0] = local_start_point[5][0] - X_TRANSLATION;
   //Y
   p_base[5][1] = local_start_point[5][1] - Y_TRANSLATION;
   //Z
   p_base[5][2] = local_start_point[5][2];

   for (uint8_t i = 0; i < 6; i++)
   {
      //rotation around Y axis
      //p_base_new[i][0] = p_base[i][0] * cos(-input_roll) - p_base[i][2] * sin(-input_roll);
      temp_x[i] = p_base[i][0] * cos(-Qy) - p_base[i][2] * sin(-Qy);
      p_base_new[i][2] = p_base[i][0] * sin(-Qy) + p_base[i][2] * cos(-Qy); //-input_roll

      //rotation around X axis
      p_base_new[i][1] = p_base[i][1] * cos(-Qx) - p_base_new[i][2] * sin(-Qx);
      p_base_new[i][2] = p_base[i][1] * sin(-Qx) + p_base_new[i][2] * cos(-Qx);  //-input_pitch

      //rotation around Z axis
      p_base_new[i][0] = temp_x[i] * cos(-Qz) - p_base_new[i][1] * sin(-Qz);
      p_base_new[i][1] = temp_x[i] * sin(-Qz) + p_base_new[i][1] * cos(-Qz);   //-input_yaw

      p_delta[i][0] = p_base_new[i][0] - p_base[i][0];
      p_delta[i][1] = p_base_new[i][1] - p_base[i][1];
      p_delta[i][2] = p_base_new[i][2] - p_base[i][2];

      Leg[i].Xt = local_start_point[i][0] + p_delta[i][0];
      Leg[i].Yt = local_start_point[i][1] + p_delta[i][1];
      Leg[i].Zt = local_start_point[i][2] + p_delta[i][2];

      if (is_move)
      {
         moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
      }
   }
}

void rotateDirection(uint8_t leg_num)
{
   //if last time we had velocity
   if (Leg[leg_num].Vx_last != 0 && Leg[leg_num].Vy_last != 0)
   {
      //current movement direction
      double direction = getAngle(Leg[leg_num].Vx_last, Leg[leg_num].Vy_last);
      //new movement direction
      double new_direction = getAngle(Vx, Vy);
      //directions delta
      double dq = new_direction - direction;
      //UART1_println_div(dq * RAD_TO_DEG);

      if (fabs(dq * RAD_TO_DEG) > 2)
      {
         //translate from leg link to step footprint (downshift)
         Leg[leg_num].Xt = Leg[leg_num].current_x - Leg[leg_num].tf_step_footprint_x;
         Leg[leg_num].Yt = Leg[leg_num].current_y - Leg[leg_num].tf_step_footprint_y;

         rotatePoint(&Leg[leg_num].Xt, &Leg[leg_num].Yt, dq);

         //translate from step footprint to leg link (upshift)
         Leg[leg_num].Xt = Leg[leg_num].Xt + Leg[leg_num].tf_step_footprint_x;
         Leg[leg_num].Yt = Leg[leg_num].Yt + Leg[leg_num].tf_step_footprint_y;
      }
   }
}

void addLinearVelocity(uint8_t leg_num, bool phase)
{
   //ground movement
   if (phase == 0)
   {
      //simply add new velocity to current coordinates
      Leg[leg_num].Xt = Leg[leg_num].Xt - Vx / MOVEMENT_FREQUENCY;
      Leg[leg_num].Yt = Leg[leg_num].Yt - Vy / MOVEMENT_FREQUENCY;
   }
   else
   {
      //simply add new velocity to current coordinates
      Leg[leg_num].Xt = Leg[leg_num].Xt + Vx / MOVEMENT_FREQUENCY;
      Leg[leg_num].Yt = Leg[leg_num].Yt + Vy / MOVEMENT_FREQUENCY;
   }
}

void evaluateZ(uint8_t leg_num, bool phase)
{
   //ground movement
   if (phase == 0)
   {
      Leg[leg_num].Zt = -H;
   }
   else
   {
      switch (leg_num)
      {
      case 0:
         Leg[0].Zt = -k * (Leg[0].Yt + Y_OFFSET) * (Leg[0].Yt + Y_OFFSET) - k * (Leg[0].Xt - X_OFFSET) * (Leg[0].Xt - X_OFFSET) - H + dH;
         break;

      case 1:
         Leg[1].Zt = -k * Leg[1].Yt * Leg[1].Yt - k * (Leg[1].Xt - X_OFFSET - 30) * (Leg[1].Xt - X_OFFSET - 30) - H + dH;
         break;

      case 2:
         Leg[2].Zt = -k * (Leg[2].Yt - Y_OFFSET) * (Leg[2].Yt - Y_OFFSET) - k * (Leg[2].Xt - X_OFFSET) * (Leg[2].Xt - X_OFFSET) - H + dH;
         break;

      case 3:
         Leg[3].Zt = -k * (Leg[3].Yt - Y_OFFSET) * (Leg[3].Yt - Y_OFFSET) - k * (Leg[3].Xt + X_OFFSET) * (Leg[3].Xt + X_OFFSET) - H + dH;
         break;

      case 4:
         Leg[4].Zt = -k * Leg[4].Yt * Leg[4].Yt - k * (Leg[4].Xt + X_OFFSET + 30) * (Leg[4].Xt + X_OFFSET + 30) - H + dH;
         break;

      case 5:
         Leg[5].Zt = -k * (Leg[5].Yt + Y_OFFSET) * (Leg[5].Yt + Y_OFFSET) - k * (Leg[5].Xt + X_OFFSET) * (Leg[5].Xt + X_OFFSET) - H + dH;
         break;
      }
   }
}

void hexapodMove(void)
{
   if (system_time >= next_time)
   {
      /*UART1_print_str("system time: ");
      UART1_println(system_time/1000);*/

      //if V=0 -> go to start points, reset phases for main gait and set last velocity to 0
      if (fabs(Vx) <= 20 && fabs(Vy) <= 20)
      {
         moveLeg(0, local_start_point[0][0], local_start_point[0][1], -H);
         moveLeg(2, local_start_point[2][0], local_start_point[2][1], -H);
         moveLeg(4, local_start_point[4][0], local_start_point[4][1], -H);
         Leg[0].phase = 0;
         Leg[2].phase = 0;
         Leg[4].phase = 0;

         moveLeg(1, local_start_point[1][0], local_start_point[1][1], -H);
         moveLeg(3, local_start_point[3][0], local_start_point[3][1], -H);
         moveLeg(5, local_start_point[5][0], local_start_point[5][1], -H);
         Leg[1].phase = 1;
         Leg[3].phase = 1;
         Leg[5].phase = 1;

         for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
         {
            Leg[leg_num].Vx_last = 0;
            Leg[leg_num].Vy_last = 0;
         }
      }
      else
      {
         //UART1_print_str("Vx: ");
         //UART1_print_div(Vx);
         //UART1_print_str(" Vy: ");
         //UART1_println_div(Vy);

         //double angle = 0.0;
         //angle = getAngle(Vx, Vy);

         //UART1_println_div(angle * RAD_TO_DEG);

         for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
         {
            //if phase 0 (ground moving)
            if (phaseControl(leg_num) == 0)
            {
               Leg[leg_num].Xt = Leg[leg_num].current_x;
               Leg[leg_num].Yt = Leg[leg_num].current_y;

               //rotate straight movement line
               rotateDirection(leg_num);

               //simply add new velocity to current coordinates
               Leg[leg_num].Xt = Leg[leg_num].Xt - Vx / MOVEMENT_FREQUENCY;
               Leg[leg_num].Yt = Leg[leg_num].Yt - Vy / MOVEMENT_FREQUENCY;
               Leg[leg_num].Zt = -H;
            }
            //if phase 1 (air movement)
            else
            {
               Leg[leg_num].Xt = Leg[leg_num].current_x;
               Leg[leg_num].Yt = Leg[leg_num].current_y;

               //rotate straight movement line
               rotateDirection(leg_num);

               //simply add new velocity to current coordinates
               Leg[leg_num].Xt = Leg[leg_num].Xt + Vx / MOVEMENT_FREQUENCY;
               Leg[leg_num].Yt = Leg[leg_num].Yt + Vy / MOVEMENT_FREQUENCY;

               switch (leg_num)
               {
               case 0:
                  Leg[0].Zt = -k * (Leg[0].Yt + Y_OFFSET) * (Leg[0].Yt + Y_OFFSET) - k * (Leg[0].Xt - X_OFFSET) * (Leg[0].Xt - X_OFFSET) - H + dH;
                  break;

               case 1:
                  Leg[1].Zt = -k * Leg[1].Yt * Leg[1].Yt - k * (Leg[1].Xt - X_OFFSET - 30) * (Leg[1].Xt - X_OFFSET - 30) - H + dH;
                  break;

               case 2:
                  Leg[2].Zt = -k * (Leg[2].Yt - Y_OFFSET) * (Leg[2].Yt - Y_OFFSET) - k * (Leg[2].Xt - X_OFFSET) * (Leg[2].Xt - X_OFFSET) - H + dH;
                  break;

               case 3:
                  Leg[3].Zt = -k * (Leg[3].Yt - Y_OFFSET) * (Leg[3].Yt - Y_OFFSET) - k * (Leg[3].Xt + X_OFFSET) * (Leg[3].Xt + X_OFFSET) - H + dH;
                  break;

               case 4:
                  Leg[4].Zt = -k * Leg[4].Yt * Leg[4].Yt - k * (Leg[4].Xt + X_OFFSET + 30) * (Leg[4].Xt + X_OFFSET + 30) - H + dH;
                  break;

               case 5:
                  Leg[5].Zt = -k * (Leg[5].Yt + Y_OFFSET) * (Leg[5].Yt + Y_OFFSET) - k * (Leg[5].Xt + X_OFFSET) * (Leg[5].Xt + X_OFFSET) - H + dH;
                  break;
               }
            }

            //save velocity for next cycle
            Leg[leg_num].Vx_last = Vx;
            Leg[leg_num].Vy_last = Vy;
         }

         for (uint8_t i = 0; i < 6; i++)
         {
            moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
         }
      }

      next_time = system_time + MOVEMENT_PERIOD;
   }
}

void newVersion(void)
{
   if (system_time >= next_time)
   {
      /*UART1_print_str("system time: ");
      UART1_println(system_time/1000);*/

      //if V=0 -> go to start points, reset phases for main gait and set last velocity to 0
      if (fabs(Vx) <= 20 && fabs(Vy) <= 20 && fabs(Wz) <= 5 * DEG_TO_RAD)
      {
         for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
         {
            moveLeg(leg_num, Leg[leg_num].start_x, Leg[leg_num].start_y, Leg[leg_num].start_z);
         }

         // moveLeg(0, local_start_point[0][0], local_start_point[0][1], -H);
         // moveLeg(2, local_start_point[2][0], local_start_point[2][1], -H);
         // moveLeg(4, local_start_point[4][0], local_start_point[4][1], -H);
         Leg[0].phase = 0;
         Leg[2].phase = 0;
         Leg[4].phase = 0;

         // moveLeg(1, local_start_point[1][0], local_start_point[1][1], -H);
         // moveLeg(3, local_start_point[3][0], local_start_point[3][1], -H);
         // moveLeg(5, local_start_point[5][0], local_start_point[5][1], -H);
         Leg[1].phase = 1;
         Leg[3].phase = 1;
         Leg[5].phase = 1;

         for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
         {
            Leg[leg_num].Vx_last = 0;
            Leg[leg_num].Vy_last = 0;
         }
      }
      //if no linear velocity and we have angular velocity
      //rotation movement
      //else if (fabs(Vx) <= 20 && fabs(Vy) <= 20 && fabs(Wz) > 5 * DEG_TO_RAD)
      //{
      //	for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
      //	{
      //		//ground movement
      //		if (phaseControl(leg_num) == 0)
      //		{
      //			Leg[leg_num].target_x = Leg[leg_num].current_x + Leg[leg_num].tf_leg_link_x;
      //			Leg[leg_num].target_y = Leg[leg_num].current_y + Leg[leg_num].tf_leg_link_y;

      //			rotatePoint(&Leg[leg_num].target_x, &Leg[leg_num].target_y, (-Wz / MOVEMENT_FREQUENCY) * (Leg[0].tip_radius / Leg[leg_num].tip_radius));

      //			Leg[leg_num].Xt = Leg[leg_num].target_x - Leg[leg_num].tf_leg_link_x;
      //			Leg[leg_num].Yt = Leg[leg_num].target_y - Leg[leg_num].tf_leg_link_y;
      //			
      //			evaluateZ(leg_num, 0);
      //		}
      //		//air movement
      //		else
      //		{
      //			Leg[leg_num].target_x = Leg[leg_num].current_x + Leg[leg_num].tf_leg_link_x;
      //			Leg[leg_num].target_y = Leg[leg_num].current_y + Leg[leg_num].tf_leg_link_y;

      //			rotatePoint(&Leg[leg_num].target_x, &Leg[leg_num].target_y, (Wz / MOVEMENT_FREQUENCY) * (Leg[0].tip_radius / Leg[leg_num].tip_radius));

      //			Leg[leg_num].Xt = Leg[leg_num].target_x - Leg[leg_num].tf_leg_link_x;
      //			Leg[leg_num].Yt = Leg[leg_num].target_y - Leg[leg_num].tf_leg_link_y;

      //			evaluateZ(leg_num, 1);
      //		}
      //	}

      //	for (uint8_t i = 0; i < 6; i++)
      //	{
      //		moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
      //	}
      //}
      else //if(fabs(Wz) <= 5 * DEG_TO_RAD)
      {
         for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
         {
            //if phase 0 (ground moving)
            if (phaseControl(leg_num) == 0)
            {
               Leg[leg_num].Xt = Leg[leg_num].current_x;
               Leg[leg_num].Yt = Leg[leg_num].current_y;

               //rotate straight movement line
               rotateDirection(leg_num);

               //simply add new velocity to current coordinates
               addLinearVelocity(leg_num, 0);

               //turning
               if (fabs(Wz) > 5 * DEG_TO_RAD)
               {
                  Leg[leg_num].target_x = Leg[leg_num].Xt + Leg[leg_num].tf_leg_link_x;
                  Leg[leg_num].target_y = Leg[leg_num].Yt + Leg[leg_num].tf_leg_link_y;

                  rotatePoint(&Leg[leg_num].target_x, &Leg[leg_num].target_y, (-Wz / MOVEMENT_FREQUENCY) * (Leg[0].tip_radius / Leg[leg_num].tip_radius));

                  Leg[leg_num].Xt = Leg[leg_num].target_x - Leg[leg_num].tf_leg_link_x;
                  Leg[leg_num].Yt = Leg[leg_num].target_y - Leg[leg_num].tf_leg_link_y;
               }

               evaluateZ(leg_num, 0);
            }
            //if phase 1 (air movement)
            else
            {
               Leg[leg_num].Xt = Leg[leg_num].current_x;
               Leg[leg_num].Yt = Leg[leg_num].current_y;

               //rotate straight movement line
               rotateDirection(leg_num);

               //simply add new velocity to current coordinates
               addLinearVelocity(leg_num, 1);

               if (fabs(Wz) > 5 * DEG_TO_RAD)
               {
                  Leg[leg_num].target_x = Leg[leg_num].Xt + Leg[leg_num].tf_leg_link_x;
                  Leg[leg_num].target_y = Leg[leg_num].Yt + Leg[leg_num].tf_leg_link_y;

                  rotatePoint(&Leg[leg_num].target_x, &Leg[leg_num].target_y, (Wz / MOVEMENT_FREQUENCY) * (Leg[0].tip_radius / Leg[leg_num].tip_radius));

                  Leg[leg_num].Xt = Leg[leg_num].target_x - Leg[leg_num].tf_leg_link_x;
                  Leg[leg_num].Yt = Leg[leg_num].target_y - Leg[leg_num].tf_leg_link_y;
               }

               evaluateZ(leg_num, 1);
            }

            //save velocity for next cycle
            Leg[leg_num].Vx_last = Vx;
            Leg[leg_num].Vy_last = Vy;
         }

         for (uint8_t i = 0; i < 6; i++)
         {
            moveLeg(i, Leg[i].Xt, Leg[i].Yt, Leg[i].Zt);
         }
      }

      next_time = system_time + MOVEMENT_PERIOD;
   }
}

void squareTest(void)
{
   float dx = 35;
   float dy = dx;

   float sx = 55;
   float sy = -sx;
   float sz = -50;

   int p = 50;

   for (int i = 0; i <= dx; i++)
   {
      moveLeg(0, sx + i, sy, sz);
      delay(p);
   }

   UART1_println_str("90,-55,-50");
   UART1_print_str("q0: ");
   UART1_print_div(q0);
   UART1_print_str(" q1: ");
   UART1_print_div(q1);
   UART1_print_str(" q2: ");
   UART1_println_div(q2);

   for (int i = 0; i <= dy; i++)
   {
      moveLeg(0, sx + dx, sy - i, sz);
      delay(p);
   }

   UART1_println_str("90,-90,-50");
   UART1_print_str("q0: ");
   UART1_print_div(q0);
   UART1_print_str(" q1: ");
   UART1_print_div(q1);
   UART1_print_str(" q2: ");
   UART1_println_div(q2);

   for (int i = 0; i <= dx; i++)
   {
      moveLeg(0, sx + dx - i, sy - dy, sz);
      delay(p);
   }

   UART1_println_str("55,-90,-50");
   UART1_print_str("q0: ");
   UART1_print_div(q0);
   UART1_print_str(" q1: ");
   UART1_print_div(q1);
   UART1_print_str(" q2: ");
   UART1_println_div(q2);

   for (int i = 0; i <= dy; i++)
   {
      moveLeg(0, sx, sy - dy + i, sz);
      delay(p);
   }

   UART1_println_str("55,-55,-50");
   UART1_print_str("q0: ");
   UART1_print_div(q0);
   UART1_print_str(" q1: ");
   UART1_print_div(q1);
   UART1_print_str(" q2: ");
   UART1_println_div(q2);
}

void senseTest(void)
{
   static double height = -70;
   static double h_max = -40;
   static double h_min = -80;
   static bool direction = 1;
   static double delta = 0.5;
   static double Vh = 0;
   static bool stop_flag = false;
   static bool flag_init = false;

   static SoftTimer_ms stop_timer;

   if (flag_init == false)
   {
      Vh = delta;
      stop_timer.delay = 0;
      stop_timer.start_time = system_time;

      flag_init = true;
   }

   if (checkTimer(&stop_timer))
   {
      if (stop_flag == 1)
      {
         stop_timer.delay = 0;
         stop_flag = 0;
      }

      if (height >= h_max)
      {
         Vh = -delta;
      }
      else if (height < h_min)
      {
         Vh = delta;
      }

      if (Vh < 0.0 && servo_current[8] > 90)
      {
         stop_flag = true;
         stop_timer.delay = 1000;
         stop_timer.start_time = system_time;
         Vh = delta;

         UART1_print_str("stop h ");
         UART1_println_div(height);

         return;
      }

      height += Vh;

      // UART1_print_str("h ");
      // UART1_println_div(height);

      stabilizationMode(0);

      moveLeg(2, Leg[2].current_x, Leg[2].current_y, height);

      moveLeg(0, Leg[0].Xt, Leg[0].Yt, Leg[0].Zt);
      moveLeg(1, Leg[1].Xt, Leg[1].Yt, Leg[1].Zt);
      moveLeg(3, Leg[3].Xt, Leg[3].Yt, Leg[3].Zt);
      moveLeg(4, Leg[4].Xt, Leg[4].Yt, Leg[4].Zt);
      moveLeg(5, Leg[5].Xt, Leg[5].Yt, Leg[5].Zt);

      // moveLeg(0, Leg[0].current_x, Leg[0].current_y, Leg[0].current_z);
      // moveLeg(1, Leg[1].current_x, Leg[1].current_y, Leg[1].current_z);
      // moveLeg(3, Leg[3].current_x, Leg[3].current_y, Leg[3].current_z);
      // moveLeg(4, Leg[4].current_x, Leg[4].current_y, Leg[4].current_z);
      // moveLeg(5, Leg[5].current_x, Leg[5].current_y, Leg[5].current_z);
   }
}

void senseTest1(void)
{
   static double start_height = -70;
   static double h_max = -40;
   static double h_min = -80;
   static double delta = 0.5;
   static double Vh = 0;
   static bool flag_init = false;
   static SoftTimer_ms stop_timer;

   if (flag_init == false)
   {
      Vh = delta;
      stop_timer.delay = 0;
      stop_timer.start_time = system_time;

      for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
      {
         //1 - in air, 0 - on ground
         Leg[leg_num].phase = 1;
      }

      flag_init = true;
   }

   //moving while not detecting ground
   for (uint8_t leg_num = 0; leg_num < 6; leg_num++)
   {
      //if this leg already on ground - skip
      if (Leg[leg_num].phase == 0)
      {
         continue;
      }
      else
      {
         if (Leg[leg_num].Zt >= h_max)
         {
            Vh = -delta;
         }
         else if (Leg[leg_num].Zt < h_min)
         {
            Vh = delta;
         }

         if (Vh < 0.0 && servo_current[leg_num * 3 + 2] > 100)
         {
            Leg[leg_num].phase = 0;
            // stop_timer.delay = 1000;
            // stop_timer.start_time = system_time;
            // Vh = delta;

            UART1_print_str("leg: ");
            UART1_print(leg_num);
            UART1_print_str(" stop h: ");
            UART1_println_div(Leg[leg_num].Zt);

            continue;
         }

         Leg[leg_num].Zt += Vh;
      }
   }

   stabilizationMode(0);

   moveLeg(2, Leg[2].current_x, Leg[2].current_y, Leg[2].Zt);

   moveLeg(0, Leg[0].Xt, Leg[0].Yt, Leg[0].Zt);
   moveLeg(1, Leg[1].Xt, Leg[1].Yt, Leg[1].Zt);
   moveLeg(3, Leg[3].Xt, Leg[3].Yt, Leg[3].Zt);
   moveLeg(4, Leg[4].Xt, Leg[4].Yt, Leg[4].Zt);
   moveLeg(5, Leg[5].Xt, Leg[5].Yt, Leg[5].Zt);

   // if (checkTimer(&stop_timer))
   // {
   //    if (stop_flag == 1)
   //    {
   //       stop_timer.delay = 0;
   //       stop_flag = 0;
   //    }

   //    if (height >= h_max)
   //    {
   //       Vh = -delta;
   //    }
   //    else if (height < h_min)
   //    {
   //       Vh = delta;
   //    }

   //    if (Vh < 0.0 && servo_current[8] > 90)
   //    {
   //       stop_flag = true;
   //       stop_timer.delay = 1000;
   //       stop_timer.start_time = system_time;
   //       Vh = delta;

   //       UART1_print_str("stop h ");
   //       UART1_println_div(height);

   //       return;
   //    }

   //    height += Vh;

   //    // UART1_print_str("h ");
   //    // UART1_println_div(height);

   //    stabilizationMode(0);

   //    // moveLeg(2, Leg[2].current_x, Leg[2].current_y, height);
   //    moveLeg(2, Leg[2].current_x, Leg[2].current_y, Leg[2].Zt);


   //    moveLeg(0, Leg[0].Xt, Leg[0].Yt, Leg[0].Zt);
   //    moveLeg(1, Leg[1].Xt, Leg[1].Yt, Leg[1].Zt);
   //    moveLeg(3, Leg[3].Xt, Leg[3].Yt, Leg[3].Zt);
   //    moveLeg(4, Leg[4].Xt, Leg[4].Yt, Leg[4].Zt);
   //    moveLeg(5, Leg[5].Xt, Leg[5].Yt, Leg[5].Zt);
   // }
}

void legManualControl(uint8_t leg_num)
{
   // static double height = -70;
   // static bool direction = 1;
   // static double delta = 0.5;

   // if (stop_flag == 1)
   // {
   //    direction = !direction;
   //    stop_flag = 0;
   // }

   // if (direction == 1)
   // {
   //    height += delta;

   //    if (height > -80)
   //    {
   //       direction = 0;
   //    }
   // }
   // else
   // {
   //    height -= delta;

   //    if (height < -120)
   //    {
   //       direction = 1;
   //    }
   // }

   // moveLeg(leg_num, Leg[leg_num].current_x, Leg[leg_num].current_y, height);

   // UART1_print_div(Leg[2].q0);
   // UART1_print_str(", ");
   // UART1_print_div(Leg[2].q1);
   // UART1_print_str(", ");
   // UART1_println_div(Leg[2].q2);
}

//************ EXTERNAL INTERRUPTIONS *******************************
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

void EXTI15_10_IRQHandler(void)
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
//************ END OF EXTERNAL INTERRUPTIONS ************************
