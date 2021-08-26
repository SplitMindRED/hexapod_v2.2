#ifndef hexapod_h
#define hexapod_h

#ifndef stdbool
#include "stdbool.h"
#endif

#ifndef stdint
#include "stdint.h"
#endif

#ifndef math
#include "math.h"
#endif

#ifndef splitmind_stm32f103_lib_h
#include "splitmind_stm32f103_lib.h"
#endif

#ifndef stm_spi_data_structure_h
#include "stm_spi_data_structure.h"
#endif

// #define SHOW_RPY

#define PWM_FREQ                    150   //Hz

// #define servomin  						287.0 // 100 Hz 700 mcs
// #define servomax  						942.0 // 100 Hz 2300 mcs

//Hexapod parametres
//offset from (0, 0) on XY plane in every local coordinate system of leg
#define X_OFFSET                    70
#define Y_OFFSET                    70

////ver 1
////#define X_TRANSLATION               55.86
////#define X_TRANSLATION_MID           64.5
////#define Y_TRANSLATION               90.86

//ver 2
#define X_TRANSLATION               62.93
#define X_TRANSLATION_MID           74.25
#define Y_TRANSLATION               97.93

#define STARTHEIGHT                 70

//height of step
#define DELTAHEIGHT                 30

//diameter of step circle. distance of step
#define DIAMETER                    60

#define MOVEMENT_PERIOD             20000       //in mcs
#define MOVEMENT_FREQUENCY          50          //in Hz

#define MAX_VEL_LINEAR              275         //mm/s
#define MAX_VEL_ANGULAR             1.0472      //rad/s (60 grad)

#define RAD_TO_DEG                  180.0 / pi
#define DEG_TO_RAD                  pi / 180.0

extern uint16_t servomin;
extern uint16_t servomax;

extern uint16_t channel[6];
// bool servo_enable;

//extern bool servo_enable;                                   //flag for enabling servo
extern float Vx, Vy, Vz;                                    //velocity for 3 dimensions (mm/s)
extern float Wz;                                            //velocity for turning (grad/s)
extern float input_roll, input_pitch, input_yaw;            //angle for 3 rotation axis
extern float current_roll, current_pitch, current_yaw;
extern double AcX, AcY, AcZ, GyX, GyY, GyZ, RadX, RadY, RadZ, GradX, GradY, GradZ;
extern double servo_current[18];

//all cordinates of this structure are 
//in local coordinate system of each leg
extern struct Legs
{
   //three start coordiantes of leg in local coordinate system
   int16_t start_x;
   int16_t start_y;
   int16_t start_z;

   //f(t) coordinates functions
   //(target point)
   double Xt, Yt, Zt;

   //current local position of leg (x, y, z) (leg_link)
   double current_x;
   double current_y;
   double current_z;

   //new local position of leg (x, y, z) (not using yet, idk why)
   double target_x;
   double target_y;
   double target_z;

   //target angle for servos
   double q0, q1, q2;

   //velocity on previous cycle step
   float Vx_last;
   float Vy_last;

   //tf from leg link to base link
   float tf_leg_link_x;
   float tf_leg_link_y;
   //tf from step footprint to leg link
   float tf_step_footprint_x;
   float tf_step_footprint_y;

   //distance between center of mass and leg tip
   float tip_radius;

   //phase of leg: 0 -> ground moving, 1 -> air moving
   bool phase;

} Leg[6];

extern const int16_t local_start_point[6][3];
extern int16_t local_stabilization_point[6][3];
extern uint16_t leg_contact_current_1[6];
extern uint16_t leg_contact_current_2[6];

extern float leg_translation[6][3];

extern float diameter;
extern double pi;

extern float k;
extern float dH;
extern float H;

//var for controlling movement function loop
extern unsigned long next_time;

//hexapod initialisation, moving legs to start positions
void hexapodInit(uint8_t* l_p_angle_array);

void senseTest(void);

void senseTest1(void);

void squareTest(void);

void evalCurrent(void);

//convert data from FlySky reciever channels to velocities and rotate angles
void servoManualControl(void);

void legManualControl(uint8_t leg_num);

void convertFlySkyData(void);

void switchMode(void);

bool checkSum(uint8_t source_sum, uint8_t* p_array, uint8_t size);

uint8_t evalSum(uint8_t* p_array, uint8_t size);

void stabilizationMode(bool is_move);

//inverse kinematics solution for leg: gets point (x,y,z), returns q0, q1, q2 servo angles of leg
void findAngles(uint8_t leg_num, double x, double y, double z);

uint16_t evalPWM(float angle);

//rotate point (x,y) for q rad
void rotatePoint(double* x, double* y, double q);

//return angle in rad for vector (x,y)
double getAngle(double x, double y);

//move leg tip to (x,y,x) point
void moveLeg(uint8_t leg_num, double x, double y, double z);

//rotate straight movement line
void rotateDirection(uint8_t leg_num);

//set angle to specified servo
// void setServoAngle(uint8_t servo_num, uint8_t Q);
void setServoAngle(uint8_t servo_num, uint16_t PWM);

////void turning

//add Vx, Vy
void addLinearVelocity(uint8_t leg_num, bool phase);

//find Z from x,y
void evaluateZ(uint8_t leg_num, bool phase);

//
bool phaseControl(uint8_t leg_num);

//rotate body in 3 axis
void rotateBody(double Qx, double Qy, double Qz, bool is_move);

//function with first gait movement
void hexapodMove(void);

//dev function for testing new movement features
void newVersion(void);

//************ EXTERNAL INTERRUPTIONS *******************************
//Interruptions for PPM 
//particular EXTI group for pin
void EXTI15_10_IRQHandler(void);
void EXTI_init(void);
//************ END OF EXTERNAL INTERRUPTIONS ************************

#endif
