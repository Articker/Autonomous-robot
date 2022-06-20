#include <util/atomic.h>
#include <stdio.h>
#include <math.h>

//GLOBAL CONSTANS//--------------------------------------------------------------------------------------------------------------------------------------------
const float pi = 3.1415926;
const float radius = 0.0325;//3.25; //cm //0.0325; //m
const float wheels_baseline = 0.125; //12.5; //m distance between wheels of the robot
const float pulsesPerTurn = 8*120; //maybe 16*120 [pulses/turn] 
float pulsesPerMeter = pulsesPerTurn*(1/(2*pi*radius)); // 4.89955903968642822 pulses / meter
//MOTORS wiring variables
const byte enca[] = {2,3}; //2,3 //encoders pin 0 //maybe 1 //2,3 pins better for interruptions
const byte encb[] = {8,12}; //8,12 //encoders pin 1 //maybe 0
const byte in1[] = {5,6}; // 5, 4 right wheels motors pins anaglog write PWM // 5,6
const byte in2[] = {7,4}; // 6,7 left wheel direciton digital write          // 7,4
//SONAR wiring variables
const byte URECHO = 9;         // PWM Output 0-50000US,Every 50US represent 1cm
const byte URTRIG = 10;         // trigger pin
//END of GLOBAL CONSTANS//-------------------------------------------------------------------------------------------------------------------------------------

//SONAR VARIABLES=======================================================
int DistanceMeasured = 0;
//END of SONAR VARIABLES================================================

//COMUNICATIONS VARIABLES>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
const byte numChars = 32; //32
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char messageFromPC[numChars] = {0};
int integerFromPC1 = 0;
int integerFromPC2 = 0; //float floatFromPC = 0.0;
int integerFromPC3 = 0;
int last_integerFromPC1 = 0;
boolean newData = false;
int n = 11;
int data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int dataI = 0;
int sortedData[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int message = 0;
byte count_send = 1;
//END of COMUNICATION VARBIABLES>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//GLOBAL ODOMETRTY VARIABLES//========================================================================================================================================
float distance_traveled_total[2] = {0.0,0.0};  //m //dist travel by each wheel from the beginning 0 - right, 1 - left 
float dist_change[2] = {0,0}; // distance traveled between 2 measures for each wheel
float distance_traveled_prev[2] = {0,0}; //distance traveled by each wheel before update 
float arc_traveled = 0.0;  // arc distance traveled since start for robot
float arc_traveled_change = 0.0; // arc distance traveled between 2 measures for robot
float angle_phi = 0.0; //change in angle since last measure
float angle_theta = 0.0; //current angle between start and current position; will be restricted to -2pi to 2pi values
float angle_theta_atan2 = 0.0; //current angle between start and current position restricted to -pi to pi values
float angle_theta_degrees = 0.0; //current angle between start and current position in [degrees] 
int angle_theta_degrees_int = 0;
float xPosition = 0.0; // current X position of robot on kartezian grid where starting position is 0,0
float yPosition = 0.0; // current Y position of robot on kartezian grid where starting position is 0,0
//END of GLOBAL ODOMETRTY VARIABLES//==================================================================================================================================

//LOCAL ODOMETRTY VARIABLES//========================================================================================================================================
float distance_traveled_total_local[2] = {0.0,0.0};  //m //dist travel by each wheel from the beginning 0 - right, 1 - left 
float dist_change_local[2] = {0,0}; // distance traveled between 2 measures for each wheel
float distance_traveled_prev_local[2] = {0,0}; //distance traveled by each wheel before update 
float arc_traveled_local = 0.0;  // arc distance traveled since start for robot
float arc_traveled_change_local = 0.0; // arc distance traveled between 2 measures for robot
float angle_phi_local = 0.0; //change in angle since last measure
float angle_theta_local = 0.0; //current angle between start and current position; will be restricted to -2pi to 2pi values
float angle_theta_atan2_local = 0.0; //current angle between start and current position restricted to -pi to pi values
float angle_theta_degrees_local = 0.0; //current angle between start and current position in [degrees]
int angle_theta_degrees_local_int = 0;
float xPosition_local = 0.0; // current X position of robot on kartezian grid where starting position is 0,0
float yPosition_local = 0.0; // current Y position of robot on kartezian grid where starting position is 0,0

long number_of_pulses_local[2] = {0,0}; //actual number of pulses for each wheel 0 right 1 left for local odometry so it can be reset
volatile int newest_number_of_pulses_local[2] = {0,0}; //variable with newest number of pulses for each wheel //in case of errors change back to INT ***
int newest_number_of_pulses_local_clean = 0;
//END of LOCAL ODOMETRTY VARIABLES//==================================================================================================================================

//ANGULAR VELOCITY PID VARIABLES--------------------------------------------------------------------------------------------------------------------------------
float error_prev_angle = 0.0;
float eintegral_angle = 0.0;
float error_angle = 0; //***
float desired_robot_speed_angular = 0;
//END of ANGULAR VELOCITY PID VARIABLES-------------------------------------------------------------------------------------------------------------------------

//WHEEL VELOCITY PID VARIABLES--------------------------------------------------------------------------------------------------------------------------------
float error_prev_angle_local = 0.0;
float eintegral_angle_local = 0.0;
float error_angle_local = 0; //***
float desired_robot_speed_wheel_local = 0;

//END of WHEEL VELOCITY PID VARIABLES-------------------------------------------------------------------------------------------------------------------------

//motors PID variables========================================================================================================================================
float new_target[]={0.0,0.0,0.0,0.0}; //new target velocity for each wheel for PID calculation
float target[] = {0,0,0,0}; //target velocity for each wheel for PID calculation
float velocity_needed_R = 0; //velocity on right wheel needed to reach X,Y // used as a current target for pid low level controler
float velocity_needed_L = 0; //velocity on left wheel needed to reach X,Y  // used as a current target for pid low level controler
//END of motors PID variables=================================================================================================================================

//------------------Desired coordinates and angle------------------------------------------------------------------------------------------------------------- 
//const float XY_goal_res = 0.03; //  x,y position error that is accepted [m]?? ***
const float X_goal_res = 0.02; //  x position error that is accepted [m]?? ***
const float Y_goal_res = 0.02; //  y position error that is accepted [m]?? ***
float constant_velocity = 0.1; //0.07 [m/s][rad / s] minimal speed [revolutions/second] that has to be maintain for robot to be in constant move in order to find x,y coordinates
float angle_res_local = 1; // resolution of angle when rotating in place [degrees]
//END of Desired coordinates and angle------------------------------------------------------------------------------------------------------------------------ 

//VELOCITY MEASURE variables =================================================================================================================================
long number_of_pulses[2] = {0,0}; //actual number of pulses for each wheel 0 right 1 left
long prev_number_of_pulses[2] = {0,0}; //previous number of pulses for each wheel 0 right 1 left
volatile int newest_number_of_pulses[2] = {0,0}; //variable with newest number of pulses for each wheel //in case of errors change back to INT ***
float actual_rotation_speed[2] = {0,0}; //actual RPM of each wheel 0 - right 1 - left
float actual_velocity_speed[2] = {0,0}; //actual m/s velocity of each wheel
float actual_angular_speed[2] = {0,0}; //actual radian/s velocity of each wheel
float actual_angular_speed_robot = 0; //actual angular speed robot in radians  // still no implementation ************
//END of VELOCITY MEASURE variables ===========================================================================================================================

//TIME MEASURE VARIABLES//-------------------------------------------------------------------------------------------------------------------------------------
float deltaT = 0; //time that passed since last measure
long prevT = 0; //previous time in seconds
const float time_interval = 0.3*1000000; //[us] time interval for printing
float prev_time = 0; //interval var for printing
float time_drive_diff = 0; //difference between time set to go backwards/rotate and remaining time
float time_drive = 0; //how long should rotate/go backwards
float action_time = 0; //time of getting the message about going backwards/rotate
long prev_time_sonar = 0; 
const float time_interval_sonar = 1*100000; //[0.1sec] time interval for printing
//END of TIME MEASURE VARIABLES//------------------------------------------------------------------------------------------------------------------------------

//STATE MACHINE VARIABLES----------------------------------------------------------------------------------------------------------------------------------------
boolean reached_goal = false;
boolean reached_goal_local = false;
boolean reached_angle = false;
byte state = 1;
boolean stop_robot = false;
float Xd = 0; //desired X [m]
float Yd = 0; //desired Y [m]
float Zd = 0; //desired theta [degrees]
float Xd_local = 0; //desired X [m]
float Yd_local = 0; //desired Y [m]
float desired_angle_local = 0; //desired theta [degrees]
float new_Xd = 0; //desired X [m]
float new_Yd = 0; //desired Y [m]
float new_Zd = 0; //desired theta [degrees]
float new_Xd_local = 0; //desired local X [m] 
float new_Yd_local = 0; //desired local Y [m]
float new_desired_angle_local = 0; //desired local theta [degrees]
float orders[5][3] = { { 0, 0, 0 }, { 0, 0, 0 } ,{ 0, 0, 0 }, { 0, 0, 0 } , { 0, 0, 0 } }; //rows , columns
int last_order_1 = 0;
int last_order_2 = 0;
int last_order_3 = 0;
byte count_row = 0;
byte count_columns = 0;
byte count_orders = 0; //total number of new orders
int goal_number = -1; //how many times goal was reached
byte point_step = 1;
int message_status = 0;
int objective_status = 0;
int comma_status = 0;
float new_desired_wheel_speed = 0;
int show_error = 0;
boolean allow_state = true;

//END of STATE MACHINE VARIABLES---------------------------------------------------------------------------------------------------------------------------------

//ROTATE VARIABLES================================================================================================================================================
int rotate_error = 0;
byte flag_rotate_goal_timer = 0;
float deltaT_rotate = 0;
float rotate_goal_timer = 0;
//END of ROTATE VARIABLES=========================================================================================================================================
float value_F_R = 0;
float value_F_L = 0;
int last_value = 0;
int last_value_2 = 0;

boolean flag_backward = true;
long backward_time = 0;
boolean set_velocity = false;
int value_calc = 0;
/////////////////////////////////////////////////>>>END OF VARIABLES<<<<<<<<<<<\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//ROTATING IN PLACE -------------------------------------------------------------------------------------------------
void ROTATE_PID(float desired_angle_local, float angle_theta_degrees_local, float deltaT, float &velocity_needed_R,float &velocity_needed_L){ 
float Kp_xy = 0.003,Ki_xy = 0.0001,Kd_xy = 0.0001; // ROTATE_PID's parameters // ROTATE PID PARAMETERS <--- kp,ki,kd <--- ***
float umax_xy = 2, eprev_xy, eintegral_xy, u; //umax = max wheel velocity of robot [revolution/sec] // 1

float error_angle_local = desired_angle_local - angle_theta_degrees_local; // [desired_angle - current_angle]
float error_angle_local_abs = abs(desired_angle_local - angle_theta_degrees_local); //for checking if it reached the goal

if (error_angle_local_abs < angle_res_local && reached_angle == false){
reached_angle = true; 
stop_robot = true; // <<<<--- ***
objective_status++;
state = 1;
}


    float dedt_angle = (error_angle_local - error_prev_angle_local)/(deltaT); // derivative
    eintegral_angle_local = eintegral_angle_local + error_angle_local*deltaT; // integral
    desired_robot_speed_wheel_local = Kp_xy*error_angle_local + Kd_xy*dedt_angle + Ki_xy*eintegral_angle_local; //[revolution/sec] control signal//result of HIGH-level PID control => desired robot's wheel velocity... 
    ///...at the moment to compensate error in angle
    if (desired_robot_speed_wheel_local > umax_xy){
    desired_robot_speed_wheel_local = umax_xy; //constraint of maximum wheel speed that can occure using umax_xy var  
    }
    if (desired_robot_speed_wheel_local < -umax_xy){
    desired_robot_speed_wheel_local = umax_xy; //constraint of maximum wheel speed that can occure using umax_xy var  
    }
    error_prev_angle_local = error_angle_local; //for updating the angle error
    
    
    //velocity_needed_R = desired_robot_speed_wheel_local;
    //velocity_needed_L = -desired_robot_speed_wheel_local;
    /*
    if (error_angle_local > 0){
    velocity_needed_R = desired_robot_speed_wheel_local;
    velocity_needed_L =  - desired_robot_speed_wheel_local; //turn left if > 0
    }
    else {
    velocity_needed_R = - desired_robot_speed_wheel_local; //turn right if < 0 
    velocity_needed_L = desired_robot_speed_wheel_local;
    } 
    */   
}//END of ROTATING IN PLACE-------------------------------------------------------------------------------------------


//HIGH LEVEL PID ============================================================================================================================================================
void XY_PID(float &Xd, float &Yd, float x_Position, float y_Position, float theta, float deltaT, float &desired_robot_speed_angular){ 
float Kp_xy = 0.6,Ki_xy = 0.05,Kd_xy = 0.02; // XY_PID's parameters // HIGH LEVEL PID PARAMETERS <--- kp,ki,kd // 0.6 0.05 0.02***
float umax_xy = 0.3, eprev_xy, eintegral_xy; //umax = max angular velocity of robot [revolution/sec] // 1
float X_difference = Xd - x_Position; // Difference between desired X and robot's X 
float Y_difference = Yd - y_Position; // Difference between desired Y and robot's Y 

if (reached_goal == false){
  if (abs(X_difference) <= X_goal_res && abs(Y_difference) <= Y_goal_res){
    reached_goal = true;
    stop_robot = true; //
    objective_status++;
    state = 1; //***
}
}
float theta_goal = atan2(Y_difference,X_difference); //angle between robot and goal
error_angle = theta_goal - theta; //error of angle that pid should compansate  
error_angle = atan2(sin(error_angle),cos(error_angle)); //error angle between goal and robot
    float dedt_angle = (error_angle - error_prev_angle)/(deltaT); // derivative
    eintegral_angle = eintegral_angle + error_angle*deltaT; // integral
    desired_robot_speed_angular = Kp_xy*error_angle + Kd_xy*dedt_angle + Ki_xy*eintegral_angle; //[revolution/sec] control signal//result of HIGH-level PID control => desired robot's angular velocity... 
    ///...at the moment to compensate error in angle between robot and goal and also set robot in montion as the lowest possible output is set by constant velocity for each wheel... 
    //...so when robot is on the right path it will proceed forward and stop close to the goal
    if (desired_robot_speed_angular > umax_xy){
    desired_robot_speed_angular = umax_xy; //constraint of maximum angular speed that can occure using umax_xy var  
    }
    if (desired_robot_speed_angular < -umax_xy){
    desired_robot_speed_angular = umax_xy; //constraint of maximum angular speed that can occure using umax_xy var  
    }
    error_prev_angle = error_angle; //for updating the angle error
} //END of HIGH LEVEL PID ========================================================================================================================================================

//HIGH LEVEL PID local ============================================================================================================================================================
void XY_PID_local(float &Xd_local, float &Yd_local, float x_Position_local, float y_Position_local, float theta_local, float deltaT, float &desired_robot_speed_angular){ 
float Kp_xy = 0.6,Ki_xy = 0.05,Kd_xy = 0.02; // XY_PID's parameters // HIGH LEVEL PID PARAMETERS <--- kp,ki,kd // 0.6 0.05 0.02***
float umax_xy = 1.5, eprev_xy, eintegral_xy; //umax = max angular velocity of robot [revolution/sec] // 1 //umax_xy = 0.8
float X_difference_local = Xd_local - x_Position_local; // Difference between desired X and robot's X local
float Y_difference_local = Yd_local - y_Position_local; // Difference between desired Y and robot's Y local
float distance = sqrt((X_difference_local*X_difference_local) + (Y_difference_local * Y_difference_local));
if (distance < 7){
Kp_xy = 0.6,Ki_xy = 0.05,Kd_xy = 0.02; // XY_PID's parameters // HIGH LEVEL PID PARAMETERS <--- kp,ki,kd // 0.6 0.05 0.02***
umax_xy = 0.6, eprev_xy, eintegral_xy;
if (Xd_local > 0){
constant_velocity = 0.06; //0.7  
}
else{
constant_velocity = -0.06;  
}
}
if (distance >= 7 and distance <= 20){
Kp_xy = 0.6,Ki_xy = 0.05,Kd_xy = 0.02; // XY_PID's parameters // HIGH LEVEL PID PARAMETERS <--- kp,ki,kd // 0.6 0.05 0.02***
umax_xy = 0.8, eprev_xy, eintegral_xy;
if (Xd_local > 0){
constant_velocity = 0.07; //0.7  
}
else{
constant_velocity = -0.07;  
}    
}

if(distance > 20){
Kp_xy = 1.6,Ki_xy = 0.05,Kd_xy = 0.02; // XY_PID's parameters // HIGH LEVEL PID PARAMETERS <--- kp,ki,kd // 0.6 0.05 0.02***
umax_xy = 1.5, eprev_xy, eintegral_xy;
if (Xd_local > 0){
constant_velocity = 0.07; //0.7  
}
else{
constant_velocity = -0.07;  
}  
}

if (reached_goal == false){
  
  //if (abs(X_difference_local) <= X_goal_res || abs(Y_difference_local) <= Y_goal_res){
  if (abs(X_difference_local) <= X_goal_res && abs(Y_difference_local) <= Y_goal_res){
    reached_goal_local = true;
    stop_robot = true; //
    //goal_number++;
    objective_status++;
    state = 1; //***
}
}
float theta_goal = atan2(Y_difference_local,X_difference_local); //angle between robot and goal
error_angle_local = theta_goal - theta_local; //error of angle that pid should compansate  
error_angle_local = atan2(sin(error_angle_local),cos(error_angle_local)); //error angle between goal and robot
    float dedt_angle = (error_angle_local - error_prev_angle_local)/(deltaT); // derivative
    eintegral_angle_local = eintegral_angle_local + error_angle_local*deltaT; // integral
    desired_robot_speed_angular = Kp_xy*error_angle_local + Kd_xy*dedt_angle + Ki_xy*eintegral_angle_local; //[revolution/sec] control signal//result of HIGH-level PID control => desired robot's angular velocity... 
    ///...at the moment to compensate error in angle between robot and goal and also set robot in montion as the lowest possible output is set by constant velocity for each wheel... 
    //...so when robot is on the right path it will proceed forward and stop close to the goal
    if (desired_robot_speed_angular > umax_xy){
    desired_robot_speed_angular = umax_xy; //constraint of maximum angular speed that can occure using umax_xy var  
    }
    if (desired_robot_speed_angular < -umax_xy){
    desired_robot_speed_angular = -umax_xy; //constraint of maximum angular speed that can occure using umax_xy var  
    }
    error_prev_angle_local = error_angle_local; //for updating the angle error
} //END of HIGH LEVEL PID local========================================================================================================================================================

//===================PID MOTORS---------------------------------------------------------------------------------------------------------------------------------------
class SimplePID{ 
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255){}
  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }
  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    //float u = kp*e + kd*dedt + ki*eintegral;
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u); //absolute number 
    if( pwr > umax ){
      pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;
  } 
    void evalu_R(int value, int target, float deltaT, int &pwr, int &dir, int &rotate_error){
    // error 
    int e = value - target;
    if (target > 0){
    rotate_error = e+180;
    }
    if (target < 0){
    rotate_error = e-180;  
    }
    
    if (e < -180) { //-180
      e += 360; //360
      }
    if (e > 180) { //180
      e -= 360; //-360ROTAT
      }
      
    //<rotate_error = e; //pass the error value for other functions
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    //float u = kp*e + kd*dedt + ki*eintegral;
    if (abs(target) < 10){
    kp = 6; //60
    ki = 3; //30
    kd = 1; //10 
    }
    else{
    kp = 3; //30
    ki = 0.3; //3
    kd = 0;  //0
    }
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u); //absolute number 
    if( pwr > umax ){
      pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;
  }
};
SimplePID pid[4]; //create 2 instances for PID motors // PID

//END OF PID MOTORS-------------------------------------------------------------------------------------------------------------------------------------------

//==========================================SETUP========================================================================
void setup() { 
  Serial.begin(115200); //might be important to increase value 57600 vs 9600
  pinMode(URTRIG, OUTPUT);                   // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH);                // Set to HIGH
  pinMode(URECHO, INPUT);                    // Sending Enable PWM mode command
  
  for(int k = 0; k < 2; k++){ //motors arduino setup
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);  
  }             
                 //kp,ki,kd,maxPWM
  pid[0].setParams(1.6,0.05,0.1,255); //max255 //right motor PID var //(1.6,0.05,0.1,255) 
  pid[1].setParams(1.6,0.05,0.1,255); //left motor PID var //(1.6,0.05,0.1,255) - rotate angle 
  pid[2].setParams(60,30,10,255);//(30,3,0,255) //max255 //right motor PID var rotate//0.3 0 0.01 //2,0.4,5 - rotate angle 
  pid[3].setParams(1,0.01,0.01,70); //left motor PID var rotate //0.3 0 0.01 //2,0.4,5 - rotate angle 
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder1,RISING); //right wheel encoder interruptions
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder2,RISING); //left wheel encoder interruptions  
}     
//======================================END of SETUP========================================================================

////////////////////////////////////START OF THE MAIN LOOP ///////////////////////////////////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {   
  // time measure--------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  long currT = micros(); //number of microseconds since the Arduino began running current program
  deltaT = ((float) (currT - prevT))/( 1.0e6 ); //time of the loop in seconds
  prevT = currT; //updating previous time in seconds
  // END of time measure-------------------------------------------------------------------------------------------------------------------------------------------------------
  
  ////COUNT TICKS////================================================================================================  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ //counting pulses using interruptions
    for(int k = 0; k < 2; k++){    
      number_of_pulses[k] = newest_number_of_pulses[k]; //updating current number of ticks for each encoder
      number_of_pulses_local[k] = newest_number_of_pulses_local[k]; //updating current number of ticks for each encoder
    }
  }
  ////COUNT TICKS////================================================================================================

  //############### MAIN FUNCTIONS ###########################################################################################################################################################################
  
  check_speed(currT,deltaT); //check angular and linear speed of each wheel
  
  if (currT - prev_time_sonar > time_interval_sonar) {
    sonar(DistanceMeasured);
    prev_time_sonar = currT;
  }
  // DATA COMUNICATIONS============================================================== 
  maintainDisplay();
  recvWithStartEndMarkers();
  //parseData(); 
  protect_data();
  if (integerFromPC1 != last_integerFromPC1 and integerFromPC1 != 7 ){ //use data only if its new - when type of data is changed
  order_protocol(integerFromPC1,integerFromPC2,integerFromPC3); //interpret integers from pc
  last_integerFromPC1 = integerFromPC1;
  }
  if (integerFromPC1 == 7) {
  order_protocol(integerFromPC1,integerFromPC2,integerFromPC3);  
  }
  // END OF DATA COMMUNICATIONS =====================================================
  odometry_global(); //Calculating distance and angle between new and starting position of robot
  odometry_local(); //Odometry just for new part of the movement
  state_machine(currT,velocity_needed_R, velocity_needed_L); 
  if (state == 2){
  XY_PID(Xd, Yd, xPosition, yPosition, angle_theta_atan2, deltaT, desired_robot_speed_angular); //calculating angular velocity of the robot using high level PID
  angular_signal_to_velocity(constant_velocity, desired_robot_speed_angular, radius, wheels_baseline, velocity_needed_R, velocity_needed_L); //changing desired robot angular speed to velocity of each wheel that will produce it
  }
  if (state == 3){
  XY_PID_local(Xd_local, Yd_local, xPosition_local, yPosition_local, angle_theta_atan2_local, deltaT, desired_robot_speed_angular); //calculating angular velocity of the robot using high level PID
  angular_signal_to_velocity(constant_velocity, desired_robot_speed_angular, radius, wheels_baseline, velocity_needed_R, velocity_needed_L); //changing desired robot angular speed to velocity of each wheel that will produce it
  }
  if (state == 4){  //***
    currT = micros();
    if (abs(rotate_error) < 2){ 
      if (flag_rotate_goal_timer == 0){
      rotate_goal_timer = micros();
      flag_rotate_goal_timer = 1;
      }
    }
    else {
      flag_rotate_goal_timer = 0;  
    }
      if (flag_rotate_goal_timer == 1){
      deltaT_rotate = ((float) (currT - rotate_goal_timer))/( 1.0e6 );
      if (deltaT_rotate > 0.2){ //after 1 second of constant valid angle goal is reached
      reached_angle = true; 
      //stop_robot = true; // <<<<--- ***
      objective_status++;
      state = 1;
      deltaT_rotate = 0;
      flag_rotate_goal_timer = 0;  
      }
      }
  }
  if (state == 5 || state == 6){
    time_drive_diff = time_drive - action_time;
    if (time_drive_diff < 5 ){
    allow_state = true;     
    }
    else{
    allow_state = false;
    state = 1;  
    }
  }
  if (allow_state == true){ 
  setTarget(currT/1.0e6,deltaT,velocity_needed_R,velocity_needed_L); // set target for low level PID and aquire needed speed for each wheel to go to XY goal
  pwm_to_motors(); //change voltage for motors after calculation to get wanted speed
  }
  //############### MAIN FUNCTIONS ###########################################################################################################################################################################

} 
//////////////////////////////////////////////END OF THE MAIN LOOP///////////////////////////////////////////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//order_protocol=======================================================
void order_protocol(int type,int value,int value_2){ //translate data from PC into actions
switch(type){

case 9:
comma_status ++; 
reset_local();
state = 1;
break;


case 1: //new desired X,Y global 
reset_local();
new_Xd = value;
new_Yd = value_2;
state = 2;
message_status ++; //got message ad 1
break;

case 2: //new desired X,Y local
reset_local(); //reset local odometry before reaching new local goal
new_Xd_local = value;
new_Yd_local = value_2;
state = 3;
message_status ++;
break;

case 3: //new desired angle local

reset_local();
value_calc = value;
if (value_calc < 0){
value_calc = value_calc - 180;  
}
if (value_calc > 0){
value_calc = value_calc - 180;  
}
new_desired_angle_local = value_calc;
state = 4; 
message_status ++;
break;


case 4: //stop the robot immediately
stop_robot = true;
reset_local();
if (message_status != objective_status or message_status != comma_status){
objective_status = message_status;
comma_status = message_status;
}
state = 1; // go to standby (no reset local movement)
break;

case 5: //sending speed value for rotating in place
reset_local();
state = 5;
//float new_value = value;
new_desired_wheel_speed = value;
time_drive = value_2; //how long should rotate
//message_status ++;
break;

case 6: //sending speed value going backwards 
reset_local();
state = 6;
//float new_value = value;
new_desired_wheel_speed = value;
time_drive = value_2; //how long should it go backwards
//message_status ++;
break;

case 7://manual control of velocity of wheels

state = 7;
stop_robot = false;
//number_of_pulses[0] = 0; //***
//number_of_pulses[1] = 0;
//target[0]=0;
//target[1]=0;
value_F_R = value;
value_F_L = value_2;

break;

case 10:
reset_local();
state = 1;
break;

case 11:
reset_local();
state = 1;
break;

case 13:
allow_state = true;
break;

case 15:
reset_local();
state = 1;
break;

}//switch end
  
}//End of order_protocol===============================================

//STATE_MACHINE=====================================================================================================
void state_machine(long currT,float &v_needed_R,float &v_needed_L){
switch(state){
case 1: //default state - standing by
stop_robot = true; // no movement
allow_state = true; 
break; 

case 2: //reaching X,Y goal (global)
Xd = new_Xd/100;  //new Xd in cm changing to meters
Yd = new_Yd/100; //update desired X , Y coordinaties with info from camera
stop_robot = false; //movement is on
reached_goal = false;
break;

case 3: //reaching X,Y goal (local)
Xd_local = new_Xd_local/100; //new Xd in cm changing to meters
Yd_local = new_Yd_local/100; //update desired X , Y coordinaties with info from camera
stop_robot = false; //movement is on
reached_goal_local = false;
break;

case 4: //changing angle in place
//maybe abort all reamaing movement first
desired_angle_local = new_desired_angle_local;
stop_robot = false;
reached_angle = false;
break;

case 5: //rotating with constans speed to find marker
stop_robot = false;
velocity_needed_R = new_desired_wheel_speed/1000;
velocity_needed_L = -new_desired_wheel_speed/1000;
action_time = currT/1.0e6;
break;

case 6: //going back for x secs
stop_robot = false;
velocity_needed_R = -new_desired_wheel_speed/1000;
velocity_needed_L = -new_desired_wheel_speed/1000;
action_time = currT/1.0e6;
break;

case 7: //manual control velocity 
v_needed_R = value_F_R/1000;
v_needed_L = value_F_L/1000;
break;

case 10:
reset_local();
state = 1;
break;

case 13:
allow_state = true;
break;
}//end of switch
}//END of STATE_MACHINE=====================================================================================================

//reset_local----------------------------------------------------------------------------------------------------------------------------------
void reset_local(){ 
for (int i = 0; i<2; i++){
number_of_pulses_local[i] = 0;
newest_number_of_pulses_local[i] = 0;
newest_number_of_pulses_local_clean = 1;
distance_traveled_total_local[i] = 0;
distance_traveled_prev_local[i] = 0;
dist_change_local[i] = 0;
new_target[i] = 0;
target[i] = 0;
}
Xd_local = 0; //***
Yd_local = 0; //***
arc_traveled_local = 0; 
arc_traveled_change_local = 0;
angle_theta_local = 0; 
xPosition_local = 0;  
yPosition_local = 0; 
eintegral_angle_local = 0;
eintegral_angle = 0;
velocity_needed_R = 0;
velocity_needed_L = 0;
desired_robot_speed_angular = 0; 
angle_theta_degrees_local = 0;
angle_phi_local = 0;
angle_theta_atan2_local = 0;
} //End of reset_local-------------------------------------------------------------------------------------------------------------------------

// CALCULATING VELOCITY NEEDED FOR EACH WHEEL TO ACHIVE PATH TO X,Y------------------------------------------------------------------------------------------------------------------
void angular_signal_to_velocity(float constant_velocity, float desired_robot_speed_angular, float radius, float wheel_baseline, float &velocity_needed_R, float &velocity_needed_L){
    velocity_needed_R = (constant_velocity + desired_robot_speed_angular * wheels_baseline/2); //calculate needed velocity in [rad/s] 960 ticks = 6.28 RAD -> 6.28rad/s = obrot/s ***
    velocity_needed_L = (constant_velocity - desired_robot_speed_angular * wheels_baseline/2); //calculate needed velocity in [rad/s]  
  } 
// END of CALCULATING VELOCITY NEEDED FOR EACH WHEEL TO ACHIVE PATH TO X,Y-----------------------------------------------------------------------------------------------------------

// LOW LEVEL PID MOTORS CALCULATIONS BASED ON NEEDED VELOCITY==============================================================================================================================================
void setTarget(float t, float deltat, float velocity_R, float velocity_L){ //taking needed velocity at the moment R/L
    float positionChange[4] = {0.0,0.0,0.0,0.0}; //for new pid motor target  
    if (stop_robot == false){
    positionChange[0] = velocity_L*deltat*pulsesPerMeter; //using velocity_needed_R calculate number of ticks that should occur by now to maintain that speed which aquaire X,Y path [right wheel]
    positionChange[1] = velocity_R*deltat*pulsesPerMeter; ////using velocity_needed_R calculate number of ticks that should occur by now to maintain that speed which aquaire X,Y path [left wheel]                
    positionChange[2] = velocity_R*deltat*pulsesPerMeter; //using velocity_needed_R calculate number of ticks that should occur by now to maintain that speed which aquaire X,Y path [right wheel]
    positionChange[3] = velocity_L*deltat*pulsesPerMeter;
    
    for (int k = 0; k < 4; k++){
    new_target[k] = new_target[k] + positionChange[k];
    target[k] = (long) new_target[k];
    /*
    if(abs(target[k] - number_of_pulses [k]) > 100){
    target[k] = 100;  //prevent long overshoot start after being stuck
    }  
    */
    }
    }
} 
// END of LOW LEVEL PID MOTORS CALCULATIONS BASED ON NEEDED VELOCITY=======================================================================================================================================

// PWM TO MOTORS USING LOW LEVEL PID AND CONTROL ROBOT---------------------------------------------------------------------------------------------------------------------------------------------------
 void pwm_to_motors(){
  for(int k = 0; k < 2; k++){ //calculating PWM signals to set velocity of the wheels using low level PID
    int pwr, dir;
    // evaluate the control signal
    if (state == 2) {
    pid[k].evalu(number_of_pulses[k],target[k],deltaT,pwr,dir); // evaluate the control signal
    }
    if (state == 3) {
    pid[k].evalu(number_of_pulses_local[k],target[k],deltaT,pwr,dir); // evaluate the control signal      
    }
    if (state == 4){ //rotate 
    pid[2].evalu_R(angle_theta_degrees_local_int,desired_angle_local,deltaT,pwr,dir,rotate_error);   
    }
    if (state == 5){
    pid[k].evalu(number_of_pulses_local[k],target[k],deltaT,pwr,dir);  
    }
    if (state == 7){
    pid[k].evalu(number_of_pulses_local[k],target[k],deltaT,pwr,dir);  
    }

    if (state == 13){
    pid[k].evalu(number_of_pulses_local[k],target[k],deltaT,pwr,dir);   
    }
    // signal the motor
    if (stop_robot == true){  //if robot must stop set 0 pwr signal on both engines
    setMotor(dir,0,in1[k],in2[k]);  
    }
    else {
    if (state == 4){ //rotate 
    setMotor(dir,pwr,in1[0],in2[0]);
    setMotor(-dir,pwr,in1[1],in2[1]);
    }
    else{
    setMotor(dir,pwr,in1[k],in2[k]);
    }
  }
  }
 } 
// END of PWM TO MOTORS USING LOW LEVEL PID AND CONTROL ROBOT----------------------------------------------------------------------------------------------------------------------------------------------

//TRANSFERING PWM FROM PID TO MOTORS======================================================================================================================================================================= 
void setMotor(int dir, int pwmVal, int in1, int in2){ //in1 (4,5) in2 (6,7)
  analogWrite(in1,pwmVal);
  if(dir == -1){
    if (in2 == 4){
    digitalWrite(in2,LOW); //digital state vary for each motor, these are for FORWARD movement
    }
    if (in2 == 7){
      digitalWrite(in2,HIGH);
    }
  }
  else if(dir == 1){
    if (in2 == 4){
    digitalWrite(in2,HIGH); //low?
    }
    if (in2 == 7){
      digitalWrite(in2,LOW); //high?
    }   
  }
} 
//END of TRANSFERING PWM FROM PID TO MOTORS==================================================================================================================================================================

//READ TICKS FORM ENCODERS-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void readEncoder1(){
  int ticks = digitalRead(encb[0]);
  if(ticks > 0){
    newest_number_of_pulses[0]++;
    newest_number_of_pulses_local[0]++;
  }
  else{
    newest_number_of_pulses[0]--; //right - 0 left - 1
    newest_number_of_pulses_local[0]--; //right - 0 left - 1
  }
  if (newest_number_of_pulses_local_clean == 1){
  newest_number_of_pulses_local[0] = 0;
  newest_number_of_pulses_local_clean = 0;  
  }
}
  void readEncoder2(){
  int ticks = digitalRead(encb[1]);
  if(ticks > 0){
    newest_number_of_pulses[1]--;
    newest_number_of_pulses_local[1]--;
  }
  else{
    newest_number_of_pulses[1]++;
    newest_number_of_pulses_local[1]++;
  }
  if (newest_number_of_pulses_local_clean == 1){
  newest_number_of_pulses_local[1] = 0;
  newest_number_of_pulses_local_clean = 0;  
  }
} 
//END of READ TICKS FORM ENCODERS-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------  

//CALCULATING GLOBAL ODOMETRY - distances and angles=================================================================================================================================================================
void odometry_global(){
  for(int i = 0; i < 2; i++){
  distance_traveled_total[i] = (float) 2*pi*radius*(number_of_pulses[i]/pulsesPerTurn); //updating distance traveled by each wheel since the beginning in [m]
  dist_change[i] = distance_traveled_total[i] - distance_traveled_prev[i]; // updating distance that each wheel traveled since last measure
  distance_traveled_prev[i] = distance_traveled_total[i]; //updating for calculating difference of distance each measurement
  }
  arc_traveled = ((distance_traveled_total[0] + distance_traveled_total[1]) / 2.0 ); //calculating distance traveled by robot since beginning
  arc_traveled_change = ((dist_change[0]+dist_change[1]) / 2.0); //calculating distance traveled by robot since last measure
  angle_phi = ((dist_change[1] - dist_change[0]) / wheels_baseline);  // calculate robot's change in angle 
  angle_theta += angle_phi; // update final angle between current and starting position 
  angle_theta_atan2 = atan2(sin(angle_theta),cos(angle_theta)); //-pi to +pi angular pos 
  angle_theta_degrees = angle_theta_atan2 * 180 / pi;
  angle_theta_degrees_int = (int)angle_theta_degrees; 
  //if (angle_theta > 2.0 * pi) angle_theta -= 2.0 * pi; //constrain theta to the range 0 to 2 pi
  //if (angle_theta < 0.0) angle_theta += 2.0 * pi; // constrain theta to the range 0 to 2 pi 
  xPosition += arc_traveled_change * cos(angle_theta); // update robot's x and y coordinates
  yPosition += arc_traveled_change * sin(angle_theta); // update robot's x and y coordinates  
} 
//END of CALCULATING GLOBAL ODOMETRY - distances and angles============================================================================================================================================================  

//CALCULATING LOCAL ODOMETRY - distances and angles=================================================================================================================================================================
//just for now solution; busted
void odometry_local(){
  for(int i = 0; i < 2; i++){
  distance_traveled_total_local[i] = (float) 2*pi*radius*(number_of_pulses_local[i]/pulsesPerTurn); //updating distance traveled by each wheel since the beginning in [m]
  dist_change_local[i] = distance_traveled_total_local[i] - distance_traveled_prev_local[i]; // updating distance that each wheel traveled since last measure
  distance_traveled_prev_local[i] = distance_traveled_total_local[i]; //updating for calculating difference of distance each measurement
  }
  arc_traveled_local = ((distance_traveled_total_local[0] + distance_traveled_total_local[1]) / 2.0 ); //calculating distance traveled by robot since beginning
  arc_traveled_change_local = ((dist_change_local[0]+dist_change_local[1]) / 2.0); //calculating distance traveled by robot since last measure
  angle_phi_local = ((dist_change_local[1] - dist_change_local[0]) / wheels_baseline);  // calculate robot's change in angle 
  angle_theta_local += angle_phi_local; // update final angle between current and starting position 
  angle_theta_atan2_local = atan2(sin(angle_theta_local),cos(angle_theta_local)); //-pi to +pi angular pos 
  angle_theta_degrees_local = angle_theta_atan2_local * 180 / pi;
  angle_theta_degrees_local_int = (int)angle_theta_degrees_local; 
  //if (angle_theta > 2.0 * pi) angle_theta -= 2.0 * pi; //constrain theta to the range 0 to 2 pi
  //if (angle_theta < 0.0) angle_theta += 2.0 * pi; // constrain theta to the range 0 to 2 pi 
  xPosition_local += arc_traveled_change_local * cos(angle_theta_local); // update robot's x and y coordinates
  yPosition_local += arc_traveled_change_local * sin(angle_theta_local); // update robot's x and y coordinates  
} 
//END of CALCULATING LOCAL ODOMETRY - distances and angles============================================================================================================================================================  

//CALCULATING CURRENT SPEEDS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_speed(float t, float deltat){
  for(int i = 0; i < 2; i++){ //calculating speeds for each wheel
  actual_rotation_speed[i] = (((number_of_pulses[i]-prev_number_of_pulses[i])/pulsesPerTurn)/deltat); // [rotations / sec ]
  actual_angular_speed[i] = (number_of_pulses[i]-prev_number_of_pulses[i])*((2*PI)/pulsesPerTurn)/deltat; //[rad / s]
  actual_velocity_speed[i] = radius * actual_angular_speed[i]; // [m/s]
  prev_number_of_pulses[i] = number_of_pulses[i]; //overwrite with new value of pulses
  } 
  }
//END of CALCULATING CURRENT SPEEDS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//SENDING INSTRUCTIONS VIA SERIAL====================================================================================================================
void maintainDisplay()
{
  static const unsigned long REFRESH_INTERVAL = 50; // ms
  static unsigned long lastRefreshTime = 0;
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
  {
    lastRefreshTime += REFRESH_INTERVAL;                      
    float xPOScm=xPosition*100;
    float yPOScm=yPosition*100;
    int xSend = xPOScm;
    int ySend = yPOScm;
    float xPOScm_local=xPosition_local*100;
    float yPOScm_local=yPosition_local*100;
    int xSend_local = xPOScm_local;
    int ySend_local = yPOScm_local;
    float Xd_local_100 = Xd_local*100;
    float Yd_local_100 = Yd_local*100;
    int Xd_local_int = int(Xd_local_100);
    int Yd_local_int = int(Yd_local_100);
    
    Serial.print(comma_status); //sending status of movement //state
    Serial.print(",");
    Serial.print(xSend); //sending actual position using orignal frame of reference 
    Serial.print(",");
    Serial.print(ySend);
    Serial.print(",");
    Serial.print(angle_theta_degrees_int);
    Serial.print(",");
    Serial.print(xSend_local); //sending actual position using orignal frame of reference 
    Serial.print(",");
    Serial.print(ySend_local);
    Serial.print(",");
    Serial.print(angle_theta_degrees_local_int);
    Serial.print(",");
    Serial.print(message_status); //+1 when message has been read 
    Serial.print(",");
    Serial.print(objective_status); //+1 when objective is aquired
    Serial.print(",");
    Serial.print(state); //+1 when comma has been read //comma_status
    Serial.print(",");
    Serial.print(DistanceMeasured); 
    /*
    Serial.print(",");
    Serial.print(Xd_local_int); 
    Serial.print(",");
    Serial.print(Yd_local_int); 
    */
    //Serial.print(",");
    //Serial.print(velocity_needed_R);
    //Serial.print(",");
    //Serial.print(velocity_needed_L);
    //Serial.print(",");
    //Serial.print(rotate_error);
    Serial.println("");
    
  }
}
//END of SENDING INSTRUCTIONS VIA SERIAL=============================================================================================================

//PUSHING NEW DATA VIA SERIAL-------------------------------------------------------------------------------------------------------------------------------
void pushNewData(int d) {
    data[dataI] = d;
    dataI++;
    if(dataI > n-1) {
        dataI = 0;
    }
}
//END of PUSHING NEW DATA VIA SERIAL------------------------------------------------------------------------------------------------------------------------

//RECIVE DATA VIA SERIAL====================================================================================================================================
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
//END of RECIVE DATA VIA SERIAL==============================================================================================================================

//ONCODE DATA FROM THE SERIAL---------------------------------------------------------------------------------------------------------------------------------
void parseData() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    //strtokIndx = strtok(tempChars,",");      // get the first part - the string
    //strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
    strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
    integerFromPC1 = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ",");
    integerFromPC2 = atoi(strtokIndx);     // convert this part to a int
    strtokIndx = strtok(NULL, ",");
    integerFromPC3 = atoi(strtokIndx);
    }
//END of ONCODE DATA FROM THE SERIAL--------------------------------------------------------------------------------------------------------------------------

//PROTECTING NEW DATA=========================================================================================================================================
void protect_data(){
if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        newData = false;
    }
}
//END of PROTECTING NEW DATA==================================================================================================================================

//SONAR------------------------------

void sonar(int &DistanceMeasured){
digitalWrite(URTRIG, LOW);
digitalWrite(URTRIG, HIGH);              
float Dist;
  unsigned long LowLevelTime = pulseIn(URECHO, LOW) ;
  if (LowLevelTime >= 10000)              // the reading is invalid.
  {
    Dist = -1;
  }
  else
  {
    Dist = LowLevelTime / 50;  // every 50us low level stands for 1cm
    DistanceMeasured = (int)Dist;
  }
}

//PRINTING RESULTS FOR TESTS OR MATLAB--------------------------------------------------------------------------------------------------------------------------------------------------------
void printing_matlab() {
  long currT = micros();
  if (currT - prev_time > time_interval){
  prev_time = currT;
  //Serial.print("Error: ");
  //Serial.print(error_angle); //***
  //Serial.print("||");
  Serial.print("actual X,Y: ");
  Serial.print(xPosition);
  Serial.print(",");
  Serial.print(yPosition);
  Serial.print("||");
  Serial.print("desired Xd,Yd: ");
  Serial.print(Xd);
  Serial.print(",");
  Serial.print(Yd);
  Serial.println(); 
  }   
} 
//END of PRINTING RESULTS FOR TESTS OR MATLAB------------------------------------------------------------------------------------------------------------------------------------------------
