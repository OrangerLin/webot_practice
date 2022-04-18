/*
 * File:          my_robot.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */


#include <webots/robot.h>
#include <webots/motor.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <webots/position_sensor.h>


#define TIME_STEP 32
#define rad_2_deg(X) ( X / pi * 180.0 )
#define deg_2_rad(X) ( X / 180.0 * pi )
#define pi 3.1415926
#define T 1


int main(int argc, char **argv)
{
  
  WbDeviceTag servos[8];
  const char *SERVO_NAMES[] = { 
   "fore_left_1",
   "fore_right_1", 
   "hind_left_1", 
   "hind_right_1", 
   "fore_left_2",
   "fore_right_2", 
   "hind_left_2", 
   "hind_right_2", 
    NULL };
    
   //左前腿
  float angle_motor_fore_left_1;  //大腿电机转动角度
  float angle_motor_fore_left_2;  //小腿电机转动角度
  float x_fore_left = -0;       
  float y_fore_left = -20;
  float L_fore_left_2 = 16; //小腿
  float L_fore_left_1 = 17.4; //大腿
  float S_fore_left;
   //右前腿
  float angle_motor_fore_right_1;  //大腿电机转动角度
  float angle_motor_fore_right_2;  //小腿电机转动角度
  float x_fore_right = -0;       
  float y_fore_right = -20;
  float L_fore_right_2 = 16; //小腿
  float L_fore_right_1 = 17.4; //大腿
  float S_fore_right; 
     //左前腿
  float angle_motor_hind_left_1;  //大腿电机转动角度
  float angle_motor_hind_left_2;  //小腿电机转动角度
  float x_hind_left = -0;       
  float y_hind_left = -20;
  float L_hind_left_2 = 16; //小腿
  float L_hind_left_1 = 17.4; //大腿
  float S_hind_left;
   //右前腿
  float angle_motor_hind_right_1;  //大腿电机转动角度
  float angle_motor_hind_right_2;  //小腿电机转动角度
  float x_hind_right = -0;       
  float y_hind_right = -20;
  float L_hind_right_2 = 16; //小腿
  float L_hind_right_1 = 17.4; //大腿
  float S_hind_right; 
    
  double t = 0.0;
  int jump_state = 0;
  wb_robot_init();
  printf("robot inited!\n");


  
  int i;
  for (i = 0; SERVO_NAMES[i]; i++) {
    servos[i] = wb_robot_get_device(SERVO_NAMES[i]);
    assert(servos[i]);
  }


  while (wb_robot_step(TIME_STEP) != -1) {
    //左前腿
    S_fore_left = x_fore_left*x_fore_left+y_fore_left*y_fore_left;
    angle_motor_fore_left_1 = -asin(y_fore_left/sqrt(S_fore_left)) - acos((S_fore_left+L_fore_left_2*L_fore_left_2-L_fore_left_1*L_fore_left_1)/(2*sqrt(S_fore_left)*L_fore_left_2));
    angle_motor_fore_left_2 = acos((S_fore_left-L_fore_left_2*L_fore_left_2-L_fore_left_1*L_fore_left_1)/(-2*L_fore_left_2*L_fore_left_1)) - angle_motor_fore_left_1;
     wb_motor_set_position(servos[0], angle_motor_fore_left_2);
     //wb_motor_set_velocity(servos[0], 50);
     //wb_motor_set_acceleration(servos[0], 50);
    // wb_motor_set_available_torque(servos[0], 300);
     wb_motor_set_position(servos[4], -angle_motor_fore_left_1-angle_motor_fore_left_2);
     //wb_motor_set_velocity(servos[4], 50);
    //右前腿
    S_fore_right = x_fore_right*x_fore_right+y_fore_right*y_fore_right;
    angle_motor_fore_right_1 = -asin(y_fore_right/sqrt(S_fore_right)) - acos((S_fore_right+L_fore_right_2*L_fore_right_2-L_fore_right_1*L_fore_right_1)/(2*sqrt(S_fore_right)*L_fore_right_2));
    angle_motor_fore_right_2 = acos((S_fore_right-L_fore_right_2*L_fore_right_2-L_fore_right_1*L_fore_right_1)/(-2*L_fore_right_2*L_fore_right_1)) - angle_motor_fore_right_1;
     wb_motor_set_position(servos[1], angle_motor_fore_right_2);
     wb_motor_set_position(servos[5], -angle_motor_fore_right_1-angle_motor_fore_right_2);
     //左后腿
    S_hind_left = x_hind_left*x_hind_left+y_hind_left*y_hind_left;
    angle_motor_hind_left_1 = -asin(y_hind_left/sqrt(S_hind_left)) - acos((S_hind_left+L_hind_left_2*L_hind_left_2-L_hind_left_1*L_hind_left_1)/(2*sqrt(S_hind_left)*L_hind_left_2));
    angle_motor_hind_left_2 = acos((S_hind_left-L_hind_left_2*L_hind_left_2-L_hind_left_1*L_hind_left_1)/(-2*L_hind_left_2*L_hind_left_1)) - angle_motor_hind_left_1;
     wb_motor_set_position(servos[2], angle_motor_hind_left_2);
     wb_motor_set_position(servos[6], -angle_motor_hind_left_1-angle_motor_hind_left_2);
     //右后腿
    S_hind_right = x_hind_right*x_hind_right+y_hind_right*y_hind_right;
    angle_motor_hind_right_1 = -asin(y_hind_right/sqrt(S_hind_right)) - acos((S_hind_right+L_hind_right_2*L_hind_right_2-L_hind_right_1*L_hind_right_1)/(2*sqrt(S_hind_right)*L_hind_right_2));
    angle_motor_hind_right_2 = acos((S_hind_right-L_hind_right_2*L_hind_right_2-L_hind_right_1*L_hind_right_1)/(-2*L_hind_right_2*L_hind_right_1)) - angle_motor_hind_right_1;
     wb_motor_set_position(servos[3], angle_motor_hind_right_2);
     wb_motor_set_position(servos[7], -angle_motor_hind_right_1-angle_motor_hind_right_2);
     
     
     t += (double)TIME_STEP / 1000.0;
     
     if(jump_state < 100)
     {
       jump_state = jump_state + 1;
     }
     else
     {
       jump_state=0;
     }
     if(jump_state > 50)
     {
       y_hind_left = -30;
       y_hind_right = -30;
       y_fore_left = -30;
       y_fore_right = -30;
     }
     else
     {
       y_hind_left = -10;
       y_hind_right = -10;
       y_fore_left = -10;
       y_fore_right = -10;
     }
     
    
  };
  
  wb_robot_cleanup();
  
  return 0;
}
