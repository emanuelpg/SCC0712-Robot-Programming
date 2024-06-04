/*
* Copyright 1996-2022 Cyberbotics Ltd.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
/*
* Description: Example of Sick LMS 291.
* 		 The velocity of each wheel is set
* 		 according to a Braitenberg-like algorithm which takes the values returned by the Sick as input.
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#define TIME_STEP 32
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define M_PI 3.14159265358979323846
#define CILINDRO_POS_X 29.0
#define CILINDRO_POS_Y -29.0
#define STEP_POS 10
// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}
// Delay function
void delay(double secs){
  double start = wb_robot_get_time();
  double end = start;
  while(end - start < secs){
    wb_robot_step(1);
    end = wb_robot_get_time();

  }
}

double get_distance(const double *position){
  double dist_x = CILINDRO_POS_X - position[0];
  double dist_y = CILINDRO_POS_Y - position[1];
  return sqrt(pow(dist_x, 2) + pow(dist_y, 2));
}

void move_to_goal(const double *position, double *previous_pos, double* back_left_speed, double* back_right_speed,
double* front_left_speed, double* front_right_speed){
  // Variables
  long double robot_dir_x;
  long double robot_dir_y;
  long double robot_dir_mod;
  long double goal_dir_x;
  long double goal_dir_y;
  long double goal_dir_mod;
  long double escalar_prod;
  long double angle;
  long double cos_value;
  double delay_time;
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");
  // Computations
  robot_dir_x = (position[0] - previous_pos[0] + 0.00001) ;
  robot_dir_y = (position[1] - previous_pos[1] + 0.00001);
  robot_dir_mod = sqrt(pow(robot_dir_x, 2) + pow(robot_dir_y, 2));
  goal_dir_x = (CILINDRO_POS_X - position[0] + 0.00001);
  goal_dir_y = (CILINDRO_POS_Y - position[1] + 0.00001);
  goal_dir_mod = sqrt(pow(goal_dir_x, 2) + pow(goal_dir_y, 2));
  escalar_prod = robot_dir_x*goal_dir_x + robot_dir_y*goal_dir_y;
  cos_value = escalar_prod / (goal_dir_mod*robot_dir_mod);
  angle = acos(cos_value);
  delay_time = 1.1*angle;
  //printf("cosValue: %.2Lf; Angle: %.2Lf; robotMod: %.2Lf e goalMod: %.2Lf\n", cos_value, angle, robot_dir_mod, goal_dir_mod);
  if(isnan(angle)){
    angle = 0;
  }
  // Moving to goal
  if(angle < 0.5){
    *back_left_speed = CRUISING_SPEED;
    *back_right_speed = CRUISING_SPEED;
    *front_left_speed = CRUISING_SPEED;
    *front_right_speed = CRUISING_SPEED;
    printf("Seguindo reto, angulo de %.2Lf\n", angle);
  }else if( angle <= M_PI/2 ){
    *back_left_speed = 0;
    *back_right_speed = CRUISING_SPEED;
    *front_left_speed = 0;
    *front_right_speed = CRUISING_SPEED;
    // set actuators
    wb_motor_set_velocity(front_left_wheel, *front_left_speed);
    wb_motor_set_velocity(front_right_wheel, *front_right_speed);
    wb_motor_set_velocity(back_left_wheel, *back_left_speed);
    wb_motor_set_velocity(back_right_wheel, *back_right_speed);
    printf("virando a esquerda, angulo de %.2Lf\n", angle);
    delay(delay_time);
    /*
    *back_left_speed = CRUISING_SPEED;
    *back_right_speed = CRUISING_SPEED;
    *front_left_speed = CRUISING_SPEED;
    *front_right_speed = CRUISING_SPEED;
    */
  }else{
    *back_left_speed = CRUISING_SPEED;
    *back_right_speed = 0;
    *front_left_speed = CRUISING_SPEED;
    *front_right_speed = 0;
    // set actuators
    wb_motor_set_velocity(front_left_wheel, *front_left_speed);
    wb_motor_set_velocity(front_right_wheel, *front_right_speed);
    wb_motor_set_velocity(back_left_wheel, *back_left_speed);
    wb_motor_set_velocity(back_right_wheel, *back_right_speed);
    printf("virando a direita, angulo de %.2Lf\n", angle);
    delay(delay_time);
    /*
    *back_left_speed = CRUISING_SPEED;
    *back_right_speed = CRUISING_SPEED;
    *front_left_speed = CRUISING_SPEED;
    *front_right_speed = CRUISING_SPEED;
    */
  }
  //delay(0.5);
}

void avoid_obstacle(const double obstacle, double left_obstacle, double right_obstacle, 
double *front_left_speed, double *front_right_speed, double *back_left_speed, double *back_right_speed){
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");
  const double speed_factor = (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle;
  *front_left_speed = speed_factor * left_obstacle;
  *front_right_speed = speed_factor * right_obstacle;
  *back_left_speed = BACK_SLOWDOWN * (*front_left_speed);
  *back_right_speed = BACK_SLOWDOWN * (*front_right_speed);
  // set actuators
  wb_motor_set_velocity(front_left_wheel, *front_left_speed);
  wb_motor_set_velocity(front_right_wheel, *front_right_speed);
  wb_motor_set_velocity(back_left_wheel, *back_left_speed);
  wb_motor_set_velocity(back_right_wheel, *back_right_speed);
  printf("Avoiding Obstacle...\n");
  //delay(0.5); // Time to avoid obstacle
}




int main(int argc, char **argv) {
int continua = 1;
int obstacle_bool = 0; 
// init webots stuff
wb_robot_init();
// get devices
WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");
// init lms291
wb_lidar_enable(lms291, TIME_STEP);
const int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
const int half_width = lms291_width / 2;
const int max_range = wb_lidar_get_max_range(lms291);
const double range_threshold = max_range / 20.0;
const float *lms291_values = NULL;
// init braitenberg coefficient
double *const braitenberg_coefficients = (double *)malloc(sizeof(double) * lms291_width);
int i, j;
for (i = 0; i < lms291_width; ++i)
braitenberg_coefficients[i] = gaussian(i, half_width, lms291_width / 5);
// init GPS
/* get and enable GPS device */
WbDeviceTag gps = wb_robot_get_device("gps");
wb_gps_enable(gps, TIME_STEP);
if (wb_gps_get_coordinate_system(gps) == WB_GPS_WGS84_COORDINATE)
	 printf("GPS is using 'WGS84' coordinates system\n");
// init motors
wb_motor_set_position(front_left_wheel, INFINITY);
wb_motor_set_position(front_right_wheel, INFINITY);
wb_motor_set_position(back_left_wheel, INFINITY);
wb_motor_set_position(back_right_wheel, INFINITY);
// init speed for each wheel
double back_left_speed = 0.0, back_right_speed = 0.0;
double front_left_speed = 0.0, front_right_speed = 0.0;
wb_motor_set_velocity(front_left_wheel, front_left_speed);
wb_motor_set_velocity(front_right_wheel, front_right_speed);
wb_motor_set_velocity(back_left_wheel, back_left_speed);
wb_motor_set_velocity(back_right_wheel, back_right_speed);
// init dynamic variables
double left_obstacle = 0.0, right_obstacle = 0.0;
// Initial position
const double *position = wb_gps_get_values(gps);
double previous_pos[2] = {position[0], position[1]};
int step_aux = 0;

// control loop
while (wb_robot_step(TIME_STEP) != -1 && continua==1) {
	// get lidar values
	lms291_values = wb_lidar_get_range_image(lms291);
	// apply the braitenberg coefficients on the resulted values of the lms291
	// near obstacle sensed on the left side
	for (i = 0; i < half_width; ++i) {
	if (lms291_values[i] < range_threshold){ // far obstacles are ignored
	left_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[i] / max_range);
	}
	// near obstacle sensed on the right side
	j = lms291_width - i - 1;
	if (lms291_values[j] < range_threshold)
	right_obstacle += braitenberg_coefficients[i] * (1.0 - lms291_values[j] / max_range);
	}
	// overall front obstacle
	const double obstacle = left_obstacle + right_obstacle;
	// compute the speed according to the information on
	// Get current position
	position = wb_gps_get_values(gps);
	// obstacles
	if (obstacle > OBSTACLE_THRESHOLD){
             obstacle_bool = 1;
             avoid_obstacle(obstacle, left_obstacle, right_obstacle, &front_left_speed, &front_right_speed,
             &back_left_speed, &back_right_speed);
              // Save current position
              previous_pos[0] = position[0];
              previous_pos[1] = position[1]; 
            }else{
              back_left_speed = CRUISING_SPEED;
              back_right_speed = CRUISING_SPEED;
              front_left_speed = CRUISING_SPEED;
              front_right_speed = CRUISING_SPEED;
              wb_motor_set_velocity(front_left_wheel, front_left_speed);
              wb_motor_set_velocity(front_right_wheel, front_right_speed);
              wb_motor_set_velocity(back_left_wheel, back_left_speed);
              wb_motor_set_velocity(back_right_wheel, back_right_speed);
            }
            
            // reset dynamic variables to zero
            left_obstacle = 0.0;
            right_obstacle = 0.0;
            printf("distance: %.2f\n", get_distance(position));
            if(get_distance(position) < 3.5){
              // set actuators
              wb_motor_set_velocity(front_left_wheel, 0);
              wb_motor_set_velocity(front_right_wheel, 0);
              wb_motor_set_velocity(back_left_wheel, 0);
              wb_motor_set_velocity(back_right_wheel, 0);
              printf("Congrats! You achieved your goal point in %.2f seconds!\n", wb_robot_get_time());
              continua = 0;
              break;
            }
            // Move towards goal
            if(step_aux % STEP_POS == 0 && continua==1 && obstacle_bool==0){
              move_to_goal(position, previous_pos, &back_left_speed, 
              &back_right_speed, &front_left_speed, &front_right_speed); 
              wb_motor_set_velocity(front_left_wheel, front_left_speed);
              wb_motor_set_velocity(front_right_wheel, front_right_speed);
              wb_motor_set_velocity(back_left_wheel, back_left_speed);
              wb_motor_set_velocity(back_right_wheel, back_right_speed);
              // Save current position
              previous_pos[0] = position[0];
              previous_pos[1] = position[1]; 
            }
            obstacle_bool = 0;
}
free(braitenberg_coefficients);
wb_robot_cleanup();
return 0;
}





