/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/lidar.h>

// #include <base.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32


/////just gonna put in tiny_math.h here

/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   mathematical functions
 */

#ifndef TINY_MATH_H
#define TINY_MATH_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double u;
  double v;
} Vector2;

typedef struct {
  double u;
  double v;
  double w;
} Vector3;

typedef struct {
  Vector3 a;
  Vector3 b;
  Vector3 c;
} Matrix33;

// --- Vector3 functions ---
void vector3_set_values(Vector3 *vect, double u, double v, double w);

// --- Matrix33 functions ---
void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw);
void matrix33_set_identity(Matrix33 *m);
void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v);  // res = m * v

// --- Vector2 functions ---
double vector2_norm(const Vector2 *v);                                 // ||v||
void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2);  // v = v1-v2
double vector2_angle(const Vector2 *v1, const Vector2 *v2);            // angle between v1 and v2 -> [0, 2Pi]

// --- Other ---
double bound(double v, double a, double b);

#ifdef __cplusplus
}
#endif

#endif


///////end of tiny_math.h


////just gonna put in tiny_math.c here
/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// #include "tiny_math.h"
#include <math.h>

void vector3_set_values(Vector3 *vect, double u, double v, double w) {
  vect->u = u;
  vect->v = v;
  vect->w = w;
}

void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw) {
  vector3_set_values(&(m->a), au, av, aw);
  vector3_set_values(&(m->b), bu, bv, bw);
  vector3_set_values(&(m->c), cu, cv, cw);
}

void matrix33_set_identity(Matrix33 *m) {
  matrix33_set_values(m, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v) {
  res->u = m->a.u * v->u + m->b.u * v->v + m->c.u * v->w;
  res->v = m->a.v * v->u + m->b.v * v->v + m->c.v * v->w;
  res->w = m->a.w * v->u + m->b.w * v->v + m->c.w * v->w;
}

double vector2_norm(const Vector2 *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double vector2_angle(const Vector2 *v1, const Vector2 *v2) {
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}

////end of tiny_math.c


////just gonna put in base.h in here so I can compile and start working on stuff
void base_init();

void base_reset();
void base_forwards();
void base_backwards();
void base_turn_left();
void base_turn_right();
void base_strafe_left();
void base_strafe_right();

void base_move(double vx, double vy, double omega);
void base_forwards_increment();
void base_backwards_increment();
void base_turn_left_increment();
void base_turn_right_increment();
void base_strafe_left_increment();
void base_strafe_right_increment();

void base_goto_init(double time_step);
void base_goto_set_target(double x, double y, double a);
void base_goto_run();
bool base_goto_reached();



////end of base.h


//////then gonna put in base.c here so I can compile and start working on stuff
/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Implement the functions defined in base.h
 */

// #include "base.h"

// #include "tiny_math.h"

#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define SPEED 4.0
#define MAX_SPEED 0.3
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0

typedef struct {
  Vector2 v_target;
  double alpha;
  bool reached;
} goto_struct;

static WbDeviceTag wheels[4];
static WbDeviceTag gps;
static WbDeviceTag compass;
static goto_struct goto_data;

static double robot_vx = 0.0;
static double robot_vy = 0.0;
static double robot_omega = 0.0;

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(const double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_init() {
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
  robot_vx = 0.0;
  robot_vy = 0.0;
  robot_omega = 0.0;
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_move(double vx, double vy, double omega) {
  double speeds[4];
  speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega);
  speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega);
  speeds[2] = 1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega);
  speeds[3] = 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega);
  base_set_wheel_speeds_helper(speeds);
  printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
}

void base_forwards_increment() {
  robot_vx += SPEED_INCREMENT;
  robot_vx = robot_vx > MAX_SPEED ? MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_backwards_increment() {
  robot_vx -= SPEED_INCREMENT;
  robot_vx = robot_vx < -MAX_SPEED ? -MAX_SPEED : robot_vx;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_left_increment() {
  robot_omega += SPEED_INCREMENT;
  robot_omega = robot_omega > MAX_SPEED ? MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_turn_right_increment() {
  robot_omega -= SPEED_INCREMENT;
  robot_omega = robot_omega < -MAX_SPEED ? -MAX_SPEED : robot_omega;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_left_increment() {
  robot_vy += SPEED_INCREMENT;
  robot_vy = robot_vy > MAX_SPEED ? MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_strafe_right_increment() {
  robot_vy -= SPEED_INCREMENT;
  robot_vy = robot_vy < -MAX_SPEED ? -MAX_SPEED : robot_vy;
  base_move(robot_vx, robot_vy, robot_omega);
}

void base_goto_init(double time_step) {
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps)
    wb_gps_enable(gps, time_step);
  if (compass)
    wb_compass_enable(compass, time_step);
  if (!gps || !compass)
    fprintf(stderr, "cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double y, double alpha) {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = x;
  goto_data.v_target.v = y;
  goto_data.alpha = alpha;
  goto_data.reached = false;
}

void base_goto_run() {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);

  // compute 2d vectors
  Vector2 v_gps = {gps_raw_values[0], gps_raw_values[1]};
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle(&v_front, &v_north);
  double delta_angle = theta - goto_data.alpha;

  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = -v_right.u;
  transform.a.v = v_front.u;
  transform.b.u = v_right.v;
  transform.b.v = -v_front.v;
  transform.c.u = v_right.u * v_gps.u - v_right.v * v_gps.v;
  transform.c.v = -v_front.u * v_gps.u + v_front.v * v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

  // compute the speeds
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  // -> first stimulus: delta_angle
  speeds[0] = -delta_angle / M_PI * K1;
  speeds[1] = delta_angle / M_PI * K1;
  speeds[2] = -delta_angle / M_PI * K1;
  speeds[3] = delta_angle / M_PI * K1;

  // -> second stimulus: u coord of the relative target vector
  speeds[0] += v_target_rel.u * K2;
  speeds[1] += v_target_rel.u * K2;
  speeds[2] += v_target_rel.u * K2;
  speeds[3] += v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] += -v_target_rel.v * K3;
  speeds[1] += v_target_rel.v * K3;
  speeds[2] += v_target_rel.v * K3;
  speeds[3] += -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i = 0; i < 4; i++) {
    speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
    speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)

    // added an arbitrary factor increasing the convergence speed
    speeds[i] *= 30.0;
    speeds[i] = bound(speeds[i], -SPEED, SPEED);
  }
  base_set_wheel_speeds_helper(speeds);

  // check if the taget is reached
  if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
    goto_data.reached = true;
}

bool base_goto_reached() {
  return goto_data.reached;
}

// /////////end of base.c

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

// static void automatic_behavior() {
  // passive_wait(2.0);
  // passive_wait(4.0);
  // passive_wait(1.0);
  // passive_wait(3.0);
  // passive_wait(1.0);
  // base_strafe_left();
  // passive_wait(5.0);
  // base_reset();
  // passive_wait(1.0);
  // base_turn_left();
  // passive_wait(1.0);
  // base_reset();
  // passive_wait(3.0);
  // passive_wait(1.0);
  // passive_wait(2.0);
  // passive_wait(4.0);
  // passive_wait(2.0);
  // passive_wait(1.0);
  // passive_wait(2.0);
  // passive_wait(2.0);
  // passive_wait(2.0);
// }

// static void display_helper_message() {
  // printf("\n \nControl commands:\n");
  // printf(" Arrows:         Move the robot\n");
  // printf(" Page Up/Down:   Rotate the robot\n");
  // printf(" Space:          Reset\n");
// }

//gonna need to find a way to statically allocate these later
 typedef struct {
   const float *dist_front;
   const float *dist_back;
   const float *dist_left;
   const float *dist_right;
    
 }lidar_distances_t;
 

void get_lidar_distances(WbDeviceTag front_tag,WbDeviceTag back_tag,
WbDeviceTag left_tag, WbDeviceTag right_tag  ,lidar_distances_t * lidar_dist){
  
  lidar_dist->dist_front = wb_lidar_get_range_image(front_tag);
  
  lidar_dist->dist_back = wb_lidar_get_range_image(back_tag);
  lidar_dist->dist_left = wb_lidar_get_range_image(left_tag);
  lidar_dist->dist_right = wb_lidar_get_range_image(right_tag);
  
}

int main(int argc, char **argv) {
  wb_robot_init();
  // void wb_lidar_enable(WbDeviceTag tag, int sampling_period);
  WbDeviceTag front_lidar = wb_robot_get_device("front_lidar");
  WbDeviceTag back_lidar = wb_robot_get_device("back_lidar");
  WbDeviceTag left_lidar = wb_robot_get_device("left_lidar");
  WbDeviceTag right_lidar = wb_robot_get_device("right_lidar");
 
  wb_lidar_enable(front_lidar,1000);
  wb_lidar_enable(back_lidar,1000);
  wb_lidar_enable(left_lidar,1000);
  wb_lidar_enable(right_lidar,1000);
  
  typedef struct {
    char dir;
    int distance;
  }movement_instruction_t;


  movement_instruction_t move_instrs[100];
  move_instrs[0].dir = 'u';
  move_instrs[0].distance = 2;
  move_instrs[1].dir = 'r';
  move_instrs[1].distance = 4;
  move_instrs[2].dir = 'd';
  move_instrs[2].distance = 3;
  move_instrs[3].dir = 'l';
  move_instrs[3].distance = 5;
 
 movement_instruction_t curr_move_instr;

 
 
  lidar_distances_t starting_lidar_distances;
  lidar_distances_t curr_lidar_distances;
  // lidar_distance_t * starting_lidar_distance_pointer = & starting_lidar_distance;
  
  int move_instr_limit = 4;
  int move_instr_index = -1;//starts at zero for first pass

  typedef enum  {robot_init,
  read_in_movement_instructions,get_starting_lidar_distances,
  get_lidar_distances_during_movement, do_movement ,stop_movement, robot_end_state } robot_state_t;
  
  robot_state_t robot_state = robot_init;

  
  base_init();
  passive_wait(2.0);

  // if (argc > 1 && strcmp(argv[1], "demo") == 0)
    // automatic_behavior();

  // display_helper_message();

  // int pc = 0;
  // wb_keyboard_enable(TIME_STEP);

  while (true) {
    step();//if time is -1 call wb_robot_cleanup to end

    //ok so should put in some proto code for reading in the pattern
    //maybe do the same thing saying direction and time? but wouldn't I want it to be more precise and 
    //be the set distance each time
    //maybe have it set as     
    
    
    //ok so state machine is like this

    //first read from
    
    switch(robot_state){//actions and transitions
      
      case robot_init:
         robot_state = read_in_movement_instructions;
         break;

      case read_in_movement_instructions:
        
        move_instr_index += 1;
        
        if (move_instr_index < move_instr_limit){
        
          curr_move_instr.dir = move_instrs[move_instr_index].dir;
          curr_move_instr.distance = move_instrs[move_instr_index].distance;
          robot_state = get_starting_lidar_distances;
        
        }
        
        else{
          robot_state = robot_end_state;
        }
        
        break;
      
      case get_starting_lidar_distances:
        //get_starting_robot_distance
        
        get_lidar_distances(front_lidar,back_lidar,left_lidar,right_lidar, &starting_lidar_distances);


        robot_state = do_movement;
        break;

      case get_lidar_distances_during_movement:
        
        
        //check if it's more than the movement instruction current index

        //only need to check the current dir lidar distance
        switch(curr_move_instr.dir){
            case 'u':

              break;
            case 'd':
            
              break;
            case 'l':
            
              break;
            case 'r':
            
              break;
          }     
        
        
        break;

      case do_movement:
      
      
      //need to redesign how to move, maybe move continously at a set speed
      //
          switch(curr_move_instr.dir){
            case 'u':
              base_forwards();
              break;
            case 'd':
             base_backwards();
              break;
            case 'l':
              base_strafe_left();
              break;
            case 'r':
              base_strafe_right();
              break;
          } 
          
          
          robot_state = get_lidar_distances_during_movement;
          break;

      case stop_movement:
          base_reset();
          break;
      case robot_end_state:
      
          break;
      default:
          robot_state = robot_init;
          break;
   }

	
    
    // int c = wb_keyboard_get_key();
    // if ((c >= 0) && c != pc) {
      // switch (c) {
        // case WB_KEYBOARD_UP:
          // base_forwards_increment();
          // break;
        // case WB_KEYBOARD_DOWN:
          // base_backwards_increment();
          // break;
        // case WB_KEYBOARD_LEFT:
          // base_strafe_left_increment();
          // break;
        // case WB_KEYBOARD_RIGHT:
          // base_strafe_right_increment();
          // break;
        // case WB_KEYBOARD_PAGEUP:
          // base_turn_left_increment();
          // break;
        // case WB_KEYBOARD_PAGEDOWN:
          // base_turn_right_increment();
          // break;
        // case WB_KEYBOARD_END:
        // case ' ':
          // printf("Reset\n");
          // base_reset();
          // break;
        // case '+':
        // case 388:
        // case 65585:
          // break;
        // case '-':
        // case 390:
          // break;
        // case 332:
        // case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          // break;
        // case 326:
        // case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          // break;
        // case 330:
        // case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          // break;
        // case 328:
        // case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          // break;
        // default:
          // fprintf(stderr, "Wrong keyboard input\n");
          // break;
      // }
    // }
    // pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
