#include "controller.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION true
#define PI 3.14159265

#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"
#endif

// Configure initial state
KobukiSensors_t sensors = {0};

// You may need to add additional variables to keep track of state here
float distance = 0;
float angle = 0;
float tilt = 0;
bool driveUp = true;;
bool cliff_is_right = false;

uint16_t previous_encoder;// = sensors.leftWheelEncoder;
int16_t leftSpeed, rightSpeed;

// Return distance traveled between two encoder values
static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder, bool dir) {
  const float CONVERSION = 0.0006108;
  if(dir)
  {
    if(current_encoder >= previous_encoder)
    {
     return (current_encoder - previous_encoder) * CONVERSION;
    }
    else{
      // overflow occur;
      return ((0xFFFF - previous_encoder) + current_encoder) * CONVERSION;
    }
  } else {
    if(current_encoder <= previous_encoder)
    {
     return (previous_encoder - current_encoder) * CONVERSION;
    }
    else{
      // overflow occur;
      return (previous_encoder + 0xFFFF -  current_encoder) * CONVERSION;
    }
  }
  return 0;
  // Your code here
}

static bool obstacle_detected(KobukiSensors_t sensors)
{
  return sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpCenter  || sensors.bumps_wheelDrops.bumpRight;
}

static int get_bump_angle(KobukiSensors_t sensors)
{
  return (sensors.bumps_wheelDrops.bumpLeft? -25:25);
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right) {
  // Your code here
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right) {
  // Your code here
  *cliff_is_right = (sensors->cliffRight || sensors->cliffCenter)? true : false;
  return (sensors->cliffCenter || sensors->cliffLeft || sensors->cliffRight);
}

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
static float read_tilt() {
  // Your code here
  lsm9ds1_measurement_t accVal = lsm9ds1_read_accelerometer();
  float degree = atan(accVal.y_axis / sqrt(accVal.x_axis * accVal.x_axis + accVal.z_axis * accVal.z_axis)) * 180 / PI; 
  printf("Degree: %f\n", degree);
  return degree;
}


// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop
robot_state_t controller(robot_state_t state) {

  kobukiSensorPoll(&sensors);
  tilt = read_tilt();
    // handle states
    switch(state) {
      case OFF: {
        printf("OFF\n");
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = ORIENT_UP;
          driveUp = true;
          leftSpeed = 70;
          rightSpeed = -70;
        } else {    
          state = OFF;
          kobukiDriveDirect(0, 0);
        }
        
        break; // each case needs to end with break!
      }
      case ORIENT_UP: {
        printf("Orient Up\n");
        if(is_button_pressed(&sensors)) {
          state = OFF;
        } else if (driveUp && (tilt <= -4)){
          state = DRIVING;
          leftSpeed = 70;
          rightSpeed = 70;
        } else {
          kobukiDriveDirect(leftSpeed, rightSpeed);
        }
        break;
      }
      case DRIVING: {
        printf("Driving\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (fabs(tilt) > 2 && check_cliff(&sensors, &cliff_is_right)) { 
          distance = 0;
          leftSpeed = 0;
          rightSpeed = 0;
          previous_encoder = sensors.leftWheelEncoder;
          state = AVOID;
        } else if (driveUp && fabs(tilt) < 1) {
          distance = 0;
          leftSpeed = 80;
          rightSpeed = 80;
          previous_encoder = sensors.leftWheelEncoder;
          state = TRANSIENT;
        }
        else {
          kobukiDriveDirect(leftSpeed, rightSpeed);
          
          state = DRIVING;
        }
        break; // each case needs to end with break!
      }
      case TRANSIENT: {
        printf("Transient\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (check_cliff(&sensors, &cliff_is_right)) { 
          distance = 0;
          leftSpeed = 0;
          rightSpeed = 0;
          previous_encoder = sensors.leftWheelEncoder;
          state = AVOID;
        } else if (distance > 0.5) {
          lsm9ds1_start_gyro_integration();
          angle = 0;
          leftSpeed = 80;
          rightSpeed = -80;
          state = ORIENT_DOWN;
        }
        else {
          kobukiDriveDirect(leftSpeed, rightSpeed);
          distance = measure_distance(sensors.leftWheelEncoder, previous_encoder, true);
          state = TRANSIENT;
        }
        break; // each case needs to end with break!
        break;
      }
      case ORIENT_DOWN: {
        printf("Orient Down\n");
        if(is_button_pressed(&sensors)) {
          state = OFF;
        } else if (fabs(angle) > 135) {
          state = DRIVING;
          leftSpeed = 80;
          rightSpeed = 80;
        } else {
          driveUp = false;
          kobukiDriveDirect(leftSpeed, rightSpeed);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          printf("%f\n", angle);
          state = ORIENT_DOWN;

        }
        break;
      }
      case BACK: {
        printf("BACK\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if(fabs(distance) >= 0.1) {
          lsm9ds1_start_gyro_integration();
          angle = 0;
          state = TURN; 
        } else {
          kobukiDriveDirect(leftSpeed, rightSpeed);
          distance = measure_distance(sensors.leftWheelEncoder, previous_encoder, false);
        }
        break;
      }
      case TURN: {
        printf("TURN\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
        } else if (fabs(angle) >= 25) {          
          distance = 0;
          angle = 0;
          
          leftSpeed = 80;
          rightSpeed = 80;
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
        }
        else {
          // display_write("TURN", DISPLAY_LINE_0);
          if(!cliff_is_right) kobukiDriveDirect(50, -50);
          else kobukiDriveDirect(-50, 50);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          printf("%f\n", angle);
          state = TURN;
        }
        break;
      }
      case AVOID: {
        printf("AVOID\n");
        kobukiDriveDirect(leftSpeed, rightSpeed);
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if(sensors.leftWheelEncoder > previous_encoder) {
          previous_encoder = sensors.leftWheelEncoder;
          state = AVOID;
        } else {
          leftSpeed = -80;
          rightSpeed = -80;
          state = BACK;
        } 
        break;
      }

    }
    return state;
}

