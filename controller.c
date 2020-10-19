#include "controller.h"
#include <math.h>
#include <stdio.h>

#define SIMULATION true

#if SIMULATION
#include "simulatorFunctions.h"
#else
#include "kobukiSensorTypes.h"
#include "display.h"
#endif

// Configure initial state
KobukiSensors_t sensors = {0};

// You may need to add additional variables to keep track of state here

// Return distance traveled between two encoder values

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right) {
  // Your code here
}

// Return true if a cliff has been seen
// Save information about which cliff
static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right) {
  // Your code here
}

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
static float read_tilt() {
  // Your code here
}
static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder)
{
  float ret = 0;
  const float CONVERSION = 0.0006108;
  if (current_encoder >= previous_encoder) 
  {
    ret = (float)(current_encoder - previous_encoder) * CONVERSION;
  }
  else
  {
    ret = (float)(65535+current_encoder-previous_encoder)*CONVERSION;
  }
  return ret;
}
static float measure_reverse_distance(uint16_t current_encoder, uint16_t previous_encoder)
{
  float ret = 0;
  const float CONVERSION = 0.0006108;
  if (current_encoder <= previous_encoder) 
  {
    ret = (float)(current_encoder - previous_encoder) * CONVERSION;
  }
  else
  {
    ret = -(float)((65535-current_encoder)+previous_encoder)*CONVERSION;
  }
  return ret;
}
bool bumpedIntoSth(KobukiSensors_t* sensors, bool* isRight)
{
  *isRight = sensors->bumps_wheelDrops.bumpRight;
  return sensors->bumps_wheelDrops.bumpLeft ||
         sensors->bumps_wheelDrops.bumpCenter ||
         sensors->bumps_wheelDrops.bumpRight;
}

// Robot controller
// State machine running on robot_state_t state
// This is called in a while loop
float dist = 0;
float angle = 0;
bool isRight = false;
uint16_t lastEncoder = 0;

robot_state_t controller(robot_state_t state) {

  kobukiSensorPoll(&sensors);
  float tilt = read_tilt();

    // handle states
    switch(state) {
      case OFF: {
      	printf("OFF\n");
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
          lastEncoder = sensors.leftWheelEncoder;
        } else {
          // perform state-specific actions here
          //display_write("OFF", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
      	printf("DRIVING\n");
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (bumpedIntoSth(&sensors, &isRight))
        {
          state = BACKING;
          dist = 0;
          kobukiDriveDirect(-70, -70);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else if (dist >= 0.5)
        {
          state = TURNING;
          lsm9ds1_start_gyro_integration();
          dist = 0;
          angle = 0;
          kobukiDriveDirect(100, -100);
        } 
        else {
          // perform state-specific actions here
          //display_write("DRIVING", DISPLAY_LINE_0);

          dist += measure_distance(sensors.leftWheelEncoder, lastEncoder);
          lastEncoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(100, 100);
          state = DRIVING;

          //char buf[16];
          //snprintf(buf, 16, "%f", dist);
          //display_write(buf, DISPLAY_LINE_1);
        }
        break; // each case needs to end with break!
      }

      case TURNING:
      {
      	printf("TURNING\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (bumpedIntoSth(&sensors, &isRight))
        {
          state = BACKING;
          dist = 0;
          angle = 0;
          kobukiDriveDirect(-70, -70);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
          lsm9ds1_stop_gyro_integration();
        }
        else if (angle <= -90)
        {
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(100, 100);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else 
        {
          //display_write("TURNING", DISPLAY_LINE_0);
          kobukiDriveDirect(100, -100);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          state = TURNING;

          char buf[16];
          snprintf(buf, 16, "%f", angle);
          //display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }

      case BACKING:
      {
      	printf("BACKING\n");
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (dist <= -0.1)
        {
          state = TURNING_45;
          lsm9ds1_start_gyro_integration();
          dist = 0;
          angle = 0;
          kobukiDriveDirect(0, 0);
        } 
        else {
          // perform state-specific actions here
          //display_write("BACKING", DISPLAY_LINE_0);

          dist += measure_reverse_distance(sensors.leftWheelEncoder, lastEncoder);
          lastEncoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(-70, -70);
          state = BACKING;
          
          //char buf[16];
          //snprintf(buf, 16, "%f", dist);
          //display_write(buf, DISPLAY_LINE_1);
        }
        break; // each case needs to end with break!
      }
      // add other cases here

      case TURNING_45:
      {
      	printf("TURNING_45\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (fabs(angle) >= 45)
        {
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(100, 100);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else 
        {
          //display_write("TURNING_45", DISPLAY_LINE_0);
          if (!isRight) kobukiDriveDirect(60, -60);
          else kobukiDriveDirect(-60, 60);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          state = TURNING_45;

          //char buf[16];
          //snprintf(buf, 16, "%f", angle);
          //display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
    }
    return state;
}
