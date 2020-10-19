#include "kobukiSensorTypes.h"

// Robot states
// Add your own states here
typedef enum {
  OFF,
  DRIVING,
  TURNING, 
  BACKING, 
  TURNING_45
} robot_state_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);

static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right);

static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right);

static float read_tilt();

static float measure_reverse_distance(uint16_t current_encoder, uint16_t previous_encoder);

bool bumpedIntoSth(KobukiSensors_t* sensors, bool* isRight);


robot_state_t controller(robot_state_t state);
