#include "kobukiSensorTypes.h"

// Robot states
// Add your own states here
typedef enum {
  OFF,
  FINDTILT,
  DRIVING,
  ORIENT_UP,
  TRANSIENT,
  ORIENT_DOWN,
  TURN,
  AVOID,
  BACK,
} robot_state_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder, bool dir);

static bool check_and_save_bump(KobukiSensors_t* sensors, bool* obstacle_is_right);

static bool check_cliff(KobukiSensors_t* sensors, bool* cliff_is_right);

static float read_tilt();

static bool obstacle_detected(KobukiSensors_t sensors);

static int get_bump_angle(KobukiSensors_t sensors);

robot_state_t controller(robot_state_t state);