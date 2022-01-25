#include "objects.h"

Lighthouse lighthouses[NUM_LH_CHANNELS];
Tracker *tracker = new Tracker;
SensorActivations *activations = new SensorActivations;

int activeLighthouses = 0;
Phase cur_phase = Phase_Calibration;
int lighthouses_in_universe = 0;
bool is_universe_known = false;

bool is_tracker_stationary = true;