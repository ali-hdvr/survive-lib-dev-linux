#pragma once
#include "definitions.h"

extern Lighthouse lighthouses[NUM_LH_CHANNELS];
extern Tracker *tracker;
extern SensorActivations *activations;

extern int activeLighthouses;

extern int lighthouses_in_universe;
extern bool is_universe_known;

extern Phase cur_phase;

extern bool is_tracker_stationary;