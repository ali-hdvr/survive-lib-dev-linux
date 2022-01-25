#pragma once

#include <stdint.h>
#include "redist/linmath.h"
#include <string>
#include "json/json.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "boost/asio.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include "spree/interface.h"
#include "io_handler.h"

using namespace boost::asio;
using namespace std;
#define TRACKER_CLK_FREQ ((float)48000000.0)

void sendPoseUDP(std::string objectName, LinmathPose *pose);
void sendPoseUDPU(std::string objectName, const LinmathPose *pose);
