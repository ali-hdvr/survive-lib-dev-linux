#pragma once

#include <libusb/libusb.h>
#include <hidapi/hidapi.h>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <wchar.h>
#include <stdint.h>
#include <vector>
#include <unordered_map>
#include <zlib.h>
#include "json/json.hpp"
#include <fstream>

#include "controller.h"
#include "utils.h"

#include "io_handler.h"

using namespace std;
#ifdef LINUX
#include <unistd.h>
#endif
#ifdef WINDOWS
#include <windows.h>
#endif

void mySleep(int sleepMs)
{
#ifdef LINUX
    usleep(sleepMs * 1000); // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef WINDOWS
    Sleep(sleepMs);
#endif
}

int main(int argc, char **argv);
