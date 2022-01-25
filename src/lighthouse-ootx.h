/**
 *  This class decodes an incoming bitstream from the individual data bits sent by the
 *  lighthouse base station.
 *
 *  Details:  First, we will be looking for a preamble, that is a binary sequence of 17 zeros
 *            and 1 one. Then, we read the length of the payload and then the payload.
 *
 *  OOTX Frame: details of the format of OOTX frames and also the code base of this class are
 *              adopted from nairol (https://github.com/nairol) - thanks for the documentation
 *              and the code !!
 *              https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
 *
 *  Gordon Wetzstein
 *  gordon.wetzstein@stanford.edu
 *  Stanford University
 *  September 12, 2017
 *
 */

#pragma once
#include <iostream>
#include <math.h>
#include <fstream>
#include <iomanip>
#include "json/json.hpp"
#include <sstream>
#include "redist/linmath.h"

using namespace std;
typedef nlohmann::json json;

#define PI 3.14159265358979323846

extern std::unordered_map<int, int> lh_idx_ch_map;

class LighthouseOOTX
{

  //////////////////////////////////////////////////////////////////////////////////////////
  // private variables
private:
  bool displayed_decoding_message = false;
  // flag that indicates if preamble of OOTX frame was received (data is only being recorded after)
  bool waiting_for_preamble;

  // flag that indicates if the length of the payload was received (this is the second 16 bit
  // chunck transmitted after the preamble
  bool waiting_for_length;

  // variable to store incoming bitstream in
  // even though any set of 2 bytes is read separately, we need to check if the 17th bit (the sync
  // bit) is set correctly, so let's use unsigned long datatype (32 bit) instead of unsigned short (16 bits)
  unsigned long accumulator;

  // count number of bits collected in accumulator
  unsigned accumulator_bits;

  // counter for current data byte that's being written to the list
  unsigned rx_bytes;
  unsigned padding; // if there is a padding byte

  // length of payload in bytes
  unsigned length;

  // flag that indicates if the entire payload was read
  bool complete;

  // ootx data already configured in file
  bool ootxCheckedinFile;

  // this is a buffer with the actual data bytes; does not contain the sync bits
  unsigned char bytes[256];

  const string filename_ootx = "lighthouses.json";
  fstream file_ootx;
  json ootx_json;

  json single_lh_ootx = {
      {"id_steamvr", "-1"},
      {"id_lighthosuedb", "-1"},
      {"mode", "-1"},
      {"index", "-1"},
      {"OOTXSet", "0"},
      {"PoseSet", "0"},
      {"pose", {"0", "0", "0", "0", "0", "0", "0"}},
      {"accelerometer", {"0", "0", "0"}},
      {"pitch", "0"},
      {"roll", "0"},
      {"fcalphase", {"0", "0"}},
      {"fcaltilt", {"0", "0"}},
      {"fcalcurve", {"0", "0"}},
      {"fcalgibpha", {"0", "0"}},
      {"fcalgibmag", {"0", "0"}},
      {"fcalogeephase", {"0", "0"}},
      {"fcalogeemag", {"0", "0"}}};

  //////////////////////////////////////////////////////////////////////////////////////////
  // public variables
public:
  // pitch and roll angles in degrees (only available afer the entire ootx frame is read at least once)

  // flag that indicates whether payload was completely read at least once
  bool bCompleteOnce;

  // if OOTX available, either calculated or read form file
  bool isOOTXavailable;

  bool isPoseSet = false;
  int index;
  LinmathPose lh_pose = {{0}, {0}}; // pose in universe

  unsigned int lh_id;

  double baseStationPitch = 0.0;
  double baseStationRoll = 0.0;

  int bs_mode;

  float fcal_phase[2];
  float fcal_tilt[2];

  int sys_unlock_count;
  int hw_version;

  float fcal_curve[2];
  float fcal_gibphase[2];
  float fcal_gibmag[2];
  int sys_faults;
  float fcal_ogeephase[2];
  float fcal_ogeemag[2];

  int fw_version;
  int protocol_version;

  double accx;
  double accy;
  double accz;

  //////////////////////////////////////////////////////////////////////////////////////////
  // private functions
private:
  // something went wrong when trying to decode the bitstream, start again
  void reset();

  // read a 16 bit chung of data, add to list
  void add_word(unsigned long word);

  // flip the order of the last two bytes in this 32 bit sequence (do not reverse bit order)
  unsigned long flipByteOrder(unsigned long bitsequence);

  void fillOOTXFromFile(int channel);

  void addLHinFile();

  //////////////////////////////////////////////////////////////////////////////////////////
  // public functions
public:
  // constructor
  LighthouseOOTX(

  );

  void fillOOTXFromJSON(json ootx_json, int channel);

  // sets pose and index of lighthouse
  void setLHPose(LinmathPose pose_in_univ, int channel);

  // add an incoming data bit for decoding
  void addBit(unsigned long bit);

  // see if OOTX info is available
  bool isOOTXInfoAvailable(int channel)
  {
    bs_mode = channel;
    if (!ootxCheckedinFile)
    {
      readFromFileIfAvailable(channel);
    }

    return isOOTXavailable;
  }

  int getBaseStationMode();
  void readFromFileIfAvailable(int channel);
  void resetOOTXFile();

  template <typename T>
  std::string int_to_hex(T i)
  {
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(sizeof(T) * 2)
           << std::hex << i;
    return stream.str();
  }

  std::string getHexID();
};
