////////////////////////////////////////////////////////////////////////////////////////////
//  This class decodes an incoming bitstream from the individual data bits sent by the
//  lighthouse base station.
//
//  Details:  First, we will be looking for a preamble, that is a binary sequence of 17 zeros
//            and 1 one. Then, we read the length of the payload and then the payload.
//
//  OOTX Frame: details of the format of OOTX frames and also the code base of this class are
//              adopted from nairol (https://github.com/nairol) - thanks for the documentation
//              and the code !!
//              https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
//
//  Gordon Wetzstein
//  gordon.wetzstein@stanford.edu
//  Stanford University
//  September 12, 2017
//
////////////////////////////////////////////////////////////////////////////////////////////
#include "lighthouse-ootx.h"

std::unordered_map<int, int> lh_idx_ch_map;

////////////////////////////////////////////////////////////////////////////////////////////
// constructor - reset all variables

LighthouseOOTX::LighthouseOOTX()
{
  reset();
  complete = 0;
  length = 0;
  bCompleteOnce = false;
  ootxCheckedinFile = false;
}

#ifndef _MSC_VER
struct __attribute__((__packed__)) unaligned_u16_t
{
  uint16_t v;
};
#else
struct unaligned_u16_t
{
  uint16_t v;
};
#endif

union iFloat
{
  uint32_t i;
  float f;
};

float _half_to_float(uint8_t *data)
{
  uint16_t x = ((struct unaligned_u16_t *)data)->v;
  union iFloat fnum;
  fnum.f = 0;

  //sign
  fnum.i = (((uint32_t)x & 0x8000) << 16);

  if ((x & 0x7FFF) == 0)
    return fnum.f; //signed zero

  if ((x & 0x7c00) == 0)
  {
    //denormalized
    x = (x & 0x3ff) << 1; //only mantissa, advance intrinsic bit forward
    uint8_t e = 0;
    //shift until intrinsic bit of mantissa overflows into exponent
    //increment exponent each time
    while ((x & 0x0400) == 0)
    {
      x <<= 1;
      e++;
    }
    fnum.i |= ((uint32_t)(112 - e)) << 23;   //bias exponent to 127, half floats are biased 15 so only need to go 112 more.
    fnum.i |= ((uint32_t)(x & 0x3ff)) << 13; //insert mantissa
    return fnum.f;
  }

  if ((x & 0x7c00) == 0x7c00)
  {
    //for infinity, fraction is 0
    //for NaN, fraction is anything non zero
    //we could just copy in bits and not shift, but the mantissa of a NaN can have meaning
    fnum.i |= 0x7f800000 | ((uint32_t)(x & 0x3ff)) << 13;
    return fnum.f;
  }

  fnum.i |= ((((uint32_t)(x & 0x7fff)) + 0x1c000u) << 13);

  return fnum.f;
}

////////////////////////////////////////////////////////////////////////////////////////////
// reset all variables and start scanning bitsequence

void LighthouseOOTX::reset()
{
  waiting_for_preamble = 1;
  waiting_for_length = 1;
  accumulator = 0;
  accumulator_bits = 0;
  rx_bytes = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////
// add a detected databit to the sequence

void LighthouseOOTX::addBit(unsigned long bit)
{
  if (!displayed_decoding_message)
  {
    cout << "Decoding OOTX for Lighthouse at channel " << bs_mode << endl;
    displayed_decoding_message = true;
  }

  cout << bit << std::flush;

  if (bit != 0 && bit != 1)
  {
    // something is wrong.  dump what we have received so far
    reset();
    return;
  }

  // add this bit to our incoming word
  accumulator = (accumulator << 1) | bit;
  accumulator_bits++;

  //////////////////////////////////////////////////////////////////////////////////////////
  // before doing anything else, wait for the preamble of
  // 17 zeros and 1 one

  if (waiting_for_preamble)
  {

    // 17 zeros, followed by a 1 == 18 bits
    if (accumulator_bits != 18)
    {
      return;
    }

    if (accumulator == 0x1)
    {
      // received preamble, start on data
      // first we'll need the length
      waiting_for_preamble = 0;
      waiting_for_length = 1;

      //      Serial.print("success: received preamble: ");
      //      Serial.println(accumulator,BIN);

      accumulator = 0;
      accumulator_bits = 0;
      return;
    }

    // we've received 18 bits worth of preamble,
    // but it is not a valid thing.  hold onto the
    // last 17 bits worth of data
    accumulator_bits--;
    accumulator = accumulator & 0x1FFFF;
    return;
  }

  ////////////////////////////////////////////////////////////////////////////////////////

  // we're receiving data!  accumulate until we get a sync bit
  if (accumulator_bits != 17)
    return;

  if ((accumulator & 1) == 0)
  {

    //    Serial.println("reset due to sync bit :( !");

    // no sync bit. go back into waiting for preamble mode
    reset();
    return;
  }

  // hurrah!  the sync bit was set
  unsigned long word = accumulator >> 1;
  accumulator = 0;
  accumulator_bits = 0;

  add_word(word);
}

//////////////////////////////////////////////////////////////////////////////////////////
// add a 16 bit / 2 byte sequence to the detected payload

void LighthouseOOTX::add_word(unsigned long word)
{

  if (waiting_for_length)
  {

    // these bits are coming in wwith  the least significant bit first!
    // let's flip the order so that
    word = flipByteOrder(word);

    length = word + 4; // add in the CRC32 length
    padding = length & 1;
    waiting_for_length = 0;
    rx_bytes = 0;

    // error!
    if (length > sizeof(bytes))
    {
      printf("WARNING: length of payload seems questionable: ");
      // Serial.println(word);
      length = 33; // just set it to 33 by default
      //reset();
    }

    return;
  }

  bytes[rx_bytes++] = (word >> 8) & 0xFF;
  bytes[rx_bytes++] = (word >> 0) & 0xFF;

  if (rx_bytes < length + padding)
    return;

  // we are at the end!

  // save base station pitch and roll from bytes 20 and 22
  //accelerometer acc axis: z points back, y is normal to top face
  accx = double(int8_t(bytes[20])) / 127.0;
  accy = double(int8_t(bytes[21])) / 127.0;
  accz = double(int8_t(bytes[22])) / 127.0;

  double acc_norm = sqrt(accx * accx + accy * accy + accz * accz);
  accx = accx / acc_norm;
  accy = accy / acc_norm;
  accz = accz / acc_norm;

  double signAccy = double((accy > 0) - (accy < 0));
  baseStationRoll = 360 * -atan2(-accx, accy) / (2 * PI);
  baseStationPitch = 360 * -atan2(accz, signAccy * sqrt(accx * accx + accy * accy)) / (2 * PI);

  bs_mode = (unsigned)(bytes[31] & 0x7F);

  lh_id = (bytes[5] << 24) + (bytes[4] << 16) + (bytes[3] << 8) + bytes[2];

  fcal_phase[0] = _half_to_float(&bytes[6]);
  fcal_phase[1] = _half_to_float(&bytes[8]);

  fcal_tilt[0] = _half_to_float(&bytes[10]);
  fcal_tilt[1] = _half_to_float(&bytes[12]);

  fcal_curve[0] = _half_to_float(&bytes[16]);
  fcal_curve[1] = _half_to_float(&bytes[18]);

  fcal_gibphase[0] = _half_to_float(&bytes[23]);
  fcal_gibphase[1] = _half_to_float(&bytes[25]);

  fcal_gibmag[0] = _half_to_float(&bytes[27]);
  fcal_gibmag[1] = _half_to_float(&bytes[29]);

  sys_unlock_count = (int)bytes[14];

  sys_faults = (unsigned)(bytes[32]);
  fcal_ogeephase[0] = _half_to_float(&bytes[33]);
  fcal_ogeephase[1] = _half_to_float(&bytes[35]);
  fcal_ogeemag[0] = _half_to_float(&bytes[37]);
  fcal_ogeemag[1] = _half_to_float(&bytes[39]);

  unsigned short first16bitvariable = (bytes[1] << 8) + bytes[0];
  int fw_version = (first16bitvariable & 0xFFC0) >> 6;
  int protocol_version = (first16bitvariable & 0x3F);

  //baseStationRoll     = 90.0 * double(int8_t(bytes[20]))/127.0;
  //baseStationPitch    = -90.0 * double(int8_t(bytes[22]))/127.0;

  complete = 1;
  bCompleteOnce = true;
  isOOTXavailable = true;
  waiting_for_length = 1;

  if (bCompleteOnce)
  {
    cout << endl
         << "OOTX decoded for Channel " << bs_mode << endl;
    single_lh_ootx["id_lighthosuedb"] = to_string(lh_id);
    single_lh_ootx["id_steamvr"] = int_to_hex(lh_id);
    single_lh_ootx["mode"] = to_string(bs_mode);
    single_lh_ootx["OOTXSet"] = to_string(isOOTXavailable);
    single_lh_ootx["accelerometer"][0] = to_string(accx);
    single_lh_ootx["accelerometer"][1] = to_string(accy);
    single_lh_ootx["accelerometer"][2] = to_string(accz);
    single_lh_ootx["pitch"] = to_string(baseStationPitch);
    single_lh_ootx["roll"] = to_string(baseStationRoll);

    single_lh_ootx["fcalphase"][0] = to_string(fcal_phase[0]);
    single_lh_ootx["fcalphase"][1] = to_string(fcal_phase[1]);

    single_lh_ootx["fcaltilt"][0] = to_string(fcal_tilt[0]);
    single_lh_ootx["fcaltilt"][1] = to_string(fcal_tilt[1]);

    single_lh_ootx["fcalcurve"][0] = to_string(fcal_curve[0]);
    single_lh_ootx["fcalcurve"][1] = to_string(fcal_curve[1]);

    single_lh_ootx["fcalgibpha"][0] = to_string(fcal_gibphase[0]);
    single_lh_ootx["fcalgibpha"][1] = to_string(fcal_gibphase[1]);

    single_lh_ootx["fcalgibmag"][0] = to_string(fcal_gibmag[0]);
    single_lh_ootx["fcalgibmag"][1] = to_string(fcal_gibmag[1]);

    single_lh_ootx["fcalogeephase"][0] = to_string(fcal_ogeephase[0]);
    single_lh_ootx["fcalogeephase"][1] = to_string(fcal_ogeephase[1]);

    single_lh_ootx["fcalogeemag"][0] = to_string(fcal_ogeemag[0]);
    single_lh_ootx["fcalogeemag"][1] = to_string(fcal_ogeemag[1]);

    addLHinFile();
  }

  // reset to wait for a preamble
  reset();
}

//////////////////////////////////////////////////////////////////////////////////////////
// flip order of the last two bytes in a 32 bit variables
// this seems necessary to reliably decode the data

unsigned long LighthouseOOTX::flipByteOrder(unsigned long bitsequence)
{
  return (bitsequence >> 8) | ((bitsequence & 0xFF) << 8);
}

// Handling file IO

void LighthouseOOTX::addLHinFile()
{
  if (file_ootx.is_open())
  {
    file_ootx.close();
  }
  file_ootx.open(filename_ootx, fstream::in | fstream::out | fstream::app);
  if (file_ootx.is_open())
  {

    try
    {

      file_ootx >> ootx_json;
      file_ootx.close();
      file_ootx.open(filename_ootx, fstream::in | fstream::out | fstream::trunc);
    }
    catch (const std::exception &exc)
    {

      std::cerr << exc.what();
      cout << "\nError in extracting " << filename_ootx << ".json. Reinitializing new file...\n";
      ootx_json.clear();
      file_ootx.close();
      file_ootx.open(filename_ootx, fstream::in | fstream::out | fstream::trunc);
    }

    ootx_json.push_back(single_lh_ootx);
    cout << setw(4) << ootx_json << endl;

    file_ootx << std::setw(4) << ootx_json << std::endl;
    file_ootx.close();
    cout << "Updated " << filename_ootx << " saved.\n";
  }
  else
  {
    cout << "OOTX file could not be opened.\n";
  }
}

void LighthouseOOTX::fillOOTXFromJSON(json ootx_json, int channel)
{
  ootxCheckedinFile = true;

  try
  {

    // file_ootx >> ootx_json;
    for (auto &lhjson : ootx_json.items())
    {
      // std::cout << "key: " << lhjson.key() << ", value: " << lhjson.value() << '\n';
      int mode = stoi((string)lhjson.value().at("mode"));
      index = stoi((string)lhjson.value().at("index"));

      if (mode == channel)
      {

        cout << "\nReading ootx for ch: " << mode << endl;

        bs_mode = mode;

        index = stoi((string)lhjson.value().at("index"));
        if (index != -1)
        {
          lh_idx_ch_map[index] = mode;
        }

        lh_id = stoul((string)lhjson.value().at("id_lighthosuedb"));

        baseStationPitch = stod((string)lhjson.value().at("pitch"));
        baseStationRoll = stod((string)lhjson.value().at("roll"));

        accx = stod((string)lhjson.value().at("accelerometer")[0]);
        accy = stod((string)lhjson.value().at("accelerometer")[1]);
        accz = stod((string)lhjson.value().at("accelerometer")[2]);

        fcal_phase[0] = stod((string)lhjson.value().at("fcalphase")[0]);
        fcal_phase[1] = stod((string)lhjson.value().at("fcalphase")[1]);

        fcal_tilt[0] = stod((string)lhjson.value().at("fcaltilt")[0]);
        fcal_tilt[1] = stod((string)lhjson.value().at("fcaltilt")[1]);

        fcal_curve[0] = stod((string)lhjson.value().at("fcalcurve")[0]);
        fcal_curve[1] = stod((string)lhjson.value().at("fcalcurve")[1]);

        fcal_gibphase[0] = stod((string)lhjson.value().at("fcalgibpha")[0]);
        fcal_gibphase[1] = stod((string)lhjson.value().at("fcalgibpha")[1]);

        fcal_gibmag[0] = stod((string)lhjson.value().at("fcalgibmag")[0]);
        fcal_gibmag[1] = stod((string)lhjson.value().at("fcalgibmag")[1]);

        fcal_ogeephase[0] = stod((string)lhjson.value().at("fcalogeephase")[0]);
        fcal_ogeephase[1] = stod((string)lhjson.value().at("fcalogeephase")[1]);

        fcal_ogeemag[0] = stod((string)lhjson.value().at("fcalogeemag")[0]);
        fcal_ogeemag[1] = stod((string)lhjson.value().at("fcalogeemag")[1]);
        isOOTXavailable = true;
        isPoseSet = stoi((string)lhjson.value().at("PoseSet"));

        lh_pose.Pos[0] = stod((string)lhjson.value().at("pose")[0]);
        lh_pose.Pos[1] = stod((string)lhjson.value().at("pose")[1]);
        lh_pose.Pos[2] = stod((string)lhjson.value().at("pose")[2]);

        lh_pose.Rot[0] = stod((string)lhjson.value().at("pose")[3]);
        lh_pose.Rot[1] = stod((string)lhjson.value().at("pose")[4]);
        lh_pose.Rot[2] = stod((string)lhjson.value().at("pose")[5]);
        lh_pose.Rot[3] = stod((string)lhjson.value().at("pose")[6]);
      }
    }
  }
  catch (const std::exception &exc)
  {

    cout << "Error: " << exc.what() << endl;

    cout << "Error in extracting values from ootx json. need to decode again \n";
    // cin.get();
    //exit(-1);
  }
}

void LighthouseOOTX::fillOOTXFromFile(int channel)
{

  if (file_ootx.is_open())
  {

    try
    {

      file_ootx >> ootx_json;
      for (auto &lhjson : ootx_json.items())
      {
        // std::cout << "key: " << lhjson.key() << ", value: " << lhjson.value() << '\n';
        int mode = stoi((string)lhjson.value().at("mode"));
        index = stoi((string)lhjson.value().at("index"));

        if (mode == channel)
        {

          cout << "\nReading ootx for ch: " << mode << endl;

          bs_mode = mode;

          index = stoi((string)lhjson.value().at("index"));
          if (index != -1)
          {
            lh_idx_ch_map[index] = mode;
          }

          lh_id = stoul((string)lhjson.value().at("id_lighthosuedb"));

          baseStationPitch = stod((string)lhjson.value().at("pitch"));
          baseStationRoll = stod((string)lhjson.value().at("roll"));

          accx = stod((string)lhjson.value().at("accelerometer")[0]);
          accy = stod((string)lhjson.value().at("accelerometer")[1]);
          accz = stod((string)lhjson.value().at("accelerometer")[2]);

          fcal_phase[0] = stod((string)lhjson.value().at("fcalphase")[0]);
          fcal_phase[1] = stod((string)lhjson.value().at("fcalphase")[1]);

          fcal_tilt[0] = stod((string)lhjson.value().at("fcaltilt")[0]);
          fcal_tilt[1] = stod((string)lhjson.value().at("fcaltilt")[1]);

          fcal_curve[0] = stod((string)lhjson.value().at("fcalcurve")[0]);
          fcal_curve[1] = stod((string)lhjson.value().at("fcalcurve")[1]);

          fcal_gibphase[0] = stod((string)lhjson.value().at("fcalgibpha")[0]);
          fcal_gibphase[1] = stod((string)lhjson.value().at("fcalgibpha")[1]);

          fcal_gibmag[0] = stod((string)lhjson.value().at("fcalgibmag")[0]);
          fcal_gibmag[1] = stod((string)lhjson.value().at("fcalgibmag")[1]);

          fcal_ogeephase[0] = stod((string)lhjson.value().at("fcalogeephase")[0]);
          fcal_ogeephase[1] = stod((string)lhjson.value().at("fcalogeephase")[1]);

          fcal_ogeemag[0] = stod((string)lhjson.value().at("fcalogeemag")[0]);
          fcal_ogeemag[1] = stod((string)lhjson.value().at("fcalogeemag")[1]);
          isOOTXavailable = true;
          isPoseSet = stoi((string)lhjson.value().at("PoseSet"));

          lh_pose.Pos[0] = stod((string)lhjson.value().at("pose")[0]);
          lh_pose.Pos[1] = stod((string)lhjson.value().at("pose")[1]);
          lh_pose.Pos[2] = stod((string)lhjson.value().at("pose")[2]);

          lh_pose.Rot[0] = stod((string)lhjson.value().at("pose")[3]);
          lh_pose.Rot[1] = stod((string)lhjson.value().at("pose")[4]);
          lh_pose.Rot[2] = stod((string)lhjson.value().at("pose")[5]);
          lh_pose.Rot[3] = stod((string)lhjson.value().at("pose")[6]);
        }
      }
    }
    catch (const std::exception &exc)
    {

      cout << "Error: " << exc.what() << endl;

      cout << "Error in extracting values from ootx json. need to decode again \n";
      // cin.get();
      //exit(-1);
    }
    file_ootx.close();
    ootxCheckedinFile = true;
  }
}

void LighthouseOOTX::resetOOTXFile()
{
  if (file_ootx.is_open())
  {
    file_ootx.open(filename_ootx, fstream::in | fstream::app);
    file_ootx.close();
  }
  file_ootx.open(filename_ootx, fstream::in | fstream::out | fstream::trunc);
  file_ootx.close();
}

void LighthouseOOTX::readFromFileIfAvailable(int channel)
{

  file_ootx.open(filename_ootx, fstream::in | fstream::app);

  // If file does not exist, Create new file
  if (!file_ootx)
  {
    cout << "File: " << filename_ootx << " does not exist\n";
  }
  else
  { // use existing file
    cout << "Reading " << filename_ootx << ". \n";
    fillOOTXFromFile(channel);
  }
}

std::string LighthouseOOTX::getHexID()
{
  return int_to_hex(lh_id);
}

void LighthouseOOTX::setLHPose(LinmathPose pose_in_univ, int channel)
{
  isPoseSet = true;

  file_ootx.open(filename_ootx, fstream::in | fstream::app);
  if (file_ootx.is_open())
  {

    try
    {

      file_ootx >> ootx_json;
      file_ootx.close();
      file_ootx.open(filename_ootx, fstream::in | fstream::out | fstream::trunc);
      int max_index_in_univ = -1;
      for (auto &lhjson : ootx_json.items())
      {
        // std::cout << "key: " << lhjson.key() << ", value: " << lhjson.value() << '\n';
        // int mode = stoi((string)lhjson.value().at("mode"));
        int index = stoi((string)lhjson.value().at("index"));

        if (index > max_index_in_univ)
        {
          max_index_in_univ = index;
        }
      }

      for (auto &lhjson : ootx_json.items())
      {
        // std::cout << "key: " << lhjson.key() << ", value: " << lhjson.value() << '\n';
        int mode = stoi((string)lhjson.value().at("mode"));

        if (mode == channel)
        {

          lhjson.value().at("pose")[0] = to_string(pose_in_univ.Pos[0]);
          lhjson.value().at("pose")[1] = to_string(pose_in_univ.Pos[1]);
          lhjson.value().at("pose")[2] = to_string(pose_in_univ.Pos[2]);

          lhjson.value().at("pose")[3] = to_string(pose_in_univ.Rot[0]);
          lhjson.value().at("pose")[4] = to_string(pose_in_univ.Rot[1]);
          lhjson.value().at("pose")[5] = to_string(pose_in_univ.Rot[2]);
          lhjson.value().at("pose")[6] = to_string(pose_in_univ.Rot[3]);

          isPoseSet = true;
          lhjson.value().at("PoseSet") = to_string(isPoseSet);

          lhjson.value().at("index") = to_string(max_index_in_univ + 1);
          lh_idx_ch_map[max_index_in_univ + 1] = mode;

          cout << "\nReading ootx for ch: " << mode << endl;
        }
      }
      file_ootx << std::setw(4) << ootx_json << std::endl;
      file_ootx.close();
      cout << "Updated " << filename_ootx << " with pose of lighthouse " << channel << endl;
    }
    catch (const std::exception &exc)
    {

      cout << "Error: " << exc.what() << endl;

      cout << "Error in updating pose in lighthouses.json \n";
      // cin.get();
      //exit(-1);
    }
    file_ootx.close();
    ootxCheckedinFile = true;
  }
}