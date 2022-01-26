#include "usb_track.h"

/* MAGIC CODES */
uint8_t magic_turn_on[5] = {0x04, 0x00, 0x00, 0x00, 0x00};
uint8_t magic_enable_more[5] = {0x07, 0x03, 0x00, 0x00, 0x00};
uint8_t magic_enable_raw[5] = {0x04, 0x01, 0x00, 0x00, 0x00};
uint8_t magic_weird[64] = {0x83, 0x00, 0x00, 0x00, 0x00, 0x00,
						   0xf0, 0xbf, 0x00, 0x00, 0x00, 0x00,
						   0x00, 0x00, 0x00, 0x00, 0x80, 0xc3,
						   0x0f, 0x4c, 0x16, 0x02, 0x00, 0x00,
						   0x90, 0x00, 0x00, 0x00, 0x00, 0x00,
						   0x00, 0x00, 0x90, 0xc3, 0x0f, 0x4c,
						   0x16, 0x02, 0x00, 0x00, 0xa0, 0xc3,
						   0x0f, 0x4c, 0x16, 0x02, 0x00, 0x00,
						   0x39, 0x8e, 0xe3, 0x38, 0x8e, 0xe3,
						   0x38, 0x0e, 0x39, 0x8e, 0xe3, 0x38,
						   0x8e, 0xe3, 0x38, 0x0e};

const int BUF_SIZE = 64;
const int W_STR_SIZE = 255;

const bool PRINT_LHD = false;

int VID = 0x28de; // tracker vendor id
int PID = 0x2300; // tracker product id

hid_device *handle_imu;
hid_device *handle_lhd;
hid_device *handle_ctrl;
uint32_t initialTime = 0;

double accBias[3];
double accScale[3];
double gyroBias[3];
double gyroScale[3];

bool enabledMore = false;

nlohmann::json config;

#define POP1 (*(buf++))
#define POP2 (((((struct unaligned_u16_t *)((buf += 2) - 2))))->v)
#define POP4 (((((struct unaligned_u32_t *)((buf += 4) - 4))))->v)

#define AS_SHORT(a, b) ((uint16_t)(((uint16_t)a) << 8) | (0x00ff & b))
#define POP_BYTE(ptr) ((uint8_t) * (ptr++))
#define POP_SHORT(ptr) (((((struct unaligned_u16_t *)((ptr += 2) - 2))))->v)

struct unaligned_16_t
{
	int16_t v;
};
struct unaligned_32_t
{
	int32_t v;
};
struct unaligned_u16_t
{
	uint16_t v;
};
struct unaligned_u32_t
{
	uint32_t v;
};

int8_t oldcode = 0;
uint32_t time_last_imu = 0;

// expands lhd timestamp to full timestamp
uint32_t fix_time24(uint32_t time24, uint32_t reftime)
{
	uint32_t upper_ref = reftime & 0xFF000000u;
	uint32_t lower_ref = reftime & 0x00FFFFFFu;

	if (lower_ref > time24 && lower_ref - time24 > (1 << 23u))
	{
		upper_ref += 0x01000000;
	}
	else if (lower_ref < time24 && time24 - lower_ref > (1 << 23u) && upper_ref > 0)
	{
		upper_ref -= 0x01000000;
	}

	return upper_ref | time24;
}

void fill_lh_ootx()
{
	json ootx_json;
	ifstream fileo;
	fileo.open("lighthouses.json", fstream::in | fstream::app);

	if (fileo.is_open())
	{

		try
		{

			fileo >> ootx_json;

			cout << "read lighthouses file: " << ootx_json.dump() << endl;

			Poser::UpdateLighthouseConfig(ootx_json);
		}
		catch (const std::exception &exc)
		{

			cout << "Error: " << exc.what() << endl;

			cout << "Error in extracting values from ootx json. need to decode again \n";
			// cin.get();
			//exit(-1);
		}
		fileo.close();
	}
}
// function to process lighthouse sweeps
void process_sweep(SWEEP sweep)
{
	Poser::Sweep(sweep);
	// controller_SWEEP(sweep);
	// printSweep(sweep);
	// writeSweepToFile(sweep);
}

// function to process lighthouse syncs
void process_sync(SYNC sync)
{
	if (lighthouses[sync.channel].ootx_handler.isPoseSet)
	{
		sendPoseUDP("4" + to_string(sync.channel), &lighthouses[sync.channel].pose_in_univ);
	}
	// controller_SYNC(sync);
	// printSync(sync);
	// writeSyncToFile(sync);

	int spd = Poser::Sync(sync);

	SpreePoseData cur_pose_data = Poser::QueryPose();

	std::cout << "q ts:" << cur_pose_data.timestamp << " updated:" << cur_pose_data.isUpdated << " Method:"
			  << cur_pose_data.poseMethod << " Pos:" << cur_pose_data.position.transpose() << " Rot:"
			  << cur_pose_data.rotation << std::endl;

	// std::cout << "Sync processed : " << spd << std::endl;

	// std::cout << "ch:" << sync.channel << " update:" << spd.is_updated << " method: " << spd.pose_method
	// 		  << " returned pose: " << spose.Pos[0] << " " << spose.Pos[1] << " " << spose.Pos[2] << " "
	// 		  << spose.Rot[0] << " " << spose.Rot[1] << " " << spose.Rot[2] << " " << spose.Rot[3] << " " << std::endl;

	LinmathPose estimate = {{
								cur_pose_data.position(0),
								cur_pose_data.position(1),
								cur_pose_data.position(2),
							},
							{cur_pose_data.rotation.w(),
							 cur_pose_data.rotation.x(),
							 cur_pose_data.rotation.y(),
							 cur_pose_data.rotation.z()}};

	// quatcopy(estimate.Rot, spose.Rot);

	sendPoseUDP("32", &estimate);
}

// function to process tracker inertial data
void process_imu(IMU imu)
{
	Poser::Inertial(imu);
	// printIMU(imu);
	// writeIMUToFile(imu);
}

// unpack lighthouse data
void unpack_lhd(unsigned char *buf)
{
	int id = POP1;
	if (id == 33)
	{
		if (PRINT_LHD)
			std::cout << "LH1 LIGHTCAP" << endl;
	}
	else if (id == 37)
	{ //v1
		if (PRINT_LHD)
			std::cout << "LH1 LIGHTCAP" << endl;
	}
	else if (id == 39)
	{ // v2
		// this would contain the modulated data
		std::cout << "LH2 Lightcap" << endl;
	}
	else if (id == 40)
	{
		// https://github.com/jdavidberger/lighthouse2tools

		uint8_t *packet = buf + 1;
		uint8_t length = buf[0];

		uint8_t idx = 0;
		uint8_t channel = 255;

		while (idx < length)
		{
			uint8_t data = packet[idx];
			if (data & 0x1u)
			{
				if ((data & 0x0Au) != 0)
				{
					if (PRINT_LHD)
						std::cout << "unsure data" << endl;
					break;
				}

				if (data & 0x04u)
				{
					uint8_t ch = data >> 4u;
					if (PRINT_LHD)
						std::cout << "lighthouses seem to be running on the same channel id: " << unsigned(ch) << endl;
				}

				channel = data >> 4u;
				// channel++; // internally channel is zero based
				idx++;
			}
			else
			{
				uint32_t timecode = 0;
				memcpy(&timecode, packet + idx, sizeof(uint32_t));

				uint32_t reference_time = time_last_imu;

				bool sync = timecode & 0x2u;
				if (!sync)
				{
					bool ootx = (timecode >> 26u) & 1u;
					bool g = (timecode >> 27u) & 1u;
					timecode = fix_time24((timecode >> 2u) & 0xFFFFFFu, reference_time);
					uint8_t unused = timecode >> 28;
					if (channel != 255)
					{
						SYNC data = {timecode, channel, ootx, g};
						process_sync(data);
					}
				}
				else
				{
					bool half_clock_flag = timecode & 0x4u;
					uint8_t sensor = (timecode >> 27u);
					timecode = fix_time24((timecode >> 3u) & 0xFFFFFFu, reference_time);
					if (channel != 255)
					{
						SWEEP data = {timecode, channel, sensor, half_clock_flag};
						process_sweep(data);
					}
				}
				idx += 4;
			}
		}
	}
	else
	{
		if (PRINT_LHD)
			std::cout << "UNHANDLED LH ID: " << id << endl;
	}
}

// unpack imu data
void unpack_imu(unsigned char *buf)
{
	int id = POP1;

	for (int i = 0; i < 3; i++)
	{
		struct unaligned_16_t *sensor = (struct unaligned_16_t *)buf;
		buf += 12;
		uint32_t timecode = POP4;
		uint8_t code = POP1;
		int8_t cd = code - oldcode;

		if (cd > 0)
		{
			oldcode = code;
			double agm[6] = {(sensor[0].v - accBias[0]) * accScale[0], (sensor[1].v - accBias[1]) * accScale[0], (sensor[2].v - accBias[2]) * accScale[2], (sensor[3].v - gyroBias[0]) * gyroScale[0], (sensor[4].v - gyroBias[1]) * gyroScale[1], (sensor[5].v - gyroBias[2]) * gyroScale[2]};

			IMU data = {timecode, {agm[0], agm[1], agm[2]}, {agm[3], agm[4], agm[5]}};
			process_imu(data);
			time_last_imu = timecode;
		}
	}
}

int main(int argc, char **argv)
{

	/* LIBUSB TESTING E.G. CHECKING FOR TRACKER CONNECTED */

	// handles
	libusb_context *context = NULL;
	libusb_device **list = NULL;

	// init libusb
	int rc = libusb_init(&context);
	assert(rc == 0);
	ssize_t count = 0;

	bool trackerConnected = false;

	fill_lh_ootx();
	Poser::SetPoseOnSync(true);

	while (!trackerConnected)
	{
		// get usb devices
		count = libusb_get_device_list(context, &list);
		assert(count > 0);

		// search through devices
		for (size_t idx = 0; idx < count; idx++)
		{
			libusb_device *device = list[idx];
			libusb_device_descriptor desc = {0};

			rc = libusb_get_device_descriptor(device, &desc);
			assert(rc == 0);

			if (desc.idVendor == VID && desc.idProduct == PID)
			{
				std::cout << "Tracker connected: " << desc.idVendor << " " << desc.idProduct << endl;
				trackerConnected = true;
			}
		}
		mySleep(2000);
	}

	libusb_free_device_list(list, count);
	libusb_exit(context);

	/* HIDAPI TESTING E.G. READING HID MESSAGES */
	int res;
	int res2;
	unsigned char buf[BUF_SIZE] = {0};
	unsigned char buf2[BUF_SIZE] = {0};
	uint8_t cfgbuff[256] = {0};
	uint8_t compressed_data[8192] = {0};
	uint8_t uncompressed_data[65536] = {0};
	wchar_t wstr[W_STR_SIZE];

	res = hid_init();

	hid_device_info *info = hid_enumerate(VID, PID);
	hid_device_info *current = info;

	hid_device_info *imu = NULL, *lhd = NULL, *ctrl = NULL;

	while (current)
	{
		// imu runs on interface 0
		if (current->interface_number == 0)
		{
			imu = current;
			wcout << "IMU: " << imu->path << endl;
		}
		// light house data runs on interface 1
		else if (current->interface_number == 1)
		{
			lhd = current;
			wcout << "LHD: " << lhd->path << endl;
		}
		// control inputs run on interface 2
		else if (current->interface_number == 2)
		{
			ctrl = current;
			wcout << "CTRL: " << ctrl->path << endl;
		}
		current = current->next;
	}

	if (imu && lhd)
	{
		handle_imu = hid_open_path(imu->path);
		handle_lhd = hid_open_path(lhd->path);
		handle_ctrl = hid_open_path(ctrl->path);

		// get device config
		int ret = 0, size = 0, cnt = 0;
		memset(cfgbuff, 0, sizeof(cfgbuff));
		cfgbuff[0] = 0x10; // prepares the device for config reading
		std::cout << "Trying to set to config read mode..." << endl;
		do
		{
			ret = hid_get_feature_report(handle_imu, cfgbuff, sizeof(cfgbuff));
			std::cout << ".";
			mySleep(1);
		} while (ret < 0);

		std::cout << endl;

		// send 17 and read until end
		memset(cfgbuff, 0, sizeof(cfgbuff));
		cfgbuff[0] = 0x11;
		cfgbuff[1] = 0xaa;

		std::cout << "Trying to read config..." << endl;

		do
		{
			do
			{
				std::cout << ".";
				ret = hid_get_feature_report(handle_imu, cfgbuff, sizeof(cfgbuff));
				mySleep(1);
			} while (ret < 0);

			size = cfgbuff[1];

			if (!size)
				break;

			if (size > (sizeof(cfgbuff) - 2))
			{
				std::cout << "Too much data (%d) on packet from config" << endl;
			}

			if (cnt + size >= sizeof(compressed_data))
			{
				std::cout << "Configuration length too long" << endl;
			}

			// Some (Tracker at least?) devices send a uint64_t before data; not sure what it means but skip it for now.
			if (cnt == 0 && size >= 2 && cfgbuff[2] != 0x78)
			{
				continue;
			}

			memcpy(compressed_data + cnt, cfgbuff + 2, size);
			cnt += size;
		} while (1);
		std::cout << endl;
		std::cout << "Total config length: " << cnt << endl;
		// decompress using zlib, puff.c doesnt work
		z_stream zs;

		zs.zalloc = Z_NULL;
		zs.zfree = Z_NULL;
		zs.opaque = Z_NULL;
		zs.avail_in = 0;
		zs.next_in = Z_NULL;

		//memset(&zs, 0, sizeof(zs));
		int init = inflateInit(&zs);
		if (init != Z_OK)
		{
			std::cout << "zlib init error: " << init << endl;
		}

		zs.avail_in = cnt;
		zs.next_in = (z_const Bytef *)compressed_data;
		zs.avail_out = sizeof(uncompressed_data) - 1;
		zs.next_out = uncompressed_data;

		int error = inflate(&zs, Z_FINISH);
		if (error != Z_STREAM_END && zs.msg != NULL)
		{
			std::cout << "zlib error: " << zs.msg << endl;
		}
		int len = zs.total_out;
		inflateEnd(&zs);
		uncompressed_data[zs.total_out] = '\0';

		char *config_string = (char *)uncompressed_data;

		try
		{
			config = nlohmann::json::parse(config_string);
		}
		catch (nlohmann::detail::parse_error e)
		{
			std::cout << "Error parsing config data.";
			return -1;
		}

		// these are used to correct MPU-6500 values
		double a, b, c;
		a = 2. / 8192.0;
		b = 1. / 1000.0;
		c = 2000.0 / (1 << 15) * 3.14159265358979323846 / 180.0;

		// load bias and scale from tracker

		double calibration[12];

		calibration[0] = config.at("imu").at("acc_bias")[0];
		calibration[1] = config.at("imu").at("acc_bias")[1];
		calibration[2] = config.at("imu").at("acc_bias")[2];

		calibration[3] = config.at("imu").at("acc_scale")[0];
		calibration[4] = config.at("imu").at("acc_scale")[1];
		calibration[5] = config.at("imu").at("acc_scale")[2];

		calibration[6] = config.at("imu").at("gyro_bias")[0];
		calibration[7] = config.at("imu").at("gyro_bias")[1];
		calibration[8] = config.at("imu").at("gyro_bias")[2];

		calibration[9] = config.at("imu").at("gyro_scale")[0];
		calibration[10] = config.at("imu").at("gyro_scale")[1];
		calibration[11] = config.at("imu").at("gyro_scale")[2];

		accBias[0] = calibration[0] * b;
		accBias[1] = calibration[1] * b;
		accBias[2] = calibration[2] * b;

		accScale[0] = calibration[3] * a;
		accScale[1] = calibration[4] * a;
		accScale[2] = calibration[5] * a;

		gyroBias[0] = calibration[6];
		gyroBias[1] = calibration[7];
		gyroBias[2] = calibration[8];

		gyroScale[0] = calibration[9] * c;
		gyroScale[1] = calibration[10] * c;
		gyroScale[2] = calibration[11] * c;

		// init_devices(config);
		Poser::UpdateTrackerConfig(config, true);

		std::cout << config.dump() << endl;

		std::cout << "Loaded tracker config." << endl;

		std::cout << "Sending magic to power on tracker and configure raw mode." << endl;

		ret = hid_send_feature_report(handle_imu, magic_turn_on, sizeof(magic_turn_on));
		ret = hid_send_feature_report(handle_imu, magic_enable_more, sizeof(magic_enable_more));
		ret = hid_send_feature_report(handle_imu, magic_enable_raw, sizeof(magic_enable_raw));
		ret = hid_send_feature_report(handle_imu, magic_enable_more, sizeof(magic_enable_more));

		// non blocking
		hid_set_nonblocking(handle_imu, 1);
		hid_set_nonblocking(handle_lhd, 1);

		// read imu data
		while (true)
		{
			res = hid_read(handle_imu, buf, 64);
			res2 = hid_read(handle_lhd, buf2, BUF_SIZE);
			if (res > 0)
			{
				unpack_imu(buf);
				std::stringstream ss;
				ss << std::hex;
				for (int i = 0; i < ret; i++)
					ss << std::setw(2) << std::setfill('0') << (int)buf[i];
			}
			if (res2 > 0)
			{
				unpack_lhd(buf2);

				std::stringstream ss;
				ss << std::hex;
				for (int i = 0; i < ret; i++)
					ss << std::setw(2) << std::setfill('0') << (int)buf2[i];
			}
		}
	}

	hid_exit();

	int x;
	cin >> x;
}