#include "io-handler.h"

template <typename T>
std::string to_stringp2(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << setw(10) << std::fixed << a_value;
    return out.str();
}

int PrintPoseLibsurviveFormatLM(Timestamp ts, std::string object_name, LinmathPose *pose)
{

    cout << "distance from LH: " << sqrt(pow(pose->Pos[0], 2) + pow(pose->Pos[1], 2) + pow(pose->Pos[2], 2)) << endl;
    std::cout << ts / 48000000 << " "
              << object_name << " "
              << "POSE "
              << setw(8) << pose->Pos[0] << " " << setw(8) << pose->Pos[1] << " " << setw(8) << pose->Pos[2] << " " << setw(8) << pose->Rot[0] << " " << setw(8) << pose->Rot[1] << " " << setw(8) << pose->Rot[2] << " " << setw(8) << pose->Rot[3] << endl;

    return 0;
}