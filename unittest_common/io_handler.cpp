#include "io_handler.h"

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 1340

using boost::asio::ip::address;
using boost::asio::ip::udp;

template <typename T>
std::string to_stringp(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << setw(10) << std::fixed << a_value;
    return out.str();
}

string getPoseViewerString(std::string objectName, LinmathPose *pose)
{
    return "{\"id\": " + objectName + " , " + "\"pose\": { \"x\": " + to_stringp(pose->Pos[0]) + " , \"y\": " + to_stringp(pose->Pos[1]) + " , \"z\" : " + to_stringp(pose->Pos[2]) + ", \"qw\" : " + to_stringp(pose->Rot[0]) + " , \"qx\" : " + to_stringp(pose->Rot[1]) + " , \"qy\" : " + to_stringp(pose->Rot[2]) + " , \"qz\" : " + to_stringp(pose->Rot[3]) + "} }";
}

void sendPoseUDP(std::string objectName, LinmathPose *pose)
{
    string pose_str = getPoseViewerString(objectName, pose);
    boost::asio::io_service io_service;
    udp::socket socket(io_service);
    udp::endpoint remote_endpoint = udp::endpoint(address::from_string(IPADDRESS), UDP_PORT);
    socket.open(udp::v4());

    boost::system::error_code err;
    auto sent = socket.send_to(boost::asio::buffer(pose_str), remote_endpoint, 0, err);
    socket.close();
    // std::cout << "Sent Payload --- " << sent << "\n";
}

void sendPoseUDPU(std::string objectName, const LinmathPose *pose)
{
    /** Files and calibration pose for ground plane
     * ceres_0 : {{0, 2.7, 0}, {0.366777, 0.195665, 0.841065, -0.346122}};
     * neverland_0 without changing origin: {{0, 0, 0}, {-0.29339666, 0.88276984, 0.10697924, 0.35098611}};
     * neverland_0 origin at lh mode 6: 
     * */

    LinmathPose upw = {{8.72303863, -0.17235806, -8.16103763}, {0.86040305, 0.26883925, -0.40729955, 0.14676215}};

    ApplyPoseToPose(&upw, &upw, pose);

    upw.Pos[1] = upw.Pos[1] + 2.7;
    double tempaxis = upw.Pos[1];
    upw.Pos[1] = -upw.Pos[2];
    upw.Pos[2] = tempaxis;

    std::cout << "sent pose: " << upw.Pos[0] << " " << upw.Pos[1] << " " << upw.Pos[2] << " "
              << upw.Rot[0] << " " << upw.Rot[1] << " " << upw.Rot[2] << " " << upw.Rot[3] << " " << std::endl;
    string pose_str = getPoseViewerString(objectName, &upw);
    boost::asio::io_service io_service;
    udp::socket socket(io_service);
    udp::endpoint remote_endpoint = udp::endpoint(address::from_string(IPADDRESS), UDP_PORT);
    socket.open(udp::v4());

    boost::system::error_code err;
    auto sent = socket.send_to(boost::asio::buffer(pose_str), remote_endpoint, 0, err);
    socket.close();
    // std::cout << "Sent Payload --- " << sent << "\n";
}
