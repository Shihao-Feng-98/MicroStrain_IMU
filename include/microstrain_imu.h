#ifndef MIRCOSTRAIN_IMU_H
#define MIRCOSTRAIN_IMU_H

#include <unistd.h>
#include <string>
#include <iostream>
using namespace std;
#include <Eigen/Dense> 
using namespace Eigen;

#include "mscl/mscl.h" 
#include <C_timer.h>

// CV5_AHRS up to 1000
// GX3_AHRS up to 500

class GX3_AHRS
{
public:
    GX3_AHRS(string port, unsigned int sample_rate = 500);
    ~GX3_AHRS();
    bool parse_data();

    // Both express in imu frame
    Vector3d acc;
    Vector3d omega;
    Matrix3d R;

private:
    void _set_current_config();

    uint32_t _sample_rate; 
    uint32_t _time_out;
    string _USB_port;
    mscl::Connection _connection;
    mscl::InertialNode *_node_ptr;
    mscl::MipChannels _ahrsImuChs;

    mscl::MipDataPackets _packets;
    mscl::MipDataPacket _packet;
    mscl::MipDataPoints _data;
};


#endif