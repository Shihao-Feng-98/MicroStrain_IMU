#ifndef MIRCOSTRAIN_IMU_H
#define MIRCOSTRAIN_IMU_H

#include <unistd.h>
#include <string>
#include <iostream>
using namespace std;
#include <eigen3/Eigen/Dense> 
using namespace Eigen;
#include <utils/CTimer.h>

#include "mscl/mscl.h" // depend on pthread

struct IMU_DATA
{
    Vector3d acc;
    Vector3d ang_vel;
    Matrix3d orientation;
    Vector4d quaternion;
    Vector3d RPY;
    // 默认构造
    IMU_DATA() 
    {
        acc.setZero();
        ang_vel.setZero();
        orientation.setIdentity();
        quaternion << 1.,0.,0.,0.;
        RPY.setZero();
    }
};

// CV5_AHRS up to 1000
// GX3_AHRS up to 500


class GX3_AHRS
{
public:
    GX3_AHRS(string port, unsigned int sample_rate = 500);
    ~GX3_AHRS();
    bool parse_data();

    void test();

    IMU_DATA imu_data;

private:
    void start_sampling();
    void set_current_config();

    uint32_t sample_rate; 
    uint32_t time_out;
    const string USB_port;
    mscl::Connection connection;
    mscl::InertialNode *node_ptr;
    mscl::MipChannels ahrsImuChs;
};


#endif