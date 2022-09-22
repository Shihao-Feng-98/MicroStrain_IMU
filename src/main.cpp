#include <iostream>
using namespace std;
#include <string.h> // menset
#include <pthread.h> // -lpthread
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)

#include <utils/C_timer.h>
#include "mscl/mscl.h" // depend on pthread

int main()
{
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        cout << "mlockall failed: %m\n"; 
        return -2;
    }

    // ======== main thread ========
    cout << "main thread start\n";
    struct sched_param param;
    memset(&param, 0, sizeof(param)); 
    param.sched_priority = 49; 
    int ret = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
    if (ret) 
    {
        cout << "main thread setschedpolicy and setinheritsched failed\n";
        return -1;
    }

    // ======= IMU ==============
    const string com_port = "/dev/ttyACM0"; 
    const uint32_t baud_rate = 115200;
    mscl::Connection connection = mscl::Connection::Serial(com_port, baud_rate);
    mscl::InertialNode node(connection);

    // print imu information
    cout << "Node Information: " << endl;
    cout << "Model Name: " << node.modelName() << endl;
    cout << "Model Number: " << node.modelNumber() << endl;
    cout << "Serial: " << node.serialNumber() << endl;
    cout << "Firmware: " << node.firmwareVersion().str() << endl << endl;

    // set congfig
    uint32_t samples_per_second = 500;
    if(node.features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        mscl::MipChannels ahrsImuChs;
        // set channels
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 
                                                mscl::SampleRate::Hertz(samples_per_second))); // 0x8004
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, 
                                                mscl::SampleRate::Hertz(samples_per_second))); // 0x8005
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, 
                                                mscl::SampleRate::Hertz(samples_per_second))); // 0x800c
        //apply to the node
        node.setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);
    } 

    // start sampling
    node.enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);

    // loop 
    double time_since_run = 0.;
    double dt = 0.002;
    uint32_t valid_interation = 0;
    CTimer timer_step; // us

    for (int i = 0; i < 10; i++)
    {
        while (time_since_run <= 1.) // check 1s
        {
            timer_step.reset();
            // wait for data packets in buffer
            // timeout 50us 
            while(timer_step.end() < 50) 
            {
                if (node.getDataPackets().size())
                {
                    node.getDataPackets(); // get all packets
                    valid_interation++;
                    break;
                }
            }

            time_since_run += dt;
            // wait the rest of the time
            while (timer_step.end() < dt*1000*1000); 
        }
        cout << "valid interation: " << valid_interation << endl;
        time_since_run = 0.;
        valid_interation = 0;
    }

    return 0;
}

