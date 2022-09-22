#include <microstrain_imu.h>

GX3_AHRS::GX3_AHRS(string port, unsigned int sample_rate):USB_port(port)
{
    this->imu_data = IMU_DATA();
    this->sample_rate = sample_rate;
    this->time_out = 1000 / sample_rate;
    // create a SerialConnection with the COM port
    this->connection = mscl::Connection::Serial(this->USB_port, 921600);

    this->node_ptr = new mscl::InertialNode(connection);

    cout << "Node Information: " << endl;
    cout << "Model Name: " << node_ptr->modelName() << endl;
    cout << "Model Number: " << node_ptr->modelNumber() << endl;
    cout << "Serial: " << node_ptr->serialNumber() << endl;
    cout << "Firmware: " << node_ptr->firmwareVersion().str() << endl << endl;

    this->set_current_config();
    this->start_sampling();
}

GX3_AHRS::~GX3_AHRS()
{
    // set the node into idle mode 
    node_ptr->setToIdle();
    delete node_ptr;
}

void GX3_AHRS::set_current_config()
{
    if(node_ptr->features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 
                                                mscl::SampleRate::Hertz(sample_rate))); // 0x8004
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, 
                                                mscl::SampleRate::Hertz(sample_rate))); // 0x8005
        // ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_MATRIX, 
        //                                         mscl::SampleRate::Hertz(sample_rate))); // 0x8009
        // ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION, 
        //                                         mscl::SampleRate::Hertz(sample_rate))); // 0x800a
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, 
                                                mscl::SampleRate::Hertz(sample_rate))); // 0x800c
        //apply to the node
        node_ptr->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);
    } 
}

void GX3_AHRS::start_sampling()
{   
    node_ptr->enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);
}

bool GX3_AHRS::parse_data()
{
    if (node_ptr->totalPackets())
    {
        // get all packets in buffer
        mscl::MipDataPackets packets = node_ptr->getDataPackets(time_out);
        // get the current packet 应该是最新的？
        mscl::MipDataPacket packet = packets.back();
        // get the data in the packet
        mscl::MipDataPoints data = packet.data();
        for (int i = 0; i < data.size(); i++)
        {
            if (!data[i].valid()) {return false;}
        }
        imu_data.acc << data[0].as_float(), data[1].as_float(), data[2].as_float();
        imu_data.ang_vel << data[3].as_float(), data[4].as_float(), data[5].as_float();
        imu_data.orientation << data[6].as_Matrix().as_floatAt(0,0), data[6].as_Matrix().as_floatAt(0,1), data[6].as_Matrix().as_floatAt(0,2),
                                data[6].as_Matrix().as_floatAt(1,0), data[6].as_Matrix().as_floatAt(1,1), data[6].as_Matrix().as_floatAt(1,2),
                                data[6].as_Matrix().as_floatAt(2,0), data[6].as_Matrix().as_floatAt(2,1), data[6].as_Matrix().as_floatAt(2,2);
        imu_data.quaternion << data[7].as_Vector().as_floatAt(0), data[7].as_Vector().as_floatAt(1), 
                                data[7].as_Vector().as_floatAt(2), data[7].as_Vector().as_floatAt(3);
        imu_data.RPY << data[8].as_float(), data[9].as_float(), data[10].as_float();
        return true;
    }
    else {return false;}
}

void GX3_AHRS::test()
{
    double time_since_run = 0.;
    int iteration = 0;
    CTimer timer_step;

    // node_ptr->getDataPackets();// 清空
    while(time_since_run < 1.)
    {
        timer_step.reset();

        if (node_ptr->totalPackets())
        {
            cout << node_ptr->totalPackets() << endl;
            node_ptr->getDataPackets();
            iteration++;
        }
        else 
        {
            cout << node_ptr->totalPackets() << endl;
        }

        time_since_run += 0.003333; // s
        while (timer_step.end() < 3333); // us
    }

    cout << "1s iteration: " << iteration << endl;
}
