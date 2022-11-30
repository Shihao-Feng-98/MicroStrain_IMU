#include <microstrain_imu.h>

GX3_AHRS::GX3_AHRS(string port, unsigned int sample_rate)
{
    _USB_port = port;
    _sample_rate = sample_rate;
    _time_out = 1000 / sample_rate;  // node_ptr->getDataPackets()
    // Create a SerialConnection with the COM port
    _connection = mscl::Connection::Serial(_USB_port, 921600);
    _node_ptr = new mscl::InertialNode(_connection);

    cout << "Node Information: " << endl;
    cout << "Model Name: " << _node_ptr->modelName() << endl;
    cout << "Model Number: " << _node_ptr->modelNumber() << endl;
    cout << "Serial: " << _node_ptr->serialNumber() << endl;
    cout << "Firmware: " << _node_ptr->firmwareVersion().str() << endl << endl;

    _set_current_config();
    _node_ptr->enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);
}

GX3_AHRS::~GX3_AHRS()
{
    // Set the node into idle mode 
    _node_ptr->setToIdle();
    delete _node_ptr;
}

void GX3_AHRS::_set_current_config()
{
    if(_node_ptr->features().supportsCategory(mscl::MipTypes::CLASS_AHRS_IMU))
    {
        // a_imu0,imu express in imu frame
        _ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 
                                                mscl::SampleRate::Hertz(_sample_rate))); // 0x8004
        // w_imu0,imu express in imu frame
        _ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC, 
                                                mscl::SampleRate::Hertz(_sample_rate))); // 0x8005
        // R_imu0,imu
        _ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_MATRIX, 
                                                mscl::SampleRate::Hertz(_sample_rate))); // 0x8009

        // ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION, 
        //                                         mscl::SampleRate::Hertz(sample_rate))); // 0x800a
        // ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, 
        //                                         mscl::SampleRate::Hertz(sample_rate))); // 0x800c

        // Apply to the node
        _node_ptr->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, _ahrsImuChs);
    } 
}

bool GX3_AHRS::parse_data()
{
    if (_node_ptr->totalPackets())
    {
        // Get all packets in buffer
        _packets = _node_ptr->getDataPackets(_time_out);
        // Get the current packet 
        _packet = _packets.back();
        // Get the data in the packet
        _data = _packet.data();
        for (int i = 0; i < (int)_data.size(); i++)
        {
            if (!_data[i].valid()) {return false;}
        }
        acc << _data[0].as_float(), _data[1].as_float(), _data[2].as_float();
        omega << _data[3].as_float(), _data[4].as_float(), _data[5].as_float();
        R << _data[6].as_Matrix().as_floatAt(0,0), _data[6].as_Matrix().as_floatAt(0,1), _data[6].as_Matrix().as_floatAt(0,2),
            _data[6].as_Matrix().as_floatAt(1,0), _data[6].as_Matrix().as_floatAt(1,1), _data[6].as_Matrix().as_floatAt(1,2),
            _data[6].as_Matrix().as_floatAt(2,0), _data[6].as_Matrix().as_floatAt(2,1), _data[6].as_Matrix().as_floatAt(2,2);
        // quaternion << data[7].as_Vector().as_floatAt(0), data[7].as_Vector().as_floatAt(1), 
        //                         data[7].as_Vector().as_floatAt(2), data[7].as_Vector().as_floatAt(3);
        // RPY << data[8].as_float(), data[9].as_float(), data[10].as_float();
        return true;
    }
    return false;
}

