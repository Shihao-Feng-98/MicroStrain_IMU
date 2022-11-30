#include <iostream>
#include <vector>
#include <numeric>
using namespace std;
#include <string.h> // menset
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)

#include "C_timer.h"
#include "periodic_rt_task.h"
#include "microstrain_imu.h"

void* main_loop(void* argc)
{
    // ======= IMU ==============
    const string com_port = "/dev/ttyACM0"; 
    GX3_AHRS gx3_ahrs(com_port, 300);

    const double dt = 0.00333;
    double time_since_run = 0.;
    int valid_iteration = 0;
    vector<double> t;
    CTimer timer_step;

    for (int i = 0; i < 10; i++)
    {
        while (time_since_run < 1.) // check 1s
        {
            timer_step.reset();
            // wait for data packets in buffer

            if (gx3_ahrs.parse_data()) {valid_iteration++;}
            t.push_back(timer_step.end());

            time_since_run += dt;
            // wait the rest of the time
            while (timer_step.end() < dt*1000*1000); 
        }
        cout << "valid iteration: " << valid_iteration << endl;
        time_since_run = 0.;
        valid_iteration = 0;
    }

    cout << *max_element(t.begin(), t.end()) << " us\n";
    cout << accumulate(t.begin(), t.end(), 0.) / t.size() << " us\n";

    return nullptr;
}


int main(int argc, char **argv)
{
    /*
    mlockall 锁定进程中所有映射到地址空间的页
    MCL_CURRENT 已经映射的进程地址，MCL_FUTURE 将来映射的进程地址
    */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        cout << "mlockall failed: %m\n"; 
        return -2;
    }

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop, 5);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}

