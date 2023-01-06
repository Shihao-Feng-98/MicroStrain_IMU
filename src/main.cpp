#include <iostream>
#include <vector>
#include <numeric>
using namespace std;
#include <string.h> // menset
#include <sys/mman.h> // mlockall(MCL_CURRENT|MCL_FUTURE)
#include <memory>

#include "C_timer.h"
#include "periodic_rt_task.h"
#include "microstrain_imu.h"

unique_ptr<GX3_AHRS> gx3_ahrs;

void* main_loop(void* argc)
{
    const double dt = 0.002;
    double time_since_run = 0.;
    int valid_iteration = 0;
    vector<double> t, iter;
    CTimer timer_step;

    gx3_ahrs->parse_data(); 
    for (int i = 0; i < 1; i++)
    {
        while (time_since_run < 1.) // check 1s
        {
            timer_step.reset();
            // wait for data packets in buffer

            if (gx3_ahrs->parse_data()) {valid_iteration++;}

            time_since_run += dt;
            // wait the rest of the time
            while (timer_step.end() < dt*1000*1000); 
        }
        iter.push_back(valid_iteration);
        time_since_run = 0.;
        valid_iteration = 0;
    }
    cout << "valid_iteration: ";
    for (int i = 0; i < iter.size(); i++)
    {
        cout << iter[i] << " ";
    }
    cout << endl;

    return nullptr;
}

// void* main_loop(void* argc)
// {
//     const double dt = 0.002;
//     double time_since_run = 0.;
//     int valid_iteration = 0;
//     vector<double> t;
//     CTimer timer_step;

//     //　一开始缓存区数据较多，第一次读取耗时几百us
//     // 后续每次读取仅几us，一般不超过30us
//     gx3_ahrs->parse_data(); 

//     while (time_since_run < 1.) // check 1s
//     {
//         timer_step.reset();
//         // wait for data packets in buffer

//         if (gx3_ahrs->parse_data()) {valid_iteration++;}
//         t.push_back(timer_step.end());

//         time_since_run += dt;
//         // wait the rest of the time
//         while (timer_step.end() < dt*1000*1000); 
//     }
//     cout << "valid_iteration: " << valid_iteration << endl;
    
//     cout << *max_element(t.begin(), t.end()) << " us\n";
//     cout << accumulate(t.begin(), t.end(), 0.) / t.size() << " us\n";

//     return nullptr;
// }

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

    gx3_ahrs = make_unique<GX3_AHRS>("/dev/ttyACM0", 500);

    // 主控制线程
    PeriodicRtTask *main_task = new PeriodicRtTask("[Main Control Thread]", 95, main_loop, 5);
    sleep(1); 
    // 析构函数会join线程，等待子线程结束
    delete main_task;

    return 0;
}

