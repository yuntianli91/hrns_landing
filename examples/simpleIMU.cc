#include "commonHeaders.h"
#include "utilities/io_function.hpp"
#include "simulator/sensors/imu_li.h"
#include "simulator/sensors/imu_mcmf.h"

using namespace std;
using namespace myFusion;

void simLI(){
    vector<ImuMotionData> traj_data;
    vector<ImuMotionData> imu_data;

    readImuMotionData("../data/ro_demo/traj_data.csv", traj_data);
    ImuParam mti_param;
    readImuParam("../config/simulator/mti_config.yaml", mti_param);

    IMU_LI mti(mti_param);

    imu_data.clear();
    for (auto it : traj_data){
        ImuMotionData data = it;
        mti.oneStepPropagate(data);
        imu_data.emplace_back(data);
    }    

    writeImuMotionData("../data/ro_demo/imu_data.csv", imu_data);
}

int main(int argc, char** argv){
    simLI();

    return 0;
}