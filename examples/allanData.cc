#include "simulator/sensors/imu_li.h"
#include "utilities/io_function.hpp"

using namespace std;

int main(int argc, char** argv){
    // generate trajectory
    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    IMU_LI mtiIMU(mtiParam);

    vector<ImuMotionData> imu_allan_data;
    mtiIMU.generateAllanData(7200, imu_allan_data);

    writeAllanData("../data/iMAR_allan.csv", imu_allan_data);
    
    return 0;
}