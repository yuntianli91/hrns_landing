#include "commonHeaders.h"
#include "utilities/io_function.hpp"
#include "simulator/sensors/imu_mci.h"
#include "simulator/sensors/imu_mcmf.h"
#include "simulator/sensors/imu_g.h"

using namespace MyFusion;

int main(int argc, char** argv){
    // read imu parmaeters
    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    // create two imu instance in MCI and MCMF
    IMU_MCMF mtiMCMF(mtiParam);

    // read designed imudata
    vector<ImuMotionData> traj_data;
    readImuMotionData("../data/standardTraj/caGeo.csv", traj_data);

    // set initialization
    traj_data[0].tnb_ = Eigen::Vector3d(20000. + R_m, 0., 0.);
    traj_data[0].vel_ = Eigen::Vector3d(0., 0., 1690.);

    Eigen::AngleAxisd r_vec_1(-0.5 * M_PI, Eigen::Vector3d(0., 0., 1.)); // z(-pi/2)
    Eigen::AngleAxisd r_vec_2(-0.5 * M_PI, Eigen::Vector3d(0., 1., 0.)); // y(-pi/2)
    Eigen::AngleAxisd r_vec_3(-8.5 / 180 * M_PI, Eigen::Vector3d(0., 0., 1.)); // z(-8.5)

    traj_data[0].qnb_ = Eigen::Quaterniond(r_vec_1 * r_vec_2 * r_vec_3); // Z-Y-X (2,1,0)
    
    // integration
    vector<ImuMotionData> imu_data_mcmf;

    size_t N = traj_data.size();
    for(size_t i = 0; i < traj_data.size(); i++){
        int per = (i + 1) * 100 / N;
        printPer("IMU MCMF generating", per);
        ImuMotionData tmp_data = traj_data[i];
        mtiMCMF.oneStepPropagate(tmp_data);
        imu_data_mcmf.emplace_back(tmp_data);
    }
    cout << endl;

    cout << "Saving data...\n";
    writeImuMotionData("../data/standardTraj/imuMCMF.csv", imu_data_mcmf);
    writePos("../data/standardTraj/posMCMF.csv", imu_data_mcmf);

    return 0;
}