/**
 * @file cvIMU.cc
 * @author yuntian li (yuntianlee91@hotmail.com)
 * @brief const velocity circular trajectory around the Moon in G frame 
 * @version 0.1
 * @date 2020-08-11
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "commonHeaders.h"
#include "utilities/io_function.hpp"
#include "simulator/sensors/imu_g.h"

using namespace std;


void cvModel(){
    ImuMotionData initPose;
    initPose.tnb_ = Vec3d(0., 0., 15000);
    initPose.vel_ = Vec3d(1672.9, 0., 0.);
    initPose.qnb_ = Eigen::Quaterniond(1., 0., 0., 0.);
    initPose.time_stamp_ = 0.;

    double a_max = 2.1825;
    double theta = 0. / 180. * M_PI;

    double time = 500;
    int N = 200 * time;

    vector<Vec3d> a_g_all, w_g_all; // all acc and gyr in G
    for (int i = 0; i < N; i++){
        a_g_all.emplace_back(Vec3d(-a_max * cos(theta), 0., -a_max * sin(theta)));
        w_g_all.emplace_back(Vec3d::Zero());
    }

    vector<ImuMotionData> traj_data;

    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    IMU_G mtiIMU(mtiParam);

    mtiIMU.setIntType(0);
    traj_data = mtiIMU.trajGenerator(initPose, a_g_all, w_g_all);

    writeImuMotionData("../data/cvGeo.csv", traj_data);
    writePosNED("../data/posNED.csv", traj_data);
}

void landModel(){
    ImuMotionData initPose;
    initPose.tnb_ = Vec3d(0., 0., 15000);
    initPose.vel_ = Vec3d(1672.9, 0., 0.);
    initPose.time_stamp_ = 0.;

    double theta = -5. / 180. * M_PI;
    Eigen::AngleAxisd r_vec(theta, Vec3d(0., 1., 0.));
    initPose.qnb_ = Eigen::Quaterniond(r_vec);
    
    double a_max = 2.1825;

    double time = 300;
    int N = 200 * time;

    vector<Vec3d> a_b_all, w_g_all; // all acc in B and all gyr in G
    for (int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d(-a_max, 0.,0.));
        w_g_all.emplace_back(Vec3d::Zero());
    }

    vector<ImuMotionData> traj_data;

    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    IMU_G mtiIMU(mtiParam);

    mtiIMU.setIntType(0);
    traj_data = mtiIMU.trajGenerator(initPose, a_b_all, w_g_all);

    writeImuMotionData("../data/caGeo.csv", traj_data);
    writePosNED("../data/posNED.csv", traj_data);
}

int main(int argc, char** argv){
   
    landModel(); 
    
    return 0;
}