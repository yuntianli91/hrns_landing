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


vector<ImuMotionData> traj_data;
vector<ImuMotionData> imu_data;

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

    // vector<ImuMotionData> traj_data;

    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    IMU_G mtiIMU(mtiParam);

    mtiIMU.setIntType(0);
    traj_data.clear();
    traj_data = mtiIMU.trajGenerator(initPose, a_g_all, w_g_all);

    writeImuMotionData("../data/cvGeo.csv", traj_data);
    writePos("../data/posNED.csv", traj_data);
}

void landModel(){
    ImuMotionData initPose;
    initPose.tnb_ = Vec3d(0., 20000, 0.); // lat, alt, lon
    initPose.vel_ = Vec3d(1690, 0., 0.);
    initPose.time_stamp_ = 0.;

    double theta = -8.5 / 180. * M_PI;
    Eigen::AngleAxisd r_vec(theta, Vec3d(0., 0., 1.));
    initPose.qnb_ = Eigen::Quaterniond(r_vec);
    
    double cur_a = 2.6658;
    // main break
    double time = 405;
    int N = 200 * time;

    vector<Vec3d> a_b_all, w_g_all; // all acc in B and all gyr in G
    for (int i = 0; i < N; i++){
        cur_a += 3.5e-5;
        a_b_all.emplace_back(Vec3d(-cur_a, 0.,0.));
        w_g_all.emplace_back(Vec3d::Zero());
    }
    
    // fast adjustment
    time = 32.6;
    N = 200 * time;

    cur_a = 2.0;
    double w_ad = 2.5 / 180. * M_PI;
    for(int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d( -cur_a, 0., 0.));
        w_g_all.emplace_back(Vec3d(0., 0., -w_ad));
    }

    // vertical approaching
    time = 74.4;
    N = 200 * time;

    for(int i = 0; i < N; i++){
        cur_a += 2.0e-4;
        a_b_all.emplace_back(Vec3d(-cur_a, 0.,0.));
        w_g_all.emplace_back(Vec3d(0., 0., 0.));
    }
    // hazard avoidance 
    time = 15;
    N = 200 * time;

    for(int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d(-1.622, 0.5, -0.6));
        w_g_all.emplace_back(Vec3d(0., 0., 0.));
    }
    
    time = 15;
    N = 200 * time;

    for(int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d(-1.622, 0.5, 0.6));
        w_g_all.emplace_back(Vec3d(0., 0., 0.));
    }
    // finla approaching
    time = 11;
    N = 200 * time;
    for(int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d(0., 0., 0.));
        w_g_all.emplace_back(Vec3d(0., 0., 0.));   
    }
    
    time = 10;
    N = 200 * time;
    for(int i = 0; i < N; i++){
        a_b_all.emplace_back(Vec3d(-3.5, 0., 0.));
        w_g_all.emplace_back(Vec3d(0., 0., 0.));   
    }
    // generate trajectory
    ImuParam mtiParam;
    readImuParam("../config/simulator/mti_config.yaml", mtiParam);
    IMU_G mtiIMU(mtiParam);

    mtiIMU.setIntType(0);
    traj_data.clear();
    traj_data = mtiIMU.trajGenerator(initPose, a_b_all, w_g_all);

    writeImuMotionData("../data/stdTraj/caGeo.csv", traj_data);
    writePos("../data/stdTraj/posNED.csv", traj_data);

    
    // test imu integration    
    imu_data.clear();
    int i = 0;
    for(auto it: traj_data){
        int per = (++i) * 100 / traj_data.size();
        printf("[#][Generating IMU data...][%d%%]\r", per);
        fflush(stdout);

        ImuMotionData tmp_data = it;
        mtiIMU.oneStepPropagate(tmp_data);
        imu_data.emplace_back(tmp_data);
    }
    printf("\n");

    writeImuMotionData("../data/stdTraj/caGeoImu.csv", imu_data);
    writePos("../data/stdTraj/posNEDImu.csv", imu_data);
}

int main(int argc, char** argv){
   
    landModel(); 

    return 0;
}