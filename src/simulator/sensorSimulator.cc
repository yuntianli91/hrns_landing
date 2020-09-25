#include "simulator/sensorSimulator.h"

namespace MyFusion{

SensorSimulator::SensorSimulator(string configFile){
    readSensorParameters(configFile);
}

SensorSimulator::~SensorSimulator(){}

void SensorSimulator::readSensorParameters(string configFile){
    cv::FileStorage fsParams(configFile, cv::FileStorage::READ);

    if(!fsParams.isOpened()){
        cout << "ERROR: failed to open config file. Please reset parameters!\n";
        return;
    }
    else{
        // imu parameters
        sensorParams_.acc_b_ = fsParams["acc_b"];
        sensorParams_.acc_n_ = fsParams["acc_n"];
        sensorParams_.acc_w_ = fsParams["acc_w"];       
        sensorParams_.gyr_b_ = fsParams["gyr_b"];
        sensorParams_.gyr_n_ = fsParams["gyr_n"];
        sensorParams_.gyr_w_ = fsParams["gyr_w"];
        sensorParams_.imu_step_ = fsParams["imu_step"];
        // cns parameters
        sensorParams_.cns_step_ = fsParams["cns_step"];
        sensorParams_.cns_sigma_ = fsParams["cns_n"];
        // virns parameters
        sensorParams_.virns_step_ = fsParams["virns_step"];
        sensorParams_.virns_bias_ = fsParams["virns_b"];
        sensorParams_.virns_sigma_ = fsParams["virns_n"];
        // cmns parameters
        sensorParams_.cmns_step_ = fsParams["cmns_step"];
        sensorParams_.cmns_sigma_ = fsParams["cmns_n"];
        // altimeter parameters
        sensorParams_.alt_step_ = fsParams["alt_step"];
        sensorParams_.alt_sigma_ = fsParams["alt_n"];

        paramInitialized_ = true;

        showParameters(sensorParams_);
    }
}

void SensorSimulator::showParameters(SensorParams params){
    printf("[0] IMU Parameters(%lf):\n", params.imu_step_);
    printf("  ACC_B:%e, ACC_N:%e, ACC_W:%e.\n", params.acc_b_, params.acc_n_, params.acc_w_);
    printf("  GYR_B:%e, GYR_N:%e, GYR_W:%e.\n", params.gyr_b_, params.gyr_n_, params.gyr_w_);

    // printf("[0] CNS Parameters(%lf):\n", params.cns_step_);
    // printf("  Noise:%lf.\n", params.cns_sigma_);

    // printf("[0] VIRNS Parameters(%lf):\n", params.virns_step_);
    // printf("  Bias:%lf, Noise:%lf.\n", params.virns_bias_, params.virns_sigma_);

    // printf("[0] CMNS Parameters(%lf):\n", params.cmns_step_);
    // printf("  Noise:%lf.\n", params.cmns_sigma_);
}

void SensorSimulator::simIMU(const vector<ImuMotionData> &trajData, vector<ImuMotionData> &imuData){
    ImuParam tmpParam;
    tmpParam.time_step_ = sensorParams_.imu_step_;
    tmpParam.acc_b_ = sensorParams_.acc_b_; tmpParam.gyr_b_ = sensorParams_.gyr_b_;
    tmpParam.acc_n_ = sensorParams_.acc_n_; tmpParam.gyr_n_ = sensorParams_.gyr_n_;
    tmpParam.acc_w_ = sensorParams_.acc_w_; tmpParam.gyr_w_ = sensorParams_.gyr_w_;

    IMU_G imuSimulator(tmpParam);

    imuData.clear();
    int totalSize = trajData.size();
    for(size_t i = 0; i < totalSize; i++){
        ImuMotionData tmp = trajData[i];
        imuSimulator.oneStepPropagate(tmp);
        imuData.emplace_back(tmp);
        
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating IMU measurements in GEO", per);
    }
    // change line
    printf("\n");
}

void SensorSimulator::simCNS(const vector<ImuMotionData> &trajData, vector<CnsData> &cnsData){
    CNS cnsSimulator(0., sensorParams_.cns_sigma_);
    
    cnsData.clear();
    double lastTime = 0.0;
    int totalSize = trajData.size();
    for(size_t i = 0; i < totalSize; i++){
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating CNS measurements in MCI", per);
        
        if(abs(trajData[i].time_stamp_ - lastTime - sensorParams_.cns_step_) > 1e-5)
            continue;
       
        CnsData tmp = cnsSimulator.getMeasurements(trajData[i]);
        cnsData.emplace_back(tmp);
        lastTime = trajData[i].time_stamp_;        
    }   
    // change line
    printf("\n");
}

void SensorSimulator::simVIRNSRelative(const vector<ImuMotionData> &trajData, vector<VirnsData> &virnsData){
    VIRNS virnsSimulator(0., sensorParams_.virns_sigma_);
    
    virnsData.clear();
    double lastTime = 0.0;
    int totalSize = trajData.size();
    VirnsData tmp = virnsSimulator.getRelativeMeasurement(trajData[0]);
    for(size_t i = 0; i < totalSize; i++){
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating VIRNS relative measurements in MCMF", per);
 
        if (abs(trajData[i].time_stamp_ - lastTime - sensorParams_.virns_step_) > 1e-5)
            continue;
        
        tmp = virnsSimulator.getRelativeMeasurement(trajData[i]);
        virnsData.emplace_back(tmp);
        lastTime = trajData[i].time_stamp_;
   }
    // change line
    printf("\n");
}

void SensorSimulator::simVIRNS(const vector<ImuMotionData> &trajData, vector<VirnsData> &virnsData){
    printf("[0] VIRNS Parameters(%lf):\n", sensorParams_.virns_step_);
    printf("  Bias:%lf, Noise:%lf.\n", sensorParams_.virns_bias_, sensorParams_.virns_sigma_);
    // generate relative measurements
    simVIRNSRelative(trajData, virnsData);
    // generate absolute measurements
    Vec3d lastPos = Vec3d::Ones() * sensorParams_.virns_bias_ + FrameConvert::geo2mcmf(trajData[0].tnb_);

    int totalSize = virnsData.size();
    for(size_t i = 0; i < totalSize; i++){

        lastPos += virnsData[i].dPos_;
        virnsData[i].pos_ = lastPos;
        
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating VIRNS absolute measurements in MCMF", per);
    }
    // change line
    printf("\n");
}

void SensorSimulator::simCMNS(const vector<ImuMotionData> &trajData, vector<CmnsData> &cmnsData){
    printf("[0] CMNS Parameters(%lf):\n", sensorParams_.cmns_step_);
    printf("  Noise:%lf.\n", sensorParams_.cmns_sigma_);

    CMNS cmnsSimulator(0., sensorParams_.cmns_sigma_);

    cmnsData.clear();
    double lastTime = 0.0;
    int totalSize = trajData.size();
    
    // CmnsData tmp = cmnsSimulator.getMeasurement(trajData[0]);
    // cmnsData.emplace_back(tmp);
    for(size_t i = 0; i < totalSize; i++){
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating CMNS measurements in MCMF", per);
 
        if (abs(trajData[i].time_stamp_ - lastTime - sensorParams_.cmns_step_) > 1e-5)
          continue;

        CmnsData tmp = cmnsSimulator.getMeasurement(trajData[i]);
        cmnsData.emplace_back(tmp);
        lastTime = trajData[i].time_stamp_;
   }
    // change line
    printf("\n");
}

void SensorSimulator::simAltimeter(const vector<ImuMotionData> &trajData, vector<AltData> &altData){
    Altimeter altSimulator(0, sensorParams_.alt_sigma_);

    altData.clear();
    double lastTime = 0.0;
    int totalSize = trajData.size();

    for(size_t i = 0; i < totalSize; i++){
        int per = (i + 1) * 100 / totalSize;
        printPer("Generating Altimeter measurements in GEO", per);
 
        if (abs(trajData[i].time_stamp_ - lastTime -sensorParams_.alt_step_) > 1e-5)
          continue;

        AltData tmp = altSimulator.getMeasurement(trajData[i]);
        altData.emplace_back(tmp);
        lastTime = trajData[i].time_stamp_;
    }
    
    // change line
    printf("\n");
}

} //namespace 



