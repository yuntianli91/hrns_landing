#include "backend/estimator.h"

namespace MyFusion{

Estimator::Estimator(string configFile){
    initEstimator(configFile);
}
Estimator::~Estimator(){
    if(filterPtr_!=nullptr){
        delete filterPtr_;
        filterPtr_ = nullptr;
    }
}

void Estimator::initEstimator(string configFile){
    // load parameters
    loadBackParam(configFile);
    // ---------- read simulation data ------------- //
    vector<ImuMotionData> imuData;
    // vector<CnsData> cnsData;
    vector<VirnsData> virnsData;
    vector<CmnsData> cmnsData;
    
    readImuMotionData(IMU_FILE, imuData);
    readVirnsData(VIRNS_FILE, virnsData);
    readCmnsData(CMNS_FILE, cmnsData);
    // extract U and Z
    extractU(allU_, imuData);
    extractZ(allZ_, virnsData, cmnsData);
    // ---------- initiate other parameters ---------- //
    dataSize_ = allU_.size();
    
    Mu0_ = VecXd::Zero(6);
    Mu0_.segment(0, 3) = imuData[0].tnb_  + INIT_ERR_P * Vec3d::Ones();
    Mu0_.segment(3, 6) = imuData[0].vel_  + INIT_ERR_V * Vec3d::Ones();

    Sigma0_ = INIT_SQRT_P * INIT_SQRT_P;
    Q0_ = INIT_SQRT_Q * INIT_SQRT_Q;
    R0_ = INIT_SQRT_R * INIT_SQRT_R;

    sigmaType_ = SampleType(SIGMA_TYPE);
    // output for debug
    cout << "[E] Mu: " << Mu0_.transpose()
        << "\nP0: \n" << Sigma0_
        << "\nQ0: \n" << Q0_
        << "\nR0: \n" << R0_;
    // set initiation flag
    dataInitiated_ = true;
}

void Estimator::extractU(queue<VecXd> &allU, const vector<ImuMotionData> & imuData){
    if(allU.size() != 0)
        clearQueue(allU);
    
    for(auto it : imuData){
        VecXd tmp = VecXd::Zero(7); // [t, ax, ay, az, gx, gy, gz]

        tmp(0) = it.time_stamp_;
        tmp.segment(1, 3) = it.acc_;
        tmp.segment(4, 3) = it.gyr_;

        allU.push(tmp);
    }    
}

void Estimator::extractZ(queue<VecXd> &allZ, const vector<VirnsData> &virnsData, const vector<CmnsData> cmnsData){
    if(allZ.size() != 0)
        clearQueue(allZ);

    int cnt = 0;
    double t2 = cmnsData[cnt].timeStamp_;
    for(auto it : virnsData){
        VecXd tmp;
        double t1 = it.timeStamp_;
        if(abs(t1 - t2) < 1e-5){
            tmp = VecXd::Zero(6); // [t, dx, dy, dz, L, l]
            tmp(0) = it.timeStamp_;
            tmp.segment(1, 3) = it.dPos_;
            tmp.segment(4, 2) = cmnsData[cnt++].pos_; 

            t2 = cmnsData[cnt].timeStamp_;
        }
        else{
            tmp = VecXd::Zero(4); // [t, dx, dy, dz]
            tmp(0) = it.timeStamp_;
            tmp.segment(1, 3) = it.dPos_;        
        }

        allZ.push(tmp);
    }
}

void Estimator::processBackend(){
    if(!dataInitiated_){
        cerr << "Please call member function initEstimator(configFile) first !\n";
        return;
    }
    // create SCSPKF pointer
    filterPtr_ = new PdSCSPKF(Mu0_, Sigma0_, Q0_, R0_, sigmaType_);
    // start estimation
    for (size_t i = 0; i < dataSize_; i++){
        filterPtr_->oneStepPrediction(allU_.front());
        // save results
        allMu.emplace_back(filterPtr_->getMu());        
        allSigma.emplace_back(filterPtr_->getSigma());        
        // print percentage
        int per = i / dataSize_;
        printPer("Backend Estimation", per);
        // pop data
        allU_.pop();
    }



}

template <typename T>
void Estimator::clearQueue(queue<T> &Q){
    queue<T> emptyQ;
    swap(emptyQ, Q);
}

}