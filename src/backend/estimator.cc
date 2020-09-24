#include "backend/estimator.h"

namespace MyFusion{

Estimator::Estimator(string configFile, int updateType){
    updateType_ = updateType;
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
    vector<AltData> altData;
    
    readImuMotionData("../data/stdTraj/caGeo.csv", trajData_);
    readImuMotionData(IMU_FILE, imuData);
    readVirnsData(VIRNS_FILE, virnsData);
    readCmnsData(CMNS_FILE, cmnsData);
    readAltData(ALT_FILE, altData);

    
    // extract U and Z
    extractU(allU_, imuData);
    extractZ(allZ_, virnsData, cmnsData, altData);
    lastZA_ = FrameConvert::geo2mcmf(imuData[0].tnb_);
    // allZ_.pop(); // pop first measurement as it is zero
    // ---------- initiate other parameters ---------- //
    dataSize_ = allU_.size();

    Mu0_ = VecXd::Zero(6);
    Mu0_(0) = imuData[0].tnb_(0)  + INIT_ERR_P / R_m;
    Mu0_(1) = imuData[0].tnb_(1)  + INIT_ERR_P;
    Mu0_(2) = imuData[0].tnb_(2)  + INIT_ERR_P / R_m;
    Mu0_.segment(3, 3) = imuData[0].vel_  + INIT_ERR_V * Vec3d::Ones();
    allMu_.emplace_back(make_pair(0., Mu0_));

    Sigma0_ = INIT_SQRT_P * INIT_SQRT_P;
    allSigma_.emplace_back(make_pair(0., Sigma0_));

    Q0_ = INIT_SQRT_Q * INIT_SQRT_Q;
    R0_ = INIT_SQRT_R * INIT_SQRT_R;

    sigmaType_ = SampleType(SIGMA_TYPE);
   
    // output for debug
    cout << "[2] Mu: " << Mu0_.transpose()
        << "\n[2] P0: \n" << Sigma0_.diagonal().transpose()
        << "\n[2] Q0: \n" << Q0_.diagonal().transpose()
        << "\n[2] R0: \n" << R0_.diagonal().transpose() << endl;
    // set initiation flag
    dataInitiated_ = true;
}

void Estimator::extractU(queue<VecXd> &allU, const vector<ImuMotionData> & imuData){
    if(!allU.empty())
        clearQueue(allU);
    
    for(auto it : imuData){
        VecXd tmp = VecXd::Zero(7); // [t, ax, ay, az, gx, gy, gz]

        tmp(0) = it.time_stamp_;
        tmp.segment(1, 3) = it.acc_; // accB
        tmp.segment(4, 3) = it.gyr_; // gyrB

        allU.push(tmp);
    }    
}

void Estimator::extractZ(queue<VecXd> &allZ, const vector<VirnsData> &virnsData, const vector<CmnsData> &cmnsData, const vector<AltData> &altData){
    if(!allZ.empty())
        clearQueue(allZ);

    
    if(updateType_ == 0){
        for(size_t i = 0; i < cmnsData.size(); i++){
            VecXd tmp = VecXd::Zero(4); // [L, l, h]
            tmp(0) = cmnsData[i].timeStamp_;
            tmp(1) = cmnsData[i].pos_.x();
            tmp(2) = cmnsData[i].pos_.y();
            tmp(3) = altData[i].range_;

            allZ.push(tmp);
        }
    }
    else{
        int cnt = 0;
        double t2 = cmnsData[cnt].timeStamp_;
        for(auto it : virnsData){
            VecXd tmp;
            double t1 = it.timeStamp_;
            if(abs(t1 - t2) < 1e-5){
                tmp = VecXd::Zero(7); // [t, dx, dy, dz, L, l, h]
                tmp(0) = it.timeStamp_;
                tmp.segment(1, 3) = it.dPos_;
                tmp.segment(4, 2) = cmnsData[cnt].pos_; 
                tmp(6) = altData[cnt].range_;
                cnt++;

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
}

void Estimator::processBackend(double time){
    if(!dataInitiated_){
        cerr << "Please call member function initEstimator(configFile) first !\n";
        return;
    }
    if(time == 0)
        time = trajData_[dataSize_ - 2].time_stamp_;

    // initiate filter
    if(sigmaType_ == SP_UKF){
        filterPtr_ = new PdSCSPKF(Mu0_, Sigma0_, Q0_, R0_, UKF_A, UKF_B, UKF_K); // create SCSPKF pointer
    }
    else{
        filterPtr_ = new PdSCSPKF(Mu0_, Sigma0_, Q0_, R0_, sigmaType_); // create SCSPKF pointer
    }
    filterPtr_->setUpdateType(updateType_);
    filterPtr_->setQnb(trajData_[0].qnb_);
    cout << "Sigma Type: " << filterPtr_->getSigmaType() << endl;
    // start estimation
    for (size_t i = 0; i < dataSize_ - 1; i++){
        // ------------ prediction ------------//
        VecXd tmp = allU_.front();
        // onestep prediction
        filterPtr_->oneStepPrediction(tmp);
        // pop data
        allU_.pop();
        // ----------- update -------------//
        double timeStamp = allU_.front()(0);
        if(abs(allZ_.front()(0) - timeStamp) < 1e-5){
            int tmpSize = allZ_.front().size() - 1;

            if(updateType_ == 1){
                // A + A
                tmp = VecXd::Zero(tmpSize);
                lastZA_ += allZ_.front().segment(1, sizeMr_);
                if(tmpSize == sizeMr_){
                    tmp = lastZA_;
                }
                else{
                    tmp.segment(0, sizeMr_) = lastZA_;
                    tmp.segment(sizeMr_, sizeMa_) = allZ_.front().tail(sizeMa_);
                }
            }
            else{
                // A + R
                tmp = allZ_.front().segment(1, tmpSize);
            }

            filterPtr_->oneStepUpdate(tmp);
            allZ_.pop();

            if(tmpSize != sizeMr_)
                lastZA_ = FrameConvert::geo2mcmf(filterPtr_->getMu().head(3));
        }
        // ---------- save results ----------//
        allMu_.emplace_back(make_pair(timeStamp, filterPtr_->getMu()));        
        allSigma_.emplace_back(make_pair(timeStamp, filterPtr_->getSigma()));        
        // print percentage
        int per = timeStamp * 100 / time;
        printPer("Backend Estimation", per);
        
        // stop
        if(timeStamp >= time)
            break;
        if(allU_.empty() || allZ_.empty())
            break;
    }

    writeResults("../output/" + outFile_ + ".csv", allMu_, allSigma_);
    // writeResults("../output/results.csv", allMu_, allSigma_, allQnb_);
}

// ======================================================================================== //

template <typename T>
void Estimator::clearQueue(queue<T> &Q){
    queue<T> emptyQ;
    swap(emptyQ, Q);
}

void Estimator::writeResults(string fileName, const vector<pair<double, VecXd>> allMu, 
        const vector<pair<double,MatXd>> allSigma,
        const vector<pair<double, Qd>> allQnb)
{
    FILE *fp;
    struct stat buffer;
    if(stat(fileName.c_str(), &buffer) == 0)
        system(("rm " + fileName).c_str());    
    fp = fopen(fileName.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], lat[rad], alt[m], lon[rad], Vn[m/s], Vu[m/s], Ve[m/s], "); 
    fprintf(fp, "CovX[], CovY[], CovZ[], CovVx[], CovVy[], CovVz[], q_w, q_x, q_y, q_z\n"); 
    for(size_t i = 0; i < allMu.size(); i++){
        fprintf(fp, "%lf,%le,%lf,%le,%lf,%lf,%lf,%le,%lf,%le,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                allMu[i].first, 
                allMu[i].second(0), allMu[i].second(1), allMu[i].second(2),
                allMu[i].second(3), allMu[i].second(4), allMu[i].second(5), 
                allSigma[i].second(0), allSigma[i].second(1), allSigma[i].second(2),
                allSigma[i].second(3), allSigma[i].second(4), allSigma[i].second(5),
                allQnb[i].second.w(), allQnb[i].second.x(), allQnb[i].second.y(), allQnb[i].second.z());
    }
}

void Estimator::writeResults(string fileName, const vector<pair<double, VecXd>> allMu, const vector<pair<double,MatXd>> allSigma)
{
    FILE *fp;
    struct stat buffer;
    if(stat(fileName.c_str(), &buffer) == 0)
        system(("rm " + fileName).c_str());    
    fp = fopen(fileName.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], lat[rad], alt[m], lon[rad], Vn[m/s], Vu[m/s], Ve[m/s], "); 
    fprintf(fp, "CovX[], CovY[], CovZ[], CovVx[], CovVy[], CovVz[]\n"); 
    for(size_t i = 0; i < allMu.size(); i++){
        fprintf(fp, "%lf,%le,%lf,%le,%lf,%lf,%lf,%le,%le,%le,%le,%le,%le\n",
                allMu[i].first, 
                allMu[i].second(0), allMu[i].second(1), allMu[i].second(2),
                allMu[i].second(3), allMu[i].second(4), allMu[i].second(5), 
                sqrt(allSigma[i].second(0, 0)), sqrt(allSigma[i].second(1, 1)), sqrt(allSigma[i].second(2, 2)),
                sqrt(allSigma[i].second(3, 3)), sqrt(allSigma[i].second(4, 4)), sqrt(allSigma[i].second(5, 5)));
    }
}

void Estimator::showResults(){
    int N = allMu_.size();
    vector<double> time(N);
    vector<double> err_lat(N), err_alt(N), err_lon(N);
    vector<double> err_px(N), err_pz(N);
    vector<double> err_vx(N), err_vy(N), err_vz(N);
    vector<double> PLat(N), PAlt(N), PLon(N);
    vector<double> Px(N), Pz(N);
    vector<double> Pvx(N), Pvy(N), Pvz(N);
    for(size_t i = 0; i < N; i++){
        time.at(i) = allMu_[i].first;
        double curLat = allMu_[i].second(0); 
        double curAlt = allMu_[i].second(1); 
        double curLon = allMu_[i].second(2); 

        VecXd errP = allMu_[i].second.segment(0, 3) - trajData_[i].tnb_;
        VecXd errV = allMu_[i].second.segment(3, 3) - trajData_[i].vel_;
        err_lat.at(i) = errP.x(); err_alt.at(i) = errP.y(); err_lon.at(i) = errP.z();
        err_px.at(i) = errP.x() * (R_m + curAlt);
        err_pz.at(i) = errP.z() * (R_m + curAlt) * cos(curLat);
        err_vx.at(i) = errV.x(); err_vy.at(i) = errV.y(); err_vz.at(i) = errV.z();
    
        PLat.at(i) = 3. * sqrt(allSigma_[i].second(0, 0)); 
        PAlt.at(i) = 3. * sqrt(allSigma_[i].second(1, 1)); 
        PLon.at(i) = 3. * sqrt(allSigma_[i].second(2, 2));
   
        Px.at(i) = PLat[i] * (R_m + curAlt);
        Pz.at(i) = PLon[i] * (R_m + curAlt) * cos(curLat);

        Pvx.at(i) = 3. * sqrt(allSigma_[i].second(3, 3)); 
        Pvy.at(i) = 3. * sqrt(allSigma_[i].second(4, 4)); 
        Pvz.at(i) = 3. * sqrt(allSigma_[i].second(5, 5));
    }
    // position
    // plt::figure();
    // plt::subplot(3,1,1);
    // plt::named_plot("SCHCKF", time, err_lat, "-b");
    // plt::named_plot("3sigma", time, PLat, "--k");
    
    // plt::subplot(3,1,2);
    // plt::named_plot("SCHCKF", time, err_alt, "-b");
    // plt::named_plot("3sigma", time, PAlt, "--k");
    
    // plt::subplot(3,1,3);
    // plt::named_plot("SCHCKF", time, err_lon, "-b");
    // plt::named_plot("3sigma", time, PLon, "--k");
    // velocity
    plt::figure();
    plt::subplot(3,1,1);
    plt::named_plot("SCHCKF", time, err_vx, "-b");
    plt::named_plot("3sigma", time, Pvx, "--k");
    
    plt::subplot(3,1,2);
    plt::named_plot("SCHCKF", time, err_vy, "-b");
    plt::named_plot("3sigma", time, Pvy, "--k");
    
    plt::subplot(3,1,3);
    plt::named_plot("SCHCKF", time, err_vz, "-b");
    plt::named_plot("3sigma", time, Pvz, "--k");

    // 
    plt::figure();
    plt::subplot(3,1,1);
    plt::named_plot("SCHCKF", time, err_px, "-b");
    plt::named_plot("3sigma", time, Px, "--k");
    
    plt::subplot(3,1,2);
    plt::named_plot("SCHCKF", time, err_alt, "-b");
    plt::named_plot("3sigma", time, PAlt, "--k");
    
    plt::subplot(3,1,3);
    plt::named_plot("SCHCKF", time, err_pz, "-b");
    plt::named_plot("3sigma", time, Pz, "--k");
    
    
    plt::show();
}
}