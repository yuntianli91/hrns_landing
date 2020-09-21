#include "backend/sckf/scspkf.h"

namespace MyFusion{

SCSPKF::SCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType):SCKF(Mu, Sigma, Q, R){
    sigmaType_ = sigmaType;
    // compute weight
    computeWeight(weightMu_, weightSigma_, xSize_); // calculate weight of state
    computeWeight(weightMuAug_, weightSigmaAug_, 2 * xSize_); // calculate weight of augState
    // compute cubature points
    genSi(allSi_, xSize_); // cubature points
    genSi(allSiAug_, 2 * xSize_); // aug cubature points
    // print si and weight for debug
    // printWeight(weightMu_, varName(weightMu_), 20);
    // printSi(allSi_, varName(allsi_), 20);
    // printWeight(weightMuAug_, varName(weightMuAug_), 20);
    // printSi(allSiAug_, varName(allSiAug_), 20);
}

void SCSPKF::setUKFParams(double alpha, double beta, double kappa){
    alpha_ = alpha;
    beta_ = beta;
    kappa_ = kappa;
    lambda_ = 3 * alpha_ * alpha_ - xSize_; 

    ukfInit_ = true;
    printf("[UKF] Parameters: alpha (%lf), beta (%lf), kappa (%lf).\n", alpha_, beta_, kappa_);
}

void SCSPKF::genSigmaPoints(vector<VecXd> &sPoints, bool aug){
    if(sPoints.size() != 0)
        sPoints.clear();

    if(aug){
        MatXd sqrtS = augSigma_.llt().matrixL(); // Cholesky decomposition of covariance matrix
        // cout << sqrtS;
    
        for(size_t i = 0; i < allSiAug_.size(); i++){
            VecXd point = augMu_ + sqrtS * allSiAug_[i];
            sPoints.emplace_back(point);
        }
    }
    else{
        MatXd sqrtS = Sigma_.llt().matrixL(); // Cholesky decomposition of covariance matrix
    
        for(size_t i = 0; i < allSi_.size(); i++){
            VecXd point = Mu_ + sqrtS * allSi_[i];
            sPoints.emplace_back(point);
        }
    }
}

void SCSPKF::genSi(vector<VecXd> &allSi, int xSize){
    switch (sigmaType_)
    {
    case SP_UKF:
        genSiUKF(allSi, xSize);   
        break;
    case SP_CKF:
        genSiCKF(allSi, xSize);
        break;
    case SP_HCKF:
        genSiHCKF(allSi, xSize);
        break;
    default:
        cout << "Unknown sample type !\n";
        break;
    }
}

void SCSPKF::genSiUKF(vector<VecXd> &allSi, int xSize){
    gamma_ = sqrt(xSize + lambda_);
    if (allSi.size() != 0)
         allSi.clear();
    // 0
    VecXd si = VecXd::Zero(xSize);
    allSi.emplace_back(si);
    // 1 ~ 2n
    MatXd I = MatXd::Identity(xSize, xSize);
    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(gamma_ * I.col(i));
    }
    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(-gamma_ * I.col(i));
    }   
    // check dims
    if(allSi.size() != 2 * xSize + 1)
        cout << "Error size of Si !\n";
}

void SCSPKF::genSiCKF(vector<VecXd> &allSi, int xSize){
    gamma_ = sqrt(xSize);
    if (allSi.size() != 0)
         allSi.clear();
    // 1 ~ 2n
    MatXd I = MatXd::Identity(xSize, xSize);
    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(gamma_ * I.col(i));
    }
    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(-gamma_ * I.col(i));
    }   
    // check dims
    if(allSi.size() != 2 * xSize)
        cout << "Error size of Si !\n";
}

void SCSPKF::genSiHCKF(vector<VecXd> &allSi, int xSize){
    gamma_ = sqrt(xSize + 2.);

    if(allSi.size() != 0)
        allSi.clear();
    
    // 0
    VecXd si = VecXd::Zero(xSize); 
    allSi.emplace_back(si);
    // 1 ~ 2n(n-1)
    for (size_t k = 0; k < 4; k++){
        for(size_t i = 0; i < xSize - 1; i++){
            for(size_t j = i + 1; j < xSize; j++){
                VecXd tmp = VecXd::Zero(xSize);
                
                double scale0, scale1;
                getScaleHCKF(scale0, scale1, k);                
                tmp(i) = scale0;
                tmp(j) = scale1;

                allSi.emplace_back(gamma_ * tmp);                               
            }
        }
    }
    // 2n(n-1) + 1 ~ 2n^2    
    MatXd I = MatXd::Identity(xSize, xSize);
    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(gamma_ * I.col(i));
    }

    for(size_t i = 0; i < xSize; i++){
        allSi.emplace_back(-gamma_ * I.col(i));
    }   
    // check dims
    if(allSi.size() != 2 * xSize * xSize + 1)
        cout << "Error size of Si !\n";
}

void SCSPKF::getScaleHCKF(double &scale0, double &scale1, size_t k){
    double tmp = 0.5 * sqrt(2.0); //sqrt(2) / 2
    
    switch (k)
    {
    case 0:
        scale0 = tmp;
        scale1 = tmp;
        break;
    case 1:
        scale0 = tmp;
        scale1 = -tmp;
        break;
    case 2:
        scale0 = -tmp;
        scale1 = tmp;
        break;
    case 3:
        scale0 = -tmp;
        scale1 = -tmp;
        break;
    default:
        cout << "error k!\n";
        break;
    }
}

void SCSPKF::computeWeight(vector<double> &weightMu, vector<double> &weightSigma, int xSize){
    switch (sigmaType_)
    {
    case SP_UKF:
        computeWeightUKF(weightMu, weightSigma, xSize);        
        break;
    case SP_CKF:
        computeWeightCKF(weightMu, weightSigma, xSize);
        break;
    case SP_HCKF:
        computeWeightHCKF(weightMu, weightSigma, xSize);
        break;
    default:
        cout << "Unknown sample type !\n";
        break;
    }
}

void SCSPKF::computeWeightUKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize){
    // clear container
    weightMu.clear(); weightSigma.clear();
    // ====== weight mu ====== //
    double scale = xSize + lambda_;
    double W0 = lambda_ / scale;
    double W1 = lambda_ / scale + 1 - alpha_ * alpha_ + beta_;
    double W2 = 1. / (2. * scale);
    // 0
    weightMu.emplace_back(W0);
    weightSigma.emplace_back(W1);
    // 1 ~ 2n
    auto iter = weightMu.begin() + 1;
    int cnt = 2 * xSize;
    weightMu.insert(iter, cnt, W2);

    iter = weightSigma.begin() + 1;
    weightSigma.insert(iter, cnt, W2);
}

void SCSPKF::computeWeightCKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize){
    // clear container
    weightMu.clear(); weightSigma.clear();
    // ====== weight mu ====== //
    double W = 1. / (2. * xSize) ;
    
    // 2n
    int cnt = 2 * xSize;
    auto iter = weightMu.begin();
    weightMu.insert(iter, cnt, W);
    // ====== weight sigma ====== //
    iter = weightSigma.begin();
    weightSigma.insert(iter, cnt, W);   
}

void SCSPKF::computeWeightHCKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize){
    // clear container
    weightMu.clear(); weightSigma.clear();
    // ====== weight mu ====== //
    double scale = xSize + 2.;
    double W0 = 2. / scale;
    double W1 = 1. / (scale * scale);
    double W2 = (4. - xSize) / (2. * scale * scale);
    // 0
    weightMu.emplace_back(W0);
    // 1 ~ 2n(n-1)
    auto iter = weightMu.begin() + 1;
    int cnt = 2 * xSize * (xSize - 1); // 2n(n-1)
    weightMu.insert(iter, cnt, W1);
    // 2n(n-1) + 1 ~ 2n^2
    iter = weightMu.begin() + cnt + 1;
    cnt = 2 * xSize; // 2n
    weightMu.insert(iter, cnt, W2);
    // ====== weight sigma ====== //
    weightSigma.assign(weightMu.begin(), weightMu.end());
}

void SCSPKF::oneStepPrediction(VecXd &U){
    if(!flagInitialized_){
        cout << "Please call initSCKF() first !\n";
        return;
    }
    if(sigmaType_ == SP_UKF && !ukfInit_){
        cout << "Please call setUKFParams() to initiate parameters.\n";
        return;
    }
    // clear point contailer
    sPointsX_.clear();
    sPointsY_.clear();
    // generate sigma points
    genSigmaPoints(sPointsX_);
    // propagate sigma points
    propagateFcn(sPointsX_, sPointsY_, U);
    // calculate mean and covariance
    Mu_ = calcWeightedMean(sPointsY_, weightMu_);
    Sigma_ = calcWeightedCov(sPointsY_, weightMu_ , weightSigma_) + Q_;
    // compute Jacobian with statistical linearization
    MatXd SigmaXY = calcWeightedCrossCov(sPointsX_, sPointsY_, weightMu_ , weightSigma_);
    MatXd F = SigmaXY.transpose() * Sigma_.inverse(); // F = Pyx * Pxx_inv
    // multiplicative matrix
    Phi_ = Phi_ * F;
}

void SCSPKF::oneStepUpdate(VecXd &Z){
    // augmented mean and covariance
    augMu_ = VecXd::Zero(2 * xSize_);
    augSigma_ = MatXd::Zero(2 * xSize_, 2 * xSize_);

    augMu_.segment(0, xSize_) = lastMu_;
    augMu_.segment(xSize_, xSize_) = Mu_;

    augSigma_.block(0, 0, xSize_, xSize_) = lastSigma_;
    augSigma_.block(0, xSize_, xSize_, xSize_) = lastSigma_ * Phi_.transpose();
    augSigma_.block(xSize_, 0, xSize_, xSize_) = Phi_ * lastSigma_;
    augSigma_.block(xSize_, xSize_, xSize_, xSize_) = Sigma_;
    // clear container
    sPointsX_.clear();
    sPointsY_.clear();
    // generate sigma points
    genSigmaPoints(sPointsX_, true);
    // propagate sigma points
    updateFcn(sPointsX_, sPointsY_);
    VecXd tmpZ = calcWeightedMean(sPointsY_, weightMuAug_);
    MatXd SigmaZZ = calcWeightedCov(sPointsY_, weightMuAug_, weightSigmaAug_) + R_;
    MatXd SigmaXZ = calcWeightedCrossCov(sPointsX_, sPointsY_, weightMuAug_, weightSigmaAug_);
    // compute Kalman gain
    MatXd K = SigmaXZ * SigmaZZ.inverse();
    residual_ = Z - tmpZ;
    // update augment state and covariance
    augMu_ += K * residual_;
    augSigma_ -= K * SigmaZZ * K.transpose();
    // margliza old state
    Mu_ = augMu_.segment(0, xSize_);
    Sigma_ = augSigma_.block(xSize_, xSize_, xSize_, xSize_);
    // reset state
    lastMu_ = Mu_;
    lastSigma_ = Sigma_;
    Phi_ = MatXd::Identity(xSize_, xSize_);
}

void SCSPKF::propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U){
    if(sPointsY_.size() != 0)
        sPointsY_.clear();
    
    // ---- test 3D const demo ---- //
    for(auto it : sPointsX_){
        sPointsY_.emplace_back(it);       
    }
}

void SCSPKF::updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY){
    if(sPointsY_.size() != 0)
        sPointsY_.clear();
    // ---- test 3D const demo ---- //
    VecXd curZ = VecXd::Zero(mSize_ / 2);
    VecXd lastZ = VecXd::Zero(mSize_ / 2);
    for(auto it: sPointsX_){
        VecXd tmpZ = VecXd::Zero(8); 
        // compute lastZ
        double scale = sqrt(it(0) * it(0) + it(1) * it(1));
        lastZ(0) = atan2(it(2), scale);
        lastZ(1) = atan2(it(1), it(0));

        scale = sqrt((it(0) - 10) * (it(0) - 10) + it(1) * it(1));
        lastZ(2) = atan2(it(2), scale);
        lastZ(3) = atan2(it(1), it(0) - 10.);
        // compute curZ
        scale = sqrt(it(3) * it(3) + it(4) * it(4));
        curZ(0) = atan2(it(5), scale);
        curZ(1) = atan2(it(4), it(3));

        scale = sqrt((it(3) - 10) * (it(3) - 10) + it(4) * it(4));
        curZ(2) = atan2(it(5), scale);
        curZ(3) = atan2(it(4), it(3) - 10.);

        tmpZ.segment(0, 4) = curZ - lastZ;
        tmpZ.segment(4, 4) = curZ;

        sPointsY.emplace_back(tmpZ);
    }        
}
//====================================================================//
//====================================================================//
VecXd SCSPKF::calcWeightedMean(vector<VecXd> &sPointsX, const vector<double> &weightMu){
    int size = sPointsX[0].size();
    VecXd mu = VecXd::Zero(size);

    for(size_t i = 0; i < sPointsX.size(); i++){
        mu += weightMu[i] * sPointsX[i];
    }

    return mu;
}

MatXd SCSPKF::calcWeightedCov(vector<VecXd> &sPointsX, const vector<double> &weightMu, const vector<double> &weightSigma){
    VecXd mu = calcWeightedMean(sPointsX, weightMu);
    
    int size = sPointsX[0].size();
    MatXd cov = MatXd::Zero(size, size);

    for(size_t i = 0; i < sPointsX.size(); i++){
        VecXd delta = sPointsX[i] - mu;
        cov += weightSigma[i] * delta * delta.transpose();
    }
    // cout << cov << endl;
    return cov;
}

MatXd SCSPKF::calcWeightedCrossCov(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, const vector<double> &weightMu, const vector<double> &weightSigma)
{
    // VecXd muX = calcWeightedMean(weights, sPointsX);
    VecXd muX = sPointsX[0];
    VecXd muY = calcWeightedMean(sPointsY, weightMu);
    
    int sizeX = sPointsX[0].size();
    int sizeY = sPointsY[0].size();

    MatXd cCov = MatXd::Zero(sizeX, sizeY);
    for(size_t i = 0; i < sPointsX.size(); i++){
        VecXd deltaX = sPointsX[i] - muX;
        VecXd deltaY = sPointsY[i] - muY;
        cCov += weightSigma[i] * deltaX * deltaY.transpose();
    }

    return cCov;
}

void SCSPKF::printSi(vector<VecXd> allSi, string name, int maxPerRow){
    cout << name << ": " << endl;
    
    MatXd outSi = MatXd::Ones(allSi[0].size(), maxPerRow);
    int cnt = 0;
    int siSize = allSi.size();
    cout << right << fixed << setprecision(3);
    for(size_t i = 0; i < siSize; i++){
        outSi.col(cnt) = allSi[i];
        cnt++;

        if(cnt == maxPerRow){
            cout << outSi << "\n\n";
            outSi = MatXd::Ones(allSi[0].size(), maxPerRow);
            cnt = 0;
        }    
    }
    cout << outSi << "\n\n";
}

void SCSPKF::printWeight(vector<double> allWeight, string name, int maxPerRow){
    cout << name << ": " << endl;
    
    int cnt = 0;
    int wSize = allWeight.size();
    cout << right << fixed << setprecision(3);
    for(size_t i = 0; i < wSize; i++){
        cout << allWeight[i] << " ";
        cnt++;
        if(cnt == maxPerRow){
            cout << endl;
            cnt = 0;
        }
    }
    cout << "\n\n";
}

}// namespace MyFusion