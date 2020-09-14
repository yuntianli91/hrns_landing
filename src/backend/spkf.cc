#include "backend/spkf.h"

namespace MyFusion{

void SPKF::initSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R)
{
    // check covariance matrix
    if (Q.cols() != Q.rows() || R.cols() != R.rows()){
        cerr << "Q or R is not square !\n";
        exit(1);
    }
    if (Sigma.cols() != Sigma.rows()){
        cerr << "Sigma is not square !\n";
        exit(1);
    }
    if(Mu.size() != Sigma.cols() || Mu.size() != Q.cols() 
        || Q.cols() != Sigma.cols())
    {
        cerr << "Mu, Sigma or Q size is wrong ! Their size are:\n"
            << "Mu: " << Mu.size() << ", "
            << "Sigma: " << Sigma.rows() << " x " << Sigma.cols() << ", "
            << "Q: " << Q.rows() << " x " << Q.cols() << endl;
        exit(1);
    }

    // set variables
    curMu_ = Mu;
    curSigma_ = Sigma;
    Q_ = Q;
    R_ = R;
    // get dimensions of state and measurements    
    xDim_ = Q_.cols();
    mDim_ = R_.cols();
    //
    computeWeight();
    //
    flagInitiated_= true;
}

void SPKF::oneStepPrediction(){
    if(!flagInitiated_){
        cerr << "Please call init() to initiate filter first !\n";
        return;
    }
    // clear containers
    sPointsX_.clear();
    sPointsY_.clear();
    // generate sigma points
    genSigmaPoints(sPointsX_);
    // propogate sigma points by f(x)
    propagateFcn(sPointsX_, sPointsY_);
    // compute predicted mean and covariance
    curMu_ = calcWeightedMean(sPointsY_);
    curSigma_ = calcWeightedCov(sPointsY_) + Q_;

    // cout << "MU_P: " << curMu_.transpose() << endl;
    // cout << "Sigma_P:\n" << curSigma_ << endl;
}

void SPKF::oneStepUpdate(VecXd &Z){
    if(!flagInitiated_){
        cerr << "Please call init() to initiate filter first !\n";
        return;
    }
    // clear containers
    sPointsX_.clear();
    sPointsY_.clear();
    // generate sigma points
    genSigmaPoints(sPointsX_);
    // propogate sigma points by h(x)
    updateFcn(sPointsX_, sPointsY_);
    // compute update mean and covariance
    VecXd tmpM = calcWeightedMean(sPointsY_);
    MatXd tmpS = calcWeightedCov(sPointsY_) + R_; 
    MatXd tmpC = calcWeightedCrossCov(sPointsX_, sPointsY_);
    // cout << "tmpC:\n" << tmpC << endl;
    // cout << "tmpS:\n" << tmpS << endl;
    // compute gain
    MatXd K = tmpC * tmpS.inverse();
    // update
    curResidual_ = Z - tmpM;
    curMu_ += K * curResidual_;
    curSigma_ -= K * tmpS * K.transpose();

    // cout << "MU_U: " << curMu_.transpose() << endl;
    // cout << "Sigma_U:\n" << curSigma_ << endl;
}

VecXd SPKF::calcWeightedMean(vector<VecXd> &sPointsX){
    int size = sPointsX[0].size();
    VecXd mu = VecXd::Zero(size);

    for(size_t i = 0; i < sPointsX.size(); i++){
        mu += weightMu_[i] * sPointsX[i];
    }

    return mu;
}

MatXd SPKF::calcWeightedCov(vector<VecXd> &sPointsX){
    VecXd mu = calcWeightedMean(sPointsX);
    
    int size = sPointsX[0].size();
    MatXd cov = MatXd::Zero(size, size);

    for(size_t i = 0; i < sPointsX.size(); i++){
        VecXd delta = sPointsX[i] - mu;
        cov += weightSigma_[i] * delta * delta.transpose();
    }
    // cout << cov << endl;
    return cov;
}

MatXd SPKF::calcWeightedCrossCov(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY){
    // VecXd muX = calcWeightedMean(weights, sPointsX);
    VecXd muX = sPointsX[0];
    VecXd muY = calcWeightedMean(sPointsY);
    
    int sizeX = sPointsX[0].size();
    int sizeY = sPointsY[0].size();

    MatXd cCov = MatXd::Zero(sizeX, sizeY);
    for(size_t i = 0; i < sPointsX.size(); i++){
        VecXd deltaX = sPointsX[i] - muX;
        VecXd deltaY = sPointsY[i] - muY;
        cCov += weightSigma_[i] * deltaX * deltaY.transpose();
    }

    return cCov;
}
    

} // namespace MyFusion    