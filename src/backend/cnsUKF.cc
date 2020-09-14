#include "backend/cnsUKF.h"

namespace MyFusion{

CnsUKF::CnsUKF():SPKF(){
}

CnsUKF::CnsUKF(double alpha, double beta, double kappa):SPKF(){
    alpha_ = alpha;
    beta_ = beta;
    kappa_ = kappa;
}

void CnsUKF::genSigmaPoints(vector<VecXd> &points){
    if(points.size() != 0)
        points.clear();

    MatXd sqrtS = curSigma_.llt().matrixL();
    VecXd tmp = curMu_;

    points.emplace_back(tmp);
    // 1 ~ n
    for(size_t i = 0; i < xDim_; i++){
        tmp = curMu_ + sqrt(xDim_ + lambda_) * sqrtS.col(i);
        points.emplace_back(tmp);
    }
    // n+1 ~ 2n
    for(size_t i = 0; i < xDim_; i++){
        tmp = curMu_ - sqrt(xDim_ + lambda_) * sqrtS.col(i);
        points.emplace_back(tmp);
    }
}

void CnsUKF::computeWeight(){
    lambda_ = alpha_ * alpha_ * (xDim_ + kappa_) - xDim_;

    weightMu_.clear(); weightSigma_.clear();

    double W0 = lambda_ / (xDim_ + lambda_);
    double Wi = 0.5 / (xDim_ + lambda_);
    weightMu_.emplace_back(W0);
    weightMu_.insert(weightMu_.begin() + 1, 2 * xDim_, Wi);

    weightSigma_.assign(weightMu_.begin(), weightMu_.end());
    W0 += (1. - alpha_ * alpha_ + beta_);
    weightSigma_[0] = W0;
}

void CnsUKF::propagateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY){
    if(pointsY.size() != 0)
        pointsY.clear();
    
    // ---- test 3D const
    for(auto it : pointsX){
        pointsY.emplace_back(it);       
    }
}

void CnsUKF::updateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY){
    if(pointsY.size() != 0)
        pointsY.clear();
    // ---- test 3D const
    VecXd tmp = VecXd::Zero(mDim_);
    for(auto it: pointsX){
        double scale = sqrt(it.x() * it.x() + it.y() * it.y());
        tmp(0) = atan2(it.z(), scale);
        tmp(1) = atan2(it.y(), it.x());

        scale = sqrt((it.x() - 10.) * (it.x() - 10.) + it.y() * it.y());
        tmp(2) = atan2(it.z(), scale);
        tmp(3) = atan2(it.y(), it.x() - 10.);
        // tmp = sqrt(it);
        pointsY.emplace_back(tmp);
    }        
}


}