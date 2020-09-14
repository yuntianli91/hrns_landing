#include "backend/cnsHCKF.h"

namespace MyFusion{

CnsHCKF::CnsHCKF():SPKF(){
}

void CnsHCKF::genSigmaPoints(vector<VecXd> &sPoints){
    if(sPoints.size() != 0)
        sPoints.clear();

    if(firstGen){
        genSi(allSi_);
        firstGen = false;
    }

    MatXd sqrtS = curSigma_.llt().matrixL();
    
    for(size_t i = 0; i < allSi_.size(); i++){
        VecXd point = curMu_ + sqrtS * allSi_[i];
        sPoints.emplace_back(point);
    }
    // ------------- print sigma points for debug
    int totalSize = sPoints.size();
    MatXd outSPoints = MatXd::Zero(xDim_, totalSize);
    for(size_t i = 0; i < totalSize; i++){
        outSPoints.col(i) = sPoints[i].transpose();
    }
    // cout << "Sigma Points:\n" << setprecision(3) << outSPoints << endl;
}

void CnsHCKF::genSi(vector<VecXd> &allSi){
    if(allSi.size() != 0)
        allSi.clear();
    
    VecXd si = VecXd::Zero(xDim_); 
    // 0
    allSi.emplace_back(si);
    // 1 ~ 2n(n-1)
    for (size_t k = 0; k < 4; k++){
        for(size_t i = 0; i < xDim_ - 1; i++){
            for(size_t j = i + 1; j < xDim_; j++){
                VecXd tmp = VecXd::Zero(xDim_);
                
                double scale0, scale1;
                getScales(scale0, scale1, k);                
                tmp(i) = scale0;
                tmp(j) = scale1;

                allSi.emplace_back(beta_ * tmp);                               
            }
        }
    }
    // 2n(n-1) + 1 ~ 2n^2    
    MatXd I = MatXd::Identity(xDim_, xDim_);
    for(size_t i = 0; i < xDim_; i++){
        allSi.emplace_back(beta_ * I.col(i));
    }

    for(size_t i = 0; i < xDim_; i++){
        allSi.emplace_back(-beta_ * I.col(i));
    }   
    // check dims
    if(allSi.size() != 2 * xDim_ * xDim_ + 1)
        cout << "Error size of Si !\n";
    // ------ for debug ------ //
    int siSize = allSi.size();
    MatXd outSi = MatXd::Zero(xDim_, siSize);
    for(size_t i = 0; i < siSize; i++){
        outSi.col(i) = allSi[i].transpose();
    }
    // cout << "Si:\n" << setprecision(3) << outSi << endl;
}

void CnsHCKF::getScales(double &scale0, double &scale1, size_t k){
    double tmp = 0.5 * sqrt(2.0);
    
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

void CnsHCKF::computeWeight(){
    beta_ = sqrt(xDim_ + 2.);

    weightMu_.clear(); weightSigma_.clear();
    // ====== weight mu ====== //
    double scale = xDim_ + 2.;
    double W0 = 2. / scale;
    double W1 = 1. / (scale * scale);
    double W2 = (4. - xDim_) / (2. * scale * scale);
    // 0
    weightMu_.emplace_back(W0);
    // 1 ~ 2n(n-1)
    auto iter = weightMu_.begin() + 1;
    int cnt = 2 * xDim_ * (xDim_ - 1);
    weightMu_.insert(iter, cnt, W1);
    // 2n(n-1) + 1 ~ 2n^2
    iter = weightMu_.begin() + cnt + 1;
    cnt = 2 * xDim_;
    weightMu_.insert(iter, cnt, W2);
    // ====== weight sigma ====== //
    weightSigma_.assign(weightMu_.begin(), weightMu_.end());
}

void CnsHCKF::propagateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY){
    if(pointsY.size() != 0)
        pointsY.clear();
    
    // ---- test 3D const
    for(auto it : pointsX){
        pointsY.emplace_back(it);       
    }
}

void CnsHCKF::updateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY){
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