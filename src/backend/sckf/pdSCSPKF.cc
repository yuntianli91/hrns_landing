#include "backend/sckf/pdSCSPKF.h"

namespace MyFusion
{
    
PdSCSPKF::PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType):SCSPKF(Mu,Sigma,Q,R,sigmaType){
    cout << "Initiated SCSPKF for power descend.\n";
}

void PdSCSPKF::propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U){
    if(sPointsY.size() != 0)
        sPointsY.clear();
    
    for (auto it: sPointsX){
        VecXd tmp = VecXd::Zero(xSize_);
        // [L h l Vn Vu Ve]       
        double lat = it(0); double alt = it(1); double lon = it(2);
        double Vn = it(3); double Vu = it(4); double Ve = it(5);
        Vec3d vel0(Vn, Vu, Ve);
        double h_m = R_m + alt;
        // ------ update position ------ //
        tmp(0) = it(0) + IMU_STEP * Vn / h_m; //latitude
        tmp(1) = it(1) + IMU_STEP * Vu; // height
        tmp(2) = it(2) + IMU_STEP * Ve / (h_m * cos(lat)); //longitude
        // ------ update velocity ------ //
        // compute w^G_im = [W_im * cosL, 0, -W_m * sinL]
        Vec3d w_im(W_im * cos(lat), W_im * sin(lat), 0.);
        // compute w^G_mg = [v_e / h_m, -v_n / h_m, -v_e * tanL / h_m]
        Vec3d w_mg(Ve / h_m, Ve * tan(lat) / h_m, -Vn / h_m);
        // compute gravity
        Vec3d gn = Vec3d::Zero(); // gravity vector
        gn.y() = -g0_m * (R_m * R_m) / (h_m * h_m); // NUE
        Vec3d acc_n = U - (2. * w_im + w_mg).cross(vel0) + gn;

        tmp.segment(3, 3) = vel0 + acc_n * IMU_STEP;
       
        sPointsY.emplace_back(tmp);
    }
}

void PdSCSPKF::updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY){
    if(sPointsY.size() != 0)
        sPointsY.clear();
    
    VecXd lastP = VecXd::Zero(3);
    VecXd curP = VecXd::Zero(3);
    
    for (auto it: sPointsX){
        lastP.x() = (R_m + lastMu_.y()) * cos(lastMu_.x()) * cos(lastMu_.z());
        lastP.y() = (R_m + lastMu_.y()) * cos(lastMu_.x()) * sin(lastMu_.z());
        lastP.z() = (R_m + lastMu_.y()) * sin(lastMu_.x());
    
        curP.x() = (R_m + Mu_.y()) * cos(Mu_.x()) * cos(Mu_.z());
        curP.y() = (R_m + Mu_.y()) * cos(Mu_.x()) * sin(Mu_.z());
        curP.z() = (R_m + Mu_.y()) * sin(Mu_.x());

        VecXd tmp = VecXd::Zero(curMSize_);
        if(curMSize_ == mSize_){
            // [dx,dy,dz,L,l]
            tmp.segment(0, 3) = curP - lastP;
            tmp.segment(3, 2) = Vec2d(Mu_.x(), Mu_.z());
        }
        else{
            // [dx dy dz]
            tmp = curP - lastP;
        }

        sPointsY_.emplace_back(tmp);
    }
}
    
} // namespace MyFusion
