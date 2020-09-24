#include "backend/sckf/pdSCSPKF.h"

namespace MyFusion
{
    
PdSCSPKF::PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType):SCSPKF(Mu, Sigma, Q, R, sigmaType){
}


PdSCSPKF::PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R,
        double alpha, double beta, double kappa):SCSPKF(Mu, Sigma, Q, R, alpha, beta, kappa, SP_UKF)
{
    
}

void PdSCSPKF::propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U){
    if(sPointsY.size() != 0)
        sPointsY.clear();
    // TODO : update quaternion    
    Vec3d accG = qnb_ * U.segment(1,3);
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
        // compute w^G_im = [W_im * cosL, W_m * sinL, 0]
        Vec3d w_im(W_im * cos(lat), W_im * sin(lat), 0.);
        // compute w^G_mg = [v_e / h_m, v_e * tanL / h_m, -v_n / h_m]
        Vec3d w_mg(Ve / h_m, Ve * tan(lat) / h_m, -Vn / h_m);
        // compute gravity
        Vec3d gn = Vec3d::Zero(); // gravity vector
        // gn.y() = -g0_m; // NUE
        gn.y() = -g0_m * (R_m * R_m) / (h_m * h_m); // NUE

        Vec3d acc_n = accG - (2. * w_im + w_mg).cross(vel0) + gn;

        tmp.segment(3, 3) = vel0 + acc_n * IMU_STEP;
       
        sPointsY.emplace_back(tmp);
    }
    // update quaternion
    double h_m = R_m + Mu_(1);
    Vec3d w_im(W_im * cos(Mu_(0)), W_im * sin(Mu_(0)), 0.);
    Vec3d w_mg(Mu_(5) / h_m, Mu_(5) * tan(Mu_(0)) / h_m, -Mu_(3) / h_m);
    
    Vec3d w_gb = U.segment(4, 3) - qnb_.conjugate() * (w_im + w_mg);
    Eigen::Quaterniond dq(1., 0.5 * w_gb.x() * IMU_STEP, 0.5 * w_gb.y() * IMU_STEP, 0.5 * w_gb.z() * IMU_STEP);
    dq.normalize();
    qnb_ *= dq;
    qnb_.normalize();
}

void PdSCSPKF::updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY){
    if(sPointsY.size() != 0)
        sPointsY.clear();
    
    VecXd curP = VecXd::Zero(3);
    double curLat, curAlt, curLon;

    switch (updateType_)
    {
    case 0:
    {
        for (auto it: sPointsX){
            // [L, l, h]
            curLat = it(0); curAlt = it(1); curLon = it(2);
            VecXd tmp = VecXd::Zero(curMSize_);
            tmp.segment(0, 2) = Vec2d(curLat, curLon);
            tmp(2) = curAlt / qnb_.toRotationMatrix()(1, 1);

            sPointsY_.emplace_back(tmp);
        }
        break;
    }
    case 1:
    {
        for (auto it: sPointsX){
            curLat = it(0); curAlt = it(1); curLon = it(2);
    
            curP.x() = (R_m + curAlt) * cos(curLat) * cos(curLon);
            curP.y() = (R_m + curAlt) * cos(curLat) * sin(curLon);
            curP.z() = (R_m + curAlt) * sin(curLat);

            VecXd tmp = VecXd::Zero(curMSize_);
            if(curMSize_ == mSize_){
                // [dx,dy,dz,L,l,h]
                tmp.segment(0, 3) = curP;
                tmp.segment(3, 2) = Vec2d(curLat, curLon);
                tmp(5) = curAlt / qnb_.toRotationMatrix()(1, 1);
            }
            else{
                // [dx dy dz]
                tmp = curP;
            }
            sPointsY_.emplace_back(tmp);
        }
        break;
    }
    case 2:
    {
        VecXd lastP = VecXd::Zero(3);
        double preLat, preAlt, preLon;
        for (auto it: sPointsX){
            preLat = it(0); preAlt = it(1); preLon = it(2);
            curLat = it(6); curAlt = it(7); curLon = it(8);

            lastP.x() = (R_m + preAlt) * cos(preLat) * cos(preLon);
            lastP.y() = (R_m + preAlt) * cos(preLat) * sin(preLon);
            lastP.z() = (R_m + preAlt) * sin(preLat);
    
            curP.x() = (R_m + curAlt) * cos(curLat) * cos(curLon);
            curP.y() = (R_m + curAlt) * cos(curLat) * sin(curLon);
            curP.z() = (R_m + curAlt) * sin(curLat);

            VecXd tmp = VecXd::Zero(curMSize_);
            if(curMSize_ == mSize_){
                // [dx,dy,dz,L,l,h]
                tmp.segment(0, 3) = curP - lastP;
                tmp.segment(3, 2) = Vec2d(curLat, curLon);
                tmp(5) = curAlt / qnb_.toRotationMatrix()(1, 1);
            }
            else{
                // [dx dy dz]
                tmp = curP - lastP;
            }
            sPointsY_.emplace_back(tmp);
        }
        break;
    }
    default:
        cout << "[E] Error update type\n.";
        return;
    }
}
    
} // namespace MyFusion
