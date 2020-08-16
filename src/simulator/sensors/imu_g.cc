#include "simulator/sensors/imu_g.h"

namespace myFusion{

void IMU_G::oneStepIntegration(){
    //
    double curLat = tnb_.x(); // current latitude
    double curLon = tnb_.y(); // current longitude
    double curAlt = tnb_.z(); // current height
    double h_m = R_m + curAlt; // (R_m + h)
    // compute w^G_im = [W_im * cosL, 0, -W_m * sinL]
    Vec3d w_im(W_im * cos(curLat), 0., -W_im * sin(curLat));
    // compute w^G_mg = [v_e / h_m, -v_n / h_m, -v_e * tanL / h_m]
    Vec3d w_mg(vel_.y() / h_m,
             -vel_.x() / h_m,
             -vel_.y() * tan(curLat) / h_m); 
    
    if (intType == 1)
    {
        // ========================= mid intergration =================== //
        // compute attitude
        Vec3d gyr_mid = 0.5 * (gyr_0_ + gyr_1_) - gyr_bias_; // mid gyro measurement
        Vec3d w_gb = gyr_mid - qnb_.conjugate() * (w_im + w_mg);
        
        Eigen::Quaterniond qnb0 = qnb_;
        Eigen::Quaterniond dq(1., 0.5 * w_gb.x() * time_step_, 0.5 * w_gb.y() * time_step_, 0.5 * w_gb.z() * time_step_);
        dq.normalize();
        qnb_ = qnb0 * dq;
        qnb_.normalize();

        // compute gravity
        Eigen::Vector3d gn = Vec3d::Zero(); // gravity vector
        gn.z() = computeG(curAlt); // NED

        // compute velocity
        Vec3d acc_mid = 0.5 * (qnb0 * (acc_0_ - acc_bias_) + qnb_ * (acc_1_ - acc_bias_));
        Vec3d vel0 = vel_;
        Vec3d acc_n = acc_mid - (2. * w_im + w_mg).cross(vel0) + gn;
        vel_ = vel0 + acc_n * time_step_;

        // compute position
        Vec3d vel_mid =0.5 * (vel0 + vel_);
        tnb_.x() += time_step_ *  vel_mid.x() / h_m; // Lat
        tnb_.y() += time_step_ * vel_mid.y() / (h_m * cos(curLat)); // Lon
        tnb_.z() += -time_step_ * vel_mid.z(); // alt
    }    
    else if(intType == 0)
    {
        // ============================= euler integration ==================== //
        Vec3d w_gb = gyr_0_ - qnb_.conjugate() * (w_im + w_mg);
                
        Eigen::Quaterniond qnb0 = qnb_;
        Eigen::Quaterniond dq(1., 0.5 * w_gb.x() * time_step_, 0.5 * w_gb.y() * time_step_, 0.5 * w_gb.z() * time_step_);
        dq.normalize();
        qnb_ = qnb0 * dq;
        qnb_.normalize();

        // compute gravity
        Eigen::Vector3d gn = Vec3d::Zero(); // gravity vector
        gn.z() = computeG(curAlt); // NED

        // compute velocity
        Vec3d vel0 = vel_;
        Vec3d acc_n = qnb0 * acc_0_ - (2. * w_im + w_mg).cross(vel0) + gn;
        vel_ = vel0 + acc_n * time_step_;

        // compute position
        tnb_.x() += time_step_ * vel0.x() / h_m; //latitude
        tnb_.y() += time_step_ * vel0.y() / (h_m * cos(curLat)); //longitude
        tnb_.z() += -time_step_ * vel0.z(); // height
        pos_ += vel0 * time_step_ + 0.5 * acc_n * time_step_ * time_step_;
    } 

}

vector<ImuMotionData> IMU_G::trajGenerator(ImuMotionData initPose, vector<Vec3d> a_b_all, vector<Vec3d> omega_gb_all){
    if (a_b_all.size() != omega_gb_all.size()){
        cout << "WARNING: v_gm_all size is not equal to omega_gb_all size.\n";
    }    

    // get initial values
    double lat = initPose.tnb_.x();
    double lon = initPose.tnb_.y();
    double alt = initPose.tnb_.z();
    double time_stamp = initPose.time_stamp_;
    Vec3d pos_g(0., 0., 0.);

    Vec3d v_g = initPose.vel_;
    Eigen::Quaterniond qnb = initPose.qnb_;    

    // ============ trajectory generation ============ // 
    vector<ImuMotionData> traj_data;
    for (size_t i = 0; i < a_b_all.size(); i++){
 
        double h_m = R_m + alt; // (R_m + h)
        // ----- compute compensation variables
        Vec3d w_im(W_im * cos(lat), 0., -W_im * sin(lat));
        
        Vec3d w_mg(v_g.y() / h_m,
                -v_g.x() / h_m,
                -v_g.y() * tan(lat) / h_m);

        Eigen::Vector3d gn = Vec3d::Zero(); // gravity vector
        gn.z() = computeG(alt); // NED
        // ----- compute specific force and angular rate
        Vec3d w_gb = omega_gb_all[i];
        Vec3d acc_b = a_b_all[i];

        Vec3d gyr_b = w_gb + qnb.conjugate() * (w_im + w_mg);  
        Vec3d acc_g = qnb * acc_b - (2. * w_im + w_mg).cross(v_g) + gn;
        
        // ----- save traj_data;
        ImuMotionData tmp_data;
        tmp_data.time_stamp_ = time_stamp; 
        tmp_data.tnb_ = Vec3d(lat, lon, alt);
        tmp_data.vel_ = v_g;
        tmp_data.qnb_ = qnb;
        tmp_data.eulerAngles_ = qnb.matrix().eulerAngles(2, 1, 0);
        tmp_data.acc_ = acc_b;
        tmp_data.gyr_ = gyr_b;
        tmp_data.pos_ = pos_g;

        traj_data.push_back(tmp_data);
        // ----- propagate trajectory
        Eigen::Quaterniond qnb0 = qnb;
        Eigen::Quaterniond dq(1., 0.5 * w_gb.x() * time_step_, 0.5 * w_gb.y() * time_step_, 0.5 * w_gb.z() * time_step_);
        dq.normalize();
        qnb = qnb0 * dq;
        qnb.normalize();

        lon += time_step_ * v_g.y() / (h_m * cos(lat)); //longitude
        lat += time_step_ * v_g.x() / h_m; //latitude
        alt += -time_step_ * v_g.z(); // height

        pos_g += v_g * time_step_ + 0.5 * acc_g * time_step_ * time_step_;

        v_g = v_g + acc_g * time_step_;

        time_stamp += time_step_;
    }

    return traj_data;
}

}