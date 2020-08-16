#ifndef IO_FUNCTION_H_
#define IO_FUNCTION_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"

using namespace std;
using namespace myFusion;

int readImuParam(string filename, ImuParam &param){
    cv::FileStorage fsParams(filename, cv::FileStorage::READ);

    if(!fsParams.isOpened()){
        cout << "ERROR: failed to open config file. Please reset parameters!\n";

    }
    else{
        param.acc_b_ = fsParams["acc_b"];
        param.gyr_b_ = fsParams["gyr_b"];
        param.acc_n_ = fsParams["acc_n"];
        param.gyr_n_ = fsParams["gyr_n"];
        param.acc_w_ = fsParams["acc_w"];
        param.gyr_w_ = fsParams["gyr_w"];

        param.time_step_ = fsParams["imu_step"];

    }
}

void readImuMotionData(string filename, vector<ImuMotionData> &imu_data){
    // string datafile = datapath + "data_imu.csv";
    FILE *fp = fopen((filename).c_str(), "r");
    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }
    // 跳过文件头
    char header[1024];
    fgets(header, 1024, fp);
    // 读取数据到相关容器中
    imu_data.clear();
    while(!feof(fp)){
        double time_stamp(0);
        double px(0.), py(0.), pz(0.);
        double qw(0.), qx(0.), qy(0.), qz(0.);
        double roll(0.), pitch(0.), yaw(0.); 
        double vx(0.), vy(0.), vz(0.);
        double wx(0.), wy(0.), wz(0.), ax(0.), ay(0.), az(0.);
        int ref = fscanf(fp, "%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le\n",
                        &time_stamp, &px, &py, &pz,
                        &qw, &qx, &qy, &qz,
                        &roll, &pitch, &yaw,
                        &vx, &vy, &vz,
                        &wx, &wy, &wz,
                        &ax, &ay, &az);
        if(ref == -1){break;} // avoid read last line twicea

        ImuMotionData tmp;
        tmp.time_stamp_ = time_stamp;
        tmp.tnb_ = Vec3d(px, py, pz);
        tmp.vel_ = Vec3d(vx, vy, vz);
        tmp.qnb_ = Qd(qw, qx, qy, qz);
        tmp.eulerAngles_ = Vec3d(roll, pitch, yaw);
        tmp.acc_ = Vec3d(ax, ay, az);
        tmp.gyr_ = Vec3d(wx, wy, wz);

        imu_data.emplace_back(tmp);
    }
    fclose(fp);    
}

void writeImuMotionData(string filename, vector<ImuMotionData> &imu_data){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], p_RS_R_x[m], p_RS_R_y[m], p_RS_R_z[m], q_RS_w[], q_RS_x[], q_RS_y[], q_RS_z[], Roll[deg], Pitch[deg], Yaw[deg],"); 
    fprintf(fp, "v_R_x[m/s], v_R_y[m/s], v_R_z[m/s], gyr_S_x[rad/s], gyr_S_y[rad/s], gyr_S_z[rad/s], acc_S_x[m/s^2], acc_S_y[m/s^2], acc_S_z[m/s^2]\n"); 

    for (auto it:imu_data){
        // fprintf(fp, "%f,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", it.time_stamp_, 
        fprintf(fp, "%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le\n", 
                it.time_stamp_, 
                it.tnb_.x(), it.tnb_.y(), it.tnb_.z(),
                it.qnb_.w(), it.qnb_.x(), it.qnb_.y(), it.qnb_.z(),
                it.eulerAngles_.x(), it.eulerAngles_.y(), it.eulerAngles_.z(),
                it.vel_.x(), it.vel_.y(), it.vel_.z(),
                it.gyr_.x(), it.gyr_.y(), it.gyr_.z(),
                it.acc_.x(), it.acc_.y(), it.acc_.z());     
    }
}

void writePosNED(string filename, vector<ImuMotionData> &imu_data){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], p_N[m], p_E[m], p_D[m]\n"); 

    for (auto it:imu_data){
        fprintf(fp, "%e,%e,%e,%e\n", it.time_stamp_, 
                        it.pos_.x(), it.pos_.y(), it.pos_.z());     
    }
}



#endif