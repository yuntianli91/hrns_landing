#include "utilities/io_function.h"

namespace MyFusion{

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
    printf("[1] Reading trajectory data...\n");
    while(!feof(fp)){
        double time_stamp(0);
        double px(0.), py(0.), pz(0.);
        double qw(0.), qx(0.), qy(0.), qz(0.);
        double roll(0.), pitch(0.), yaw(0.); 
        double vx(0.), vy(0.), vz(0.);
        double wx(0.), wy(0.), wz(0.), ax(0.), ay(0.), az(0.);
        // int ref = fscanf(fp, "%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le,%le",
        int ref = fscanf(fp, "%lf,%le,%lf,%le,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%le,%le,%le,%le,%le,%le",
                        &time_stamp, &px, &py, &pz,
                        &qw, &qx, &qy, &qz,
                        &pitch, &yaw, &roll,
                        &vx, &vy, &vz,
                        &wx, &wy, &wz,
                        &ax, &ay, &az);
        if(ref == -1){
            break;
        } // avoid read last line twicea

        ImuMotionData tmp;
        tmp.time_stamp_ = time_stamp;
        tmp.tnb_ = Vec3d(px, py, pz);
        tmp.vel_ = Vec3d(vx, vy, vz);
        tmp.qnb_ = Qd(qw, qx, qy, qz);
        tmp.eulerAngles_ = Vec3d(pitch, yaw, roll);
        tmp.acc_ = Vec3d(ax, ay, az);
        tmp.gyr_ = Vec3d(wx, wy, wz);

        imu_data.emplace_back(tmp);
    }
    cout << "[1] Totally read " << imu_data.size() << " traj data.\n";
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

    fprintf(fp, "# time_stamp[s], p_RS_R_x[m], p_RS_R_y[m], p_RS_R_z[m], q_RS_w[], q_RS_x[], q_RS_y[], q_RS_z[], Pitch[deg], Yaw[deg], Roll[deg],"); 
    fprintf(fp, "v_R_x[m/s], v_R_y[m/s], v_R_z[m/s], gyr_S_x[rad/s], gyr_S_y[rad/s], gyr_S_z[rad/s], acc_S_x[m/s^2], acc_S_y[m/s^2], acc_S_z[m/s^2]\n"); 

    for (auto it:imu_data){
        fprintf(fp, "%lf,%le,%lf,%le,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%le,%le,%le,%le,%le,%le\n",
        // fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", 
        // fprintf(fp, "%le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le %le\n", 
                it.time_stamp_, 
                it.tnb_.x(), it.tnb_.y(), it.tnb_.z(),
                it.qnb_.w(), it.qnb_.x(), it.qnb_.y(), it.qnb_.z(),
                it.eulerAngles_.x(), it.eulerAngles_.y(), it.eulerAngles_.z(),
                it.vel_.x(), it.vel_.y(), it.vel_.z(),
                it.gyr_.x(), it.gyr_.y(), it.gyr_.z(),
                it.acc_.x(), it.acc_.y(), it.acc_.z());     
    }
}

void writeCnsData(string filename, vector<CnsData> &cnsData){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s],q_RS_w[],q_RS_x[],q_RS_y[],q_RS_z[],Pitch[deg],Yaw[deg],Roll[deg]\n"); 
    
    for (auto it:cnsData){
        fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", 
                it.timeStamp_, 
                it.qnb_.w(), it.qnb_.x(), it.qnb_.y(), it.qnb_.z(),
                it.eulerAngle_.x(), it.eulerAngle_.y(), it.eulerAngle_.z());     
    }
}

void writeVirnsData(string filename, vector<VirnsData> &virnsData){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], dp_x[m], dp_x[m], dp_z[m], p_x[m], p_y[m], p_z[m]\n"); 
    
    for (auto it:virnsData){
        fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", 
                it.timeStamp_, 
                it.dPos_.x(), it.dPos_.y(), it.dPos_.z(),
                it.pos_.x(), it.pos_.y(), it.pos_.z());     
    }
}

void writeCmnsData(string filename, vector<CmnsData> &cmnsData){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], lat[rad], lon[rad]\n"); 
    
    for (auto it:cmnsData){
        fprintf(fp, "%lf,%le,%le\n", 
                it.timeStamp_, 
                it.pos_.x(), it.pos_.y());     
    }
}

void writeAltData(string filename, vector<AltData> &altData){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], range[m]\n"); 
    
    for (auto it:altData){
        fprintf(fp, "%lf,%lf\n", 
                it.timeStamp_, it.range_);     
    }
}


void writePos(string filename, vector<ImuMotionData> &imu_data){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "# time_stamp[s], p_N[m], p_U[m], p_E[m], acc_N[m/s^2], acc_N[m/s^2], acc_N[m/s^2]\n"); 

    for (auto it:imu_data){
        fprintf(fp, "%e,%e,%e,%e,%e,%e,%e\n", it.time_stamp_, 
                        it.pos_.x(), it.pos_.y(), it.pos_.z(),
                        it.acc_n_.x(), it.acc_n_.y(), it.acc_n_.z());     
    }
}

void writeAllanData(string filename, vector<ImuMotionData> &imu_data){
    FILE *fp;
    struct stat buffer;
    if(stat(filename.c_str(), &buffer) == 0)
        system(("rm " + filename).c_str());    
    fp = fopen(filename.c_str(), "w+");

    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << filename << endl;
        return;
    }

    fprintf(fp, "#time_stamp[s],acc_x[m/s^2],acc_y[m/s^2],acc_z[m/s^2],gyr_x[rad/s],gyr_y[rad/s],gyr_z[rad/s],\n"); 

    for (auto it:imu_data){
        fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", 
                it.time_stamp_, 
                it.acc_.x(), it.acc_.y(), it.acc_.z(),
                it.gyr_.x(), it.gyr_.y(), it.gyr_.z());     
    }

}

void readCnsData(string fileName, vector<CnsData> &cnsData){
    // string datafile = datapath + "data_imu.csv";
    FILE *fp = fopen((fileName).c_str(), "r");
    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }
    // 跳过文件头
    char header[1024];
    fgets(header, 1024, fp);
    // 读取数据到相关容器中
    cnsData.clear();
    printf("[1] Reading CNS data...\n");
    while(!feof(fp)){
        double time_stamp(0);
        double qw(0.), qx(0.), qy(0.), qz(0.);
        double roll(0.), pitch(0.), yaw(0.); 
        int ref = fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                        &time_stamp, 
                        &qw, &qx, &qy, &qz,
                        &pitch, &yaw, &roll);
        if(ref == -1){
            break;
        } // avoid read last line twicea

        CnsData tmp;
        tmp.timeStamp_ = time_stamp;
        tmp.qnb_ = Qd(qw, qx, qy, qz);
        tmp.eulerAngle_ = Vec3d(pitch, yaw, roll);
        
        cnsData.emplace_back(tmp);
    }
    cout << "[1] Totally read " << cnsData.size() << " CNS data.\n";
    fclose(fp);    
}

void readVirnsData(string fileName, vector<VirnsData> &virnsData){
    // string datafile = datapath + "data_imu.csv";
    FILE *fp = fopen((fileName).c_str(), "r");
    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }
    // 跳过文件头
    char header[1024];
    fgets(header, 1024, fp);
    // 读取数据到相关容器中
    virnsData.clear();
    printf("[1] Reading VIRNS data...\n");
    while(!feof(fp)){
        double time_stamp(0);
        double dPx(0.), dPy(0.), dPz(0.);
        double px(0.), py(0.), pz(0.); 
        int ref = fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                        &time_stamp, 
                        &dPx, &dPy, &dPz,
                        &px, &py, &pz);
        if(ref == -1){
            break;
        } // avoid read last line twicea

        VirnsData tmp;
        tmp.timeStamp_ = time_stamp;
        tmp.dPos_ = Vec3d(dPx, dPy, dPz);
        tmp.pos_ = Vec3d(px, py, pz);
        
        virnsData.emplace_back(tmp);
    }
    cout << "[1] Totally read " << virnsData.size() << " VIRNS data.\n";
    fclose(fp);    
}

void readCmnsData(string fileName, vector<CmnsData> &cmnsData){
    // string datafile = datapath + "data_imu.csv";
    FILE *fp = fopen((fileName).c_str(), "r");
    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }
    // 跳过文件头
    char header[1024];
    fgets(header, 1024, fp);
    // 读取数据到相关容器中
    cmnsData.clear();
    printf("[1] Reading CMNS data...\n");
    while(!feof(fp)){
        double time_stamp(0);
        double lat(0.), lon(0.); 
        int ref = fscanf(fp, "%lf,%le,%le\n",
                        &time_stamp, &lat, &lon);
        if(ref == -1){
            break;
        } // avoid read last line twicea

        CmnsData tmp;
        tmp.timeStamp_ = time_stamp;
        tmp.pos_ = Vec2d(lat, lon);

        cmnsData.emplace_back(tmp);        
    }
    cout << "[1] Totally read " << cmnsData.size() << " CMNS data.\n";
    fclose(fp);    
}


void readAltData(string fileName, vector<AltData> &altData){
    // string datafile = datapath + "data_imu.csv";
    FILE *fp = fopen((fileName).c_str(), "r");
    if (fp == nullptr){
        cerr << "ERROR: failed to open file: " << fileName << endl;
        return;
    }
    // 跳过文件头
    char header[1024];
    fgets(header, 1024, fp);
    // 读取数据到相关容器中
    altData.clear();
    printf("[1] Reading CMNS data...\n");
    while(!feof(fp)){
        double time_stamp(0);
        double range(0.); 
        int ref = fscanf(fp, "%lf,%lf\n",
                        &time_stamp, &range);
        if(ref == -1){
            break;
        } // avoid read last line twicea

        AltData tmp;
        tmp.timeStamp_ = time_stamp;
        tmp.range_ = range;

        altData.emplace_back(tmp);        
    }
    cout << "[1] Totally read " << altData.size() << " Altimeter data.\n";
    fclose(fp);    
}

/**
 * @brief print percentage of progress
 * 
 * @param name 
 * @param per 
 */
void printPer(string name, float per){
    const char symbol[4] = {'|','/','-','\\'};
    printf("[#][%s][%.2f%%][%c]\r", name.c_str(), per, symbol[(int)per%4]);
    fflush(stdout);
}
/**
 * @brief print percentage of progress
 * 
 * @param name 
 * @param per 
 */
void printPer(string name, int per){
    const char symbol[4] = {'|','/','-','\\'};
    printf("[#][%s][%d%%][%c]\r", name.c_str(), per, symbol[(int)per%4]);
    fflush(stdout);
}

} // namespace MyFusion


// #endif