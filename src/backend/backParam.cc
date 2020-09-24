#include "backend/backParam.h"

namespace MyFusion
{
double IMU_STEP;
double INIT_ERR_P, INIT_ERR_V;
MatXd INIT_SQRT_P, INIT_SQRT_Q, INIT_SQRT_R;
string IMU_FILE, CNS_FILE, VIRNS_FILE, CMNS_FILE, ALT_FILE;
int SIGMA_TYPE;
double UKF_A, UKF_B, UKF_K;

void loadBackParam(string configFile){
    cv::FileStorage fsBackendParams(configFile, cv::FileStorage::READ);
    if(!fsBackendParams.isOpened()){
        cerr << "ERROR: failed to open backend config file !" << endl;
        return;
    }
    // data files
    string dataPath;
    fsBackendParams["dataPath"] >> dataPath;
    IMU_FILE = dataPath + "imuData.csv";
    CNS_FILE = dataPath + "cnsData.csv";
    VIRNS_FILE = dataPath + "virnsData.csv";
    CMNS_FILE = dataPath + "cmnsData.csv";
    ALT_FILE = dataPath + "altData.csv";
    // IMU Step
    fsBackendParams["imuStep"] >> IMU_STEP;
    // Sigma type
    fsBackendParams["sigmaType"] >> SIGMA_TYPE;
    // UKF parameters
    fsBackendParams["alpha"] >> UKF_A;
    fsBackendParams["beta"] >> UKF_B;
    fsBackendParams["kappa"] >> UKF_K;

    // initial position and velocity error
    fsBackendParams["initErrP"] >> INIT_ERR_P;
    fsBackendParams["initErrV"] >> INIT_ERR_V;
    // initial P, Q, R
    cv::Mat tmp;
    fsBackendParams["initSqrtP"] >> tmp;
    cv::cv2eigen(tmp, INIT_SQRT_P);
    
    fsBackendParams["initSqrtQ"] >> tmp;
    cv::cv2eigen(tmp, INIT_SQRT_Q);
    
    fsBackendParams["initSqrtR"] >> tmp;
    cv::cv2eigen(tmp, INIT_SQRT_R);

    cout << "[0] Data file: "
        << "\n[0] IMU_FILE: " << IMU_FILE
        << "\n[0] CNS_FILE: " << CNS_FILE
        << "\n[0] VIRNS_FILE: " << VIRNS_FILE
        << "\n[0] CMNS_FILE: " << CMNS_FILE;
        
    // cout << "[B] Backend Parameters: "
    //     << "\nIMU_STEP: " << IMU_STEP
    //     << "\nINIT_ERR_P: " << INIT_ERR_P 
    //     << "\nINIT_ERR_V: " << INIT_ERR_V
    //     << "\nINIT_P diag:\n " << setiosflags(ios::right) << INIT_SQRT_P.diagonal().transpose() 
    //     << "\nINIT_Q diag:\n " << INIT_SQRT_Q.diagonal().transpose() 
    //     << "\nINIT_R diag:\n " << INIT_SQRT_R.diagonal().transpose() << endl; 
}
    
} // namespace MyFusion
