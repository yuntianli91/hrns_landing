#ifndef BACKPARAM_H_
#define BACKPARAM_H_
#include "commonHeaders.h"
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace MyFusion{

extern double IMU_STEP;
extern double INIT_ERR_P, INIT_ERR_V;
extern MatXd INIT_SQRT_P, INIT_SQRT_Q, INIT_SQRT_R;
extern string IMU_FILE, CNS_FILE, VIRNS_FILE, CMNS_FILE;
extern int SIGMA_TYPE;

// load backend parameters
void loadBackParam(string configFile);

}

#endif