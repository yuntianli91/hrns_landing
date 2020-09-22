#include "backend/sckf/pdSCSPKF.h"
#include "backend/estimator.h"
#include "utilities/io_function.h"
#include <matplotlibcpp.h>

using namespace std;
using namespace MyFusion;
namespace plt = matplotlibcpp;

vector<ImuMotionData> trajData; 

int main(int argc, char** argv){
    readImuMotionData("../data/stdTraj/caGeo.csv", trajData);

    Estimator myEstimator("../config/fusion/backParam.yaml");    
   
    // ---------- simulation ---------- //

    // ---------- plot figure ---------- //
    return 0;
}