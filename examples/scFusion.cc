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

    float simTime(0);
    int sigmaType(1);
    int updateType;
    string fileName;
    if(argc == 3){
        sigmaType = atoi(argv[1]);
        fileName = argv[2]; 
    }
    else if(argc == 4){
        simTime = atof(argv[1]);
        sigmaType = atoi(argv[2]);
        fileName = argv[3];
    }
    else if(argc == 5){
        simTime = atof(argv[1]);
        sigmaType = atoi(argv[2]);
        updateType = atoi(argv[3]);
        fileName = argv[4];
    }
    else{
        cerr << "Wrong parameters number.\n";
        return -1;
    }
    // ---------- simulation ---------- //
    Estimator myEstimator("../config/fusion/backParam.yaml", updateType);    
    myEstimator.setSigmaType(sigmaType);
    myEstimator.setOutFile(fileName);
    // myEstimator.setUpdateType(updateType);

    if(simTime == 0){
        myEstimator.processBackend();
    }
    else{
        myEstimator.processBackend(simTime);
    }

    // myEstimator.showResults(); 

    // ---------- plot figure ---------- //
    return 0;
}