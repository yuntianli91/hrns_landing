#include "simulator/sensorSimulator.h"
#include "utilities/io_function.hpp"

using namespace std;
using namespace MyFusion;

int main(int argc, char** argv){

    SensorSimulator mySimulator("../config/simulator/sensorConfig.yaml");
    
    vector<ImuMotionData> trajData;
    readImuMotionData("../data/standardTraj/trajMCMF.csv", trajData);

    // vector<ImuMotionData> imuData;
    // mySimulator.simIMU(trajData, imuData);
    // writeImuMotionData("../data/sensorSimData/imuData.csv", imuData);

    vector<CnsData> cnsData;
    mySimulator.simCNS(trajData, cnsData);
    writeCnsData("../data/sensorSimData/cnsData.csv", cnsData);

    vector<VirnsData> virnsData;
    mySimulator.simAbsVIRNS(trajData, virnsData);
    writeVirnsData("../data/sensorSimData/virnsData.csv", virnsData);

    vector<CmnsData> cmnsData;
    mySimulator.simCMNS(trajData, cmnsData);
    writeCmnsData("../data/sensorSimData/cmnsData.csv", cmnsData);


    return 0;
}