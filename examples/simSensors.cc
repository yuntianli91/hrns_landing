#include "simulator/sensorSimulator.h"
#include "utilities/io_function.h"

using namespace std;
using namespace MyFusion;

int main(int argc, char** argv){
    printf("\n#################### Sensor Simulation Start ####################\n");
    float acc, period;
   
    SensorSimulator mySimulator("../config/simulator/sensorConfig.yaml");
    if(argc == 3){
        acc = atof(argv[1]);
        period = atof(argv[2]) / 10.;
        mySimulator.sensorParams_.virns_sigma_ = acc;
        mySimulator.sensorParams_.alt_step_ = period;
        mySimulator.sensorParams_.cmns_step_ = period;
    }
    // 读取轨迹数据
    vector<ImuMotionData> trajData;
    readImuMotionData("../data/stdTraj/caGeo.csv", trajData);
    // // 生成IMU数据
    vector<ImuMotionData> imuData;
    mySimulator.simIMU(trajData, imuData);
    writeImuMotionData("../data/sensorSimData/imuData.csv", imuData);
    writePos("../data/sensorSimData/posNED.csv", imuData);
    // 生成CNS数据
    // vector<CnsData> cnsData;
    // mySimulator.simCNS(trajData, cnsData);
    // writeCnsData("../data/sensorSimData/cnsData.csv", cnsData);
    // 生成相对量测数据
    vector<VirnsData> virnsData;
    mySimulator.simVIRNS(trajData, virnsData);
    writeVirnsData("../data/sensorSimData/virnsData.csv", virnsData);
    // // 生成绝对量测数据
    vector<CmnsData> cmnsData;
    mySimulator.simCMNS(trajData, cmnsData);
    writeCmnsData("../data/sensorSimData/cmnsData.csv", cmnsData);

    vector<AltData> altData;
    mySimulator.simAltimeter(trajData, altData);
    writeAltData("../data/sensorSimData/altData.csv", altData);
    printf("#################### Sensor Simulation Done ####################\n");

    return 0;
}