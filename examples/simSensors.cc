#include "simulator/sensorSimulator.h"
#include "utilities/io_function.h"

using namespace std;
using namespace MyFusion;

int main(int argc, char** argv){

    SensorSimulator mySimulator("../config/simulator/sensorConfig.yaml");
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

    return 0;
}