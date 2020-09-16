#ifndef SENSOR_SIMULATOR_H_
#define SENSOR_SIMULATOR_H_
#include "commonHeaders.h"
#include "utilities/utilities.hpp"
#include "utilities/io_function.hpp"
#include "simulator/sensors/imu_g.h"
#include "simulator/sensors/cns.h"
#include "simulator/sensors/cmns.h"
#include "simulator/sensors/virns.h"

using namespace std;

namespace MyFusion{

struct SensorParams
{
    // imu parameters
    double acc_b_, gyr_b_;
    double acc_n_, acc_w_;
    double gyr_n_, gyr_w_;
    double imu_step_;
    // cns parameters
    double cns_sigma_;
    double cns_step_;
    // virns parameters
    double virns_bias_;
    double virns_sigma_;
    double virns_step_;
    // cmns parameters
    double cmns_sigma_;
    double cmns_step_;
};



class SensorSimulator{
public:
    // must construct with config file
    SensorSimulator() = delete;
    SensorSimulator(string configFile);
    ~SensorSimulator();

    void readSensorParameters(string configFile);
    void showParameters(SensorParams params);

    /**
     * @brief 
     * 
     * @param trajData 
     * @param imuData 
     */
    void simIMU(const vector<ImuMotionData> trajData, vector<ImuMotionData> &imuData);
    
    /**
     * @brief 
     * 
     * @param trajData 
     * @param cnsData 
     */
    void simCNS(const vector<ImuMotionData> trajData, vector<CnsData> &cnsData);
    
    /**
     * @brief 
     * 
     * @param trajData 
     * @param virnsData 
     */
    void simVIRNSRelative(const vector<ImuMotionData> trajData, vector<VirnsData> &virnsData);

    /**
     * @brief 
     * 
     * @param trajData 
     * @param virnsData 
     */
    void simVIRNS(const vector<ImuMotionData> trajData, vector<VirnsData> &virnsData);
    
    /**
     * @brief 
     * 
     * @param trajData 
     * @param cmnsData 
     */
    void simCMNS(const vector<ImuMotionData> trajData, vector<CmnsData> &cmnsData);

    SensorParams sensorParams_;
    
    bool paramInitialized_ = false;

};

}

#endif