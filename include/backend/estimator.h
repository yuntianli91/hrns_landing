#ifndef ESTIMATOR_BACK_H_
#define ESTIMATOR_BACK_H_
#include "backend/sckf/pdSCSPKF.h"
#include "utilities/io_function.h"
#include "utilities/utilities.hpp"
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;
namespace MyFusion
{

class Estimator{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Estimator(string configFile);
    ~Estimator();
    void initEstimator(string configFile);

    /**
     * @brief extract control vectors consist of acc and gyt
     * 
     * @param allU : queue of all control vectors 
     * @param imuData : imu simulation data
     */
    void extractU(queue<VecXd> &allU, const vector<ImuMotionData> &imuData);
    /**
     * @brief extract measurement vectors consist of relative and absolute
     * 
     * @param allZ : queue of all measurement vectors 
     * @param virnsData : data of visual-inertial relative navigation system
     * @param cmnsData : data of crater matching navigation system
     * @param altData : data of altimeter
     */
    void extractZ(queue<VecXd> &allZ, const vector<VirnsData> &virnsData, const vector<CmnsData> &cmnsData, const vector<AltData> &altData);

    void processBackend(double time=0);

    // ========== other functions ========= //
    template <typename T>
    void clearQueue(queue<T> &Q);

    void writeResults(string fileName, const vector<pair<double, VecXd>> allMu, 
            const vector<pair<double, MatXd>> allSigma, 
            const vector<pair<double, Qd>> allQnb);
    void writeResults(string fileName, const vector<pair<double, VecXd>> allMu, const vector<pair<double, MatXd>> allSigma);

    void showResults();
    void setOutFile(string fileName){outFile_ = fileName;}
    void setSigmaType(int type){sigmaType_ = SampleType(type);}
    
protected:
    vector<ImuMotionData> trajData_; // traj data
    queue<VecXd> allU_, allZ_; // container of all control and measurement vectors
    vector<pair<double, VecXd>> allMu_; // estimated mean
    vector<pair<double, MatXd>> allSigma_; // estimated covariance
    vector<pair<double, Qd>> allQnb_;

    int dataSize_; // size of all control vector
    bool dataInitiated_ = false; // estimator has been initiated or not

    VecXd Mu0_;
    MatXd Sigma0_, Q0_, R0_;
    SampleType sigmaType_;
    PdSCSPKF *filterPtr_; // pointer of filter

    string outFile_;
};
    
} // namespace MyFusion

#endif
