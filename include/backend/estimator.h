#ifndef ESTIMATOR_BACK_H_
#define ESTIMATOR_BACK_H_
#include "backend/sckf/pdSCSPKF.h"
#include "utilities/io_function.h"

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
     */
    void extractZ(queue<VecXd> &allZ, const vector<VirnsData> &virnsData, const vector<CmnsData> cmnsData);

    void processBackend();
    // ========== other functions ========= //
    template <typename T>
    void clearQueue(queue<T> &Q);

protected:
    queue<VecXd> allU_, allZ_; // container of all control and measurement vectors
    vector<VecXd> allMu; // estimated mean
    vector<MatXd> allSigma; // estimated covariance

    int dataSize_; // size of all control vector
    bool dataInitiated_ = false; // estimator has been initiated or not

    VecXd Mu0_;
    MatXd Sigma0_, Q0_, R0_;
    SampleType sigmaType_;
    PdSCSPKF *filterPtr_; // pointer of filter
};
    
} // namespace MyFusion

#endif
