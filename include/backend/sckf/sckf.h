#ifndef SCKF_H_
#define SCKF_H_
#include "commonHeaders.h"

using namespace std;

namespace MyFusion
{
class SCKF{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SCKF(){}
    SCKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R);
    void initSCKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R);
    // ------ filter functions ------ //
    /**
     * @brief one step prediction
     * 
     * @param U : control vector 
     */
    virtual void oneStepPrediction(VecXd &U) = 0;
    /**
     * @brief one step update
     * 
     * @param Z : measurement vector 
     */
    virtual void oneStepUpdate(VecXd &Z) = 0;
    // ------ io functions ------ //
    VecXd getMu(){return Mu_;}
    MatXd getSigma(){return Sigma_;}


protected:
    VecXd Mu_, lastMu_, augMu_; // current, clonal and augmented state
    MatXd Sigma_, lastSigma_, augSigma_; // current, clonal and augmented covariance
    MatXd Phi_; // multiplicative Jacobian
    MatXd Q_, R_;// process and measurement noise covariance matrix
    VecXd residual_; // current measurement residual

    int xSize_, mSize_; // size of state and measurements

    bool flagInitialized_ = false;
};

} // namespace MyFusion


#endif