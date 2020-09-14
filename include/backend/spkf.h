#ifndef SPKF_H_
#define SPKF_H_
#include "commonHeaders.h"

using namespace std;

namespace MyFusion
{

/**
 * @brief base class for all sigma-points based Kalman filter
 * 
 */
class SPKF{
public:
    SPKF(){}
    virtual ~SPKF(){}

    void initSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R);

    // ======================= //
    virtual void genSigmaPoints(vector<VecXd> &sPoints) = 0;
    
    virtual void computeWeight() = 0;

    virtual void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY) = 0;

    virtual void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY) = 0;
    // ====================== //
    void oneStepPrediction();
    void oneStepUpdate(VecXd &Z);
    VecXd calcWeightedMean(vector<VecXd> &sPointsX);
    MatXd calcWeightedCov(vector<VecXd> &sPointsX);
    MatXd calcWeightedCrossCov(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);
    // ====================== //
    VecXd getMu(){return curMu_;}
    MatXd getSigma(){return curSigma_;}

protected:
    VecXd curMu_; // current mean
    MatXd curSigma_; // current covariance
    VecXd curResidual_;

    vector<VecXd> sPointsX_; // sigma points before propagation
    vector<VecXd> sPointsY_; // sigma point after propagation

    int xDim_, mDim_;
    MatXd Q_, R_; // noise matrice

    vector<double> weightMu_;
    vector<double> weightSigma_;

    bool flagInitiated_ = false;                                                                 
};

} // namespace MyFusion

#endif