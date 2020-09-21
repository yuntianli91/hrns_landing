#ifndef SCSPKF_H_
#define SCSPKF_H_
#include "backend/sckf/sckf.h"

#define varName(x) #x //get variable name

// type of sample points
enum SampleType{
    SP_UKF = 0,
    SP_CKF,
    SP_HCKF
};

namespace MyFusion{

class SCSPKF : public SCKF{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SCSPKF(){} // delete default constructor
    SCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType=SP_HCKF);
    void setUKFParams(double alpha, double beta, double kappa);
    // ------ filter functions ------ //
    void genSigmaPoints(vector<VecXd> &sPoints, bool aug=0);
    // void genSigmaPointsCKF(vector<VecXd> &sPoints);
    void genSi(vector<VecXd> &allSi, int xSize);
    void genSiUKF(vector<VecXd> &allSi, int xSize); // generate UKF cubature points
    void genSiCKF(vector<VecXd> &allSi, int xSize); // generate CKF cubature points
    void genSiHCKF(vector<VecXd> &allSi, int xSize); // generate HCKF cubature points
    void getScaleHCKF(double &scale0, double &scale1, size_t k);

    void computeWeight(vector<double> &weightMu, vector<double> &weightSigma, int xSize);
    void computeWeightUKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize); // compute UKF weights
    void computeWeightCKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize); // compute CKF weights
    void computeWeightHCKF(vector<double> &weightMu, vector<double> &weightSigma, int xSize); // compute HCKF weights
 
    void oneStepPrediction(VecXd &U) override;
    void oneStepUpdate(VecXd &Z) override;
 
    virtual void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U);
    virtual void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);
    // ------ support functions ------ //
    VecXd calcWeightedMean(vector<VecXd> &sPointsX, const vector<double> &weightMu); // calculate weighted mean
    MatXd calcWeightedCov(vector<VecXd> &sPointsX, const vector<double> &weightMu, const vector<double> &weightSigma); // calculate weighted covariance
    MatXd calcWeightedCrossCov(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, const vector<double> &weightMu, const vector<double> &weightSigma); // calculate weighted corss-covariance
    // ------ io and debug functions ------ //
    void printSi(vector<VecXd> allSi, string name, int maxPerRow);
    void printWeight(vector<double> allWeight, string name, int maxPerRow);

protected:
    SampleType sigmaType_; // type of sigma points
    vector<VecXd> allSi_, allSiAug_; // cubature points for mu and augMu
    vector<VecXd> sPointsX_; // sigma points before propagation
    vector<VecXd> sPointsY_; // sigma point after propagation

    vector<double> weightMu_, weightMuAug_; // weight of mu and augMu
    vector<double> weightSigma_, weightSigmaAug_; // weight of sigma and augSigma

    double alpha_, beta_, kappa_, lambda_; // parameters of UKF 
    double gamma_; // coefficient HCKF(sqrt(n+2)), CKF(sqrt(n)), UKF(sqrt(n+lambda)) 
    // bool firstGen_ = true;
    bool ukfInit_ = false;
};


}// namespace MyFusion

#endif