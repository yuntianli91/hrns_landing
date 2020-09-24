#ifndef PD_SCSPKF_H_
#define PD_SCSPKF_H_
#include "backend/sckf/scspkf.h"
#include "backend/backParam.h"

namespace MyFusion
{
class PdSCSPKF : public SCSPKF{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PdSCSPKF();
    PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType=SP_HCKF);
    PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, double alpha, double beta, double kappa);

    virtual void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U);
    virtual void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);

    void setQnb(Qd qnb){qnb_ = qnb;}
    Qd getQnb(){return qnb_;}
protected:
    Eigen::Quaterniond qnb_;
};

} // namespace MyFusion
#endif