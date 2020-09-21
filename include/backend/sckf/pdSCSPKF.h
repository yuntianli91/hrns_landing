#ifndef PD_SCSPKF_H_
#define PF_SCSPKF_H_
#include "backend/sckf/scspkf.h"

namespace MyFusion
{
class PdSCSPKF : public SCSPKF{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType=SP_HCKF);

    virtual void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U);
    virtual void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);

};

} // namespace MyFusion
#endif