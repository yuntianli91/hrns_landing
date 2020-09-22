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

    virtual void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U);
    virtual void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);

};

} // namespace MyFusion
#endif