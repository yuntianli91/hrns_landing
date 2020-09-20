#ifndef SCEKF_H_
#define SCEKF_H_
#include "backend/sckf/sckf.h"

namespace MyFusion{

class SCEKF : public SCKF{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SCEKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R);

    void oneStepPrediction(VecXd &U) override;
    void oneStepUpdate(VecXd &Z) override;

    virtual void computeJacobianF();
    virtual void computeJacobianG();
    virtual void computeJacobianH();

protected:
    MatXd F_, G_, H_; // Jacobians

};

}// namespace MyFusion

#endif