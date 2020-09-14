#ifndef CNS_UKF_H_
#define CNS_UKF_H_
#include "backend/spkf.h"

namespace MyFusion{

class CnsUKF : public SPKF{
public:
    CnsUKF();
    CnsUKF(double alpha, double beta, double kappa);
    ~CnsUKF(){}

    void genSigmaPoints(vector<VecXd> &points);

    void computeWeight();

    void propagateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY);

    void updateFcn(vector<VecXd> &pointsX, vector<VecXd> &pointsY);

protected:
    double alpha_ = 0.001;
    double beta_ = 2.0;
    double kappa_ = 0.;
    double lambda_;


}; // class

} // namespace

#endif