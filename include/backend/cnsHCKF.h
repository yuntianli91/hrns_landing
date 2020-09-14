#ifndef CNS_HCKF_H_
#define CNS_HCKF_H_
#include "backend/spkf.h"

namespace MyFusion{

class CnsHCKF : public SPKF{
public:
    CnsHCKF();
    ~CnsHCKF(){}

    void genSigmaPoints(vector<VecXd> &sPoints);
    void genSi(vector<VecXd> &allSi);
    void getScales(double &scale0, double &scale1, size_t k);

    void computeWeight();

    void propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);

    void updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY);

protected:
    double beta_;
    vector<VecXd> allSi_;
    bool firstGen = true;

}; // class

} // namespace

#endif