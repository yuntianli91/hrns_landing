#include "backend/sckf/pdSCSPKF.h"

namespace MyFusion
{
    
PdSCSPKF::PdSCSPKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R, SampleType sigmaType=SP_HCKF):SCSPKF(Mu,Sigma,Q,R,sigmaType){
    cout << "Initiated SCSPKF for power descend.\n";
}

void PdSCSPKF::propagateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY, VecXd &U){
    if(sPointsY.size() != 0)
        sPointsY.clear();
}

void PdSCSPKF::updateFcn(vector<VecXd> &sPointsX, vector<VecXd> &sPointsY){
    if(sPointsY.size() != 0)
        sPointsY.clear();
}
    
} // namespace MyFusion
