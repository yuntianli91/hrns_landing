#include "backend/sckf/sckf.h"

namespace MyFusion{

SCKF::SCKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R){
    initSCKF(Mu, Sigma, Q, R);
}

void SCKF::initSCKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R){
    // ------ check size ------ //
    assert(Sigma.cols() == Sigma.rows());
    assert(Q.cols() == Q.rows());
    assert(R.cols() == R.rows());
    assert(Mu.size() == Sigma.rows());
    // ------ assigment ------ //
    xSize_ = Q.cols();
    mSize_ = R.cols();

    if(Mu.size() != xSize_ || Sigma.cols() != xSize_)
        cout << "[E] Unequal state and covariance size.\n";
    
    Mu_ = Mu; Sigma_ = Sigma;
    Q_ = Q; R_ = R;
    // ------ clone ------ //
    lastMu_ = Mu_;
    lastSigma_ = Sigma_;
    Phi_ = MatXd::Identity(xSize_, xSize_);

    flagInitialized_ = true;
}


}