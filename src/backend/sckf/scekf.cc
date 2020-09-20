#include "backend/sckf/scekf.h"

namespace MyFusion{

SCEKF::SCEKF(VecXd Mu, MatXd Sigma, MatXd Q, MatXd R):SCKF(Mu, Sigma, Q, R){

}

void SCEKF::oneStepPrediction(VecXd &U){

}

void SCEKF::oneStepUpdate(VecXd &Z){

}


void SCEKF::computeJacobianF(){

}

void SCEKF::computeJacobianG(){

}

void SCEKF::computeJacobianH(){

}

} // namespace MyFusion