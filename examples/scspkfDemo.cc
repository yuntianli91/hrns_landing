#include "commonHeaders.h"
#include "backend/sckf/scspkf.h"
#include <matplotlibcpp.h>
#include "utilities/tic_toc.h"
#include "simulator/sensorNoise.hpp"

using namespace std;
using namespace MyFusion;

namespace plt = matplotlibcpp; // matplotlib-cpp
/**
 * @brief convert data from Eigen to vector for matplotlib
 * 
 * @param Mu 
 * @param Sigma 
 * @param idx : idx of state needed to be convert 
 * @param allX  : converted state
 * @param allP  : converted covariance (3sigma)
 */
void getForPlot(vector<vector<VecXd>> &Mu, vector<vector<MatXd>> &Sigma, int idx,
            vector<vector<double>> &allX, vector<vector<double>> &allP);

void getForPlot(vector<vector<VecXd>> &Mu, int idx, vector<vector<double>> &allX);

int main(int argc, char ** argv){
    // -------- data generation -------- // 
    int N = 500;
    vector<Vec3d> x(N);
    vector<VecXd> y(N);

    int mCnt = 0;
    for(int i = 0; i < N; i++){
        x[i] = Vec3d::Ones() * 5.0;
        if(mCnt % 20 == 0){
            // relative plus absolute
            VecXd noiseY = VecXd::Zero(8);
            noiseY.segment(4, 4) = Vec4d(0.61548, M_PI_4, 0.61548, 3. * M_PI_4);
            SensorNoise::addGlintNoise(noiseY, 0.1, 0.6, GAUSSIAN, 0.0);
            y[i] = noiseY;
        }
        else{
            // relative
            VecXd noiseY = VecXd::Zero(4);
            SensorNoise::addGlintNoise(noiseY, 0.1, 0.6, GAUSSIAN, 0.0);
            y[i] = noiseY;   
        }
   }
    // cout << x.transpose() << endl;
    // cout << y.transpose() << endl;

    // -------- filter -------- //
    if(argc != 2){
        cout << "Usage: cnsFusion [errScale]\n";    
        return -1;
    }
    int errScale = atoi(argv[1]);

    Eigen::VectorXd Mu0 = Vec3d::Ones() * 5.0 + (double)errScale * Vec3d(0.1,0.1,0.1);
    Eigen::MatrixXd Sigma0 = Eigen::MatrixXd::Identity(3, 3) * 1;
    Eigen::MatrixXd Q0 = Eigen::MatrixXd::Identity(3, 3) * 0.001;
    Eigen::MatrixXd R0 = Eigen::MatrixXd::Identity(8, 8) * 0.01;
    // cout << Mu0 << endl << Sigma0 << endl << Q0 << endl << R0 << endl;
    
    SCSPKF mySCHCKF(Mu0, Sigma0, Q0, R0, SP_HCKF);
    SCSPKF mySCCKF(Mu0, Sigma0, Q0, R0, SP_CKF);
    SCSPKF mySCUKF(Mu0, Sigma0, Q0, R0, SP_UKF);

    mySCUKF.setUKFParams(0.001, 2, 0);

    vector<vector<VecXd>> all_err;
    vector<vector<VecXd>> all_mu;
    vector<vector<MatXd>> all_sigma;
    
    double filterCost[3] = {0., 0., 0.};
    vector<VecXd> tmpMu;
    vector<VecXd> tmpErr;
    vector<MatXd> tmpSigma;    

    TicToc filterTimer;
    // ============ SC-HCKF ============= //
    for (int i = 0; i < N; i++){
        Eigen::VectorXd Zk = y[i];
        Eigen::VectorXd Uk = VecXd::Zero(3);

        filterTimer.tic();
        mySCHCKF.oneStepPrediction(Uk);
        mySCHCKF.oneStepUpdate(Zk);
        filterCost[0] += filterTimer.toc();

        VecXd mu = mySCHCKF.getMu();
        MatXd Sigma = mySCHCKF.getSigma();
    
        tmpMu.emplace_back(mu);
        tmpErr.emplace_back((mu - x[i]).cwiseAbs());
        tmpSigma.emplace_back(Sigma);
    }
    cout << "SC-HCKF total cost: " << filterCost[0] << endl;

    all_mu.emplace_back(tmpMu);
    all_err.emplace_back(tmpErr);
    all_sigma.emplace_back(tmpSigma);  
    
    tmpMu.clear(); tmpErr.clear(); tmpSigma.clear();

    // ============= SC-CKF =============== //
    for (int i = 0; i < N; i++){
        Eigen::VectorXd Zk = y[i];
        Eigen::VectorXd Uk = VecXd::Zero(3);
        // ------ SC-HCKF ------- //
        filterTimer.tic();
        mySCCKF.oneStepPrediction(Uk);
        mySCCKF.oneStepUpdate(Zk);
        filterCost[1] += filterTimer.toc();

        VecXd mu = mySCCKF.getMu();
        MatXd Sigma = mySCCKF.getSigma();
        
        tmpMu.emplace_back(mu);
        tmpErr.emplace_back((mu - x[i]).cwiseAbs());
        tmpSigma.emplace_back(Sigma);
    }
    cout << "SC-CKF total cost: " << filterCost[1] << endl;

    all_mu.emplace_back(tmpMu);
    all_err.emplace_back(tmpErr);
    all_sigma.emplace_back(tmpSigma);  
    tmpMu.clear(); tmpErr.clear(); tmpSigma.clear();

    // ============ SC-UKF ================ //
    for (int i = 0; i < N; i++){
        Eigen::VectorXd Zk = y[i];
        Eigen::VectorXd Uk = VecXd::Zero(3);

        filterTimer.tic();
        mySCUKF.oneStepPrediction(Uk);
        mySCUKF.oneStepUpdate(Zk);
        filterCost[2] += filterTimer.toc();

        VecXd mu = mySCUKF.getMu();
        MatXd Sigma = mySCUKF.getSigma();
        
        tmpMu.emplace_back(mu);
        tmpErr.emplace_back((mu - x[i]).cwiseAbs());
        tmpSigma.emplace_back(Sigma);
    }
    cout << "SC-UKF total cost: " << filterCost[2] << endl;

    all_mu.emplace_back(tmpMu);
    all_err.emplace_back(tmpErr);
    all_sigma.emplace_back(tmpSigma);  
    tmpMu.clear(); tmpErr.clear(); tmpSigma.clear();

    // ====================== matplotlib-cpp ================================= //
    vector<vector<double>> xTrue; // true value
    vector<vector<double>> xHCKF, xCKF, xUKF; // mean
    vector<vector<double>> pHCKF, pCKF, pUKF; // covariance
    vector<vector<double>> eHCKF, eCKF, eUKF; // error

    getForPlot(all_mu, all_sigma, 0, xHCKF, pHCKF);
    getForPlot(all_mu, all_sigma, 1, xCKF, pCKF);
    getForPlot(all_mu, all_sigma, 2, xUKF, pUKF);

    getForPlot(all_err, 0, eHCKF);
    getForPlot(all_err, 1, eCKF);
    getForPlot(all_err, 2, eUKF);

   // -------------------------------
    plt::figure();
    plt::subplot(3, 1, 1);
    plt::named_plot("HCKF", eHCKF[0], "--c");    
    plt::named_plot("CKF", eCKF[0], "--r");    
    plt::named_plot("UKF", eUKF[0], "--b");    
    plt::named_plot("3sigma", pHCKF[0], "--k");    
   
    plt::subplot(3, 1, 2);
    plt::named_plot("HCKF", eHCKF[1], "--c");    
    plt::named_plot("CKF", eCKF[1], "--r");    
    plt::named_plot("UKF", eUKF[1], "--b");    
    plt::named_plot("3sigma", pHCKF[1], "--k");    
    
    plt::subplot(3, 1, 3);
    plt::named_plot("HCKF", eHCKF[2], "--c");    
    plt::named_plot("CKF", eCKF[2], "--r");    
    plt::named_plot("UKF", eUKF[2], "--b");    
    plt::named_plot("3sigma", pHCKF[2], "--k");    
   
    plt::show();
    // ============================================================ //

    return 0;

}

void getForPlot(vector<vector<VecXd>> &Mu, vector<vector<MatXd>> &Sigma, int idx,
            vector<vector<double>> &allX, vector<vector<double>> &allP)
{
    int N = Mu[0].size();
    vector<double> tmpX(N), tmpY(N), tmpZ(N);
    vector<double> tmpPx(N), tmpPy(N), tmpPz(N);

    for(size_t i = 0; i < N; i++){
        tmpX.at(i) = Mu[idx][i](0);
        tmpY.at(i) = Mu[idx][i](1);
        tmpZ.at(i) = Mu[idx][i](2);

        tmpPx.at(i) = 3. * sqrt(Sigma[idx][i](0, 0));
        tmpPy.at(i) = 3. * sqrt(Sigma[idx][i](1, 1));
        tmpPz.at(i) = 3. * sqrt(Sigma[idx][i](2, 2));
    }

    allX.clear(); allP.clear();
    allX.emplace_back(tmpX); allX.emplace_back(tmpY); allX.emplace_back(tmpZ);
    allP.emplace_back(tmpPx); allP.emplace_back(tmpPy); allP.emplace_back(tmpPz);
}

void getForPlot(vector<vector<VecXd>> &Mu, int idx, vector<vector<double>> &allX)
{
    int N = Mu[0].size();
   
    vector<double> tmpX(N), tmpY(N), tmpZ(N);
    for(size_t i = 0; i < N; i++){
        tmpX.at(i) = Mu[idx][i](0);
        tmpY.at(i) = Mu[idx][i](1);
        tmpZ.at(i) = Mu[idx][i](2);
    }

    allX.clear();
    allX.emplace_back(tmpX); allX.emplace_back(tmpY); allX.emplace_back(tmpZ);
}

