#include "commonHeaders.h"
#include "backend/cnsUKF.h"
#include "backend/cnsHCKF.h"
#include <matplotlibcpp.h>
#include "utilities/tic_toc.h"
#include "simulator/sensorNoise.hpp"

using namespace std;
using namespace MyFusion;

namespace plt = matplotlibcpp; // matplotlib-cpp

int main(int argc, char ** argv){

    // default_random_engine e;
    // normal_distribution<double> n(0, 0.1);
    // -------- data generation -------- // 
    int N = 500;
    vector<Vec3d> x(N);
    vector<Vec4d> y(N);

    for(int i = 0; i < N; i++){
        x[i] = Vec3d::Ones() * 5.0;
        VecXd noiseY = Vec4d(0.61548, M_PI_4, 0.61548, 3. * M_PI_4);
        SensorNoise::addGlintNoise(noiseY, 0.1, 0.6, GAUSSIAN, 0.0);
        y[i] = noiseY;
        // y[i](0) = 0.61548 + n(e);
        // y[i](1) = M_PI_4 + n(e);
        // y[i](2) = 0.61548 + n(e);
        // y[i](3) = 3. * M_PI_4 + n(e);
    }
    // cout << x.transpose() << endl;
    // cout << y.transpose() << endl;

    // -------- filter -------- //
    CnsUKF myCns;
    CnsHCKF myHCKF;

    if(argc != 2){
        cout << "Usage: cnsFusion [errScale]\n";    
        return -1;
    }
    int errScale = atoi(argv[1]);

    Eigen::VectorXd Mu0 = Vec3d::Ones() * 5.0 + (double)errScale * Vec3d(0.5,0.5,0.5);
    Eigen::MatrixXd Sigma0 = Eigen::MatrixXd::Identity(3, 3) * 9;
    Eigen::MatrixXd Q0 = Eigen::MatrixXd::Identity(3, 3) * 0.0001;
    Eigen::MatrixXd R0 = Eigen::MatrixXd::Identity(4, 4) * 0.01;
    // cout << Mu0 << endl << Sigma0 << endl << Q0 << endl << R0 << endl;
    
    myCns.initSPKF(Mu0, Sigma0, Q0, R0);
    myHCKF.initSPKF(Mu0, Sigma0, Q0, R0);

    vector<VecXd> all_mu;
    vector<VecXd> all_mu_h;
    vector<MatXd> all_sigma;

    double costUKF(0.), costHCKF(0.);
    TicToc filterTimer;
    for (int i = 0; i < N; i++){
        Eigen::VectorXd Zk = y[i];
        VecXd mu;
        MatXd Sigma;
        //-----------------------
        filterTimer.tic();
        myCns.oneStepPrediction();
        myCns.oneStepUpdate(Zk);
        costUKF += filterTimer.toc();

        mu = myCns.getMu();
        Sigma = myCns.getSigma();
        
        all_mu.emplace_back(mu);
        all_sigma.emplace_back(Sigma);
        //------------------------
        filterTimer.tic();
        myHCKF.oneStepPrediction();
        myHCKF.oneStepUpdate(Zk);
        costHCKF += filterTimer.toc();

        mu = myHCKF.getMu();       
        all_mu_h.emplace_back(mu);
    }
    cout << "UKF total cost: " << costUKF << endl;
    cout << "HCKF total cost: " << costHCKF << endl;

    // ====================== matplotlib-cpp ================================= //
    vector<double> x_est1(N), x_est2(N), x_est3(N);
    vector<double> x_est_h1(N), x_est_h2(N), x_est_h3(N);
    
    vector<double> p_est1(N), p_est2(N), p_est3(N);
    vector<double> x_true(N), y_true(N);



    for(int i = 0; i < N; i++){
        x_est1.at(i) = all_mu[i](0);
        x_est2.at(i) = all_mu[i](1);
        x_est3.at(i) = all_mu[i](2);

        x_est_h1.at(i) = all_mu_h[i](0);
        x_est_h2.at(i) = all_mu_h[i](1);
        x_est_h3.at(i) = all_mu_h[i](2);
 
        p_est1.at(i) = all_sigma[i](0, 0);         
        p_est2.at(i) = all_sigma[i](1, 1);         
        p_est3.at(i) = all_sigma[i](2, 2);         

        x_true.at(i) = x[i](0);
        y_true.at(i) = y[i](0);
    }
    // -------------------------------
    plt::figure();
    
    plt::subplot(3, 1, 1);
    plt::named_plot("true", x_true, "-b");
    // plt::named_plot("obs", y_true, "-g");
    // 设置属性
    std::map<string, string> keywords;
    keywords.insert(make_pair("Color","tomato"));
    keywords.insert(make_pair("ls","--"));

    plt::named_plot("ukf", x_est1, "--r");
    plt::named_plot("hckf", x_est_h1, "--c");
    plt::grid(true);
    plt::legend();

    plt::subplot(3, 1, 2);
    plt::named_plot("true", x_true, "-b");
    // plt::named_plot("obs", y_true, "-g");

    plt::named_plot("est", x_est2, "--r");
    plt::named_plot("hckf", x_est_h2, "--c");
    plt::grid(true);
    plt::legend();

    plt::subplot(3, 1, 3);
    plt::named_plot("true", x_true, "-b");
    // plt::named_plot("obs", y_true, "-g");

    plt::named_plot("est", x_est3, "--r");
    plt::named_plot("hckf", x_est_h3, "--c");
    plt::grid(true);
    plt::legend();


    // -----------------------------
    // plt::figure();
    // plt::named_plot("P", p_est1, "-b");
    // plt::legend();
    // plt::grid(true);
    // -----------------------------
    plt::show();
    // ============================================================ //

    return 0;

}

