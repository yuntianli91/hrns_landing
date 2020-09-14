#include "commonHeaders.h"
#include "simulator/sensorNoise.hpp"
#include <matplotlibcpp.h>
#include "utilities/tic_toc.h"

using namespace std;
using namespace MyFusion;
namespace plt = matplotlibcpp; // matplotlib-cpp

int main(int argc, char** argv){
    // generate noise sequence    
    LaplacianDistribution<double> lap(0., 5.);
    default_random_engine rg;

    vector<double> laplacianSeq;
    
    vector<VecXd> noiseSeqGau;
    vector<VecXd> noiseSeqLap;
    
    vector<int> glintIdxGau;
    vector<int> glintIdxLap;
    for(size_t i = 0; i < 1000; i++){
        VecXd value = Vec3d(0., 0., 0.);
        int idx = SensorNoise::addGlintNoiseAll(value, 1., 5, GAUSSIAN, 0.2);
        noiseSeqGau.emplace_back(value);
        glintIdxGau.emplace_back(idx);
    
        value = Vec3d(0., 0., 0.);
        idx = SensorNoise::addGlintNoiseAll(value, 1., 5, LAPLACIAN, 0.2);
        noiseSeqLap.emplace_back(value);
        glintIdxLap.emplace_back(idx);

        laplacianSeq.emplace_back(lap(rg));
    }
    // check glint probability (for debug)
    int cnt = 0;
    for(auto it : glintIdxGau){
        if(it == 1)
            cnt++;
    }
    cout << "Real Glint probability Gaussian: " << (double)cnt / (double)glintIdxGau.size() << endl;
    
    cnt = 0;
    for(auto it : glintIdxLap){
        if(it == 1)
            cnt++;
    }
    cout << "Real Glint probability Laplacian: " << (double)cnt / (double)glintIdxLap.size() << endl;
    // convert for plt
    int M = noiseSeqGau.size();
    int N = noiseSeqGau[0].size();
    
    vector<vector<double>> noiseSeqGauPlt;
    vector<vector<double>> noiseSeqLapPlt;
    for(size_t i = 0; i < N; i++){
        vector<double> tmpGau, tmpLap;
        for(size_t j = 0; j < M; j++){
            tmpGau.emplace_back(noiseSeqGau[j](i));
            tmpLap.emplace_back(noiseSeqLap[j](i));
        }
        noiseSeqGauPlt.emplace_back(tmpGau);
        noiseSeqLapPlt.emplace_back(tmpLap);
    } 
    // plt
    plt::figure(1);   

    plt::subplot(3, 1, 1);
    plt::named_plot("GLMM", noiseSeqLapPlt[0], "-m");
    plt::named_plot("DGMM", noiseSeqGauPlt[0], "-c");
    plt::legend();
    plt::subplot(3, 1, 2);
    plt::named_plot("x", noiseSeqLapPlt[1], "-m");
    plt::subplot(3, 1, 3);
    plt::named_plot("x", noiseSeqLapPlt[2], "-m");

    // plt::figure(2);
    // plt::named_plot("x", laplacianSeq, "-b");
    // plt::save("glintNoise.pdf");
    plt::show();

    return 0;
}