#ifndef SENSOR_NOISE_H_
#define SENSOR_NOISE_H_
#include "commonHeaders.h"

using namespace std;

namespace MyFusion
{

enum GlintType {
    GAUSSIAN,
    LAPLACIAN,
    STUDENTT
};

template <typename _RealType = double>
class LaplacianDistribution{
public:
    // constructor with default (0., 1.)
    LaplacianDistribution(_RealType mu = _RealType(0), _RealType b = _RealType(1)):_mu(mu), _b(b){};
    
    template <typename _RandomGenerator>
    _RealType operator()(_RandomGenerator &gen){
        uniform_real_distribution<double> rng(0.0, 1.0);

        double cdf = rng(gen);
        double x;
        if(cdf <= 0.5){
            x = _mu + _b * log(2 * cdf);           
        }
        else{
            x = _mu - _b * log(2 * (1 - cdf)); 
        }
        return _RealType(x);
    }

    _RealType _mu, _b;
};


class SensorNoise{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    /**
     * @brief 
     * 
     * @param value true vector value 
     * @param sigma sigma of the noise 
     */
    static void addGaussianNoise(VecXd &value, double sigma){
        // set random distribution
        random_device rd;
        default_random_engine rg(rd());
        normal_distribution<double> stdGau(0., 1.); // standard gaissuain distribution        

        for (size_t i = 0; i < value.size(); i++){
            // TODO : add noise
            value(i) += sigma * stdGau(rg);
        }
    }

    /**
     * @brief 
     * 
     * @param value true vector value 
     * @param sigma1 sigma of the main gaussian distribution
     * @param sigma2 sigma of the glint gaussian/laplacian distribution
     * @param gProbability glint probability
     */
    static int addGlintNoise(VecXd &value, double sigma1, double sigma2, GlintType type=GAUSSIAN, double gProbability=0.1){
        // set random distribution
        random_device rd;
        default_random_engine rg(rd());
        
        uniform_real_distribution<double> stdUni(0.,1.); // standatd uniform distribution
        normal_distribution<double> stdGau(0., 1.); // standard gaussian distribution  
        LaplacianDistribution<double> stdLap(0., 1.); // standard laplacian distribution

        int glintIdx = stdUni(rg) <= gProbability ? 1 : 0;
        
        if(glintIdx){
            // glint
            switch (type)
            {
            case GAUSSIAN:
                for (size_t i = 0; i < value.size(); i++){
                    value(i) += sigma2 * stdGau(rg);
                }
                break;
            case LAPLACIAN:
                for (size_t i = 0; i < value.size(); i++){
                    value(i) += sigma2 * stdLap(rg);
                }
               break;
            }
        }   
        else{
            //unglint
            for (size_t i = 0; i < value.size(); i++){
                value(i) += sigma1 * stdGau(rg);
            }
        }     
        // return current glint index
        return (glintIdx);
    }
    
    /**
     * @brief 
     * 
     * @param value true vector value 
     * @param sigma1 sigma of the main gaussian distribution
     * @param sigma2 sigma of the glint gaussian/laplacian distribution
     * @param gProbability glint probability
     */
    static int addGlintNoiseAll(VecXd &value, double sigma1, double sigma2, GlintType type=GAUSSIAN, double gProbability=0.1){
        // set random distribution
        random_device rd;
        default_random_engine rg(rd());
        
        uniform_real_distribution<double> stdUni(0.,1.); // standatd uniform distribution
        normal_distribution<double> stdGau(0., 1.); // standard gaussian distribution  
        LaplacianDistribution<double> stdLap(0., 1.); // standard laplacian distribution

        int glintIdx = stdUni(rg) <= gProbability ? 1 : 0;
        
        double noise = stdGau(rg);
        if(glintIdx){
            // glint
            switch (type)
            {
            case GAUSSIAN:
                for (size_t i = 0; i < value.size(); i++){
                    value(i) += sigma2 * noise;
                }
                break;
            case LAPLACIAN:
                noise = stdLap(rg);
                for (size_t i = 0; i < value.size(); i++){
                    value(i) += sigma2 * noise;
                }
               break;
            }
        }   
        else{
            //unglint
            for (size_t i = 0; i < value.size(); i++){
                value(i) += sigma1 * noise;
            }
        }     
        // return current glint index
        return (glintIdx);
    }
};


} // namespace MyFusion

#endif