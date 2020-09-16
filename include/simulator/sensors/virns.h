#ifndef VIRNS_H_
#define VIRNS_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"

namespace MyFusion{

struct VirnsData{
    double timeStamp_;
    Vec3d dPos_;
    Vec3d pos_;
};

class VIRNS{
public:
    VIRNS(double bias, double sigma):bias_(bias), sigma_(sigma){
        flagInit_ = true;
    }
    ~VIRNS(){}


    void setParams(double bias, double sigma);
    /**
     * @brief Get the Relative Measurement dp
     * 
     * @param currMotion 
     * @return Vec3d 
     */
    VirnsData getRelativeMeasurement(ImuMotionData currMotion);

    /**
     * @brief convert geo coordinate to mcmf coordinate with sphere model
     * 
     * @param geo : [lat, alt, lon] 
     * @return Vec3d : [Px, Py, Pz]
     */
    Vec3d geo2mcmf(Vec3d geo);

    /**
     * @brief convert mcmf coordinate to geo coordinate with sphere model
     * 
     * @param mcmf : [Px, Py, Pz] 
     * @return Vec3d : [lat, alt, lon]
     */
    Vec3d mcmf2geo(Vec3d mcmf);
    
    // ImuMotionData lastMotion_;
    Vec3d lastP_, curP_; // last position and current position
    double bias_, sigma_; // noise bias and sigma
    bool flagInit_ = false;
    bool flagFirst = true;
};

}

#endif