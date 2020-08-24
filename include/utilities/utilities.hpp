#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <cmath> 
#include <Eigen/Dense>

class AttUtility{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    
    /**
     * @brief return euler angle in [Yaw(z), Pitch(Y), Roll(X)], sequence Z-Y-X
     * 
     * @param R 
     * @return Eigen::Vector3d 
     */
    static Eigen::Vector3d R2Euler(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    /**
     * @brief 
     * 
     * @param R 
     * @return Eigen::Vector3d 
     */
    // static Eigen::Vector3d R2Euler(const Eigen::Matrix3d &R){
    //     Eigen::Vector3d pyr;

    //     if(R(2,0) > -0.99998f && R(2,0) < 0.99998f){
    //         pyr(0) = atan2(R(1,0), R(0,0));
    //         pyr(1) = asin(-R(2,0));
    //         pyr(2) = atan2(R(2,1), R(2,2));
    //     }
    //     else
    //     {
    //         R(2,0) > 0.0f ? pyr(1) = -M_PI_2 : pyr(1) = M_PI_2;
    //         pyr(0) = asin(-R(0,1));
    //         pyr(2) = 0.;
    //     }

    //     return pyr;
    // }

    /**
     * @brief Get the rotation matrix Cge(e->g) 
     * 
     * @param lat : latitude 
     * @param lon : longitude
     * @return Eigen::Matrix3d 
     */
    static Eigen::Matrix3d getCge(double lat, double lon){
        Eigen::AngleAxisd rvec0(lat, Eigen::Vector3d(0., 0., 1.));
        Eigen::AngleAxisd rvec1(M_PI_2, Eigen::Vector3d(0., 1., 0.));
        Eigen::AngleAxisd rvec2((M_PI_2 - lon), Eigen::Vector3d(0., 0., 1.));

        Eigen::Quaterniond qgb(rvec0 * rvec1 * rvec2);
        return qgb.toRotationMatrix();
    }

    /**
     * @brief Get the rotation matrix Cge(e->g) 
     * 
     * @param lat : latitude 
     * @param lon : longitude
     * @return Eigen::Matrix3d 
     */
    static Eigen::Matrix3d getCge(Eigen::Vector3d &tnb){
        double scale = sqrt(tnb.x() * tnb.x() + tnb.y() * tnb.y());
        double lat = atan2(tnb.z(), scale);
        double lon = atan2(tnb.y(), tnb.x());

        Eigen::AngleAxisd rvec0(lat, Eigen::Vector3d(0., 0., 1.));
        Eigen::AngleAxisd rvec1(M_PI_2, Eigen::Vector3d(0., 1., 0.));
        Eigen::AngleAxisd rvec2((M_PI_2 - lon), Eigen::Vector3d(0., 0., 1.));

        Eigen::Quaterniond qgb(rvec0 * rvec1 * rvec2);
        return qgb.toRotationMatrix();
    }


};

#endif