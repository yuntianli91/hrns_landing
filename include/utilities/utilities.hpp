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


};

#endif