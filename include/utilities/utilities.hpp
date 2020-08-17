#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <cmath> 
#include <Eigen/Dense>

class AttUtility{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @brief Cnb(Z-Y-X)
     * 
     * @param R : rotation matrix
     * @return Eigen::Vector3d : Euler angle [Roll, Yaw, Pitch] in rad.
     */
    static Eigen::Vector3d R2Euler(Eigen::Matrix3d &R){
        double C11(R(0, 0)), C12(R(0, 1)), C13(R(0, 2));
        double C21(R(1, 0)), C22(R(1, 1)), C23(R(1, 2));
        double C31(R(2, 0)), C32(R(2, 1)), C33(R(2, 2));

        Eigen::Vector3d RYP;
        if(abs(C31 - 1.0) < 1e-5){
            RYP(1) = M_PI / 2.0;
        }
        else if(abs(C31 + 1.0) < 1e-5){
            RYP(1) = - M_PI / 2.0;
        }
        else{
            RYP(0) = atan2(C32, C33);
            RYP(1) = asin(-C31);
            RYP(2) = atan2(C21, C11);
        }

        return RYP;
    }


};

#endif