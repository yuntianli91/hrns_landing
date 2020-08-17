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
    // static Eigen::Vector3d R2Euler(Eigen::Matrix3d &R){
    //     double C11(R(0, 0)), C12(R(0, 1)), C13(R(0, 2));
    //     double C21(R(1, 0)), C22(R(1, 1)), C23(R(1, 2));
    //     double C31(R(2, 0)), C32(R(2, 1)), C33(R(2, 2));

    //     Eigen::Vector3d RYP;
    //     if(abs(C31 - 1.0) < 1e-5){
    //         RYP(1) = M_PI / 2.0;
    //     }
    //     else if(abs(C31 + 1.0) < 1e-5){
    //         RYP(1) = - M_PI / 2.0;
    //     }
    //     else{
    //         RYP(0) = atan2(C32, C33);
    //         RYP(1) = asin(-C31);
    //         RYP(2) = atan2(C21, C11);
    //     }

    //     return RYP;
    // }

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