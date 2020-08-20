add_library(myFusion SHARED
            simulator/sensors/imu_base.cc
            simulator/sensors/imu_li.cc
            simulator/sensors/imu_g.cc
            simulator/sensors/imu_mcmf.cc
            simulator/sensors/imu_mci.cc
            )