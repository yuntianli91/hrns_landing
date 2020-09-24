add_library(myFusion SHARED
            simulator/sensors/imu_base.cc
            simulator/sensors/imu_li.cc
            simulator/sensors/imu_g.cc
            simulator/sensors/imu_mcmf.cc
            simulator/sensors/imu_mci.cc
            simulator/sensors/cns.cc
            simulator/sensors/virns.cc
            simulator/sensors/cmns.cc
            simulator/sensors/altimeter.cc
            simulator/sensorSimulator.cc
            backend/backParam.cc
            backend/spkf.cc
            backend/cnsUKF.cc
            backend/cnsHCKF.cc
            backend/estimator.cc
            backend/sckf/sckf.cc
            # backend/sckf/scekf.cc
            backend/sckf/scspkf.cc
            backend/sckf/pdSCSPKF.cc
            utilities/io_function.cc
            )