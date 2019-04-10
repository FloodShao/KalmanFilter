//
//  fusionEKF.hpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#ifndef fusionEKF_hpp
#define fusionEKF_hpp

#include <stdio.h>
#include "Eigen/Dense"
#include "kalmanfilter.hpp"
#include "datapoint.hpp"
#include "tools.hpp"


using Eigen::MatrixXd;
using Eigen::VectorXd;

class FusionEKF{
    
private:
    const int x_dim_ = 4;
    const int L_dim_ = 2; //lidar measurement [x, y]
    const int R_dim_ = 3; //radar measurement [rho, phi, drho]
    
    bool initialized = false;
    long long timestamp;
    const double sigma_v2 = 5.0;
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd F_;
    MatrixXd radar_R_;
    MatrixXd lidar_R_;
    MatrixXd lidar_H_;
    ExtendedKalmanFilter EKF_;
    
public:
    FusionEKF();
    virtual ~FusionEKF(){}
    
    void Start(const DataPoint &data_point);
    
    void UpdateQ(const double dt);
    
    void ProcessMeasurement(const DataPoint &data_point);
    
    void Process(const DataPoint &data_point);
    
    VectorXd Get();
    
};


#endif /* fusionEKF_hpp */
