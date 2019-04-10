//
//  kalmanfilter.hpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#ifndef kalmanfilter_hpp
#define kalmanfilter_hpp

#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class ExtendedKalmanFilter{
    
private:
    int dim_ = 4; // dimension of the state
    VectorXd x_; //estimated state [px, py, vx, vy]
    MatrixXd P_; //state covariance
    MatrixXd F_; //motion model
    MatrixXd Q_; //process covariance
    MatrixXd R_; //measurement covariance
    
public:
    ExtendedKalmanFilter(){}
    virtual ~ExtendedKalmanFilter(){}
    
    /**
     * initialize the kalman filter, the observation is non-linear
     * @param dim  State dimension
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix, using CV model
     * @param Q_in Process covariance matrix
     */
    void Init(const int &dim, const VectorXd &x_in, const MatrixXd &P_in, const MatrixXd &F_in, const MatrixXd &Q_in);
    
    /**
     * predict the k-th state with k-1 state
     * @param none
     */
    void Predict();
    
    /**
     * update the k-th state with measurements
     * @param z Measurements at k-th state
     */
    void UpdateEKF(const VectorXd &z, const MatrixXd &H, const VectorXd &z_pred, const MatrixXd &R);
    
    
    /**
     * get the current state
     * @param none
     */
    VectorXd Get();
    
    /**
     * update F_ with dt
     * @param
     * @param
     */
    void updateF(const double dt);
    
    void UpdateR(const MatrixXd &R_in);
    
    void UpdateQ(const MatrixXd &Q_in);
};

#endif /* kalmanfilter_hpp */
