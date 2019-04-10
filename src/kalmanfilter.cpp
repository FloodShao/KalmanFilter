//
//  kalmanfilter.cpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#include "kalmanfilter.hpp"
#include "tools.hpp"
#include <math.h>

void ExtendedKalmanFilter::Init(const int &dim, const VectorXd &x_in, const MatrixXd &P_in, const MatrixXd &F_in, const MatrixXd &Q_in) {
    
    this->x_ = x_in;
    this->P_ = P_in;
    this->F_ = F_in;
    this->Q_ = Q_in;
}

void ExtendedKalmanFilter::Predict(){
    /*
     By using the CV model, the prediction process is linear
     */
    this->x_ = this->F_ * this->x_;
    this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}


//void ExtendedKalmanFilter::UpdateEKF(VectorXd &z){
//    /*
//     input with Radar measurements (rho, phi, drho)
//     */
//    VectorXd z_pred = convert_polar_to_cartesian(x_); //this is the observation function H
//    MatrixXd Hj = calculate_Hj(x_); // this is the jacobian matrix of the observation function H, 3*4
//
//    MatrixXd Hj_T = Hj.transpose();
//    MatrixXd S = Hj * P_ * Hj_T + R_;
//    MatrixXd K = P_ * Hj_T * S.inverse();
//
//    x_ = x_ + K * (z - z_pred);
//    MatrixXd I = MatrixXd::Identity(dim_, dim_);
//    P_ = (I - K * Hj) * P_;
//
//}


void ExtendedKalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &H, const VectorXd &z_pred, const MatrixXd &R){
    
    //Update step
    MatrixXd H_T = H.transpose();
    MatrixXd S = H * this->P_ * H_T + R;
    MatrixXd K = this->P_ * H_T * S.inverse();
    
    this->x_ = this->x_ + K * (z - z_pred);
    MatrixXd I = MatrixXd::Identity(this->dim_, this->dim_);
    this->P_ = (I - K * H) * this->P_;
    
}

VectorXd ExtendedKalmanFilter::Get(){
    return this->x_;
}

void ExtendedKalmanFilter::updateF(const double dt){
    this->F_(0, 2) = dt;
    this->F_(1, 3) = dt;
}

void ExtendedKalmanFilter::UpdateR(const MatrixXd &R_in){
    this->R_ = R_in;
}

void ExtendedKalmanFilter::UpdateQ(const MatrixXd &Q_in){
    this->Q_ = Q_in;
}
