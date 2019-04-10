//
//  fusionEKF.cpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#include "fusionEKF.hpp"

FusionEKF::FusionEKF(){
    this->initialized = false;
    this->P_ = MatrixXd(this->x_dim_, this->x_dim_);
    this->F_ = MatrixXd::Identity(this->x_dim_, this->x_dim_); // use linear CV model
    this->Q_ = MatrixXd::Zero(this->x_dim_, this->x_dim_); //initialize the Q
    
    this->radar_R_ = MatrixXd(this->R_dim_, this->R_dim_);
    this->lidar_R_ = MatrixXd(this->L_dim_, this->L_dim_);
    this->lidar_H_ = MatrixXd(this->L_dim_, this->x_dim_);
    
    this->lidar_R_ << 0.00225, 0.,
                      0., 0.00225;
    
    this->radar_R_ << 0.009, 0., 0.,
                      0., 0.09, 0.,
                      0., 0., 0.009;
    
    this->lidar_H_ << 1., 0., 0., 0.,
                      0., 1., 0., 0.;
    
    this->P_ << 1., 0., 0., 0.,
                0., 1., 0., 0.,
                0., 0., 1000., 0.,
                0., 0., 0., 1000.;
}

void FusionEKF::UpdateQ(const double dt){
    
    const double dt2 = dt*dt;
    const double dt3 = dt*dt2;
    const double dt4 = dt*dt3;
    
    const double r11 = 0.25*dt4*sigma_v2;
    const double r13 = 0.5*dt3*sigma_v2;
    const double r33 = dt2*sigma_v2;
    
    this->Q_ << r11, 0., r13, 0.,
                0., r11, 0., r13,
                r13, 0., r33, 0.,
                0., r13, 0., r33;
    
    // the results show correlating x dimension and y dimension is not a good idea
//    this->Q_ << r11, r11, r13, r13,
//                r11, r11, r13, r13,
//                r13, r13, r33, r33,
//                r13, r13, r33, r33;

    
    this->EKF_.UpdateQ(this->Q_);
}

void FusionEKF::Start(const DataPoint &data_point){
    
    this->timestamp = data_point.get_timestamp();
    VectorXd x = data_point.get_state();
    this->EKF_.Init(this->x_dim_, x, this->P_, this->F_, this->Q_); //here we do not update R, because different measurement dim
    this->initialized = true;
}

void FusionEKF::ProcessMeasurement(const DataPoint &data_point){
    
    const double dt = (data_point.get_timestamp() - this->timestamp) / 1.e6;
    this->timestamp = data_point.get_timestamp();
    
    /*
     Predict Session
     */
    this->UpdateQ(dt); //update Q in EKF also
    this->EKF_.updateF(dt);
    this->EKF_.Predict(); //Mind that here EKF_.R_ is not updated
    
    /*
     Update Session
     */
    const VectorXd z = data_point.get(); //datapoint input
    const VectorXd x = this->EKF_.Get(); //the state is updated when initialized
    
    MatrixXd H;
    VectorXd z_pred;
    MatrixXd R;
    
    if(data_point.get_type() == DataPointType::RADAR){
        H = calculate_Hj(x); // here initially we use the estimated value of the predict position
        //H = calculate_Hj(convert_polar_to_cartesian(z)); //here we use the measurement retrieval position
        z_pred = convert_cartesian_to_polar(x); //it is a nonlinear observation function
        R = this->radar_R_;
    } else if(data_point.get_type() == DataPointType::LIDAR){
        H = this->lidar_H_;
        z_pred = H * x;
        R = this->lidar_R_;
    }
    
    this->EKF_.UpdateEKF(z, H, z_pred, R);
}


void FusionEKF::Process(const DataPoint &data_point){
    
    this->initialized?  this->ProcessMeasurement(data_point) : this->Start(data_point);
}

VectorXd FusionEKF::Get(){
    
    return this->EKF_.Get();
}


