//
//  tools.cpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#include "tools.hpp"

VectorXd convert_polar_to_cartesian(const VectorXd& polar_v){
    VectorXd cart_v(4);
    
    const double rho = polar_v(0);
    const double phi = polar_v(1);
    const double drho = polar_v(2);
    
    const double px = rho * cos(phi);
    const double py = rho * sin(phi);
    const double vx = drho * cos(phi);
    const double vy = drho * sin(phi);
    
    cart_v << px, py, vx, vy;
    
    return cart_v;
    
}
VectorXd convert_cartesian_to_polar(const VectorXd& cart_v){
    
    VectorXd polar_v(3);
    const double THRESHOLD = 0.0001;
    
    const double px = cart_v(0);
    const double py = cart_v(1);
    const double vx = cart_v(2);
    const double vy = cart_v(3);
    
    const double rho = sqrt(px*px + py*py);
    const double phi = atan2(py, px);
    const double drho = (rho > THRESHOLD) ? (py * vy + px * vx)/rho : 0.0; //the radial velocity
    
    polar_v << rho, phi, drho;
    return polar_v;
    
}

MatrixXd calculate_Hj(const VectorXd& cart_v){
    
    const double THRESHOLD = 0.0001;
    MatrixXd Hj = MatrixXd::Zero(3, 4);
    
    const double px = cart_v(0);
    const double py = cart_v(1);
    const double vx = cart_v(2);
    const double vy = cart_v(3);
    
    const double r_sq = (px*px + py*py);
    const double r = sqrt(r_sq);
    const double  r_cu = r_sq * r;
    
    if(r >= THRESHOLD){
        const double r11 = px / r;
        const double r12 = py / r;
        const double r21 = -py / r_sq;
        const double r22 = px / r_sq;
        const double r31 = py * (py * vx - px * vy) / r_cu;
        const double r32 = px * (px * vy - py * vx) / r_cu;
        
        Hj << r11, r12, 0., 0.,
             r21, r22, 0., 0.,
             r31, r32, r11, r12;
    }
    
    return Hj;
}


VectorXd calculate_rmse(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truths){
    
    VectorXd rmse(4);
    rmse << 0., 0., 0., 0.;
    
    for(int i = 0; i<estimations.size(); i++){
        VectorXd diff = estimations[i] - ground_truths[i];
        diff = diff.array() * diff.array();
        rmse += diff;
    }
    
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}

