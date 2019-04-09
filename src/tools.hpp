//
//  tools.hpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#ifndef tools_hpp
#define tools_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

VectorXd convert_polar_to_cartesian(const VectorXd& polar_v);
VectorXd convert_cartesian_to_polar(const VectorXd& cart_v);
MatrixXd calculate_Hj(const VectorXd& v);
VectorXd calculate_rmse(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truths);

#endif /* tools_hpp */
