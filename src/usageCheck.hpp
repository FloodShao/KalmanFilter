//
//  usageCheck.hpp
//  SensorFusion
//
//  Created by 邵国亮 on 10/4/19.
//

#ifndef usageCheck_hpp
#define usageCheck_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iomanip>
#include "datapoint.hpp"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_arguments(int argc, char* argv[]);
void check_files(std::ifstream &in_file, std::string &in_names, std::ofstream &out_file, std::string &out_name);
void print_EKF_data(const VectorXd &RMSE, const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truths, const std::vector<DataPoint> &all_sensor_data);

#endif /* usageCheck_hpp */
