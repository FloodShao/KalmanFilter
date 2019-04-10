//
//  datapoint.hpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#ifndef datapoint_hpp
#define datapoint_hpp

#include <stdlib.h>
#include <iostream>
#include "Eigen/Dense"
#include "tools.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class DataPointType{
    LIDAR, RADAR, STATE
};

class DataPoint{
    
private:
    long long timestamp_;
    bool initialized_ = false;
    DataPointType data_type_;
    VectorXd raw_;
    
public:
    DataPoint(){}
    DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw);
    virtual ~DataPoint(){}
    
    void set(const long long timestamp, const DataPointType data_type, const VectorXd raw);
    VectorXd get() const;
    VectorXd get_state() const;
    long long get_timestamp() const;
    DataPointType get_type() const;
    
};
#endif /* datapoint_hpp */
