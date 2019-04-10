//
//  datapoint.cpp
//  SensorFusion
//
//  Created by 邵国亮 on 9/4/19.
//

#include "datapoint.hpp"

DataPoint::DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw){
    
    this->timestamp_ = timestamp;
    this->data_type_ = data_type;
    this->raw_ = raw;
    this->initialized_ = true;
}

VectorXd DataPoint::get() const{
    return this->raw_;
}

VectorXd DataPoint::get_state() const{
    VectorXd cart_v(4);
    
    if(this->data_type_ == DataPointType::LIDAR){
        
        double px = this->raw_(0);
        double py = this->raw_(1);
        cart_v << px, py, 0., 0.;
        
    } else if(this->data_type_ == DataPointType::RADAR){
        
        cart_v = convert_polar_to_cartesian(this->raw_);
        
    } else if(this->data_type_ == DataPointType::STATE){
        
        cart_v = this->raw_;
        
    }
    
    return cart_v;
}

long long DataPoint::get_timestamp() const{
    return this->timestamp_;
}

DataPointType DataPoint::get_type() const{
    return this->data_type_;
}


void DataPoint::set(const long long timestamp, const DataPointType data_type, const VectorXd raw){
    this->timestamp_ = timestamp;
    this->data_type_ = data_type;
    this->raw_ = raw;
    this->initialized_ = true;
}
