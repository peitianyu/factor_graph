#ifndef __VARIABLE_H__
#define __VARIABLE_H__

#include<Eigen/Core>
#include <iostream>

class Variable
{
public:
    virtual ~Variable()
    {
        // std::cerr<<"Never delete Variable"<<std::endl;
    }
    virtual size_t Dim() const = 0;

    virtual void Plus(const Eigen::VectorXd &delta) = 0;

    virtual void Print() const = 0;

    bool fixed = false;
};

#endif // __VARIABLE_H__