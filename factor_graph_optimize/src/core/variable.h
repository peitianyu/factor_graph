#ifndef __VARIABLE_H__
#define __VARIABLE_H__

#include<Eigen/Core>

class Variable
{
public:
    virtual size_t Dim() const = 0;

    virtual void Plus(const Eigen::VectorXd &delta) = 0;

    virtual void Print() const = 0;

    bool fixed = false;
};

#endif // __VARIABLE_H__