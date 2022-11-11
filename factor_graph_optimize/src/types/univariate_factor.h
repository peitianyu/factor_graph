#ifndef __UNIVARIATE_FACTOR_H__
#define __UNIVARIATE_FACTOR_H__

#include"factor.h"
#include"variable_type.h"
#include<Eigen/Core>

class PriorFactor : public Factor
{
public:
    PriorFactor(Point2d *p, const Eigen::Vector2d &measurement) : m_measurement(measurement)
    {
        AddVariable(p);
    }

    virtual int Dim()const { return 2; }

    virtual Eigen::VectorXd Error()const override
    {
        return (static_cast<Point2d*>(this->VariableAt(0))->Position() - m_measurement);
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd& e1, const Eigen::VectorXd& e2)const override
    {
        return (e1 - e2);
    }

    virtual Eigen::MatrixXd Jacobian(int idx) const
    {
        GRAPH_ASSERT(m_num_variables > 0 && idx < m_num_variables);
        return ComputeNumericalJacobian(m_variables[idx]);
    }
private:
    Eigen::Vector2d m_measurement;
};

class GPSFactor : public Factor
{
public:
    GPSFactor(Point2d *p, const Eigen::Vector2d &measurement) : m_measurement(measurement)
    {
        AddVariable(p);
    }

    virtual int Dim()const { return 2; }

    virtual Eigen::VectorXd Error()const override
    {
        return (static_cast<Point2d*>(this->VariableAt(0))->Position() - m_measurement);
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd& e1, const Eigen::VectorXd& e2)const override
    {
        return (e1 - e2);
    }

    virtual Eigen::MatrixXd Jacobian(int idx) const
    {
        GRAPH_ASSERT(m_num_variables > 0 && idx < m_num_variables);
        Eigen::MatrixXd J(3, 2);
        J.setZero();
        J.block(0, 0, 2, 2) = ComputeNumericalJacobian(m_variables[idx]);
        return J;
    }

private:
    Eigen::Vector2d m_measurement;
};

#endif // __UNIVARIATE_FACTOR_H__