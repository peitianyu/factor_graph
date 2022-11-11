#ifndef __FACTOR_H__
#define __FACTOR_H__

#include<vector>
#include<memory>
#include<Eigen/Core>
#include"variable.h"
#include "utils.h"

class Factor
{
public:
    static constexpr int kMaxVariables = 2;
    int NumVariables() const;

    void AddVariable(Variable *v);

    Variable *VariableAt(int idx) const;

    // Dimensionality of the error.
    virtual int Dim() const = 0;

    virtual Eigen::VectorXd Error() const = 0;

    // Returns e1 - e2.
    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const = 0;

    // Jacobian wrt to the variable at idx. Defaults
    // to computing the jacobian numerically.
    virtual Eigen::MatrixXd Jacobian(int idx) const;
    
    void SetSqrtInfo(const Eigen::MatrixXd &sqrt_info);
    Eigen::VectorXd WeightedError() const;
    Eigen::MatrixXd WeightedJacobian(int idx) const;

protected:
    Eigen::MatrixXd ComputeNumericalJacobian(Variable * v) const;
    std::array<Variable *, kMaxVariables> m_variables;
    int m_num_variables = 0;
    Eigen::MatrixXd m_sqrt_info;
    bool m_sqrt_info_set = false;
};

#endif // __FACTOR_H__