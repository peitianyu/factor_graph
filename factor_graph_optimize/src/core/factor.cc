#include"factor.h"


int Factor::NumVariables() const 
{ 
    return m_num_variables; 
}

void Factor::AddVariable(Variable *v)
{
    GRAPH_ASSERT(m_num_variables < kMaxVariables);
    m_variables[m_num_variables] = v;
    ++m_num_variables;
}

Variable *Factor::VariableAt(int idx) const
{
    GRAPH_ASSERT(m_num_variables > 0 && idx < m_num_variables);
    return m_variables[idx];
}

// Jacobian wrt to the variable at idx. Defaults
// to computing the jacobian numerically.
Eigen::MatrixXd Factor::Jacobian(int idx) const
{
    GRAPH_ASSERT(m_num_variables > 0 && idx < m_num_variables);
    return ComputeNumericalJacobian(m_variables[idx]);
}

void Factor::SetSqrtInfo(const Eigen::MatrixXd &sqrt_info)
{
    m_sqrt_info = sqrt_info;
    m_sqrt_info_set = true;
}

Eigen::VectorXd Factor::WeightedError() const
{
    Eigen::VectorXd error = Error();

    if (m_sqrt_info_set)
    {
        GRAPH_ASSERT(m_sqrt_info.cols() == error.size());
        error = m_sqrt_info * error;
    }

    return error;
}

Eigen::MatrixXd Factor::WeightedJacobian(int idx) const
{
    Eigen::MatrixXd jacobian = Jacobian(idx);

    if (m_sqrt_info_set)
    {
        GRAPH_ASSERT(m_sqrt_info.cols() == jacobian.rows());
        jacobian = m_sqrt_info * jacobian;
    }

    return jacobian;
}

Eigen::MatrixXd Factor::ComputeNumericalJacobian(Variable * v) const
{
    constexpr double h = 1e-5;
    const int N = v->Dim();
    const int M = this->Dim();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(M, N);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd dy0 = this->Error();
    constexpr double k = 1.0 / (2.0 * h);
    for (int i = 0; i < N; ++i)
    {
    dx(i) = h;
    v->Plus(dx); // right
    const Eigen::VectorXd dy1 = this->SubtractError(this->Error(), dy0);
    dx(i) = -2.0 * h;
    v->Plus(dx); // left
    const Eigen::VectorXd dy2 = this->SubtractError(this->Error(), dy0);
    dx(i) = h;
    v->Plus(dx); // return to original state.
    dx(i) = 0.0;
    J.col(i) << (dy1 - dy2) * k;
    }
    return J;
}
