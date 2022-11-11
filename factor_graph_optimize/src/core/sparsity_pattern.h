#ifndef __SPARSITY_PATTERN_H__
#define __SPARSITY_PATTERN_H__

#include <factor_graph.h>
#include <Eigen/Sparse>

struct VariableOrdering
{
public:
    inline VariableOrdering &SetDim(int val)
    {
        m_dim = val;
        return *this;
    }
    inline VariableOrdering &SetIdx(int val)
    {
        m_idx = val;
        return *this;
    }
    inline int Dim() const { return m_dim; }
    inline int Idx() const { return m_idx; }

private:
    int m_dim;
    int m_idx; // The index of this variable in the H matrix.
};

struct SparsityPattern
{
    int total_variables_dim = 0;
    int total_factors_dim = 0;
    std::map<Variable *, VariableOrdering> variable_lookup_table;
    inline const VariableOrdering &VariableLookup(Variable *var)
    {
        GRAPH_ASSERT(variable_lookup_table.count(var) != 0);
        return variable_lookup_table[var];
    }
    typedef Eigen::Triplet<double> T;
    std::vector<T> tripletList;

    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd b;
};

class SparsityPatternBuilder
{
public:
    void ConstructSparsityPattern(const FactorGraph &graph, SparsityPattern *pattern);

    void UpdateSparsityPattern(FactorGraph *graph, SparsityPattern *pattern, const Eigen::VectorXd& dx);
};

#endif // __SPARSITY_PATTERN_H__