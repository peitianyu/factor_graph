#include"sparsity_pattern.h"

void SparsityPatternBuilder::ConstructSparsityPattern(const FactorGraph &graph, SparsityPattern *pattern)
{
    // Construct the sparsity pattern.
    const std::vector<Variable *> &variables = graph.GetVariables();
    int total_variables_dim = 0;
    std::map<Variable *, VariableOrdering> variable_lookup_table;
    for (int i = 0, var_idx = 0, count = variables.size(); i < count; ++i)
    {
        Variable *var = variables[i];
        if (!var->fixed)
        {
            const int v_dim = var->Dim();
            total_variables_dim += v_dim;
            variable_lookup_table[var] = VariableOrdering().SetDim(v_dim).SetIdx(var_idx);
            var_idx += v_dim;
        }
    }
    pattern->total_variables_dim = total_variables_dim;
    pattern->variable_lookup_table.swap(variable_lookup_table);

    const std::vector<Factor *> &factors = graph.GetFactors();
    int total_factors_dim = 0;
    for (int i = 0, count = factors.size(); i < count; ++i)
    {
        const int f_dim = factors[i]->Dim();
        total_factors_dim += f_dim;
    }
    pattern->total_factors_dim = total_factors_dim;

    pattern->H = Eigen::SparseMatrix<double>(total_variables_dim, total_variables_dim);
    pattern->b = Eigen::VectorXd::Zero(total_variables_dim);
}

void SparsityPatternBuilder::UpdateSparsityPattern(FactorGraph *graph, SparsityPattern *pattern, const Eigen::VectorXd &dx)
{
    std::vector<Variable *> &variables = graph->GetVariables(); // 更新变量
    for (int i = 0, d = 0, count = variables.size(); i < count; ++i)
    {
        Variable *var = variables[i];
        if (!var->fixed)
        {
            const int vd = pattern->variable_lookup_table[var].Dim();
            var->Plus(dx.segment(d, vd));
            d += vd;
        }
    }
}