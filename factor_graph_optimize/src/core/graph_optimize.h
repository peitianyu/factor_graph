#ifndef __GRAPH_OPTIMIZE_H__
#define __GRAPH_OPTIMIZE_H__

#include <iostream>
#include<map>
#include<factor_graph.h>
#include <Eigen/Sparse>
#include "utils.h"
#include "sparsity_pattern.h"

class GraphOptimize
{
public:
    struct Option
    {
        int max_iterations = 3000;        // Maximum number of iterations.
        double relative_error_th = 1e-5; // Maximum relative error decrease.
        double absolute_error_th = 1e-5; // Maximum absolute error decrease.
    };

    GraphOptimize(Option option);

    bool OptimizeGN(FactorGraph *graph);

private:
    bool Iterate(FactorGraph *graph, SparsityPattern *pattern);

    void LinearizeSingleFactor(Factor *factor, SparsityPattern *pattern);

    bool ContinueIteratingCheck(int iter_num, double current_error, double new_error, bool *converged);

    double ComputeErrorNormSquared(const FactorGraph &graph);
private:
    Option m_option;
    SparsityPatternBuilder  m_sparsity_pattern_builder;
};

#endif // __GRAPH_OPTIMIZE_H__