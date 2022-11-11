#ifndef __FACTOR_GRAPH_H__
#define __FACTOR_GRAPH_H__

#include "factor.h"
#include "variable.h"
class FactorGraph
{
public:
    FactorGraph() = default;
    ~FactorGraph()
    {
        for (Factor* f : m_factors){delete f;}
        for (Variable* v : m_variables){delete v;}
    }
    
    void AddFactor(Factor* f) { m_factors.push_back(f); }
    void AddVariable(Variable* v) { m_variables.push_back(v); }

    const std::vector<Variable*>& GetVariables() const{ return m_variables; }
    std::vector<Variable*>& GetVariables() { return m_variables; }

    std::vector<Factor*>& GetFactors() { return m_factors; }
    const std::vector<Factor*>& GetFactors() const{return m_factors; }

private:
    std::vector<Variable*> m_variables;
    std::vector<Factor*> m_factors;
};

#endif // __FACTOR_GRAPH_H__