#ifndef __TEST_POINT2POINT_H__
#define __TEST_POINT2POINT_H__

#include <iostream>
#include <Eigen/Core>

#include"univariate_factor.h"
#include"bivariate_factor.h"
#include"variable_type.h"
#include"graph_optimize.h"

class TestPoint2PointSlam
{
public:
    void Run()
    {
        AddVariableAndFactor();

        Optimize();
    }
private:
    void AddVariableAndFactor()
    {
        Point2d *x1 = new Point2d(Eigen::Vector2d(0.0, 0.0));
        Point2d *x2 = new Point2d(Eigen::Vector2d(0.0, 0.0));
        Point2d *x3 = new Point2d(Eigen::Vector2d(0.0, 0.0));
        m_graph.AddVariable(x1);
        m_graph.AddVariable(x2);
        m_graph.AddVariable(x3);

        PriorFactor *f1 = new PriorFactor(x1, Eigen::Vector2d(0.0, 0.0));
        m_graph.AddFactor(f1);

        Point2PointFactor *f12 = new Point2PointFactor(x1, x2, Eigen::Vector2d(1.0, 1.0));
        m_graph.AddFactor(f12);
        Point2PointFactor *f23 = new Point2PointFactor(x2, x3, Eigen::Vector2d(1.0, 1.0));
        m_graph.AddFactor(f23);
        Point2PointFactor *f13 = new Point2PointFactor(x1, x3, Eigen::Vector2d(1.0, 1.0));
        m_graph.AddFactor(f13);
    }

    void Optimize()
    {
        std::cout << "Original poses:" << std::endl;
        Printposes(m_graph);
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);
        graph_optimize.OptimizeGN(&m_graph);
        std::cout << "Optimized poses:" << std::endl;
        Printposes(m_graph);
    }

    void Printposes(const FactorGraph& graph)
    {
        const std::vector<Variable*>& variables = graph.GetVariables();
        for (int i = 0, count = variables.size(); i < count; ++i)
        {
            Point2d* p = static_cast<Point2d*>(variables[i]); // We know our variables are of type Point.
            std::cout << i << ": " << p->Position().x() << " " << p->Position().y() << std::endl;
        }
    }
private:
    FactorGraph m_graph;
};


#endif // __TEST_POINT2POINT_H__