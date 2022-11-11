#ifndef __TEST_POINT2POSE_H__
#define __TEST_POINT2POSE_H__

#include <iostream>
#include <Eigen/Core>

#include"univariate_factor.h"
#include"bivariate_factor.h"
#include"variable_type.h"
#include"graph_optimize.h"

class TestPoint2PoseSlam
{
public:
    void Run()
    {
        AddVariableAndFactor();

        // m_graph.GetVariables()[0]->fixed = true;

        Optimize();
    }
private:
    void AddVariableAndFactor()
    {
        Point2d *l1 = new Point2d(Eigen::Vector2d(0.45, 0.45));
        Point2d *l2 = new Point2d(Eigen::Vector2d(0.2, 0.2));
        Pose2d *x1 = new Pose2d(Eigen::Vector3d(0.0, 0.0, 0.0));
        Pose2d *x2 = new Pose2d(Eigen::Vector3d(1.0, 0.0, 0.0));
        Pose2d *x3 = new Pose2d(Eigen::Vector3d(1.0, 1.0, 0.0));
        Pose2d *x4 = new Pose2d(Eigen::Vector3d(0.0, 1.0, 0.0));
        m_graph.AddVariable(l1);
        m_graph.AddVariable(l2);
        m_graph.AddVariable(x1);
        m_graph.AddVariable(x2);
        m_graph.AddVariable(x3);
        m_graph.AddVariable(x4);
        
        // 注意这里的fl为先验Factor已经被固定住了
        GPSFactor *f1 = new GPSFactor(l1, Eigen::Vector2d(0.0, 0.0));
        GPSFactor *f2 = new GPSFactor(l2, Eigen::Vector2d(0.5, 0.5));
        m_graph.AddFactor(f1);
        m_graph.AddFactor(f2);

        Pose2PoseFactor *f12 = new Pose2PoseFactor(x1, x2, 0.1, 0.05, 0.01, Eigen::Matrix3d::Zero());
        Pose2PoseFactor *f23 = new Pose2PoseFactor(x2, x3, 0.2, 0.1, 0.01, Eigen::Matrix3d::Zero());
        Pose2PoseFactor *f34 = new Pose2PoseFactor(x3, x4, 0.1, 0.15, 0.01, Eigen::Matrix3d::Zero());
        Pose2PoseFactor *f14 = new Pose2PoseFactor(x1, x4, 0.01, 0.01, 0.01, Eigen::Matrix3d::Zero());
        m_graph.AddFactor(f12);
        m_graph.AddFactor(f23);
        m_graph.AddFactor(f34);
        m_graph.AddFactor(f14);

        Point2PoseFactor *fl11 = new Point2PoseFactor(l1, x1, Eigen::Vector2d(0.4, 0.6));
        Point2PoseFactor *fl12 = new Point2PoseFactor(l1, x2, Eigen::Vector2d(0.6, 0.4));
        Point2PoseFactor *fl21 = new Point2PoseFactor(l2, x1, Eigen::Vector2d(0.4, 0.6));
        Point2PoseFactor *fl22 = new Point2PoseFactor(l2, x2, Eigen::Vector2d(0.6, 0.4));
        m_graph.AddFactor(fl11);
        m_graph.AddFactor(fl12);
        m_graph.AddFactor(fl21);
        m_graph.AddFactor(fl22);
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

    void Printposes(const FactorGraph &graph)
    {
        const std::vector<Variable *> &variables = graph.GetVariables();
        for(Variable *var:variables)
            var->Print();
    }

private:
    FactorGraph m_graph;
};


#endif // __TEST_POINT2POSE_H__