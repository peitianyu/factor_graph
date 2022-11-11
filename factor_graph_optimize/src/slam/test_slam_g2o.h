#ifndef __TEST_SLAM_G2O_H__
#define __TEST_SLAM_G2O_H__

#include <iostream>
#include <Eigen/Core>
#include<fstream>

#include"univariate_factor.h"
#include"bivariate_factor.h"
#include"variable_type.h"
#include"graph_optimize.h"
#include"utils.h"

class TestSlamG2o
{
public:
    void Run()
    {
        if (!LoadG2O("../dataset/killian.g2o", &m_graph))
            std::cerr<<"Cant load g2o file"<<std::endl;

        // Fix the first variable.
        m_graph.GetVariables()[0]->fixed = true;

        Optimize();
    }
private:
    void Optimize()
    {
        DumpPoses("../log/optimized.txt", m_graph);
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);
        graph_optimize.OptimizeGN(&m_graph);
        DumpPoses("../log/optimized.txt", m_graph);
    }

    bool LoadG2O(const std::string &filename, FactorGraph *graph)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            GRAPH_LOG("Failed to open file: %s", filename.c_str());
            return false;
        }

        std::string line;
        std::map<int, Pose2d *> id_to_pose;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string data_type;
            ss >> data_type;
            if (data_type == "VERTEX_SE2")
            {
                int id;
                double x, y, th;
                ss >> id >> x >> y >> th;
                Pose2d *p = new Pose2d(x, y, NormalizeAngle(th));
                graph->AddVariable(p);
                id_to_pose[id] = p;
            }
            else if (data_type == "EDGE_SE2")
            {
                int id_a, id_b;
                double dx, dy, d_yaw, i11, i12, i13, i22, i23, i33;
                ss >> id_a >> id_b >> dx >> dy >> d_yaw >> i11 >> i12 >> i13 >> i22 >> i23 >> i33;
                Eigen::Matrix3d info_mtrx = (Eigen::Matrix3d() << i11, i12, i13,
                                                                i12, i22, i23,
                                                                i13, i23, i33)
                                                                .finished();
                GRAPH_ASSERT(id_to_pose.count(id_a) != 0);
                GRAPH_ASSERT(id_to_pose.count(id_b) != 0);
                graph->AddFactor(new Pose2PoseFactor(id_to_pose[id_a], id_to_pose[id_b], dx, dy,
                                                    d_yaw, info_mtrx.llt().matrixL()));
            }
            else
            {
                GRAPH_LOG("Unhandled type: %s", data_type.c_str());
                return false;
            }
        }
    }

    void DumpPoses(const std::string &filename, const FactorGraph &graph)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            GRAPH_LOG("Failed to open file: %s", filename.c_str());
            return;
        }

        const std::vector<Variable *> &variables = graph.GetVariables();
        for (int i = 0, count = variables.size(); i < count; ++i)
        {
            Pose2d *p = static_cast<Pose2d *>(variables[i]);
            file << i << " " << p->x() << " " << p->y() << " " << p->yaw_rad() << std::endl;
        }
    }

    double NormalizeAngle(double theta_rad) const
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }

private:
    FactorGraph m_graph;
};

#endif // __TEST_SLAM_G2O_H__