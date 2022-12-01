#ifndef __TEST_SLAM3D_G2O_H__
#define __TEST_SLAM3D_G2O_H__

#include <iostream>
#include <Eigen/Core>
#include<fstream>

#include"univariate_factor.h"
#include"bivariate_factor.h"
#include"variable_type.h"
#include"graph_optimize.h"
#include"utils.h"

class TestSlam3dG2o
{
public:
    void Run()
    {
        if (!LoadG2O("../dataset/torus3d_guess.g2o", &m_graph))
        // if (!LoadG2O("../dataset/test3d.g2o", &m_graph))
            std::cerr<<"Cant load g2o file"<<std::endl;

        // Fix the first variable.
        m_graph.GetVariables()[0]->fixed = true;

        Optimize();
    }
private:
    void Optimize()
    {
        DumpPoses("../log/original3d.txt", m_graph);
        GraphOptimize::Option option = GraphOptimize::Option();
        GraphOptimize graph_optimize = GraphOptimize(option);
        graph_optimize.OptimizeGN(&m_graph);
        DumpPoses("../log/optimized3d.txt", m_graph);
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
        std::map<int, Pose3D *> id_to_pose;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string data_type;
            ss >> data_type;
            if (data_type == "VERTEX_SE3:QUAT")
            {
                int id;
                Eigen::Vector3d point, rot;
                ss >> id >> point(0) >> point(1) >> point(2) >> rot(0) >> rot(1) >> rot(2);
                Pose3D *p = new Pose3D(point, rot);
                graph->AddVariable(p);
                id_to_pose[id] = p;
            }
            else if (data_type == "EDGE_SE3:QUAT")
            {
                int id_a, id_b;
                Eigen::Vector3d point = Eigen::Vector3d::Zero();
                double q_x, q_y, q_z, q_w;
                double i11, i12, i13, i14, i15, i16, i22, i23, i24, i25, i26, i33, i34, i35, i36, i44, i45, i46, i55, i56, i66;
                ss >> id_a >> id_b >> point(0) >> point(1) >> point(2) 
                        >> q_x >> q_y >> q_z >> q_w >> i11 >> i12 >> i13 >> i14 >> i15 >> i16 
                        >> i22 >> i23 >> i24 >> i25 >> i26 >> i33 >> i34 >> i35 >> i36 >> i44 >> i45 >> i46 >> i55 >> i56 >> i66;
                Eigen::Quaterniond quaternion(q_w, q_x, q_y, q_z);
                Pose3D measurement(point, quaternion);

                Eigen::Matrix<double, 6, 6> info_mtrx = Eigen::Matrix<double, 6, 6>::Zero();
                info_mtrx<< i11, i12, i13, i14, i15, i16,
                            i12, i22, i23, i24, i25, i26,
                            i13, i23, i33, i34, i35, i36,
                            i14, i24, i34, i44, i45, i46,
                            i15, i25, i35, i45, i55, i56,
                            i16, i26, i36, i46, i56, i66;
                GRAPH_ASSERT(id_to_pose.count(id_a) != 0);
                GRAPH_ASSERT(id_to_pose.count(id_b) != 0);
                graph->AddFactor(new RotVec2RotVecFactor(id_to_pose[id_a], id_to_pose[id_b], measurement, info_mtrx.llt().matrixL()));
            }
            else
            {
                GRAPH_LOG("Unhandled type: %s", data_type.c_str());
                return false;
            }
        }
        return true;
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
            Pose3D *p = static_cast<Pose3D *>(variables[i]);
            file << i << " " << p->Point()(0) << " " << p->Point()(1) << " " << p->Point()(2)
                 << " " << p->Rot().Rot()(0) << " " << p->Rot().Rot()(1) << " " << p->Rot().Rot()(2) << std::endl;
        }
    }

    double NormalizeAngle(double theta_rad) const
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }

    Eigen::Vector3d Quaternion2Angle(const Eigen::Quaterniond &in_q)
    {
        Eigen::Vector3d out_angle;
        Eigen::Quaterniond q = in_q.normalized();
        if (q.w() < 0)
            q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
        double x = q.x();
        double y = q.y();
        double z = q.z();
        double norm_im = sqrt(x * x + y * y + z * z);
        if (norm_im < 1e-7)
        {
            out_angle(0) = 2 * x;
            out_angle(1) = 2 * y;
            out_angle(2) = 2 * z;
        }
        else
        {
            double th = 2 * atan2(norm_im, q.w());
            out_angle(0) = x / norm_im * th;
            out_angle(1) = y / norm_im * th;
            out_angle(2) = z / norm_im * th;
        }

        return out_angle;
    }

private:
    FactorGraph m_graph;
};

#endif // __TEST_SLAM3D_G2O_H__