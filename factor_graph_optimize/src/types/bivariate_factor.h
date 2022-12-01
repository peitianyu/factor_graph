#ifndef __BIVARIATE_FACTOR_H__
#define __BIVARIATE_FACTOR_H__

#include"factor.h"
#include"variable_type.h"
#include<Eigen/Core>
#include <Eigen/Geometry>

class Point2PointFactor : public Factor
{
public:
    Point2PointFactor(Point2d *p_a, Point2d *p_b, const Eigen::Vector2d &measurement) : m_measurement(measurement)
    {
        AddVariable(p_a);
        AddVariable(p_b);
    }

    virtual int Dim() const { return 2; }
    virtual Eigen::VectorXd Error() const override
    {
        const auto p1 = static_cast<Point2d *>(this->VariableAt(0))->Position();
        const auto p2 = static_cast<Point2d *>(this->VariableAt(1))->Position();
        return ((p2 - p1) - m_measurement);
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const override
    {
        return (e1 - e2);
    }

private:
    Eigen::Vector2d m_measurement;
};

class Point2PoseFactor : public Factor
{
public:
    Point2PoseFactor(Point2d *p_a, Pose2d *p_b, const Eigen::Vector2d &measurement) : m_measurement(measurement)
    {
        AddVariable(p_a);
        AddVariable(p_b);
    }

    virtual int Dim() const { return 2; }
    virtual Eigen::VectorXd Error() const override
    {
        const auto p1 = static_cast<Point2d *>(this->VariableAt(0))->Position();
        const auto p2 = static_cast<Pose2d *>(this->VariableAt(1))->Position();
        return ((p2 - p1) - m_measurement);
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const override
    {
        return (e1 - e2);
    }

    virtual Eigen::MatrixXd Jacobian(int idx) const override
    {
        GRAPH_ASSERT(m_num_variables > 0 && idx < m_num_variables);
        Eigen::MatrixXd J(3, 2);
        J.setZero();
        J.block(0, 0, 2, 2) = ComputeNumericalJacobian(m_variables[idx]);
        return J;
    }

private:
    Eigen::Vector2d m_measurement;
};

class Pose2PoseFactor : public Factor
{
public:
    Pose2PoseFactor(Pose2d *v_a, Pose2d *v_b, double x_ab, double y_ab,
                 double yaw_ab_rad, const Eigen::Matrix3d &sqrt_info) : m_pos_ab(x_ab, y_ab), m_yaw_ab_rad(yaw_ab_rad), m_sqrt_info(sqrt_info)
    {
        AddVariable(v_a);
        AddVariable(v_b);
        SetSqrtInfo(sqrt_info);
    }

    virtual int Dim() const { return 3; }

    virtual Eigen::VectorXd Error() const override
    {
        GRAPH_ASSERT(this->NumVariables() == 2);
        const Pose2d *v_a = static_cast<Pose2d *>(this->VariableAt(0));
        const Pose2d *v_b = static_cast<Pose2d *>(this->VariableAt(1));
        Eigen::Vector3d r;
        Eigen::Vector2d pos_ab_pred = {v_b->x() - v_a->x(), v_b->y() - v_a->y()};
        r.head<2>() = Eigen::Rotation2Dd(v_a->yaw_rad()).toRotationMatrix().transpose() * pos_ab_pred - m_pos_ab;
        r(2) = NormalizeAngle((v_b->yaw_rad() - v_a->yaw_rad()) - m_yaw_ab_rad);
        return r;
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const override
    {
        Eigen::Vector3d diff;
        diff << (e1(0) - e2(0)), (e1(1) - e2(1)), NormalizeAngle(e1(2) - e2(2));
        return diff;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
    double NormalizeAngle(double theta_rad) const
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }
private:
    Eigen::Vector2d m_pos_ab;
    double m_yaw_ab_rad;
    Eigen::Matrix3d m_sqrt_info;
};

class RotVec2RotVecFactor : public Factor
{
public:
    RotVec2RotVecFactor(Pose3D *v_a, Pose3D *v_b, Pose3D measurement, const Eigen::MatrixXd &sqrt_info)
        : m_measurement(measurement), m_sqrt_info(sqrt_info)
    {
        AddVariable(v_a);
        AddVariable(v_b);
        // SetSqrtInfo(sqrt_info);
    }

    virtual int Dim() const { return 6; }

    virtual Eigen::VectorXd Error() const override 
    {
        GRAPH_ASSERT(this->NumVariables() == 2);
        Pose3D *v_a = static_cast<Pose3D *>(this->VariableAt(0));
        Pose3D *v_b = static_cast<Pose3D *>(this->VariableAt(1));
        Eigen::VectorXd ret = Eigen::VectorXd::Zero(6);
        
        Pose3D v_ba = v_a->TransformFrom(*v_b);
        RotVec d_r_v(m_measurement.Rot().ToQuaternion().conjugate()*v_a->Rot().ToQuaternion());

        ret.head<3>() = v_ba.Point() - m_measurement.Point();
        ret.tail<3>() = d_r_v.Rot();
        
        // std::cout<<ret.transpose()<<std::endl;
        return ret;
    }

    virtual Eigen::VectorXd SubtractError(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2) const override
    {
        Eigen::VectorXd diff = Eigen::VectorXd::Zero(6);
        diff << (e1(0) - e2(0)), (e1(1) - e2(1)), (e1(2) - e2(2)),
            NormalizeAngle(e1(3) - e2(3)), NormalizeAngle(e1(4) - e2(4)), NormalizeAngle(e1(5) - e2(5));
        return diff;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
    double NormalizeAngle(double theta_rad) const
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }
private:
    Pose3D m_measurement;
    Eigen::MatrixXd m_sqrt_info;
};



#endif // __BIVARIATE_FACTOR_H__