#ifndef __VARIABLE_TYPE_H__
#define __VARIABLE_TYPE_H__

#include "variable.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class Point2d : public Variable
{
public:
    Point2d(const Eigen::Vector2d &pos) : m_position(pos) {}

    Eigen::Vector2d Position()const { return m_position; }
    virtual size_t Dim()const override { return 2; }
    virtual void Plus(const Eigen::VectorXd& delta) override
    {
        m_position[0] += delta[0];
        m_position[1] += delta[1];
    }

    virtual void Print() const override
    {
        std::cout << m_position.transpose() << std::endl;
    }

private:
  Eigen::Vector2d m_position;
};


class Pose2d : public Variable
{
public:
    Pose2d(double x, double y, double yaw_rad) : m_x(x), m_y(y), m_yaw_rad(yaw_rad)
    {}

    Pose2d(const Eigen::Vector3d& p) : m_x(p(0)), m_y(p(1)), m_yaw_rad(p(2))
    {}

    Eigen::Vector2d Position() const { return Eigen::Vector2d(m_x, m_y); }

    Eigen::Vector3d Pose() const { return Eigen::Vector3d(m_x, m_y, m_yaw_rad); }

    virtual size_t Dim() const override { return 3; }

    virtual void Plus(const Eigen::VectorXd &delta) override
    {
        m_x += delta[0];
        m_y += delta[1];
        m_yaw_rad = NormalizeAngle(m_yaw_rad + delta[2]);
    }

    virtual void Print() const override
    {
        std::cout << m_x << " " << m_y << " " << m_yaw_rad << std::endl;
    }

    double x() const { return m_x; }
    double y() const { return m_y; }
    double yaw_rad() const { return m_yaw_rad; }

private:
    double NormalizeAngle(double theta_rad)
    {
        // Normalize the angle to the range [-pi, pi).
        constexpr double kPI = 3.14159265358979323846;
        constexpr double k2PI = 2.0 * kPI;
        return (theta_rad - k2PI * std::floor((theta_rad + kPI) / k2PI));
    }
private:
    double m_x, m_y, m_yaw_rad;
};

class Point3d : public Variable
{
public:
    Point3d(const Eigen::Vector3d &pos) : m_position(pos) {}

    Eigen::Vector3d Position()const { return m_position; }
    virtual size_t Dim()const override { return 3; }
    virtual void Plus(const Eigen::VectorXd& delta) override
    {
        m_position[0] += delta[0];
        m_position[1] += delta[1];
        m_position[2] += delta[2];
    }

    virtual void Print() const override
    {
        std::cout << m_position.transpose() << std::endl;
    }

private:
  Eigen::Vector3d m_position;
};

class RotVec
{
public:
    RotVec(const Eigen::Vector3d &r_v)
        : m_r_v(r_v)
    {
    }

    RotVec(const Eigen::Quaterniond &in_q)
    {
        Eigen::Quaterniond q = in_q.normalized();
        if (q.w() < 0)
            q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
        Eigen::Vector3d r_v(q.x(), q.y(), q.z());
        double norm = r_v.norm();
        if (norm < 1e-7)
        {
            m_r_v = 2 * r_v;
        }
        else
        {
            double th = 2 * atan2(norm, q.w());
            m_r_v = r_v / norm * th;
        }
    }

    Eigen::Vector3d Rot() const { return m_r_v; }

    double Norm() const { return m_r_v.norm(); }

    RotVec Inverted() const { return RotVec(m_r_v * (-1.0)); }

    Eigen::Quaterniond ToQuaternion() const
    {
        double v = m_r_v.norm();
        if (v < 1e-6)
            return Eigen::Quaterniond(1, 0, 0, 0);

        double cs = cos(v / 2);
        double ss = sin(v / 2) / v;
        return Eigen::Quaterniond(cs, ss * m_r_v(0), ss * m_r_v(1), ss * m_r_v(2));
    }

    Eigen::Matrix3d ToRotationMatrix() const { return ToQuaternion().toRotationMatrix(); }

private:
    Eigen::Vector3d m_r_v;
};

class Pose3D : public Variable
{
public:
    Pose3D(const Eigen::Vector3d &point, const Eigen::Vector3d &rot)
        : m_point(point), m_rot(rot)
    {
    }

    Pose3D(const Eigen::Vector3d &point, const RotVec &rot)
        : m_point(point), m_rot(rot)
    {
    }

    Eigen::Vector3d Point() const { return m_point; }
    RotVec Rot() const { return m_rot; }
    virtual size_t Dim() const override { return 6; }
    virtual void Print() const override { std::cout << m_point.transpose() << std::endl
                                                    << m_rot.Rot().transpose() << std::endl; }
    virtual void Plus(const Eigen::VectorXd &delta) override
    {
        Pose3D pose = TransformAdd(Pose3D(delta.head<3>(), delta.tail<3>()));
        m_point = pose.Point();
        m_rot = pose.Rot();
    }

    Pose3D TransformAdd(const Pose3D &pose)
    {
        RotVec rv(pose.Rot());
        Eigen::Vector3d transition = m_rot.ToRotationMatrix() * pose.Point() + m_point;
        RotVec r_v(m_rot.ToQuaternion() * rv.ToQuaternion());
        return Pose3D(transition, r_v);
    }

    Pose3D TransformFrom(const Pose3D &pose)
    {
        RotVec rv(pose.Rot());
        Eigen::Vector3d transition = rv.ToRotationMatrix().transpose() * (m_point - pose.Point());
        RotVec r_v(rv.ToQuaternion().conjugate() * m_rot.ToQuaternion());
        return Pose3D(transition, r_v);
    }

private:
    Eigen::Vector3d m_point;
    RotVec m_rot;
};

#endif // __VARIABLE_TYPE_H__