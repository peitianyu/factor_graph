#ifndef __VARIABLE_TYPE_H__
#define __VARIABLE_TYPE_H__

#include "variable.h"
#include <Eigen/Core>

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

#endif // __VARIABLE_TYPE_H__