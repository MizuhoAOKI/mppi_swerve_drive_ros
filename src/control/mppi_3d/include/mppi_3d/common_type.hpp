# pragma once
#include <Eigen/Dense>

namespace common_type
{
    struct XYYaw
    {
        int dim = 3; // dimension
        double x; // [m] x pos. in global frame
        double y; // [m] y pos. in global frame
        double yaw; // [rad] yaw angle in global frame, counter-clockwise is positive

        // unwrap yaw angle
        void unwrap()
        {
            if (yaw > 2.0 * M_PI)
            {
                yaw -= 2.0 * M_PI;
            }
            else if (yaw < 0.0)
            {
                yaw += 2.0 * M_PI;
            }
        }

        // return value as eigen matrix
        Eigen::Matrix<double, 3, 1> eigen()
        {
            Eigen::Matrix<double, 3, 1> vector;
            vector << x, y, yaw;
            return vector;
        }

        // update values with eigen matrix
        void update(const Eigen::Matrix<double, 3, 1>& vec)
        {
            x = vec(0, 0);
            y = vec(1, 0);
            yaw = vec(2, 0);
        }
    };

    struct VxVyOmega
    {
        int dim = 3; // dimension
        double vx; // [m/s] in vehicle base frame, forward velocity
        double vy; // [m/s] in vehicle base frame, left velocity
        double omega; // [rad/s] in vehicle base frame, yaw rate, counter-clockwise is positive

        // set min/max values
        double vx_min = -2.0;
        double vx_max = +2.0;
        double vy_min = -2.0;
        double vy_max = +2.0;
        double omega_min = -1.57;
        double omega_max = +1.57;

        // clamp control input
        void clamp()
        {
            vx = std::max(vx_min, std::min(vx_max, vx));
            vy = std::max(vy_min, std::min(vy_max, vy));
            omega = std::max(omega_min, std::min(omega_max, omega));
        }

        // return value as eigen matrix
        Eigen::Matrix<double, 3, 1> eigen()
        {
            Eigen::Matrix<double, 3, 1> vector;
            vector << vx, vy, omega;
            return vector;
        }

        // update values with eigen matrix
        void update(const Eigen::Matrix<double, 3, 1>& vec)
        {
            vx = vec(0, 0);
            vy = vec(1, 0);
            omega = vec(2, 0);
        }

        // initialize control input
        void setZero()
        {
            vx = 0.0;
            vy = 0.0;
            omega = 0.0;
        }

        // definition of iterator
        double& operator[](int idx)
        {
            switch (idx)
            {
            case 0:
                return vx;
            case 1:
                return vy;
            case 2:
                return omega;
            default:
                throw std::out_of_range("Index out of range");
            }
        }
    };
} // namespace controller
