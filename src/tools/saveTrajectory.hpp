#ifndef SAVE_TRAJECTORY_HPP_
#define SAVE_TRAJECTORY_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace zjloc
{
    void saveTrajectoryKITTIformat(std::fstream &fout, Eigen::Matrix4d &T)
    {
        fout << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
             << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
             << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << std::endl;
    }
    void saveTrajectoryKITTIformat(std::fstream &fout, Eigen::Isometry3d &T)
    {
        fout << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
             << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
             << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << std::endl;
    }
    void saveTrajectoryKITTIformat(std::fstream &fout, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l)
    {
        fout << a << " " << b << " " << c << " " << d << " "
             << e << " " << f << " " << g << " " << h << " "
             << i << " " << j << " " << k << " " << l << std::endl;
    }

    void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, Eigen::Vector3d &xyz, Eigen::Quaterniond &xyzw)
    {
        fout << stamp << " " << xyz[0] << " " << xyz[1] << " " << xyz[2] << " " << xyzw.x() << " " << xyzw.y() << " " << xyzw.z() << " " << xyzw.w() << std::endl;
    }
    void saveTrajectoryTUMformat(std::fstream &fout, std::string &stamp, double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        fout << stamp << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
    }
}

#endif