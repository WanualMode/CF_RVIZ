#ifndef SEDAS_ROT_HPP
#define SEDAS_ROT_HPP

#include <Eigen/Dense>
#include <cmath>

// Function to compute the rotation matrix from roll, pitch, and yaw
inline Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw) {
    // Z-axis (Yaw) rotation
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw),  std::cos(yaw), 0,
          0, 0, 1;

    // Y-axis (Pitch) rotation
    Eigen::Matrix3d Ry;
    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
         -std::sin(pitch), 0, std::cos(pitch);

    // X-axis (Roll) rotation
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll),  std::cos(roll);

    // Combined rotation: R = Rx * Ry * Rz
    return Rz * Ry * Rx;
}

// Function to transform a vector from Global Frame to Body Frame
inline Eigen::Vector3d Rot_G2D(const Eigen::Vector3d& global_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);
    return R.transpose() * global_vector;
}

// Function to transform a vector from Body Frame to Global Frame
inline Eigen::Vector3d Rot_D2G(const Eigen::Vector3d& body_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);
    return R * body_vector;
}

// Function to transform a vector from Global Frame to Normal Frame
inline Eigen::Vector3d Rot_G2N(const Eigen::Vector3d& global_vector, double roll, double pitch, double yaw) {
    Eigen::Matrix3d R = get_rotation_matrix(roll, pitch, yaw);
    return R.transpose() * global_vector;
}



#endif // SEDAS_ROT_HPP

