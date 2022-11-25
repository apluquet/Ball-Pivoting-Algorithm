#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace tnp {

// return the unit normal vector of a 'right-handed oriented' triangle  
Eigen::Vector3f triangle_normal(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2);

// return the center of circumcercle of a triangle
Eigen::Vector3f triangle_circumcenter(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2);

// return the center of the upper r-ball passing through three points if it exists  
std::optional<Eigen::Vector3f> compute_center(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2,
    float ball_radius);

} // namespace tnp