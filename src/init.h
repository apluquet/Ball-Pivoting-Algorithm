#pragma once

#include <Eigen/Core>
#include <kdtree.h>
#include <string>

namespace tnp {

std::pair<Eigen::Vector3i,Eigen::Vector3f> initial_triangle(
    const std::vector<Eigen::Vector3f>& points, 
    const KdTree& kdtree,
    float ball_radius);

} // namespace tnp