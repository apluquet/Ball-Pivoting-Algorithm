#pragma once

#include <Eigen/Core>
#include <optional>
#include <vector>
#include <kdtree.h>

namespace tnp {

// return a pair containing the index of the 'first touched' point 
// and the resulting ball center if it exists
//
// current ball with center ball_center pivots around the edge e_ij
// 
std::optional<std::pair<int,Eigen::Vector3f>> pivot(
    int i, 
    int j,
    int k,
    const Eigen::Vector3f& ball_center, // current ball center
    std::vector<Eigen::Vector3f>& points, 
    const tnp::KdTree& kdtree,
    float ball_radius);

} // namespace tnp