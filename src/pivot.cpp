#include <pivot.h>
#include <geometry.h>

namespace tnp {

template<typename T>
T safe_acos(T value) 
{
    if(value <= T{-1.0}) 
    {
        return T{M_PI};
    } 
    else if(value >= T{1.0}) 
    {
        return T{0.0};
    } 
    else 
    {
        return std::acos(value);
    }
}

std::optional<std::pair<int,Eigen::Vector3f>> pivot(
    int i,
    int j,
    int k,
    const Eigen::Vector3f& ball_center,
    std::vector<Eigen::Vector3f>& points, 
    const tnp::KdTree& kdtree,
    float ball_radius)
{
    // ...
    return {};
}

} // namespace tnp