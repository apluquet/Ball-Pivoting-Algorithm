#include <geometry.h>

namespace tnp {

Eigen::Vector3f triangle_normal(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2)
{
    // ...
}

Eigen::Vector3f triangle_circumcenter(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2)
{
    // ...
}

std::optional<Eigen::Vector3f> compute_center(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2,
    float ball_radius)
{
    // ...
}

} // namespace tnp