#include <geometry.h>
#include <pivot.h>

namespace tnp {

template <typename T>
T safe_acos(T value) {
  if (value <= T{-1.0}) {
    return T{M_PI};
  } else if (value >= T{1.0}) {
    return T{0.0};
  } else {
    return std::acos(value);
  }
}

std::optional<std::pair<int, Eigen::Vector3f>> pivot(
    int i, int j, int k, const Eigen::Vector3f& ball_center,
    std::vector<Eigen::Vector3f>& points, const tnp::KdTree& kdtree,
    float ball_radius) {
  Eigen::Vector3f m = (points[i] + points[j]) / 2.0;
  Eigen::Vector3f u = (points[j] - m).normalized();
  Eigen::Vector3f v = (ball_center - m).normalized();
  Eigen::Vector3f w = u.cross(v).normalized();

  float theta_min = std::numeric_limits<float>::max();
  Eigen::Vector3f s_min = {0, 0, 0};
  int l_min = -1;

  int size = points.size();

  for (int l = 0; l < size; l++) {
    if (l == i || l == j || l == k || (m - points[l]).norm() > 2 * ball_radius)
      continue;
    std::optional<Eigen::Vector3f> s =
        compute_center(points[i], points[l], points[j], ball_radius);
    if (!s.has_value()) continue;

    Eigen::Vector3f v_ms = s.value() - m;

    // Appartient [0, 2PI]
    float theta =
        safe_acos(v.dot(v_ms) / (v.squaredNorm() * v_ms.squaredNorm()));

    if (theta < theta_min) {
      theta_min = theta;
      s_min = s.value();
      l_min = l;
    }
  }

  if (l_min == -1) return std::nullopt;

  return std::pair{l_min, s_min};
}

}  // namespace tnp