#include <geometry.h>

namespace tnp {

Eigen::Vector3f triangle_normal(const Eigen::Vector3f& p0,
                                const Eigen::Vector3f& p1,
                                const Eigen::Vector3f& p2) {
  Eigen::Vector3f v01 = p1 - p0;
  Eigen::Vector3f v02 = p2 - p0;
  return v01.cross(v02).normalized();
}

Eigen::Vector3f triangle_circumcenter(const Eigen::Vector3f& p0,
                                      const Eigen::Vector3f& p1,
                                      const Eigen::Vector3f& p2) {
  Eigen::Vector3f circumcenter = {0, 0, 0};

  Eigen::Vector3f v01 = p1 - p0;
  Eigen::Vector3f v02 = p2 - p0;
  Eigen::Vector3f v = v01.cross(v02).normalized();

  float v01_square = v01.squaredNorm();
  float v02_square = v02.squaredNorm();
  float v_square = v.squaredNorm();

  circumcenter =
      p0 + ((v01_square * v02 - v02_square * v01).cross(v)) / 2 * v_square;

  return circumcenter;
}

std::optional<Eigen::Vector3f> compute_center(const Eigen::Vector3f& p0,
                                              const Eigen::Vector3f& p1,
                                              const Eigen::Vector3f& p2,
                                              float ball_radius) {
  // ...
}

}  // namespace tnp