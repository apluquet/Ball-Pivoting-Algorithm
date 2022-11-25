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

/**
 * Compute center of sphere that passes through the three points of a triangle.
 * 
 * Soit p0, p1, p2 les trois points du triangle, c le centre de la sphère et r le
 * rayon de la sphère. On a alors :
 *
 * On sait que :
 * ||p0 - c||^2 = r^2
 *
 * Or, c = q + tn, donc on a :
 * || q + tn - p0 ||^2 - r^2 = 0
 * (q + tn - p0).dot(q + tn - p0) - r^2 = 0
 * (q - p0).dot(q - p0) + 2t * (q - p0).dot(n) + t^2 * n.dot(n) - r^2 = 0
 *
 * Or, n.dot(n) = 1 et (q - p0).dot(n) = 0, donc on a :
 * (q - p0).dot(q - p0) + t^2 - r^2 = 0
 *
 * Soit :
 * t^2 = r^2 - ||q - p0||^2
 *
 * Ainsi, on a :
 * t = sqrt(r^2 - ||q - p0||^2) si r^2 - ||q - p0||^2 >= 0
 */
std::optional<Eigen::Vector3f> compute_center(const Eigen::Vector3f& p0,
                                              const Eigen::Vector3f& p1,
                                              const Eigen::Vector3f& p2,
                                              float ball_radius) {
  Eigen::Vector3f q = triangle_circumcenter(p0, p1, p2);
  Eigen::Vector3f n = triangle_normal(p0, p1, p2);
  float r = ball_radius;

  float t = std::sqrt(r * r - (q - p0).squaredNorm());
  if (t < 0) {
    return std::nullopt;
  }

  return q + t * n;
}
}  // namespace tnp