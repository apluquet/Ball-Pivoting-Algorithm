#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace tnp {

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points);

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points,
    std::vector<Eigen::Vector3f>& normals);

bool load_obj(
    const std::string& filename, 
    std::vector<Eigen::Vector3f>& points,
    std::vector<Eigen::Vector3f>& normals,
    std::vector<Eigen::Vector3f>& colors);

bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3f>& colors);

bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3i>& faces);

bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3i>& faces);
    
bool save_obj(
    const std::string& filename, 
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<Eigen::Vector3f>& normals,
    const std::vector<Eigen::Vector3f>& colors,
    const std::vector<Eigen::Vector3i>& faces);


} // namespace tnp