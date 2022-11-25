#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <numeric>
#include <iostream>
#include <vector>

namespace tnp {

// Used by a leaf to define the range of points indices it contains
using iterator = std::vector<int>::iterator;

//
// Axis-aligned bounding box in 3D
//
// Example:
//     Box3f box;
//     box.extend(Eigen::Vector3f{1,2,3});
//      ...
//     box.extend(Eigen::Vector3f-1,5,9});
//     const Eigen::Vector3f min = box.min();
//     const Eigen::Vector3f max = box.max();
//     const Eigen::Vector3f diagonal = box.diagonal();
// 
using Box3f = Eigen::AlignedBox<float,3>;

//
// Type of function called on point indices by 
// the search methods KdTree::for_each_neighbors 
// and KdTree::for_each_neighbors_rec
//
using Func = std::function<void(int)>;

//
// KdTree node
//   leaf if left_child == nullptr
//   intermediate node otherwise
// 
struct Node
{
    // leaf -------------------------------------------------------------------
    iterator begin;     // begin iterator to the indices of this leaf (iterator over Kdtree::m_indices)
    iterator end;       // "past-the-end" iterator
    // intermediate node ------------------------------------------------------
    Node* left_child;   // contains points p such that p[cut_dim] < cut_value
    Node* right_child;  // contains points p such that p[cut_dim] >= cut_value
    int cut_dim;        // 0, 1 or 2 (for x, y and z)
    float cut_value;    // cut space in half along cut_dim
};

//
// for each leaf: std::distance(begin, end) <= 25
// this is only used for the stopping condition of the recursive method KdTree::build_rec
//
constexpr auto max_number_point_per_leaf = 25;

//
// 3D binary search tree recursively cutting in half 
// along the dimension where points spread the most
//
class KdTree
{
public:
    KdTree() = default;
    ~KdTree();

public:
    // build the kdtree (without modifying the points)
    void build(const std::vector<Eigen::Vector3f>& points);

    //
    // neighbors range search from point p and distance r
    // call f on each point at index i such that (p - points[i]).norm() < r
    //
    // Example: 
    //     kdtree.for_each_neighbors(points, p, r, [&points](int i)
    //     {
    //         std::cout << "Found point " << i << ": " << points[i].transpose() << std::endl;
    //     });
    //
    void for_each_neighbors(
        const std::vector<Eigen::Vector3f>& points, // point cloud
        const Eigen::Vector3f& p,                   // query point
        float r,                                    // query radius
        Func f) const;                              // function to called on resulting indices

private:
    // recursively build the tree
    // consider points at indices in the range (begin,end(
    // node is already allocated and must be filled
    // points are not changed
    void build_rec(
        const std::vector<Eigen::Vector3f>& points, // point cloud
        iterator begin,                             // begin iterator over the current points indices
        iterator end,                               // end iterator over the current points indices
        Node* node);                                // node to fill

    // recursively delete nodes
    void delete_rec(Node* node);

public:
    Node* m_root;               // root node of the tree
    std::vector<int> m_indices; // vector of unique indices that references the points
};

} // namespace tnp