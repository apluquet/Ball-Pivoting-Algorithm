#include <kdtree.h>
#include <obj.h>
#include <geometry.h>
#include <pivot.h>
#include <init.h>

#include <stack>

// Edge e_ij oriented from point i to point j
// point opp is the 3rd vertex of the triangle defined by this edge
// ball_center is the center of the ball associated to this edge
// next and prev allow to consider a doubly linked list
// to_be_removed is set to true if the edge is no longer needed 
struct Edge
{
    int from, to, opp;
    Eigen::Vector3f ball_center;
    Edge* next;
    Edge* prev;
    bool to_be_removed{false};
};

// point status
enum class Status 
{
    FREE,   // no triangle 
    FRONT,  // on the propagated frot
    INSIDE  // surrounded by triangles
};

using namespace tnp;

int main(int argc, char *argv[])
{
    // option -----------------------------------------------------------------
    if(argc <= 1) {
        std::cout << "Error: missing argument" << std::endl;
        std::cout << "Usage: ball_pivoting <filename>.obj [ball_radius]" << std::endl;
        return 0;
    }
    const auto filename = std::string(argv[1]);

    float ball_radius = 0.25;
    if(argc >= 3) {
        ball_radius = std::stof(argv[2]);
    }

    // load -------------------------------------------------------------------
    auto points = std::vector<Eigen::Vector3f>();
    auto normals = std::vector<Eigen::Vector3f>();
    if(not tnp::load_obj(filename, points, normals))
    {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    // process ----------------------------------------------------------------
    auto outter_edges = std::vector<std::vector<Edge*>>(points.size());
    auto status = std::vector<Status>(points.size(), Status::FREE);
    auto faces = std::vector<Eigen::Vector3i>();

    tnp::KdTree kdtree;
    kdtree.build(points);

    const auto init = initial_triangle(points, kdtree, ball_radius);

    // ...

    tnp::save_obj("mesh.obj", points, faces);

    return 0;
}
