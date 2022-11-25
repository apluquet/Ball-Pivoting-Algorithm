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


    // create initial triangle edges
    Edge *e01 = new Edge{init.first[0], init.first[1], init.first[2], init.second, nullptr, nullptr};
    Edge *e12 = new Edge{init.first[1], init.first[2], init.first[0], init.second, nullptr, nullptr};
    Edge *e20 = new Edge{init.first[2], init.first[0], init.first[1], init.second, nullptr, nullptr};

    // link edges
    e01->prev = e20;
    e01->next = e12;
    e12->prev = e01;
    e12->next = e20;
    e20->prev = e12;
    e20->next = e01;

    // add edges to the outter edges list
    outter_edges[e01->from].push_back(e01);
    outter_edges[e12->from].push_back(e12);
    outter_edges[e20->from].push_back(e20);

    // set status for the 3 points of the initial triangle
    status[e01->from] = Status::FRONT;
    status[e12->from] = Status::FRONT;
    status[e20->from] = Status::FRONT;

    // add initial triangle to the faces
    faces.push_back(init.first);

    // init front
    std::stack<Edge*> front = std::stack<Edge*>();

    // add initial triangle edges to the front
    front.push(e01);
    front.push(e12);
    front.push(e20);

    while (!front.empty()) {
        Edge *edge = front.top();
        front.pop();

        // if the edge is no longer needed, we remove it from the front and continue
        if (edge->to_be_removed) {
            delete edge;
            continue;
        }
        if (status[edge->from] != Status::FRONT || status[edge->to] != Status::FRONT) {
            continue;
        }

        // compute pivot
        std::optional<std::pair<int, Eigen::Vector3f>> pivot = tnp::pivot(edge->from, edge->to, edge->opp, edge->ball_center, points, kdtree, ball_radius);
        
        // if cannot pivot, continue
        if (!pivot.has_value()) {
            continue;
        }

        // get pivot point and ball center
        int pivot_point = pivot.value().first;
        Eigen::Vector3f pivot_ball_center = pivot.value().second;

        // add new face
        faces.push_back(Eigen::Vector3i(edge->from, pivot_point, edge->to));

        /////////////////////////////////////////////
        // update the front depending on the case
        /////////////////////////////////////////////

        // case 2 to 5
        if (status[pivot_point] != Status::FREE) continue;

        /////////////////////////////////////////////
        // case 1: pivot_point is free
        /////////////////////////////////////////////

        // create new edges from edge->from to pivot and from pivot to edge->to
        Edge *new_edge1 = new Edge();
        new_edge1->from = edge->from;
        new_edge1->to = pivot_point;
        new_edge1->opp = edge->to;
        new_edge1->ball_center = pivot_ball_center;
        new_edge1->to_be_removed = false;

        Edge *new_edge2 = new Edge();
        new_edge2->from = pivot_point;
        new_edge2->to = edge->to;
        new_edge2->opp = edge->from;
        new_edge2->ball_center = pivot_ball_center;
        new_edge2->to_be_removed = false;

        // update prev and next pointers
        new_edge1->prev = edge->prev;
        new_edge1->next = new_edge2;
        new_edge2->prev = new_edge1;
        new_edge2->next = edge->next;
        edge->prev->next = new_edge1;
        edge->next->prev = new_edge2;

        // update outter_edges
        outter_edges[new_edge1->from].push_back(new_edge1);
        outter_edges[new_edge2->from].push_back(new_edge2);

        // update status
        status[new_edge1->to] = Status::FRONT;

        // update to_be_removed
        // edge->to_be_removed = true;

        // push new edges to the front
        front.push(new_edge1);
        front.push(new_edge2);
    }

    tnp::save_obj("mesh.obj", points, faces);

    return 0;
}
