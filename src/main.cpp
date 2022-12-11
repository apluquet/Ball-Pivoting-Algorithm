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

    // float ball_radius = 0.25;
    float ball_radius = 6;
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
            outter_edges[edge->from].erase(std::remove(outter_edges[edge->from].begin(), outter_edges[edge->from].end(), edge), outter_edges[edge->from].end());
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

        /////////////////////////////////////////////
        // update the front depending on the case
        /////////////////////////////////////////////

        if (status[pivot_point] == Status::INSIDE) {
            continue;
        }

        // add new face
        faces.push_back(Eigen::Vector3i(edge->from, pivot_point, edge->to));


        if (status[pivot_point] == Status::FREE) {
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

            // push new edges to the front
            front.push(new_edge1);
            front.push(new_edge2);
        }
        else if (pivot_point == edge->next->to && pivot_point == edge->prev->from) {
            // case 2: pivot_point is on the front and is the next point of edge->next
            // and the previous point of edge->prev

            // update status
            // We check the outter edges to avoid setting a point as inside if it is on another front loop
            // For edge->from, we already deleted the current edge from outter_edges so we check if the size is 0
            if (outter_edges[edge->from].size() == 0)
                status[edge->from] = Status::INSIDE;
            // For edge->to and pivot_point, edge->next and edge->prev are still alive so they should have only 1 outter edge if they are not in another front loop
            // We check if they are <= 1 (not == 1) because we might have already deleted them from the outter edges list and kept them alive (case where the pivot point is inside)
            if (outter_edges[edge->to].size() <= 1)
                status[edge->to] = Status::INSIDE;
            if (outter_edges[pivot_point].size() <= 1)
                status[pivot_point] = Status::INSIDE;

            // update to_be_removed
            edge->next->to_be_removed = true;
            edge->prev->to_be_removed = true;
        }
        else if (pivot_point == edge->next->to) {
            // case 3: pivot_point is front and is pointed to by edge->next

            // create new edge from edge->from to pivot
            Edge *new_edge = new Edge();
            new_edge->from = edge->from;
            new_edge->to = pivot_point;
            new_edge->opp = edge->to;
            new_edge->ball_center = pivot_ball_center;
            new_edge->to_be_removed = false;

            // update prev and next pointers
            new_edge->prev = edge->prev;
            new_edge->next = edge->next->next;
            edge->prev->next = new_edge;
            edge->next->next->prev = new_edge;

            // update outter_edges
            outter_edges[new_edge->from].push_back(new_edge);

            // update status
            status[edge->to] = Status::INSIDE;

            // update to_be_removed
            edge->next->to_be_removed = true;

            // push new edges to the front
            front.push(new_edge);
        }
        else if (pivot_point == edge->prev->from) 
        {
            // case 4: pivot_point is front and is the beginning of edge->prev

            // create new edges from edge->from to pivot and from pivot to edge->to
            Edge *new_edge = new Edge();
            new_edge->from = pivot_point;
            new_edge->to = edge->to;
            new_edge->opp = edge->from;
            new_edge->ball_center = pivot_ball_center;
            new_edge->to_be_removed = false;

            // update prev and next pointers
            new_edge->prev = edge->prev->prev;
            new_edge->next = edge->next;
            edge->prev->prev->next = new_edge;
            edge->next->prev = new_edge;

            // update outter_edges
            outter_edges[new_edge->from].push_back(new_edge);

            // update status
            status[edge->from] = Status::INSIDE;

            // update to_be_removed
            edge->prev->to_be_removed = true;

            // push new edges to the front
            front.push(new_edge);
        }
        else {
            // case 5: pivot_point is front and is neither the beginning of edge->prev
            // nor the end of edge->next

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

            // Find the edge that points to pivot_point and is part of the current front loop
            Edge *edge_to_pivot = edge->next->next;
            while (edge_to_pivot->to != pivot_point && edge_to_pivot != edge->prev) {
                edge_to_pivot = edge_to_pivot->next;
            }

            if (edge_to_pivot == edge->prev) {
                continue;
            }

            // update prev and next pointers
            new_edge1->prev = edge->prev;
            new_edge1->next = edge_to_pivot->next;
            new_edge2->prev = edge_to_pivot;
            new_edge2->next = edge->next;
            // Update 1 before 2 to avoid losing edge_to_pivot->next.
            edge->prev->next = new_edge1;
            edge_to_pivot->next->prev = new_edge1; // This
            edge->next->prev = new_edge2;
            edge_to_pivot->next = new_edge2; // Before this

            // update outter_edges
            outter_edges[new_edge1->from].push_back(new_edge1);
            outter_edges[new_edge2->from].push_back(new_edge2);

            // push new edges to the front
            front.push(new_edge1);
            front.push(new_edge2);
        }

        // remove edge from outter edges list to avoid founding it in another edge case 5
        outter_edges[edge->from].erase(std::remove(outter_edges[edge->from].begin(), outter_edges[edge->from].end(), edge), outter_edges[edge->from].end());
    }

    save_obj("mesh.obj", points, faces);

    return 0;
}
