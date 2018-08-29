//
// Created by Brett Miller on 8/15/18.
//

#include <iostream>

#include "TetMesh/TetNode.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/TetEdge.h"


namespace destroyer {

TetNode::TetNode(Real x, Real y, Real z, Index id) {

    position_ = Vec3(x,y,z);
    id_ = id;
    depth_ = 0;

}

TetNode::TetNode(Vec3 position, Index id) {

    position_ = position;
    id_ = id;
    depth_ = 0;

}

Vec3 TetNode::Position() const {

    return position_;

}

void TetNode::SetPosition(Vec3 position) {

    position_ = position;

}

Index TetNode::Id() const {

    return id_;

}

void TetNode::SetId(Index id) {

    id_ = id;

}

uint TetNode::Depth() const {

    return depth_;

}

void TetNode::SetDepth(uint depth) {

    depth_ = depth;

}

Real TetNode::Sdf() const {

    return sdf_;

}

void TetNode::SetSdf(Real sdf) {

    sdf_ = sdf;

}

bool TetNode::IsConnected() const {

    // We consider a node disconnected if it has no incident tetrahedra.
    return (incident_tets_.size() > 0);

}

bool TetNode::IsBoundary() const {

    // If a node has an incident boundary face, it is boundary.
    for (auto& face: incident_faces_) {
        if (face->IsBoundary())
            return true;
    }
    return false;

}

bool TetNode::IsNonManifold() const {
    //std::cout << "is nonmanifold in " << std::endl;

    // A node can only be non-manifold if it lies on the boundary.
    if (!IsBoundary())
        return false;
    //std::cout << "is nonmanifold edge " << std::endl;

    // If a node is connected to a non-manifold edge, it is considered non-manifold.
    for (auto& edge: incident_edges_) {
        if (edge->IsNonManifold())
            return true;
    }
    //std::cout << "is nonmanifold ring " << std::endl;

    // If a node has more than one edge ring, it is considered non-manifold.
    // First, filter the boundary faces and collect the edge rings.
    auto ring_edges = GetAllRingEdges();
    auto ring_edge_count = ring_edges.size();
    //std::cout << "is nonmanifold ringA " << std::endl;

    auto first_edge_ring = GetFirstEdgeRing();
    auto first_ring_count = first_edge_ring.size();
    //std::cout << "is nonmanifold ringB" << std::endl;

    // If we've traversed an entire edge loop and there are still more edges, we have multiple edge loops.
    if (first_ring_count < ring_edge_count)
        return true;

    //std::cout << "is nonmanifold clear" << std::endl;
    // All clear.
    return false;

}

std::vector<TetEdgeRef> TetNode::GetFirstEdgeRing() const {

    //std::cout << "here goes" << std::endl;
    auto ring_edges = GetAllRingEdges();
    auto ring_edge_count = ring_edges.size();

    std::vector<TetEdgeRef> first_ring_edges;

    int current_edge = 0;
    first_ring_edges.push_back(ring_edges[current_edge]);
    auto head_node = ring_edges[0]->GetFirstNode();
    auto current_node = ring_edges[0]->GetOtherNode(head_node);
    do {

        // Find the edge that isn't this one that has the current node.
        int new_edge = 0;
        while (new_edge<ring_edge_count) {
            //std::cout << "is nonmanifold ring 3 " << new_edge << "/" << ring_edge_count << std::endl;
            //std::cout << "Neat " << ring_edges[new_edge]->HasMidpoint() << std::endl;
            if ((current_edge!=new_edge)&(ring_edges[new_edge]->HasNode(current_node))) {
                current_edge = new_edge;
                current_node = ring_edges[current_edge]->GetOtherNode(current_node);
                break;
            }
            new_edge++;
        }
        first_ring_edges.push_back(ring_edges[current_edge]);

        // Unlikely scenario...
        if (new_edge == ring_edge_count) {
            std::cout << "Never found the head" << std::endl;
            break;
        }

    } while (current_node!=head_node);

    return first_ring_edges;

}

std::vector<TetrahedronRef> TetNode::GetIncidentTets() const {

    return incident_tets_;

}

TetEdgeRef TetNode::GetEdgeTo(TetNodeRef node) const {

    for (auto& edge: incident_edges_) {
        if (edge->GetOtherNode(node) != nullptr)
            return edge;
    }
    return nullptr;

}

void TetNode::ConnectTetrahedron(TetrahedronRef tet) {

    incident_tets_.push_back(tet);

}

void TetNode::DisconnectTetrahedron(TetrahedronRef tet) {

    auto found = std::find(incident_tets_.begin(), incident_tets_.end(), tet);
    if (found != incident_tets_.end())
        incident_tets_.erase(found);

}

void TetNode::ConnectFace(TetFaceRef face) {

    incident_faces_.push_back(face);

}

void TetNode::DisconnectFace(TetFaceRef face) {

    auto found = std::find(incident_faces_.begin(), incident_faces_.end(), face);
    if (found != incident_faces_.end())
        incident_faces_.erase(found);

}

void TetNode::ConnectEdge(TetEdgeRef edge) {

    incident_edges_.push_back(edge);

}

void TetNode::DisconnectEdge(TetEdgeRef edge) {

    auto found = std::find(incident_edges_.begin(), incident_edges_.end(), edge);
    if (found != incident_edges_.end())
        incident_edges_.erase(found);

}

std::vector<TetEdgeRef> TetNode::GetAllRingEdges() const {

    // Filter the boundary faces and collect the edges forming rings around this node.
    std::vector<TetEdgeRef> ring_edges;

    //std::cout << "I have " << incident_faces_.size() << " faces" << std::endl;

    for (auto& face: incident_faces_) {
        if (face->IsBoundary())
            ring_edges.push_back(face->GetOppositeEdge(TetNodeRef(this)));
    }

    return ring_edges;

}

}; // namespace destroyer