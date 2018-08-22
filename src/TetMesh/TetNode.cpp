//
// Created by Brett Miller on 8/15/18.
//

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

}; // namespace destroyer