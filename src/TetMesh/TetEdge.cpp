//
// Created by Brett Miller on 8/15/18.
//

#include <iostream>

#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/TetNode.h"

namespace destroyer {

TetEdge::TetEdge(TetNodeRef n0, TetNodeRef n1) {

    nodes_[0] = n0;
    n0->ConnectEdge(this);
    nodes_[1] = n1;
    n1->ConnectEdge(this);
    midpoint_ = nullptr;

}

TetEdge::~TetEdge() {

    nodes_[0]->DisconnectEdge(this);
    nodes_[1]->DisconnectEdge(this);

}

Vec3 TetEdge::MidpointPosition() const {

    return (nodes_[0]->Position() + nodes_[1]->Position()) / 2.0;

}

bool TetEdge::HasMidpoint() const {

    return (midpoint_ != nullptr);

}

void TetEdge::SetMidpoint(TetNodeRef node) {

    midpoint_ = node;

}

TetNodeRef TetEdge::Midpoint() const {

    return midpoint_;

}

TetNodeRef TetEdge::GetFirstNode() const {

    return nodes_[0];

}

TetNodeRef TetEdge::GetOtherNode(TetNodeRef node) const {

    if (nodes_[0] == node)
        return nodes_[1];
    else if (nodes_[1] == node)
        return nodes_[0];
    else
        return nullptr;

}

bool TetEdge::HasNode(TetNodeRef node) const {

    return (nodes_[0] == node) | (nodes_[1] == node);

}

uint TetEdge::IncidentFaceCount() const {

    return incident_faces_.size();

}

std::vector<TetFaceRef> TetEdge::GetIncidentFaces() const {

    return incident_faces_;

}

uint TetEdge::IncidentBoundaryFaceCount() const {

    uint boundary_face_count = 0;

    for (auto& face: incident_faces_) {
        if (face->IsBoundary())
            boundary_face_count++;
    }

    return boundary_face_count;

}

std::vector<TetFaceRef> TetEdge::GetIncidentBoundaryFaces() const {

    std::vector<TetFaceRef> boundary_faces;

    for (auto& face: incident_faces_) {
        if (face->IsBoundary())
            boundary_faces.push_back(face);
    }

    return boundary_faces;

}

TetFaceRef TetEdge::GetFaceWithEdge(TetEdgeRef edge) const {

    for (auto& face: incident_faces_) {
        if (face->HasEdge(edge))
            return face;
    }
    return nullptr;

}

std::vector<TetrahedronRef> TetEdge::GetIncidentTets() const {

    return incident_tets_;

}

bool TetEdge::IsConnected() const {

    // We consider the edge connected if there are incidental tetrahedra.
    return (incident_tets_.size() > 0);

}

Real TetEdge::Length() const {

    Vec3 edge_vec = nodes_[0]->Position() - nodes_[1]->Position();
    return edge_vec.length();

}

Vec3 TetEdge::AsVector() const {

    Vec3 edge_vec = nodes_[0]->Position() - nodes_[1]->Position();
    return edge_vec;

}

bool TetEdge::IsNonManifold() const {

    // The edge is considered non-manifold if more that two boundary faces are connected to it.
    return (IncidentBoundaryFaceCount()>2);

}

void TetEdge::ConnectFace(TetFaceRef face) {

    incident_faces_.push_back(face);

}

void TetEdge::DisconnectFace(TetFaceRef face) {

    auto found_face = std::find(incident_faces_.begin(), incident_faces_.end(), face);
    if (found_face != incident_faces_.end())
        incident_faces_.erase(found_face);

}

void TetEdge::ConnectTetrahedron(TetrahedronRef tet) {

    incident_tets_.push_back(tet);

}

void TetEdge::DisconnectTetrahedron(TetrahedronRef tet) {

    auto found_tet = std::find(incident_tets_.begin(), incident_tets_.end(), tet);
    if (found_tet != incident_tets_.end())
        incident_tets_.erase(found_tet);
}

}; // namespace destroyer

