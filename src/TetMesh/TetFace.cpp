//
// Created by Brett Miller on 8/15/18.
//

#include <iostream>
#include <limits>

#include "TetMesh/TetFace.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"

namespace destroyer {

TetFace::TetFace(TetEdgeRef e0, TetEdgeRef e1, TetEdgeRef e2) {

    edges_[0] = e0;
    edges_[0]->ConnectFace(this);
    edges_[1] = e1;
    edges_[1]->ConnectFace(this);
    edges_[2] = e2;
    edges_[2]->ConnectFace(this);

    nodes_[0] = edges_[0]->GetFirstNode();
    nodes_[0]->ConnectFace(this);
    nodes_[1] = edges_[0]->GetOtherNode(nodes_[0]);
    nodes_[1]->ConnectFace(this);

    // Final node should be found on either e1 or e2.
    nodes_[2] = edges_[1]->GetOtherNode(nodes_[1]);
    if (nodes_[2] == nullptr)
        nodes_[2] = edges_[2]->GetOtherNode(nodes_[1]);
    nodes_[2]->ConnectFace(this);

}

TetFace::~TetFace() {

    for (auto& node: nodes_) {
        node->DisconnectFace(this);
    }

    for (auto& edge: edges_) {
        edge->DisconnectFace(this);
    }

}

bool TetFace::IsBoundary() const {

    // If there is only a single tet associated with this face, it's on the boundary.
    return (incident_tets_.size() == 1);

}

TetrahedronRef TetFace::GetFirstTet() const {

    if (!incident_tets_.empty())
        return incident_tets_[0];
    else
        return nullptr;
}

TetrahedronRef TetFace::GetOtherTet(TetrahedronRef tet) const {

    if (incident_tets_.size() < 2)
        return nullptr;
    if (incident_tets_[0] == tet)
        return incident_tets_[1];
    else if (incident_tets_[1] == tet)
        return incident_tets_[0];
    else
        return nullptr;

}

TetEdgeRef TetFace::GetOppositeEdge(const TetNodeRef node) const {

    // The edge that does not contain the node is opposite of the node.
    for (auto edge_index: {0,1,2}) {
        if (!edges_[edge_index]->HasNode(node))
            return edges_[edge_index];
    }
    return nullptr;

}

TetEdgeRef TetFace::GetRightEdge(TetNodeRef node) const {

    // The edge that does not contain the node is opposite of the node.
    // The edge preceeding it is the Right edge.
    for (auto edge_index: {0,1,2}) {
        if (!edges_[edge_index]->HasNode(node))
            return edges_[(edge_index-1)%3];
    }
    return nullptr;

}

TetEdgeRef TetFace::GetLeftEdge(TetNodeRef node) const {

    // The edge that does not contain the node is opposite of the node.
    // The edge following it is the Left edge.
    for (auto edge_index: {0,1,2}) {
        if (!edges_[edge_index]->HasNode(node))
            return edges_[(edge_index+1)%3];
    }
    return nullptr;

}


void TetFace::ReplaceEdge(TetEdgeRef edge) {

    // Which edge is it?
    auto node0 = edge->GetFirstNode();
    auto node1 = edge->GetOtherNode(node0);

    for (int i: {0,1,2}) {
        if (edges_[i]->HasNode(node0) & edges_[i]->HasNode(node1)) {
            if (edges_[i] != edge) {
                edges_[i]->DisconnectFace(this);
                edges_[i] = edge;
                edges_[i]->ConnectFace(this);
            }
            break;
        }
    }
}

void TetFace::ReplaceNode(TetNodeRef original, TetNodeRef replacement) {

    // Find the index
    int index = 0;
    for (;index<3;index++)
        if (nodes_[index] == original)
            break;

    // Foud the corresponsing node.
    if (index<3) {

        nodes_[index] = replacement;
        original->DisconnectFace(this);
        replacement->ConnectFace(this);

    }
}


void TetFace::ConnectTetrahedron(TetrahedronRef tet) {

    incident_tets_.push_back(tet);

}

void TetFace::DisconnectTetrahedron(TetrahedronRef tet) {

    auto found_tet = std::find(incident_tets_.begin(), incident_tets_.end(), tet);
    if (found_tet != incident_tets_.end())
        incident_tets_.erase(found_tet);
}

bool TetFace::IsConnected() const {

    // We consider the face disconnected if there are no incident tets.
    return (incident_tets_.size() > 0);

}

bool TetFace::HasEdge(TetEdgeRef edge) const {

    for (auto edge_index: {0,1,2}) {
        if (edges_[edge_index] == edge)
            return true;
    }
    return false;

}

Vec3 TetFace::Normal(TetrahedronRef owner_tet) const {

    auto edge0 = nodes_[1]->Position() - nodes_[0]->Position();
    auto edge1 = nodes_[2]->Position() - nodes_[0]->Position();

    auto normal = cross(edge0, edge1);

    // Ensure that the vector is facing out from the tetrahedron.
    Vec3 out_vector(0.0,0.0,0.0);
    if (owner_tet == nullptr) {
        out_vector = Centroid() - incident_tets_[0]->Centroid();
    } else
        out_vector = Centroid() - owner_tet->Centroid();

    if (normal.dot(out_vector) < 0.0)
        normal *= -1.0;

    normal.normalize();

    return normal;

}

Real TetFace::EdgeLengthRatio() const {

    Real max_length = 0.0;
    Real min_length = std::numeric_limits<Real>::max();

    for (auto& edge: edges_) {
        auto length = edge->Length();
        if (length > max_length)
            max_length = length;
        if (length < min_length)
            min_length = length;
    }

    return min_length/max_length;

}

Real TetFace::MinNodeAltitude() const {

    Real min_altitude = std::numeric_limits<Real>::max();

    for (auto& node: nodes_) {
        auto pos = node->Position();
        auto opposite_edge = GetOppositeEdge(node);
        auto to_node = pos - opposite_edge->GetFirstNode()->Position();
        auto ground = opposite_edge->AsVector();
        ground.normalize();
        auto projected = to_node - (to_node.dot(ground) * ground);
        auto altitude = projected.length();
        if (altitude < min_altitude)
            min_altitude = altitude;
    }

    return min_altitude;

}

Real TetFace::MaxEdgeLength() const {

    Real edge_length = 0.0;

    for (auto& edge: edges_) {
        auto length = edge->Length();
        if (length > edge_length) {
            edge_length = length;
        }
    }

    return edge_length;

}

Real TetFace::MaxAngle() const {

    Real node_angle = 0.0;

    std::array<Vec3,3> points {{nodes_[0]->Position(),  nodes_[1]->Position(), nodes_[2]->Position()}};

    for (auto node_index: {0,1,2}) {
        Vec3 left = points[(node_index-1)%3] - points[node_index];
        left.normalize();

        Vec3 right = points[(node_index+1)%3] - points[node_index];
        right.normalize();

        auto angle = acos(left.dot(right));
        if (angle > node_angle) {
            node_angle = angle;
        }
    }

    return node_angle;

}

Real TetFace::Area() const {

    auto edge0 = nodes_[1]->Position() - nodes_[0]->Position();
    auto edge1 = nodes_[2]->Position() - nodes_[0]->Position();
    auto area = cross(edge0,edge1).length() / 2.0;

    return area;

}

Vec3 TetFace::Centroid() const {

    Vec3 centroid(0.0,0.0,0.0);

    for (auto& node: nodes_) {
        centroid += node->Position();
    }

    return centroid / 3.0;

}

Real TetFace::Inradius() const {

    // Area = inradius / semiperimeter
    return Area() / Semiperimeter();

}

Real TetFace::Circumradius() const {

    // R = abc / 4rs
    Real product = 1.0;
    for ( auto& edge: edges_) {
        product *= edge->Length();
    }

    return product / ( 4.0 * Inradius() * Semiperimeter() );

}

Real TetFace::Semiperimeter() const {

    // Half the perimeter
    Real perimeter = 0.0;
    for ( auto& edge: edges_) {
        perimeter += edge->Length();
    }

    return perimeter / 2.0;

}

Real TetFace::QualityMeasure() const {

    auto ratio = 2.0 * (Inradius() / Circumradius());
    auto edge_to_alt = MinNodeAltitude() / MaxEdgeLength();
    return (ratio + edge_to_alt) / 2.0;

}

void TetFace::CorrectWinding() {

    auto edge0 = nodes_[1]->Position() - nodes_[0]->Position();
    auto edge1 = nodes_[2]->Position() - nodes_[0]->Position();

    auto normal = cross(edge0, edge1);
    auto out_vector = Centroid() - incident_tets_[0]->Centroid();

    if (normal.dot(out_vector) < 0.0) {
        auto temp = nodes_[1];
        nodes_[1] = nodes_[2];
        nodes_[2] = temp;
    }

}

}; // namespace destroyer