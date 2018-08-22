//
// Created by Brett Miller on 8/15/18.
//

#include <iostream>

#include "TetMesh/TetFace.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetNode.h"

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

TetEdgeRef TetFace::GetOppositeEdge(TetNodeRef node) const {

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

}; // namespace destroyer