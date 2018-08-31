//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <array>
#include <vector>

namespace destroyer {

class TetEdge {
public:
    TetEdge(TetNodeRef n0, TetNodeRef n1);
    ~TetEdge();

    // Return the first found node. Which node is returned can be arbitrary since the edge has no specific direction.
    TetNodeRef GetFirstNode() const;

    // Provided with one of the nodes in the edge, return the other end.
    // If the edge does not contain the passed node, return nullptr.
    TetNodeRef GetOtherNode(TetNodeRef node) const;

    // Returns true if the passed node is one of the two end points of the edge.
    bool HasNode(TetNodeRef node) const;

    bool HasMidpoint() const;
    Vec3 MidpointPosition() const;
    void SetMidpoint(TetNodeRef node);

    // If the edge has no midpoint, return nullptr.
    TetNodeRef Midpoint() const;

    uint IncidentFaceCount() const;
    std::vector<TetFaceRef> GetIncidentFaces() const;

    uint IncidentBoundaryFaceCount() const;
    std::vector<TetFaceRef> GetIncidentBoundaryFaces() const;

    // Return the incident face that also contains the specified edge.
    // If there is no face with the passed edge, return nullptr.
    TetFaceRef GetFaceWithEdge(TetEdgeRef edge) const;

    std::vector<TetrahedronRef> GetIncidentTets() const;

    // The edge is considered connected if there are incident tets.
    bool IsConnected() const;

    void ConnectFace(TetFaceRef face);
    void DisconnectFace(TetFaceRef face);

    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    Real Length() const;
    Vec3 AsVector() const;

    bool IsNonManifold() const;


private:
    std::array<TetNodeRef,2> nodes_;
    std::vector<TetFaceRef> incident_faces_;
    std::vector<TetrahedronRef> incident_tets_;
    TetNodeRef midpoint_;

};

}; // namespace destroyer
