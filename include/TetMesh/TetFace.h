//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <array>
#include <vector>

namespace destroyer {

class TetFace {
public:
    TetFace(TetEdgeRef e0, TetEdgeRef e1, TetEdgeRef e2);
    ~TetFace();

    // Return true if the face is on the TetMesh boundary.
    bool IsBoundary() const;

    // Return the edge opposite the passed node on the face.
    TetEdgeRef GetOppositeEdge(const TetNodeRef node) const;

    // Return the edge anti-clockwise from the passed node.
    TetEdgeRef GetRightEdge(TetNodeRef node) const;

    // Return the edge clockwise from the passed node.
    TetEdgeRef GetLeftEdge(TetNodeRef node) const;

    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    // The face is considered connected if there are incident tetrahedra.
    bool IsConnected() const;

    // Returns true if the specified edge is one of the edges comprising the face.
    bool HasEdge(TetEdgeRef edge) const;


private:
    std::vector<TetrahedronRef> incident_tets_;
    std::array<TetNodeRef,3> nodes_;
    std::array<TetEdgeRef,3> edges_;

};

}; // namespace destroyer
