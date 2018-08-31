//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <vector>

namespace destroyer {

class TetNode {
public:
    TetNode(Real x, Real y, Real z, Index id=0);
    TetNode(Vec3 position, Index id=0);
    ~TetNode() = default;

    Vec3 Position() const;
    void SetPosition(Vec3 position);

    Index Id() const;
    void SetId(Index id);

    uint Depth() const;
    void SetDepth(uint depth);

    Real Sdf() const;
    void SetSdf(Real sdf);

    bool IsConnected() const;

    bool IsBoundary() const;

    bool IsNonManifold() const;

    std::vector<TetEdgeRef> GetFirstEdgeRing() const;

    std::vector<TetrahedronRef> GetIncidentTets() const;

    std::vector<TetFaceRef> GetIncidentFaces() const;

    // Returns the edge incident to this node that is connected to the passed node.
    // If this node is not connected to the passed node, return nullptr.
    TetEdgeRef GetEdgeTo(TetNodeRef node) const;

    // Register and deregister tetrahedron connections.
    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    // Register and deregister faces.
    void ConnectFace(TetFaceRef face);
    void DisconnectFace(TetFaceRef face);

    // Register and deregister edges.
    void ConnectEdge(TetEdgeRef edge);
    void DisconnectEdge(TetEdgeRef edge);

    Vec3 Normal() const;

private:
    std::vector<TetEdgeRef> GetAllRingEdges() const;

private:
    Vec3 position_;
    Index id_;
    uint depth_;
    Real sdf_;
    std::vector<TetEdgeRef> incident_edges_;
    std::vector<TetFaceRef> incident_faces_;
    std::vector<TetrahedronRef> incident_tets_;

};

}; // namespace destroyer
