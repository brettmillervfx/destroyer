//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <vector>
#include <random>

namespace destroyer {


enum SDFFlag {
    IN_SDF = 0,
    OUT_SDF = 1,
    ON_SDF= 2,
    NO_FLAG = 3
};

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
    void SetSDFFlag(SDFFlag flag);
    SDFFlag SdfFlag() const;

    bool ConnectedToSdfFlag(SDFFlag flag);

    bool IsConnected() const;

    bool IsBoundary() const;

    bool IsNonManifold() const;

    std::vector<TetEdgeRef> GetFirstEdgeRing() const;

    std::vector<TetrahedronRef> GetIncidentTets() const;
    std::vector<TetFaceRef> GetIncidentFaces() const;
    std::vector<TetEdgeRef> GetIncidentEdges() const;

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

    bool IsInverted() const;

    Real GetMinAltitude() const;
    Real GetMinEdgeLength() const;

    Real CalculateLocalQuality() const;

    void CacheLocalQuality();
    inline Real LocalQuality() const { return quality_; };

private:
    std::vector<TetEdgeRef> GetAllRingEdges() const;

private:
    Vec3 position_;
    Index id_;
    uint depth_;
    Real sdf_;
    Real quality_;
    SDFFlag sdf_flag_;
    std::vector<TetEdgeRef> incident_edges_;
    std::vector<TetFaceRef> incident_faces_;
    std::vector<TetrahedronRef> incident_tets_;

};

}; // namespace destroyer
