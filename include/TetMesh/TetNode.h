//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <vector>
#include <random>

namespace destroyer {

/*************************************************************************************

 TetNode implements a vertex in the tetrahedral mesh. The primary function is to define
 positions in cartesian space. It also provides additional methods for querying level set
 values, connectivity, etc.

 TetNodes should generally only be created and deleted by the owner TetMesh.

*************************************************************************************/


// Flag used during mesh clipping to indicate if the node is inside, outside
// or on the level set (within a tolerance). See ClipTetMesh for use.
enum SDFFlag {
    IN_SDF = 0,
    OUT_SDF = 1,
    ON_SDF= 2
};

class TetNode {
public:

    // Nodes are created by specifiying the 3d position. Id is optional and
    // useful when conditioning to and fromHoudini Detail.
    explicit TetNode(Real x, Real y, Real z, Index id=0);
    explicit TetNode(Vec3 position, Index id=0);
    ~TetNode() = default;

    // Position getter/setter.
    inline Vec3 Position() const { return position_; };
    inline void SetPosition(Vec3 position) { position_ = position; };

    // Id getter/setter.
    inline Index Id() const { return id_; };
    inline void SetId(Index id) { id_ = id; };

    // Sign distance field getter/setter.
    // Note that the Node has no knowledge of the VDBSampler.
    inline Real Sdf() const { return sdf_; };
    inline void SetSdf(Real sdf) { sdf_ = sdf; };

    // SDF Flag getter/setter.
    inline SDFFlag SdfFlag() const { return sdf_flag_; };
    inline void SetSDFFlag(SDFFlag flag) { sdf_flag_ = flag; };

    // Returns true if this node has an edge to a node with the
    // specified flag. Used in mesh clipping if we need to know if,
    // for instance, a node is connected to an "inside" node.
    bool ConnectedToSdfFlag(SDFFlag flag);

    // Returns true if the node is used by a tetrahedron.
    bool IsConnected() const;

    // Returns true if the node sits on the topological surface of the TetMesh.
    bool IsBoundary() const;

    // Returns true if the node represents a non-manifold point on the TetMesh boundary.
    // This is typically a case of the node being a connected saddle point between two surfaces.
    bool IsNonManifold() const;

    // Generate the list of edges encircling the node on the boundary. This method is only valid
    // for boundary nodes and will return an empty vector for interior nodes. Note also that a node may in fact
    // have more than one edge ring, but that it is considered non-manifold if it does.
    std::vector<TetEdgeRef> GetFirstEdgeRing() const;

    // Query topological connections/
    inline std::vector<TetrahedronRef> GetIncidentTets() const { return incident_tets_; };
    inline std::vector<TetFaceRef> GetIncidentFaces() const { return incident_faces_; };
    inline std::vector<TetEdgeRef> GetIncidentEdges() const { return incident_edges_; };

    // Returns the edge incident to this node that is connected to the passed node.
    // If this node is not connected to the passed node, return nullptr.
    TetEdgeRef GetEdgeTo(TetNodeRef node) const;

    // Register and deregister tetrahedron connections.
    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    // Register and deregister face connections.
    void ConnectFace(TetFaceRef face);
    void DisconnectFace(TetFaceRef face);

    // Register and deregister edge connections.
    void ConnectEdge(TetEdgeRef edge);
    void DisconnectEdge(TetEdgeRef edge);

    // return the surface normal at boundary nodes. This method is invalid if called on an interior node.
    Vec3 Normal() const;

    // Returns true if the position of the node inverts any of it's incident tetrahedrons.
    bool IsInverted() const;

    // Altitude is defined as the orthogonal distance from this node to the plane of the opposite face in
    // a tetrahedron. This method returns the smallest altitude found for this node in all of it's incident tets.
    Real GetMinAltitude() const;

    // Returns the length of the shortest connected edge.
    Real GetMinEdgeLength() const;

    // The node's local quality is defined as the lowest quality of any of the incident tets.
    Real CalculateLocalQuality() const;

    // Calculating node qulaity can be expensive, so a caching mechanism is provided.
    inline void CacheLocalQuality() { quality_ = CalculateLocalQuality(); };
    inline Real LocalQuality() const { return quality_; };

private:
    std::vector<TetEdgeRef> GetAllRingEdges() const;

private:
    Vec3 position_;
    Index id_;
    Real sdf_;
    Real quality_;
    SDFFlag sdf_flag_;
    std::vector<TetEdgeRef> incident_edges_;
    std::vector<TetFaceRef> incident_faces_;
    std::vector<TetrahedronRef> incident_tets_;

};

}; // namespace destroyer
