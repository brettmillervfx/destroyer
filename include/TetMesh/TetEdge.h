//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <array>
#include <vector>


namespace destroyer {

/*

 TetEdge class

 TetEdges are objects created and maintained by the TetMesh class -- clients to TetMesh should have no requirement
 for interfacing with TetEdges.

 A TetEdge connects 2 adjacent nodes int the TetMesh. It will contribute to the definition of TetFaces and Tetrahedra
 by definition and maintains connectivity with TetFaces and Tetrahedrons.

 < How are TetEdges used in the code? >

 A TetEdge may also maintain a "Midpoint": this is a node that bisects the edge but is otherwise unconnected to
 the TetMesh structure. This is useful for defining what edges we would like to split during tet refinement, without
 immediately requiring Tetrahedron subdivision.

*/

class TetEdge {
public:

    // Construct a TetEdge by supplying two nodes comprising the endpoints.
    TetEdge(TetNodeRef n0, TetNodeRef n1);

    ~TetEdge();

    // Return the first found node. Which node is returned can be arbitrary
    // since the edge has no specific direction.
    TetNodeRef GetFirstNode() const;

    // Given a reference to one of the endpoints nodes, return the other endpoint.
    // If the edge does not contain the passed node, return nullptr.
    TetNodeRef GetOtherNode(TetNodeRef node) const;

    // Returns true if the passed node reference is one of the two end points of the edge.
    bool HasNode(TetNodeRef node) const;

    // Return the world space position between the nodal endpoints. Note that this does not require
    // that a Midpoint has been set, nor does it set the Midpoint. This is merely a geometric query.
    Vec3 MidpointPosition() const;

    // Returns true if the TetEdge has had a Midpoint set on it.
    bool HasMidpoint() const;

    // A node should be created (and registered with TetMesh) then passed to the edge to set the Midpoint for the edge.
    // Presumably, the position of the node should be set using the MidpointPosition vector.
    void SetMidpoint(TetNodeRef node);

    // If the edge has no midpoint, return nullptr.
    TetNodeRef Midpoint() const;

    // Methods that give access to the incident faces. The returned faces will be both internal and boundary.
    uint IncidentFaceCount() const;
    std::vector<TetFaceRef> GetIncidentFaces() const;

    // Methods that give access to only the boundary faces incident to the TetEdge.
    uint IncidentBoundaryFaceCount() const;
    std::vector<TetFaceRef> GetIncidentBoundaryFaces() const;

    // Return the incident face that also contains the specified edge.
    // If there is no face with the passed edge, return nullptr.
    TetFaceRef GetFaceWithEdge(TetEdgeRef edge) const;

    // Method that gives access to all Tetrahedra that use this TetEdge.
    std::vector<TetrahedronRef> GetIncidentTets() const;

    // The edge is considered connected if there are incident tets.
    bool IsConnected() const;

    // Return the Euclidean distance separating the edge's endpoints.
    Real Length() const;

    // Return the node as a 3d vector representation. Note that node ordering is
    // arbitrary, so this representtaion should only be used if the head and tail of the
    // vector are interchangable.
    Vec3 AsVector() const;

    // A non-manifold edge is one that lies on the boundary, but is connected to
    // more that two boundary faces. This implies two tetrahedra that share the edge
    // but share no faces.
    bool IsNonManifold() const;

    // Public methods that should only be used internally to the TetMesh structure. Clients of
    // TetMesh should have no requirement to manage connectivity relationships.
    void ConnectFace(TetFaceRef face);
    void DisconnectFace(TetFaceRef face);
    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    void ReplaceNode(TetNodeRef original, TetNodeRef replacement);

private:
    std::array<TetNodeRef,2> nodes_;
    std::vector<TetFaceRef> incident_faces_;
    std::vector<TetrahedronRef> incident_tets_;
    TetNodeRef midpoint_;

};

}; // namespace destroyer
