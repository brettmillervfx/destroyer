//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "Types.h"

#include <array>
#include <vector>

namespace destroyer {


/*************************************************************************************

 TetFace implements a triangle polygon representing a face of a tetrahedron. It is used primarily
 as a object describing topology, facilitating queries for connectivity between tetrahedrons in the mesh.

 A TetFace may be connected to one of two tetrahedrons. If fewer, ie. zero, the face is disconnected and invalid.
 If the face is connected to more that two tetrahedrons, it is non-manifold and invalid. Connectivity to
 a single tetrahedron indicates that the face is on the boundary of the mesh.

 TetFaces should generally only be created and deleted by the owner TetMesh.

*************************************************************************************/


class TetFace {
public:

    // Constructor requires the edges comprising the triangle. Assumes that the connectivity is handled by
    // Tetrahedron and/or TetMesh.
    explicit TetFace(TetEdgeRef e0, TetEdgeRef e1, TetEdgeRef e2);
    ~TetFace();

    // Return true if the face is on the TetMesh boundary.
    bool IsBoundary() const;

    // Return reference to TetNode at node_index position. Valid indices are 0-2.
    // Nodes are in an arbitrary order.
    inline TetNodeRef GetNodeRef(int node_index) const { return nodes_[node_index]; };

    // Return a reference to the first (and possibly only) tet connected to the face.
    TetrahedronRef GetFirstTet() const;

    // Return the tet that the face is connected to that is not the provided tet.
    // If the provided tet is invalid, or if the face is boundary, ie there is no ther tet,
    // this method returns a nullptr.
    TetrahedronRef GetOtherTet(TetrahedronRef tet) const;

    // Return the edge opposite the passed node on the face.
    TetEdgeRef GetOppositeEdge(TetNodeRef node) const;

    // Swap out one of the face's edges. Be careful with this on as there a lot of topology connections to manage.
    void ReplaceEdge(TetEdgeRef edge);

    // Swap out one of the face's nodes. Be careful with this on as there a lot of topology connections to manage.
    void ReplaceNode(TetNodeRef original, TetNodeRef replacement);

    // Manage tetrahedron connections. In practice, should anly be called from a Tetrahedron.
    void ConnectTetrahedron(TetrahedronRef tet);
    void DisconnectTetrahedron(TetrahedronRef tet);

    // The face is considered connected if there are incident tetrahedra.
    bool IsConnected() const;

    // Returns true if the specified edge is one of the edges comprising the face.
    bool HasEdge(TetEdgeRef edge) const;

    // Returns the face normal, oriented out from the center of the "owner_tet". If no owner tet is provided,
    // the returned normal will be from the center of the first connect tet (the only connected tet if the
    // face is boundary.)
    Vec3 Normal(TetrahedronRef owner_tet=nullptr) const;

    // Return the ratio of the shortest edge over the longest edge of the face.
    Real EdgeLengthRatio() const;

    // Altitude is defined as the orthogonal distance from a node to it's opposite edge.
    // Returns the shortest altitude of the face.
    Real MinNodeAltitude() const;

    // Return the longest edge length on the face.
    Real MaxEdgeLength() const;

    // Return the most oblique angle between edges on the face.
    Real MaxAngle() const;

    // Return the area of the face.
    Real Area() const;

    // Caclulate the face centroid.
    Vec3 Centroid() const;

    // Return the radius on the inscribed cirle, ie. the interior circle tangent to every edge.
    Real Inradius() const;

    // Return the radius of the circumscribed circle, ie. the circle which has every node on it's radius.
    Real Circumradius() const;

    // A useful measure: half the perimeter of the polygon.
    Real Semiperimeter() const;

    // Measure of quality of the triangle. Normalize from 0-1 where 1 is equilateral.
    Real QualityMeasure() const;

    // Fix the winding order of the face so that the calculated normal faces out from the first tetrahedron.
    void CorrectWinding();


private:
    std::vector<TetrahedronRef> incident_tets_;
    std::array<TetNodeRef,3> nodes_;
    std::array<TetEdgeRef,3> edges_;

};

}; // namespace destroyer
