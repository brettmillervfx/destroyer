//
// Created by Brett Miller on 8/15/18.
//

// https://en.wikipedia.org/wiki/Tetrahedron#More_vector_formulas_in_a_general_tetrahedron

#pragma once

#include "TetMesh/Types.h"

#include <array>
#include <vector>
#include <bitset>

namespace destroyer {

/*************************************************************************************

 Tetrahedron class

 Implements an tetrahedron used by TetMesh to define topology. A Tetrahedron is primarily defined by the
 4 TetNode at it's vertices (which it shares with other tets in the TetMesh). It also maintains connectivty
 relationships with other tetrahedrons via TetFaces and TetEdges.

 A Tetrahedron is responsible for creating (and registering) faces and edges, but does not manage their
 allocations, not does it manage it's own pointer or those of the nodes that define it.

 Tetrahedrons manage splitting and subdivision using the categories "red" and "green" as defined in
 ADAPTIVE PHYSICS BASED TETRAHEDRAL MESH GENERATION USING LEVEL SETS, Bridson et al.
 From the paper: "The general idea of a red green hierarchy is to regularly (red) refine any tetrahedron
 where more resolution is required, and then irregularly (green) refine tetrahedra to restore the mesh to
 a valid simplicial complex." A tetrahedron set as "red" will have each of it's edges split and will be refined
 into 8 resulting tetrahedra. The splits create T-junctions in adjacent tetrahedra. These tets are labelled
 "green" and flagged for irregular refinement.

 The Tetrahedron itself does not manage splitting but does track the categorization.

*************************************************************************************/


// A tetrahedron has 6 edges. The edges are named by the node indices they connect.
enum EdgeIndex {
    EDGE_0_1 = 0,
    EDGE_0_2 = 1,
    EDGE_0_3 = 2,
    EDGE_1_2 = 3,
    EDGE_2_3 = 4,
    EDGE_1_3 = 5,
    NO_EDGE = 7,

};

// A tetrahedron has 4 faces. The faces are named for the node indices they connect.
enum FaceIndex {
    FACE_0_1_2 = 0,
    FACE_0_2_3 = 1,
    FACE_0_3_1 = 2,
    FACE_1_2_3 = 3
};


class Tetrahedron {
public:

    // Constructor requires a pointer to the owner TetMesh, pointers to the TetNodes defining the vertices, and
    // and optional id (useful for maintaining the id of a tet prim conditioned from a Houdini Detail, for instance).
    // Given the nodes, the remaining topology and connectivity are established internally, ie. the Tetrahedron
    // creation communicates with the owner TetMesh to handle all topology -- the client needs only to supply TetMesh
    // and TetNodes.
    Tetrahedron(TetMeshRef tet_mesh, TetNodeRef n0, TetNodeRef n1, TetNodeRef n2, TetNodeRef n3, Index id=0);
    ~Tetrahedron();

    inline Index Id() const { return id_; };

    // Returns true if the tetrahedron contains SDF interior.
    bool ContainsSolid(VDBSamplerPtr sdf_sampler, int recursion_depth) const;

    // Get the node reference assigned to the Tetrahedron's vertex at index i (range is 0 to 3).
    inline TetNodeRef GetNodeRef(int node_index) const { return nodes_[node_index]; };
    int GetNodeIndex(TetNodeRef node) const;

    // Get the edge between the two specified node indices.
    TetEdgeRef GetEdgeRef(int node0, int node1) const;
    TetEdgeRef GetEdgeRef(EdgeIndex index) const;

    // "Split" the specified edge or the edge that connects the specified nodes (indicate by their indices in
    // the tetrahedron. See TetEdge for details regarding splitting TetEdges.
    TetNodeRef SplitEdge(EdgeIndex index);
    TetNodeRef SplitEdge(int node0, int node1);

    // Find referred edge in the tet and return it's tet-specific index.
    EdgeIndex GetEdgeIndex(TetEdgeRef edge) const;

    // Replace the edge at the specified index with the supplied new edge. The method will also handle
    // replacing the edge in the associated faces.
    void ReplaceEdge(EdgeIndex index, TetEdgeRef edge);

    // Replace the specified "original" node with a replacement.
    void ReplaceNode(TetNodeRef original, TetNodeRef replacement);

    // Returns true if the face is one of the 4 tet faces.
    bool HasFace(TetFaceRef face) const;

    // Returns true if this tet and the "other" tet have a face in common, ie. they are incident.
    bool SharesFaceWith(TetrahedronRef other) const;

    // Returns true if the edge is one of the 6 tet edges.
    bool HasEdge(TetEdgeRef edge) const;

    // Returns true if this tet and the "other" tet have an edge in common, ie. they are incident (at least) along an edge.
    bool SharesEdgeWith(TetrahedronRef other) const;

    // Get the 2 faces (in this tet) sharing an edge.
    std::array<TetFaceRef,2> GetFacesIncidentTo(EdgeIndex index) const;

    // Return the face at the requested index. (Valid indice are 0-3).
    inline TetFaceRef GetFaceRef(int index) const { return faces_[index]; };

    // Return the number of boundary faces on this tet.
    int BoundaryFaceCount() const;

    // Return the number of boundary nodes on this tet.
    int BoundaryNodeCount() const;

    // Update edge splitting configuration. This method should be called before making queries
    // regarding edge split counts or green/red categorization.
    void ClassifySplitEdgeConfiguration();

    // Query if a particular edge (specified by it's bitflag) is split in the tetrahedron.
    inline bool IsEdgeSplit(int bitflag) const { return split_edges_bitmask_[bitflag]; };

    // Returns the number of edges of the tetrahedron currently split.
    inline int GetSplitEdgeCount() const { return split_edges_bitmask_.count(); };

    // A tetrahedron requires edge splitting if it has two edge splits but the split edges are adjacent.
    // Returns true if such refinement is necessary, false otherwise.
    bool RequiresGreenPrep() const;

    // If exactly two edges are split, return the edge code for the third edge comprising a completed edge loop.
    // If there are not two edges split, or the two edges are opposite, returns NO_EDGE.
    EdgeIndex ThirdEdgeToCompleteSplitFace();

    // A "Red" tetrahedron is one that will be refined regularly, ie. all edges are split and the tet
    // is subdivided 1:8. We prefer "Green" refinement when possible.
    // Returns true if the tet is not already a Red tet and is unsuitable for Green refinement.
    bool IsPotentialRed() const;

    // Returns true if a node move has inverted the tetrahedron.
    bool IsInverted() const;

    // Return the ratio of the shortest edge length over the longest edge length.
    // This can be used as a quality heuristic.
    Real EdgeLengthRatio() const;

    // Return the centroid of the 4 tet nodes.
    Vec3 Centroid() const;

    // Altitude is defined as the orthogonal distance from a node to the plane of the opposite face.
    Real Altitude(int node_index) const;

    // Find the smallest altitude in the tetrahedron.
    Real GetMinAltitude() const;

    // Return a tuple of the shortest and longest edge in the tetrahedron.
    MinMaxReal GetMinMaxEdgeLengths() const;

    // REturn a tuple of the most acute and most oblique angles between faces.
    MinMaxReal GetMinMaxDihedralAngles() const;

    // Return a reference to the shortest edge in the tetrahedron/
    TetEdgeRef ShortestEdge() const;

    // Calculate the volume of the tetrahedorn.
    Real Volume() const;

    // Calculate the radius of the inscribed sphere, ie. the sphere tangent to all faces of the tet.
    Real Inradius() const;

    // Calculate the radius of the circumscribed sphere, ie. the sphere upon which all nodes of the tet lie.
    Real Circumradius() const;

    // Calculate the quality heuristic. We use the ratio of the area of the inscribed sphere to the area of the
    // curcumscribed sphere, normalized so that an ideal regular tetrahedron has quality 1 and a flattened tet has 0.
    Real QualityMeasure() const;

    // Nodes may be flagged as inside, on or outside the level set (see TetNodes). Setting this flag happens bia the
    // TetNode interface. However it is often useful to know how many of the Tetrahedron's nodes fall in each category
    // and specifically which ones.
    int CountInNodes() const;
    std::vector<TetNodeRef> InNodes() const;
    int CountOnNodes() const;
    std::vector<TetNodeRef> OnNodes() const;
    int CountOutNodes() const;
    std::vector<TetNodeRef> OutNodes() const;

    // Boundary split case getter/setter. This is a method used by ClipTetMesh to record and later
    // recall the subdivision scheme to use for the tetrahedron when clipping it to a plane.
    inline void SetBoundarySplitCase(int split_case) { boundary_split_case_ = split_case; };
    inline int BoundarySplitCase() const { return boundary_split_case_; };

    // Once the tet's nodes are properly SDFFlagged, this moethod splits all edges that cross from
    // inside the level set to outside.
    void SplitBoundaryCrossingEdges();


private:
    void CorrectWinding();
    void GetNewEdges();
    void GetNewFaces();

    bool IsOutsideSDFNarrowband() const;
    bool IsCompletelyInsideLevelSet() const;
    bool CrossesLevelSet() const;
    bool EncroachedByLevelSet(int recursion_depth, const VDBSamplerPtr sdf_sampler,
                              const Vec3 &vertex0, const Vec3 &vertex1, const Vec3 &vertex2) const;

private:
    Index id_;
    TetMeshRef tet_mesh_;
    std::array<TetNodeRef,4> nodes_;
    std::array<TetEdgeRef,6> edges_;
    std::array<TetFaceRef,4> faces_;
    std::bitset<6> split_edges_bitmask_;
    int boundary_split_case_;

};

}; // namespace destroyer
