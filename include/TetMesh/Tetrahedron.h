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

enum EdgeIndex {
    EDGE_0_1 = 0,
    EDGE_0_2 = 1,
    EDGE_0_3 = 2,
    EDGE_1_2 = 3,
    EDGE_2_3 = 4,
    EDGE_1_3 = 5,
    NO_EDGE = 7,

};

enum FaceIndex {
    FACE_0_1_2 = 0,
    FACE_0_2_3 = 1,
    FACE_0_3_1 = 2,
    FACE_1_2_3 = 3
};


class Tetrahedron {
public:
    Tetrahedron(TetMeshRef tet_mesh, TetNodeRef n0, TetNodeRef n1, TetNodeRef n2, TetNodeRef n3, Index id=0);
    ~Tetrahedron();

    inline Index Id() const { return id_; };

    //inline Real Quality() const { return quality_; };

    // Returns true if the tetrahedron contains SDF interior.
    bool ContainsSolid(VDBSamplerPtr sdf_sampler, int recursion_depth) const;

    // Get the node reference assigned to the Tetrahedron's vertex at index i (range is 0 to 3).
    inline TetNodeRef GetNodeRef(int node_index) const { return nodes_[node_index]; };
    int GetNodeIndex(TetNodeRef node) const;

    // Get the edge between the two specified node indices.
    TetEdgeRef GetEdgeRef(int node0, int node1) const;
    TetEdgeRef GetEdgeRef(EdgeIndex index) const;

    TetNodeRef SplitEdge(EdgeIndex index);
    TetNodeRef SplitEdge(int node0, int node1);

    EdgeIndex GetEdgeIndex(TetEdgeRef edge) const;

    void ReplaceEdge(EdgeIndex index, TetEdgeRef edge);

    void ReplaceNode(TetNodeRef original, TetNodeRef replacement);

    bool HasFace(TetFaceRef face) const;
    bool SharesFaceWith(TetrahedronRef other) const;
    bool HasEdge(TetEdgeRef edge) const;
    bool SharesEdgeWith(TetrahedronRef other) const;

    // Get the 2 faces sharing an edge.
    std::array<TetFaceRef,2> GetFacesIncidentTo(EdgeIndex index) const;

    inline TetFaceRef GetFaceRef(int index) const { return faces_[index]; };

    // Return the number of boundary faces on this tet.
    int BoundaryFaceCount() const;

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

    bool IsInverted() const;

    Real EdgeLengthRatio() const;

    Vec3 Centroid() const;

    Real Altitude(int node_index) const;
    Real GetMinAltitude() const;
    MinMaxReal GetMinMaxEdgeLengths() const;
    MinMaxReal GetMinMaxDihedralAngles() const;

    TetEdgeRef ShortestEdge() const;

    //Real CalculateAspectRatio();

    Real Volume() const;
    Real Inradius() const;
    Real Circumradius() const;
    Real QualityMeasure() const;
    void CacheQualityMeasure();
    Real CachedQualityMeasure() const;

    int CountInNodes() const;
    std::vector<TetNodeRef> InNodes() const;
    int CountOnNodes() const;
    std::vector<TetNodeRef> OnNodes() const;
    int CountOutNodes() const;
    std::vector<TetNodeRef> OutNodes() const;

    inline void SetBoundarySplitCase(int split_case) { boundary_split_case_ = split_case; };
    inline int BoundarySplitCase() const { return boundary_split_case_; };

    void SplitBoundaryCrossingEdges();


private:
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
    Real cached_quality_measure_;

};

}; // namespace destroyer
