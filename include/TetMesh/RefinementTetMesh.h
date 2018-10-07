//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh/TetMesh.h"

#include <set>


namespace destroyer {

/*************************************************************************************

 RefinementTetMesh class

 RefinementTetMesh is a specialization of TetMesh used to subdivide a subset of the TetMesh's tetrahedra.

 The technique used is described in ADAPTIVE PHYSICS BASED TETRAHEDRAL MESH GENERATION USING LEVEL SETS,
 Bridson et al.

 The algorithm makes use of a so-called red-green hierarchy. "Red" tetrahedra are subdivided "regularly" during
 refinement: each edge of the tetrahedron is split and 8 new tetrahedra are formed using this new configuration
 of nodes (see the paper for a useful illustration). The splitting of edges creates T-intersections on adjacent
 tetrahedra, which need to be resolved. These "irregularly" subdivided tets are called "green". The subdivision
 of a green tet falls into one of several categories: a template is rotated into proper configuration and the
 subdivision is performed (again, we refer to the paper for a useful illustration).

 The algorithm proceeds by marking an input group of tets as red and splitting the edge of these tets. Non-red
 tets with split edges are marked green. Certain configurations of green tets will produce better quality refinement
 if they are instead considered red. Such tets are further split, generating more green tets.

 Once all red and green tets are identified, and all necessary edge splits performed, the marked tets are subdivided.

 This class also provides a cleanup method. Refinement and tet culling may produce tets ad nodes that are non-manifold,
 or otherwise unsuitable for FEM simulation. The cleanup routine iteratively repairs these defects in the TetMesh.

*************************************************************************************/


class RefinementTetMesh : public TetMesh {
public:

    using TetMesh::TetMesh;

    ~RefinementTetMesh() = default;

    // Refine all tets identified by id. (This requires that id is set when the TetMesh is constructed).
    void RefineIdGroup(const std::set<Index> id_group);

    // Repair defective tets.
    // lone_tets: Disconnected tets are subdivided 8:1 to strengthen the structure for FEM solves.
    // weak_tets: Tets with all four nodes exposed to boundary are eliminated.
    // weak_edges: Interior edges with both nodes on the boundary are split to strengthen the tet structure.
    // nonmanifold_edges: Boundary edges with more than 2 incident boundary faces are eliminated.
    // nonmanifold_nodes: Boundary nodes with more than one boundary edge ring are eliminated.
    // If there are remaining defects after max_iterations is reached, returns false.
    bool Cleanup(bool lone_tets, bool weak_tets, bool weak_edges, bool nonmanifold_edges, bool nonmanifold_nodes, int max_iterations=2);


private:
    void PropagateRefinement();

    void SplitInteriorEdge(TetrahedronRef tet);

    void PrepTetForGreenRefinement(TetrahedronRef tet);

    void PrepTetForRedRefinement(TetrahedronRef tet);

    void SubdivideRefinedTets();

    void SubdivideTetrahedron(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronOne(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronTwo(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronThree(TetrahedronRef tet);

    // Returns the number of components corrected
    int RefineLoneTets();
    int RemoveWeakExteriorTets();
    int SplitWeakInteriorEdges();
    int RefineNonManifoldEdges();
    int RefineNonManifoldNodes();

private:
    std::set<TetrahedronRef> refine_list_;

};

}; // namespace destroyer
