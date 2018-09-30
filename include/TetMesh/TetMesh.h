//
// Created by Brett Miller on 8/15/18.
//

/*************************************************************************************

 TetMesh is a structure for containing a tetrahedral mesh. It implements any topology
 required to perform its various functions.

 Clients of TetMesh should instantiate a single TetMesh object, fill it with nodes,
 use these nodes to define connected tetrahedra, then discard any unused nodes. It is entirely
 possible to create faulty, non-manifold meshes -- very little error checking is performed
 so it is on the client code to make sure tetrahedron configuration is valid.

 Internally, the TetMesh implements faces and edges for internal topological queries
 necessary to perform it's more advanced functions. Client code should never deal with
 such structures directly: only Tets and Nodes can be manipulated.

*************************************************************************************/


#pragma once

#include "Types.h"

#include <list>
#include <map>

namespace destroyer {

class TetMesh {
public:
    TetMesh();
    TetMesh(VDBSamplerPtr sampler);
    ~TetMesh();

    bool IsEmpty() const;

    void TearDown();

    TetNodeRef AddNode(Real x, Real y, Real z, Index id=0);
    TetEdgeRef AddEdge(TetNodeRef node0, TetNodeRef node1);
    TetFaceRef AddFace(TetEdgeRef edge0, TetEdgeRef edge1, TetEdgeRef edge2);
    TetrahedronRef AddTetrahedron(TetNodeRef n0, TetNodeRef n1, TetNodeRef n2, TetNodeRef n3, Index id=0);

    // Remove unconnected topology.
    void DeleteUnusedTopology();

    void DeleteTetrahedron(TetrahedronRef tet);

    // Remove all Tets that we believe exist outside the level set described by the sampler.
    // Culldepth determines the recursive face subdivision used when testing marginal cases.
    void CullOutsideTets(int cullDepth=2);

    // Iterate the nodes. To use, reset the iterator, then use NextNode to get a pointer
    // to the next TetNode. A nullptr is returned when the list is exhausted.
    void ResetNodeIterator();
    TetNodeRef NextNode();

    // Iterate the tets. To use, reset the iterator, then use NextTet to get a pointer
    // to the next Tetrahedron. A nullptr is returned when the list is exhausted.
    void ResetTetIterator();
    TetrahedronRef NextTet();

    // Iterate the faces. To use, reset the iterator, then use NextFace to get a pointer
    // to the next TetFace. A nullptr is returned when the list is exhausted.
    void ResetFaceIterator();
    TetFaceRef NextFace();


protected:
    std::list<TetNodePtr> nodes_;
    std::list<TetNodePtr>::iterator node_iter_;

    std::map<TetrahedronRef,TetrahedronPtr> tets_;
    std::map<TetrahedronRef,TetrahedronPtr>::iterator tet_iter_;

    std::map<TetFaceRef,TetFacePtr> faces_;
    std::map<TetFaceRef,TetFacePtr>::iterator face_iter_;

    std::map<TetEdgeRef,TetEdgePtr> edges_;

    VDBSamplerPtr sampler_;

};

}; // namespace destroyer
