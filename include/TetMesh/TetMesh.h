//
// Created by Brett Miller on 8/15/18.
//

/*************************************************************************************

 TetMesh is a structure for containing a lattice of connected tetrahedrons. It implements any topology
 required to perform its various functions.

 Clients of TetMesh should instantiate a single TetMesh object, fill it with nodes,
 use these nodes to define connected tetrahedra, then discard any unused nodes. It is entirely
 possible to create faulty, non-manifold meshes -- very little error checking is performed
 so it is on the client code to make sure tetrahedron configuration is valid.

 Internally, the TetMesh implements faces and edges for internal topological queries
 necessary to perform it's more advanced functions. Client code should avoid dealing with
 such structures directly: only Tets and Nodes can be manipulated.

*************************************************************************************/


#pragma once

#include "Types.h"
#include "TetMesh/Tetrahedron.h"

#include <list>
#include <map>

namespace destroyer {

class TetMesh {
public:

    // TetMesh may be constructed with a VDBSampler defining the level set that the TetMesh will
    // tetrahedralize. A TetMesh without a sampler can still be very useful but many important operations,
    // such as culling outside tets, will be unavailable.
    explicit TetMesh();
    explicit TetMesh(VDBSamplerPtr sampler);

    ~TetMesh();

    // Returns true if the TetMesh contains no topology.
    bool IsEmpty() const;

    // Deallocate all of the internal topology in correct order.
    void TearDown();

    // Calling AddNode and AddTetrahedron are the primary way to build a TetMesh. A Node may be added to the TetMesh
    // using AddNode. The client should record the returned reference: Tetrahedrons may be added by providing
    // those references. See BCCLatticeGenerator implementation for a clear example.
    // Id is optional, useful for conditioning to and from Houdini Detail.
    TetNodeRef AddNode(Real x, Real y, Real z, Index id=0);
    TetrahedronRef AddTetrahedron(TetNodeRef n0, TetNodeRef n1, TetNodeRef n2, TetNodeRef n3, Index id=0);

    // Remove unconnected topology like disconnected nodes.
    void DeleteUnusedTopology();

    // Remove the tetrahedron from the mesh and eliminate unnecessary topology.
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
    // Tetrahedron is allowed access to TetMesh for purposed of registering new topology.
    TetEdgeRef AddEdge(TetNodeRef node0, TetNodeRef node1);
    TetFaceRef AddFace(TetEdgeRef edge0, TetEdgeRef edge1, TetEdgeRef edge2);
    friend Tetrahedron;


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
