//
// Created by Brett Miller on 8/15/18.
//

/*************************************************************************************

 TetMesh implementation

 TetMesh owns all topology objects, ie. Nodes, Tets, Edges, and Faces. TetMesh owns all objects (as Ptrs)
 and the other objects are "connected" to each other via Refs. Only a Ptr may be deleted. Refs are essentially
 weak pointers to Ptrs and must be handled carefully. Generally, only the TetMesh may allocate and de-allocate
 topology objects. Tetrahedrons are an exception, however. When a Tetrahedron is created, it is passed a reference
 to it's owner TetMesh. The Tetrahedron allocates it's edges and faces and registers the objects with the owner
 TetMesh. The TetMesh becomes the owner of the new topology and is responsible for de-allocation.

 Tetrahedrons connect to:
    4 TetNodes, 6 TetEdges, 4 TetFaces

 TetEdges connect to:
    2 TetNodes, any number of TetFaces, any number of Tetrahedrons

 TetFaces connect to:
    3 TetEdges, 3 TetNodes, 1 or 2 Tetrahedrons

 TetNodes connect to:
    any number of TetEdges, any number of TetFaces, any number of Tetrahedrons.


 When the TetMesh creates a TetNode...
    A new TetNode is added without any connections.

 When the TetMesh deletes a TetNode...
    The TetNode is simply deallocated. All topological references shpould be resolved before this is invoked.

 When the TetMesh creates a Tetrahedron...
    Tetrahedron connects nodes to itself.
    Tetrahedron connects itself to the nodes.
    Tetrahedron determines if edges already exist between nodes and has TetMesh instantiate missing edges.
        Tetrahedron connects edges to itself.
        Tetrahedron connects itself to edges.
    Tetrahedron determines if faces already exist between edges and has TetMesh instantiate missing faces.
        Tetrahedron connects faces to itself.
        Tetrahedron connects itself to faces.
        New faces that are created are passed back to TetMesh.

 When Tetrahedrons are deleted...
    Tetrahedron disconnects itself from faces.
    Tetrahedron disconnects itself from edges.
    Tetrahedron disconnects itself from nodes.

 When TetFaces are created...
    TetFace connects edges to itself.
    TetFace connects itself to edges.
    TetFace gathers nodes from edges and
        TetFace connects nodes to itself.
        TetFace connects itself to nodes.

 When TetFaces are deleted...
    TetFace disconnects itself from nodes.
    TetFace disconnects itself from edges.

 When TetEdges are created...
    TetEdge connects nodes to itself.
    TetEdge connects itself to nodes.

 When TetEdges are deleted...
    TetEdge disconnects itself from nodes.

 Following Tetrahedron deletions, Client should DeleteUnusedTopology to sweep out any disconnected objects.


*************************************************************************************/


#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/VDBSampler.h"

namespace destroyer {

TetMesh::TetMesh() {

    sampler_ = nullptr;

}

TetMesh::TetMesh(VDBSamplerPtr sampler) {

    sampler_ = sampler;

}

TetMesh::~TetMesh() {

    TearDown();

}

bool TetMesh::IsEmpty() const {

    // No nodes == no topology
    return nodes_.empty();

}

void TetMesh::TearDown() {

    // Important to do this in the correct order.
    tets_.clear();
    faces_.clear();
    edges_.clear();
    nodes_.clear();

}

TetNodeRef TetMesh::AddNode(Real x, Real y, Real z, Index id) {

    // Add a new Node without any topological connections.

    auto new_node_ref = new TetNode(x,y,z,id);

    if (sampler_ != nullptr)
        new_node_ref->SetSdf(sampler_->Sample(x,y,z));

    nodes_.push_back(TetNodePtr(new_node_ref));

    return new_node_ref;

}

TetrahedronRef TetMesh::AddTetrahedron(TetNodeRef n0,
                                       TetNodeRef n1,
                                       TetNodeRef n2,
                                       TetNodeRef n3,
                                       Index id) {

    // The new Tetrahedron will create Edges and Faces in this TetMesh upon instantiation.
    auto new_tet_ref = new Tetrahedron(this, n0, n1, n2, n3, id);

    // Register the new tetrahedron.
    tets_[new_tet_ref] = TetrahedronPtr(new_tet_ref);

    return new_tet_ref;

}

void TetMesh::DeleteUnusedTopology() {

    // Eliminate disconnected faces.
    auto faces_iter = faces_.begin();
    while(faces_iter != faces_.end()) {
        if (!faces_iter->second->IsConnected()) {
            faces_iter = faces_.erase(faces_iter);
        } else {
            faces_iter++;
        }
    }

    // Eliminiate disconnected edges.
    auto edges_iter = edges_.begin();
    while(edges_iter != edges_.end()) {
        if (!edges_iter->second->IsConnected()) {
            edges_iter = edges_.erase(edges_iter);
        } else {
            edges_iter++;
        }
    }

    // Eliminate disconnected nodes.
    auto nodes_iter = nodes_.begin();
    while(nodes_iter != nodes_.end()) {
        if (!(*nodes_iter)->IsConnected()) {
            nodes_iter = nodes_.erase(nodes_iter);
        } else {
            nodes_iter++;
        }
    }

}

void TetMesh::DeleteTetrahedron(TetrahedronRef tet) {

    tets_.erase(tet);

}

void TetMesh::CullOutsideTets(int cull_depth) {

    // This function is essentially meaningless without a level set.
    if (sampler_ == nullptr)
        return;

    auto iter = tets_.begin();
    while (iter != tets_.end()) {

        bool contains_solid = iter->second->ContainsSolid(sampler_, cull_depth);
        if (contains_solid) {
            iter++;
        } else {
            iter = tets_.erase(iter);
        }

    }

    DeleteUnusedTopology();

}

void TetMesh::ResetNodeIterator() {

    node_iter_ = nodes_.begin();

}

TetNodeRef TetMesh::NextNode() {

    if (node_iter_ == nodes_.end())
        return nullptr;
    else {
        auto ret = node_iter_->get();
        ++node_iter_;
        return ret;
    }

}

void TetMesh::ResetTetIterator() {

    tet_iter_ = tets_.begin();

}

TetrahedronRef TetMesh::NextTet() {

    if (tet_iter_ == tets_.end())
        return nullptr;
    else {
        auto ret = tet_iter_->first;
        ++tet_iter_;
        return ret;
    }

}

void TetMesh::ResetFaceIterator() {

    face_iter_ = faces_.begin();

}

TetFaceRef TetMesh::NextFace() {

    if (face_iter_ == faces_.end())
        return nullptr;
    else {
        auto ret = face_iter_->first;
        ++face_iter_;
        return ret;
    }

}

TetEdgeRef TetMesh::AddEdge(TetNodeRef node0, TetNodeRef node1) {

    auto new_edge_ref = new TetEdge(node0, node1);

    edges_[new_edge_ref] = TetEdgePtr(new_edge_ref);

    return new_edge_ref;

}

TetFaceRef TetMesh::AddFace(TetEdgeRef edge0, TetEdgeRef edge1, TetEdgeRef edge2) {

    auto new_face_ref = new TetFace(edge0, edge1, edge2);

    faces_[new_face_ref] = TetFacePtr(new_face_ref);

    return new_face_ref;

}

}; // namespace destroyer