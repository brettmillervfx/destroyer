//
// Created by Brett Miller on 8/15/18.
//

#include "TetMesh/Tetrahedron.h"

#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/VDBSampler.h"

namespace destroyer {

// Edge and Face identification tables

// Given 2 nodes, this table provides the connecting edge.
constexpr EdgeIndex NODE_TABLE[4][4] =
        {   { NO_EDGE, EDGE_0_1, EDGE_0_2, EDGE_0_3 },
            { EDGE_0_1, NO_EDGE, EDGE_1_2, EDGE_1_3 },
            { EDGE_0_2, EDGE_1_2, NO_EDGE, EDGE_2_3 },
            { EDGE_0_3, EDGE_1_3, EDGE_2_3, NO_EDGE } };

// Given an edge, this table provides the comprising nodes.
constexpr std::array<std::array<int,2>,6> EDGE_TABLE {{
    {{0,1}}, {{0,2}}, {{0,3}}, {{1,2}}, {{2,3}}, {{1,3}} }};

// Given a face, this table provides the comprising edges.
constexpr std::array<std::array<EdgeIndex,3>,4> FACE_TABLE {{
    {{ EDGE_0_1, EDGE_1_2, EDGE_0_2}},
    {{ EDGE_0_2, EDGE_2_3, EDGE_0_3}},
    {{ EDGE_0_3, EDGE_1_3, EDGE_0_1}},
    {{ EDGE_1_2, EDGE_2_3, EDGE_1_3}} }};

// Given an edge, this table provides the two face that share that edge.
constexpr std::array<std::array<FaceIndex,2>,6> INCIDENT_FACE_TABLE {{
    {{FACE_0_1_2, FACE_0_3_1}},
    {{FACE_0_1_2, FACE_0_2_3}},
    {{FACE_0_2_3, FACE_0_3_1}},
    {{FACE_0_1_2, FACE_1_2_3}},
    {{FACE_0_2_3, FACE_1_2_3}},
    {{FACE_0_3_1, FACE_1_2_3}} }};

// Given a node, this table provides the three connected edges.
constexpr std::array<std::array<EdgeIndex,3>,4> ADJACENT_EDGES_TABLE {{
    {{ EDGE_0_1, EDGE_0_2, EDGE_0_3}},
    {{ EDGE_1_2, EDGE_1_3, EDGE_0_1}},
    {{ EDGE_0_2, EDGE_1_2, EDGE_2_3}},
    {{ EDGE_0_3, EDGE_1_3, EDGE_2_3}} }};

// Given a node, this table provides the three connected faces.
constexpr std::array<std::array<FaceIndex,3>,4> ADJACENT_FACES_TABLE {{
    {{ FACE_0_1_2, FACE_0_2_3, FACE_0_3_1}},
    {{ FACE_0_1_2, FACE_0_3_1, FACE_1_2_3}},
    {{ FACE_0_1_2, FACE_0_2_3, FACE_1_2_3}},
    {{ FACE_0_2_3, FACE_0_3_1, FACE_1_2_3}} }};

// Given two edges, this table provides the edge that completes the
// loop, ie. the last co-planar edge completing the face.
constexpr EdgeIndex EDGE_COMPLETE_TABLE[7][7] =
        //    EDGE_0_1  EDGE_0_2  EDGE_0_3  EDGE_1_2  EDGE_2_3  EDGE_1_3  NO_EDGE
        {   { NO_EDGE,  EDGE_1_2, EDGE_1_3, EDGE_0_2, NO_EDGE,  EDGE_0_3, NO_EDGE,  },   // EDGE_0_1
            { EDGE_1_2, NO_EDGE,  EDGE_2_3, EDGE_0_1, EDGE_0_3, NO_EDGE,  NO_EDGE,  },   // EDGE_0_2
            { EDGE_1_3, EDGE_2_3, NO_EDGE,  NO_EDGE,  EDGE_0_2, EDGE_0_1, NO_EDGE,  },   // EDGE_0_3
            { EDGE_0_2, EDGE_0_1, NO_EDGE,  NO_EDGE,  EDGE_1_3, EDGE_2_3, NO_EDGE,  },   // EDGE_1_2
            { NO_EDGE,  EDGE_0_3, EDGE_0_2, EDGE_1_3, NO_EDGE,  EDGE_1_2, NO_EDGE,  },   // EDGE_2_3
            { EDGE_0_3, NO_EDGE,  EDGE_0_1, EDGE_2_3, EDGE_1_2, NO_EDGE,  NO_EDGE,  },   // EDGE_1_3
            { NO_EDGE,  NO_EDGE,  NO_EDGE,  NO_EDGE,  NO_EDGE,  NO_EDGE,  NO_EDGE,  },   // NO_EDGE
        };


///////////////////////////////////////////////


Tetrahedron::Tetrahedron(TetMeshRef tet_mesh, TetNodeRef n0, TetNodeRef n1, TetNodeRef n2, TetNodeRef n3, Index id) {

    tet_mesh_ = tet_mesh;
    id_ = id;

    nodes_[0] = n0;
    n0->ConnectTetrahedron(this);
    nodes_[1] = n1;
    n1->ConnectTetrahedron(this);
    nodes_[2] = n2;
    n2->ConnectTetrahedron(this);
    nodes_[3] = n3;
    n3->ConnectTetrahedron(this);

    GetNewEdges();
    GetNewFaces();

}

Tetrahedron::~Tetrahedron() {

    for (auto& face: faces_) {
        face->DisconnectTetrahedron(this);
    }

    for (auto& edge: edges_) {
        edge->DisconnectTetrahedron(this);
    }

    for (auto& node: nodes_) {
        node->DisconnectTetrahedron(this);
    }

}

bool Tetrahedron::ContainsSolid(VDBSamplerPtr sdf_sampler, int recursion_depth) const {

    if (IsOutsideSDFNarrowband())
        return false;

    if (IsCompletelyInsideLevelSet())
        return true;

    if (CrossesLevelSet())
        return true;


    // Possibly the level set intersects the tet while all nodes are outside. This can
    // be detected if the smallest phi is smaller than the longest edge length.
    // If his condition is met, we recursively subdivide the faces to detect possible level set incursion.
    Real min_unsigned_distance = std::numeric_limits<Real>::max();
    for (auto node : nodes_) {
        if (node->Sdf() < min_unsigned_distance) {
            min_unsigned_distance = node->Sdf();
        }
    }
    MinMaxReal edgeLengths = GetMinMaxEdgeLengths();
    if (min_unsigned_distance < edgeLengths[1]) {
        auto vertex0 = nodes_[0]->Position();
        auto vertex1 = nodes_[1]->Position();
        auto vertex2 = nodes_[2]->Position();
        auto vertex3 = nodes_[3]->Position();
        if (EncroachedByLevelSet(recursion_depth, sdf_sampler, vertex0, vertex1, vertex2))
            return true;
        if (EncroachedByLevelSet(recursion_depth, sdf_sampler, vertex1, vertex2, vertex3))
            return true;
        if (EncroachedByLevelSet(recursion_depth, sdf_sampler, vertex2, vertex3, vertex0))
            return true;
        if (EncroachedByLevelSet(recursion_depth, sdf_sampler, vertex3, vertex0, vertex1))
            return true;
    }



    return false;

}

int Tetrahedron::GetNodeIndex(TetNodeRef node) const {

    int ret_index=0;
    for (;ret_index<4;ret_index++) {
        if (nodes_[ret_index] == node)
            break;
    }

    return (ret_index<4) ? ret_index : -1;

}

TetEdgeRef Tetrahedron::GetEdgeRef(int node0, int node1) const {

    auto edge_index = NODE_TABLE[node0][node1];
    if (edge_index == NO_EDGE) {
        return nullptr;
    } else
        return edges_[edge_index];

}

TetEdgeRef Tetrahedron::GetEdgeRef(EdgeIndex index) const {

    return edges_[index];

}

TetNodeRef Tetrahedron::SplitEdge(EdgeIndex index) {

    if (!edges_[index]->HasMidpoint()) {
        auto mid = edges_[index]->MidpointPosition();
        auto mid_ref = tet_mesh_->AddNode(mid[0], mid[1], mid[2]);
        edges_[index]->SetMidpoint(mid_ref);
    }
    return edges_[index]->Midpoint();

}

TetNodeRef Tetrahedron::SplitEdge(int node0, int node1) {

    auto edge_index = NODE_TABLE[node0][node1];
    return SplitEdge(edge_index);

}

EdgeIndex Tetrahedron::GetEdgeIndex(TetEdgeRef edge) const {

    for (auto index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3}) {
        if (edges_[index] == edge)
            return index;
    }
    return NO_EDGE;

}

void Tetrahedron::ReplaceEdge(EdgeIndex index, TetEdgeRef edge) {

    // If the edge already matches, we have nothing to do.
    if (edges_[index] != edge) {

        // First make changes to the incident faces.
        auto faces = GetFacesIncidentTo(index);
        faces[0]->ReplaceEdge(edge);
        faces[1]->ReplaceEdge(edge);

        // Then replace the edge.
        edges_[index]->DisconnectTetrahedron(this);
        edges_[index] = edge;
        edges_[index]->ConnectTetrahedron(this);

    }

}

bool Tetrahedron::HasFace(TetFaceRef face) const {

    for (auto& f: faces_)
        if (f == face)
            return true;

    return false;

}

bool Tetrahedron::SharesFaceWith(TetrahedronRef other) const {

    for (auto& f: faces_)
        if (other->HasFace(f))
            return true;

    return false;

}

bool Tetrahedron::HasEdge(TetEdgeRef edge) const {

    for (auto& e: edges_)
        if (e == edge)
            return true;

    return false;

}

bool Tetrahedron::SharesEdgeWith(TetrahedronRef other) const {

    for (auto& e: edges_)
        if (other->HasEdge(e))
            return true;

    return false;

}

std::array<TetFaceRef,2> Tetrahedron::GetFacesIncidentTo(EdgeIndex index) const {

    std::array<TetFaceRef,2> incident_faces;

    incident_faces[0] = faces_[INCIDENT_FACE_TABLE[index][0]];
    incident_faces[1] = faces_[INCIDENT_FACE_TABLE[index][1]];

    return incident_faces;

}

int Tetrahedron::BoundaryFaceCount() const {

    int count = 0;
    for (auto& face: faces_) {
        if (face->IsBoundary())
            count++;
    }

    return count;

}

int Tetrahedron::BoundaryNodeCount() const {

    int count = 0;
    for (auto& node: nodes_) {
        if (node->IsBoundary())
            count++;
    }

    return count;

}

void Tetrahedron::ClassifySplitEdgeConfiguration() {

    // Apply bitmask flags to cache which of the tet's 6 edges are split.
    split_edges_bitmask_.reset();
    for (auto index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3}) {
        if (edges_[index]->HasMidpoint())
            split_edges_bitmask_.set(index);
    }

}

bool Tetrahedron::RequiresGreenPrep() const {

    // Only two-splits with opposite split edges are suitable for two-split Green refinement.
    // Other two-splits require further prep.
    auto split_edge_count = GetSplitEdgeCount();
    if (split_edge_count == 2) {

        // Tet has two broken edges and they are opposite.
        if ( (split_edges_bitmask_[EDGE_0_1] & split_edges_bitmask_[EDGE_2_3]) |
             (split_edges_bitmask_[EDGE_0_2] & split_edges_bitmask_[EDGE_1_3]) |
             (split_edges_bitmask_[EDGE_0_3] & split_edges_bitmask_[EDGE_1_2]) ) {

            return false;
        }

        // Two broken edges but they are adjacent, indicating a Green prep is required.
        return true;

    } else {
        // More or less than 2 splits, no not considered.
        return false;
    }

}

EdgeIndex Tetrahedron::ThirdEdgeToCompleteSplitFace() {

    // We assume that there are two edge split, return the flag to the third edge that completes the
    // edge loop on the shared face. Return NO_EDGE if the edges are not co-planar.
    if (GetSplitEdgeCount() != 2)
        return NO_EDGE;

    int first_edge = NO_EDGE;
    int second_edge = NO_EDGE;
    for (auto flag: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3}) {
        if (split_edges_bitmask_[flag]) {
            if (first_edge == NO_EDGE)
                first_edge = flag;
            else
                second_edge = flag;
        }
    }

    return EDGE_COMPLETE_TABLE[first_edge][second_edge];

}

bool Tetrahedron::IsPotentialRed() const {

    auto split_edge_count = GetSplitEdgeCount();

    // No: Tet shares no edges with subdivided Tets.
    if (split_edge_count == 0)
        return false;

    // No: Tet is already "Red".
    if (split_edge_count == 6)
        return false;

    // No: Tet has only a single broken edge.
    if (split_edge_count == 1)
        return false;

    if (split_edge_count == 2) {

        // No: Tet has two broken edges and they are opposite.
        if ( (split_edges_bitmask_[EDGE_0_1] & split_edges_bitmask_[EDGE_2_3]) |
             (split_edges_bitmask_[EDGE_0_2] & split_edges_bitmask_[EDGE_1_3]) |
             (split_edges_bitmask_[EDGE_0_3] & split_edges_bitmask_[EDGE_1_2]) ) {

            return false;
        }

        // Yes: Two broken edges but they are adjacent, indicating a Red refinement.
        return true;

    }

    // No: Tet has three broken edges and all are adjacent to the same face.
    if (split_edge_count == 3) {
        if ( (split_edges_bitmask_[EDGE_0_1] & split_edges_bitmask_[EDGE_1_2] & split_edges_bitmask_[EDGE_0_2]) |
             (split_edges_bitmask_[EDGE_0_2] & split_edges_bitmask_[EDGE_0_3] & split_edges_bitmask_[EDGE_2_3]) |
             (split_edges_bitmask_[EDGE_0_1] & split_edges_bitmask_[EDGE_0_3] & split_edges_bitmask_[EDGE_1_3]) |
             (split_edges_bitmask_[EDGE_1_3] & split_edges_bitmask_[EDGE_1_2] & split_edges_bitmask_[EDGE_2_3]) ) {

            return false;
        }

        // Yes: Three broken edges not on the same face.
        return true;
    }

    // Yes!
    return true;
}

bool Tetrahedron::IsInverted() const {

    // If a tet has a face with normals both pointing in the same direction, we know that either this tetrahedron
    // or the adjacent is inverted. This test does not function on boundary faces, however, so the test is not
    // perfect -- it may produce a false negative if all faces are boundary, for instance.
    for (auto& face: faces_) {
        if (!face->IsBoundary()) {
            auto other_tet = face->GetOtherTet(TetrahedronRef(this));
            auto my_normal = face->Normal(TetrahedronRef(this));
            auto other_normal = face->Normal(other_tet);
            if (my_normal.dot(other_normal)>0.0)
                return true;
        }
    }

    return false;

}

Real Tetrahedron::EdgeLengthRatio() const {

    Real max_length = 0.0;
    Real min_length = std::numeric_limits<Real>::max();

    for (auto& edge: edges_) {
        auto length = edge->Length();
        if (length > max_length)
            max_length = length;
        if (length < min_length)
            min_length = length;
    }

    return min_length/max_length;

}

Vec3 Tetrahedron::Centroid() const {

    Vec3 centroid(0.0,0.0,0.0);

    for (auto& node: nodes_) {
        centroid += node->Position();
    }

    return centroid / 4.0;
}

Real Tetrahedron::Altitude(int node_index) const {

    // Find the normal of the opposite triangle and a vector from the plane to the apex.
    // Project the vector onto the normal and find it's length. This is the altitude.
    auto edge0 = nodes_[(node_index+2)%4]->Position() - nodes_[(node_index+1)%4]->Position();
    auto edge1 = nodes_[(node_index+3)%4]->Position() - nodes_[(node_index+1)%4]->Position();
    auto normal = cross(edge0,edge1);
    normal.normalize();
    auto to_apex = nodes_[node_index]->Position() - nodes_[(node_index+1)%4]->Position();
    auto altitude = abs(to_apex.dot(normal));

    return altitude;

}

Real Tetrahedron::GetMinAltitude() const {

    // We define the altitude of a node as the minimum distance to the plane defined by the opposite face.
    auto min_altitude = std::numeric_limits<Real>::max();
    for (Index i = 0; i<4; i++){

        auto altitude = Altitude(i);

        if (altitude<min_altitude)
            min_altitude = altitude;
    }

    return min_altitude;

}

MinMaxReal Tetrahedron::GetMinMaxEdgeLengths() const {

    MinMaxReal edge_lengths{ {std::numeric_limits<Real>::max(), 0.0} };

    for (auto& edge: edges_) {
        auto length = edge->Length();
        if (length > edge_lengths[1]) {
            edge_lengths[1] = length;
        }
        if (length < edge_lengths[0]) {
            edge_lengths[0] = length;
        }
    }

    return edge_lengths;
}

MinMaxReal Tetrahedron::GetMinMaxDihedralAngles() const {

    // Compare the dihedral angles of each edge and return the most acute and the most obtuse.
    MinMaxReal dihedral_angles{ {std::numeric_limits<Real>::max(), 0.0} };

    for (auto edge_index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3} ) {

        auto face0 = faces_[INCIDENT_FACE_TABLE[edge_index][0]];
        auto normal0 = -face0->Normal(TetrahedronRef(this));

        auto face1 = faces_[INCIDENT_FACE_TABLE[edge_index][1]];
        auto normal1 = -face1->Normal(TetrahedronRef(this));

        Real angle = 3.14159265359 - std::acos(normal0.dot(normal1));

        if (angle > dihedral_angles[1]) {
            dihedral_angles[1] = angle;
        }
        if (angle < dihedral_angles[0]) {
            dihedral_angles[0] = angle;
        }
    }

    return dihedral_angles;

}

/*
Real Tetrahedron::CalculateAspectRatio() {

    // Aspect ratio is defined as maximum edge length divided by minimum altitude.
    auto edge_lengths = GetMinMaxEdgeLengths();
    auto altitude = GetMinAltitude();

    quality_ = edge_lengths[1] / altitude;

    return quality_;

}
*/

Real Tetrahedron::Volume() const {

    auto edge03 = edges_[EDGE_0_3]->AsVector();
    auto edge13 = edges_[EDGE_1_3]->AsVector();
    auto edge23 = edges_[EDGE_2_3]->AsVector();
    auto volume = abs(dot(edge03,cross(edge13,edge23))/6.0);

    return volume;

}

Real Tetrahedron::Inradius() const {

    Real face_area_sum = 0.0;
    for (auto& face: faces_)
        face_area_sum += face->Area();

    auto inradius = (3.0 * Volume()) / face_area_sum;

    return inradius;

}

Real Tetrahedron::Circumradius() const {

    auto a = (nodes_[1]->Position() - nodes_[0]->Position()).length();
    auto b = (nodes_[2]->Position() - nodes_[0]->Position()).length();
    auto c = (nodes_[3]->Position() - nodes_[0]->Position()).length();

    auto A = (nodes_[3]->Position() - nodes_[2]->Position()).length();
    auto B = (nodes_[3]->Position() - nodes_[1]->Position()).length();
    auto C = (nodes_[2]->Position() - nodes_[1]->Position()).length();

    auto p = (a*A + b*B + c*C) / 2.0;

    auto radicand = p * (p - a*A) * (p - b*B) * (p - c*C);

    return sqrt(radicand) / (6.0 * Volume());

}

Real Tetrahedron::QualityMeasure() const {

    // Quality heiristic is the ratio of the inscribed sphere area
    // to the circumscribed sphere area. We can eliminate the constants
    // in the area calculation (as they cancel out). We then multiply the
    // ratio to normalize, ie. a regular tetrahedron has a quality of 1.0.
    // Lower results denote lower quality.

    auto inradius = Inradius();
    auto insphere_area = inradius * inradius;

    auto circumradius = Circumradius();
    auto circumsphere_area = circumradius * circumradius;

    return 9.0 * ( insphere_area / circumsphere_area );

}

void Tetrahedron::GetNewEdges() {

    // Retrieve existing required edges and create new any that do not pre-exist.
    for (auto edge_index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3} ) {

        auto node_index_0 = EDGE_TABLE[edge_index][0];
        auto node_index_1 = EDGE_TABLE[edge_index][1];
        edges_[edge_index] = nodes_[node_index_0]->GetEdgeTo(nodes_[node_index_1]);
        if (edges_[edge_index] == nullptr) {
            edges_[edge_index] = tet_mesh_->AddEdge(nodes_[node_index_0], nodes_[node_index_1]);
        }
        edges_[edge_index]->ConnectTetrahedron(this);

    }

}

void Tetrahedron::GetNewFaces() {

    // Retrieve existing required faces and create new any that do not pre-exist.
    for (auto face_index: {FACE_0_1_2, FACE_0_2_3, FACE_0_3_1, FACE_1_2_3} ) {

        auto edge_index_0 = FACE_TABLE[face_index][0];
        auto edge_index_1 = FACE_TABLE[face_index][1];
        auto edge_index_2 = FACE_TABLE[face_index][2];
        faces_[face_index] = edges_[edge_index_0]->GetFaceWithEdge(edges_[edge_index_1]);
        if (faces_[face_index] == nullptr) {
            faces_[face_index] = tet_mesh_->AddFace(edges_[edge_index_0], edges_[edge_index_1], edges_[edge_index_2]);
        }
        faces_[face_index]->ConnectTetrahedron(this);

    }

}

bool Tetrahedron::IsOutsideSDFNarrowband() const {

    // We consider a tet outside the SDF narrowband if all of it's
    // nodes have the same sdf sdf and all are positive.
    Real sdf = nodes_[0]->Sdf();

    if (sdf<0.0)
        return false;

    for (auto& node: nodes_){
        Real distance = fabs(node->Sdf() - sdf);
        if (distance>std::numeric_limits<Real>::epsilon())
            return false;
    }

    return true;

}

bool Tetrahedron::IsCompletelyInsideLevelSet() const {

    // We consider a tet completely inside the level set if all of it's nodes
    // have negative sdf sdf.
    for (auto& node: nodes_){
        if  (node->Sdf() > 0.0)
            return false;
    }
    return true;
}

bool Tetrahedron::CrossesLevelSet() const {

    // A tet is known to cross the level set if one of it's nodes has positive sdf and one has negative.
    bool positive_distance = false;
    bool negative_distance = false;
    for (auto& node: nodes_){
        if  (node->Sdf() > 0.0)
            positive_distance = true;
        else
            negative_distance = true;
    }
    return positive_distance & negative_distance;

}

bool Tetrahedron::EncroachedByLevelSet(int recursion_depth,
                                       const VDBSamplerPtr sdf_sampler,
                                       const Vec3 &vertex0,
                                       const Vec3 &vertex1,
                                       const Vec3 &vertex2) const {

    // A tet is said to be encroached by the level set if the level set passes into the
    // volume of the tetrahedron without enveloping any of it's nodes.
    // This is a difficult condition to test for: we recursively subdivide the faces
    // until an interior point is found or until we are satisfied with the depth attained.

    // Find centroid of the face.
    Vec3 centroid = (vertex0 + vertex1 + vertex2) / 3.0;

    // If the centroid is inside the level set, cut out.
    if (sdf_sampler->Sample(centroid[0], centroid[1], centroid[2]) <= 0.0)
        return true;

    // Otherwise, recurse.
    if (recursion_depth == 0)
        return false;

    if (EncroachedByLevelSet(recursion_depth-1, sdf_sampler, vertex0, vertex1, centroid))
        return true;
    if (EncroachedByLevelSet(recursion_depth-1, sdf_sampler, vertex0, centroid, vertex2))
        return true;
    if (EncroachedByLevelSet(recursion_depth-1, sdf_sampler, centroid, vertex1, vertex2))
        return true;

    return false;

}

}; // namespace destroyer
