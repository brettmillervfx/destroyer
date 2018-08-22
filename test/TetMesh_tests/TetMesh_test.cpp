//
// Created by Brett Miller on 8/11/18.
//

#include "TetMesh/TetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/VDBSampler.h"
#include "gtest/gtest.h"

/*
class MockSampler : public destroyer::VDBSampler {
public:
    MockSampler() {};

    destroyer::Real Sample(destroyer::Real x,
                                    destroyer::Real y,
                                    destroyer::Real z) const override {

        // x=3 plane, narrow band is 5 units
        auto dist = x-3.0;
        destroyer::Real sdf = dist;
        if (dist>5.0)
            sdf = 5.0;
        if (dist<-5.0)
            sdf = -5.0;

        return sdf;
    }

    destroyer::Real GetBackgroundValue() { return 2.0; };

};
*/


TEST(TetMeshTest, test_constructors_empty) {

    auto tet_mesh = new destroyer::TetMesh();
    EXPECT_TRUE(tet_mesh->IsEmpty());
    tet_mesh->AddNode(1.0,2.0,3.0);
    EXPECT_FALSE(tet_mesh->IsEmpty());
    delete tet_mesh;

}

TEST(TetMeshTest, test_unused_nodes) {

    auto tet_mesh = new destroyer::TetMesh();

    tet_mesh->AddNode(1.0,2.0,3.0);
    EXPECT_FALSE(tet_mesh->IsEmpty());
    tet_mesh->DeleteUnusedTopology();
    EXPECT_TRUE(tet_mesh->IsEmpty());

    auto n0 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n1 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n2 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n3 = tet_mesh->AddNode(1.0,2.0,3.0);
    tet_mesh->AddTetrahedron(n0,n1,n2,n3);
    tet_mesh->DeleteUnusedTopology();
    EXPECT_FALSE(tet_mesh->IsEmpty());

    delete tet_mesh;

}

TEST(TetMeshTest, test_integration) {

    auto tet_mesh = new destroyer::TetMesh();

    auto n0 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n1 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n2 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n3 = tet_mesh->AddNode(1.0,2.0,3.0);
    auto n4 = tet_mesh->AddNode(1.0,2.0,3.0);
    tet_mesh->AddTetrahedron(n0,n1,n2,n3);
    tet_mesh->AddTetrahedron(n4,n3,n2,n1);

    EXPECT_TRUE(n0->IsConnected());
    auto edge = n1->GetEdgeTo(n2);
    EXPECT_EQ(edge->IncidentFaceCount(),3);

    auto edge1 = n2->GetEdgeTo(n3);
    auto face = edge->GetFaceWithEdge(edge1);
    EXPECT_FALSE(face->IsBoundary());

    auto edge2 = n2->GetEdgeTo(n4);
    auto face1 = edge->GetFaceWithEdge(edge2);
    EXPECT_TRUE(face1->IsBoundary());

    delete tet_mesh;

}

