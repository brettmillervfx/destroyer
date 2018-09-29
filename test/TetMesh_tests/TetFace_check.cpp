//
// Created by Brett Miller on 9/1/18.
//

#include <UT/UT_Vector3.h>
#include "TetMesh/TetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "gtest/gtest.h"

TEST(TetFaceCheck, test_normal) {
    auto mesh = destroyer::TetMesh();
    auto node0 = mesh.AddNode(1.0, 0.0, 0.0);
    auto node1 = mesh.AddNode(1.0, 0.0, 1.0);
    auto node2 = mesh.AddNode(0.0, 0.0, 1.0);
    auto node3 = mesh.AddNode(0.0, -1.0, 0.0);
    mesh.AddTetrahedron(node0, node1, node2, node3);
    mesh.ResetTetIterator();
    auto tet = mesh.NextTet();
    auto centroid = tet->Centroid();

    for (int i = 0; i < 4; i++) {
        auto face = tet->GetFaceRef(i);
        auto normal = face->Normal();
        auto face_centroid = face->Centroid();
        EXPECT_GT(dot(normal, (face_centroid - centroid)), 0.0);
    }
}

TEST(TetFaceCheck, test_area) {
    auto mesh = destroyer::TetMesh();

    // Regular tetrahedron
    auto node0 = new destroyer::TetNode( sqrt(8.0/9.0), 0.0, -1.0/3.0 );
    auto node1 = new destroyer::TetNode( -sqrt(2.0/9.0), sqrt(2.0/3.0), -1.0/3.0 );
    auto node2 = new destroyer::TetNode(-sqrt(2.0/9.0), -sqrt(2.0/3.0), -1.0/3.0 );
    auto node3 = new destroyer::TetNode( 0.0, 0.0, 1.0 );

    mesh.AddTetrahedron(node0,node1,node2,node3);
    mesh.ResetTetIterator();
    auto tet = mesh.NextTet();
    auto face = tet->GetFaceRef(0);

    auto a = sqrt(8.0/3.0);
    auto expected_area = (sqrt(3.0)/4.0) * a * a;
    EXPECT_LT(abs(face->Area()-expected_area),0.0001);
}
