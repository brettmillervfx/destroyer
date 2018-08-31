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
    mesh.AddTetrahedron(node0,node1,node2,node3);
    mesh.ResetTetIterator();
    auto tet = mesh.NextTet();
    auto face = tet->GetFaceRef(0);

    auto normal = face->Normal();
    EXPECT_EQ(normal[0], 0.0);
    EXPECT_EQ(normal[1], 1.0);
    EXPECT_EQ(normal[2], 0.0);
}