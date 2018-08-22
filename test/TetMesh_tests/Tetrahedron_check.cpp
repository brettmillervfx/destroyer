//
// Created by Brett Miller on 8/9/18.
//

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "gtest/gtest.h"

TEST(TetrahedronTest, test_disconnect) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeD = new destroyer::TetNode(2.0,2.0,2.0);

    auto mesh = new destroyer::TetMesh();
    auto tet = new destroyer::Tetrahedron(mesh, nodeA, nodeB, nodeC, nodeD);

    //delete nodeA;
    //delete nodeB;
    //delete nodeC;
    //delete nodeD;
}