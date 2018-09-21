//
// Created by Brett Miller on 8/9/18.
//

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetMesh.h"
#include "gtest/gtest.h"

/*
TEST(TetrahedronTest, test_disconnect) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeD = new destroyer::TetNode(2.0,2.0,2.0);

    auto mesh = new destroyer::TetMesh();
    auto tet = new destroyer::Tetrahedron(mesh, nodeA, nodeB, nodeC, nodeD);

}
*/
TEST(TetrahedronTest, test_inversion) {

    auto nodeA = new destroyer::TetNode(0.0,1.0,0.0);
    auto nodeB = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeC = new destroyer::TetNode(1.0,0.0,0.0);
    auto nodeD = new destroyer::TetNode(0.0,0.0,1.0);
    auto nodeE = new destroyer::TetNode(0.0,-1.0,0.0);

    auto mesh = new destroyer::TetMesh();
    mesh->AddTetrahedron(nodeA, nodeB, nodeC, nodeD);
    mesh->AddTetrahedron(nodeE, nodeD, nodeC, nodeB);

    mesh->ResetTetIterator();

    auto tet0 = mesh->NextTet();
    auto tet1 = mesh->NextTet();

    EXPECT_FALSE(tet0->IsInverted());
    EXPECT_FALSE(tet1->IsInverted());

    nodeE->SetPosition(destroyer::Vec3(0.0,0.5,0.0));

    EXPECT_TRUE(tet0->IsInverted());
    EXPECT_TRUE(tet1->IsInverted());

    delete mesh;
}

TEST(TetrahedronTest, test_volume) {

    // Regular tetrahedron
    auto nodeA = new destroyer::TetNode( sqrt(8.0/9.0), 0.0, -1.0/3.0 );
    auto nodeB = new destroyer::TetNode( -sqrt(2.0/9.0), sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeC = new destroyer::TetNode(-sqrt(2.0/9.0), -sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeD = new destroyer::TetNode( 0.0, 0.0, 1.0 );

    auto mesh = new destroyer::TetMesh();
    mesh->AddTetrahedron(nodeA, nodeB, nodeC, nodeD);

    mesh->ResetTetIterator();

    auto tet = mesh->NextTet();

    auto a = sqrt(8.0/3.0);
    auto expected_vol = (a*a*a) * (1.0/12.0) * sqrt(2.0);
    EXPECT_LT(tet->Volume()-expected_vol, 0.0001);

    delete mesh;
}

TEST(TetrahedronTest, test_inradius) {

    // Regular tetrahedron
    auto nodeA = new destroyer::TetNode( sqrt(8.0/9.0), 0.0, -1.0/3.0 );
    auto nodeB = new destroyer::TetNode( -sqrt(2.0/9.0), sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeC = new destroyer::TetNode(-sqrt(2.0/9.0), -sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeD = new destroyer::TetNode( 0.0, 0.0, 1.0 );

    auto mesh = new destroyer::TetMesh();
    mesh->AddTetrahedron(nodeA, nodeB, nodeC, nodeD);

    mesh->ResetTetIterator();

    auto tet = mesh->NextTet();

    auto a = sqrt(8.0/3.0);
    auto expected_inradius = ((sqrt(6.0)/12.0)) * a;
    EXPECT_LT(abs(tet->Inradius()-expected_inradius), 0.0001);

    delete mesh;
}

TEST(TetrahedronTest, test_circumradius) {

    // Regular tetrahedron
    auto nodeA = new destroyer::TetNode( sqrt(8.0/9.0), 0.0, -1.0/3.0 );
    auto nodeB = new destroyer::TetNode( -sqrt(2.0/9.0), sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeC = new destroyer::TetNode(-sqrt(2.0/9.0), -sqrt(2.0/3.0), -1.0/3.0 );
    auto nodeD = new destroyer::TetNode( 0.0, 0.0, 1.0 );

    auto mesh = new destroyer::TetMesh();
    mesh->AddTetrahedron(nodeA, nodeB, nodeC, nodeD);

    mesh->ResetTetIterator();

    auto tet = mesh->NextTet();

    auto a = sqrt(8.0/3.0);
    auto expected_circumradius = ((sqrt(6.0)/4.0)) * a;
    EXPECT_LT(abs(tet->Circumradius()-expected_circumradius), 0.0001);

    delete mesh;
}

