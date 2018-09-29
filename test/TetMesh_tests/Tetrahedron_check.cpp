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

TEST(TetrahedronTest, test_normals) {

    // Regular tetrahedron
    std::array<destroyer::TetNodeRef,4> nodes;
    nodes[0] = new destroyer::TetNode( sqrt(8.0/9.0), 0.0, -1.0/3.0 );
    nodes[1] = new destroyer::TetNode( -sqrt(2.0/9.0), sqrt(2.0/3.0), -1.0/3.0 );
    nodes[2] = new destroyer::TetNode(-sqrt(2.0/9.0), -sqrt(2.0/3.0), -1.0/3.0 );
    nodes[3] = new destroyer::TetNode( 0.0, 0.0, 1.0 );

    // sanity
    // 1 -> 0
    // 2 -> 1
    // 4 -> 2
    // 3 -> 3

    // test every node permutation for correct normals (24)

    for (int i=0;i<4;i++) {

        for(int j=0;j<4;j++) {

            if (j==i) continue;

            for(int k=0;k<4;k++) {

                if ((k==j) || (k==i)) continue;

                for(int l=0;l<4;l++) {

                    if ((l==i) || (l==j) || (l==k)) continue;

                    // i,j,k,l is a permutation

                    auto mesh = new destroyer::TetMesh();
                    mesh->AddTetrahedron(nodes[i], nodes[j], nodes[k], nodes[l]);

                    mesh->ResetTetIterator();

                    auto tet = mesh->NextTet();

                    auto centroid = tet->Centroid();
                    auto n0 = tet->GetNodeRef(0)->Position();
                    auto n1 = tet->GetNodeRef(1)->Position();
                    auto n2 = tet->GetNodeRef(2)->Position();
                    auto n3 = tet->GetNodeRef(3)->Position();

                    auto v0 = cross((n3-n1),(n2-n1));
                    EXPECT_GT(dot(v0,(centroid-n0)),0.0);
                    auto v1 = cross((n3-n2),(n0-n2));
                    EXPECT_GT(dot(v1,(centroid-n1)),0.0);
                    auto v2 = cross((n3-n0),(n1-n0));
                    EXPECT_GT(dot(v2,(centroid-n2)),0.0);
                    auto v3 = cross((n0-n2),(n1-n2));
                    EXPECT_GT(dot(v3,(centroid-n3)),0.0);

                    delete mesh;

                }

            }

        }

    }

}

