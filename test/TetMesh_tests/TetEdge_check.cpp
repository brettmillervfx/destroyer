//
// Created by Brett Miller on 8/16/18.
//

#include <memory>

#include <UT/UT_Vector3.h>
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "gtest/gtest.h"

TEST(TetEdgeTest, test_constructor) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto edge = new destroyer::TetEdge(nodeA, nodeB);

    EXPECT_TRUE(edge->HasNode(nodeA));
    EXPECT_TRUE(edge->HasNode(nodeB));
    EXPECT_FALSE(edge->HasNode(nodeC));

    delete edge;
    delete nodeA;
    delete nodeB;
    delete nodeC;
}

TEST(TetEdgeTest, test_get_nodes) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto edge = new destroyer::TetEdge(nodeA, nodeB);

    EXPECT_EQ(edge->GetFirstNode(), nodeA);
    EXPECT_EQ(edge->GetOtherNode(nodeB), nodeA);
    EXPECT_EQ(edge->GetOtherNode(nodeA), nodeB);
    EXPECT_EQ(edge->GetOtherNode(nodeC), nullptr);

    delete edge;
    delete nodeA;
    delete nodeB;
    delete nodeC;
}

TEST(TetEdgeTest, test_midpoint) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto edge = new destroyer::TetEdge(nodeA, nodeB);

    EXPECT_FALSE(edge->HasMidpoint());
    EXPECT_EQ(edge->Midpoint(), nullptr);

    auto pos = edge->MidpointPosition();
    auto nodeC = new destroyer::TetNode(pos);
    edge->SetMidpoint(nodeC);

    EXPECT_TRUE(edge->HasMidpoint());
    EXPECT_EQ(edge->Midpoint(), nodeC);

    delete edge;
    delete nodeA;
    delete nodeB;
    delete nodeC;
}
/*
TEST(TetEdgeTest, test_face_connections) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeD = new destroyer::TetNode(2.0,2.0,2.0);
    auto edgeAB = new destroyer::TetEdge(nodeA, nodeB);

    EXPECT_EQ(edgeAB->IncidentFaceCount(),0);

    auto edgeBC = new destroyer::TetEdge(nodeB, nodeC);
    auto edgeCA = new destroyer::TetEdge(nodeC, nodeA);
    auto faceABC = new destroyer::TetFace(edgeAB, edgeBC, edgeCA);

    EXPECT_EQ(edgeAB->IncidentFaceCount(),1);

    auto edgeBD = new destroyer::TetEdge(nodeB, nodeD);
    auto edgeDA = new destroyer::TetEdge(nodeD, nodeA);
    auto faceABD = new destroyer::TetFace(edgeAB, edgeBD, edgeDA);

    EXPECT_EQ(edgeAB->IncidentFaceCount(),2);

    EXPECT_EQ(edgeAB->GetFaceWithEdge(edgeBC),faceABC);

    edgeAB->DisconnectFace(faceABC);

    EXPECT_EQ(edgeAB->IncidentFaceCount(),1);

    EXPECT_EQ(edgeAB->GetFaceWithEdge(edgeBC),nullptr);

    delete nodeA;
    delete nodeB;
    delete nodeC;
    delete nodeD;
    delete edgeAB;
    delete edgeBC;
    delete edgeCA;
    delete faceABC;
    delete edgeBD;
    delete edgeDA;
    delete faceABD;
}
 */

/*

TEST(TetEdgeTest, test_tet_connections) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeD = new destroyer::TetNode(2.0,2.0,2.0);

    auto edgeAB = new destroyer::TetEdge(nodeA, nodeB);

    EXPECT_FALSE(edgeAB->IsConnected());

    auto tet =new destroyer::Tetrahedron(nodeA, nodeB, nodeC, nodeD);
    edgeAB->ConnectTetrahedron(tet);

    EXPECT_TRUE(edgeAB->IsConnected());

    edgeAB->DisconnectTetrahedron(tet);
    EXPECT_FALSE(edgeAB->IsConnected());

    delete nodeA;
    delete nodeB;
    delete nodeC;
    delete nodeD;
    delete edgeAB;
    delete tet;
}
*/
/*
TEST(TetEdgeTest, test_boundary) {
    auto nodeA = new destroyer::TetNode(0.0,0.0,0.0);
    auto nodeB = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeC = new destroyer::TetNode(2.0,2.0,2.0);
    auto nodeD = new destroyer::TetNode(2.0,2.0,2.0);

    auto edgeAB = new destroyer::TetEdge(nodeA, nodeB);
    auto edgeBC = new destroyer::TetEdge(nodeB, nodeC);
    auto edgeCA = new destroyer::TetEdge(nodeC, nodeA);
    auto faceABC = new destroyer::TetFace(edgeAB, edgeBC, edgeCA);

    auto tet = new destroyer::Tetrahedron(nodeA, nodeB, nodeC, nodeD);

    auto boundary_faces = edgeAB->GetIncidentBoundaryFaces();
    EXPECT_EQ(boundary_faces.size(),0);

    faceABC->ConnectTetrahedron(tet);

    auto boundary_faces2 = edgeAB->GetIncidentBoundaryFaces();
    EXPECT_EQ(boundary_faces2.size(),1);

    delete nodeA;
    delete nodeB;
    delete nodeC;
    delete nodeD;
    delete edgeAB;
    delete edgeBC;
    delete edgeCA;
    delete faceABC;
    delete tet;
}
 */