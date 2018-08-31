//
// Created by Brett Miller on 8/7/18.
//

#include <memory>

#include <UT/UT_Vector3.h>
#include "TetMesh/Types.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "gtest/gtest.h"


TEST(TetNodeTest, test_component_constructor) {
    destroyer::TetNode node( 1.0, 2.0, 3.0);
    auto vec = node.Position();
    EXPECT_EQ( vec[0], 1.0 );
    EXPECT_EQ( vec[1], 2.0 );
    EXPECT_EQ( vec[2], 3.0 );
    EXPECT_EQ( node.Id(), 0 );
}

TEST(TetNodeTest, test_component_constructor_w_id) {
    destroyer::TetNode node( 1.0, 2.0, 3.0, 444);
    EXPECT_EQ( node.Id(), 444 );
}

TEST(TetNodeTest, test_vector3d_constructor) {
    destroyer::Vec3 inVec(4.0,3.0,2.0);
    destroyer::TetNode node( inVec);
    auto vec = node.Position();
    EXPECT_EQ( vec[0], 4.0 );
    EXPECT_EQ( vec[1], 3.0 );
    EXPECT_EQ( vec[2], 2.0 );
    EXPECT_EQ( node.Id(), 0 );
}

TEST(TetNodeTest, test_vector3d_constructor_w_id) {
    destroyer::Vec3 inVec(4.0,3.0,2.0);
    destroyer::TetNode node( inVec, 555);
    EXPECT_EQ( node.Id(), 555 );
}

TEST(TetNodeTest, test_set_position) {
    destroyer::TetNode node( 1.0, 2.0, 3.0);
    destroyer::Vec3 inVec(4.0,3.0,2.0);
    node.SetPosition(inVec);
    auto vec = node.Position();
    EXPECT_EQ( vec[0], 4.0 );
    EXPECT_EQ( vec[1], 3.0 );
    EXPECT_EQ( vec[2], 2.0 );
}

TEST(TetNodeTest, test_set_id) {
    destroyer::Vec3 inVec(4.0,3.0,2.0);
    destroyer::TetNode node( inVec, 555);
    node.SetId(3);
    EXPECT_EQ( node.Id(), 3 );
}

TEST(TetNodeTest, test_set_depth) {
    destroyer::Vec3 inVec(4.0,3.0,2.0);
    destroyer::TetNode node(inVec);
    node.SetDepth(3);
    EXPECT_EQ( node.Depth(), 3 );
}

TEST(TetNodeTest, test_tet_connections) {

    auto mesh = new destroyer::TetMesh();
    auto node = mesh->AddNode( 1.0, 2.0, 3.0 );
    EXPECT_FALSE(node->IsConnected());

    auto nodeA = mesh->AddNode( 1.0, 2.0, 3.0 );
    auto nodeB = mesh->AddNode( 1.0, 2.0, 3.0 );
    auto nodeC = mesh->AddNode( 1.0, 2.0, 3.0 );

    auto tet = mesh->AddTetrahedron(nodeA, nodeB, nodeC, node); //new destroyer::Tetrahedron(nodeA, nodeB, nodeC, node);
    EXPECT_TRUE(node->IsConnected());

    node->DisconnectTetrahedron(tet);
    EXPECT_FALSE(node->IsConnected());

    node->ConnectTetrahedron(tet);
    EXPECT_TRUE(node->IsConnected());

    delete mesh;
}

TEST(TetNodeTest, test_edge_connections) {
    auto node = new destroyer::TetNode( 1.0, 2.0, 3.0 );
    auto nodeB = new destroyer::TetNode( 1.0, 2.0, 3.0 );
    auto nodeC = new destroyer::TetNode( 1.0, 2.0, 3.0 );
    auto edgeB = new destroyer::TetEdge(node,nodeB);
    auto edgeC = new destroyer::TetEdge(node,nodeC);

    EXPECT_EQ(node->GetEdgeTo(nodeC), edgeC);
    node->DisconnectEdge(edgeC);
    EXPECT_EQ(node->GetEdgeTo(nodeC), nullptr);

    delete node;
    delete nodeB;
    delete nodeC;
    delete edgeB;
    delete edgeC;
}

/*
TEST(TetNodeTest, test_face_connections) {
    auto node = new destroyer::TetNode( 1.0, 2.0, 3.0 );
    auto nodeB = new destroyer::TetNode( 1.0, 2.0, 3.0 );
    auto nodeC = new destroyer::TetNode( 1.0, 2.0, 3.0 );

    auto edgeB = new destroyer::TetEdge(node,nodeB);
    auto edgeC = new destroyer::TetEdge(node,nodeC);
    auto edgeBC = new destroyer::TetEdge(nodeB,nodeC);


    auto face = new destroyer::TetFace(edgeB, edgeC, edgeBC);

    EXPECT_FALSE(node->IsBoundary());

    auto tet =new destroyer::Tetrahedron(node, nodeB, nodeC, node);
    face->ConnectTetrahedron(tet);

    EXPECT_TRUE(node->IsBoundary());

    node->DisconnectFace(face);

    EXPECT_FALSE(node->IsBoundary());

    delete node;
    delete nodeB;
    delete nodeC;
    delete edgeB;
    delete edgeC;
    delete edgeBC;
    delete face;
    delete tet;
}
*/

