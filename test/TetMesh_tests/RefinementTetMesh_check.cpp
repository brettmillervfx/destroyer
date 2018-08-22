//
// Created by Brett Miller on 8/21/18.
//

#include <memory>

#include <Eigen/Dense>
#include "TetMesh/Types.h"
#include "TetMesh/RefinementTetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/TetNode.h"
#include "gtest/gtest.h"

TEST(RefinementTetMeshTest, test_single_full_refinement) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);

    std::set<destroyer::Index> refine = {666};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int tcount = 0;
    while(t != nullptr) {
        tcount++;
        t = mesh->NextTet();
    }

    EXPECT_EQ(tcount,8);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_share_edge) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);
    auto n5 = mesh->AddNode(2,1,0);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n2,n3,n4,n5, 7);

    std::set<destroyer::Index> refine = {666};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular = 0;
    int irregular = 0;
    while(t != nullptr) {
        if (t->Id() == 666)
            regular++;
        else if (t->Id() == 7)
            irregular++;
        t = mesh->NextTet();
    }

    EXPECT_EQ(regular,8);
    EXPECT_EQ(irregular,2);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_share_face) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n0,n1,n2,n4, 7);

    std::set<destroyer::Index> refine = {666};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular = 0;
    int irregular = 0;
    while(t != nullptr) {
        if (t->Id() == 666)
            regular++;
        else if (t->Id() == 7)
            irregular++;
        t = mesh->NextTet();
    }

    EXPECT_EQ(regular,8);
    EXPECT_EQ(irregular,4);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_share_node) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);
    auto n5 = mesh->AddNode(2,1,0);
    auto n6 = mesh->AddNode(2,1,3);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n0,n4,n5,n6, 7);

    std::set<destroyer::Index> refine = {666};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular = 0;
    int irregular = 0;
    while(t != nullptr) {
        if (t->Id() == 666)
            regular++;
        else if (t->Id() == 7)
            irregular++;
        t = mesh->NextTet();
    }

    EXPECT_EQ(regular,8);
    EXPECT_EQ(irregular,1);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_share_two_tets) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);
    auto n5 = mesh->AddNode(2,1,0);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n0,n1,n2,n4, 7);
    mesh->AddTetrahedron(n1,n2,n4,n5, 667);

    std::set<destroyer::Index> refine = {666, 667};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular666 = 0;
    int regular667 = 0;
    int irregular = 0;
    while(t != nullptr) {
        switch(t->Id()) {
            case 666:
                regular666++;
                break;
            case 667:
                regular667++;
                break;
            case 7:
                irregular++;
                break;
        }
        t = mesh->NextTet();
    }

    EXPECT_EQ(regular666,8);
    EXPECT_EQ(regular667,8);
    EXPECT_EQ(irregular,8);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_green_two_complete) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);
    auto n5 = mesh->AddNode(2,1,0);
    auto n6 = mesh->AddNode(2,1,0);
    auto n7 = mesh->AddNode(2,1,0);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n0,n1,n4,n5, 7);
    mesh->AddTetrahedron(n4,n5,n6,n7, 667);

    std::set<destroyer::Index> refine = {666, 667};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular666 = 0;
    int regular667 = 0;
    int irregular = 0;
    while(t != nullptr) {
        switch(t->Id()) {
            case 666:
                regular666++;
                break;
            case 667:
                regular667++;
                break;
            case 7:
                irregular++;
                break;
        }
        t = mesh->NextTet();
    }

    int nodeCount = 0;
    mesh->ResetNodeIterator();
    auto n = mesh->NextNode();
    while(n != nullptr) {
        nodeCount++;
        n = mesh->NextNode();
    }

    EXPECT_EQ(regular666,8);
    EXPECT_EQ(regular667,8);
    EXPECT_EQ(irregular,4);
    EXPECT_EQ(nodeCount, 20);

    delete mesh;
}

TEST(RefinementTetMeshTest, test_green_two_to_three) {
    auto mesh = new destroyer::RefinementTetMesh();

    auto n0 = mesh->AddNode(0,0,0);
    auto n1 = mesh->AddNode(0,1,0);
    auto n2 = mesh->AddNode(0,0,1);
    auto n3 = mesh->AddNode(0,1,0);
    auto n4 = mesh->AddNode(0,0,2);
    auto n5 = mesh->AddNode(2,1,0);
    auto n6 = mesh->AddNode(2,1,0);
    auto n7 = mesh->AddNode(2,1,0);

    mesh->AddTetrahedron(n0,n1,n2,n3, 666);
    mesh->AddTetrahedron(n0,n1,n4,n5, 7);
    mesh->AddTetrahedron(n1,n4,n6,n7, 667);

    std::set<destroyer::Index> refine = {666, 667};

    mesh->RefineIdGroup(refine);

    mesh->ResetTetIterator();
    auto t = mesh->NextTet();
    int regular666 = 0;
    int regular667 = 0;
    int irregular = 0;
    while(t != nullptr) {
        switch(t->Id()) {
            case 666:
                regular666++;
                break;
            case 667:
                regular667++;
                break;
            case 7:
                irregular++;
                break;
        }
        t = mesh->NextTet();
    }

    int nodeCount = 0;
    mesh->ResetNodeIterator();
    auto n = mesh->NextNode();
    while(n != nullptr) {
        nodeCount++;
        n = mesh->NextNode();
    }

    EXPECT_EQ(regular666,8);
    EXPECT_EQ(regular667,8);
    EXPECT_EQ(irregular,4);
    EXPECT_EQ(nodeCount, 21);

    delete mesh;
}