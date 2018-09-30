//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_CleanupTetMesh.h"

#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/RefinementTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {


static PRM_Name max_iter_prm_name("maxIter", "Max Iterations");
static PRM_Default max_iter_prm_default(3);

static PRM_Name lone_tets_prm_name("refineLoneTets", "Refine Lone Tets");
static PRM_Default lone_tets_prm_default(true);

static PRM_Name weak_ext_tets_prm_name("removeWeakExteriorTets", "Remove Weak Exterior Tets");
static PRM_Default weak_ext_tets_prm_default(true);

static PRM_Name weak_int_edges_prm_name("splitWeakInteriorEdges", "Split Weak Interior Edges");
static PRM_Default weak_int_edges_prm_default(true);

static PRM_Name nonmanifold_edges_prm_name("refineNonmanifoldEdges", "Refine Non-Manifold Edges");
static PRM_Default nonmanifold_edges_prm_default(true);

static PRM_Name nonmanifold_nodes_prm_name("refineNonmanifoldNodes", "Refine Non-Manifold Nodes");
static PRM_Default nonmanifold_nodes_prm_default(true);


PRM_Template
SOP_CleanupTetMesh::myTemplateList[] = {
        PRM_Template(PRM_INT_J, 1, &max_iter_prm_name, &max_iter_prm_default),
        PRM_Template(PRM_TOGGLE, 1, &lone_tets_prm_name, &lone_tets_prm_default),
        PRM_Template(PRM_TOGGLE, 1, &weak_ext_tets_prm_name, &weak_ext_tets_prm_default),
        PRM_Template(PRM_TOGGLE, 1, &weak_int_edges_prm_name, &weak_int_edges_prm_default),
        PRM_Template(PRM_TOGGLE, 1, &nonmanifold_edges_prm_name, &nonmanifold_edges_prm_default),
        PRM_Template(PRM_TOGGLE, 1, &nonmanifold_nodes_prm_name, &nonmanifold_nodes_prm_default),
        PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_CleanupTetMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CleanupTetMesh(net, name, op);
}


SOP_CleanupTetMesh::SOP_CleanupTetMesh(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_CleanupTetMesh::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // Collect parameters.
    auto now = context.getTime();

    auto max_iter = evalInt("maxIter", 0, now);
    auto lone_tets = evalInt("refineLoneTets", 0, now);
    auto weak_tets = evalInt("removeWeakExteriorTets", 0, now);
    auto weak_edges = evalInt("splitWeakInteriorEdges", 0, now);
    auto nonmanifold_edges = evalInt("refineNonmanifoldEdges", 0, now);
    auto nonmanifold_nodes = evalInt("refineNonmanifoldNodes", 0, now);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<RefinementTetMesh>();

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    if (!tet_mesh->Cleanup(lone_tets, weak_tets, weak_edges, nonmanifold_edges, nonmanifold_nodes, max_iter))
        addWarning(SOP_MESSAGE, "Cleanup incomplete. Increase max iterations.");

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    // Tear down TetMesh.
    tet_mesh->TearDown();


    return error();
}


const char *
SOP_CleanupTetMesh::inputLabel(unsigned idx) const
{
    switch(idx)
    {
        case 0:
            return "Tet Mesh Geo";
        default:
            return "default";
    }
}


}; // namespace destroyer