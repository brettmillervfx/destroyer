//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_CompressTetMesh.h"

#include <chrono>
#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/CompressionTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {

static PRM_Name soft_sweeps_prm_name("softSweeps", "Soft Sweeps");
static PRM_Default soft_sweeps_prm_default(5);

static PRM_Name hard_sweeps_prm_name("hardSweeps", "Hard Sweeps");
static PRM_Default hard_sweeps_prm_default(5);

PRM_Template
        SOP_CompressTetMesh::myTemplateList[] = {
        PRM_Template(PRM_INT_J, 1, &soft_sweeps_prm_name, &soft_sweeps_prm_default),
        PRM_Template(PRM_INT_J, 1, &hard_sweeps_prm_name, &hard_sweeps_prm_default),
        PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_CompressTetMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CompressTetMesh(net, name, op);
}


SOP_CompressTetMesh::SOP_CompressTetMesh(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_CompressTetMesh::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // An SDF sampler wil be necessary for tetrahedra culling and surface compression.
    auto sdf_sampler = std::make_shared<VDBSampler>(inputGeo(1, context));

    if (!sdf_sampler->IsValid()) {
        addError(SOP_ERR_INVALID_SRC, "No SDF VDB found in second input.");
        return error();
    }

    // Collect parameters.
    auto now = context.getTime();

    auto soft_sweeps = evalInt("softSweeps", 0, now);
    auto hard_sweeps = evalInt("hardSweeps", 0, now);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<CompressionTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    tet_mesh->Compress(soft_sweeps, hard_sweeps);

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.convert();

    // Tear down TetMesh.
    tet_mesh->TearDown();



    return error();
}


const char *
SOP_CompressTetMesh::inputLabel(unsigned idx) const
{
    switch(idx)
    {
        case 0:
            return "Tet Mesh Geo";
        case 1:
            return "VDB Level Set";
        default:
            return "default";
    }
}


}; // namespace destroyer