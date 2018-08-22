//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_CleanupTetMesh.h"

#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/RefinementTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {



PRM_Template
        SOP_CleanupTetMesh::myTemplateList[] = {
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

    // An SDF sampler wil be necessary for tetrahedra culling and surface compression.
    auto sdf_sampler = std::make_shared<VDBSampler>(inputGeo(1, context));

    if (!sdf_sampler->IsValid()) {
        addError(SOP_ERR_INVALID_SRC, "No SDF VDB found in second input.");
        return error();
    }

    // Collect parameters.
    auto now = context.getTime();

    // No parameters currently

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<RefinementTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    tet_mesh->Cleanup();


    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.convert();

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
        case 1:
            return "VDB Level Set";
        default:
            return "default";
    }
}


}; // namespace destroyer