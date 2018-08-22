//
// Created by Brett Miller on 8/15/18.
//

#include "SOP_CompressTetMesh.h"

#include <chrono>
#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "DetailGenerator.h"
#include "TetMeshToHoudiniDetail.h"
#include "VDBSampler.h"
#include "CompressionTetMesh.h"
#include "TetNode.h"
#include "Tetrahedron.h"


namespace destroyer {



PRM_Template
        SOP_CompressTetMesh::myTemplateList[] = {
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

    // No parameters currently

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<CompressionTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    tet_mesh->SortNodesByDepth();


    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.convert();

    /*
    // Test depth sorting mechanism.
    auto depth_handle = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_POINT, "d", 1));
    tet_mesh->ResetNodeIterator();
    auto node = tet_mesh->NextNode();
    while(node != nullptr) {
        //std::cout << "depth " << node->Depth() << std::endl;
        depth_handle.set(node->Id(), node->Depth());
        node = tet_mesh->NextNode();
    }
    */

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