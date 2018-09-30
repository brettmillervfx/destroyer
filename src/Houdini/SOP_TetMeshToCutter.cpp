//
// Created by Brett Miller on 9/30/18.
//

#include "Houdini/SOP_TetMeshToCutter.h"

#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/TetMesh.h"


namespace destroyer {


    PRM_Template
            SOP_TetMeshToCutter::myTemplateList[] = {
            PRM_Template(),
    };


//////////////////////////////////////
//////////////////////////////////////


    OP_Node *
    SOP_TetMeshToCutter::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new SOP_TetMeshToCutter(net, name, op);
    }


    SOP_TetMeshToCutter::SOP_TetMeshToCutter(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
    {
    }


    OP_ERROR
    SOP_TetMeshToCutter::cookMySop(OP_Context &context)
    {
        OP_AutoLockInputs inputs(this);
        if (inputs.lock(context) >= UT_ERROR_ABORT)
            return error();

        duplicateSource(0, context);

        // Collect parameters.
        auto now = context.getTime();

        // Instantiate an empty TetMesh.
        auto tet_mesh = std::make_shared<TetMesh>();

        // Fill the TetMesh using detail geometry.
        auto detail_generator = new DetailGenerator(tet_mesh, gdp);
        detail_generator->FillTetMesh();
        delete detail_generator;

        // Condition TetMesh into Houdini detail geometry.
        TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
        conditioner.ConvertBoundary();

        // Tear down TetMesh.
        tet_mesh->TearDown();


        return error();
    }


    const char *
    SOP_TetMeshToCutter::inputLabel(unsigned idx) const
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