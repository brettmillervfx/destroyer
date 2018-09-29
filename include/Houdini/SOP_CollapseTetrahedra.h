//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {

class SOP_CollapseTetrahedra : public SOP_Node
{
public:
    SOP_CollapseTetrahedra(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_CollapseTetrahedra() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer