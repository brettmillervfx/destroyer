//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh/Types.h"

#include <GU/GU_Detail.h>
#include <UT/UT_BoundingBox.h>


namespace destroyer {

/*************************************************************************************

 VDBSampler class

 Implements an interface to a Houdini VDB grid. (Specifically useful for SDF level set implementation.
 If the sampler is constructed with min and max values, any sampled
 value will be normalized and clamped to the described interval.

*************************************************************************************/


class VDBSampler
{
public:

    // Constructor may be passed a scalar vdb grid representing the level set or a Houdii Detail containing
    // a vdb primitive representing the level set.
    explicit VDBSampler(VDBGridPtr vdb);
    explicit VDBSampler(const GU_Detail *detail);

    ~VDBSampler() = default;

    // Test if we're holding a valid SDF.
    inline bool IsValid() const { return (vdb_ != nullptr); };

    // Get bounds of the contained SDF.
    UT_BoundingBox GetBBox() const;

    // Sample the VDB.
    Real Sample(Real x, Real y, Real z) const;
    Vec3 SampleGradient(Real x, Real y, Real z) const;

    inline Real GetBackgroundValue() const { return background_; };

private:
    VDBGridPtr LoadSDF(const GU_Detail *detail);
    Real EvaluateBackgroundValue();

private:
    VDBGridPtr vdb_;
    Real background_;

};

}; // namespace destroyer

