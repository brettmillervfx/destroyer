//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh/Types.h"

#include <GU/GU_Detail.h>
#include <UT/UT_BoundingBox.h>


namespace destroyer {


/*

 VDBSampler class

 Implements an interface to a Houdini VDB grid. If the sampler is constructed with min and max values, any sampled
 value will be normalized and clamped to the described interval.

*/


class VDBSampler
{
public:
    VDBSampler() {};
    VDBSampler(VDBGridPtr vdb);
    VDBSampler(VDBGridPtr vdb, Real min, Real max);
    VDBSampler(const GU_Detail *detail);
    VDBSampler(const GU_Detail *detail, Real min, Real max);
    ~VDBSampler() = default;

    // Test if we're holding a valid SDF.
    inline bool IsValid() const { return (vdb_ != nullptr); };

    // Get bounds of the contained SDF.
    UT_BoundingBox GetBBox() const;

    // Sample the VDB and apply the fit function if required.
    Real Sample(Real x, Real y, Real z) const;
    Vec3 SampleGradient(Real x, Real y, Real z) const;

    inline Real GetBackgroundValue() const { return background_; };

private:
    VDBGridPtr LoadSDF(const GU_Detail *detail);
    Real EvaluateBackgroundValue();

private:
    VDBGridPtr vdb_;
    Real background_;
    bool apply_fit_;
    Real fit_min_;
    Real fit_max_;
};

}; // namespace destroyer

