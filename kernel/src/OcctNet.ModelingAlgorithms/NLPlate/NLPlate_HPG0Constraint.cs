// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A G0 constraint of a non-linear plate: the surface passes through a target point at a UV point.</summary>
public class NLPlate_HPG0Constraint : NLPlate_HGPPConstraint
{
    private readonly gp_XYZ myXYZTarget;
    private bool UVIsFree;
    private bool IncrementalLoadingAllowed;

    public NLPlate_HPG0Constraint(in gp_XY UV, in gp_XYZ Value)
    {
        myXYZTarget = Value;
        SetUV(UV);
        SetActiveOrder(0);
        UVIsFree = false;
        IncrementalLoadingAllowed = false;
    }

    public override void SetUVFreeSliding(bool UVFree)
    {
        UVIsFree = UVFree;
    }

    public override void SetIncrementalLoadAllowed(bool ILA)
    {
        IncrementalLoadingAllowed = ILA;
    }

    public override bool UVFreeSliding() => UVIsFree;

    public override bool IncrementalLoadAllowed() => IncrementalLoadingAllowed;

    public override int ActiveOrder()
    {
        if (myActiveOrder < 0)
        {
            return myActiveOrder;
        }
        else
        {
            return 0;
        }
    }

    public override bool IsG0() => true;

    public override gp_XYZ G0Target() => myXYZTarget;
}
