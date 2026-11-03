// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG1Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG1Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>A G1 constraint of a non-linear plate: target first derivatives at a UV point, the position free.</summary>
public class NLPlate_HPG1Constraint : NLPlate_HGPPConstraint
{
    private bool IncrementalLoadingAllowed;
    private readonly Plate_D1 myG1Target;
    private int myOrientation;

    public NLPlate_HPG1Constraint(in Xy UV, in Plate_D1 D1T)
    {
        myG1Target = D1T;
        SetUV(UV);
        SetActiveOrder(1);
        IncrementalLoadingAllowed = false;
        myOrientation = 0;
    }

    public override void SetIncrementalLoadAllowed(bool ILA)
    {
        IncrementalLoadingAllowed = ILA;
    }

    public override void SetOrientation(int Orient = 0)
    {
        myOrientation = Orient;
    }

    public override bool IncrementalLoadAllowed() => IncrementalLoadingAllowed;

    public override int ActiveOrder()
    {
        if (myActiveOrder < 1)
        {
            return myActiveOrder;
        }
        else
        {
            return 1;
        }
    }

    public override bool IsG0() => false;

    public override int Orientation() => myOrientation;

    public override Plate_D1 G1Target() => myG1Target;
}
