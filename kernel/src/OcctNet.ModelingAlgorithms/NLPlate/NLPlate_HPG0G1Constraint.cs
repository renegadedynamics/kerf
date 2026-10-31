// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G1Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G1Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A G0 and G1 constraint of a non-linear plate: a target point and target first derivatives.</summary>
public class NLPlate_HPG0G1Constraint : NLPlate_HPG0Constraint
{
    private readonly Plate_D1 myG1Target;
    private int myOrientation;

    public NLPlate_HPG0G1Constraint(in gp_XY UV, in gp_XYZ Value, in Plate_D1 D1T) : base(UV, Value)
    {
        myG1Target = D1T;
        SetActiveOrder(1);
        myOrientation = 0;
    }

    public override void SetOrientation(int Orient = 0)
    {
        myOrientation = Orient;
    }

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

    public override int Orientation() => myOrientation;

    public override Plate_D1 G1Target() => myG1Target;
}
