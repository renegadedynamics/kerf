// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G2Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G2Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A G0, G1 and G2 constraint of a non-linear plate.</summary>
public class NLPlate_HPG0G2Constraint : NLPlate_HPG0G1Constraint
{
    private readonly Plate_D2 myG2Target;

    public NLPlate_HPG0G2Constraint(in gp_XY UV, in gp_XYZ Value, in Plate_D1 D1T, in Plate_D2 D2T) : base(UV, Value, D1T)
    {
        myG2Target = D2T;
        SetActiveOrder(2);
    }

    public override int ActiveOrder()
    {
        if (myActiveOrder < 2)
        {
            return myActiveOrder;
        }
        else
        {
            return 2;
        }
    }

    public override Plate_D2 G2Target() => myG2Target;
}
