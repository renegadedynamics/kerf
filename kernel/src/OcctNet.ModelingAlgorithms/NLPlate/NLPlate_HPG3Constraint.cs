// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG3Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG3Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A G1, G2 and G3 constraint of a non-linear plate, the position free.</summary>
public class NLPlate_HPG3Constraint : NLPlate_HPG2Constraint
{
    private readonly Plate_D3 myG3Target;

    public NLPlate_HPG3Constraint(in gp_XY UV, in Plate_D1 D1T, in Plate_D2 D2T, in Plate_D3 D3T) : base(UV, D1T, D2T)
    {
        myG3Target = D3T;
        SetActiveOrder(3);
    }

    public override int ActiveOrder()
    {
        if (myActiveOrder < 3)
        {
            return myActiveOrder;
        }
        else
        {
            return 3;
        }
    }

    public override Plate_D3 G3Target() => myG3Target;
}
