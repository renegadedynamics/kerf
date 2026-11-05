// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G1Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G1Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A G0 and G1 constraint of a non-linear plate: a target point and target first derivatives.</summary>
internal class Hpg0G1Constraint : Hpg0Constraint
{
    private readonly D1 _g1Target;
    private int _orientation;

    public Hpg0G1Constraint(in Xy uv, in Xyz value, in D1 d1T) : base(uv, value)
    {
        _g1Target = d1T;
        SetActiveOrder(1);
        _orientation = 0;
    }

    public override void SetOrientation(int orient = 0)
    {
        _orientation = orient;
    }

    public override int ActiveOrder()
    {
        if (_activeOrder < 1)
        {
            return _activeOrder;
        }
        else
        {
            return 1;
        }
    }

    public override int Orientation() => _orientation;

    public override D1 G1Target() => _g1Target;
}
