// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G3Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0G3Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A G0, G1, G2 and G3 constraint of a non-linear plate.</summary>
internal class Hpg0G3Constraint : Hpg0G2Constraint
{
    private readonly D3 _g3Target;

    public Hpg0G3Constraint(in Xy uv, in Xyz value, in D1 d1T, in D2 d2T, in D3 d3T)
        : base(uv, value, d1T, d2T)
    {
        _g3Target = d3T;
        SetActiveOrder(3);
    }

    public override int ActiveOrder()
    {
        if (_activeOrder < 3)
        {
            return _activeOrder;
        }
        else
        {
            return 3;
        }
    }

    public override D3 G3Target() => _g3Target;
}
