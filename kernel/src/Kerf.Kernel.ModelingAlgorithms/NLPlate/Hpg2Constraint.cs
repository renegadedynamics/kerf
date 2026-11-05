// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG2Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG2Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A G1 and G2 constraint of a non-linear plate, the position free.</summary>
internal class Hpg2Constraint : Hpg1Constraint
{
    private readonly D2 _g2Target;

    public Hpg2Constraint(in Xy uv, in D1 d1T, in D2 d2T) : base(uv, d1T)
    {
        _g2Target = d2T;
        SetActiveOrder(2);
    }

    public override int ActiveOrder()
    {
        if (_activeOrder < 2)
        {
            return _activeOrder;
        }
        else
        {
            return 2;
        }
    }

    public override D2 G2Target() => _g2Target;
}
