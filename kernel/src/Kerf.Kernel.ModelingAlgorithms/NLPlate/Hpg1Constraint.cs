// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG1Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG1Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A G1 constraint of a non-linear plate: target first derivatives at a UV point, the position free.</summary>
public class Hpg1Constraint : HgppConstraint
{
    private bool _incrementalLoadingAllowed;
    private readonly D1 _g1Target;
    private int _orientation;

    public Hpg1Constraint(in Xy uv, in D1 d1T)
    {
        _g1Target = d1T;
        SetUV(uv);
        SetActiveOrder(1);
        _incrementalLoadingAllowed = false;
        _orientation = 0;
    }

    public override void SetIncrementalLoadAllowed(bool ila)
    {
        _incrementalLoadingAllowed = ila;
    }

    public override void SetOrientation(int orient = 0)
    {
        _orientation = orient;
    }

    public override bool IncrementalLoadAllowed() => _incrementalLoadingAllowed;

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

    public override bool IsG0 => false;

    public override int Orientation() => _orientation;

    public override D1 G1Target() => _g1Target;
}
