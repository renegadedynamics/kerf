// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0Constraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HPG0Constraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A G0 constraint of a non-linear plate: the surface passes through a target point at a UV point.</summary>
internal class Hpg0Constraint : HgppConstraint
{
    private readonly Xyz _xyzTarget;
    private bool _uvIsFree;
    private bool _incrementalLoadingAllowed;

    public Hpg0Constraint(in Xy uv, in Xyz value)
    {
        _xyzTarget = value;
        SetUV(uv);
        SetActiveOrder(0);
        _uvIsFree = false;
        _incrementalLoadingAllowed = false;
    }

    public override void SetUVFreeSliding(bool uvFree)
    {
        _uvIsFree = uvFree;
    }

    public override void SetIncrementalLoadAllowed(bool ila)
    {
        _incrementalLoadingAllowed = ila;
    }

    public override bool UVFreeSliding() => _uvIsFree;

    public override bool IncrementalLoadAllowed() => _incrementalLoadingAllowed;

    public override int ActiveOrder()
    {
        if (_activeOrder < 0)
        {
            return _activeOrder;
        }
        else
        {
            return 0;
        }
    }

    public override bool IsG0 => true;

    public override Xyz G0Target() => _xyzTarget;
}
