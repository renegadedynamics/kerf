// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BCirc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D circle.</summary>
internal sealed class BCirc : ConstraintBisec
{
    private readonly Circle2D _cir;

    public BCirc(in Circle2D circ)
    {
        _cir = circ;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Circle2D Circle()
    {
        return _cir;
    }

    /// <summary>GccInt_Cir.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Cir;
    }
}
