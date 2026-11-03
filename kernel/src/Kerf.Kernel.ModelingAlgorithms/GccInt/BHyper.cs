// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BHyper.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BHyper.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D hyperbola.</summary>
public sealed class BHyper : ConstraintBisec
{
    private readonly Hyperbola2D _hyp;

    public BHyper(in Hyperbola2D hyper)
    {
        _hyp = hyper;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Hyperbola2D Hyperbola()
    {
        return _hyp;
    }

    /// <summary>GccInt_Hpr.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Hpr;
    }
}
