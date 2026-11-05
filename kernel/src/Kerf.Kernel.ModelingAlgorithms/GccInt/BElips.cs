// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BElips.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BElips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D ellipse.</summary>
internal sealed class BElips : ConstraintBisec
{
    private readonly Ellipse2D _eli;

    public BElips(in Ellipse2D ellipse)
    {
        _eli = ellipse;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Ellipse2D Ellipse()
    {
        return _eli;
    }

    /// <summary>GccInt_Ell.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Ell;
    }
}
