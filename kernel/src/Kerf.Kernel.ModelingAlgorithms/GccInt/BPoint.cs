// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_BPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>A bisecting curve whose geometry is a 2D point.</summary>
internal sealed class BPoint : ConstraintBisec
{
    private readonly Point2D _pnt;

    public BPoint(in Point2D point)
    {
        _pnt = point;
    }

    /// <summary>The geometry of this bisecting curve.</summary>
    public override Point2D Point()
    {
        return _pnt;
    }

    /// <summary>GccInt_Pnt.</summary>
    public override ConstraintCurveType ArcType()
    {
        return ConstraintCurveType.Pnt;
    }
}
