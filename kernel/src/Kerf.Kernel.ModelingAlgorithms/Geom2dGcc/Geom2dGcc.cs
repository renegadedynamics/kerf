// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// Qualified curves for the Geom2dGcc solvers: circles and lines tangent to curves, through points, or centred on
/// curves.
/// </summary>
internal static class Geom2dGcc
{
    /// <summary>A curve whose solutions are not qualified: all solutions apply.</summary>
    public static QualifiedCurve Unqualified(CurveAdaptor2D curve)
    {
        return new QualifiedCurve(curve, Qualifier.Unqualified);
    }

    /// <summary>A curve that the solutions enclose.</summary>
    public static QualifiedCurve Enclosing(CurveAdaptor2D curve)
    {
        return new QualifiedCurve(curve, Qualifier.Enclosing);
    }

    /// <summary>A curve that encloses the solutions.</summary>
    public static QualifiedCurve Enclosed(CurveAdaptor2D curve)
    {
        return new QualifiedCurve(curve, Qualifier.Enclosed);
    }

    /// <summary>A curve outside the solutions, and they outside it.</summary>
    public static QualifiedCurve Outside(CurveAdaptor2D curve)
    {
        return new QualifiedCurve(curve, Qualifier.Outside);
    }
}
