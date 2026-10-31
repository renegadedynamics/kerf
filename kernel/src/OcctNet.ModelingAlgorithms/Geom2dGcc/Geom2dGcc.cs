// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// Qualified curves for the Geom2dGcc solvers: circles and lines tangent to curves, through points, or centred on
/// curves.
/// </summary>
public static class Geom2dGcc
{
    /// <summary>A curve whose solutions are not qualified: all solutions apply.</summary>
    public static Geom2dGcc_QualifiedCurve Unqualified(Geom2dAdaptor_Curve Curve)
    {
        return new Geom2dGcc_QualifiedCurve(Curve, GccEnt_unqualified);
    }

    /// <summary>A curve that the solutions enclose.</summary>
    public static Geom2dGcc_QualifiedCurve Enclosing(Geom2dAdaptor_Curve Curve)
    {
        return new Geom2dGcc_QualifiedCurve(Curve, GccEnt_enclosing);
    }

    /// <summary>A curve that encloses the solutions.</summary>
    public static Geom2dGcc_QualifiedCurve Enclosed(Geom2dAdaptor_Curve Curve)
    {
        return new Geom2dGcc_QualifiedCurve(Curve, GccEnt_enclosed);
    }

    /// <summary>A curve outside the solutions, and they outside it.</summary>
    public static Geom2dGcc_QualifiedCurve Outside(Geom2dAdaptor_Curve Curve)
    {
        return new Geom2dGcc_QualifiedCurve(Curve, GccEnt_outside);
    }
}
