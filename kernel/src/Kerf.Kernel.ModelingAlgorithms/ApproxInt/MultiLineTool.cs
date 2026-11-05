// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_MultiLineTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Approximation.Interfaces;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The line tool of <see cref="MultiLine{TheLine}"/> for the algorithms of AppParCurves and Approx: every
/// member forwards to the MultiLine; there are never curvatures.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_MultiLineTool.lxx, ported once as a generic struct of static members implementing
/// <see cref="IApproxLineTool{MultiLine}"/> (decisions 142, 202 and 607). OCCT's instantiations
/// (GeomInt_TheMultiLineToolOfWLApprox, BRepApprox_TheMultiLineToolOfApprox) are this struct over IntPatch_WLine and
/// BRepApprox_ApproxLine: a struct can't be derived, and no caller names them.
/// </remarks>
/// <typeparam name="TheLine">The walking line.</typeparam>
internal readonly struct MultiLineTool<TheLine> : IApproxLineTool<global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine>>
    where TheLine : class, IWLine<TheLine>
{
    public static int NbP2d(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml) => (ml.P2dCount);

    public static int NbP3d(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml) => (ml.P3dCount);

    public static int FirstPoint(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml) => (ml.FirstPoint());

    public static int LastPoint(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml) => (ml.LastPoint());

    public static void Value(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Point3D> tabPnt)
    {
        ml.Value(index, tabPnt);
    }

    public static void Value(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Point2D> tabPnt2d)
    {
        ml.Value(index, tabPnt2d);
    }

    public static void Value(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Point3D> tabPnt,
                             NCollection_Array1<Point2D> tabPnt2d)
    {
        ml.Value(index, tabPnt, tabPnt2d);
    }

    public static bool Tangency(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector3D> tabVec)
    {
        return (ml.Tangency(index, tabVec));
    }

    public static bool Tangency(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector2D> tabVec2d)
    {
        return (ml.Tangency(index, tabVec2d));
    }

    public static bool Tangency(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector3D> tabVec,
                                NCollection_Array1<Vector2D> tabVec2d)
    {
        return (ml.Tangency(index, tabVec, tabVec2d));
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector3D> tabVec)
    {
        return false;
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector2D> tabVec2d)
    {
        return false;
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int index, NCollection_Array1<Vector3D> tabVec,
                                 NCollection_Array1<Vector2D> tabVec2d)
    {
        return false;
    }

    public static Status WhatStatus(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int i1, int i2)
    {
        //--  PointsAdded,
        //--  NoPointsAdded,
        //--  NoApproximation
        //--  Approx_PointsAdded
        return (ml.WhatStatus());
    }

    public static global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> MakeMLBetween(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int i1, int i2,
                                                             int nbPMin)
    {
        return (ml.MakeMLBetween(i1, i2, nbPMin));
    }

    public static bool MakeMLOneMorePoint(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, int i1, int i2, int indbad,
                                          out global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> otherLine)
    {
        return (ml.MakeMLOneMorePoint(i1, i2, indbad, out otherLine));
    }

    /// <summary>Writes the points of the MultiLine (OCCT prints them on the standard output).</summary>
    public static void Dump(global::Kerf.Engine.Intersection.Approximation.MultiLine<TheLine> ml, TextWriter theOut)
    {
        ml.Dump(theOut);
    }
}
