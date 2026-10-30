// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_MultiLineTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The line tool of <see cref="ApproxInt_MultiLine{TheLine}"/> for the algorithms of AppParCurves and Approx: every
/// member forwards to the MultiLine; there are never curvatures.
/// </summary>
/// <remarks>
/// OCCT's generic file ApproxInt_MultiLineTool.lxx, ported once as a generic struct of static members implementing
/// <see cref="IApprox_LineTool{MultiLine}"/> (decisions 142, 202 and 607). OCCT's instantiations
/// (GeomInt_TheMultiLineToolOfWLApprox, BRepApprox_TheMultiLineToolOfApprox) are this struct over IntPatch_WLine and
/// BRepApprox_ApproxLine: a struct can't be derived, and no caller names them.
/// </remarks>
/// <typeparam name="TheLine">The walking line.</typeparam>
public readonly struct ApproxInt_MultiLineTool<TheLine> : IApprox_LineTool<ApproxInt_MultiLine<TheLine>>
    where TheLine : class, IApproxInt_WLine<TheLine>
{
    public static int NbP2d(ApproxInt_MultiLine<TheLine> ML) => (ML.NbP2d());

    public static int NbP3d(ApproxInt_MultiLine<TheLine> ML) => (ML.NbP3d());

    public static int FirstPoint(ApproxInt_MultiLine<TheLine> ML) => (ML.FirstPoint());

    public static int LastPoint(ApproxInt_MultiLine<TheLine> ML) => (ML.LastPoint());

    public static void Value(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Pnt> TabPnt)
    {
        ML.Value(Index, TabPnt);
    }

    public static void Value(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Pnt2d> TabPnt2d)
    {
        ML.Value(Index, TabPnt2d);
    }

    public static void Value(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Pnt> TabPnt,
                             NCollection_Array1<gp_Pnt2d> TabPnt2d)
    {
        ML.Value(Index, TabPnt, TabPnt2d);
    }

    public static bool Tangency(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec> TabVec)
    {
        return (ML.Tangency(Index, TabVec));
    }

    public static bool Tangency(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec2d> TabVec2d)
    {
        return (ML.Tangency(Index, TabVec2d));
    }

    public static bool Tangency(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec> TabVec,
                                NCollection_Array1<gp_Vec2d> TabVec2d)
    {
        return (ML.Tangency(Index, TabVec, TabVec2d));
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec> TabVec)
    {
        return false;
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec2d> TabVec2d)
    {
        return false;
    }

    /// <summary>False: there are no curvatures.</summary>
    public static bool Curvature(ApproxInt_MultiLine<TheLine> ML, int Index, NCollection_Array1<gp_Vec> TabVec,
                                 NCollection_Array1<gp_Vec2d> TabVec2d)
    {
        return false;
    }

    public static Approx_Status WhatStatus(ApproxInt_MultiLine<TheLine> ML, int I1, int I2)
    {
        //--  PointsAdded,
        //--  NoPointsAdded,
        //--  NoApproximation
        //--  Approx_PointsAdded
        return (ML.WhatStatus());
    }

    public static ApproxInt_MultiLine<TheLine> MakeMLBetween(ApproxInt_MultiLine<TheLine> ML, int I1, int I2,
                                                             int NbPMin)
    {
        return (ML.MakeMLBetween(I1, I2, NbPMin));
    }

    public static bool MakeMLOneMorePoint(ApproxInt_MultiLine<TheLine> ML, int I1, int I2, int indbad,
                                          out ApproxInt_MultiLine<TheLine> OtherLine)
    {
        return (ML.MakeMLOneMorePoint(I1, I2, indbad, out OtherLine));
    }

    /// <summary>Writes the points of the MultiLine (OCCT prints them on the standard output).</summary>
    public static void Dump(ApproxInt_MultiLine<TheLine> ML, TextWriter theOut)
    {
        ML.Dump(theOut);
    }
}
