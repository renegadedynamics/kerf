// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// A MultiLine for the AppParCurves tests, written as AppDef_MultiLine will be: points 1 to NbPoints, each made
/// of NbP3d 3d points and NbP2d 2d points, with optional tangents and curvatures at chosen points.
/// </summary>
internal sealed class AppParCurves_TestLine
{
    public readonly gp_Pnt[][] P3d; // [point - 1][curve - 1]
    public readonly gp_Pnt2d[][] P2d;
    public readonly Dictionary<int, (gp_Vec[] V3d, gp_Vec2d[] V2d)> Tangents = [];
    public readonly Dictionary<int, (gp_Vec[] V3d, gp_Vec2d[] V2d)> Curvatures = [];

    public AppParCurves_TestLine(int theNbPoints, int theNbP3d, int theNbP2d)
    {
        P3d = new gp_Pnt[theNbPoints][];
        P2d = new gp_Pnt2d[theNbPoints][];
        for (int i = 0; i < theNbPoints; i++)
        {
            P3d[i] = new gp_Pnt[theNbP3d];
            P2d[i] = new gp_Pnt2d[theNbP2d];
        }
    }

    public int NbPoints => P3d.Length;

    public int NbP3d => P3d[0].Length;

    public int NbP2d => P2d[0].Length;

    /// <summary>Samples the curves of <paramref name="theCurve"/> at <paramref name="theParams"/> (0-based array).</summary>
    public static AppParCurves_TestLine Sample(AppParCurves_MultiCurve theCurve, int theNbP3d, int theNbP2d, double[] theParams)
    {
        AppParCurves_TestLine aLine = new(theParams.Length, theNbP3d, theNbP2d);
        for (int i = 0; i < theParams.Length; i++)
        {
            for (int c = 1; c <= theNbP3d; c++)
            {
                theCurve.Value(c, theParams[i], out aLine.P3d[i][c - 1]);
            }
            for (int c = 1; c <= theNbP2d; c++)
            {
                theCurve.Value(theNbP3d + c, theParams[i], out aLine.P2d[i][c - 1]);
            }
        }
        return aLine;
    }

    /// <summary>Stores the derivatives of <paramref name="theCurve"/> at the point <paramref name="theIndex"/> as its tangents.</summary>
    public void SetTangents(AppParCurves_MultiCurve theCurve, int theIndex, double theParam)
    {
        gp_Vec[] aV3d = new gp_Vec[NbP3d];
        gp_Vec2d[] aV2d = new gp_Vec2d[NbP2d];
        for (int c = 1; c <= NbP3d; c++)
        {
            theCurve.D1(c, theParam, out _, out aV3d[c - 1]);
        }
        for (int c = 1; c <= NbP2d; c++)
        {
            theCurve.D1(NbP3d + c, theParam, out _, out aV2d[c - 1]);
        }
        Tangents[theIndex] = (aV3d, aV2d);
    }
}

/// <summary>The line tool of <see cref="AppParCurves_TestLine"/>, as AppDef_MyLineTool will be for AppDef_MultiLine.</summary>
internal struct AppParCurves_TestLineTool : IAppParCurves_LineTool<AppParCurves_TestLine>
{
    public static int FirstPoint(AppParCurves_TestLine ML) => 1;

    public static int LastPoint(AppParCurves_TestLine ML) => ML.NbPoints;

    public static int NbP2d(AppParCurves_TestLine ML) => ML.NbP2d;

    public static int NbP3d(AppParCurves_TestLine ML) => ML.NbP3d;

    public static void Value(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Pnt> tabPt)
    {
        for (int c = 0; c < ML.NbP3d; c++)
        {
            tabPt[tabPt.Lower() + c] = ML.P3d[MPointIndex - 1][c];
        }
    }

    public static void Value(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Pnt2d> tabPt2d)
    {
        for (int c = 0; c < ML.NbP2d; c++)
        {
            tabPt2d[tabPt2d.Lower() + c] = ML.P2d[MPointIndex - 1][c];
        }
    }

    public static void Value(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Pnt> tabPt,
                             NCollection_Array1<gp_Pnt2d> tabPt2d)
    {
        Value(ML, MPointIndex, tabPt);
        Value(ML, MPointIndex, tabPt2d);
    }

    public static bool Tangency(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec> tabV)
    {
        if (!ML.Tangents.TryGetValue(MPointIndex, out var aT))
        {
            return false;
        }
        for (int c = 0; c < ML.NbP3d; c++)
        {
            tabV[tabV.Lower() + c] = aT.V3d[c];
        }
        return true;
    }

    public static bool Tangency(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec2d> tabV2d)
    {
        if (!ML.Tangents.TryGetValue(MPointIndex, out var aT))
        {
            return false;
        }
        for (int c = 0; c < ML.NbP2d; c++)
        {
            tabV2d[tabV2d.Lower() + c] = aT.V2d[c];
        }
        return true;
    }

    public static bool Tangency(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec> tabV,
                                NCollection_Array1<gp_Vec2d> tabV2d) =>
        Tangency(ML, MPointIndex, tabV) && Tangency(ML, MPointIndex, tabV2d);

    public static bool Curvature(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec> tabV)
    {
        if (!ML.Curvatures.TryGetValue(MPointIndex, out var aC))
        {
            return false;
        }
        for (int c = 0; c < ML.NbP3d; c++)
        {
            tabV[tabV.Lower() + c] = aC.V3d[c];
        }
        return true;
    }

    public static bool Curvature(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec2d> tabV2d)
    {
        if (!ML.Curvatures.TryGetValue(MPointIndex, out var aC))
        {
            return false;
        }
        for (int c = 0; c < ML.NbP2d; c++)
        {
            tabV2d[tabV2d.Lower() + c] = aC.V2d[c];
        }
        return true;
    }

    public static bool Curvature(AppParCurves_TestLine ML, int MPointIndex, NCollection_Array1<gp_Vec> tabV,
                                 NCollection_Array1<gp_Vec2d> tabV2d) =>
        Curvature(ML, MPointIndex, tabV) && Curvature(ML, MPointIndex, tabV2d);
}
