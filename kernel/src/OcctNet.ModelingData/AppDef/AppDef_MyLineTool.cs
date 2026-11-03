// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyLineTool.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.Approx_Status;

namespace Occt;

/// <summary>
/// The line tool of <see cref="AppDef_MultiLine"/> for the algorithms of AppParCurves and Approx. It never adds
/// points when the algorithms ask for more.
/// </summary>
/// <remarks>
/// OCCT's class of static functions is an empty struct implementing the static members of
/// <see cref="IApprox_LineTool{MultiLine}"/>, so that it can be the <c>LineTool</c> type argument of the generic
/// algorithms (decisions 142 and 202).
/// </remarks>
public readonly struct AppDef_MyLineTool : IApprox_LineTool<AppDef_MultiLine>
{
    /// <summary>The first index of the MultiPoints of the MultiLine: 1.</summary>
    public static int FirstPoint(AppDef_MultiLine ML) => 1;

    /// <summary>The last index of the MultiPoints of the MultiLine.</summary>
    public static int LastPoint(AppDef_MultiLine ML) => ML.NbMultiPoints();

    /// <summary>The number of 2d points of a MultiLine.</summary>
    public static int NbP2d(AppDef_MultiLine ML) => ML.Value(1).NbPoints2d();

    /// <summary>The number of 3d points of a MultiLine.</summary>
    public static int NbP3d(AppDef_MultiLine ML) => ML.Value(1).NbPoints();

    /// <summary>The 3d points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    public static void Value(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Point3D> tabPt)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        int nbp3d = MPC.NbPoints(), low = tabPt.Lower();
        for (int i = 1; i <= nbp3d; i++)
        {
            tabPt[i + low - 1] = MPC.Point(i);
        }
    }

    /// <summary>The 2d points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    public static void Value(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Point2D> tabPt2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        int nbp3d = MPC.NbPoints();
        int nbp2d = MPC.NbPoints2d();
        int low = tabPt2d.Lower();
        for (int i = 1; i <= nbp2d; i++)
        {
            tabPt2d[i + low - 1] = MPC.Point2d(nbp3d + i);
        }
    }

    /// <summary>The 3d and 2d points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    public static void Value(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Point3D> tabPt,
                             NCollection_Array1<Point2D> tabPt2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        int i, nbp2d = MPC.NbPoints2d(), low2d = tabPt2d.Lower();
        int nbp3d = MPC.NbPoints(), low = tabPt.Lower();
        for (i = 1; i <= nbp3d; i++)
        {
            tabPt[i + low - 1] = MPC.Point(i);
        }
        for (i = 1; i <= nbp2d; i++)
        {
            tabPt2d[i + low2d - 1] = MPC.Point2d(nbp3d + i);
        }
    }

    /// <summary>The 3d tangents of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsTangencyPoint())
        {
            int nbp3d = MPC.NbPoints(), low = tabV.Lower();
            for (int i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = MPC.Tang(i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 2d tangents of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector2D> tabV2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsTangencyPoint())
        {
            int nbp3d = MPC.NbPoints();
            int nbp2d = MPC.NbPoints2d(), low = tabV2d.Lower();
            for (int i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low - 1] = MPC.Tang2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 3d and 2d tangents of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV,
                                NCollection_Array1<Vector2D> tabV2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsTangencyPoint())
        {
            int i, nbp3d = MPC.NbPoints(), low = tabV.Lower();
            int nbp2d = MPC.NbPoints2d(), low2d = tabV2d.Lower();
            for (i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = MPC.Tang(i);
            }
            for (i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low2d - 1] = MPC.Tang2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>Never called by the algorithms: returns <paramref name="theML"/>.</summary>
    public static AppDef_MultiLine MakeMLBetween(AppDef_MultiLine theML, int I1, int I2, int NbPMin)
    {
        return theML; // stub
    }

    /// <summary>
    /// Never called by the algorithms: returns false, and <paramref name="OtherLine"/> is an undefined MultiLine
    /// (OCCT leaves the caller's default-constructed line unchanged).
    /// </summary>
    public static bool MakeMLOneMorePoint(AppDef_MultiLine ML, int I1, int I2, int indbad, out AppDef_MultiLine OtherLine)
    {
        OtherLine = new AppDef_MultiLine();
        return false;
    }

    /// <summary>Approx_NoPointsAdded: this tool never adds points.</summary>
    public static Approx_Status WhatStatus(AppDef_MultiLine ML, int I1, int I2)
    {
        return Approx_NoPointsAdded;
    }

    /// <summary>The 3d curvatures of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsCurvaturePoint())
        {
            int nbp3d = MPC.NbPoints(), low = tabV.Lower();
            for (int i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = MPC.Curv(i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 2d curvatures of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector2D> tabV2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsCurvaturePoint())
        {
            int nbp3d = MPC.NbPoints();
            int nbp2d = MPC.NbPoints2d(), low = tabV2d.Lower();
            for (int i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low - 1] = MPC.Curv2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 3d and 2d curvatures of the MultiPoint <paramref name="MPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(AppDef_MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV,
                                 NCollection_Array1<Vector2D> tabV2d)
    {
        AppDef_MultiPointConstraint MPC = ML.Value(MPointIndex);
        if (MPC.IsCurvaturePoint())
        {
            int i, nbp3d = MPC.NbPoints(), low = tabV.Lower();
            int nbp2d = MPC.NbPoints2d(), low2d = tabV2d.Lower();
            for (i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = MPC.Curv(i);
            }
            for (i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low2d - 1] = MPC.Curv2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }
}
