// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyLineTool.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Interfaces;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Approximation.Status;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The line tool of <see cref="MultiLine"/> for the algorithms of AppParCurves and Approx. It never adds
/// points when the algorithms ask for more.
/// </summary>
/// <remarks>
/// OCCT's class of static functions is an empty struct implementing the static members of
/// <see cref="IApproxLineTool{MultiLine}"/>, so that it can be the <c>LineTool</c> type argument of the generic
/// algorithms (decisions 142 and 202).
/// </remarks>
internal readonly struct MyLineTool : IApproxLineTool<MultiLine>
{
    /// <summary>The first index of the MultiPoints of the MultiLine: 1.</summary>
    public static int FirstPoint(MultiLine ml) => 1;

    /// <summary>The last index of the MultiPoints of the MultiLine.</summary>
    public static int LastPoint(MultiLine ml) => ml.MultiPointCount;

    /// <summary>The number of 2d points of a MultiLine.</summary>
    public static int NbP2d(MultiLine ml) => ml.Value(1).Points2dCount;

    /// <summary>The number of 3d points of a MultiLine.</summary>
    public static int NbP3d(MultiLine ml) => ml.Value(1).PointCount;

    /// <summary>The 3d points of the MultiPoint <paramref name="mPointIndex"/>.</summary>
    public static void Value(MultiLine ml, int mPointIndex, NCollection_Array1<Point3D> tabPt)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        int nbp3d = mpc.PointCount, low = tabPt.Lower();
        for (int i = 1; i <= nbp3d; i++)
        {
            tabPt[i + low - 1] = mpc.Point(i);
        }
    }

    /// <summary>The 2d points of the MultiPoint <paramref name="mPointIndex"/>.</summary>
    public static void Value(MultiLine ml, int mPointIndex, NCollection_Array1<Point2D> tabPt2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        int nbp3d = mpc.PointCount;
        int nbp2d = mpc.Points2dCount;
        int low = tabPt2d.Lower();
        for (int i = 1; i <= nbp2d; i++)
        {
            tabPt2d[i + low - 1] = mpc.Point2d(nbp3d + i);
        }
    }

    /// <summary>The 3d and 2d points of the MultiPoint <paramref name="mPointIndex"/>.</summary>
    public static void Value(MultiLine ml, int mPointIndex, NCollection_Array1<Point3D> tabPt,
                             NCollection_Array1<Point2D> tabPt2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        int i, nbp2d = mpc.Points2dCount, low2d = tabPt2d.Lower();
        int nbp3d = mpc.PointCount, low = tabPt.Lower();
        for (i = 1; i <= nbp3d; i++)
        {
            tabPt[i + low - 1] = mpc.Point(i);
        }
        for (i = 1; i <= nbp2d; i++)
        {
            tabPt2d[i + low2d - 1] = mpc.Point2d(nbp3d + i);
        }
    }

    /// <summary>The 3d tangents of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(MultiLine ml, int mPointIndex, NCollection_Array1<Vector3D> tabV)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsTangencyPoint)
        {
            int nbp3d = mpc.PointCount, low = tabV.Lower();
            for (int i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = mpc.Tang(i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 2d tangents of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(MultiLine ml, int mPointIndex, NCollection_Array1<Vector2D> tabV2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsTangencyPoint)
        {
            int nbp3d = mpc.PointCount;
            int nbp2d = mpc.Points2dCount, low = tabV2d.Lower();
            for (int i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low - 1] = mpc.Tang2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 3d and 2d tangents of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Tangency(MultiLine ml, int mPointIndex, NCollection_Array1<Vector3D> tabV,
                                NCollection_Array1<Vector2D> tabV2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsTangencyPoint)
        {
            int i, nbp3d = mpc.PointCount, low = tabV.Lower();
            int nbp2d = mpc.Points2dCount, low2d = tabV2d.Lower();
            for (i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = mpc.Tang(i);
            }
            for (i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low2d - 1] = mpc.Tang2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>Never called by the algorithms: returns <paramref name="ml"/>.</summary>
    public static MultiLine MakeMLBetween(MultiLine ml, int i1, int i2, int nbPMin)
    {
        return ml; // stub
    }

    /// <summary>
    /// Never called by the algorithms: returns false, and <paramref name="otherLine"/> is an undefined MultiLine
    /// (OCCT leaves the caller's default-constructed line unchanged).
    /// </summary>
    public static bool MakeMLOneMorePoint(MultiLine ml, int i1, int i2, int indbad, out MultiLine otherLine)
    {
        otherLine = new MultiLine();
        return false;
    }

    /// <summary>Approx_NoPointsAdded: this tool never adds points.</summary>
    public static Status WhatStatus(MultiLine ml, int i1, int i2)
    {
        return NoPointsAdded;
    }

    /// <summary>The 3d curvatures of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(MultiLine ml, int mPointIndex, NCollection_Array1<Vector3D> tabV)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsCurvaturePoint)
        {
            int nbp3d = mpc.PointCount, low = tabV.Lower();
            for (int i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = mpc.Curv(i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 2d curvatures of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(MultiLine ml, int mPointIndex, NCollection_Array1<Vector2D> tabV2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsCurvaturePoint)
        {
            int nbp3d = mpc.PointCount;
            int nbp2d = mpc.Points2dCount, low = tabV2d.Lower();
            for (int i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low - 1] = mpc.Curv2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>The 3d and 2d curvatures of the MultiPoint <paramref name="mPointIndex"/>; false if it has none.</summary>
    public static bool Curvature(MultiLine ml, int mPointIndex, NCollection_Array1<Vector3D> tabV,
                                 NCollection_Array1<Vector2D> tabV2d)
    {
        MultiPointConstraint mpc = ml.Value(mPointIndex);
        if (mpc.IsCurvaturePoint)
        {
            int i, nbp3d = mpc.PointCount, low = tabV.Lower();
            int nbp2d = mpc.Points2dCount, low2d = tabV2d.Lower();
            for (i = 1; i <= nbp3d; i++)
            {
                tabV[i + low - 1] = mpc.Curv(i);
            }
            for (i = 1; i <= nbp2d; i++)
            {
                tabV2d[i + low2d - 1] = mpc.Curv2d(nbp3d + i);
            }
            return true;
        }
        else
        {
            return false;
        }
    }
}
