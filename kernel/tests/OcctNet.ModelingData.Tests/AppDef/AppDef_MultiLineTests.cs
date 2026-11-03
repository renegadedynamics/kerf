// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.Approx_Status;

namespace OcctNet.ModelingData.Tests;

// AppDef_MultiPointConstraint, AppDef_MultiLine and AppDef_MyLineTool.
public class AppDef_MultiLineTests
{
    [Fact]
    public void MultiPointConstraint_StoresPointsTangentsAndCurvatures()
    {
        NCollection_Array1<Point3D> aP = NCollection_Array1<Point3D>.From([new Point3D(1, 2, 3)]);
        NCollection_Array1<Point2D> aP2d = NCollection_Array1<Point2D>.From([new Point2D(4, 5), new Point2D(6, 7)]);
        NCollection_Array1<Vector3D> aV = NCollection_Array1<Vector3D>.From([new Vector3D(1, 0, 0)]);
        NCollection_Array1<Vector2D> aV2d = NCollection_Array1<Vector2D>.From([new Vector2D(0, 1), new Vector2D(1, 1)]);
        NCollection_Array1<Vector3D> aC = NCollection_Array1<Vector3D>.From([new Vector3D(0, 0, 2)]);
        NCollection_Array1<Vector2D> aC2d = NCollection_Array1<Vector2D>.From([new Vector2D(3, 0), new Vector2D(0, 3)]);

        AppDef_MultiPointConstraint aMPC = new(aP, aP2d, aV, aV2d, aC, aC2d);
        Assert.Equal(1, aMPC.NbPoints());
        Assert.Equal(2, aMPC.NbPoints2d());
        Assert.True(aMPC.IsTangencyPoint());
        Assert.True(aMPC.IsCurvaturePoint());
        Assert.Equal(new Vector3D(1, 0, 0), aMPC.Tang(1));
        Assert.Equal(new Vector2D(1, 1), aMPC.Tang2d(3));
        Assert.Equal(new Vector3D(0, 0, 2), aMPC.Curv(1));
        Assert.Equal(new Vector2D(3, 0), aMPC.Curv2d(2));
        Assert.Equal(new Point2D(6, 7), aMPC.Point2d(3));
        Assert.Throws<OutOfRangeException>(() => aMPC.Tang(2));
        Assert.Throws<OutOfRangeException>(() => aMPC.Tang2d(1));
        Assert.Throws<ConstructionErrorException>(() => new AppDef_MultiPointConstraint(aP, aP2d, aV, aV2d, aC, NCollection_Array1<Vector2D>.From([new Vector2D(3, 0)])));

        AppDef_MultiPointConstraint aPlain = new(2, 0);
        Assert.False(aPlain.IsTangencyPoint());
        aPlain.SetTang(2, new Vector3D(0, 1, 0));
        Assert.True(aPlain.IsTangencyPoint());
        Assert.False(aPlain.IsCurvaturePoint());
        aPlain.SetCurv(1, new Vector3D(0, 0, 1));
        Assert.True(aPlain.IsCurvaturePoint());
        Assert.Throws<OutOfRangeException>(() => aPlain.SetTang(3, new Vector3D(1, 0, 0)));
        Assert.Equal("AppDef_MultiPointConstraint dump:\n", aPlain.ToString());
    }

    [Fact]
    public void MultiLine_StoresMultiPoints()
    {
        NCollection_Array1<Point3D> aPnts = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 1, 0)]);
        AppDef_MultiLine aLine = new(aPnts);
        Assert.Equal(3, aLine.NbMultiPoints());
        Assert.Equal(1, aLine.NbPoints());
        Assert.Equal(new Point3D(2, 1, 0), aLine.Value(3).Point(1));
        Assert.Throws<OutOfRangeException>(() => aLine.Value(0));
        Assert.Throws<OutOfRangeException>(() => aLine.Value(4));
        Assert.Throws<ConstructionErrorException>(() => new AppDef_MultiLine(-1));
        Assert.Equal("AppDef_MultiLine dump:\nIt contains 3 MultiPointConstraint\n", aLine.ToString());

        AppDef_MultiLine aLine2d = new(NCollection_Array1<Point2D>.From([new Point2D(1, 1), new Point2D(2, 2)]));
        Assert.Equal(new Point2D(2, 2), aLine2d.Value(2).Point2d(1));
    }

    [Fact]
    public void MyLineTool_ReadsTheLine()
    {
        int aNb = 3;
        AppDef_MultiLine aLine = new(aNb);
        for (int i = 1; i <= aNb; i++)
        {
            NCollection_Array1<Point3D> aP = NCollection_Array1<Point3D>.From([new Point3D(i, 0, 0)]);
            NCollection_Array1<Point2D> aP2d = NCollection_Array1<Point2D>.From([new Point2D(0, i)]);
            if (i == 2)
            {
                aLine.SetValue(i, new AppDef_MultiPointConstraint(aP, aP2d, NCollection_Array1<Vector3D>.From([new Vector3D(1, 0, 0)]),
                                                                  NCollection_Array1<Vector2D>.From([new Vector2D(0, 1)])));
            }
            else
            {
                aLine.SetValue(i, new AppDef_MultiPointConstraint(aP, aP2d));
            }
        }
        Assert.Equal(1, AppDef_MyLineTool.FirstPoint(aLine));
        Assert.Equal(3, AppDef_MyLineTool.LastPoint(aLine));
        Assert.Equal(1, AppDef_MyLineTool.NbP3d(aLine));
        Assert.Equal(1, AppDef_MyLineTool.NbP2d(aLine));

        NCollection_Array1<Point3D> aTab = new(5, 5);
        NCollection_Array1<Point2D> aTab2d = new(0, 0);
        AppDef_MyLineTool.Value(aLine, 3, aTab, aTab2d);
        Assert.Equal(new Point3D(3, 0, 0), aTab[5]);
        Assert.Equal(new Point2D(0, 3), aTab2d[0]);

        NCollection_Array1<Vector3D> aV = new(1, 1);
        NCollection_Array1<Vector2D> aV2d = new(1, 1);
        Assert.False(AppDef_MyLineTool.Tangency(aLine, 1, aV, aV2d));
        Assert.True(AppDef_MyLineTool.Tangency(aLine, 2, aV, aV2d));
        Assert.Equal(new Vector2D(0, 1), aV2d[1]);
        Assert.False(AppDef_MyLineTool.Curvature(aLine, 2, aV, aV2d));

        Assert.Equal(Approx_NoPointsAdded, AppDef_MyLineTool.WhatStatus(aLine, 1, 3));
        Assert.Same(aLine, AppDef_MyLineTool.MakeMLBetween(aLine, 1, 3, 2));
        Assert.False(AppDef_MyLineTool.MakeMLOneMorePoint(aLine, 1, 3, 2, out _));
    }
}
