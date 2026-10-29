// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Approx_Status;

namespace OcctNet.ModelingData.Tests;

// AppDef_MultiPointConstraint, AppDef_MultiLine and AppDef_MyLineTool.
public class AppDef_MultiLineTests
{
    [Fact]
    public void MultiPointConstraint_StoresPointsTangentsAndCurvatures()
    {
        NCollection_Array1<gp_Pnt> aP = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(1, 2, 3)]);
        NCollection_Array1<gp_Pnt2d> aP2d = NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(4, 5), new gp_Pnt2d(6, 7)]);
        NCollection_Array1<gp_Vec> aV = NCollection_Array1<gp_Vec>.From([new gp_Vec(1, 0, 0)]);
        NCollection_Array1<gp_Vec2d> aV2d = NCollection_Array1<gp_Vec2d>.From([new gp_Vec2d(0, 1), new gp_Vec2d(1, 1)]);
        NCollection_Array1<gp_Vec> aC = NCollection_Array1<gp_Vec>.From([new gp_Vec(0, 0, 2)]);
        NCollection_Array1<gp_Vec2d> aC2d = NCollection_Array1<gp_Vec2d>.From([new gp_Vec2d(3, 0), new gp_Vec2d(0, 3)]);

        AppDef_MultiPointConstraint aMPC = new(aP, aP2d, aV, aV2d, aC, aC2d);
        Assert.Equal(1, aMPC.NbPoints());
        Assert.Equal(2, aMPC.NbPoints2d());
        Assert.True(aMPC.IsTangencyPoint());
        Assert.True(aMPC.IsCurvaturePoint());
        Assert.Equal(new gp_Vec(1, 0, 0), aMPC.Tang(1));
        Assert.Equal(new gp_Vec2d(1, 1), aMPC.Tang2d(3));
        Assert.Equal(new gp_Vec(0, 0, 2), aMPC.Curv(1));
        Assert.Equal(new gp_Vec2d(3, 0), aMPC.Curv2d(2));
        Assert.Equal(new gp_Pnt2d(6, 7), aMPC.Point2d(3));
        Assert.Throws<Standard_OutOfRange>(() => aMPC.Tang(2));
        Assert.Throws<Standard_OutOfRange>(() => aMPC.Tang2d(1));
        Assert.Throws<Standard_ConstructionError>(() => new AppDef_MultiPointConstraint(aP, aP2d, aV, aV2d, aC, NCollection_Array1<gp_Vec2d>.From([new gp_Vec2d(3, 0)])));

        AppDef_MultiPointConstraint aPlain = new(2, 0);
        Assert.False(aPlain.IsTangencyPoint());
        aPlain.SetTang(2, new gp_Vec(0, 1, 0));
        Assert.True(aPlain.IsTangencyPoint());
        Assert.False(aPlain.IsCurvaturePoint());
        aPlain.SetCurv(1, new gp_Vec(0, 0, 1));
        Assert.True(aPlain.IsCurvaturePoint());
        Assert.Throws<Standard_OutOfRange>(() => aPlain.SetTang(3, new gp_Vec(1, 0, 0)));
        Assert.Equal("AppDef_MultiPointConstraint dump:\n", aPlain.ToString());
    }

    [Fact]
    public void MultiLine_StoresMultiPoints()
    {
        NCollection_Array1<gp_Pnt> aPnts = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(2, 1, 0)]);
        AppDef_MultiLine aLine = new(aPnts);
        Assert.Equal(3, aLine.NbMultiPoints());
        Assert.Equal(1, aLine.NbPoints());
        Assert.Equal(new gp_Pnt(2, 1, 0), aLine.Value(3).Point(1));
        Assert.Throws<Standard_OutOfRange>(() => aLine.Value(0));
        Assert.Throws<Standard_OutOfRange>(() => aLine.Value(4));
        Assert.Throws<Standard_ConstructionError>(() => new AppDef_MultiLine(-1));
        Assert.Equal("AppDef_MultiLine dump:\nIt contains 3 MultiPointConstraint\n", aLine.ToString());

        AppDef_MultiLine aLine2d = new(NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 2)]));
        Assert.Equal(new gp_Pnt2d(2, 2), aLine2d.Value(2).Point2d(1));
    }

    [Fact]
    public void MyLineTool_ReadsTheLine()
    {
        int aNb = 3;
        AppDef_MultiLine aLine = new(aNb);
        for (int i = 1; i <= aNb; i++)
        {
            NCollection_Array1<gp_Pnt> aP = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(i, 0, 0)]);
            NCollection_Array1<gp_Pnt2d> aP2d = NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(0, i)]);
            if (i == 2)
            {
                aLine.SetValue(i, new AppDef_MultiPointConstraint(aP, aP2d, NCollection_Array1<gp_Vec>.From([new gp_Vec(1, 0, 0)]),
                                                                  NCollection_Array1<gp_Vec2d>.From([new gp_Vec2d(0, 1)])));
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

        NCollection_Array1<gp_Pnt> aTab = new(5, 5);
        NCollection_Array1<gp_Pnt2d> aTab2d = new(0, 0);
        AppDef_MyLineTool.Value(aLine, 3, aTab, aTab2d);
        Assert.Equal(new gp_Pnt(3, 0, 0), aTab[5]);
        Assert.Equal(new gp_Pnt2d(0, 3), aTab2d[0]);

        NCollection_Array1<gp_Vec> aV = new(1, 1);
        NCollection_Array1<gp_Vec2d> aV2d = new(1, 1);
        Assert.False(AppDef_MyLineTool.Tangency(aLine, 1, aV, aV2d));
        Assert.True(AppDef_MyLineTool.Tangency(aLine, 2, aV, aV2d));
        Assert.Equal(new gp_Vec2d(0, 1), aV2d[1]);
        Assert.False(AppDef_MyLineTool.Curvature(aLine, 2, aV, aV2d));

        Assert.Equal(Approx_NoPointsAdded, AppDef_MyLineTool.WhatStatus(aLine, 1, 3));
        Assert.Same(aLine, AppDef_MyLineTool.MakeMLBetween(aLine, 1, 3, 2));
        Assert.False(AppDef_MyLineTool.MakeMLOneMorePoint(aLine, 1, 3, 2, out _));
    }
}
