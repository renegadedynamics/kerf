// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>TopOpeBRepTool_ShapeTool (unit 7L) against OCCT's semantics on primitives.</summary>
public class TopOpeBRepTool_ShapeToolTests
{
    private static TopoDS_Shape Sub(in TopoDS_Shape S, TopAbs_ShapeEnum T, int k)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, T, M);
        return M.FindKey(k);
    }

    [Fact]
    public void TolerancesAndPoints()
    {
        TopoDS_Shape B = new BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape();
        Assert.Equal(0.0, TopOpeBRepTool_ShapeTool.Tolerance(new TopoDS_Shape()));
        Assert.Equal(BRep_Tool.Tolerance(TopoDS.Face(Sub(B, TopAbs_FACE, 1))), TopOpeBRepTool_ShapeTool.Tolerance(Sub(B, TopAbs_FACE, 1)));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => TopOpeBRepTool_ShapeTool.Tolerance(B));
        Assert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => TopOpeBRepTool_ShapeTool.Pnt(Sub(B, TopAbs_EDGE, 1)));
        Assert.True(TopOpeBRepTool_ShapeTool.Pnt(Sub(B, TopAbs_VERTEX, 1)).IsEqual(BRep_Tool.Pnt(TopoDS.Vertex(Sub(B, TopAbs_VERTEX, 1))), 0.0));
    }

    [Fact]
    public void SameOrientation()
    {
        TopoDS_Shape B = new BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape();
        TopoDS_Shape E = Sub(B, TopAbs_EDGE, 1), F = Sub(B, TopAbs_FACE, 1);
        Assert.True(TopOpeBRepTool_ShapeTool.ShapesSameOriented(E, E));
        Assert.False(TopOpeBRepTool_ShapeTool.ShapesSameOriented(E, E.Reversed()));
        Assert.False(TopOpeBRepTool_ShapeTool.ShapesSameOriented(F, F.Reversed()));
        Assert.True(TopOpeBRepTool_ShapeTool.ShapesSameOriented(B, B));

        // opposite faces of a box: parallel planes whose outward normals are opposite
        TopoDS_Shape F1 = Sub(B, TopAbs_FACE, 1), F2 = Sub(B, TopAbs_FACE, 2);
        Assert.False(TopOpeBRepTool_ShapeTool.FacesSameOriented(F1, F2));
        Assert.True(TopOpeBRepTool_ShapeTool.FacesSameOriented(F1, F2.Reversed()));
        // OCCT only compares lines; any other pair is "same oriented"
        TopoDS_Shape C = new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape();
        TopoDS_Shape aCircle = TopExp_EdgeOfType(C, CurveKind.Circle);
        Assert.True(TopOpeBRepTool_ShapeTool.EdgesSameOriented(aCircle, aCircle.Reversed()) == false);
        Assert.True(TopOpeBRepTool_ShapeTool.CurvesSameOriented(new BRepAdaptor_Curve(TopoDS.Edge(aCircle)),
                                                                new BRepAdaptor_Curve(TopoDS.Edge(aCircle.Reversed()))));
    }

    private static TopoDS_Shape TopExp_EdgeOfType(in TopoDS_Shape S, CurveKind T)
    {
        for (TopExp_Explorer ex = new(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            if (new BRepAdaptor_Curve(TopoDS.Edge(ex.Current())).GetType() == T)
            {
                return ex.Current();
            }
        }
        throw new InvalidOperationException();
    }

    [Fact]
    public void SeamsAndPeriods()
    {
        TopoDS_Shape C = new BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape();
        TopoDS_Shape aSeam = TopExp_EdgeOfType(C, CurveKind.Line);
        TopoDS_Shape aLateral = TopoDS_FaceOfType(C, SurfaceKind.Cylinder);
        Assert.True(TopOpeBRepTool_ShapeTool.Closed(aSeam, aLateral));
        Assert.False(TopOpeBRepTool_ShapeTool.Closed(TopExp_EdgeOfType(C, CurveKind.Circle), aLateral));

        // the seam is an iso-U line: a U parameter inside the bounds moves to the other side, a bound to the other bound
        double aPeriod = 2.0 * Math.PI;
        Assert.Equal(1.0 + aPeriod, TopOpeBRepTool_ShapeTool.PeriodizeParameter(1.0, aSeam, aLateral), 1.0e-12);
        Assert.Equal(aPeriod, TopOpeBRepTool_ShapeTool.PeriodizeParameter(0.0, aSeam, aLateral), 1.0e-12);
        Assert.Equal(0.0, TopOpeBRepTool_ShapeTool.PeriodizeParameter(aPeriod, aSeam, aLateral), 1.0e-12);

        double u = 7.0, v = 0.5;
        TopOpeBRepTool_ShapeTool.AdjustOnPeriodic(aLateral, ref u, ref v);
        Assert.Equal(7.0 - aPeriod, u, 1.0e-12);
        Assert.Equal(0.5, v);

        bool aUPer = false, aVPer = false;
        double u1 = 0, u2 = 0, v1 = 0, v2 = 0;
        TopOpeBRepTool_ShapeTool.UVBOUNDS(TopoDS.Face(aLateral), ref aUPer, ref aVPer, ref u1, ref u2, ref v1, ref v2);
        Assert.True(aUPer);
        Assert.False(aVPer);
        Assert.Equal(aPeriod, u2, 1.0e-12);

        Assert.Equal(1.0e-3, TopOpeBRepTool_ShapeTool.Resolution3d(TopoDS.Face(aLateral), 1.0e-3), 1.0e-12);
        Assert.IsType<Geom_Line>(TopOpeBRepTool_ShapeTool.BASISCURVE(new Geom_TrimmedCurve(new Geom_Line(new Axis3D()), 0.0, 1.0)));
    }

    private static TopoDS_Shape TopoDS_FaceOfType(in TopoDS_Shape S, SurfaceKind T)
    {
        for (TopExp_Explorer ex = new(S, TopAbs_FACE); ex.More(); ex.Next())
        {
            if (new BRepAdaptor_Surface(TopoDS.Face(ex.Current())).GetType() == T)
            {
                return ex.Current();
            }
        }
        throw new InvalidOperationException();
    }

    [Fact]
    public void EdgeDataNeverComputesTheNormal()
    {
        // OCCT compares the curvature with RealLast, so the normal is never set (occt-issues 7L)
        TopoDS_Shape C = new BRepPrimAPI_MakeCylinder(2.0, 2.0).Shape();
        TopoDS_Shape aCircle = TopExp_EdgeOfType(C, CurveKind.Circle);
        Direction3D T = new(), N = new(0, 0, 1);
        double aCurv = 0.0;
        double aTol = TopOpeBRepTool_ShapeTool.EdgeData(aCircle, 0.0, ref T, ref N, ref aCurv);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), aTol);
        Assert.Equal(0.5, aCurv, 1.0e-12);
        Assert.Equal(1.0, N.Z());
    }
}
