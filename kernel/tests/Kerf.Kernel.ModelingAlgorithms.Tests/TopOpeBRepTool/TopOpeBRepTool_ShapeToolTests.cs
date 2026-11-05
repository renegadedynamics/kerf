// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>TopOpeBRepTool_ShapeTool (unit 7L) against OCCT's semantics on primitives.</summary>
public class TopOpeBRepTool_ShapeToolTests
{
    private static Shape Sub(in Shape S, ShapeType T, int k)
    {
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, T, M);
        return M[k - 1];
    }

    [Fact]
    public void TolerancesAndPoints()
    {
        Shape B = new MakeBox(1.0, 2.0, 3.0).Shape();
        Assert.Equal(0.0, ToolShapeTool.Tolerance(new Shape()));
        Assert.Equal(Tool.Tolerance(ShapeCasts.Face(Sub(B, ShapeType.Face, 1))), ToolShapeTool.Tolerance(Sub(B, ShapeType.Face, 1)));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => ToolShapeTool.Tolerance(B));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ProgramErrorException>(() => ToolShapeTool.Pnt(Sub(B, ShapeType.Edge, 1)));
        Assert.True(ToolShapeTool.Pnt(Sub(B, ShapeType.Vertex, 1)).IsEqual(Tool.Pnt(ShapeCasts.Vertex(Sub(B, ShapeType.Vertex, 1))), 0.0));
    }

    [Fact]
    public void SameOrientation()
    {
        Shape B = new MakeBox(1.0, 2.0, 3.0).Shape();
        Shape E = Sub(B, ShapeType.Edge, 1), F = Sub(B, ShapeType.Face, 1);
        Assert.True(ToolShapeTool.ShapesSameOriented(E, E));
        Assert.False(ToolShapeTool.ShapesSameOriented(E, E.Reversed()));
        Assert.False(ToolShapeTool.ShapesSameOriented(F, F.Reversed()));
        Assert.True(ToolShapeTool.ShapesSameOriented(B, B));

        // opposite faces of a box: parallel planes whose outward normals are opposite
        Shape F1 = Sub(B, ShapeType.Face, 1), F2 = Sub(B, ShapeType.Face, 2);
        Assert.False(ToolShapeTool.FacesSameOriented(F1, F2));
        Assert.True(ToolShapeTool.FacesSameOriented(F1, F2.Reversed()));
        // OCCT only compares lines; any other pair is "same oriented"
        Shape C = new ApiMakeCylinder(1.0, 2.0).Shape();
        Shape aCircle = TopExp_EdgeOfType(C, CurveKind.Circle);
        Assert.True(ToolShapeTool.EdgesSameOriented(aCircle, aCircle.Reversed()) == false);
        Assert.True(ToolShapeTool.CurvesSameOriented(new EdgeCurveAdaptor(ShapeCasts.Edge(aCircle)),
                                                                new EdgeCurveAdaptor(ShapeCasts.Edge(aCircle.Reversed()))));
    }

    private static Shape TopExp_EdgeOfType(in Shape S, CurveKind T)
    {
        for (Explorer ex = new(S, ShapeType.Edge); ex.More(); ex.Next())
        {
            if (new EdgeCurveAdaptor(ShapeCasts.Edge(ex.Current())).GetType() == T)
            {
                return ex.Current();
            }
        }
        throw new InvalidOperationException();
    }

    [Fact]
    public void SeamsAndPeriods()
    {
        Shape C = new ApiMakeCylinder(1.0, 2.0).Shape();
        Shape aSeam = TopExp_EdgeOfType(C, CurveKind.Line);
        Shape aLateral = TopoDS_FaceOfType(C, SurfaceKind.Cylinder);
        Assert.True(ToolShapeTool.Closed(aSeam, aLateral));
        Assert.False(ToolShapeTool.Closed(TopExp_EdgeOfType(C, CurveKind.Circle), aLateral));

        // the seam is an iso-U line: a U parameter inside the bounds moves to the other side, a bound to the other bound
        double aPeriod = 2.0 * Math.PI;
        Assert.Equal(1.0 + aPeriod, ToolShapeTool.PeriodizeParameter(1.0, aSeam, aLateral), 1.0e-12);
        Assert.Equal(aPeriod, ToolShapeTool.PeriodizeParameter(0.0, aSeam, aLateral), 1.0e-12);
        Assert.Equal(0.0, ToolShapeTool.PeriodizeParameter(aPeriod, aSeam, aLateral), 1.0e-12);

        double u = 7.0, v = 0.5;
        ToolShapeTool.AdjustOnPeriodic(aLateral, ref u, ref v);
        Assert.Equal(7.0 - aPeriod, u, 1.0e-12);
        Assert.Equal(0.5, v);

        bool aUPer = false, aVPer = false;
        double u1 = 0, u2 = 0, v1 = 0, v2 = 0;
        ToolShapeTool.UVBOUNDS(ShapeCasts.Face(aLateral), ref aUPer, ref aVPer, ref u1, ref u2, ref v1, ref v2);
        Assert.True(aUPer);
        Assert.False(aVPer);
        Assert.Equal(aPeriod, u2, 1.0e-12);

        Assert.Equal(1.0e-3, ToolShapeTool.Resolution3d(ShapeCasts.Face(aLateral), 1.0e-3), 1.0e-12);
        Assert.IsType<LineCurve3D>(ToolShapeTool.BASISCURVE(new TrimmedCurve3D(new LineCurve3D(new Axis3D()), 0.0, 1.0)));
    }

    private static Shape TopoDS_FaceOfType(in Shape S, SurfaceKind T)
    {
        for (Explorer ex = new(S, ShapeType.Face); ex.More(); ex.Next())
        {
            if (new FaceSurfaceAdaptor(ShapeCasts.Face(ex.Current())).GetType() == T)
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
        Shape C = new ApiMakeCylinder(2.0, 2.0).Shape();
        Shape aCircle = TopExp_EdgeOfType(C, CurveKind.Circle);
        Direction3D T = new(), N = new(0, 0, 1);
        double aCurv = 0.0;
        double aTol = ToolShapeTool.EdgeData(aCircle, 0.0, ref T, ref N, ref aCurv);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Angular(), aTol);
        Assert.Equal(0.5, aCurv, 1.0e-12);
        Assert.Equal(1.0, N.Z);
    }
}
