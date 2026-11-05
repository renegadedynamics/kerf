// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Foundation;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_MultiLine, BRepFill_ComputeCLine, BRepFill_ApproxSeewing and BRepFill_TrimSurfaceTool (unit
/// 5O2), BRepFill_Evolved's tools for the faces of an evolved shape that a bisector separates. On a face in the XY plane
/// the MultiLine maps a point of the bisector to its own (u, v), which the tests check exactly or within the
/// approximation's tolerance. OCCT V8_0_1 has no GTests for these classes.
/// </summary>
public class BRepFill_TrimSurfaceToolTests
{
    // The rectangle [0, 10] x [0, 5] of the XY plane (frozen, as a caller's) and its edge on y = 0
    private static (Face Face, Edge Bottom) Rectangle()
    {
        Face aFace = new MakeFace(new Plane3D(), 0.0, 10.0, 0.0, 5.0).Face();
        for (Explorer anExp = new(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Tool.UVPoints(anEdge, aFace, out Point2D aP1, out Point2D aP2);
            if (Math.Abs(aP1.Y) < 1.0e-12 && Math.Abs(aP2.Y) < 1.0e-12)
            {
                return (aFace, anEdge);
            }
        }
        throw new InvalidOperationException("no bottom edge");
    }

    private static TrimmedCurve2D Segment(double x1, double y1, double x2, double y2)
    {
        Point2D aP1 = new(x1, y1), aP2 = new(x2, y2);
        return new TrimmedCurve2D(new LineCurve2D(aP1, new Direction2D(new Vector2D(aP1, aP2))), 0.0, aP1.DistanceTo(aP2));
    }

    // An arc of the circle of centre (5, 0) and radius 3, inside the rectangle
    private static TrimmedCurve2D Arc() => new(new CircleCurve2D(new Axis2D(new Point2D(5, 0), new Direction2D(1, 0)), 3.0), 0.3, 2.8);

    [Fact]
    public void MultiLine_on_a_plane_maps_the_bisector_to_its_parameters()
    {
        (Face aFace, Edge anEdge) = Rectangle();
        TrimmedCurve2D aBis = Arc();
        FillingMultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        Assert.False(aML.IsParticularCase);
        Assert.Equal(Continuity.C0, aML.Continuity());
        Assert.Equal(0.3, aML.FirstParameter());
        Assert.Equal(2.8, aML.LastParameter());
        aML.GetNumberOfPoints(out int aNb3d, out int aNb2d);
        Assert.Equal((1, 2), (aNb3d, aNb2d));
        for (int i = 0; i <= 10; i++)
        {
            double u = 0.3 + i * 0.25;
            Point2D aP = aBis.Value(u);
            Point2D aUV1 = aML.ValueOnF1(u), aUV2 = aML.ValueOnF2(u);
            Assert.True(aUV1.DistanceTo(aP) < 1.0e-9, $"{u}: {aUV1} for {aP}");
            Assert.True(aUV2.DistanceTo(aP) < 1.0e-9);
            Assert.True(aML.Value(u).DistanceTo(new Point3D(aP.X, aP.Y, 0)) < 1.0e-9);
            aML.Value3dOnF1OnF2(u, out Point3D aP3d, out Point2D aPF1, out Point2D aPF2);
            Assert.Equal(aML.Value(u), aP3d);
            Assert.Equal(aUV1, aPF1);
            Assert.Equal(aUV2, aPF2);
        }
        NCollection_Array1<Vector2D> aV2d = new(1, 2);
        NCollection_Array1<Vector3D> aV3d = new(1, 1);
        Assert.False(aML.D1(1.0, aV2d, aV3d));
    }

    [Fact]
    public void MultiLine_between_planes_on_a_line_is_the_exact_particular_case()
    {
        (Face aFace, Edge anEdge) = Rectangle();
        TrimmedCurve2D aBis = Segment(2, 1, 8, 4);
        FillingMultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        Assert.True(aML.IsParticularCase);
        aML.Curves(out Curve3D? aCurve, out Curve2D? aPC1, out Curve2D? aPC2);
        Assert.NotNull(aCurve);
        Assert.NotNull(aPC1);
        Assert.NotNull(aPC2);
        Assert.True(aCurve.Value(aCurve.FirstParameter).DistanceTo(new Point3D(2, 1, 0)) < 1.0e-9);
        Assert.True(aCurve.Value(aCurve.LastParameter).DistanceTo(new Point3D(8, 4, 0)) < 1.0e-9);
        Assert.True(aPC1.Value(aPC1.FirstParameter).DistanceTo(new Point2D(2, 1)) < 1.0e-9);
        Assert.True(aPC2.Value(aPC2.LastParameter).DistanceTo(new Point2D(8, 4)) < 1.0e-9);
    }

    [Fact]
    public void ApproxSeewing_and_ComputeCLine_follow_the_MultiLine()
    {
        (Face aFace, Edge anEdge) = Rectangle();
        TrimmedCurve2D aBis = Arc();
        FillingMultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        ApproxSeewing anEmpty = new();
        Assert.False(anEmpty.IsDone);
        EngineAssert.Throws<NotDoneException>(() => anEmpty.Curve());

        ApproxSeewing anApprox = new(aML);
        Assert.True(anApprox.IsDone);
        Curve3D aCurve = anApprox.Curve();
        Curve2D aPC1 = anApprox.CurveOnF1(), aPC2 = anApprox.CurveOnF2();
        Assert.True(aCurve.Value(aCurve.FirstParameter).DistanceTo(aML.Value(0.3)) < 1.0e-6);
        Assert.True(aCurve.Value(aCurve.LastParameter).DistanceTo(aML.Value(2.8)) < 1.0e-6);
        for (int i = 0; i <= 20; i++)
        {
            double t = aCurve.FirstParameter + i * (aCurve.LastParameter - aCurve.FirstParameter) / 20;
            Point3D aP = aCurve.Value(t);
            // on the circle, at the height of the plane, and the pcurves give the same point
            Assert.True(Math.Abs(new Point2D(aP.X, aP.Y).DistanceTo(new Point2D(5, 0)) - 3.0) < 1.0e-3, $"{t}: {aP}");
            Assert.True(Math.Abs(aP.Z) < 1.0e-9);
            Assert.True(aPC1.Value(t).DistanceTo(new Point2D(aP.X, aP.Y)) < 1.0e-3);
            Assert.True(aPC2.Value(t).DistanceTo(new Point2D(aP.X, aP.Y)) < 1.0e-3);
        }

        FillingComputeCLine aFit = new(aML, 3, 8, 1.0e-5, 1.0e-5, true);
        Assert.True(aFit.MultiCurveCount >= 1);
        aFit.Parameters(1, out double aFirst, out double _);
        aFit.Parameters(aFit.MultiCurveCount, out double _, out double aLast);
        Assert.Equal(0.3, aFirst, 12);
        Assert.Equal(2.8, aLast, 12);
        for (int k = 1; k <= aFit.MultiCurveCount; k++)
        {
            aFit.Error(k, out double aTol3d, out double aTol2d);
            Assert.True(aTol3d < 1.0e-3 && aTol2d < 1.0e-3, $"{aTol3d} {aTol2d}");
            MultiCurve aMC = aFit.Value(k);
            Assert.Equal(3, aMC.NbCurves());
        }
    }

    [Fact]
    public void TrimSurfaceTool_intersects_projects_and_classifies()
    {
        (Face aFace, Edge anEdge) = Rectangle();
        string aBefore = Text(aFace);

        // a bisector crossing the bottom edge at (3.2, 0)
        TrimmedCurve2D aBis = Segment(2, -1, 8, 4);
        TrimSurfaceTool aTool = new(aBis, aFace, aFace, anEdge, anEdge, false, false);
        NCollection_Sequence<Point3D> aPoints = new();
        aTool.IntersectWith(anEdge, anEdge, aPoints);
        Assert.Equal(1, aPoints.Length());
        Point3D aPar = aPoints.Value(1);
        Assert.True(aBis.Value(aPar.X).DistanceTo(new Point2D(3.2, 0)) < 1.0e-9);
        Curve3D anEdgeCurve = Tool.Curve(anEdge, out double _, out double _)!;
        Assert.True(anEdgeCurve.Value(aPar.Y).DistanceTo(new Point3D(3.2, 0, 0)) < 1.0e-9);
        Assert.Equal(aPar.Y, aPar.Z);

        Assert.True(aTool.IsOnFace(new Point2D(5, 2)));
        Assert.False(aTool.IsOnFace(new Point2D(20, 20)));
        Assert.True(anEdgeCurve.Value(aTool.ProjOn(new Point2D(3, 7), anEdge)).DistanceTo(new Point3D(3, 0, 0)) < 1.0e-9);

        aTool.Project(0.0, aBis.LastParameter, out Curve3D? aCurve, out Curve2D? aPC1, out Curve2D? _, out Continuity _);
        Assert.NotNull(aCurve);
        Assert.NotNull(aPC1);
        Assert.True(aCurve.Value(aCurve.LastParameter).DistanceTo(new Point3D(8, 4, 0)) < 1.0e-9);

        // the caller's face is never written
        Assert.Equal(aBefore, Text(aFace));
    }

    [Fact]
    public void TrimSurfaceTool_without_intersection_lengthens_the_bisector_in_place_as_OCCT()
    {
        (Face aFace, Edge anEdge) = Rectangle();
        // ends 5e-6 above the edge: found only once the curves are lengthened by the tolerance, in each of the two calls
        TrimmedCurve2D aBis = Segment(5, 3, 5, 5.0e-6);
        double aLast = aBis.LastParameter;
        TrimSurfaceTool aTool = new(aBis, aFace, aFace, anEdge, anEdge, false, false);
        NCollection_Sequence<Point3D> aPoints = new();
        aTool.IntersectWith(anEdge, anEdge, aPoints);
        Assert.Equal(1, aPoints.Length());
        Assert.Equal(aLast + 2 * 1.0e-6, aBis.LastParameter, 12);
    }

    private static string Text(in Shape theShape)
    {
        using StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }
}
