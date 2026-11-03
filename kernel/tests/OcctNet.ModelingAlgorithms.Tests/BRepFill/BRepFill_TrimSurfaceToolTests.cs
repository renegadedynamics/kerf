// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_MultiLine, BRepFill_ComputeCLine, BRepFill_ApproxSeewing and BRepFill_TrimSurfaceTool (unit
/// 5O2), BRepFill_Evolved's tools for the faces of an evolved shape that a bisector separates. On a face in the XY plane
/// the MultiLine maps a point of the bisector to its own (u, v), which the tests check exactly or within the
/// approximation's tolerance. OCCT V8_0_1 has no GTests for these classes.
/// </summary>
public class BRepFill_TrimSurfaceToolTests
{
    // The rectangle [0, 10] x [0, 5] of the XY plane (frozen, as a caller's) and its edge on y = 0
    private static (TopoDS_Face Face, TopoDS_Edge Bottom) Rectangle()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new Plane3D(), 0.0, 10.0, 0.0, 5.0).Face();
        for (TopExp_Explorer anExp = new(aFace, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            BRep_Tool.UVPoints(anEdge, aFace, out Point2D aP1, out Point2D aP2);
            if (Math.Abs(aP1.Y()) < 1.0e-12 && Math.Abs(aP2.Y()) < 1.0e-12)
            {
                return (aFace, anEdge);
            }
        }
        throw new InvalidOperationException("no bottom edge");
    }

    private static Geom2d_TrimmedCurve Segment(double x1, double y1, double x2, double y2)
    {
        Point2D aP1 = new(x1, y1), aP2 = new(x2, y2);
        return new Geom2d_TrimmedCurve(new Geom2d_Line(aP1, new Direction2D(new Vector2D(aP1, aP2))), 0.0, aP1.Distance(aP2));
    }

    // An arc of the circle of centre (5, 0) and radius 3, inside the rectangle
    private static Geom2d_TrimmedCurve Arc() => new(new Geom2d_Circle(new Axis2D(new Point2D(5, 0), new Direction2D(1, 0)), 3.0), 0.3, 2.8);

    [Fact]
    public void MultiLine_on_a_plane_maps_the_bisector_to_its_parameters()
    {
        (TopoDS_Face aFace, TopoDS_Edge anEdge) = Rectangle();
        Geom2d_TrimmedCurve aBis = Arc();
        BRepFill_MultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        Assert.False(aML.IsParticularCase());
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
            Assert.True(aUV1.Distance(aP) < 1.0e-9, $"{u}: {aUV1} for {aP}");
            Assert.True(aUV2.Distance(aP) < 1.0e-9);
            Assert.True(aML.Value(u).Distance(new Point3D(aP.X(), aP.Y(), 0)) < 1.0e-9);
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
        (TopoDS_Face aFace, TopoDS_Edge anEdge) = Rectangle();
        Geom2d_TrimmedCurve aBis = Segment(2, 1, 8, 4);
        BRepFill_MultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        Assert.True(aML.IsParticularCase());
        aML.Curves(out Geom_Curve? aCurve, out Geom2d_Curve? aPC1, out Geom2d_Curve? aPC2);
        Assert.NotNull(aCurve);
        Assert.NotNull(aPC1);
        Assert.NotNull(aPC2);
        Assert.True(aCurve.Value(aCurve.FirstParameter()).Distance(new Point3D(2, 1, 0)) < 1.0e-9);
        Assert.True(aCurve.Value(aCurve.LastParameter()).Distance(new Point3D(8, 4, 0)) < 1.0e-9);
        Assert.True(aPC1.Value(aPC1.FirstParameter()).Distance(new Point2D(2, 1)) < 1.0e-9);
        Assert.True(aPC2.Value(aPC2.LastParameter()).Distance(new Point2D(8, 4)) < 1.0e-9);
    }

    [Fact]
    public void ApproxSeewing_and_ComputeCLine_follow_the_MultiLine()
    {
        (TopoDS_Face aFace, TopoDS_Edge anEdge) = Rectangle();
        Geom2d_TrimmedCurve aBis = Arc();
        BRepFill_MultiLine aML = new(aFace, aFace, anEdge, anEdge, false, false, aBis);

        BRepFill_ApproxSeewing anEmpty = new();
        Assert.False(anEmpty.IsDone());
        Assert.Throws<NotDoneException>(() => anEmpty.Curve());

        BRepFill_ApproxSeewing anApprox = new(aML);
        Assert.True(anApprox.IsDone());
        Geom_Curve aCurve = anApprox.Curve();
        Geom2d_Curve aPC1 = anApprox.CurveOnF1(), aPC2 = anApprox.CurveOnF2();
        Assert.True(aCurve.Value(aCurve.FirstParameter()).Distance(aML.Value(0.3)) < 1.0e-6);
        Assert.True(aCurve.Value(aCurve.LastParameter()).Distance(aML.Value(2.8)) < 1.0e-6);
        for (int i = 0; i <= 20; i++)
        {
            double t = aCurve.FirstParameter() + i * (aCurve.LastParameter() - aCurve.FirstParameter()) / 20;
            Point3D aP = aCurve.Value(t);
            // on the circle, at the height of the plane, and the pcurves give the same point
            Assert.True(Math.Abs(new Point2D(aP.X(), aP.Y()).Distance(new Point2D(5, 0)) - 3.0) < 1.0e-3, $"{t}: {aP}");
            Assert.True(Math.Abs(aP.Z()) < 1.0e-9);
            Assert.True(aPC1.Value(t).Distance(new Point2D(aP.X(), aP.Y())) < 1.0e-3);
            Assert.True(aPC2.Value(t).Distance(new Point2D(aP.X(), aP.Y())) < 1.0e-3);
        }

        BRepFill_ComputeCLine aFit = new(aML, 3, 8, 1.0e-5, 1.0e-5, true);
        Assert.True(aFit.NbMultiCurves() >= 1);
        aFit.Parameters(1, out double aFirst, out double _);
        aFit.Parameters(aFit.NbMultiCurves(), out double _, out double aLast);
        Assert.Equal(0.3, aFirst, 12);
        Assert.Equal(2.8, aLast, 12);
        for (int k = 1; k <= aFit.NbMultiCurves(); k++)
        {
            aFit.Error(k, out double aTol3d, out double aTol2d);
            Assert.True(aTol3d < 1.0e-3 && aTol2d < 1.0e-3, $"{aTol3d} {aTol2d}");
            AppParCurves_MultiCurve aMC = aFit.Value(k);
            Assert.Equal(3, aMC.NbCurves());
        }
    }

    [Fact]
    public void TrimSurfaceTool_intersects_projects_and_classifies()
    {
        (TopoDS_Face aFace, TopoDS_Edge anEdge) = Rectangle();
        string aBefore = Text(aFace);

        // a bisector crossing the bottom edge at (3.2, 0)
        Geom2d_TrimmedCurve aBis = Segment(2, -1, 8, 4);
        BRepFill_TrimSurfaceTool aTool = new(aBis, aFace, aFace, anEdge, anEdge, false, false);
        NCollection_Sequence<Point3D> aPoints = new();
        aTool.IntersectWith(anEdge, anEdge, aPoints);
        Assert.Equal(1, aPoints.Length());
        Point3D aPar = aPoints.Value(1);
        Assert.True(aBis.Value(aPar.X()).Distance(new Point2D(3.2, 0)) < 1.0e-9);
        Geom_Curve anEdgeCurve = BRep_Tool.Curve(anEdge, out double _, out double _)!;
        Assert.True(anEdgeCurve.Value(aPar.Y()).Distance(new Point3D(3.2, 0, 0)) < 1.0e-9);
        Assert.Equal(aPar.Y(), aPar.Z());

        Assert.True(aTool.IsOnFace(new Point2D(5, 2)));
        Assert.False(aTool.IsOnFace(new Point2D(20, 20)));
        Assert.True(anEdgeCurve.Value(aTool.ProjOn(new Point2D(3, 7), anEdge)).Distance(new Point3D(3, 0, 0)) < 1.0e-9);

        aTool.Project(0.0, aBis.LastParameter(), out Geom_Curve? aCurve, out Geom2d_Curve? aPC1, out Geom2d_Curve? _, out Continuity _);
        Assert.NotNull(aCurve);
        Assert.NotNull(aPC1);
        Assert.True(aCurve.Value(aCurve.LastParameter()).Distance(new Point3D(8, 4, 0)) < 1.0e-9);

        // the caller's face is never written
        Assert.Equal(aBefore, Text(aFace));
    }

    [Fact]
    public void TrimSurfaceTool_without_intersection_lengthens_the_bisector_in_place_as_OCCT()
    {
        (TopoDS_Face aFace, TopoDS_Edge anEdge) = Rectangle();
        // ends 5e-6 above the edge: found only once the curves are lengthened by the tolerance, in each of the two calls
        Geom2d_TrimmedCurve aBis = Segment(5, 3, 5, 5.0e-6);
        double aLast = aBis.LastParameter();
        BRepFill_TrimSurfaceTool aTool = new(aBis, aFace, aFace, anEdge, anEdge, false, false);
        NCollection_Sequence<Point3D> aPoints = new();
        aTool.IntersectWith(anEdge, anEdge, aPoints);
        Assert.Equal(1, aPoints.Length());
        Assert.Equal(aLast + 2 * 1.0e-6, aBis.LastParameter(), 12);
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aWriter = new();
        BRepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }
}
