// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The fillet surfaces by what they must be (decision 19805): the curve a fillet leaves on a face lies on it, in space and
/// by its pcurve; on the fillet both boundaries are where they say; sections are arcs of the radius, between those curves.
/// </summary>
public class FilletSurf_LawsTests
{
    private static NCollection_List<Shape> ListOf(params Shape[] theEdges)
    {
        var aList = new NCollection_List<Shape>();
        foreach (Shape anEdge in theEdges)
        {
            aList.Append(anEdge);
        }
        return aList;
    }

    private static Edge EdgeBetween(in Shape theShape, Point3D theA, Point3D theB)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
            Point3D aP1 = Tool.Pnt(aV1), aP2 = Tool.Pnt(aV2);
            if ((aP1.DistanceTo(theA) < 1e-7 && aP2.DistanceTo(theB) < 1e-7) || (aP1.DistanceTo(theB) < 1e-7 && aP2.DistanceTo(theA) < 1e-7))
            {
                return anEdge;
            }
        }
        throw new InvalidOperationException("no such edge");
    }

    // The circular edge at the top of a cylinder
    private static Edge TopCircle(in Shape theCylinder, double theZ)
    {
        for (var anExp = new Explorer(theCylinder, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Curve3D? aCurve = Tool.Curve(anEdge, out _, out _);
            if (aCurve is CircleCurve3D aCircle && Math.Abs(aCircle.Circle.Location.Z - theZ) < 1e-7)
            {
                return anEdge;
            }
        }
        throw new InvalidOperationException("no such edge");
    }

    private static void AssertSurfaces(SurfaceBuilder theBuilder)
    {
        Assert.Equal(StatusDone.IsOk, theBuilder.IsDone);
        Assert.True(theBuilder.NbSurface() >= 1);
        for (int i = 1; i <= theBuilder.NbSurface(); i++)
        {
            Assert.True(theBuilder.TolApp3d(i) < 1e-2);
            foreach ((Face aFace, Curve3D aCurve, Curve2D aOnFace, Curve2D aOnFillet) in new[]
            {
                (theBuilder.SupportFace1(i), theBuilder.CurveOnFace1(i), theBuilder.PCurveOnFace1(i), theBuilder.PCurve1OnFillet(i)),
                (theBuilder.SupportFace2(i), theBuilder.CurveOnFace2(i), theBuilder.PCurveOnFace2(i), theBuilder.PCurve2OnFillet(i)),
            })
            {
                Surface aSupport = Tool.Surface(aFace, out Placement aPlacement)!;
                Surface aFillet = theBuilder.SurfaceFillet(i);
                for (int k = 0; k <= 4; k++)
                {
                    double t = aCurve.FirstParameter + ((aCurve.LastParameter - aCurve.FirstParameter) * k / 4);
                    Point3D aP = aCurve.Value(t);
                    Point2D aUV = aOnFace.Value(t);
                    Point3D aOn = aSupport.Value(aUV.X, aUV.Y).Transformed(aPlacement.Transformation);
                    Assert.True(aP.DistanceTo(aOn) < 1e-3, $"{aP} not on its face at {aOn}");
                    Point2D aFilletUV = aOnFillet.Value(t);
                    Assert.True(aP.DistanceTo(aFillet.Value(aFilletUV.X, aFilletUV.Y)) < 1e-3, "not on the fillet");
                }
            }
        }
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(2.0)]
    [InlineData(4.5)]
    public void A_fillet_of_a_box_edge_keeps_its_curves_on_its_faces(double theRadius)
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Edge anEdge = EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10));
        var aBuilder = new SurfaceBuilder(aBox, ListOf(anEdge), theRadius);
        Assert.Equal(StatusDone.IsOk, aBuilder.IsDone);
        aBuilder.Perform();
        AssertSurfaces(aBuilder);
        Assert.Equal(1, aBuilder.NbSurface());
        // along the whole edge, a quarter cylinder: both curves stay the radius away from the edge
        foreach (Curve3D aCurve in new[] { aBuilder.CurveOnFace1(1), aBuilder.CurveOnFace2(1) })
        {
            Point3D aMid = aCurve.Value(0.5 * (aCurve.FirstParameter + aCurve.LastParameter));
            Assert.Equal(theRadius, Math.Sqrt((aMid.Y * aMid.Y) + ((aMid.Z - 10) * (aMid.Z - 10))), 1e-4);
            Assert.Equal(5, aMid.X, 1e-4);
        }
    }

    [Fact]
    public void A_fillet_around_the_top_of_a_cylinder_is_one_closed_section_of_the_edge()
    {
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        Edge anEdge = TopCircle(aCylinder, 10);
        var aBuilder = new SurfaceBuilder(aCylinder, ListOf(anEdge), 1);
        aBuilder.Perform();
        AssertSurfaces(aBuilder);
        foreach (Curve3D aCurve in new[] { aBuilder.CurveOnFace1(1), aBuilder.CurveOnFace2(1) })
        {
            Point3D aMid = aCurve.Value(0.5 * (aCurve.FirstParameter + aCurve.LastParameter));
            double aRadial = Math.Sqrt((aMid.X * aMid.X) + (aMid.Y * aMid.Y));
            // on the top at radius 4, or on the side at height 9
            Assert.True(Math.Abs(aRadial - 4) < 1e-3 || Math.Abs(aMid.Z - 9) < 1e-3, aMid.ToString());
        }
        Assert.True(aBuilder.LastParameter() > aBuilder.FirstParameter());
    }

    [Fact]
    public void Sections_are_arcs_of_the_radius_between_the_two_faces()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Edge anEdge = EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10));
        var aBuilder = new SurfaceBuilder(aBox, ListOf(anEdge), 2);
        aBuilder.Simulate();
        Assert.Equal(StatusDone.IsOk, aBuilder.IsDone);
        Assert.True(aBuilder.NbSurface() >= 1);
        Assert.True(aBuilder.NbSection(1) >= 2);
        for (int s = 1; s <= aBuilder.NbSection(1); s++)
        {
            TrimmedCurve3D anArc = aBuilder.Section(1, s);
            Assert.Equal(2, ((CircleCurve3D)anArc.BasisCurve!).Radius, 1e-9);
            // a quarter of a turn, from the top to the side
            Assert.Equal(Math.PI / 2, Math.Abs(anArc.LastParameter - anArc.FirstParameter), 1e-6);
        }
    }

    [Fact]
    public void Answers_are_asked_for_by_a_surface_it_has()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Edge anEdge = EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10));
        var aBuilder = new SurfaceBuilder(aBox, ListOf(anEdge), 2);
        aBuilder.Perform();
        Assert.Throws<OutOfRangeException>(() => aBuilder.SurfaceFillet(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.TolApp3d(0));
        Assert.Throws<OutOfRangeException>(() => aBuilder.SupportFace1(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.SupportFace2(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.CurveOnFace1(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.CurveOnFace2(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.PCurveOnFace1(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.PCurve1OnFillet(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.PCurveOnFace2(2));
        Assert.Throws<OutOfRangeException>(() => aBuilder.PCurve2OnFillet(2));
    }

    [Fact]
    public void Edges_that_do_not_chain_smoothly_are_refused()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Edge aFirst = EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10));
        Edge aSecond = EdgeBetween(aBox, new Point3D(10, 0, 10), new Point3D(10, 10, 10));
        var aBuilder = new SurfaceBuilder(aBox, ListOf(aFirst, aSecond), 1);
        Assert.Equal(StatusDone.IsNotOk, aBuilder.IsDone);
        Assert.Equal(ErrorTypeStatus.EdgeNotG1, aBuilder.StatusError());
    }

    [Fact]
    public void A_smooth_edge_has_no_corner_to_fillet()
    {
        // the edge between a fillet and the face it runs into is tangent
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aFillet = new MakeFillet(aBox);
        aFillet.Add(2, EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10)));
        Shape aRounded = aFillet.Shape();
        Edge aTangent = EdgeBetween(aRounded, new Point3D(0, 2, 10), new Point3D(10, 2, 10));
        var aBuilder = new SurfaceBuilder(aRounded, ListOf(aTangent), 1);
        Assert.Equal(StatusDone.IsNotOk, aBuilder.IsDone);
        Assert.Equal(ErrorTypeStatus.NotSharpEdge, aBuilder.StatusError());
    }

    [Fact]
    public void A_radius_too_big_for_the_faces_throws_as_in_OCCT()
    {
        // nothing in FilletSurf catches the failure of its start, so the status isn't reached
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Edge anEdge = EdgeBetween(aBox, new Point3D(0, 0, 10), new Point3D(10, 0, 10));
        Assert.Throws<FailureException>(() => new SurfaceBuilder(aBox, ListOf(anEdge), 20).Perform());
        Assert.Throws<FailureException>(() => new SurfaceBuilder(aBox, ListOf(anEdge), 20).Simulate());
    }
}
