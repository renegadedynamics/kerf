// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Kerf.Drawing's members no test named until unit 12R (direct-members.md), and the polygonal hidden-line algorithm
/// by the law that it draws what the exact one does (unit 12Q): its lines are chords of a fine mesh, so each kind's
/// visible and hidden lengths are the exact drawing's, a little short where the lines curve.
/// </summary>
public class DrawingDirectTests
{
    [Fact]
    public void A_camera_is_where_it_was_put_and_looks_at_its_target()
    {
        var aCamera = Camera.LookAt(new Point3D(10, 0, 0), new Point3D(0, 0, 0), Direction3D.UnitZ);
        Assert.Equal(new Point3D(10, 0, 0), aCamera.Eye);
        Assert.Equal(Point3D.Origin, aCamera.Target);
        Assert.Equal(10, aCamera.Distance, 12);
        Assert.Equal(Projection.Orthographic, aCamera.Projection);
        // the view frame: X right, Y up, Z towards the eye
        Frame3D aFrame = aCamera.ViewFrame;
        Assert.True(aFrame.Direction.IsEqual(Direction3D.UnitX, 1e-12));
        Assert.True(aFrame.YDirection.IsEqual(Direction3D.UnitZ, 1e-12));
        Assert.Equal(new Point2D(0, 5), aCamera.Project(new Point3D(3, 0, 5)));
        // a camera moved to another target looks from the same eye
        Camera aMoved = aCamera with { Target = new Point3D(0, 0, 0.5) };
        Assert.Equal(new Point3D(0, 0, 0.5), aMoved.Target);
        Assert.True(new Camera().Eye == new Point3D(0, 0, 1) && new Camera().Target == Point3D.Origin);
    }

    [Fact]
    public void Drawing_options_default_as_documented()
    {
        DrawingOptions d = DrawingOptions.Default;
        Assert.Equal(HiddenLineMethod.Exact, d.Method);
        Assert.True(d.IncludeHidden);
        Assert.Equal(0, d.IsoLineCount);
        Assert.Null(d.Tolerance);
        Assert.Null(d.Mesh);
        var aSet = new DrawingOptions { IsoLineCount = 3, Tolerance = 0.01, IncludeHidden = false };
        Assert.Equal((3, 0.01, false), (aSet.IsoLineCount, aSet.Tolerance, aSet.IncludeHidden));
    }

    // Views with no edge lying along an outline or seen edge-on, where which algorithm hides it is a matter of rounding:
    // a sphere's or a torus's seam on its outline from the front, a torus's equator from the side
    public static TheoryData<string, string> Views()
    {
        var aData = new TheoryData<string, string>();
        foreach (string aView in new[] { "top", "front", "isometric", "oblique" })
        {
            aData.Add("holed box", aView);
            aData.Add("two boxes", aView);
        }
        foreach (string aShape in new[] { "sphere", "cone", "torus" })
        {
            aData.Add(aShape, "isometric");
            aData.Add(aShape, "oblique");
        }
        return aData;
    }

    private static Shape[] Shapes(string theName) => theName switch
    {
        "holed box" => [HoledBox()],
        "sphere" => [Solid.Sphere(5).Value.Shape],
        "cone" => [Solid.Cone(5, 2, 10).Value.Shape],
        "torus" => [Solid.Torus(6, 2).Value.Shape],
        // apart: the small one above a corner of the large one, hiding some of it from above and the side
        _ => [Solid.Box(10, 10, 10).Value.Shape, Solid.Box(6, 6, 6, new Frame3D(new Point3D(7, 7, 12), Direction3D.UnitZ)).Value.Shape],
    };

    private static Shape HoledBox()
    {
        Solid aBlock = Solid.Box(10, 10, 10).Value;
        Solid aHole = Solid.Cylinder(2, 20, new Frame3D(new Point3D(5, 5, -5), Direction3D.UnitZ)).Value;
        return aBlock.Cut(aHole).Value.Shape;
    }

    private static Camera View(string theName) => theName switch
    {
        "top" => Camera.Top,
        "front" => Camera.Front,
        "isometric" => Camera.Isometric,
        _ => Camera.Along(new Direction3D(-0.3, 0.8, -0.5), Direction3D.UnitZ),
    };

    [Theory]
    [MemberData(nameof(Views))]
    public void The_polygonal_drawing_is_the_exact_one_in_chords(string theShape, string theView)
    {
        Shape[] aShapes = Shapes(theShape);
        Camera aCamera = View(theView);
        LineDrawing anExact = DrawingOperations.ProjectLines(aShapes, aCamera).Value;
        LineDrawing aPolygonal = DrawingOperations.ProjectLines(aShapes, aCamera, new DrawingOptions { Method = HiddenLineMethod.Polygonal }).Value;
        double aSize = anExact.GetBounds()!.Value is var (aMin, aMax) ? aMin.DistanceTo(aMax) : 0;
        foreach (bool aVisible in new[] { true, false })
        {
            // the visible sharp lines and outlines together, as an outline of a curved face may be drawn as its edge in
            // one and as an outline in the other where they meet; the hidden sharp lines (OCCT's exact algorithm draws
            // no hidden outlines)
            double Total(LineDrawing theDrawing) =>
                theDrawing.Edges.Where(e => e.IsVisible == aVisible && (e.Kind == LineKind.Sharp || (aVisible && e.Kind == LineKind.Outline)))
                          .Sum(e => DrawingTests.Length(e.Curve));
            double anExactLength = Total(anExact), aPolygonalLength = Total(aPolygonal);
            Assert.True(Math.Abs(anExactLength - aPolygonalLength) <= (0.01 * anExactLength) + (0.002 * aSize),
                        $"{(aVisible ? "visible" : "hidden")}: exact {anExactLength}, polygonal {aPolygonalLength}");
        }
        // every polygonal line is a polyline in the view plane, within the exact drawing's bounds
        (Point2D eMin, Point2D eMax) = anExact.GetBounds()!.Value;
        (Point2D pMin, Point2D pMax) = aPolygonal.GetBounds()!.Value;
        Assert.True(pMin.X >= eMin.X - (0.002 * aSize) && pMin.Y >= eMin.Y - (0.002 * aSize));
        Assert.True(pMax.X <= eMax.X + (0.002 * aSize) && pMax.Y <= eMax.Y + (0.002 * aSize));
        Assert.All(aPolygonal.Edges, e => Assert.True(e.Curve is BSplineCurve2D { Degree: 1 } || e.Curve is TrimmedCurve2D, e.Curve.GetType().Name));
    }

    [Fact]
    public void Without_hidden_lines_only_the_visible_ones_are_drawn()
    {
        Solid aBox = Solid.Box(10, 10, 10).Value;
        foreach (HiddenLineMethod aMethod in new[] { HiddenLineMethod.Exact, HiddenLineMethod.Polygonal })
        {
            LineDrawing anAll = aBox.ProjectLines(Camera.Isometric, new DrawingOptions { Method = aMethod }).Value;
            LineDrawing aVisible = aBox.ProjectLines(Camera.Isometric, new DrawingOptions { Method = aMethod, IncludeHidden = false }).Value;
            Assert.All(aVisible.Edges, e => Assert.True(e.IsVisible));
            Assert.Equal(anAll.Edges.Count(e => e.IsVisible), aVisible.Edges.Length);
        }
    }
}
