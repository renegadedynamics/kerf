// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Kerf.Drawing's line drawings (docs/api/drawing.md): hidden-line removal of simple solids from known views, against
/// lengths worked by hand. The fixtures' box is 10 × 20 × 30 at the origin, the cylinder of radius 5 and height 10 on Z.
/// </summary>
public class DrawingTests
{
    // An edge of length L seen along (1, -1, 1) is L √(2/3) long
    private static readonly double Isometric = Math.Sqrt(2.0 / 3.0);

    internal static double Length(Curve2D curve)
    {
        double u0 = curve.FirstParameter, u1 = curve.LastParameter, length = 0;
        Point2D p = curve.Evaluate(u0);
        for (int i = 1; i <= 4000; i++)
        {
            Point2D q = curve.Evaluate(u0 + (u1 - u0) * i / 4000);
            length += p.DistanceTo(q);
            p = q;
        }

        return length;
    }

    private static double Length(IEnumerable<DrawnEdge> edges) => edges.Sum(e => Length(e.Curve));

    private static byte[] BrepBytes(Shape shape)
    {
        var stream = new MemoryStream();
        Brep.Write(shape, stream);
        return stream.ToArray();
    }

    [Fact]
    public void A_box_from_the_top_shows_its_top_and_hides_its_bottom()
    {
        Solid box = Fixtures.Box();
        byte[] before = BrepBytes(box);
        LineDrawing drawing = box.ProjectLines(Camera.Top).Value;

        // The vertical edges project to points; the top's four edges are visible, the bottom's hidden under them
        Assert.Equal(4, drawing.Visible.Count());
        Assert.Equal(4, drawing.Hidden.Count());
        Assert.All(drawing.Edges, e => Assert.Equal(LineKind.Sharp, e.Kind));
        Assert.Equal(60, Length(drawing.Visible), 1e-9);
        Assert.Equal(60, Length(drawing.Hidden), 1e-9);
        Assert.All(drawing.Edges, e => Assert.True(e.Curve is TrimmedCurve2D { BasisCurve: LineCurve2D }));

        // Each line's source is one of the box's edges, a visible one at the top
        Edge[] edges = box.AllEdges().ToArray();
        Assert.All(drawing.Edges, e => Assert.Contains(edges, edge => edge.IsSame(e.Source)));
        Assert.All(drawing.Visible, e => Assert.All(e.Source.AllVertices(), v => Assert.Equal(30, v.Point.Z, 1e-9)));
        (Point2D min, Point2D max) = drawing.GetBounds()!.Value;
        Assert.Equal(new Point2D(0, 0), min);
        Assert.Equal(new Point2D(10, 20), max);
        Assert.Equal(before, BrepBytes(box));
    }

    [Theory]
    [InlineData(HiddenLineMethod.Exact)]
    [InlineData(HiddenLineMethod.Polygonal)]
    public void An_isometric_box_shows_nine_edges_and_hides_three(HiddenLineMethod method)
    {
        Solid box = Fixtures.Box();
        LineDrawing drawing = box.ProjectLines(Camera.Isometric, new DrawingOptions { Method = method }).Value;
        Assert.Equal(9, drawing.Visible.Count());
        Assert.Equal(3, drawing.Hidden.Count());
        // The three hidden edges meet at the far corner: one of each length
        Assert.Equal((10 + 20 + 30) * Isometric, Length(drawing.Hidden), 1e-9);
        Assert.Equal(3 * (10 + 20 + 30) * Isometric, Length(drawing.Visible), 1e-9);
    }

    [Fact]
    public void A_cylinder_from_the_front_is_a_rectangle_with_its_back_halves_hidden()
    {
        LineDrawing drawing = Fixtures.Cylinder().ProjectLines(Camera.Front).Value;
        // Visible: the two circles' front halves seen edge on (10 each), the silhouette at x = -5 and, at x = 5, the
        // seam, which lies on the silhouette there (an outline edge is drawn as sharp, as OCCT does)
        Assert.Equal(40, Length(drawing.Visible), 1e-6);
        Assert.Equal(20, Length(drawing.Hidden), 1e-6);
        Assert.Equal(3, drawing.Of(LineKind.Sharp, true).Count());
        Assert.Single(drawing.Of(LineKind.Outline, true));
        Assert.Equal(2, drawing.Hidden.Count());
        DrawnEdge outline = drawing.Of(LineKind.Outline, true).Single();
        Assert.Equal(-5, outline.Start.X, 1e-9);
        Assert.Equal(ShapeType.Face, outline.Source.Type);
    }

    [Fact]
    public void A_cylinder_from_the_top_is_two_true_circles()
    {
        LineDrawing drawing = Fixtures.Cylinder().ProjectLines(Camera.Top).Value;
        DrawnEdge top = Assert.Single(drawing.Visible), bottom = Assert.Single(drawing.Hidden);
        foreach (DrawnEdge circle in new[] { top, bottom })
        {
            var arc = Assert.IsType<TrimmedCurve2D>(circle.Curve);
            Assert.Equal(5, Assert.IsType<CircleCurve2D>(arc.BasisCurve).Radius, 1e-12);
            Assert.Equal(2 * Math.PI, arc.LastParameter - arc.FirstParameter, 1e-9);
        }
    }

    [Fact]
    public void An_isometric_cylinder_has_ellipses_outlines_and_its_seam()
    {
        Solid cylinder = Fixtures.Cylinder();
        LineDrawing drawing = cylinder.ProjectLines(Camera.Isometric).Value;
        // The circles are ellipses of radii 5 and 5/√3 (Ramanujan's perimeter); the top's whole and the bottom's front
        // half visible, its back half hidden
        double a = 5, b = 5 / Math.Sqrt(3), h = (a - b) * (a - b) / ((a + b) * (a + b));
        double perimeter = Math.PI * (a + b) * (1 + 3 * h / (10 + Math.Sqrt(4 - 3 * h)));
        Assert.Equal(1.5 * perimeter, Length(drawing.Of(LineKind.Sharp, true)), 1e-3);
        Assert.Equal(0.5 * perimeter, Length(drawing.Of(LineKind.Sharp, false)), 1e-3);
        Assert.All(drawing.Of(LineKind.Sharp, true), e => Assert.IsType<EllipseCurve2D>(((TrimmedCurve2D)e.Curve).BasisCurve));
        // Two silhouettes on the cylindrical face, and the seam in front, each 10 √(2/3)
        Face side = cylinder.AllFaces().Single(f => f.AllEdges().Length == 3); // the side: its seam and two circles
        Assert.Equal(2, drawing.Of(LineKind.Outline, true).Count());
        Assert.All(drawing.Of(LineKind.Outline, true), e => Assert.True(e.Source.IsSame(side)));
        Assert.Equal(2 * 10 * Isometric, Length(drawing.Of(LineKind.Outline, true)), 1e-9);
        DrawnEdge seam = Assert.Single(drawing.Of(LineKind.Sewn, true));
        Assert.Equal(10 * Isometric, Length(seam.Curve), 1e-9);
        Assert.Equal(ShapeType.Edge, seam.Source.Type);
        Assert.Empty(drawing.Of(LineKind.IsoLine, true));

        // The polygonal algorithm: the same lines as polylines, within the mesh's deflection
        LineDrawing polygonal = cylinder.ProjectLines(Camera.Isometric, new DrawingOptions { Method = HiddenLineMethod.Polygonal }).Value;
        Assert.Equal(Length(drawing.Visible), Length(polygonal.Visible), 0.05);
        Assert.Equal(Length(drawing.Hidden), Length(polygonal.Hidden), 0.05);
        Assert.Equal(2, polygonal.Of(LineKind.Outline, true).Count());
    }

    [Fact]
    public void Isolines_are_drawn_on_their_faces()
    {
        Solid cylinder = Fixtures.Cylinder();
        LineDrawing drawing = cylinder.ProjectLines(Camera.Isometric, new DrawingOptions { IsoLineCount = 2 }).Value;
        Assert.NotEmpty(drawing.Of(LineKind.IsoLine, true));
        Assert.NotEmpty(drawing.Of(LineKind.IsoLine, false));
        Face[] faces = cylinder.AllFaces().ToArray();
        Assert.All(drawing.Edges.Where(e => e.Kind == LineKind.IsoLine), e => Assert.Contains(faces, f => f.IsSame(e.Source)));
    }

    [Fact]
    public void A_sphere_is_its_outline()
    {
        Solid sphere = Solid.Sphere(10).Value;
        LineDrawing drawing = sphere.ProjectLines(Camera.Isometric).Value;
        DrawnEdge outline = Assert.Single(drawing.Of(LineKind.Outline, true));
        Assert.Equal(10, Assert.IsType<CircleCurve2D>(((TrimmedCurve2D)outline.Curve).BasisCurve).Radius, 1e-9);
        Assert.Equal(2 * Math.PI * 10, Length(outline.Curve), 1e-4);
    }

    [Fact]
    public void In_perspective_the_box_corners_are_the_cameras_projections()
    {
        Solid box = Fixtures.Box();
        Camera camera = Camera.LookAt(new Point3D(40, -60, 70), new Point3D(5, 10, 15), projection: Projection.Perspective);
        LineDrawing drawing = box.ProjectLines(camera).Value;
        Assert.Equal(9, drawing.Visible.Count());
        Assert.Equal(3, drawing.Hidden.Count());
        Point2D[] corners = [.. box.AllVertices().Select(v => camera.Project(v.Point))];
        foreach (DrawnEdge edge in drawing.Edges)
        {
            Assert.True(edge.Curve is TrimmedCurve2D { BasisCurve: LineCurve2D });
            Assert.Contains(corners, c => c.DistanceTo(edge.Start) < 1e-6);
            Assert.Contains(corners, c => c.DistanceTo(edge.End) < 1e-6);
        }

        // Nearer is larger: the perspective drawing of the box isn't the orthographic one
        Assert.NotEqual(Length(box.ProjectLines(camera with { Projection = Projection.Orthographic }).Value.Visible), Length(drawing.Visible), 1e-3);
    }

    [Fact]
    public void Several_shapes_hide_each_other()
    {
        Solid box = Fixtures.Box();
        // A plate above the box, larger than its top: from the top, it hides all of the box
        Solid plate = Solid.Box(20, 30, 1, new Frame3D(new Point3D(-5, -5, 40), Direction3D.UnitZ)).Value;
        LineDrawing alone = box.ProjectLines(Camera.Top).Value;
        LineDrawing both = DrawingOperations.ProjectLines([box, plate], Camera.Top).Value;
        Assert.Equal(4, alone.Visible.Count());
        Assert.Equal(4, both.Visible.Count()); // the plate's top
        Assert.Equal(100, Length(both.Visible), 1e-9);
        Assert.Equal(100 + 120, Length(both.Hidden), 1e-9); // the plate's bottom, and the box's top and bottom
    }

    [Fact]
    public void Options_and_misuse()
    {
        Solid box = Fixtures.Box();
        Assert.Empty(box.ProjectLines(Camera.Isometric, new DrawingOptions { IncludeHidden = false }).Value.Hidden);
        KernelAssert.Throws<ArgumentException>(() => default(Solid).ProjectLines(Camera.Top));
        KernelAssert.Throws<ArgumentException>(() => box.ProjectLines(Camera.Top with { Up = Direction3D.UnitZ }));
        KernelAssert.Throws<ArgumentException>(() => box.ProjectLines(Camera.Top with { Eye = Point3D.Origin }));
        KernelAssert.Throws<ArgumentException>(() => box.ProjectLines(Camera.Isometric with { Projection = Projection.Perspective },
                                                                new DrawingOptions { Method = HiddenLineMethod.Polygonal }));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new DrawingOptions { IsoLineCount = -1 });
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => box.ProjectLines(Camera.Top, cancellationToken: cancelled.Token));
        // A compound of nothing draws nothing
        Assert.Empty(Compound.Create([]).ProjectLines(Camera.Top).Value.Edges);
    }

    [Fact]
    public void The_same_shape_gives_the_same_drawing()
    {
        Solid cylinder = Fixtures.Cylinder();
        string first = Svg.WriteToString(cylinder.ProjectLines(Camera.Isometric).Value);
        string[] others = new string[4];
        Parallel.For(0, others.Length, i => others[i] = Svg.WriteToString(cylinder.ProjectLines(Camera.Isometric).Value));
        Assert.All(others, o => Assert.Equal(first, o));
    }

    // OCCT #1561 (decision 2448): a smooth loft through 3 or 5 circles, each a periodic spline through 48 points, seen from
    // the front. Its face has 49 knots one way and 2 the other, and the exact method's start points for the outline were laid
    // with their two indices swapped, all in two columns by the seam: the silhouettes half a turn away had none, and the
    // drawing had one silhouette of the two, or none. Lengths as the issue gives them once mended.
    [Theory]
    [InlineData(new[] { 0, 2, 4 }, 371.5)]
    [InlineData(new[] { 0, 1, 2, 3, 4 }, 376.3)]
    public void A_loft_through_many_point_splines_has_both_its_silhouettes(int[] levels, double expected)
    {
        double[] radii = [30, 40, 47, 28, 25], heights = [0, 45, 99, 144, 180];
        const int n = 48;
        List<Shape> sections = [];
        foreach (int l in levels)
        {
            Point3D[] points = [.. Enumerable.Range(0, n).Select(i => new Point3D(radii[l] * Math.Cos(2 * Math.PI * i / n),
                                                                                  radii[l] * Math.Sin(2 * Math.PI * i / n), heights[l]))];
            BSplineCurve3D curve = GeometryOperations.Interpolate(points, new InterpolationOptions { Periodic = true, Tolerance = 1e-7 }).Value;
            sections.Add(ShapeFactory.FromEdges([ShapeFactory.FromCurve(curve).Value]).Value);
        }

        Shape loft = Sweeps.Loft(sections, new LoftOptions { MakeSolid = true, Ruled = false }).Value.Shape;
        LineDrawing drawing = loft.ProjectLines(Camera.Front, new DrawingOptions { Method = HiddenLineMethod.Exact }).Value;
        DrawnEdge[] outlines = [.. drawing.Of(LineKind.Outline, true)];
        Assert.InRange(Length(outlines), expected - 0.2, expected + 0.2);
        Assert.Contains(outlines, e => e.Start.X < -20);
        Assert.Contains(outlines, e => e.Start.X > 20);
    }
}
