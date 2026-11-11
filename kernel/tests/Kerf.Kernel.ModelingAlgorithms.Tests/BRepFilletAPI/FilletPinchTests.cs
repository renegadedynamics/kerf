// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// A fillet face that narrows to a point between two of its edges, which ChFi3d leaves with no vertex there and 15L's check
/// fails (decision 2126): built with the vertex, and its face in two (unit 15Q). The shapes are the corpus's
/// <c>blend/simple/X4</c> and <c>blend/tolblend_simple/B7</c> and <c>C2</c>, made again here.
/// </summary>
public class FilletPinchTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-7 };

    // CCV_1_h1_gsk.rle: a block 120 by 100 by 50 with a half round notch of radius 30 in its front, about x = 0, the cylinder's
    // seam outside the block as the case's is (with its seam on the notch's edge at x = 30, ChFi3d fails from 30 up)
    private static Shape Notched() =>
        Solid.Box(new Point3D(-60, -50, 0), new Point3D(60, 50, 50)).Value.Shape
             .Cut(Solid.Cylinder(30, 50, new Frame3D(new Point3D(0, -50, 0), new Direction3D(0, 0, 1), new Direction3D(0, -1, 0))).Value.Shape).Value.Shape;

    // The front's top edge beside the notch, from x = 30 to 60, as the case's s_14
    private static Edge FrontTop(Shape notched) =>
        notched.AllEdges().Single(e => e.AllVertices().All(v => Math.Abs(v.Point.Y + 50) < 1e-9 && Math.Abs(v.Point.Z - 50) < 1e-9)
                                       && e.AllVertices().All(v => v.Point.X > 29));

    // The volume at radius 30 by the section: what the fillet takes from the corner of the front and top, all along the block, less
    // what the notch had already taken (u = y + 50, from the front; the notch leaves 120 - 2 sqrt(900 - u^2) of the block's width)
    private static double NotchedAt30()
    {
        double block = 120.0 * 100 * 50 - Math.PI * 900 / 2 * 50;
        double corner = (1 - Math.PI / 4) * 900;
        const int n = 200_000;
        double notch = 0;
        for (int i = 0; i < n; i++)
        {
            // The midpoint rule on u in [0, 30]: the corner's depth below the top at u, times the notch's width there
            double u = 30.0 * (i + 0.5) / n;
            double depth = 30 - Math.Sqrt(900 - (30 - u) * (30 - u));
            notch += 2 * Math.Sqrt(900 - u * u) * depth * (30.0 / n);
        }
        return block - 120 * corner + notch;
    }

    [Fact]
    public void A_fillet_whose_contact_line_just_reaches_a_notch_is_built_with_a_vertex_where_it_touches()
    {
        // At 30 the fillet's line on the top just reaches the notch's deepest point, (0, -20, 50): the fillet runs on past the
        // notch to the far side, and its face is two faces that meet there, as the top's edge round the notch has gone to a point
        Shape notched = Notched();
        Built<Shape> made = notched.Fillet([FrontTop(notched)], 30).Value;
        Shape shape = made.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(notched, shape));
        Assert.Equal(10, shape.AllFaces().Length);
        Assert.Contains(shape.AllVertices(), v => Math.Abs(v.Point.X) < 0.01 && Math.Abs(v.Point.Y + 20) < 1e-5 && Math.Abs(v.Point.Z - 50) < 1e-5);
        Assert.Equal(NotchedAt30(), shape.IntegrateVolume(Precise), 0.01);

        // OCCT's figure for the case (checkprops -s 43339.8)
        Assert.Equal(43339.8, shape.IntegrateArea(Precise), 0.05);

        // Its history gives the edge both faces of the fillet
        Assert.Equal(2, made.History.Generated(FrontTop(notched)).Count(s => s.Type == ShapeType.Face));
    }

    [Theory]
    [InlineData(29.999)]
    [InlineData(30.0001)]
    [InlineData(30.001)]
    public void Either_side_of_the_touch_the_notch_fillet_builds(double radius)
    {
        Shape notched = Notched();
        Shape shape = notched.Fillet([FrontTop(notched)], radius).Value.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(notched, shape));
    }

    // 7_C4.draw and CFI_7_a4_fhl.rle at the case's scale of 5: a cone from radius 125 at z 250 to 250 at z 500, standing on a prism of
    // its base circle along (0, 125, -250), which leans as the cone does at y = -125, so the two faces are tangent there; C2's prism
    // leans along (0, 125.0005, -250)
    private static (Shape Shape, Edge Edge) ConeOnPrism(double lean)
    {
        Shape cone = Solid.Cone(125, 250, 250, new Frame3D(new Point3D(0, 0, 250), Direction3D.D.Z)).Value.Shape;
        Face foot = cone.AllFaces().Single(f => f.AllVertices().All(v => Math.Abs(v.Point.Z - 250) < 1e-9));
        Shape prism = foot.Extrude(new Vector3D(0, lean, -250)).Value.Shape;
        Shape shape = cone.Fuse(prism).Value.Shape;
        Edge rim = shape.AllEdges().Single(e => e.AllVertices().All(v => Math.Abs(v.Point.Z - 250) < 1e-9)
                                              && shape.AllFaces().Count(f => f.AllEdges().Any(o => o.IsSame(e))) == 2);
        return (shape, rim);
    }

    [Fact]
    public void A_fillet_between_faces_tangent_at_a_point_narrows_to_that_point_at_every_radius()
    {
        // ChFi3d splits the fillet where the faces are tangent at some radii and not at others (30 to 70 here), leaving one face
        // whose edges touch; either way the fillet has no width there, and now each is built with a vertex there
        (Shape shape, Edge rim) = ConeOnPrism(125);
        double before = shape.IntegrateVolume(Precise);
        var added = new List<double>();
        foreach (double radius in (double[])[20, 40, 60, 75, 80, 100])
        {
            Shape made = shape.Fillet([rim], radius).Value.Shape;
            Assert.True(made.Validate().IsValid);
            Assert.Empty(EdgeCrossings.All(shape, made));
            Assert.Contains(made.AllVertices(), v => Math.Abs(v.Point.X) < 0.1 && Math.Abs(v.Point.Y + 125) < 1e-3 && Math.Abs(v.Point.Z - 250) < 1e-3);
            added.Add((made.IntegrateVolume(Precise) - before) / (radius * radius));
        }

        // What it adds, over the radius squared, changes smoothly with the radius, through the radii where ChFi3d split the face
        // itself and where it didn't (on the ball, decision 2160, it rises by 7.5e-4 between two of them, where the
        // off-ball sections fell)
        for (int i = 1; i < added.Count; i++)
        {
            Assert.InRange(added[i] / added[i - 1], 0.99, 1.01);
        }
    }

    [Fact]
    public void A_fillet_whose_edges_cross_where_the_faces_turn_from_concave_to_convex_still_fails()
    {
        // C2's faces are tangent at two points 0.7 apart near y = -125, convex between: the fillet there would change sides, which
        // ChFi3d can't do, so its two edges cross twice; that is no narrowing to a point, and the failure stands
        (Shape shape, Edge rim) = ConeOnPrism(125.0005);
        Result<Built<Shape>> made = shape.Fillet([rim], 75);
        Assert.False(made.IsSuccess);
        Assert.Equal(FilletOperations.SelfIntersecting, made.Error!.Code);
        Assert.True(made.TryGetInvalid(out _));
    }
}
