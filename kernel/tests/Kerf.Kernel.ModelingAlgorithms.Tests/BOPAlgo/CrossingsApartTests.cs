// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The pave filler's steps that close a Boolean's faces in their parameters where OCCT leaves them open inside a vertex
/// (<c>SetCrossingsApart</c>; decisions 2276 and 32901 to 32905), each on equal cylinders, one's axis in the other's end
/// plane, a little short of crossing its axis or a little past: the family of decision 2257's sweep.
/// </summary>
public class CrossingsApartTests
{
    private static (Shape Cylinder, Shape Tool) Inputs(double radius, double offset)
    {
        Shape cylinder = Solid.Cylinder(radius, 2 * radius).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(0, 0, -radius))).Value.Shape;
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), Math.PI / 2);
        Shape tool = Solid.Cylinder(radius, 4 * radius).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(offset, 2 * radius, radius))).Value.Shape;
        return (cylinder, tool);
    }

    // The cut as the engine makes it, with crossings held apart at so many tolerances (0: OCCT's)
    private static Shape Cut(double radius, double offset, double apart, bool touches = true)
    {
        (Shape cylinder, Shape tool) = Inputs(radius, offset);
        var cut = new Cut();
        var arguments = new NCollection_List<Shape>();
        arguments.Append(cylinder);
        var tools = new NCollection_List<Shape>();
        tools.Append(tool);
        cut.SetArguments(arguments);
        cut.SetTools(tools);
        cut.SetNonDestructive(true);
        cut.SetCrossingsApart(apart, touches);
        cut.Build();
        Assert.True(cut.IsDone);
        return cut.Shape();
    }

    private static int Gaps(Shape shape)
    {
        int gaps = 0;
        for (var faces = new Explorer(shape, ShapeType.Face); faces.More(); faces.Next())
        {
            gaps += GapClosures.Of(ShapeCasts.Face(faces.Current()))?.Count ?? 0;
        }

        return gaps;
    }

    // The volume as OCCT reads it, with no gap closed
    private static double AsOcct(Shape shape)
    {
        var properties = new Properties();
        MassPropertiesBuilder.VolumeProperties(shape, properties, 1e-9, massOnly: true, closeGaps: false);
        return properties.Mass();
    }

    // The cylinder less half of two equal cylinders' common part, 16/3 r^3, which these offsets change by less than 1e-9 of it
    private static double CutVolume(double radius) => Math.PI * radius * radius * 2 * radius - 8.0 / 3 * radius * radius * radius;

    private static double WidestVertex(Shape shape)
    {
        double widest = 0;
        for (var vertices = new Explorer(shape, ShapeType.Vertex); vertices.More(); vertices.Next())
        {
            widest = Math.Max(widest, Tool.Tolerance(ShapeCasts.Vertex(vertices.Current())));
        }

        return widest;
    }

    // 1e-10 short of crossing at radius 500, the end circle crosses the tool 3.2e-4 either side of the touch. OCCT's cut is
    // open there, and reads 2e-7 out. Held apart, each crossing is a vertex, the sections' curves end at them exactly, and the
    // cut is closed and reads right (decisions 32901 and 32902)
    [Theory]
    [InlineData(-1e-10)]
    [InlineData(1e-10)]
    public void A_cut_a_little_off_a_touch_is_closed_with_its_crossings_apart(double offset)
    {
        double expected = CutVolume(500), apart = Math.Sqrt(2 * 500 * 1e-10);
        Shape occt = Cut(500, offset, 0);
        Assert.True(Gaps(occt) > 0);
        Assert.True(WidestVertex(occt) > 0.9 * apart);

        Shape made = Cut(500, offset, 10);
        Assert.Equal(0, Gaps(made));
        Assert.True(made.Validate().IsValid);
        Assert.Null(BooleanVolumes.Check(BooleanVolumeRule.Cut, [Inputs(500, offset).Cylinder], [Inputs(500, offset).Tool], made, 0));
        Assert.Equal(expected, AsOcct(made), 1e-9 * expected);
        Assert.True(WidestVertex(made) < 0.1 * apart);
    }

    // Past the touch the end circle dips through the tool by less than a tolerance, on the side where it has no vertex. The
    // edge and face step sees one touch and makes one vertex at its middle; given as its two crossings, 1.4e-3 apart at
    // 5e-10, the circle has a vertex at each, within 1e-6 of where the geometry puts them (decision 32902)
    [Fact]
    public void A_touch_that_hides_two_crossings_is_given_as_the_two()
    {
        const double Radius = 500, Offset = 5e-10;
        double apart = Math.Sqrt(2 * Radius * Offset);
        Shape made = Cut(Radius, Offset, 10);
        int found = 0;
        for (var vertices = new Explorer(made, ShapeType.Vertex); vertices.More(); vertices.Next())
        {
            Point3D at = Tool.Pnt(ShapeCasts.Vertex(vertices.Current()));
            if (Math.Abs(at.Z - Radius) < 1e-9 && Math.Abs(Math.Abs(at.Y) - apart) < 1e-6 && Math.Abs(Math.Abs(at.X) - Radius) < 1e-6)
            {
                found |= at.Y > 0 ? 1 : 2;
            }
        }
        Assert.Equal(3, found);
    }

    // There the walk of one branch stops 4e-5 short of the end circle. Left with a vertex of its own, the branch is open, the
    // face isn't split, and the cut removes nothing: 74% out, which the check refuses. Carried on to the crossing, the cut is
    // closed and right (decision 32903)
    [Fact]
    public void A_curve_that_stops_short_within_a_touch_is_carried_on_to_the_crossing()
    {
        const double Radius = 500, Offset = 5e-10;
        double expected = CutVolume(Radius);
        Shape made = Cut(Radius, Offset, 10);
        Assert.Equal(0, Gaps(made));
        Assert.True(made.Validate().IsValid);
        Assert.Equal(expected, AsOcct(made), 1e-9 * expected);
    }

    // At radius 5 the walk's own tolerance, 3e-4, is more than the crossings are apart, 1e-4 at 1e-9: one vertex covers them
    // whatever is held apart. The pieces of the end circle inside it, which OCCT drops, are taken into their neighbours, and
    // the sections' curves end where the circle does: closed, and right as OCCT reads it (decisions 32901 and 32905)
    [Fact]
    public void A_piece_dropped_inside_a_vertex_is_taken_into_its_neighbour()
    {
        const double Radius = 5, Offset = -1e-9;
        double expected = CutVolume(Radius);
        Shape occt = Cut(Radius, Offset, 0);
        Assert.True(Gaps(occt) > 0);

        Shape made = Cut(Radius, Offset, 10);
        Assert.Equal(0, Gaps(made));
        Assert.True(made.Validate().IsValid);
        Assert.Equal(expected, AsOcct(made), 1e-8 * expected);
        Assert.True(WidestVertex(made) > Math.Sqrt(2 * Radius * -Offset));
    }

    // With no crossing held apart at all the vertices are OCCT's, and the same two steps close the faces
    [Fact]
    public void OCCTs_vertices_are_closed_by_the_same_steps()
    {
        const double Radius = 5, Offset = -1e-9;
        double expected = CutVolume(Radius);
        Shape made = Cut(Radius, Offset, double.PositiveInfinity, touches: false);
        Assert.Equal(0, Gaps(made));
        Assert.True(made.Validate().IsValid);
        Assert.Equal(expected, AsOcct(made), 1e-8 * expected);
    }

    // Off by default: the engine's cut is OCCT's, open inside its vertex, whatever these steps would do
    [Fact]
    public void Off_the_cut_is_as_OCCT_makes_it()
    {
        Shape made = Cut(500, -3e-7, 0);
        Assert.True(Gaps(made) > 0);
        Assert.True(WidestVertex(made) > 0.017);
    }

    // The public Booleans, which make a result open inside a vertex again these ways and keep the one with the fewest gaps
    // (decision 32904), over both sides of the touch and three radii: closed, valid, and right as OCCT reads them
    [Theory]
    [InlineData(500, -1e-10)]
    [InlineData(500, 1e-10)]
    [InlineData(500, 5e-10)]
    [InlineData(500, 1e-12)]
    [InlineData(5, -1e-9)]
    [InlineData(5, 1e-11)]
    [InlineData(50, 0)]
    public void The_public_Booleans_near_a_touch_are_closed_in_their_parameters(double radius, double offset)
    {
        (Shape cylinder, Shape tool) = Inputs(radius, offset);
        double one = Math.PI * radius * radius * 2 * radius, common = 8.0 / 3 * radius * radius * radius;
        foreach ((Shape made, double expected) in new[]
                 {
                     (cylinder.Cut(tool).Value.Shape, one - common),
                     (cylinder.Common(tool).Value.Shape, common),
                     (cylinder.Fuse(tool).Value.Shape, 3 * one - common),
                 })
        {
            Assert.Equal(0, Gaps(made));
            Assert.True(made.Validate().IsValid);
            Assert.Equal(expected, AsOcct(made), 1e-8 * expected);
        }
    }

    // The cut of radius 0.1, 2e-7 off, which decision 2228 failed and decision 2258 passed as right read through its vertex:
    // five gaps, 2.8e-4 apart as OCCT read it. The cylinders' sections there are the ellipses of crossing axes, a little off
    // the faces' edges; their curves on the faces now end where the edges do, and it's closed, OCCT's reading the public one
    [Fact]
    public void The_cut_of_radius_a_tenth_nearly_crossing_is_closed()
    {
        const double Radius = 0.1;
        Shape cylinder = Solid.Cylinder(Radius, 2 * Radius).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(0, 0, -Radius))).Value.Shape;
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), -Math.PI / 2);
        Shape tool = Solid.Cylinder(Radius, 4 * Radius).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(-2e-7, -2 * Radius, 0.5 * Radius))).Value.Shape;
        Shape made = cylinder.Cut(tool).Value.Shape;
        Assert.Equal(0, Gaps(made));
        Assert.True(made.Validate().IsValid);
        double read = made.IntegrateVolume(new MassOptions { Tolerance = 1e-9 });
        Assert.Equal(read, AsOcct(made), 1e-9 * read);
        Assert.Equal(0.0020677179, read, 1e-9);
    }
}
