// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// No operation reports success for an invalid result (decision 1742): it fails with <c>result.invalid</c>, and a caller who
/// wants the shape asks for it. Each shape here was found invalid by the check before it was built in (unit 11R).
/// </summary>
public class InvalidResultTests
{
    private static Shape Box20() => Solid.Box(20, 20, 20).Value.Shape;

    // Radius 11 on a 20 mm box: the engine's done, with an invalid shape (decision 1740)
    private static Result<Built<Shape>> OverFilleted(FilletOptions? options = null)
    {
        Shape box = Box20();
        return box.Fillet(box.AllEdges(), 11, options);
    }

    private static Shape Invalid() => OverFilleted(new FilletOptions { CheckResult = false }).Value.Shape;

    // Lines that cross, so a planar face on them isn't valid
    private static Wire Bowtie() =>
        Wire.Polygon([new Point3D(0, 0, 0), new Point3D(10, 10, 0), new Point3D(10, 0, 0), new Point3D(0, 10, 0)], closed: true).Value;

    private static Face BowtieFace() => Face.FromWire(Bowtie(), checkResult: false).Value;

    private static void Carries<T>(Result<T> result, Func<T, Shape> shape, string says)
    {
        Assert.True(result.IsFailure);
        Assert.Equal(DiagnosticCode.InvalidResult, result.Error!.Code);
        Assert.Contains(says, result.Error.Message, StringComparison.Ordinal);
        Assert.Throws<InvalidOperationException>(() => result.Value);
        Assert.False(result.TryGetValue(out _));
        Assert.True(result.TryGetInvalid(out T? invalid));
        Assert.False(shape(invalid).Validate().IsValid);
    }

    private static void Carries(Result<Built<Shape>> result, string says) => Carries(result, static b => b.Shape, says);

    [Fact]
    public void A_fillet_says_what_is_wrong_and_names_the_shapes()
    {
        // A top that turns by 1 degree 5 along, and a radius just under that: no limit is claimed there, as the fillet rolls on to the
        // next face, and just here the engine's result isn't valid (fillet-design.md, section 8). The exact path, which has taken that over
        // for a circular section (decision 1910), makes no other kind
        double fall = 20 * Math.Tan(Math.PI / 180);
        Wire section = Wire.Polygon([new Point3D(0, 0, 0), new Point3D(0, -10, 0), new Point3D(25, -10, 0), new Point3D(25, -fall, 0), new Point3D(5, 0, 0)], closed: true).Value;
        Shape prism = Face.FromWire(section).Value.Extrude(new Vector3D(0, 0, 30)).Value.Shape;
        Edge side = prism.AllEdges().First(e => e.AllVertices().All(v => Math.Abs(v.Point.X) < 1e-9 && Math.Abs(v.Point.Y) < 1e-9));
        Result<Built<Shape>> result = prism.Fillet([side], 5 - 1e-8, new FilletOptions { Shape = FilletShape.Polynomial });
        Carries(result, "The fillet's result isn't valid: ");
        Assert.NotEmpty(result.Error!.Shapes);
        Assert.True(result.TryGetInvalid(out Built<Shape>? invalid));
        Assert.NotNull(invalid.History);
    }

    [Fact]
    public void A_fillet_whose_tolerance_passes_its_radius_is_not_a_success()
    {
        // #1639: a drafted plate; a radius 2 fillet on the top arc and the straight edge meeting it. The engine reports
        // success and the check passes, with an edge's tolerance of 190 and a volume of 10,596 for a true 29,800.
        var arc = new Circle3D(new Frame3D(new Point3D(0, 22.5, 0), Direction3D.UnitZ), 50);
        Wire profile = Wire.FromEdges(
        [
            Edge.Line(new(0, 0, 0), new(0, 72.5, 0)).Value,
            Edge.Arc(arc, 0.5 * Math.PI, Math.PI).Value,
            Edge.Line(new(-50, 22.5, 0), new(-50, 0, 0)).Value,
            Edge.Line(new(-50, 0, 0), new(0, 0, 0)).Value,
        ]).Value;
        Shape prism = Face.FromWire(profile).Value.Extrude(new Vector3D(0, 0, 10)).Value.Shape;
        Face[] walls = [.. prism.AllFaces().Where(f => !IsCap(f))];
        Shape plate = prism.Draft(walls.Select(f => (f, 5 * Math.PI / 180)), Direction3D.UnitZ, new Plane3D(Point3D.Origin, Direction3D.UnitZ)).Value.Shape;
        Edge[] edges = [.. plate.AllEdges().Where(IsArcOrStraightOnTop)];
        Assert.Equal(2, edges.Length);

        Result<Built<Shape>> result = plate.Fillet(edges, 2);

        if (result.IsSuccess)
        {
            Assert.Equal(29799.87, result.Value.Shape.IntegrateVolume(), 1);
        }
        else
        {
            Assert.Equal(DiagnosticCode.InvalidResult, result.Error!.Code);
            Assert.Contains("tolerance", result.Error.Message, StringComparison.Ordinal);
            Assert.True(result.TryGetInvalid(out _));
        }

        static bool IsCap(Face face) => face.AllVertices().Select(v => v.Point.Z).Distinct().Count() == 1;

        static bool IsArcOrStraightOnTop(Edge edge)
        {
            Point3D[] ends = [.. edge.AllVertices().Select(v => v.Point)];
            return ends.All(p => p.Z > 9.9) && ends.Any(p => p.Y > 70);
        }
    }

    [Fact]
    public void A_good_fillet_is_a_success_with_nothing_carried()
    {
        Shape box = Box20();
        Result<Built<Shape>> result = box.Fillet(box.AllEdges(), 2);
        Assert.True(result.IsSuccess);
        Assert.False(result.TryGetInvalid(out _));
    }

    [Fact]
    public void The_check_can_be_left_out() =>
        Assert.True(OverFilleted(new FilletOptions { CheckResult = false }).IsSuccess);

    [Fact]
    public void A_failure_of_the_algorithm_carries_nothing()
    {
        Shape box = Box20();
        Result<Built<Shape>> result = box.Chamfer(box.AllEdges(), 11);
        Assert.True(result.IsFailure);
        Assert.NotEqual(DiagnosticCode.InvalidResult, result.Error!.Code);
        Assert.False(result.TryGetInvalid(out _));
    }

    [Fact]
    public void A_boolean_of_an_invalid_shape_fails()
    {
        // Through the middle of the box
        Shape cylinder = Solid.Cylinder(5, 40).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(10, 10, -10))).Value.Shape;
        Carries(Invalid().Cut(cylinder), "The Boolean's result isn't valid: ");
        Assert.True(Invalid().Cut(cylinder, new BooleanOptions { CheckResult = false }).IsSuccess);
        Assert.True(Box20().Cut(cylinder).IsSuccess);
    }

    [Fact]
    public void Unifying_an_invalid_shape_fails()
    {
        Carries(Invalid().Unify(), "The unified shape isn't valid: ");
        Assert.True(Invalid().Unify(new UnifyOptions { CheckResult = false }).IsSuccess);
    }

    [Fact]
    public void A_face_on_crossing_wires_fails()
    {
        Carries(Face.FromWire(Bowtie()), static f => f, "The face isn't valid: ");
        Carries(Face.FromWires(Bowtie(), []), static f => f, "The face isn't valid: ");
        Assert.True(Face.FromWire(Bowtie(), checkResult: false).IsSuccess);
    }

    [Fact]
    public void An_extrusion_is_as_good_as_its_profile()
    {
        Carries(BowtieFace().Extrude(new Vector3D(0, 0, 5)), "The extrusion isn't valid: ");
        Assert.True(BowtieFace().Extrude(new Vector3D(0, 0, 5), new ExtrudeOptions { CheckResult = false }).IsSuccess);
        Assert.True(Face.FromPlane(new Plane3D(Point3D.Origin, new Direction3D(0, 0, 1)), Wire.Polygon([new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0)], closed: true).Value).Value
                        .Extrude(new Vector3D(0, 0, 5)).IsSuccess);
    }

    [Fact]
    public void A_sweep_that_folds_over_itself_fails()
    {
        Wire path = Wire.Polygon([new Point3D(0, 20, 0), new Point3D(20, 20, 0), new Point3D(20, 0, 0)]).Value;
        Wire section = Wire.Polygon([new Point3D(0, 15, -5), new Point3D(0, 25, -5), new Point3D(0, 25, 5), new Point3D(0, 15, 5)], closed: true).Value;
        Carries(Sweeps.Sweep(section, path, new SweepOptions { MakeSolid = true }), "The sweep's result isn't valid: ");
        Assert.True(Sweeps.Sweep(section, path, new SweepOptions { MakeSolid = true, CheckResult = false }).IsSuccess);
    }

    [Fact]
    public void A_loft_through_crossing_wires_fails()
    {
        Wire top = Wire.Polygon([new Point3D(0, 0, 10), new Point3D(0, 10, 10), new Point3D(10, 10, 10), new Point3D(10, 0, 10)], closed: true).Value;
        var options = new LoftOptions { MakeSolid = true, Ruled = true };
        Carries(Sweeps.Loft([Bowtie(), top], options), "The loft's result isn't valid: ");
        Assert.True(Sweeps.Loft([Bowtie(), top], options with { CheckResult = false }).IsSuccess);
    }

    [Fact]
    public void A_2D_chamfer_past_the_face_fails()
    {
        Face square = Face.FromPlane(new Plane3D(Point3D.Origin, new Direction3D(0, 0, 1)), Wire.Polygon([new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)], closed: true).Value).Value;
        Carries(square.Chamfer2D(square.AllVertices(), 6), static b => b.Shape, "isn't valid: ");
        Assert.True(square.Chamfer2D(square.AllVertices(), 6, checkResult: false).IsSuccess);
        Assert.True(square.Chamfer2D(square.AllVertices(), 2).IsSuccess);
    }

    [Fact]
    public void A_sphere_touching_a_torus_along_a_circle_fuses_cuts_and_commons()
    {
        // OCCT #1567 (decision 1818): a sphere of the minor radius, centred on the centre circle where a torus sector ends,
        // touches it along that end's circle. Native reports success with an empty fuse, and a cut that removes nothing.
        Solid torus = Solid.Torus(5, 1, new TorusOptions { Angle = Math.PI / 2 }).Value.Shape;
        Solid sphere = Solid.Sphere(1, new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))).Value.Shape;
        double sector = Math.PI * 5 * (Math.PI / 2), half = 2.0 / 3.0 * Math.PI;
        Assert.Equal(sector + half, torus.Fuse(sphere).Value.Shape.IntegrateVolume(), 1e-6);
        Assert.Equal(sector - half, torus.Cut(sphere).Value.Shape.IntegrateVolume(), 1e-6);
        Assert.Equal(half, torus.Common(sphere).Value.Shape.IntegrateVolume(), 1e-6);
    }

    [Theory]
    [InlineData(1.0 + 1e-6)]
    [InlineData(1.0 - 1e-6)]
    [InlineData(1.0 + 1e-9)]
    public void A_fuse_is_never_empty_and_a_success(double theRadius)
    {
        // Just off that tangency the intersection is still walked, and a fuse holds all of what it was given
        Solid torus = Solid.Torus(5, 1, new TorusOptions { Angle = Math.PI / 2 }).Value.Shape;
        Solid sphere = Solid.Sphere(theRadius, new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))).Value.Shape;
        Result<Built<Shape>> result = torus.Fuse(sphere, new BooleanOptions { CheckResult = false });
        if (result.IsSuccess)
        {
            Assert.NotEmpty(result.Value.Shape.Children);
        }
        else
        {
            Assert.Equal(DiagnosticCode.InvalidResult, result.Error!.Code);
        }
    }

    [Fact]
    public void Approximating_keeps_a_valid_shape_valid()
    {
        // Decision 1817: native's are invalid, as its edges keep their tolerance when their faces move
        Solid cylinder = Fixtures.Cylinder();
        Result<Built<Shape>> result = cylinder.ApproximateBSplines();
        Assert.True(result.IsSuccess);
        Assert.Equal(cylinder.IntegrateVolume(), result.Value.Shape.IntegrateVolume(), 1e-3 * cylinder.IntegrateVolume());
        Assert.All(result.Value.Shape.AllEdges(), e => Assert.True(e.Tolerance < 0.02));
    }
}
