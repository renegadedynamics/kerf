// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Collections;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Unit 15AK2 (decisions 2335 to 2338): a fillet of evolving radius whose ball comes to rest on an edge. Where it does, and
/// where it leaves the edge, the path of the ball's centre has a corner unless the edge lies along the way the contact was
/// going, and with the radius changing there the one ball has two circles, one for each side. How far the walked surfaces are
/// then off the surface the balls sweep is measured here apart from how they were built, against the boundary of the union of
/// the balls, and the fillet is built only within the approximation's tolerance of it.
/// </summary>
public class EvolvingFilletOnEdgeTests
{
    private const double Approximation = 1e-4;

    private delegate double Distance(Point3D p);

    private delegate Vector3D Normal(Point3D p);

    // A cylinder of radius 5 about z, from z = -10 up to the top given, and one of radius 3 along y through it: the fillet of
    // the curves they meet in, once its radius passes the height of the first's rim above the second, rests on the rim
    private static Shape RimTee(double top) =>
        Solid.Cylinder(5, 10 + top, new Frame3D(new Point3D(0, 0, -10), Direction3D.D.Z, Direction3D.D.X)).Value.Shape
             .Fuse(Solid.Cylinder(3, 30, new Frame3D(new Point3D(0, -15, 0), Direction3D.D.Y, Direction3D.D.X)).Value.Shape).Value.Shape;

    private static MakeFillet RimTeeFillet(double top, double low, double high, bool refused, bool cornersBuilt = true)
    {
        Shape shape = RimTee(top);
        Edge[] edges = [.. shape.AllEdges().Where(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - 5) < 1e-6 && Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - 3) < 1e-6;
        })];
        Assert.Equal(2, edges.Length);
        var chfi3d = new MakeFillet(shape);
        chfi3d.InternalBuilder().EnvelopeCornersRefused = refused;
        chfi3d.InternalBuilder().EnvelopeCornersBuilt = cornersBuilt;
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0.0, low));
        law.SetValue(2, new Point2D(0.5, high));
        law.SetValue(3, new Point2D(1.0, low));
        foreach (Edge e in edges)
        {
            chfi3d.Add(law, e);
        }
        chfi3d.Build();
        return chfi3d;
    }

    // The distance to the first cylinder as a body, its rim and its top with it, and what a ball resting on the second is
    // centred on
    private static Distance MainBody(double top) => p =>
    {
        double dr = Math.Sqrt(p.X * p.X + p.Y * p.Y) - 5, dz = p.Z - top;
        return dr <= 0 && dz <= 0 ? Math.Max(dr, dz) : Math.Sqrt(Math.Max(dr, 0) * Math.Max(dr, 0) + Math.Max(dz, 0) * Math.Max(dz, 0));
    };

    // The radius goes from the low value at the seam to the high one opposite it and back, so it is changing where the ball
    // comes to rest on the rim and where it leaves it: two corners of the centre's path to each curve the cylinders meet in.
    // At each the later surface is made to end on the earlier's circle (EnvelopeTilt), and the measure is how far inside its
    // own balls that puts it (EnvelopeEnd.Depth). With the refusal turned off, the measure at each kind of corner and what
    // the union of the balls shows beside it:
    //   rim at 3.05, 0.4 to 1.0: 4.9e-6 and 2.6e-6; 4.4e-6 and 2.6e-6. Built.
    //   rim at 3.2, 0.4 to 1.6: 3.1e-5 and 1.5e-5; 3.0e-5 and 1.5e-5. Built.
    //   rim at 3.2, 0.2 to 2.0: 8.7e-5 and 3.5e-5; 7.8e-5 and 3.5e-5. Built.
    //   rim at 3.2, 0.4 to 2.4: 1.39e-4 and 6.4e-5; 1.33e-4 and 6.5e-5. Refused.
    //   rim at 3.3, 1.0 to 3.0: 1.74e-4 and 9.9e-5; 1.65e-4 and 1.00e-4. Refused.
    //   rim at 3.5, 0.4 to 3.0: 2.44e-4 and 1.14e-4; 2.18e-4 and 1.15e-4. Refused.
    //   rim at 3.2, 0.4 to 3.0: 2.79e-4 and 1.35e-4; 2.55e-4 and 1.35e-4. Refused.
    // The second of each pair is a corner where the two sides' surfaces would cross: there the measure is what the faces are
    // off by, to 1%. The first is one where their own circles would leave a gap: there it is the most the balls behind the
    // tilted sections can hold of them, and the faces are off by 0.89 to 0.97 of it. So the measure is never less than what
    // the faces are off by, and every fillet built so is within the tolerance.
    // Unit 15AK3: the last four, which were refused, are built with those corners made: the piece of the ball between the two
    // circles where they leave a gap (EnvelopeLune), a face of its own with the contacts for corners, and the crease the two
    // surfaces cross in where they cross, each face ending on it. Every point of them is then within 1.4e-5 of the boundary
    // of the union of the balls (2.4e-6 beside the corners), no edge covers a gap, and the result is a valid shape of 15
    // faces for 13. Since decision 2420 a corner is built wherever its two circles are more than a thousandth of the
    // ball's radius apart, whatever its measure, as a tilted surface isn't tangent to its ball along the circle it ends on: so
    // all seven have a piece of the ball and a crease on each of their two curves, and each is within 1.4e-5 of the union.
    [Theory]
    [InlineData(3.05, 0.4, 1.0, true)]
    [InlineData(3.2, 0.4, 1.6, true)]
    [InlineData(3.2, 0.2, 2.0, true)]
    [InlineData(3.2, 0.4, 2.4, false)]
    [InlineData(3.3, 1.0, 3.0, false)]
    [InlineData(3.5, 0.4, 3.0, false)]
    [InlineData(3.2, 0.4, 3.0, false)]
    public void An_evolving_fillet_that_rests_on_an_edge_is_within_tolerance_of_its_balls_its_corners_built_where_they_must_be(double top, double low, double high, bool tilted)
    {
        // Built whatever the measure, to see what it is measuring
        MakeFillet unrefused = RimTeeFillet(top, low, high, refused: false);
        Assert.True(unrefused.IsDone);
        Assert.True(unrefused.Shape().Validate().IsValid);
        double measure = unrefused.InternalBuilder().EnvelopeCornerWorst;
        List<(double Miss, Point3D At)> points = UnionMiss(unrefused, p => Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - 3),
                                                           p => new Vector3D(p.X, 0, p.Z).Normalized(), MainBody(top), 10);
        double worst = points.Max(p => p.Miss);
        Assert.Equal(tilted, measure <= Approximation);

        // The faces are off the union of the balls by no more than the measure says, and beside each corner by the measure
        // there, or a little less where the circles would leave a gap
        Assert.True(worst < 1.03 * measure + 1.5e-5, $"off the union of the balls by {worst}, the measure {measure}");
        foreach ((Point3D at, double off, _, _) in unrefused.InternalBuilder().EnvelopeCorners.Where(c => c.Off > 2e-5))
        {
            double beside = points.Where(p => p.At.DistanceTo(at) < 0.1).Max(p => p.Miss);
            Assert.True(beside < 1.03 * off && beside > 0.85 * off, $"off the union of the balls by {beside} beside a corner whose measure is {off}");
        }

        // And the faces meet: no edge of the result covers a gap with its tolerance
        Assert.True(unrefused.Shape().AllEdges().Max(e => Tool.Tolerance(e)) < 2 * Approximation);

        // As a user gets it: its corners built, and the fillet on the union
        MakeFillet chfi3d = RimTeeFillet(top, low, high, refused: true);
        Assert.True(chfi3d.IsDone);
        Assert.Equal((2, 2), chfi3d.InternalBuilder().EnvelopeCornersMade);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        Assert.Equal(unrefused.Shape().AllFaces().Count() + 2, chfi3d.Shape().AllFaces().Count());
        Assert.True(chfi3d.Shape().AllEdges().Max(e => Tool.Tolerance(e)) < 2 * Approximation);
        List<(double Miss, Point3D At)> made = UnionMiss(chfi3d, p => Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - 3),
                                                         p => new Vector3D(p.X, 0, p.Z).Normalized(), MainBody(top), 10);
        Assert.True(made.Count > 0.9 * points.Count);
        double madeWorst = made.Max(p => p.Miss);
        Assert.True(madeWorst < 0.2 * Approximation, $"its corners built, and off the union of the balls by {madeWorst}");
        foreach ((Point3D at, double off, double gap, _) in chfi3d.InternalBuilder().EnvelopeCorners.Where(c => c.Off == 0 && c.Gap > Approximation))
        {
            double beside = made.Where(p => p.At.DistanceTo(at) < 0.1).Max(p => p.Miss);
            Assert.True(beside < 0.05 * Approximation, $"off the union of the balls by {beside} beside a corner that was built");
        }

        if (tilted)
        {
            Assert.True(worst < Approximation, $"with its corners tilted, off the union of the balls by {worst}");
            return;
        }

        // Past the tolerance with every corner tilted, and refused where the tests turn the building off
        Assert.True(worst > Approximation, $"with its corners tilted it was off the union of the balls by only {worst}");
        MakeFillet refused = RimTeeFillet(top, low, high, refused: true, cornersBuilt: false);
        Assert.False(refused.IsDone);
        Assert.Equal(Kerf.Engine.Fillets.Data.ErrorStatus.EnvelopeCorner, refused.StripeStatus(1));
        Assert.Contains("fillet.radius-changes-at-a-corner", Kerf.Engine.Foundation.StatusDiagnostics.ToDiagnostic(refused.StripeStatus(1))!.Code.ToString());
    }

    // Decision 2341: before it each surface ended on its own circle, and at a corner the two didn't meet. The edge between
    // them took the distance as its tolerance, in a shape that was valid: 0.013 on the rim at 3.2 with 0.2 to 2.0, a fillet
    // within the tolerance of the swept surface everywhere. With the later surface ending on the earlier's circle they meet.
    [Fact]
    public void The_surfaces_either_side_of_a_corner_meet()
    {
        MakeFillet Rim(bool meet)
        {
            Shape shape = RimTee(3.2);
            var chfi3d = new MakeFillet(shape);
            chfi3d.InternalBuilder().EnvelopeEndsMeet = meet;
            chfi3d.InternalBuilder().EnvelopeTiltLimit = double.MaxValue; // the tilt itself: no corner built for being wide
            NCollection_Array1<Point2D> law = new(1, 3);
            law.SetValue(1, new Point2D(0.0, 0.2));
            law.SetValue(2, new Point2D(0.5, 2.0));
            law.SetValue(3, new Point2D(1.0, 0.2));
            foreach (Edge e in shape.AllEdges().Where(e =>
            {
                var curve = new EdgeCurveAdaptor(e);
                Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
                return Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - 5) < 1e-6 && Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - 3) < 1e-6;
            }))
            {
                chfi3d.Add(law, e);
            }
            chfi3d.Build();
            Assert.True(chfi3d.IsDone);
            Assert.True(chfi3d.Shape().Validate().IsValid);
            return chfi3d;
        }

        Assert.InRange(Rim(false).Shape().AllEdges().Max(e => Tool.Tolerance(e)), 0.01, 0.02);
        Assert.True(Rim(true).Shape().AllEdges().Max(e => Tool.Tolerance(e)) < Approximation);
    }

    // Two of the corpus's cases the measure refuses, from its own file: a cylinder of radius 5 about z, 12 high, and one of
    // radius 3 along y through it, 2 below its rim. Built with the refusal off, against the union of the balls:
    //   blend/buildevol/K6, the fillet round one of the curves they meet in, 1 to 3 to 2: outside both, its ball comes to
    //   rest on the rim. Its faces are 1.76e-4 inside the union there, the measure 1.78e-4.
    //   blend/buildevol/K7, the fillet of the rim itself, 1 to 3 to 3 to 1: inside the cylinder, under its top, its ball
    //   comes to rest on the curves the cylinders meet in once its radius passes 2. Its faces are 7.1e-4 inside the union
    //   where it does (the measure there 6.8e-4), and where it leaves them the two sides' own circles are 0.22 apart and
    //   the measure is 1.0e-2.
    // OCCT builds both, and Kerf did, with OCCT's areas.
    // Unit 15AK3: both are built, those corners made. K6 has a crease where its ball comes to rest on the rim (its other
    // corner, 6.6e-6, is a crease too since decision 2420) and is within 1.4e-5 of the union of its balls everywhere; K7 has a crease at each of
    // the two corners where its ball comes to rest on the curves and a piece of the ball, 0.22 wide, at each of the two
    // where it leaves them, and is within 5.1e-6.
    [Theory]
    [InlineData("K6", 1.78e-4, 1.78e-4, 0, 2)]
    [InlineData("K7", 1.02e-2, 6.8e-4, 2, 2)]
    public void The_corpuss_fillets_onto_edges_have_their_corners_built_and_were_off_the_union_of_their_balls(string name, double worst, double corner, int pieces, int creases)
    {
        if (Corpus("CFI_6_i12fhh.rle") is not { } shape)
        {
            return;
        }

        bool rim = name == "K7";
        MakeFillet Fillet(bool refused, bool cornersBuilt = true)
        {
            var chfi3d = new MakeFillet(shape);
            chfi3d.InternalBuilder().EnvelopeCornersRefused = refused;
            chfi3d.InternalBuilder().EnvelopeCornersBuilt = cornersBuilt;
            NCollection_Array1<Point2D> law = new(1, rim ? 4 : 3);
            law.SetValue(1, new Point2D(0, 1));
            law.SetValue(2, new Point2D(1, 3));
            law.SetValue(3, new Point2D(2, rim ? 3 : 2));
            if (rim)
            {
                law.SetValue(4, new Point2D(3, 1));
            }

            // The scripts' s_1, the rim, and s_6, the upper half of the curve the cylinders meet in at y < 0
            chfi3d.Add(law, shape.AllEdges()[rim ? 0 : 5]);
            chfi3d.Build();
            return chfi3d;
        }

        MakeFillet unrefused = Fillet(false);
        Assert.True(unrefused.IsDone);
        var corners = unrefused.InternalBuilder().EnvelopeCorners;
        Assert.InRange(unrefused.InternalBuilder().EnvelopeCornerWorst, 0.95 * worst, 1.05 * worst);

        // The cylinder's side as a surface: up to its rim, and less the holes the other cylinder leaves in it, bounded by
        // the curves they meet in, (3 cos t, +-sqrt(25 - 9 cos^2 t), 7 + 3 sin t)
        Distance side = p =>
        {
            double rho = Math.Sqrt(p.X * p.X + p.Y * p.Y), fx = 5 * p.X / rho, fz = Math.Min(p.Z, 12);
            if (fx * fx + (fz - 7) * (fz - 7) >= 9)
            {
                double dr = 5 - rho, dz = p.Z - fz;
                return Math.Sqrt(dr * dr + dz * dz);
            }

            double Curve(double t)
            {
                double x = 3 * Math.Cos(t), y = Math.Sign(p.Y) * Math.Sqrt(25 - x * x), z = 7 + 3 * Math.Sin(t);
                return Math.Sqrt((p.X - x) * (p.X - x) + (p.Y - y) * (p.Y - y) + (p.Z - z) * (p.Z - z));
            }

            int best = 0;
            double least = double.MaxValue;
            for (int k = 0; k < 48; k++)
            {
                double d = Curve(k * Math.PI / 24);
                if (d < least)
                {
                    least = d;
                    best = k;
                }
            }

            double lo = (best - 1) * Math.PI / 24, hi = (best + 1) * Math.PI / 24;
            for (int iter = 0; iter < 40; iter++)
            {
                double a = lo + (hi - lo) / 3, b = hi - (hi - lo) / 3;
                if (Curve(a) < Curve(b))
                {
                    hi = b;
                }
                else
                {
                    lo = a;
                }
            }

            return Curve((lo + hi) / 2);
        };

        // K6's balls rest on the smaller cylinder, outside it; K7's on the top, under it
        List<(double Miss, Point3D At)> Read(MakeFillet fillet) => rim
            ? UnionMiss(fillet, p => Math.Abs(p.Z - 12), p => new Vector3D(0, 0, -1), side, 10)
            : UnionMiss(fillet, p => Math.Abs(Math.Sqrt(p.X * p.X + (p.Z - 7) * (p.Z - 7)) - 3),
                        p => new Vector3D(p.X, 0, p.Z - 7).Normalized(), side, 10);
        List<(double Miss, Point3D At)> points = Read(unrefused);

        // Where the ball comes to rest on the edge
        (Point3D at, double off, _, _) = corners.MinBy(c => Math.Abs(c.Off - corner));
        Assert.InRange(off, 0.95 * corner, 1.05 * corner);
        double beside = points.Where(p => p.At.DistanceTo(at) < 0.1).Max(p => p.Miss);
        Assert.True(Math.Abs(beside - off) < 0.06 * off, $"{name}: off the union of the balls by {beside} beside a corner whose measure is {off}");
        Assert.True(beside > Approximation);

        MakeFillet refused = Fillet(true, cornersBuilt: false);
        Assert.False(refused.IsDone);
        Assert.Equal(Kerf.Engine.Fillets.Data.ErrorStatus.EnvelopeCorner, refused.StripeStatus(1));

        // With the corners built: valid, on the union everywhere, its faces meeting along their edges
        MakeFillet chfi3d = Fillet(true);
        Assert.True(chfi3d.IsDone);
        Assert.Equal((pieces, creases), chfi3d.InternalBuilder().EnvelopeCornersMade);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        List<(double Miss, Point3D At)> made = Read(chfi3d);
        Assert.True(made.Count > 0.9 * points.Count);
        Assert.True(made.Max(p => p.Miss) < 0.2 * Approximation, $"{name}: its corners built, and off the union of the balls by {made.Max(p => p.Miss)}");
        Assert.True(made.Where(p => p.At.DistanceTo(at) < 0.1).Max(p => p.Miss) < 0.05 * Approximation);

        // The warning for each crease past the angular tolerance (0.01 radians), with the greatest angle between the faces'
        // normals along it and where its middle is: K6's 0.88 degrees (its other crease, 0.20, is within it), K7's two of 1.17
        double crease = chfi3d.InternalBuilder().EnvelopeCreaseAngle * 180 / Math.PI;
        Assert.InRange(crease, rim ? 1.15 : 0.86, rim ? 1.19 : 0.90);
        var warnings = chfi3d.InternalBuilder().EnvelopeCreaseWarnings;
        Assert.Equal(rim ? 2 : 1, warnings.Count);
        Assert.All(warnings, w => Assert.Equal(Kerf.Severity.Warning, w.Warning.Severity));
        Assert.All(warnings, w => Assert.Equal("fillet.creased", w.Warning.Code.ToString()));
        Assert.All(warnings, w => Assert.Contains(rim ? "1.17 degrees at (0.5" : "0.88 degrees at (1.2", w.Warning.Message));
        Assert.Null(Kerf.Engine.Foundation.StatusDiagnostics.FilletCreased(0.009, 0.01, default));

        // As the operation gives them (FilletOperations): each with its crease's edge in the result, and carried to the
        // result's diagnostics with the shape, which is a success
        var given = Kerf.Fillets.FilletOperations.Creased(chfi3d, chfi3d.Shape);
        Assert.Equal(warnings.Count, given.Length);
        Assert.All(given, d =>
        {
            Edge edge = (Edge)Assert.Single(d.Shapes);
            Assert.Contains(chfi3d.Shape().AllEdges(), e => e.IsSame(edge));
            var curve = new EdgeCurveAdaptor(edge);
            Assert.Contains(warnings, w => w.Curve.Value(0.5).DistanceTo(curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()))) < 1e-6);
        });
        var result = Kerf.Engine.Foundation.EngineOperation.Run<Shape>(
            _ => new Kerf.Engine.Foundation.EngineOutcome(true, chfi3d.Shape, Warnings: given), Kerf.Diagnostic.Error(new Kerf.DiagnosticCode("fillet.failed"), "failed"));
        Assert.True(result.IsSuccess);
        Assert.Equal(given.Length, result.Diagnostics.Count(d => d.Code.ToString() == "fillet.creased" && d.Severity == Kerf.Severity.Warning));

        // And every face of it meshes, the pieces of the ball, whose two corners are single points of their surfaces, too
        var mesh = new Kerf.Engine.Meshing.IncrementalMesh(chfi3d.Shape(), 0.05);
        Assert.True(mesh.IsDone);
        Assert.All(mesh.Shape().AllFaces(), f => Assert.NotNull(Tool.Triangulation(f, out _)));
    }

    // The corpus's blend/buildevol/K8, built: K7's cylinders with a third of radius 3 standing on the top, and the fillet of
    // the rim, 1 to 3 to 2, whose ball comes to rest between two edges, the foot of the third cylinder and the curve the
    // first two meet in (BRepBlend_RstRstEvolRad). Decision 2339: the second derivative of the ball's centre there is the
    // equations'; taken by differences, it was wrong at 59 of that function's 320 points, beside the corners the walk closes
    // in on, and the area was 766.9269 for 766.9257. Decision 2341: its surfaces meet at the corners, where two of them
    // were 0.017 apart, the edge between them carrying that as its tolerance; closing them brings the area to 766.9996.
    [Fact]
    public void The_corpuss_K8_between_two_edges_is_built_from_the_equations_second_order()
    {
        if (Corpus("CFI_6_i56fhi.rle") is not { } shape)
        {
            return;
        }

        var chfi3d = new MakeFillet(shape);
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0, 1));
        law.SetValue(2, new Point2D(1, 3));
        law.SetValue(3, new Point2D(2, 2));
        chfi3d.Add(law, shape.AllEdges()[5]);
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Shape result = chfi3d.Shape();
        Assert.True(result.Validate().IsValid);
        Assert.InRange(chfi3d.InternalBuilder().EnvelopeCornerWorst, 4.7e-5, 5.1e-5);
        Assert.InRange(result.IntegrateArea(new MassOptions { Tolerance = 1e-10 }), 766.9996 - 3e-4, 766.9996 + 3e-4);
        Assert.True(result.AllEdges().Max(e => Tool.Tolerance(e)) < 1e-3);
    }

    // Decision 1736's case, the corpus's blend/buildevol/J5, which ran through Draw-lite until its fillet was refused (its
    // measure 2.6e-4): where a radius of 4 to 8 to 5 reaches past a circle on the top face the fillet's boundary runs along
    // it, and the edge's part under the boundary goes. Built here with the refusal off, as the fix is to the reconstruction:
    // valid, with OCCT's area. Unit 15AK3: and as a user gets it, its four corners built as creases: valid, the same area
    // to the script's tolerance.
    [Fact]
    public void An_edge_under_a_fillets_boundary_goes()
    {
        if (Corpus("CFI_6_e56fha.rle") is not { } shape)
        {
            return;
        }

        var chfi3d = new MakeFillet(shape);
        chfi3d.InternalBuilder().EnvelopeCornersRefused = false;
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0, 4));
        law.SetValue(2, new Point2D(1, 8));
        law.SetValue(3, new Point2D(2, 5));
        chfi3d.Add(law, shape.AllEdges()[6]);
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Shape result = chfi3d.Shape();
        Assert.True(result.Validate().IsValid);
        // 19573.68, within the script's tolerance (1e-4 of it) of OCCT's 19572.1
        Assert.InRange(result.IntegrateArea(new MassOptions { Tolerance = 1e-9 }), 19572.1 * (1 - 1e-4), 19572.1 * (1 + 1e-4));

        var refused = new MakeFillet(shape);
        refused.InternalBuilder().EnvelopeCornersBuilt = false;
        refused.Add(law, shape.AllEdges()[6]);
        refused.Build();
        Assert.False(refused.IsDone);
        Assert.Equal(Kerf.Engine.Fillets.Data.ErrorStatus.EnvelopeCorner, refused.StripeStatus(1));

        var made = new MakeFillet(shape);
        made.Add(law, shape.AllEdges()[6]);
        made.Build();
        Assert.True(made.IsDone);
        Assert.Equal((0, 4), made.InternalBuilder().EnvelopeCornersMade);
        Assert.True(made.Shape().Validate().IsValid);
        Assert.InRange(made.Shape().IntegrateArea(new MassOptions { Tolerance = 1e-9 }), 19572.1 * (1 - 1e-4), 19572.1 * (1 + 1e-4));
    }

    // One of the corpus's that was always built, blend/buildevol/H9: a cylinder of radius 8 on a box of 20, and the fillet of
    // the circle they meet in, 1 to 4 to 3, whose ball comes to rest on the edges of the box's top and leaves them, six times.
    // Its worst corner's measure is 9.6e-5, and with every corner tilted its faces were 9.6e-5 inside the union of their balls
    // there, and up to 0.64 degrees from tangent across the joins. Since decision 2420 its six corners are built, two pieces
    // of the ball and four creases, and it is within 4.4e-6 of the union everywhere.
    [Fact]
    public void The_corpuss_H9_is_built_within_tolerance_of_the_union_of_its_balls()
    {
        if (Corpus("CFI_6_a12fgr.rle") is not { } shape)
        {
            return;
        }

        var chfi3d = new MakeFillet(shape);
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0, 1));
        law.SetValue(2, new Point2D(1, 4));
        law.SetValue(3, new Point2D(2, 3));
        chfi3d.Add(law, shape.AllEdges()[4]);
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        double measure = chfi3d.InternalBuilder().EnvelopeCornerWorst;
        Assert.InRange(measure, 9.0e-5, Approximation);
        Assert.Equal((2, 4), chfi3d.InternalBuilder().EnvelopeCornersMade);

        // Its four creases are under the angular tolerance (0.15 and 0.47 degrees, twice each, for 0.573): no warning
        Assert.InRange(chfi3d.InternalBuilder().EnvelopeCreaseAngle * 180 / Math.PI, 0.45, 0.49);
        Assert.Empty(chfi3d.InternalBuilder().EnvelopeCreaseWarnings);
        Assert.Empty(Kerf.Fillets.FilletOperations.Creased(chfi3d, chfi3d.Shape));

        // The box's top as a surface, to its edges
        Distance top = p =>
        {
            double dx = Math.Max(Math.Abs(p.X) - 10, 0), dy = Math.Max(Math.Abs(p.Y) - 10, 0), dz = p.Z - 10;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        };
        List<(double Miss, Point3D At)> points = UnionMiss(chfi3d, p => Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - 8),
                                                           p => new Vector3D(p.X, p.Y, 0).Normalized(), top, 10);
        double worst = points.Max(p => p.Miss);
        Assert.True(worst < 0.1 * Approximation, $"off the union of the balls by {worst}");
    }


    private static Shape? Corpus(string file)
    {
        string path = Path.Combine(typeof(EvolvingFilletOnEdgeTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                       .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", file);
        if (!File.Exists(path))
        {
            return null;
        }
        return BrepTools.Read(out Shape shape, path) ? shape : null;
    }

    // The fillet's new faces against the union of its balls, apart from the law, the guide and how the faces were made: each
    // point of a face's boundary on the first surface is where a ball rests on it, the ball centred on that surface's normal
    // there whose radius is its centre's distance from the other body (one root: a distance grows no faster than the radius);
    // 30,001 of them along each face. A point of the fillet is on the surface the balls sweep if it is on the boundary of
    // their union: how far each of 69 by 69 points of each new face, crowded towards its ends, is inside the union or outside
    // it.
    private static List<(double Miss, Point3D At)> UnionMiss(MakeFillet fillet, Distance onFirst, Normal normal, Distance other, double greatest)
    {
        const int Count = 30000, Coarse = 50;
        List<(Point3D C, double R)[]> balls = [];
        List<(FaceSurfaceAdaptor Surface, Face Face)> faces = [];
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape face in fillet.NewFaces(i))
            {
                var surface = new FaceSurfaceAdaptor((Face)face);
                faces.Add((surface, (Face)face));
                double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
                foreach ((bool alongV, double fix) in new[] { (true, u0), (true, u1), (false, v0), (false, v1) })
                {
                    Point3D On(double t) => alongV ? surface.Value(fix, v0 + (v1 - v0) * t) : surface.Value(u0 + (u1 - u0) * t, fix);
                    if (Enumerable.Range(0, 21).Any(k => onFirst(On(k / 20.0)) > 1e-4))
                    {
                        continue;
                    }

                    var line = new (Point3D C, double R)[Count + 1];
                    for (int k = 0; k <= Count; k++)
                    {
                        Point3D q = On(k / (double)Count);
                        Vector3D n = normal(q);
                        double lo = 0, hi = greatest;
                        for (int iter = 0; iter < 60; iter++)
                        {
                            double mid = (lo + hi) / 2;
                            if (other(q.Translated(mid * n)) > mid)
                            {
                                lo = mid;
                            }
                            else
                            {
                                hi = mid;
                            }
                        }

                        double r = (lo + hi) / 2;
                        line[k] = (q.Translated(r * n), r);
                    }

                    balls.Add(line);
                }
            }
        }

        Assert.NotEmpty(balls);
        List<(double, Point3D)> points = [];
        double[] near = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 2e-3, 4e-3, 7e-3, 0.012, 0.02, 0.03, 0.045, 0.06, 0.08, 0.1, 0.13, 0.16, 0.2, 0.25, 0.3, 0.35, 0.4];
        double[] ts = [.. Enumerable.Range(0, 25).Select(k => k / 24.0), .. near, .. near.Select(x => 1 - x)];
        foreach ((FaceSurfaceAdaptor surface, Face face) in faces)
        {
            double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
            foreach (double ta in ts)
            {
                foreach (double tb in ts)
                {
                    double u = u0 + (u1 - u0) * ta, v = v0 + (v1 - v0) * tb;
                    if (new Kerf.Engine.Classification.FaceClassifier(face, new Point2D(u, v), 1e-7).State() != Containment.In)
                    {
                        continue;
                    }

                    Point3D p = surface.Value(u, v);
                    double depth = double.MinValue;
                    foreach ((Point3D C, double R)[] line in balls)
                    {
                        // The deepest of every fiftieth ball, then of all the balls about the three best of those
                        Span<(double D, int K)> best = [(double.MinValue, 0), (double.MinValue, 0), (double.MinValue, 0)];
                        for (int k = 0; k <= Count; k += Coarse)
                        {
                            double d = line[k].R - p.DistanceTo(line[k].C);
                            if (d > best[0].D)
                            {
                                best[2] = best[1];
                                best[1] = best[0];
                                best[0] = (d, k);
                            }
                            else if (d > best[1].D)
                            {
                                best[2] = best[1];
                                best[1] = (d, k);
                            }
                            else if (d > best[2].D)
                            {
                                best[2] = (d, k);
                            }
                        }

                        foreach ((double _, int centre) in best)
                        {
                            for (int k = Math.Max(0, centre - Coarse); k <= Math.Min(Count, centre + Coarse); k++)
                            {
                                depth = Math.Max(depth, line[k].R - p.DistanceTo(line[k].C));
                            }
                        }
                    }

                    points.Add((Math.Abs(depth), p));
                }
            }
        }

        return points;
    }
}
