// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks.Full;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The check that takes shortcuts on small shapes against the check as it was (tests/Shared/FullCheck), on small shapes
/// broken on purpose, and by amounts on either side of what each shortcut allows: out of the plane, off the pcurve, sloped,
/// short, far from the origin, placed and with a pcurve or without. They must find the same on every sub-shape.
/// </summary>
public class BRepCheckComparisonTests(ITestOutputHelper output)
{
    private static readonly double[] s_factors = [0.0, 0.1, 0.25, 0.49, 0.5, 0.51, 0.75, 0.99, 1.0, 1.01, 1.1, 2.0, 10.0, 1000.0];

    public enum PCurves
    {
        None,
        Exact,
        Shifted,
        Skewed,
        Reversed,
    }

    private sealed record Spec(
        int Sides,
        double Radius,
        Point3D Origin,
        Direction3D Normal,
        double Tolerance,
        PCurves PCurves,
        int BadEdge,
        double OutOfPlane,
        double Slope,
        double Shift,
        Placement Placement,
        double FarAlong,
        bool ClearSameParameter,
        bool ReverseWire,
        bool ReverseFace,
        bool Hole,
        bool Bowtie,
        bool Open);

    // A polygon on a plane. Its edges' lines may start far from where the edge is, so the parameters are large, as a slope needs
    private static Face Polygon(Spec s)
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(s.Origin, s.Normal);
        Plane3D aPln = aPlane.Plane;
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, s.Tolerance);
        Wire aW = Ring(aB, aF, aPln, s, s.Radius, false);
        if (s.ReverseWire)
        {
            aW.Reverse();
        }

        aB.Add(aF, aW);
        if (s.Hole)
        {
            Spec aHole = s with { BadEdge = -1, Bowtie = false, Open = false };
            // With the wrong way round when the wire is: the hole is the other way from what it's in
            aB.Add(aF, Ring(aB, aF, aPln, aHole, s.Radius * 0.25, true));
        }

        Shape aShape = aF;
        if (s.ReverseFace)
        {
            aShape = aShape.Reversed();
        }

        return ShapeCasts.Face(aShape.LocatedAny(s.Placement));
    }

    private static Wire Ring(BrepBuilder aB, in Face aF, in Plane3D aPln, Spec s, double radius, bool isHole)
    {
        var aPoints = new Point2D[s.Sides];
        for (int i = 0; i < s.Sides; i++)
        {
            double anAngle = 2.0 * Math.PI * i / s.Sides * (isHole ? -1.0 : 1.0);
            aPoints[i] = new Point2D(radius * Math.Cos(anAngle), radius * Math.Sin(anAngle));
        }

        if (s.Bowtie && s.Sides >= 4)
        {
            (aPoints[1], aPoints[2]) = (aPoints[2], aPoints[1]);
        }

        var aVertices = new Vertex[s.Sides];
        for (int i = 0; i < s.Sides; i++)
        {
            aB.MakeVertex(ref aVertices[i], On(aPln, aPoints[i]), s.Tolerance);
        }

        Wire aW = new();
        aB.MakeWire(ref aW);
        int aCount = s.Open ? s.Sides - 1 : s.Sides;
        for (int i = 0; i < aCount; i++)
        {
            int j = (i + 1) % s.Sides;
            Point3D aP1 = On(aPln, aPoints[i]), aP2 = On(aPln, aPoints[j]);
            bool isBad = i == s.BadEdge;
            if (isBad)
            {
                // The edge away from the plane, or sloped by moving its far end
                Vector3D anUp = new Vector3D(aPln.Axis.Direction);
                aP1 = aP1.Translated(anUp * s.OutOfPlane);
                aP2 = aP2.Translated(anUp * (s.OutOfPlane + s.Slope * aP1.DistanceTo(aP2)));
            }

            double aLength = aP1.DistanceTo(aP2);
            var aDirection = new Direction3D(new Vector3D(aP1, aP2));
            double aFar = isBad ? s.FarAlong : 0.0;
            var aLine = new LineCurve3D(aP1.Translated(new Vector3D(aDirection) * -aFar), aDirection);
            Edge anE = new();
            aB.MakeEdge(ref anE, aLine, s.Tolerance);
            aB.Add(anE, aVertices[i].Oriented(Forward));
            aB.Add(anE, aVertices[j].Oriented(Reversed));
            aB.Range(anE, aFar, aFar + aLength);
            if (s.PCurves != PCurves.None)
            {
                var aDir2 = new Direction2D(new Vector2D(aPoints[i], aPoints[j]));
                Point2D aStart = aPoints[i];
                if (isBad && s.PCurves == PCurves.Shifted)
                {
                    aStart = aStart.Translated(new Vector2D(-aDir2.Y, aDir2.X) * s.Shift);
                }
                else if (isBad && s.PCurves == PCurves.Skewed)
                {
                    aDir2 = new Direction2D(new Vector2D(aDir2.X - aDir2.Y * s.Shift / aLength, aDir2.Y + aDir2.X * s.Shift / aLength));
                }
                else if (isBad && s.PCurves == PCurves.Reversed)
                {
                    aStart = aPoints[j];
                    aDir2 = aDir2.Reversed();
                }

                aStart = aStart.Translated(new Vector2D(aDir2) * -aFar);
                aB.UpdateEdge(anE, new LineCurve2D(aStart, aDir2), aF, s.Tolerance);
            }

            if (isBad && s.ClearSameParameter)
            {
                aB.SameParameter(anE, false);
            }

            aB.Add(aW, anE);
        }

        aW.Closed(!s.Open);
        return aW;
    }

    private static Point3D On(in Plane3D thePlane, in Point2D theUV)
        => global::Kerf.Engine.Geometry.ElSLib.Value(theUV.X, theUV.Y, thePlane);

    private static Spec Simple(double theTolerance = 1.0e-7) => new(
        4, 10.0, new Point3D(), new Direction3D(0, 0, 1), theTolerance, PCurves.None, -1, 0, 0, 0, new Placement(), 0, false, false, false, false, false, false);

    private static void Same(in Shape theShape)
    {
        foreach ((bool aGeometry, bool aParallel, bool anExact) in new[] { (true, true, false), (true, false, false), (false, false, false), (true, false, true) })
        {
            string? aDifference = CheckComparison.Compare(theShape, aGeometry, aParallel, anExact);
            Assert.True(aDifference is null, $"geometry {aGeometry}, parallel {aParallel}, exact {anExact}: {aDifference}");
        }
    }

    private static bool IsValid(in Shape theShape) => new Kerf.Engine.Healing.Checks.Analyzer(theShape, true, true).IsValid();

    [Theory]
    [InlineData(PCurves.None)]
    [InlineData(PCurves.Exact)]
    public void A_sound_polygon_is_valid_both_ways(PCurves thePCurves)
    {
        Face aF = Polygon(Simple() with { PCurves = thePCurves });
        Same(aF);
        Assert.True(IsValid(aF));
    }

    [Theory]
    [InlineData(PCurves.None)]
    [InlineData(PCurves.Exact)]
    public void An_edge_out_of_its_plane_by_either_side_of_its_tolerance(PCurves thePCurves)
    {
        const double aTol = 1.0e-5;
        foreach (double aFactor in s_factors)
        {
            Face aF = Polygon(Simple(aTol) with { PCurves = thePCurves, BadEdge = 1, OutOfPlane = aFactor * aTol });
            Same(aF);
        }

        Assert.False(IsValid(Polygon(Simple(aTol) with { PCurves = thePCurves, BadEdge = 1, OutOfPlane = 1000.0 * aTol })));
    }

    [Fact]
    public void A_pcurve_off_its_edge_by_either_side_of_the_tolerance()
    {
        const double aTol = 1.0e-5;
        foreach (PCurves aKind in new[] { PCurves.Shifted, PCurves.Skewed })
        {
            foreach (double aFactor in s_factors)
            {
                Same(Polygon(Simple(aTol) with { PCurves = aKind, BadEdge = 2, Shift = aFactor * aTol }));
            }

            Assert.False(IsValid(Polygon(Simple(aTol) with { PCurves = aKind, BadEdge = 2, Shift = 1000.0 * aTol })));
        }
    }

    [Fact]
    public void A_reversed_pcurve_is_invalid_though_its_line_is_the_same()
    {
        // Both ends are far apart; the middle sample alone would match
        Face aF = Polygon(Simple() with { PCurves = PCurves.Reversed, BadEdge = 0 });
        Same(aF);
        Assert.False(IsValid(aF));
    }

    [Fact]
    public void A_slope_whose_ends_are_within_tolerance_but_whose_parameters_are_large()
    {
        // The line's projection may be given a unit direction, so its parameters drift by the parameter times the slope's square, halved
        const double aTol = 1.0e-3;
        foreach (double aSlope in new[] { 1.0e-7, 1.0e-5, 5.0e-5, 1.0e-4, 3.9e-4, 4.4e-4, 1.0e-3, 2.0e-3, 0.5 })
        {
            foreach (double aFar in new[] { 0.0, 10.0, 1.0e3, 1.0e6, 1.0e9 })
            {
                foreach (PCurves aKind in new[] { PCurves.None, PCurves.Exact })
                {
                    Same(Polygon(Simple(aTol) with { Radius = 0.01, PCurves = aKind, BadEdge = 0, Slope = aSlope, FarAlong = aFar }));
                    Same(Polygon(Simple(aTol) with { Radius = 1.0, PCurves = aKind, BadEdge = 0, Slope = aSlope, FarAlong = aFar }));
                }
            }
        }
    }

    [Theory]
    [InlineData(1.0e-7)]
    [InlineData(1.0e-5)]
    [InlineData(1.0e-3)]
    public void An_edge_across_the_plane_and_shorter_than_the_tolerance(double theLength)
    {
        // Sloped straight across, within tolerance at both ends: the projection has nothing to project
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new Point3D(), new Direction3D(0, 0, 1));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, 1.0e-2);
        Vertex aV1 = new(), aV2 = new(), aV3 = new();
        aB.MakeVertex(ref aV1, new Point3D(0, 0, 0), 1.0e-2);
        aB.MakeVertex(ref aV2, new Point3D(0, 0, theLength), 1.0e-2);
        aB.MakeVertex(ref aV3, new Point3D(10, 0, 0), 1.0e-2);
        Edge aAcross = new(), aOut = new(), aBack = new();
        aB.MakeEdge(ref aAcross, new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0e-2);
        aB.Add(aAcross, aV1.Oriented(Forward));
        aB.Add(aAcross, aV2.Oriented(Reversed));
        aB.Range(aAcross, 0.0, theLength);
        aOut = TestShapes.Edge(aV2, aV3);
        aBack = TestShapes.Edge(aV3, aV1);
        aB.UpdateEdge(aOut, 1.0e-2);
        aB.UpdateEdge(aBack, 1.0e-2);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aAcross);
        aB.Add(aW, aOut);
        aB.Add(aW, aBack);
        aB.Add(aF, aW);
        Same(aF);
    }

    [Theory]
    [InlineData(1.0e-13)]
    [InlineData(1.0e-11)]
    [InlineData(5.0e-10)]
    [InlineData(1.0e-9)]
    [InlineData(2.0e-9)]
    [InlineData(1.0e-8)]
    [InlineData(1.0e-7)]
    [InlineData(1.0e-6)]
    [InlineData(1.0e-5)]
    [InlineData(1.0e-4)]
    public void A_stub_on_the_plane_with_no_pcurve(double theLength)
    {
        // Too short to trim, the projection fails; that's a fault the shortcut mustn't hide
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new Point3D(), new Direction3D(0, 0, 1)), 1.0e-7);
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(theLength, 0, 0));
        Vertex aV3 = TestShapes.Vertex(new Point3D(10, 10, 0));
        Edge aStub = new();
        aB.MakeEdge(ref aStub, new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1.0e-7);
        aB.Add(aStub, aV1.Oriented(Forward));
        aB.Add(aStub, aV2.Oriented(Reversed));
        aB.Range(aStub, 0.0, theLength);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aStub);
        aB.Add(aW, TestShapes.Edge(aV2, aV3));
        aB.Add(aW, TestShapes.Edge(aV3, aV1));
        aB.Add(aF, aW);
        Same(aF);
    }

    [Fact]
    public void Far_from_the_origin_tiny_and_with_tolerances_near_rounding()
    {
        foreach (double anOffset in new[] { 0.0, 1.0e3, 1.0e6, 1.0e9, 1.0e12 })
        {
            foreach (double aRadius in new[] { 1.0e-6, 1.0e-3, 1.0, 1.0e6 })
            {
                foreach (double aTol in new[] { 0.0, 1.0e-15, 1.0e-12, 1.0e-9, 1.0e-7, 1.0e-3 })
                {
                    if (aRadius < 1.0e-12 * anOffset)
                    {
                        continue; // Its corners are the same point
                    }

                    foreach (PCurves aKind in new[] { PCurves.None, PCurves.Exact })
                    {
                        Same(Polygon(Simple(aTol) with
                        {
                            Radius = aRadius,
                            Origin = new Point3D(anOffset, -anOffset, anOffset * 0.5),
                            Normal = new Direction3D(1, 2, 3),
                            PCurves = aKind,
                        }));
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(PCurves.None)]
    [InlineData(PCurves.Exact)]
    public void Wires_wrong_in_each_way(PCurves thePCurves)
    {
        Spec aBase = Simple() with { PCurves = thePCurves };
        foreach (Spec aBad in new[]
                 {
                     aBase with { Bowtie = true },
                     aBase with { Open = true },
                     aBase with { ReverseWire = true },
                     aBase with { ReverseFace = true },
                     aBase with { Hole = true },
                     aBase with { Hole = true, ReverseWire = true },
                     aBase with { Hole = true, Sides = 3 },
                     aBase with { Sides = 2 },
                     aBase with { ClearSameParameter = true, BadEdge = 0 },
                 })
        {
            Same(Polygon(aBad));
        }

        Assert.False(IsValid(Polygon(aBase with { Bowtie = true })));
        Assert.False(IsValid(Polygon(aBase with { Open = true })));
        Assert.False(IsValid(Polygon(aBase with { ReverseWire = true })));
        Assert.False(IsValid(Polygon(aBase with { ClearSameParameter = true, BadEdge = 0 })));
    }

    [Fact]
    public void An_edge_with_its_pcurve_on_another_range_or_no_3D_curve()
    {
        var aB = new BrepBuilder();
        Spec aSpec = Simple() with { PCurves = PCurves.Exact };
        Face aF = Polygon(aSpec);
        Same(aF);
        var anExp = new Explorer(aF, ShapeType.Edge);
        Edge anE = ShapeCasts.Edge(anExp.Current());
        Tool.Range(anE, out double aFirst, out double aLast);
        aB.Range(anE, aF, aFirst + 0.5, aLast);
        Same(aF);
        aB.Range(anE, aF, aFirst, aLast + 3.0e-10);
        Same(aF);
        aB.Range(anE, aF, aFirst, aLast);
        aB.UpdateEdge(anE, (Curve3D?)null, 1.0e-7);
        Same(aF);
    }

    [Fact]
    public void Placed_turned_and_mirrored()
    {
        Transform3D aMove = Transform3D.Translation(new Vector3D(1000, -2000, 3000));
        Transform3D aTurn = Transform3D.Rotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 1)), 1.1);
        Transform3D aScale = Transform3D.Scale(new Point3D(1, 1, 1), 3.0);
        Transform3D aMirror = Transform3D.Mirror(new Point3D(1, 2, 3));
        foreach (Transform3D aTrsf in new[] { aMove, aTurn, aMove * aTurn, aScale, aMirror })
        {
            foreach (PCurves aKind in new[] { PCurves.None, PCurves.Exact })
            {
                foreach (double aFactor in s_factors)
                {
                    Same(Polygon(Simple(1.0e-5) with { PCurves = aKind, Placement = new Placement(aTrsf), BadEdge = 1, OutOfPlane = aFactor * 1.0e-5 }));
                }
            }
        }
    }

    [Fact]
    public void Random_small_faces_with_all_of_it_at_once()
    {
        var aRandom = new Random(20801);
        double[] aTolerances = [1.0e-7, 1.0e-5, 1.0e-3];
        double[] aRadii = [1.0e-3, 1.0, 100.0, 1.0e5];
        double[] anOffsets = [0.0, 100.0, 1.0e6, 1.0e9];
        int aValid = 0, anInvalid = 0;
        for (int i = 0; i < 3000; i++)
        {
            double aTol = aTolerances[aRandom.Next(aTolerances.Length)];
            double anOffset = anOffsets[aRandom.Next(anOffsets.Length)];
            Transform3D aTrsf = aRandom.Next(2) == 0
                ? Transform3D.Rotation(new Axis3D(new Point3D(aRandom.NextDouble(), 0, 0), new Direction3D(aRandom.NextDouble() + 0.1, aRandom.NextDouble(), 1)), aRandom.NextDouble() * 6.0)
                : new Transform3D();

            bool isBad = aRandom.Next(4) != 0;
            Spec aSpec = new(
                3 + aRandom.Next(6),
                aRadii[aRandom.Next(aRadii.Length)],
                new Point3D(anOffset * aRandom.NextDouble(), anOffset * aRandom.NextDouble(), anOffset * aRandom.NextDouble()),
                new Direction3D(aRandom.NextDouble() - 0.5, aRandom.NextDouble() - 0.5, aRandom.NextDouble() + 0.1),
                aTol,
                (PCurves)aRandom.Next(5),
                isBad ? aRandom.Next(3) : -1,
                aRandom.Next(2) == 0 ? 0.0 : s_factors[aRandom.Next(s_factors.Length)] * aTol,
                aRandom.Next(4) == 0 ? Math.Pow(10.0, -aRandom.Next(1, 9)) : 0.0,
                s_factors[aRandom.Next(s_factors.Length)] * aTol,
                new Placement(aTrsf),
                aRandom.Next(4) == 0 ? Math.Pow(10.0, aRandom.Next(0, 9)) : 0.0,
                aRandom.Next(16) == 0,
                aRandom.Next(16) == 0,
                aRandom.Next(16) == 0,
                aRandom.Next(8) == 0,
                aRandom.Next(16) == 0,
                aRandom.Next(16) == 0);
            Face aF = Polygon(aSpec);
            string? aDifference = CheckComparison.Compare(aF, true, true);
            Assert.True(aDifference is null, $"{aDifference} for {aSpec}");
            if (IsValid(aF))
            {
                aValid++;
            }
            else
            {
                anInvalid++;
            }
        }

        output.WriteLine($"{aValid} valid and {anInvalid} invalid");
        Assert.True(aValid > 300 && anInvalid > 300, $"{aValid} valid and {anInvalid} invalid: this tests too little of one side");
    }
}
