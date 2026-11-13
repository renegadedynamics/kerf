// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Curves against surfaces (decision 1753). Every point found must lie on the curve at its parameter and on the surface
/// at its parameters, and the number of points is the geometry's: this holds for lines against infinite surfaces, and for
/// conics against every kind of surface.
/// </summary>
public class IntCurveSurface_LawsTests
{
    private const double Tol = 1e-6;
    private static readonly Direction3D X = new(1, 0, 0);
    private static readonly Direction3D Y = new(0, 1, 0);
    private static readonly Direction3D Z = new(0, 0, 1);

    private static Line3D LineThrough(double x, double y, double z, double dx, double dy, double dz) =>
        new(new Point3D(x, y, z), new Direction3D(dx, dy, dz));

    private static LineCurve3D Line(double x, double y, double z, double dx, double dy, double dz) =>
        new(LineThrough(x, y, z, dx, dy, dz));

    private static HInter Check(Curve3D theCurve, Surface theSurface, int theExpected)
    {
        var aCurve = new CurveAdaptor(theCurve);
        var aSurface = new SurfaceAdaptor(theSurface);
        var anInter = new HInter();
        anInter.Perform(aCurve, aSurface);
        Assert.True(anInter.IsDone);
        Verify(anInter, theCurve, theSurface);
        Assert.Equal(theExpected, anInter.PointCount);
        return anInter;
    }

    private static void Verify(HInter theInter, Curve3D theCurve, Surface theSurface)
    {
        for (int i = 1; i <= theInter.PointCount; i++)
        {
            CurveSurfaceIntersectionPoint aP = theInter.Point(i);
            Assert.True(aP.Pnt().DistanceTo(theCurve.Value(aP.W())) < Tol, $"point {i} off the curve");
            Assert.True(aP.Pnt().DistanceTo(theSurface.Value(aP.U(), aP.V())) < Tol, $"point {i} off the surface");
        }
    }

    private static NCollection_Array1<Point3D> Poles(params double[] theXYZ)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, theXYZ.Length / 3);
        for (int i = 0; i < theXYZ.Length / 3; i++)
        {
            aPoles[i + 1] = new Point3D(theXYZ[3 * i], theXYZ[3 * i + 1], theXYZ[3 * i + 2]);
        }
        return aPoles;
    }

    private static BezierCurve3D Arch() => new(Poles(-5, 0, 0, -2, 6, 0, 2, 6, 0, 5, 0, 0));

    private static ExtrusionSurface ArchTunnel() => new(Arch(), Z);

    // ---- infinite surfaces ----

    [Theory]
    [InlineData(0, -5, 3, 0, 1, 0, 1)] // up into the arch's top, along the section: the arch is hit once going up
    [InlineData(-10, 2, 3, 1, 0, 0, 2)] // across both legs
    [InlineData(-10, 2, 3, 1, 0, 0.5, 2)] // slanting along the tunnel
    [InlineData(-10, 20, 3, 1, 0, 0, 0)] // over the top
    [InlineData(0, 2, -50, 0, 0, 1, 0)] // along the tunnel, inside it
    public void A_line_against_an_extrusion_with_no_end(double x, double y, double z, double dx, double dy, double dz, int theExpected)
    {
        Check(Line(x, y, z, dx, dy, dz), ArchTunnel(), theExpected);
    }

    [Fact]
    public void A_line_against_an_extrusion_with_one_end()
    {
        var aHalf = new TrimmedSurface(ArchTunnel(), 0, 1, 0, 100); // z from 0 to 100
        Check(Line(-10, 2, 3, 1, 0, 0), aHalf, 2);
        Check(Line(-10, 2, -3, 1, 0, 0), aHalf, 0);
    }

    [Fact]
    public void A_line_against_an_extrusion_along_a_slanted_direction()
    {
        var aSlanted = new ExtrusionSurface(Arch(), new Direction3D(0.3, 0, 1));
        Check(Line(-10, 2, 3, 1, 0, 0), aSlanted, 2);
        // along its direction, outside it: OCCT clamps the surface's limits to 1e50 before asking whether they're infinite
        // (occt-issues.md), so at those parameters rounding alone can put the line on the surface, and nowhere nearer
        LineCurve3D aParallel = Line(0, 2, -50, 0.3, 0, 1);
        var anInter = new HInter();
        anInter.Perform(new CurveAdaptor(aParallel), new SurfaceAdaptor(aSlanted));
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            Assert.True(Math.Abs(anInter.Point(i).W()) > 1e30);
        }
    }

    // A parabola x = z^2 / 4F in the plane y = 0, revolved about z, has infinite V in both directions
    private static RevolutionSurface Bowl()
    {
        var aFrame = new Frame3D(new Point3D(5, 0, 0), Y, Z); // its apex at x = 5, opening towards +z
        return new RevolutionSurface(new ParabolaCurve3D(new Parabola3D(aFrame, 1)), new Axis3D(new Point3D(0, 0, 0), Z));
    }

    [Theory]
    [InlineData(-50, 0, 10, 1, 0, 0, 2)] // across the bowl at a height
    [InlineData(0, 0, 10, 1, 0, 0, 2)]
    public void A_line_against_a_revolution_with_no_end(double x, double y, double z, double dx, double dy, double dz, int theMin)
    {
        var aCurve = new CurveAdaptor(Line(x, y, z, dx, dy, dz));
        var aSurface = new SurfaceAdaptor(Bowl());
        var anInter = new HInter();
        anInter.Perform(aCurve, aSurface);
        Assert.True(anInter.IsDone);
        Verify(anInter, Line(x, y, z, dx, dy, dz), Bowl());
        Assert.True(anInter.PointCount >= theMin, anInter.PointCount.ToString());
    }

    [Fact]
    public void A_line_along_the_axis_of_a_revolution_misses_it()
    {
        // the bowl closes on the axis where the parabola crosses it, at z = 25 / 4, for every angle round it
        var aLine = Line(0, 0, -50, 0, 0, 1);
        var anInter = new HInter();
        anInter.Perform(new CurveAdaptor(aLine), new SurfaceAdaptor(Bowl()));
        Assert.True(anInter.IsDone);
        Verify(anInter, aLine, Bowl());
        Assert.True(anInter.PointCount >= 1);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            Assert.Equal(6.25, anInter.Point(i).Pnt().Z, 5);
        }
    }

    [Fact]
    public void A_line_against_a_revolution_of_a_line_is_against_a_cone_or_a_plane()
    {
        var aCone = new RevolutionSurface(Line(5, 0, 0, 1, 0, 1), new Axis3D(new Point3D(0, 0, 0), Z));
        Check(Line(-50, 0, 10, 1, 0, 0), aCone, 4 - 2);
        var aDisc = new RevolutionSurface(Line(5, 0, 7, 1, 0, 0), new Axis3D(new Point3D(0, 0, 0), Z));
        Check(Line(0, 1, -5, 0, 0, 1), aDisc, 2); // the infinite line reaches it from both sides of the axis
    }

    public static TheoryData<int> Offsets() => [0, 1, 2, 3, 4];

    private static Surface OffsetBasis(int theIndex) => theIndex switch
    {
        0 => new PlaneSurface(new Plane3D(new Point3D(0, 0, 0), Z)),
        1 => new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Z, X), 5),
        2 => ArchTunnel(),
        3 => new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Z, X), 0.3, 5),
        _ => new TrimmedSurface(new PlaneSurface(new Plane3D(new Point3D(0, 0, 0), Z)), -50, 50, -50, 50),
    };

    [Theory]
    [MemberData(nameof(Offsets))]
    public void A_line_against_an_infinite_offset(int theIndex)
    {
        var anOffset = new OffsetSurface(OffsetBasis(theIndex), 1.0);
        foreach (LineCurve3D aLine in new[] { Line(0, 0, -50, 0.01, 0.02, 1), Line(-50, 2, 3, 1, 0.01, 0.2), Line(-50, 0.5, 20, 1, 0, 0) })
        {
            var anInter = new HInter();
            anInter.Perform(new CurveAdaptor(aLine), new SurfaceAdaptor(anOffset));
            Assert.True(anInter.IsDone);
            Verify(anInter, aLine, anOffset);
        }
    }

    [Fact]
    public void An_offset_of_a_plane_is_met_where_the_plane_moved()
    {
        var anOffset = new OffsetSurface(OffsetBasis(0), 3.0);
        HInter anInter = Check(Line(1, 2, -50, 0, 0, 1), anOffset, 1);
        Assert.Equal(3.0, anInter.Point(1).Pnt().Z, 7);
        Assert.Equal(53.0, anInter.Point(1).W(), 7);
    }

    [Fact]
    public void An_offset_of_a_cylinder_is_a_wider_cylinder()
    {
        var anOffset = new OffsetSurface(OffsetBasis(1), 2.0);
        HInter anInter = Check(Line(-50, 0, 3, 1, 0, 0), anOffset, 2);
        Assert.Equal(50 - 7, anInter.Point(1).W(), 6);
        Assert.Equal(50 + 7, anInter.Point(2).W(), 6);
    }

    [Fact]
    public void An_offset_of_an_infinite_extrusion_is_estimated_from_its_basis()
    {
        var anOffset = new OffsetSurface(ArchTunnel(), 0.5);
        Check(Line(-20, 2, 3, 1, 0, 0), anOffset, 2);
    }

    // ---- conics against each kind of surface ----

    private static Curve3D Conic(int theKind) => theKind switch
    {
        // each runs through (0, 0, 0) in the plane z = 0 and reaches well past x = 3 either side
        0 => new CircleCurve3D(new Circle3D(new Frame3D(new Point3D(0, 0, 0), Z), 4)),
        1 => new EllipseCurve3D(new Ellipse3D(new Frame3D(new Point3D(0, 0, 0), Z), 6, 4)),
        2 => new TrimmedCurve3D(new ParabolaCurve3D(new Parabola3D(new Frame3D(new Point3D(0, -6, 0), Z, Y), 1)), -8, 8),
        3 => new TrimmedCurve3D(new HyperbolaCurve3D(new Hyperbola3D(new Frame3D(new Point3D(-4, 0, 0), Z, X), 1, 1)), -3, 3),
        _ => new TrimmedCurve3D(new LineCurve3D(new Line3D(new Point3D(-10, 0, 0), X)), 0, 20),
    };

    private static BSplineSurface Sheet()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D((i - 2) * 20, (j - 2) * 20, 0.1 * (i - 2) * (j - 2));
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 2, 2);
    }

    private static Surface Target(int theKind) => theKind switch
    {
        // each crosses the plane z = 0 in a curve that the conics above cross
        0 => new PlaneSurface(new Plane3D(new Point3D(3, 0, 0), X)),
        1 => new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Y, X), 3),
        2 => new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, -5), Z, X), 0.2, 2),
        3 => new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Z, X), 3),
        4 => new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Y, X), 3, 1),
        5 => new ExtrusionSurface(new BezierCurve3D(Poles(3, -9, -5, 2, 0, 0, 3, 9, 5)), new Direction3D(0, 0.1, 1)),
        6 => new RevolutionSurface(new BezierCurve3D(Poles(3, 0, -5, 2, 0, 0, 3, 0, 5)), new Axis3D(new Point3D(0, 0, 0), Z)),
        7 => new OffsetSurface(new TrimmedSurface(new PlaneSurface(new Plane3D(new Point3D(2, 0, 0), X)), -30, 30, -30, 30), 1),
        _ => new BezierSurface(TiltedPoles()),
    };

    private static NCollection_Array2<Point3D> TiltedPoles()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(3, -20, -20);
        aPoles[1, 2] = new Point3D(3, 20, -20);
        aPoles[2, 1] = new Point3D(3, -20, 20);
        aPoles[2, 2] = new Point3D(3, 20, 20);
        return aPoles;
    }

    public static TheoryData<int, int> ConicsAndTargets()
    {
        var aData = new TheoryData<int, int>();
        for (int c = 0; c < 5; c++)
        {
            for (int s = 0; s < 9; s++)
            {
                aData.Add(c, s);
            }
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(ConicsAndTargets))]
    public void A_conic_against_a_surface_finds_only_points_on_both(int theCurve, int theSurface)
    {
        Curve3D aCurve = Conic(theCurve);
        Surface aSurface = Target(theSurface);
        var anInter = new HInter();
        anInter.Perform(new CurveAdaptor(aCurve), new SurfaceAdaptor(aSurface));
        Assert.True(anInter.IsDone);
        Verify(anInter, aCurve, aSurface);
    }

    [Theory]
    [InlineData(0, 2)]
    [InlineData(1, 2)]
    [InlineData(4, 1)]
    public void A_conic_crosses_the_plane_as_many_times_as_it_should(int theCurve, int theExpected)
    {
        Check(Conic(theCurve), Target(0), theExpected);
    }

    [Theory]
    [InlineData(0, 8)] // the circle, against the torus round y: 2 x 2 x 2 tube crossings or so, by symmetry
    [InlineData(4, 4)]
    public void A_curve_through_a_torus_crosses_its_tube_by_symmetry(int theCurve, int theExpected)
    {
        var aTorus = new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Y, X), 3, 1);
        var anInter = new HInter();
        Curve3D aCurve = Conic(theCurve);
        anInter.Perform(new CurveAdaptor(aCurve), new SurfaceAdaptor(aTorus));
        Verify(anInter, aCurve, aTorus);
        if (theCurve == 4)
        {
            Assert.Equal(theExpected, anInter.PointCount);
        }
        else
        {
            Assert.True(anInter.PointCount >= 2);
        }
    }

    [Fact]
    public void A_B_spline_curve_against_a_B_spline_surface()
    {
        var aCurve = new BezierCurve3D(Poles(1, 2, -5, 2, 3, -1, 1, 4, 2, 3, 2, 6));
        Surface aSheet = Sheet();
        HInter anInter = Check(aCurve, aSheet, 1);
        Assert.InRange(anInter.Point(1).W(), 0, 1);
        Assert.Equal(TransitionOnCurve.In == anInter.Point(1).Transition() ? TransitionOnCurve.In : TransitionOnCurve.Out, anInter.Point(1).Transition());
    }

    [Fact]
    public void A_curve_entering_and_leaving_a_sphere_goes_in_and_out()
    {
        var aSphere = new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Z, X), 3);
        HInter anInter = Check(Line(-10, 0.2, 0.1, 1, 0, 0), aSphere, 2);
        Assert.NotEqual(anInter.Point(1).Transition(), anInter.Point(2).Transition());
        double aW1 = Math.Min(anInter.Point(1).W(), anInter.Point(2).W()), aW2 = Math.Max(anInter.Point(1).W(), anInter.Point(2).W());
        double aHalfChord = Math.Sqrt(9 - (0.2 * 0.2) - (0.1 * 0.1));
        Assert.Equal(10 - aHalfChord, aW1, 7);
        Assert.Equal(10 + aHalfChord, aW2, 7);
    }

    // ---- given polygons and polyhedra ----

    [Fact]
    public void The_given_polygon_or_polyhedron_gives_the_same_points_as_the_default_ones()
    {
        Curve3D aCurve = new BezierCurve3D(Poles(1, 2, -5, 2, 3, -1, 1, 4, 2, 3, 2, 6));
        Surface aSheet = Sheet();
        var aC = new CurveAdaptor(aCurve);
        var aS = new SurfaceAdaptor(aSheet);
        var aPolygon = new ThePolygonOfHInter(aC, 30);
        var aPolyhedron = new ThePolyhedronOfHInter(aS, 20, 20, 0, 0, 1, 1);

        var aDefault = new HInter();
        aDefault.Perform(aC, aS);
        Assert.Equal(1, aDefault.PointCount);
        Point3D anExpected = aDefault.Point(1).Pnt();

        var aOne = new HInter();
        aOne.Perform(aC, aPolygon, aS);
        Assert.Equal(1, aOne.PointCount);
        Assert.True(anExpected.DistanceTo(aOne.Point(1).Pnt()) < Tol);

        var aTwo = new HInter();
        aTwo.Perform(aC, aS, aPolyhedron);
        Assert.Equal(1, aTwo.PointCount);
        Assert.True(anExpected.DistanceTo(aTwo.Point(1).Pnt()) < Tol);

        var aThree = new HInter();
        aThree.Perform(aC, aPolygon, aS, aPolyhedron);
        Assert.Equal(1, aThree.PointCount);
        Assert.True(anExpected.DistanceTo(aThree.Point(1).Pnt()) < Tol);
    }

    [Fact]
    public void The_results_object_answers_only_within_its_range()
    {
        var anInter = new HInter();
        Assert.False(anInter.IsDone);
        Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => anInter.PointCount);
        Assert.Throws<Kerf.Engine.Foundation.NotDoneException>(() => anInter.SegmentCount);
        HInter aDone = Check(Line(-10, 0.2, 0.1, 1, 0, 0), new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), Z, X), 3), 2);
        Assert.Equal(0, aDone.SegmentCount);
        Assert.False(aDone.IsParallel);
        Assert.Throws<Kerf.Engine.Foundation.OutOfRangeException>(() => aDone.Point(3));
        Assert.Throws<Kerf.Engine.Foundation.OutOfRangeException>(() => aDone.Segment(1));
        Assert.Contains("IntCurveSurface", aDone.ToString());
    }

    // ---- segments, as values ----

    [Fact]
    public void A_segment_holds_its_two_ends_and_compares_by_them()
    {
        var aP1 = new CurveSurfaceIntersectionPoint(new Point3D(1, 2, 3), 0.1, 0.2, 0.3, TransitionOnCurve.In);
        var aP2 = new CurveSurfaceIntersectionPoint(new Point3D(4, 5, 6), 0.4, 0.5, 0.6, TransitionOnCurve.Out);
        var aSegment = new IntersectionSegment(aP1, aP2);
        aSegment.Values(out CurveSurfaceIntersectionPoint aFirst, out CurveSurfaceIntersectionPoint aSecond);
        Assert.Equal(aP1, aFirst);
        Assert.Equal(aP2, aSecond);
        aSegment.FirstPoint(out aFirst);
        aSegment.SecondPoint(out aSecond);
        Assert.Equal(aP1, aFirst);
        Assert.Equal(aP2, aSecond);
        Assert.Equal(aP1, aSegment.FirstPoint());
        Assert.Equal(aP2, aSegment.SecondPoint());
        var aSame = new IntersectionSegment();
        aSame.SetValues(aP1, aP2);
        Assert.True(aSegment == aSame);
        Assert.False(aSegment != aSame);
        Assert.True(aSegment.Equals((object)aSame));
        Assert.Equal(aSegment.GetHashCode(), aSame.GetHashCode());
        Assert.False(aSegment == new IntersectionSegment(aP2, aP1));
        Assert.False(aSegment.Equals("no"));
        Assert.Contains("IntersectionSegment", aSegment.ToString());
    }

    // Upstream 53706f78a8 (decision 2354): the transition was read from the surface's derivatives as they are, so a
    // surface whose parameters run over a long range for its size (derivatives of 1e-7) gave every crossing as a
    // tangency, which a solid's classifier skips; the crossing's direction doesn't depend on the parameters' scale
    [Theory]
    [InlineData(1.0)]
    [InlineData(1e7)]
    [InlineData(1e-7)]
    public void A_crossing_is_in_or_out_whatever_the_scale_of_the_surfaces_parameters(double theRange)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(1, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1, 0);
        aPoles[2, 2] = new Point3D(1, 1, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = theRange;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        var aSheet = new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 1, 1);
        HInter anUp = Check(new TrimmedCurve3D(Line(0.3, 0.6, -1, 0, 0, 1), 0, 2), aSheet, 1);
        Assert.Equal(TransitionOnCurve.Out, anUp.Point(1).Transition());
        HInter aDown = Check(new TrimmedCurve3D(Line(0.3, 0.6, 1, 0, 0, -1), 0, 2), aSheet, 1);
        Assert.Equal(TransitionOnCurve.In, aDown.Point(1).Transition());
        HInter aSlant = Check(new TrimmedCurve3D(Line(0.3, 0.6, -1, 0.001, 0, 1), 0, 2), aSheet, 1);
        Assert.Equal(TransitionOnCurve.Out, aSlant.Point(1).Transition());
    }
}
