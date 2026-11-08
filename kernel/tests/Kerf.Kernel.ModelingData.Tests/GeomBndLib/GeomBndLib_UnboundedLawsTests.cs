// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Boxes of what runs to infinity, by the law every box keeps (unit 12Q, decision 1753): it holds every point of its
/// curve or surface, so it is open on each side the curve runs off towards, and closed, at the curve's own bound, on a
/// side it never passes. OCCT opened the three minima or maxima by the sign of the infinite end, which left out a
/// coordinate going the other way (decision 23104). And the general surface's box, sampled and then pushed out to the
/// surface's own extremes.
/// </summary>
public class GeomBndLib_UnboundedLawsTests
{
    private const double Inf = 2.0e100;

    private static readonly Frame3D Turned = new(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 1, 0));
    private static readonly Frame2D Turned2d = new(new Point2D(1, 2), new Direction2D(1, 1));

    private static void AssertHolds(in Box theBox, IEnumerable<Point3D> thePoints)
    {
        foreach (Point3D aP in thePoints)
        {
            Assert.True(aP.X >= theBox.GetXMin() && aP.X <= theBox.GetXMax() && aP.Y >= theBox.GetYMin() && aP.Y <= theBox.GetYMax()
                        && aP.Z >= theBox.GetZMin() && aP.Z <= theBox.GetZMax(), $"{aP} is out of the box");
        }
    }

    private static void AssertHolds(in Box2d theBox, IEnumerable<Point2D> thePoints)
    {
        foreach (Point2D aP in thePoints)
        {
            Assert.True(aP.X >= theBox.GetXMin() && aP.X <= theBox.GetXMax() && aP.Y >= theBox.GetYMin() && aP.Y <= theBox.GetYMax(),
                        $"{aP} is out of the box");
        }
    }

    // A closed side is at the curve's bound: no further out than the least of the samples, and only as far in as the
    // samples' spacing allows
    private static void AssertTight(double theSampled, double theSide) => Assert.InRange(theSide, theSampled - 1e-4, theSampled);

    private static IEnumerable<double> Along(double theFrom, double theTo, int theNb = 2000) =>
        Enumerable.Range(0, theNb + 1).Select(i => theFrom + ((theTo - theFrom) * i / theNb));

    [Theory]
    [InlineData(1.0)]
    [InlineData(-1.0)]
    public void A_ray_slanting_down_is_open_where_it_goes(double theSign)
    {
        // from (1, 2, 3) along (1, -1, 0) / sqrt 2: x grows and y falls without end, z stays
        var aLine = new Line3D(new Point3D(1, 2, 3), new Direction3D(theSign, -theSign, 0));
        Box aBox = LineBounds.Box(aLine, 0, Inf, 0);
        Assert.True(aBox.IsOpenXmax == theSign > 0 && aBox.IsOpenXmin == theSign < 0);
        Assert.True(aBox.IsOpenYmin == theSign > 0 && aBox.IsOpenYmax == theSign < 0);
        Assert.False(aBox.IsOpenZmin || aBox.IsOpenZmax);
        Assert.Equal(3, aBox.GetZMax(), 12);
        AssertHolds(aBox, Along(0, 1e6).Select(u => ElCLib.Value(u, aLine)));
        // and the other way along the line
        Box aBack = LineBounds.Box(aLine, -Inf, 0, 0);
        AssertHolds(aBack, Along(-1e6, 0).Select(u => ElCLib.Value(u, aLine)));
        Assert.True(aBack.IsOpenXmin == theSign > 0 && aBack.IsOpenYmax == theSign > 0);
    }

    [Fact]
    public void A_ray_slanting_down_in_the_plane_is_open_where_it_goes()
    {
        var aLine = new Line2D(new Point2D(1, 2), new Direction2D(1, -1));
        Box2d aBox = Line2dBounds.Box(aLine, 0, Inf, 0);
        Assert.True(aBox.IsOpenXmax && !aBox.IsOpenXmin && aBox.IsOpenYmin && !aBox.IsOpenYmax);
        Assert.Equal(1, aBox.GetXMin(), 12);
        Assert.Equal(2, aBox.GetYMax(), 12);
        AssertHolds(aBox, Along(0, 1e6).Select(u => ElCLib.Value(u, aLine)));
    }

    [Theory]
    [InlineData(-Inf, 3.0)]
    [InlineData(-3.0, Inf)]
    [InlineData(Inf, 3.0)]
    [InlineData(-6.0, -Inf)]
    public void Half_a_turned_parabola_is_open_where_it_goes_and_bounded_where_it_turns(double theU1, double theU2)
    {
        // axis (1, 1, 0) / sqrt 2, focal 2: the point goes off along +X whichever way, so x and y grow without end
        var aParab = new Parabola3D(Turned, 2);
        Box aBox = ParabolaBounds.Box(aParab, theU1, theU2, 0);
        Assert.True(aBox.IsOpenXmax && aBox.IsOpenYmax);
        Assert.False(aBox.IsOpenXmin || aBox.IsOpenYmin || aBox.IsOpenZmin || aBox.IsOpenZmax);
        double aFinite = Math.Abs(theU1) < 1e50 ? theU1 : theU2, aFar = Math.Sign(Math.Abs(theU1) < 1e50 ? theU2 : theU1) * 1e4;
        List<Point3D> aPoints = Along(Math.Min(aFinite, aFar), Math.Max(aFinite, aFar), 20000).Select(u => ElCLib.Value(u, aParab)).ToList();
        AssertHolds(aBox, aPoints);
        // the closed sides are the curve's own least coordinates, at its finite end or where it turns
        AssertTight(aPoints.Min(p => p.X), aBox.GetXMin());
        AssertTight(aPoints.Min(p => p.Y), aBox.GetYMin());
        Assert.Equal(3, aBox.GetZMin(), 12);
    }

    [Theory]
    [InlineData(-Inf, 3.0)]
    [InlineData(-3.0, Inf)]
    public void Half_a_turned_parabola_in_the_plane_is_open_where_it_goes(double theU1, double theU2)
    {
        var aParab = new Parabola2D(Turned2d, 2);
        Box2d aBox = Parabola2dBounds.Box(aParab, theU1, theU2, 0);
        Assert.True(aBox.IsOpenXmax && aBox.IsOpenYmax && !aBox.IsOpenXmin);
        double aFinite = Math.Abs(theU1) < 1e50 ? theU1 : theU2, aFar = Math.Sign(Math.Abs(theU1) < 1e50 ? theU2 : theU1) * 1e4;
        List<Point2D> aPoints = Along(Math.Min(aFinite, aFar), Math.Max(aFinite, aFar), 20000).Select(u => ElCLib.Value(u, aParab)).ToList();
        AssertHolds(aBox, aPoints);
        AssertTight(aPoints.Min(p => p.X), aBox.GetXMin());
    }

    [Theory]
    [InlineData(-Inf, 1.0)]
    [InlineData(-1.0, Inf)]
    [InlineData(Inf, -1.0)]
    public void Half_a_turned_hyperbola_is_open_where_it_goes(double theU1, double theU2)
    {
        // radii 3 and 1, turned by 45 degrees: one way it goes along 3 X + Y, the other along 3 X - Y
        var aHypr = new Hyperbola3D(Turned, 3, 1);
        Box aBox = HyperbolaBounds.Box(aHypr, theU1, theU2, 0);
        double aFinite = Math.Abs(theU1) < 1e50 ? theU1 : theU2, aFar = Math.Sign(Math.Abs(theU1) < 1e50 ? theU2 : theU1) * 20;
        List<Point3D> aPoints = Along(Math.Min(aFinite, aFar), Math.Max(aFinite, aFar), 20000).Select(u => ElCLib.Value(u, aHypr)).ToList();
        AssertHolds(aBox, aPoints);
        Assert.False(aBox.IsOpenZmin || aBox.IsOpenZmax);
        // X is (1, 1, 0) / sqrt 2 and Y (-1, 1, 0) / sqrt 2: 3 X + Y and 3 X - Y both have x and y growing, so those
        // sides are open, and the others closed at the curve's bound
        Assert.True(aBox.IsOpenXmax && aBox.IsOpenYmax);
        Assert.False(aBox.IsOpenXmin || aBox.IsOpenYmin);
        AssertTight(aPoints.Min(p => p.X), aBox.GetXMin());
        AssertTight(aPoints.Min(p => p.Y), aBox.GetYMin());
    }

    [Theory]
    [InlineData(-Inf, 1.0)]
    [InlineData(-1.0, Inf)]
    public void Half_a_turned_hyperbola_in_the_plane_is_open_where_it_goes(double theU1, double theU2)
    {
        var aHypr = new Hyperbola2D(Turned2d, 3, 1);
        Box2d aBox = Hyperbola2dBounds.Box(aHypr, theU1, theU2, 0);
        double aFinite = Math.Abs(theU1) < 1e50 ? theU1 : theU2, aFar = Math.Sign(Math.Abs(theU1) < 1e50 ? theU2 : theU1) * 20;
        List<Point2D> aPoints = Along(Math.Min(aFinite, aFar), Math.Max(aFinite, aFar), 20000).Select(u => ElCLib.Value(u, aHypr)).ToList();
        AssertHolds(aBox, aPoints);
        Assert.False(aBox.IsOpenXmin);
        AssertTight(aPoints.Min(p => p.X), aBox.GetXMin());
    }

    [Fact]
    public void A_whole_conic_is_open_all_round_and_an_arc_from_infinity_to_itself_is_refused()
    {
        var aParab = new Parabola3D(Turned, 2);
        var aHypr = new Hyperbola3D(Turned, 3, 1);
        foreach (Box aBox in new[] { ParabolaBounds.Box(aParab, -Inf, Inf, 0), HyperbolaBounds.Box(aHypr, Inf, -Inf, 0) })
        {
            Assert.True(aBox.IsOpenXmin && aBox.IsOpenXmax && aBox.IsOpenYmin && aBox.IsOpenYmax && aBox.IsOpenZmin && aBox.IsOpenZmax);
        }
        Box2d aBox2d = Parabola2dBounds.Box(new Parabola2D(Turned2d, 2), -Inf, Inf, 0);
        Assert.True(aBox2d.IsOpenXmin && aBox2d.IsOpenXmax && aBox2d.IsOpenYmin && aBox2d.IsOpenYmax);
        EngineAssert.Throws<FailureException>(() => ParabolaBounds.Box(aParab, Inf, Inf, 0));
        EngineAssert.Throws<FailureException>(() => HyperbolaBounds.Box(aHypr, -Inf, -Inf, 0));
        EngineAssert.Throws<FailureException>(() => Parabola2dBounds.Box(new Parabola2D(Turned2d, 2), -Inf, -Inf, 0));
        EngineAssert.Throws<FailureException>(() => Hyperbola2dBounds.Box(new Hyperbola2D(Turned2d, 3, 1), Inf, Inf, 0));
    }

    // z = x^2 + y^2 over a rectangle, through an adaptor of its own: a surface of no type the bounds know
    private sealed class Paraboloid(double theU1, double theU2, double theV1, double theV2) : SurfaceAdaptorBase
    {
        public override SurfaceAdaptorBase ShallowCopy() => new Paraboloid(theU1, theU2, theV1, theV2);
        public override double FirstUParameter() => theU1;
        public override double LastUParameter() => theU2;
        public override double FirstVParameter() => theV1;
        public override double LastVParameter() => theV2;
        public override Continuity UContinuity() => Continuity.CN;
        public override Continuity VContinuity() => Continuity.CN;
        public override int NbUIntervals(Continuity S) => 1;
        public override int NbVIntervals(Continuity S) => 1;
        public override void UIntervals(NCollection_Array1<double> T, Continuity S) { T[T.Lower()] = theU1; T[T.Lower() + 1] = theU2; }
        public override void VIntervals(NCollection_Array1<double> T, Continuity S) { T[T.Lower()] = theV1; T[T.Lower() + 1] = theV2; }
        public override SurfaceAdaptorBase UTrim(double First, double Last, double Tol) => new Paraboloid(First, Last, theV1, theV2);
        public override SurfaceAdaptorBase VTrim(double First, double Last, double Tol) => new Paraboloid(theU1, theU2, First, Last);
        public override bool IsUClosed => false;
        public override bool IsVClosed => false;
        public override bool IsUPeriodic => false;
        public override bool IsVPeriodic => false;
        public override double UResolution(double R3d) => R3d;
        public override double VResolution(double R3d) => R3d;
        public override SurfaceKind GetType() => SurfaceKind.OtherSurface;
        public override Point3D EvalD0(double u, double v) => new(u, v, (u * u) + (v * v));
        internal override Surface.ResD1 EvalD1(double u, double v) =>
            new() { Point = EvalD0(u, v), D1U = new Vector3D(1, 0, 2 * u), D1V = new Vector3D(0, 1, 2 * v) };
    }

    [Fact]
    public void A_surface_of_its_own_kind_is_boxed_by_its_samples_and_then_its_extremes()
    {
        // over [-0.7, 2] x [-1.3, 1]: x and y are the rectangle, z runs from 0 at the origin, between samples, to 5.69 at a corner
        var aSurf = new Paraboloid(-0.7, 2, -1.3, 1);
        var aBounds = new SurfaceBounds(aSurf);
        Assert.Equal(SurfaceKind.OtherSurface, aBounds.GetType());
        Box aSampled = aBounds.Box(0);
        Box anOptimal = aBounds.BoxOptimal(0);
        foreach (Box aBox in new[] { aSampled, anOptimal })
        {
            Assert.Equal(-0.7, aBox.GetXMin(), 6);
            Assert.Equal(2, aBox.GetXMax(), 6);
            Assert.Equal(-1.3, aBox.GetYMin(), 6);
            Assert.Equal(1, aBox.GetYMax(), 6);
            Assert.Equal(4 + 1.69, aBox.GetZMax(), 6);
        }
        // the optimal box finds the bottom of the bowl that the samples step over
        Assert.Equal(0, anOptimal.GetZMin(), 6);
        Assert.True(aSampled.GetZMin() >= anOptimal.GetZMin() - 1e-9);
        // a patch of it: [0.5, 1] x [0.5, 1] rises from 0.5 to 2
        Box aPatch = aBounds.BoxOptimal(0.5, 1, 0.5, 1, 0);
        Assert.Equal(0.5, aPatch.GetZMin(), 6);
        Assert.Equal(2, aPatch.GetZMax(), 6);
        Box aPatchSampled = aBounds.Box(0.5, 1, 0.5, 1, 0);
        Assert.Equal(0.5, aPatchSampled.GetZMin(), 6);
        Assert.Equal(2, aPatchSampled.GetZMax(), 6);
    }
}
