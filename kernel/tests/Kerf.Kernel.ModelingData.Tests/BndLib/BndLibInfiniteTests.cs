// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// BndLib where a parameter is infinite, for each conic and surface in each order of its bounds: the side towards an
/// infinite parameter opens and a finite end is in the box (BndLib.cxx); two bounds at the same infinity are refused. Also
/// the 2D arcs and the degenerate torus. Unit 11I: BndLibTests has the finite and commonest infinite forms.
/// </summary>
public class BndLibInfiniteTests
{
    private const double Inf = 2e100;
    private const double Tol = 1e-7;

    private static readonly Frame3D XY = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
    private static readonly Frame2D XY2 = new(new Point2D(0, 0), new Direction2D(1, 0), new Direction2D(0, 1));

    /// <summary>The bounds, with which sides of the box they open: towards -Inf the mins, towards +Inf the maxes.</summary>
    public static TheoryData<double, double, bool, bool> Bounds => new()
    {
        { -Inf, Inf, true, true },
        { -Inf, 1, true, false },
        { Inf, -Inf, true, true },
        { Inf, 1, false, true },
        { 1, -Inf, true, false },
        { 1, Inf, false, true },
    };

    private static void Opens(Box theBox, bool theMins, bool theMaxes)
    {
        Assert.Equal(theMins, theBox.IsOpenXmin);
        Assert.Equal(theMins, theBox.IsOpenYmin);
        Assert.Equal(theMins, theBox.IsOpenZmin);
        Assert.Equal(theMaxes, theBox.IsOpenXmax);
        Assert.Equal(theMaxes, theBox.IsOpenYmax);
        Assert.Equal(theMaxes, theBox.IsOpenZmax);
    }

    private static void Opens(Box2d theBox, bool theMins, bool theMaxes)
    {
        Assert.Equal(theMins, theBox.IsOpenXmin);
        Assert.Equal(theMins, theBox.IsOpenYmin);
        Assert.Equal(theMaxes, theBox.IsOpenXmax);
        Assert.Equal(theMaxes, theBox.IsOpenYmax);
    }

    [Theory]
    [MemberData(nameof(Bounds))]
    public void A_parabola_opens_towards_its_infinite_bounds(double theP1, double theP2, bool theMins, bool theMaxes)
    {
        Box aBox = new();
        BndLib.Add(new Parabola3D(XY, 2), theP1, theP2, 0, ref aBox);
        Opens(aBox, theMins, theMaxes);

        Box2d aBox2d = new();
        BndLib.Add(new Parabola2D(XY2, 2), theP1, theP2, 0, ref aBox2d);
        Opens(aBox2d, theMins, theMaxes);
    }

    [Theory]
    [MemberData(nameof(Bounds))]
    public void A_hyperbola_opens_towards_its_infinite_bounds(double theP1, double theP2, bool theMins, bool theMaxes)
    {
        Box aBox = new();
        BndLib.Add(new Hyperbola3D(XY, 3, 2), theP1, theP2, 0, ref aBox);
        Opens(aBox, theMins, theMaxes);

        Box2d aBox2d = new();
        BndLib.Add(new Hyperbola2D(XY2, 3, 2), theP1, theP2, 0, ref aBox2d);
        Opens(aBox2d, theMins, theMaxes);
    }

    [Theory]
    [InlineData(Inf)]
    [InlineData(-Inf)]
    public void Two_bounds_at_the_same_infinity_are_refused(double theP)
    {
        Box aBox = new();
        Box2d aBox2d = new();
        CylinderAndCone(out Cylinder3D aCylinder, out Cone3D aCone);
        EngineAssert.Throws<FailureException>(() => { Box b = new(); BndLib.Add(new Parabola3D(XY, 2), theP, theP, 0, ref b); });
        EngineAssert.Throws<FailureException>(() => { Box2d b = new(); BndLib.Add(new Parabola2D(XY2, 2), theP, theP, 0, ref b); });
        EngineAssert.Throws<FailureException>(() => { Box b = new(); BndLib.Add(new Hyperbola3D(XY, 3, 2), theP, theP, 0, ref b); });
        EngineAssert.Throws<FailureException>(() => { Box2d b = new(); BndLib.Add(new Hyperbola2D(XY2, 3, 2), theP, theP, 0, ref b); });
        EngineAssert.Throws<FailureException>(() => { Box b = new(); BndLib.Add(aCylinder, 0, 1, theP, theP, 0, ref b); });
        EngineAssert.Throws<FailureException>(() => { Box b = new(); BndLib.Add(aCone, 0, 1, theP, theP, 0, ref b); });
        Assert.True(aBox.IsVoid);
        Assert.True(aBox2d.IsVoid);
    }

    private static void CylinderAndCone(out Cylinder3D theCylinder, out Cone3D theCone)
    {
        theCylinder = new Cylinder3D(new CoordinateSystem(XY), 2);
        theCone = new Cone3D(new CoordinateSystem(XY), 0.25, 2);
    }

    [Theory]
    [MemberData(nameof(Bounds))]
    public void A_cylinder_opens_along_its_axis_only(double theV1, double theV2, bool theMins, bool theMaxes)
    {
        CylinderAndCone(out Cylinder3D aCylinder, out _);
        Box aBox = new();
        BndLib.Add(aCylinder, 0, 2 * Math.PI, theV1, theV2, 0, ref aBox);

        Assert.Equal(theMins, aBox.IsOpenZmin);
        Assert.Equal(theMaxes, aBox.IsOpenZmax);
        Assert.False(aBox.IsOpenXmin || aBox.IsOpenXmax || aBox.IsOpenYmin || aBox.IsOpenYmax);
        // with a finite bound the cross-section is in the box. With neither, nothing's added: occt-issues.md, unit 11I
        if (!(theMins && theMaxes))
        {
            aBox.Get(out double aXMin, out _, out _, out double aXMax, out _, out _);
            Assert.Equal(-2.0, aXMin, Tol);
            Assert.Equal(2.0, aXMax, Tol);
        }
    }

    [Theory]
    [MemberData(nameof(Bounds))]
    public void A_cone_opens_along_its_axis(double theV1, double theV2, bool theMins, bool theMaxes)
    {
        CylinderAndCone(out _, out Cone3D aCone);
        Box aBox = new();
        BndLib.Add(aCone, 0, 2 * Math.PI, theV1, theV2, 0, ref aBox);

        // it widens without limit too, which OCCT's box doesn't show: not asserted here, see occt-issues.md, unit 11I
        Assert.Equal(theMins, aBox.IsOpenZmin);
        Assert.Equal(theMaxes, aBox.IsOpenZmax);
    }

    [Fact]
    public void Arcs_of_2d_conics_hold_their_arcs()
    {
        // BndLib's 2D arcs are bounded by the tangents at the ends and the middle of steps of at most a quarter turn or so,
        // so they're not tight: a quarter of radius 5 reaches 5 / cos(pi / 8) at most
        double aLoose = 5 / Math.Cos(Math.PI / 8) + Tol;
        Box2d aCircle = new();
        BndLib.Add(new Circle2D(XY2, 5), 0, Math.PI / 2, 0, ref aCircle);
        aCircle.Get(out double aX1, out double aY1, out double aX2, out double aY2);
        Assert.InRange(aX1, -aLoose + 5, Tol);
        Assert.InRange(aY1, -aLoose + 5, Tol);
        Assert.InRange(aX2, 5 - Tol, aLoose);
        Assert.InRange(aY2, 5 - Tol, aLoose);

        // the arc from a quarter to three quarters reaches the far side, and stays left of its ends' line
        Box2d anEllipse = new();
        BndLib.Add(new Ellipse2D(XY2, 3, 2), Math.PI / 2, 3 * Math.PI / 2, 0, ref anEllipse);
        anEllipse.Get(out aX1, out aY1, out aX2, out aY2);
        Assert.InRange(aX1, -3.5, -3 + Tol);
        Assert.InRange(aX2, -Tol, 0.5);
        Assert.InRange(aY1, -2.5, -2 + Tol);
        Assert.InRange(aY2, 2 - Tol, 2.5);

        // a whole 2D ellipse is exact, with the tolerance
        Box2d aWhole = new();
        BndLib.Add(new Ellipse2D(XY2, 3, 2), 0.5, ref aWhole);
        aWhole.Get(out aX1, out aY1, out aX2, out aY2);
        Assert.Equal(-3.5, aX1, Tol);
        Assert.Equal(2.5, aY2, Tol);

        // bounds given backwards are the same arc; more than a turn apart, the whole circle
        Box2d aBackwards = new();
        BndLib.Add(new Circle2D(XY2, 5), Math.PI / 2, 0, 0, ref aBackwards);
        Assert.Equal(aCircle.Get(), aBackwards.Get());
        Box2d aTurnAndMore = new();
        BndLib.Add(new Circle2D(XY2, 5), 0, 7, 0, ref aTurnAndMore);
        aTurnAndMore.Get(out aX1, out aY1, out aX2, out aY2);
        Assert.InRange(aX1, -aLoose, -5 + Tol);
        Assert.InRange(aX2, 5 - Tol, aLoose);
    }

    [Fact]
    public void A_degenerate_torus_is_bounded_by_its_radii()
    {
        // a major radius below the minor: the tube self-intersects, reaching ra + ri out and ri up
        Torus3D aTorus = new(new CoordinateSystem(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), 1, 3);
        Box aWhole = new();
        BndLib.Add(aTorus, 0, ref aWhole);
        aWhole.Get(out double aX1, out double aY1, out double aZ1, out double aX2, out double aY2, out double aZ2);
        Assert.Equal(1 - 4.0, aX1, Tol);
        Assert.Equal(1 + 4.0, aX2, Tol);
        Assert.Equal(2 - 4.0, aY1, Tol);
        Assert.Equal(2 + 4.0, aY2, Tol);
        Assert.Equal(3 - 3.0, aZ1, Tol);
        Assert.Equal(3 + 3.0, aZ2, Tol);

        // the patch from u = 0 to pi/2 and v = 0 to acos(-1/3), within which v = 0 reaches ra + ri along x and y
        Box aPatch = new();
        double aPhi = Math.Acos(-1.0 / 3.0);
        BndLib.Add(aTorus, 0, Math.PI / 2, 0, aPhi, 0, ref aPatch);
        aPatch.Get(out aX1, out aY1, out aZ1, out aX2, out aY2, out aZ2);
        Assert.True(aX2 >= 1 + 4.0 - Tol);
        Assert.True(aY2 >= 2 + 4.0 - Tol);
        Assert.True(aZ2 >= 3 + 3.0 - Tol);
        Assert.True(aZ1 >= 3 - Tol);
        // and nothing of the patch has gone below the centre on either axis by more than the tube's return to the axis
        Assert.True(aX1 >= 1 - 1e-6);
        Assert.True(aY1 >= 2 - 1e-6);
    }
}
