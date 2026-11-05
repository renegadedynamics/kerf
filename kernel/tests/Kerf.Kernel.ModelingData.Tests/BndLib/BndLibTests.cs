// Port of OCCT's src/ModelingData/TKGeomBase/GTests/BndLib_Test.cxx (V8_0_1).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// The BndLibTest suite (BndLib's elementary gp primitives). The BndLib_Add3dCurve, Add2dCurve and AddSurface suites
// of the same file are in their own classes.
public partial class BndLibTest
{
    private static readonly double THE_INF = double.PositiveInfinity;

    //=============================================================================================
    // Line Tests
    //=============================================================================================

    [Fact]
    public void Line_FiniteSegment()
    {
        // Line along X axis from (0,0,0) in direction (1,0,0)
        Line3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Box aBox = new();

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, 0.0, 10.0, 0.0, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line_PositiveDirection_NegativeInfinite()
    {
        // Line along +X axis with P1 = -Inf, P2 = 5
        // As t -> -Inf, x -> -Inf, so Xmin should be open
        Line3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Box aBox = new();

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, -THE_INF, 5.0, 0.0, ref aBox);

        Assert.True(aBox.IsOpenXmin, "Xmin should be open for line in +X direction with P1=-Inf");
        Assert.False(aBox.IsOpenXmax, "Xmax should not be open");
    }

    [Fact]
    public void Line_NegativeDirection_NegativeInfinite()
    {
        // Line along -X axis with P1 = -Inf, P2 = 5
        // Direction is (-1, 0, 0), so as t -> -Inf, x -> +Inf
        // Therefore Xmax should be open, not Xmin
        Line3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(-1.0, 0.0, 0.0));
        Box aBox = new();

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, -THE_INF, 5.0, 0.0, ref aBox);

        Assert.True(aBox.IsOpenXmax, "Xmax should be open for line in -X direction with P1=-Inf");
        Assert.False(aBox.IsOpenXmin, "Xmin should not be open");
    }

    [Fact]
    public void Line_NegativeDirection_PositiveInfinite()
    {
        // Line along -X axis with P1 = -5, P2 = +Inf
        // Direction is (-1, 0, 0), so as t -> +Inf, x -> -Inf
        // Therefore Xmin should be open
        Line3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(-1.0, 0.0, 0.0));
        Box aBox = new();

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, -5.0, THE_INF, 0.0, ref aBox);

        Assert.True(aBox.IsOpenXmin, "Xmin should be open for line in -X direction with P2=+Inf");
        Assert.False(aBox.IsOpenXmax, "Xmax should not be open");
    }

    [Fact]
    public void Line_DiagonalDirection_BothInfinite()
    {
        // Line in diagonal direction (-1, -1, -1) with P1=-Inf, P2=+Inf
        // As t -> -Inf: position goes to (+Inf, +Inf, +Inf) - opens max
        // As t -> +Inf: position goes to (-Inf, -Inf, -Inf) - opens min
        Line3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(-1.0, -1.0, -1.0));
        Box aBox = new();

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, -THE_INF, THE_INF, 0.0, ref aBox);

        Assert.True(aBox.IsOpenXmin, "Xmin should be open");
        Assert.True(aBox.IsOpenXmax, "Xmax should be open");
        Assert.True(aBox.IsOpenYmin, "Ymin should be open");
        Assert.True(aBox.IsOpenYmax, "Ymax should be open");
        Assert.True(aBox.IsOpenZmin, "Zmin should be open");
        Assert.True(aBox.IsOpenZmax, "Zmax should be open");
    }

    //=============================================================================================
    // Circle Tests
    //=============================================================================================

    [Fact]
    public void Circle_Full()
    {
        // Circle of radius 5 centered at origin in XY plane
        Circle3D aCirc = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Box aBoxFull = new();
        Box aBoxArc = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, aTol, ref aBoxFull);

        aBoxFull.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Full circle via parameters [0, 2*PI]
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, 0.0, 2.0 * M_PI, aTol, ref aBoxArc);
        aBoxArc.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        Assert.Equal(-5.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_Arc_FirstQuadrant()
    {
        // Circle arc from 0 to PI/2 (first quadrant)
        Circle3D aCirc = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_RotatedAxis()
    {
        // Circle in XZ plane (axis along Y)
        Circle3D aCirc = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0)), 3.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-3.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // Ellipse Tests
    //=============================================================================================

    [Fact]
    public void Ellipse_Full()
    {
        // Ellipse with major radius 10, minor radius 5 in XY plane
        Ellipse3D aElips = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 5.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aElips, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-10.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Ellipse_Arc()
    {
        // Ellipse arc from 0 to PI/2
        Ellipse3D aElips = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 5.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aElips, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // At t=0: (10, 0, 0), at t=PI/2: (0, 5, 0)
        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // Hyperbola Tests
    //=============================================================================================

    [Fact]
    public void Hyperbola_SimpleArc()
    {
        // Hyperbola with RealAxis=3, ImaginaryAxis=2 in XY plane
        // Parameterization: (3*cosh(t), 2*sinh(t), 0)
        Hyperbola3D aHypr = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0, 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aHypr, -1.0, 1.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // At t=-1: (3*cosh(-1), 2*sinh(-1)) = (4.629..., -2.350...)
        // At t=1:  (3*cosh(1), 2*sinh(1))   = (4.629...,  2.350...)
        // At t=0:  (3, 0) - minimum X
        double aExpectedXmin = 3.0;                   // at t=0
        double aExpectedXmax = 3.0 * Math.Cosh(1.0);  // at t=+-1
        double aExpectedYmin = 2.0 * Math.Sinh(-1.0); // at t=-1
        double aExpectedYmax = 2.0 * Math.Sinh(1.0);  // at t=1

        Assert.Equal(aExpectedXmin, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpectedXmax, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpectedYmin, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpectedYmax, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Hyperbola_Rotated_MultipleExtrema()
    {
        // Rotated hyperbola where X, Y, Z extrema occur at different parameter values.
        // This tests the fix for the extrema loop that previously broke after finding
        // the first extremum.
        Frame3D aAx2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
        Hyperbola3D aHypr = new(aAx2, 5.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aHypr, -2.0, 2.0, aTol, ref aBox);

        // Verify box is valid and contains reasonable bounds
        Assert.False(aBox.IsVoid, "Box should not be void");

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // For a rotated hyperbola, the box should be non-degenerate in all dimensions
        Assert.True(aXmin < aXmax, "X range should be non-zero");
        Assert.True(aYmin < aYmax, "Y range should be non-zero");
        // Z may be zero if hyperbola lies in a plane, but should still be valid
    }

    [Fact]
    public void Hyperbola_InfiniteParameter()
    {
        // Hyperbola with one infinite parameter
        Hyperbola3D aHypr = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0, 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aHypr, 0.0, THE_INF, aTol, ref aBox);

        Assert.True(aBox.IsOpenXmax, "Xmax should be open for hyperbola with P2=+Inf");
        Assert.True(aBox.IsOpenYmax, "Ymax should be open for hyperbola with P2=+Inf");
    }

    //=============================================================================================
    // Parabola Tests
    //=============================================================================================

    [Fact]
    public void Parabola_FiniteArc()
    {
        // Parabola with focal length 2 in XY plane
        // Parameterization: (t^2/(4*f), t, 0) = (t^2/8, t, 0) for f=2
        Parabola3D aParab = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aParab, -4.0, 4.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // At t=0: (0, 0, 0)
        // At t=+-4: (16/8, +-4, 0) = (2, +-4, 0)
        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parabola_InfiniteParameter()
    {
        // Parabola with infinite parameter
        Parabola3D aParab = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aParab, 0.0, THE_INF, aTol, ref aBox);

        Assert.True(aBox.IsOpenXmax, "Xmax should be open for parabola with P2=+Inf");
        Assert.True(aBox.IsOpenYmax, "Ymax should be open for parabola with P2=+Inf");
    }

    //=============================================================================================
    // Sphere Tests
    //=============================================================================================

    [Fact]
    public void Sphere_Full()
    {
        // Sphere of radius 7 centered at origin
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 7.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aSphere, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-7.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-7.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-7.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(7.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Sphere_Patch()
    {
        // Sphere patch: upper hemisphere (V from 0 to PI/2)
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from 0 to PI/2 (upper hemisphere)
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aSphere, 0.0, 2.0 * M_PI, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Sphere_Translated()
    {
        // Sphere centered at (10, 20, 30) with radius 3
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(10.0, 20.0, 30.0), new Direction3D(0.0, 0.0, 1.0)), 3.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aSphere, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(7.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(13.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(17.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(23.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(27.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(33.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // Cylinder Tests
    //=============================================================================================

    [Fact]
    public void Cylinder_FinitePatch()
    {
        // Cylinder of radius 4, axis along Z
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 4.0);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from 0 to 10
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCyl, 0.0, 2.0 * M_PI, 0.0, 10.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-4.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Cylinder_InfiniteV()
    {
        // Cylinder with infinite V parameter
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 4.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCyl, 0.0, THE_INF, aTol, ref aBox);

        Assert.True(aBox.IsOpenZmax, "Zmax should be open for cylinder with VMax=+Inf");
        Assert.False(aBox.IsOpenZmin, "Zmin should not be open");
    }

    [Fact]
    public void Cylinder_PartialU()
    {
        // Cylinder with partial U range (quarter cylinder)
        Cylinder3D aCyl = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Box aBox = new();
        double aTol = 0.0;

        // U from 0 to PI/2 (first quadrant), V from 0 to 10
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCyl, 0.0, M_PI / 2.0, 0.0, 10.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // Cone Tests
    //=============================================================================================

    [Fact]
    public void Cone_FinitePatch()
    {
        // Cone with semi-angle PI/4 (45 deg), reference radius 2
        // Cone parameterization: P(u,v) = Apex + (RefRadius + v*sin(SemiAngle)) * (cos(u)*XDir +
        // sin(u)*YDir) + v*cos(SemiAngle) * Axis
        Cone3D aCone = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), M_PI / 4.0, 2.0);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from 0 to 5
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCone, 0.0, 2.0 * M_PI, 0.0, 5.0, aTol, ref aBox);

        aBox.Get(out _, out _, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        // V=0: radius = RefRadius = 2, Z = 0
        // V=5: radius = 2 + 5*sin(PI/4) = 2 + 5*sqrt(2)/2, Z = 5*cos(PI/4) = 5*sqrt(2)/2
        double aMaxRadius = 2.0 + 5.0 * Math.Sin(M_PI / 4.0);
        double aMaxZ = 5.0 * Math.Cos(M_PI / 4.0);
        Assert.Equal(aMaxRadius, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aMaxRadius, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aMaxZ, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Cone_NegativeV()
    {
        // Cone with negative V range (cone extending in opposite direction)
        // Cone parameterization: Z = v*cos(SemiAngle)
        Cone3D aCone = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), M_PI / 6.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from -5 to 0
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCone, 0.0, 2.0 * M_PI, -5.0, 0.0, aTol, ref aBox);

        aBox.Get(out _, out _, out double aZmin, out _, out _, out double aZmax);

        // At V=0: radius = 3, Z = 0
        // At V=-5: radius = 3 + (-5)*sin(PI/6) = 3 - 2.5 = 0.5, Z = -5*cos(PI/6)
        double aExpectedZmin = -5.0 * Math.Cos(M_PI / 6.0);
        Assert.Equal(aExpectedZmin, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Cone_InfiniteV()
    {
        // Cone with infinite V parameter
        // Note: BndLib::Add for cone with U,V parameters opens Z but not X/Y
        // since the cone's bounding in X/Y is computed from the circular cross-sections
        Cone3D aCone = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), M_PI / 4.0, 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCone, 0.0, THE_INF, aTol, ref aBox);

        // For cone with infinite V, the Z direction should be open
        Assert.True(aBox.IsOpenZmax, "Zmax should be open for cone with VMax=+Inf");
    }

    //=============================================================================================
    // Torus Tests
    //=============================================================================================

    [Fact]
    public void Torus_Full()
    {
        // Torus with major radius 10, minor radius 3
        Torus3D aTorus = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aTorus, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        // Torus extends from -(R+r) to +(R+r) in X and Y
        // and from -r to +r in Z
        Assert.Equal(-13.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(13.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-13.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(13.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-3.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Torus_Patch()
    {
        // Torus patch: quarter in U, half in V
        // Torus parameterization: P(u,v) = Center + (R + r*cos(v)) * (cos(u)*XDir + sin(u)*YDir)
        //                                         + r*sin(v) * Axis
        Torus3D aTorus = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        // U from 0 to PI/2, V from 0 to PI
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aTorus, 0.0, M_PI / 2.0, 0.0, M_PI, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out _, out _, out double aZmax);

        // Box should be valid and contain reasonable bounds
        Assert.False(aBox.IsVoid, "Box should not be void for torus patch");

        // For U in [0, PI/2], X and Y should be non-negative at minimum
        // The bounding box algorithm produces conservative bounds that may slightly
        // exceed R+r due to sampling/approximation
        Assert.True(aXmin >= -global::Kerf.Engine.Foundation.Precision.Confusion(), "Xmin should be >= 0 for first quadrant");
        Assert.True(aYmin >= -global::Kerf.Engine.Foundation.Precision.Confusion(), "Ymin should be >= 0 for first quadrant");

        // Z max should be approximately r (minor radius)
        Assert.True(aZmax <= 3.0 + global::Kerf.Engine.Foundation.Precision.Confusion(), "Zmax should not exceed r");
    }

    [Fact]
    public void Torus_NegativeVParameter()
    {
        // Torus patch with negative V parameters.
        // This tests the fix for negative modulo: ((i % 8) + 8) % 8
        // The main purpose is to verify no crash occurs with negative V.
        Torus3D aTorus = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        // Full U, V from -PI to 0
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aTorus, 0.0, 2.0 * M_PI, -M_PI, 0.0, aTol, ref aBox);

        aBox.Get(out _, out _, out double aZmin, out _, out _, out _);

        // Should produce valid bounds without crash (main test for the fix)
        Assert.False(aBox.IsVoid, "Box should not be void for negative V parameters");

        // For V in [-PI, 0], Z = r*sin(v) ranges from -r (at v=-PI/2) to 0
        // The bounding box should contain this range
        Assert.True(aZmin <= 0.0, "Zmin should be <= 0 for V in [-PI, 0]");
        Assert.True(aZmin >= -3.0 - global::Kerf.Engine.Foundation.Precision.Confusion(), "Zmin should be >= -r");
    }

    [Fact]
    public void Torus_LargeNegativeVParameter()
    {
        // Torus with large negative V parameter that would cause
        // out-of-bounds array access with incorrect modulo.
        // This is the primary test for the negative modulo fix.
        Torus3D aTorus = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 10.0, 3.0);
        Box aBox = new();
        double aTol = 0.0;

        // V from -3*PI to -2*PI covers one full revolution
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aTorus, 0.0, 2.0 * M_PI, -3.0 * M_PI, -2.0 * M_PI, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // Should produce valid bounds without crash (main test for the fix)
        Assert.False(aBox.IsVoid, "Box should not be void for large negative V parameters");

        // Full revolution in U gives full X/Y extent, the box may be slightly larger
        // due to conservative bounding box algorithm
        Assert.True(aXmin <= -13.0 + global::Kerf.Engine.Foundation.Precision.Confusion(), "Xmin should be at least -(R+r)");
        Assert.True(aXmax >= 13.0 - global::Kerf.Engine.Foundation.Precision.Confusion(), "Xmax should be at least R+r");
        Assert.True(aYmin <= -13.0 + global::Kerf.Engine.Foundation.Precision.Confusion(), "Ymin should be at least -(R+r)");
        Assert.True(aYmax >= 13.0 - global::Kerf.Engine.Foundation.Precision.Confusion(), "Ymax should be at least R+r");
    }

    [Fact]
    public void Torus_Translated()
    {
        // Torus centered at (5, 5, 5)
        Torus3D aTorus = new(new CoordinateSystem(new Point3D(5.0, 5.0, 5.0), new Direction3D(0.0, 0.0, 1.0)), 8.0, 2.0);
        Box aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aTorus, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(5.0 - 10.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0 + 10.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0 - 10.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0 + 10.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0 - 2.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0 + 2.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // 2D Tests
    //=============================================================================================

    [Fact]
    public void Line2d_FiniteSegment()
    {
        Line2D aLine = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 1.0));
        Box2d aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aLine, 0.0, 10.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        // Line in direction (1,1) normalized, so at t=10:
        // position = (10/sqrt(2), 10/sqrt(2))
        double aExpected = 10.0 / Math.Sqrt(2.0);
        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpected, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpected, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle2d_Full()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        Box2d aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(-5.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Ellipse2d_Full()
    {
        Ellipse2D aElips = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 8.0, 4.0);
        Box2d aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aElips, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(-8.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(8.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Hyperbola2d_SimpleArc()
    {
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 3.0, 2.0);
        Box2d aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aHypr, -1.0, 1.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(3.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0 * Math.Cosh(1.0), aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0 * Math.Sinh(-1.0), aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0 * Math.Sinh(1.0), aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parabola2d_FiniteArc()
    {
        Parabola2D aParab = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0);
        Box2d aBox = new();
        double aTol = 0.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aParab, -4.0, 4.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);

        Assert.Equal(0.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    //=============================================================================================
    // Tolerance Tests
    //=============================================================================================

    [Fact]
    public void Circle_WithTolerance()
    {
        Circle3D aCirc = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        Box aBox = new();
        double aTol = 0.5;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aCirc, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        // Box should be enlarged by tolerance
        Assert.Equal(-5.5, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.5, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-5.5, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.5, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-0.5, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.5, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Sphere_WithTolerance()
    {
        Sphere3D aSphere = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0);
        Box aBox = new();
        double aTol = 1.0;

        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(aSphere, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-4.0, aXmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aXmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aYmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aYmax, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-4.0, aZmin, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.0, aZmax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
