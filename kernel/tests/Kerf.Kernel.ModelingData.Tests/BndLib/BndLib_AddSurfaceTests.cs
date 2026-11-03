// Port of OCCT's src/ModelingData/TKGeomBase/GTests/BndLib_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// The BndLib_AddSurfaceTest suite of BndLib_Test.cxx.
public class BndLib_AddSurfaceTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static CoordinateSystem Ax3(double theX, double theY, double theZ, double theDX, double theDY, double theDZ)
        => new(new Point3D(theX, theY, theZ), new Direction3D(theDX, theDY, theDZ));

    private static void ExpectGE(double theA, double theB, string theWhat = "") => GeomBndLibSurfaceTestUtils.ExpectGE(theA, theB, theWhat);

    private static void ExpectLE(double theA, double theB, string theWhat = "") => GeomBndLibSurfaceTestUtils.ExpectLE(theA, theB, theWhat);

    [Fact]
    public void Plane()
    {
        // Create a plane and add a finite patch
        PlaneSurface aPlane = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
        SurfaceAdaptor aSurf = new(aPlane);
        Box aBox = new();
        double aTol = 0.0;

        // Add a 10x10 patch centered at origin
        AddSurface.Add(aSurf, -5.0, 5.0, -5.0, 5.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Cylinder_Full()
    {
        // Create a cylindrical surface
        CylindricalSurface aCyl = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 5.0);
        SurfaceAdaptor aSurf = new(aCyl);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from 0 to 10
        AddSurface.Add(aSurf, 0.0, 2.0 * M_PI, 0.0, 10.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(10.0, aZmax, Conf);
    }

    [Fact]
    public void Sphere_Full()
    {
        // Create a spherical surface
        SphericalSurface aSphere = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 7.0);
        SurfaceAdaptor aSurf = new(aSphere);
        Box aBox = new();
        double aTol = 0.0;

        AddSurface.Add(aSurf, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-7.0, aXmin, Conf);
        Assert.Equal(7.0, aXmax, Conf);
        Assert.Equal(-7.0, aYmin, Conf);
        Assert.Equal(7.0, aYmax, Conf);
        Assert.Equal(-7.0, aZmin, Conf);
        Assert.Equal(7.0, aZmax, Conf);
    }

    [Fact]
    public void Sphere_Patch()
    {
        // Create a spherical surface patch (upper hemisphere)
        SphericalSurface aSphere = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 5.0);
        SurfaceAdaptor aSurf = new(aSphere);
        Box aBox = new();
        double aTol = 0.0;

        // Full U, V from 0 to PI/2 (upper hemisphere)
        AddSurface.Add(aSurf, 0.0, 2.0 * M_PI, 0.0, M_PI / 2.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(-5.0, aYmin, Conf);
        Assert.Equal(5.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(5.0, aZmax, Conf);
    }

    [Fact]
    public void Cone_Full()
    {
        // Create a conical surface
        ConicalSurface aCone = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), M_PI / 4.0, 2.0);
        SurfaceAdaptor aSurf = new(aCone);
        Box aBox = new();
        double aTol = 0.0;

        // Full U range, V from 0 to 5
        AddSurface.Add(aSurf, 0.0, 2.0 * M_PI, 0.0, 5.0, aTol, ref aBox);

        aBox.Get(out _, out _, out _, out double aXmax, out double aYmax, out double aZmax);

        // V=5: radius = 2 + 5*sin(PI/4), Z = 5*cos(PI/4)
        double aMaxRadius = 2.0 + 5.0 * Math.Sin(M_PI / 4.0);
        double aMaxZ = 5.0 * Math.Cos(M_PI / 4.0);
        Assert.Equal(aMaxRadius, aXmax, Conf);
        Assert.Equal(aMaxRadius, aYmax, Conf);
        Assert.Equal(aMaxZ, aZmax, Conf);
    }

    [Fact]
    public void Torus_Full()
    {
        // Create a toroidal surface
        ToroidalSurface aTorus = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 10.0, 3.0);
        SurfaceAdaptor aSurf = new(aTorus);
        Box aBox = new();
        double aTol = 0.0;

        AddSurface.Add(aSurf, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        // BndLib_AddSurface uses conservative bounding for torus
        // The exact bounds are +-13 for X,Y and +-3 for Z
        // The algorithm may produce slightly larger bounds
        ExpectLE(aXmin, -13.0 + Conf);
        ExpectGE(aXmax, 13.0 - Conf);
        ExpectLE(aYmin, -13.0 + Conf);
        ExpectGE(aYmax, 13.0 - Conf);
        Assert.Equal(-3.0, aZmin, Conf);
        Assert.Equal(3.0, aZmax, Conf);
    }

    [Fact]
    public void BezierSurface()
    {
        // Create a simple Bezier surface (2x2 control points)
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(0.0, 10.0, 2.0);
        aPoles[2, 1] = new Point3D(10.0, 0.0, 2.0);
        aPoles[2, 2] = new Point3D(10.0, 10.0, 0.0);

        BezierSurface aBezier = new(aPoles);
        SurfaceAdaptor aSurf = new(aBezier);
        Box aBox = new();
        double aTol = 0.0;

        AddSurface.Add(aSurf, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        // Box should contain all control points
        ExpectLE(aXmin, 0.0);
        ExpectGE(aXmax, 10.0);
        ExpectLE(aYmin, 0.0);
        ExpectGE(aYmax, 10.0);
        ExpectLE(aZmin, 0.0);
        ExpectGE(aZmax, 2.0);
    }

    [Fact]
    public void AddOptimal_Sphere()
    {
        // Test AddOptimal method with a sphere
        SphericalSurface aSphere = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 4.0);
        SurfaceAdaptor aSurf = new(aSphere);
        Box aBox = new();
        double aTol = 0.0;

        AddSurface.AddOptimal(aSurf, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(-4.0, aXmin, Conf);
        Assert.Equal(4.0, aXmax, Conf);
        Assert.Equal(-4.0, aYmin, Conf);
        Assert.Equal(4.0, aYmax, Conf);
        Assert.Equal(-4.0, aZmin, Conf);
        Assert.Equal(4.0, aZmax, Conf);
    }

    [Fact]
    public void Cylinder_Translated()
    {
        // Create a cylindrical surface centered at (5, 5, 0)
        CylindricalSurface aCyl = new(Ax3(5.0, 5.0, 0.0, 0.0, 0.0, 1.0), 3.0);
        SurfaceAdaptor aSurf = new(aCyl);
        Box aBox = new();
        double aTol = 0.0;

        AddSurface.Add(aSurf, 0.0, 2.0 * M_PI, 0.0, 10.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);

        Assert.Equal(2.0, aXmin, Conf);
        Assert.Equal(8.0, aXmax, Conf);
        Assert.Equal(2.0, aYmin, Conf);
        Assert.Equal(8.0, aYmax, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(10.0, aZmax, Conf);
    }

    [Fact]
    public void Plane_Tilted()
    {
        // Create a tilted plane (normal at 45 degrees)
        // gp_Dir automatically normalizes the input vector
        PlaneSurface aPlane = new(Ax3(0.0, 0.0, 0.0, 0.0, 1.0, 1.0));
        SurfaceAdaptor aSurf = new(aPlane);
        Box aBox = new();
        double aTol = 0.0;

        // Add a unit square patch
        AddSurface.Add(aSurf, 0.0, 1.0, 0.0, 1.0, aTol, ref aBox);

        Assert.False(aBox.IsVoid, "Box should not be void for tilted plane");
    }

    // Test for edge case with large parameter values and small step sizes.
    // This verifies that the algorithm correctly samples different points
    // even when parameter values are large relative to the step size,
    // which could cause floating-point precision issues if accumulation
    // (U += dU) is used instead of multiplication (U = UMin + dU * i).
    [Fact]
    public void LargeParameters_PrecisionTest()
    {
        // Create a Bezier surface - uses the default sampling path
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);

        // Create a simple surface with known extent
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D((i - 1) * 10.0, (j - 1) * 10.0, 0.0);
            }
        }

        BezierSurface aBezier = new(aPoles);
        SurfaceAdaptor aSurf = new(aBezier);
        Box aBox = new();
        double aTol = 0.0;

        // Use large parameter offset with small parameter range
        // This simulates the edge case where UMin/UMax are large but (UMax-UMin) is small
        // The actual Bezier is defined on [0,1] x [0,1], but we test a sub-range
        // to ensure the sampling algorithm works correctly
        AddSurface.Add(aSurf, 0.0, 1.0, 0.0, 1.0, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // The surface spans [0,20] x [0,20] x [0,0]
        // Bounding box should contain all sample points
        ExpectLE(aXmin, 1.0, "XMin should capture sampled points");
        ExpectGE(aXmax, 19.0, "XMax should capture sampled points");
        ExpectLE(aYmin, 1.0, "YMin should capture sampled points");
        ExpectGE(aYmax, 19.0, "YMax should capture sampled points");
    }

    // Test with translated surface where parameters are offset
    // This is a more realistic test of the large parameter scenario
    [Fact]
    public void OffsetSurface_ParameterPrecision()
    {
        // Create a simple plane
        PlaneSurface aPlane = new(Ax3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
        SurfaceAdaptor aSurf = new(aPlane);
        Box aBox = new();
        double aTol = 0.0;

        // Test with very large parameter values but small range
        // UMin = 1e10, UMax = 1e10 + 100, so dU ~ 100/(Nu-1) ~ small relative to 1e10
        double aLargeOffset = 1.0e10;
        double aRange = 100.0;

        AddSurface.Add(aSurf, aLargeOffset, aLargeOffset + aRange, aLargeOffset, aLargeOffset + aRange, aTol, ref aBox);

        aBox.Get(out double aXmin, out double aYmin, out _, out double aXmax, out double aYmax, out _);

        // For a plane at z=0 with parameters as X,Y coordinates:
        // The box should span approximately [1e10, 1e10+100] in both X and Y
        // Allow some tolerance due to floating-point representation
        double aTolerance = 1.0; // 1 unit tolerance for large values
        Assert.Equal(aLargeOffset, aXmin, aTolerance);
        Assert.Equal(aLargeOffset + aRange, aXmax, aTolerance);
        Assert.Equal(aLargeOffset, aYmin, aTolerance);
        Assert.Equal(aLargeOffset + aRange, aYmax, aTolerance);

        // Most importantly: verify the range is captured (not collapsed due to precision loss)
        double aComputedRangeX = aXmax - aXmin;
        double aComputedRangeY = aYmax - aYmin;
        Assert.True(aComputedRangeX > aRange * 0.9, "X range should not collapse due to floating-point precision issues");
        Assert.True(aComputedRangeY > aRange * 0.9, "Y range should not collapse due to floating-point precision issues");
    }
}
