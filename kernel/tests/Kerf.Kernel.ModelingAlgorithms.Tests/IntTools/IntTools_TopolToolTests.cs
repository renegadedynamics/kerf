// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Tools;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>IntTools_TopolTool's sample grids.</summary>
public class IntTools_TopolToolTests
{
    [Fact]
    public void Plane_TenByTenInsideTheRange()
    {
        var aS = new SurfaceAdaptor(new PlaneSurface(new Plane3D()), 0, 11, 0, 22);
        var aTool = new IntersectionTopolTool(aS);
        Assert.Equal(10, aTool.NbSamplesU());
        Assert.Equal(10, aTool.NbSamplesV());
        Assert.Equal(100, aTool.NbSamples());
        // iv = 1 + Index / NbU, iu = 1 + Index - (iv - 1) NbU; steps (sup - inf) / (n + 1)
        aTool.SamplePoint(1, out Point2D aP2d, out Point3D aP3d);
        Assert.Equal(2.0, aP2d.X, 1e-15);
        Assert.Equal(2.0, aP2d.Y, 1e-15);
        Assert.Equal(aP3d.X, aP2d.X);
        aTool.SamplePoint(23, out aP2d, out _);
        Assert.Equal(4.0, aP2d.X, 1e-15);
        Assert.Equal(6.0, aP2d.Y, 1e-15);
    }

    [Fact]
    public void Cylinder_SamplesFromTheDeflection()
    {
        var aS = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(), 10.0), 0, 2 * M_PI, 0, 100);
        var aTool = new IntersectionTopolTool(aS);
        double aMaxAngle = Math.Acos(1.0 - 1e-2 / 10.0) * 2.0;
        Assert.Equal(Math.Min(50, (int)(2 * M_PI / aMaxAngle)), aTool.NbSamplesU());
        Assert.Equal(10, aTool.NbSamplesV());
    }

    [Fact]
    public void Sphere_AtLeastTen_AtMostFifty()
    {
        var aS = new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(), 0.001));
        var aTool = new IntersectionTopolTool(aS);
        Assert.Equal(10, aTool.NbSamplesU());
        Assert.Equal(10, aTool.NbSamplesV());
        var aBig = new IntersectionTopolTool(new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(), 1000.0)));
        Assert.Equal(50, aBig.NbSamplesU());
        Assert.Equal(50, aBig.NbSamplesV());
    }

    [Fact]
    public void InfiniteRanges_AreBounded()
    {
        var aTool = new IntersectionTopolTool(new SurfaceAdaptor(new PlaneSurface(new Plane3D())));
        aTool.SamplePoint(1, out Point2D aP2d, out _);
        // As OCCT, index 1 is the second sample in U: the grid is read from index 0 (occt-issues.md)
        double aDU = 2.0e5 / 11;
        Assert.Equal(-1.0e5 + 2 * aDU, aP2d.X, 1e-9);
    }

    [Fact]
    public void BSpline_AnalysesThePoles()
    {
        // A wavy 12 x 4 pole grid: 12 knots x degree 1 in U, more than 10, so Analyse counts the sign changes
        var aPoles = new NCollection_Array2<Point3D>(1, 12, 1, 4);
        for (int i = 1; i <= 12; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, (i % 2) * 0.5);
            }
        }
        var aUK = new NCollection_Array1<double>(1, 12);
        var aUM = new NCollection_Array1<int>(1, 12);
        for (int i = 1; i <= 12; i++)
        {
            aUK[i] = i;
            aUM[i] = 1;
        }
        aUM[1] = aUM[12] = 2;
        var aVK = new NCollection_Array1<double>(1, 2);
        var aVM = new NCollection_Array1<int>(1, 2);
        aVK[1] = 0;
        aVK[2] = 1;
        aVM[1] = aVM[2] = 4;
        var aSurf = new BSplineSurface(aPoles, aUK, aVK, aUM, aVM, 1, 3);
        var aTool = new IntersectionTopolTool(new SurfaceAdaptor(aSurf));
        Assert.True(aTool.NbSamplesU() >= 10);
        Assert.True(aTool.NbSamplesV() >= 10);
        Assert.Equal(aTool.NbSamplesU() * aTool.NbSamplesV(), aTool.NbSamples());
    }

    [Fact]
    public void Initialize_WithoutSurfaceThrows()
    {
        var aTool = new IntersectionTopolTool();
        EngineAssert.ThrowsAny<MethodNotImplementedException>(() => aTool.Initialize());
    }
}
