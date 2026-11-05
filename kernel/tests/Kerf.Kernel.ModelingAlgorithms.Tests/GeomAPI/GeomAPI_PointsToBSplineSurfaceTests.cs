// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomAPI_PointsToBSplineSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomAPI_PointsToBSplineSurface GTest.</summary>
public class GeomAPI_PointsToBSplineSurface_Test
{
    [Fact]
    public void FailedDegenerateRebuildResetsDoneState()
    {
        NCollection_Array2<double> aValidZPoints = new(1, 2, 1, 2);
        aValidZPoints[1, 1] = 0.0;
        aValidZPoints[1, 2] = 1.0;
        aValidZPoints[2, 1] = 1.0;
        aValidZPoints[2, 2] = 0.0;

        PointsToBSplineSurface anApprox = new();
        Assert.Null(Record.Exception(() =>
            anApprox.Init(aValidZPoints, 0.0, 1.0, 0.0, 1.0, 3, 5, Continuity.C2, global::Kerf.Engine.Foundation.Precision.Confusion())));
        Assert.True(anApprox.IsDone);

        NCollection_Array2<double> aZPoints = new(1, 3, 1, 1);
        aZPoints[1, 1] = 0.0;
        aZPoints[2, 1] = 1.0;
        aZPoints[3, 1] = 0.0;

        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() =>
            anApprox.Init(aZPoints, 0.0, 1.0, 0.0, 0.0, 3, 5, Continuity.C2, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.False(anApprox.IsDone);
    }
}

/// <summary>GeomAPI_PointsToBSplineSurface's methods on smooth grids (OCCT.NET's own tests).</summary>
public class GeomAPI_PointsToBSplineSurfaceTests
{
    private static double Height(double x, double y) => 0.3 * Math.Sin(x) + 0.2 * Math.Cos(1.5 * y) + 0.05 * x * y;

    private static NCollection_Array2<Point3D> Grid(int theNbU, int theNbV)
    {
        NCollection_Array2<Point3D> aPoints = new(1, theNbU, 1, theNbV);
        for (int i = 1; i <= theNbU; i++)
        {
            for (int j = 1; j <= theNbV; j++)
            {
                double x = 0.4 * (i - 1), y = 0.3 * (j - 1);
                aPoints[i, j] = new Point3D(x, y, Height(x, y));
            }
        }
        return aPoints;
    }

    [Fact]
    public void Interpolate_PassesThroughThePoints()
    {
        NCollection_Array2<Point3D> aPoints = Grid(7, 5);
        PointsToBSplineSurface anApprox = new();
        anApprox.Interpolate(aPoints);
        Assert.True(anApprox.IsDone);
        BSplineSurface aSurface = anApprox.Surface();
        Assert.True(aSurface.IsFrozen);
        Assert.Equal(3, aSurface.UDegree);
        Assert.Equal(3, aSurface.VDegree);
        Assert.Same(aSurface, (BSplineSurface)anApprox);

        // The corners are interpolated at the corners of the parameter domain
        aSurface.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.True(aSurface.Value(aU1, aV1).DistanceTo(aPoints[1, 1]) < 1e-9);
        Assert.True(aSurface.Value(aU2, aV2).DistanceTo(aPoints[7, 5]) < 1e-9);
        Assert.True(aSurface.Value(aU2, aV1).DistanceTo(aPoints[7, 1]) < 1e-9);
    }

    [Fact]
    public void Approximate_EveryMethodIsClose()
    {
        NCollection_Array2<Point3D> aPoints = Grid(9, 8);
        foreach (global::Kerf.Geometry.ParametrizationType aParType in new[] { global::Kerf.Geometry.ParametrizationType.ChordLength,
                                                                  global::Kerf.Geometry.ParametrizationType.Centripetal,
                                                                  global::Kerf.Geometry.ParametrizationType.IsoParametric })
        {
            PointsToBSplineSurface anApprox = new(aPoints, aParType, 3, 8, Continuity.C2, 1.0e-3);
            Assert.True(anApprox.IsDone);
            BSplineSurface aSurface = anApprox.Surface();
            aSurface.ParameterBounds(out double aU1, out _, out double aV1, out _);
            Assert.True(aSurface.Value(aU1, aV1).DistanceTo(aPoints[1, 1]) < 1e-2);
        }

        PointsToBSplineSurface aVariational = new(aPoints, 1.0, 0.0, 0.0, 8, Continuity.C2, 1.0e-3);
        Assert.True(aVariational.IsDone);
        Assert.True(aVariational.Surface().UPoleCount >= 4);
    }

    [Fact]
    public void Interpolate_Periodic()
    {
        // Rings of a torus-like tube: periodic in U (the first index)
        NCollection_Array2<Point3D> aPoints = new(1, 8, 1, 4);
        for (int i = 1; i <= 8; i++)
        {
            double a = 2 * Math.PI * (i - 1) / 8;
            for (int j = 1; j <= 4; j++)
            {
                aPoints[i, j] = new Point3D((3 + Math.Cos(a)) * 1.0, Math.Sin(a), j);
            }
        }
        PointsToBSplineSurface anApprox = new();
        anApprox.Interpolate(aPoints, true);
        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.Surface().IsUPeriodic);

        anApprox.Init(aPoints, global::Kerf.Geometry.ParametrizationType.ChordLength, 3, 8, Continuity.C2, 1.0e-3, true);
        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.Surface().IsUPeriodic);
    }

    [Fact]
    public void ZPoints_InterpolatesTheHeights()
    {
        NCollection_Array2<double> aZ = new(1, 6, 1, 5);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 5; j++)
            {
                aZ[i, j] = Height(0.5 * (i - 1), 0.25 * (j - 1));
            }
        }
        PointsToBSplineSurface anApprox = new();
        anApprox.Interpolate(aZ, 1.0, 0.5, -2.0, 0.25);
        Assert.True(anApprox.IsDone);
        BSplineSurface aSurface = anApprox.Surface();
        aSurface.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(1.0, aU1, 1e-12);
        Assert.Equal(3.5, aU2, 1e-12);
        Assert.Equal(-2.0, aV1, 1e-12);
        Assert.Equal(-1.0, aV2, 1e-12);
        Point3D aCorner = aSurface.Value(aU2, aV2);
        Assert.Equal(3.5, aCorner.X, 1e-9);
        Assert.Equal(-1.0, aCorner.Y, 1e-9);
        Assert.Equal(aZ[6, 5], aCorner.Z, 1e-9);

        PointsToBSplineSurface anApprox2 = new(aZ, 1.0, 0.5, -2.0, 0.25);
        Assert.True(anApprox2.IsDone);
    }

    [Fact]
    public void Surface_NotDoneThrows()
    {
        PointsToBSplineSurface anApprox = new();
        Assert.False(anApprox.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anApprox.Surface());
    }
}
