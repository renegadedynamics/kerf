// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntPolyh_Point_Test.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntPolyh_Intersection_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Polyhedra;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntPolyh_Point GTest.</summary>
public class IntPolyh_PointTest
{
    [Fact]
    public void DefaultConstructor_AllZero()
    {
        PolyhedronPoint aPt = new();
        Assert.Equal(0.0, aPt.X());
        Assert.Equal(0.0, aPt.Y());
        Assert.Equal(0.0, aPt.Z());
        Assert.Equal(0.0, aPt.U());
        Assert.Equal(0.0, aPt.V());
    }

    [Fact]
    public void Divide_NormalDivisor_CorrectResult()
    {
        PolyhedronPoint aPt = new(10.0, 20.0, 30.0, 0.5, 0.8);
        PolyhedronPoint aRes = aPt.Divide(2.0);

        Assert.Equal(5.0, aRes.X());
        Assert.Equal(10.0, aRes.Y());
        Assert.Equal(15.0, aRes.Z());
        Assert.Equal(0.25, aRes.U());
        Assert.Equal(0.4, aRes.V());
    }

    // Regression test for bug #18: Divide by zero must not crash or print to stdout;
    // returns default-initialized (zero) point instead.
    [Fact]
    public void Divide_ZeroDivisor_ReturnsDefaultPoint()
    {
        PolyhedronPoint aPt = new(10.0, 20.0, 30.0, 0.5, 0.8);
        PolyhedronPoint aRes = aPt.Divide(0.0);

        Assert.Equal(0.0, aRes.X());
        Assert.Equal(0.0, aRes.Y());
        Assert.Equal(0.0, aRes.Z());
        Assert.Equal(0.0, aRes.U());
        Assert.Equal(0.0, aRes.V());
    }

    // Near-zero divisor (below Precision::Computational) returns default point.
    [Fact]
    public void Divide_NearZeroDivisor_ReturnsDefaultPoint()
    {
        PolyhedronPoint aPt = new(1.0, 2.0, 3.0, 0.1, 0.2);
        PolyhedronPoint aRes = aPt.Divide(1.0e-20);

        Assert.Equal(0.0, aRes.X());
        Assert.Equal(0.0, aRes.Y());
        Assert.Equal(0.0, aRes.Z());
    }

    // Negative divisor works correctly.
    [Fact]
    public void Divide_NegativeDivisor_CorrectResult()
    {
        PolyhedronPoint aPt = new(6.0, -9.0, 12.0, 0.3, 0.6);
        PolyhedronPoint aRes = aPt.Divide(-3.0);

        Assert.Equal(-2.0, aRes.X());
        Assert.Equal(3.0, aRes.Y());
        Assert.Equal(-4.0, aRes.Z());
    }

    [Fact]
    public void Add_CorrectResult()
    {
        PolyhedronPoint aP1 = new(1.0, 2.0, 3.0, 0.1, 0.2);
        PolyhedronPoint aP2 = new(4.0, 5.0, 6.0, 0.3, 0.4);
        PolyhedronPoint aRes = aP1.Add(aP2);

        Assert.Equal(5.0, aRes.X());
        Assert.Equal(7.0, aRes.Y());
        Assert.Equal(9.0, aRes.Z());
    }

    [Fact]
    public void Sub_CorrectResult()
    {
        PolyhedronPoint aP1 = new(5.0, 7.0, 9.0, 0.5, 0.8);
        PolyhedronPoint aP2 = new(1.0, 2.0, 3.0, 0.1, 0.2);
        PolyhedronPoint aRes = aP1.Sub(aP2);

        Assert.Equal(4.0, aRes.X());
        Assert.Equal(5.0, aRes.Y());
        Assert.Equal(6.0, aRes.Z());
    }

    [Fact]
    public void SquareModulus_CorrectResult()
    {
        PolyhedronPoint aPt = new(1.0, 2.0, 3.0, 0.0, 0.0);
        Assert.Equal(14.0, aPt.SquareModulus());
    }

    [Fact]
    public void SquareDistance_SamePoint_Zero()
    {
        PolyhedronPoint aPt = new(3.0, 4.0, 5.0, 0.0, 0.0);
        Assert.Equal(0.0, aPt.SquareDistance(aPt));
    }

    [Fact]
    public void Dot_OrthogonalVectors_Zero()
    {
        PolyhedronPoint aP1 = new(1.0, 0.0, 0.0, 0.0, 0.0);
        PolyhedronPoint aP2 = new(0.0, 1.0, 0.0, 0.0, 0.0);
        Assert.Equal(0.0, aP1.Dot(aP2));
    }
}

/// <summary>OCCT's IntPolyh_Intersection GTest.</summary>
public class IntPolyh_IntersectionTest
{
    // Helper to validate UV coordinates: they must be finite.
    private static void validateUV(double theU, double theV)
    {
        Assert.True(double.IsFinite(theU));
        Assert.True(double.IsFinite(theV));
    }

    // Regression test for bugs #13/#14: UV coordinate assignment in degenerate
    // alpha branches of IntPolyh_MaillageAffinage must correctly pair SetUV1/SetUV2.
    [Fact]
    public void SpherePlane_ValidUVCoordinates()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

        SphericalSurface aSphere = new(anAxis, 1.0);
        SurfaceAdaptor aSurfS = new(aSphere);

        PlaneSurface aPlane = new(anAxis);
        SurfaceAdaptor aSurfP = new(aPlane);

        PolyhedronIntersection anInter = new(aSurfS, aSurfP);

        Assert.True(anInter.IsDone);

        int aNbLines = anInter.SectionLineCount;
        Assert.True(aNbLines > 0);

        // Verify all section-line points have valid UV coordinates on both surfaces.
        for (int iL = 1; iL <= aNbLines; ++iL)
        {
            int aNbPts = anInter.NbPointsInLine(iL);
            for (int iP = 1; iP <= aNbPts; ++iP)
            {
                anInter.GetLinePoint(iL, iP, out _, out _, out _, out double u1, out double v1, out double u2,
                                     out double v2, out _);

                validateUV(u1, v1);
                validateUV(u2, v2);
            }
        }
    }

    // Sphere-cylinder intersection produces section lines with correct UVs.
    [Fact]
    public void SphereCylinder_ValidUVCoordinates()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

        SphericalSurface aSphere = new(anAxis, 2.0);
        SurfaceAdaptor aSurfS = new(aSphere);

        CoordinateSystem aCylAxis = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CylindricalSurface aCyl = new(aCylAxis, 1.0);
        SurfaceAdaptor aSurfC = new(aCyl);

        PolyhedronIntersection anInter = new(aSurfS, aSurfC);
        Assert.True(anInter.IsDone);

        int aNbLines = anInter.SectionLineCount;
        Assert.True(aNbLines > 0);

        for (int iL = 1; iL <= aNbLines; ++iL)
        {
            int aNbPts = anInter.NbPointsInLine(iL);
            for (int iP = 1; iP <= aNbPts; ++iP)
            {
                anInter.GetLinePoint(iL, iP, out _, out _, out _, out double u1, out double v1, out double u2,
                                     out double v2, out _);

                validateUV(u1, v1);
                validateUV(u2, v2);

                // UV1 must not equal UV2 (different surfaces have different parametrization).
                bool isSameUV = Math.Abs(u1 - u2) < global::Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(v1 - v2) < global::Kerf.Engine.Foundation.Precision.PConfusion();
                Assert.False(isSameUV, $"UV1 == UV2 at line={iL} pt={iP} (possible SetUV copy/paste bug)");
            }
        }
    }

    // Basic intersection: two tilted planes should intersect along a line.
    [Fact]
    public void TwoPlanes_ProducesSectionLine()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        CoordinateSystem anAxis2 = new(new Point3D(0, 0, 0), new Direction3D(0, 1, 1));

        PlaneSurface aPlane1 = new(anAxis1);
        SurfaceAdaptor aSurf1 = new(aPlane1, -5.0, 5.0, -5.0, 5.0);

        PlaneSurface aPlane2 = new(anAxis2);
        SurfaceAdaptor aSurf2 = new(aPlane2, -5.0, 5.0, -5.0, 5.0);

        PolyhedronIntersection anInter = new(aSurf1, aSurf2);
        Assert.True(anInter.IsDone);
        Assert.True(anInter.SectionLineCount > 0);
    }
}
