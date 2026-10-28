// Port of: src/FoundationClasses/TKMath/GTests/CSLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_DerivativeStatus;
using static Occt.CSLib_NormalStatus;

namespace OcctNet.Foundation.Tests;

internal static class CSLibTestHelpers
{
    // Helper function for comparing directions with tolerance
    public static void checkDirEqual(in gp_Dir theDir1, in gp_Dir theDir2, double theTolerance = 1e-10)
    {
        Assert.Equal(theDir2.X(), theDir1.X(), theTolerance);
        Assert.Equal(theDir2.Y(), theDir1.Y(), theTolerance);
        Assert.Equal(theDir2.Z(), theDir1.Z(), theTolerance);
    }

    // Helper function for comparing vectors with tolerance
    public static void checkVecEqual(in gp_Vec theVec1, in gp_Vec theVec2, double theTolerance = 1e-10)
    {
        Assert.Equal(theVec2.X(), theVec1.X(), theTolerance);
        Assert.Equal(theVec2.Y(), theVec1.Y(), theTolerance);
        Assert.Equal(theVec2.Z(), theVec1.Z(), theTolerance);
    }

    public static NCollection_Array1<gp_Pnt2d> UnitSquare()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = new(1, 4);
        aPnts[1] = new gp_Pnt2d(0.0, 0.0);
        aPnts[2] = new gp_Pnt2d(1.0, 0.0);
        aPnts[3] = new gp_Pnt2d(1.0, 1.0);
        aPnts[4] = new gp_Pnt2d(0.0, 1.0);
        return aPnts;
    }
}

public class CSLibNormalTest
{
    // Test Normal calculation from D1U and D1V (first overload)
    [Fact]
    public void Normal_FromD1U_D1V_OrthogonalVectors()
    {
        // D1U along X, D1V along Y -> Normal should be along Z
        gp_Vec aD1U = new(1.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 1.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_Done, aStatus);
        CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(0.0, 0.0, 1.0));
    }

    [Fact]
    public void Normal_FromD1U_D1V_ScaledVectors()
    {
        // Non-unit vectors should still produce correct normal
        gp_Vec aD1U = new(3.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 5.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_Done, aStatus);
        CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(0.0, 0.0, 1.0));
    }

    [Fact]
    public void Normal_FromD1U_D1V_ParallelVectors()
    {
        // Parallel vectors should return D1uIsParallelD1v status
        gp_Vec aD1U = new(1.0, 0.0, 0.0);
        gp_Vec aD1V = new(2.0, 0.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-6, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_D1uIsParallelD1v, aStatus);
    }

    [Fact]
    public void Normal_FromD1U_D1V_D1UIsNull()
    {
        gp_Vec aD1U = new(0.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 1.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_D1uIsNull, aStatus);
    }

    [Fact]
    public void Normal_FromD1U_D1V_D1VIsNull()
    {
        gp_Vec aD1U = new(1.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 0.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_D1vIsNull, aStatus);
    }

    [Fact]
    public void Normal_FromD1U_D1V_BothNull()
    {
        gp_Vec aD1U = new(0.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 0.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_D1IsNull, aStatus);
    }

    // Test Normal with MagTol (third overload)
    [Fact]
    public void Normal_WithMagTol_Defined()
    {
        gp_Vec aD1U = new(1.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 1.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_NormalStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_Defined, aStatus);
        CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(0.0, 0.0, 1.0));
    }

    [Fact]
    public void Normal_WithMagTol_Singular()
    {
        // Very small vectors below tolerance
        gp_Vec aD1U = new(1e-12, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 1e-12, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_NormalStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_Singular, aStatus);
    }

    // Test DNNUV function
    [Fact]
    public void DNNUV_ZeroDerivative()
    {
        // Create a simple surface derivative array
        NCollection_Array2<gp_Vec> aDerSurf = new(0, 2, 0, 2);

        // Set D1U and D1V at index (1,0) and (0,1)
        aDerSurf.SetValue(1, 0, new gp_Vec(1.0, 0.0, 0.0)); // D1U
        aDerSurf.SetValue(0, 1, new gp_Vec(0.0, 1.0, 0.0)); // D1V
        aDerSurf.SetValue(0, 0, new gp_Vec(0.0, 0.0, 0.0)); // Position (not used)
        aDerSurf.SetValue(1, 1, new gp_Vec(0.0, 0.0, 0.0)); // D2UV
        aDerSurf.SetValue(2, 0, new gp_Vec(0.0, 0.0, 0.0)); // D2U
        aDerSurf.SetValue(0, 2, new gp_Vec(0.0, 0.0, 0.0)); // D2V

        // DNNUV(0,0) should give D1U x D1V
        gp_Vec aResult = CSLib.DNNUV(0, 0, aDerSurf);

        // D1U x D1V = (1,0,0) x (0,1,0) = (0,0,1)
        CSLibTestHelpers.checkVecEqual(aResult, new gp_Vec(0.0, 0.0, 1.0));
    }
}

public class CSLibClass2dTest
{
    // Test point inside a simple square polygon
    [Fact]
    public void SiDans_PointInsideSquare()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = CSLibTestHelpers.UnitSquare();

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 1.0, 1.0);

        // Point clearly inside
        gp_Pnt2d aPointInside = new(0.5, 0.5);
        Assert.Equal(1, (int)aClassifier.SiDans(aPointInside));
    }

    [Fact]
    public void SiDans_PointOutsideSquare()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = CSLibTestHelpers.UnitSquare();

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 1.0, 1.0);

        // Point clearly outside
        gp_Pnt2d aPointOutside = new(2.0, 2.0);
        Assert.Equal(-1, (int)aClassifier.SiDans(aPointOutside));
    }

    [Fact]
    public void SiDans_PointOnBoundary()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = CSLibTestHelpers.UnitSquare();

        CSLib_Class2d aClassifier = new(aPnts, 0.01, 0.01, 0.0, 0.0, 1.0, 1.0);

        // Point on boundary should return 0 (uncertain)
        gp_Pnt2d aPointOnEdge = new(0.5, 0.0);
        Assert.Equal(0, (int)aClassifier.SiDans(aPointOnEdge));
    }

    [Fact]
    public void SiDans_TriangularPolygon()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = new(1, 3);
        aPnts[1] = new gp_Pnt2d(0.0, 0.0);
        aPnts[2] = new gp_Pnt2d(2.0, 0.0);
        aPnts[3] = new gp_Pnt2d(1.0, 2.0);

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 2.0, 2.0);

        // Point inside triangle (centroid)
        Assert.Equal(1, (int)aClassifier.SiDans(new gp_Pnt2d(1.0, 0.5)));

        // Point outside triangle
        Assert.Equal(-1, (int)aClassifier.SiDans(new gp_Pnt2d(0.0, 1.0)));
    }

    // Test with Sequence constructor
    [Fact]
    public void SequenceConstructor_PointInsideSquare()
    {
        NCollection_Sequence<gp_Pnt2d> aPnts = new();
        aPnts.Append(new gp_Pnt2d(0.0, 0.0));
        aPnts.Append(new gp_Pnt2d(1.0, 0.0));
        aPnts.Append(new gp_Pnt2d(1.0, 1.0));
        aPnts.Append(new gp_Pnt2d(0.0, 1.0));

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 1.0, 1.0);

        Assert.Equal(1, (int)aClassifier.SiDans(new gp_Pnt2d(0.5, 0.5)));
    }

    // Test InternalSiDans with normalized coordinates
    [Fact]
    public void InternalSiDans_NormalizedCoordinates()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = new(1, 4);
        aPnts[1] = new gp_Pnt2d(0.0, 0.0);
        aPnts[2] = new gp_Pnt2d(10.0, 0.0);
        aPnts[3] = new gp_Pnt2d(10.0, 10.0);
        aPnts[4] = new gp_Pnt2d(0.0, 10.0);

        CSLib_Class2d aClassifier = new(aPnts, 0.1, 0.1, 0.0, 0.0, 10.0, 10.0);

        // Test classification with actual coordinates
        Assert.Equal(CSLib_Class2d.Result.Result_Inside, aClassifier.SiDans(new gp_Pnt2d(5.0, 5.0)));
        Assert.Equal(CSLib_Class2d.Result.Result_Outside, aClassifier.SiDans(new gp_Pnt2d(15.0, 15.0)));
    }

    // Test SiDans_OnMode
    [Fact]
    public void SiDans_OnMode_PointInside()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = CSLibTestHelpers.UnitSquare();

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 1.0, 1.0);

        Assert.Equal(1, (int)aClassifier.SiDans_OnMode(new gp_Pnt2d(0.5, 0.5), 0.01));
    }

    // Test with degenerate polygon (less than 3 points effective)
    [Fact]
    public void DegeneratePolygon_InvalidBounds()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = CSLibTestHelpers.UnitSquare();

        // Invalid bounds (umax <= umin)
        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 1.0, 0.0, 0.0, 1.0);

        // Should return 0 for any point with invalid bounds
        Assert.Equal(0, (int)aClassifier.SiDans(new gp_Pnt2d(0.5, 0.5)));
    }
}

public class CSLibNormalPolyDefTest
{
    // Test Value function
    [Fact]
    public void Value_ConstantPolynomial()
    {
        // k0 = 0, lambda_0 = 1.0 -> f(X) = 1 * cos^0 * sin^0 = 1 (constant)
        NCollection_Array1<double> aLambda = new(0, 0);
        aLambda[0] = 1.0;

        CSLib_NormalPolyDef aPoly = new(0, aLambda);

        Assert.True(aPoly.Value(0.5, out double aValue));
        Assert.Equal(1.0, aValue, 1e-10);

        Assert.True(aPoly.Value(1.0, out aValue));
        Assert.Equal(1.0, aValue, 1e-10);
    }

    [Fact]
    public void Value_LinearPolynomial()
    {
        // k0 = 1, lambda = [1, 1] -> f(X) = cos(X) + sin(X)
        NCollection_Array1<double> aLambda = new(0, 1);
        aLambda[0] = 1.0;
        aLambda[1] = 1.0;

        CSLib_NormalPolyDef aPoly = new(1, aLambda);

        // f(pi/4) = cos(pi/4) + sin(pi/4) = sqrt(2)
        Assert.True(aPoly.Value(M_PI / 4.0, out double aValue));
        Assert.Equal(Math.Sqrt(2.0), aValue, 1e-10);
    }

    [Fact]
    public void Value_AtSingularPoints()
    {
        // At singular points (X=0, pi/2, pi, 3*pi/2), the function returns 0
        // as a safety measure when computing would involve 0^n terms.
        NCollection_Array1<double> aLambda = new(0, 2);
        aLambda[0] = 1.0;
        aLambda[1] = 1.0;
        aLambda[2] = 1.0;

        CSLib_NormalPolyDef aPoly = new(2, aLambda);

        // Test that the function doesn't crash at these points.
        Assert.True(aPoly.Value(0.0, out double aValue));
        Assert.True(double.IsFinite(aValue));

        Assert.True(aPoly.Value(M_PI / 2.0, out aValue));
        Assert.True(double.IsFinite(aValue));
    }

    // Test Derivative function
    [Fact]
    public void Derivative_AtRegularPoint()
    {
        // k0 = 2, test derivative at non-singular point
        NCollection_Array1<double> aLambda = new(0, 2);
        aLambda[0] = 1.0;
        aLambda[1] = 0.0;
        aLambda[2] = 1.0;

        CSLib_NormalPolyDef aPoly = new(2, aLambda);

        Assert.True(aPoly.Derivative(M_PI / 4.0, out double aDeriv));
        // Derivative should be computed without crash
        Assert.True(double.IsFinite(aDeriv));
    }

    [Fact]
    public void Derivative_AtSingularPoint()
    {
        NCollection_Array1<double> aLambda = new(0, 2);
        aLambda[0] = 1.0;
        aLambda[1] = 1.0;
        aLambda[2] = 1.0;

        CSLib_NormalPolyDef aPoly = new(2, aLambda);

        // At singular points, derivative should be 0
        Assert.True(aPoly.Derivative(0.0, out double aDeriv));
        Assert.Equal(0.0, aDeriv, 1e-10);
    }

    // Test Values function (combined value and derivative)
    [Fact]
    public void Values_Consistency()
    {
        NCollection_Array1<double> aLambda = new(0, 2);
        aLambda[0] = 1.0;
        aLambda[1] = 2.0;
        aLambda[2] = 1.0;

        CSLib_NormalPolyDef aPoly = new(2, aLambda);

        // Get values using separate functions
        Assert.True(aPoly.Value(0.7, out double aF1));
        Assert.True(aPoly.Derivative(0.7, out double aD1));

        // Get values using combined function
        Assert.True(aPoly.Values(0.7, out double aF2, out double aD2));

        // Results should be consistent
        Assert.Equal(aF2, aF1, 1e-10);
        Assert.Equal(aD2, aD1, 1e-10);
    }

    // Test numerical derivative approximation
    [Fact]
    public void Derivative_NumericalApproximation()
    {
        NCollection_Array1<double> aLambda = new(0, 2);
        aLambda[0] = 1.0;
        aLambda[1] = 2.0;
        aLambda[2] = 3.0;

        CSLib_NormalPolyDef aPoly = new(2, aLambda);

        const double aX = 0.8; // Away from singular points
        const double aH = 1e-6;

        Assert.True(aPoly.Value(aX - aH, out double aFMinus));
        Assert.True(aPoly.Value(aX + aH, out double aFPlus));
        Assert.True(aPoly.Derivative(aX, out double aAnalyticDeriv));

        double aNumericDeriv = (aFPlus - aFMinus) / (2.0 * aH);

        // Analytical derivative should match numerical approximation
        Assert.Equal(aNumericDeriv, aAnalyticDeriv, 1e-4);
    }
}

public class CSLibEnumTest
{
    [Fact]
    public void DerivativeStatus_AllValues()
    {
        // Verify all enum values are distinct
        Assert.NotEqual(CSLib_Done, CSLib_D1uIsNull);
        Assert.NotEqual(CSLib_Done, CSLib_D1vIsNull);
        Assert.NotEqual(CSLib_Done, CSLib_D1IsNull);
        Assert.NotEqual(CSLib_Done, CSLib_D1uD1vRatioIsNull);
        Assert.NotEqual(CSLib_Done, CSLib_D1vD1uRatioIsNull);
        Assert.NotEqual(CSLib_Done, CSLib_D1uIsParallelD1v);
    }

    [Fact]
    public void NormalStatus_AllValues()
    {
        // Verify all enum values are distinct
        Assert.NotEqual(CSLib_Singular, CSLib_Defined);
        Assert.NotEqual(CSLib_Singular, CSLib_InfinityOfSolutions);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NuIsNull);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NvIsNull);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NIsNull);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NuNvRatioIsNull);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NvNuRatioIsNull);
        Assert.NotEqual(CSLib_Singular, CSLib_D1NuIsParallelD1Nv);
    }
}

public class CSLibIntegrationTest
{
    // Test normal computation for a planar surface
    [Fact]
    public void PlanarSurface_ConstantNormal()
    {
        // For a plane z = 0, D1U = (1,0,0), D1V = (0,1,0): Normal should be (0,0,1) at all points
        gp_Vec aD1U = new(1.0, 0.0, 0.0);
        gp_Vec aD1V = new(0.0, 1.0, 0.0);

        gp_Dir aNormal = new();
        CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

        Assert.Equal(CSLib_Done, aStatus);
        CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(0.0, 0.0, 1.0));
    }

    // Test normal computation for a cylindrical surface
    [Fact]
    public void CylindricalSurface_VaryingNormal()
    {
        // For a cylinder x = cos(u), y = sin(u), z = v
        // At u = 0: D1U = (0,1,0), D1V = (0,0,1), Normal = (1,0,0)
        // At u = pi/2: D1U = (-1,0,0), D1V = (0,0,1), Normal = (0,1,0)

        // At u = 0
        {
            gp_Vec aD1U = new(0.0, 1.0, 0.0);
            gp_Vec aD1V = new(0.0, 0.0, 1.0);

            gp_Dir aNormal = new();
            CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

            Assert.Equal(CSLib_Done, aStatus);
            CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(1.0, 0.0, 0.0));
        }

        // At u = pi/2
        {
            gp_Vec aD1U = new(-1.0, 0.0, 0.0);
            gp_Vec aD1V = new(0.0, 0.0, 1.0);

            gp_Dir aNormal = new();
            CSLib.Normal(aD1U, aD1V, 1e-10, out CSLib_DerivativeStatus aStatus, ref aNormal);

            Assert.Equal(CSLib_Done, aStatus);
            CSLibTestHelpers.checkDirEqual(aNormal, new gp_Dir(0.0, 1.0, 0.0));
        }
    }

    // Test Class2d with complex polygon
    [Fact]
    public void Class2d_ComplexPolygon()
    {
        // L-shaped polygon
        NCollection_Array1<gp_Pnt2d> aPnts = new(1, 6);
        aPnts[1] = new gp_Pnt2d(0.0, 0.0);
        aPnts[2] = new gp_Pnt2d(2.0, 0.0);
        aPnts[3] = new gp_Pnt2d(2.0, 1.0);
        aPnts[4] = new gp_Pnt2d(1.0, 1.0);
        aPnts[5] = new gp_Pnt2d(1.0, 2.0);
        aPnts[6] = new gp_Pnt2d(0.0, 2.0);

        CSLib_Class2d aClassifier = new(aPnts, 1e-10, 1e-10, 0.0, 0.0, 2.0, 2.0);

        // Point inside the L
        Assert.Equal(1, (int)aClassifier.SiDans(new gp_Pnt2d(0.5, 0.5)));

        // Point in the "notch" of the L (outside)
        Assert.Equal(-1, (int)aClassifier.SiDans(new gp_Pnt2d(1.5, 1.5)));

        // Point clearly outside
        Assert.Equal(-1, (int)aClassifier.SiDans(new gp_Pnt2d(3.0, 3.0)));
    }
}
