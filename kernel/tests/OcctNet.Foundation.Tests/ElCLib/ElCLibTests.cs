// Port of: src/FoundationClasses/TKMath/GTests/ElCLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class ElClibTests
{
    // Helper function for comparing points with tolerance
    private static void checkPointsEqual(in gp_Pnt theP1, in gp_Pnt theP2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theP2.X(), theP1.X(), theTolerance);
        Assert.Equal(theP2.Y(), theP1.Y(), theTolerance);
        Assert.Equal(theP2.Z(), theP1.Z(), theTolerance);
    }

    // Helper function for comparing vectors with tolerance
    private static void checkVectorsEqual(in gp_Vec theV1, in gp_Vec theV2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theV2.X(), theV1.X(), theTolerance);
        Assert.Equal(theV2.Y(), theV1.Y(), theTolerance);
        Assert.Equal(theV2.Z(), theV1.Z(), theTolerance);
    }

    // Helper function for comparing directions with tolerance
    private static void checkDirectorsEqual(in gp_Dir theD1, in gp_Dir theD2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theD2.X(), theD1.X(), theTolerance);
        Assert.Equal(theD2.Y(), theD1.Y(), theTolerance);
        Assert.Equal(theD2.Z(), theD1.Z(), theTolerance);
    }

    [Fact]
    public void InPeriod()
    {
        // Test with standard range [0, 2pi]
        double PI2 = 2.0 * M_PI;

        Assert.Equal(0.5, ElCLib.InPeriod(0.5, 0.0, PI2), Precision.Confusion());
        Assert.Equal(0.5, ElCLib.InPeriod(PI2 + 0.5, 0.0, PI2), Precision.Confusion());
        Assert.Equal(PI2 - 0.5, ElCLib.InPeriod(-0.5, 0.0, PI2), Precision.Confusion());
        Assert.Equal(PI2 - 0.5, ElCLib.InPeriod(-PI2 - 0.5, 0.0, PI2), Precision.Confusion());

        // Test with arbitrary range [1.5, 4.5]
        Assert.Equal(1.7, ElCLib.InPeriod(1.7, 1.5, 4.5), Precision.Confusion());
        Assert.Equal(1.7, ElCLib.InPeriod(4.7, 1.5, 4.5), Precision.Confusion());
        Assert.Equal(1.7, ElCLib.InPeriod(7.7, 1.5, 4.5), Precision.Confusion());
        Assert.Equal(4.3, ElCLib.InPeriod(1.3, 1.5, 4.5), Precision.Confusion());
    }

    [Fact]
    public void AdjustPeriodic()
    {
        double U1, U2;
        double PI2 = 2.0 * M_PI;

        // Test with standard range [0, 2pi]
        // Case 1: U1 and U2 within range, no adjustment needed
        U1 = 0.5;
        U2 = 0.7;
        ElCLib.AdjustPeriodic(0.0, PI2, Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, Precision.Confusion());
        Assert.Equal(0.7, U2, Precision.Confusion());

        // Case 2: U1 within range, U2 outside range, should adjust U2 to be within period from U1
        U1 = 0.5;
        U2 = 0.5 + PI2 + 0.2;
        ElCLib.AdjustPeriodic(0.0, PI2, Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, Precision.Confusion());
        // U2 is adjusted to U1 + period
        Assert.Equal(0.7, U2, Precision.Confusion());

        // Case 3: Both U1 and U2 outside range but within same period
        U1 = 0.5 + PI2;
        U2 = 0.7 + PI2;
        ElCLib.AdjustPeriodic(0.0, PI2, Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, Precision.Confusion());
        Assert.Equal(0.7, U2, Precision.Confusion());

        // Test with negative U1
        U1 = -0.5;
        U2 = 0.7;
        ElCLib.AdjustPeriodic(0.0, PI2, Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(PI2 - 0.5, U1, Precision.Confusion());
        Assert.Equal(0.7 + PI2, U2, Precision.Confusion());

        // Test where U2 is very close to U1 (should add a period to U2)
        U1 = 1.0;
        U2 = 1.0 + 0.5 * Precision.Confusion(); // Very close to U1
        ElCLib.AdjustPeriodic(0.0, PI2, Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(1.0, U1, Precision.Confusion());
        Assert.Equal(1.0 + PI2, U2, Precision.Confusion());
    }

    [Fact]
    public void Line3D()
    {
        gp_Pnt aLoc = new(1.0, 2.0, 3.0);
        gp_Dir aDir = new(gp_Dir.D.Z);
        gp_Lin aLin = new(aLoc, aDir);
        double aParam = 5.0;

        // Test Value
        gp_Pnt aPoint = ElCLib.Value(aParam, aLin);
        gp_Pnt aExpectedPoint = new(1.0, 2.0, 8.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        ElCLib.D1(aParam, aLin, out gp_Pnt aPointD1, out gp_Vec aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        checkVectorsEqual(aVecD1, new gp_Vec(aDir));

        // Test DN
        gp_Vec aVecDN = ElCLib.DN(aParam, aLin, 1);
        checkVectorsEqual(aVecDN, new gp_Vec(aDir));
        gp_Vec aVecDN2 = ElCLib.DN(aParam, aLin, 2);
        checkVectorsEqual(aVecDN2, new gp_Vec(0.0, 0.0, 0.0));

        // Test Parameter
        gp_Pnt aTestPoint = new(1.0, 2.0, 10.0);
        double aCalculatedParam = ElCLib.Parameter(aLin, aTestPoint);
        Assert.Equal(7.0, aCalculatedParam, Precision.Confusion());
    }

    [Fact]
    public void Circle3D()
    {
        gp_Pnt aLoc = new(0.0, 0.0, 0.0);
        gp_Dir aDirZ = new(gp_Dir.D.Z);
        gp_Dir aDirX = new(gp_Dir.D.X);
        gp_Ax2 anAxis = new(aLoc, aDirZ, aDirX);
        double aRadius = 2.0;
        gp_Circ aCircle = new(anAxis, aRadius);
        double aParam = M_PI / 4.0; // 45 degrees

        // Test Value
        gp_Pnt aPoint = ElCLib.Value(aParam, aCircle);
        gp_Pnt aExpectedPoint = new(aRadius * Math.Cos(aParam), aRadius * Math.Sin(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        ElCLib.D1(aParam, aCircle, out gp_Pnt aPointD1, out gp_Vec aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        gp_Vec aExpectedVecD1 = new(-aRadius * Math.Sin(aParam), aRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test D2
        ElCLib.D2(aParam, aCircle, out gp_Pnt aPointD2, out gp_Vec aVecD2_1, out gp_Vec aVecD2_2);
        checkPointsEqual(aPointD2, aExpectedPoint);
        checkVectorsEqual(aVecD2_1, aExpectedVecD1);
        gp_Vec aExpectedVecD2_2 = new(-aRadius * Math.Cos(aParam), -aRadius * Math.Sin(aParam), 0.0);
        checkVectorsEqual(aVecD2_2, aExpectedVecD2_2);

        // Test D3
        ElCLib.D3(aParam, aCircle, out gp_Pnt aPointD3, out gp_Vec aVecD3_1, out gp_Vec aVecD3_2, out gp_Vec aVecD3_3);
        checkPointsEqual(aPointD3, aExpectedPoint);
        checkVectorsEqual(aVecD3_1, aExpectedVecD1);
        checkVectorsEqual(aVecD3_2, aExpectedVecD2_2);
        gp_Vec aExpectedVecD3_3 = new(aRadius * Math.Sin(aParam), -aRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD3_3, aExpectedVecD3_3);

        // Test DN
        gp_Vec aVecDN1 = ElCLib.DN(aParam, aCircle, 1);
        checkVectorsEqual(aVecDN1, aExpectedVecD1);

        gp_Vec aVecDN2 = ElCLib.DN(aParam, aCircle, 2);
        checkVectorsEqual(aVecDN2, aExpectedVecD2_2);

        gp_Vec aVecDN3 = ElCLib.DN(aParam, aCircle, 3);
        checkVectorsEqual(aVecDN3, aExpectedVecD3_3);

        // Test Parameter
        double aCalculatedParam = ElCLib.Parameter(aCircle, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, Precision.Confusion());
    }

    [Fact]
    public void Ellipse3D()
    {
        gp_Pnt aLoc = new(0.0, 0.0, 0.0);
        gp_Dir aDirZ = new(gp_Dir.D.Z);
        gp_Dir aDirX = new(gp_Dir.D.X);
        gp_Ax2 anAxis = new(aLoc, aDirZ, aDirX);
        double aMajorRadius = 3.0;
        double aMinorRadius = 2.0;
        gp_Elips anEllipse = new(anAxis, aMajorRadius, aMinorRadius);
        double aParam = M_PI / 4.0; // 45 degrees

        // Test Value
        gp_Pnt aPoint = ElCLib.Value(aParam, anEllipse);
        gp_Pnt aExpectedPoint = new(aMajorRadius * Math.Cos(aParam), aMinorRadius * Math.Sin(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        ElCLib.D1(aParam, anEllipse, out gp_Pnt aPointD1, out gp_Vec aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        gp_Vec aExpectedVecD1 = new(-aMajorRadius * Math.Sin(aParam), aMinorRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = ElCLib.Parameter(anEllipse, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, Precision.Confusion());
    }

    [Fact]
    public void Hyperbola3D()
    {
        gp_Pnt aLoc = new(0.0, 0.0, 0.0);
        gp_Dir aDirZ = new(gp_Dir.D.Z);
        gp_Dir aDirX = new(gp_Dir.D.X);
        gp_Ax2 anAxis = new(aLoc, aDirZ, aDirX);
        double aMajorRadius = 3.0;
        double aMinorRadius = 2.0;
        gp_Hypr aHyperbola = new(anAxis, aMajorRadius, aMinorRadius);
        double aParam = 0.5;

        // Test Value
        gp_Pnt aPoint = ElCLib.Value(aParam, aHyperbola);
        gp_Pnt aExpectedPoint = new(aMajorRadius * Math.Cosh(aParam), aMinorRadius * Math.Sinh(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        ElCLib.D1(aParam, aHyperbola, out gp_Pnt aPointD1, out gp_Vec aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        gp_Vec aExpectedVecD1 = new(aMajorRadius * Math.Sinh(aParam), aMinorRadius * Math.Cosh(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = ElCLib.Parameter(aHyperbola, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, Precision.Confusion());
    }

    [Fact]
    public void Parabola3D()
    {
        gp_Pnt aLoc = new(0.0, 0.0, 0.0);
        gp_Dir aDirZ = new(gp_Dir.D.Z);
        gp_Dir aDirX = new(gp_Dir.D.X);
        gp_Ax2 anAxis = new(aLoc, aDirZ, aDirX);
        double aFocal = 2.0;
        gp_Parab aParabola = new(anAxis, aFocal);
        double aParam = 3.0;

        // Test Value
        gp_Pnt aPoint = ElCLib.Value(aParam, aParabola);
        gp_Pnt aExpectedPoint = new(aParam * aParam / (4.0 * aFocal), aParam, 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        ElCLib.D1(aParam, aParabola, out gp_Pnt aPointD1, out gp_Vec aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        gp_Vec aExpectedVecD1 = new(aParam / (2.0 * aFocal), 1.0, 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = ElCLib.Parameter(aParabola, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, Precision.Confusion());
    }

    [Fact]
    public void To3dConversion()
    {
        gp_Pnt aLoc = new(1.0, 2.0, 3.0);
        gp_Dir aDirZ = new(gp_Dir.D.Z);
        gp_Dir aDirX = new(gp_Dir.D.X);
        gp_Ax2 anAxis = new(aLoc, aDirZ, aDirX);

        // Test conversion of a point
        gp_Pnt2d aPnt2d = new(2.0, 3.0);
        gp_Pnt aPnt3d = ElCLib.To3d(anAxis, aPnt2d);
        gp_Pnt aExpectedPnt3d = new(3.0, 5.0, 3.0);
        checkPointsEqual(aPnt3d, aExpectedPnt3d);

        // Test conversion of a vector
        gp_Vec2d aVec2d = new(1.0, 2.0);
        gp_Vec aVec3d = ElCLib.To3d(anAxis, aVec2d);
        gp_Vec aExpectedVec3d = new(1.0, 2.0, 0.0);
        checkVectorsEqual(aVec3d, aExpectedVec3d);

        // Test conversion of a direction
        gp_Dir2d aDir2d = new(1.0, 2.0);
        gp_Dir aDir3d = ElCLib.To3d(anAxis, aDir2d);
        gp_Dir aExpectedDir3d = new(1.0 / Math.Sqrt(5.0), 2.0 / Math.Sqrt(5.0), 0.0);
        checkDirectorsEqual(aDir3d, aExpectedDir3d, Precision.Confusion());

        // Test conversion of a circle
        gp_Pnt2d aLoc2d = new(0.0, 0.0);
        gp_Dir2d aDir2dX = new(gp_Dir2d.D.X);
        gp_Ax22d anAxis2d = new(aLoc2d, aDir2dX, true);
        double aRadius = 2.0;
        gp_Circ2d aCirc2d = new(anAxis2d, aRadius);
        gp_Circ aCirc3d = ElCLib.To3d(anAxis, aCirc2d);
        Assert.Equal(aRadius, aCirc3d.Radius(), Precision.Confusion());
        checkPointsEqual(aCirc3d.Location(), aLoc, Precision.Confusion());
    }

    [Fact]
    public void Parabola3D_NearZeroFocal_DegeneratesToLine()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));
        double aFocal = 0.5 * gp.Resolution();
        double aU = 4.0;

        gp_Pnt aValue = ElCLib.ParabolaValue(aU, anAxis, aFocal);
        checkPointsEqual(aValue, new gp_Pnt(1.0 + aU, 2.0, 3.0));

        ElCLib.ParabolaD1(aU, anAxis, aFocal, out gp_Pnt aP, out gp_Vec aD1);
        checkPointsEqual(aP, new gp_Pnt(1.0 + aU, 2.0, 3.0));
        checkVectorsEqual(aD1, new gp_Vec(1.0, 0.0, 0.0));

        ElCLib.ParabolaD2(aU, anAxis, aFocal, out aP, out aD1, out gp_Vec aD2);
        checkVectorsEqual(aD2, new gp_Vec(0.0, 0.0, 0.0));

        checkVectorsEqual(ElCLib.ParabolaDN(aU, anAxis, aFocal, 1), new gp_Vec(1.0, 0.0, 0.0));
        checkVectorsEqual(ElCLib.ParabolaDN(aU, anAxis, aFocal, 2), new gp_Vec(0.0, 0.0, 0.0));
    }

    [Fact]
    public void Parabola2D_NearZeroFocal_DegeneratesToLine()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0), true);
        double aFocal = 0.5 * gp.Resolution();
        double aU = 3.0;

        gp_Pnt2d aValue = ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.Equal(1.0 + aU, aValue.X(), Precision.Confusion());
        Assert.Equal(2.0, aValue.Y(), Precision.Confusion());

        ElCLib.ParabolaD1(aU, anAxis, aFocal, out gp_Pnt2d aP, out gp_Vec2d aD1);
        Assert.Equal(1.0 + aU, aP.X(), Precision.Confusion());
        Assert.Equal(2.0, aP.Y(), Precision.Confusion());
        Assert.Equal(1.0, aD1.X(), Precision.Confusion());
        Assert.Equal(0.0, aD1.Y(), Precision.Confusion());

        ElCLib.ParabolaD2(aU, anAxis, aFocal, out aP, out aD1, out gp_Vec2d aD2);
        Assert.Equal(0.0, aD2.X(), Precision.Confusion());
        Assert.Equal(0.0, aD2.Y(), Precision.Confusion());

        gp_Vec2d aDN1 = ElCLib.ParabolaDN(aU, anAxis, aFocal, 1);
        gp_Vec2d aDN2 = ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.Equal(1.0, aDN1.X(), Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y(), Precision.Confusion());
        Assert.Equal(0.0, aDN2.X(), Precision.Confusion());
        Assert.Equal(0.0, aDN2.Y(), Precision.Confusion());
    }

    [Fact]
    public void Parabola3D_AboveResolutionFocal_NotDegenerate()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));
        double aFocal = 2.0 * gp.Resolution();
        double aU = 1.0;

        gp_Pnt aValue = ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.True(Math.Abs(aValue.X() - (1.0 + aU)) > 1.0);

        gp_Vec aD2 = ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.True(aD2.Magnitude() > 1.0);
    }

    [Fact]
    public void Parabola2D_AboveResolutionFocal_NotDegenerate()
    {
        gp_Ax22d anAxis = new(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0), true);
        double aFocal = 2.0 * gp.Resolution();
        double aU = 1.0;

        gp_Pnt2d aValue = ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.True(Math.Abs(aValue.X() - (1.0 + aU)) > 1.0);

        gp_Vec2d aD2 = ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.True(aD2.Magnitude() > 1.0);
    }
}
