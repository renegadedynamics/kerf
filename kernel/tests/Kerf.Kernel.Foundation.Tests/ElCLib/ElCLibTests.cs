// Port of: src/FoundationClasses/TKMath/GTests/ElCLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class ElClibTests
{
    // Helper function for comparing points with tolerance
    private static void checkPointsEqual(in Point3D theP1, in Point3D theP2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theP2.X, theP1.X, theTolerance);
        Assert.Equal(theP2.Y, theP1.Y, theTolerance);
        Assert.Equal(theP2.Z, theP1.Z, theTolerance);
    }

    // Helper function for comparing vectors with tolerance
    private static void checkVectorsEqual(in Vector3D theV1, in Vector3D theV2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theV2.X, theV1.X, theTolerance);
        Assert.Equal(theV2.Y, theV1.Y, theTolerance);
        Assert.Equal(theV2.Z, theV1.Z, theTolerance);
    }

    // Helper function for comparing directions with tolerance
    private static void checkDirectorsEqual(in Direction3D theD1, in Direction3D theD2, double theTolerance = 1.0e-7)
    {
        Assert.Equal(theD2.X, theD1.X, theTolerance);
        Assert.Equal(theD2.Y, theD1.Y, theTolerance);
        Assert.Equal(theD2.Z, theD1.Z, theTolerance);
    }

    [Fact]
    public void InPeriod()
    {
        // Test with standard range [0, 2pi]
        double PI2 = 2.0 * M_PI;

        Assert.Equal(0.5, global::Kerf.Engine.Geometry.ElCLib.InPeriod(0.5, 0.0, PI2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.5, global::Kerf.Engine.Geometry.ElCLib.InPeriod(PI2 + 0.5, 0.0, PI2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(PI2 - 0.5, global::Kerf.Engine.Geometry.ElCLib.InPeriod(-0.5, 0.0, PI2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(PI2 - 0.5, global::Kerf.Engine.Geometry.ElCLib.InPeriod(-PI2 - 0.5, 0.0, PI2), global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test with arbitrary range [1.5, 4.5]
        Assert.Equal(1.7, global::Kerf.Engine.Geometry.ElCLib.InPeriod(1.7, 1.5, 4.5), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.7, global::Kerf.Engine.Geometry.ElCLib.InPeriod(4.7, 1.5, 4.5), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.7, global::Kerf.Engine.Geometry.ElCLib.InPeriod(7.7, 1.5, 4.5), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(4.3, global::Kerf.Engine.Geometry.ElCLib.InPeriod(1.3, 1.5, 4.5), global::Kerf.Engine.Foundation.Precision.Confusion());
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
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, PI2, global::Kerf.Engine.Foundation.Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.7, U2, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Case 2: U1 within range, U2 outside range, should adjust U2 to be within period from U1
        U1 = 0.5;
        U2 = 0.5 + PI2 + 0.2;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, PI2, global::Kerf.Engine.Foundation.Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, global::Kerf.Engine.Foundation.Precision.Confusion());
        // U2 is adjusted to U1 + period
        Assert.Equal(0.7, U2, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Case 3: Both U1 and U2 outside range but within same period
        U1 = 0.5 + PI2;
        U2 = 0.7 + PI2;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, PI2, global::Kerf.Engine.Foundation.Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(0.5, U1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.7, U2, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test with negative U1
        U1 = -0.5;
        U2 = 0.7;
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, PI2, global::Kerf.Engine.Foundation.Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(PI2 - 0.5, U1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.7 + PI2, U2, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test where U2 is very close to U1 (should add a period to U2)
        U1 = 1.0;
        U2 = 1.0 + 0.5 * global::Kerf.Engine.Foundation.Precision.Confusion(); // Very close to U1
        global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, PI2, global::Kerf.Engine.Foundation.Precision.Confusion(), ref U1, ref U2);
        Assert.Equal(1.0, U1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0 + PI2, U2, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Line3D()
    {
        Point3D aLoc = new(1.0, 2.0, 3.0);
        Direction3D aDir = new(Direction3D.D.Z);
        Line3D aLin = new(aLoc, aDir);
        double aParam = 5.0;

        // Test Value
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(aParam, aLin);
        Point3D aExpectedPoint = new(1.0, 2.0, 8.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        global::Kerf.Engine.Geometry.ElCLib.D1(aParam, aLin, out Point3D aPointD1, out Vector3D aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        checkVectorsEqual(aVecD1, new Vector3D(aDir));

        // Test DN
        Vector3D aVecDN = global::Kerf.Engine.Geometry.ElCLib.DN(aParam, aLin, 1);
        checkVectorsEqual(aVecDN, new Vector3D(aDir));
        Vector3D aVecDN2 = global::Kerf.Engine.Geometry.ElCLib.DN(aParam, aLin, 2);
        checkVectorsEqual(aVecDN2, new Vector3D(0.0, 0.0, 0.0));

        // Test Parameter
        Point3D aTestPoint = new(1.0, 2.0, 10.0);
        double aCalculatedParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin, aTestPoint);
        Assert.Equal(7.0, aCalculatedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle3D()
    {
        Point3D aLoc = new(0.0, 0.0, 0.0);
        Direction3D aDirZ = new(Direction3D.D.Z);
        Direction3D aDirX = new(Direction3D.D.X);
        Frame3D anAxis = new(aLoc, aDirZ, aDirX);
        double aRadius = 2.0;
        Circle3D aCircle = new(anAxis, aRadius);
        double aParam = M_PI / 4.0; // 45 degrees

        // Test Value
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(aParam, aCircle);
        Point3D aExpectedPoint = new(aRadius * Math.Cos(aParam), aRadius * Math.Sin(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        global::Kerf.Engine.Geometry.ElCLib.D1(aParam, aCircle, out Point3D aPointD1, out Vector3D aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        Vector3D aExpectedVecD1 = new(-aRadius * Math.Sin(aParam), aRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test D2
        global::Kerf.Engine.Geometry.ElCLib.D2(aParam, aCircle, out Point3D aPointD2, out Vector3D aVecD2_1, out Vector3D aVecD2_2);
        checkPointsEqual(aPointD2, aExpectedPoint);
        checkVectorsEqual(aVecD2_1, aExpectedVecD1);
        Vector3D aExpectedVecD2_2 = new(-aRadius * Math.Cos(aParam), -aRadius * Math.Sin(aParam), 0.0);
        checkVectorsEqual(aVecD2_2, aExpectedVecD2_2);

        // Test D3
        global::Kerf.Engine.Geometry.ElCLib.D3(aParam, aCircle, out Point3D aPointD3, out Vector3D aVecD3_1, out Vector3D aVecD3_2, out Vector3D aVecD3_3);
        checkPointsEqual(aPointD3, aExpectedPoint);
        checkVectorsEqual(aVecD3_1, aExpectedVecD1);
        checkVectorsEqual(aVecD3_2, aExpectedVecD2_2);
        Vector3D aExpectedVecD3_3 = new(aRadius * Math.Sin(aParam), -aRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD3_3, aExpectedVecD3_3);

        // Test DN
        Vector3D aVecDN1 = global::Kerf.Engine.Geometry.ElCLib.DN(aParam, aCircle, 1);
        checkVectorsEqual(aVecDN1, aExpectedVecD1);

        Vector3D aVecDN2 = global::Kerf.Engine.Geometry.ElCLib.DN(aParam, aCircle, 2);
        checkVectorsEqual(aVecDN2, aExpectedVecD2_2);

        Vector3D aVecDN3 = global::Kerf.Engine.Geometry.ElCLib.DN(aParam, aCircle, 3);
        checkVectorsEqual(aVecDN3, aExpectedVecD3_3);

        // Test Parameter
        double aCalculatedParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aCircle, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Ellipse3D()
    {
        Point3D aLoc = new(0.0, 0.0, 0.0);
        Direction3D aDirZ = new(Direction3D.D.Z);
        Direction3D aDirX = new(Direction3D.D.X);
        Frame3D anAxis = new(aLoc, aDirZ, aDirX);
        double aMajorRadius = 3.0;
        double aMinorRadius = 2.0;
        Ellipse3D anEllipse = new(anAxis, aMajorRadius, aMinorRadius);
        double aParam = M_PI / 4.0; // 45 degrees

        // Test Value
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(aParam, anEllipse);
        Point3D aExpectedPoint = new(aMajorRadius * Math.Cos(aParam), aMinorRadius * Math.Sin(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        global::Kerf.Engine.Geometry.ElCLib.D1(aParam, anEllipse, out Point3D aPointD1, out Vector3D aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        Vector3D aExpectedVecD1 = new(-aMajorRadius * Math.Sin(aParam), aMinorRadius * Math.Cos(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(anEllipse, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Hyperbola3D()
    {
        Point3D aLoc = new(0.0, 0.0, 0.0);
        Direction3D aDirZ = new(Direction3D.D.Z);
        Direction3D aDirX = new(Direction3D.D.X);
        Frame3D anAxis = new(aLoc, aDirZ, aDirX);
        double aMajorRadius = 3.0;
        double aMinorRadius = 2.0;
        Hyperbola3D aHyperbola = new(anAxis, aMajorRadius, aMinorRadius);
        double aParam = 0.5;

        // Test Value
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(aParam, aHyperbola);
        Point3D aExpectedPoint = new(aMajorRadius * Math.Cosh(aParam), aMinorRadius * Math.Sinh(aParam), 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        global::Kerf.Engine.Geometry.ElCLib.D1(aParam, aHyperbola, out Point3D aPointD1, out Vector3D aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        Vector3D aExpectedVecD1 = new(aMajorRadius * Math.Sinh(aParam), aMinorRadius * Math.Cosh(aParam), 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aHyperbola, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parabola3D()
    {
        Point3D aLoc = new(0.0, 0.0, 0.0);
        Direction3D aDirZ = new(Direction3D.D.Z);
        Direction3D aDirX = new(Direction3D.D.X);
        Frame3D anAxis = new(aLoc, aDirZ, aDirX);
        double aFocal = 2.0;
        Parabola3D aParabola = new(anAxis, aFocal);
        double aParam = 3.0;

        // Test Value
        Point3D aPoint = global::Kerf.Engine.Geometry.ElCLib.Value(aParam, aParabola);
        Point3D aExpectedPoint = new(aParam * aParam / (4.0 * aFocal), aParam, 0.0);
        checkPointsEqual(aPoint, aExpectedPoint);

        // Test D1
        global::Kerf.Engine.Geometry.ElCLib.D1(aParam, aParabola, out Point3D aPointD1, out Vector3D aVecD1);
        checkPointsEqual(aPointD1, aExpectedPoint);
        Vector3D aExpectedVecD1 = new(aParam / (2.0 * aFocal), 1.0, 0.0);
        checkVectorsEqual(aVecD1, aExpectedVecD1);

        // Test Parameter
        double aCalculatedParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aParabola, aExpectedPoint);
        Assert.Equal(aParam, aCalculatedParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void To3dConversion()
    {
        Point3D aLoc = new(1.0, 2.0, 3.0);
        Direction3D aDirZ = new(Direction3D.D.Z);
        Direction3D aDirX = new(Direction3D.D.X);
        Frame3D anAxis = new(aLoc, aDirZ, aDirX);

        // Test conversion of a point
        Point2D aPnt2d = new(2.0, 3.0);
        Point3D aPnt3d = global::Kerf.Engine.Geometry.ElCLib.To3d(anAxis, aPnt2d);
        Point3D aExpectedPnt3d = new(3.0, 5.0, 3.0);
        checkPointsEqual(aPnt3d, aExpectedPnt3d);

        // Test conversion of a vector
        Vector2D aVec2d = new(1.0, 2.0);
        Vector3D aVec3d = global::Kerf.Engine.Geometry.ElCLib.To3d(anAxis, aVec2d);
        Vector3D aExpectedVec3d = new(1.0, 2.0, 0.0);
        checkVectorsEqual(aVec3d, aExpectedVec3d);

        // Test conversion of a direction
        Direction2D aDir2d = new(1.0, 2.0);
        Direction3D aDir3d = global::Kerf.Engine.Geometry.ElCLib.To3d(anAxis, aDir2d);
        Direction3D aExpectedDir3d = new(1.0 / Math.Sqrt(5.0), 2.0 / Math.Sqrt(5.0), 0.0);
        checkDirectorsEqual(aDir3d, aExpectedDir3d, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Test conversion of a circle
        Point2D aLoc2d = new(0.0, 0.0);
        Direction2D aDir2dX = new(Direction2D.D.X);
        Frame2D anAxis2d = new(aLoc2d, aDir2dX, true);
        double aRadius = 2.0;
        Circle2D aCirc2d = new(anAxis2d, aRadius);
        Circle3D aCirc3d = global::Kerf.Engine.Geometry.ElCLib.To3d(anAxis, aCirc2d);
        Assert.Equal(aRadius, aCirc3d.Radius, global::Kerf.Engine.Foundation.Precision.Confusion());
        checkPointsEqual(aCirc3d.Location, aLoc, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parabola3D_NearZeroFocal_DegeneratesToLine()
    {
        Frame3D anAxis = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0));
        double aFocal = 0.5 * global::Kerf.Engine.Geometry.Constants.Resolution();
        double aU = 4.0;

        Point3D aValue = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(aU, anAxis, aFocal);
        checkPointsEqual(aValue, new Point3D(1.0 + aU, 2.0, 3.0));

        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(aU, anAxis, aFocal, out Point3D aP, out Vector3D aD1);
        checkPointsEqual(aP, new Point3D(1.0 + aU, 2.0, 3.0));
        checkVectorsEqual(aD1, new Vector3D(1.0, 0.0, 0.0));

        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(aU, anAxis, aFocal, out aP, out aD1, out Vector3D aD2);
        checkVectorsEqual(aD2, new Vector3D(0.0, 0.0, 0.0));

        checkVectorsEqual(global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 1), new Vector3D(1.0, 0.0, 0.0));
        checkVectorsEqual(global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 2), new Vector3D(0.0, 0.0, 0.0));
    }

    [Fact]
    public void Parabola2D_NearZeroFocal_DegeneratesToLine()
    {
        Frame2D anAxis = new(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0), true);
        double aFocal = 0.5 * global::Kerf.Engine.Geometry.Constants.Resolution();
        double aU = 3.0;

        Point2D aValue = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.Equal(1.0 + aU, aValue.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aValue.Y, global::Kerf.Engine.Foundation.Precision.Confusion());

        global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(aU, anAxis, aFocal, out Point2D aP, out Vector2D aD1);
        Assert.Equal(1.0 + aU, aP.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aP.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aD1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());

        global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(aU, anAxis, aFocal, out aP, out aD1, out Vector2D aD2);
        Assert.Equal(0.0, aD2.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD2.Y, global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aDN1 = global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 1);
        Vector2D aDN2 = global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.Equal(1.0, aDN1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDN1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDN2.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aDN2.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Parabola3D_AboveResolutionFocal_NotDegenerate()
    {
        Frame3D anAxis = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0));
        double aFocal = 2.0 * global::Kerf.Engine.Geometry.Constants.Resolution();
        double aU = 1.0;

        Point3D aValue = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.True(Math.Abs(aValue.X - (1.0 + aU)) > 1.0);

        Vector3D aD2 = global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.True(aD2.Magnitude > 1.0);
    }

    [Fact]
    public void Parabola2D_AboveResolutionFocal_NotDegenerate()
    {
        Frame2D anAxis = new(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0), true);
        double aFocal = 2.0 * global::Kerf.Engine.Geometry.Constants.Resolution();
        double aU = 1.0;

        Point2D aValue = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(aU, anAxis, aFocal);
        Assert.True(Math.Abs(aValue.X - (1.0 + aU)) > 1.0);

        Vector2D aD2 = global::Kerf.Engine.Geometry.ElCLib.ParabolaDN(aU, anAxis, aFocal, 2);
        Assert.True(aD2.Magnitude > 1.0);
    }
}
