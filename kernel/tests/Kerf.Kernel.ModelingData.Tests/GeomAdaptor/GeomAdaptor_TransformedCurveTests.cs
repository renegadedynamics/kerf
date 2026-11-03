// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomAdaptor_TransformedCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>GeomAdaptor_TransformedCurve. All of OCCT's tests are ported (none needs GeomGridEval).</summary>
public class GeomAdaptor_TransformedCurveTest
{
    private const double THE_TOLERANCE = 1e-10;

    //! Helper function to create uniform parameters.
    private static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints)
    {
        NCollection_Array1<double> aParams = new(1, theNbPoints);
        double aStep = (theLast - theFirst) / (theNbPoints - 1);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            aParams.SetValue(i, theFirst + (i - 1) * aStep);
        }
        return aParams;
    }

    //! Helper function to create a translation transformation.
    private static Transform3D CreateTranslation(double theDx, double theDy, double theDz)
    {
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(theDx, theDy, theDz));
        return aTrsf;
    }

    //! Helper function to create a rotation transformation around Z axis.
    private static Transform3D CreateRotationZ(double theAngleRad)
    {
        Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theAngleRad);
        return aTrsf;
    }

    private static CircleCurve3D UnitCircle2() => new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);

    // Tests for GeomAdaptor_TransformedCurve construction

    [Fact]
    public void DefaultConstructor()
    {
        TransformedCurve aCurve = new();
        Assert.True(aCurve.Is3DCurve());
        Assert.False(aCurve.IsCurveOnSurface);
        Assert.Equal(TransformForm.Identity, aCurve.Trsf().Form);
    }

    [Fact]
    public void ConstructWithCurveAndTrsf()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Transform3D aTrsf = CreateTranslation(0, 0, 5);

        TransformedCurve aCurve = new(aLine, aTrsf);
        Assert.True(aCurve.Is3DCurve());
        Assert.Equal(CurveKind.Line, aCurve.GetType());
        Assert.Equal(TransformForm.Translation, aCurve.Trsf().Form);
    }

    [Fact]
    public void ConstructWithBounds()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Transform3D aTrsf = CreateTranslation(0, 0, 5);

        TransformedCurve aCurve = new(aLine, 0.0, 10.0, aTrsf);
        Assert.Equal(0.0, aCurve.FirstParameter(), THE_TOLERANCE);
        Assert.Equal(10.0, aCurve.LastParameter(), THE_TOLERANCE);
    }

    // Tests for evaluation with identity transform

    [Fact]
    public void Line_IdentityTransform()
    {
        LineCurve3D aLine = new(new Point3D(1, 2, 3), new Direction3D(1, 0, 0));
        Transform3D aTrsf = new(); // identity

        TransformedCurve aCurve = new(aLine, aTrsf);

        // With identity transform, results should match direct Geom_Line evaluation
        double aT = 5.0;
        Point3D aPnt = aCurve.Value(aT);
        Point3D aExpected = aLine.Value(aT);
        Assert.Equal(0.0, aPnt.DistanceTo(aExpected), THE_TOLERANCE);
    }

    [Fact]
    public void Circle_IdentityTransform()
    {
        CircleCurve3D aCircle = UnitCircle2();
        Transform3D aTrsf = new(); // identity

        TransformedCurve aCurve = new(aCircle, aTrsf);
        Assert.Equal(CurveKind.Circle, aCurve.GetType());

        double[] aParams = [0.0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI];
        foreach (double aParam in aParams)
        {
            Point3D aPnt = aCurve.Value(aParam);
            Point3D aExpected = aCircle.Value(aParam);
            Assert.Equal(0.0, aPnt.DistanceTo(aExpected), THE_TOLERANCE);
        }
    }

    // Tests for evaluation with non-identity transform

    [Fact]
    public void Line_Translation()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TransformedCurve aCurve = new(aLine, CreateTranslation(0, 0, 5));

        // Line along X at Z=0, translated to Z=5
        Point3D aPnt = aCurve.Value(3.0);
        Assert.Equal(3.0, aPnt.X, THE_TOLERANCE);
        Assert.Equal(0.0, aPnt.Y, THE_TOLERANCE);
        Assert.Equal(5.0, aPnt.Z, THE_TOLERANCE);
    }

    [Fact]
    public void Line_Rotation()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TransformedCurve aCurve = new(aLine, CreateRotationZ(M_PI / 2));

        // Line along X rotated 90 degrees -> line along Y
        Point3D aPnt = aCurve.Value(3.0);
        Assert.Equal(0.0, aPnt.X, THE_TOLERANCE);
        Assert.Equal(3.0, aPnt.Y, THE_TOLERANCE);
        Assert.Equal(0.0, aPnt.Z, THE_TOLERANCE);
    }

    [Fact]
    public void Circle_Translation()
    {
        CircleCurve3D aCircle = UnitCircle2();
        TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 17);

        for (int i = 1; i <= 17; ++i)
        {
            Point3D aPnt = aCurve.Value(aParams.Value(i));
            Point3D aLocal = aCircle.Value(aParams.Value(i));
            Point3D aExpected = new(aLocal.X + 10, aLocal.Y + 20, aLocal.Z + 30);
            Assert.Equal(0.0, aPnt.DistanceTo(aExpected), THE_TOLERANCE);
        }
    }

    // Tests for derivative evaluation with transform

    [Fact]
    public void D1_Translation()
    {
        CircleCurve3D aCircle = UnitCircle2();
        TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D1(aParams.Value(i), out Point3D aPnt, out Vector3D aD1);

            // Point should be translated
            Point3D aLocalPnt = aCircle.Value(aParams.Value(i));
            Point3D aExpectedPnt = new(aLocalPnt.X + 10, aLocalPnt.Y + 20, aLocalPnt.Z + 30);
            Assert.Equal(0.0, aPnt.DistanceTo(aExpectedPnt), THE_TOLERANCE);

            // D1 vector: translation doesn't change vectors, so should match local D1
            aCircle.D1(aParams.Value(i), out _, out Vector3D aLocalD1);
            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude, THE_TOLERANCE);
        }
    }

    [Fact]
    public void D1_Rotation()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TransformedCurve aCurve = new(aLine, CreateRotationZ(M_PI / 2));

        aCurve.D1(3.0, out _, out Vector3D aD1);

        // D1 for line along X is (1,0,0), rotated 90 degrees becomes (0,1,0)
        Assert.Equal(0.0, aD1.X, THE_TOLERANCE);
        Assert.Equal(1.0, aD1.Y, THE_TOLERANCE);
        Assert.Equal(0.0, aD1.Z, THE_TOLERANCE);
    }

    [Fact]
    public void D2_Translation()
    {
        CircleCurve3D aCircle = UnitCircle2();
        TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D2(aParams.Value(i), out _, out Vector3D aD1, out Vector3D aD2);
            aCircle.D2(aParams.Value(i), out _, out Vector3D aLocalD1, out Vector3D aLocalD2);

            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude, THE_TOLERANCE);
            Assert.Equal(0.0, (aD2 - aLocalD2).Magnitude, THE_TOLERANCE);
        }
    }

    [Fact]
    public void D3_Rotation()
    {
        CircleCurve3D aCircle = UnitCircle2();
        Transform3D aTrsf = CreateRotationZ(M_PI / 4);
        TransformedCurve aCurve = new(aCircle, aTrsf);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);

            // Get local values and transform manually
            aCircle.D3(aParams.Value(i), out Point3D aLocalP, out Vector3D aLocalD1, out Vector3D aLocalD2, out Vector3D aLocalD3);
            aLocalP = aLocalP.Transformed(aTrsf);
            aLocalD1 = aLocalD1.Transformed(aTrsf);
            aLocalD2 = aLocalD2.Transformed(aTrsf);
            aLocalD3 = aLocalD3.Transformed(aTrsf);

            Assert.Equal(0.0, aPnt.DistanceTo(aLocalP), THE_TOLERANCE);
            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude, THE_TOLERANCE);
            Assert.Equal(0.0, (aD2 - aLocalD2).Magnitude, THE_TOLERANCE);
            Assert.Equal(0.0, (aD3 - aLocalD3).Magnitude, THE_TOLERANCE);
        }
    }

    [Fact]
    public void DN_Rotation()
    {
        CircleCurve3D aCircle = UnitCircle2();
        Transform3D aTrsf = CreateRotationZ(M_PI / 3);
        TransformedCurve aCurve = new(aCircle, aTrsf);

        for (int aOrder = 1; aOrder <= 3; ++aOrder)
        {
            Vector3D aResult = aCurve.DN(1.0, aOrder);
            Vector3D aExpected = aCircle.DN(1.0, aOrder);
            aExpected = aExpected.Transformed(aTrsf);
            Assert.Equal(0.0, (aResult - aExpected).Magnitude, THE_TOLERANCE);
        }
    }

    // Tests for geometry extraction with transform

    [Fact]
    public void LineExtraction_Transform()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TransformedCurve aCurve = new(aLine, CreateTranslation(0, 0, 5));

        Line3D aExtracted = aCurve.Line();

        // Line should be translated: origin at (0,0,5), direction unchanged
        Assert.Equal(0.0, aExtracted.Location.X, THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location.Y, THE_TOLERANCE);
        Assert.Equal(5.0, aExtracted.Location.Z, THE_TOLERANCE);
        Assert.Equal(1.0, aExtracted.Direction.X, THE_TOLERANCE);
    }

    [Fact]
    public void CircleExtraction_Transform()
    {
        CircleCurve3D aCircle = UnitCircle2();
        TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 0, 0));

        Circle3D aExtracted = aCurve.Circle();

        // Circle center should be translated to (10,0,0)
        Assert.Equal(10.0, aExtracted.Location.X, THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location.Y, THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location.Z, THE_TOLERANCE);
        Assert.Equal(2.0, aExtracted.Radius, THE_TOLERANCE);
    }

    // Tests for ShallowCopy

    [Fact]
    public void ShallowCopy()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TransformedCurve aCurve = new(aLine, 0.0, 10.0, CreateTranslation(0, 0, 5));

        CurveAdaptorBase aCopy = aCurve.ShallowCopy();
        Assert.NotNull(aCopy);

        // Copy should evaluate the same
        Point3D aPnt1 = aCurve.Value(3.0);
        Point3D aPnt2 = aCopy.Value(3.0);
        Assert.Equal(0.0, aPnt1.DistanceTo(aPnt2), THE_TOLERANCE);
    }
}
