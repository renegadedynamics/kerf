// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomAdaptor_TransformedCurve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

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
    private static gp_Trsf CreateTranslation(double theDx, double theDy, double theDz)
    {
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(theDx, theDy, theDz));
        return aTrsf;
    }

    //! Helper function to create a rotation transformation around Z axis.
    private static gp_Trsf CreateRotationZ(double theAngleRad)
    {
        gp_Trsf aTrsf = new();
        aTrsf.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), theAngleRad);
        return aTrsf;
    }

    private static Geom_Circle UnitCircle2() => new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);

    // Tests for GeomAdaptor_TransformedCurve construction

    [Fact]
    public void DefaultConstructor()
    {
        GeomAdaptor_TransformedCurve aCurve = new();
        Assert.True(aCurve.Is3DCurve());
        Assert.False(aCurve.IsCurveOnSurface());
        Assert.Equal(gp_TrsfForm.gp_Identity, aCurve.Trsf().Form());
    }

    [Fact]
    public void ConstructWithCurveAndTrsf()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        gp_Trsf aTrsf = CreateTranslation(0, 0, 5);

        GeomAdaptor_TransformedCurve aCurve = new(aLine, aTrsf);
        Assert.True(aCurve.Is3DCurve());
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, aCurve.GetType());
        Assert.Equal(gp_TrsfForm.gp_Translation, aCurve.Trsf().Form());
    }

    [Fact]
    public void ConstructWithBounds()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        gp_Trsf aTrsf = CreateTranslation(0, 0, 5);

        GeomAdaptor_TransformedCurve aCurve = new(aLine, 0.0, 10.0, aTrsf);
        Assert.Equal(0.0, aCurve.FirstParameter(), THE_TOLERANCE);
        Assert.Equal(10.0, aCurve.LastParameter(), THE_TOLERANCE);
    }

    // Tests for evaluation with identity transform

    [Fact]
    public void Line_IdentityTransform()
    {
        Geom_Line aLine = new(new gp_Pnt(1, 2, 3), new gp_Dir(1, 0, 0));
        gp_Trsf aTrsf = new(); // identity

        GeomAdaptor_TransformedCurve aCurve = new(aLine, aTrsf);

        // With identity transform, results should match direct Geom_Line evaluation
        double aT = 5.0;
        gp_Pnt aPnt = aCurve.Value(aT);
        gp_Pnt aExpected = aLine.Value(aT);
        Assert.Equal(0.0, aPnt.Distance(aExpected), THE_TOLERANCE);
    }

    [Fact]
    public void Circle_IdentityTransform()
    {
        Geom_Circle aCircle = UnitCircle2();
        gp_Trsf aTrsf = new(); // identity

        GeomAdaptor_TransformedCurve aCurve = new(aCircle, aTrsf);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Circle, aCurve.GetType());

        double[] aParams = [0.0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI];
        foreach (double aParam in aParams)
        {
            gp_Pnt aPnt = aCurve.Value(aParam);
            gp_Pnt aExpected = aCircle.Value(aParam);
            Assert.Equal(0.0, aPnt.Distance(aExpected), THE_TOLERANCE);
        }
    }

    // Tests for evaluation with non-identity transform

    [Fact]
    public void Line_Translation()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_TransformedCurve aCurve = new(aLine, CreateTranslation(0, 0, 5));

        // Line along X at Z=0, translated to Z=5
        gp_Pnt aPnt = aCurve.Value(3.0);
        Assert.Equal(3.0, aPnt.X(), THE_TOLERANCE);
        Assert.Equal(0.0, aPnt.Y(), THE_TOLERANCE);
        Assert.Equal(5.0, aPnt.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void Line_Rotation()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_TransformedCurve aCurve = new(aLine, CreateRotationZ(M_PI / 2));

        // Line along X rotated 90 degrees -> line along Y
        gp_Pnt aPnt = aCurve.Value(3.0);
        Assert.Equal(0.0, aPnt.X(), THE_TOLERANCE);
        Assert.Equal(3.0, aPnt.Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aPnt.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void Circle_Translation()
    {
        Geom_Circle aCircle = UnitCircle2();
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 17);

        for (int i = 1; i <= 17; ++i)
        {
            gp_Pnt aPnt = aCurve.Value(aParams.Value(i));
            gp_Pnt aLocal = aCircle.Value(aParams.Value(i));
            gp_Pnt aExpected = new(aLocal.X() + 10, aLocal.Y() + 20, aLocal.Z() + 30);
            Assert.Equal(0.0, aPnt.Distance(aExpected), THE_TOLERANCE);
        }
    }

    // Tests for derivative evaluation with transform

    [Fact]
    public void D1_Translation()
    {
        Geom_Circle aCircle = UnitCircle2();
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D1(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1);

            // Point should be translated
            gp_Pnt aLocalPnt = aCircle.Value(aParams.Value(i));
            gp_Pnt aExpectedPnt = new(aLocalPnt.X() + 10, aLocalPnt.Y() + 20, aLocalPnt.Z() + 30);
            Assert.Equal(0.0, aPnt.Distance(aExpectedPnt), THE_TOLERANCE);

            // D1 vector: translation doesn't change vectors, so should match local D1
            aCircle.D1(aParams.Value(i), out _, out gp_Vec aLocalD1);
            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D1_Rotation()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_TransformedCurve aCurve = new(aLine, CreateRotationZ(M_PI / 2));

        aCurve.D1(3.0, out _, out gp_Vec aD1);

        // D1 for line along X is (1,0,0), rotated 90 degrees becomes (0,1,0)
        Assert.Equal(0.0, aD1.X(), THE_TOLERANCE);
        Assert.Equal(1.0, aD1.Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aD1.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void D2_Translation()
    {
        Geom_Circle aCircle = UnitCircle2();
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 20, 30));

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D2(aParams.Value(i), out _, out gp_Vec aD1, out gp_Vec aD2);
            aCircle.D2(aParams.Value(i), out _, out gp_Vec aLocalD1, out gp_Vec aLocalD2);

            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aD2 - aLocalD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void D3_Rotation()
    {
        Geom_Circle aCircle = UnitCircle2();
        gp_Trsf aTrsf = CreateRotationZ(M_PI / 4);
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, aTrsf);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D3(aParams.Value(i), out gp_Pnt aPnt, out gp_Vec aD1, out gp_Vec aD2, out gp_Vec aD3);

            // Get local values and transform manually
            aCircle.D3(aParams.Value(i), out gp_Pnt aLocalP, out gp_Vec aLocalD1, out gp_Vec aLocalD2, out gp_Vec aLocalD3);
            aLocalP.Transform(aTrsf);
            aLocalD1.Transform(aTrsf);
            aLocalD2.Transform(aTrsf);
            aLocalD3.Transform(aTrsf);

            Assert.Equal(0.0, aPnt.Distance(aLocalP), THE_TOLERANCE);
            Assert.Equal(0.0, (aD1 - aLocalD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aD2 - aLocalD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aD3 - aLocalD3).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DN_Rotation()
    {
        Geom_Circle aCircle = UnitCircle2();
        gp_Trsf aTrsf = CreateRotationZ(M_PI / 3);
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, aTrsf);

        for (int aOrder = 1; aOrder <= 3; ++aOrder)
        {
            gp_Vec aResult = aCurve.DN(1.0, aOrder);
            gp_Vec aExpected = aCircle.DN(1.0, aOrder);
            aExpected.Transform(aTrsf);
            Assert.Equal(0.0, (aResult - aExpected).Magnitude(), THE_TOLERANCE);
        }
    }

    // Tests for geometry extraction with transform

    [Fact]
    public void LineExtraction_Transform()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_TransformedCurve aCurve = new(aLine, CreateTranslation(0, 0, 5));

        gp_Lin aExtracted = aCurve.Line();

        // Line should be translated: origin at (0,0,5), direction unchanged
        Assert.Equal(0.0, aExtracted.Location().X(), THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location().Y(), THE_TOLERANCE);
        Assert.Equal(5.0, aExtracted.Location().Z(), THE_TOLERANCE);
        Assert.Equal(1.0, aExtracted.Direction().X(), THE_TOLERANCE);
    }

    [Fact]
    public void CircleExtraction_Transform()
    {
        Geom_Circle aCircle = UnitCircle2();
        GeomAdaptor_TransformedCurve aCurve = new(aCircle, CreateTranslation(10, 0, 0));

        gp_Circ aExtracted = aCurve.Circle();

        // Circle center should be translated to (10,0,0)
        Assert.Equal(10.0, aExtracted.Location().X(), THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location().Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aExtracted.Location().Z(), THE_TOLERANCE);
        Assert.Equal(2.0, aExtracted.Radius(), THE_TOLERANCE);
    }

    // Tests for ShallowCopy

    [Fact]
    public void ShallowCopy()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_TransformedCurve aCurve = new(aLine, 0.0, 10.0, CreateTranslation(0, 0, 5));

        Adaptor3d_Curve aCopy = aCurve.ShallowCopy();
        Assert.NotNull(aCopy);

        // Copy should evaluate the same
        gp_Pnt aPnt1 = aCurve.Value(3.0);
        gp_Pnt aPnt2 = aCopy.Value(3.0);
        Assert.Equal(0.0, aPnt1.Distance(aPnt2), THE_TOLERANCE);
    }
}
