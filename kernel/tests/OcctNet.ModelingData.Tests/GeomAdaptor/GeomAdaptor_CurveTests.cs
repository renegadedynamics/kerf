// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomAdaptor_Curve_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>GeomAdaptor_Curve: degenerated curve handling.</summary>
public class GeomAdaptor_Curve_Test
{
    private readonly Geom_Line myLine;
    private readonly Geom_Circle myCircle;

    public GeomAdaptor_Curve_Test()
    {
        // Create a simple 3D line for testing
        gp_Pnt aP1 = new(0.0, 0.0, 0.0);
        gp_Lin aLine = new(aP1, new gp_Dir(1.0, 1.0, 0.0));
        myLine = new Geom_Line(aLine);

        // Create a 3D circle for testing
        gp_Pnt aCenter = new(5.0, 5.0, 0.0);
        gp_Circ aCirc = new(new gp_Ax2(aCenter, new gp_Dir(0.0, 0.0, 1.0)), 3.0);
        myCircle = new Geom_Circle(aCirc);
    }

    [Fact]
    public void Load_ValidParameters_Success()
    {
        // Test loading with valid parameters
        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, 0.0, 10.0);

        Assert.Equal(0.0, anAdaptor.FirstParameter());
        Assert.Equal(10.0, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_EqualParameters_Success()
    {
        // Test loading with equal parameters (degenerated curve)
        // This should be allowed as it represents a point
        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, 5.0, 5.0);

        Assert.Equal(5.0, anAdaptor.FirstParameter());
        Assert.Equal(5.0, anAdaptor.LastParameter());

        // Verify it evaluates to a single point
        gp_Pnt aP1 = anAdaptor.Value(5.0);
        gp_Pnt aP2 = myLine.Value(5.0);
        Assert.True(aP1.IsEqual(aP2, Precision.Confusion()));
    }

    [Fact]
    public void Load_ParametersWithinConfusion_Success()
    {
        // Test loading with parameters within Precision::Confusion()
        // This should be allowed (degenerated curve handling)
        double aParam1 = 5.0;
        double aParam2 = 5.0 + Precision.Confusion() * 0.5;

        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_ParametersAtConfusionBoundary_Success()
    {
        // Test loading with parameters exactly at the confusion tolerance boundary
        double aParam1 = 5.0;
        double aParam2 = 5.0 + Precision.Confusion();

        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_FirstGreaterThanLastWithinConfusion_Success()
    {
        // Test loading with theUFirst > theULast but within Precision::Confusion()
        // This represents a degenerated curve and should be allowed
        double aParam1 = 5.0 + Precision.Confusion() * 0.5;
        double aParam2 = 5.0;

        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_FirstGreaterThanLastBeyondConfusion_ThrowsException()
    {
        // Test loading with UFirst > ULast + Precision::Confusion()
        // This should throw Standard_ConstructionError
        GeomAdaptor_Curve anAdaptor = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => anAdaptor.Load(myLine, 10.0, 5.0));
    }

    [Fact]
    public void Load_FirstSlightlyGreaterThanLast_ThrowsException()
    {
        // Test loading with UFirst slightly greater than ULast beyond tolerance
        double aParam1 = 5.0;
        double aParam2 = 5.0 - Precision.Confusion() * 2.0;

        GeomAdaptor_Curve anAdaptor = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => anAdaptor.Load(myLine, aParam1, aParam2));
    }

    [Fact]
    public void Constructor_DegeneratedCurve_Success()
    {
        // Test constructor with degenerated curve parameters
        GeomAdaptor_Curve anAdaptor = new(myCircle, 0.0, 0.0);
        Assert.Equal(0.0, anAdaptor.FirstParameter());
        Assert.Equal(0.0, anAdaptor.LastParameter());
    }

    [Fact]
    public void Constructor_InvalidParameters_ThrowsException()
    {
        // Test constructor with invalid parameters
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomAdaptor_Curve(myCircle, 10.0, 0.0));
    }

    [Fact]
    public void Load_NullCurve_ThrowsException()
    {
        // Test loading with null curve
        Geom_Curve? aNullCurve = null;
        GeomAdaptor_Curve anAdaptor = new();

        Assert.ThrowsAny<Standard_NullObject>(() => anAdaptor.Load(aNullCurve!, 0.0, 10.0));
    }

    [Fact]
    public void DegeneratedCurve_CircleAtZeroLength_Success()
    {
        // Test degenerated circle (zero length arc)
        double aParam = M_PI;

        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myCircle, aParam, aParam);

        // Verify the curve represents a single point
        gp_Pnt aPoint = anAdaptor.Value(aParam);
        gp_Pnt aExpectedPoint = myCircle.Value(aParam);

        Assert.True(aPoint.IsEqual(aExpectedPoint, Precision.Confusion()));
        Assert.True(anAdaptor.IsClosed() || anAdaptor.FirstParameter() == anAdaptor.LastParameter());
    }

    [Fact]
    public void DegeneratedCurve_TrimmedCurve_Success()
    {
        // Create a trimmed curve and test degenerated case
        Geom_TrimmedCurve aTrimmedCurve = new(myLine, 0.0, 20.0);

        double aParam = 10.0;
        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(aTrimmedCurve, aParam, aParam);

        Assert.Equal(aParam, anAdaptor.FirstParameter());
        Assert.Equal(aParam, anAdaptor.LastParameter());
    }

    [Fact]
    public void ToleranceBoundary_NegativeCase_ThrowsException()
    {
        // Test parameters just beyond the negative tolerance boundary
        double aParam1 = 5.0;
        double aParam2 = 5.0 - Precision.Confusion() - 1e-10;

        GeomAdaptor_Curve anAdaptor = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => anAdaptor.Load(myLine, aParam1, aParam2));
    }

    [Fact]
    public void LoadWithoutParameters_Success()
    {
        // Test loading curve without specifying parameters (uses curve's own parameters)
        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myCircle);

        Assert.Equal(myCircle.FirstParameter(), anAdaptor.FirstParameter(), Precision.Confusion());
        Assert.Equal(myCircle.LastParameter(), anAdaptor.LastParameter(), Precision.Confusion());
        Assert.True(anAdaptor.IsPeriodic());
    }

    [Fact]
    public void DegeneratedCurve_MultipleLocations_Success()
    {
        // Test degenerated curves at different parameter locations
        double[] aParams = [0.0, 1.0, -5.0, 100.0, M_PI];

        foreach (double aParam in aParams)
        {
            GeomAdaptor_Curve anAdaptor = new();
            anAdaptor.Load(myLine, aParam, aParam);

            gp_Pnt aPoint1 = anAdaptor.Value(aParam);
            gp_Pnt aPoint2 = myLine.Value(aParam);
            Assert.True(aPoint1.IsEqual(aPoint2, Precision.Confusion()));
        }
    }

    [Fact]
    public void BoundaryConditions_VerySmallInterval_Success()
    {
        // Test with very small but valid interval (just above tolerance)
        double aParam1 = 5.0;
        double aParam2 = 5.0 + Precision.Confusion() + 1e-12;

        GeomAdaptor_Curve anAdaptor = new();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Constructor_WithValidRange_Success()
    {
        // Test the 3-parameter constructor with valid range
        double aFirst = 0.0;
        double aLast = 2.0 * M_PI;

        GeomAdaptor_Curve anAdaptor = new(myCircle, aFirst, aLast);
        Assert.Equal(aFirst, anAdaptor.FirstParameter());
        Assert.Equal(aLast, anAdaptor.LastParameter());
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Circle, anAdaptor.GetType());
    }
}
