// Port of: src/ModelingData/TKG2d/GTests/Geom2dAdaptor_Curve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2dAdaptor_Curve_Test
{
    private readonly LineCurve2D myLine;
    private readonly CircleCurve2D myCircle;

    public Geom2dAdaptor_Curve_Test()
    {
        // Create a simple 2D line for testing
        Point2D aP1 = new Point2D(0.0, 0.0);
        Line2D aLine = new Line2D(aP1, new Direction2D(1.0, 1.0));
        myLine = new LineCurve2D(aLine);

        // Create a 2D circle for testing
        Point2D aCenter = new Point2D(5.0, 5.0);
        Circle2D aCirc = new Circle2D(new Axis2D(aCenter, new Direction2D(1.0, 0.0)), 3.0);
        myCircle = new CircleCurve2D(aCirc);
    }

    [Fact]
    public void Load_ValidParameters_Success()
    {
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myLine, 0.0, 10.0);

        Assert.Equal(0.0, anAdaptor.FirstParameter());
        Assert.Equal(10.0, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_EqualParameters_Success()
    {
        // Degenerated curve: allowed, it represents a point
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myLine, 5.0, 5.0);

        Assert.Equal(5.0, anAdaptor.FirstParameter());
        Assert.Equal(5.0, anAdaptor.LastParameter());

        Point2D aP1 = anAdaptor.Value(5.0);
        Point2D aP2 = myLine.Value(5.0);
        Assert.True(aP1.IsEqual(aP2, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void Load_ParametersWithinConfusion_Success()
    {
        double aParam1 = 5.0;
        double aParam2 = 5.0 + global::Kerf.Engine.Foundation.Precision.Confusion() * 0.5;

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_ParametersAtConfusionBoundary_Success()
    {
        double aParam1 = 5.0;
        double aParam2 = 5.0 + global::Kerf.Engine.Foundation.Precision.Confusion();

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_FirstGreaterThanLastWithinConfusion_Success()
    {
        double aParam1 = 5.0 + global::Kerf.Engine.Foundation.Precision.Confusion() * 0.5;
        double aParam2 = 5.0;

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myLine, aParam1, aParam2);

        Assert.Equal(aParam1, anAdaptor.FirstParameter());
        Assert.Equal(aParam2, anAdaptor.LastParameter());
    }

    [Fact]
    public void Load_FirstGreaterThanLastBeyondConfusion_ThrowsException()
    {
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => anAdaptor.Load(myLine, 10.0, 5.0));
    }

    [Fact]
    public void Load_FirstSlightlyGreaterThanLast_ThrowsException()
    {
        double aParam1 = 5.0;
        double aParam2 = 5.0 - global::Kerf.Engine.Foundation.Precision.Confusion() * 2.0;

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => anAdaptor.Load(myLine, aParam1, aParam2));
    }

    [Fact]
    public void Constructor_DegeneratedCurve_Success()
    {
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myCircle, 0.0, 0.0);
        Assert.Equal(0.0, anAdaptor.FirstParameter());
        Assert.Equal(0.0, anAdaptor.LastParameter());
    }

    [Fact]
    public void Constructor_InvalidParameters_ThrowsException()
    {
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myCircle, 10.0, 0.0));
    }

    [Fact]
    public void Load_NullCurve_ThrowsException()
    {
        Curve2D aNullCurve = null!;
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();

        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NullObjectException>(() => anAdaptor.Load(aNullCurve, 0.0, 10.0));
    }

    [Fact]
    public void DegeneratedCurve_CircleAtZeroLength_Success()
    {
        double aParam = Math.PI;

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myCircle, aParam, aParam);

        Point2D aPoint = anAdaptor.Value(aParam);
        Point2D aExpectedPoint = myCircle.Value(aParam);

        Assert.True(aPoint.IsEqual(aExpectedPoint, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(anAdaptor.IsClosed || anAdaptor.FirstParameter() == anAdaptor.LastParameter());
    }

    [Fact]
    public void DegeneratedCurve_TrimmedCurve_Success()
    {
        TrimmedCurve2D aTrimmedCurve = new TrimmedCurve2D(myLine, 0.0, 20.0);

        double aParam = 10.0;
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(aTrimmedCurve, aParam, aParam);

        Assert.Equal(aParam, anAdaptor.FirstParameter());
        Assert.Equal(aParam, anAdaptor.LastParameter());
    }

    [Fact]
    public void ToleranceBoundary_NegativeCase_ThrowsException()
    {
        double aParam1 = 5.0;
        double aParam2 = 5.0 - global::Kerf.Engine.Foundation.Precision.Confusion() - 1e-10;

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => anAdaptor.Load(myLine, aParam1, aParam2));
    }

    [Fact]
    public void LoadWithoutParameters_Success()
    {
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D();
        anAdaptor.Load(myCircle);

        Assert.Equal(myCircle.FirstParameter, anAdaptor.FirstParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(myCircle.LastParameter, anAdaptor.LastParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(anAdaptor.IsPeriodic);
    }

    // Kerf additions: every curve type evaluates as the curve itself, and the type queries.

    private static BSplineCurve2D MakeBSpline()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 6);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 2);
        aPoles[3] = new Point2D(2, -1);
        aPoles[4] = new Point2D(3, 3);
        aPoles[5] = new Point2D(4, 0);
        aPoles[6] = new Point2D(5, 1);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
    }

    private static BezierCurve2D MakeBezier()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 2);
        aPoles[3] = new Point2D(2, -1);
        aPoles[4] = new Point2D(3, 1);
        NCollection_Array1<double> aWeights = new(1, 4);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 0.5;
        aWeights[4] = 1.0;
        return new BezierCurve2D(aPoles, aWeights);
    }

    public static TheoryData<string> CurveNames() =>
        new() { "line", "circle", "ellipse", "hyperbola", "parabola", "bezier", "bspline", "offset", "trimmed" };

    private Curve2D MakeCurve(string theName) => theName switch
    {
        "line" => myLine,
        "circle" => myCircle,
        "ellipse" => new EllipseCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 1)), 4.0, 2.0),
        "hyperbola" => new HyperbolaCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 1)), 4.0, 2.0),
        "parabola" => new ParabolaCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 1)), 1.5),
        "bezier" => MakeBezier(),
        "bspline" => MakeBSpline(),
        "offset" => new OffsetCurve2D(MakeBSpline(), 0.25),
        _ => new TrimmedCurve2D(myCircle, 0.5, 2.5),
    };

    [Theory]
    [MemberData(nameof(CurveNames))]
    public void Evaluation_MatchesCurve(string theName)
    {
        Curve2D aCurve = MakeCurve(theName);
        double aFirst = global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurve.FirstParameter) ? -2.0 : aCurve.FirstParameter;
        double aLast = global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurve.LastParameter) ? 2.0 : aCurve.LastParameter;
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(aCurve, aFirst, aLast);
        for (int i = 0; i <= 20; i++)
        {
            double U = aFirst + (aLast - aFirst) * i / 20.0;
            aCurve.D3(U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3);
            anAdaptor.D3(U, out Point2D AP, out Vector2D AV1, out Vector2D AV2, out Vector2D AV3);
            Assert.True(P.IsEqual(AP, 1e-9), $"{theName} D0 at {U}");
            Assert.True(V1.IsEqual(AV1, 1e-9, 1e-9) || (V1 - AV1).Magnitude < 1e-8, $"{theName} D1 at {U}");
            Assert.True((V2 - AV2).Magnitude < 1e-7, $"{theName} D2 at {U}");
            Assert.True((V3 - AV3).Magnitude < 1e-6, $"{theName} D3 at {U}: {V3} {AV3}");
            Assert.True(anAdaptor.Value(U).IsEqual(P, 1e-9));
            anAdaptor.D1(U, out AP, out AV1);
            Assert.True((V1 - AV1).Magnitude < 1e-8);
            // Compared with the curve's D2: OCCT's offset D3 computes a different second derivative (occt-issues.md).
            aCurve.D2(U, out P, out V1, out V2);
            anAdaptor.D2(U, out AP, out AV1, out AV2);
            Assert.True((V2 - AV2).Magnitude < 1e-7, $"{theName} D2 at {U}: {V2} {AV2}");
            Assert.True((aCurve.DN(U, 1) - anAdaptor.DN(U, 1)).Magnitude < 1e-8);
            Assert.True((aCurve.DN(U, 2) - anAdaptor.DN(U, 2)).Magnitude < 1e-7);
        }
    }

    [Theory]
    [MemberData(nameof(CurveNames))]
    public void ShallowCopy_EvaluatesAsOriginal(string theName)
    {
        Curve2D aCurve = MakeCurve(theName);
        double aFirst = global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurve.FirstParameter) ? -2.0 : aCurve.FirstParameter;
        double aLast = global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurve.LastParameter) ? 2.0 : aCurve.LastParameter;
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(aCurve, aFirst, aLast);
        global::Kerf.Engine.Geometry.Adaptors.Curve2DAdaptorBase aCopy = anAdaptor.ShallowCopy();
        Assert.IsType<global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D>(aCopy);
        Assert.Equal(anAdaptor.GetType(), aCopy.GetType());
        for (int i = 0; i <= 10; i++)
        {
            double U = aFirst + (aLast - aFirst) * i / 10.0;
            Assert.Equal(anAdaptor.Value(U), aCopy.Value(U));
        }
    }

    [Fact]
    public void Types_AndElementaryQueries()
    {
        Assert.Equal(Line, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myLine).GetType());
        Assert.Equal(Circle, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myCircle).GetType());
        Assert.Equal(Circle, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("trimmed")).GetType());
        Assert.Equal(Ellipse, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("ellipse")).GetType());
        Assert.Equal(Hyperbola, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("hyperbola")).GetType());
        Assert.Equal(Parabola, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("parabola")).GetType());
        Assert.Equal(BezierCurve, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeBezier()).GetType());
        Assert.Equal(BSplineCurve, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeBSpline()).GetType());
        Assert.Equal(OffsetCurve, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("offset")).GetType());
        Assert.Equal(OtherCurve, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D().GetType());

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D aCirc = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myCircle);
        Assert.Equal(3.0, aCirc.Circle().Radius);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCirc.Line());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCirc.Degree());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCirc.KnotCount);
        Assert.Equal(CN, aCirc.Continuity());
        Assert.Equal(1, aCirc.NbIntervals(C2));
        Assert.Equal(2 * Math.Asin(0.1 / 6.0), aCirc.Resolution(0.1));
        Assert.Equal(20, aCirc.SampleCount);
        Assert.Equal(2, new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myLine).SampleCount);
        Assert.Null(aCirc.BSpline());
        Assert.Null(aCirc.Bezier());

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D aBS = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeBSpline());
        Assert.Equal(3, aBS.Degree());
        Assert.Equal(6, aBS.PoleCount);
        Assert.Equal(3, aBS.KnotCount);
        Assert.False(aBS.IsRational);
        Assert.Equal(C1, aBS.Continuity());
        Assert.Equal(1, aBS.NbIntervals(C1));
        Assert.Equal(2, aBS.NbIntervals(C2));
        NCollection_Array1<double> T = new(1, 3);
        aBS.Intervals(T, C2);
        Assert.Equal(new[] { 0.0, 1.0, 2.0 }, new[] { T[1], T[2], T[3] });
        Assert.Equal(9, aBS.SampleCount);
        Assert.Same(aBS.Curve(), aBS.BSpline());

        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D aBz = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeBezier());
        Assert.True(aBz.IsRational);
        Assert.Equal(3, aBz.Degree());
        Assert.Equal(4, aBz.PoleCount);
        Assert.Equal(7, aBz.SampleCount);

        // Offset of a C1 B-spline: C0, and its C1 intervals are the basis curve's C2 intervals.
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anOff = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeCurve("offset"));
        Assert.Equal(C0, anOff.Continuity());
        Assert.Equal(2, anOff.NbIntervals(C1));
        NCollection_Array1<double> TO = new(1, 3);
        anOff.Intervals(TO, C1);
        Assert.Equal(new[] { 0.0, 1.0, 2.0 }, new[] { TO[1], TO[2], TO[3] });
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => anOff.NbIntervals(G1));
    }

    [Fact]
    public void Reset_And_Trim()
    {
        global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D anAdaptor = new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(MakeBSpline());
        Assert.True(anAdaptor.IsInitialized);
        global::Kerf.Engine.Geometry.Adaptors.Curve2DAdaptorBase aTrim = anAdaptor.Trim(0.5, 1.5, 1e-7);
        Assert.Equal(0.5, aTrim.FirstParameter());
        Assert.Equal(1.5, aTrim.LastParameter());
        Assert.Equal(anAdaptor.Value(0.5), aTrim.Value(0.5));
        anAdaptor.Reset();
        Assert.False(anAdaptor.IsInitialized);
        Assert.Equal(OtherCurve, anAdaptor.GetType());
        Assert.Null(anAdaptor.Curve());
    }

    [Fact]
    public void MakeCurve_RoundTrips()
    {
        // Elementary curves are rebuilt, then trimmed to the adaptor's range.
        Curve2D aC = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(myCircle, 0.5, 2.0));
        TrimmedCurve2D aT = Assert.IsType<TrimmedCurve2D>(aC);
        Assert.IsType<CircleCurve2D>(aT.BasisCurve);
        Assert.Equal(0.5, aT.FirstParameter);
        Assert.Equal(2.0, aT.LastParameter);

        // A B-spline over its whole range is returned as is.
        BSplineCurve2D aBS = MakeBSpline();
        Assert.Same(aBS, global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(aBS)));

        // A range beyond the curve's is cut to the curve's.
        Curve2D aCut = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(aBS, -1.0, 1.0));
        Assert.Equal(0.0, aCut.FirstParameter);
        Assert.Equal(1.0, aCut.LastParameter);

        // Offset curves need a Geom2dAdaptor_Curve.
        Curve2D anOff = MakeCurve("offset");
        Assert.Same(anOff, global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D(anOff)));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(new global::Kerf.Engine.Geometry.Adaptors.CurveAdaptor2D()));
    }
}
