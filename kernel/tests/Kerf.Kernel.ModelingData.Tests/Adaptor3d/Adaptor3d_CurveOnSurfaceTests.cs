// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Adaptor3d_CurveOnSurface against direct evaluation: the surface at the 2D curve's point, with derivatives by the
/// chain rule. OCCT has no GTest for this class.
/// </summary>
public class Adaptor3d_CurveOnSurfaceTests
{
    private const double Tol = 1e-9;

    private readonly struct Ref
    {
        public readonly Vector3D P, D1, D2, D3;

        public Ref(Vector3D p, Vector3D d1, Vector3D d2, Vector3D d3)
        {
            P = p;
            D1 = d1;
            D2 = d2;
            D3 = d3;
        }
    }

    /// <summary>
    /// S(C(t)) and its derivatives by the chain rule.
    /// </summary>
    private static Ref Reference(Surface theS, Curve2D theC, double t)
    {
        theC.D3(t, out Point2D uv, out Vector2D d1, out Vector2D d2, out Vector2D d3);
        double u = uv.X(), v = uv.Y();
        double u1 = d1.X(), v1 = d1.Y(), u2 = d2.X(), v2 = d2.Y(), u3 = d3.X(), v3 = d3.Y();
        Surface.ResD3 r = theS.EvalD3(u, v);
        Vector3D P = new Vector3D(r.Point.XYZ());
        Vector3D D1 = r.D1U * u1 + r.D1V * v1;
        Vector3D D2 = r.D2U * (u1 * u1) + r.D2UV * (2 * u1 * v1) + r.D2V * (v1 * v1) + r.D1U * u2 + r.D1V * v2;
        Vector3D common = r.D3U * (u1 * u1 * u1) + r.D3UUV * (3 * u1 * u1 * v1) + r.D3V * (v1 * v1 * v1)
                        + r.D2U * (3 * u1 * u2) + r.D2UV * (3 * (u1 * v2 + u2 * v1)) + r.D2V * (3 * v1 * v2) + r.D1V * v3;
        Vector3D D3 = common + r.D3UVV * (3 * u1 * v1 * v1) + r.D1U * u3;
        return new Ref(P, D1, D2, D3);
    }

    private static void Near(Vector3D theExpected, Vector3D theActual, string theWhat, double theTol = Tol)
    {
        double aScale = Math.Max(1.0, theExpected.Magnitude());
        Assert.True((theExpected - theActual).Magnitude() <= theTol * aScale,
                    $"{theWhat}: expected {theExpected}, got {theActual}");
    }

    /// <summary>Compares D0 to D2 (and D3, unless theSkipD3) with the reference at the ends, the knots and inside.</summary>
    private static void CheckEval(CurveOnSurface theCOS, Surface theS, Curve2D theC,
                                  IEnumerable<double> theParams, double theTol = Tol)
    {
        foreach (double t in theParams)
        {
            Ref r = Reference(theS, theC, t);
            Near(r.P, new Vector3D(theCOS.Value(t).XYZ()), $"D0({t})", theTol);
            theCOS.D1(t, out Point3D P1, out Vector3D V1);
            Near(r.P, new Vector3D(P1.XYZ()), $"D1 P({t})", theTol);
            Near(r.D1, V1, $"D1({t})", theTol);
            theCOS.D2(t, out Point3D P2, out Vector3D W1, out Vector3D W2);
            Near(r.P, new Vector3D(P2.XYZ()), $"D2 P({t})", theTol);
            Near(r.D1, W1, $"D2 V1({t})", theTol);
            Near(r.D2, W2, $"D2({t})", theTol);
            theCOS.D3(t, out Point3D P3, out Vector3D X1, out Vector3D X2, out Vector3D X3);
            Near(r.P, new Vector3D(P3.XYZ()), $"D3 P({t})", theTol);
            Near(r.D1, X1, $"D3 V1({t})", theTol);
            Near(r.D2, X2, $"D3 V2({t})", theTol);
            Near(r.D3, X3, $"D3({t})", theTol);
            Near(r.D1, theCOS.DN(t, 1), $"DN1({t})", theTol);
            Near(r.D2, theCOS.DN(t, 2), $"DN2({t})", theTol);
        }
    }

    private static IEnumerable<double> Params(double theFirst, double theLast, params double[] theExtra)
    {
        for (int i = 0; i <= 8; i++)
        {
            yield return theFirst + (theLast - theFirst) * i / 8.0;
        }
        foreach (double t in theExtra)
        {
            yield return t;
        }
    }

    private static CoordinateSystem SomeAxes() =>
        new CoordinateSystem(new Point3D(1, -2, 0.5), new Direction3D(0.3, 0.4, 1.0), new Direction3D(1.0, -0.3, 0.0));

    // A bicubic B-spline surface on [0, 3] x [0, 2], simple interior knots (C2).
    private static BSplineSurface MakeBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 6, 1, 5);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 5; j++)
            {
                aPoles[i, j] = new Point3D(i, j, Math.Sin(i * 1.3 + j * 0.7) + 0.1 * i * j);
            }
        }
        NCollection_Array1<double> aUKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0, 3.0 });
        NCollection_Array1<int> aUMults = NCollection_Array1<int>.From(new[] { 4, 1, 1, 4 });
        NCollection_Array1<double> aVKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0 });
        NCollection_Array1<int> aVMults = NCollection_Array1<int>.From(new[] { 4, 1, 4 });
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
    }

    private static BSplineCurve2D Make2dBSpline(double theScaleU, double theScaleV)
    {
        NCollection_Array1<Point2D> aPoles = new(1, 5);
        aPoles[1] = new Point2D(0.0 * theScaleU, 0.1 * theScaleV);
        aPoles[2] = new Point2D(0.6 * theScaleU, 0.9 * theScaleV);
        aPoles[3] = new Point2D(1.4 * theScaleU, 0.2 * theScaleV);
        aPoles[4] = new Point2D(2.2 * theScaleU, 0.7 * theScaleV);
        aPoles[5] = new Point2D(3.0 * theScaleU, 1.0 * theScaleV);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 0.5, 1.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 4 });
        return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
    }

    private static LineCurve2D Line2d(double theX, double theY, double theDX, double theDY) =>
        new LineCurve2D(new Point2D(theX, theY), new Direction2D(theDX, theDY));

    [Fact]
    public void Plane_Line_IsLine()
    {
        PlaneSurface aPlane = new PlaneSurface(SomeAxes());
        LineCurve2D aLine = Line2d(1.0, 2.0, 0.6, -0.8);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, -3.0, 4.0), new TestSurfaceAdaptor(aPlane));

        Assert.Equal(Line, aCOS.GetType());
        Line3D aLin = aCOS.Line();
        Assert.True(aLin.Location().IsEqual(aPlane.Value(1.0, 2.0), Tol));
        Assert.Equal(-3.0, aCOS.FirstParameter());
        Assert.Equal(4.0, aCOS.LastParameter());
        Assert.False(aCOS.IsPeriodic);
        CheckEval(aCOS, aPlane, aLine, Params(-3.0, 4.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCOS.Circle());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCOS.DN(0.5, 4));
    }

    [Fact]
    public void Plane_CircleAndEllipse()
    {
        PlaneSurface aPlane = new PlaneSurface(SomeAxes());
        CircleCurve2D aCircle = new CircleCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 1)), 2.5);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aCircle), new TestSurfaceAdaptor(aPlane));
        Assert.Equal(Circle, aCOS.GetType());
        Assert.Equal(2.5, aCOS.Circle().Radius(), 1e-12);
        Assert.True(aCOS.IsPeriodic);
        Assert.Equal(2 * Math.PI, aCOS.Period());
        Assert.True(aCOS.Circle().Location().IsEqual(aPlane.Value(1, 2), Tol));
        CheckEval(aCOS, aPlane, aCircle, Params(0.0, 2 * Math.PI));

        EllipseCurve2D anEllipse = new EllipseCurve2D(new Axis2D(new Point2D(-1, 0.5), new Direction2D(1, -2)), 3.0, 1.0);
        var aCOSE = new CurveOnSurface(new CurveAdaptor2D(anEllipse), new TestSurfaceAdaptor(aPlane));
        Assert.Equal(Ellipse, aCOSE.GetType());
        Ellipse3D anE = aCOSE.Ellipse();
        Assert.Equal(3.0, anE.MajorRadius(), 1e-12);
        Assert.Equal(1.0, anE.MinorRadius(), 1e-12);
        Assert.True(aCOSE.IsPeriodic);
        // An ellipse goes through the general path; on a plane the second derivatives vanish, so D3 is exact
        // except for OCCT's u' in place of u''' (see Reference).
        CheckEval(aCOSE, aPlane, anEllipse, Params(0.0, 2 * Math.PI));
        for (int i = 0; i < 8; i++)
        {
            double t = i * 0.7;
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(t, anE).IsEqual(aCOSE.Value(t), Tol));
        }

        HyperbolaCurve2D aHyp = new HyperbolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0, 1.0);
        var aCOSH = new CurveOnSurface(new CurveAdaptor2D(aHyp, -1, 1), new TestSurfaceAdaptor(aPlane));
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(0.3, aCOSH.Hyperbola()).IsEqual(aCOSH.Value(0.3), Tol));
        ParabolaCurve2D aPar = new ParabolaCurve2D(new Axis2D(new Point2D(0, 1), new Direction2D(0, 1)), 0.5);
        var aCOSP = new CurveOnSurface(new CurveAdaptor2D(aPar, -1, 1), new TestSurfaceAdaptor(aPlane));
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(0.3, aCOSP.Parabola()).IsEqual(aCOSP.Value(0.3), Tol));
    }

    public static TheoryData<string, double, double, double, double> IsoLines() => new()
    {
        // surface, 2D line location (x, y) and direction (dx, dy): V isos (dx = +-1) and U isos (dy = +-1)
        { "cylinder", 0.7, 1.5, 1, 0 }, { "cylinder", 0.7, 1.5, -1, 0 }, { "cylinder", 0.7, 1.5, 0, 1 }, { "cylinder", 0.7, 1.5, 0, -1 },
        { "cone", 0.7, 1.5, 1, 0 }, { "cone", 0.7, 1.5, -1, 0 }, { "cone", 0.7, 1.5, 0, 1 }, { "cone", 0.7, 1.5, 0, -1 },
        { "sphere", 0.7, 0.4, 1, 0 }, { "sphere", 0.7, 0.4, -1, 0 }, { "sphere", 0.7, 0.4, 0, 1 }, { "sphere", 0.7, 0.4, 0, -1 },
        { "torus", 0.7, 0.4, 1, 0 }, { "torus", 0.7, 0.4, -1, 0 }, { "torus", 0.7, 0.4, 0, 1 }, { "torus", 0.7, 0.4, 0, -1 },
    };

    private static Surface Elementary(string theName) => theName switch
    {
        "cylinder" => new CylindricalSurface(SomeAxes(), 2.0),
        "cone" => new ConicalSurface(SomeAxes(), 0.4, 1.5),
        "sphere" => new SphericalSurface(SomeAxes(), 2.0),
        _ => new ToroidalSurface(SomeAxes(), 3.0, 1.0),
    };

    [Theory]
    [MemberData(nameof(IsoLines))]
    public void ElementarySurfaces_IsoLines_AreLinesAndCircles(string theSurface, double X, double Y, double DX, double DY)
    {
        Surface aSurf = Elementary(theSurface);
        LineCurve2D aLine = Line2d(X, Y, DX, DY);
        double aFirst = -0.5, aLast = 0.5;
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, aFirst, aLast), new TestSurfaceAdaptor(aSurf));

        CurveKind anExpected = (DX == 0 && (theSurface == "cylinder" || theSurface == "cone")) ? Line : Circle;
        Assert.Equal(anExpected, aCOS.GetType());
        // KPart: the line and circle evaluate as the surface on the 2D line (same parametrization)
        CheckEval(aCOS, aSurf, aLine, Params(aFirst, aLast));
        if (anExpected == Circle)
        {
            Assert.True(aCOS.IsPeriodic);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(0.3, aCOS.Circle()).IsEqual(aSurf.Value(X + 0.3 * DX, Y + 0.3 * DY), Tol));
        }
        else
        {
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(0.3, aCOS.Line()).IsEqual(aSurf.Value(X + 0.3 * DX, Y + 0.3 * DY), Tol));
        }
    }

    [Fact]
    public void Sphere_IsoVAtPole_StaysGeneral()
    {
        // A V iso through the pole of a sphere is a point: not turned into a circle
        SphericalSurface aSph = new SphericalSurface(SomeAxes(), 2.0);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(Line2d(0, Math.PI / 2, 1, 0), 0, 1),
                                                new TestSurfaceAdaptor(aSph));
        Assert.Equal(OtherCurve, aCOS.GetType());
        Assert.True(aCOS.Value(0.5).IsEqual(aSph.Value(0.5, Math.PI / 2), Tol));
    }

    [Fact]
    public void ObliqueLine_OnCylinder_GeneralPath()
    {
        CylindricalSurface aCyl = new CylindricalSurface(SomeAxes(), 2.0);
        LineCurve2D aLine = Line2d(0.3, -1.0, 1, 2);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, -1, 2), new TestSurfaceAdaptor(aCyl));
        Assert.Equal(OtherCurve, aCOS.GetType());
        // A 2D line has u''' = 0 but u' != 0, and Suvv = 0 on a cylinder: OCCT's D3 adds Su * u'.
        CheckEval(aCOS, aCyl, aLine, Params(-1, 2));
    }

    [Fact]
    public void BSplineSurface_BSplineCurve_EndsAndKnots()
    {
        BSplineSurface aSurf = MakeBSplineSurface();
        BSplineCurve2D aC = Make2dBSpline(1.0, 2.0);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aC), new TestSurfaceAdaptor(aSurf));
        Assert.Equal(OtherCurve, aCOS.GetType());
        // The ends go through myFirstSurf / myLastSurf (patches located by LocatePart)
        CheckEval(aCOS, aSurf, aC, Params(0.0, 1.0, 0.5, 1e-11, 1.0 - 1e-11), theTol: 1e-8);
        Assert.True(aCOS.IsRational == false);
        Assert.Equal(3, aCOS.Degree());
        Assert.Equal(5, aCOS.PoleCount);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCOS.KnotCount);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCOS.BSpline());
    }

    public static TheoryData<double, double, double, double> BSplineIsoLines() => new()
    {
        // start (u, v) and direction: starting on knots, along and across them, forwards and backwards
        { 1.0, 0.5, 0, 1 },  // U iso on a U knot: DU is null, Locate2Coord(1)
        { 1.0, 0.5, 0, -1 },
        { 0.5, 1.0, 1, 0 },  // V iso on a V knot
        { 2.5, 1.0, -1, 0 },
        { 0.0, 0.0, 1, 1 },  // from the corner
        { 3.0, 2.0, -1, -0.5 }, // from the opposite corner
        { 0.5, 0.5, 1, 0.3 },  // inside a span
        { 3.0, 0.5, 0, 1 },  // U iso on the last U knot
    };

    [Theory]
    [MemberData(nameof(BSplineIsoLines))]
    public void BSplineSurface_Lines_LocatePart(double U, double V, double DU, double DV)
    {
        BSplineSurface aSurf = MakeBSplineSurface();
        LineCurve2D aLine = Line2d(U, V, DU, DV);
        double aLen = 0.4;
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, 0.0, aLen), new TestSurfaceAdaptor(aSurf));
        // A 2D line on a non-planar surface: OCCT's D3 formula (see Reference)
        CheckEval(aCOS, aSurf, aLine, Params(0.0, aLen), theTol: 1e-8);
    }

    [Fact]
    public void SurfaceOfRevolutionAndExtrusion_LocatePartRevExt()
    {
        // A planar cubic B-spline in the XZ plane, revolved around Z and extruded along a direction
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(1.0, 0, 0);
        aPoles[2] = new Point3D(2.0, 0, 1);
        aPoles[3] = new Point3D(1.5, 0, 2);
        aPoles[4] = new Point3D(2.5, 0, 3);
        aPoles[5] = new Point3D(1.0, 0, 4);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 1.0, 2.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 1, 4 });
        BSplineCurve3D aBasis = new BSplineCurve3D(aPoles, aKnots, aMults, 3);

        RevolutionSurface aRev = new RevolutionSurface(aBasis, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        ExtrusionSurface anExt = new ExtrusionSurface(aBasis, new Direction3D(0.2, 1, 0.3));

        // Revolution: U is the angle, V the basis parameter; extrusion: U the basis parameter, V the length
        foreach (var (aSurf, aSA, aLine, aRange) in new (Surface, TestSurfaceAdaptor, LineCurve2D, double)[]
                 {
                     (aRev, new TestSurfaceAdaptor(aRev, 0, 2 * Math.PI, 0, 2), Line2d(0.5, 1.0, 0.3, 1), 0.9),
                     (aRev, new TestSurfaceAdaptor(aRev, 0, 2 * Math.PI, 0, 2), Line2d(0.5, 0.3, 1, 0), 1.0),
                     (aRev, new TestSurfaceAdaptor(aRev, 0, 2 * Math.PI, 0, 2), Line2d(0.5, 1.7, 0.2, -1), 1.5),
                     (anExt, new TestSurfaceAdaptor(anExt, 0, 2, -5, 5), Line2d(1.0, 0.0, 1, 0.5), 0.8),
                     (anExt, new TestSurfaceAdaptor(anExt, 0, 2, -5, 5), Line2d(0.3, -1.0, 0, 1), 2.0),
                     (anExt, new TestSurfaceAdaptor(anExt, 0, 2, -5, 5), Line2d(2.0, 1.0, -1, 0.2), 1.5),
                 })
        {
            var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, 0, aRange), aSA);
            Assert.Equal(OtherCurve, aCOS.GetType());
            CheckEval(aCOS, aSurf, aLine, Params(0, aRange), theTol: 1e-8);
        }
    }

    // A surface adaptor whose U range is split at its middle (C1 there), to check the curve's intervals
    private sealed class SplitSurface(SurfaceAdaptorBase theBase, double theUSplit) : SurfaceAdaptorBase
    {
        public override SurfaceAdaptorBase ShallowCopy() => new SplitSurface(theBase.ShallowCopy(), theUSplit);
        public override double FirstUParameter() => theBase.FirstUParameter();
        public override double LastUParameter() => theBase.LastUParameter();
        public override double FirstVParameter() => theBase.FirstVParameter();
        public override double LastVParameter() => theBase.LastVParameter();
        public override Continuity UContinuity() => C1;
        public override Continuity VContinuity() => theBase.VContinuity();
        public override int NbUIntervals(Continuity S) => S >= C2 ? 2 : 1;
        public override int NbVIntervals(Continuity S) => 1;

        public override void UIntervals(NCollection_Array1<double> T, Continuity S)
        {
            T[1] = FirstUParameter();
            if (S >= C2)
            {
                T[2] = theUSplit;
                T[3] = LastUParameter();
            }
            else
            {
                T[2] = LastUParameter();
            }
        }

        public override void VIntervals(NCollection_Array1<double> T, Continuity S) => theBase.VIntervals(T, S);
        public override SurfaceKind GetType() => SurfaceKind.OtherSurface;
        public override double UResolution(double R3d) => theBase.UResolution(R3d);
        public override double VResolution(double R3d) => theBase.VResolution(R3d);
        public override bool IsURational => false;
        public override bool IsVRational => false;
        public override Point3D EvalD0(double theU, double theV) => theBase.EvalD0(theU, theV);
        public override Surface.ResD1 EvalD1(double theU, double theV) => theBase.EvalD1(theU, theV);
        public override Surface.ResD2 EvalD2(double theU, double theV) => theBase.EvalD2(theU, theV);
        public override Surface.ResD3 EvalD3(double theU, double theV) => theBase.EvalD3(theU, theV);
        public override Vector3D EvalDN(double theU, double theV, int theNu, int theNv) => theBase.EvalDN(theU, theV, theNu, theNv);
    }

    [Fact]
    public void Intervals_SplitWhereTheCurveCrossesSurfaceBounds()
    {
        PlaneSurface aPlane = new PlaneSurface(SomeAxes());
        // A curve C1 at its knot 0.5, crossing u = 1.2 twice
        NCollection_Array1<Point2D> aPoles = new(1, 6);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 0);
        aPoles[4] = new Point2D(3, 1);
        aPoles[5] = new Point2D(1, 2);
        aPoles[6] = new Point2D(0, 3);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(new[] { 0.0, 0.5, 1.0 });
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From(new[] { 4, 2, 4 });
        BSplineCurve2D aC = new BSplineCurve2D(aPoles, aKnots, aMults, 3);
        var aCurve = new CurveAdaptor2D(aC);
        var aSurf = new SplitSurface(new TestSurfaceAdaptor(aPlane, -10, 10, -10, 10), 1.2);
        var aCOS = new CurveOnSurface(aCurve, aSurf);

        Assert.Equal(C1, aCOS.Continuity());
        Assert.Equal(1, aCOS.NbIntervals(C1));
        int aNb = aCOS.NbIntervals(C2);
        Assert.Equal(4, aNb); // the knot 0.5 and two crossings of u = 1.2
        NCollection_Array1<double> T = new(1, aNb + 1);
        aCOS.Intervals(T, C2);
        Assert.Equal(0.0, T[1]);
        Assert.Equal(1.0, T[aNb + 1]);
        int aCrossings = 0;
        bool aKnot = false;
        for (int i = 2; i <= aNb; i++)
        {
            Assert.True(T[i] > T[i - 1]);
            if (Math.Abs(T[i] - 0.5) < 1e-12)
            {
                aKnot = true;
            }
            else
            {
                Assert.Equal(1.2, aC.Value(T[i]).X(), 1e-8);
                aCrossings++;
            }
        }
        Assert.True(aKnot);
        Assert.Equal(2, aCrossings);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ProgramErrorException>(() => aCOS.Intervals(new NCollection_Array1<double>(1, 2), C2));

        // Cached per continuity
        Assert.Equal(1, aCOS.NbIntervals(C1));
        Assert.Equal(aNb, aCOS.NbIntervals(C2));
    }

    [Fact]
    public void Trim_ShallowCopy_Load()
    {
        BSplineSurface aSurf = MakeBSplineSurface();
        BSplineCurve2D aC = Make2dBSpline(1.0, 2.0);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aC), new TestSurfaceAdaptor(aSurf));

        CurveAdaptorBase aTrim = aCOS.Trim(0.2, 0.7, 1e-7);
        Assert.IsType<CurveOnSurface>(aTrim);
        Assert.Equal(0.2, aTrim.FirstParameter());
        Assert.Equal(0.7, aTrim.LastParameter());
        Assert.True(aTrim.Value(0.2).IsEqual(aCOS.Value(0.2), Tol));
        Assert.True(aTrim.Value(0.45).IsEqual(aCOS.Value(0.45), Tol));

        CurveAdaptorBase aCopy = aCOS.ShallowCopy();
        Assert.Equal(aCOS.GetType(), aCopy.GetType());
        foreach (double t in Params(0, 1))
        {
            Assert.True(aCopy.Value(t).IsEqual(aCOS.Value(t), 0.0));
        }

        // Load in two steps, in either order
        var aCOS2 = new CurveOnSurface(new TestSurfaceAdaptor(aSurf));
        Assert.Null(aCOS2.GetCurve());
        aCOS2.Load(new CurveAdaptor2D(aC));
        Assert.True(aCOS2.Value(0.3).IsEqual(aCOS.Value(0.3), 0.0));
        var aCOS3 = new CurveOnSurface();
        aCOS3.Load(new CurveAdaptor2D(aC), new TestSurfaceAdaptor(aSurf));
        Assert.True(aCOS3.Value(0.3).IsEqual(aCOS.Value(0.3), 0.0));
        Assert.NotNull(aCOS3.GetSurface());

        // ChangeCurve gives the stored reference
        ref Curve2DAdaptorBase? aRef = ref aCOS3.ChangeCurve();
        Assert.Same(aCOS3.GetCurve(), aRef);
        Assert.Same(aCOS3.GetSurface(), aCOS3.ChangeSurface());
    }

    [Fact]
    public void Plane_BezierAndBSpline_To3d()
    {
        PlaneSurface aPlane = new PlaneSurface(SomeAxes());
        BSplineCurve2D aBS = Make2dBSpline(1.0, 1.0);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aBS), new TestSurfaceAdaptor(aPlane));
        Assert.Equal(BSplineCurve, aCOS.GetType());
        Assert.Equal(3, aCOS.KnotCount);
        BSplineCurve3D a3d = aCOS.BSpline();
        Assert.Equal(3, a3d.Degree());
        foreach (double t in Params(0, 1))
        {
            Assert.True(a3d.Value(t).IsEqual(aPlane.Value(aBS.Value(t).X(), aBS.Value(t).Y()), Tol));
        }
        CheckEval(aCOS, aPlane, aBS, Params(0, 1, 0.5));

        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 2);
        aPoles[3] = new Point2D(2, -1);
        aPoles[4] = new Point2D(3, 1);
        NCollection_Array1<double> aWeights = NCollection_Array1<double>.From(new[] { 1.0, 2.0, 0.5, 1.0 });
        BezierCurve2D aBz = new BezierCurve2D(aPoles, aWeights);
        var aCOSB = new CurveOnSurface(new CurveAdaptor2D(aBz), new TestSurfaceAdaptor(aPlane));
        Assert.Equal(BezierCurve, aCOSB.GetType());
        Assert.True(aCOSB.IsRational);
        BezierCurve3D aBz3 = aCOSB.Bezier();
        Assert.True(aBz3.IsRational);
        foreach (double t in Params(0, 1))
        {
            Assert.True(aBz3.Value(t).IsEqual(aCOSB.Value(t), Tol));
        }

        var aCOSCyl = new CurveOnSurface(new CurveAdaptor2D(aBz),
                                                   new TestSurfaceAdaptor(new CylindricalSurface(SomeAxes(), 1.0)));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCOSCyl.Bezier());
    }

    [Fact]
    public void Resolution_UsesSurfaceThenCurve()
    {
        PlaneSurface aPlane = new PlaneSurface(SomeAxes());
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(Line2d(0, 0, 1, 0), 0, 1), new TestSurfaceAdaptor(aPlane));
        // TestSurfaceAdaptor's resolutions are Precision::Parametric (R * 0.01); a 2D line's is the identity
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Parametric(0.5), aCOS.Resolution(0.5));
    }

    [Fact]
    public void EvalD3_FollowsTheChainRule()
    {
        // occt-issues.md, Adaptor3d_CurveOnSurface::EvalD3 (Kerf decision 1488): OCCT's general D3 weighted Su by u'
        // instead of u''' and Suvv by 5u'v'^2 instead of 3u'v'^2, off by about 0.9 here. D3 is now the chain rule's,
        // which a finite difference of D2 agrees with.
        CylindricalSurface aCyl = new CylindricalSurface(SomeAxes(), 2.0);
        LineCurve2D aLine = Line2d(0.3, -1.0, 1, 2);
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, -1, 2), new TestSurfaceAdaptor(aCyl));
        double t = 0.4, h = 1e-4;
        Vector3D aFD = (aCOS.DN(t + h, 2) - aCOS.DN(t - h, 2)) / (2 * h);
        Ref r = Reference(aCyl, aLine, t);
        Near(r.D3, aFD, "chain rule vs finite difference", 1e-6);
        Near(aFD, aCOS.DN(t, 3), "D3 vs finite difference", 1e-6);
        Near(r.D3, aCOS.DN(t, 3), "D3 vs the chain rule");
    }
}
