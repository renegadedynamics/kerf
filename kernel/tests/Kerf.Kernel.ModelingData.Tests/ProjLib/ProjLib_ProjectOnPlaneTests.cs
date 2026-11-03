// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// ProjLib_ProjectOnPlane and ProjLib_ProjectOnSurface: projections of lines, conics, Bezier, B-spline and offset
/// curves on planes along their normal and obliquely, with and without the parametrization kept; the result types,
/// points and derivatives against the projection of the curve's own; the adaptor's queries; frozen results; and
/// the OCCT issues reproduced.
/// </summary>
public class ProjLib_ProjectOnPlaneTests
{
    private const double Tol = 1.0e-7;

    // A plane through (1, 2, 3) tilted by 30 degrees about X.
    private static CoordinateSystem TiltedPlane()
    {
        double a = M_PI / 6;
        return new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, -Math.Sin(a), Math.Cos(a)), new Direction3D(1, 0, 0));
    }

    // A plane tilted about an axis that is not one of the curve's.
    private static CoordinateSystem SkewPlane()
    {
        return new CoordinateSystem(new Point3D(0.5, -1, 2), new Direction3D(0.3, -0.4, 1), new Direction3D(1, 0, -0.3));
    }

    private static Point3D ProjectPnt(in CoordinateSystem thePl, in Direction3D theD, in Point3D theP)
    {
        Vector3D aPO = new(theP, thePl.Location());
        double anAlpha = (aPO * new Vector3D(thePl.Direction())) / (theD * thePl.Direction());
        return theP.Translated(anAlpha * new Vector3D(theD));
    }

    private static Vector3D ProjectVec(in CoordinateSystem thePl, in Direction3D theD, in Vector3D theV)
    {
        double anAlpha = (theV * new Vector3D(thePl.Direction())) / (theD * thePl.Direction());
        return theV - anAlpha * new Vector3D(theD);
    }

    private static double DistanceToPlane(in CoordinateSystem thePl, in Point3D theP)
    {
        return Math.Abs(new Vector3D(thePl.Location(), theP) * new Vector3D(thePl.Direction()));
    }

    private static double DistanceToCurve(Curve3D theC, in Point3D theP)
    {
        CurvePointExtrema anExt = new(theP, new CurveAdaptor(theC));
        double aMin = double.MaxValue;
        for (int i = 1; anExt.IsDone && i <= anExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, Math.Sqrt(anExt.SquareDistance(i)));
        }
        return aMin;
    }

    // The projection keeps the curve's parameters: the point at u is the projection of C(u).
    private static void AssertKeepsParameters(ProjectOnPlane theProj, CurveAdaptorBase theC, double theTol, bool theDerivs = true)
    {
        CoordinateSystem aPl = theProj.GetPlane();
        Direction3D aD = theProj.GetDirection();
        double f = theProj.FirstParameter(), l = theProj.LastParameter();
        Assert.Equal(theC.FirstParameter(), f);
        Assert.Equal(theC.LastParameter(), l);
        for (int i = 0; i <= 16; i++)
        {
            double u = f + (l - f) * i / 16;
            Point3D anExp = ProjectPnt(aPl, aD, theC.Value(u));
            Assert.True(theProj.Value(u).Distance(anExp) < theTol, $"point at {u}");
            if (theDerivs)
            {
                theC.D1(u, out _, out Vector3D aCD1);
                theProj.D1(u, out _, out Vector3D aD1);
                Assert.True((aD1 - ProjectVec(aPl, aD, aCD1)).Magnitude() < theTol * 100, $"D1 at {u}");
            }
        }
    }

    // The projection lies on the plane and goes through the projection of every point of the curve.
    private static void AssertSameImage(ProjectOnPlane theProj, CurveAdaptorBase theC, double theTol)
    {
        CoordinateSystem aPl = theProj.GetPlane();
        Direction3D aD = theProj.GetDirection();
        Curve3D aRes = theProj.GetResult()!.Curve()!;
        double f = theProj.FirstParameter(), l = theProj.LastParameter();
        for (int i = 0; i <= 16; i++)
        {
            double u = f + (l - f) * i / 16;
            Assert.True(DistanceToPlane(aPl, theProj.Value(u)) < theTol, $"on plane at {u}");
            double t = theC.FirstParameter() + (theC.LastParameter() - theC.FirstParameter()) * i / 16;
            Point3D aP = ProjectPnt(aPl, aD, theC.Value(t));
            Assert.True(DistanceToCurve(aRes, aP) < theTol, $"image of {t}");
        }
        // The bounds are the projections of the curve's ends.
        Assert.True(theProj.Value(f).Distance(ProjectPnt(aPl, aD, theC.Value(theC.FirstParameter()))) < theTol);
        Assert.True(theProj.Value(l).Distance(ProjectPnt(aPl, aD, theC.Value(theC.LastParameter()))) < theTol);
    }

    [Fact]
    public void Line_Oblique_NoKeep_IsTrimmedLine()
    {
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 1)), -2.0, 3.0);
        ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Line, aProj.GetType());
        Assert.False(aProj.IsPeriodic);
        Line3D aLin = aProj.Line();
        Assert.True(aLin.Direction().IsNormal(TiltedPlane().Direction(), 1.0e-12));
        AssertSameImage(aProj, aC, 1.0e-9);
        Assert.Throws<NoSuchObjectException>(() => aProj.Circle());
        Assert.Throws<NoSuchObjectException>(() => aProj.Degree());
        Assert.Throws<NoSuchObjectException>(() => aProj.BSpline());
    }

    [Fact]
    public void Line_Oblique_Keep_IsDegreeOneBSpline()
    {
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 1)), -2.0, 3.0);
        ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        BSplineCurve3D aBS = aProj.BSpline();
        Assert.Equal(1, aBS.Degree());
        Assert.Equal(-2.0, aBS.FirstParameter(), 1.0e-12);
        Assert.Equal(3.0, aBS.LastParameter(), 1.0e-12);
        AssertKeepsParameters(aProj, aC, 1.0e-9);
    }

    [Fact]
    public void Line_ParallelToPlane_KeepsLine()
    {
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(0, 0, 7), new Direction3D(1, 2, 0)), -1.0, 4.0);
        ProjectOnPlane aProj = new(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Line, aProj.GetType());
        Assert.Equal(-1.0, aProj.FirstParameter());
        Assert.Equal(4.0, aProj.LastParameter());
        Point3D aP = aProj.Value(2.0);
        Point3D anExp = aC.Value(2.0);
        Assert.Equal(anExp.X(), aP.X(), 1.0e-12);
        Assert.Equal(anExp.Y(), aP.Y(), 1.0e-12);
        Assert.Equal(1.0, aP.Z(), 1.0e-12);
    }

    [Fact]
    public void Line_AlongDirection_IsPoint()
    {
        CurveAdaptor aC = new(new LineCurve3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 0.0, 5.0);
        ProjectOnPlane aProj = new(new CoordinateSystem(new Point3D(0, 0, -1), new Direction3D(0, 0, 1)));
        aProj.Load(aC, Tol);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        BSplineCurve3D aBS = aProj.BSpline();
        Assert.Equal(2, aBS.PoleCount);
        for (double u = 0.0; u <= 5.0; u += 1.0)
        {
            Point3D aP = aProj.Value(u);
            Assert.True(aP.Distance(new Point3D(1, 2, -1)) < 1.0e-12);
        }
    }

    [Fact]
    public void Circle_ParallelToPlane_IsCircle()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(1, 1, 5), new Direction3D(0, 0, 1)), 2.0), 0.5, 4.0);
        ProjectOnPlane aProj = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Circle, aProj.GetType());
        Assert.Equal(2.0, aProj.Circle().Radius(), 1.0e-9);
        Assert.Equal(0.0, aProj.Circle().Location().Z(), 1.0e-12);
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Circle_TiltedPlane_NoKeep_IsEllipse()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0));
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Ellipse, aProj.GetType());
        Ellipse3D anE = aProj.Ellipse();
        Assert.Equal(3.0, anE.MajorRadius(), 1.0e-9);
        double aCos = SkewPlane().Direction().Z();
        Assert.Equal(3.0 * aCos, anE.MinorRadius(), 1.0e-9);
        AssertSameImage(aProj, aC, 1.0e-7);
        Assert.True(aProj.IsPeriodic);
        Assert.Equal(2 * M_PI, aProj.Period(), 1.0e-12);
    }

    [Fact]
    public void Circle_TiltedAboutAxis_Keep_IsEllipseWithSameParameters()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3.0));
        ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(CurveKind.Ellipse, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-9);
    }

    [Fact]
    public void Circle_SkewPlane_Keep_IsApproximated()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3.0));
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.False(aProj.IsPeriodic);
        Assert.Throws<NoSuchObjectException>(() => aProj.Period());
        Assert.Equal(aProj.BSpline().Degree(), aProj.Degree());
        Assert.Equal(aProj.BSpline().PoleCount, aProj.PoleCount);
        Assert.Equal(aProj.BSpline().KnotCount, aProj.KnotCount);
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Circle_EdgeOn_IsApproximatedSegment()
    {
        // The plane's normal lies in the circle's plane: the circle projects onto a segment.
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0), 0.2, 2.5);
        ProjectOnPlane aProj = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Ellipse_SkewPlane_NoKeep_IsEllipse()
    {
        CurveAdaptor aC = new(new EllipseCurve3D(new Frame3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 1, 0)), 4.0, 1.5), 0.3, 5.0);
        ProjectOnPlane aProj = new(SkewPlane(), new Direction3D(0.1, 0.2, 1));
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Ellipse, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-7);
        // Oblique direction: every projected point is the curve's point moved along it.
        Point3D aP = aProj.Value(aProj.FirstParameter());
        Vector3D aV = new(aC.Value(0.3), aP);
        Assert.True(aV.IsParallel(new Vector3D(0.1, 0.2, 1), 1.0e-9));
    }

    [Fact]
    public void Parabola_TiltedAboutItsYAxis_IsParabola()
    {
        // The projection shortens the parabola's axis direction only: the focal becomes f / cos.
        double a = M_PI / 6;
        CoordinateSystem aPl = new(new Point3D(1, 2, 3), new Direction3D(-Math.Sin(a), 0, Math.Cos(a)), new Direction3D(Math.Cos(a), 0, Math.Sin(a)));
        CurveAdaptor aC = new(new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 4), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjectOnPlane aProj = new(aPl);
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Parabola, aProj.GetType());
        Assert.Equal(0.5 / Math.Cos(a), aProj.Parabola().Focal(), 1.0e-12);
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Parabola_TiltedAboutItsXAxis_WrongFocal_OcctBug()
    {
        // OCCT: the focal of the projected parabola is Focal / |f(Xc)|, which ignores the shortening of the Y
        // direction: here the image is the parabola of focal 0.5 * cos^2, but OCCT builds focal 0.5.
        double a = M_PI / 6;
        CurveAdaptor aC = new(new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 4), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Parabola, aProj.GetType());
        Assert.Equal(0.5, aProj.Parabola().Focal(), 1.0e-12);
        Assert.NotEqual(0.5 * Math.Cos(a) * Math.Cos(a), aProj.Parabola().Focal(), 1.0e-3);
        Point3D anImage = ProjectPnt(TiltedPlane(), TiltedPlane().Direction(), aC.Value(-3.0));
        Assert.True(DistanceToCurve(aProj.GetResult()!.Curve()!, anImage) > 1.0e-2);
    }

    [Fact]
    public void Parabola_SkewPlane_NoKeep_BuiltByApex()
    {
        CurveAdaptor aC = new(new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Parabola, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-5);
    }

    [Fact]
    public void Parabola_SkewPlane_Keep_IsApproximated()
    {
        CurveAdaptor aC = new(new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.Equal(2, aProj.Degree());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Parabola_ProjectedToLine_OcctDereferencesNull()
    {
        // OCCT: the parabola's axis is along the projection, so the image is a line; Load builds it but passes the
        // null parabola to GetTrimmedResult (a crash in OCCT, NullReferenceException in the port).
        CurveAdaptor aC = new(new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjectOnPlane aProj = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.Throws<NullReferenceException>(() => aProj.Load(aC, Tol, false));
    }

    [Fact]
    public void Hyperbola_NormalProjection_IsHyperbola()
    {
        CurveAdaptor aC = new(new HyperbolaCurve3D(new Hyperbola3D(new Frame3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Hyperbola, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Hyperbola_SkewPlane_NoKeep_BuiltByApex()
    {
        CurveAdaptor aC = new(new HyperbolaCurve3D(new Hyperbola3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.Hyperbola, aProj.GetType());
        double f = aProj.FirstParameter(), l = aProj.LastParameter();
        for (int i = 0; i <= 8; i++)
        {
            Assert.True(DistanceToPlane(SkewPlane(), aProj.Value(f + (l - f) * i / 8)) < 1.0e-7);
        }
    }

    [Fact]
    public void Hyperbola_SkewPlane_Keep_IsApproximated()
    {
        CurveAdaptor aC = new(new HyperbolaCurve3D(new Hyperbola3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Bezier_ProjectsPoles_FrozenResult_CallerUnchanged()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 3);
        aPoles[3] = new Point3D(2, -1, 1);
        aPoles[4] = new Point3D(4, 0, 2);
        NCollection_Array1<double> aWeights = new(1, 4);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 0.5;
        aWeights[4] = 1.0;
        BezierCurve3D aBz = new(aPoles, aWeights);
        CurveAdaptor aC = new(aBz);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.BezierCurve, aProj.GetType());
        Assert.True(aProj.IsRational);
        Assert.Equal(3, aProj.Degree());
        AssertKeepsParameters(aProj, aC, 1.0e-9);

        BezierCurve3D aRes = aProj.Bezier();
        Assert.True(aRes.IsFrozen);
        Assert.Same(aRes, aProj.Bezier());
        Assert.NotSame(aBz, aRes);
        Assert.False(aBz.IsFrozen);
        Assert.Equal(new Point3D(1, 2, 3), aBz.Pole(2));
    }

    [Fact]
    public void BSpline_ProjectsPoles_Derivatives()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new Point3D(i, Math.Sin(i), 0.3 * i * i);
        }
        NCollection_Array1<double> aWeights = new(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aWeights[i] = 1.0 + 0.1 * i;
        }
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.4;
        aKnots[3] = 1.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        BSplineCurve3D aBS = new(aPoles, aWeights, aKnots, aMults, 3);
        CurveAdaptor aC = new(aBS);
        ProjectOnPlane aProj = new(TiltedPlane(), new Direction3D(0.2, 0.1, 1));
        aProj.Load(aC, Tol);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.True(aProj.IsRational);
        Assert.Equal(3, aProj.KnotCount);
        AssertKeepsParameters(aProj, aC, 1.0e-9);

        // D2, D3 and DN of the result: the projected derivatives (the projection is affine).
        for (int i = 0; i <= 8; i++)
        {
            double u = i / 8.0;
            aC.D3(u, out _, out _, out Vector3D aC2, out Vector3D aC3);
            aProj.D3(u, out _, out _, out Vector3D aP2, out Vector3D aP3);
            Assert.True((aP2 - ProjectVec(TiltedPlane(), new Direction3D(0.2, 0.1, 1), aC2)).Magnitude() < 1.0e-6);
            Assert.True((aP3 - ProjectVec(TiltedPlane(), new Direction3D(0.2, 0.1, 1), aC3)).Magnitude() < 1.0e-5);
            Assert.True((aProj.DN(u, 2) - aP2).Magnitude() < 1.0e-9);
        }

        // Intervals and continuity are the curve's.
        Assert.Equal(aC.Continuity(), aProj.Continuity());
        Assert.Equal(aC.NbIntervals(Continuity.C2), aProj.NbIntervals(Continuity.C2));
        NCollection_Array1<double> aT = new(1, aProj.NbIntervals(Continuity.C2) + 1);
        aProj.Intervals(aT, Continuity.C2);
        Assert.Equal(0.4, aT[2], 1.0e-15);

        BSplineCurve3D aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen);
        Assert.Same(aRes, aProj.BSpline());
        Assert.Same(aRes, aProj.GetResult()!.Curve());
        Assert.False(aBS.IsFrozen);
    }

    [Fact]
    public void OffsetCurve_IsApproximated()
    {
        CircleCurve3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        TrimmedCurve3D aTrim = new(aCirc, 0.0, 2.0);
        CurveAdaptor aC = new(new OffsetCurve3D(aTrim, 0.5, new Direction3D(0, 0, 1)), 0.0, 2.0);
        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.False(aProj.IsPeriodic);
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Trim_ShallowCopy_Queries()
    {
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0));
        ProjectOnPlane anEmpty = new();
        Assert.Equal(CurveKind.OtherCurve, anEmpty.GetType());
        Assert.Null(anEmpty.GetCurve());
        Assert.Null(anEmpty.GetResult());
        Assert.Throws<MethodNotImplementedException>(() => anEmpty.Trim(0, 1, Tol));
        Assert.Equal(0.0, anEmpty.Resolution(1.0));

        ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Same(aC, aProj.GetCurve());
        Assert.True(aProj.IsClosed);
        CurveAdaptorBase aTrim = aProj.Trim(1.0, 2.0, Tol);
        Assert.Equal(1.0, aTrim.FirstParameter());
        Assert.Equal(2.0, aTrim.LastParameter());
        Assert.True(aTrim.Value(1.5).Distance(aProj.Value(1.5)) < 1.0e-15);

        ProjectOnPlane aCopy = (ProjectOnPlane)aProj.ShallowCopy();
        Assert.Equal(aProj.GetType(), aCopy.GetType());
        Assert.NotSame(aProj.GetResult(), aCopy.GetResult());
        Assert.Equal(aProj.FirstParameter(), aCopy.FirstParameter());
        Assert.Equal(aProj.LastParameter(), aCopy.LastParameter());
        Assert.True(aCopy.Value(0.7).Distance(aProj.Value(0.7)) < 1.0e-15);
        Assert.Equal(aProj.Resolution(1.0), aCopy.Resolution(1.0));
    }
}

/// <summary>ProjLib_ProjectOnSurface: the approximation and the OCCT issue that empties its poles.</summary>
public class ProjLib_ProjectOnSurfaceTests
{
    [Fact]
    public void NoSurface_NotDone()
    {
        ProjectOnSurface aProj = new();
        aProj.Load(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)), 0, 1), 1.0e-7);
        Assert.False(aProj.IsDone);
        Assert.Throws<NoSuchObjectException>(() => aProj.BSpline());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Load_PolesLost_OcctBug(int theSurface)
    {
        Surface aSurf = theSurface switch
        {
            0 => new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))),
            1 => new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0),
            _ => BumpSurface(),
        };
        SurfaceAdaptor aS = new(aSurf);
        CurveAdaptor aC = new(new CircleCurve3D(new Frame3D(new Point3D(0.1, 0.2, 1.0), new Direction3D(0, 0, 1)), 0.5), 0.0, 3.0);

        ProjectOnSurface aProj = new(aS);
        aProj.Load(aC, 1.0e-6);
        Assert.True(aProj.IsDone);
        BSplineCurve3D aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen);
        Assert.Same(aRes, aProj.BSpline());
        Assert.Equal(0.0, aRes.FirstParameter(), 1.0e-12);
        Assert.Equal(3.0, aRes.LastParameter(), 1.0e-12);
        // OCCT: each segment's poles are written at the start of the result's poles, which the copy loop then
        // overwrites with the segment's empty local poles: all poles after the first Degree() are the origin (the
        // first ones are the last segment's), so the curve ends at the origin instead of on the surface.
        for (int i = aRes.Degree() + 1; i <= aRes.PoleCount; i++)
        {
            Assert.Equal(new Point3D(0, 0, 0), aRes.Pole(i));
        }
        Assert.Equal(new Point3D(0, 0, 0), aRes.Value(3.0));
    }

    private static BSplineSurface BumpSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double z = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.3 : 0.0;
                aPoles[i, j] = new Point3D(-2 + (i - 1) * 4.0 / 3, -2 + (j - 1) * 4.0 / 3, z);
            }
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }
}
