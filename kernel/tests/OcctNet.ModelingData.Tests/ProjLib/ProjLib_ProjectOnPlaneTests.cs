// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

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
    private static gp_Ax3 TiltedPlane()
    {
        double a = M_PI / 6;
        return new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, -Math.Sin(a), Math.Cos(a)), new gp_Dir(1, 0, 0));
    }

    // A plane tilted about an axis that is not one of the curve's.
    private static gp_Ax3 SkewPlane()
    {
        return new gp_Ax3(new gp_Pnt(0.5, -1, 2), new gp_Dir(0.3, -0.4, 1), new gp_Dir(1, 0, -0.3));
    }

    private static gp_Pnt ProjectPnt(in gp_Ax3 thePl, in gp_Dir theD, in gp_Pnt theP)
    {
        gp_Vec aPO = new(theP, thePl.Location());
        double anAlpha = (aPO * new gp_Vec(thePl.Direction())) / (theD * thePl.Direction());
        return theP.Translated(anAlpha * new gp_Vec(theD));
    }

    private static gp_Vec ProjectVec(in gp_Ax3 thePl, in gp_Dir theD, in gp_Vec theV)
    {
        double anAlpha = (theV * new gp_Vec(thePl.Direction())) / (theD * thePl.Direction());
        return theV - anAlpha * new gp_Vec(theD);
    }

    private static double DistanceToPlane(in gp_Ax3 thePl, in gp_Pnt theP)
    {
        return Math.Abs(new gp_Vec(thePl.Location(), theP) * new gp_Vec(thePl.Direction()));
    }

    private static double DistanceToCurve(Geom_Curve theC, in gp_Pnt theP)
    {
        Extrema_ExtPC anExt = new(theP, new GeomAdaptor_Curve(theC));
        double aMin = double.MaxValue;
        for (int i = 1; anExt.IsDone() && i <= anExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, Math.Sqrt(anExt.SquareDistance(i)));
        }
        return aMin;
    }

    // The projection keeps the curve's parameters: the point at u is the projection of C(u).
    private static void AssertKeepsParameters(ProjLib_ProjectOnPlane theProj, Adaptor3d_Curve theC, double theTol, bool theDerivs = true)
    {
        gp_Ax3 aPl = theProj.GetPlane();
        gp_Dir aD = theProj.GetDirection();
        double f = theProj.FirstParameter(), l = theProj.LastParameter();
        Assert.Equal(theC.FirstParameter(), f);
        Assert.Equal(theC.LastParameter(), l);
        for (int i = 0; i <= 16; i++)
        {
            double u = f + (l - f) * i / 16;
            gp_Pnt anExp = ProjectPnt(aPl, aD, theC.Value(u));
            Assert.True(theProj.Value(u).Distance(anExp) < theTol, $"point at {u}");
            if (theDerivs)
            {
                theC.D1(u, out _, out gp_Vec aCD1);
                theProj.D1(u, out _, out gp_Vec aD1);
                Assert.True((aD1 - ProjectVec(aPl, aD, aCD1)).Magnitude() < theTol * 100, $"D1 at {u}");
            }
        }
    }

    // The projection lies on the plane and goes through the projection of every point of the curve.
    private static void AssertSameImage(ProjLib_ProjectOnPlane theProj, Adaptor3d_Curve theC, double theTol)
    {
        gp_Ax3 aPl = theProj.GetPlane();
        gp_Dir aD = theProj.GetDirection();
        Geom_Curve aRes = theProj.GetResult()!.Curve()!;
        double f = theProj.FirstParameter(), l = theProj.LastParameter();
        for (int i = 0; i <= 16; i++)
        {
            double u = f + (l - f) * i / 16;
            Assert.True(DistanceToPlane(aPl, theProj.Value(u)) < theTol, $"on plane at {u}");
            double t = theC.FirstParameter() + (theC.LastParameter() - theC.FirstParameter()) * i / 16;
            gp_Pnt aP = ProjectPnt(aPl, aD, theC.Value(t));
            Assert.True(DistanceToCurve(aRes, aP) < theTol, $"image of {t}");
        }
        // The bounds are the projections of the curve's ends.
        Assert.True(theProj.Value(f).Distance(ProjectPnt(aPl, aD, theC.Value(theC.FirstParameter()))) < theTol);
        Assert.True(theProj.Value(l).Distance(ProjectPnt(aPl, aD, theC.Value(theC.LastParameter()))) < theTol);
    }

    [Fact]
    public void Line_Oblique_NoKeep_IsTrimmedLine()
    {
        GeomAdaptor_Curve aC = new(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 1, 1)), -2.0, 3.0);
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, aProj.GetType());
        Assert.False(aProj.IsPeriodic());
        gp_Lin aLin = aProj.Line();
        Assert.True(aLin.Direction().IsNormal(TiltedPlane().Direction(), 1.0e-12));
        AssertSameImage(aProj, aC, 1.0e-9);
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Circle());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Degree());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.BSpline());
    }

    [Fact]
    public void Line_Oblique_Keep_IsDegreeOneBSpline()
    {
        GeomAdaptor_Curve aC = new(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 1, 1)), -2.0, 3.0);
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Geom_BSplineCurve aBS = aProj.BSpline();
        Assert.Equal(1, aBS.Degree());
        Assert.Equal(-2.0, aBS.FirstParameter(), 1.0e-12);
        Assert.Equal(3.0, aBS.LastParameter(), 1.0e-12);
        AssertKeepsParameters(aProj, aC, 1.0e-9);
    }

    [Fact]
    public void Line_ParallelToPlane_KeepsLine()
    {
        GeomAdaptor_Curve aC = new(new Geom_Line(new gp_Pnt(0, 0, 7), new gp_Dir(1, 2, 0)), -1.0, 4.0);
        ProjLib_ProjectOnPlane aProj = new(new gp_Ax3(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Line, aProj.GetType());
        Assert.Equal(-1.0, aProj.FirstParameter());
        Assert.Equal(4.0, aProj.LastParameter());
        gp_Pnt aP = aProj.Value(2.0);
        gp_Pnt anExp = aC.Value(2.0);
        Assert.Equal(anExp.X(), aP.X(), 1.0e-12);
        Assert.Equal(anExp.Y(), aP.Y(), 1.0e-12);
        Assert.Equal(1.0, aP.Z(), 1.0e-12);
    }

    [Fact]
    public void Line_AlongDirection_IsPoint()
    {
        GeomAdaptor_Curve aC = new(new Geom_Line(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 0.0, 5.0);
        ProjLib_ProjectOnPlane aProj = new(new gp_Ax3(new gp_Pnt(0, 0, -1), new gp_Dir(0, 0, 1)));
        aProj.Load(aC, Tol);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Geom_BSplineCurve aBS = aProj.BSpline();
        Assert.Equal(2, aBS.NbPoles());
        for (double u = 0.0; u <= 5.0; u += 1.0)
        {
            gp_Pnt aP = aProj.Value(u);
            Assert.True(aP.Distance(new gp_Pnt(1, 2, -1)) < 1.0e-12);
        }
    }

    [Fact]
    public void Circle_ParallelToPlane_IsCircle()
    {
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 1, 5), new gp_Dir(0, 0, 1)), 2.0), 0.5, 4.0);
        ProjLib_ProjectOnPlane aProj = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Circle, aProj.GetType());
        Assert.Equal(2.0, aProj.Circle().Radius(), 1.0e-9);
        Assert.Equal(0.0, aProj.Circle().Location().Z(), 1.0e-12);
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Circle_TiltedPlane_NoKeep_IsEllipse()
    {
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 3.0));
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Ellipse, aProj.GetType());
        gp_Elips anE = aProj.Ellipse();
        Assert.Equal(3.0, anE.MajorRadius(), 1.0e-9);
        double aCos = SkewPlane().Direction().Z();
        Assert.Equal(3.0 * aCos, anE.MinorRadius(), 1.0e-9);
        AssertSameImage(aProj, aC, 1.0e-7);
        Assert.True(aProj.IsPeriodic());
        Assert.Equal(2 * M_PI, aProj.Period(), 1.0e-12);
    }

    [Fact]
    public void Circle_TiltedAboutAxis_Keep_IsEllipseWithSameParameters()
    {
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3.0));
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Ellipse, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-9);
    }

    [Fact]
    public void Circle_SkewPlane_Keep_IsApproximated()
    {
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3.0));
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Assert.False(aProj.IsPeriodic());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Period());
        Assert.Equal(aProj.BSpline().Degree(), aProj.Degree());
        Assert.Equal(aProj.BSpline().NbPoles(), aProj.NbPoles());
        Assert.Equal(aProj.BSpline().NbKnots(), aProj.NbKnots());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Circle_EdgeOn_IsApproximatedSegment()
    {
        // The plane's normal lies in the circle's plane: the circle projects onto a segment.
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0), 0.2, 2.5);
        ProjLib_ProjectOnPlane aProj = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0), new gp_Dir(0, 0, 1)));
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Ellipse_SkewPlane_NoKeep_IsEllipse()
    {
        GeomAdaptor_Curve aC = new(new Geom_Ellipse(new gp_Ax2(new gp_Pnt(1, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 1, 0)), 4.0, 1.5), 0.3, 5.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane(), new gp_Dir(0.1, 0.2, 1));
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Ellipse, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-7);
        // Oblique direction: every projected point is the curve's point moved along it.
        gp_Pnt aP = aProj.Value(aProj.FirstParameter());
        gp_Vec aV = new(aC.Value(0.3), aP);
        Assert.True(aV.IsParallel(new gp_Vec(0.1, 0.2, 1), 1.0e-9));
    }

    [Fact]
    public void Parabola_TiltedAboutItsYAxis_IsParabola()
    {
        // The projection shortens the parabola's axis direction only: the focal becomes f / cos.
        double a = M_PI / 6;
        gp_Ax3 aPl = new(new gp_Pnt(1, 2, 3), new gp_Dir(-Math.Sin(a), 0, Math.Cos(a)), new gp_Dir(Math.Cos(a), 0, Math.Sin(a)));
        GeomAdaptor_Curve aC = new(new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 4), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(aPl);
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Parabola, aProj.GetType());
        Assert.Equal(0.5 / Math.Cos(a), aProj.Parabola().Focal(), 1.0e-12);
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Parabola_TiltedAboutItsXAxis_WrongFocal_OcctBug()
    {
        // OCCT: the focal of the projected parabola is Focal / |f(Xc)|, which ignores the shortening of the Y
        // direction: here the image is the parabola of focal 0.5 * cos^2, but OCCT builds focal 0.5.
        double a = M_PI / 6;
        GeomAdaptor_Curve aC = new(new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 4), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Parabola, aProj.GetType());
        Assert.Equal(0.5, aProj.Parabola().Focal(), 1.0e-12);
        Assert.NotEqual(0.5 * Math.Cos(a) * Math.Cos(a), aProj.Parabola().Focal(), 1.0e-3);
        gp_Pnt anImage = ProjectPnt(TiltedPlane(), TiltedPlane().Direction(), aC.Value(-3.0));
        Assert.True(DistanceToCurve(aProj.GetResult()!.Curve()!, anImage) > 1.0e-2);
    }

    [Fact]
    public void Parabola_SkewPlane_NoKeep_BuiltByApex()
    {
        GeomAdaptor_Curve aC = new(new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Parabola, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-5);
    }

    [Fact]
    public void Parabola_SkewPlane_Keep_IsApproximated()
    {
        GeomAdaptor_Curve aC = new(new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Assert.Equal(2, aProj.Degree());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Parabola_ProjectedToLine_OcctDereferencesNull()
    {
        // OCCT: the parabola's axis is along the projection, so the image is a line; Load builds it but passes the
        // null parabola to GetTrimmedResult (a crash in OCCT, NullReferenceException in the port).
        GeomAdaptor_Curve aC = new(new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 0.5), -3.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)));
        Assert.Throws<NullReferenceException>(() => aProj.Load(aC, Tol, false));
    }

    [Fact]
    public void Hyperbola_NormalProjection_IsHyperbola()
    {
        GeomAdaptor_Curve aC = new(new Geom_Hyperbola(new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Hyperbola, aProj.GetType());
        AssertSameImage(aProj, aC, 1.0e-7);
    }

    [Fact]
    public void Hyperbola_SkewPlane_NoKeep_BuiltByApex()
    {
        GeomAdaptor_Curve aC = new(new Geom_Hyperbola(new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_Hyperbola, aProj.GetType());
        double f = aProj.FirstParameter(), l = aProj.LastParameter();
        for (int i = 0; i <= 8; i++)
        {
            Assert.True(DistanceToPlane(SkewPlane(), aProj.Value(f + (l - f) * i / 8)) < 1.0e-7);
        }
    }

    [Fact]
    public void Hyperbola_SkewPlane_Keep_IsApproximated()
    {
        GeomAdaptor_Curve aC = new(new Geom_Hyperbola(new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2.0, 1.0)), -1.5, 1.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, true);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Bezier_ProjectsPoles_FrozenResult_CallerUnchanged()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 3);
        aPoles[3] = new gp_Pnt(2, -1, 1);
        aPoles[4] = new gp_Pnt(4, 0, 2);
        NCollection_Array1<double> aWeights = new(1, 4);
        aWeights[1] = 1.0;
        aWeights[2] = 2.0;
        aWeights[3] = 0.5;
        aWeights[4] = 1.0;
        Geom_BezierCurve aBz = new(aPoles, aWeights);
        GeomAdaptor_Curve aC = new(aBz);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BezierCurve, aProj.GetType());
        Assert.True(aProj.IsRational());
        Assert.Equal(3, aProj.Degree());
        AssertKeepsParameters(aProj, aC, 1.0e-9);

        Geom_BezierCurve aRes = aProj.Bezier();
        Assert.True(aRes.IsFrozen());
        Assert.Same(aRes, aProj.Bezier());
        Assert.NotSame(aBz, aRes);
        Assert.False(aBz.IsFrozen());
        Assert.Equal(new gp_Pnt(1, 2, 3), aBz.Pole(2));
    }

    [Fact]
    public void BSpline_ProjectsPoles_Derivatives()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new gp_Pnt(i, Math.Sin(i), 0.3 * i * i);
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
        Geom_BSplineCurve aBS = new(aPoles, aWeights, aKnots, aMults, 3);
        GeomAdaptor_Curve aC = new(aBS);
        ProjLib_ProjectOnPlane aProj = new(TiltedPlane(), new gp_Dir(0.2, 0.1, 1));
        aProj.Load(aC, Tol);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Assert.True(aProj.IsRational());
        Assert.Equal(3, aProj.NbKnots());
        AssertKeepsParameters(aProj, aC, 1.0e-9);

        // D2, D3 and DN of the result: the projected derivatives (the projection is affine).
        for (int i = 0; i <= 8; i++)
        {
            double u = i / 8.0;
            aC.D3(u, out _, out _, out gp_Vec aC2, out gp_Vec aC3);
            aProj.D3(u, out _, out _, out gp_Vec aP2, out gp_Vec aP3);
            Assert.True((aP2 - ProjectVec(TiltedPlane(), new gp_Dir(0.2, 0.1, 1), aC2)).Magnitude() < 1.0e-6);
            Assert.True((aP3 - ProjectVec(TiltedPlane(), new gp_Dir(0.2, 0.1, 1), aC3)).Magnitude() < 1.0e-5);
            Assert.True((aProj.DN(u, 2) - aP2).Magnitude() < 1.0e-9);
        }

        // Intervals and continuity are the curve's.
        Assert.Equal(aC.Continuity(), aProj.Continuity());
        Assert.Equal(aC.NbIntervals(GeomAbs_Shape.GeomAbs_C2), aProj.NbIntervals(GeomAbs_Shape.GeomAbs_C2));
        NCollection_Array1<double> aT = new(1, aProj.NbIntervals(GeomAbs_Shape.GeomAbs_C2) + 1);
        aProj.Intervals(aT, GeomAbs_Shape.GeomAbs_C2);
        Assert.Equal(0.4, aT[2], 1.0e-15);

        Geom_BSplineCurve aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen());
        Assert.Same(aRes, aProj.BSpline());
        Assert.Same(aRes, aProj.GetResult()!.Curve());
        Assert.False(aBS.IsFrozen());
    }

    [Fact]
    public void OffsetCurve_IsApproximated()
    {
        Geom_Circle aCirc = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        Geom_TrimmedCurve aTrim = new(aCirc, 0.0, 2.0);
        GeomAdaptor_Curve aC = new(new Geom_OffsetCurve(aTrim, 0.5, new gp_Dir(0, 0, 1)), 0.0, 2.0);
        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_BSplineCurve, aProj.GetType());
        Assert.False(aProj.IsPeriodic());
        AssertKeepsParameters(aProj, aC, 1.0e-5, false);
    }

    [Fact]
    public void Trim_ShallowCopy_Queries()
    {
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 3.0));
        ProjLib_ProjectOnPlane anEmpty = new();
        Assert.Equal(GeomAbs_CurveType.GeomAbs_OtherCurve, anEmpty.GetType());
        Assert.Null(anEmpty.GetCurve());
        Assert.Null(anEmpty.GetResult());
        Assert.Throws<Standard_NotImplemented>(() => anEmpty.Trim(0, 1, Tol));
        Assert.Equal(0.0, anEmpty.Resolution(1.0));

        ProjLib_ProjectOnPlane aProj = new(SkewPlane());
        aProj.Load(aC, Tol, false);
        Assert.Same(aC, aProj.GetCurve());
        Assert.True(aProj.IsClosed());
        Adaptor3d_Curve aTrim = aProj.Trim(1.0, 2.0, Tol);
        Assert.Equal(1.0, aTrim.FirstParameter());
        Assert.Equal(2.0, aTrim.LastParameter());
        Assert.True(aTrim.Value(1.5).Distance(aProj.Value(1.5)) < 1.0e-15);

        ProjLib_ProjectOnPlane aCopy = (ProjLib_ProjectOnPlane)aProj.ShallowCopy();
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
        ProjLib_ProjectOnSurface aProj = new();
        aProj.Load(new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0)), 0, 1), 1.0e-7);
        Assert.False(aProj.IsDone());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.BSpline());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Load_PolesLost_OcctBug(int theSurface)
    {
        Geom_Surface aSurf = theSurface switch
        {
            0 => new Geom_Plane(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1))),
            1 => new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0),
            _ => BumpSurface(),
        };
        GeomAdaptor_Surface aS = new(aSurf);
        GeomAdaptor_Curve aC = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0.1, 0.2, 1.0), new gp_Dir(0, 0, 1)), 0.5), 0.0, 3.0);

        ProjLib_ProjectOnSurface aProj = new(aS);
        aProj.Load(aC, 1.0e-6);
        Assert.True(aProj.IsDone());
        Geom_BSplineCurve aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen());
        Assert.Same(aRes, aProj.BSpline());
        Assert.Equal(0.0, aRes.FirstParameter(), 1.0e-12);
        Assert.Equal(3.0, aRes.LastParameter(), 1.0e-12);
        // OCCT: each segment's poles are written at the start of the result's poles, which the copy loop then
        // overwrites with the segment's empty local poles: all poles after the first Degree() are the origin (the
        // first ones are the last segment's), so the curve ends at the origin instead of on the surface.
        for (int i = aRes.Degree() + 1; i <= aRes.NbPoles(); i++)
        {
            Assert.Equal(new gp_Pnt(0, 0, 0), aRes.Pole(i));
        }
        Assert.Equal(new gp_Pnt(0, 0, 0), aRes.Value(3.0));
    }

    private static Geom_BSplineSurface BumpSurface()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double z = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.3 : 0.0;
                aPoles[i, j] = new gp_Pnt(-2 + (i - 1) * 4.0 / 3, -2 + (j - 1) * 4.0 / 3, z);
            }
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }
}
