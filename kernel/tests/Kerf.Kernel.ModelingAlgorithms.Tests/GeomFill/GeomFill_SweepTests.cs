// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;
using static Kerf.Engine.Construction.Surfaces.SurfacePipeError;
using static Kerf.Engine.Construction.Surfaces.Trihedron;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 5L2's classes: GeomFill_SweepFunction, Sweep, EvolvedSection, NSections, the convertors and the package
/// functions, CircularBlendFunc, SweepSectionGenerator, AppSweep and Pipe; and the frozen-input rules (decisions.md).
/// </summary>
public class GeomFill_SweepTests
{
    private static BSplineCurve3D Path3d(double theWiggle = 2.0)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(10, theWiggle, 1);
        aPoles[3] = new Point3D(20, -theWiggle, 2);
        aPoles[4] = new Point3D(30, theWiggle, 0.5);
        aPoles[5] = new Point3D(40, 0, 0);
        NCollection_Array1<double> aK = new(1, 3);
        NCollection_Array1<int> aM = new(1, 3);
        aK[1] = 0;
        aK[2] = 1;
        aK[3] = 2;
        aM[1] = aM[3] = 4;
        aM[2] = 1;
        return new BSplineCurve3D(aPoles, aK, aM, 3);
    }

    private static BSplineCurve3D ConvexPlanarPath()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            double anA = 0.3 * (i - 1);
            aPoles[i] = new Point3D(20 * Math.Sin(anA), 20 - 20 * Math.Cos(anA), 0);
        }
        NCollection_Array1<double> aK = new(1, 2);
        NCollection_Array1<int> aM = new(1, 2);
        aK[1] = 0;
        aK[2] = 1;
        aM[1] = aM[2] = 5;
        return new BSplineCurve3D(aPoles, aK, aM, 4);
    }

    private static CircleCurve3D CircleAtStart(Curve3D thePath, double theRadius)
    {
        thePath.D1(thePath.FirstParameter(), out Point3D aP, out Vector3D aT);
        return new CircleCurve3D(new Frame3D(aP, new Direction3D(aT)), theRadius);
    }

    private static BSplineCurve3D OpenSection(Point3D theCentre)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = theCentre.Translated(new Vector3D(0, -1, 0));
        aPoles[2] = theCentre.Translated(new Vector3D(0, -0.3, 1));
        aPoles[3] = theCentre.Translated(new Vector3D(0, 0.3, 1));
        aPoles[4] = theCentre.Translated(new Vector3D(0, 1, 0));
        NCollection_Array1<double> aK = new(1, 2);
        NCollection_Array1<int> aM = new(1, 2);
        aK[1] = 0;
        aK[2] = 1;
        aM[1] = aM[2] = 4;
        return new BSplineCurve3D(aPoles, aK, aM, 3);
    }

    //==================================================================================================
    // GeomFill_SweepFunction
    //==================================================================================================

    [Fact]
    public void SweepFunction_places_the_section_with_the_location_law()
    {
        BSplineCurve3D aPath = Path3d();
        global::Kerf.Engine.Construction.Surfaces.CurveAndTrihedron aLoc = new(new global::Kerf.Engine.Construction.Surfaces.Frenet());
        aLoc.SetCurve(new CurveAdaptor(aPath));
        BSplineCurve3D aSect = OpenSection(new Point3D(0, 0, 0));
        global::Kerf.Engine.Construction.Surfaces.UniformSection aSec = new(aSect, 0, 2);
        global::Kerf.Engine.Construction.Surfaces.SurfaceSweepFunction aF = new(aSec, aLoc, 0, 0, 1);
        aF.SectionShape(out int aNb, out _, out _);
        NCollection_Array1<Point3D> aP = new(1, aNb), aP0 = new(1, aNb);
        NCollection_Array1<Vector3D> aD = new(1, aNb), aD2 = new(1, aNb);
        NCollection_Array1<double> aW = new(1, aNb), aDW = new(1, aNb), aD2W = new(1, aNb);
        NCollection_Array1<Point2D> aP2 = new(1, 1);
        NCollection_Array1<Vector2D> aV2 = new(1, 1), aV22 = new(1, 1);
        double v = 0.7, h = 1e-6;

        Assert.True(aF.D0(v, 0, 2, aP, aP2, aW));
        Matrix3x3 M = new();
        Vector3D V = new();
        aLoc.D0(v, ref M, ref V);
        for (int i = 1; i <= aNb; i++)
        {
            Xyz aXYZ = M * aSect.Pole(i).XYZ() + V.XYZ();
            Assert.True(new Point3D(aXYZ).Distance(aP[i]) < 1e-12);
        }

        // D1 and D2 against central differences
        NCollection_Array1<Point3D> aPm = new(1, aNb), aPp = new(1, aNb);
        NCollection_Array1<Vector3D> aDm = new(1, aNb), aDp = new(1, aNb);
        aF.D1(v - h, 0, 2, aPm, aDm, aP2, aV2, aW, aDW);
        aF.D1(v + h, 0, 2, aPp, aDp, aP2, aV2, aW, aDW);
        Assert.True(aF.D2(v, 0, 2, aP0, aD, aD2, aP2, aV2, aV22, aW, aDW, aD2W));
        for (int i = 1; i <= aNb; i++)
        {
            Vector3D aFD = new Vector3D(aPm[i], aPp[i]) / (2 * h);
            Assert.True((aFD - aD[i]).Magnitude() < 1e-5 * (1 + aD[i].Magnitude()));
            Vector3D aFD2 = (aDp[i] - aDm[i]) / (2 * h);
            Assert.True((aFD2 - aD2[i]).Magnitude() < 1e-3 * (1 + aD2[i].Magnitude()));
        }

        // a constant section: the location's intervals
        Assert.Equal(aLoc.NbIntervals(C2), aF.NbIntervals(C2));
        Assert.Equal(0, aF.Nb2dCurves());
        Assert.False(aF.IsRational);
    }

    //==================================================================================================
    // GeomFill_Sweep
    //==================================================================================================

    [Fact]
    public void Sweep_recognises_elementary_surfaces()
    {
        // a straight path and a circle: a cylinder
        TrimmedCurve3D aLine = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 10);
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aLine, CircleAtStart(aLine, 2), global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet);
        aPipe.Perform(1e-4, false);
        Assert.True(aPipe.IsDone);
        var aCyl = Assert.IsType<TrimmedSurface>(aPipe.Surface());
        Assert.IsType<CylindricalSurface>(aCyl.BasisSurface());
        Assert.Equal(0.0, aPipe.ErrorOnSurf());

        // a circular path and a circle off the axis: a torus, with the sections as u-isos
        TrimmedCurve3D anArc = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 8), 0, 2);
        aPipe = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(anArc, CircleAtStart(anArc, 1.5), global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet);
        aPipe.Perform(1e-4, false);
        Assert.True(aPipe.IsDone);
        Assert.True(aPipe.ExchangeUV() || aPipe.Surface() is TrimmedSurface);
        var aTor = Assert.IsType<TrimmedSurface>(aPipe.Surface());
        Assert.IsType<ToroidalSurface>(aTor.BasisSurface());

        // a straight path and an ellipse: an extrusion
        EllipseCurve3D anEllipse = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3, 1);
        aPipe = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aLine, anEllipse, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet);
        aPipe.Perform(1e-4, false);
        var anExt = Assert.IsType<TrimmedSurface>(aPipe.Surface());
        Assert.IsType<ExtrusionSurface>(anExt.BasisSurface());

        // without the particular cases, a B-spline surface
        aPipe = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aLine, CircleAtStart(aLine, 2), global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet);
        aPipe.GenerateParticularCase(false);
        Assert.False(aPipe.GenerateParticularCase());
        aPipe.Perform(1e-4, false);
        Assert.IsType<BSplineSurface>(aPipe.Surface());
    }

    [Fact]
    public void Sweep_hands_out_frozen_results_and_reads_frozen_laws()
    {
        BSplineCurve3D aPath = Path3d();
        aPath.Freeze();
        global::Kerf.Engine.Construction.Surfaces.CurveAndTrihedron aLoc = new(new global::Kerf.Engine.Construction.Surfaces.CorrectedFrenet());
        aLoc.SetCurve(new CurveAdaptor(aPath));
        BSplineCurve3D aSect = OpenSection(new Point3D(0, 0, 0));
        aSect.Freeze();
        Linear aLaw = new();
        aLaw.Set(0, 1, 2, 2);
        global::Kerf.Engine.Construction.Surfaces.SurfaceSweep aSweep = new(aLoc);
        aSweep.Build(new global::Kerf.Engine.Construction.Surfaces.EvolvedSection(aSect, aLaw), global::Kerf.Engine.Construction.Surfaces.ApproxStyle.Location, C2, 10, 30);
        Assert.True(aSweep.IsDone);
        Assert.False(aSweep.InternalSurface()!.IsFrozen);
        Surface aS = aSweep.Surface()!;
        Assert.True(aS.IsFrozen);
        Assert.Same(aS, aSweep.Surface());
        Assert.True(aSweep.Restriction(true)!.IsFrozen);
        Assert.True(aSweep.Restriction(false)!.IsFrozen);
        Assert.Equal(0, aSweep.NumberOfTrace());
        Assert.Throws<OutOfRangeException>(() => aSweep.Trace(1));
        Assert.Throws<OutOfRangeException>(() => aSweep.ErrorOnTrace(1, out _, out _));
        aSweep.ErrorOnRestriction(true, out double aU, out double aV);
        Assert.Equal(0.0, aU);
        Assert.Equal(0.0, aV);

        // the surface's iso-v at the start is the scaled section placed at the path's start
        aS.Bounds(out double u1, out double u2, out double v1, out _);
        Point3D aP0 = aS.Value(u1, v1);
        Assert.True(aP0.Distance(aPath.Value(0)) < 5);
        Assert.True(aSweep.ErrorOnSurface() < 1e-4);
        Assert.True(u2 > u1);
    }

    //==================================================================================================
    // GeomFill_EvolvedSection
    //==================================================================================================

    [Fact]
    public void EvolvedSection_scales_the_section_by_the_law()
    {
        BSplineCurve3D aSect = OpenSection(new Point3D(1, 2, 3));
        S aLaw = new();
        aLaw.Set(0, 1, 3, 2);
        global::Kerf.Engine.Construction.Surfaces.EvolvedSection aSec = new(aSect, aLaw);
        aSec.SectionShape(out int aNb, out int aNbK, out int aDeg);
        Assert.Equal((4, 2, 3), (aNb, aNbK, aDeg));
        NCollection_Array1<Point3D> aP = new(1, aNb);
        NCollection_Array1<Vector3D> aD = new(1, aNb), aD2 = new(1, aNb);
        NCollection_Array1<double> aW = new(1, aNb), aDW = new(1, aNb), aD2W = new(1, aNb);
        Assert.True(aSec.D2(1.3, aP, aD, aD2, aW, aDW, aD2W));
        aLaw.D2(1.3, out double f, out double df, out double d2f);
        for (int i = 1; i <= aNb; i++)
        {
            Assert.True(aP[i].Distance(new Point3D(aSect.Pole(i).XYZ() * f)) < 1e-12);
            Assert.True((aD[i] - new Vector3D(aSect.Pole(i).XYZ() * df)).Magnitude() < 1e-12);
            Assert.True((aD2[i] - new Vector3D(aSect.Pole(i).XYZ() * d2f)).Magnitude() < 1e-12);
            Assert.Equal(0.0, aDW[i]);
        }
        Assert.False(aSec.IsVPeriodic);
        Assert.Null(aSec.BSplineSurface());
        double anErr = 1;
        Assert.False(aSec.IsConstant(ref anErr));
        Assert.Throws<NotDoneException>(() => aSec.ConstantSection());
        aSec.GetDomain(out double d1, out double d2);
        Assert.Equal((0.0, 3.0), (d1, d2));
        aSec.SetInterval(1, 2);
        aSec.GetInterval(out double i1, out double i2);
        Assert.Equal((1.0, 2.0), (i1, i2));
        Assert.True(aSec.MaximalSection() > 0);

        // a non-B-spline section is converted; the caller's curve is kept as it was
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        aCircle.Freeze();
        global::Kerf.Engine.Construction.Surfaces.EvolvedSection aSec2 = new(aCircle, aLaw);
        Assert.True(aSec2.IsRational);
        // converted as UniformSection converts it (a periodic B-spline: RemoveKnot leaves it periodic)
        Assert.Equal(new global::Kerf.Engine.Construction.Surfaces.UniformSection(aCircle).IsUPeriodic, aSec2.IsUPeriodic);
    }

    //==================================================================================================
    // GeomFill_NSections
    //==================================================================================================

    [Fact]
    public void NSections_reparametrises_a_copy_of_the_callers_curves()
    {
        // OCCT calls SetKnots on the caller's B-spline sections; the port reparametrises copies (decisions.md)
        BSplineCurve3D aS1 = OpenSection(new Point3D(0, 0, 0));
        BSplineCurve3D aS2 = OpenSection(new Point3D(5, 0, 0));
        aS1.Freeze();
        NCollection_Sequence<Curve3D> aSeq = new();
        aSeq.Append(aS1);
        aSeq.Append(aS2);
        NCollection_Sequence<double> aPar = new();
        aPar.Append(0);
        aPar.Append(1);
        global::Kerf.Engine.Construction.Surfaces.SurfaceNSections aLaw = new(aSeq, aPar, 0, 10, 0, 1);
        Assert.Equal(1.0, aS1.Knot(2));
        Assert.Equal(1.0, aS2.Knot(2));
        Assert.Same(aS1, aSeq[1]);

        BSplineSurface aSurf = aLaw.BSplineSurface()!;
        Assert.True(aSurf.IsFrozen);
        Assert.Same(aSurf, aLaw.InternalBSplineSurface());
        aSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 10.0, 0.0, 1.0), (u1, u2, v1, v2));
        Assert.True(aSurf.VDegree() >= 2);

        // the sections are the v-isos at their parameters
        aLaw.SectionShape(out int aNb, out _, out _);
        NCollection_Array1<Point3D> aP = new(1, aNb), aPm = new(1, aNb), aPp = new(1, aNb);
        NCollection_Array1<Vector3D> aD = new(1, aNb), aD2 = new(1, aNb), aDm = new(1, aNb), aDp = new(1, aNb);
        NCollection_Array1<double> aW = new(1, aNb), aDW = new(1, aNb), aD2W = new(1, aNb);
        Assert.True(aLaw.D0(0, aP, aW));
        Assert.True(aSurf.Value(5, 0).Distance(aS1.Value(0.5)) < 1e-5);

        // D1, D2 against central differences
        double v = 0.4, h = 1e-6;
        aLaw.D1(v - h, aPm, aDm, aW, aDW);
        aLaw.D1(v + h, aPp, aDp, aW, aDW);
        Assert.True(aLaw.D2(v, aP, aD, aD2, aW, aDW, aD2W));
        for (int i = 1; i <= aNb; i++)
        {
            Assert.True((new Vector3D(aPm[i], aPp[i]) / (2 * h) - aD[i]).Magnitude() < 1e-5 * (1 + aD[i].Magnitude()));
            Assert.True(((aDp[i] - aDm[i]) / (2 * h) - aD2[i]).Magnitude() < 1e-3 * (1 + aD2[i].Magnitude()));
        }
    }

    [Fact]
    public void NSections_recognises_constant_and_conical_laws()
    {
        Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        NCollection_Sequence<Curve3D> aSeq = new();
        aSeq.Append(new CircleCurve3D(anAx, 2));
        aSeq.Append(new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 5)), 2));
        NCollection_Sequence<double> aPar = new();
        aPar.Append(0);
        aPar.Append(5);
        global::Kerf.Engine.Construction.Surfaces.SurfaceNSections aLaw = new(aSeq, aPar, 0, 2 * M_PI, 0, 5);
        double anErr = -1;
        Assert.True(aLaw.IsConstant(ref anErr));
        Assert.Equal(0.0, anErr);
        Assert.True(aLaw.IsConicalLaw(ref anErr));
        Curve3D aMid = aLaw.CirclSection(2.5);
        Assert.Equal(2.0, Assert.IsType<CircleCurve3D>(aMid).Radius(), 1e-12);
        Curve3D aConst = aLaw.ConstantSection();
        Assert.NotSame(aSeq[1], aConst);

        aSeq[2] = new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 5)), 4);
        aLaw = new global::Kerf.Engine.Construction.Surfaces.SurfaceNSections(aSeq, aPar, 0, 2 * M_PI, 0, 5);
        Assert.False(aLaw.IsConstant(ref anErr));
        Assert.True(aLaw.IsConicalLaw(ref anErr));
        Assert.Equal(3.0, ((CircleCurve3D)aLaw.CirclSection(2.5)).Radius(), 1e-12);

        // three sections: neither
        aSeq.Append(new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 9)), 1));
        aPar.Append(9);
        aLaw = new global::Kerf.Engine.Construction.Surfaces.SurfaceNSections(aSeq, aPar, 0, 2 * M_PI, 0, 9);
        Assert.False(aLaw.IsConicalLaw(ref anErr));
        Assert.Throws<NotDoneException>(() => aLaw.CirclSection(1));
    }

    [Fact]
    public void NSections_cuts_a_reference_surface()
    {
        NCollection_Sequence<Curve3D> aSeq = new();
        aSeq.Append(OpenSection(new Point3D(0, 0, 0)));
        aSeq.Append(OpenSection(new Point3D(5, 0, 0)));
        NCollection_Sequence<double> aPar = new();
        aPar.Append(0);
        aPar.Append(1);
        global::Kerf.Engine.Construction.Surfaces.SurfaceNSections aFirst = new(aSeq, aPar, 0, 1, 0, 1);
        BSplineSurface aRef = aFirst.BSplineSurface()!;
        NCollection_Sequence<Transform3D> aTrsfs = new();
        global::Kerf.Engine.Construction.Surfaces.SurfaceNSections aCut = new(aSeq, aTrsfs, aPar, 0.25, 0.75, 0, 1, aRef);
        BSplineSurface aS = aCut.BSplineSurface()!;
        aS.Bounds(out double u1, out double u2, out _, out _);
        Assert.Equal(0.25, u1, 1e-12);
        Assert.Equal(0.75, u2, 1e-12);
        Assert.NotSame(aRef, aS);
        Assert.True(aRef.IsFrozen);
    }

    //==================================================================================================
    // The convertors and the package functions
    //==================================================================================================

    [Theory]
    [InlineData(QuasiAngular, 0.5)]
    [InlineData(QuasiAngular, 4.0)]
    [InlineData(Polynomial, 1.0)]
    [InlineData(TgtThetaOver2, 1.0)]
    [InlineData(TgtThetaOver2, 4.5)] // GetCircle's angles are in ]-pi/2, 3pi/2]
    public void GetCircle_gives_the_arc_and_its_derivatives(ParameterisationType theConv, double theAngle)
    {
        ParameterisationType aConv = theConv;
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetShape(theAngle, out int aNbP, out int aNbK, out int aDeg, ref aConv);
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Knots(aConv, aK);
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Mults(aConv, aM);

        double aR = 2;
        Point3D aC = new(1, 2, 3);
        Direction3D aN = new(0.1, 0.2, 1);
        Frame3D anAx = new(aC, aN);
        Vector3D aX = new(anAx.XDirection()), aY = new(anAx.YDirection());

        // the arc of theAngle at time t, its first point turning with t
        void Arc(double t, out Vector3D ns1, out Vector3D ns2, out Point3D P1, out Point3D P2)
        {
            Vector3D u1 = aX * Math.Cos(t) + aY * Math.Sin(t);
            Vector3D u2 = aX * Math.Cos(t + theAngle) + aY * Math.Sin(t + theAngle);
            // GetCircle's "normals" run from the centre to the ends: it builds the interior poles as
            // pts1 + R ((cos a - 1) ns1 + sin a (nplan ^ ns1))
            ns1 = u1;
            ns2 = u2;
            P1 = aC.Translated(u1 * aR);
            P2 = aC.Translated(u2 * aR);
        }

        double t0 = 0.3, h = 1e-6;
        Arc(t0, out Vector3D n1, out Vector3D n2, out Point3D p1, out Point3D p2);
        NCollection_Array1<Point3D> aP = new(1, aNbP);
        NCollection_Array1<double> aW = new(1, aNbP);
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, n1, n2, new Vector3D(aN), p1, p2, aR, aC, aP, aW);
        BSplineCurve3D anArc = new(aP, aW, aK, aM, aDeg);
        for (int i = 0; i <= 10; i++)
        {
            double u = anArc.FirstParameter() + (anArc.LastParameter() - anArc.FirstParameter()) * i / 10;
            // exact for the rational conversions; the polynomial one approximates the circle
            Assert.Equal(aR, anArc.Value(u).Distance(aC), theConv == Polynomial ? 1e-6 : 1e-9);
        }
        Assert.True(anArc.StartPoint().Distance(p1) < 1e-9);
        Assert.True(anArc.EndPoint().Distance(p2) < 1e-9);

        // the first derivatives against differences: the arc turns about the normal at unit speed
        Arc(t0 - h, out Vector3D m1, out Vector3D m2, out Point3D q1m, out Point3D q2m);
        Arc(t0 + h, out Vector3D pl1, out Vector3D pl2, out Point3D q1p, out Point3D q2p);
        Vector3D dn1 = (pl1 - m1) / (2 * h), dn2 = (pl2 - m2) / (2 * h);
        Vector3D t1 = new Vector3D(q1m, q1p) / (2 * h), t2 = new Vector3D(q2m, q2p) / (2 * h);
        NCollection_Array1<Vector3D> aD = new(1, aNbP);
        NCollection_Array1<double> aDW = new(1, aNbP);
        Assert.True(global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, n1, n2, dn1, dn2, new Vector3D(aN), new Vector3D(0, 0, 0), p1, p2, t1, t2, aR,
                                       0, aC, new Vector3D(0, 0, 0), aP, aD, aW, aDW));
        NCollection_Array1<Point3D> aPm = new(1, aNbP), aPp = new(1, aNbP);
        NCollection_Array1<double> aWm = new(1, aNbP), aWp = new(1, aNbP);
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, m1, m2, new Vector3D(aN), q1m, q2m, aR, aC, aPm, aWm);
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, pl1, pl2, new Vector3D(aN), q1p, q2p, aR, aC, aPp, aWp);
        for (int i = 1; i <= aNbP; i++)
        {
            Assert.True((new Vector3D(aPm[i], aPp[i]) / (2 * h) - aD[i]).Magnitude() < 1e-4);
            Assert.Equal((aWp[i] - aWm[i]) / (2 * h), aDW[i], 1e-4);
        }

        Assert.True(global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(aConv, 0.5, aR, 0.01, 1e-4) > 0);
        NCollection_Array1<double> aMinW = new(1, aNbP);
        if (aConv != TgtThetaOver2_3 || aNbP == 7)
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetMinimalWeights(aConv, 0.2, theAngle, aMinW);
            for (int i = 1; i <= aNbP; i++)
            {
                Assert.True(aMinW[i] > 0);
            }
        }
    }

    [Fact]
    public void Convertors_need_Init()
    {
        global::Kerf.Engine.Construction.Surfaces.QuasiAngularConvertor aQ = new();
        Assert.False(aQ.Initialized());
        aQ.Init();
        Assert.True(aQ.Initialized());
        global::Kerf.Engine.Construction.Surfaces.PolynomialConvertor aP = new();
        Assert.False(aP.Initialized());
        aP.Init();
        Assert.True(aP.Initialized());
        NCollection_Array1<Point3D> aPoles = new(1, 8);
        aP.Section(new Point3D(1, 0, 0), new Point3D(0, 0, 0), new Vector3D(0, 0, 1), M_PI / 2, aPoles);
        Assert.True(aPoles[1].Distance(new Point3D(1, 0, 0)) < 1e-12);
        Assert.True(aPoles[8].Distance(new Point3D(0, 1, 0)) < 1e-12);
    }

    [Fact]
    public void Surface_rules_between_two_curves()
    {
        // two parallel segments: a plane
        TrimmedCurve3D aL1 = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 4);
        TrimmedCurve3D aL2 = new(new LineCurve3D(new Point3D(0, 3, 0), new Direction3D(1, 0, 0)), 0, 4);
        var aPlane = Assert.IsType<TrimmedSurface>(global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(aL1, aL2));
        Assert.IsType<PlaneSurface>(aPlane.BasisSurface());
        Assert.True(aPlane.IsFrozen);

        // coaxial circles: a cylinder, a cone
        Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        CircleCurve3D aC1 = new(anAx, 2);
        var aCyl = (TrimmedSurface)global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(aC1, new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 3)), 2))!;
        Assert.IsType<CylindricalSurface>(aCyl.BasisSurface());
        var aCone = (TrimmedSurface)global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(aC1, new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 3)), 3))!;
        Assert.IsType<ConicalSurface>(aCone.BasisSurface());
        aCone.Bounds(out _, out _, out double aV1, out double aV2);
        Point3D aTop = aCone.Value(0, Math.Abs(aV1) > Math.Abs(aV2) ? aV1 : aV2);
        Assert.Equal(3.0, aTop.Z(), 1e-9);
        Assert.Equal(3.0, new Point3D(aTop.X(), aTop.Y(), 0).Distance(new Point3D(0, 0, 0)), 1e-9);

        // otherwise: GeomFill_Generator's B-spline surface
        Surface aRuled = global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(Path3d(), OpenSection(new Point3D(0, 5, 0)))!;
        Assert.IsType<BSplineSurface>(aRuled);
        Assert.True(aRuled.IsFrozen);
    }

    //==================================================================================================
    // GeomFill_CircularBlendFunc and GeomFill_SweepSectionGenerator
    //==================================================================================================

    private static (BSplineCurve3D Path, BSplineCurve3D C1, BSplineCurve3D C2) EdgeCurves(double theR)
    {
        BSplineCurve3D aPath = Path3d(0.5);
        NCollection_Array1<Point3D> aP1 = new(1, aPath.PoleCount), aP2 = new(1, aPath.PoleCount);
        for (int i = 1; i <= aPath.PoleCount; i++)
        {
            aP1[i] = aPath.Pole(i).Translated(new Vector3D(0, theR, 0));
            aP2[i] = aPath.Pole(i).Translated(new Vector3D(0, 0, theR));
        }
        return (aPath,
                new BSplineCurve3D(aP1, aPath.InternalKnots(), aPath.InternalMultiplicities(), aPath.Degree()),
                new BSplineCurve3D(aP2, aPath.InternalKnots(), aPath.InternalMultiplicities(), aPath.Degree()));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircularBlendFunc_gives_arcs_of_the_radius_between_the_guides(bool thePolynomial)
    {
        (BSplineCurve3D aPath, BSplineCurve3D aC1, BSplineCurve3D aC2) = EdgeCurves(2);
        global::Kerf.Engine.Construction.Surfaces.CircularBlendFunc aF = new(new CurveAdaptor(aPath), new CurveAdaptor(aC1),
                                            new CurveAdaptor(aC2), 2, thePolynomial);
        Assert.Equal(!thePolynomial, aF.IsRational);
        aF.SectionShape(out int aNbP, out int aNbK, out int aDeg);
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        aF.Knots(aK);
        aF.Mults(aM);
        NCollection_Array1<Point3D> aP = new(1, aNbP);
        NCollection_Array1<double> aW = new(1, aNbP);
        NCollection_Array1<Point2D> aP2 = new(1, 1);
        double v = 0.8;
        Assert.True(aF.D0(v, 0, 2, aP, aP2, aW));
        BSplineCurve3D anArc = new(aP, aW, aK, aM, aDeg);
        Assert.True(anArc.StartPoint().Distance(aC1.Value(v)) < 1e-9);
        Assert.True(anArc.EndPoint().Distance(aC2.Value(v)) < 1e-9);
        Assert.Equal(0, aF.Nb2dCurves());
        Assert.True(aF.MaximalSection() > 0);

        // trimmed to a sub-interval, the same sections
        aF.SetInterval(0.5, 1.5);
        NCollection_Array1<Point3D> aQ = new(1, aNbP);
        aF.D0(v, 0.5, 1.5, aQ, aP2, aW);
        for (int i = 1; i <= aNbP; i++)
        {
            Assert.True(aQ[i].Distance(aP[i]) < 1e-12);
        }
    }

    [Fact]
    public void SweepSectionGenerator_moves_the_sections_along_the_path()
    {
        TrimmedCurve3D anArc = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 8), 0, 2);
        global::Kerf.Engine.Construction.Surfaces.SweepSectionGenerator aGen = new(anArc, OpenSection(new Point3D(8, 0, 0)));
        aGen.Perform();
        aGen.GetShape(out int aNbP, out _, out _, out int aNb2d);
        Assert.Equal(0, aNb2d);
        int aNbS = aGen.SectionCount;
        Assert.True(aNbS >= 21);
        Assert.Equal(0.0, aGen.Parameter(1));
        Assert.Equal(2.0, aGen.Parameter(aNbS));
        NCollection_Array1<Point3D> aP = new(1, aNbP);
        NCollection_Array1<double> aW = new(1, aNbP);
        NCollection_Array1<Point2D> aP2 = new(1, 1);
        aGen.Section(aNbS, aP, aP2, aW);
        // the last section is the first one rotated by 2 radians about the circle's axis
        Transform3D aRot = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        BSplineCurve3D aFirst = OpenSection(new Point3D(8, 0, 0));
        Assert.True(aP[1].Distance(aFirst.Pole(1).Transformed(aRot)) < 1e-9);
        Assert.Throws<RangeErrorException>(() => aGen.Transformation(aNbS));
    }

    //==================================================================================================
    // GeomFill_Pipe
    //==================================================================================================

    [Fact]
    public void Pipe_of_constant_radius_stays_at_the_radius()
    {
        BSplineCurve3D aPath = Path3d();
        aPath.Freeze();
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, 1.5);
        aPipe.Perform(1e-4, false, C2);
        Assert.True(aPipe.IsDone);
        Assert.Equal(global::Kerf.Engine.Construction.Surfaces.SurfacePipeError.PipeOk, aPipe.GetStatus());
        Surface aS = aPipe.Surface()!;
        Assert.True(aS.IsFrozen);
        aS.Bounds(out double u1, out double u2, out double v1, out double v2);
        ProjectPointOnCurve aProj = new();
        for (int i = 0; i <= 4; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                Point3D aP = aS.Value(u1 + (u2 - u1) * i / 4, v1 + (v2 - v1) * j / 4);
                aProj.Init(aP, aPath);
                Assert.Equal(1.5, aProj.LowerDistance(), 1e-3);
            }
        }
    }

    [Fact]
    public void Pipes_accept_frozen_inputs()
    {
        // every construction reads the caller's curves only: frozen paths and sections build as unfrozen ones
        BSplineCurve3D aPath = ConvexPlanarPath();
        aPath.Freeze();
        CircleCurve3D aS1 = CircleAtStart(aPath, 1);
        CircleCurve3D aS2 = CircleAtStart(aPath, 2);
        BSplineCurve3D aS3 = OpenSection(aPath.StartPoint());
        aS1.Freeze();
        aS2.Freeze();
        aS3.Freeze();
        foreach (global::Kerf.Engine.Construction.Surfaces.Trihedron anOption in new[] { global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsFrenet, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsFixed,
                                                        global::Kerf.Engine.Construction.Surfaces.Trihedron.IsConstantNormal, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsDarboux })
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, aS3, anOption);
            aPipe.Perform(1e-4, false);
            Assert.True(aPipe.IsDone, anOption.ToString());
        }
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe anEvolving = new(aPath, aS1, aS2);
        anEvolving.Perform(1e-4, false);
        Assert.True(anEvolving.IsDone);

        aPath.D1(aPath.LastParameter(), out Point3D anEnd, out Vector3D anEndT);
        CircleCurve3D aS4 = new(new Frame3D(anEnd, new Direction3D(anEndT)), 2);
        aS4.Freeze();
        NCollection_Sequence<Curve3D> aSeq = new();
        aSeq.Append(aS1);
        aSeq.Append(aS4);
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aN = new(aPath, aSeq);
        aN.Perform(1e-4, false);
        Assert.True(aN.IsDone);

        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aDir = new(aPath, aS3, new Direction3D(0, 0, 1));
        aDir.Perform(1e-4, false);
        Assert.True(aDir.IsDone);

        (BSplineCurve3D aP, BSplineCurve3D aC1, BSplineCurve3D aC2) = EdgeCurves(2);
        aP.Freeze();
        aC1.Freeze();
        aC2.Freeze();
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe anEdge = new(aP, aC1, aC2, 2);
        anEdge.Perform(1e-4, false, C2);
        Assert.True(anEdge.IsDone);
        anEdge = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aP, aC1, aC2, 2);
        anEdge.Perform(true);
        Assert.True(anEdge.IsDone);
    }

    [Fact]
    public void Pipe_reports_OCCTs_failures()
    {
        // two sections on a path the corrected Frenet trihedron must correct: OCCT reads SetCurve's flag as a
        // failure (occt-issues.md)
        BSplineCurve3D aPath = Path3d();
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, CircleAtStart(aPath, 1), CircleAtStart(aPath, 2));
        Assert.Equal(global::Kerf.Engine.Construction.Surfaces.SurfacePipeError.ImpossibleContact, aPipe.GetStatus());
        aPipe.Perform(1e-4, false);
        Assert.False(aPipe.IsDone);
        Assert.Null(aPipe.Surface());

        // the old algorithm of a polynomial pipe on an edge: the generator's shape is the quasi-angular one
        (BSplineCurve3D aP, BSplineCurve3D aC1, BSplineCurve3D aC2) = EdgeCurves(2);
        aPipe = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aP, aC1, aC2, 2);
        Assert.Throws<OutOfRangeException>(() => aPipe.Perform(true, true));

        // N sections at one parameter of the path
        NCollection_Sequence<Curve3D> aSeq = new();
        aSeq.Append(CircleAtStart(aPath, 1));
        aSeq.Append(CircleAtStart(aPath, 2));
        Assert.Throws<ConstructionErrorException>(() => new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aPath, aSeq));

        // an option the pipe doesn't build
        Assert.Throws<ConstructionErrorException>(() => new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(aPath, CircleAtStart(aPath, 1), global::Kerf.Engine.Construction.Surfaces.Trihedron.IsGuideAC));
    }

    [Fact]
    public void Pipe_on_an_edge_recognises_cylinders_and_tori()
    {
        // three parallel lines: a cylinder
        TrimmedCurve3D aPath = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 5);
        TrimmedCurve3D aC1 = new(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 0, 5);
        TrimmedCurve3D aC2 = new(new LineCurve3D(new Point3D(0, 2, 0), new Direction3D(0, 0, 1)), 0, 5);
        global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, aC1, aC2, 2);
        aPipe.Perform(true);
        Assert.True(aPipe.IsDone);
        var aCyl = Assert.IsType<TrimmedSurface>(aPipe.Surface());
        Assert.IsType<CylindricalSurface>(aCyl.BasisSurface());
        Assert.False(aPipe.ExchangeUV());

        // three coaxial arcs: a torus
        Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        TrimmedCurve3D anArc = new(new CircleCurve3D(anAx, 10), 0, 1);
        TrimmedCurve3D anArc1 = new(new CircleCurve3D(anAx, 12), 0, 1);
        TrimmedCurve3D anArc2 = new(new CircleCurve3D(anAx.Translated(new Vector3D(0, 0, 2)), 10), 0, 1);
        aPipe = new global::Kerf.Engine.Construction.Surfaces.SurfacePipe(anArc, anArc1, anArc2, 2);
        aPipe.Perform(true);
        Assert.True(aPipe.IsDone);
        var aTor = Assert.IsType<TrimmedSurface>(aPipe.Surface());
        Assert.IsType<ToroidalSurface>(aTor.BasisSurface());
        Assert.True(aPipe.ExchangeUV());
    }

    [Fact]
    public void Pipe_with_a_guide()
    {
        BSplineCurve3D aPath = Path3d(0.5);
        NCollection_Array1<Point3D> aPoles = new(1, aPath.PoleCount);
        for (int i = 1; i <= aPath.PoleCount; i++)
        {
            aPoles[i] = aPath.Pole(i).Translated(new Vector3D(0, 3, 0));
        }
        BSplineCurve3D aGuide = new(aPoles, aPath.InternalKnots(), aPath.InternalMultiplicities(), aPath.Degree());
        foreach (bool byACR in new[] { false, true })
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, new CurveAdaptor(aGuide), CircleAtStart(aPath, 3), byACR, false);
            aPipe.Perform(1e-4, false, C2, 11, 30);
            Assert.True(aPipe.IsDone);
        }
    }
}
