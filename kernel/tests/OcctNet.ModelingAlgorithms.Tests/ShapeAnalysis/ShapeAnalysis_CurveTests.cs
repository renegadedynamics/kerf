// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>ShapeAnalysis_Curve and ShapeExtend_ComplexCurve: known answers on lines, circles, B-splines, trimmed and offset curves.</summary>
public class ShapeAnalysis_CurveTests
{
    private const double Tol = 1e-9;

    private static readonly ShapeAnalysis_Curve SAC = new();

    private static NCollection_Array1<T> Array<T>(params T[] theValues)
    {
        var anArr = new NCollection_Array1<T>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArr[i + 1] = theValues[i];
        }

        return anArr;
    }

    /// <summary>The quadratic Bezier arc (2t, 4t(1-t), 0) on [0, 1], as a B-spline.</summary>
    private static Geom_BSplineCurve Arch()
        => new(Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 2, 0), new gp_Pnt(2, 0, 0)), Array(0.0, 1.0), Array(3, 3), 2);

    private static Geom_Circle Circle(double theR) => new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), theR);

    [Fact]
    public void Project_on_an_infinite_line()
    {
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        double aDist = SAC.Project(aLine, new gp_Pnt(1, 2, 0), 1e-7, out gp_Pnt aProj, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(1.0, aParam, Tol);
        Assert.Equal(1.0, aProj.X(), Tol);
    }

    [Fact]
    public void Project_on_a_circle()
    {
        Geom_Circle aCircle = Circle(2);
        double aDist = SAC.Project(aCircle, new gp_Pnt(0, 5, 0), 1e-7, out gp_Pnt aProj, out double aParam);
        Assert.Equal(3.0, aDist, Tol);
        Assert.Equal(M_PI / 2, aParam, Tol);
        Assert.Equal(2.0, aProj.Y(), Tol);

        aDist = SAC.Project(aCircle, new gp_Pnt(3, 0, 1), 1e-7, out _, out aParam);
        Assert.Equal(Math.Sqrt(2), aDist, Tol);
        Assert.True(aParam < Tol || Math.Abs(aParam - 2 * M_PI) < Tol);
    }

    [Fact]
    public void Project_on_a_bspline_and_its_ends()
    {
        Geom_BSplineCurve anArch = Arch();
        double aDist = SAC.Project(anArch, new gp_Pnt(1, 3, 0), 1e-7, out gp_Pnt aProj, out double aParam);
        Assert.Equal(2.0, aDist, 1e-7);
        Assert.Equal(0.5, aParam, 1e-7);
        Assert.Equal(1.0, aProj.Y(), 1e-7);

        // within preci of an end of a bounded curve: the end, with AdjustToEnds
        aDist = SAC.Project(anArch, new gp_Pnt(0, 0, 0.001), 0.01, out aProj, out aParam);
        Assert.Equal(0.0, aParam);
        Assert.Equal(0.001, aDist, 1e-12);
        aDist = SAC.Project(anArch, new gp_Pnt(2, 0, 0.001), 0.01, out _, out aParam, false);
        Assert.Equal(0.001, aDist, 1e-9);

        // restricted to [0.6, 1]: the nearest end, or the curve point
        aDist = SAC.Project(anArch, new gp_Pnt(1, 3, 0), 1e-7, out aProj, out aParam, 0.6, 1.0);
        Assert.True(aParam >= 0.6 - 0.1, $"param {aParam}");
        Assert.Equal(aProj.Distance(new gp_Pnt(1, 3, 0)), aDist, 1e-12);

        // NextProject from a neighbour finds the same point
        aDist = SAC.NextProject(0.45, anArch, new gp_Pnt(1, 3, 0), 1e-7, out aProj, out aParam, 0.0, 1.0);
        Assert.Equal(2.0, aDist, 1e-7);
        Assert.Equal(0.5, aParam, 1e-7);
    }

    [Fact]
    public void Project_on_trimmed_and_offset_curves()
    {
        var aTrimmed = new Geom_TrimmedCurve(Circle(1), 0, M_PI);
        double aDist = SAC.Project(aTrimmed, new gp_Pnt(0, 3, 0), 1e-7, out _, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(M_PI / 2, aParam, Tol);

        var anOffset = new Geom_OffsetCurve(Circle(1), 0.5, new gp_Dir(0, 0, 1));
        aDist = SAC.Project(anOffset, new gp_Pnt(-3, 0, 0), 1e-7, out gp_Pnt aProj, out aParam);
        Assert.Equal(1.5, aDist, 1e-7);
        Assert.Equal(-1.5, aProj.X(), 1e-7);

        // the adaptor form
        var anAdaptor = new GeomAdaptor_Curve(aTrimmed);
        aDist = SAC.Project(anAdaptor, new gp_Pnt(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(M_PI / 2, aParam, Tol);
        aDist = SAC.NextProject(1.5, anAdaptor, new gp_Pnt(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(M_PI / 2, aParam, 1e-7);
        aDist = SAC.ProjectAct(anAdaptor, new gp_Pnt(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(2.0, aDist, Tol);
    }

    [Fact]
    public void ProjectAct_falls_back_on_elementary_curves()
    {
        // a point on the axis of a circle: every point is a minimum, Extrema reports no solution;
        // the ElCLib fallback takes the start of the circle
        var anAdaptor = new GeomAdaptor_Curve(Circle(2));
        double aDist = SAC.ProjectAct(anAdaptor, new gp_Pnt(0, 0, 0), 1e-7, out gp_Pnt aProj, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(0.0, aParam);
        Assert.Equal(2.0, aProj.X(), Tol);
    }

    [Fact]
    public void ValidateRange_clamps_adjusts_and_orders()
    {
        var aSegment = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 10);
        double aFirst = -5, aLast = 20;
        Assert.True(SAC.ValidateRange(aSegment, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.0, aFirst);
        Assert.Equal(10.0, aLast);

        // periodic: brought into the period
        aFirst = 7;
        aLast  = 8;
        Assert.True(SAC.ValidateRange(Circle(1), ref aFirst, ref aLast, 1e-7));
        Assert.Equal(7 - 2 * M_PI, aFirst, 1e-14);
        Assert.Equal(8 - 2 * M_PI, aLast, 1e-14);

        // a reversed range on an open B-spline reverses the curve
        Geom_BSplineCurve anArch = Arch();
        aFirst = 0.8;
        aLast  = 0.2;
        Assert.True(SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.2, aFirst, 1e-15);
        Assert.Equal(0.8, aLast, 1e-15);
        Assert.Equal(2.0, anArch.StartPoint().X(), 1e-15);

        // equal parameters on an open B-spline: made up, false
        aFirst = aLast = 0.3;
        Assert.False(SAC.ValidateRange(Arch(), ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.0, aFirst);
        Assert.Equal(1.0, aLast);

        // other curves: reversed, then widened when equal; false
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        aFirst = aLast = 2;
        Assert.False(SAC.ValidateRange(aLine, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(2 - Precision.PConfusion(), aFirst);
        Assert.Equal(2 + Precision.PConfusion(), aLast);
    }

    [Fact]
    public void ValidateRange_reversing_a_frozen_curve_throws()
    {
        Geom_BSplineCurve anArch = Arch();
        anArch.Freeze();
        double aFirst = 0.8, aLast = 0.2;
        Assert.Throws<Standard_ImmutableObject>(() => SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));

        // a range needing no reversal reads a frozen curve
        aFirst = 0.2;
        aLast  = 0.8;
        Assert.True(SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));
    }

    [Fact]
    public void ValidateRange_closed_curves()
    {
        // a closed (not periodic) B-spline: a square loop; Last at the origin goes to the end
        var aLoop = new Geom_BSplineCurve(
            Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0), new gp_Pnt(0, 0, 0)),
            Array(0.0, 1.0, 2.0, 3.0, 4.0), Array(2, 1, 1, 1, 2), 1);
        Assert.True(aLoop.IsClosed());
        double aFirst = 2, aLast = 0;
        Assert.True(SAC.ValidateRange(aLoop, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(2.0, aFirst);
        Assert.Equal(4.0, aLast);
        aFirst = 4;
        aLast  = 3;
        Assert.True(SAC.ValidateRange(aLoop, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.0, aFirst);
        Assert.Equal(3.0, aLast);
    }

    [Fact]
    public void ValidateRange_on_a_trimmed_arc_wraps_with_the_trimmed_range_ReproducesOcctError()
    {
        // OCCT: IsPeriodic asks the basis circle, but the period given to ElCLib::AdjustPeriodic is the trimmed range
        // [0, 3]: the reversed range (2, 1) becomes (2, 4), past the end of the arc, where the circle's period would
        // give (2, 1 + 2pi) and a non-periodic curve the reversed (1, 2) of the reversed curve.
        var anArc = new Geom_TrimmedCurve(Circle(1), 0, 3);
        double aFirst = 2, aLast = 1;
        Assert.True(SAC.ValidateRange(anArc, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(2.0, aFirst);
        Assert.Equal(4.0, aLast);
        Assert.True(aLast > anArc.LastParameter());
    }

    [Fact]
    public void FillBndBox_exact_finds_the_extrema()
    {
        var aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3);
        var aBox = new Bnd_Box2d();
        SAC.FillBndBox(aCircle, 0.3, 0.3 + 2 * M_PI, 5, true, ref aBox);
        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
        Assert.Equal(-2.0, aXmin, 1e-9);
        Assert.Equal(4.0, aXmax, 1e-9);
        Assert.Equal(-1.0, aYmin, 1e-9);
        Assert.Equal(5.0, aYmax, 1e-9);

        // not exact: only the sampled points
        var aBox2 = new Bnd_Box2d();
        SAC.FillBndBox(aCircle, 0.3, 0.3 + 2 * M_PI, 3, false, ref aBox2);
        aBox2.Get(out aXmin, out _, out aXmax, out _);
        Assert.True(aXmax < 4.0);
    }

    [Fact]
    public void SelectForwardSeam_tells_the_forward_pcurve()
    {
        var aL1 = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1));
        var aL2 = new Geom2d_Line(new gp_Pnt2d(2 * M_PI, 0), new gp_Dir2d(0, 1));
        Assert.Equal(2, SAC.SelectForwardSeam(aL1, aL2));
        Assert.Equal(1, SAC.SelectForwardSeam(aL2, aL1));
        var aD1 = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, -1));
        Assert.Equal(1, SAC.SelectForwardSeam(aD1, aL2));
        var aU1 = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        var aU2 = new Geom2d_Line(new gp_Pnt2d(0, 1), new gp_Dir2d(1, 0));
        Assert.Equal(1, SAC.SelectForwardSeam(aU1, aU2));

        // bounded curves are taken as the line through their ends
        var aT1 = new Geom2d_TrimmedCurve(aL1, 0, 1);
        Assert.Equal(2, SAC.SelectForwardSeam(aT1, aL2));
        // an unbounded non-line: 0
        var aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1);
        Assert.Equal(0, SAC.SelectForwardSeam(aCircle, aL2));
    }

    [Fact]
    public void IsPlanar_on_points_and_curves()
    {
        gp_XYZ aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(Circle(1), ref aNormal));
        Assert.Equal(1.0, aNormal.Z());

        aNormal = new gp_XYZ(1, 0, 0);
        Assert.False(ShapeAnalysis_Curve.IsPlanar(Circle(1), ref aNormal));

        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(Arch(), ref aNormal));
        Assert.Equal(1.0, Math.Abs(aNormal.Z()), 1e-15);

        var aTwisted = new Geom_BSplineCurve(
            Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(1, 1, 1)),
            Array(0.0, 1.0), Array(4, 4), 3);
        aNormal = new gp_XYZ();
        Assert.False(ShapeAnalysis_Curve.IsPlanar(aTwisted, ref aNormal));

        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), ref aNormal));
        Assert.Equal(1.0, aNormal.Modulus(), 1e-15);
        Assert.Equal(0.0, aNormal.Z(), 1e-15);

        // trimmed and offset curves ask their basis
        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(new Geom_TrimmedCurve(Circle(1), 0, 1), ref aNormal));
        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(new Geom_OffsetCurve(Circle(1), 1, new gp_Dir(0, 0, 1)), ref aNormal));

        // points: collinear ones are planar with any normal
        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 1), new gp_Pnt(2, 2, 2)), ref aNormal));
        Assert.Equal(1.0, aNormal.Modulus(), 1e-15);
        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)), ref aNormal));
        aNormal = new gp_XYZ();
        Assert.False(ShapeAnalysis_Curve.IsPlanar(
            Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0.1)), ref aNormal, 0.01));
        aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(
            Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0.1)), ref aNormal, 0.1));
    }

    [Fact]
    public void GetSamplePoints_counts()
    {
        var aSeq = new NCollection_Sequence<gp_Pnt>();
        Assert.True(ShapeAnalysis_Curve.GetSamplePoints(new Geom_TrimmedCurve(Circle(1), 0, M_PI), 0, M_PI, aSeq));
        Assert.Equal(360, aSeq.Length()); // the basis circle: 360 per period
        Assert.Equal(-1.0, aSeq[360].X(), 1e-15);

        aSeq = new NCollection_Sequence<gp_Pnt>();
        Assert.True(ShapeAnalysis_Curve.GetSamplePoints(Arch(), 0, 1, aSeq));
        Assert.Equal(4, aSeq.Length()); // 2 knots x degree 2

        aSeq = new NCollection_Sequence<gp_Pnt>();
        Assert.True(ShapeAnalysis_Curve.GetSamplePoints(new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 3), 0, 3, aSeq));
        Assert.Equal(2, aSeq.Length());
        Assert.Equal(3.0, aSeq[2].X());

        var aBezier = new Geom_BezierCurve(Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 0, 0)));
        aSeq = new NCollection_Sequence<gp_Pnt>();
        Assert.True(ShapeAnalysis_Curve.GetSamplePoints(aBezier, 0, 1, aSeq));
        Assert.Equal(6, aSeq.Length());

        var aSeq2 = new NCollection_Sequence<gp_Pnt2d>();
        Assert.True(ShapeAnalysis_Curve.GetSamplePoints(new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 0, 2, aSeq2));
        Assert.True(aSeq2.Length() >= 2);
        Assert.Equal(2.0, aSeq2[aSeq2.Length()].X());
    }

    [Fact]
    public void IsClosed_and_IsPeriodic()
    {
        Assert.True(ShapeAnalysis_Curve.IsClosed(Circle(1)));
        Assert.False(ShapeAnalysis_Curve.IsClosed(Arch()));
        Assert.False(ShapeAnalysis_Curve.IsClosed(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0))));
        var anAlmost = new Geom_TrimmedCurve(Circle(1), 0, 2 * M_PI - 1e-4);
        Assert.False(ShapeAnalysis_Curve.IsClosed(anAlmost));
        Assert.True(ShapeAnalysis_Curve.IsClosed(anAlmost, 1e-3));

        Assert.True(ShapeAnalysis_Curve.IsPeriodic(new Geom_TrimmedCurve(Circle(1), 0, 1)));
        Assert.True(ShapeAnalysis_Curve.IsPeriodic(new Geom_OffsetCurve(new Geom_TrimmedCurve(Circle(1), 0, 1), 1, new gp_Dir(0, 0, 1))));
        Assert.False(ShapeAnalysis_Curve.IsPeriodic(Arch()));
        var aCircle2d = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1);
        Assert.True(ShapeAnalysis_Curve.IsPeriodic(new Geom2d_TrimmedCurve(aCircle2d, 0, 1)));
        Assert.False(ShapeAnalysis_Curve.IsPeriodic(new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0))));
    }

    /// <summary>A complex curve of two straight segments of lengths 1 and 3, each on half of [0, 1].</summary>
    private sealed class TwoSegments : ShapeExtend_ComplexCurve
    {
        private readonly Geom_Curve[] myCurves;

        public TwoSegments(gp_Pnt theP0, gp_Pnt theP1, gp_Pnt theP2)
        {
            myCurves =
            [
                new Geom_TrimmedCurve(new Geom_Line(theP0, new gp_Dir(new gp_Vec(theP0, theP1))), 0, theP0.Distance(theP1)),
                new Geom_TrimmedCurve(new Geom_Line(theP1, new gp_Dir(new gp_Vec(theP1, theP2))), 0, theP1.Distance(theP2)),
            ];
        }

        public override int NbCurves() => 2;

        public override Geom_Curve Curve(int index) => myCurves[index - 1];

        public override int LocateParameter(double U, out double UOut)
        {
            int anInd = U < 0.5 ? 1 : 2;
            Geom_Curve aC = myCurves[anInd - 1];
            double aLocal = anInd == 1 ? U * 2 : (U - 0.5) * 2;
            UOut = aC.FirstParameter() + aLocal * (aC.LastParameter() - aC.FirstParameter());
            return anInd;
        }

        public override double LocalToGlobal(int index, double Ulocal)
        {
            Geom_Curve aC = myCurves[index - 1];
            double aLocal = (Ulocal - aC.FirstParameter()) / (aC.LastParameter() - aC.FirstParameter());
            return 0.5 * (index - 1) + 0.5 * aLocal;
        }

        public override double GetScaleFactor(int ind)
        {
            Geom_Curve aC = myCurves[ind - 1];
            return 2 * (aC.LastParameter() - aC.FirstParameter());
        }

        public bool Check(double thePreci) => CheckConnectivity(thePreci);

        protected internal override void Reverse() => throw new Standard_NotImplemented();

        public override Geom_Geometry Copy() => new TwoSegments(myCurves[0].Value(myCurves[0].FirstParameter()),
                                                                 myCurves[1].Value(myCurves[1].FirstParameter()),
                                                                 myCurves[1].Value(myCurves[1].LastParameter()));
    }

    [Fact]
    public void ComplexCurve_evaluates_its_segments()
    {
        var aCurve = new TwoSegments(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 3, 0));
        Assert.Equal(0.0, aCurve.FirstParameter());
        Assert.Equal(1.0, aCurve.LastParameter());
        Assert.Equal(0.25, aCurve.ReversedParameter(0.75));
        Assert.Equal(GeomAbs_Shape.GeomAbs_C0, aCurve.Continuity());
        Assert.True(aCurve.IsCN(0));
        Assert.False(aCurve.IsCN(1));
        Assert.False(aCurve.IsPeriodic());

        gp_Pnt aP = aCurve.Value(0.75);
        Assert.Equal(1.0, aP.X(), 1e-15);
        Assert.Equal(1.5, aP.Y(), 1e-15);

        // derivatives scaled by the segment's factor: d/dU of the second segment is 2 x 3 along Y
        aCurve.D1(0.75, out _, out gp_Vec aV1);
        Assert.Equal(6.0, aV1.Y(), 1e-15);
        aCurve.D2(0.25, out _, out aV1, out gp_Vec aV2);
        Assert.Equal(2.0, aV1.X(), 1e-15);
        Assert.Equal(0.0, aV2.Magnitude(), 1e-15);
        Assert.Equal(6.0, aCurve.DN(0.75, 1).Y(), 1e-15);
        aCurve.D3(0.75, out _, out aV1, out _, out _);
        Assert.Equal(6.0, aV1.Y(), 1e-15);

        Assert.True(aCurve.Check(1e-7));
        Assert.False(aCurve.IsClosed());

        gp_XYZ aNormal = new gp_XYZ();
        Assert.True(ShapeAnalysis_Curve.IsPlanar(aCurve, ref aNormal));
        Assert.Equal(1.0, Math.Abs(aNormal.Z()), 1e-15);

        // Transform applies to each segment
        var aT = new gp_Trsf();
        aT.SetTranslation(new gp_Vec(0, 0, 5));
        var aMoved = (ShapeExtend_ComplexCurve)aCurve.Transformed(aT);
        Assert.Equal(5.0, aMoved.Value(0.75).Z(), 1e-15);
        Assert.Equal(0.0, aCurve.Value(0.75).Z());
    }
}
