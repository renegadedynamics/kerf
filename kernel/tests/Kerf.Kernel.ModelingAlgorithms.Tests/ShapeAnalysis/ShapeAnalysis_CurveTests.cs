// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>ShapeAnalysis_Curve and ShapeExtend_ComplexCurve: known answers on lines, circles, B-splines, trimmed and offset curves.</summary>
public class ShapeAnalysis_CurveTests
{
    private const double Tol = 1e-9;

    private static readonly CurveAnalysis SAC = new();

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
    private static BSplineCurve3D Arch()
        => new(Array(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0)), Array(0.0, 1.0), Array(3, 3), 2);

    private static CircleCurve3D Circle(double theR) => new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theR);

    [Fact]
    public void Project_on_an_infinite_line()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        double aDist = SAC.Project(aLine, new Point3D(1, 2, 0), 1e-7, out Point3D aProj, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(1.0, aParam, Tol);
        Assert.Equal(1.0, aProj.X, Tol);
    }

    [Fact]
    public void Project_on_a_circle()
    {
        CircleCurve3D aCircle = Circle(2);
        double aDist = SAC.Project(aCircle, new Point3D(0, 5, 0), 1e-7, out Point3D aProj, out double aParam);
        Assert.Equal(3.0, aDist, Tol);
        Assert.Equal(M_PI / 2, aParam, Tol);
        Assert.Equal(2.0, aProj.Y, Tol);

        aDist = SAC.Project(aCircle, new Point3D(3, 0, 1), 1e-7, out _, out aParam);
        Assert.Equal(Math.Sqrt(2), aDist, Tol);
        Assert.True(aParam < Tol || Math.Abs(aParam - 2 * M_PI) < Tol);
    }

    [Fact]
    public void Project_on_a_bspline_and_its_ends()
    {
        BSplineCurve3D anArch = Arch();
        double aDist = SAC.Project(anArch, new Point3D(1, 3, 0), 1e-7, out Point3D aProj, out double aParam);
        Assert.Equal(2.0, aDist, 1e-7);
        Assert.Equal(0.5, aParam, 1e-7);
        Assert.Equal(1.0, aProj.Y, 1e-7);

        // within preci of an end of a bounded curve: the end, with AdjustToEnds
        aDist = SAC.Project(anArch, new Point3D(0, 0, 0.001), 0.01, out aProj, out aParam);
        Assert.Equal(0.0, aParam);
        Assert.Equal(0.001, aDist, 1e-12);
        aDist = SAC.Project(anArch, new Point3D(2, 0, 0.001), 0.01, out _, out aParam, false);
        Assert.Equal(0.001, aDist, 1e-9);

        // restricted to [0.6, 1]: the nearest end, or the curve point
        aDist = SAC.Project(anArch, new Point3D(1, 3, 0), 1e-7, out aProj, out aParam, 0.6, 1.0);
        Assert.True(aParam >= 0.6 - 0.1, $"param {aParam}");
        Assert.Equal(aProj.DistanceTo(new Point3D(1, 3, 0)), aDist, 1e-12);

        // NextProject from a neighbour finds the same point
        aDist = SAC.NextProject(0.45, anArch, new Point3D(1, 3, 0), 1e-7, out aProj, out aParam, 0.0, 1.0);
        Assert.Equal(2.0, aDist, 1e-7);
        Assert.Equal(0.5, aParam, 1e-7);
    }

    [Fact]
    public void Project_on_trimmed_and_offset_curves()
    {
        var aTrimmed = new TrimmedCurve3D(Circle(1), 0, M_PI);
        double aDist = SAC.Project(aTrimmed, new Point3D(0, 3, 0), 1e-7, out _, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(M_PI / 2, aParam, Tol);

        var anOffset = new OffsetCurve3D(Circle(1), 0.5, new Direction3D(0, 0, 1));
        aDist = SAC.Project(anOffset, new Point3D(-3, 0, 0), 1e-7, out Point3D aProj, out aParam);
        Assert.Equal(1.5, aDist, 1e-7);
        Assert.Equal(-1.5, aProj.X, 1e-7);

        // the adaptor form
        var anAdaptor = new CurveAdaptor(aTrimmed);
        aDist = SAC.Project(anAdaptor, new Point3D(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(M_PI / 2, aParam, Tol);
        aDist = SAC.NextProject(1.5, anAdaptor, new Point3D(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(M_PI / 2, aParam, 1e-7);
        aDist = SAC.ProjectAct(anAdaptor, new Point3D(0, 3, 0), 1e-7, out _, out aParam);
        Assert.Equal(2.0, aDist, Tol);
    }

    [Fact]
    public void ProjectAct_falls_back_on_elementary_curves()
    {
        // a point on the axis of a circle: every point is a minimum, Extrema reports no solution;
        // the ElCLib fallback takes the start of the circle
        var anAdaptor = new CurveAdaptor(Circle(2));
        double aDist = SAC.ProjectAct(anAdaptor, new Point3D(0, 0, 0), 1e-7, out Point3D aProj, out double aParam);
        Assert.Equal(2.0, aDist, Tol);
        Assert.Equal(0.0, aParam);
        Assert.Equal(2.0, aProj.X, Tol);
    }

    [Fact]
    public void ValidateRange_clamps_adjusts_and_orders()
    {
        var aSegment = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10);
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
        BSplineCurve3D anArch = Arch();
        aFirst = 0.8;
        aLast  = 0.2;
        Assert.True(SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.2, aFirst, 1e-15);
        Assert.Equal(0.8, aLast, 1e-15);
        Assert.Equal(2.0, anArch.StartPoint.X, 1e-15);

        // equal parameters on an open B-spline: made up, false
        aFirst = aLast = 0.3;
        Assert.False(SAC.ValidateRange(Arch(), ref aFirst, ref aLast, 1e-7));
        Assert.Equal(0.0, aFirst);
        Assert.Equal(1.0, aLast);

        // other curves: reversed, then widened when equal; false
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        aFirst = aLast = 2;
        Assert.False(SAC.ValidateRange(aLine, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(2 - global::Kerf.Engine.Foundation.Precision.PConfusion(), aFirst);
        Assert.Equal(2 + global::Kerf.Engine.Foundation.Precision.PConfusion(), aLast);
    }

    [Fact]
    public void ValidateRange_reversing_a_frozen_curve_throws()
    {
        BSplineCurve3D anArch = Arch();
        anArch.Freeze();
        double aFirst = 0.8, aLast = 0.2;
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));

        // a range needing no reversal reads a frozen curve
        aFirst = 0.2;
        aLast  = 0.8;
        Assert.True(SAC.ValidateRange(anArch, ref aFirst, ref aLast, 1e-7));
    }

    [Fact]
    public void ValidateRange_closed_curves()
    {
        // a closed (not periodic) B-spline: a square loop; Last at the origin goes to the end
        var aLoop = new BSplineCurve3D(
            Array(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0), new Point3D(0, 0, 0)),
            Array(0.0, 1.0, 2.0, 3.0, 4.0), Array(2, 1, 1, 1, 2), 1);
        Assert.True(aLoop.IsClosed);
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
        var anArc = new TrimmedCurve3D(Circle(1), 0, 3);
        double aFirst = 2, aLast = 1;
        Assert.True(SAC.ValidateRange(anArc, ref aFirst, ref aLast, 1e-7));
        Assert.Equal(2.0, aFirst);
        Assert.Equal(4.0, aLast);
        Assert.True(aLast > anArc.LastParameter);
    }

    [Fact]
    public void FillBndBox_exact_finds_the_extrema()
    {
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(1, 2), new Direction2D(1, 0)), 3);
        var aBox = new Box2d();
        SAC.FillBndBox(aCircle, 0.3, 0.3 + 2 * M_PI, 5, true, ref aBox);
        aBox.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
        Assert.Equal(-2.0, aXmin, 1e-9);
        Assert.Equal(4.0, aXmax, 1e-9);
        Assert.Equal(-1.0, aYmin, 1e-9);
        Assert.Equal(5.0, aYmax, 1e-9);

        // not exact: only the sampled points
        var aBox2 = new Box2d();
        SAC.FillBndBox(aCircle, 0.3, 0.3 + 2 * M_PI, 3, false, ref aBox2);
        aBox2.Get(out aXmin, out _, out aXmax, out _);
        Assert.True(aXmax < 4.0);
    }

    [Fact]
    public void SelectForwardSeam_tells_the_forward_pcurve()
    {
        var aL1 = new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1));
        var aL2 = new LineCurve2D(new Point2D(2 * M_PI, 0), new Direction2D(0, 1));
        Assert.Equal(2, SAC.SelectForwardSeam(aL1, aL2));
        Assert.Equal(1, SAC.SelectForwardSeam(aL2, aL1));
        var aD1 = new LineCurve2D(new Point2D(0, 0), new Direction2D(0, -1));
        Assert.Equal(1, SAC.SelectForwardSeam(aD1, aL2));
        var aU1 = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        var aU2 = new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0));
        Assert.Equal(1, SAC.SelectForwardSeam(aU1, aU2));

        // bounded curves are taken as the line through their ends
        var aT1 = new TrimmedCurve2D(aL1, 0, 1);
        Assert.Equal(2, SAC.SelectForwardSeam(aT1, aL2));
        // an unbounded non-line: 0
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        Assert.Equal(0, SAC.SelectForwardSeam(aCircle, aL2));
    }

    [Fact]
    public void IsPlanar_on_points_and_curves()
    {
        Xyz aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(Circle(1), ref aNormal));
        Assert.Equal(1.0, aNormal.Z());

        aNormal = new Xyz(1, 0, 0);
        Assert.False(CurveAnalysis.IsPlanar(Circle(1), ref aNormal));

        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(Arch(), ref aNormal));
        Assert.Equal(1.0, Math.Abs(aNormal.Z()), 1e-15);

        var aTwisted = new BSplineCurve3D(
            Array(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 1, 1)),
            Array(0.0, 1.0), Array(4, 4), 3);
        aNormal = new Xyz();
        Assert.False(CurveAnalysis.IsPlanar(aTwisted, ref aNormal));

        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), ref aNormal));
        Assert.Equal(1.0, aNormal.Modulus(), 1e-15);
        Assert.Equal(0.0, aNormal.Z(), 1e-15);

        // trimmed and offset curves ask their basis
        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(new TrimmedCurve3D(Circle(1), 0, 1), ref aNormal));
        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(new OffsetCurve3D(Circle(1), 1, new Direction3D(0, 0, 1)), ref aNormal));

        // points: collinear ones are planar with any normal
        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(Array(new Point3D(0, 0, 0), new Point3D(1, 1, 1), new Point3D(2, 2, 2)), ref aNormal));
        Assert.Equal(1.0, aNormal.Modulus(), 1e-15);
        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(Array(new Point3D(0, 0, 0), new Point3D(1, 0, 0)), ref aNormal));
        aNormal = new Xyz();
        Assert.False(CurveAnalysis.IsPlanar(
            Array(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0.1)), ref aNormal, 0.01));
        aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(
            Array(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0.1)), ref aNormal, 0.1));
    }

    [Fact]
    public void GetSamplePoints_counts()
    {
        var aSeq = new NCollection_Sequence<Point3D>();
        Assert.True(CurveAnalysis.GetSamplePoints(new TrimmedCurve3D(Circle(1), 0, M_PI), 0, M_PI, aSeq));
        Assert.Equal(360, aSeq.Length()); // the basis circle: 360 per period
        Assert.Equal(-1.0, aSeq[360].X, 1e-15);

        aSeq = new NCollection_Sequence<Point3D>();
        Assert.True(CurveAnalysis.GetSamplePoints(Arch(), 0, 1, aSeq));
        Assert.Equal(4, aSeq.Length()); // 2 knots x degree 2

        aSeq = new NCollection_Sequence<Point3D>();
        Assert.True(CurveAnalysis.GetSamplePoints(new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 3), 0, 3, aSeq));
        Assert.Equal(2, aSeq.Length());
        Assert.Equal(3.0, aSeq[2].X);

        var aBezier = new BezierCurve3D(Array(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)));
        aSeq = new NCollection_Sequence<Point3D>();
        Assert.True(CurveAnalysis.GetSamplePoints(aBezier, 0, 1, aSeq));
        Assert.Equal(6, aSeq.Length());

        var aSeq2 = new NCollection_Sequence<Point2D>();
        Assert.True(CurveAnalysis.GetSamplePoints(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 2, aSeq2));
        Assert.True(aSeq2.Length() >= 2);
        Assert.Equal(2.0, aSeq2[aSeq2.Length()].X);
    }

    [Fact]
    public void IsClosed_and_IsPeriodic()
    {
        Assert.True(CurveAnalysis.IsClosed(Circle(1)));
        Assert.False(CurveAnalysis.IsClosed(Arch()));
        Assert.False(CurveAnalysis.IsClosed(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))));
        var anAlmost = new TrimmedCurve3D(Circle(1), 0, 2 * M_PI - 1e-4);
        Assert.False(CurveAnalysis.IsClosed(anAlmost));
        Assert.True(CurveAnalysis.IsClosed(anAlmost, 1e-3));

        Assert.True(CurveAnalysis.IsPeriodic(new TrimmedCurve3D(Circle(1), 0, 1)));
        Assert.True(CurveAnalysis.IsPeriodic(new OffsetCurve3D(new TrimmedCurve3D(Circle(1), 0, 1), 1, new Direction3D(0, 0, 1))));
        Assert.False(CurveAnalysis.IsPeriodic(Arch()));
        var aCircle2d = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        Assert.True(CurveAnalysis.IsPeriodic(new TrimmedCurve2D(aCircle2d, 0, 1)));
        Assert.False(CurveAnalysis.IsPeriodic(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0))));
    }

    /// <summary>A complex curve of two straight segments of lengths 1 and 3, each on half of [0, 1].</summary>
    private sealed class TwoSegments : ComplexCurve
    {
        private readonly Curve3D[] myCurves;

        public TwoSegments(Point3D theP0, Point3D theP1, Point3D theP2)
        {
            myCurves =
            [
                new TrimmedCurve3D(new LineCurve3D(theP0, new Direction3D(new Vector3D(theP0, theP1))), 0, theP0.DistanceTo(theP1)),
                new TrimmedCurve3D(new LineCurve3D(theP1, new Direction3D(new Vector3D(theP1, theP2))), 0, theP1.DistanceTo(theP2)),
            ];
        }

        public override int CurveCount => 2;

        public override Curve3D Curve(int index) => myCurves[index - 1];

        public override int LocateParameter(double U, out double UOut)
        {
            int anInd = U < 0.5 ? 1 : 2;
            Curve3D aC = myCurves[anInd - 1];
            double aLocal = anInd == 1 ? U * 2 : (U - 0.5) * 2;
            UOut = aC.FirstParameter + aLocal * (aC.LastParameter - aC.FirstParameter);
            return anInd;
        }

        public override double LocalToGlobal(int index, double Ulocal)
        {
            Curve3D aC = myCurves[index - 1];
            double aLocal = (Ulocal - aC.FirstParameter) / (aC.LastParameter - aC.FirstParameter);
            return 0.5 * (index - 1) + 0.5 * aLocal;
        }

        public override double GetScaleFactor(int ind)
        {
            Curve3D aC = myCurves[ind - 1];
            return 2 * (aC.LastParameter - aC.FirstParameter);
        }

        public bool Check(double thePreci) => CheckConnectivity(thePreci);

        protected internal override void Reverse() => throw new global::Kerf.Engine.Foundation.MethodNotImplementedException();

        public override Geometry3D Copy() => new TwoSegments(myCurves[0].Value(myCurves[0].FirstParameter),
                                                                 myCurves[1].Value(myCurves[1].FirstParameter),
                                                                 myCurves[1].Value(myCurves[1].LastParameter));
    }

    [Fact]
    public void ComplexCurve_evaluates_its_segments()
    {
        var aCurve = new TwoSegments(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 3, 0));
        Assert.Equal(0.0, aCurve.FirstParameter);
        Assert.Equal(1.0, aCurve.LastParameter);
        Assert.Equal(0.25, aCurve.ReversedParameter(0.75));
        Assert.Equal(Continuity.C0, aCurve.Continuity);
        Assert.True(aCurve.IsCN(0));
        Assert.False(aCurve.IsCN(1));
        Assert.False(aCurve.IsPeriodic);

        Point3D aP = aCurve.Value(0.75);
        Assert.Equal(1.0, aP.X, 1e-15);
        Assert.Equal(1.5, aP.Y, 1e-15);

        // derivatives scaled by the segment's factor: d/dU of the second segment is 2 x 3 along Y
        aCurve.D1(0.75, out _, out Vector3D aV1);
        Assert.Equal(6.0, aV1.Y, 1e-15);
        aCurve.D2(0.25, out _, out aV1, out Vector3D aV2);
        Assert.Equal(2.0, aV1.X, 1e-15);
        Assert.Equal(0.0, aV2.Magnitude, 1e-15);
        Assert.Equal(6.0, aCurve.DN(0.75, 1).Y, 1e-15);
        aCurve.D3(0.75, out _, out aV1, out _, out _);
        Assert.Equal(6.0, aV1.Y, 1e-15);

        Assert.True(aCurve.Check(1e-7));
        Assert.False(aCurve.IsClosed);

        Xyz aNormal = new Xyz();
        Assert.True(CurveAnalysis.IsPlanar(aCurve, ref aNormal));
        Assert.Equal(1.0, Math.Abs(aNormal.Z()), 1e-15);

        // Transform applies to each segment
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 0, 5));
        var aMoved = (ComplexCurve)aCurve.Transformed(aT);
        Assert.Equal(5.0, aMoved.Value(0.75).Z, 1e-15);
        Assert.Equal(0.0, aCurve.Value(0.75).Z);
    }
}
