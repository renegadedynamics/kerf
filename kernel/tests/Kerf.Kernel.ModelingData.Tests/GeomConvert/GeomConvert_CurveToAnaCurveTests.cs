// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Geometry.Conversion.ConvType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// GeomConvert_CurveToAnaCurve (OCCT has no GTests for it; the curve cases of
/// ShapeAnalysis_CanonicalRecognition_Test.cxx, which reach it through ShapeAnalysis, are adapted here).
/// </summary>
public class GeomConvert_CurveToAnaCurveTests
{
    private static readonly Frame3D s_Ax2 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.3, 0.4, 1.0), new Direction3D(1.0, 0.0, -0.3));

    // An ellipse whose major axis is at 15 degrees from the X axis of the frame ComputeEllipse builds: OCCT
    // recognises it (see ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError)
    private static Frame3D EllipseAx2(double theAngle)
        => new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.0, 0.0, 1.0), new Direction3D(Math.Cos(theAngle), Math.Sin(theAngle), 0.0));

    /// <summary>A rational B-spline of a conic, built as GeomConvert's BSplineCurveBuilder does.</summary>
    private static BSplineCurve3D Lift(ConicToBSplineCurve theConvert, in Frame3D thePosition)
    {
        NCollection_Array1<Point2D> aPoles2d = theConvert.Poles();
        var aPoles = new NCollection_Array1<Point3D>(1, aPoles2d.Length());
        Transform3D aT = Transform3D.Transformation(new CoordinateSystem(thePosition), new CoordinateSystem(Constants.XOY()));
        for (int i = aPoles2d.Lower(); i <= aPoles2d.Upper(); i++)
        {
            aPoles[i] = new Point3D(aPoles2d[i].X, aPoles2d[i].Y, 0.0).Transformed(aT);
        }

        return new BSplineCurve3D(aPoles, theConvert.Weights(), theConvert.Knots(), theConvert.Multiplicities(),
                                     theConvert.Degree(), theConvert.IsPeriodic);
    }

    private static BSplineCurve3D CircleArc(double theR, double theU1, double theU2)
        => Lift(new CircleToBSplineCurve(new Circle2D(Constants.OX2d(), theR), theU1, theU2), s_Ax2);

    private static BSplineCurve3D EllipseArc(double theR1, double theR2, double theU1, double theU2, double theAngle = Math.PI / 12)
        => Lift(new EllipseToBSplineCurve(new Ellipse2D(Constants.OX2d(), theR1, theR2), theU1, theU2), EllipseAx2(theAngle));

    private static NCollection_Array1<Point3D> Points(params Point3D[] thePoints)
    {
        var anArray = new NCollection_Array1<Point3D>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; i++)
        {
            anArray[i + 1] = thePoints[i];
        }

        return anArray;
    }

    [Fact]
    public void IsLinear()
    {
        double aDev = -1.0;
        Assert.True(CurveToAnaCurve.IsLinear(
            Points(new Point3D(0, 0, 0), new Point3D(1, 1, 1), new Point3D(2, 2, 2.001)), 1e-2, ref aDev));
        Assert.True(aDev > 0.0 && aDev < 1e-3);

        aDev = -1.0;
        Assert.False(CurveToAnaCurve.IsLinear(
            Points(new Point3D(0, 0, 0), new Point3D(1, 0.1, 0), new Point3D(2, 0, 0)), 1e-2, ref aDev));
        Assert.False(CurveToAnaCurve.IsLinear(Points(new Point3D(0, 0, 0)), 1e-2, ref aDev));
        Assert.False(CurveToAnaCurve.IsLinear(Points(new Point3D(0, 0, 0), new Point3D(0, 0, 1e-9)), 1e-2, ref aDev));
        Assert.Equal(-1.0, aDev);
    }

    [Fact]
    public void GetLine_And_GetCircle()
    {
        Line3D aLin = CurveToAnaCurve.GetLine(new Point3D(1, 2, 3), new Point3D(1, 2, 7), out double cf, out double cl);
        Assert.Equal(0.0, cf, 12);
        Assert.Equal(4.0, cl, 12);
        Assert.True(aLin.Direction.IsEqual(new Direction3D(0, 0, 1), 1e-12));

        var aCirc = new Circle3D();
        var aP0 = new Point3D(3, 1, 0);
        Assert.True(CurveToAnaCurve.GetCircle(ref aCirc, aP0, new Point3D(1, 3, 0), new Point3D(-1, 1, 0)));
        Assert.Equal(2.0, aCirc.Radius, 12);
        Assert.True(aCirc.Location.DistanceTo(new Point3D(1, 1, 0)) < 1e-12);
        // The X axis points to P0
        Assert.True(aCirc.XAxis.Direction.IsEqual(new Direction3D(1, 0, 0), 1e-12));

        var aSame = new Circle3D();
        Assert.False(CurveToAnaCurve.GetCircle(ref aSame, new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)));
        Assert.False(CurveToAnaCurve.GetCircle(ref aSame, new Point3D(1e60, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 1, 0)));
    }

    [Fact]
    public void ComputeLine()
    {
        // A degree 2 B-spline with collinear poles
        var aPoles = Points(new Point3D(0, 0, 0), new Point3D(1, 2, 3), new Point3D(3, 6, 9));
        var aBez = new BezierCurve3D(aPoles);
        double cf = 0, cl = 0, aDev = -1;
        LineCurve3D? aLine = CurveToAnaCurve.ComputeLine(aBez, 1e-7, 0.0, 1.0, ref cf, ref cl, ref aDev);
        Assert.NotNull(aLine);
        Assert.Equal(0.0, cf, 12);
        Assert.Equal(Math.Sqrt(126.0), cl, 12);
        Assert.Equal(0.0, aDev, 12);

        // A circle arc is not linear; neither is a closed curve
        Assert.Null(CurveToAnaCurve.ComputeLine(CircleArc(2.0, 0.0, 1.0), 1e-3, 0.0, 1.0, ref cf, ref cl, ref aDev));

        // A line is returned as it is
        var aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Assert.Same(aGeomLine, CurveToAnaCurve.ComputeLine(aGeomLine, 1e-7, -1.0, 5.0, ref cf, ref cl, ref aDev));
        Assert.Equal(-1.0, cf);
        Assert.Equal(5.0, cl);
    }

    [Fact]
    public void ComputeCircle_FromBSplineArc()
    {
        BSplineCurve3D anArc = CircleArc(2.5, 0.4, 2.9);
        double cf = 0, cl = 0, aDev = -1;
        Curve3D? aRes = CurveToAnaCurve.ComputeCircle(anArc, 1e-7, anArc.FirstParameter, anArc.LastParameter,
                                                                     ref cf, ref cl, ref aDev);
        CircleCurve3D aCircle = Assert.IsType<CircleCurve3D>(aRes);
        Assert.Equal(2.5, aCircle.Radius, 10);
        Assert.True(aCircle.Location.DistanceTo(s_Ax2.Location) < 1e-10);
        Assert.True(aDev < 1e-10);
        Assert.Equal(0.0, cf, 12); // the X axis is put at the start point
        Assert.Equal(2.5, cl, 9);
        Assert.True(aCircle.Value(cl).DistanceTo(anArc.Value(anArc.LastParameter)) < 1e-9);

        // Too strict a tolerance for a perturbed arc
        BSplineCurve3D aPerturbed = anArc.WithPole(3, anArc.Pole(3).Translated(new Vector3D(0.0, 0.0, 0.01)));
        Assert.Null(CurveToAnaCurve.ComputeCircle(aPerturbed, 1e-5, aPerturbed.FirstParameter,
                                                              aPerturbed.LastParameter, ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeEllipse_FromBSplineArc()
    {
        BSplineCurve3D anArc = EllipseArc(3.0, 1.2, 0.3, 2.5);
        double cf = 0, cl = 0, aDev = -1;
        Curve3D? aRes = CurveToAnaCurve.ComputeEllipse(anArc, 1e-6, anArc.FirstParameter, anArc.LastParameter,
                                                                      ref cf, ref cl, ref aDev);
        EllipseCurve3D anEllipse = Assert.IsType<EllipseCurve3D>(aRes);
        Assert.Equal(3.0, anEllipse.MajorRadius, 7);
        Assert.Equal(1.2, anEllipse.MinorRadius, 7);
        Assert.True(anEllipse.Location.DistanceTo(new Point3D(1.0, -2.0, 0.5)) < 1e-7);
        Assert.True(aDev < 1e-6);
        Assert.True(anEllipse.Value(cf).DistanceTo(anArc.Value(anArc.FirstParameter)) < 1e-6);
        Assert.True(anEllipse.Value(cl).DistanceTo(anArc.Value(anArc.LastParameter)) < 1e-6);

        // A circle is not taken for an ellipse
        BSplineCurve3D aCircle = CircleArc(2.0, 0.0, 2.0);
        Assert.Null(CurveToAnaCurve.ComputeEllipse(aCircle, 1e-6, 0.0, 2.0, ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError()
    {
        // ConicDefinition loses the sign of the rotation and takes the axis within 45 degrees of the frame's X axis
        // (occt-issues.md): the same arc rotated by 60 degrees is not recognised
        BSplineCurve3D anArc = EllipseArc(3.0, 1.2, 0.3, 2.5, Math.PI / 3);
        double cf = 0, cl = 0, aDev = -1;
        Assert.Null(CurveToAnaCurve.ComputeEllipse(anArc, 1e-6, anArc.FirstParameter, anArc.LastParameter,
                                                               ref cf, ref cl, ref aDev));
        Assert.Null(CurveToAnaCurve.ComputeEllipse(anArc, 1e-1, anArc.FirstParameter, anArc.LastParameter,
                                                               ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeCurve_ConversionTypes()
    {
        BSplineCurve3D anEllipse = EllipseArc(3.0, 1.2, 0.3, 2.5);
        double aU1 = anEllipse.FirstParameter, aU2 = anEllipse.LastParameter;
        double aGap = 0.0;

        Curve3D? aMin = CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out _, out _, ref aGap);
        Assert.IsType<EllipseCurve3D>(aMin);
        Assert.True(aGap >= 0.0 && aGap < 1e-6);

        Curve3D? aSimplest = CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out _, out _, ref aGap,
                                                                         Simplest);
        Assert.IsType<EllipseCurve3D>(aSimplest);

        // Target circle: not a circle within the tolerance
        Assert.Null(CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out double cf, out double cl, ref aGap,
                                                             Target, Circle));
        Assert.Equal(RealLast(), aGap);

        // MinGap failing: gap -1 and the input parameters
        BSplineCurve3D aWavy = anEllipse.WithPole(3, anEllipse.Pole(3).Translated(new Vector3D(0.1, 0.0, 0.3)));
        Assert.Null(CurveToAnaCurve.ComputeCurve(aWavy, 1e-6, aU1, aU2, out cf, out cl, ref aGap));
        Assert.Equal(-1.0, aGap);
        Assert.Equal(aU1, cf);
        Assert.Equal(aU2, cl);

        // A circle arc: MinGap prefers the circle (exact) to the ellipse (not found)
        BSplineCurve3D aCircle = CircleArc(1.5, 1.0, 4.0);
        Assert.IsType<CircleCurve3D>(CurveToAnaCurve.ComputeCurve(aCircle, 1e-6, 1.0, 4.0, out _, out _, ref aGap));
    }

    [Fact]
    public void ConvertToAnalytical_UsesTheBasisOfATrimmedCurve()
    {
        var aCircle = new CircleCurve3D(s_Ax2, 2.0);
        var aTrimmed = new TrimmedCurve3D(new TrimmedCurve3D(aCircle, 0.5, 2.0), 0.7, 1.5);
        var aConv = new CurveToAnaCurve(aTrimmed);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aConv.Gap());
        Curve3D? aRes = null;
        Assert.True(aConv.ConvertToAnalytical(1e-7, ref aRes, 0.7, 1.5, out double aNewF, out double aNewL));
        CircleCurve3D aResCircle = Assert.IsType<CircleCurve3D>(aRes);
        Assert.Same(aTrimmed.BasisCurve, aResCircle);
        Assert.Equal(0.7, aNewF);
        Assert.Equal(1.5, aNewL);
        Assert.Equal(0.0, aConv.Gap());

        Assert.False(new CurveToAnaCurve().ConvertToAnalytical(1e-7, ref aRes, 0.0, 1.0, out _, out _));

        aConv.SetConvType(Target);
        aConv.SetTarget(Ellipse);
        Assert.Equal(Target, aConv.GetConvType());
        Assert.Equal(Ellipse, aConv.GetTarget());
        Curve3D? aNone = null;
        Assert.False(aConv.ConvertToAnalytical(1e-7, ref aNone, 0.7, 1.5, out _, out _));
        Assert.Null(aNone);
    }

    // ShapeAnalysis_CanonicalRecognition_Test.cxx, cr/base A1 and A2: a nearly linear Bezier curve
    [Fact]
    public void BezierToLineRecognition_A1()
    {
        var aBezier = new BezierCurve3D(Points(new Point3D(0, 0, 0), new Point3D(0.5, 0.0005, 0), new Point3D(1, 0, 0)));
        double aGap = 0.0;
        Curve3D? aRes = CurveToAnaCurve.ComputeCurve(aBezier, 1.0e-3, 0.0, 1.0, out _, out _, ref aGap,
                                                                    Target, Line);
        LineCurve3D aLine = Assert.IsType<LineCurve3D>(aRes);
        Assert.Equal(1.0, Math.Abs(aLine.Line.Direction.X), 1e-3);
    }

    [Fact]
    public void BezierToCircleRecognition_A2()
    {
        var aBezier = new BezierCurve3D(Points(new Point3D(0, 0, 0), new Point3D(0.5, 0.0005, 0), new Point3D(1, 0, 0)));
        double aGap = 0.0;
        Curve3D? aRes = CurveToAnaCurve.ComputeCurve(aBezier, 1.0e-3, 0.0, 1.0, out _, out _, ref aGap,
                                                                    Target, Circle);
        CircleCurve3D aCircle = Assert.IsType<CircleCurve3D>(aRes);
        Assert.True(aCircle.Radius > 0.0);
    }

    [Fact]
    public void EllipseToEllipseRecognition_A3()
    {
        // The whole ellipse as a periodic B-spline (GeomConvert::CurveToBSplineCurve of the ellipse)
        var anAx2 = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        BSplineCurve3D aBSpline = Lift(new EllipseToBSplineCurve(new Ellipse2D(Constants.OX2d(), 1.0, 0.5)), anAx2)
            .WithPeriodic();
        double aGap = 0.0;
        Curve3D? aRes = CurveToAnaCurve.ComputeCurve(aBSpline, 1.0e-3, aBSpline.FirstParameter,
                                                                    aBSpline.LastParameter, out _, out _, ref aGap,
                                                                    Target, Ellipse);
        EllipseCurve3D anEllipse = Assert.IsType<EllipseCurve3D>(aRes);
        Assert.Equal(1.0, anEllipse.MajorRadius, 1e-3);
        Assert.Equal(0.5, anEllipse.MinorRadius, 1e-3);
    }
}
