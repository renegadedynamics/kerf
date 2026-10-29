// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomConvert_ConvType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// GeomConvert_CurveToAnaCurve (OCCT has no GTests for it; the curve cases of
/// ShapeAnalysis_CanonicalRecognition_Test.cxx, which reach it through ShapeAnalysis, are adapted here).
/// </summary>
public class GeomConvert_CurveToAnaCurveTests
{
    private static readonly gp_Ax2 s_Ax2 = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(0.3, 0.4, 1.0), new gp_Dir(1.0, 0.0, -0.3));

    // An ellipse whose major axis is at 15 degrees from the X axis of the frame ComputeEllipse builds: OCCT
    // recognises it (see ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError)
    private static gp_Ax2 EllipseAx2(double theAngle)
        => new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(Math.Cos(theAngle), Math.Sin(theAngle), 0.0));

    /// <summary>A rational B-spline of a conic, built as GeomConvert's BSplineCurveBuilder does.</summary>
    private static Geom_BSplineCurve Lift(Convert_ConicToBSplineCurve theConvert, in gp_Ax2 thePosition)
    {
        NCollection_Array1<gp_Pnt2d> aPoles2d = theConvert.Poles();
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aPoles2d.Length());
        gp_Trsf aT = gp_Trsf.Transformation(new gp_Ax3(thePosition), new gp_Ax3(gp.XOY()));
        for (int i = aPoles2d.Lower(); i <= aPoles2d.Upper(); i++)
        {
            aPoles[i] = new gp_Pnt(aPoles2d[i].X(), aPoles2d[i].Y(), 0.0).Transformed(aT);
        }

        return new Geom_BSplineCurve(aPoles, theConvert.Weights(), theConvert.Knots(), theConvert.Multiplicities(),
                                     theConvert.Degree(), theConvert.IsPeriodic());
    }

    private static Geom_BSplineCurve CircleArc(double theR, double theU1, double theU2)
        => Lift(new Convert_CircleToBSplineCurve(new gp_Circ2d(gp.OX2d(), theR), theU1, theU2), s_Ax2);

    private static Geom_BSplineCurve EllipseArc(double theR1, double theR2, double theU1, double theU2, double theAngle = Math.PI / 12)
        => Lift(new Convert_EllipseToBSplineCurve(new gp_Elips2d(gp.OX2d(), theR1, theR2), theU1, theU2), EllipseAx2(theAngle));

    private static NCollection_Array1<gp_Pnt> Points(params gp_Pnt[] thePoints)
    {
        var anArray = new NCollection_Array1<gp_Pnt>(1, thePoints.Length);
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
        Assert.True(GeomConvert_CurveToAnaCurve.IsLinear(
            Points(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 1), new gp_Pnt(2, 2, 2.001)), 1e-2, ref aDev));
        Assert.True(aDev > 0.0 && aDev < 1e-3);

        aDev = -1.0;
        Assert.False(GeomConvert_CurveToAnaCurve.IsLinear(
            Points(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0.1, 0), new gp_Pnt(2, 0, 0)), 1e-2, ref aDev));
        Assert.False(GeomConvert_CurveToAnaCurve.IsLinear(Points(new gp_Pnt(0, 0, 0)), 1e-2, ref aDev));
        Assert.False(GeomConvert_CurveToAnaCurve.IsLinear(Points(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 1e-9)), 1e-2, ref aDev));
        Assert.Equal(-1.0, aDev);
    }

    [Fact]
    public void GetLine_And_GetCircle()
    {
        gp_Lin aLin = GeomConvert_CurveToAnaCurve.GetLine(new gp_Pnt(1, 2, 3), new gp_Pnt(1, 2, 7), out double cf, out double cl);
        Assert.Equal(0.0, cf, 12);
        Assert.Equal(4.0, cl, 12);
        Assert.True(aLin.Direction().IsEqual(new gp_Dir(0, 0, 1), 1e-12));

        var aCirc = new gp_Circ();
        var aP0 = new gp_Pnt(3, 1, 0);
        Assert.True(GeomConvert_CurveToAnaCurve.GetCircle(ref aCirc, aP0, new gp_Pnt(1, 3, 0), new gp_Pnt(-1, 1, 0)));
        Assert.Equal(2.0, aCirc.Radius(), 12);
        Assert.True(aCirc.Location().Distance(new gp_Pnt(1, 1, 0)) < 1e-12);
        // The X axis points to P0
        Assert.True(aCirc.XAxis().Direction().IsEqual(new gp_Dir(1, 0, 0), 1e-12));

        var aSame = new gp_Circ();
        Assert.False(GeomConvert_CurveToAnaCurve.GetCircle(ref aSame, new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(2, 0, 0)));
        Assert.False(GeomConvert_CurveToAnaCurve.GetCircle(ref aSame, new gp_Pnt(1e60, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(2, 1, 0)));
    }

    [Fact]
    public void ComputeLine()
    {
        // A degree 2 B-spline with collinear poles
        var aPoles = Points(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 2, 3), new gp_Pnt(3, 6, 9));
        var aBez = new Geom_BezierCurve(aPoles);
        double cf = 0, cl = 0, aDev = -1;
        Geom_Line? aLine = GeomConvert_CurveToAnaCurve.ComputeLine(aBez, 1e-7, 0.0, 1.0, ref cf, ref cl, ref aDev);
        Assert.NotNull(aLine);
        Assert.Equal(0.0, cf, 12);
        Assert.Equal(Math.Sqrt(126.0), cl, 12);
        Assert.Equal(0.0, aDev, 12);

        // A circle arc is not linear; neither is a closed curve
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeLine(CircleArc(2.0, 0.0, 1.0), 1e-3, 0.0, 1.0, ref cf, ref cl, ref aDev));

        // A line is returned as it is
        var aGeomLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        Assert.Same(aGeomLine, GeomConvert_CurveToAnaCurve.ComputeLine(aGeomLine, 1e-7, -1.0, 5.0, ref cf, ref cl, ref aDev));
        Assert.Equal(-1.0, cf);
        Assert.Equal(5.0, cl);
    }

    [Fact]
    public void ComputeCircle_FromBSplineArc()
    {
        Geom_BSplineCurve anArc = CircleArc(2.5, 0.4, 2.9);
        double cf = 0, cl = 0, aDev = -1;
        Geom_Curve? aRes = GeomConvert_CurveToAnaCurve.ComputeCircle(anArc, 1e-7, anArc.FirstParameter(), anArc.LastParameter(),
                                                                     ref cf, ref cl, ref aDev);
        Geom_Circle aCircle = Assert.IsType<Geom_Circle>(aRes);
        Assert.Equal(2.5, aCircle.Radius(), 10);
        Assert.True(aCircle.Location().Distance(s_Ax2.Location()) < 1e-10);
        Assert.True(aDev < 1e-10);
        Assert.Equal(0.0, cf, 12); // the X axis is put at the start point
        Assert.Equal(2.5, cl, 9);
        Assert.True(aCircle.Value(cl).Distance(anArc.Value(anArc.LastParameter())) < 1e-9);

        // Too strict a tolerance for a perturbed arc
        Geom_BSplineCurve aPerturbed = anArc.WithPole(3, anArc.Pole(3).Translated(new gp_Vec(0.0, 0.0, 0.01)));
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeCircle(aPerturbed, 1e-5, aPerturbed.FirstParameter(),
                                                              aPerturbed.LastParameter(), ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeEllipse_FromBSplineArc()
    {
        Geom_BSplineCurve anArc = EllipseArc(3.0, 1.2, 0.3, 2.5);
        double cf = 0, cl = 0, aDev = -1;
        Geom_Curve? aRes = GeomConvert_CurveToAnaCurve.ComputeEllipse(anArc, 1e-6, anArc.FirstParameter(), anArc.LastParameter(),
                                                                      ref cf, ref cl, ref aDev);
        Geom_Ellipse anEllipse = Assert.IsType<Geom_Ellipse>(aRes);
        Assert.Equal(3.0, anEllipse.MajorRadius(), 7);
        Assert.Equal(1.2, anEllipse.MinorRadius(), 7);
        Assert.True(anEllipse.Location().Distance(new gp_Pnt(1.0, -2.0, 0.5)) < 1e-7);
        Assert.True(aDev < 1e-6);
        Assert.True(anEllipse.Value(cf).Distance(anArc.Value(anArc.FirstParameter())) < 1e-6);
        Assert.True(anEllipse.Value(cl).Distance(anArc.Value(anArc.LastParameter())) < 1e-6);

        // A circle is not taken for an ellipse
        Geom_BSplineCurve aCircle = CircleArc(2.0, 0.0, 2.0);
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeEllipse(aCircle, 1e-6, 0.0, 2.0, ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeEllipse_MajorAxisAt60Degrees_ReproducesOcctError()
    {
        // ConicDefinition loses the sign of the rotation and takes the axis within 45 degrees of the frame's X axis
        // (occt-issues.md): the same arc rotated by 60 degrees is not recognised
        Geom_BSplineCurve anArc = EllipseArc(3.0, 1.2, 0.3, 2.5, Math.PI / 3);
        double cf = 0, cl = 0, aDev = -1;
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeEllipse(anArc, 1e-6, anArc.FirstParameter(), anArc.LastParameter(),
                                                               ref cf, ref cl, ref aDev));
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeEllipse(anArc, 1e-1, anArc.FirstParameter(), anArc.LastParameter(),
                                                               ref cf, ref cl, ref aDev));
    }

    [Fact]
    public void ComputeCurve_ConversionTypes()
    {
        Geom_BSplineCurve anEllipse = EllipseArc(3.0, 1.2, 0.3, 2.5);
        double aU1 = anEllipse.FirstParameter(), aU2 = anEllipse.LastParameter();
        double aGap = 0.0;

        Geom_Curve? aMin = GeomConvert_CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out _, out _, ref aGap);
        Assert.IsType<Geom_Ellipse>(aMin);
        Assert.True(aGap >= 0.0 && aGap < 1e-6);

        Geom_Curve? aSimplest = GeomConvert_CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out _, out _, ref aGap,
                                                                         GeomConvert_Simplest);
        Assert.IsType<Geom_Ellipse>(aSimplest);

        // Target circle: not a circle within the tolerance
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeCurve(anEllipse, 1e-6, aU1, aU2, out double cf, out double cl, ref aGap,
                                                             GeomConvert_Target, GeomAbs_Circle));
        Assert.Equal(RealLast(), aGap);

        // MinGap failing: gap -1 and the input parameters
        Geom_BSplineCurve aWavy = anEllipse.WithPole(3, anEllipse.Pole(3).Translated(new gp_Vec(0.1, 0.0, 0.3)));
        Assert.Null(GeomConvert_CurveToAnaCurve.ComputeCurve(aWavy, 1e-6, aU1, aU2, out cf, out cl, ref aGap));
        Assert.Equal(-1.0, aGap);
        Assert.Equal(aU1, cf);
        Assert.Equal(aU2, cl);

        // A circle arc: MinGap prefers the circle (exact) to the ellipse (not found)
        Geom_BSplineCurve aCircle = CircleArc(1.5, 1.0, 4.0);
        Assert.IsType<Geom_Circle>(GeomConvert_CurveToAnaCurve.ComputeCurve(aCircle, 1e-6, 1.0, 4.0, out _, out _, ref aGap));
    }

    [Fact]
    public void ConvertToAnalytical_UsesTheBasisOfATrimmedCurve()
    {
        var aCircle = new Geom_Circle(s_Ax2, 2.0);
        var aTrimmed = new Geom_TrimmedCurve(new Geom_TrimmedCurve(aCircle, 0.5, 2.0), 0.7, 1.5);
        var aConv = new GeomConvert_CurveToAnaCurve(aTrimmed);
        Assert.Equal(Precision.Infinite(), aConv.Gap());
        Geom_Curve? aRes = null;
        Assert.True(aConv.ConvertToAnalytical(1e-7, ref aRes, 0.7, 1.5, out double aNewF, out double aNewL));
        Geom_Circle aResCircle = Assert.IsType<Geom_Circle>(aRes);
        Assert.Same(aTrimmed.BasisCurve(), aResCircle);
        Assert.Equal(0.7, aNewF);
        Assert.Equal(1.5, aNewL);
        Assert.Equal(0.0, aConv.Gap());

        Assert.False(new GeomConvert_CurveToAnaCurve().ConvertToAnalytical(1e-7, ref aRes, 0.0, 1.0, out _, out _));

        aConv.SetConvType(GeomConvert_Target);
        aConv.SetTarget(GeomAbs_Ellipse);
        Assert.Equal(GeomConvert_Target, aConv.GetConvType());
        Assert.Equal(GeomAbs_Ellipse, aConv.GetTarget());
        Geom_Curve? aNone = null;
        Assert.False(aConv.ConvertToAnalytical(1e-7, ref aNone, 0.7, 1.5, out _, out _));
        Assert.Null(aNone);
    }

    // ShapeAnalysis_CanonicalRecognition_Test.cxx, cr/base A1 and A2: a nearly linear Bezier curve
    [Fact]
    public void BezierToLineRecognition_A1()
    {
        var aBezier = new Geom_BezierCurve(Points(new gp_Pnt(0, 0, 0), new gp_Pnt(0.5, 0.0005, 0), new gp_Pnt(1, 0, 0)));
        double aGap = 0.0;
        Geom_Curve? aRes = GeomConvert_CurveToAnaCurve.ComputeCurve(aBezier, 1.0e-3, 0.0, 1.0, out _, out _, ref aGap,
                                                                    GeomConvert_Target, GeomAbs_Line);
        Geom_Line aLine = Assert.IsType<Geom_Line>(aRes);
        Assert.Equal(1.0, Math.Abs(aLine.Lin().Direction().X()), 1e-3);
    }

    [Fact]
    public void BezierToCircleRecognition_A2()
    {
        var aBezier = new Geom_BezierCurve(Points(new gp_Pnt(0, 0, 0), new gp_Pnt(0.5, 0.0005, 0), new gp_Pnt(1, 0, 0)));
        double aGap = 0.0;
        Geom_Curve? aRes = GeomConvert_CurveToAnaCurve.ComputeCurve(aBezier, 1.0e-3, 0.0, 1.0, out _, out _, ref aGap,
                                                                    GeomConvert_Target, GeomAbs_Circle);
        Geom_Circle aCircle = Assert.IsType<Geom_Circle>(aRes);
        Assert.True(aCircle.Radius() > 0.0);
    }

    [Fact]
    public void EllipseToEllipseRecognition_A3()
    {
        // The whole ellipse as a periodic B-spline (GeomConvert::CurveToBSplineCurve of the ellipse)
        var anAx2 = new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X));
        Geom_BSplineCurve aBSpline = Lift(new Convert_EllipseToBSplineCurve(new gp_Elips2d(gp.OX2d(), 1.0, 0.5)), anAx2)
            .WithPeriodic();
        double aGap = 0.0;
        Geom_Curve? aRes = GeomConvert_CurveToAnaCurve.ComputeCurve(aBSpline, 1.0e-3, aBSpline.FirstParameter(),
                                                                    aBSpline.LastParameter(), out _, out _, ref aGap,
                                                                    GeomConvert_Target, GeomAbs_Ellipse);
        Geom_Ellipse anEllipse = Assert.IsType<Geom_Ellipse>(aRes);
        Assert.Equal(1.0, anEllipse.MajorRadius(), 1e-3);
        Assert.Equal(0.5, anEllipse.MinorRadius(), 1e-3);
    }
}
