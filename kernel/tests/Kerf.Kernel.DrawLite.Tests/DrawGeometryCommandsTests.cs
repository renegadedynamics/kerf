// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// GeomliteTest's curve and surface commands (unit 3L2b): the geometry each builds and binds, its <c>dump</c>
/// (GeomTools' PrintCurve and PrintSurface), the values of cvalue, 2dcvalue, svalue, bounds and parameters, and
/// the failures of wrong argument counts (a C++ command returning 1 with an empty result).
/// </summary>
public class DrawGeometryCommandsTests
{
    private static DrawInterpreter New() => new(new StringWriter { NewLine = "\n" });

    private static object Geometry(DrawInterpreter theDI, string theName)
        => Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry;

    private static T Geometry<T>(DrawInterpreter theDI, string theName) => Assert.IsAssignableFrom<T>(Geometry(theDI, theName));

    private static double Number(DrawInterpreter theDI, string theName)
    {
        Assert.True(theDI.TryGetNumber(theName, out double aValue), theName);
        return aValue;
    }

    private static string Error(DrawInterpreter theDI, string theScript) => Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    [Fact]
    public void Analytic_curves_in_3d_and_2d_by_argument_count()
    {
        DrawInterpreter i = New();
        i.Eval("line l 1 2 3 0 0 1; line l2 1 2 1 1");
        Assert.Equal(new Point3D(1, 2, 3), Geometry<LineCurve3D>(i, "l").Lin().Location());
        Assert.Equal("\n\n*********** Dump of l *************\nLine\n  Origin :1, 2, 3 \n  Axis   :0, 0, 1 \n\n", i.Eval("dump l"));
        Assert.Equal(1 / Math.Sqrt(2), Geometry<LineCurve2D>(i, "l2").Direction().X());

        i.Eval("circle c 0 0 0 5; circle c2 1 1 2; circle c3 0 0 0 1 0 0 3; circle c4 0 0 0 0 0 1 0 1 0 4; circle c5 1 1 0 1 7");
        Assert.Equal(5, Geometry<CircleCurve3D>(i, "c").Radius());
        Assert.Equal(2, Geometry<CircleCurve2D>(i, "c2").Radius());
        Assert.Equal(new Direction3D(1, 0, 0), Geometry<CircleCurve3D>(i, "c3").Axis().Direction());
        Assert.Equal(new Direction3D(0, 1, 0), Geometry<CircleCurve3D>(i, "c4").XAxis().Direction());
        Assert.Equal(7, Geometry<CircleCurve2D>(i, "c5").Radius());
        // gp_Ax2(P, N) computes the X direction by cross products: a negative zero, which Draw prints as -0
        Assert.Equal("\n\n*********** Dump of c *************\nCircle\n  Center :0, 0, 0 \n  Axis   :0, 0, 1 \n  XAxis  :1, 0, -0 \n  YAxis  :-0, 1, 0 \n  Radius :5\n\n", i.Eval("dump c"));
        Assert.Equal("\n\n*********** Dump of c2 *************\nCircle\n  Center :1, 1 \n  XAxis  :1, 0 \n  YAxis  :-0, 1 \n  Radius :2\n\n", i.Eval("dump c2"));

        i.Eval("ellipse e 0 0 0 5 2; ellipse e2 0 0 3 1; hyperbola h 0 0 0 3 1; hyperbola h2 0 0 2 1; parabola p 0 0 0 2; parabola p2 0 0 1");
        Assert.Equal(5, Geometry<EllipseCurve3D>(i, "e").MajorRadius());
        Assert.Equal(1, Geometry<EllipseCurve2D>(i, "e2").MinorRadius());
        Assert.Equal(3, Geometry<HyperbolaCurve3D>(i, "h").MajorRadius());
        Assert.IsType<HyperbolaCurve2D>(Geometry(i, "h2"));
        Assert.Equal(2, Geometry<ParabolaCurve3D>(i, "p").Focal());
        Assert.IsType<ParabolaCurve2D>(Geometry(i, "p2"));
        Assert.Equal("DrawTrSurf_Curve", i.GetDrawable("e")!.TypeName);
        Assert.Equal("DrawTrSurf_Curve2d", i.GetDrawable("e2")!.TypeName);

        // a wrong count returns 1: a Tcl error with an empty message
        Assert.Equal("", Error(i, "line x 1 2 3"));
        Assert.Equal("", Error(i, "circle x 1 2 3 4 5 6"));
        Assert.Equal("", Error(i, "ellipse x 1 2"));
        Assert.Equal("", Error(i, "parabola x 1 2 3 4 5 6 7 8 9"));
        Assert.Equal("0", i.Eval("isdraw x"));

        // a null direction is gp_Dir's construction error, reported as Draw reports exceptions
        Assert.StartsWith("An exception was caught Standard_ConstructionError", Error(i, "line x 0 0 0 0 0 0"));
    }

    [Fact]
    public void Bezier_and_bspline_curves_from_poles()
    {
        DrawInterpreter i = New();
        i.Eval("beziercurve bz 3 0 0 0 1 1 0 2 0 0");
        var aBz = Geometry<BezierCurve3D>(i, "bz");
        Assert.Equal(3, aBz.PoleCount);
        Assert.False(aBz.IsRational);
        Assert.Equal("DrawTrSurf_BezierCurve", i.GetDrawable("bz")!.TypeName);
        i.Eval("beziercurve bzw 3 0 0 0 1 1 1 0 2 2 0 0 1");
        Assert.True(Geometry<BezierCurve3D>(i, "bzw").IsRational);
        Assert.Equal("", Error(i, "beziercurve x 3 0 0 0 1"));
        Assert.Equal("", Error(i, "beziercurve x 0"));

        i.Eval("bsplinecurve bs 2 2 0 3 1 3 0 0 0 1 1 1 0 1 2 0 0 1");
        var aBs = Geometry<BSplineCurve3D>(i, "bs");
        Assert.Equal(2, aBs.Degree());
        Assert.Equal(3, aBs.PoleCount);
        Assert.Equal("DrawTrSurf_BSplineCurve", i.GetDrawable("bs")!.TypeName);
        i.Eval("pbsplinecurve pbs 1 3 0 1 1 1 2 1 0 0 0 1 1 0 0 1 0 1 0 1");
        Assert.True(Geometry<BSplineCurve3D>(i, "pbs").IsPeriodic);

        i.Eval("2dbeziercurve b2 3 0 0 1 1 2 0");
        Assert.Equal(3, Geometry<BezierCurve2D>(i, "b2").PoleCount);
        i.Eval("2dbeziercurve b2w 3 0 0 1 1 1 2 2 0 1");
        Assert.True(Geometry<BezierCurve2D>(i, "b2w").IsRational);
        i.Eval("2dbsplinecurve s2 1 2 0 2 1 2 0 0 1 1 1 1");
        Assert.Equal(2, Geometry<BSplineCurve2D>(i, "s2").PoleCount);
        i.Eval("2dpbsplinecurve ps2 1 3 0 1 1 1 2 1 0 0 1 1 0 1 0 1 1");
        Assert.True(Geometry<BSplineCurve2D>(i, "ps2").IsPeriodic);

        // bad knots: the B-spline constructor's error
        Assert.StartsWith("An exception was caught Standard_ConstructionError", Error(i, "bsplinecurve x 2 2 1 3 0 3 0 0 0 1 1 1 0 1 2 0 0 1"));
    }

    [Fact]
    public void Cvalue_2dcvalue_and_coord_bind_numbers_and_points()
    {
        DrawInterpreter i = New();
        i.Eval("circle c 0 0 0 2");
        i.Eval("cvalue c pi/2 x y z dx dy dz d2x d2y d2z");
        var aCircle = Geometry<CircleCurve3D>(i, "c");
        aCircle.D2(Math.PI / 2, out Point3D P, out Vector3D V1, out Vector3D V2);
        Assert.Equal(new[] { P.X(), P.Y(), P.Z(), V1.X(), V1.Y(), V1.Z(), V2.X(), V2.Y(), V2.Z() },
                     new[] { "x", "y", "z", "dx", "dy", "dz", "d2x", "d2y", "d2z" }.Select(n => Number(i, n)));

        // the last word is a point when the count says so
        i.Eval("cvalue c 0 p");
        Assert.Equal(new Point3D(2, 0, 0), Assert.IsType<Point3D>(Geometry(i, "p")));
        i.Eval("coord p a b cc");
        Assert.Equal(2, Number(i, "a"));
        i.Eval("cvalue c 0 x y z dx dy dz q");
        Assert.Equal(2, Number(i, "dy"));
        Assert.IsType<Point3D>(Geometry(i, "q"));
        Assert.Equal("", Error(i, "cvalue c 0 x y z dx dy"));
        Assert.Equal("", Error(i, "cvalue nothing 0 p"));

        i.Eval("circle c2 0 0 3");
        i.Eval("2dcvalue c2 0 x y dx dy d2x d2y");
        Assert.Equal([3.0, 0, 0, 3, -3, 0], new[] { "x", "y", "dx", "dy", "d2x", "d2y" }.Select(n => Number(i, n)));
        i.Eval("2dcvalue c2 0 p2");
        Assert.IsType<Point2D>(Geometry(i, "p2"));
        i.Eval("coord p2 u v");
        Assert.Equal(3, Number(i, "u"));
        Assert.Equal("", Error(i, "coord p2 u v w"));
        Assert.Equal("", Error(i, "coord p2 u"));
    }

    [Fact]
    public void Reverse_segment_and_setorigin_bind_the_changed_copy()
    {
        DrawInterpreter i = New();
        i.Eval("line l 0 0 0 1 0 0; circle c2 0 0 1");
        var aLine = Geometry<LineCurve3D>(i, "l");
        i.Eval("reverse l c2");
        Assert.Equal(new Direction3D(-1, 0, 0), Geometry<LineCurve3D>(i, "l").Lin().Direction());
        Assert.Equal(new Direction3D(1, 0, 0), aLine.Lin().Direction());
        Assert.Equal(new Direction2D(0, -1), Geometry<CircleCurve2D>(i, "c2").Position().YDirection());

        i.Eval("beziercurve bz 3 0 0 0 1 1 0 2 0 0; segment bz 0.25 0.75");
        Assert.Equal(new Point3D(0.5, 0.375, 0), Geometry<BezierCurve3D>(i, "bz").StartPoint());
        i.Eval("bsplinecurve bs 1 3 0 2 1 1 2 2 0 0 0 1 1 1 0 1 2 0 0 1; segment bs 0.5 1.5");
        Assert.Equal(0.5, Geometry<BSplineCurve3D>(i, "bs").FirstParameter());
        Assert.Equal("", Error(i, "segment l 0 1"));

        i.Eval("pbsplinecurve pbs 1 3 0 1 1 1 2 1 0 0 0 1 1 0 0 1 0 1 0 1");
        // OCCT's setorigin changes a 3D curve and then fails: its else goes with the 2D test
        Assert.Equal("", Error(i, "setorigin pbs 2"));
        Assert.Equal(new Point3D(1, 0, 0), Geometry<BSplineCurve3D>(i, "pbs").Pole(1));
        i.Eval("2dpbsplinecurve ps2 1 3 0 1 1 1 2 1 0 0 1 1 0 1 0 1 1; setorigin ps2 2");
        Assert.Equal(new Point2D(1, 0), Geometry<BSplineCurve2D>(i, "ps2").Pole(1));
    }

    [Fact]
    public void Analytic_surfaces_in_each_placement()
    {
        DrawInterpreter i = New();
        i.Eval("plane p; plane p1 1 2 3; plane p2 0 0 0 1 0 0; plane p3 0 0 0 0 0 1 0 1 0");
        Assert.Equal(new Point3D(1, 2, 3), Geometry<PlaneSurface>(i, "p1").Location());
        Assert.Equal(new Direction3D(1, 0, 0), Geometry<PlaneSurface>(i, "p2").Axis().Direction());
        Assert.Equal(new Direction3D(0, 1, 0), Geometry<PlaneSurface>(i, "p3").Position().XDirection());
        Assert.Equal("\n\n*********** Dump of p *************\nPlane\n  Origin :0, 0, 0 \n  Axis   :0, 0, 1 \n  XAxis  :1, 0, -0 \n  YAxis  :-0, 1, 0 \n\n", i.Eval("dump p"));

        i.Eval("cylinder cy 3; sphere sp 1 1 1 2; cone co 0 0 0 0 0 1 30 5; torus to 0 0 0 0 0 1 1 0 0 10 2");
        Assert.Equal(3, Geometry<CylindricalSurface>(i, "cy").Radius());
        Assert.Equal(2, Geometry<SphericalSurface>(i, "sp").Radius());
        Assert.Equal(30 * (Math.PI / 180.0), Geometry<ConicalSurface>(i, "co").SemiAngle());
        Assert.Equal(10, Geometry<ToroidalSurface>(i, "to").MajorRadius());
        Assert.Equal("DrawTrSurf_Surface", i.GetDrawable("to")!.TypeName);

        Assert.Equal("", Error(i, "cylinder x"));
        Assert.Equal("", Error(i, "torus x 5"));
        Assert.Equal("", Error(i, "plane"));
        Assert.Equal("", Error(i, "plane x 0 0 0 0 0 1 1 0 0 1 2 3"));
    }

    [Fact]
    public void Bezier_and_bspline_surfaces_from_poles()
    {
        DrawInterpreter i = New();
        i.Eval("beziersurf bz 2 2 0 0 0 1 0 0 0 1 0 1 1 1");
        var aBz = Geometry<BezierSurface>(i, "bz");
        Assert.Equal(new Point3D(1, 0, 0), aBz.Pole(2, 1));
        Assert.Equal("DrawTrSurf_BezierSurface", i.GetDrawable("bz")!.TypeName);
        i.Eval("beziersurf bzw 2 2 0 0 0 1 1 0 0 2 0 1 0 1 1 1 1 1");
        Assert.True(Geometry<BezierSurface>(i, "bzw").IsURational);
        Assert.Equal("", Error(i, "beziersurf x 2 2 0 0 0"));

        i.Eval("bsplinesurf bs 1 2 0 2 1 2 1 2 0 2 1 2 0 0 0 1 1 0 0 1 0 1 0 1 1 1 1 1");
        var aBs = Geometry<BSplineSurface>(i, "bs");
        Assert.Equal(new Point3D(0, 1, 0), aBs.Pole(1, 2));
        Assert.Equal("DrawTrSurf_BSplineSurface", i.GetDrawable("bs")!.TypeName);
        i.Eval("upbsplinesurf ub 1 4 0 1 1 1 2 1 3 1 1 2 0 2 1 2 "
               + "0 0 0 1 1 0 0 1 0 1 0 1 0 0 1 1 1 0 1 1 0 1 1 1");
        Assert.True(Geometry<BSplineSurface>(i, "ub").IsUPeriodic);
        Assert.False(Geometry<BSplineSurface>(i, "ub").IsVPeriodic);
        i.Eval("vpbsplinesurf vb 1 2 0 2 1 2 1 4 0 1 1 1 2 1 3 1 "
               + "0 0 0 1 1 0 0 1 0 1 0 1 1 1 0 1 0 0 1 1 1 0 1 1");
        Assert.True(Geometry<BSplineSurface>(i, "vb").IsVPeriodic);
        i.Eval("uvpbsplinesurf uvb 1 4 0 1 1 1 2 1 3 1 1 4 0 1 1 1 2 1 3 1 "
               + string.Join(" ", Enumerable.Range(0, 9).Select(k => $"{k % 3} {k / 3} {k % 2} 1")));
        Assert.True(Geometry<BSplineSurface>(i, "uvb").IsUPeriodic);
        Assert.True(Geometry<BSplineSurface>(i, "uvb").IsVPeriodic);
    }

    [Fact]
    public void Swept_offset_and_trimmed_geometry()
    {
        DrawInterpreter i = New();
        i.Eval("circle c 0 0 0 1; line l 2 0 0 0 0 1; circle c2 0 0 1");
        i.Eval("extsurf ex c 0 0 1; revsurf rv l 0 0 0 0 0 1");
        Assert.Equal(new Direction3D(0, 0, 1), Geometry<ExtrusionSurface>(i, "ex").Direction());
        Assert.IsType<RevolutionSurface>(Geometry(i, "rv"));
        Assert.Equal("", Error(i, "revsurf x l 0 0 0 0"));
        Assert.Equal("", Error(i, "extsurf x nothing 0 0 1"));

        i.Eval("offset oc c 0.5 0 0 1; offset o2 c2 0.5; plane p; offset op p 2");
        Assert.Equal(0.5, Geometry<OffsetCurve3D>(i, "oc").Offset());
        Assert.Equal(0.5, Geometry<OffsetCurve2D>(i, "o2").Offset());
        Assert.Equal(2, Geometry<OffsetSurface>(i, "op").Offset());
        Assert.Equal("", Error(i, "offset x c 0.5"));

        i.Eval("trim tc c 0 1; trim tc0 c 0 1 0; trim t2 c2 0 1; trim tp p -1 1 -2 2; trimu tu p 0 1; trimv tv p 0 1");
        var aTc = Geometry<TrimmedCurve3D>(i, "tc");
        Assert.Equal((0.0, 1.0), (aTc.FirstParameter(), aTc.LastParameter()));
        // the reversed sense trims the reversed circle: [2pi - 1, 2pi]
        Assert.Equal(2 * Math.PI - 1, Geometry<TrimmedCurve3D>(i, "tc0").FirstParameter());
        Assert.IsType<TrimmedCurve2D>(Geometry(i, "t2"));
        Geometry<TrimmedSurface>(i, "tp").Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((-1.0, 1.0, -2.0, 2.0), (u1, u2, v1, v2));
        Geometry<TrimmedSurface>(i, "tu").Bounds(out u1, out u2, out v1, out _);
        Assert.Equal((0.0, 1.0, double.NegativeInfinity), (u1, u2, v1 < -1e100 ? double.NegativeInfinity : v1));
        Geometry<TrimmedSurface>(i, "tv").Bounds(out _, out _, out v1, out v2);
        Assert.Equal((0.0, 1.0), (v1, v2));

        // trim with only a name removes the trim
        i.Eval("trim b tc; trim bp tp; trim b2 t2");
        Assert.IsType<CircleCurve3D>(Geometry(i, "b"));
        Assert.IsType<PlaneSurface>(Geometry(i, "bp"));
        Assert.IsType<CircleCurve2D>(Geometry(i, "b2"));
        Assert.Equal("", Error(i, "trim x p 0 1"));
        Assert.Equal("", Error(i, "trimu x c 0 1"));
        Assert.Equal("", Error(i, "trim x nothing 0 1"));
    }

    [Fact]
    public void Isos_svalue_sderivative_bounds_parameters_and_reversal()
    {
        DrawInterpreter i = New();
        i.Eval("sphere s 0 0 0 2");
        i.Eval("uiso u s 0; viso v s 0");
        Assert.IsType<CircleCurve3D>(Geometry(i, "v"));
        Assert.IsAssignableFrom<Curve3D>(Geometry(i, "u"));

        i.Eval("svalue s 0 0 x y z du1 du2 du3 dv1 dv2 dv3 a b c d e f g h k");
        var aSphere = Geometry<SphericalSurface>(i, "s");
        aSphere.D2(0, 0, out Point3D P, out Vector3D DU, out Vector3D DV, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
        Assert.Equal(new[] { P.X(), P.Y(), P.Z(), DU.X(), DU.Y(), DU.Z(), DV.X(), DV.Y(), DV.Z(), D2U.X(), D2U.Y(), D2U.Z(),
                      D2V.X(), D2V.Y(), D2V.Z(), D2UV.X(), D2UV.Y(), D2UV.Z() },
                     new[] { "x", "y", "z", "du1", "du2", "du3", "dv1", "dv2", "dv3", "a", "b", "c", "d", "e", "f", "g", "h", "k" }.Select(n => Number(i, n)));
        i.Eval("svalue s 0 0 pt");
        Assert.Equal(new Point3D(2, 0, 0), Assert.IsType<Point3D>(Geometry(i, "pt")));
        i.Eval("svalue s 0 0 x y");
        Assert.Equal("", Error(i, "svalue s 0 0"));
        i.Eval("sderivative s 0 0 1 0 dx dy dz");
        Assert.Equal(2, Number(i, "dy"));
        Assert.Equal("", Error(i, "sderivative s 0 0 1 0 dx dy"));

        i.Eval("bounds s u1 u2 v1 v2");
        Assert.Equal([0, 2 * Math.PI, -Math.PI / 2, Math.PI / 2], new[] { "u1", "u2", "v1", "v2" }.Select(n => Number(i, n)));
        i.Eval("circle c2 0 0 1; bounds c2 f l");
        Assert.Equal(2 * Math.PI, Number(i, "l"));

        i.Eval("parameters s 0 2 0 1e-7 pu pv");
        Assert.Equal(Math.PI / 2, Number(i, "pu"), 14);
        Assert.Equal("Wrong point\n", Error(i, "parameters s 0 5 0 1e-7 pu pv"));
        i.Eval("parameters c2 0 1 1e-7 cu");
        Assert.Equal(Math.PI / 2, Number(i, "cu"), 14);
        i.Eval("line l 0 0 0 1 0 0; parameters l 3 0 0 1e-7 lu");
        Assert.Equal(3, Number(i, "lu"));
        Assert.Equal("Unknown curve\n", Error(i, "parameters c2 0 1 0 1e-7 cu"));
        Assert.StartsWith("Invalid parameters!\nUsage:\n", Error(i, "parameters s"));

        i.Eval("plane p; ureverse p");
        Assert.Equal(new Direction3D(-1, 0, 0), Geometry<PlaneSurface>(i, "p").Position().XDirection());
        i.Eval("plane q; vreverse q");
        // Geom_Plane::VReverse reverses the Y direction only: the frame becomes indirect
        Assert.Equal(new Direction3D(0, -1, 0), Geometry<PlaneSurface>(i, "q").Position().YDirection());
        Assert.False(Geometry<PlaneSurface>(i, "q").Position().Direct());
    }

    [Fact]
    public void Bound_geometry_is_frozen_and_restores_as_saved()
    {
        DrawInterpreter i = New();
        i.Eval("bsplinecurve bs 2 2 0 3 1 3 0 0 0 1 1 1 0 1 2 0 0 1");
        Assert.True(Geometry<BSplineCurve3D>(i, "bs").IsFrozen);
        i.Eval("copy bs bs2");
        Assert.Equal(i.Eval("dump bs").Replace("bs ", "bs2 ", StringComparison.Ordinal), i.Eval("dump bs2"));
    }
}
