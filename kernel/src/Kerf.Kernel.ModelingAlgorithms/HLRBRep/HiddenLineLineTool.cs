// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.HiddenLines;

/// <summary>The tool over a line (a line of sight of the projector) that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). The line is
/// infinite: its parameters run from RealFirst to RealLast. OCCT's deprecated <c>SamplePars</c> with an output
/// argument is left out.
/// </remarks>
internal readonly struct HiddenLineLineTool : IIntersectionCurveTool<Line3D>
{
    public static double FirstParameter(in Line3D c) => RealFirst();

    public static double LastParameter(in Line3D c) => RealLast();

    public static Continuity Continuity(in Line3D c) => CN;

    public static int NbIntervals(in Line3D c, Continuity s) => 1;

    public static void Intervals(in Line3D c, NCollection_Array1<double> t, Continuity sh)
    {
    }

    public static double IntervalFirst(in Line3D c) => RealFirst();

    public static double IntervalLast(in Line3D c) => RealLast();

    public static Continuity IntervalContinuity(in Line3D c) => CN;

    public static bool IsClosed(in Line3D c) => false;

    public static bool IsPeriodic(in Line3D c) => false;

    public static double Period(in Line3D c) => 0.0;

    public static Point3D Value(in Line3D c, double u) => global::Kerf.Engine.Geometry.ElCLib.Value(u, c);

    public static void D0(in Line3D c, double u, out Point3D p)
    {
        p = global::Kerf.Engine.Geometry.ElCLib.Value(u, c);
    }

    public static void D1(in Line3D c, double u, out Point3D p, out Vector3D v)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(u, c, out p, out v);
    }

    public static void D2(in Line3D c, double u, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(u, c, out p, out v1);
        v2 = new Vector3D(0, 0, 0);
    }

    public static void D3(in Line3D c, double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(u, c, out p, out v1);
        v2 = v3 = new Vector3D(0, 0, 0);
    }

    public static Vector3D DN(in Line3D c, double u, int n) => global::Kerf.Engine.Geometry.ElCLib.DN(u, c, n);

    public static double Resolution(in Line3D c, double r3D) => r3D;

    public static CurveKind GetType(in Line3D c) => global::Kerf.Geometry.CurveKind.Line;

    public static Line3D Line(in Line3D c) => c;

    public static Circle3D Circle(in Line3D c) => new();

    public static Ellipse3D Ellipse(in Line3D c) => new();

    public static Hyperbola3D Hyperbola(in Line3D c) => new();

    public static Parabola3D Parabola(in Line3D c) => new();

    public static BezierCurve3D? Bezier(in Line3D c) => null;

    public static BSplineCurve3D? BSpline(in Line3D c) => null;

    public static int Degree(in Line3D c) => 1;

    public static int NbPoles(in Line3D c) => 0;

    public static void Poles(in Line3D c, NCollection_Array1<Point3D> t)
    {
    }

    public static bool IsRational(in Line3D c) => false;

    public static void PolesAndWeights(in Line3D c, NCollection_Array1<Point3D> t, NCollection_Array1<double> w)
    {
    }

    public static int NbKnots(in Line3D c) => 0;

    public static void KnotsAndMultiplicities(in Line3D c, NCollection_Array1<double> tk, NCollection_Array1<int> tm)
    {
    }

    public static int NbSamples(in Line3D c, double u0, double u1) => 3;

    public static NCollection_HArray1<double> SamplePars(in Line3D c, double u0, double u1, double defl, int nbMin)
    {
        NCollection_HArray1<double> pars = new(1, 3);
        pars.SetValue(1, u0);
        pars.SetValue(2, (u0 + u1) * 0.5);
        pars.SetValue(3, u1);
        return pars;
    }
}
