// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>The tool over a line (a line of sight of the projector) that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). The line is
/// infinite: its parameters run from RealFirst to RealLast. OCCT's deprecated <c>SamplePars</c> with an output
/// argument is left out.
/// </remarks>
public readonly struct HLRBRep_LineTool : IIntCurveSurface_CurveTool<Line3D>
{
    public static double FirstParameter(in Line3D C) => RealFirst();

    public static double LastParameter(in Line3D C) => RealLast();

    public static Continuity Continuity(in Line3D C) => CN;

    public static int NbIntervals(in Line3D C, Continuity S) => 1;

    public static void Intervals(in Line3D C, NCollection_Array1<double> T, Continuity Sh)
    {
    }

    public static double IntervalFirst(in Line3D C) => RealFirst();

    public static double IntervalLast(in Line3D C) => RealLast();

    public static Continuity IntervalContinuity(in Line3D C) => CN;

    public static bool IsClosed(in Line3D C) => false;

    public static bool IsPeriodic(in Line3D C) => false;

    public static double Period(in Line3D C) => 0.0;

    public static Point3D Value(in Line3D C, double U) => global::Kerf.Engine.Geometry.ElCLib.Value(U, C);

    public static void D0(in Line3D C, double U, out Point3D P)
    {
        P = global::Kerf.Engine.Geometry.ElCLib.Value(U, C);
    }

    public static void D1(in Line3D C, double U, out Point3D P, out Vector3D V)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(U, C, out P, out V);
    }

    public static void D2(in Line3D C, double U, out Point3D P, out Vector3D V1, out Vector3D V2)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(U, C, out P, out V1);
        V2 = new Vector3D(0, 0, 0);
    }

    public static void D3(in Line3D C, double U, out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3)
    {
        global::Kerf.Engine.Geometry.ElCLib.D1(U, C, out P, out V1);
        V2 = V3 = new Vector3D(0, 0, 0);
    }

    public static Vector3D DN(in Line3D C, double U, int N) => global::Kerf.Engine.Geometry.ElCLib.DN(U, C, N);

    public static double Resolution(in Line3D C, double R3D) => R3D;

    public static CurveKind GetType(in Line3D C) => global::Kerf.Geometry.CurveKind.Line;

    public static Line3D Line(in Line3D C) => C;

    public static Circle3D Circle(in Line3D C) => new();

    public static Ellipse3D Ellipse(in Line3D C) => new();

    public static Hyperbola3D Hyperbola(in Line3D C) => new();

    public static Parabola3D Parabola(in Line3D C) => new();

    public static BezierCurve3D? Bezier(in Line3D C) => null;

    public static BSplineCurve3D? BSpline(in Line3D C) => null;

    public static int Degree(in Line3D C) => 1;

    public static int NbPoles(in Line3D C) => 0;

    public static void Poles(in Line3D C, NCollection_Array1<Point3D> T)
    {
    }

    public static bool IsRational(in Line3D C) => false;

    public static void PolesAndWeights(in Line3D C, NCollection_Array1<Point3D> T, NCollection_Array1<double> W)
    {
    }

    public static int NbKnots(in Line3D C) => 0;

    public static void KnotsAndMultiplicities(in Line3D C, NCollection_Array1<double> TK, NCollection_Array1<int> TM)
    {
    }

    public static int NbSamples(in Line3D C, double U0, double U1) => 3;

    public static NCollection_HArray1<double> SamplePars(in Line3D C, double U0, double U1, double Defl, int NbMin)
    {
        NCollection_HArray1<double> aPars = new(1, 3);
        aPars.SetValue(1, U0);
        aPars.SetValue(2, (U0 + U1) * 0.5);
        aPars.SetValue(3, U1);
        return aPars;
    }
}
