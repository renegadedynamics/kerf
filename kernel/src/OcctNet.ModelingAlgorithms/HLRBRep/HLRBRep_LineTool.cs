// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_LineTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>The tool over a line (a line of sight of the projector) that the curve/surface intersection of the hiding uses.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of the generic intersections (decision 338). The line is
/// infinite: its parameters run from RealFirst to RealLast. OCCT's deprecated <c>SamplePars</c> with an output
/// argument is left out.
/// </remarks>
public readonly struct HLRBRep_LineTool : IIntCurveSurface_CurveTool<gp_Lin>
{
    public static double FirstParameter(in gp_Lin C) => RealFirst();

    public static double LastParameter(in gp_Lin C) => RealLast();

    public static GeomAbs_Shape Continuity(in gp_Lin C) => GeomAbs_CN;

    public static int NbIntervals(in gp_Lin C, GeomAbs_Shape S) => 1;

    public static void Intervals(in gp_Lin C, NCollection_Array1<double> T, GeomAbs_Shape Sh)
    {
    }

    public static double IntervalFirst(in gp_Lin C) => RealFirst();

    public static double IntervalLast(in gp_Lin C) => RealLast();

    public static GeomAbs_Shape IntervalContinuity(in gp_Lin C) => GeomAbs_CN;

    public static bool IsClosed(in gp_Lin C) => false;

    public static bool IsPeriodic(in gp_Lin C) => false;

    public static double Period(in gp_Lin C) => 0.0;

    public static gp_Pnt Value(in gp_Lin C, double U) => ElCLib.Value(U, C);

    public static void D0(in gp_Lin C, double U, out gp_Pnt P)
    {
        P = ElCLib.Value(U, C);
    }

    public static void D1(in gp_Lin C, double U, out gp_Pnt P, out gp_Vec V)
    {
        ElCLib.D1(U, C, out P, out V);
    }

    public static void D2(in gp_Lin C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        ElCLib.D1(U, C, out P, out V1);
        V2 = new gp_Vec(0, 0, 0);
    }

    public static void D3(in gp_Lin C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3)
    {
        ElCLib.D1(U, C, out P, out V1);
        V2 = V3 = new gp_Vec(0, 0, 0);
    }

    public static gp_Vec DN(in gp_Lin C, double U, int N) => ElCLib.DN(U, C, N);

    public static double Resolution(in gp_Lin C, double R3D) => R3D;

    public static GeomAbs_CurveType GetType(in gp_Lin C) => GeomAbs_Line;

    public static gp_Lin Line(in gp_Lin C) => C;

    public static gp_Circ Circle(in gp_Lin C) => new();

    public static gp_Elips Ellipse(in gp_Lin C) => new();

    public static gp_Hypr Hyperbola(in gp_Lin C) => new();

    public static gp_Parab Parabola(in gp_Lin C) => new();

    public static Geom_BezierCurve? Bezier(in gp_Lin C) => null;

    public static Geom_BSplineCurve? BSpline(in gp_Lin C) => null;

    public static int Degree(in gp_Lin C) => 1;

    public static int NbPoles(in gp_Lin C) => 0;

    public static void Poles(in gp_Lin C, NCollection_Array1<gp_Pnt> T)
    {
    }

    public static bool IsRational(in gp_Lin C) => false;

    public static void PolesAndWeights(in gp_Lin C, NCollection_Array1<gp_Pnt> T, NCollection_Array1<double> W)
    {
    }

    public static int NbKnots(in gp_Lin C) => 0;

    public static void KnotsAndMultiplicities(in gp_Lin C, NCollection_Array1<double> TK, NCollection_Array1<int> TM)
    {
    }

    public static int NbSamples(in gp_Lin C, double U0, double U1) => 3;

    public static NCollection_HArray1<double> SamplePars(in gp_Lin C, double U0, double U1, double Defl, int NbMin)
    {
        NCollection_HArray1<double> aPars = new(1, 3);
        aPars.SetValue(1, U0);
        aPars.SetValue(2, (U0 + U1) * 0.5);
        aPars.SetValue(3, U1);
        return aPars;
    }
}
