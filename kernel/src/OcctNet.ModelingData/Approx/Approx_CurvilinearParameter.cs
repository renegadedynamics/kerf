// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvilinearParameter.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvilinearParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a curve with its curvilinear abscissa as parameter (normalised to [0, 1]): a free 3D curve, a curve
/// on a surface (the 3D curve and its 2D curve), or a curve on two surfaces (1/2 (S1(C2D1(u)) + S2(C2D2(u))), with
/// both 2D curves). The approximation is made by the constructor.
/// </summary>
/// <remarks>
/// The curves it keeps are frozen when handed out (<see cref="Curve3d"/>, <see cref="Curve2d1"/>,
/// <see cref="Curve2d2"/>): the same objects at each call. The caller's adaptors are never changed; the internal
/// <see cref="Approx_CurvlinFunc"/> replaces its own by trimmed ones as the approximation cuts the range.
/// </remarks>
public sealed class Approx_CurvilinearParameter
{
    private sealed class Approx_CurvilinearParameter_EvalCurv : AdvApprox_EvaluatorFunction
    {
        private readonly Approx_CurvlinFunc fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_CurvilinearParameter_EvalCurv(Approx_CurvlinFunc theFunc, double First, double Last)
        {
            fonct = theFunc;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double S = Param;
            NCollection_Array1<double> Res = new(0, 2);
            int i;

            // Dimension is incorrect
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }
            // Parameter is incorrect
            if (S < StartEnd[0] || S > StartEnd[1])
            {
                ErrorCode = 2;
            }

            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct.Trim(StartEnd[0], StartEnd[1], global::Kerf.Engine.Foundation.Precision.Confusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            if (!fonct.EvalCase1(S, Order, Res))
            {
                ErrorCode = 3;
            }

            for (i = 0; i <= 2; i++)
            {
                Result[i] = Res[i];
            }
        }
    }

    private sealed class Approx_CurvilinearParameter_EvalCurvOnSurf : AdvApprox_EvaluatorFunction
    {
        private readonly Approx_CurvlinFunc fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_CurvilinearParameter_EvalCurvOnSurf(Approx_CurvlinFunc theFunc, double First, double Last)
        {
            fonct = theFunc;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double S = Param;
            NCollection_Array1<double> Res = new(0, 4);
            int i;

            // Dimension is incorrect
            if (Dimension != 5)
            {
                ErrorCode = 1;
            }
            // Parameter is incorrect
            if (S < StartEnd[0] || S > StartEnd[1])
            {
                ErrorCode = 2;
            }

            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct.Trim(StartEnd[0], StartEnd[1], global::Kerf.Engine.Foundation.Precision.Confusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            if (!fonct.EvalCase2(S, Order, Res))
            {
                ErrorCode = 3;
            }

            for (i = 0; i <= 4; i++)
            {
                Result[i] = Res[i];
            }
        }
    }

    private sealed class Approx_CurvilinearParameter_EvalCurvOn2Surf : AdvApprox_EvaluatorFunction
    {
        private readonly Approx_CurvlinFunc fonct;

        // OCCT keeps the range in StartEndSav too, but the trimming that reads it is commented out.
        public Approx_CurvilinearParameter_EvalCurvOn2Surf(Approx_CurvlinFunc theFunc)
        {
            fonct = theFunc;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double S = Param;
            NCollection_Array1<double> Res = new(0, 6);
            int i;

            // Dimension is incorrect
            if (Dimension != 7)
            {
                ErrorCode = 1;
            }
            // Parameter is incorrect
            if (S < StartEnd[0] || S > StartEnd[1])
            {
                ErrorCode = 2;
            }

            if (!fonct.EvalCase3(S, Order, Res))
            {
                ErrorCode = 3;
            }

            for (i = 0; i <= 6; i++)
            {
                Result[i] = Res[i];
            }
        }
    }

    private readonly int myCase;
    private readonly bool myDone;
    private readonly bool myHasResult;
    private readonly Geom_BSplineCurve? myCurve3d;
    private readonly double myMaxError3d;
    private readonly Geom2d_BSplineCurve? myCurve2d1;
    private readonly double myMaxError2d1;
    private readonly Geom2d_BSplineCurve? myCurve2d2;
    private readonly double myMaxError2d2;

    /// <summary>Case of a free 3D curve.</summary>
    public Approx_CurvilinearParameter(Adaptor3d_Curve C3D, double Tol, Continuity Order, int MaxDegree,
                                       int MaxSegments)
    {
        myMaxError2d1 = 0.0;
        myMaxError2d2 = 0.0;
        myCase = 1;
        // Initialisation of input parameters of AdvApprox

        int Num1DSS = 0, Num2DSS = 0, Num3DSS = 1;
        NCollection_HArray1<double>? OneDTolNul = null, TwoDTolNul = null;
        NCollection_HArray1<double> ThreeDTol = new(1, Num3DSS);
        ThreeDTol.Init(Tol);

        Approx_CurvlinFunc fonct = new(C3D, Tol / 10);

        double FirstS = fonct.FirstParameter();
        double LastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        AdvApprox_PrefAndRec CutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurv evC = new(fonct, FirstS, LastS);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTolNul,
                                                TwoDTolNul,
                                                ThreeDTol,
                                                FirstS,
                                                LastS,
                                                Order,
                                                MaxDegree,
                                                MaxSegments,
                                                evC,
                                                CutTool);

        myDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            NCollection_Array1<Point3D> Poles = new(1, aApprox.NbPoles());
            aApprox.Poles(1, Poles);
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myCurve3d = new Geom_BSplineCurve(Poles, Knots.Array1(), Mults.Array1(), Degree);
        }
        myMaxError3d = aApprox.MaxError(3, 1);
    }

    /// <summary>Case of a curve on one surface.</summary>
    public Approx_CurvilinearParameter(Adaptor2d_Curve2d C2D, Adaptor3d_Surface Surf, double Tol,
                                       Continuity Order, int MaxDegree, int MaxSegments)
    {
        myCase = 2;

        // Initialisation of input parameters of AdvApprox

        int Num1DSS = 2, Num2DSS = 0, Num3DSS = 1, i;

        NCollection_HArray1<double> OneDTol = new(1, Num1DSS);
        double TolV, TolW;

        ToleranceComputation(C2D, Surf, 10, Tol, out TolV, out TolW);
        OneDTol.SetValue(1, TolV);
        OneDTol.SetValue(2, TolW);

        // OCCT overwrites the computed tolerances at once.
        OneDTol.SetValue(1, Tol);
        OneDTol.SetValue(2, Tol);

        NCollection_HArray1<double>? TwoDTolNul = null;
        NCollection_HArray1<double> ThreeDTol = new(1, Num3DSS);
        ThreeDTol.Init(Tol / 2.0);

        Approx_CurvlinFunc fonct = new(C2D, Surf, Tol / 20);

        double FirstS = fonct.FirstParameter();
        double LastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        AdvApprox_PrefAndRec CutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurvOnSurf evCOnS = new(fonct, FirstS, LastS);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTol,
                                                TwoDTolNul,
                                                ThreeDTol,
                                                FirstS,
                                                LastS,
                                                Order,
                                                MaxDegree,
                                                MaxSegments,
                                                evCOnS,
                                                CutTool);

        myDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            int NbPoles = aApprox.NbPoles();
            NCollection_Array1<Point3D> Poles = new(1, NbPoles);
            NCollection_Array1<Point2D> Poles2d = new(1, NbPoles);
            NCollection_Array1<double> Poles1d = new(1, NbPoles);
            aApprox.Poles(1, Poles);
            aApprox.Poles1d(1, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetX(Poles1d[i]);
            }
            aApprox.Poles1d(2, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetY(Poles1d[i]);
            }
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myCurve3d = new Geom_BSplineCurve(Poles, Knots.Array1(), Mults.Array1(), Degree);
            myCurve2d1 = new Geom2d_BSplineCurve(Poles2d, Knots.Array1(), Mults.Array1(), Degree);
        }
        myMaxError2d1 = Max(aApprox.MaxError(1, 1), aApprox.MaxError(1, 2));
        myMaxError3d = aApprox.MaxError(3, 1);
        // OCCT leaves myMaxError2d2 uninitialised here; it is 0 in the port.
    }

    /// <summary>Case of a curve on two surfaces.</summary>
    public Approx_CurvilinearParameter(Adaptor2d_Curve2d C2D1, Adaptor3d_Surface Surf1, Adaptor2d_Curve2d C2D2,
                                       Adaptor3d_Surface Surf2, double Tol, Continuity Order, int MaxDegree,
                                       int MaxSegments)
    {
        int i;

        myCase = 3;

        // Initialisation of input parameters of AdvApprox

        int Num1DSS = 4, Num2DSS = 0, Num3DSS = 1;
        NCollection_HArray1<double> OneDTol = new(1, Num1DSS);

        double TolV, TolW;
        ToleranceComputation(C2D1, Surf1, 10, Tol, out TolV, out TolW);
        OneDTol.SetValue(1, TolV);
        OneDTol.SetValue(2, TolW);

        ToleranceComputation(C2D2, Surf2, 10, Tol, out TolV, out TolW);
        OneDTol.SetValue(3, TolV);
        OneDTol.SetValue(4, TolW);

        NCollection_HArray1<double>? TwoDTolNul = null;
        NCollection_HArray1<double> ThreeDTol = new(1, Num3DSS);
        ThreeDTol.Init(Tol / 2);

        Approx_CurvlinFunc fonct = new(C2D1, C2D2, Surf1, Surf2, Tol / 20);

        double FirstS = fonct.FirstParameter();
        double LastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        AdvApprox_PrefAndRec CutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurvOn2Surf evCOn2S = new(fonct);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTol,
                                                TwoDTolNul,
                                                ThreeDTol,
                                                FirstS,
                                                LastS,
                                                Order,
                                                MaxDegree,
                                                MaxSegments,
                                                evCOn2S,
                                                CutTool);

        myDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            int NbPoles = aApprox.NbPoles();
            NCollection_Array1<Point3D> Poles = new(1, NbPoles);
            NCollection_Array1<Point2D> Poles2d = new(1, NbPoles);
            NCollection_Array1<double> Poles1d = new(1, NbPoles);
            aApprox.Poles(1, Poles);
            aApprox.Poles1d(1, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetX(Poles1d[i]);
            }
            aApprox.Poles1d(2, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetY(Poles1d[i]);
            }
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myCurve3d = new Geom_BSplineCurve(Poles, Knots.Array1(), Mults.Array1(), Degree);
            myCurve2d1 = new Geom2d_BSplineCurve(Poles2d, Knots.Array1(), Mults.Array1(), Degree);
            aApprox.Poles1d(3, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetX(Poles1d[i]);
            }
            aApprox.Poles1d(4, Poles1d);
            for (i = 1; i <= NbPoles; i++)
            {
                Poles2d[i].SetY(Poles1d[i]);
            }
            myCurve2d2 = new Geom2d_BSplineCurve(Poles2d, Knots.Array1(), Mults.Array1(), Degree);
        }
        myMaxError2d1 = Max(aApprox.MaxError(1, 1), aApprox.MaxError(1, 2));
        myMaxError2d2 = Max(aApprox.MaxError(1, 3), aApprox.MaxError(1, 4));
        myMaxError3d = aApprox.MaxError(3, 1);
    }

    /// <summary>Whether the approximation is within the tolerance.</summary>
    public bool IsDone() => myDone;

    /// <summary>Whether the approximation has a result, not necessarily within the tolerance.</summary>
    public bool HasResult() => myHasResult;

    /// <summary>The reparametrized 3D curve (frozen; null without a result).</summary>
    public Geom_BSplineCurve? Curve3d()
    {
        myCurve3d?.Freeze();
        return myCurve3d;
    }

    /// <summary>The maximum error on the reparametrized 3D curve.</summary>
    public double MaxError3d() => myMaxError3d;

    /// <summary>The reparametrized 2D curve on the first surface (frozen; null without one).</summary>
    public Geom2d_BSplineCurve? Curve2d1()
    {
        myCurve2d1?.Freeze();
        return myCurve2d1;
    }

    /// <summary>The reparametrized 2D curve on the first surface, unfrozen, for ported code that changes it (BRepLib::SameParameter re-knots it).</summary>
    internal Geom2d_BSplineCurve? InternalCurve2d1() => myCurve2d1;

    /// <summary>The maximum error on the first reparametrized 2D curve.</summary>
    public double MaxError2d1() => myMaxError2d1;

    /// <summary>The reparametrized 2D curve on the second surface (frozen; null without one).</summary>
    public Geom2d_BSplineCurve? Curve2d2()
    {
        myCurve2d2?.Freeze();
        return myCurve2d2;
    }

    /// <summary>The maximum error on the second reparametrized 2D curve.</summary>
    public double MaxError2d2() => myMaxError2d2;

    /// <summary>Writes the maximum errors.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("Dump of Approx_CurvilinearParameter" + '\n');
        if (myCase == 2 || myCase == 3)
        {
            o.Write("myMaxError2d1 = " + myMaxError2d1.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        if (myCase == 3)
        {
            o.Write("myMaxError2d2 = " + myMaxError2d2.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        o.Write("myMaxError3d = " + myMaxError3d.ToString(CultureInfo.InvariantCulture) + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    private static void ToleranceComputation(Adaptor2d_Curve2d C2D, Adaptor3d_Surface S, int MaxNumber, double Tol,
                                             out double TolV, out double TolW)
    {
        double FirstU = C2D.FirstParameter(), LastU = C2D.LastParameter();
        double Max_dS_dv = 1.0, Max_dS_dw = 1.0;
        Point2D pntVW;

        for (int i = 1; i <= MaxNumber; i++)
        {
            pntVW = C2D.Value(FirstU + (i - 1) * (LastU - FirstU) / (MaxNumber - 1));
            S.D1(pntVW.X(), pntVW.Y(), out _, out Vector3D dS_dv, out Vector3D dS_dw);
            Max_dS_dv = Max(Max_dS_dv, dS_dv.Magnitude());
            Max_dS_dw = Max(Max_dS_dw, dS_dw.Magnitude());
        }
        TolV = Tol / (4.0 * Max_dS_dv);
        TolW = Tol / (4.0 * Max_dS_dw);
    }
}
