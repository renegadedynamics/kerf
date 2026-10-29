// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Curve2d.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Approximates a 2D curve given by an adaptor on [First, Last] with a B-spline curve (AdvApprox, its U and V as two
/// 1D subspaces with their own tolerances). The approximation is made by the constructor.
/// </summary>
public sealed class Approx_Curve2d
{
    private sealed class Approx_Curve2d_Eval : AdvApprox_EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handle is.
        private Adaptor2d_Curve2d fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public Approx_Curve2d_Eval(Adaptor2d_Curve2d theFunc, double First, double Last)
        {
            fonct = theFunc;
            StartEndSav0 = First;
            StartEndSav1 = Last;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Param, int Order,
                                      Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            double par = Param;

            // Dimension is incorrect
            if (Dimension != 2)
            {
                ErrorCode = 1;
            }
            // Parameter is incorrect
            if (par < StartEnd[0] || par > StartEnd[1])
            {
                ErrorCode = 2;
            }
            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct = fonct.Trim(StartEnd[0], StartEnd[1], Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }
            gp_Pnt2d pnt;
            gp_Vec2d v1, v2;

            switch (Order)
            {
                case 0:
                    pnt = fonct.Value(par);
                    Result[0] = pnt.X();
                    Result[1] = pnt.Y();
                    break;
                case 1:
                    fonct.D1(par, out pnt, out v1);
                    Result[0] = v1.X();
                    Result[1] = v1.Y();
                    break;
                case 2:
                    fonct.D2(par, out pnt, out v1, out v2);
                    Result[0] = v2.X();
                    Result[1] = v2.Y();
                    break;
                default:
                    Result[0] = Result[1] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    private readonly Geom2d_BSplineCurve? myCurve;
    private readonly bool myIsDone;
    private readonly bool myHasResult;
    private readonly double myMaxError2dU;
    private readonly double myMaxError2dV;

    /// <summary>
    /// Approximates C2D on [First, Last] within TolU in U and TolV in V, with the given continuity, degree at most
    /// MaxDegree and at most MaxSegments segments.
    /// </summary>
    public Approx_Curve2d(Adaptor2d_Curve2d C2D, double First, double Last, double TolU, double TolV,
                          GeomAbs_Shape Continuity, int MaxDegree, int MaxSegments)
    {
        // OCCT discards the trimmed adaptor this returns: C2D is evaluated as given.
        _ = C2D.Trim(First, Last, Precision.PConfusion());

        int Num1DSS = 2, Num2DSS = 0, Num3DSS = 0;
        NCollection_HArray1<double>? TwoDTolNul = null, ThreeDTolNul = null;
        NCollection_HArray1<double> OneDTol = new(1, Num1DSS);
        OneDTol[1] = TolU;
        OneDTol[2] = TolV;

        int NbInterv_C2 = C2D.NbIntervals(GeomAbs_C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        C2D.Intervals(CutPnts_C2, GeomAbs_C2);
        int NbInterv_C3 = C2D.NbIntervals(GeomAbs_C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        C2D.Intervals(CutPnts_C3, GeomAbs_C3);

        AdvApprox_PrefAndRec CutTool = new(CutPnts_C2, CutPnts_C3);

        myMaxError2dU = 0;
        myMaxError2dV = 0;

        Approx_Curve2d_Eval ev = new(C2D, First, Last);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTol,
                                                TwoDTolNul,
                                                ThreeDTolNul,
                                                First,
                                                Last,
                                                Continuity,
                                                MaxDegree,
                                                MaxSegments,
                                                ev,
                                                CutTool);

        myIsDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            int aNbPoles = aApprox.NbPoles();
            NCollection_Array1<gp_Pnt2d> Poles2d = new(1, aNbPoles);
            NCollection_Array1<double> Poles1dU = new(1, aNbPoles);
            aApprox.Poles1d(1, Poles1dU);
            NCollection_Array1<double> Poles1dV = new(1, aNbPoles);
            aApprox.Poles1d(2, Poles1dV);
            for (int i = 1; i <= aNbPoles; i++)
            {
                Poles2d.SetValue(i, new gp_Pnt2d(Poles1dU.Value(i), Poles1dV.Value(i)));
            }

            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myCurve = new Geom2d_BSplineCurve(Poles2d, Knots.Array1(), Mults.Array1(), Degree);
            myMaxError2dU = aApprox.MaxError(1, 1);
            myMaxError2dV = aApprox.MaxError(1, 2);
        }
    }

    /// <summary>Whether the approximation is within the required tolerances.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerances.</summary>
    public bool HasResult() => myHasResult;

    /// <summary>
    /// The B-spline approximation (frozen: the same object at each call), or null when there is no result.
    /// </summary>
    public Geom2d_BSplineCurve? Curve()
    {
        myCurve?.Freeze();
        return myCurve;
    }

    // The result without freezing it, for ported code that discards this object.
    internal Geom2d_BSplineCurve? InternalCurve() => myCurve;

    /// <summary>The largest error in U (0 without an approximation).</summary>
    public double MaxError2dU() => myMaxError2dU;

    /// <summary>The largest error in V (0 without an approximation).</summary>
    public double MaxError2dV() => myMaxError2dV;
}
