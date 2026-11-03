// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Converts a 2D curve to a B-spline curve by approximation (AdvApprox) within a tolerance, a continuity, a maximum
/// number of segments and a maximum degree. The approximation is made by the constructor.
/// </summary>
public sealed class Geom2dConvert_ApproxCurve
{
    private sealed class Geom2dConvert_ApproxCurve_Eval : AdvApprox_EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handle is.
        private Adaptor2d_Curve2d fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public Geom2dConvert_ApproxCurve_Eval(Adaptor2d_Curve2d theFunc, double First, double Last)
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
                fonct = fonct.Trim(StartEnd[0], StartEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            Point2D pnt;
            Vector2D v1, v2;

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

    private bool myIsDone;
    private bool myHasResult;
    private Geom2d_BSplineCurve? myBSplCurve;
    private double myMaxError;

    /// <summary>
    /// Approximates Curve within Tol2d, with continuity Order, at most MaxSegments segments and degree at most
    /// MaxDegree.
    /// </summary>
    public Geom2dConvert_ApproxCurve(Geom2d_Curve Curve, double Tol2d, Continuity Order, int MaxSegments, int MaxDegree)
    {
        Geom2dAdaptor_Curve HCurve = new(Curve);
        Approximate(HCurve, Tol2d, Order, MaxSegments, MaxDegree);
    }

    /// <summary>As the other constructor, for a curve given by an adaptor (which the approximation evaluates).</summary>
    public Geom2dConvert_ApproxCurve(Adaptor2d_Curve2d Curve, double Tol2d, Continuity Order, int MaxSegments, int MaxDegree)
    {
        Approximate(Curve, Tol2d, Order, MaxSegments, MaxDegree);
    }

    private void Approximate(Adaptor2d_Curve2d theCurve, double theTol2d, Continuity theOrder, int theMaxSegments,
                             int theMaxDegree)
    {
        // Initialisation of input parameters of AdvApprox

        int Num1DSS = 0, Num2DSS = 1, Num3DSS = 0;
        NCollection_HArray1<double>? OneDTolNul = null, ThreeDTolNul = null;
        NCollection_HArray1<double> TwoDTol = new(1, Num2DSS);
        TwoDTol.Init(theTol2d);

        double First = theCurve.FirstParameter();
        double Last = theCurve.LastParameter();

        int NbInterv_C2 = theCurve.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        theCurve.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = theCurve.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        theCurve.Intervals(CutPnts_C3, C3);
        AdvApprox_PrefAndRec CutTool = new(CutPnts_C2, CutPnts_C3);

        myMaxError = 0;

        Geom2dConvert_ApproxCurve_Eval ev = new(theCurve, First, Last);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTolNul,
                                                TwoDTol,
                                                ThreeDTolNul,
                                                First,
                                                Last,
                                                theOrder,
                                                theMaxDegree,
                                                theMaxSegments,
                                                ev,
                                                CutTool);

        myIsDone = aApprox.IsDone();
        myHasResult = aApprox.HasResult();

        if (myHasResult)
        {
            NCollection_Array1<Point2D> Poles = new(1, aApprox.NbPoles());
            aApprox.Poles2d(1, Poles);
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myBSplCurve = new Geom2d_BSplineCurve(Poles, Knots, Mults, Degree);
            myMaxError = aApprox.MaxError(2, 1);
        }
    }

    /// <summary>
    /// The B-spline approximation (frozen: the same object at each call), or null when there is no result
    /// (<see cref="HasResult"/> false).
    /// </summary>
    public Geom2d_BSplineCurve? Curve()
    {
        myBSplCurve?.Freeze();
        return myBSplCurve;
    }

    // The result without freezing it, for ported code that discards this object (Geom2dConvert's statics).
    internal Geom2d_BSplineCurve? InternalCurve() => myBSplCurve;

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerance.</summary>
    public bool HasResult() => myHasResult;

    /// <summary>The largest distance between the curve and its approximation (0 without an approximation).</summary>
    public double MaxError() => myMaxError;

    /// <summary>Writes the maximum error.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("******* Dump of ApproxCurve *******" + '\n');
        o.Write("******* Error   " + MaxError().ToString(CultureInfo.InvariantCulture) + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
