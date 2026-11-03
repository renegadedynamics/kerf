// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ApproxCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Converts a 3D curve to a B-spline curve by approximation (AdvApprox) within a tolerance, a continuity, a maximum
/// number of segments and a maximum degree. The approximation is made by the constructor.
/// </summary>
public sealed class GeomConvert_ApproxCurve
{
    private sealed class GeomConvert_ApproxCurve_Eval : AdvApprox_EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handle is.
        private Adaptor3d_Curve fonct;
        private double StartEndSav0;
        private double StartEndSav1;

        public GeomConvert_ApproxCurve_Eval(Adaptor3d_Curve theFunc, double First, double Last)
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
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }

            if (StartEnd[0] != StartEndSav0 || StartEnd[1] != StartEndSav1)
            {
                fonct = fonct.Trim(StartEnd[0], StartEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                StartEndSav0 = StartEnd[0];
                StartEndSav1 = StartEnd[1];
            }

            Point3D pnt;
            Vector3D v1, v2;

            switch (Order)
            {
                case 0:
                    pnt = fonct.Value(par);
                    Result[0] = pnt.X();
                    Result[1] = pnt.Y();
                    Result[2] = pnt.Z();
                    break;
                case 1:
                    fonct.D1(par, out pnt, out v1);
                    Result[0] = v1.X();
                    Result[1] = v1.Y();
                    Result[2] = v1.Z();
                    break;
                case 2:
                    fonct.D2(par, out pnt, out v1, out v2);
                    Result[0] = v2.X();
                    Result[1] = v2.Y();
                    Result[2] = v2.Z();
                    break;
                default:
                    Result[0] = Result[1] = Result[2] = 0.0;
                    ErrorCode = 3;
                    break;
            }
        }
    }

    private bool myIsDone;
    private bool myHasResult;
    private Geom_BSplineCurve? myBSplCurve;
    private double myMaxError;

    /// <summary>
    /// Approximates Curve within Tol3d, with continuity Order, at most MaxSegments segments and degree at most
    /// MaxDegree.
    /// </summary>
    public GeomConvert_ApproxCurve(Geom_Curve Curve, double Tol3d, Continuity Order, int MaxSegments, int MaxDegree)
    {
        GeomAdaptor_Curve HCurve = new(Curve);
        Approximate(HCurve, Tol3d, Order, MaxSegments, MaxDegree);
    }

    /// <summary>As the other constructor, for a curve given by an adaptor (which the approximation evaluates).</summary>
    public GeomConvert_ApproxCurve(Adaptor3d_Curve Curve, double Tol3d, Continuity Order, int MaxSegments, int MaxDegree)
    {
        Approximate(Curve, Tol3d, Order, MaxSegments, MaxDegree);
    }

    private void Approximate(Adaptor3d_Curve theCurve, double theTol3d, Continuity theOrder, int theMaxSegments,
                             int theMaxDegree)
    {
        // Initialisation of input parameters of AdvApprox

        int Num1DSS = 0, Num2DSS = 0, Num3DSS = 1;
        NCollection_HArray1<double>? OneDTolNul = null, TwoDTolNul = null;
        NCollection_HArray1<double> ThreeDTol = new(1, Num3DSS);
        ThreeDTol.Init(theTol3d);

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

        GeomConvert_ApproxCurve_Eval ev = new(theCurve, First, Last);
        AdvApprox_ApproxAFunction aApprox = new(Num1DSS,
                                                Num2DSS,
                                                Num3DSS,
                                                OneDTolNul,
                                                TwoDTolNul,
                                                ThreeDTol,
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
            NCollection_Array1<Point3D> Poles = new(1, aApprox.NbPoles());
            aApprox.Poles(1, Poles);
            NCollection_HArray1<double> Knots = aApprox.Knots()!;
            NCollection_HArray1<int> Mults = aApprox.Multiplicities()!;
            int Degree = aApprox.Degree();
            myBSplCurve = new Geom_BSplineCurve(Poles, Knots, Mults, Degree);
            myMaxError = aApprox.MaxError(3, 1);
        }
    }

    /// <summary>
    /// The B-spline approximation (frozen: the same object at each call), or null when there is no result
    /// (<see cref="HasResult"/> false).
    /// </summary>
    public Geom_BSplineCurve? Curve()
    {
        myBSplCurve?.Freeze();
        return myBSplCurve;
    }

    // The result without freezing it, for ported code that discards this object (GeomConvert's statics).
    internal Geom_BSplineCurve? InternalCurve() => myBSplCurve;

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
        o.Write("*******Error   " + MaxError().ToString(CultureInfo.InvariantCulture) + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
