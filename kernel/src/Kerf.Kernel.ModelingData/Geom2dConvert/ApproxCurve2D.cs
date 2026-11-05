// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxCurve.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a 2D curve to a B-spline curve by approximation (AdvApprox) within a tolerance, a continuity, a maximum
/// number of segments and a maximum degree. The approximation is made by the constructor.
/// </summary>
internal sealed class ApproxCurve2D
{
    private sealed class Geom2dConvert_ApproxCurve_Eval : EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handle is.
        private Curve2DAdaptorBase _fonct;
        private double _startEndSav0;
        private double _startEndSav1;

        public Geom2dConvert_ApproxCurve_Eval(Curve2DAdaptorBase func, double first, double last)
        {
            _fonct = func;
            _startEndSav0 = first;
            _startEndSav1 = last;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double par = param;

            // Dimension is incorrect
            if (dimension != 2)
            {
                errorCode = 1;
            }

            // Parameter is incorrect
            if (par < startEnd[0] || par > startEnd[1])
            {
                errorCode = 2;
            }

            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct = _fonct.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }

            Point2D pnt;
            Vector2D v1, v2;

            switch (order)
            {
                case 0:
                    pnt = _fonct.Value(par);
                    result[0] = pnt.X;
                    result[1] = pnt.Y;
                    break;
                case 1:
                    _fonct.D1(par, out pnt, out v1);
                    result[0] = v1.X;
                    result[1] = v1.Y;
                    break;
                case 2:
                    _fonct.D2(par, out pnt, out v1, out v2);
                    result[0] = v2.X;
                    result[1] = v2.Y;
                    break;
                default:
                    result[0] = result[1] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    private bool _isDone;
    private bool _hasResult;
    private BSplineCurve2D? _bSplCurve;
    private double _maxError;

    /// <summary>
    /// Approximates Curve within Tol2d, with continuity Order, at most MaxSegments segments and degree at most
    /// MaxDegree.
    /// </summary>
    public ApproxCurve2D(Curve2D curve, double tol2d, Continuity order, int maxSegments, int maxDegree)
    {
        CurveAdaptor2D hCurve = new(curve);
        Approximate(hCurve, tol2d, order, maxSegments, maxDegree);
    }

    /// <summary>As the other constructor, for a curve given by an adaptor (which the approximation evaluates).</summary>
    public ApproxCurve2D(Curve2DAdaptorBase curve, double tol2d, Continuity order, int maxSegments, int maxDegree)
    {
        Approximate(curve, tol2d, order, maxSegments, maxDegree);
    }

    private void Approximate(Curve2DAdaptorBase curve, double tol2d, Continuity order, int maxSegments,
                             int maxDegree)
    {
        // Initialisation of input parameters of AdvApprox

        int num1DSS = 0, num2DSS = 1, num3DSS = 0;
        NCollection_HArray1<double>? oneDTolNul = null, threeDTolNul = null;
        NCollection_HArray1<double> twoDTol = new(1, num2DSS);
        twoDTol.Init(tol2d);

        double first = curve.FirstParameter();
        double last = curve.LastParameter();

        int NbInterv_C2 = curve.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        curve.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = curve.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        curve.Intervals(CutPnts_C3, C3);
        PrefAndRec cutTool = new(CutPnts_C2, CutPnts_C3);

        _maxError = 0;

        Geom2dConvert_ApproxCurve_Eval ev = new(curve, first, last);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTolNul,
                                                twoDTol,
                                                threeDTolNul,
                                                first,
                                                last,
                                                order,
                                                maxDegree,
                                                maxSegments,
                                                ev,
                                                cutTool);

        _isDone = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            NCollection_Array1<Point2D> poles = new(1, approx.NbPoles());
            approx.Poles2d(1, poles);
            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();
            _bSplCurve = new BSplineCurve2D(poles, knots, mults, degree);
            _maxError = approx.MaxError(2, 1);
        }
    }

    /// <summary>
    /// The B-spline approximation (frozen: the same object at each call), or null when there is no result
    /// (<see cref="HasResult"/> false).
    /// </summary>
    public BSplineCurve2D? Curve()
    {
        _bSplCurve?.Freeze();
        return _bSplCurve;
    }

    // The result without freezing it, for ported code that discards this object (Geom2dConvert's statics).
    internal BSplineCurve2D? InternalCurve() => _bSplCurve;

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone => _isDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerance.</summary>
    public bool HasResult => _hasResult;

    /// <summary>The largest distance between the curve and its approximation (0 without an approximation).</summary>
    public double MaxError() => _maxError;

    /// <summary>Writes the maximum error.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("******* Dump of ApproxCurve *******" + '\n');
        o.Write("******* Error   " + MaxError().ToString(CultureInfo.InvariantCulture) + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
