// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Curve2d.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a 2D curve given by an adaptor on [First, Last] with a B-spline curve (AdvApprox, its U and V as two
/// 1D subspaces with their own tolerances). The approximation is made by the constructor.
/// </summary>
public sealed class Curve2d
{
    private sealed class Approx_Curve2d_Eval : EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handle is.
        private Curve2DAdaptorBase _fonct;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_Curve2d_Eval(Curve2DAdaptorBase func, double first, double last)
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
                    result[0] = pnt.X();
                    result[1] = pnt.Y();
                    break;
                case 1:
                    _fonct.D1(par, out pnt, out v1);
                    result[0] = v1.X();
                    result[1] = v1.Y();
                    break;
                case 2:
                    _fonct.D2(par, out pnt, out v1, out v2);
                    result[0] = v2.X();
                    result[1] = v2.Y();
                    break;
                default:
                    result[0] = result[1] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    private readonly BSplineCurve2D? _curve;
    private readonly bool _isDone;
    private readonly bool _hasResult;
    private readonly double _maxError2dU;
    private readonly double _maxError2dV;

    /// <summary>
    /// Approximates C2D on [First, Last] within TolU in U and TolV in V, with the given continuity, degree at most
    /// MaxDegree and at most MaxSegments segments.
    /// </summary>
    public Curve2d(Curve2DAdaptorBase c2D, double first, double last, double tolU, double tolV,
                          Continuity continuity, int maxDegree, int maxSegments)
    {
        // OCCT discards the trimmed adaptor this returns: C2D is evaluated as given.
        _ = c2D.Trim(first, last, global::Kerf.Engine.Foundation.Precision.PConfusion());

        int num1DSS = 2, num2DSS = 0, num3DSS = 0;
        NCollection_HArray1<double>? twoDTolNul = null, threeDTolNul = null;
        NCollection_HArray1<double> oneDTol = new(1, num1DSS);
        oneDTol[1] = tolU;
        oneDTol[2] = tolV;

        int NbInterv_C2 = c2D.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        c2D.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = c2D.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        c2D.Intervals(CutPnts_C3, C3);

        PrefAndRec cutTool = new(CutPnts_C2, CutPnts_C3);

        _maxError2dU = 0;
        _maxError2dV = 0;

        Approx_Curve2d_Eval ev = new(c2D, first, last);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTol,
                                                twoDTolNul,
                                                threeDTolNul,
                                                first,
                                                last,
                                                continuity,
                                                maxDegree,
                                                maxSegments,
                                                ev,
                                                cutTool);

        _isDone = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            int nbPoles = approx.NbPoles();
            NCollection_Array1<Point2D> poles2d = new(1, nbPoles);
            NCollection_Array1<double> poles1dU = new(1, nbPoles);
            approx.Poles1d(1, poles1dU);
            NCollection_Array1<double> poles1dV = new(1, nbPoles);
            approx.Poles1d(2, poles1dV);
            for (int i = 1; i <= nbPoles; i++)
            {
                poles2d.SetValue(i, new Point2D(poles1dU.Value(i), poles1dV.Value(i)));
            }

            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();
            _curve = new BSplineCurve2D(poles2d, knots.Array1(), mults.Array1(), degree);
            _maxError2dU = approx.MaxError(1, 1);
            _maxError2dV = approx.MaxError(1, 2);
        }
    }

    /// <summary>Whether the approximation is within the required tolerances.</summary>
    public bool IsDone => _isDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerances.</summary>
    public bool HasResult => _hasResult;

    /// <summary>
    /// The B-spline approximation (frozen: the same object at each call), or null when there is no result.
    /// </summary>
    public BSplineCurve2D? Curve()
    {
        _curve?.Freeze();
        return _curve;
    }

    // The result without freezing it, for ported code that discards this object.
    internal BSplineCurve2D? InternalCurve() => _curve;

    /// <summary>The largest error in U (0 without an approximation).</summary>
    public double MaxError2dU() => _maxError2dU;

    /// <summary>The largest error in V (0 without an approximation).</summary>
    public double MaxError2dV() => _maxError2dV;
}
