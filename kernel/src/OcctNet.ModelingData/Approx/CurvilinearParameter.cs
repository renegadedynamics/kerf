// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvilinearParameter.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurvilinearParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a curve with its curvilinear abscissa as parameter (normalised to [0, 1]): a free 3D curve, a curve
/// on a surface (the 3D curve and its 2D curve), or a curve on two surfaces (1/2 (S1(C2D1(u)) + S2(C2D2(u))), with
/// both 2D curves). The approximation is made by the constructor.
/// </summary>
/// <remarks>
/// The curves it keeps are frozen when handed out (<see cref="Curve3d"/>, <see cref="Curve2d1"/>,
/// <see cref="Curve2d2"/>): the same objects at each call. The caller's adaptors are never changed; the internal
/// <see cref="CurvlinFunc"/> replaces its own by trimmed ones as the approximation cuts the range.
/// </remarks>
public sealed class CurvilinearParameter
{
    private sealed class Approx_CurvilinearParameter_EvalCurv : EvaluatorFunction
    {
        private readonly CurvlinFunc _fonct;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_CurvilinearParameter_EvalCurv(CurvlinFunc func, double first, double last)
        {
            _fonct = func;
            _startEndSav0 = first;
            _startEndSav1 = last;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double s = param;
            NCollection_Array1<double> res = new(0, 2);
            int i;

            // Dimension is incorrect
            if (dimension != 3)
            {
                errorCode = 1;
            }
            // Parameter is incorrect
            if (s < startEnd[0] || s > startEnd[1])
            {
                errorCode = 2;
            }

            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.Confusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }

            if (!_fonct.EvalCase1(s, order, res))
            {
                errorCode = 3;
            }

            for (i = 0; i <= 2; i++)
            {
                result[i] = res[i];
            }
        }
    }

    private sealed class Approx_CurvilinearParameter_EvalCurvOnSurf : EvaluatorFunction
    {
        private readonly CurvlinFunc _fonct;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_CurvilinearParameter_EvalCurvOnSurf(CurvlinFunc func, double first, double last)
        {
            _fonct = func;
            _startEndSav0 = first;
            _startEndSav1 = last;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double s = param;
            NCollection_Array1<double> res = new(0, 4);
            int i;

            // Dimension is incorrect
            if (dimension != 5)
            {
                errorCode = 1;
            }
            // Parameter is incorrect
            if (s < startEnd[0] || s > startEnd[1])
            {
                errorCode = 2;
            }

            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.Confusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }

            if (!_fonct.EvalCase2(s, order, res))
            {
                errorCode = 3;
            }

            for (i = 0; i <= 4; i++)
            {
                result[i] = res[i];
            }
        }
    }

    private sealed class Approx_CurvilinearParameter_EvalCurvOn2Surf : EvaluatorFunction
    {
        private readonly CurvlinFunc _fonct;

        // OCCT keeps the range in StartEndSav too, but the trimming that reads it is commented out.
        public Approx_CurvilinearParameter_EvalCurvOn2Surf(CurvlinFunc func)
        {
            _fonct = func;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double s = param;
            NCollection_Array1<double> res = new(0, 6);
            int i;

            // Dimension is incorrect
            if (dimension != 7)
            {
                errorCode = 1;
            }
            // Parameter is incorrect
            if (s < startEnd[0] || s > startEnd[1])
            {
                errorCode = 2;
            }

            if (!_fonct.EvalCase3(s, order, res))
            {
                errorCode = 3;
            }

            for (i = 0; i <= 6; i++)
            {
                result[i] = res[i];
            }
        }
    }

    private readonly int _case;
    private readonly bool _done;
    private readonly bool _hasResult;
    private readonly BSplineCurve3D? _curve3d;
    private readonly double _maxError3d;
    private readonly BSplineCurve2D? _curve2d1;
    private readonly double _maxError2d1;
    private readonly BSplineCurve2D? _curve2d2;
    private readonly double _maxError2d2;

    /// <summary>Case of a free 3D curve.</summary>
    public CurvilinearParameter(CurveAdaptorBase c3D, double tol, Continuity order, int maxDegree,
                                       int maxSegments)
    {
        _maxError2d1 = 0.0;
        _maxError2d2 = 0.0;
        _case = 1;
        // Initialisation of input parameters of AdvApprox

        int num1DSS = 0, num2DSS = 0, num3DSS = 1;
        NCollection_HArray1<double>? oneDTolNul = null, twoDTolNul = null;
        NCollection_HArray1<double> threeDTol = new(1, num3DSS);
        threeDTol.Init(tol);

        CurvlinFunc fonct = new(c3D, tol / 10);

        double firstS = fonct.FirstParameter();
        double lastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        PrefAndRec cutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurv evC = new(fonct, firstS, lastS);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTolNul,
                                                twoDTolNul,
                                                threeDTol,
                                                firstS,
                                                lastS,
                                                order,
                                                maxDegree,
                                                maxSegments,
                                                evC,
                                                cutTool);

        _done = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            NCollection_Array1<Point3D> poles = new(1, approx.NbPoles());
            approx.Poles(1, poles);
            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();
            _curve3d = new BSplineCurve3D(poles, knots.Array1(), mults.Array1(), degree);
        }
        _maxError3d = approx.MaxError(3, 1);
    }

    /// <summary>Case of a curve on one surface.</summary>
    public CurvilinearParameter(Curve2DAdaptorBase c2D, SurfaceAdaptorBase surf, double tol,
                                       Continuity order, int maxDegree, int maxSegments)
    {
        _case = 2;

        // Initialisation of input parameters of AdvApprox

        int num1DSS = 2, num2DSS = 0, num3DSS = 1, i;

        NCollection_HArray1<double> oneDTol = new(1, num1DSS);
        double tolV, tolW;

        ToleranceComputation(c2D, surf, 10, tol, out tolV, out tolW);
        oneDTol.SetValue(1, tolV);
        oneDTol.SetValue(2, tolW);

        // OCCT overwrites the computed tolerances at once.
        oneDTol.SetValue(1, tol);
        oneDTol.SetValue(2, tol);

        NCollection_HArray1<double>? twoDTolNul = null;
        NCollection_HArray1<double> threeDTol = new(1, num3DSS);
        threeDTol.Init(tol / 2.0);

        CurvlinFunc fonct = new(c2D, surf, tol / 20);

        double firstS = fonct.FirstParameter();
        double lastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        PrefAndRec cutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurvOnSurf evCOnS = new(fonct, firstS, lastS);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTol,
                                                twoDTolNul,
                                                threeDTol,
                                                firstS,
                                                lastS,
                                                order,
                                                maxDegree,
                                                maxSegments,
                                                evCOnS,
                                                cutTool);

        _done = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            int nbPoles = approx.NbPoles();
            NCollection_Array1<Point3D> poles = new(1, nbPoles);
            NCollection_Array1<Point2D> poles2d = new(1, nbPoles);
            NCollection_Array1<double> poles1d = new(1, nbPoles);
            approx.Poles(1, poles);
            approx.Poles1d(1, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetX(poles1d[i]);
            }
            approx.Poles1d(2, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetY(poles1d[i]);
            }
            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();
            _curve3d = new BSplineCurve3D(poles, knots.Array1(), mults.Array1(), degree);
            _curve2d1 = new BSplineCurve2D(poles2d, knots.Array1(), mults.Array1(), degree);
        }
        _maxError2d1 = Max(approx.MaxError(1, 1), approx.MaxError(1, 2));
        _maxError3d = approx.MaxError(3, 1);
        // OCCT leaves myMaxError2d2 uninitialised here; it is 0 in the port.
    }

    /// <summary>Case of a curve on two surfaces.</summary>
    public CurvilinearParameter(Curve2DAdaptorBase c2D1, SurfaceAdaptorBase surf1, Curve2DAdaptorBase c2D2,
                                       SurfaceAdaptorBase surf2, double tol, Continuity order, int maxDegree,
                                       int maxSegments)
    {
        int i;

        _case = 3;

        // Initialisation of input parameters of AdvApprox

        int num1DSS = 4, num2DSS = 0, num3DSS = 1;
        NCollection_HArray1<double> oneDTol = new(1, num1DSS);

        double tolV, tolW;
        ToleranceComputation(c2D1, surf1, 10, tol, out tolV, out tolW);
        oneDTol.SetValue(1, tolV);
        oneDTol.SetValue(2, tolW);

        ToleranceComputation(c2D2, surf2, 10, tol, out tolV, out tolW);
        oneDTol.SetValue(3, tolV);
        oneDTol.SetValue(4, tolW);

        NCollection_HArray1<double>? twoDTolNul = null;
        NCollection_HArray1<double> threeDTol = new(1, num3DSS);
        threeDTol.Init(tol / 2);

        CurvlinFunc fonct = new(c2D1, c2D2, surf1, surf2, tol / 20);

        double firstS = fonct.FirstParameter();
        double lastS = fonct.LastParameter();

        int NbInterv_C2 = fonct.NbIntervals(C2);
        NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
        fonct.Intervals(CutPnts_C2, C2);
        int NbInterv_C3 = fonct.NbIntervals(C3);
        NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
        fonct.Intervals(CutPnts_C3, C3);
        PrefAndRec cutTool = new(CutPnts_C2, CutPnts_C3);

        Approx_CurvilinearParameter_EvalCurvOn2Surf evCOn2S = new(fonct);
        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTol,
                                                twoDTolNul,
                                                threeDTol,
                                                firstS,
                                                lastS,
                                                order,
                                                maxDegree,
                                                maxSegments,
                                                evCOn2S,
                                                cutTool);

        _done = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            int nbPoles = approx.NbPoles();
            NCollection_Array1<Point3D> poles = new(1, nbPoles);
            NCollection_Array1<Point2D> poles2d = new(1, nbPoles);
            NCollection_Array1<double> poles1d = new(1, nbPoles);
            approx.Poles(1, poles);
            approx.Poles1d(1, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetX(poles1d[i]);
            }
            approx.Poles1d(2, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetY(poles1d[i]);
            }
            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();
            _curve3d = new BSplineCurve3D(poles, knots.Array1(), mults.Array1(), degree);
            _curve2d1 = new BSplineCurve2D(poles2d, knots.Array1(), mults.Array1(), degree);
            approx.Poles1d(3, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetX(poles1d[i]);
            }
            approx.Poles1d(4, poles1d);
            for (i = 1; i <= nbPoles; i++)
            {
                poles2d[i].SetY(poles1d[i]);
            }
            _curve2d2 = new BSplineCurve2D(poles2d, knots.Array1(), mults.Array1(), degree);
        }
        _maxError2d1 = Max(approx.MaxError(1, 1), approx.MaxError(1, 2));
        _maxError2d2 = Max(approx.MaxError(1, 3), approx.MaxError(1, 4));
        _maxError3d = approx.MaxError(3, 1);
    }

    /// <summary>Whether the approximation is within the tolerance.</summary>
    public bool IsDone => _done;

    /// <summary>Whether the approximation has a result, not necessarily within the tolerance.</summary>
    public bool HasResult => _hasResult;

    /// <summary>The reparametrized 3D curve (frozen; null without a result).</summary>
    public BSplineCurve3D? Curve3d()
    {
        _curve3d?.Freeze();
        return _curve3d;
    }

    /// <summary>The maximum error on the reparametrized 3D curve.</summary>
    public double MaxError3d() => _maxError3d;

    /// <summary>The reparametrized 2D curve on the first surface (frozen; null without one).</summary>
    public BSplineCurve2D? Curve2d1()
    {
        _curve2d1?.Freeze();
        return _curve2d1;
    }

    /// <summary>The reparametrized 2D curve on the first surface, unfrozen, for ported code that changes it (BRepLib::SameParameter re-knots it).</summary>
    internal BSplineCurve2D? InternalCurve2d1() => _curve2d1;

    /// <summary>The maximum error on the first reparametrized 2D curve.</summary>
    public double MaxError2d1() => _maxError2d1;

    /// <summary>The reparametrized 2D curve on the second surface (frozen; null without one).</summary>
    public BSplineCurve2D? Curve2d2()
    {
        _curve2d2?.Freeze();
        return _curve2d2;
    }

    /// <summary>The maximum error on the second reparametrized 2D curve.</summary>
    public double MaxError2d2() => _maxError2d2;

    /// <summary>Writes the maximum errors.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("Dump of Approx_CurvilinearParameter" + '\n');
        if (_case == 2 || _case == 3)
        {
            o.Write("myMaxError2d1 = " + _maxError2d1.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        if (_case == 3)
        {
            o.Write("myMaxError2d2 = " + _maxError2d2.ToString(CultureInfo.InvariantCulture) + '\n');
        }
        o.Write("myMaxError3d = " + _maxError3d.ToString(CultureInfo.InvariantCulture) + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    private static void ToleranceComputation(Curve2DAdaptorBase c2D, SurfaceAdaptorBase s, int maxNumber, double tol,
                                             out double tolV, out double tolW)
    {
        double firstU = c2D.FirstParameter(), lastU = c2D.LastParameter();
        double Max_dS_dv = 1.0, Max_dS_dw = 1.0;
        Point2D pntVW;

        for (int i = 1; i <= maxNumber; i++)
        {
            pntVW = c2D.Value(firstU + (i - 1) * (lastU - firstU) / (maxNumber - 1));
            s.D1(pntVW.X(), pntVW.Y(), out _, out Vector3D dS_dv, out Vector3D dS_dw);
            Max_dS_dv = Max(Max_dS_dv, dS_dv.Magnitude());
            Max_dS_dw = Max(Max_dS_dw, dS_dw.Magnitude());
        }
        tolV = tol / (4.0 * Max_dS_dv);
        tolW = tol / (4.0 * Max_dS_dw);
    }
}
