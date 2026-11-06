// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurveOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a curve on a surface (a 2D curve and the surface it lies on) with a 3D B-spline curve, a 2D B-spline
/// curve, or both, by AdvApprox. Isolines of a GeomAdaptor_Surface are taken from the surface when only the 3D curve
/// is asked for.
/// </summary>
internal class CurveOnSurfaceApproximation
{
    private sealed class Approx_CurveOnSurface_Eval : EvaluatorFunction
    {
        // Reassigned when AdvApprox asks for another interval, as OCCT's mutable handles are.
        private CurveAdaptorBase _fonct;
        private Curve2DAdaptorBase _fonct2d;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_CurveOnSurface_Eval(CurveAdaptorBase func, Curve2DAdaptorBase func2d, double first,
                                          double last)
        {
            _fonct = func;
            _fonct2d = func2d;
            _startEndSav0 = first;
            _startEndSav1 = last;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double param, int order,
                                      Span<double> result, out int errorCode)
        {
            errorCode = 0;
            double par = param;

            // Dimension is incorrect
            if (dimension != 5)
            {
                errorCode = 1;
            }

            // Parameter is incorrect
            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct = _fonct.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _fonct2d = _fonct2d.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }
            Point3D pnt;

            Point2D pnt2d;

            switch (order)
            {
                case 0:
                {
                    _fonct2d.D0(par, out pnt2d);
                    _fonct.D0(par, out pnt);
                    result[0] = pnt2d.X;
                    result[1] = pnt2d.Y;
                    result[2] = pnt.X;
                    result[3] = pnt.Y;
                    result[4] = pnt.Z;
                    break;
                }
                case 1:
                {
                    Vector3D v1;
                    Vector2D v21;
                    _fonct2d.D1(par, out pnt2d, out v21);
                    _fonct.D1(par, out pnt, out v1);
                    result[0] = v21.X;
                    result[1] = v21.Y;
                    result[2] = v1.X;
                    result[3] = v1.Y;
                    result[4] = v1.Z;
                    break;
                }
                case 2:
                {
                    Vector3D v1, v2;
                    Vector2D v21, v22;
                    _fonct2d.D2(par, out pnt2d, out v21, out v22);
                    _fonct.D2(par, out pnt, out v1, out v2);
                    result[0] = v22.X;
                    result[1] = v22.Y;
                    result[2] = v2.X;
                    result[3] = v2.Y;
                    result[4] = v2.Z;
                    break;
                }
                default:
                    result[0] = result[1] = result[2] = result[3] = result[4] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    private sealed class Approx_CurveOnSurface_Eval3d : EvaluatorFunction
    {
        private CurveAdaptorBase _fonct;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_CurveOnSurface_Eval3d(CurveAdaptorBase func, double first, double last)
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
            if (dimension != 3)
            {
                errorCode = 1;
            }

            // Parameter is incorrect
            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct = _fonct.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }

            Point3D pnt;

            switch (order)
            {
                case 0:
                    pnt = _fonct.Value(par);
                    result[0] = pnt.X;
                    result[1] = pnt.Y;
                    result[2] = pnt.Z;
                    break;
                case 1:
                {
                    Vector3D v1;
                    _fonct.D1(par, out pnt, out v1);
                    result[0] = v1.X;
                    result[1] = v1.Y;
                    result[2] = v1.Z;
                    break;
                }
                case 2:
                {
                    Vector3D v1, v2;
                    _fonct.D2(par, out pnt, out v1, out v2);
                    result[0] = v2.X;
                    result[1] = v2.Y;
                    result[2] = v2.Z;
                    break;
                }
                default:
                    result[0] = result[1] = result[2] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    private sealed class Approx_CurveOnSurface_Eval2d : EvaluatorFunction
    {
        private Curve2DAdaptorBase _fonct2d;
        private double _startEndSav0;
        private double _startEndSav1;

        public Approx_CurveOnSurface_Eval2d(Curve2DAdaptorBase func2d, double first, double last)
        {
            _fonct2d = func2d;
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
            if (startEnd[0] != _startEndSav0 || startEnd[1] != _startEndSav1)
            {
                _fonct2d = _fonct2d.Trim(startEnd[0], startEnd[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _startEndSav0 = startEnd[0];
                _startEndSav1 = startEnd[1];
            }

            Point2D pnt;

            switch (order)
            {
                case 0:
                {
                    pnt = _fonct2d.Value(par);
                    result[0] = pnt.X;
                    result[1] = pnt.Y;
                    break;
                }
                case 1:
                {
                    Vector2D v1;
                    _fonct2d.D1(par, out pnt, out v1);
                    result[0] = v1.X;
                    result[1] = v1.Y;
                    break;
                }
                case 2:
                {
                    Vector2D v1, v2;
                    _fonct2d.D2(par, out pnt, out v1, out v2);
                    result[0] = v2.X;
                    result[1] = v2.Y;
                    break;
                }
                default:
                    result[0] = result[1] = 0.0;
                    errorCode = 3;
                    break;
            }
        }
    }

    // Input curve.
    private readonly Curve2DAdaptorBase _c2D;
    // Input surface.
    private readonly SurfaceAdaptorBase _surf;
    // First parameter of the result.
    private readonly double _first;
    // Last parameter of the result.
    private readonly double _last;
    // Tolerance.
    private readonly double _tol;
    private BSplineCurve2D? _curve2d;
    private BSplineCurve3D? _curve3d;
    private bool _isDone;
    private bool _hasResult;
    private double _error3d;
    private double _error2dU;
    private double _error2dV;
    /// <summary>
    /// Sets up the approximation of theC2D on theSurf over [theFirst, theLast] within theTol, without computing it.
    /// The adaptors are kept, as OCCT keeps the handles.
    /// </summary>
    public CurveOnSurfaceApproximation(Curve2DAdaptorBase c2D, SurfaceAdaptorBase surf, double first, double last,
                                 double tol)
    {
        _c2D = c2D;
        _surf = surf;
        _first = first;
        _last = last;
        _tol = tol;
        _isDone = false;
        _hasResult = false;
        _error3d = 0.0;
        _error2dU = 0.0;
        _error2dV = 0.0;
    }

    /// <summary>
    /// Builds the curves: the 3D one only (theOnly3d), the 2D one only (theOnly2d), or both. When only the 3D curve
    /// is asked for and the 2D curve is an isoline, the isoline of the surface is taken and the other arguments are
    /// ignored.
    /// </summary>
    /// <exception cref="ConstructionErrorException">theOnly3d and theOnly2d are both true.</exception>
    public void Perform(int maxSegments, int maxDegree, Continuity continuity, bool only3d = false,
                        bool only2d = false)
    {
        _isDone = false;
        _hasResult = false;
        _error2dU = 0.0;
        _error2dV = 0.0;
        _error3d = 0.0;

        if (only3d && only2d)
        {
            throw new ConstructionErrorException();
        }

        Continuity aContinuity = continuity;
        if (aContinuity == G1)
        {
            aContinuity = C1;
        }
        else if (aContinuity == G2)
        {
            aContinuity = C2;
        }
        else if (aContinuity > C2)
        {
            aContinuity = C2; // Restriction of AdvApprox_ApproxAFunction
        }

        Curve2DAdaptorBase trimmedC2D = _c2D.Trim(_first, _last, global::Kerf.Engine.Foundation.Precision.PConfusion());

        bool isU, isForward;
        double param;
        if (only3d && IsIsoLine(trimmedC2D, out isU, out param, out isForward))
        {
            if (BuildC3dOnIsoLine(trimmedC2D, isU, param, isForward))
            {
                _isDone = true;
                _hasResult = true;
                return;
            }
        }

        CurveOnSurface hcOnS = new(trimmedC2D, _surf);

        int num1DSS = 0, num2DSS = 0, num3DSS = 0;
        NCollection_HArray1<double>? oneDTol = null;
        NCollection_HArray1<double>? twoDTolNul = null;
        NCollection_HArray1<double>? threeDTol = null;

        // create evaluators and choose appropriate one
        Approx_CurveOnSurface_Eval3d eval3dCvOnSurf = new(hcOnS, _first, _last);
        Approx_CurveOnSurface_Eval2d eval2dCvOnSurf = new(trimmedC2D, _first, _last);
        Approx_CurveOnSurface_Eval evalCvOnSurf = new(hcOnS, trimmedC2D, _first, _last);
        EvaluatorFunction evalPtr;
        if (only3d)
        {
            evalPtr = eval3dCvOnSurf;
        }
        else if (only2d)
        {
            evalPtr = eval2dCvOnSurf;
        }
        else
        {
            evalPtr = evalCvOnSurf;
        }

        // Initialization for 2d approximation
        if (!only3d)
        {
            num1DSS = 2;
            oneDTol = new NCollection_HArray1<double>(1, num1DSS);

            double tolU, tolV;

            tolU = _surf.UResolution(_tol) / 2.0;
            tolV = _surf.VResolution(_tol) / 2.0;

            if (_surf.UContinuity() == C0)
            {
                if (!HSurfaceTool.IsSurfG1(_surf, true, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    tolU = Min(1.0e-3, 1.0e3 * tolU);
                }
                if (!HSurfaceTool.IsSurfG1(_surf, true, global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    tolU = Min(1.0e-3, 1.0e2 * tolU);
                }
            }

            if (_surf.VContinuity() == C0)
            {
                if (!HSurfaceTool.IsSurfG1(_surf, false, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    tolV = Min(1.0e-3, 1.0e3 * tolV);
                }
                if (!HSurfaceTool.IsSurfG1(_surf, false, global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    tolV = Min(1.0e-3, 1.0e2 * tolV);
                }
            }

            oneDTol.SetValue(1, tolU);
            oneDTol.SetValue(2, tolV);
        }

        if (!only2d)
        {
            num3DSS = 1;
            threeDTol = new NCollection_HArray1<double>(1, num3DSS);
            threeDTol.Init(_tol / 2);
        }

        Cutting cutTool;

        if (aContinuity <= _c2D.Continuity() && aContinuity <= _surf.UContinuity()
            && aContinuity <= _surf.VContinuity())
        {
            cutTool = new DichoCutting();
        }
        else if (aContinuity == C1)
        {
            int NbInterv_C1 = hcOnS.NbIntervals(C1);
            NCollection_Array1<double> CutPnts_C1 = new(1, NbInterv_C1 + 1);
            hcOnS.Intervals(CutPnts_C1, C1);
            int NbInterv_C2 = hcOnS.NbIntervals(C2);
            NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
            hcOnS.Intervals(CutPnts_C2, C2);

            cutTool = new PrefAndRec(CutPnts_C1, CutPnts_C2);
        }
        else
        {
            int NbInterv_C2 = hcOnS.NbIntervals(C2);
            NCollection_Array1<double> CutPnts_C2 = new(1, NbInterv_C2 + 1);
            hcOnS.Intervals(CutPnts_C2, C2);
            int NbInterv_C3 = hcOnS.NbIntervals(C3);
            NCollection_Array1<double> CutPnts_C3 = new(1, NbInterv_C3 + 1);
            hcOnS.Intervals(CutPnts_C3, C3);

            cutTool = new PrefAndRec(CutPnts_C2, CutPnts_C3);
        }

        ApproxAFunction approx = new(num1DSS,
                                                num2DSS,
                                                num3DSS,
                                                oneDTol,
                                                twoDTolNul,
                                                threeDTol,
                                                _first,
                                                _last,
                                                aContinuity,
                                                maxDegree,
                                                maxSegments,
                                                evalPtr,
                                                cutTool);

        _isDone = approx.IsDone;
        _hasResult = approx.HasResult;

        if (_hasResult)
        {
            NCollection_HArray1<double> knots = approx.Knots()!;
            NCollection_HArray1<int> mults = approx.Multiplicities()!;
            int degree = approx.Degree();

            int nbPoles = approx.NbPoles();
            if (!only2d)
            {
                NCollection_Array1<Point3D> poles = new(1, nbPoles);
                approx.Poles(1, poles);
                _curve3d = new BSplineCurve3D(poles, knots.Array1(), mults.Array1(), degree);
                _error3d = approx.MaxError(3, 1);
            }
            if (!only3d)
            {
                NCollection_Array1<Point2D> poles2d = new(1, nbPoles);
                NCollection_Array1<double> poles1dU = new(1, nbPoles);
                approx.Poles1d(1, poles1dU);
                NCollection_Array1<double> poles1dV = new(1, nbPoles);
                approx.Poles1d(2, poles1dV);
                for (int i = 1; i <= nbPoles; i++)
                {
                    poles2d.SetValue(i, new Point2D(poles1dU.Value(i), poles1dV.Value(i)));
                }
                _curve2d = new BSplineCurve2D(poles2d, knots.Array1(), mults.Array1(), degree);

                _error2dU = approx.MaxError(1, 1);
                _error2dV = approx.MaxError(1, 2);
            }
        }
    }

    /// <summary>Whether the approximation is within the required tolerance.</summary>
    public bool IsDone => _isDone;

    /// <summary>Whether the approximation has a result, not necessarily within the required tolerance.</summary>
    public bool HasResult => _hasResult;

    /// <summary>The 3D curve (frozen: the same object until the next Perform), or null if none was built.</summary>
    public BSplineCurve3D? Curve3d()
    {
        _curve3d?.Freeze();
        return _curve3d;
    }

    /// <summary>The 3D curve without freezing it, for ported code that discards this object.</summary>
    internal BSplineCurve3D? InternalCurve3d() => _curve3d;

    /// <summary>The largest 3D error.</summary>
    public double MaxError3d() => _error3d;

    /// <summary>The 2D curve (frozen: the same object until the next Perform), or null if none was built.</summary>
    public BSplineCurve2D? Curve2d()
    {
        _curve2d?.Freeze();
        return _curve2d;
    }

    /// <summary>The 2D curve without freezing it, for ported code that discards this object.</summary>
    internal BSplineCurve2D? InternalCurve2d() => _curve2d;

    /// <summary>The largest error of the U component of the 2D curve.</summary>
    public double MaxError2dU() => _error2dU;

    /// <summary>The largest error of the V component of the 2D curve.</summary>
    public double MaxError2dV() => _error2dV;

    /// <summary>
    /// Checks whether the 2D curve is an isoline with natural parametrisation: a line, or a B-spline or Bezier curve
    /// of degree 1 with two poles, along U or V. Leaves the outputs unchanged when it is not.
    /// </summary>
    protected bool IsIsoLine(Curve2DAdaptorBase c2D, out bool isU, out double param, out bool isForward)
    {
        // OCCT leaves the outputs unset when it returns false; the port assigns them first (decision 32).
        isU = false;
        param = 0.0;
        isForward = false;

        // These variables are used to check line state (vertical or horizontal).
        bool isAppropriateType = false;
        Point2D loc2d = new();
        Direction2D dir2d = new();

        // Test type.
        CurveKind type = c2D.GetType();
        if (type == Line)
        {
            Line2D lin2d = c2D.Line();
            loc2d = lin2d.Location;
            dir2d = lin2d.Direction;
            isAppropriateType = true;
        }
        else if (type == BSplineCurve)
        {
            BSplineCurve2D bSpline2d = c2D.BSpline();
            if (bSpline2d.Degree != 1 || bSpline2d.PoleCount != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            loc2d = bSpline2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D vec2d = new(bSpline2d.Pole(1), bSpline2d.Pole(2));
            if (vec2d.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            dir2d = new Direction2D(vec2d);

            isAppropriateType = true;
        }
        else if (type == BezierCurve)
        {
            BezierCurve2D bezier2d = c2D.Bezier();
            if (bezier2d.Degree != 1 || bezier2d.PoleCount != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            loc2d = bezier2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D vec2d = new(bezier2d.Pole(1), bezier2d.Pole(2));
            if (vec2d.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            dir2d = new Direction2D(vec2d);

            isAppropriateType = true;
        }

        if (!isAppropriateType)
        {
            return false;
        }

        // Check line to be vertical or horizontal.
        if (dir2d.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Horizontal line. V = const.
            isU = false;
            param = loc2d.Y;
            isForward = dir2d.Dot(Constants.DX2d()) > 0.0;
            return true;
        }
        else if (dir2d.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Vertical line. U = const.
            isU = true;
            param = loc2d.X;
            isForward = dir2d.Dot(Constants.DY2d()) > 0.0;
            return true;
        }

        return false;
    }

    /// <summary>
    /// Builds the 3D curve of an isoline from the surface's own isoline (GeomAdaptor_Surface only, not spheres),
    /// reparametrised as the 2D curve. True when it is within the tolerance.
    /// </summary>
    protected bool BuildC3dOnIsoLine(Curve2DAdaptorBase c2D, bool isU, double param, bool isForward)
    {
        // Convert adapter to the appropriate type.
        SurfaceAdaptor? geomAdapter = _surf as SurfaceAdaptor;
        if (geomAdapter is null)
        {
            return false;
        }

        if (_surf.GetType() == SurfaceKind.Sphere)
        {
            return false;
        }

        // Extract isoline
        Surface surf = geomAdapter.Surface()!;
        Curve3D c3d;

        Point2D f2d = c2D.Value(c2D.FirstParameter());
        Point2D l2d = c2D.Value(c2D.LastParameter());

        bool isToTrim = true;
        double u1, u2, v1, v2;
        surf.ParameterBounds(out u1, out u2, out v1, out v2);

        if (isU)
        {
            double v1Param = Min(f2d.Y, l2d.Y);
            double v2Param = Max(f2d.Y, l2d.Y);
            if (v2Param < v1 - _tol || v1Param > v2 + _tol)
            {
                return false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                if (Math.Abs(v2Param - v1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
                surf = new TrimmedSurface(surf, u1, u2, v1Param, v2Param);
                isToTrim = false;
            }
            else
            {
                v1Param = Max(v1Param, v1);
                v2Param = Min(v2Param, v2);
                if (Math.Abs(v2Param - v1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
            c3d = surf.UIso(param);
            if (isToTrim)
            {
                c3d = new TrimmedCurve3D(c3d, v1Param, v2Param);
            }
        }
        else
        {
            double u1Param = Min(f2d.X, l2d.X);
            double u2Param = Max(f2d.X, l2d.X);
            if (u2Param < u1 - _tol || u1Param > u2 + _tol)
            {
                return false;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                if (Math.Abs(u2Param - u1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
                surf = new TrimmedSurface(surf, u1Param, u2Param, v1, v2);
                isToTrim = false;
            }
            else
            {
                u1Param = Max(u1Param, u1);
                u2Param = Min(u2Param, u2);
                if (Math.Abs(u2Param - u1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
            c3d = surf.VIso(param);
            if (isToTrim)
            {
                c3d = new TrimmedCurve3D(c3d, u1Param, u2Param);
            }
        }

        // Convert arbitrary curve type to the b-spline.
        BSplineCurve3D curve3d = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c3d, QuasiAngular);
        // The conversion can return a frozen curve that others share (decision 244): change a copy.
        if (curve3d.IsFrozen)
        {
            curve3d = (BSplineCurve3D)curve3d.Copy();
        }
        _curve3d = curve3d;
        if (!isForward)
        {
            curve3d.Reverse();
        }

        // Rebuild parameterization for the 3d curve to have the same parameterization with
        // a two-dimensional curve.
        NCollection_Array1<double> knots = new(curve3d.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(c2D.FirstParameter(), c2D.LastParameter(), knots);
        curve3d.SetKnots(knots);

        // Evaluate error.
        _error3d = 0.0;

        double parF = _first;
        double parL = _last;
        const int nbPnt = 23;
        for (int idx = 0; idx <= nbPnt; ++idx)
        {
            double par = parF + ((parL - parF) * idx) / nbPnt;

            Point2D pnt2d = c2D.Value(par);

            Point3D pntC3D = curve3d.Value(par);
            Point3D pntC2D = _surf.Value(pnt2d.X, pnt2d.Y);

            double sqDeviation = pntC3D.SquareDistanceTo(pntC2D);
            _error3d = Max(sqDeviation, _error3d);
        }

        _error3d = Math.Sqrt(_error3d);

        // Target tolerance is not obtained. This situation happens for isolines on the sphere.
        // OCCT is unable to convert it keeping original parameterization, while the geometric
        // form of the result is entirely identical. In that case, it is better to utilize
        // a general-purpose approach.
        return _error3d <= _tol;
    }
}
