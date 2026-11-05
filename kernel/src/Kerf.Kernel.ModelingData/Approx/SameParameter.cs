// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SameParameter.hxx
// Port of: src/ModelingData/TKGeomBase/Approx/Approx_SameParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a pcurve on a surface so that its parameter is the same as that of a given 3D reference curve:
/// evaluated on the surface at a parameter, the new pcurve gives the point of the 3D curve at that parameter, within
/// the tolerance. The computation is made by the constructor. The 3D curve and the pcurve must have the same
/// parametric range.
/// </summary>
internal sealed class SameParameter
{
    // Evaluates the pcurve through the 1D B-spline (degree 3) of the change of parameter.
    private sealed class Approx_SameParameter_Evaluator : EvaluatorFunction
    {
        // OCCT keeps const references to the caller's arrays, which outlive the approximation.
        private readonly NCollection_Array1<double> _flatKnots;
        private readonly NCollection_Array1<double> _poles;
        private readonly Curve2DAdaptorBase _hCurve2d;

        public Approx_SameParameter_Evaluator(NCollection_Array1<double> flatKnots,
                                              NCollection_Array1<double> poles,
                                              Curve2DAdaptorBase hCurve2d)
        {
            _flatKnots = flatKnots;
            _poles = poles;
            _hCurve2d = hCurve2d;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int returnCode)
        {
            const int degree = 3;
            ReadOnlySpan<int> extrap_mode = [degree, degree];
            // OCCT's two-element buffer: AdvApprox asks for at most the first derivative here (continuity C1 at most).
            Span<double> eval_result = stackalloc double[2];
            ReadOnlySpan<double> polesArray = _poles.AsSpan();

            // Evaluate the 1D B-Spline that represents the change in parameterization.
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(parameter,
                          false,
                          derivativeRequest,
                          extrap_mode,
                          degree,
                          _flatKnots,
                          1,
                          polesArray,
                          eval_result);

            Point2D point;
            Vector2D vector;
            if (derivativeRequest == 0)
            {
                _hCurve2d.D0(eval_result[0], out point);
                point.Coord(out result[0], out result[1]);
            }
            else if (derivativeRequest == 1)
            {
                _hCurve2d.D1(eval_result[0], out point, out vector);
                vector = vector.Multiplied(eval_result[1]);
                vector.Coord(out result[0], out result[1]);
            }

            returnCode = 0;
        }
    }

    // Internal data structure to unify access to the most actively used data. OCCT's points into four stack arrays
    // of myMaxArraySize values; the port owns four arrays, swapped as OCCT swaps the pointers.
    private sealed class Approx_SameParameter_Data
    {
        public CurveOnSurface COnS = null!; // Curve on surface.
        public int NbPnt;                             // Number of points.
        public double[] PC3d = null!;                 // Parameters on 3d curve.
        public double[] PC2d = null!;                 // Parameters on 2d curve.

        // Second data arrays. Used in loop over poles.
        public double[] NewPC3d = null!; // Parameters on 3d curve.
        public double[] NewPC2d = null!; // Parameters on 2d curve.

        // Parameters ranges.
        public double C3dPF; // Curve 3d Parameter First.
        public double C3dPL; // Curve 3d Parameter Last.
        public double C2dPF; // Curve 2d Parameter First.
        public double C2dPL; // Curve 2d Parameter Last.

        public double myTol; // Working tolerance.

        // Swap data arrays and update number of points.
        public void Swap(int newNbPoints)
        {
            NbPnt = newNbPoints;
            double[] temp;

            // 3-D
            temp = PC3d;
            PC3d = NewPC3d;
            NewPC3d = temp;

            // 2-D
            temp = PC2d;
            PC2d = NewPC2d;
            NewPC2d = temp;
        }
    }

    private const int NbSamples = 22; // To be consistent with "checkshape".
    private const int MaxArraySize = 1000;
    private readonly double _deltaMin;
    private bool _sameParameter;
    private bool _done;
    private double _tolReached;
    private Curve2D? _curve2d;
    // Whether myCurve2d was built here (and so may be frozen when handed out), not the caller's own pcurve.
    private bool _curve2dIsBuilt;
    private Curve2DAdaptorBase _hCurve2d;
    private readonly CurveAdaptorBase _c3d;
    private readonly SurfaceAdaptorBase _surf;
    private CurveOnSurface? _curveOnSurface;

    /// <summary>Makes C2D on S have the parameter of C3D, within Tol. The curves must have the same range.</summary>
    public SameParameter(Curve3D c3D, Curve2D c2D, Surface s, double tol)
    {
        _deltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _sameParameter = true;
        _done = false;
        _hCurve2d = new CurveAdaptor2D(c2D);
        _c3d = new CurveAdaptor(c3D);
        _surf = new SurfaceAdaptor(s);
        Build(tol);
    }

    /// <summary>As the other constructors, for curves and a surface given by adaptors (kept by reference).</summary>
    public SameParameter(CurveAdaptorBase c3D, Curve2D c2D, SurfaceAdaptorBase s, double tol)
    {
        _deltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _sameParameter = true;
        _done = false;
        _c3d = c3D;
        _surf = s;
        _hCurve2d = new CurveAdaptor2D(c2D);
        Build(tol);
    }

    /// <summary>As the other constructors, for curves and a surface given by adaptors (kept by reference).</summary>
    public SameParameter(CurveAdaptorBase c3D, Curve2DAdaptorBase c2D, SurfaceAdaptorBase s, double tol)
    {
        _deltaMin = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _sameParameter = true;
        _done = false;
        _c3d = c3D;
        _surf = s;
        _hCurve2d = c2D;
        Build(tol);
    }

    /// <summary>False if the computation failed, true if it succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>The tolerance (maximal distance) between the 3D curve and the curve on surface of the result.</summary>
    public double TolReached() => _tolReached;

    /// <summary>Whether the data already had the same parameter within the tolerance (then nothing is built).</summary>
    public bool IsSameParameter => _sameParameter;

    /// <summary>
    /// The 2D curve that has the parameter of the 3D curve on the surface within the tolerance, or null when none was
    /// computed. A curve built by the algorithm is frozen (the same object at each call); otherwise it is the
    /// caller's pcurve (or a trimmed curve on it), as OCCT returns it.
    /// </summary>
    public Curve2D? Curve2d()
    {
        if (_curve2dIsBuilt)
        {
            _curve2d?.Freeze();
        }
        return _curve2d;
    }

    /// <summary>The 2D curve without freezing it, for ported code that discards this object.</summary>
    internal Curve2D? InternalCurve2d() => _curve2d;

    /// <summary>The 3D curve (the caller's adaptor).</summary>
    public CurveAdaptorBase Curve3d() => _c3d;

    /// <summary>
    /// The curve on surface of the result (the algorithm's own adaptor), or null when the computation stopped early.
    /// </summary>
    public CurveOnSurface? CurveOnSurface() => _curveOnSurface;

    private static void ProjectPointOnCurve(double initValue,
                                            in Point3D aPoint,
                                            double tolerance,
                                            int numIteration,
                                            CurveAdaptorBase curve,
                                            out bool status,
                                            out double result)
    {
        int num_iter = 0, not_done = 1;

        Point3D a_point;
        Vector3D vector, d1, d2;
        double func, func_derivative, param = initValue;
        status = false;
        do
        {
            num_iter++;
            curve.D2(param, out a_point, out d1, out d2);
            vector = new Vector3D(a_point, aPoint);

            func = vector.Dot(d1);
            if (Math.Abs(func) < tolerance * d1.Magnitude)
            {
                not_done = 0;
                status = true;
            }
            else
            {
                func_derivative = vector.Dot(d2) - d1.Dot(d1);

                // Avoid division by zero.
                const double toler = 1.0e-12;
                if (Math.Abs(func_derivative) > toler)
                {
                    param -= func / func_derivative;
                }

                param = Max(param, curve.FirstParameter());
                param = Min(param, curve.LastParameter());
            }
        } while (not_done != 0 && num_iter <= numIteration);

        result = param;
    }

    private static double ComputeTolReached(CurveAdaptorBase c3d, CurveOnSurface cons, int nbp)
    {
        double d2 = 0.0; // Square max discrete deviation.
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        for (int i = 0; i <= nbp; i++)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            Point3D pc3d, pcons;
            try
            {
                pc3d = c3d.Value(u);
                pcons = cons.Value(u);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Y)
                || global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Z))
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            d2 = Max(d2, pc3d.SquareDistanceTo(pcons));
        }

        const double mult = 1.0 + 0.05; //
        double deviation = mult * Math.Sqrt(d2);
        deviation = Max(deviation, global::Kerf.Engine.Foundation.Precision.Confusion()); // Tolerance in modeling space.
        return deviation;
    }

    private static bool Check(NCollection_Array1<double> flatKnots,
                              NCollection_Array1<double> poles,
                              int nbp,
                              double[] pc3d,
                              CurveAdaptorBase c3d,
                              CurveOnSurface cons,
                              ref double tol,
                              double oldtol)
    {
        const int degree = 3;
        ReadOnlySpan<int> extrap_mode = [degree, degree];
        Span<double> tcons = stackalloc double[1];

        // Correction of the interval of valid values. This condition has no sensible
        // grounds. But it is better then the old one (which is commented out) because
        // it fixes the bug OCC5898. To develop more or less sensible criterion it is
        // necessary to deeply investigate this problem which is not possible in frames
        // of debugging.
        double paramFirst = 3.0 * pc3d[0] - 2.0 * pc3d[nbp - 1];
        double paramLast = 3.0 * pc3d[nbp - 1] - 2.0 * pc3d[0];

        double firstPar = cons.FirstParameter();
        double lastPar = cons.LastParameter();
        if (paramFirst < firstPar)
        {
            paramFirst = firstPar;
        }
        if (paramLast > lastPar)
        {
            paramLast = lastPar;
        }

        double d2 = 0.0; // Maximum square deviation on the samples.
        double d = tol;
        int nn = 2 * nbp;
        double unsurnn = 1.0 / nn;
        double tprev = paramFirst;
        for (int i = 0; i <= nn; i++)
        {
            // Compute corresponding parameter on 2d curve.
            // It should be inside of 3d curve parameter space.
            double t = unsurnn * i;
            double tc3d = pc3d[0] * (1.0 - t) + pc3d[nbp - 1] * t; // weight function.
            Point3D Pc3d = c3d.Value(tc3d);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(tc3d, false, 0, extrap_mode, degree, flatKnots, 1, poles.AsSpan(1), tcons);

            if (tcons[0] < tprev || tcons[0] > paramLast)
            {
                tol = global::Kerf.Engine.Foundation.Precision.Infinite();
                return false;
            }
            tprev = tcons[0];
            Point3D pcons = cons.Value(tcons[0]);
            double temp = Pc3d.SquareDistanceTo(pcons);
            if (temp > d2)
            {
                d2 = temp;
            }
        }
        tol = Math.Sqrt(d2);

        // Check poles parameters to be ordered.
        for (int i = poles.Lower() + 1; i <= poles.Upper(); ++i)
        {
            double previousParam = poles[i - 1];
            double currentParam = poles[i];

            if (previousParam > currentParam)
            {
                return false;
            }
        }

        return (tol <= d || tol > 0.8 * oldtol);
    }

    // Computes the pcurve (internal use only).
    private void Build(double tolerance)
    {
        // Algorithm:
        // 1) Build initial distribution. Increase number of samples in case of C0 continuity.
        // 2.1) Check same parameter state on samples.
        // 2.2) Compute parameters in 2d space if not same parameter.
        // 3) Loop over poles number and try to interpolate 2d curve.
        // 4) If loop is failed build 2d curve forcibly or use original pcurve.
        double[] qpcons = new double[MaxArraySize], qnewpcons = new double[MaxArraySize],
                 qpc3d = new double[MaxArraySize], qnewpc3d = new double[MaxArraySize];

        // Create and fill data structure.
        Approx_SameParameter_Data data = new();
        data.COnS = new CurveOnSurface(_hCurve2d, _surf);
        data.C2dPF = data.COnS.FirstParameter();
        data.C2dPL = data.COnS.LastParameter();
        data.C3dPF = _c3d.FirstParameter();
        data.C3dPL = _c3d.LastParameter();
        data.NbPnt = 0; // No points initially.
        data.PC2d = qpcons;
        data.PC3d = qpc3d;
        data.NewPC2d = qnewpcons;
        data.NewPC3d = qnewpc3d;
        data.myTol = tolerance;

        // Build initial distribution.
        if (!BuildInitialDistribution(data))
        {
            _sameParameter = false;
            _done = false;
            return;
        }

        // Check same parameter state on distribution.
        double maxSqDeviation = 0.0;
        const double percentOfBadProj = 0.3;
        int nbPnt = data.NbPnt - RealToInt(percentOfBadProj * data.NbPnt);
        _sameParameter = CheckSameParameter(data, out maxSqDeviation);
        if (_sameParameter)
        {
            _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
            _done = true;
            return;
        }
        else
        {
            // Control number of sample points after checking sameparameter
            // If number of points is less then initial one, it means that there are
            // problems with projection
            if (data.NbPnt < nbPnt)
            {
                _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
                _curve2d = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(_hCurve2d);
                _curve2dIsBuilt = false;
                _done = false;
                return;
            }
        }

        // Control tangents at the extremities to know if the
        // reparametring is possible and calculate the tangents
        // at the extremities of the function of change of variable.
        double tangent0 = 0.0, tangent1 = 0.0;
        if (!ComputeTangents(data.COnS, ref tangent0, ref tangent1))
        {
            // Cannot compute tangents.
            _sameParameter = false;
            _done = false;
            _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
            return;
        }

        // There is at least one point where same parameter is broken.
        // Try to build B-spline approximation curve using interpolation with degree 3.
        // The loop is organized over number of poles.
        Continuity continuity = _hCurve2d.Continuity();
        if (continuity > C1)
        {
            continuity = C1;
        }

        double besttol2 = data.myTol * data.myTol, tolsov = global::Kerf.Engine.Foundation.Precision.Infinite();
        bool interpolok = false, hasCountChanged = false;
        do
        {
            // Interpolation data.
            int num_knots = data.NbPnt + 7;
            int num_poles = data.NbPnt + 3;
            NCollection_Array1<double> poles = new(1, num_poles);
            NCollection_Array1<double> flatKnots = new(1, num_knots);

            if (!Interpolate(data, tangent0, tangent1, poles, flatKnots))
            {
                // Interpolation fails.
                _sameParameter = false;
                _done = false;
                _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
                return;
            }

            double algtol = Math.Sqrt(besttol2);
            interpolok =
              Check(flatKnots, poles, data.NbPnt + 1, data.PC3d, _c3d, data.COnS, ref algtol, tolsov);
            tolsov = algtol;

            // Try to build 2d curve and check it for validity.
            if (interpolok)
            {
                double besttol = Math.Sqrt(besttol2);

                NCollection_HArray1<double> tol1d;
                NCollection_HArray1<double>? tol2d = null, tol3d = null;
                tol1d = new NCollection_HArray1<double>(1, 2);
                tol1d.SetValue(1, _surf.UResolution(besttol));
                tol1d.SetValue(2, _surf.VResolution(besttol));

                Approx_SameParameter_Evaluator ev = new(flatKnots, poles, _hCurve2d);
                int maxDeg = 11, maxSeg = 1000;
                ApproxAFunction approximator = new(2,
                                                               0,
                                                               0,
                                                               tol1d,
                                                               tol2d,
                                                               tol3d,
                                                               data.C3dPF,
                                                               data.C3dPL,
                                                               continuity,
                                                               maxDeg,
                                                               maxSeg,
                                                               ev);

                if (approximator.IsDone || approximator.HasResult)
                {
                    // OCCT copies the curve on surface to restore it below; only its 2D curve changes, so the port
                    // keeps that curve and loads it back.
                    Curve2DAdaptorBase acs = data.COnS.GetCurve()!;
                    MakeCurvefromApprox curveBuilder = new(approximator);
                    BSplineCurve2D c2d = curveBuilder.Curve2dFromTwo1d(1, 2);
                    Curve2DAdaptorBase hCurve2d = new CurveAdaptor2D(c2d);
                    data.COnS.Load(hCurve2d);
                    _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);

                    const double mult = 250.0; // To be tolerant with discrete tolerance.
                    if (_tolReached < mult * besttol)
                    {
                        _curve2d = c2d;
                        _curve2dIsBuilt = true;
                        _hCurve2d = hCurve2d;
                        _done = true;
                        break;
                    }
                    else if (data.NbPnt < MaxArraySize - 1)
                    {
                        interpolok = false;
                        data.COnS.Load(acs);
                    }
                    else
                    {
                        break;
                    }
                }
            }

            if (!interpolok)
            {
                hasCountChanged = IncreaseNbPoles(poles, flatKnots, data, ref besttol2);
            }
        } while (!interpolok && hasCountChanged);

        if (!_done)
        {
            // Loop is finished unsuccessfully. Fix tolerance by maximal deviation,
            // using data from the last loop iteration or initial data. Use data set with minimal
            // deflection.

            // Original 2d curve.
            data.COnS.Load(_hCurve2d);
            _tolReached = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
            _curve2d = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(_hCurve2d);
            _curve2dIsBuilt = false;

            // Approximation curve.
            int num_knots = data.NbPnt + 7;
            int num_poles = data.NbPnt + 3;
            NCollection_Array1<double> poles = new(1, num_poles);
            NCollection_Array1<double> flatKnots = new(1, num_knots);

            Interpolate(data, tangent0, tangent1, poles, flatKnots);

            double besttol = Math.Sqrt(besttol2);
            NCollection_HArray1<double> tol1d;
            NCollection_HArray1<double>? tol2d = null, tol3d = null;
            tol1d = new NCollection_HArray1<double>(1, 2);
            tol1d.SetValue(1, _surf.UResolution(besttol));
            tol1d.SetValue(2, _surf.VResolution(besttol));

            Approx_SameParameter_Evaluator ev = new(flatKnots, poles, _hCurve2d);
            ApproxAFunction approximator = new(2,
                                                           0,
                                                           0,
                                                           tol1d,
                                                           tol2d,
                                                           tol3d,
                                                           data.C3dPF,
                                                           data.C3dPL,
                                                           continuity,
                                                           11,
                                                           40,
                                                           ev);

            if (!approximator.IsDone && !approximator.HasResult)
            {
                _done = false;
                return;
            }

            MakeCurvefromApprox curveBuilder = new(approximator);
            BSplineCurve2D c2d = curveBuilder.Curve2dFromTwo1d(1, 2);
            Curve2DAdaptorBase hCurve2d = new CurveAdaptor2D(c2d);
            data.COnS.Load(hCurve2d);

            double approxTol = ComputeTolReached(_c3d, data.COnS, 2 * NbSamples);
            if (approxTol < _tolReached)
            {
                _tolReached = approxTol;
                _curve2d = c2d;
                _curve2dIsBuilt = true;
                _hCurve2d = hCurve2d;
            }
            _done = true;
        }

        _curveOnSurface = (CurveOnSurface)data.COnS.ShallowCopy();
    }

    // Computes initial point distribution.
    private bool BuildInitialDistribution(Approx_SameParameter_Data data)
    {
        // Take a multiple of the sample pof CheckShape,
        // at least the control points will be correct.
        // Take parameters with constant step on the curve on surface
        // and on curve 3d.
        double deltacons = (data.C2dPL - data.C2dPF) / NbSamples;
        double deltac3d = (data.C3dPL - data.C3dPF) / NbSamples;
        double wcons = data.C2dPF;
        double wc3d = data.C3dPF;
        for (int ii = 0; ii < NbSamples; ii++)
        {
            data.PC2d[ii] = wcons;
            data.PC3d[ii] = wc3d;
            wcons += deltacons;
            wc3d += deltac3d;
        }
        data.NbPnt = NbSamples;
        data.PC2d[data.NbPnt] = data.C2dPL;
        data.PC3d[data.NbPnt] = data.C3dPL;

        // Change number of points in case of C0 continuity.
        Continuity continuity = _hCurve2d.Continuity();
        if (continuity < C1)
        {
            if (!IncreaseInitialNbSamples(data))
            {
                // Number of samples is too big.
                return false;
            }
        }

        return true;
    }

    // Get number of C1 intervals and build new distribution on them.
    // Sub-method in BuildInitialDistribution.
    private bool IncreaseInitialNbSamples(Approx_SameParameter_Data data)
    {
        int nbInt = _hCurve2d.NbIntervals(C1) + 1;
        NCollection_Array1<double> c1Intervals = new(1, nbInt);
        _hCurve2d.Intervals(c1Intervals, C1);

        int inter = 1;
        while (inter <= nbInt && c1Intervals[inter] <= data.C3dPF + _deltaMin)
        {
            inter++;
        }
        while (nbInt > 0 && c1Intervals[nbInt] >= data.C3dPL - _deltaMin)
        {
            nbInt--;
        }

        // Compute new parameters.
        NCollection_Sequence<double> newPar = new();
        newPar.Append(data.C3dPF);
        int ii = 1;
        while (inter <= nbInt || (ii < NbSamples && inter <= c1Intervals.Length()))
        {
            if (c1Intervals[inter] < data.PC2d[ii])
            {
                newPar.Append(c1Intervals[inter]);
                if ((data.PC2d[ii] - c1Intervals[inter]) <= _deltaMin)
                {
                    ii++;
                    if (ii > NbSamples)
                    {
                        ii = NbSamples;
                    }
                }
                inter++;
            }
            else
            {
                if ((c1Intervals[inter] - data.PC2d[ii]) > _deltaMin)
                {
                    newPar.Append(data.PC2d[ii]);
                }
                ii++;
            }
        }
        // Simple protection if theNewNbPoints > allocated elements in array but one
        // myMaxArraySize - 1 index may be filled after projection.
        data.NbPnt = newPar.Length();
        if (data.NbPnt > MaxArraySize - 1)
        {
            return false;
        }

        for (ii = 1; ii < data.NbPnt; ii++)
        {
            // Copy only internal points.
            data.PC2d[ii] = data.PC3d[ii] = newPar.Value(ii + 1);
        }
        data.PC3d[data.NbPnt] = data.C3dPL;
        data.PC2d[data.NbPnt] = data.C2dPL;

        return true;
    }

    // Checks the same parameter state and builds the dependency between the 2d and 3d curves.
    private bool CheckSameParameter(Approx_SameParameter_Data data, out double sqDist)
    {
        double tol2 = data.myTol * data.myTol;
        bool isSameParam = true;

        // Compute initial distance on boundary points.
        Point3D pcons, pc3d;
        data.COnS.D0(data.C2dPF, out pcons);
        _c3d.D0(data.C3dPF, out pc3d);
        double dist2 = pcons.SquareDistanceTo(pc3d);
        double dmax2 = dist2;

        data.COnS.D0(data.C2dPL, out pcons);
        _c3d.D0(data.C3dPL, out pc3d);
        dist2 = pcons.SquareDistanceTo(pc3d);
        dmax2 = Max(dmax2, dist2);

        LocateExtPC projector = new();
        projector.Initialize(_c3d, data.C3dPF, data.C3dPL, data.myTol);

        int count = 1;
        double previousp = data.C3dPF, initp = 0, curp = 0;
        double bornesup = data.C3dPL - _deltaMin;
        bool isProjOk = false;
        for (int ii = 1; ii < data.NbPnt; ii++)
        {
            data.COnS.D0(data.PC2d[ii], out pcons);
            _c3d.D0(data.PC3d[ii], out pc3d);
            dist2 = pcons.SquareDistanceTo(pc3d);

            // Same parameter point.
            bool isUseParam = (dist2 <= tol2 && // Good distance.
                               (data.PC3d[ii] > data.PC3d[count - 1]
                                                       + _deltaMin)); // Point is separated from previous.
            if (isUseParam)
            {
                if (dmax2 < dist2)
                {
                    dmax2 = dist2;
                }
                initp = previousp = data.PC3d[count] = data.PC3d[ii];
                data.PC2d[count] = data.PC2d[ii];
                count++;
                continue;
            }

            // Local search: local extrema and iterative projection algorithm.
            if (!isProjOk)
            {
                initp = data.PC3d[ii];
            }
            isProjOk = isSameParam = false;
            projector.Perform(pcons, initp);
            if (projector.IsDone)
            {
                // Local extrema is found.
                curp = projector.Point().Parameter();
                isProjOk = true;
            }
            else
            {
                ProjectPointOnCurve(initp, pcons, data.myTol, 30, _c3d, out isProjOk, out curp);
            }
            isProjOk = isProjOk &&                      // Good projection.
                       curp > previousp + _deltaMin && // Point is separated from previous.
                       curp < bornesup;                 // Inside of parameter space.
            if (isProjOk)
            {
                initp = previousp = data.PC3d[count] = curp;
                data.PC2d[count] = data.PC2d[ii];
                count++;
                continue;
            }

            // Whole parameter space search using general extrema.
            CurvePointExtrema pr = new(pcons, _c3d, data.C3dPF, data.C3dPL, data.myTol);
            if (!pr.IsDone || pr.ExtCount == 0)
            { // Lazy evaluation is used.
                continue;
            }

            int nbExt = pr.ExtCount;
            int indMin = 0;
            double curDistMin = RealLast();
            for (int i = 1; i <= nbExt; i++)
            {
                Point3D p = pr.Point(i).Value();
                double aDist2 = p.SquareDistanceTo(pcons);
                if (aDist2 < curDistMin)
                {
                    curDistMin = aDist2;
                    indMin = i;
                }
            }
            if (indMin != 0)
            {
                curp = pr.Point(indMin).Parameter();
                if (curp > previousp + _deltaMin && curp < bornesup)
                {
                    initp = previousp = data.PC3d[count] = curp;
                    data.PC2d[count] = data.PC2d[ii];
                    count++;
                    isProjOk = true;
                }
            }
        }
        data.NbPnt = count;
        data.PC2d[data.NbPnt] = data.C2dPL;
        data.PC3d[data.NbPnt] = data.C3dPL;

        sqDist = dmax2;
        return isSameParam;
    }

    // Computes the tangents on the boundary points; false if they are null. The outputs are left unchanged then, as
    // in OCCT.
    private bool ComputeTangents(CurveOnSurface cOnS, ref double firstTangent,
                                 ref double lastTangent)
    {
        const double smallMagnitude = 1.0e-12;
        // Check tangency on curve border.
        Point3D pnt, pntCOnS;
        Vector3D vec, vecConS;

        // First point.
        double paramFirst = _c3d.FirstParameter();
        cOnS.D1(paramFirst, out pntCOnS, out vecConS);
        _c3d.D1(paramFirst, out pnt, out vec);
        double magnitude = vecConS.Magnitude;
        if (magnitude > smallMagnitude)
        {
            firstTangent = vec.Magnitude / magnitude;
        }
        else
        {
            return false;
        }

        // Last point.
        double paramLast = _c3d.LastParameter();
        cOnS.D1(paramLast, out pntCOnS, out vecConS);
        _c3d.D1(paramLast, out pnt, out vec);

        magnitude = vecConS.Magnitude;
        if (magnitude > smallMagnitude)
        {
            lastTangent = vec.Magnitude / magnitude;
        }
        else
        {
            return false;
        }

        return true;
    }

    // Computes interpolated values; false if the computation failed.
    private static bool Interpolate(Approx_SameParameter_Data data,
                                    double aTangFirst,
                                    double aTangLast,
                                    NCollection_Array1<double> poles,
                                    NCollection_Array1<double> flatKnots)
    {
        int num_poles = data.NbPnt + 3;
        NCollection_Array1<int> contactOrder = new(1, num_poles);
        NCollection_Array1<double> parameters = new(1, num_poles);

        // Fill tables taking attention to end values.
        contactOrder.Init(0);
        contactOrder[2] = contactOrder[num_poles - 1] = 1;

        flatKnots[1] = flatKnots[2] = flatKnots[3] = flatKnots[4] = data.C3dPF;
        flatKnots[num_poles + 1] = flatKnots[num_poles + 2] = flatKnots[num_poles + 3] =
          flatKnots[num_poles + 4] = data.C3dPL;

        poles[1] = data.C2dPF;
        poles[num_poles] = data.C2dPL;
        poles[2] = aTangFirst;
        poles[num_poles - 1] = aTangLast;

        parameters[1] = parameters[2] = data.C3dPF;
        parameters[num_poles - 1] = parameters[num_poles] = data.C3dPL;

        for (int ii = 3; ii <= num_poles - 2; ii++)
        {
            poles[ii] = data.PC2d[ii - 2];
            parameters[ii] = flatKnots[ii + 2] = data.PC3d[ii - 2];
        }
        int inversion_problem;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(3,
                             flatKnots,
                             parameters,
                             contactOrder,
                             1,
                             poles.AsSpan(1),
                             out inversion_problem);
        return inversion_problem == 0;
    }

    // Increases the number of poles in the poles loop; true if the distribution changed.
    private bool IncreaseNbPoles(NCollection_Array1<double> poles,
                                 NCollection_Array1<double> flatKnots,
                                 Approx_SameParameter_Data data,
                                 ref double bestSqTol)
    {
        LocateExtPC projector = new();
        projector.Initialize(_c3d, _c3d.FirstParameter(), _c3d.LastParameter(), data.myTol);
        double curp = 0.0;
        bool projok = false;

        // Project middle point to fix parameterization and check projection existence.
        const int degree = 3;
        const int derivativeRequest = 0;
        ReadOnlySpan<int> extrap_mode = [degree, degree];
        Span<double> eval_result = stackalloc double[1];
        ReadOnlySpan<double> polesArray = poles.AsSpan();
        int newcount = 0;
        for (int ii = 0; ii < data.NbPnt; ii++)
        {
            data.NewPC2d[newcount] = data.PC2d[ii];
            data.NewPC3d[newcount] = data.PC3d[ii];
            newcount++;

            if (data.NbPnt - ii + newcount == MaxArraySize)
            {
                continue;
            }

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(0.5 * (data.PC3d[ii] + data.PC3d[ii + 1]),
                          false,
                          derivativeRequest,
                          extrap_mode,
                          3,
                          flatKnots,
                          1,
                          polesArray,
                          eval_result);

            if (eval_result[0] < data.PC2d[ii] || eval_result[0] > data.PC2d[ii + 1])
            {
                double ucons = 0.5 * (data.PC2d[ii] + data.PC2d[ii + 1]);
                double uc3d = 0.5 * (data.PC3d[ii] + data.PC3d[ii + 1]);

                Point3D pcons;
                data.COnS.D0(ucons, out pcons);
                projector.Perform(pcons, uc3d);
                if (projector.IsDone)
                {
                    curp = projector.Point().Parameter();
                    double dist_2 = projector.SquareDistance();
                    if (dist_2 > bestSqTol)
                    {
                        bestSqTol = dist_2;
                    }
                    projok = true;
                }
                else
                {
                    ProjectPointOnCurve(uc3d, pcons, data.myTol, 30, _c3d, out projok, out curp);
                }
                if (projok)
                {
                    if (curp > data.PC3d[ii] + _deltaMin && curp < data.PC3d[ii + 1] - _deltaMin)
                    {
                        data.NewPC3d[newcount] = curp;
                        data.NewPC2d[newcount] = ucons;
                        newcount++;
                    }
                }
            }
        } //     for (ii = 0; ii < count; ii++)
        data.NewPC3d[newcount] = data.PC3d[data.NbPnt];
        data.NewPC2d[newcount] = data.PC2d[data.NbPnt];

        if ((data.NbPnt != newcount) && newcount < MaxArraySize - 1)
        {
            // Distribution is changed.
            data.Swap(newcount);
            return true;
        }

        // Increase number of samples in two times.
        newcount = 0;
        for (int n = 0; n < data.NbPnt; n++)
        {
            data.NewPC3d[newcount] = data.PC3d[n];
            data.NewPC2d[newcount] = data.PC2d[n];
            newcount++;

            if (data.NbPnt - n + newcount == MaxArraySize)
            {
                continue;
            }

            double ucons = 0.5 * (data.PC2d[n] + data.PC2d[n + 1]);
            double uc3d = 0.5 * (data.PC3d[n] + data.PC3d[n + 1]);

            Point3D pcons;
            data.COnS.D0(ucons, out pcons);
            projector.Perform(pcons, uc3d);
            if (projector.IsDone)
            {
                curp = projector.Point().Parameter();
                double dist_2 = projector.SquareDistance();
                if (dist_2 > bestSqTol)
                {
                    bestSqTol = dist_2;
                }
                projok = true;
            }
            else
            {
                ProjectPointOnCurve(uc3d, pcons, data.myTol, 30, _c3d, out projok, out curp);
            }
            if (projok)
            {
                if (curp > data.PC3d[n] + _deltaMin && curp < data.PC3d[n + 1] - _deltaMin)
                {
                    data.NewPC3d[newcount] = curp;
                    data.NewPC2d[newcount] = ucons;
                    newcount++;
                }
            }
        }
        data.NewPC3d[newcount] = data.PC3d[data.NbPnt];
        data.NewPC2d[newcount] = data.PC2d[data.NbPnt];

        if (data.NbPnt != newcount)
        {
            // Distribution is changed.
            data.Swap(newcount);
            return true;
        }

        return false;
    }
}
