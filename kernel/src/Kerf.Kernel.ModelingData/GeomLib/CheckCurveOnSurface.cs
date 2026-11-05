// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckCurveOnSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_CheckCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// Computes the maximal distance between a 3D curve and a 2D curve on a surface (an Adaptor3d_CurveOnSurface)
/// over the 3D curve's range. A working object (decision 21); the adaptors given must outlive it.
/// </summary>
internal sealed class CheckCurveOnSurface
{
    private CurveAdaptorBase? _curve;
    private int _errorStatus;
    private double _maxDistance;
    private double _maxParameter;
    private double _tolRange;
    private bool _isParallel;

    /// <summary>An empty check: <see cref="Init(CurveAdaptorBase?, double)"/> sets the curve.</summary>
    public CheckCurveOnSurface()
    {
        _errorStatus = 0;
        _maxDistance = RealLast();
        _maxParameter = 0.0;
        _tolRange = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _isParallel = false;
    }

    /// <summary>A check of theCurve, whose range may exceed the curve on surface's by theTolRange.</summary>
    public CheckCurveOnSurface(CurveAdaptorBase? curve, double tolRange = 1.0e-7 * 0.01)
    {
        _curve = curve;
        _errorStatus = 0;
        _maxDistance = RealLast();
        _maxParameter = 0.0;
        _tolRange = tolRange;
        _isParallel = false;
    }

    /// <summary>Sets the data for the algorithm.</summary>
    public void Init(CurveAdaptorBase? curve, double tolRange = 1.0e-7 * 0.01)
    {
        _curve = curve;
        _errorStatus = 0;
        _maxDistance = RealLast();
        _maxParameter = 0.0;
        _tolRange = tolRange;
    }

    /// <summary>Resets all members to their default values.</summary>
    public void Init()
    {
        _curve = null;
        _errorStatus = 0;
        _maxDistance = RealLast();
        _maxParameter = 0.0;
        _tolRange = global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>
    /// Computes the maximal distance between the 3D curve and theCurveOnSurface, in parallel if
    /// <see cref="SetParallel"/> was given true.
    /// </summary>
    public void Perform(CurveOnSurface? curveOnSurface)
    {
        if (_curve is null || curveOnSurface is null)
        {
            _errorStatus = 1;
            return;
        }

        if ((_curve.FirstParameter() - curveOnSurface.FirstParameter() > _tolRange)
            || (_curve.LastParameter() - curveOnSurface.LastParameter() < -_tolRange))
        {
            _errorStatus = 2;
            return;
        }

        const double epsilonRange = 1.0e-3;

        int nbParticles = 3;

        // Polynomial function with degree n has not more than n-1 maxima and
        // minima (degree of 1st derivative is equal to n-1 => 1st derivative has
        // no greater than n-1 roots). Consequently, this function has
        // maximum n monotonicity intervals. That is a good idea to try to put
        // at least one particle in every monotonicity interval. Therefore,
        // number of particles should be equal to n.

        int nbSubIntervals = FillSubIntervals(_curve,
                                               curveOnSurface.GetCurve()!,
                                               _curve.FirstParameter(),
                                               _curve.LastParameter(),
                                               ref nbParticles);

        if (nbSubIntervals == 0)
        {
            _errorStatus = 3;
            return;
        }

        try
        {
            NCollection_Array1<double> intervals = new(1, nbSubIntervals + 1);
            FillSubIntervals(_curve,
                             curveOnSurface.GetCurve()!,
                             _curve.FirstParameter(),
                             _curve.LastParameter(),
                             ref nbParticles,
                             intervals);

            // OSD_ThreadPool's default pool launches one thread per logical processor.
            int nbThreads = _isParallel ? Min(intervals.Length(), OsdParallel.LogicalProcessorCount) : 1;
            NCollection_Array1<CurveAdaptorBase> curveArray = new(0, nbThreads - 1);
            NCollection_Array1<CurveAdaptorBase> curveOnSurfaceArray = new(0, nbThreads - 1);
            for (int i = 0; i < nbThreads; ++i)
            {
                curveArray.SetValue(i, nbThreads > 1 ? _curve.ShallowCopy() : _curve);
                curveOnSurfaceArray.SetValue(i,
                                              nbThreads > 1 ? curveOnSurface.ShallowCopy() : curveOnSurface);
            }
            GeomLib_CheckCurveOnSurface_Local comp = new(curveArray,
                                                          curveOnSurfaceArray,
                                                          intervals,
                                                          epsilonRange,
                                                          nbParticles);
            if (nbThreads > 1)
            {
                // OSD_ThreadPool::Launcher: each thread takes the next element of [Lower, Upper); every element is
                // computed on its own, so the distribution doesn't change the results.
                int lower = intervals.Lower(), upper = intervals.Upper();
                OsdParallel.For(0, nbThreads, threadIndex =>
                {
                    for (int i = lower + threadIndex; i < upper; i += nbThreads)
                    {
                        comp.Run(threadIndex, i);
                    }
                });
            }
            else
            {
                for (int i = intervals.Lower(); i < intervals.Upper(); ++i)
                {
                    comp.Run(0, i);
                }
            }
            comp.OptimalValues(out _maxDistance, out _maxParameter);

            _maxDistance = Math.Sqrt(Math.Abs(_maxDistance));
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            _errorStatus = 3;
        }
    }

    /// <summary>Sets whether <see cref="Perform"/> runs in parallel.</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;

    /// <summary>Whether the parallel flag is set.</summary>
    public bool IsParallel => _isParallel;

    /// <summary>Whether the maximal distance has been found.</summary>
    public bool IsDone => _errorStatus == 0;

    /// <summary>
    /// The error status: 0 OK; 1 null curve, surface or 2D curve; 2 invalid parametric range; 3 error in
    /// calculations.
    /// </summary>
    public int ErrorStatus() => _errorStatus;

    /// <summary>The maximal distance.</summary>
    public double MaxDistance() => _maxDistance;

    /// <summary>The parameter at which the distance is maximal.</summary>
    public double MaxParameter() => _maxParameter;

    private sealed class GeomLib_CheckCurveOnSurface_TargetFunc : MultipleVarFunctionWithHessian
    {
        private readonly CurveAdaptorBase _curve1;
        private readonly CurveAdaptorBase _curve2;
        private readonly double _first;
        private readonly double _last;

        public GeomLib_CheckCurveOnSurface_TargetFunc(CurveAdaptorBase c3D, CurveAdaptorBase curveOnSurface,
                                                      double first, double last)
        {
            _curve1 = c3D;
            _curve2 = curveOnSurface;
            _first = first;
            _last = last;
        }

        // returns the number of parameters of the function
        // (the function is one-dimension).
        public override int VariableCount => 1;

        // returns value of the function when parameters are equal to theX
        public override bool Value(math_Vector x, out double fVal) => Value(x[1], out fVal);

        // returns value of the one-dimension-function when parameter
        // is equal to theX (0 on failure, where OCCT leaves theFVal unchanged)
        public bool Value(double x, out double fVal)
        {
            fVal = 0.0;
            try
            {
                if (!CheckParameter(x))
                {
                    return false;
                }

                Point3D p1 = _curve1.Value(x), p2 = _curve2.Value(x);

                fVal = -1.0 * p1.SquareDistanceTo(p2);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                return false;
            }
            return true;
        }

        // see analogical method for abstract owner class math_MultipleVarFunction
        public override int GetStateNumber() => 0;

        // returns the gradient of the function when parameters are
        // equal to theX
        public override bool Gradient(math_Vector x, math_Vector grad)
        {
            if (!Derive(x[1], out double deriv1, false, out _))
            {
                return false;
            }
            grad[1] = deriv1;
            return true;
        }

        // returns 1st derivative of the one-dimension-function when
        // parameter is equal to theX, and the 2nd one if theWithDeriv2 (OCCT's optional pointer)
        private bool Derive(double x, out double deriv1, bool withDeriv2, out double deriv2)
        {
            deriv1 = 0.0;
            deriv2 = 0.0;
            try
            {
                if (!CheckParameter(x))
                {
                    return false;
                }
                //
                Point3D p1, p2;
                Vector3D dc1, dc2, dcc1 = new(), dcc2 = new();
                //
                if (!withDeriv2)
                {
                    _curve1.D1(x, out p1, out dc1);
                    _curve2.D1(x, out p2, out dc2);
                }
                else
                {
                    _curve1.D2(x, out p1, out dc1, out dcc1);
                    _curve2.D2(x, out p2, out dc2, out dcc2);
                }

                Vector3D vec1 = new(p1, p2), vec2 = dc2 - dc1;
                //
                deriv1 = -2.0 * vec1.Dot(vec2);

                if (withDeriv2)
                {
                    Vector3D vec3 = dcc2 - dcc1;
                    deriv2 = -2.0 * (vec2.SquareMagnitude + vec1.Dot(vec3));
                }
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                return false;
            }

            return true;
        }

        // returns value and gradient
        public override bool Values(math_Vector x, out double val, math_Vector grad)
        {
            if (!Value(x, out val))
            {
                return false;
            }
            //
            if (!Gradient(x, grad))
            {
                return false;
            }
            //
            return true;
        }

        // returns value, gradient and hessian
        public override bool Values(math_Vector x, out double val, math_Vector grad, Matrix hessian)
        {
            if (!Value(x, out val))
            {
                return false;
            }
            //
            if (!Derive(x[1], out double deriv1, true, out double deriv2))
            {
                return false;
            }
            grad[1] = deriv1;
            hessian[1, 1] = deriv2;
            //
            return true;
        }

        //
        public double FirstParameter() => _first;

        //
        public double LastParameter() => _last;

        private struct Coefficients
        {
            public Xyz Origin;
            public Xyz Cos;
            public Xyz Sin;
        }

        private static Coefficients GetCoefficients(CurveAdaptorBase curve, CurveKind type)
        {
            if (type == Circle)
            {
                Circle3D circle = curve.Circle();
                return new Coefficients
                {
                    Origin = circle.Location.XYZ(),
                    Cos = circle.XAxis.Direction.XYZ() * circle.Radius,
                    Sin = circle.YAxis.Direction.XYZ() * circle.Radius,
                };
            }

            Ellipse3D ellipse = curve.Ellipse();
            return new Coefficients
            {
                Origin = ellipse.Location.XYZ(),
                Cos = ellipse.XAxis.Direction.XYZ() * ellipse.MajorRadius,
                Sin = ellipse.YAxis.Direction.XYZ() * ellipse.MinorRadius,
            };
        }

        // Computes the exact maximum for elementary compositions supported by the adaptors.
        public bool ElementaryMaximum(ref double bestValue, ref double bestParameter)
        {
            CurveKind type1 = _curve1.GetType();
            CurveKind type2 = _curve2.GetType();
            if (type1 == Line && type2 == Line)
            {
                double firstValue, lastValue;
                if (!Value(_first, out firstValue) || !Value(_last, out lastValue))
                {
                    return false;
                }

                if (firstValue <= lastValue)
                {
                    bestValue = firstValue;
                    bestParameter = _first;
                }
                else
                {
                    bestValue = lastValue;
                    bestParameter = _last;
                }
                return true;
            }

            bool isTrigType1 = type1 == Circle || type1 == Ellipse;
            bool isTrigType2 = type2 == Circle || type2 == Ellipse;
            if (!isTrigType1 || !isTrigType2)
            {
                return false;
            }

            Coefficients coeffs1 = GetCoefficients(_curve1, type1);
            Coefficients coeffs2 = GetCoefficients(_curve2, type2);

            Xyz offset = coeffs1.Origin - coeffs2.Origin;
            Xyz cosCoeff = coeffs1.Cos - coeffs2.Cos;
            Xyz sinCoeff = coeffs1.Sin - coeffs2.Sin;

            // Coefficients of the derivative of the squared distance in trigonometric form.
            double cosCos = 2.0 * cosCoeff.Dot(sinCoeff);
            double cosSin = 0.5 * (sinCoeff.SquareModulus() - cosCoeff.SquareModulus());
            double cos = offset.Dot(sinCoeff);
            double sin = -offset.Dot(cosCoeff);
            double constant = -cosCoeff.Dot(sinCoeff);

            // Exact zero is required here: a tolerance could classify a varying distance as constant.
            if (cosCos == 0.0 && cosSin == 0.0 && cos == 0.0 && sin == 0.0 && constant == 0.0)
            {
                bestParameter = _first;
                bool isOk = Value(_first, out double value);
                if (isOk)
                {
                    bestValue = value;
                }
                return isOk;
            }

            double period = 2.0 * M_PI;
            double first = global::Kerf.Engine.Geometry.ElCLib.InPeriod(_first, 0.0, period);
            double shift = first - _first;
            double last = first + Min(_last - _first, period);
            // std::max of an initializer list
            double scale = Math.Abs(cosCos);
            scale = Max(scale, Math.Abs(cosSin));
            scale = Max(scale, Math.Abs(cos));
            scale = Max(scale, Math.Abs(sin));
            scale = Max(scale, Math.Abs(constant));
            TrigonometricFunctionRoots roots = new(cosCos / scale,
                                                         cosSin / scale,
                                                         cos / scale,
                                                         sin / scale,
                                                         constant / scale,
                                                         first,
                                                         last);
            if (!roots.IsDone || roots.InfiniteRoots()
                || (_last - _first >= period && roots.SolutionCount == 0))
            {
                return false;
            }

            bestValue = RealLast();
            bestParameter = _first;

            if (!UpdateBest(_first, ref bestValue, ref bestParameter)
                || !UpdateBest(_last, ref bestValue, ref bestParameter))
            {
                return false;
            }
            for (int index = 1; index <= roots.SolutionCount; ++index)
            {
                if (!UpdateBest(roots.Value(index) - shift, ref bestValue, ref bestParameter))
                {
                    return false;
                }
            }
            return true;
        }

        // OCCT's lambda updateBest
        private bool UpdateBest(double parameter, ref double bestValue, ref double bestParameter)
        {
            if (!Value(parameter, out double value))
            {
                return false;
            }
            if (value < bestValue)
            {
                bestValue = value;
                bestParameter = parameter;
            }
            return true;
        }

        // checks if the function can be computed when its parameter is
        // equal to theParam
        private bool CheckParameter(double param)
        {
            return ((_first <= param) && (param <= _last));
        }
    }

    private sealed class GeomLib_CheckCurveOnSurface_Local
    {
        private readonly NCollection_Array1<CurveAdaptorBase> _curveArray;
        private readonly NCollection_Array1<CurveAdaptorBase> _curveOnSurfaceArray;

        private readonly NCollection_Array1<double> _subIntervals;
        private readonly double _epsilonRange;
        private readonly int _nbParticles;
        private readonly NCollection_Array1<double> _arrOfDist;
        private readonly NCollection_Array1<double> _arrOfParam;

        public GeomLib_CheckCurveOnSurface_Local(NCollection_Array1<CurveAdaptorBase> curveArray,
                                                 NCollection_Array1<CurveAdaptorBase> curveOnSurfaceArray,
                                                 NCollection_Array1<double> intervalsArr,
                                                 double epsilonRange,
                                                 int nbParticles)
        {
            _curveArray = curveArray;
            _curveOnSurfaceArray = curveOnSurfaceArray;
            _subIntervals = intervalsArr;
            _epsilonRange = epsilonRange;
            _nbParticles = nbParticles;
            _arrOfDist = new(intervalsArr.Lower(), intervalsArr.Upper() - 1);
            _arrOfParam = new(intervalsArr.Lower(), intervalsArr.Upper() - 1);
        }

        // OCCT's operator()(theThreadIndex, theElemIndex)
        public void Run(int threadIndex, int elemIndex)
        {
            // For every sub-interval (which is set by mySubIntervals array) this method
            // computes optimal value of GeomLib_CheckCurveOnSurface_TargetFunc function.
            // This optimal value will be put in corresponding (depending on theIndex - the
            // identificator of the current interval in mySubIntervals array) cell of
            // myArrOfDist and myArrOfParam arrays.
            GeomLib_CheckCurveOnSurface_TargetFunc func = new(_curveArray.Value(threadIndex),
                                                               _curveOnSurfaceArray.Value(threadIndex),
                                                               _subIntervals.Value(elemIndex),
                                                               _subIntervals.Value(elemIndex + 1));

            double minDist = RealLast(), par = 0.0;
            if (!MinComputing(func, _epsilonRange, _nbParticles, ref minDist, ref par))
            {
                _arrOfDist[elemIndex] = RealLast();
                _arrOfParam[elemIndex] = func.FirstParameter();
                return;
            }

            _arrOfDist[elemIndex] = minDist;
            _arrOfParam[elemIndex] = par;
        }

        // Returns optimal value (inverse of square of maximal distance)
        public void OptimalValues(out double minimalValue, out double parameter)
        {
            // This method looks for the minimal value of myArrOfDist.

            int startInd = _arrOfDist.Lower();
            minimalValue = _arrOfDist[startInd];
            parameter = _arrOfParam[startInd];
            for (int i = startInd + 1; i <= _arrOfDist.Upper(); i++)
            {
                if (_arrOfDist[i] < minimalValue)
                {
                    minimalValue = _arrOfDist[i];
                    parameter = _arrOfParam[i];
                }
            }
        }
    }

    // theSubIntervals is OCCT's optional pointer (null: only count)
    private static int FillSubIntervals(CurveAdaptorBase curve3d,
                                        Curve2DAdaptorBase curve2d,
                                        double first,
                                        double last,
                                        ref int nbParticles,
                                        NCollection_Array1<double>? subIntervals = null)
    {
        const int maxKnots = 101;
        NCollection_Array1<double> arrTemp = new(1, 2);
        arrTemp[1] = first;
        arrTemp[2] = last;

        nbParticles = 3;
        BSplineCurve2D? bs2DCurv = null;
        BSplineCurve3D? bs3DCurv = null;
        bool isTrimmed3D = false, isTrimmed2D = false;

        //
        if (curve3d.GetType() == BSplineCurve)
        {
            bs3DCurv = curve3d.BSpline();
        }
        if (curve2d.GetType() == BSplineCurve)
        {
            bs2DCurv = curve2d.BSpline();
        }

        NCollection_HArray1<double> arrKnots3D, arrKnots2D;

        if (bs3DCurv is not null)
        {
            if (bs3DCurv.KnotCount <= maxKnots)
            {
                arrKnots3D = new NCollection_HArray1<double>(bs3DCurv.InternalKnots());
            }
            else
            {
                int knotCount;
                if (isTrimmed3D)
                {
                    int i;
                    knotCount = 0;
                    NCollection_Array1<double> knots = bs3DCurv.InternalKnots();
                    for (i = bs3DCurv.FirstUKnotIndex(); i <= bs3DCurv.LastUKnotIndex(); ++i)
                    {
                        if (knots[i] > first && knots[i] < last)
                        {
                            ++knotCount;
                        }
                    }
                    knotCount += 2;
                }
                else
                {
                    knotCount = bs3DCurv.LastUKnotIndex() - bs3DCurv.FirstUKnotIndex() + 1;
                }
                if (knotCount <= maxKnots)
                {
                    arrKnots3D = new NCollection_HArray1<double>(bs3DCurv.InternalKnots());
                }
                else
                {
                    arrKnots3D = new NCollection_HArray1<double>(1, maxKnots);
                    arrKnots3D.SetValue(1, first);
                    arrKnots3D.SetValue(maxKnots, last);
                    int i;
                    double dt = (last - first) / (maxKnots - 1);
                    double t = first + dt;
                    for (i = 2; i < maxKnots; ++i, t += dt)
                    {
                        arrKnots3D.SetValue(i, t);
                    }
                }
            }
        }
        else
        {
            arrKnots3D = new NCollection_HArray1<double>(arrTemp);
        }
        if (bs2DCurv is not null)
        {
            if (bs2DCurv.KnotCount <= maxKnots)
            {
                arrKnots2D = new NCollection_HArray1<double>(bs2DCurv.InternalKnots());
            }
            else
            {
                int knotCount;
                if (isTrimmed2D)
                {
                    int i;
                    knotCount = 0;
                    NCollection_Array1<double> knots = bs2DCurv.InternalKnots();
                    for (i = bs2DCurv.FirstUKnotIndex(); i <= bs2DCurv.LastUKnotIndex(); ++i)
                    {
                        if (knots[i] > first && knots[i] < last)
                        {
                            ++knotCount;
                        }
                    }
                    knotCount += 2;
                }
                else
                {
                    knotCount = bs2DCurv.LastUKnotIndex() - bs2DCurv.FirstUKnotIndex() + 1;
                }
                if (knotCount <= maxKnots)
                {
                    arrKnots2D = new NCollection_HArray1<double>(bs2DCurv.InternalKnots());
                }
                else
                {
                    arrKnots2D = new NCollection_HArray1<double>(1, maxKnots);
                    arrKnots2D.SetValue(1, first);
                    arrKnots2D.SetValue(maxKnots, last);
                    int i;
                    double dt = (last - first) / (maxKnots - 1);
                    double t = first + dt;
                    for (i = 2; i < maxKnots; ++i, t += dt)
                    {
                        arrKnots2D.SetValue(i, t);
                    }
                }
            }
        }
        else
        {
            arrKnots2D = new NCollection_HArray1<double>(arrTemp);
        }

        int nbSubIntervals = 1;

        try
        {
            int indMax3D = arrKnots3D.Upper(), indMax2D = arrKnots2D.Upper();

            int index3D = arrKnots3D.Lower(), index2D = arrKnots2D.Lower();

            if (subIntervals is not null)
            {
                subIntervals[nbSubIntervals] = first;
            }

            while ((index3D <= indMax3D) && (index2D <= indMax2D))
            {
                double val3D = arrKnots3D.Value(index3D), val2D = arrKnots2D.Value(index2D);
                double delta = val3D - val2D;

                if (delta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                { // aVal3D <= aVal2D
                    if ((val3D > first) && (val3D < last))
                    {
                        nbSubIntervals++;

                        if (subIntervals is not null)
                        {
                            subIntervals[nbSubIntervals] = val3D;
                        }
                    }

                    index3D++;

                    if (-delta < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    { // aVal3D == aVal2D
                        index2D++;
                    }
                }
                else
                { // aVal2D < aVal3D
                    if ((val2D > first) && (val2D < last))
                    {
                        nbSubIntervals++;

                        if (subIntervals is not null)
                        {
                            subIntervals[nbSubIntervals] = val2D;
                        }
                    }

                    index2D++;
                }
            }

            if (subIntervals is not null)
            {
                subIntervals[nbSubIntervals + 1] = last;
            }

            if (bs3DCurv is not null)
            {
                nbParticles = Max(nbParticles, bs3DCurv.Degree);
            }

            if (bs2DCurv is not null)
            {
                nbParticles = Max(nbParticles, bs2DCurv.Degree);
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            nbSubIntervals = 0;
        }

        return nbSubIntervals;
    }

    private static bool PSO_Perform(GeomLib_CheckCurveOnSurface_TargetFunc function,
                                    math_Vector parInf,
                                    math_Vector parSup,
                                    double epsilon,
                                    int nbParticles,
                                    out double bestValue,
                                    math_Vector outputParam)
    {
        bestValue = 0.0;
        double deltaParam = parSup[1] - parInf[1];
        if (deltaParam < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        math_Vector stepPar = new(1, 1);
        stepPar[1] = epsilon * deltaParam;

        PsoParticlesPool particles = new(nbParticles, 1);

        // They are used for finding a position of theNbParticles worst places
        int nbControlPoints = 3 * nbParticles;

        double step = deltaParam / (nbControlPoints - 1);
        int count = 1;
        for (double prm = parInf[1]; count <= nbControlPoints;
             count++, prm = (count == nbControlPoints) ? parSup[1] : prm + step)
        {
            double val;
            if (!function.Value(prm, out val))
            {
                continue;
            }

            Particle particle = particles.GetWorstParticle();

            if (val > particle.BestDistance)
            {
                continue;
            }

            particle.Position[particle.Position.Lower()] = prm;
            particle.BestPosition[particle.BestPosition.Lower()] = prm;
            particle.Distance = val;
            particle.BestDistance = val;
        }

        Pso pso = new(function, parInf, parSup, stepPar);
        pso.Perform(particles, nbParticles, out bestValue, outputParam);

        return true;
    }

    private static bool MinComputing(GeomLib_CheckCurveOnSurface_TargetFunc function,
                                     double epsilon, // 1.0e-3
                                     int nbParticles,
                                     ref double bestValue,
                                     ref double bestParameter)
    {
        try
        {
            //
            math_Vector parInf = new(1, 1), parSup = new(1, 1), outputParam = new(1, 1);
            parInf[1] = function.FirstParameter();
            parSup[1] = function.LastParameter();
            bestParameter = parInf[1];
            bestValue = RealLast();

            if (function.ElementaryMaximum(ref bestValue, ref bestParameter))
            {
                return true;
            }

            if (!PSO_Perform(function,
                             parInf,
                             parSup,
                             epsilon,
                             nbParticles,
                             out bestValue,
                             outputParam))
            {
                return false;
            }

            bestParameter = outputParam[1];

            // Here, anOutputParam contains parameter, which is near to optimal.
            // It needs to be more precise. Precision is made by math_NewtonMinimum.
            NewtonMinimum minSol = new(function);
            minSol.Perform(function, outputParam);

            if (minSol.IsDone && (minSol.GetStatus() == OK))
            { // math_NewtonMinimum has precised the value. We take it.
                minSol.Location(outputParam);
                bestParameter = outputParam[1];
                bestValue = minSol.Minimum();
            }
            else
            { // Use math_PSO again but on smaller range.
                double step = epsilon * (parSup[1] - parInf[1]);
                parInf[1] = bestParameter - 0.5 * step;
                parSup[1] = bestParameter + 0.5 * step;

                double value;
                if (PSO_Perform(function,
                                parInf,
                                parSup,
                                epsilon,
                                nbParticles,
                                out value,
                                outputParam))
                {
                    if (value < bestValue)
                    {
                        bestValue = value;
                        bestParameter = outputParam[1];
                    }
                }
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return false;
        }

        return true;
    }
}
