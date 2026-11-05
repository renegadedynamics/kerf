// Port of: src/FoundationClasses/TKMath/math/math_GlobOptMin.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GlobOptMin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>
/// Evtushenko's algorithm of global optimization on a non-uniform mesh (Yu. Evtushenko, Numerical methods for
/// finding global extreme (case of a non-uniform mesh), U.S.S.R. Comput. Maths. Math. Phys., Vol. 11, N 6, pp. 38-54).
/// </summary>
/// <remarks>
/// The search space is a box in R^n; by default all the minima in it are found (maxima are not supported). The box can
/// be split by discontinuity criteria (<see cref="SetGlobalParams"/>, <see cref="SetLocalParams"/>). The continuity of
/// the local boxes (<see cref="SetContinuity"/>) selects the local method: Newton (C2, when the function has a
/// hessian), BFGS (C1, with a gradient) or Powell. The Lipschitz constant can be locked
/// (<see cref="SetLipConstState"/>); a single solution can be searched (<see cref="Perform"/>), which works well
/// with a known minimal value (<see cref="SetFunctionalMinimalValue"/>).
/// </remarks>
internal class GlobOptMin
{
    // Class for duplicate fast search. For internal usage only.
    private sealed class NCollection_CellFilter_Inspector : ICellFilterInspector<math_Vector, math_Vector>
    {
        public NCollection_CellFilter_Inspector(int dim, double tol)
        {
            _current = new math_Vector(1, dim);
            _tol = tol * tol;
            _isFind = false;
            _dimension = dim;
        }

        // Access to coordinate.
        public static double Coord(int i, math_Vector pnt) => pnt[i + 1];

        // Auxiliary method to shift point by each coordinate on given value;
        // useful for preparing a points range for Inspect with tolerance.
        public void Shift(math_Vector pnt, NCollection_Array1<double> tol, math_Vector lowPnt, math_Vector uppPnt)
        {
            for (int idx = 1; idx <= _dimension; idx++)
            {
                lowPnt[idx] = pnt[idx] - tol[idx - 1];
                uppPnt[idx] = pnt[idx] + tol[idx - 1];
            }
        }

        public void ClearFind() => _isFind = false;

        public bool IsFind() => _isFind;

        // Set current point to search for coincidence
        public void SetCurrent(math_Vector curPnt) => _current.Initialized(curPnt);

        // Implementation of inspection method
        public CellFilterAction Inspect(math_Vector theObject)
        {
            double sqDist = (_current - theObject).Norm2();

            if (sqDist < _tol)
            {
                _isFind = true;
            }

            return CellFilterAction.CellFilter_Keep;
        }

        private double _tol;
        private math_Vector _current;
        private bool _isFind;
        private int _dimension;
    }

    // Input.
    private MultipleVarFunction _func;
    private int _n;
    private math_Vector _a; // Left border on current C2 interval.
    private math_Vector _b; // Right border on current C2 interval.
    private math_Vector _globA; // Global left border.
    private math_Vector _globB; // Global right border.
    private double _tol; // Discretization tolerance, default 1.0e-2.
    private double _sameTol; // points with ||p1 - p2|| < mySameTol is equal,
                              // function values |val1 - val2| * 0.01 < mySameTol is equal,
                              // default value is 1.0e-7.
    private double _c; // Lipchitz constant, default 9
    private double _initC; // Lipchitz constant initial value.
    private bool _isFindSingleSolution; // Default value is false.
    private double _functionalMinimalValue; // Default value is -Precision::Infinite
    private bool _isConstLocked; // Is constant locked for modifications.

    // Output.
    private bool _done;
    private NCollection_Sequence<double> _y = new(); // Current solutions.
    private int _solCount; // Count of solutions.

    // Algorithm data.
    private double _z;
    private double _e1; // Border coefficient.
    private double _e2; // Minimum step size.
    private double _e3; // Local extrema starting parameter.

    private math_Vector _x; // Current modified solution.
    private math_Vector _tmp; // Current modified solution.
    private math_Vector _v; // Steps array.
    private math_Vector _maxV; // Max Steps array.
    private double _lastStep; // Last step.

    private NCollection_Array1<double> _cellSize;
    private int _minCellFilterSol;
    private bool _isFirstCellFilterInvoke;
    private CellFilter<NCollection_CellFilter_Inspector, math_Vector, math_Vector> _filter;

    // Continuity of local borders.
    private int _cont;

    private double _f; // Current value of Global optimum.

    private static double DistanceToBorder(math_Vector x, math_Vector min, math_Vector max)
    {
        double dist = RealLast();

        for (int idx = min.Lower(); idx <= min.Upper(); ++idx)
        {
            double dist1 = Math.Abs(x[idx] - min[idx]);
            double dist2 = Math.Abs(x[idx] - max[idx]);
            dist = Min(dist, Min(dist1, dist2));
        }

        return dist;
    }

    /// <summary>
    /// Prepares the algorithm for the function <paramref name="func"/> on the box [theA, theB], with the Lipschitz
    /// constant theC, the parameter space discretization tolerance and the functional value indifference tolerance.
    /// <see cref="Perform"/> does the job.
    /// </summary>
    public GlobOptMin(MultipleVarFunction func, math_Vector a, math_Vector b, double c = 9,
                           double discretizationTol = 1.0e-2, double sameTol = 1.0e-7)
    {
        _n = func.VariableCount;
        _a = new math_Vector(1, _n);
        _b = new math_Vector(1, _n);
        _globA = new math_Vector(1, _n);
        _globB = new math_Vector(1, _n);
        _isConstLocked = false;
        _x = new math_Vector(1, _n);
        _tmp = new math_Vector(1, _n);
        _v = new math_Vector(1, _n);
        _maxV = new math_Vector(1, _n);
        _cellSize = new NCollection_Array1<double>(0, _n - 1);
        _filter = new CellFilter<NCollection_CellFilter_Inspector, math_Vector, math_Vector>(func.VariableCount);
        _cont = 2;
        _f = global::Kerf.Engine.Foundation.Precision.Infinite();

        int i;

        _func = func;
        _c = c;
        _initC = c;
        _isFindSingleSolution = false;
        _functionalMinimalValue = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _z = -1;
        _solCount = 0;

        for (i = 1; i <= _n; i++)
        {
            _globA[i] = a[i];
            _globB[i] = b[i];

            _a[i] = a[i];
            _b[i] = b[i];
        }

        for (i = 1; i <= _n; i++)
        {
            _maxV[i] = (_b[i] - _a[i]) / 3.0;
        }

        _tol = discretizationTol;
        _sameTol = sameTol;

        const int maxSquareSearchSol = 200;
        int solNb = (int)Math.Pow(3.0, (double)_n);
        _minCellFilterSol = Max(2 * solNb, maxSquareSearchSol);
        InitCellSize();
        ComputeInitSol();

        _done = false;
    }

    /// <summary>Sets the parameters again, without allocating memory.</summary>
    public void SetGlobalParams(MultipleVarFunction func, math_Vector a, math_Vector b, double c = 9,
                                double discretizationTol = 1.0e-2, double sameTol = 1.0e-7)
    {
        int i;

        _func = func;
        _c = c;
        _initC = c;
        _z = -1;
        _solCount = 0;

        for (i = 1; i <= _n; i++)
        {
            _globA[i] = a[i];
            _globB[i] = b[i];

            _a[i] = a[i];
            _b[i] = b[i];
        }

        for (i = 1; i <= _n; i++)
        {
            _maxV[i] = (_b[i] - _a[i]) / 3.0;
        }

        _tol = discretizationTol;
        _sameTol = sameTol;

        InitCellSize();
        ComputeInitSol();

        _done = false;
    }

    /// <summary>Reduces the search box to [theLocalA, theLocalB]; <see cref="Perform"/> uses it.</summary>
    public void SetLocalParams(math_Vector localA, math_Vector localB)
    {
        int i;

        _z = -1;
        for (i = 1; i <= _n; i++)
        {
            _a[i] = localA[i];
            _b[i] = localB[i];
        }

        for (i = 1; i <= _n; i++)
        {
            _maxV[i] = (_b[i] - _a[i]) / 3.0;
        }

        _done = false;
    }

    /// <summary>Sets the parameter space discretization tolerance and the functional value indifference tolerance.</summary>
    public void SetTol(double discretizationTol, double sameTol)
    {
        _tol = discretizationTol;
        _sameTol = sameTol;
    }

    /// <summary>The parameter space discretization tolerance and the functional value indifference tolerance.</summary>
    public void GetTol(out double discretizationTol, out double sameTol)
    {
        discretizationTol = _tol;
        sameTol = _sameTol;
    }

    /// <summary>Searches the minima; a single one if <paramref name="isFindSingleSolution"/>.</summary>
    public void Perform(bool isFindSingleSolution = false)
    {
        // In this algo indexes started from 1, not from 0.
        _done = false;

        // Compute parameters range
        double minLength = RealLast();
        double maxLength = RealFirst();
        for (int i = 1; i <= _n; i++)
        {
            double currentLength = _b[i] - _a[i];
            if (currentLength < minLength)
            {
                minLength = currentLength;
            }

            if (currentLength > maxLength)
            {
                maxLength = currentLength;
            }

            _v[i] = 0.0;
        }

        if (minLength < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return;
        }

        if (!_isConstLocked)
        {
            // Compute initial value for myC.
            ComputeInitialValues();
        }

        _e1 = minLength * _tol;
        _e2 = maxLength * _tol;

        _isFindSingleSolution = isFindSingleSolution;
        if (isFindSingleSolution)
        {
            // Run local optimization if current value better than optimal.
            _e3 = 0.0;
        }
        else
        {
            if (_c > 1.0)
            {
                _e3 = -maxLength * _tol / 4.0;
            }
            else
            {
                _e3 = -maxLength * _tol * _c / 4.0;
            }
        }

        // Search single solution and current solution in its neighborhood.
        if (CheckFunctionalStopCriteria())
        {
            _done = true;
            return;
        }

        _lastStep = 0.0;
        _isFirstCellFilterInvoke = true;
        ComputeGlobalExtremum(_n);

        _done = true;
    }

    // theVal is ref: OCCT leaves it unchanged when no extremum is found.
    private bool ComputeLocalExtremum(math_Vector pnt, ref double val, math_Vector outPnt)
    {
        int i;

        // OCCT: ComputeInitSol passes the same vector as thePnt and theOutPnt, so a local method that fails after
        // writing theOutPnt changes the starting point of the next method.

        // Newton method
        if (_cont >= 2 && _func is MultipleVarFunctionWithHessian)
        {
            MultipleVarFunctionWithHessian tmp = (MultipleVarFunctionWithHessian)_func;
            NewtonMinimum newtonMinimum = new(tmp);
            newtonMinimum.SetBoundary(_globA, _globB);
            newtonMinimum.Perform(tmp, pnt);

            if (newtonMinimum.IsDone)
            {
                newtonMinimum.Location(outPnt);
                val = newtonMinimum.Minimum();

                if (IsInside(outPnt))
                {
                    return true;
                }
            }
        }

        // BFGS method used.
        if (_cont >= 1 && _func is MultipleVarFunctionWithGradient)
        {
            MultipleVarFunctionWithGradient tmp = (MultipleVarFunctionWithGradient)_func;
            Bfgs bfgs = new(tmp.VariableCount);
            bfgs.SetBoundary(_globA, _globB);
            bfgs.Perform(tmp, pnt);

            if (bfgs.IsDone)
            {
                bfgs.Location(outPnt);
                val = bfgs.Minimum();

                if (IsInside(outPnt))
                {
                    return true;
                }
            }
        }

        // Powell method used.
        if (_func is not null)
        {
            Matrix m = new(1, _n, 1, _n, 0.0);
            for (i = 1; i <= _n; i++)
            {
                m[i, i] = 1.0;
            }

            Powell powell = new(_func, 1e-10);
            powell.Perform(_func, pnt, m);

            if (powell.IsDone)
            {
                powell.Location(outPnt);
                val = powell.Minimum();

                if (IsInside(outPnt))
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Computes the starting values: myF (initial best value), myY (initial best point) and myC (approximation of
    // the Lipschitz constant), to improve convergence speed.
    private void ComputeInitialValues()
    {
        const double minLC = 0.01;
        const double maxLC = 1000.0;
        const double minEps = 0.1;
        const double maxEps = 100.0;
        int i;
        math_Vector currPnt = new(1, _n);
        math_Vector bestPnt = new(1, _n);
        math_Vector paramStep = new(1, _n);
        double currVal = RealLast();

        // Lipchitz const approximation.
        double lipConst = 0.0, prevValDiag, prevValProj;
        int pntNb = 13;
        _func.Value(_a, out prevValDiag);
        prevValProj = prevValDiag;
        double step = (_b - _a).Norm() / pntNb;
        paramStep.Initialized((_b - _a) / pntNb);
        for (i = 1; i <= pntNb; i++)
        {
            currPnt.Initialized(_a + paramStep * i);

            // Walk over diagonal.
            _func.Value(currPnt, out currVal);
            lipConst = Max(Math.Abs(currVal - prevValDiag), lipConst);
            prevValDiag = currVal;

            // Walk over diag in projected space aPnt(1) = myA(1) = const.
            currPnt[1] = _a[1];
            _func.Value(currPnt, out currVal);
            lipConst = Max(Math.Abs(currVal - prevValProj), lipConst);
            prevValProj = currVal;
        }

        _c = _initC;
        lipConst *= Math.Sqrt(_n) / step;
        if (lipConst < _c * minEps)
        {
            _c = Max(lipConst * minEps, minLC);
        }
        else if (lipConst > _c * maxEps)
        {
            _c = Min(_c * maxEps, maxLC);
        }
    }

    private void ComputeGlobalExtremum(int j)
    {
        int i;
        double d = RealLast(), prevVal; // Functional in original and moved points.
        double val = RealLast(); // Local extrema computed in moved point.
        double stepBestValue = RealLast();
        math_Vector stepBestPoint = new(1, _n);
        bool isInside = false, isReached = false;

        double r1, r2, r;

        for (_x[j] = _a[j] + _e1; !isReached; _x[j] += _v[j])
        {
            if (_x[j] > _b[j])
            {
                _x[j] = _b[j];
                isReached = true;
            }

            if (CheckFunctionalStopCriteria())
            {
                return; // Best possible value is obtained.
            }

            if (j == 1)
            {
                isInside = false;
                prevVal = d;
                _func.Value(_x, out d);
                r1 = (d + _z * _c * _lastStep - _f) * _z; // Evtushenko estimation.
                r2 = ((d + prevVal - _c * _lastStep) * 0.5 - _f) * _z; // Shubert / Piyavsky estimation.
                r = Min(r1, r2);
                if (r > _e3)
                {
                    double saveParam = _x[1];

                    // Piyavsky midpoint estimation.
                    double param = (2 * _x[1] - _v[1]) * 0.5 + (prevVal - d) * 0.5 / _c;
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(prevVal))
                    {
                        param = _x[1] - _v[1] * 0.5; // Protection from upper dimension step.
                    }

                    _x[1] = param;
                    double aVal = 0;
                    _func.Value(_x, out aVal);
                    _x[1] = saveParam;

                    if ((aVal < d && aVal < prevVal)
                        || DistanceToBorder(_x, _a, _b) < _e1) // Condition optimization case near the border.
                    {
                        isInside = ComputeLocalExtremum(_x, ref val, _tmp);
                    }
                }

                stepBestValue = (isInside && (val < d)) ? val : d;
                stepBestPoint.Initialized((isInside && (val < d)) ? _tmp : _x);

                // Check point and value on the current step to be optimal.
                CheckAddCandidate(stepBestPoint, stepBestValue);

                if (CheckFunctionalStopCriteria())
                {
                    return; // Best possible value is obtained.
                }

                _v[1] = Min(_e2 + Math.Abs(_f - d) / _c, _maxV[1]);
                _lastStep = _v[1];
            }
            else
            {
                _v[j] = RealLast() / 2.0;
                ComputeGlobalExtremum(j - 1);

                // Nullify steps on lower dimensions.
                for (i = 1; i < j; i++)
                {
                    _v[i] = 0.0;
                }
            }

            if (j < _n)
            {
                double upperDimStep = Max(_v[j], _e2);
                if (_v[j + 1] > upperDimStep)
                {
                    if (upperDimStep > _maxV[j + 1])
                    { // Case of too big step.
                        _v[j + 1] = _maxV[j + 1];
                    }
                    else
                    {
                        _v[j + 1] = upperDimStep;
                    }
                }
            }
        }
    }

    // Checks that myGlobA <= thePnt <= myGlobB.
    private bool IsInside(math_Vector pnt)
    {
        int i;

        for (i = 1; i <= _n; i++)
        {
            if (pnt[i] < _globA[i] || pnt[i] > _globB[i])
            {
                return false;
            }
        }

        return true;
    }

    // Checks presence of thePnt in the solutions.
    private bool IsStored(math_Vector pnt)
    {
        int i, j;
        bool isSame = true;
        math_Vector tol = new(1, _n);
        tol.Initialized((_b - _a) * _sameTol);

        // C1 * n^2 = C2 * 3^dim * n
        if (_solCount < _minCellFilterSol)
        {
            for (i = 0; i < _solCount; i++)
            {
                isSame = true;
                for (j = 1; j <= _n; j++)
                {
                    if ((Math.Abs(pnt[j] - _y[i * _n + j])) > tol[j])
                    {
                        isSame = false;
                        break;
                    }
                }

                if (isSame)
                {
                    return true;
                }
            }
        }
        else
        {
            NCollection_CellFilter_Inspector inspector = new(_n, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (_isFirstCellFilterInvoke)
            {
                _filter.Reset(_cellSize);

                // Copy initial data into cell filter.
                for (int solIdx = 0; solIdx < _solCount; solIdx++)
                {
                    math_Vector vec = new(1, _n);
                    for (int solDim = 1; solDim <= _n; solDim++)
                    {
                        vec[solDim] = _y[solIdx * _n + solDim];
                    }

                    _filter.Add(vec, vec);
                }
            }

            _isFirstCellFilterInvoke = false;

            math_Vector low = new(1, _n), up = new(1, _n);
            inspector.Shift(pnt, _cellSize, low, up);

            inspector.ClearFind();
            inspector.SetCurrent(pnt);
            _filter.Inspect(low, up, ref inspector);
            if (!inspector.IsFind())
            {
                // Point is out of close cells, add new one.
                // The C++ filter stores a copy of the target; callers reuse their vector, so copy it here.
                math_Vector stored = new(pnt);
                _filter.Add(stored, stored);
            }
        }

        return false;
    }

    /// <summary>Copies the solution number <paramref name="index"/>, 1 &lt;= theIndex &lt;= NbExtrema, into <paramref name="sol"/>.</summary>
    public void Points(int index, math_Vector sol)
    {
        int j;

        for (j = 1; j <= _n; j++)
        {
            sol[j] = _y[(index - 1) * _n + j];
        }
    }

    private void InitCellSize()
    {
        for (int idx = 1; idx <= _n; idx++)
        {
            _cellSize[idx - 1] = (_globB[idx] - _globA[idx]) * global::Kerf.Engine.Foundation.Precision.PConfusion() / (2.0 * M_SQRT2);
        }
    }

    // Checks the possibility to stop computations: a single solution is searched and the best possible one is found.
    private bool CheckFunctionalStopCriteria()
    {
        // Search single solution and current solution in its neighborhood.
        return _isFindSingleSolution && Math.Abs(_f - _functionalMinimalValue) < _sameTol * 0.01;
    }

    private void ComputeInitSol()
    {
        double val;
        math_Vector pnt = new(1, _n);

        // Check functional value in midpoint. It is necessary since local optimization
        // algorithm may fail and return nothing. This is a protection from uninitialized
        // variables.
        pnt.Initialized((_globA + _globB) * 0.5);
        _func.Value(pnt, out val);
        CheckAddCandidate(pnt, val);

        // Run local optimization from lower corner, midpoint, and upper corner.
        for (int i = 1; i <= 3; i++)
        {
            pnt.Initialized(_a + (_b - _a) * (i - 1) / 2.0);

            if (ComputeLocalExtremum(pnt, ref val, pnt))
            {
                CheckAddCandidate(pnt, val);
            }
        }
    }

    // Checks and adds a candidate point into the solutions.
    private void CheckAddCandidate(math_Vector pnt, double value)
    {
        if (Math.Abs(value - _f) < _sameTol * 0.01 && // Value in point is close to optimal value.
            !_isFindSingleSolution) // Several optimal solutions are allowed.
        {
            if (!IsStored(pnt))
            {
                if ((value - _f) * _z > 0.0)
                {
                    _f = value;
                }

                for (int j = 1; j <= _n; j++)
                {
                    _y.Append(pnt[j]);
                }

                _solCount++;
            }
        }

        // New best solution:
        // new point is out of (mySameTol * 0.01) surrounding or
        // new point is better than old and single point search.
        double delta = (value - _f) * _z;
        if (delta > _sameTol * 0.01 || (delta > 0.0 && _isFindSingleSolution))
        {
            _f = value;
            _y.Clear();
            for (int j = 1; j <= _n; j++)
            {
                _y.Append(pnt[j]);
            }

            _solCount = 1;

            _isFirstCellFilterInvoke = true;
        }
    }

    /// <summary>Sets the continuity of the local borders splits (0 ~ C0, 1 ~ C1, 2 ~ C2).</summary>
    public void SetContinuity(int cont) => _cont = cont;

    /// <summary>The continuity of the local borders splits (0 ~ C0, 1 ~ C1, 2 ~ C2).</summary>
    public int GetContinuity() => _cont;

    /// <summary>Sets the functional minimal value.</summary>
    public void SetFunctionalMinimalValue(double minimalValue) => _functionalMinimalValue = minimalValue;

    public double GetFunctionalMinimalValue() => _functionalMinimalValue;

    /// <summary>Locks (true) or unlocks the Lipschitz constant against modification.</summary>
    public void SetLipConstState(bool flag) => _isConstLocked = flag;

    public bool GetLipConstState() => _isConstLocked;

    /// <summary>The computation state of the algorithm.</summary>
    public bool IsDone() => _done;

    /// <summary>The best functional value.</summary>
    public double GetF() => _f;

    /// <summary>The number of global extrema.</summary>
    public int ExtremaCount => _solCount;
}
