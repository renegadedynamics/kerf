// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_SimpleApprox.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_SimpleApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// Approximates a function on one interval by a polynomial <c>P(t) = R(t) + W(t) * Q(t)</c> in the Jacobi base
/// (the Hermite interpolation of the end constraints plus a weighted least-squares part), then reduces its degree
/// to the tolerances of the subspaces.
/// </summary>
/// <remarks>
/// A working object of <see cref="ApproxAFunction"/> (decision 21): <see cref="Perform"/> may be called
/// again on other intervals. The array accessors return the arrays it works in (shared: don't modify).
/// </remarks>
public class SimpleApprox
{
    private readonly int _totalNumSS;
    private readonly int _totalDimension;
    private readonly int _nbGaussPoints;
    private readonly int _workDegree;
    private readonly int _nivConstr;
    private readonly JacobiPolynomial _jacPol;
    private readonly NCollection_HArray1<double> _tabPoints;
    private readonly NCollection_HArray2<double> _tabWeights;
    private readonly EvaluatorFunction _evaluator;
    private int _degree;
    private readonly NCollection_HArray1<double> _coeff;
    private readonly NCollection_HArray2<double> _firstConstr;
    private readonly NCollection_HArray2<double> _lastConstr;
    private readonly NCollection_HArray1<double> _somTab;
    private readonly NCollection_HArray1<double> _difTab;
    private NCollection_HArray1<double>? _maxError;
    private NCollection_HArray1<double>? _averageError;
    private bool _done;

    /// <summary>
    /// Prepares the approximation of <paramref name="func"/>, of dimension <paramref name="totalDimension"/>
    /// split into <paramref name="totalNumSS"/> subspaces, with the continuity C0, C1 or C2 at the ends.
    /// </summary>
    public SimpleApprox(int totalDimension, int totalNumSS, Continuity continuity, int workDegree,
                                  int nbGaussPoints, JacobiPolynomial jacobiBase, EvaluatorFunction func)
    {
        _totalNumSS = totalNumSS;
        _totalDimension = totalDimension;
        _nbGaussPoints = nbGaussPoints;
        _workDegree = workDegree;
        _jacPol = jacobiBase;
        _evaluator = func;

        // the Check of input parameters

        switch (continuity)
        {
            case C0:
                _nivConstr = 0;
                break;
            case C1:
                _nivConstr = 1;
                break;
            case C2:
                _nivConstr = 2;
                break;
            default:
                throw new ConstructionErrorException("Invalid Continuity");
        }

        int degreeQ = _workDegree - 2 * (_nivConstr + 1);

        // the extraction of the Legendre roots
        _tabPoints = new NCollection_HArray1<double>(0, nbGaussPoints / 2);
        jacobiBase.Points(nbGaussPoints, _tabPoints.ChangeArray1());

        // the extraction of the Gauss Weights
        _tabWeights = new NCollection_HArray2<double>(0, nbGaussPoints / 2, 0, degreeQ);
        jacobiBase.Weights(nbGaussPoints, _tabWeights.ChangeArray2());

        _coeff = new NCollection_HArray1<double>(0, (_workDegree + 1) * _totalDimension - 1);
        _firstConstr = new NCollection_HArray2<double>(1, _totalDimension, 0, _nivConstr);
        _lastConstr = new NCollection_HArray2<double>(1, _totalDimension, 0, _nivConstr);
        _somTab = new NCollection_HArray1<double>(0, (_nbGaussPoints / 2 + 1) * _totalDimension - 1);
        _difTab = new NCollection_HArray1<double>(0, (_nbGaussPoints / 2 + 1) * _totalDimension - 1);
        _done = false;
    }

    /// <summary>
    /// Constructs the approximation on [<paramref name="first"/>, <paramref name="last"/>], with the dimensions
    /// and tolerances of the subspaces, and a degree of at most <paramref name="maxDegree"/>.
    /// </summary>
    public void Perform(NCollection_Array1<int> localDimension, NCollection_Array1<double> localTolerancesArray,
                        double first, double last, int maxDegree)
    {
        // ======= the computation of Pp(t) = Rr(t) + W(t)*Qq(t) =======

        _done = false;
        int i, idim, k, numss;

        int dimension = _totalDimension;
        EvaluatorFunction evaluator = _evaluator;

        // ===== the computation of Rr(t) (the first part of Pp) ======

        int degreeR = 2 * _nivConstr + 1;
        int degreeQ = _workDegree - 2 * (_nivConstr + 1);

        ReadOnlySpan<double> firstLast = [first, last];

        math_Vector result = new(1, _totalDimension);
        int errorCode, derive, i_idim;
        double fact = (last - first) / 2;
        Span<double> pResult = result.AsSpan();
        double param;

        for (param = first, derive = _nivConstr; derive >= 0; derive--)
        {
            evaluator.Evaluate(dimension, firstLast, param, derive, pResult, out errorCode);
            if (errorCode != 0)
            {
                return; // Evaluation error
            }
            if (derive >= 1)
            {
                result *= fact;
            }
            if (derive == 2)
            {
                result *= fact;
            }
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                _firstConstr.SetValue(idim, derive, result[idim]);
            }
        }

        for (param = last, derive = _nivConstr; derive >= 0; derive--)
        {
            evaluator.Evaluate(dimension, firstLast, param, derive, pResult, out errorCode);
            if (errorCode != 0)
            {
                return; // Evaluation error
            }
            if (derive >= 1)
            {
                result *= fact;
            }
            if (derive == 2)
            {
                result *= fact;
            }
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                _lastConstr.SetValue(idim, derive, result[idim]);
            }
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteInterpolate(_totalDimension, -1.0, 1.0, _nivConstr, _nivConstr, _firstConstr.Array2(),
                                _lastConstr.Array2(), _coeff.ChangeArray1());

        // ===== the computation of the coefficients of Qq(t) (the second part of Pp) ======

        math_Vector fti = new(1, _totalDimension);
        math_Vector rpti = new(1, _totalDimension);
        math_Vector rmti = new(1, _totalDimension);
        Span<double> pFti = fti.AsSpan();
        ReadOnlySpan<double> coef1 = _coeff.ChangeArray1().AsSpan(0);

        derive = 0;
        double ti, tip, tin, alin = (last - first) / 2, blin = (last + first) / 2.0;

        i_idim = _totalDimension;
        for (i = 1; i <= _nbGaussPoints / 2; i++)
        {
            ti = _tabPoints.Value(i);
            tip = alin * ti + blin;
            evaluator.Evaluate(dimension, firstLast, tip, derive, pFti, out errorCode);
            if (errorCode != 0)
            {
                return; // Evaluation error
            }
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                _somTab.SetValue(i_idim, fti[idim]);
                _difTab.SetValue(i_idim, fti[idim]);
                i_idim++;
            }
        }

        i_idim = _totalDimension;
        for (i = 1; i <= _nbGaussPoints / 2; i++)
        {
            ti = _tabPoints.Value(i);
            tin = -alin * ti + blin;
            evaluator.Evaluate(dimension, firstLast, tin, derive, pFti, out errorCode);
            if (errorCode != 0)
            {
                return; // Evaluation error
            }

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, degreeR, _totalDimension, coef1, rpti.AsSpan());
            ti = -ti;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, degreeR, _totalDimension, coef1, rmti.AsSpan());

            for (idim = 1; idim <= _totalDimension; idim++)
            {
                _somTab.SetValue(i_idim, _somTab.Value(i_idim) + fti[idim] - rpti[idim] - rmti[idim]);
                _difTab.SetValue(i_idim, _difTab.Value(i_idim) - fti[idim] - rpti[idim] + rmti[idim]);
                i_idim++;
            }
        }

        // for odd NbGaussPoints - the computation of [ F(0) - R(0) ]
        if (_nbGaussPoints % 2 == 1)
        {
            ti = _tabPoints.Value(0);
            tip = blin;
            evaluator.Evaluate(dimension, firstLast, tip, derive, pFti, out errorCode);
            if (errorCode != 0)
            {
                return; // Evaluation error
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, degreeR, _totalDimension, coef1, rpti.AsSpan());
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                _somTab.SetValue(idim - 1, fti[idim] - rpti[idim]);
                _difTab.SetValue(idim - 1, fti[idim] - rpti[idim]);
            }
        }

        // the computation of Qq(t)
        double sum = 0.0;
        for (k = 0; k <= degreeQ; k += 2)
        {
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                sum = 0.0;
                for (i = 1; i <= _nbGaussPoints / 2; i++)
                {
                    sum += _tabWeights.Value(i, k) * _somTab.Value(i * _totalDimension + idim - 1);
                }
                _coeff.SetValue((k + degreeR + 1) * _totalDimension + idim - 1, sum);
            }
        }
        for (k = 1; k <= degreeQ; k += 2)
        {
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                sum = 0.0;
                for (i = 1; i <= _nbGaussPoints / 2; i++)
                {
                    sum += _tabWeights.Value(i, k) * _difTab.Value(i * _totalDimension + idim - 1);
                }
                _coeff.SetValue((k + degreeR + 1) * _totalDimension + idim - 1, sum);
            }
        }
        if (_nbGaussPoints % 2 == 1)
        {
            for (idim = 1; idim <= _totalDimension; idim++)
            {
                for (k = 0; k <= degreeQ; k += 2)
                {
                    // OCCT: Sum still holds the last coefficient computed, not this one, so each even coefficient
                    // is overwritten by a running sum carried from coefficient to coefficient (occt-issues.md).
                    sum += _tabWeights.Value(0, k) * _somTab.Value(0 * _totalDimension + idim - 1);
                    _coeff.SetValue((k + degreeR + 1) * _totalDimension + idim - 1, sum);
                }
            }
        }

        // the computing of NewDegree
        NCollection_Array1<double> jacCoeff = new(0, _totalDimension * (_workDegree + 1) - 1);

        double maxErr, averageErr;
        int dim, rangSS, rangCoeff, rangJacCoeff, rangDim, newDegree, newDegreeMax = 0;

        _maxError = new NCollection_HArray1<double>(1, _totalNumSS);
        _averageError = new NCollection_HArray1<double>(1, _totalNumSS);
        rangSS = 0;
        rangJacCoeff = 0;
        for (numss = 1; numss <= _totalNumSS; numss++)
        {
            dim = localDimension[numss];
            rangCoeff = 0;
            rangDim = 0;
            for (k = 0; k <= _workDegree; k++)
            {
                for (idim = 1; idim <= dim; idim++)
                {
                    jacCoeff[rangJacCoeff + rangDim + idim - 1] = _coeff.Value(rangCoeff + rangSS + idim - 1);
                }
                rangDim = rangDim + dim;
                rangCoeff = rangCoeff + _totalDimension;
            }

            ReadOnlySpan<double> jacSS = jacCoeff.AsSpan(rangJacCoeff);
            _jacPol.ReduceDegree(dim, maxDegree, localTolerancesArray[numss], jacSS, out newDegree, out maxErr);
            if (newDegree > newDegreeMax)
            {
                newDegreeMax = newDegree;
            }
            rangSS = rangSS + dim;
            rangJacCoeff = rangJacCoeff + (_workDegree + 1) * dim;
        }

        // the computing of MaxError and AverageError
        rangSS = 0;
        rangJacCoeff = 0;
        for (numss = 1; numss <= _totalNumSS; numss++)
        {
            dim = localDimension[numss];

            ReadOnlySpan<double> jacSS = jacCoeff.AsSpan(rangJacCoeff);
            maxErr = _jacPol.MaxError(localDimension[numss], jacSS, newDegreeMax);
            _maxError.SetValue(numss, maxErr);
            averageErr = _jacPol.AverageError(localDimension[numss], jacSS, newDegreeMax);
            _averageError.SetValue(numss, averageErr);
            rangSS = rangSS + dim;
            rangJacCoeff = rangJacCoeff + (_workDegree + 1) * dim;
        }

        _degree = newDegreeMax;
        _done = true;
    }

    public bool IsDone => _done;

    public int Degree() => _degree;

    /// <summary>The coefficients in the Jacobi base (shared: don't modify).</summary>
    public NCollection_HArray1<double> Coefficients() => _coeff;

    /// <summary>The constraints at the first parameter (shared: don't modify).</summary>
    public NCollection_HArray2<double> FirstConstr() => _firstConstr;

    /// <summary>The constraints at the last parameter (shared: don't modify).</summary>
    public NCollection_HArray2<double> LastConstr() => _lastConstr;

    /// <summary>A working array (shared: don't modify).</summary>
    public NCollection_HArray1<double> SomTab() => _somTab;

    /// <summary>A working array (shared: don't modify).</summary>
    public NCollection_HArray1<double> DifTab() => _difTab;

    public double MaxError(int index) => _maxError!.Value(index);

    public double AverageError(int index) => _averageError!.Value(index);

    /// <summary>Writes the maximum error of each subspace.</summary>
    public void Dump(TextWriter o)
    {
        int ii;
        o.Write("Dump of SimpleApprox \n");
        for (ii = 1; ii <= _totalNumSS; ii++)
        {
            o.Write("Error   " + MaxError(ii).ToString(CultureInfo.InvariantCulture) + "\n");
        }
    }
}
