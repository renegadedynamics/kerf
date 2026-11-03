// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompPolynomialToPoles.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompPolynomialToPoles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a composite curve made of polynomial pieces (n-dimensional) into the poles of an equivalent
/// B-spline, by interpolation at the Schoenberg points.
/// </summary>
/// <remarks>
/// The coefficients are in the "C form" Coefficients[NumCurves][MaxDegree + 1][Dimension]; the number of
/// coefficients of a polynomial is its degree + 1. Example: f(x) = 2x + 1 on [2, 5], to a B-spline on [-1, 1]:
/// NumCurves = 1, Continuity = 1, Dimension = 1, MaxDegree = 1, NumCoeffPerCurve = {2}, Coefficients = {1, 2},
/// PolynomialIntervals = {{2, 5}}, TrueIntervals = {-1, 1}. The batch accessors return copies, where OCCT returns
/// references to its fields.
/// </remarks>
public class CompPolynomialToPoles
{
    private NCollection_Array1<double> _flatKnots = new();
    private NCollection_Array1<double> _knots = new();
    private NCollection_Array1<int> _mults = new();
    private NCollection_Array2<double> _poles = new();
    private int _degree;
    private bool _done;

    /// <summary>
    /// Converts <paramref name="numCurves"/> polynomials with the same continuity between them (at most the
    /// maximum degree). <paramref name="trueIntervals"/> is the parametrisation of the result: curve n is
    /// parametrised on [TrueIntervals(n), TrueIntervals(n + 1)].
    /// </summary>
    public CompPolynomialToPoles(int numCurves,
                                         int continuity,
                                         int dimension,
                                         int maxDegree,
                                         NCollection_HArray1<int>? numCoeffPerCurve,
                                         NCollection_HArray1<double>? coefficients,
                                         NCollection_HArray2<double>? polynomialIntervals,
                                         NCollection_HArray1<double>? trueIntervals)
    {
        _degree = 0;
        _done = false;
        if (numCurves <= 0 || numCoeffPerCurve is null || coefficients is null
            || polynomialIntervals is null || trueIntervals is null || continuity < 0 || maxDegree <= 0
            || dimension <= 0 || polynomialIntervals.RowLength() != 2)
        {
            throw new ConstructionErrorException("Convert_CompPolynomialToPoles:bad arguments");
        }

        int delta = numCurves - 1;
        for (int ii = numCoeffPerCurve.Lower(); ii <= numCoeffPerCurve.Lower() + delta; ii++)
        {
            _degree = Max(numCoeffPerCurve.Value(ii) - 1, _degree);
        }
        if ((continuity > _degree) && (numCurves > 1))
        {
            throw new ConstructionErrorException("Convert_CompPolynomialToPoles:Continuity is too great");
        }

        _knots = new NCollection_Array1<double>(1, numCurves + 1);
        for (int ii = 1, tindex = trueIntervals.Lower(); ii <= numCurves + 1; ii++, tindex++)
        {
            _knots.SetValue(ii, trueIntervals.Value(tindex));
        }

        int multiplicities = _degree - continuity;
        _mults = new NCollection_Array1<int>(1, numCurves + 1);
        for (int ii = 2; ii < numCurves + 1; ii++)
        {
            _mults.SetValue(ii, multiplicities);
        }
        _mults.SetValue(1, _degree + 1);
        _mults.SetValue(numCurves + 1, _degree + 1);

        Perform(numCurves,
                maxDegree,
                dimension,
                numCoeffPerCurve.Array1(),
                coefficients.Array1(),
                polynomialIntervals.Array2(),
                trueIntervals.Array1());
    }

    /// <summary>
    /// Converts <paramref name="numCurves"/> polynomials with a continuity of its own at each junction
    /// (<paramref name="continuity"/>(ii) between curves ii - 1 and ii, for ii from 2 to NumCurves).
    /// </summary>
    public CompPolynomialToPoles(int numCurves,
                                         int dimension,
                                         int maxDegree,
                                         NCollection_Array1<int> continuity,
                                         NCollection_Array1<int> numCoeffPerCurve,
                                         NCollection_Array1<double> coefficients,
                                         NCollection_Array2<double> polynomialIntervals,
                                         NCollection_Array1<double> trueIntervals)
    {
        _degree = 0;
        _done = false;
        if (numCurves <= 0 || maxDegree <= 0 || dimension <= 0 || polynomialIntervals.RowLength() != 2)
        {
            throw new ConstructionErrorException("Convert_CompPolynomialToPoles:bad arguments");
        }

        int delta = numCurves - 1;
        for (int ii = numCoeffPerCurve.Lower(); ii <= numCoeffPerCurve.Lower() + delta; ii++)
        {
            _degree = Max(numCoeffPerCurve.Value(ii) - 1, _degree);
        }

        _knots = new NCollection_Array1<double>(1, numCurves + 1);
        for (int ii = 1, tindex = trueIntervals.Lower(); ii <= numCurves + 1; ii++, tindex++)
        {
            _knots.SetValue(ii, trueIntervals.Value(tindex));
        }

        _mults = new NCollection_Array1<int>(1, numCurves + 1);
        for (int ii = 2; ii < numCurves + 1; ii++)
        {
            if ((continuity[ii] > _degree) && (numCurves > 1))
            {
                throw new ConstructionErrorException("Convert_CompPolynomialToPoles:Continuity is too great");
            }

            _mults.SetValue(ii, _degree - continuity[ii]);
        }
        _mults.SetValue(1, _degree + 1);
        _mults.SetValue(numCurves + 1, _degree + 1);

        Perform(numCurves,
                maxDegree,
                dimension,
                numCoeffPerCurve,
                coefficients,
                polynomialIntervals,
                trueIntervals);
    }

    /// <summary>Converts one polynomial of degree <paramref name="degree"/>.</summary>
    public CompPolynomialToPoles(int dimension,
                                         int maxDegree,
                                         int degree,
                                         NCollection_Array1<double> coefficients,
                                         NCollection_Array1<double> polynomialIntervals,
                                         NCollection_Array1<double> trueIntervals)
    {
        _degree = degree;
        _done = false;
        if (maxDegree <= 0 || dimension <= 0 || polynomialIntervals.Length() != 2)
        {
            throw new ConstructionErrorException("Convert_CompPolynomialToPoles:bad arguments");
        }

        NCollection_Array2<double> thePolynomialIntervals = new(1, 1, 1, 2);
        thePolynomialIntervals.SetValue(1, 1, polynomialIntervals[polynomialIntervals.Lower()]);
        thePolynomialIntervals.SetValue(1, 2, polynomialIntervals[polynomialIntervals.Upper()]);

        NCollection_Array1<int> numCoeffPerCurve = new(1, 1);
        numCoeffPerCurve[1] = degree + 1;

        _knots = new NCollection_Array1<double>(1, 2);
        _knots.SetValue(1, trueIntervals.Value(trueIntervals.Lower()));
        _knots.SetValue(2, trueIntervals.Value(trueIntervals.Lower() + 1));

        _mults = new NCollection_Array1<int>(1, 2);
        _mults.Init(_degree + 1);

        Perform(1,
                maxDegree,
                dimension,
                numCoeffPerCurve,
                coefficients,
                thePolynomialIntervals,
                trueIntervals);
    }

    private void Perform(int numCurves,
                         int maxDegree,
                         int dimension,
                         NCollection_Array1<int> numCoeffPerCurve,
                         NCollection_Array1<double> coefficients,
                         NCollection_Array2<double> polynomialIntervals,
                         NCollection_Array1<double> trueIntervals)
    {
        int ii, num_flat_knots, index, tindex, pindex, coeff_index, inversion_problem, poles_index,
          num_poles;
        double normalized_value;

        num_flat_knots = 2 * _degree + 2;
        for (ii = 2; ii < _mults.Length(); ii++)
        {
            num_flat_knots += _mults.Value(ii);
        }
        num_poles = num_flat_knots - _degree - 1;

        _flatKnots = new NCollection_Array1<double>(1, num_flat_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_knots, _mults, _degree, false, _flatKnots);

        // Where OCCT uses stack buffers of 128 elements (when large enough), the port allocates
        NCollection_Array1<double> parameters = new(1, num_poles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(_degree, _flatKnots, parameters);
        _poles = new NCollection_Array2<double>(1, num_poles, 1, dimension);
        index = 2;
        tindex = trueIntervals.Lower() + 1;
        pindex = polynomialIntervals.LowerRow();
        Span<double> poles_array = _poles.AsSpan();

        NCollection_Array1<int> contact_array = new(1, num_poles);

        poles_index = 0;
        for (ii = 1; ii <= num_poles; ii++, poles_index += dimension)
        {
            contact_array.SetValue(ii, 0);
            while (parameters.Value(ii) >= trueIntervals[tindex] && index <= numCurves)
            {
                index++;
                tindex++;
                pindex++;
            }
            //
            // normalized value so that it fits the original intervals for
            // the polynomial definition of the curves
            //
            normalized_value = parameters.Value(ii) - trueIntervals[tindex - 1];
            normalized_value /= trueIntervals[tindex] - trueIntervals[tindex - 1];
            normalized_value =
              (1.0e0 - normalized_value) * polynomialIntervals[pindex, polynomialIntervals.LowerCol()]
              + normalized_value * polynomialIntervals[pindex, polynomialIntervals.UpperCol()];
            coeff_index =
              ((index - 2) * dimension * (Max(maxDegree, _degree) + 1)) + coefficients.Lower();

            ReadOnlySpan<double> coefficient_array = coefficients.AsSpan(coeff_index);
            int deg = numCoeffPerCurve[numCoeffPerCurve.Lower() + index - 2] - 1;

            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(normalized_value,
                                            deg,
                                            dimension,
                                            deg * dimension,
                                            coefficient_array,
                                            poles_array[poles_index..]);
        }
        //
        // interpolation at schoenberg points should yield the desired
        // result
        //
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(_degree,
                             _flatKnots,
                             parameters,
                             contact_array,
                             dimension,
                             poles_array,
                             out inversion_problem);
        if (inversion_problem != 0)
        {
            throw new ConstructionErrorException("Convert_CompPolynomialToPoles:inversion_problem");
        }
        _done = true;
    }

    /// <summary>The number of poles of the B-spline (0 if not done).</summary>
    public int NbPoles()
    {
        if (_done)
        {
            return _poles.ColLength();
        }
        return 0;
    }

    /// <summary>The poles of the B-spline, [1..NumPoles][1..Dimension]: a copy.</summary>
    public NCollection_Array2<double> Poles()
    {
        StdFail_NotDone_Raise_if(!_done, "Convert_CompPolynomialToPoles::Poles");
        return new NCollection_Array2<double>(_poles);
    }

    /// <summary>
    /// Sets <paramref name="p"/> to a copy of the poles, if done (unchanged otherwise). Deprecated in OCCT: use
    /// <see cref="Poles()"/>.
    /// </summary>
    public void Poles(ref NCollection_HArray2<double>? p)
    {
        if (_done)
        {
            p = new NCollection_HArray2<double>(_poles);
        }
    }

    /// <summary>The degree of the B-spline (0 if not done).</summary>
    public int Degree()
    {
        if (_done)
        {
            return _degree;
        }
        return 0;
    }

    /// <summary>The number of knots of the B-spline (0 if not done).</summary>
    public int NbKnots()
    {
        if (_done)
        {
            return _knots.Length();
        }
        return 0;
    }

    /// <summary>The knots of the B-spline: a copy.</summary>
    public NCollection_Array1<double> Knots()
    {
        StdFail_NotDone_Raise_if(!_done, "Convert_CompPolynomialToPoles::Knots");
        return new NCollection_Array1<double>(_knots);
    }

    /// <summary>
    /// Sets <paramref name="k"/> to a copy of the knots, if done (unchanged otherwise). Deprecated in OCCT: use
    /// <see cref="Knots()"/>.
    /// </summary>
    public void Knots(ref NCollection_HArray1<double>? k)
    {
        if (_done)
        {
            k = new NCollection_HArray1<double>(_knots);
        }
    }

    /// <summary>The multiplicities of the knots: a copy.</summary>
    public NCollection_Array1<int> Multiplicities()
    {
        StdFail_NotDone_Raise_if(!_done, "Convert_CompPolynomialToPoles::Multiplicities");
        return new NCollection_Array1<int>(_mults);
    }

    /// <summary>
    /// Sets <paramref name="m"/> to a copy of the multiplicities, if done (unchanged otherwise). Deprecated in
    /// OCCT: use <see cref="Multiplicities()"/>.
    /// </summary>
    public void Multiplicities(ref NCollection_HArray1<int>? m)
    {
        if (_done)
        {
            m = new NCollection_HArray1<int>(_mults);
        }
    }

    /// <summary>True if the conversion succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }
}
