// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics.Polynomials;
using Kerf.Engine;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>
/// A curve made of polynomial elements in a Hermite-Jacobi base, one per knot interval, each parametrised on
/// [-1, 1].
/// </summary>
/// <remarks>
/// A working object of AppDef_Variational (decision 21): <see cref="Knots"/> returns the curve's own knot array,
/// which callers fill or change in place as in OCCT (shared: changing it changes the curve). The evaluators cache
/// the canonical coefficients of each element, so they are not thread-safe.
/// </remarks>
internal class Curve : Transient
{
    private readonly int _nbElements;
    private readonly int _dimension;
    private readonly HermitJacobi _base;
    private readonly NCollection_HArray1<double> _knots;
    private readonly NCollection_Array1<int> _degree;
    private readonly NCollection_Array1<double> _coeff;
    private readonly NCollection_Array1<double> _poly;
    private readonly NCollection_Array1<double> _deri;
    private readonly NCollection_Array1<double> _dsecn;
    private readonly NCollection_Array1<int> _hasPoly;
    private readonly NCollection_Array1<int> _hasDeri;
    private readonly NCollection_Array1<int> _hasSecn;
    private readonly NCollection_Array1<double> _length;
    private double _uf;
    private double _ul;
    private double _denom;
    private double _uSum;
    private int _index;
    private int _ptr;

    /// <summary>
    /// A curve of dimension <paramref name="dimension"/> with <paramref name="nbElements"/> elements in the base
    /// <paramref name="theBase"/>. The tolerance is not used (as in OCCT). The knots must be set through
    /// <see cref="Knots"/>.
    /// </summary>
    public Curve(int dimension, int nbElements, HermitJacobi theBase, double tolerance)
    {
        _nbElements = nbElements;
        _dimension = dimension;
        _base = theBase;
        _degree = new NCollection_Array1<int>(1, _nbElements);
        _coeff = new NCollection_Array1<double>(1, _dimension * _nbElements * (_base.WorkDegree() + 1));
        _poly = new NCollection_Array1<double>(1, _dimension * _nbElements * (_base.WorkDegree() + 1));
        _deri = new NCollection_Array1<double>(1, _dimension * _nbElements * (_base.WorkDegree()));
        _dsecn = new NCollection_Array1<double>(1, _dimension * _nbElements * (_base.WorkDegree() - 1));
        _hasPoly = new NCollection_Array1<int>(1, _nbElements);
        _hasDeri = new NCollection_Array1<int>(1, _nbElements);
        _hasSecn = new NCollection_Array1<int>(1, _nbElements);
        _length = new NCollection_Array1<double>(1, _nbElements);
        _index = 0;

        _knots = new NCollection_HArray1<double>(1, _nbElements + 1);
        _degree.Init(_base.WorkDegree());
        _hasPoly.Init(0);
        _hasDeri.Init(0);
        _hasSecn.Init(0);
        _length.Init(-1);
    }

    /// <summary>The knots (1 .. NbElements + 1): the curve's own array (shared: changing it changes the curve).</summary>
    public NCollection_Array1<double> Knots() => _knots.ChangeArray1();

    /// <summary>
    /// Sets the coefficients of element <paramref name="indexOfElement"/> in the Hermite-Jacobi base:
    /// <paramref name="coeffs"/>(degree, dimension), with the Hermite part relative to the knot interval.
    /// </summary>
    public void SetElement(int indexOfElement, NCollection_Array2<double> coeffs)
    {
        int i, j, degBase, deg;
        if (indexOfElement > _nbElements || indexOfElement < 1)
        {
            throw new OutOfRangeException();
        }
        degBase = _base.WorkDegree();
        deg = _degree[indexOfElement];
        int iBase = (indexOfElement - 1) * (degBase + 1) * _dimension, i1 = iBase - _dimension,
            i2 = coeffs.LowerRow() - 1, j1 = coeffs.LowerCol() - 1;
        for (i = 1; i <= deg + 1; i++)
        {
            i1 += _dimension;
            i2++;
            for (j = 1; j <= _dimension; j++)
            {
                _coeff[i1 + j] = coeffs[i2, j1 + j];
            }
        }

        double stenor = (_knots.Value(indexOfElement + 1) - _knots.Value(indexOfElement)) / 2.0, mfact;
        i1 = iBase;
        i2 = iBase + (_base.NivConstr() + 1) * _dimension;
        for (i = 1; i <= _base.NivConstr(); i++)
        {
            i1 += _dimension;
            i2 += _dimension;
            mfact = Math.Pow(stenor, i);
            for (j = 1; j <= _dimension; j++)
            {
                _coeff[i1 + j] *= mfact;
                _coeff[i2 + j] *= mfact;
            }
        }

        _hasPoly[indexOfElement] = _hasDeri[indexOfElement] = _hasSecn[indexOfElement] = 0;
        _length[indexOfElement] = -1;
    }

    /// <summary>The coefficients of element <paramref name="indexOfElement"/>, as given to <see cref="SetElement"/>.</summary>
    public void GetElement(int indexOfElement, NCollection_Array2<double> coeffs)
    {
        int i, j, degBase, deg;
        if (indexOfElement > _nbElements || indexOfElement < 1)
        {
            throw new OutOfRangeException();
        }
        degBase = _base.WorkDegree();
        deg = _degree[indexOfElement];
        int iBase = (indexOfElement - 1) * (degBase + 1) * _dimension, i1 = iBase - _dimension,
            i2 = coeffs.LowerRow() - 1, j1 = coeffs.LowerCol() - 1;
        for (i = 1; i <= deg + 1; i++)
        {
            i1 += _dimension;
            i2++;
            for (j = 1; j <= _dimension; j++)
            {
                coeffs[i2, j1 + j] = _coeff.Value(i1 + j);
            }
        }

        double stenor = 2.0 / (_knots.Value(indexOfElement + 1) - _knots.Value(indexOfElement)), mfact;
        i2 = coeffs.LowerRow();
        int i3 = i2 + _base.NivConstr() + 1;

        for (i = 1; i <= _base.NivConstr(); i++)
        {
            mfact = Math.Pow(stenor, i);
            // OCCT: the bound should be j1 + myDimension; with Coeffs.LowerCol() > 1 the last columns are not
            // rescaled (occt-issues.md)
            for (j = j1 + 1; j <= _dimension; j++)
            {
                coeffs[i2 + i, j] *= mfact;
                coeffs[i3 + i, j] *= mfact;
            }
        }
    }

    /// <summary>The coefficients of all elements in the canonical base, each on [-1, 1].</summary>
    public void GetPolynom(NCollection_Array1<double> coeffs)
    {
        int indexOfElement, i, di = coeffs.Lower() - _poly.Lower();

        for (indexOfElement = 1; indexOfElement <= _nbElements; indexOfElement++)
        {
            if (_hasPoly.Value(indexOfElement) == 0)
            {
                Update(indexOfElement, 0);
            }
        }

        for (i = _poly.Lower(); i <= _poly.Upper(); i++)
        {
            coeffs[di + i] = _poly[i];
        }
    }

    // The span search shared by D0, D1 and D2
    private void LocateSpan(double u)
    {
        if (_index == 0 || (u < _uf) || (u > _ul) || (_knots.Value(_index) != _uf)
            || (_knots.Value(_index + 1) != _ul))
        {
            // Search the span
            if (u <= _knots.Value(2))
            {
                _index = 1;
            }
            else
            {
                for (_index = 2; _index <= _nbElements; _index++)
                {
                    if (u >= _knots.Value(_index) && u <= _knots.Value(_index + 1))
                    {
                        break;
                    }
                }
                if (_index > _nbElements)
                {
                    _index = _nbElements;
                }
            }

            _uf = _knots.Value(_index);
            _ul = _knots.Value(_index + 1);
            _denom = 1.0 / (_ul - _uf);
            _uSum = _uf + _ul;
            _ptr = (_index - 1) * (_base.WorkDegree() + 1) * _dimension + 1;
        }
    }

    /// <summary>The point at <paramref name="u"/> into <paramref name="pnt"/> (Dimension values).</summary>
    public void D0(double u, NCollection_Array1<double> pnt)
    {
        int deg;
        double s;

        LocateSpan(u);

        deg = _degree[_index];
        if (_hasPoly.Value(_index) == 0)
        {
            Update(_index, 0);
        }

        // Parameter normalization: S [-1, 1]
        s = (2 * u - _uSum) * _denom;
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(s, deg, _dimension, deg * _dimension, _poly.AsSpan(_ptr),
                                        pnt.AsSpan(pnt.Lower()));
    }

    /// <summary>The first derivative at <paramref name="u"/> into <paramref name="vec"/>.</summary>
    public void D1(double u, NCollection_Array1<double> vec)
    {
        int deg, i;
        double s;

        LocateSpan(u);

        deg = _degree[_index];
        if (_hasDeri.Value(_index) == 0)
        {
            Update(_index, 1);
        }

        // Parameter normalization: S [-1, 1]
        s = (2 * u - _uSum) * _denom;
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(s, deg - 1, _dimension, (deg - 1) * _dimension,
                                        _deri.AsSpan(1 + (_index - 1) * _base.WorkDegree() * _dimension),
                                        vec.AsSpan(vec.Lower()));

        s = 2 * _denom;
        for (i = vec.Lower(); i <= vec.Upper(); i++)
        {
            vec[i] *= s;
        }
    }

    /// <summary>The second derivative at <paramref name="u"/> into <paramref name="vec"/>.</summary>
    public void D2(double u, NCollection_Array1<double> vec)
    {
        int deg, i;
        double s;

        LocateSpan(u);

        deg = _degree[_index];
        if (_hasSecn.Value(_index) == 0)
        {
            Update(_index, 2);
        }

        // Parameter normalization: S [-1, 1]
        s = (2 * u - _uSum) * _denom;
        if (deg < 2)
        {
            // OCCT: an element of degree 1 (ReduceDegree allows it at C0) has no second-derivative coefficients, and
            // OCCT evaluates a polynomial of degree -1, whose value is the Dimension coefficients just before the
            // element's block. For a later element they are the last coefficients of the previous element's block,
            // which the port reads as OCCT does; for the first element they lie before the array (uninitialised
            // memory in OCCT), and the port uses zeros (occt-issues.md, decision 206).
            int top = 1 + (_index - 1) * (_base.WorkDegree() - 1) * _dimension - _dimension;
            for (i = 0; i < _dimension; i++)
            {
                vec[vec.Lower() + i] = top >= 1 ? _dsecn[top + i] : 0.0;
            }
        }
        else
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(s, deg - 2, _dimension, (deg - 2) * _dimension,
                                            _dsecn.AsSpan(1 + (_index - 1) * (_base.WorkDegree() - 1) * _dimension),
                                            vec.AsSpan(vec.Lower()));
        }

        s = 4 * _denom * _denom;
        for (i = vec.Lower(); i <= vec.Upper(); i++)
        {
            vec[i] *= s;
        }
    }

    /// <summary>The length of the curve between <paramref name="firstU"/> and <paramref name="lastU"/>.</summary>
    public void Length(double firstU, double lastU, out double length)
    {
        int low, high, deg, degBase, i, ptr;
        if (firstU > lastU)
        {
            throw new OutOfRangeException("FEmTool_Curve::Length");
        }

        if (_knots.Value(1) > firstU)
        {
            low = 1;
        }
        else
        {
            for (low = 1; low <= _nbElements; low++)
            {
                if (firstU >= _knots.Value(low) && firstU <= _knots.Value(low + 1))
                {
                    break;
                }
            }
        }
        if (low > _nbElements)
        {
            low = _nbElements;
        }

        if (_knots.Value(1) > lastU)
        {
            high = 1;
        }
        else
        {
            for (high = low; high <= _nbElements; high++)
            {
                if (lastU >= _knots.Value(high) && lastU <= _knots.Value(high + 1))
                {
                    break;
                }
            }
        }
        if (_knots.Value(_nbElements + 1) < lastU)
        {
            high = _nbElements;
        }

        double li;
        degBase = _base.WorkDegree();
        length = 0;

        double firstS, lastS;
        firstS = (2 * firstU - _knots.Value(low) - _knots.Value(low + 1))
                 / (_knots.Value(low + 1) - _knots.Value(low));
        lastS = (2 * lastU - _knots.Value(high) - _knots.Value(high + 1))
                / (_knots.Value(high + 1) - _knots.Value(high));

        if (low == high)
        {
            ptr = (low - 1) * (degBase + 1) * _dimension + 1;
            deg = _degree[low];

            if (_hasPoly[low] == 0)
            {
                Update(low, 0);
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), firstS, lastS, out length);
            return;
        }

        deg = _degree[low];
        ptr = (low - 1) * (degBase + 1) * _dimension + 1;

        if (_hasPoly[low] == 0)
        {
            Update(low, 0);
        }
        if (firstS < -1.0)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), firstS, -1.0, out li);
            length += li;
            if (_length[low] < 0.0)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), -1.0, 1.0, out li);
                _length[low] = li;
            }
            length += _length[low];
        }
        else
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), firstS, 1.0, out li);
            length += li;
        }

        deg = _degree[high];
        ptr = (high - 1) * (degBase + 1) * _dimension + 1;

        if (_hasPoly[high] == 0)
        {
            Update(high, 0);
        }
        if (lastS > 1.0)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), 1.0, lastS, out li);
            length += li;
            if (_length[high] < 0.0)
            {
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), -1.0, 1.0, out li);
                _length[high] = li;
            }
            length += _length[high];
        }
        else
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), -1.0, lastS, out li);
            length += li;
        }

        for (i = low + 1; i < high; i++)
        {
            if (_length.Value(i) < 0)
            {
                ptr = (i - 1) * (degBase + 1) * _dimension + 1;
                deg = _degree[i];
                if (_hasPoly[i] == 0)
                {
                    Update(i, 0);
                }
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLength(deg, _dimension, _poly.AsSpan(ptr), -1.0, 1.0, out li);
                _length[i] = li;
            }
            length += _length[i];
        }
    }

    public int ElementCount => _nbElements;

    public int Dimension() => _dimension;

    public HermitJacobi Base() => _base;

    public int Degree(int indexOfElement) => _degree.Value(indexOfElement);

    /// <summary>Sets the degree of an element (at most the base's work degree).</summary>
    public void SetDegree(int indexOfElement, int degree)
    {
        if (degree <= _base.WorkDegree())
        {
            _degree[indexOfElement] = degree;
            _hasPoly[indexOfElement] = _hasDeri[indexOfElement] = _hasSecn[indexOfElement] = 0;
            _length[indexOfElement] = -1;
        }
        else if (degree > _base.WorkDegree())
        {
            throw new OutOfRangeException("FEmTool_Curve::SetDegree");
        }
    }

    /// <summary>
    /// Lowers the degree of an element while the error stays below <paramref name="tol"/> (not below
    /// 2 * NivConstr + 1).
    /// </summary>
    public void ReduceDegree(int indexOfElement, double tol, out int newDegree, out double maxError)
    {
        int deg = _degree[indexOfElement];

        int ptr = (indexOfElement - 1) * (_base.WorkDegree() + 1) * _dimension + 1;

        _base.ReduceDegree(_dimension, deg, tol, _coeff.AsSpan(ptr), out newDegree, out maxError);
        newDegree = Max(newDegree, 2 * _base.NivConstr() + 1);

        if (newDegree < deg)
        {
            _degree[indexOfElement] = newDegree;
            _hasPoly[indexOfElement] = _hasDeri[indexOfElement] = _hasSecn[indexOfElement] = 0;
            _length[indexOfElement] = -1;
        }
    }

    private void Update(int index, int order)
    {
        int degBase = _base.WorkDegree(), deg = _degree[index];

        if (_hasPoly[index] == 0)
        {
            int ptr = (index - 1) * (degBase + 1) * _dimension + 1;

            NCollection_Array1<double> coeff = new(_poly, ptr, 0, _dimension * (deg + 1) - 1);
            NCollection_Array1<double> baseCoeff = new(_coeff, ptr, 0, _dimension * (deg + 1) - 1);

            _base.ToCoefficients(_dimension, deg, baseCoeff, coeff);
            _hasPoly[index] = 1;
        }

        if (order >= 1)
        {
            int i1 = (index - 1) * (degBase) * _dimension - _dimension,
                i2 = (index - 1) * (degBase + 1) * _dimension;
            int i, j;
            if (_hasDeri.Value(index) == 0)
            {
                for (i = 1; i <= deg; i++)
                {
                    i1 += _dimension;
                    i2 += _dimension;
                    for (j = 1; j <= _dimension; j++)
                    {
                        _deri[i1 + j] = i * _poly.Value(i2 + j);
                    }
                }
                _hasDeri[index] = 1;
            }
            if ((order >= 2) && _hasSecn.Value(index) == 0)
            {
                i1 = (index - 1) * (degBase - 1) * _dimension - _dimension;
                i2 = (index - 1) * (degBase) * _dimension;
                for (i = 1; i < deg; i++)
                {
                    i1 += _dimension;
                    i2 += _dimension;
                    for (j = 1; j <= _dimension; j++)
                    {
                        _dsecn[i1 + j] = i * _deri.Value(i2 + j);
                    }
                }
                _hasSecn[index] = 1;
            }
        }
    }
}
