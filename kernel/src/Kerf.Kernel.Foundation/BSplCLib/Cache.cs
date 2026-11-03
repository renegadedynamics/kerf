// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_Cache.hxx
// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_Cache.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// A cache of the polynomial coefficients of one span of a Bezier or B-spline curve (2D or 3D), for fast
/// evaluation. The data must be rebuilt (<see cref="BuildCache(double, NCollection_Array1{double}, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>)
/// when going from span to span (<see cref="IsCacheValid"/>).
/// </summary>
/// <remarks>
/// Mutable: BuildCache changes it. OCCT keeps the coefficients in a 128-element member buffer; the port allocates
/// a (Degree + 1) x row-length array once, in the constructor. Evaluation allocates nothing.
/// </remarks>
public class Cache : Transient
{
    private readonly bool _isRational;          //!< identifies the rationality of Bezier/B-spline curve
    private readonly CacheParams _params; //!< cache parameters
    private readonly int _rowLength;            //!< number of columns in the cache array
    // Poles and weights of the cache, (Degree + 1) rows of x y [z] [w]
    // (for 2D curves: no z component; for non-rational curves: no weight)
    private readonly NCollection_Array2<double> _polesWeights;

    /// <summary>
    /// Prepares the cache for a 2D curve of the given degree and flat knots (knots with repetitions); the poles
    /// only distinguish this constructor from the 3D one.
    /// </summary>
    public Cache(int degree, bool periodic, NCollection_Array1<double> flatKnots,
                          NCollection_Array1<Point2D> poles2d, NCollection_Array1<double>? weights = null)
    {
        _ = poles2d;
        _isRational = weights != null;
        _params = new CacheParams(degree, periodic, flatKnots);
        _rowLength = _isRational ? 3 : 2;
        _polesWeights = new NCollection_Array2<double>(1, _params.Degree + 1, 1, _rowLength);
    }

    /// <summary>
    /// Prepares the cache for a 3D curve of the given degree and flat knots (knots with repetitions); the poles
    /// only distinguish this constructor from the 2D one.
    /// </summary>
    public Cache(int degree, bool periodic, NCollection_Array1<double> flatKnots,
                          NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights = null)
    {
        _ = poles;
        _isRational = weights != null;
        _params = new CacheParams(degree, periodic, flatKnots);
        _rowLength = _isRational ? 4 : 3;
        _polesWeights = new NCollection_Array2<double>(1, _params.Degree + 1, 1, _rowLength);
    }

    /// <summary>True if the cached span is the one for the parameter (a flat-knot parameter).</summary>
    public bool IsCacheValid(double parameter)
    {
        return _params.IsCacheValid(parameter);
    }

    /// <summary>Recomputes the cache for the span of <paramref name="parameter"/> of a 2D curve, without checking whether it is valid.</summary>
    internal void BuildCache(double parameter, NCollection_Array1<double> flatKnots,
                             NCollection_Array1<Point2D> poles2d, NCollection_Array1<double>? weights)
    {
        // Normalize theParameter for periodical B-splines
        double newParam = _params.PeriodicNormalization(parameter);
        _params.LocateParameter(ref newParam, flatKnots);

        // Calculate new cache data
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(_params.SpanStart,
                            _params.SpanLength,
                            _params.IsPeriodic,
                            _params.Degree,
                            _params.SpanIndex,
                            flatKnots,
                            poles2d,
                            weights,
                            _polesWeights);
    }

    /// <summary>Recomputes the cache for the span of <paramref name="parameter"/> of a 3D curve, without checking whether it is valid.</summary>
    internal void BuildCache(double parameter, NCollection_Array1<double> flatKnots,
                             NCollection_Array1<Point3D> poles, NCollection_Array1<double>? weights = null)
    {
        // Create list of knots with repetitions and normalize theParameter for periodical B-splines
        double newParam = _params.PeriodicNormalization(parameter);
        _params.LocateParameter(ref newParam, flatKnots);

        // Calculate new cache data
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(_params.SpanStart,
                            _params.SpanLength,
                            _params.IsPeriodic,
                            _params.Degree,
                            _params.SpanIndex,
                            flatKnots,
                            poles,
                            weights,
                            _polesWeights);
    }

    /// <summary>
    /// Fills <paramref name="derivArray"/> with the derivatives up to <paramref name="derivative"/> at the
    /// parameter: (theDerivative + 1) * (PntDim + 1) values, PntDim being 2 or 3.
    /// </summary>
    protected void CalculateDerivative(double parameter, int derivative, Span<double> derivArray)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        CalculateDerivativeLocal(localParam, derivative, derivArray);
    }

    /// <summary>
    /// Fills <paramref name="derivArray"/> with the derivatives up to <paramref name="derivative"/> (1, 2 or
    /// 3) at the local parameter (Param - SpanStart) / SpanLength.
    /// </summary>
    protected void CalculateDerivativeLocal(double localParam, int derivative, Span<double> derivArray)
    {
        int dimension = _rowLength;

        // Temporary container. The maximal size of this container is defined by:
        //    1) maximal derivative for cache evaluation, which is 3, plus one row for function values,
        //    2) and maximal dimension of the point, which is 3, plus one column for weights.
        Span<double> tmpContainer = stackalloc double[16];

        // When the PLib::RationalDerivative needs to be called, use temporary container
        Span<double> pntDeriv = _isRational ? tmpContainer : derivArray;

        // When the degree of curve is lesser than the requested derivative,
        // nullify array cells corresponding to greater derivatives
        int aDerivative = derivative;
        if (!_isRational && _params.Degree < derivative)
        {
            aDerivative = _params.Degree;
            for (int ind = _params.Degree * dimension; ind < (derivative + 1) * dimension; ind++)
            {
                pntDeriv[ind] = 0.0;
            }
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(localParam,
                            aDerivative,
                            _params.Degree,
                            dimension,
                            _polesWeights.AsSpan(),
                            pntDeriv);

        // Unnormalize derivatives since those are computed normalized
        // Use division by SpanLength instead of multiplication by precomputed inverse
        // for better numerical stability with very small span lengths
        double factor = 1.0;
        for (int deriv = 1; deriv <= aDerivative; deriv++)
        {
            factor /= _params.SpanLength;
            for (int ind = 0; ind < dimension; ind++)
            {
                pntDeriv[dimension * deriv + ind] *= factor;
            }
        }

        if (_isRational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(aDerivative,
                                    aDerivative,
                                    dimension - 1,
                                    pntDeriv,
                                    derivArray);
        }
    }

    /// <summary>The point of the 2D curve at the parameter.</summary>
    public void D0(double parameter, out Point2D point)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D0Local(localParam, out point);
    }

    /// <summary>The point of the 3D curve at the parameter.</summary>
    public void D0(double parameter, out Point3D point)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D0Local(localParam, out point);
    }

    /// <summary>
    /// The 3D point at the local parameter (Param - SpanStart) / SpanLength, in [0, 1]; skips the periodic
    /// normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double localParam, out Point3D point)
    {
        // theLocalParam is already computed as (param - SpanStart) / SpanLength
        Span<double> aPoint = stackalloc double[4];

        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(localParam,
                                        _params.Degree,
                                        _rowLength,
                                        _params.Degree * _rowLength,
                                        _polesWeights.AsSpan(),
                                        aPoint);

        point = new Point3D(aPoint[0], aPoint[1], aPoint[2]);
        if (_isRational)
        {
            point.ChangeCoord().Divide(aPoint[3]);
        }
    }

    /// <summary>The point and first derivative of the 2D curve at the parameter.</summary>
    public void D1(double parameter, out Point2D point, out Vector2D tangent)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D1Local(localParam, out point, out tangent);
    }

    /// <summary>The point and first derivative of the 3D curve at the parameter.</summary>
    public void D1(double parameter, out Point3D point, out Vector3D tangent)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D1Local(localParam, out point, out tangent);
    }

    /// <summary>The 3D point and first derivative at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D1Local(double localParam, out Point3D point, out Vector3D tangent)
    {
        Span<double> derivArray = stackalloc double[8];
        CalculateDerivativeLocal(localParam, 1, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        point = new Point3D(derivArray[0], derivArray[1], derivArray[2]);
        tangent = new Vector3D(derivArray[dim], derivArray[dim + 1], derivArray[dim + 2]);
    }

    /// <summary>The point and first two derivatives of the 2D curve at the parameter.</summary>
    public void D2(double parameter, out Point2D point, out Vector2D tangent, out Vector2D curvature)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D2Local(localParam, out point, out tangent, out curvature);
    }

    /// <summary>The point and first two derivatives of the 3D curve at the parameter.</summary>
    public void D2(double parameter, out Point3D point, out Vector3D tangent, out Vector3D curvature)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D2Local(localParam, out point, out tangent, out curvature);
    }

    /// <summary>The 3D point and first two derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D2Local(double localParam, out Point3D point, out Vector3D tangent, out Vector3D curvature)
    {
        Span<double> derivArray = stackalloc double[12];
        CalculateDerivativeLocal(localParam, 2, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        int shift = dim << 1;
        point = new Point3D(derivArray[0], derivArray[1], derivArray[2]);
        tangent = new Vector3D(derivArray[dim], derivArray[dim + 1], derivArray[dim + 2]);
        curvature = new Vector3D(derivArray[shift], derivArray[shift + 1], derivArray[shift + 2]);
    }

    /// <summary>The point and first three derivatives of the 2D curve at the parameter.</summary>
    public void D3(double parameter, out Point2D point, out Vector2D tangent, out Vector2D curvature,
                   out Vector2D torsion)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D3Local(localParam, out point, out tangent, out curvature, out torsion);
    }

    /// <summary>The point and first three derivatives of the 3D curve at the parameter.</summary>
    public void D3(double parameter, out Point3D point, out Vector3D tangent, out Vector3D curvature,
                   out Vector3D torsion)
    {
        double localParam = _params.PeriodicNormalization(parameter);
        localParam = (localParam - _params.SpanStart) / _params.SpanLength;
        D3Local(localParam, out point, out tangent, out curvature, out torsion);
    }

    /// <summary>The 3D point and first three derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D3Local(double localParam, out Point3D point, out Vector3D tangent, out Vector3D curvature,
                        out Vector3D torsion)
    {
        Span<double> derivArray = stackalloc double[16];
        CalculateDerivativeLocal(localParam, 3, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        int shift2 = dim << 1;
        int shift3 = shift2 + dim;
        point = new Point3D(derivArray[0], derivArray[1], derivArray[2]);
        tangent = new Vector3D(derivArray[dim], derivArray[dim + 1], derivArray[dim + 2]);
        curvature = new Vector3D(derivArray[shift2], derivArray[shift2 + 1], derivArray[shift2 + 2]);
        torsion = new Vector3D(derivArray[shift3], derivArray[shift3 + 1], derivArray[shift3 + 2]);
    }

    /// <summary>
    /// The 2D point at the local parameter (Param - SpanStart) / SpanLength, in [0, 1]; skips the periodic
    /// normalization and the local parameter computation.
    /// </summary>
    public void D0Local(double localParam, out Point2D point)
    {
        Span<double> aPoint = stackalloc double[4];

        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(localParam,
                                        _params.Degree,
                                        _rowLength,
                                        _params.Degree * _rowLength,
                                        _polesWeights.AsSpan(),
                                        aPoint);

        point = new Point2D(aPoint[0], aPoint[1]);
        if (_isRational)
        {
            point.ChangeCoord().Divide(aPoint[2]);
        }
    }

    /// <summary>The 2D point and first derivative at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D1Local(double localParam, out Point2D point, out Vector2D tangent)
    {
        Span<double> derivArray = stackalloc double[8];
        CalculateDerivativeLocal(localParam, 1, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        point = new Point2D(derivArray[0], derivArray[1]);
        tangent = new Vector2D(derivArray[dim], derivArray[dim + 1]);
    }

    /// <summary>The 2D point and first two derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D2Local(double localParam, out Point2D point, out Vector2D tangent, out Vector2D curvature)
    {
        Span<double> derivArray = stackalloc double[12];
        CalculateDerivativeLocal(localParam, 2, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        int shift = dim << 1;
        point = new Point2D(derivArray[0], derivArray[1]);
        tangent = new Vector2D(derivArray[dim], derivArray[dim + 1]);
        curvature = new Vector2D(derivArray[shift], derivArray[shift + 1]);
    }

    /// <summary>The 2D point and first three derivatives at the local parameter (Param - SpanStart) / SpanLength.</summary>
    public void D3Local(double localParam, out Point2D point, out Vector2D tangent, out Vector2D curvature,
                        out Vector2D torsion)
    {
        Span<double> derivArray = stackalloc double[16];
        CalculateDerivativeLocal(localParam, 3, derivArray);

        int dim = _isRational ? _rowLength - 1 : _rowLength;
        int shift2 = dim << 1;
        int shift3 = shift2 + dim;
        point = new Point2D(derivArray[0], derivArray[1]);
        tangent = new Vector2D(derivArray[dim], derivArray[dim + 1]);
        curvature = new Vector2D(derivArray[shift2], derivArray[shift2 + 1]);
        torsion = new Vector2D(derivArray[shift3], derivArray[shift3 + 1]);
    }
}
