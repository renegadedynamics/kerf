// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A tensor-product Algebraic-Hyperbolic-Trigonometric Bezier surface, over the mixed basis
/// {1, t, ..., t^k, sinh(alpha*t), cosh(alpha*t), sin(beta*t), cos(beta*t)} in each direction, with separate
/// parameters (algDegreeU, alphaU, betaU) and (algDegreeV, alphaV, betaV); a hyperbolic or trigonometric pair is
/// present when its frequency is &gt; 0. The poles grid has one row per U basis function and one column per V
/// basis function. The domain is [0, 1] x [0, 1]. Reversal, transformation and iso-curves are not supported
/// (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class AhtBezierSurface : BoundedSurface
{
    // The number of basis functions for the given parameters.
    private static int BasisDimension(int algDegree, double alpha, double beta) =>
        algDegree + 1 + (alpha > 0.0 ? 2 : 0) + (beta > 0.0 ? 2 : 0);

    private struct PolySurfaceDerivData
    {
        public Xyz S00;
        public Xyz S10;
        public Xyz S01;
        public Xyz S20;
        public Xyz S02;
        public Xyz S11;
        public Xyz S30;
        public Xyz S03;
        public Xyz S21;
        public Xyz S12;
    }

    private static Xyz CubicVal(in Xyz c0, in Xyz c1, in Xyz c2, in Xyz c3, double t)
    {
        return ((c3 * t + c2) * t + c1) * t + c0;
    }

    private static Xyz CubicD1(in Xyz c0, in Xyz c1, in Xyz c2, in Xyz c3, double t)
    {
        return (c3 * (3.0 * t) + c2 * 2.0) * t + c1;
    }

    private static Xyz CubicD2(in Xyz c0, in Xyz c1, in Xyz c2, in Xyz c3, double t)
    {
        return c3 * (6.0 * t) + c2 * 2.0;
    }

    private static Xyz CubicD3(in Xyz c0, in Xyz c1, in Xyz c2, in Xyz c3)
    {
        return c3 * 6.0;
    }

    // The C++ template evalPolynomialSurfaceCubicNonRational<theMaxOrder>: the `if constexpr` are plain tests.
    private static void EvalPolynomialSurfaceCubicNonRational(int maxOrder,
                                                              NCollection_Array2<Point3D> poles,
                                                              double u,
                                                              double v,
                                                              out PolySurfaceDerivData data)
    {
        data.S00 = new Xyz(0.0, 0.0, 0.0);
        data.S10 = new Xyz(0.0, 0.0, 0.0);
        data.S01 = new Xyz(0.0, 0.0, 0.0);
        data.S20 = new Xyz(0.0, 0.0, 0.0);
        data.S02 = new Xyz(0.0, 0.0, 0.0);
        data.S11 = new Xyz(0.0, 0.0, 0.0);
        data.S30 = new Xyz(0.0, 0.0, 0.0);
        data.S03 = new Xyz(0.0, 0.0, 0.0);
        data.S21 = new Xyz(0.0, 0.0, 0.0);
        data.S12 = new Xyz(0.0, 0.0, 0.0);

        int lr = poles.LowerRow();
        int lc = poles.LowerCol();

        Xyz p00 = poles.Value(lr + 0, lc + 0).XYZ();
        Xyz p01 = poles.Value(lr + 0, lc + 1).XYZ();
        Xyz p02 = poles.Value(lr + 0, lc + 2).XYZ();
        Xyz p03 = poles.Value(lr + 0, lc + 3).XYZ();
        Xyz p10 = poles.Value(lr + 1, lc + 0).XYZ();
        Xyz p11 = poles.Value(lr + 1, lc + 1).XYZ();
        Xyz p12 = poles.Value(lr + 1, lc + 2).XYZ();
        Xyz p13 = poles.Value(lr + 1, lc + 3).XYZ();
        Xyz p20 = poles.Value(lr + 2, lc + 0).XYZ();
        Xyz p21 = poles.Value(lr + 2, lc + 1).XYZ();
        Xyz p22 = poles.Value(lr + 2, lc + 2).XYZ();
        Xyz p23 = poles.Value(lr + 2, lc + 3).XYZ();
        Xyz p30 = poles.Value(lr + 3, lc + 0).XYZ();
        Xyz p31 = poles.Value(lr + 3, lc + 1).XYZ();
        Xyz p32 = poles.Value(lr + 3, lc + 2).XYZ();
        Xyz p33 = poles.Value(lr + 3, lc + 3).XYZ();

        Xyz q00 = CubicVal(p00, p10, p20, p30, u);
        Xyz q01 = CubicVal(p01, p11, p21, p31, u);
        Xyz q02 = CubicVal(p02, p12, p22, p32, u);
        Xyz q03 = CubicVal(p03, p13, p23, p33, u);

        data.S00 = CubicVal(q00, q01, q02, q03, v);
        if (maxOrder >= 1)
        {
            Xyz q10 = CubicD1(p00, p10, p20, p30, u);
            Xyz q11 = CubicD1(p01, p11, p21, p31, u);
            Xyz q12 = CubicD1(p02, p12, p22, p32, u);
            Xyz q13 = CubicD1(p03, p13, p23, p33, u);

            data.S10 = CubicVal(q10, q11, q12, q13, v);
            data.S01 = CubicD1(q00, q01, q02, q03, v);

            if (maxOrder >= 2)
            {
                Xyz q20 = CubicD2(p00, p10, p20, p30, u);
                Xyz q21 = CubicD2(p01, p11, p21, p31, u);
                Xyz q22 = CubicD2(p02, p12, p22, p32, u);
                Xyz q23 = CubicD2(p03, p13, p23, p33, u);

                data.S20 = CubicVal(q20, q21, q22, q23, v);
                data.S11 = CubicD1(q10, q11, q12, q13, v);
                data.S02 = CubicD2(q00, q01, q02, q03, v);

                if (maxOrder >= 3)
                {
                    Xyz q30 = CubicD3(p00, p10, p20, p30);
                    Xyz q31 = CubicD3(p01, p11, p21, p31);
                    Xyz q32 = CubicD3(p02, p12, p22, p32);
                    Xyz q33 = CubicD3(p03, p13, p23, p33);

                    data.S30 = CubicVal(q30, q31, q32, q33, v);
                    data.S21 = CubicD1(q20, q21, q22, q23, v);
                    data.S12 = CubicD2(q10, q11, q12, q13, v);
                    data.S03 = CubicD3(q00, q01, q02, q03);
                }
            }
        }
    }

    // The C++ template evalPolynomialSurfaceNonRational<theMaxOrder>: the `if constexpr` are plain tests.
    private static void EvalPolynomialSurfaceNonRational(int maxOrder,
                                                         NCollection_Array2<Point3D> poles,
                                                         int degreeU,
                                                         int degreeV,
                                                         double u,
                                                         double v,
                                                         out PolySurfaceDerivData data)
    {
        data.S00 = new Xyz(0.0, 0.0, 0.0);
        data.S10 = new Xyz(0.0, 0.0, 0.0);
        data.S01 = new Xyz(0.0, 0.0, 0.0);
        data.S20 = new Xyz(0.0, 0.0, 0.0);
        data.S02 = new Xyz(0.0, 0.0, 0.0);
        data.S11 = new Xyz(0.0, 0.0, 0.0);
        data.S30 = new Xyz(0.0, 0.0, 0.0);
        data.S03 = new Xyz(0.0, 0.0, 0.0);
        data.S21 = new Xyz(0.0, 0.0, 0.0);
        data.S12 = new Xyz(0.0, 0.0, 0.0);

        int lr = poles.LowerRow();
        int lc = poles.LowerCol();

        Xyz r00 = new(0.0, 0.0, 0.0);
        Xyz r01 = new(0.0, 0.0, 0.0);
        Xyz r02 = new(0.0, 0.0, 0.0);
        Xyz r03 = new(0.0, 0.0, 0.0);
        Xyz r10 = new(0.0, 0.0, 0.0);
        Xyz r11 = new(0.0, 0.0, 0.0);
        Xyz r12 = new(0.0, 0.0, 0.0);
        Xyz r20 = new(0.0, 0.0, 0.0);
        Xyz r21 = new(0.0, 0.0, 0.0);
        Xyz r30 = new(0.0, 0.0, 0.0);

        for (int j = degreeV; j >= 0; --j)
        {
            Xyz d0 = poles.Value(lr + degreeU, lc + j).XYZ();
            Xyz d1 = new(0.0, 0.0, 0.0);
            Xyz d2 = new(0.0, 0.0, 0.0);
            Xyz d3 = new(0.0, 0.0, 0.0);

            for (int i = degreeU - 1; i >= 0; --i)
            {
                if (maxOrder >= 3)
                {
                    d3 = d3 * u + d2 * 3.0;
                }
                if (maxOrder >= 2)
                {
                    d2 = d2 * u + d1 * 2.0;
                }
                if (maxOrder >= 1)
                {
                    d1 = d1 * u + d0;
                }
                d0 = d0 * u + poles.Value(lr + i, lc + j).XYZ();
            }

            if (maxOrder >= 3)
            {
                r03 = r03 * v + r02 * 3.0;
            }
            if (maxOrder >= 2)
            {
                r02 = r02 * v + r01 * 2.0;
            }
            if (maxOrder >= 1)
            {
                r01 = r01 * v + r00;
            }
            r00 = r00 * v + d0;

            if (maxOrder >= 1)
            {
                if (maxOrder >= 2)
                {
                    r12 = r12 * v + r11 * 2.0;
                }
                if (maxOrder >= 1)
                {
                    r11 = r11 * v + r10;
                }
                r10 = r10 * v + d1;
            }

            if (maxOrder >= 2)
            {
                if (maxOrder >= 3)
                {
                    r21 = r21 * v + r20;
                }
                r20 = r20 * v + d2;
            }

            if (maxOrder >= 3)
            {
                r30 = r30 * v + d3;
            }
        }

        data.S00 = r00;
        if (maxOrder >= 1)
        {
            data.S01 = r01;
            data.S10 = r10;
        }
        if (maxOrder >= 2)
        {
            data.S02 = r02;
            data.S11 = r11;
            data.S20 = r20;
        }
        if (maxOrder >= 3)
        {
            data.S03 = r03;
            data.S12 = r12;
            data.S21 = r21;
            data.S30 = r30;
        }
    }

    private static ref double AxisVal(Span<double> data, int dim, int order, int idx)
    {
        return ref data[order * dim + idx];
    }

    // theDerivs holds (theMaxOrder + 1) blocks of theDim values, one per derivative order.
    private static void EvalAxisDerivs(double t,
                                       int maxOrder,
                                       int algDegree,
                                       double alpha,
                                       double beta,
                                       int dim,
                                       Span<double> derivs)
    {
        int idx = 0;

        if (t == 0.0)
        {
            double factK = 1.0;
            for (int k = 0; k <= algDegree; ++k)
            {
                if (k > 0)
                {
                    factK *= (double)k;
                }
                for (int d = 0; d <= maxOrder; ++d)
                {
                    AxisVal(derivs, dim, d, idx) = (d == k) ? factK : 0.0;
                }
                ++idx;
            }
        }
        else
        {
            double pow = 1.0;
            for (int k = 0; k <= algDegree; ++k)
            {
                int maxD = Min(k, maxOrder);
                double val = pow;
                for (int d = 0; d <= maxD; ++d)
                {
                    AxisVal(derivs, dim, d, idx) = val;
                    if (d < maxD)
                    {
                        val *= (double)(k - d) / t;
                    }
                }
                for (int d = maxD + 1; d <= maxOrder; ++d)
                {
                    AxisVal(derivs, dim, d, idx) = 0.0;
                }
                pow *= t;
                ++idx;
            }
        }

        if (alpha > 0.0)
        {
            double alphaT = alpha * t;
            double sinh = Math.Sinh(alphaT);
            double cosh = Math.Cosh(alphaT);
            double alphaPow = 1.0;
            for (int d = 0; d <= maxOrder; ++d)
            {
                bool isEven = (d % 2 == 0);
                AxisVal(derivs, dim, d, idx) = alphaPow * (isEven ? sinh : cosh);
                AxisVal(derivs, dim, d, idx + 1) = alphaPow * (isEven ? cosh : sinh);
                alphaPow *= alpha;
            }
            idx += 2;
        }

        if (beta > 0.0)
        {
            double betaT = beta * t;
            double sin = Math.Sin(betaT);
            double cos = Math.Cos(betaT);
            double betaPow = 1.0;
            for (int d = 0; d <= maxOrder; ++d)
            {
                double dSin = 0.0;
                double dCos = 0.0;
                switch (d & 3)
                {
                    case 0:
                        dSin = sin;
                        dCos = cos;
                        break;
                    case 1:
                        dSin = cos;
                        dCos = -sin;
                        break;
                    case 2:
                        dSin = -sin;
                        dCos = -cos;
                        break;
                    case 3:
                        dSin = -cos;
                        dCos = sin;
                        break;
                }
                AxisVal(derivs, dim, d, idx) = betaPow * dSin;
                AxisVal(derivs, dim, d, idx + 1) = betaPow * dCos;
                betaPow *= beta;
            }
        }
    }

    private static int TensorIdx(int du, int dv, int maxV)
    {
        return du * (maxV + 1) + dv;
    }

    // A null theWDerivs (non-rational) in C++ is an empty span here.
    private static void EvalTensorDerivs(NCollection_Array2<Point3D> poles,
                                         NCollection_Array2<double>? weights,
                                         int maxU,
                                         int maxV,
                                         int dimU,
                                         int dimV,
                                         ReadOnlySpan<double> buDerivs,
                                         ReadOnlySpan<double> bvDerivs,
                                         Span<Xyz> nDerivs,
                                         Span<double> wDerivs)
    {
        int tensorSize = (maxU + 1) * (maxV + 1);
        for (int idx = 0; idx < tensorSize; ++idx)
        {
            nDerivs[idx] = new Xyz(0.0, 0.0, 0.0);
            if (!wDerivs.IsEmpty)
            {
                wDerivs[idx] = 0.0;
            }
        }

        int lr = poles.LowerRow();
        int lc = poles.LowerCol();
        int wlr = (weights != null) ? weights.LowerRow() : 0;
        int wlc = (weights != null) ? weights.LowerCol() : 0;

        for (int i = 0; i < dimU; ++i)
        {
            for (int j = 0; j < dimV; ++j)
            {
                Xyz pole = poles.Value(lr + i, lc + j).XYZ();
                double wij = (weights != null) ? weights.Value(wlr + i, wlc + j) : 1.0;
                for (int du = 0; du <= maxU; ++du)
                {
                    double bu = buDerivs[du * dimU + i];
                    for (int dv = 0; dv <= maxV; ++dv)
                    {
                        int aIdx = TensorIdx(du, dv, maxV);
                        double b = bu * bvDerivs[dv * dimV + j];
                        if (weights != null)
                        {
                            double wb = wij * b;
                            nDerivs[aIdx] += pole * wb;
                            wDerivs[aIdx] += wb;
                        }
                        else
                        {
                            nDerivs[aIdx] += pole * b;
                        }
                    }
                }
            }
        }
    }

    private static void ComputeRationalDerivs(ReadOnlySpan<Xyz> nDerivs,
                                              ReadOnlySpan<double> wDerivs,
                                              int maxU,
                                              int maxV,
                                              Span<Xyz> cDerivs)
    {
        double invW = 1.0 / wDerivs[TensorIdx(0, 0, maxV)];
        for (int ord = 0; ord <= maxU + maxV; ++ord)
        {
            int duUpper = Min(ord, maxU);
            for (int du = 0; du <= duUpper; ++du)
            {
                int dv = ord - du;
                if (dv > maxV)
                {
                    continue;
                }

                Xyz sum = nDerivs[TensorIdx(du, dv, maxV)];
                for (int p = 0; p <= du; ++p)
                {
                    double binU = 1.0;
                    for (int m = 0; m < p; ++m)
                    {
                        binU = binU * (double)(du - m) / (double)(m + 1);
                    }
                    for (int q = 0; q <= dv; ++q)
                    {
                        if (p == 0 && q == 0)
                        {
                            continue;
                        }
                        double binV = 1.0;
                        for (int m = 0; m < q; ++m)
                        {
                            binV = binV * (double)(dv - m) / (double)(m + 1);
                        }
                        double coeff = binU * binV * wDerivs[TensorIdx(p, q, maxV)];
                        sum -= cDerivs[TensorIdx(du - p, dv - q, maxV)] * coeff;
                    }
                }
                cDerivs[TensorIdx(du, dv, maxV)] = sum * invW;
            }
        }
    }

    private NCollection_Array2<Point3D> _poles;
    private NCollection_Array2<double> _weights;
    private int _algDegreeU;
    private int _algDegreeV;
    private double _alphaU;
    private double _alphaV;
    private double _betaU;
    private double _betaV;
    private bool _uRational;
    private bool _vRational;

    /// <summary>A non-rational AHT-Bezier surface.</summary>
    /// <param name="poles">The control points: one row per U basis function, one column per V basis function.</param>
    /// <param name="algDegreeU">The algebraic polynomial degree in U (&gt;= 0).</param>
    /// <param name="algDegreeV">The algebraic polynomial degree in V (&gt;= 0).</param>
    /// <param name="alphaU">The hyperbolic frequency in U (&gt;= 0; 0 leaves out the hyperbolic pair).</param>
    /// <param name="alphaV">The hyperbolic frequency in V (&gt;= 0).</param>
    /// <param name="betaU">The trigonometric frequency in U (&gt;= 0; 0 leaves out the trigonometric pair).</param>
    /// <param name="betaV">The trigonometric frequency in V (&gt;= 0).</param>
    /// <exception cref="ConstructionErrorException">A parameter is negative, or the poles don't match the bases.</exception>
    public AhtBezierSurface(NCollection_Array2<Point3D> poles,
                                     int algDegreeU,
                                     int algDegreeV,
                                     double alphaU,
                                     double alphaV,
                                     double betaU,
                                     double betaV)
    {
        _poles = new NCollection_Array2<Point3D>(poles);
        // OCCT: a 1 x 1 weights array, left uninitialised (0 here), which Weights() returns.
        _weights = new NCollection_Array2<double>(1, 1, 1, 1);
        _algDegreeU = algDegreeU;
        _algDegreeV = algDegreeV;
        _alphaU = alphaU;
        _alphaV = alphaV;
        _betaU = betaU;
        _betaV = betaV;
        _uRational = false;
        _vRational = false;
        if (algDegreeU < 0 || algDegreeV < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: algebraic degree must be >= 0");
        }
        if (alphaU < 0.0 || alphaV < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: alpha must be >= 0");
        }
        if (betaU < 0.0 || betaV < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: beta must be >= 0");
        }
        int dimU = BasisDimension(algDegreeU, alphaU, betaU);
        int dimV = BasisDimension(algDegreeV, alphaV, betaV);
        if (poles.NbRows() != dimU || poles.NbColumns() != dimV)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierSurface: poles array dimensions must match basis dimensions");
        }
    }

    /// <summary>A rational AHT-Bezier surface.</summary>
    /// <param name="poles">The control points: one row per U basis function, one column per V basis function.</param>
    /// <param name="weights">The weights, with the dimensions of the poles, all &gt; 0.</param>
    /// <param name="algDegreeU">The algebraic polynomial degree in U (&gt;= 0).</param>
    /// <param name="algDegreeV">The algebraic polynomial degree in V (&gt;= 0).</param>
    /// <param name="alphaU">The hyperbolic frequency in U (&gt;= 0).</param>
    /// <param name="alphaV">The hyperbolic frequency in V (&gt;= 0).</param>
    /// <param name="betaU">The trigonometric frequency in U (&gt;= 0).</param>
    /// <param name="betaV">The trigonometric frequency in V (&gt;= 0).</param>
    /// <exception cref="ConstructionErrorException">The validation fails.</exception>
    public AhtBezierSurface(NCollection_Array2<Point3D> poles,
                                     NCollection_Array2<double> weights,
                                     int algDegreeU,
                                     int algDegreeV,
                                     double alphaU,
                                     double alphaV,
                                     double betaU,
                                     double betaV)
    {
        _poles = new NCollection_Array2<Point3D>(poles);
        _weights = new NCollection_Array2<double>(weights);
        _algDegreeU = algDegreeU;
        _algDegreeV = algDegreeV;
        _alphaU = alphaU;
        _alphaV = alphaV;
        _betaU = betaU;
        _betaV = betaV;
        _uRational = true;
        _vRational = true;
        if (algDegreeU < 0 || algDegreeV < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: algebraic degree must be >= 0");
        }
        if (alphaU < 0.0 || alphaV < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: alpha must be >= 0");
        }
        if (betaU < 0.0 || betaV < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierSurface: beta must be >= 0");
        }
        int dimU = BasisDimension(algDegreeU, alphaU, betaU);
        int dimV = BasisDimension(algDegreeV, alphaV, betaV);
        if (poles.NbRows() != dimU || poles.NbColumns() != dimV)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierSurface: poles array dimensions must match basis dimensions");
        }
        if (weights.NbRows() != poles.NbRows() || weights.NbColumns() != poles.NbColumns())
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierSurface: weights array dimensions must match poles");
        }
        for (int i = weights.LowerRow(); i <= weights.UpperRow(); ++i)
        {
            for (int j = weights.LowerCol(); j <= weights.UpperCol(); ++j)
            {
                if (weights.Value(i, j) <= 0.0)
                {
                    throw new ConstructionErrorException("GeomEval_AHTBezierSurface: all weights must be > 0");
                }
            }
        }
    }

    /// <summary>
    /// The poles grid, row by row: the pole of (0-based) U index i and V index j is at <c>i * NbPolesV() + j</c>.
    /// </summary>
    public IReadOnlyList<Point3D> Poles() => ImmutableArrays.Copy<Point3D>(_poles.AsSpan());

    /// <summary>
    /// The weights grid, laid out as <see cref="Poles"/>. For a non-rational surface, OCCT's single unset weight
    /// (0 here).
    /// </summary>
    public IReadOnlyList<double> Weights() => ImmutableArrays.Copy<double>(_weights.AsSpan());

    /// <summary>The algebraic polynomial degree in U.</summary>
    public int AlgDegreeU() => _algDegreeU;

    /// <summary>The algebraic polynomial degree in V.</summary>
    public int AlgDegreeV() => _algDegreeV;

    /// <summary>The hyperbolic frequency in U.</summary>
    public double AlphaU() => _alphaU;

    /// <summary>The hyperbolic frequency in V.</summary>
    public double AlphaV() => _alphaV;

    /// <summary>The trigonometric frequency in U.</summary>
    public double BetaU() => _betaU;

    /// <summary>The trigonometric frequency in V.</summary>
    public double BetaV() => _betaV;

    /// <summary>The number of poles in the U direction.</summary>
    public int PolesUCount => _poles.NbRows();

    /// <summary>The number of poles in the V direction.</summary>
    public int PolesVCount => _poles.NbColumns();

    /// <summary>True if the surface is rational in the U direction.</summary>
    public bool IsURational => _uRational;

    /// <summary>True if the surface is rational in the V direction.</summary>
    public bool IsVRational => _vRational;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::VReversedParameter");
    }

    /// <summary>[0, 1] x [0, 1].</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 1.0;
        v1 = 0.0;
        v2 = 1.0;
    }

    public override bool IsUClosed => false;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => false;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::VIso");
    }

    public override Continuity Continuity => CN;

    internal override bool IsCNu(int n) => true;

    internal override bool IsCNv(int n) => true;

    internal override Point3D EvalD0(double u, double v)
    {
        int dimU = PolesUCount;
        int dimV = PolesVCount;
        bool isRational = (_uRational || _vRational);
        bool isPurePolynomial =
            !isRational && _alphaU <= 0.0 && _betaU <= 0.0 && _alphaV <= 0.0 && _betaV <= 0.0;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData polyData;
            if (_algDegreeU == 3 && _algDegreeV == 3)
            {
                EvalPolynomialSurfaceCubicNonRational(0, _poles, u, v, out polyData);
            }
            else
            {
                EvalPolynomialSurfaceNonRational(0, _poles, _algDegreeU, _algDegreeV, u, v, out polyData);
            }
            return new Point3D(polyData.S00);
        }

        Span<double> buDerivs = dimU <= 16 ? stackalloc double[16] : new double[dimU];
        Span<double> bvDerivs = dimV <= 16 ? stackalloc double[16] : new double[dimV];
        EvalAxisDerivs(u, 0, _algDegreeU, _alphaU, _betaU, dimU, buDerivs);
        EvalAxisDerivs(v, 0, _algDegreeV, _alphaV, _betaV, dimV, bvDerivs);

        Span<Xyz> n = stackalloc Xyz[1];
        Span<double> w = stackalloc double[1];
        EvalTensorDerivs(_poles,
                         isRational ? _weights : null,
                         0,
                         0,
                         dimU,
                         dimV,
                         buDerivs,
                         bvDerivs,
                         n,
                         isRational ? w : default);
        if (!isRational)
        {
            return new Point3D(n[0]);
        }
        return new Point3D(n[0] / w[0]);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        int dimU = PolesUCount;
        int dimV = PolesVCount;
        bool isRational = (_uRational || _vRational);
        bool isPurePolynomial =
            !isRational && _alphaU <= 0.0 && _betaU <= 0.0 && _alphaV <= 0.0 && _betaV <= 0.0;
        ResD1 result;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData polyData;
            if (_algDegreeU == 3 && _algDegreeV == 3)
            {
                EvalPolynomialSurfaceCubicNonRational(1, _poles, u, v, out polyData);
            }
            else
            {
                EvalPolynomialSurfaceNonRational(1, _poles, _algDegreeU, _algDegreeV, u, v, out polyData);
            }
            result.Point = new Point3D(polyData.S00);
            result.D1U = new Vector3D(polyData.S10);
            result.D1V = new Vector3D(polyData.S01);
            return result;
        }

        Span<double> buDerivs = (1 + 1) * dimU <= 16 ? stackalloc double[16] : new double[(1 + 1) * dimU];
        Span<double> bvDerivs = (1 + 1) * dimV <= 16 ? stackalloc double[16] : new double[(1 + 1) * dimV];
        EvalAxisDerivs(u, 1, _algDegreeU, _alphaU, _betaU, dimU, buDerivs);
        EvalAxisDerivs(v, 1, _algDegreeV, _alphaV, _betaV, dimV, bvDerivs);

        Span<Xyz> n = stackalloc Xyz[(1 + 1) * (1 + 1)];
        Span<double> w = stackalloc double[(1 + 1) * (1 + 1)];
        EvalTensorDerivs(_poles,
                         isRational ? _weights : null,
                         1,
                         1,
                         dimU,
                         dimV,
                         buDerivs,
                         bvDerivs,
                         n,
                         isRational ? w : default);

        if (!isRational)
        {
            result.Point = new Point3D(n[TensorIdx(0, 0, 1)]);
            result.D1U = new Vector3D(n[TensorIdx(1, 0, 1)]);
            result.D1V = new Vector3D(n[TensorIdx(0, 1, 1)]);
            return result;
        }

        double invW00 = 1.0 / w[TensorIdx(0, 0, 1)];
        Xyz c00 = n[TensorIdx(0, 0, 1)] * invW00;
        Xyz c10 = (n[TensorIdx(1, 0, 1)] - c00 * w[TensorIdx(1, 0, 1)]) * invW00;
        Xyz c01 = (n[TensorIdx(0, 1, 1)] - c00 * w[TensorIdx(0, 1, 1)]) * invW00;

        result.Point = new Point3D(c00);
        result.D1U = new Vector3D(c10);
        result.D1V = new Vector3D(c01);
        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        int dimU = PolesUCount;
        int dimV = PolesVCount;
        bool isRational = (_uRational || _vRational);
        bool isPurePolynomial =
            !isRational && _alphaU <= 0.0 && _betaU <= 0.0 && _alphaV <= 0.0 && _betaV <= 0.0;
        ResD2 result;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData polyData;
            if (_algDegreeU == 3 && _algDegreeV == 3)
            {
                EvalPolynomialSurfaceCubicNonRational(2, _poles, u, v, out polyData);
            }
            else
            {
                EvalPolynomialSurfaceNonRational(2, _poles, _algDegreeU, _algDegreeV, u, v, out polyData);
            }
            result.Point = new Point3D(polyData.S00);
            result.D1U = new Vector3D(polyData.S10);
            result.D1V = new Vector3D(polyData.S01);
            result.D2U = new Vector3D(polyData.S20);
            result.D2V = new Vector3D(polyData.S02);
            result.D2UV = new Vector3D(polyData.S11);
            return result;
        }

        Span<double> buDerivs = (2 + 1) * dimU <= 16 ? stackalloc double[16] : new double[(2 + 1) * dimU];
        Span<double> bvDerivs = (2 + 1) * dimV <= 16 ? stackalloc double[16] : new double[(2 + 1) * dimV];
        EvalAxisDerivs(u, 2, _algDegreeU, _alphaU, _betaU, dimU, buDerivs);
        EvalAxisDerivs(v, 2, _algDegreeV, _alphaV, _betaV, dimV, bvDerivs);

        Span<Xyz> n = stackalloc Xyz[(2 + 1) * (2 + 1)];
        Span<double> w = stackalloc double[(2 + 1) * (2 + 1)];
        EvalTensorDerivs(_poles,
                         isRational ? _weights : null,
                         2,
                         2,
                         dimU,
                         dimV,
                         buDerivs,
                         bvDerivs,
                         n,
                         isRational ? w : default);

        if (!isRational)
        {
            result.Point = new Point3D(n[TensorIdx(0, 0, 2)]);
            result.D1U = new Vector3D(n[TensorIdx(1, 0, 2)]);
            result.D1V = new Vector3D(n[TensorIdx(0, 1, 2)]);
            result.D2U = new Vector3D(n[TensorIdx(2, 0, 2)]);
            result.D2V = new Vector3D(n[TensorIdx(0, 2, 2)]);
            result.D2UV = new Vector3D(n[TensorIdx(1, 1, 2)]);
            return result;
        }

        double invW00 = 1.0 / w[TensorIdx(0, 0, 2)];
        Xyz c00 = n[TensorIdx(0, 0, 2)] * invW00;
        Xyz c10 = (n[TensorIdx(1, 0, 2)] - c00 * w[TensorIdx(1, 0, 2)]) * invW00;
        Xyz c01 = (n[TensorIdx(0, 1, 2)] - c00 * w[TensorIdx(0, 1, 2)]) * invW00;
        Xyz c20 =
            (n[TensorIdx(2, 0, 2)] - c10 * (2.0 * w[TensorIdx(1, 0, 2)]) - c00 * w[TensorIdx(2, 0, 2)])
            * invW00;
        Xyz c02 =
            (n[TensorIdx(0, 2, 2)] - c01 * (2.0 * w[TensorIdx(0, 1, 2)]) - c00 * w[TensorIdx(0, 2, 2)])
            * invW00;
        Xyz c11 = (n[TensorIdx(1, 1, 2)] - c10 * w[TensorIdx(0, 1, 2)]
                       - c01 * w[TensorIdx(1, 0, 2)] - c00 * w[TensorIdx(1, 1, 2)])
                      * invW00;

        result.Point = new Point3D(c00);
        result.D1U = new Vector3D(c10);
        result.D1V = new Vector3D(c01);
        result.D2U = new Vector3D(c20);
        result.D2V = new Vector3D(c02);
        result.D2UV = new Vector3D(c11);
        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        int dimU = PolesUCount;
        int dimV = PolesVCount;
        bool isRational = (_uRational || _vRational);
        bool isPurePolynomial =
            !isRational && _alphaU <= 0.0 && _betaU <= 0.0 && _alphaV <= 0.0 && _betaV <= 0.0;
        ResD3 result;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData polyData;
            if (_algDegreeU == 3 && _algDegreeV == 3)
            {
                EvalPolynomialSurfaceCubicNonRational(3, _poles, u, v, out polyData);
            }
            else
            {
                EvalPolynomialSurfaceNonRational(3, _poles, _algDegreeU, _algDegreeV, u, v, out polyData);
            }
            result.Point = new Point3D(polyData.S00);
            result.D1U = new Vector3D(polyData.S10);
            result.D1V = new Vector3D(polyData.S01);
            result.D2U = new Vector3D(polyData.S20);
            result.D2V = new Vector3D(polyData.S02);
            result.D2UV = new Vector3D(polyData.S11);
            result.D3U = new Vector3D(polyData.S30);
            result.D3V = new Vector3D(polyData.S03);
            result.D3UUV = new Vector3D(polyData.S21);
            result.D3UVV = new Vector3D(polyData.S12);
            return result;
        }

        Span<double> buDerivs = (3 + 1) * dimU <= 16 ? stackalloc double[16] : new double[(3 + 1) * dimU];
        Span<double> bvDerivs = (3 + 1) * dimV <= 16 ? stackalloc double[16] : new double[(3 + 1) * dimV];
        EvalAxisDerivs(u, 3, _algDegreeU, _alphaU, _betaU, dimU, buDerivs);
        EvalAxisDerivs(v, 3, _algDegreeV, _alphaV, _betaV, dimV, bvDerivs);

        Span<Xyz> n = stackalloc Xyz[(3 + 1) * (3 + 1)];
        Span<double> w = stackalloc double[(3 + 1) * (3 + 1)];
        EvalTensorDerivs(_poles,
                         isRational ? _weights : null,
                         3,
                         3,
                         dimU,
                         dimV,
                         buDerivs,
                         bvDerivs,
                         n,
                         isRational ? w : default);

        if (!isRational)
        {
            result.Point = new Point3D(n[TensorIdx(0, 0, 3)]);
            result.D1U = new Vector3D(n[TensorIdx(1, 0, 3)]);
            result.D1V = new Vector3D(n[TensorIdx(0, 1, 3)]);
            result.D2U = new Vector3D(n[TensorIdx(2, 0, 3)]);
            result.D2V = new Vector3D(n[TensorIdx(0, 2, 3)]);
            result.D2UV = new Vector3D(n[TensorIdx(1, 1, 3)]);
            result.D3U = new Vector3D(n[TensorIdx(3, 0, 3)]);
            result.D3V = new Vector3D(n[TensorIdx(0, 3, 3)]);
            result.D3UUV = new Vector3D(n[TensorIdx(2, 1, 3)]);
            result.D3UVV = new Vector3D(n[TensorIdx(1, 2, 3)]);
            return result;
        }

        double invW00 = 1.0 / w[TensorIdx(0, 0, 3)];
        Xyz c00 = n[TensorIdx(0, 0, 3)] * invW00;
        Xyz c10 = (n[TensorIdx(1, 0, 3)] - c00 * w[TensorIdx(1, 0, 3)]) * invW00;
        Xyz c01 = (n[TensorIdx(0, 1, 3)] - c00 * w[TensorIdx(0, 1, 3)]) * invW00;
        Xyz c20 =
            (n[TensorIdx(2, 0, 3)] - c10 * (2.0 * w[TensorIdx(1, 0, 3)]) - c00 * w[TensorIdx(2, 0, 3)])
            * invW00;
        Xyz c02 =
            (n[TensorIdx(0, 2, 3)] - c01 * (2.0 * w[TensorIdx(0, 1, 3)]) - c00 * w[TensorIdx(0, 2, 3)])
            * invW00;
        Xyz c11 = (n[TensorIdx(1, 1, 3)] - c10 * w[TensorIdx(0, 1, 3)]
                       - c01 * w[TensorIdx(1, 0, 3)] - c00 * w[TensorIdx(1, 1, 3)])
                      * invW00;
        Xyz c30 = (n[TensorIdx(3, 0, 3)] - c20 * (3.0 * w[TensorIdx(1, 0, 3)])
                       - c10 * (3.0 * w[TensorIdx(2, 0, 3)]) - c00 * w[TensorIdx(3, 0, 3)])
                      * invW00;
        Xyz c03 = (n[TensorIdx(0, 3, 3)] - c02 * (3.0 * w[TensorIdx(0, 1, 3)])
                       - c01 * (3.0 * w[TensorIdx(0, 2, 3)]) - c00 * w[TensorIdx(0, 3, 3)])
                      * invW00;
        Xyz c21 = (n[TensorIdx(2, 1, 3)] - c11 * (2.0 * w[TensorIdx(1, 0, 3)])
                       - c20 * w[TensorIdx(0, 1, 3)] - c01 * w[TensorIdx(2, 0, 3)]
                       - c10 * (2.0 * w[TensorIdx(1, 1, 3)]) - c00 * w[TensorIdx(2, 1, 3)])
                      * invW00;
        Xyz c12 = (n[TensorIdx(1, 2, 3)] - c11 * (2.0 * w[TensorIdx(0, 1, 3)])
                       - c02 * w[TensorIdx(1, 0, 3)] - c10 * w[TensorIdx(0, 2, 3)]
                       - c01 * (2.0 * w[TensorIdx(1, 1, 3)]) - c00 * w[TensorIdx(1, 2, 3)])
                      * invW00;

        result.Point = new Point3D(c00);
        result.D1U = new Vector3D(c10);
        result.D1V = new Vector3D(c01);
        result.D2U = new Vector3D(c20);
        result.D2V = new Vector3D(c02);
        result.D2UV = new Vector3D(c11);
        result.D3U = new Vector3D(c30);
        result.D3V = new Vector3D(c03);
        result.D3UUV = new Vector3D(c21);
        result.D3UVV = new Vector3D(c12);
        return result;
    }

    /// <exception cref="RangeErrorException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new RangeErrorException("GeomEval_AHTBezierSurface::EvalDN: invalid derivative order");
        }

        int totOrd = nu + nv;
        if (totOrd <= 3)
        {
            if (totOrd == 1)
            {
                ResD1 d1 = EvalD1(u, v);
                return (nu == 1) ? d1.D1U : d1.D1V;
            }
            if (totOrd == 2)
            {
                ResD2 d2 = EvalD2(u, v);
                if (nu == 2)
                {
                    return d2.D2U;
                }
                if (nv == 2)
                {
                    return d2.D2V;
                }
                return d2.D2UV;
            }
            ResD3 d3 = EvalD3(u, v);
            if (nu == 3)
            {
                return d3.D3U;
            }
            if (nv == 3)
            {
                return d3.D3V;
            }
            return (nu == 2) ? d3.D3UUV : d3.D3UVV;
        }

        int dimU = PolesUCount;
        int dimV = PolesVCount;
        Span<double> buDerivs = (nu + 1) * dimU <= 16 ? stackalloc double[16] : new double[(nu + 1) * dimU];
        Span<double> bvDerivs = (nv + 1) * dimV <= 16 ? stackalloc double[16] : new double[(nv + 1) * dimV];
        EvalAxisDerivs(u, nu, _algDegreeU, _alphaU, _betaU, dimU, buDerivs);
        EvalAxisDerivs(v, nv, _algDegreeV, _alphaV, _betaV, dimV, bvDerivs);

        bool isRational = (_uRational || _vRational);
        if (!isRational)
        {
            Xyz sum = new(0.0, 0.0, 0.0);
            int lr = _poles.LowerRow();
            int lc = _poles.LowerCol();
            for (int i = 0; i < dimU; ++i)
            {
                for (int j = 0; j < dimV; ++j)
                {
                    sum += _poles.Value(lr + i, lc + j).XYZ()
                            * (buDerivs[nu * dimU + i] * bvDerivs[nv * dimV + j]);
                }
            }
            return new Vector3D(sum);
        }

        int tensorSize = (nu + 1) * (nv + 1);
        Span<Xyz> nDerivs = tensorSize <= 16 ? stackalloc Xyz[16] : new Xyz[tensorSize];
        Span<double> wDerivs = tensorSize <= 16 ? stackalloc double[16] : new double[tensorSize];
        Span<Xyz> cDerivs = tensorSize <= 16 ? stackalloc Xyz[16] : new Xyz[tensorSize];
        EvalTensorDerivs(_poles,
                         _weights,
                         nu,
                         nv,
                         dimU,
                         dimV,
                         buDerivs,
                         bvDerivs,
                         nDerivs,
                         wDerivs);
        ComputeRationalDerivs(nDerivs, wDerivs, nu, nv, cDerivs);
        return new Vector3D(cDerivs[TensorIdx(nu, nv, nv)]);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierSurface::Transform");
    }

    public override Geometry3D Copy()
    {
        if (_uRational || _vRational)
        {
            return new AhtBezierSurface(_poles,
                                                 _weights,
                                                 _algDegreeU,
                                                 _algDegreeV,
                                                 _alphaU,
                                                 _alphaV,
                                                 _betaU,
                                                 _betaV);
        }
        return new AhtBezierSurface(_poles,
                                             _algDegreeU,
                                             _algDegreeV,
                                             _alphaU,
                                             _alphaV,
                                             _betaU,
                                             _betaV);
    }
}
