// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A tensor-product trigonometric Bezier surface, over the basis {1, sin(alpha*t), cos(alpha*t), ...,
/// sin(n*alpha*t), cos(n*alpha*t)} in each direction: <c>S(u,v) = sum_i sum_j P_ij * Bu_i(u) * Bv_j(v)</c>, or
/// for a rational surface <c>sum(w_ij*P_ij*Bu_i*Bv_j) / sum(w_ij*Bu_i*Bv_j)</c>. The poles form a
/// (2*nU + 1) x (2*nV + 1) grid (rows are U, columns V); the domain is [0, Pi/alphaU] x [0, Pi/alphaV].
/// Reversal, transformation and iso-curves are not supported (they throw <see cref="MethodNotImplementedException"/>),
/// nor are rational derivatives of order above 3 (<see cref="UndefinedDerivative3DException"/>).
/// </summary>
internal sealed class TBezierSurface : BoundedSurface
{
    // Validate that a dimension count is odd and >= 3.
    private static void ValidateDimension(int size, string name)
    {
        if (size < 3 || (size % 2) == 0)
        {
            string msg = "GeomEval_TBezierSurface: ";
            msg += name;
            msg += " pole count must be odd and >= 3";
            throw new ConstructionErrorException(msg);
        }
    }

    private static double PowInt(double value, int power)
    {
        double res = 1.0;
        for (int i = 0; i < power; ++i)
        {
            res *= value;
        }
        return res;
    }

    private static void EvalTrigPairNthDeriv(double sin, double cos, int orderMod4,
                                             out double sinDeriv, out double cosDeriv)
    {
        switch (orderMod4)
        {
            case 0:
                sinDeriv = sin;
                cosDeriv = cos;
                break;
            case 1:
                sinDeriv = cos;
                cosDeriv = -sin;
                break;
            case 2:
                sinDeriv = -sin;
                cosDeriv = -cos;
                break;
            default:
                sinDeriv = -cos;
                cosDeriv = sin;
                break;
        }
    }

    // The C++ template evalTrigAxisDerivs<theMaxOrder>: the `if constexpr` on the order are plain tests here.
    // theDerivs holds (theMaxOrder + 1) blocks of 2 * theOrder + 1 values, one per derivative order.
    private static void EvalTrigAxisDerivs(int maxOrder, double t, double alpha, int order,
                                           Span<double> derivs)
    {
        int dim = 2 * order + 1;
        for (int d = 0; d <= maxOrder; ++d)
        {
            derivs[d * dim] = (d == 0) ? 1.0 : 0.0;
        }

        int idx = 1;
        for (int k = 1; k <= order; ++k, idx += 2)
        {
            double freq = (double)k * alpha;
            double arg = freq * t;
            double sin = Math.Sin(arg);
            double cos = Math.Cos(arg);
            double freq2 = freq * freq;
            double freq3 = freq2 * freq;

            derivs[idx] = sin;
            derivs[idx + 1] = cos;
            if (maxOrder >= 1)
            {
                derivs[dim + idx] = freq * cos;
                derivs[dim + idx + 1] = -freq * sin;
            }
            if (maxOrder >= 2)
            {
                derivs[2 * dim + idx] = -freq2 * sin;
                derivs[2 * dim + idx + 1] = -freq2 * cos;
            }
            if (maxOrder >= 3)
            {
                derivs[3 * dim + idx] = -freq3 * cos;
                derivs[3 * dim + idx + 1] = freq3 * sin;
            }
        }
    }

    private static void EvalTrigAxisNthDeriv(double t, double alpha, int order, int derivOrder,
                                             Span<double> derivs)
    {
        derivs[0] = (derivOrder == 0) ? 1.0 : 0.0;
        if (derivOrder == 0)
        {
            int idx0 = 1;
            for (int k = 1; k <= order; ++k, idx0 += 2)
            {
                double arg = (double)k * alpha * t;
                derivs[idx0] = Math.Sin(arg);
                derivs[idx0 + 1] = Math.Cos(arg);
            }
            return;
        }

        int orderMod4 = derivOrder & 3;
        int idx = 1;
        for (int k = 1; k <= order; ++k, idx += 2)
        {
            double freq = (double)k * alpha;
            double arg = freq * t;
            double sin = Math.Sin(arg);
            double cos = Math.Cos(arg);
            double pow = PowInt(freq, derivOrder);
            EvalTrigPairNthDeriv(sin, cos, orderMod4, out double sinDeriv, out double cosDeriv);
            derivs[idx] = pow * sinDeriv;
            derivs[idx + 1] = pow * cosDeriv;
        }
    }

    private NCollection_Array2<Point3D> _poles;   // Control points grid
    private NCollection_Array2<double> _weights; // Weights grid (empty if non-rational)
    private double _alphaU;                      // Frequency parameter in U
    private double _alphaV;                      // Frequency parameter in V
    private bool _rational;                      // True if surface is rational

    /// <summary>A non-rational T-Bezier surface.</summary>
    /// <param name="poles">The control points; the numbers of rows (U) and columns (V) must be odd and at least 3.</param>
    /// <param name="alphaU">The frequency parameter in U (&gt; 0).</param>
    /// <param name="alphaV">The frequency parameter in V (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">The validation fails.</exception>
    public TBezierSurface(NCollection_Array2<Point3D> poles, double alphaU, double alphaV)
    {
        _poles = new NCollection_Array2<Point3D>(poles);
        _weights = new NCollection_Array2<double>();
        _alphaU = alphaU;
        _alphaV = alphaV;
        _rational = false;
        ValidateDimension(poles.NbRows(), "U");
        ValidateDimension(poles.NbColumns(), "V");
        if (alphaU <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaU must be > 0");
        }
        if (alphaV <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaV must be > 0");
        }
    }

    /// <summary>A rational T-Bezier surface.</summary>
    /// <param name="poles">The control points; the numbers of rows (U) and columns (V) must be odd and at least 3.</param>
    /// <param name="weights">The weights, with the dimensions of the poles, all &gt; 0.</param>
    /// <param name="alphaU">The frequency parameter in U (&gt; 0).</param>
    /// <param name="alphaV">The frequency parameter in V (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">The validation fails.</exception>
    public TBezierSurface(NCollection_Array2<Point3D> poles, NCollection_Array2<double> weights,
                                   double alphaU, double alphaV)
    {
        _poles = new NCollection_Array2<Point3D>(poles);
        _weights = new NCollection_Array2<double>(weights);
        _alphaU = alphaU;
        _alphaV = alphaV;
        _rational = true;
        ValidateDimension(poles.NbRows(), "U");
        ValidateDimension(poles.NbColumns(), "V");
        if (alphaU <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaU must be > 0");
        }
        if (alphaV <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaV must be > 0");
        }
        if (weights.NbRows() != poles.NbRows() || weights.NbColumns() != poles.NbColumns())
        {
            throw new ConstructionErrorException(
                "GeomEval_TBezierSurface: weights dimensions must match poles dimensions");
        }
        for (int i = weights.LowerRow(); i <= weights.UpperRow(); ++i)
        {
            for (int j = weights.LowerCol(); j <= weights.UpperCol(); ++j)
            {
                if (weights.Value(i, j) <= 0.0)
                {
                    throw new ConstructionErrorException("GeomEval_TBezierSurface: all weights must be > 0");
                }
            }
        }
    }

    /// <summary>
    /// The poles grid, row by row: the pole of (0-based) U index i and V index j is at <c>i * NbVPoles() + j</c>.
    /// </summary>
    public IReadOnlyList<Point3D> Poles() => ImmutableArrays.Copy<Point3D>(_poles.AsSpan());

    /// <summary>The weights grid, laid out as <see cref="Poles"/>; empty if the surface is non-rational.</summary>
    public IReadOnlyList<double> Weights() => ImmutableArrays.Copy<double>(_weights.AsSpan());

    /// <summary>The frequency parameter alpha in the U direction.</summary>
    public double AlphaU() => _alphaU;

    /// <summary>The frequency parameter alpha in the V direction.</summary>
    public double AlphaV() => _alphaV;

    /// <summary>The number of poles in the U direction.</summary>
    public int UPoleCount => _poles.NbRows();

    /// <summary>The number of poles in the V direction.</summary>
    public int VPoleCount => _poles.NbColumns();

    /// <summary>The trigonometric order in U (NbUPoles = 2*nU + 1).</summary>
    public int OrderU() => (_poles.NbRows() - 1) / 2;

    /// <summary>The trigonometric order in V (NbVPoles = 2*nV + 1).</summary>
    public int OrderV() => (_poles.NbColumns() - 1) / 2;

    /// <summary>True if the surface is rational.</summary>
    public bool IsRational => _rational;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::VReversedParameter");
    }

    /// <summary>[0, Pi/alphaU] x [0, Pi/alphaV].</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = M_PI / _alphaU;
        v1 = 0.0;
        v2 = M_PI / _alphaV;
    }

    /// <summary>True if the first and last U-rows of poles coincide.</summary>
    public override bool IsUClosed
    {
        get
        {
            // Check if the first and last U-rows of poles coincide.
            // OCCT compares pole rows, but in the trigonometric basis the boundary iso-curves are
            // S(0, v) = sum_j (P_0j + P_2j + P_4j + ...) Bv_j and S(Pi/alphaU, v) = sum_j (P_0j - P_2j + P_4j ...) Bv_j,
            // so the result says nothing about closure (see occt-issues.md).
            int lr = _poles.LowerRow();
            int ur = _poles.UpperRow();
            for (int j = _poles.LowerCol(); j <= _poles.UpperCol(); ++j)
            {
                if (_poles.Value(lr, j).DistanceTo(_poles.Value(ur, j)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return false;
                }
            }
            return true;
        }
    }

    /// <summary>True if the first and last V-columns of poles coincide.</summary>
    public override bool IsVClosed
    {
        get
        {
            // Check if the first and last V-columns of poles coincide.
            // OCCT: the same pole test as IsUClosed, which says nothing about closure (see occt-issues.md).
            int lc = _poles.LowerCol();
            int uc = _poles.UpperCol();
            for (int i = _poles.LowerRow(); i <= _poles.UpperRow(); ++i)
            {
                if (_poles.Value(i, lc).DistanceTo(_poles.Value(i, uc)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return false;
                }
            }
            return true;
        }
    }

    public override bool IsUPeriodic => false;

    public override bool IsVPeriodic => false;

    public override Continuity Continuity => CN;

    internal override bool IsCNu(int n) => true;

    internal override bool IsCNv(int n) => true;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::VIso");
    }

    // Evaluates a single trigonometric basis array for the given alpha. Unused, as in OCCT.
    private static void EvalTrigBasis(double t, double alpha, int order, NCollection_Array1<double> basis)
    {
        // T_0(t) = 1
        basis.SetValue(basis.Lower(), 1.0);
        for (int k = 1; k <= order; ++k)
        {
            double arg = k * alpha * t;
            basis.SetValue(basis.Lower() + 2 * k - 1, Math.Sin(arg));
            basis.SetValue(basis.Lower() + 2 * k, Math.Cos(arg));
        }
    }

    // Evaluates a derivative of the trigonometric basis for the given alpha. Unused, as in OCCT.
    private static void EvalTrigBasisDeriv(double t, double alpha, int order, int derivOrder,
                                           NCollection_Array1<double> basisDeriv)
    {
        int dim = 2 * order + 1;
        Span<double> derivs = dim <= 16 ? stackalloc double[16] : new double[dim];
        EvalTrigAxisNthDeriv(t, alpha, order, derivOrder, derivs);
        for (int i = 0; i < dim; ++i)
        {
            basisDeriv.SetValue(basisDeriv.Lower() + i, derivs[i]);
        }
    }

    private void EvalBasisU(double u, NCollection_Array1<double> basis)
    {
        EvalTrigBasis(u, _alphaU, OrderU(), basis);
    }

    private void EvalBasisV(double v, NCollection_Array1<double> basis)
    {
        EvalTrigBasis(v, _alphaV, OrderV(), basis);
    }

    private void EvalBasisDerivU(double u, int order, NCollection_Array1<double> basisDeriv)
    {
        EvalTrigBasisDeriv(u, _alphaU, OrderU(), order, basisDeriv);
    }

    private void EvalBasisDerivV(double v, int order, NCollection_Array1<double> basisDeriv)
    {
        EvalTrigBasisDeriv(v, _alphaV, OrderV(), order, basisDeriv);
    }

    internal override Point3D EvalD0(double u, double v)
    {
        int nbU = _poles.NbRows();
        int nbV = _poles.NbColumns();
        Span<double> basisU = nbU <= 16 ? stackalloc double[16] : new double[nbU];
        Span<double> basisV = nbV <= 16 ? stackalloc double[16] : new double[nbV];
        EvalTrigAxisDerivs(0, u, _alphaU, (nbU - 1) / 2, basisU);
        EvalTrigAxisDerivs(0, v, _alphaV, (nbV - 1) / 2, basisV);

        if (!_rational)
        {
            Xyz sum = new(0.0, 0.0, 0.0);
            for (int i = 0; i < nbU; ++i)
            {
                double bu = basisU[i];
                for (int j = 0; j < nbV; ++j)
                {
                    sum +=
                        (bu * basisV[j]) * _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                }
            }
            return new Point3D(sum);
        }

        // Rational evaluation.
        Xyz numer = new(0.0, 0.0, 0.0);
        double denom = 0.0;
        for (int i = 0; i < nbU; ++i)
        {
            double bu = basisU[i];
            for (int j = 0; j < nbV; ++j)
            {
                double wBuBv =
                    _weights.Value(_weights.LowerRow() + i, _weights.LowerCol() + j) * bu * basisV[j];
                numer += wBuBv * _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                denom += wBuBv;
            }
        }
        return new Point3D(numer / denom);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        int nbU = _poles.NbRows();
        int nbV = _poles.NbColumns();
        Span<double> buDerivs = (1 + 1) * nbU <= 16 ? stackalloc double[16] : new double[(1 + 1) * nbU];
        Span<double> bvDerivs = (1 + 1) * nbV <= 16 ? stackalloc double[16] : new double[(1 + 1) * nbV];
        EvalTrigAxisDerivs(1, u, _alphaU, (nbU - 1) / 2, buDerivs);
        EvalTrigAxisDerivs(1, v, _alphaV, (nbV - 1) / 2, bvDerivs);

        ReadOnlySpan<double> basisU = buDerivs;
        ReadOnlySpan<double> basisDU = buDerivs[nbU..];
        ReadOnlySpan<double> basisV = bvDerivs;
        ReadOnlySpan<double> basisDV = bvDerivs[nbV..];

        ResD1 result;

        if (!_rational)
        {
            Xyz s = new(0.0, 0.0, 0.0);
            Xyz du = new(0.0, 0.0, 0.0);
            Xyz dv = new(0.0, 0.0, 0.0);
            for (int i = 0; i < nbU; ++i)
            {
                double bu = basisU[i];
                double buD = basisDU[i];
                for (int j = 0; j < nbV; ++j)
                {
                    double bv = basisV[j];
                    double bvD = basisDV[j];
                    Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                    s += (bu * bv) * pij;
                    du += (buD * bv) * pij;
                    dv += (bu * bvD) * pij;
                }
            }
            result.Point = new Point3D(s);
            result.D1U = new Vector3D(du);
            result.D1V = new Vector3D(dv);
            return result;
        }

        // Rational: S = Sw/w, dS/du = (Sw_u - w_u*S)/w, dS/dv = (Sw_v - w_v*S)/w
        Xyz sw = new(0.0, 0.0, 0.0);
        Xyz swDU = new(0.0, 0.0, 0.0);
        Xyz swDV = new(0.0, 0.0, 0.0);
        double w = 0.0;
        double wdu = 0.0;
        double wdv = 0.0;
        for (int i = 0; i < nbU; ++i)
        {
            double bu = basisU[i];
            double buD = basisDU[i];
            for (int j = 0; j < nbV; ++j)
            {
                double bv = basisV[j];
                double bvD = basisDV[j];
                double wij = _weights.Value(_weights.LowerRow() + i, _weights.LowerCol() + j);
                Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                double wBuBv = wij * bu * bv;
                double wBuDBv = wij * buD * bv;
                double wBuBvD = wij * bu * bvD;
                sw += wBuBv * pij;
                swDU += wBuDBv * pij;
                swDV += wBuBvD * pij;
                w += wBuBv;
                wdu += wBuDBv;
                wdv += wBuBvD;
            }
        }

        Xyz sr = sw / w;
        result.Point = new Point3D(sr);
        result.D1U = new Vector3D((swDU - wdu * sr) / w);
        result.D1V = new Vector3D((swDV - wdv * sr) / w);
        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        int nbU = _poles.NbRows();
        int nbV = _poles.NbColumns();
        Span<double> buDerivs = (2 + 1) * nbU <= 16 ? stackalloc double[16] : new double[(2 + 1) * nbU];
        Span<double> bvDerivs = (2 + 1) * nbV <= 16 ? stackalloc double[16] : new double[(2 + 1) * nbV];
        EvalTrigAxisDerivs(2, u, _alphaU, (nbU - 1) / 2, buDerivs);
        EvalTrigAxisDerivs(2, v, _alphaV, (nbV - 1) / 2, bvDerivs);

        ReadOnlySpan<double> basisU = buDerivs;
        ReadOnlySpan<double> basisDU = buDerivs[nbU..];
        ReadOnlySpan<double> basisD2U = buDerivs[(2 * nbU)..];
        ReadOnlySpan<double> basisV = bvDerivs;
        ReadOnlySpan<double> basisDV = bvDerivs[nbV..];
        ReadOnlySpan<double> basisD2V = bvDerivs[(2 * nbV)..];

        ResD2 result;

        if (!_rational)
        {
            Xyz s = new(0.0, 0.0, 0.0);
            Xyz du = new(0.0, 0.0, 0.0);
            Xyz dv = new(0.0, 0.0, 0.0);
            Xyz d2U = new(0.0, 0.0, 0.0);
            Xyz d2V = new(0.0, 0.0, 0.0);
            Xyz d2UV = new(0.0, 0.0, 0.0);
            for (int i = 0; i < nbU; ++i)
            {
                double bu = basisU[i];
                double buD = basisDU[i];
                double buD2 = basisD2U[i];
                for (int j = 0; j < nbV; ++j)
                {
                    double bv = basisV[j];
                    double bvD = basisDV[j];
                    double bvD2 = basisD2V[j];
                    Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                    s += (bu * bv) * pij;
                    du += (buD * bv) * pij;
                    dv += (bu * bvD) * pij;
                    d2U += (buD2 * bv) * pij;
                    d2V += (bu * bvD2) * pij;
                    d2UV += (buD * bvD) * pij;
                }
            }
            result.Point = new Point3D(s);
            result.D1U = new Vector3D(du);
            result.D1V = new Vector3D(dv);
            result.D2U = new Vector3D(d2U);
            result.D2V = new Vector3D(d2V);
            result.D2UV = new Vector3D(d2UV);
            return result;
        }

        // Rational second derivatives using quotient rule.
        Xyz sw = new(0.0, 0.0, 0.0);
        Xyz swDU = new(0.0, 0.0, 0.0);
        Xyz swDV = new(0.0, 0.0, 0.0);
        Xyz swD2U = new(0.0, 0.0, 0.0);
        Xyz swD2V = new(0.0, 0.0, 0.0);
        Xyz swDUV = new(0.0, 0.0, 0.0);
        double w = 0.0;
        double wdu = 0.0;
        double wdv = 0.0;
        double wd2U = 0.0;
        double wd2V = 0.0;
        double wduv = 0.0;
        for (int i = 0; i < nbU; ++i)
        {
            double bu = basisU[i];
            double buD = basisDU[i];
            double buD2 = basisD2U[i];
            for (int j = 0; j < nbV; ++j)
            {
                double bv = basisV[j];
                double bvD = basisDV[j];
                double bvD2 = basisD2V[j];
                double wij = _weights.Value(_weights.LowerRow() + i, _weights.LowerCol() + j);
                Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();

                double wBuBv = wij * bu * bv;
                double wBuDBv = wij * buD * bv;
                double wBuBvD = wij * bu * bvD;
                double wBuD2Bv = wij * buD2 * bv;
                double wBuBvD2 = wij * bu * bvD2;
                double wBuDBvD = wij * buD * bvD;

                sw += wBuBv * pij;
                swDU += wBuDBv * pij;
                swDV += wBuBvD * pij;
                swD2U += wBuD2Bv * pij;
                swD2V += wBuBvD2 * pij;
                swDUV += wBuDBvD * pij;
                w += wBuBv;
                wdu += wBuDBv;
                wdv += wBuBvD;
                wd2U += wBuD2Bv;
                wd2V += wBuBvD2;
                wduv += wBuDBvD;
            }
        }

        // S = Sw / w
        Xyz sr = sw / w;
        // First derivatives: S_u = (Sw_u - w_u * S) / w
        Xyz sd1U = (swDU - wdu * sr) / w;
        Xyz sd1V = (swDV - wdv * sr) / w;
        // Second derivatives: S_uu = (Sw_uu - 2*w_u*S_u - w_uu*S) / w
        Xyz sd2U = (swD2U - 2.0 * wdu * sd1U - wd2U * sr) / w;
        Xyz sd2V = (swD2V - 2.0 * wdv * sd1V - wd2V * sr) / w;
        Xyz sd2UV = (swDUV - wdu * sd1V - wdv * sd1U - wduv * sr) / w;

        result.Point = new Point3D(sr);
        result.D1U = new Vector3D(sd1U);
        result.D1V = new Vector3D(sd1V);
        result.D2U = new Vector3D(sd2U);
        result.D2V = new Vector3D(sd2V);
        result.D2UV = new Vector3D(sd2UV);
        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        int nbU = _poles.NbRows();
        int nbV = _poles.NbColumns();
        Span<double> buDerivs = (3 + 1) * nbU <= 16 ? stackalloc double[16] : new double[(3 + 1) * nbU];
        Span<double> bvDerivs = (3 + 1) * nbV <= 16 ? stackalloc double[16] : new double[(3 + 1) * nbV];
        EvalTrigAxisDerivs(3, u, _alphaU, (nbU - 1) / 2, buDerivs);
        EvalTrigAxisDerivs(3, v, _alphaV, (nbV - 1) / 2, bvDerivs);

        ReadOnlySpan<double> basisU = buDerivs;
        ReadOnlySpan<double> basisDU = buDerivs[nbU..];
        ReadOnlySpan<double> basisD2U = buDerivs[(2 * nbU)..];
        ReadOnlySpan<double> basisD3U = buDerivs[(3 * nbU)..];
        ReadOnlySpan<double> basisV = bvDerivs;
        ReadOnlySpan<double> basisDV = bvDerivs[nbV..];
        ReadOnlySpan<double> basisD2V = bvDerivs[(2 * nbV)..];
        ReadOnlySpan<double> basisD3V = bvDerivs[(3 * nbV)..];

        ResD3 result;

        if (!_rational)
        {
            Xyz s = new(0.0, 0.0, 0.0);
            Xyz du = new(0.0, 0.0, 0.0);
            Xyz dv = new(0.0, 0.0, 0.0);
            Xyz d2U = new(0.0, 0.0, 0.0);
            Xyz d2V = new(0.0, 0.0, 0.0);
            Xyz d2UV = new(0.0, 0.0, 0.0);
            Xyz d3U = new(0.0, 0.0, 0.0);
            Xyz d3V = new(0.0, 0.0, 0.0);
            Xyz d3UUV = new(0.0, 0.0, 0.0);
            Xyz d3UVV = new(0.0, 0.0, 0.0);

            for (int i = 0; i < nbU; ++i)
            {
                double bu = basisU[i];
                double buD = basisDU[i];
                double buD2 = basisD2U[i];
                double buD3 = basisD3U[i];
                for (int j = 0; j < nbV; ++j)
                {
                    double bv = basisV[j];
                    double bvD = basisDV[j];
                    double bvD2 = basisD2V[j];
                    double bvD3 = basisD3V[j];
                    Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();

                    s += (bu * bv) * pij;
                    du += (buD * bv) * pij;
                    dv += (bu * bvD) * pij;
                    d2U += (buD2 * bv) * pij;
                    d2V += (bu * bvD2) * pij;
                    d2UV += (buD * bvD) * pij;
                    d3U += (buD3 * bv) * pij;
                    d3V += (bu * bvD3) * pij;
                    d3UUV += (buD2 * bvD) * pij;
                    d3UVV += (buD * bvD2) * pij;
                }
            }
            result.Point = new Point3D(s);
            result.D1U = new Vector3D(du);
            result.D1V = new Vector3D(dv);
            result.D2U = new Vector3D(d2U);
            result.D2V = new Vector3D(d2V);
            result.D2UV = new Vector3D(d2UV);
            result.D3U = new Vector3D(d3U);
            result.D3V = new Vector3D(d3V);
            result.D3UUV = new Vector3D(d3UUV);
            result.D3UVV = new Vector3D(d3UVV);
            return result;
        }

        // Rational third derivatives: accumulate all weighted sums.
        Xyz sw = new(0.0, 0.0, 0.0);
        Xyz swDU = new(0.0, 0.0, 0.0);
        Xyz swDV = new(0.0, 0.0, 0.0);
        Xyz swD2U = new(0.0, 0.0, 0.0);
        Xyz swD2V = new(0.0, 0.0, 0.0);
        Xyz swDUV = new(0.0, 0.0, 0.0);
        Xyz swD3U = new(0.0, 0.0, 0.0);
        Xyz swD3V = new(0.0, 0.0, 0.0);
        Xyz swD2UDV = new(0.0, 0.0, 0.0);
        Xyz swDUD2V = new(0.0, 0.0, 0.0);
        double w = 0.0;
        double wdu = 0.0;
        double wdv = 0.0;
        double wd2U = 0.0;
        double wd2V = 0.0;
        double wduv = 0.0;
        double wd3U = 0.0;
        double wd3V = 0.0;
        double wd2UDV = 0.0;
        double wdud2V = 0.0;

        for (int i = 0; i < nbU; ++i)
        {
            double bu = basisU[i];
            double buD = basisDU[i];
            double buD2 = basisD2U[i];
            double buD3 = basisD3U[i];
            for (int j = 0; j < nbV; ++j)
            {
                double bv = basisV[j];
                double bvD = basisDV[j];
                double bvD2 = basisD2V[j];
                double bvD3 = basisD3V[j];
                double wij = _weights.Value(_weights.LowerRow() + i, _weights.LowerCol() + j);
                Xyz pij = _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();

                double wBuBv = wij * bu * bv;
                double wBuDBv = wij * buD * bv;
                double wBuBvD = wij * bu * bvD;
                double wBuD2Bv = wij * buD2 * bv;
                double wBuBvD2 = wij * bu * bvD2;
                double wBuDBvD = wij * buD * bvD;
                double wBuD3Bv = wij * buD3 * bv;
                double wBuBvD3 = wij * bu * bvD3;
                double wBuD2BvD = wij * buD2 * bvD;
                double wBuDBvD2 = wij * buD * bvD2;

                sw += wBuBv * pij;
                swDU += wBuDBv * pij;
                swDV += wBuBvD * pij;
                swD2U += wBuD2Bv * pij;
                swD2V += wBuBvD2 * pij;
                swDUV += wBuDBvD * pij;
                swD3U += wBuD3Bv * pij;
                swD3V += wBuBvD3 * pij;
                swD2UDV += wBuD2BvD * pij;
                swDUD2V += wBuDBvD2 * pij;
                w += wBuBv;
                wdu += wBuDBv;
                wdv += wBuBvD;
                wd2U += wBuD2Bv;
                wd2V += wBuBvD2;
                wduv += wBuDBvD;
                wd3U += wBuD3Bv;
                wd3V += wBuBvD3;
                wd2UDV += wBuD2BvD;
                wdud2V += wBuDBvD2;
            }
        }

        // S = Sw / w
        Xyz sr = sw / w;
        // First derivatives: S_u = (Sw_u - w_u * S) / w
        Xyz sd1U = (swDU - wdu * sr) / w;
        Xyz sd1V = (swDV - wdv * sr) / w;
        // Second derivatives: S_uu = (Sw_uu - 2*w_u*S_u - w_uu*S) / w
        Xyz sd2U = (swD2U - 2.0 * wdu * sd1U - wd2U * sr) / w;
        Xyz sd2V = (swD2V - 2.0 * wdv * sd1V - wd2V * sr) / w;
        Xyz sd2UV = (swDUV - wdu * sd1V - wdv * sd1U - wduv * sr) / w;
        // Third derivatives: obtained by differentiating the Leibniz rule expressions.
        // S_uuu = (Sw_uuu - 3*w_u*S_uu - 3*w_uu*S_u - w_uuu*S) / w
        Xyz sd3U = (swD3U - 3.0 * wdu * sd2U - 3.0 * wd2U * sd1U - wd3U * sr) / w;
        // S_vvv = (Sw_vvv - 3*w_v*S_vv - 3*w_vv*S_v - w_vvv*S) / w
        Xyz sd3V = (swD3V - 3.0 * wdv * sd2V - 3.0 * wd2V * sd1V - wd3V * sr) / w;
        // S_uuv: differentiate w*S_uu = Sw_uu - 2*w_u*S_u - w_uu*S with respect to v.
        Xyz sd3UUV = (swD2UDV - wd2UDV * sr - 2.0 * wdu * sd2UV - wdv * sd2U
                          - wd2U * sd1V - 2.0 * wduv * sd1U)
                         / w;
        // S_uvv: differentiate w*S_vv = Sw_vv - 2*w_v*S_v - w_vv*S with respect to u.
        Xyz sd3UVV = (swDUD2V - wdud2V * sr - 2.0 * wdv * sd2UV - wdu * sd2V
                          - wd2V * sd1U - 2.0 * wduv * sd1V)
                         / w;

        result.Point = new Point3D(sr);
        result.D1U = new Vector3D(sd1U);
        result.D1V = new Vector3D(sd1V);
        result.D2U = new Vector3D(sd2U);
        result.D2V = new Vector3D(sd2V);
        result.D2UV = new Vector3D(sd2UV);
        result.D3U = new Vector3D(sd3U);
        result.D3V = new Vector3D(sd3V);
        result.D3UUV = new Vector3D(sd3UUV);
        result.D3UVV = new Vector3D(sd3UVV);
        return result;
    }

    /// <exception cref="RangeErrorException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    /// <exception cref="UndefinedDerivative3DException">A rational surface and Nu + Nv &gt; 3.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new RangeErrorException("GeomEval_TBezierSurface::EvalDN: invalid derivative order");
        }

        // For non-rational surfaces, compute directly from basis derivatives.
        if (!_rational)
        {
            int nbU = _poles.NbRows();
            int nbV = _poles.NbColumns();
            Span<double> basisU = nbU <= 16 ? stackalloc double[16] : new double[nbU];
            Span<double> basisV = nbV <= 16 ? stackalloc double[16] : new double[nbV];
            EvalTrigAxisNthDeriv(u, _alphaU, (nbU - 1) / 2, nu, basisU);
            EvalTrigAxisNthDeriv(v, _alphaV, (nbV - 1) / 2, nv, basisV);

            Xyz sum = new(0.0, 0.0, 0.0);
            for (int i = 0; i < nbU; ++i)
            {
                double bu = basisU[i];
                for (int j = 0; j < nbV; ++j)
                {
                    sum +=
                        (bu * basisV[j]) * _poles.Value(_poles.LowerRow() + i, _poles.LowerCol() + j).XYZ();
                }
            }
            return new Vector3D(sum);
        }

        // For rational surfaces, delegate to the specialized methods for low orders.
        if (nu + nv <= 3)
        {
            if (nu == 1 && nv == 0)
            {
                return EvalD1(u, v).D1U;
            }
            if (nu == 0 && nv == 1)
            {
                return EvalD1(u, v).D1V;
            }
            if (nu == 2 && nv == 0)
            {
                return EvalD2(u, v).D2U;
            }
            if (nu == 0 && nv == 2)
            {
                return EvalD2(u, v).D2V;
            }
            if (nu == 1 && nv == 1)
            {
                return EvalD2(u, v).D2UV;
            }
            if (nu == 3 && nv == 0)
            {
                return EvalD3(u, v).D3U;
            }
            if (nu == 0 && nv == 3)
            {
                return EvalD3(u, v).D3V;
            }
            if (nu == 2 && nv == 1)
            {
                return EvalD3(u, v).D3UUV;
            }
            if (nu == 1 && nv == 2)
            {
                return EvalD3(u, v).D3UVV;
            }
        }

        // For higher-order rational derivatives, throw.
        throw new UndefinedDerivative3DException(
            "GeomEval_TBezierSurface::EvalDN: rational derivatives of order > 3 not implemented");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::Transform");
    }

    public override Geometry3D Copy()
    {
        if (_rational)
        {
            return new TBezierSurface(_poles, _weights, _alphaU, _alphaV);
        }
        return new TBezierSurface(_poles, _alphaU, _alphaV);
    }
}
