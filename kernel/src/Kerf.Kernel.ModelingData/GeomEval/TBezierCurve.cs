// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 3D trigonometric Bezier curve, over the basis {1, sin(alpha*t), cos(alpha*t), ..., sin(n*alpha*t),
/// cos(n*alpha*t)}: <c>C(t) = P_0*T_0(t) + ... + P_2n*T_2n(t)</c>, with T_0 = 1, T_2k-1 = sin(k*alpha*t) and
/// T_2k = cos(k*alpha*t). A curve of order n has 2n + 1 poles; the parameter domain is [0, Pi/alpha]. A rational
/// curve weights each pole: <c>C(t) = sum(w_i*P_i*T_i(t)) / sum(w_i*T_i(t))</c>.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class TBezierCurve : BoundedCurve3D
{
    // The numerators (N) and denominators (W) of the derivatives up to the order evaluated.
    // The C++ CurveBasisEval<theMaxOrder>.
    private struct CurveBasisEval
    {
        public Xyz N0;
        public Xyz N1;
        public Xyz N2;
        public Xyz N3;
        public double W0;
        public double W1;
        public double W2;
        public double W3;
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

    // The C++ template evalCurveBasis<theMaxOrder>: the `if constexpr` on the order are plain tests here.
    private static void EvalCurveBasis(int maxOrder,
                                       NCollection_Array1<Point3D> poles,
                                       NCollection_Array1<double>? weights,
                                       double alpha,
                                       double u,
                                       out CurveBasisEval data)
    {
        data = new CurveBasisEval();
        int nbPoles = poles.Length();
        ReadOnlySpan<Point3D> aPoles = poles.AsSpan();
        ReadOnlySpan<double> aWeights = weights != null ? weights.AsSpan() : default;

        Xyz pole0 = aPoles[0].XYZ();
        if (weights == null)
        {
            data.N0 += pole0;
        }
        else
        {
            double w0 = aWeights[0];
            data.N0 += pole0 * w0;
            data.W0 += w0;
        }

        int order = (nbPoles - 1) / 2;
        if (order <= 0)
        {
            return;
        }

        double theta = alpha * u;
        double sin1 = Math.Sin(theta);
        double cos1 = Math.Cos(theta);
        double sinK = sin1;
        double cosK = cos1;
        double freq = alpha;
        int idx = 1;
        for (int k = 1; k <= order; ++k, idx += 2, freq += alpha)
        {
            double sin = sinK;
            double cos = cosK;
            double freq2 = freq * freq;
            double freq3 = freq2 * freq;

            Xyz poleSin = aPoles[idx].XYZ();
            Xyz poleCos = aPoles[idx + 1].XYZ();

            double b0Sin = sin;
            double b0Cos = cos;

            double b1Sin = 0.0;
            double b1Cos = 0.0;
            double b2Sin = 0.0;
            double b2Cos = 0.0;
            double b3Sin = 0.0;
            double b3Cos = 0.0;
            if (maxOrder >= 1)
            {
                b1Sin = freq * cos;
                b1Cos = -freq * sin;
            }
            if (maxOrder >= 2)
            {
                b2Sin = -freq2 * sin;
                b2Cos = -freq2 * cos;
            }
            if (maxOrder >= 3)
            {
                b3Sin = -freq3 * cos;
                b3Cos = freq3 * sin;
            }

            if (weights == null)
            {
                data.N0 += poleSin * b0Sin + poleCos * b0Cos;
                if (maxOrder >= 1)
                {
                    data.N1 += poleSin * b1Sin + poleCos * b1Cos;
                }
                if (maxOrder >= 2)
                {
                    data.N2 += poleSin * b2Sin + poleCos * b2Cos;
                }
                if (maxOrder >= 3)
                {
                    data.N3 += poleSin * b3Sin + poleCos * b3Cos;
                }
            }
            else
            {
                double wSin = aWeights[idx];
                double wCos = aWeights[idx + 1];

                data.N0 += poleSin * (wSin * b0Sin) + poleCos * (wCos * b0Cos);
                data.W0 += wSin * b0Sin + wCos * b0Cos;
                if (maxOrder >= 1)
                {
                    data.N1 += poleSin * (wSin * b1Sin) + poleCos * (wCos * b1Cos);
                    data.W1 += wSin * b1Sin + wCos * b1Cos;
                }
                if (maxOrder >= 2)
                {
                    data.N2 += poleSin * (wSin * b2Sin) + poleCos * (wCos * b2Cos);
                    data.W2 += wSin * b2Sin + wCos * b2Cos;
                }
                if (maxOrder >= 3)
                {
                    data.N3 += poleSin * (wSin * b3Sin) + poleCos * (wCos * b3Cos);
                    data.W3 += wSin * b3Sin + wCos * b3Cos;
                }
            }

            if (k < order)
            {
                double sinNext = sinK * cos1 + cosK * sin1;
                double cosNext = cosK * cos1 - sinK * sin1;
                sinK = sinNext;
                cosK = cosNext;
            }
        }
    }

    private NCollection_Array1<Point3D> _poles;  // Control points
    private NCollection_Array1<double> _weights;  // Weights (empty if non-rational)
    private double _alpha;                        // Frequency parameter
    private bool _rational;                       // True if curve is rational

    /// <summary>A non-rational T-Bezier curve.</summary>
    /// <param name="poles">The control points; their number must be odd and at least 3.</param>
    /// <param name="alpha">The frequency parameter (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">The number of poles is even or below 3, or theAlpha &lt;= 0.</exception>
    public TBezierCurve(NCollection_Array1<Point3D> poles, double alpha)
    {
        _poles = new NCollection_Array1<Point3D>(poles);
        _weights = new NCollection_Array1<double>();
        _alpha = alpha;
        _rational = false;
        int nbPoles = poles.Length();
        if (nbPoles < 3 || (nbPoles % 2) == 0)
        {
            throw new ConstructionErrorException(
                "GeomEval_TBezierCurve: number of poles must be odd and >= 3");
        }
        if (alpha <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierCurve: alpha must be > 0");
        }
    }

    /// <summary>A rational T-Bezier curve.</summary>
    /// <param name="poles">The control points; their number must be odd and at least 3.</param>
    /// <param name="weights">The weights, as many as the poles, all &gt; 0.</param>
    /// <param name="alpha">The frequency parameter (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">Any of the conditions above fails.</exception>
    public TBezierCurve(NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights,
                                   double alpha)
    {
        _poles = new NCollection_Array1<Point3D>(poles);
        _weights = new NCollection_Array1<double>(weights);
        _alpha = alpha;
        _rational = true;
        int nbPoles = poles.Length();
        if (nbPoles < 3 || (nbPoles % 2) == 0)
        {
            throw new ConstructionErrorException(
                "GeomEval_TBezierCurve: number of poles must be odd and >= 3");
        }
        if (alpha <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierCurve: alpha must be > 0");
        }
        if (weights.Length() != nbPoles)
        {
            throw new ConstructionErrorException("GeomEval_TBezierCurve: weights size must equal poles size");
        }
        for (int i = weights.Lower(); i <= weights.Upper(); ++i)
        {
            if (weights.Value(i) <= 0.0)
            {
                throw new ConstructionErrorException("GeomEval_TBezierCurve: all weights must be > 0");
            }
        }
    }

    /// <summary>The poles (a copy, indexed from 0).</summary>
    public IReadOnlyList<Point3D> Poles() => _poles.ToImmutableArray();

    /// <summary>The weights (a copy, indexed from 0), empty if the curve is non-rational.</summary>
    public IReadOnlyList<double> Weights() => _weights.ToImmutableArray();

    /// <summary>The frequency parameter alpha.</summary>
    public double Alpha() => _alpha;

    /// <summary>The number of poles.</summary>
    public int PoleCount => _poles.Length();

    /// <summary>The trigonometric order n (NbPoles = 2*n + 1).</summary>
    public int Order() => (_poles.Length() - 1) / 2;

    /// <summary>True if the curve is rational.</summary>
    public bool IsRational => _rational;

    /// <summary>The start point C(0).</summary>
    public override Point3D StartPoint => EvalD0(0.0);

    /// <summary>The end point C(Pi/alpha).</summary>
    public override Point3D EndPoint => EvalD0(M_PI / _alpha);

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter => 0.0;

    /// <summary>Pi/alpha.</summary>
    public override double LastParameter => M_PI / _alpha;

    /// <summary>True if the start and end points coincide (within Precision::Confusion()).</summary>
    public override bool IsClosed => StartPoint.DistanceTo(EndPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion();

    public override bool IsPeriodic => false;

    public override Continuity Continuity => CN;

    internal override bool IsCN(int n) => true;

    // Evaluate trigonometric basis functions at parameter theT: {T_0(t), T_1(t), ..., T_{2n}(t)}.
    private void EvalBasis(double t, NCollection_Array1<double> basis)
    {
        // T_0(t) = 1
        Span<double> aBasis = basis.AsSpan();
        aBasis[0] = 1.0;
        int order = Order();
        if (order <= 0)
        {
            return;
        }

        double theta = _alpha * t;
        double sin1 = Math.Sin(theta);
        double cos1 = Math.Cos(theta);
        double sinK = sin1;
        double cosK = cos1;
        for (int k = 1; k <= order; ++k)
        {
            aBasis[2 * k - 1] = sinK;
            aBasis[2 * k] = cosK;
            if (k < order)
            {
                double sinNext = sinK * cos1 + cosK * sin1;
                double cosNext = cosK * cos1 - sinK * sin1;
                sinK = sinNext;
                cosK = cosNext;
            }
        }
    }

    // Evaluate derivatives of order theDerivOrder of the basis functions.
    private void EvalBasisDeriv(double t, int derivOrder, NCollection_Array1<double> basisDeriv)
    {
        // Derivative of constant basis function T_0 = 1 is always 0.
        Span<double> basis = basisDeriv.AsSpan();
        basis[0] = 0.0;

        int order = Order();
        if (order <= 0)
        {
            return;
        }

        int orderMod4 = derivOrder & 3;
        double theta = _alpha * t;
        double sin1 = Math.Sin(theta);
        double cos1 = Math.Cos(theta);
        double sinK = sin1;
        double cosK = cos1;
        double freq = _alpha;
        for (int k = 1; k <= order; ++k)
        {
            double sin = sinK;
            double cos = cosK;
            double freqPow = PowInt(freq, derivOrder);
            EvalTrigPairNthDeriv(sin, cos, orderMod4, out double sinDeriv, out double cosDeriv);
            basis[2 * k - 1] = freqPow * sinDeriv;
            basis[2 * k] = freqPow * cosDeriv;
            freq += _alpha;
            if (k < order)
            {
                double sinNext = sinK * cos1 + cosK * sin1;
                double cosNext = cosK * cos1 - sinK * sin1;
                sinK = sinNext;
                cosK = cosNext;
            }
        }
    }

    // Evaluate point on a non-rational curve: sum(P_i * B_i(t)). Unused, as in OCCT.
    private Point3D EvalNonRationalPoint(NCollection_Array1<double> basis)
    {
        Xyz sum = new(0.0, 0.0, 0.0);
        int nbPoles = _poles.Length();
        ReadOnlySpan<Point3D> polesData = _poles.AsSpan();
        ReadOnlySpan<double> basisData = basis.AsSpan();
        for (int i = 0; i < nbPoles; ++i)
        {
            sum += basisData[i] * polesData[i].XYZ();
        }
        return new Point3D(sum);
    }

    // Evaluate derivative vector on a non-rational curve: sum(P_i * B_i'(t)). Unused, as in OCCT.
    private Vector3D EvalNonRationalDeriv(NCollection_Array1<double> basisDeriv)
    {
        Xyz sum = new(0.0, 0.0, 0.0);
        int nbPoles = _poles.Length();
        ReadOnlySpan<Point3D> polesData = _poles.AsSpan();
        ReadOnlySpan<double> basisData = basisDeriv.AsSpan();
        for (int i = 0; i < nbPoles; ++i)
        {
            sum += basisData[i] * polesData[i].XYZ();
        }
        return new Vector3D(sum);
    }

    internal override Point3D EvalD0(double u)
    {
        CurveBasisEval data;
        if (!_rational)
        {
            EvalCurveBasis(0, _poles, null, _alpha, u, out data);
            return new Point3D(data.N0);
        }
        EvalCurveBasis(0, _poles, _weights, _alpha, u, out data);
        return new Point3D(data.N0 / data.W0);
    }

    internal override ResD1 EvalD1(double u)
    {
        ResD1 result;
        CurveBasisEval data;

        if (!_rational)
        {
            EvalCurveBasis(1, _poles, null, _alpha, u, out data);
            result.Point = new Point3D(data.N0);
            result.D1 = new Vector3D(data.N1);
            return result;
        }

        EvalCurveBasis(1, _poles, _weights, _alpha, u, out data);
        Xyz c = data.N0 / data.W0;
        result.Point = new Point3D(c);
        result.D1 = new Vector3D((data.N1 - data.W1 * c) / data.W0);
        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        ResD2 result;
        CurveBasisEval data;

        if (!_rational)
        {
            EvalCurveBasis(2, _poles, null, _alpha, u, out data);
            result.Point = new Point3D(data.N0);
            result.D1 = new Vector3D(data.N1);
            result.D2 = new Vector3D(data.N2);
            return result;
        }

        EvalCurveBasis(2, _poles, _weights, _alpha, u, out data);
        Xyz c = data.N0 / data.W0;
        Xyz cd1 = (data.N1 - data.W1 * c) / data.W0;
        Xyz cd2 = (data.N2 - 2.0 * data.W1 * cd1 - data.W2 * c) / data.W0;

        result.Point = new Point3D(c);
        result.D1 = new Vector3D(cd1);
        result.D2 = new Vector3D(cd2);
        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        ResD3 result;
        CurveBasisEval data;

        if (!_rational)
        {
            EvalCurveBasis(3, _poles, null, _alpha, u, out data);
            result.Point = new Point3D(data.N0);
            result.D1 = new Vector3D(data.N1);
            result.D2 = new Vector3D(data.N2);
            result.D3 = new Vector3D(data.N3);
            return result;
        }

        EvalCurveBasis(3, _poles, _weights, _alpha, u, out data);
        Xyz c = data.N0 / data.W0;
        Xyz cd1 = (data.N1 - data.W1 * c) / data.W0;
        Xyz cd2 = (data.N2 - 2.0 * data.W1 * cd1 - data.W2 * c) / data.W0;
        Xyz cd3 =
            (data.N3 - 3.0 * data.W2 * cd1 - 3.0 * data.W1 * cd2 - data.W3 * c) / data.W0;

        result.Point = new Point3D(c);
        result.D1 = new Vector3D(cd1);
        result.D2 = new Vector3D(cd2);
        result.D3 = new Vector3D(cd3);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    internal override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("GeomEval_TBezierCurve::EvalDN: derivative order must be >= 1");
        }

        // For non-rational curves, compute directly.
        if (!_rational)
        {
            Xyz sum = new(0.0, 0.0, 0.0);
            int order = Order();
            int orderMod4 = n & 3;
            if (order <= 0)
            {
                return new Vector3D(sum);
            }

            ReadOnlySpan<Point3D> polesData = _poles.AsSpan();
            double theta = _alpha * u;
            double sin1 = Math.Sin(theta);
            double cos1 = Math.Cos(theta);
            double sinK = sin1;
            double cosK = cos1;
            double freq = _alpha;
            int idx = 1;

            for (int k = 1; k <= order; ++k, idx += 2, freq += _alpha)
            {
                double freqPow = PowInt(freq, n);
                double sin = sinK;
                double cos = cosK;

                EvalTrigPairNthDeriv(sin, cos, orderMod4, out double sinDeriv, out double cosDeriv);

                sum += polesData[idx].XYZ() * (freqPow * sinDeriv);
                sum += polesData[idx + 1].XYZ() * (freqPow * cosDeriv);

                if (k < order)
                {
                    double sinNext = sinK * cos1 + cosK * sin1;
                    double cosNext = cosK * cos1 - sinK * sin1;
                    sinK = sinNext;
                    cosK = cosNext;
                }
            }
            return new Vector3D(sum);
        }

        // For rational curves with N <= 3, delegate to the full evaluation.
        if (n == 1)
        {
            return EvalD1(u).D1;
        }
        if (n == 2)
        {
            return EvalD2(u).D2;
        }
        if (n == 3)
        {
            return EvalD3(u).D3;
        }

        // For N > 3 on rational curves, use the general recurrence formula:
        // C^(n)(t) = (Cw^(n)(t) - sum_{k=1..n} C(n,k)*w^(k)(t)*C^(n-k)(t)) / w(t)
        int nbPoles = _poles.Length();

        // Precompute all basis derivative arrays up to order N.
        NCollection_Array1<NCollection_Array1<double>> basisDerivs = new(0, n);
        for (int d = 0; d <= n; ++d)
        {
            // C++ resizes a default-constructed element; C# elements of a class type start null.
            basisDerivs[d] = new NCollection_Array1<double>(1, nbPoles);
            if (d == 0)
            {
                EvalBasis(u, basisDerivs[0]);
            }
            else
            {
                EvalBasisDeriv(u, d, basisDerivs[d]);
            }
        }

        // Compute w^(d) and Cw^(d) for d = 0..N.
        NCollection_Array1<double> wDerivs = new(0, n);
        NCollection_Array1<Xyz> cwDerivs = new(0, n);
        ReadOnlySpan<Point3D> polesData2 = _poles.AsSpan();
        ReadOnlySpan<double> weightsData = _weights.AsSpan();
        for (int d = 0; d <= n; ++d)
        {
            NCollection_Array1<double> basisD = basisDerivs.Value(d);
            ReadOnlySpan<double> basisData = basisD.AsSpan(1);
            double wd = 0.0;
            Xyz cwd = new(0.0, 0.0, 0.0);
            for (int i = 0; i < nbPoles; ++i)
            {
                double wi = weightsData[i];
                double bd = basisData[i];
                Xyz pi = polesData2[i].XYZ();
                cwd += (wi * bd) * pi;
                wd += wi * bd;
            }
            wDerivs.SetValue(d, wd);
            cwDerivs.SetValue(d, cwd);
        }

        // Compute C^(d) for d = 0..N using the recurrence.
        NCollection_Array1<Xyz> cDerivs = new(0, n);
        cDerivs.SetValue(0, cwDerivs.Value(0) / wDerivs.Value(0));

        for (int d = 1; d <= n; ++d)
        {
            Xyz sum = new(0.0, 0.0, 0.0);
            // Binomial coefficients via incremental computation.
            double binom = 1.0;
            for (int k = 1; k <= d; ++k)
            {
                binom *= (double)(d - k + 1) / (double)k;
                sum += binom * wDerivs.Value(k) * cDerivs.Value(d - k);
            }
            cDerivs.SetValue(d, (cwDerivs.Value(d) - sum) / wDerivs.Value(0));
        }

        return new Vector3D(cDerivs.Value(n));
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierCurve::Transform");
    }

    public override Geometry3D Copy()
    {
        if (_rational)
        {
            return new TBezierCurve(_poles, _weights, _alpha);
        }
        return new TBezierCurve(_poles, _alpha);
    }
}
