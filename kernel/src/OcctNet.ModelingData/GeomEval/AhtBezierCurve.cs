// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 3D Algebraic-Hyperbolic-Trigonometric Bezier curve, over the mixed basis
/// {1, t, ..., t^k, sinh(alpha*t), cosh(alpha*t), sin(beta*t), cos(beta*t)}, where the hyperbolic pair is present
/// when alpha &gt; 0 and the trigonometric pair when beta &gt; 0. The number of poles must equal the number of
/// basis functions, algDegree + 1 + 2*(alpha&gt;0) + 2*(beta&gt;0). The parameter range is [0, 1].
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class AhtBezierCurve : BoundedCurve3D
{
    // The numerators (N) and denominators (W) of the derivatives up to the order evaluated.
    private struct PolyEvalData
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

    private static double PowInt(double theBase, int exp)
    {
        double res = 1.0;
        for (int i = 0; i < exp; ++i)
        {
            res *= theBase;
        }
        return res;
    }

    private static double FallingFactorial(int n, int k)
    {
        double res = 1.0;
        for (int i = 0; i < k; ++i)
        {
            res *= (double)(n - i);
        }
        return res;
    }

    // The C++ template evalPolynomialNonRational<theMaxOrder>.
    private static void EvalPolynomialNonRational(int maxOrder,
                                                  NCollection_Array1<Point3D> poles,
                                                  int degree,
                                                  double u,
                                                  out PolyEvalData data)
    {
        data = new PolyEvalData();
        int poleLow = poles.Lower();
        data.N0 = poles.Value(poleLow + degree).XYZ();

        for (int k = degree - 1; k >= 0; --k)
        {
            if (maxOrder >= 3)
            {
                data.N3 = data.N3 * u + data.N2 * 3.0;
            }
            if (maxOrder >= 2)
            {
                data.N2 = data.N2 * u + data.N1 * 2.0;
            }
            if (maxOrder >= 1)
            {
                data.N1 = data.N1 * u + data.N0;
            }
            data.N0 = data.N0 * u + poles.Value(poleLow + k).XYZ();
        }
    }

    // The C++ template evalPolynomialRational<theMaxOrder>.
    private static void EvalPolynomialRational(int maxOrder,
                                               NCollection_Array1<Point3D> poles,
                                               NCollection_Array1<double> weights,
                                               int degree,
                                               double u,
                                               out PolyEvalData data)
    {
        data = new PolyEvalData();
        int poleLow = poles.Lower();
        int weightLow = weights.Lower();

        double wLead = weights.Value(weightLow + degree);
        data.N0 = poles.Value(poleLow + degree).XYZ() * wLead;
        data.W0 = wLead;

        for (int k = degree - 1; k >= 0; --k)
        {
            if (maxOrder >= 3)
            {
                data.N3 = data.N3 * u + data.N2 * 3.0;
                data.W3 = data.W3 * u + data.W2 * 3.0;
            }
            if (maxOrder >= 2)
            {
                data.N2 = data.N2 * u + data.N1 * 2.0;
                data.W2 = data.W2 * u + data.W1 * 2.0;
            }
            if (maxOrder >= 1)
            {
                data.N1 = data.N1 * u + data.N0;
                data.W1 = data.W1 * u + data.W0;
            }

            double wk = weights.Value(weightLow + k);
            data.N0 = data.N0 * u + poles.Value(poleLow + k).XYZ() * wk;
            data.W0 = data.W0 * u + wk;
        }
    }

    // The C++ template evalMixedAHT<theMaxOrder, theIsRational>.
    private static void EvalMixedAHT(int maxOrder,
                                     bool isRational,
                                     NCollection_Array1<Point3D> poles,
                                     NCollection_Array1<double> weights,
                                     int algDegree,
                                     double alpha,
                                     double beta,
                                     double u,
                                     out PolyEvalData data)
    {
        data = new PolyEvalData();
        int poleIdx = poles.Lower();
        int weightIdx = weights.Lower();

        // Polynomial part: {1, t, ..., t^k}.

        double pow = 1.0;
        double powM1 = 0.0;
        double powM2 = 0.0;
        double powM3 = 0.0;
        for (int k = 0; k <= algDegree; ++k)
        {
            Xyz pole = poles.Value(poleIdx++).XYZ();
            double b0 = pow;
            double b1 = 0.0;
            double b2 = 0.0;
            double b3 = 0.0;
            if (maxOrder >= 1)
            {
                b1 = (k >= 1) ? (double)k * powM1 : 0.0;
            }
            if (maxOrder >= 2)
            {
                b2 = (k >= 2) ? (double)k * (double)(k - 1) * powM2 : 0.0;
            }
            if (maxOrder >= 3)
            {
                b3 = (k >= 3) ? (double)k * (double)(k - 1) * (double)(k - 2) * powM3 : 0.0;
            }

            if (isRational)
            {
                double wi = weights.Value(weightIdx++);
                data.N0 += pole * (wi * b0);
                data.W0 += wi * b0;
                if (maxOrder >= 1)
                {
                    data.N1 += pole * (wi * b1);
                    data.W1 += wi * b1;
                }
                if (maxOrder >= 2)
                {
                    data.N2 += pole * (wi * b2);
                    data.W2 += wi * b2;
                }
                if (maxOrder >= 3)
                {
                    data.N3 += pole * (wi * b3);
                    data.W3 += wi * b3;
                }
            }
            else
            {
                data.N0 += pole * b0;
                if (maxOrder >= 1)
                {
                    data.N1 += pole * b1;
                }
                if (maxOrder >= 2)
                {
                    data.N2 += pole * b2;
                }
                if (maxOrder >= 3)
                {
                    data.N3 += pole * b3;
                }
            }

            powM3 = powM2;
            powM2 = powM1;
            powM1 = pow;
            pow *= u;
        }

        if (alpha > 0.0)
        {
            double alphaU = alpha * u;
            double sinh = Math.Sinh(alphaU);
            double cosh = Math.Cosh(alphaU);
            double alpha2 = alpha * alpha;
            double alpha3 = alpha2 * alpha;

            Xyz poleSinh = poles.Value(poleIdx++).XYZ();
            Xyz poleCosh = poles.Value(poleIdx++).XYZ();
            double wiSinh = 1.0;
            double wiCosh = 1.0;
            if (isRational)
            {
                wiSinh = weights.Value(weightIdx++);
                wiCosh = weights.Value(weightIdx++);
            }

            double b0Sinh = sinh;
            double b0Cosh = cosh;
            double b1Sinh = 0.0;
            double b1Cosh = 0.0;
            double b2Sinh = 0.0;
            double b2Cosh = 0.0;
            double b3Sinh = 0.0;
            double b3Cosh = 0.0;
            if (maxOrder >= 1)
            {
                b1Sinh = alpha * cosh;
                b1Cosh = alpha * sinh;
            }
            if (maxOrder >= 2)
            {
                b2Sinh = alpha2 * sinh;
                b2Cosh = alpha2 * cosh;
            }
            if (maxOrder >= 3)
            {
                b3Sinh = alpha3 * cosh;
                b3Cosh = alpha3 * sinh;
            }

            data.N0 += poleSinh * (wiSinh * b0Sinh) + poleCosh * (wiCosh * b0Cosh);
            if (isRational)
            {
                data.W0 += wiSinh * b0Sinh + wiCosh * b0Cosh;
            }
            if (maxOrder >= 1)
            {
                data.N1 += poleSinh * (wiSinh * b1Sinh) + poleCosh * (wiCosh * b1Cosh);
                if (isRational)
                {
                    data.W1 += wiSinh * b1Sinh + wiCosh * b1Cosh;
                }
            }
            if (maxOrder >= 2)
            {
                data.N2 += poleSinh * (wiSinh * b2Sinh) + poleCosh * (wiCosh * b2Cosh);
                if (isRational)
                {
                    data.W2 += wiSinh * b2Sinh + wiCosh * b2Cosh;
                }
            }
            if (maxOrder >= 3)
            {
                data.N3 += poleSinh * (wiSinh * b3Sinh) + poleCosh * (wiCosh * b3Cosh);
                if (isRational)
                {
                    data.W3 += wiSinh * b3Sinh + wiCosh * b3Cosh;
                }
            }
        }

        if (beta > 0.0)
        {
            double betaU = beta * u;
            double sin = Math.Sin(betaU);
            double cos = Math.Cos(betaU);
            double beta2 = beta * beta;
            double beta3 = beta2 * beta;

            Xyz poleSin = poles.Value(poleIdx++).XYZ();
            Xyz poleCos = poles.Value(poleIdx++).XYZ();
            double wiSin = 1.0;
            double wiCos = 1.0;
            if (isRational)
            {
                wiSin = weights.Value(weightIdx++);
                wiCos = weights.Value(weightIdx++);
            }

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
                b1Sin = beta * cos;
                b1Cos = -beta * sin;
            }
            if (maxOrder >= 2)
            {
                b2Sin = -beta2 * sin;
                b2Cos = -beta2 * cos;
            }
            if (maxOrder >= 3)
            {
                b3Sin = -beta3 * cos;
                b3Cos = beta3 * sin;
            }

            data.N0 += poleSin * (wiSin * b0Sin) + poleCos * (wiCos * b0Cos);
            if (isRational)
            {
                data.W0 += wiSin * b0Sin + wiCos * b0Cos;
            }
            if (maxOrder >= 1)
            {
                data.N1 += poleSin * (wiSin * b1Sin) + poleCos * (wiCos * b1Cos);
                if (isRational)
                {
                    data.W1 += wiSin * b1Sin + wiCos * b1Cos;
                }
            }
            if (maxOrder >= 2)
            {
                data.N2 += poleSin * (wiSin * b2Sin) + poleCos * (wiCos * b2Cos);
                if (isRational)
                {
                    data.W2 += wiSin * b2Sin + wiCos * b2Cos;
                }
            }
            if (maxOrder >= 3)
            {
                data.N3 += poleSin * (wiSin * b3Sin) + poleCos * (wiCos * b3Cos);
                if (isRational)
                {
                    data.W3 += wiSin * b3Sin + wiCos * b3Cos;
                }
            }
        }
    }

    private NCollection_Array1<Point3D> _poles;
    private NCollection_Array1<double> _weights;
    private int _algDegree;
    private double _alpha;
    private double _beta;
    private bool _rational;

    // The number of basis functions for the given parameters.
    private static int BasisDimension(int algDegree, double alpha, double beta) =>
        algDegree + 1 + (alpha > 0.0 ? 2 : 0) + (beta > 0.0 ? 2 : 0);

    /// <summary>A non-rational AHT-Bezier curve.</summary>
    /// <param name="poles">The control points, as many as the basis functions.</param>
    /// <param name="algDegree">The degree of the polynomial part (&gt;= 0).</param>
    /// <param name="alpha">The hyperbolic frequency (&gt;= 0; 0 leaves the hyperbolic pair out).</param>
    /// <param name="beta">The trigonometric frequency (&gt;= 0; 0 leaves the trigonometric pair out).</param>
    /// <exception cref="ConstructionErrorException">A parameter is out of range or the number of poles is wrong.</exception>
    public AhtBezierCurve(NCollection_Array1<Point3D> poles, int algDegree, double alpha,
                                     double beta)
    {
        _poles = new NCollection_Array1<Point3D>(poles);
        // OCCT: a non-rational curve has one weight, left uninitialised (0 here), which Weights() returns.
        _weights = new NCollection_Array1<double>(1, 1);
        _algDegree = algDegree;
        _alpha = alpha;
        _beta = beta;
        _rational = false;
        if (algDegree < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (alpha < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (beta < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: beta must be >= 0");
        }
        int dim = BasisDimension(algDegree, alpha, beta);
        if (poles.Length() != dim)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
    }

    /// <summary>A rational AHT-Bezier curve.</summary>
    /// <param name="poles">The control points, as many as the basis functions.</param>
    /// <param name="weights">The weights, as many as the poles, all &gt; 0.</param>
    /// <param name="algDegree">The degree of the polynomial part (&gt;= 0).</param>
    /// <param name="alpha">The hyperbolic frequency (&gt;= 0).</param>
    /// <param name="beta">The trigonometric frequency (&gt;= 0).</param>
    /// <exception cref="ConstructionErrorException">A parameter is out of range or an array has the wrong size.</exception>
    public AhtBezierCurve(NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights,
                                     int algDegree, double alpha, double beta)
    {
        _poles = new NCollection_Array1<Point3D>(poles);
        _weights = new NCollection_Array1<double>(weights);
        _algDegree = algDegree;
        _alpha = alpha;
        _beta = beta;
        _rational = true;
        if (algDegree < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (alpha < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (beta < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: beta must be >= 0");
        }
        int dim = BasisDimension(algDegree, alpha, beta);
        if (poles.Length() != dim)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
        if (weights.Length() != poles.Length())
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: weights array size must match poles array size");
        }
        for (int idx = weights.Lower(); idx <= weights.Upper(); ++idx)
        {
            if (weights.Value(idx) <= 0.0)
            {
                throw new ConstructionErrorException("GeomEval_AHTBezierCurve: all weights must be > 0");
            }
        }
    }

    /// <summary>The poles (a copy, indexed from 0).</summary>
    public PackedList<Point3D> Poles() => _poles.ToPackedList();

    /// <summary>The weights (a copy, indexed from 0). A non-rational curve has one weight, 0, as OCCT's.</summary>
    public PackedList<double> Weights() => _weights.ToPackedList();

    /// <summary>The degree of the polynomial part.</summary>
    public int AlgDegree() => _algDegree;

    /// <summary>The hyperbolic frequency.</summary>
    public double Alpha() => _alpha;

    /// <summary>The trigonometric frequency.</summary>
    public double Beta() => _beta;

    /// <summary>The number of poles.</summary>
    public int PoleCount => _poles.Length();

    /// <summary>True if the curve is rational.</summary>
    public bool IsRational => _rational;

    /// <summary>The start point C(0).</summary>
    public override Point3D StartPoint() => EvalD0(0.0);

    /// <summary>The end point C(1).</summary>
    public override Point3D EndPoint() => EvalD0(1.0);

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public override double LastParameter() => 1.0;

    /// <summary>True if the start and end points coincide (within Precision::Confusion()).</summary>
    public override bool IsClosed => StartPoint().Distance(EndPoint()) <= global::Kerf.Engine.Foundation.Precision.Confusion();

    public override bool IsPeriodic => false;

    public override Continuity Continuity() => CN;

    public override bool IsCN(int n) => true;

    public override Point3D EvalD0(double u)
    {
        if (_alpha <= 0.0 && _beta <= 0.0)
        {
            PolyEvalData poly;
            if (!_rational)
            {
                EvalPolynomialNonRational(0, _poles, _algDegree, u, out poly);
                return new Point3D(poly.N0);
            }

            EvalPolynomialRational(0, _poles, _weights, _algDegree, u, out poly);
            return new Point3D(poly.N0 / poly.W0);
        }

        PolyEvalData mixed;
        if (!_rational)
        {
            EvalMixedAHT(0, false, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
            return new Point3D(mixed.N0);
        }

        EvalMixedAHT(0, true, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
        return new Point3D(mixed.N0 / mixed.W0);
    }

    public override ResD1 EvalD1(double u)
    {
        if (_alpha <= 0.0 && _beta <= 0.0)
        {
            PolyEvalData poly;
            ResD1 polyResult;
            if (!_rational)
            {
                EvalPolynomialNonRational(1, _poles, _algDegree, u, out poly);
                polyResult.Point = new Point3D(poly.N0);
                polyResult.D1 = new Vector3D(poly.N1);
                return polyResult;
            }

            EvalPolynomialRational(1, _poles, _weights, _algDegree, u, out poly);
            double polyInvW = 1.0 / poly.W0;
            polyResult.Point = new Point3D(poly.N0 * polyInvW);
            polyResult.D1 = new Vector3D((poly.N1 * poly.W0 - poly.N0 * poly.W1) * (polyInvW * polyInvW));
            return polyResult;
        }

        ResD1 result;
        PolyEvalData mixed;
        if (!_rational)
        {
            EvalMixedAHT(1, false, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
            result.Point = new Point3D(mixed.N0);
            result.D1 = new Vector3D(mixed.N1);
            return result;
        }

        EvalMixedAHT(1, true, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
        double invW = 1.0 / mixed.W0;
        result.Point = new Point3D(mixed.N0 * invW);
        result.D1 = new Vector3D((mixed.N1 * mixed.W0 - mixed.N0 * mixed.W1) * (invW * invW));
        return result;
    }

    public override ResD2 EvalD2(double u)
    {
        if (_alpha <= 0.0 && _beta <= 0.0)
        {
            PolyEvalData poly;
            ResD2 polyResult;
            if (!_rational)
            {
                EvalPolynomialNonRational(2, _poles, _algDegree, u, out poly);
                polyResult.Point = new Point3D(poly.N0);
                polyResult.D1 = new Vector3D(poly.N1);
                polyResult.D2 = new Vector3D(poly.N2);
                return polyResult;
            }

            EvalPolynomialRational(2, _poles, _weights, _algDegree, u, out poly);
            double polyInvW = 1.0 / poly.W0;
            Xyz polyC = poly.N0 * polyInvW;
            Xyz polyCD1 = (poly.N1 * poly.W0 - poly.N0 * poly.W1) * (polyInvW * polyInvW);
            polyResult.Point = new Point3D(polyC);
            polyResult.D1 = new Vector3D(polyCD1);
            polyResult.D2 = new Vector3D((poly.N2 - polyCD1 * (2.0 * poly.W1) - polyC * poly.W2) * polyInvW);
            return polyResult;
        }

        ResD2 result;
        PolyEvalData mixed;
        if (!_rational)
        {
            EvalMixedAHT(2, false, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
            result.Point = new Point3D(mixed.N0);
            result.D1 = new Vector3D(mixed.N1);
            result.D2 = new Vector3D(mixed.N2);
            return result;
        }

        EvalMixedAHT(2, true, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
        double invW = 1.0 / mixed.W0;
        Xyz c = mixed.N0 * invW;
        Xyz cd1 = (mixed.N1 * mixed.W0 - mixed.N0 * mixed.W1) * (invW * invW);

        result.Point = new Point3D(c);
        result.D1 = new Vector3D(cd1);
        result.D2 = new Vector3D((mixed.N2 - cd1 * (2.0 * mixed.W1) - c * mixed.W2) * invW);
        return result;
    }

    public override ResD3 EvalD3(double u)
    {
        if (_alpha <= 0.0 && _beta <= 0.0)
        {
            PolyEvalData poly;
            ResD3 polyResult;
            if (!_rational)
            {
                EvalPolynomialNonRational(3, _poles, _algDegree, u, out poly);
                polyResult.Point = new Point3D(poly.N0);
                polyResult.D1 = new Vector3D(poly.N1);
                polyResult.D2 = new Vector3D(poly.N2);
                polyResult.D3 = new Vector3D(poly.N3);
                return polyResult;
            }

            EvalPolynomialRational(3, _poles, _weights, _algDegree, u, out poly);
            double polyInvW = 1.0 / poly.W0;
            Xyz polyC = poly.N0 * polyInvW;
            Xyz polyCD1 = (poly.N1 - polyC * poly.W1) * polyInvW;
            Xyz polyCD2 = (poly.N2 - polyCD1 * (2.0 * poly.W1) - polyC * poly.W2) * polyInvW;
            Xyz polyCD3 =
                (poly.N3 - polyCD2 * (3.0 * poly.W1) - polyCD1 * (3.0 * poly.W2) - polyC * poly.W3) * polyInvW;

            polyResult.Point = new Point3D(polyC);
            polyResult.D1 = new Vector3D(polyCD1);
            polyResult.D2 = new Vector3D(polyCD2);
            polyResult.D3 = new Vector3D(polyCD3);
            return polyResult;
        }

        ResD3 result;
        PolyEvalData mixed;
        if (!_rational)
        {
            EvalMixedAHT(3, false, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
            result.Point = new Point3D(mixed.N0);
            result.D1 = new Vector3D(mixed.N1);
            result.D2 = new Vector3D(mixed.N2);
            result.D3 = new Vector3D(mixed.N3);
            return result;
        }

        EvalMixedAHT(3, true, _poles, _weights, _algDegree, _alpha, _beta, u, out mixed);
        double invW = 1.0 / mixed.W0;
        Xyz c = mixed.N0 * invW;
        Xyz cd1 = (mixed.N1 - c * mixed.W1) * invW;
        Xyz cd2 = (mixed.N2 - cd1 * (2.0 * mixed.W1) - c * mixed.W2) * invW;
        Xyz cd3 =
            (mixed.N3 - cd2 * (3.0 * mixed.W1) - cd1 * (3.0 * mixed.W2) - c * mixed.W3) * invW;

        result.Point = new Point3D(c);
        result.D1 = new Vector3D(cd1);
        result.D2 = new Vector3D(cd2);
        result.D3 = new Vector3D(cd3);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("GeomEval_AHTBezierCurve::EvalDN: N must be >= 1");
        }

        // For low orders, delegate to explicit implementations.
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

        int poleLow = _poles.Lower();

        if (!_rational)
        {
            int poleIdx = poleLow;
            Xyz sum = new(0.0, 0.0, 0.0);

            for (int k = 0; k <= _algDegree; ++k)
            {
                double basisN = 0.0;
                if (n <= k)
                {
                    if (u == 0.0)
                    {
                        basisN = (n == k) ? FallingFactorial(k, n) : 0.0;
                    }
                    else
                    {
                        basisN = FallingFactorial(k, n) * PowInt(u, k - n);
                    }
                }
                sum += _poles.Value(poleIdx++).XYZ() * basisN;
            }

            if (_alpha > 0.0)
            {
                double alphaU = _alpha * u;
                double alphaPow = PowInt(_alpha, n);
                double sinh = Math.Sinh(alphaU);
                double cosh = Math.Cosh(alphaU);
                bool isEvenN = (n % 2 == 0);
                double sinhN = alphaPow * (isEvenN ? sinh : cosh);
                double coshN = alphaPow * (isEvenN ? cosh : sinh);
                sum += _poles.Value(poleIdx++).XYZ() * sinhN;
                sum += _poles.Value(poleIdx++).XYZ() * coshN;
            }

            if (_beta > 0.0)
            {
                double betaU = _beta * u;
                double betaPow = PowInt(_beta, n);
                double sin = Math.Sin(betaU);
                double cos = Math.Cos(betaU);
                double sinN = 0.0;
                double cosN = 0.0;
                switch (n & 3)
                {
                    case 0:
                        sinN = sin;
                        cosN = cos;
                        break;
                    case 1:
                        sinN = cos;
                        cosN = -sin;
                        break;
                    case 2:
                        sinN = -sin;
                        cosN = -cos;
                        break;
                    case 3:
                        sinN = -cos;
                        cosN = sin;
                        break;
                }
                sum += _poles.Value(poleIdx++).XYZ() * (betaPow * sinN);
                sum += _poles.Value(poleIdx++).XYZ() * (betaPow * cosN);
            }

            return new Vector3D(sum);
        }

        // C++: NCollection_LocalArray<double, 16> and <gp_XYZ, 16>.
        Span<double> wDerivs = n + 1 <= 16 ? stackalloc double[16] : new double[n + 1];
        Span<Xyz> cDerivs = n + 1 <= 16 ? stackalloc Xyz[16] : new Xyz[n + 1];

        int weightLow = _weights.Lower();
        int polyCount = _algDegree + 1;
        int hyperIdx = poleLow + polyCount;
        int trigIdx = hyperIdx + (_alpha > 0.0 ? 2 : 0);

        // The C++ pointers to the poles, null when unused.
        Xyz poleSinh = new();
        Xyz poleCosh = new();
        Xyz poleSin = new();
        Xyz poleCos = new();
        double wiSinh = 0.0;
        double wiCosh = 0.0;
        double wiSin = 0.0;
        double wiCos = 0.0;

        if (_alpha > 0.0)
        {
            poleSinh = _poles.Value(hyperIdx).XYZ();
            poleCosh = _poles.Value(hyperIdx + 1).XYZ();
            wiSinh = _weights.Value(weightLow + polyCount);
            wiCosh = _weights.Value(weightLow + polyCount + 1);
        }

        if (_beta > 0.0)
        {
            int trigWIdx = weightLow + polyCount + (_alpha > 0.0 ? 2 : 0);
            poleSin = _poles.Value(trigIdx).XYZ();
            poleCos = _poles.Value(trigIdx + 1).XYZ();
            wiSin = _weights.Value(trigWIdx);
            wiCos = _weights.Value(trigWIdx + 1);
        }

        double alphaU2 = _alpha * u;
        double sinh2 = (_alpha > 0.0) ? Math.Sinh(alphaU2) : 0.0;
        double cosh2 = (_alpha > 0.0) ? Math.Cosh(alphaU2) : 0.0;
        double betaU2 = _beta * u;
        double sin2 = (_beta > 0.0) ? Math.Sin(betaU2) : 0.0;
        double cos2 = (_beta > 0.0) ? Math.Cos(betaU2) : 0.0;

        double alphaPow2 = 1.0;
        double betaPow2 = 1.0;
        double invW0 = 0.0;
        for (int d = 0; d <= n; ++d)
        {
            Xyz nd = new(0.0, 0.0, 0.0);
            double wd = 0.0;

            for (int k = 0; k <= _algDegree; ++k)
            {
                Xyz pole = _poles.Value(poleLow + k).XYZ();
                double wi = _weights.Value(weightLow + k);
                double basisD = 0.0;
                if (d <= k)
                {
                    if (u == 0.0)
                    {
                        basisD = (d == k) ? FallingFactorial(k, d) : 0.0;
                    }
                    else
                    {
                        basisD = FallingFactorial(k, d) * PowInt(u, k - d);
                    }
                }

                nd += pole * (wi * basisD);
                wd += wi * basisD;
            }

            if (_alpha > 0.0)
            {
                bool isEvenD = (d % 2 == 0);
                double dSinh = alphaPow2 * (isEvenD ? sinh2 : cosh2);
                double dCosh = alphaPow2 * (isEvenD ? cosh2 : sinh2);
                nd += poleSinh * (wiSinh * dSinh) + poleCosh * (wiCosh * dCosh);
                wd += wiSinh * dSinh + wiCosh * dCosh;
            }

            if (_beta > 0.0)
            {
                double dSin = 0.0;
                double dCos = 0.0;
                switch (d & 3)
                {
                    case 0:
                        dSin = sin2;
                        dCos = cos2;
                        break;
                    case 1:
                        dSin = cos2;
                        dCos = -sin2;
                        break;
                    case 2:
                        dSin = -sin2;
                        dCos = -cos2;
                        break;
                    case 3:
                        dSin = -cos2;
                        dCos = sin2;
                        break;
                }
                nd += poleSin * (wiSin * betaPow2 * dSin) + poleCos * (wiCos * betaPow2 * dCos);
                wd += wiSin * betaPow2 * dSin + wiCos * betaPow2 * dCos;
            }

            wDerivs[d] = wd;
            if (d == 0)
            {
                invW0 = 1.0 / wd;
                cDerivs[0] = nd * invW0;
            }
            else
            {
                Xyz sum = nd;
                double binom = 1.0;
                for (int j = 1; j <= d; ++j)
                {
                    binom = binom * (double)(d - j + 1) / (double)j;
                    sum -= cDerivs[d - j] * (binom * wDerivs[j]);
                }
                cDerivs[d] = sum * invW0;
            }

            alphaPow2 *= _alpha;
            betaPow2 *= _beta;
        }
        return new Vector3D(cDerivs[n]);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierCurve::Transform");
    }

    public override Geometry3D Copy()
    {
        if (_rational)
        {
            return new AhtBezierCurve(_poles, _weights, _algDegree, _alpha, _beta);
        }
        return new AhtBezierCurve(_poles, _algDegree, _alpha, _beta);
    }
}
