// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_TBezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_TBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A 2D trigonometric Bezier curve, over the basis {1, sin(alpha*t), cos(alpha*t), ..., sin(n*alpha*t),
/// cos(n*alpha*t)}: <c>C(t) = P_0*T_0(t) + ... + P_2n*T_2n(t)</c>, with T_0 = 1, T_2k-1 = sin(k*alpha*t) and
/// T_2k = cos(k*alpha*t). A curve of order n has 2n + 1 poles; the parameter domain is [0, Pi/alpha]. A rational
/// curve weights each pole: <c>C(t) = sum(w_i*P_i*T_i(t)) / sum(w_i*T_i(t))</c>.
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class Geom2dEval_TBezierCurve : Geom2d_BoundedCurve
{
    // The numerators (N) and denominators (W) of the derivatives up to the order evaluated.
    // OCCT's CurveBasisEval<theMaxOrder>.
    private struct CurveBasisEval
    {
        public gp_XY N0;
        public gp_XY N1;
        public gp_XY N2;
        public gp_XY N3;
        public double W0;
        public double W1;
        public double W2;
        public double W3;
    }

    private static double powInt(double theValue, int thePower)
    {
        double aRes = 1.0;
        for (int i = 0; i < thePower; ++i)
        {
            aRes *= theValue;
        }
        return aRes;
    }

    private static void evalTrigPairNthDeriv(double theSin, double theCos, int theOrderMod4,
                                             out double theSinDeriv, out double theCosDeriv)
    {
        switch (theOrderMod4)
        {
            case 0:
                theSinDeriv = theSin;
                theCosDeriv = theCos;
                break;
            case 1:
                theSinDeriv = theCos;
                theCosDeriv = -theSin;
                break;
            case 2:
                theSinDeriv = -theSin;
                theCosDeriv = -theCos;
                break;
            default:
                theSinDeriv = -theCos;
                theCosDeriv = theSin;
                break;
        }
    }

    // OCCT's template evalCurveBasis<theMaxOrder>: the `if constexpr` on the order are plain tests here.
    private static void evalCurveBasis(int theMaxOrder,
                                       NCollection_Array1<gp_Pnt2d> thePoles,
                                       NCollection_Array1<double>? theWeights,
                                       double theAlpha,
                                       double theU,
                                       out CurveBasisEval theData)
    {
        theData = new CurveBasisEval();
        int aNbPoles = thePoles.Length();
        ReadOnlySpan<gp_Pnt2d> aPoles = thePoles.AsSpan();
        ReadOnlySpan<double> aWeights = theWeights != null ? theWeights.AsSpan() : default;

        gp_XY aPole0 = aPoles[0].XY();
        if (theWeights == null)
        {
            theData.N0 += aPole0;
        }
        else
        {
            double aW0 = aWeights[0];
            theData.N0 += aPole0 * aW0;
            theData.W0 += aW0;
        }

        int anOrder = (aNbPoles - 1) / 2;
        if (anOrder <= 0)
        {
            return;
        }

        double aTheta = theAlpha * theU;
        double aSin1 = Math.Sin(aTheta);
        double aCos1 = Math.Cos(aTheta);
        double aSinK = aSin1;
        double aCosK = aCos1;
        double aFreq = theAlpha;
        int aIdx = 1;
        for (int k = 1; k <= anOrder; ++k, aIdx += 2, aFreq += theAlpha)
        {
            double aSin = aSinK;
            double aCos = aCosK;
            double aFreq2 = aFreq * aFreq;
            double aFreq3 = aFreq2 * aFreq;

            gp_XY aPoleSin = aPoles[aIdx].XY();
            gp_XY aPoleCos = aPoles[aIdx + 1].XY();

            double aB0Sin = aSin;
            double aB0Cos = aCos;

            double aB1Sin = 0.0;
            double aB1Cos = 0.0;
            double aB2Sin = 0.0;
            double aB2Cos = 0.0;
            double aB3Sin = 0.0;
            double aB3Cos = 0.0;
            if (theMaxOrder >= 1)
            {
                aB1Sin = aFreq * aCos;
                aB1Cos = -aFreq * aSin;
            }
            if (theMaxOrder >= 2)
            {
                aB2Sin = -aFreq2 * aSin;
                aB2Cos = -aFreq2 * aCos;
            }
            if (theMaxOrder >= 3)
            {
                aB3Sin = -aFreq3 * aCos;
                aB3Cos = aFreq3 * aSin;
            }

            if (theWeights == null)
            {
                theData.N0 += aPoleSin * aB0Sin + aPoleCos * aB0Cos;
                if (theMaxOrder >= 1)
                {
                    theData.N1 += aPoleSin * aB1Sin + aPoleCos * aB1Cos;
                }
                if (theMaxOrder >= 2)
                {
                    theData.N2 += aPoleSin * aB2Sin + aPoleCos * aB2Cos;
                }
                if (theMaxOrder >= 3)
                {
                    theData.N3 += aPoleSin * aB3Sin + aPoleCos * aB3Cos;
                }
            }
            else
            {
                double aWSin = aWeights[aIdx];
                double aWCos = aWeights[aIdx + 1];

                theData.N0 += aPoleSin * (aWSin * aB0Sin) + aPoleCos * (aWCos * aB0Cos);
                theData.W0 += aWSin * aB0Sin + aWCos * aB0Cos;
                if (theMaxOrder >= 1)
                {
                    theData.N1 += aPoleSin * (aWSin * aB1Sin) + aPoleCos * (aWCos * aB1Cos);
                    theData.W1 += aWSin * aB1Sin + aWCos * aB1Cos;
                }
                if (theMaxOrder >= 2)
                {
                    theData.N2 += aPoleSin * (aWSin * aB2Sin) + aPoleCos * (aWCos * aB2Cos);
                    theData.W2 += aWSin * aB2Sin + aWCos * aB2Cos;
                }
                if (theMaxOrder >= 3)
                {
                    theData.N3 += aPoleSin * (aWSin * aB3Sin) + aPoleCos * (aWCos * aB3Cos);
                    theData.W3 += aWSin * aB3Sin + aWCos * aB3Cos;
                }
            }

            if (k < anOrder)
            {
                double aSinNext = aSinK * aCos1 + aCosK * aSin1;
                double aCosNext = aCosK * aCos1 - aSinK * aSin1;
                aSinK = aSinNext;
                aCosK = aCosNext;
            }
        }
    }

    private NCollection_Array1<gp_Pnt2d> myPoles;  // Control points
    private NCollection_Array1<double> myWeights;  // Weights (empty if non-rational)
    private double myAlpha;                        // Frequency parameter
    private bool myRational;                       // True if curve is rational

    /// <summary>A non-rational T-Bezier curve.</summary>
    /// <param name="thePoles">The control points; their number must be odd and at least 3.</param>
    /// <param name="theAlpha">The frequency parameter (&gt; 0).</param>
    /// <exception cref="Standard_ConstructionError">The number of poles is even or below 3, or theAlpha &lt;= 0.</exception>
    public Geom2dEval_TBezierCurve(NCollection_Array1<gp_Pnt2d> thePoles, double theAlpha)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(thePoles);
        myWeights = new NCollection_Array1<double>();
        myAlpha = theAlpha;
        myRational = false;
        int aNbPoles = thePoles.Length();
        if (aNbPoles < 3 || (aNbPoles % 2) == 0)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_TBezierCurve: number of poles must be odd and >= 3");
        }
        if (theAlpha <= 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_TBezierCurve: alpha must be > 0");
        }
    }

    /// <summary>A rational T-Bezier curve.</summary>
    /// <param name="thePoles">The control points; their number must be odd and at least 3.</param>
    /// <param name="theWeights">The weights, as many as the poles, all &gt; 0.</param>
    /// <param name="theAlpha">The frequency parameter (&gt; 0).</param>
    /// <exception cref="Standard_ConstructionError">Any of the conditions above fails.</exception>
    public Geom2dEval_TBezierCurve(NCollection_Array1<gp_Pnt2d> thePoles, NCollection_Array1<double> theWeights,
                                   double theAlpha)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(thePoles);
        myWeights = new NCollection_Array1<double>(theWeights);
        myAlpha = theAlpha;
        myRational = true;
        int aNbPoles = thePoles.Length();
        if (aNbPoles < 3 || (aNbPoles % 2) == 0)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_TBezierCurve: number of poles must be odd and >= 3");
        }
        if (theAlpha <= 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_TBezierCurve: alpha must be > 0");
        }
        if (theWeights.Length() != aNbPoles)
        {
            throw new Standard_ConstructionError("Geom2dEval_TBezierCurve: weights size must equal poles size");
        }
        for (int i = theWeights.Lower(); i <= theWeights.Upper(); ++i)
        {
            if (theWeights.Value(i) <= 0.0)
            {
                throw new Standard_ConstructionError("Geom2dEval_TBezierCurve: all weights must be > 0");
            }
        }
    }

    /// <summary>The poles (a copy, indexed from 0).</summary>
    public PackedList<gp_Pnt2d> Poles() => myPoles.ToPackedList();

    /// <summary>The weights (a copy, indexed from 0), empty if the curve is non-rational.</summary>
    public PackedList<double> Weights() => myWeights.ToPackedList();

    /// <summary>The frequency parameter alpha.</summary>
    public double Alpha() => myAlpha;

    /// <summary>The number of poles.</summary>
    public int NbPoles() => myPoles.Length();

    /// <summary>The trigonometric order n (NbPoles = 2*n + 1).</summary>
    public int Order() => (myPoles.Length() - 1) / 2;

    /// <summary>True if the curve is rational.</summary>
    public bool IsRational() => myRational;

    /// <summary>The start point C(0).</summary>
    public override gp_Pnt2d StartPoint() => EvalD0(0.0);

    /// <summary>The end point C(Pi/alpha).</summary>
    public override gp_Pnt2d EndPoint() => EvalD0(M_PI / myAlpha);

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_TBezierCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("Geom2dEval_TBezierCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

    /// <summary>Pi/alpha.</summary>
    public override double LastParameter() => M_PI / myAlpha;

    /// <summary>True if the start and end points coincide (within Precision::Confusion()).</summary>
    public override bool IsClosed() => StartPoint().Distance(EndPoint()) <= Precision.Confusion();

    public override bool IsPeriodic() => false;

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override bool IsCN(int N) => true;

    // Evaluate trigonometric basis functions at parameter theT: {T_0(t), T_1(t), ..., T_{2n}(t)}.
    private void evalBasis(double theT, NCollection_Array1<double> theBasis)
    {
        // T_0(t) = 1
        Span<double> aBasis = theBasis.AsSpan();
        aBasis[0] = 1.0;
        int anOrder = Order();
        if (anOrder <= 0)
        {
            return;
        }

        double aTheta = myAlpha * theT;
        double aSin1 = Math.Sin(aTheta);
        double aCos1 = Math.Cos(aTheta);
        double aSinK = aSin1;
        double aCosK = aCos1;
        for (int k = 1; k <= anOrder; ++k)
        {
            aBasis[2 * k - 1] = aSinK;
            aBasis[2 * k] = aCosK;
            if (k < anOrder)
            {
                double aSinNext = aSinK * aCos1 + aCosK * aSin1;
                double aCosNext = aCosK * aCos1 - aSinK * aSin1;
                aSinK = aSinNext;
                aCosK = aCosNext;
            }
        }
    }

    // Evaluate derivatives of order theDerivOrder of the basis functions.
    private void evalBasisDeriv(double theT, int theDerivOrder, NCollection_Array1<double> theBasisDeriv)
    {
        // Derivative of constant basis function T_0 = 1 is always 0.
        Span<double> aBasis = theBasisDeriv.AsSpan();
        aBasis[0] = 0.0;

        int anOrder = Order();
        if (anOrder <= 0)
        {
            return;
        }

        int anOrderMod4 = theDerivOrder & 3;
        double aTheta = myAlpha * theT;
        double aSin1 = Math.Sin(aTheta);
        double aCos1 = Math.Cos(aTheta);
        double aSinK = aSin1;
        double aCosK = aCos1;
        double aFreq = myAlpha;
        for (int k = 1; k <= anOrder; ++k)
        {
            double aSin = aSinK;
            double aCos = aCosK;
            double aFreqPow = powInt(aFreq, theDerivOrder);
            evalTrigPairNthDeriv(aSin, aCos, anOrderMod4, out double aSinDeriv, out double aCosDeriv);
            aBasis[2 * k - 1] = aFreqPow * aSinDeriv;
            aBasis[2 * k] = aFreqPow * aCosDeriv;
            aFreq += myAlpha;
            if (k < anOrder)
            {
                double aSinNext = aSinK * aCos1 + aCosK * aSin1;
                double aCosNext = aCosK * aCos1 - aSinK * aSin1;
                aSinK = aSinNext;
                aCosK = aCosNext;
            }
        }
    }

    // Evaluate point on a non-rational curve: sum(P_i * B_i(t)). Unused, as in OCCT.
    private gp_Pnt2d evalNonRationalPoint(NCollection_Array1<double> theBasis)
    {
        gp_XY aSum = new(0.0, 0.0);
        int aNbPoles = myPoles.Length();
        ReadOnlySpan<gp_Pnt2d> aPolesData = myPoles.AsSpan();
        ReadOnlySpan<double> aBasisData = theBasis.AsSpan();
        for (int i = 0; i < aNbPoles; ++i)
        {
            aSum += aBasisData[i] * aPolesData[i].XY();
        }
        return new gp_Pnt2d(aSum);
    }

    // Evaluate derivative vector on a non-rational curve: sum(P_i * B_i'(t)). Unused, as in OCCT.
    private gp_Vec2d evalNonRationalDeriv(NCollection_Array1<double> theBasisDeriv)
    {
        gp_XY aSum = new(0.0, 0.0);
        int aNbPoles = myPoles.Length();
        ReadOnlySpan<gp_Pnt2d> aPolesData = myPoles.AsSpan();
        ReadOnlySpan<double> aBasisData = theBasisDeriv.AsSpan();
        for (int i = 0; i < aNbPoles; ++i)
        {
            aSum += aBasisData[i] * aPolesData[i].XY();
        }
        return new gp_Vec2d(aSum);
    }

    public override gp_Pnt2d EvalD0(double U)
    {
        CurveBasisEval aData;
        if (!myRational)
        {
            evalCurveBasis(0, myPoles, null, myAlpha, U, out aData);
            return new gp_Pnt2d(aData.N0);
        }
        evalCurveBasis(0, myPoles, myWeights, myAlpha, U, out aData);
        return new gp_Pnt2d(aData.N0 / aData.W0);
    }

    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        CurveBasisEval aData;

        if (!myRational)
        {
            evalCurveBasis(1, myPoles, null, myAlpha, U, out aData);
            aResult.Point = new gp_Pnt2d(aData.N0);
            aResult.D1 = new gp_Vec2d(aData.N1);
            return aResult;
        }

        evalCurveBasis(1, myPoles, myWeights, myAlpha, U, out aData);
        gp_XY aC = aData.N0 / aData.W0;
        aResult.Point = new gp_Pnt2d(aC);
        aResult.D1 = new gp_Vec2d((aData.N1 - aData.W1 * aC) / aData.W0);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        CurveBasisEval aData;

        if (!myRational)
        {
            evalCurveBasis(2, myPoles, null, myAlpha, U, out aData);
            aResult.Point = new gp_Pnt2d(aData.N0);
            aResult.D1 = new gp_Vec2d(aData.N1);
            aResult.D2 = new gp_Vec2d(aData.N2);
            return aResult;
        }

        evalCurveBasis(2, myPoles, myWeights, myAlpha, U, out aData);
        gp_XY aC = aData.N0 / aData.W0;
        gp_XY aCD1 = (aData.N1 - aData.W1 * aC) / aData.W0;
        gp_XY aCD2 = (aData.N2 - 2.0 * aData.W1 * aCD1 - aData.W2 * aC) / aData.W0;

        aResult.Point = new gp_Pnt2d(aC);
        aResult.D1 = new gp_Vec2d(aCD1);
        aResult.D2 = new gp_Vec2d(aCD2);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
        CurveBasisEval aData;

        if (!myRational)
        {
            evalCurveBasis(3, myPoles, null, myAlpha, U, out aData);
            aResult.Point = new gp_Pnt2d(aData.N0);
            aResult.D1 = new gp_Vec2d(aData.N1);
            aResult.D2 = new gp_Vec2d(aData.N2);
            aResult.D3 = new gp_Vec2d(aData.N3);
            return aResult;
        }

        evalCurveBasis(3, myPoles, myWeights, myAlpha, U, out aData);
        gp_XY aC = aData.N0 / aData.W0;
        gp_XY aCD1 = (aData.N1 - aData.W1 * aC) / aData.W0;
        gp_XY aCD2 = (aData.N2 - 2.0 * aData.W1 * aCD1 - aData.W2 * aC) / aData.W0;
        gp_XY aCD3 =
            (aData.N3 - 3.0 * aData.W2 * aCD1 - 3.0 * aData.W1 * aCD2 - aData.W3 * aC) / aData.W0;

        aResult.Point = new gp_Pnt2d(aC);
        aResult.D1 = new gp_Vec2d(aCD1);
        aResult.D2 = new gp_Vec2d(aCD2);
        aResult.D3 = new gp_Vec2d(aCD3);
        return aResult;
    }

    /// <exception cref="Standard_RangeError">N &lt; 1.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Standard_RangeError("Geom2dEval_TBezierCurve::EvalDN: derivative order must be >= 1");
        }

        // For non-rational curves, compute directly.
        if (!myRational)
        {
            gp_XY aSum = new(0.0, 0.0);
            int anOrder = Order();
            int aOrderMod4 = N & 3;
            if (anOrder <= 0)
            {
                return new gp_Vec2d(aSum);
            }

            ReadOnlySpan<gp_Pnt2d> aPolesData = myPoles.AsSpan();
            double aTheta = myAlpha * U;
            double aSin1 = Math.Sin(aTheta);
            double aCos1 = Math.Cos(aTheta);
            double aSinK = aSin1;
            double aCosK = aCos1;
            double aFreq = myAlpha;
            int aIdx = 1;

            for (int k = 1; k <= anOrder; ++k, aIdx += 2, aFreq += myAlpha)
            {
                double aFreqPow = powInt(aFreq, N);
                double aSin = aSinK;
                double aCos = aCosK;

                evalTrigPairNthDeriv(aSin, aCos, aOrderMod4, out double aSinDeriv, out double aCosDeriv);

                aSum += aPolesData[aIdx].XY() * (aFreqPow * aSinDeriv);
                aSum += aPolesData[aIdx + 1].XY() * (aFreqPow * aCosDeriv);

                if (k < anOrder)
                {
                    double aSinNext = aSinK * aCos1 + aCosK * aSin1;
                    double aCosNext = aCosK * aCos1 - aSinK * aSin1;
                    aSinK = aSinNext;
                    aCosK = aCosNext;
                }
            }
            return new gp_Vec2d(aSum);
        }

        // For rational curves with N <= 3, delegate to the full evaluation.
        if (N == 1)
        {
            return EvalD1(U).D1;
        }
        if (N == 2)
        {
            return EvalD2(U).D2;
        }
        if (N == 3)
        {
            return EvalD3(U).D3;
        }

        // For N > 3 on rational curves, use the general recurrence formula:
        // C^(n)(t) = (Cw^(n)(t) - sum_{k=1..n} C(n,k)*w^(k)(t)*C^(n-k)(t)) / w(t)
        int aNbPoles = myPoles.Length();

        // Precompute all basis derivative arrays up to order N.
        NCollection_Array1<NCollection_Array1<double>> aBasisDerivs = new(0, N);
        for (int d = 0; d <= N; ++d)
        {
            // OCCT resizes a default-constructed element; C# elements of a class type start null.
            aBasisDerivs[d] = new NCollection_Array1<double>(1, aNbPoles);
            if (d == 0)
            {
                evalBasis(U, aBasisDerivs[0]);
            }
            else
            {
                evalBasisDeriv(U, d, aBasisDerivs[d]);
            }
        }

        // Compute w^(d) and Cw^(d) for d = 0..N.
        NCollection_Array1<double> aWDerivs = new(0, N);
        NCollection_Array1<gp_XY> aCwDerivs = new(0, N);
        ReadOnlySpan<gp_Pnt2d> aPolesData2 = myPoles.AsSpan();
        ReadOnlySpan<double> aWeightsData = myWeights.AsSpan();
        for (int d = 0; d <= N; ++d)
        {
            NCollection_Array1<double> aBasisD = aBasisDerivs.Value(d);
            ReadOnlySpan<double> aBasisData = aBasisD.AsSpan(1);
            double aWd = 0.0;
            gp_XY aCwd = new(0.0, 0.0);
            for (int i = 0; i < aNbPoles; ++i)
            {
                double aWi = aWeightsData[i];
                double aBd = aBasisData[i];
                gp_XY aPi = aPolesData2[i].XY();
                aCwd += (aWi * aBd) * aPi;
                aWd += aWi * aBd;
            }
            aWDerivs.SetValue(d, aWd);
            aCwDerivs.SetValue(d, aCwd);
        }

        // Compute C^(d) for d = 0..N using the recurrence.
        NCollection_Array1<gp_XY> aCDerivs = new(0, N);
        aCDerivs.SetValue(0, aCwDerivs.Value(0) / aWDerivs.Value(0));

        for (int d = 1; d <= N; ++d)
        {
            gp_XY aSum = new(0.0, 0.0);
            // Binomial coefficients via incremental computation.
            double aBinom = 1.0;
            for (int k = 1; k <= d; ++k)
            {
                aBinom *= (double)(d - k + 1) / (double)k;
                aSum += aBinom * aWDerivs.Value(k) * aCDerivs.Value(d - k);
            }
            aCDerivs.SetValue(d, (aCwDerivs.Value(d) - aSum) / aWDerivs.Value(0));
        }

        return new gp_Vec2d(aCDerivs.Value(N));
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_TBezierCurve::Transform");
    }

    public override Geom2d_Geometry Copy()
    {
        if (myRational)
        {
            return new Geom2dEval_TBezierCurve(myPoles, myWeights, myAlpha);
        }
        return new Geom2dEval_TBezierCurve(myPoles, myAlpha);
    }
}
