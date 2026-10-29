// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_AHTBezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_AHTBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A 2D Algebraic-Hyperbolic-Trigonometric Bezier curve, over the mixed basis
/// {1, t, ..., t^k, sinh(alpha*t), cosh(alpha*t), sin(beta*t), cos(beta*t)}, where the hyperbolic pair is present
/// when alpha &gt; 0 and the trigonometric pair when beta &gt; 0. The number of poles must equal the number of
/// basis functions, algDegree + 1 + 2*(alpha&gt;0) + 2*(beta&gt;0). The parameter range is [0, 1].
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class Geom2dEval_AHTBezierCurve : Geom2d_BoundedCurve
{
    // The numerators (N) and denominators (W) of the derivatives up to the order evaluated.
    private struct PolyEvalData
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

    private static double powInt(double theBase, int theExp)
    {
        double aRes = 1.0;
        for (int i = 0; i < theExp; ++i)
        {
            aRes *= theBase;
        }
        return aRes;
    }

    private static double fallingFactorial(int theN, int theK)
    {
        double aRes = 1.0;
        for (int i = 0; i < theK; ++i)
        {
            aRes *= (double)(theN - i);
        }
        return aRes;
    }

    // The C++ template evalPolynomialNonRational<theMaxOrder>.
    private static void evalPolynomialNonRational(int theMaxOrder,
                                                  NCollection_Array1<gp_Pnt2d> thePoles,
                                                  int theDegree,
                                                  double theU,
                                                  out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        ReadOnlySpan<gp_Pnt2d> aPoles = thePoles.AsSpan();
        theData.N0 = aPoles[theDegree].XY();

        for (int k = theDegree - 1; k >= 0; --k)
        {
            if (theMaxOrder >= 3)
            {
                theData.N3 = theData.N3 * theU + theData.N2 * 3.0;
            }
            if (theMaxOrder >= 2)
            {
                theData.N2 = theData.N2 * theU + theData.N1 * 2.0;
            }
            if (theMaxOrder >= 1)
            {
                theData.N1 = theData.N1 * theU + theData.N0;
            }
            theData.N0 = theData.N0 * theU + aPoles[k].XY();
        }
    }

    // The C++ template evalPolynomialRational<theMaxOrder>.
    private static void evalPolynomialRational(int theMaxOrder,
                                               NCollection_Array1<gp_Pnt2d> thePoles,
                                               NCollection_Array1<double> theWeights,
                                               int theDegree,
                                               double theU,
                                               out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        ReadOnlySpan<gp_Pnt2d> aPoles = thePoles.AsSpan();
        ReadOnlySpan<double> aWeights = theWeights.AsSpan();

        double aWLead = aWeights[theDegree];
        theData.N0 = aPoles[theDegree].XY() * aWLead;
        theData.W0 = aWLead;

        for (int k = theDegree - 1; k >= 0; --k)
        {
            if (theMaxOrder >= 3)
            {
                theData.N3 = theData.N3 * theU + theData.N2 * 3.0;
                theData.W3 = theData.W3 * theU + theData.W2 * 3.0;
            }
            if (theMaxOrder >= 2)
            {
                theData.N2 = theData.N2 * theU + theData.N1 * 2.0;
                theData.W2 = theData.W2 * theU + theData.W1 * 2.0;
            }
            if (theMaxOrder >= 1)
            {
                theData.N1 = theData.N1 * theU + theData.N0;
                theData.W1 = theData.W1 * theU + theData.W0;
            }

            double aWk = aWeights[k];
            theData.N0 = theData.N0 * theU + aPoles[k].XY() * aWk;
            theData.W0 = theData.W0 * theU + aWk;
        }
    }

    // The C++ template evalMixedAHT<theMaxOrder, theIsRational>.
    private static void evalMixedAHT(int theMaxOrder,
                                     bool theIsRational,
                                     NCollection_Array1<gp_Pnt2d> thePoles,
                                     NCollection_Array1<double> theWeights,
                                     int theAlgDegree,
                                     double theAlpha,
                                     double theBeta,
                                     double theU,
                                     out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        ReadOnlySpan<gp_Pnt2d> aPoles = thePoles.AsSpan();
        ReadOnlySpan<double> aWeights = default;
        if (theIsRational)
        {
            aWeights = theWeights.AsSpan();
        }
        int aPoleIdx = 0;
        int aWeightIdx = 0;

        double aPow = 1.0;
        double aPowM1 = 0.0;
        double aPowM2 = 0.0;
        double aPowM3 = 0.0;
        for (int k = 0; k <= theAlgDegree; ++k)
        {
            gp_XY aPole = aPoles[aPoleIdx++].XY();
            double aB0 = aPow;
            double aB1 = 0.0;
            double aB2 = 0.0;
            double aB3 = 0.0;
            if (theMaxOrder >= 1)
            {
                aB1 = (k >= 1) ? (double)k * aPowM1 : 0.0;
            }
            if (theMaxOrder >= 2)
            {
                aB2 = (k >= 2) ? (double)k * (double)(k - 1) * aPowM2 : 0.0;
            }
            if (theMaxOrder >= 3)
            {
                aB3 = (k >= 3) ? (double)k * (double)(k - 1) * (double)(k - 2) * aPowM3 : 0.0;
            }

            if (theIsRational)
            {
                double aWi = aWeights[aWeightIdx++];
                theData.N0 += aPole * (aWi * aB0);
                theData.W0 += aWi * aB0;
                if (theMaxOrder >= 1)
                {
                    theData.N1 += aPole * (aWi * aB1);
                    theData.W1 += aWi * aB1;
                }
                if (theMaxOrder >= 2)
                {
                    theData.N2 += aPole * (aWi * aB2);
                    theData.W2 += aWi * aB2;
                }
                if (theMaxOrder >= 3)
                {
                    theData.N3 += aPole * (aWi * aB3);
                    theData.W3 += aWi * aB3;
                }
            }
            else
            {
                theData.N0 += aPole * aB0;
                if (theMaxOrder >= 1)
                {
                    theData.N1 += aPole * aB1;
                }
                if (theMaxOrder >= 2)
                {
                    theData.N2 += aPole * aB2;
                }
                if (theMaxOrder >= 3)
                {
                    theData.N3 += aPole * aB3;
                }
            }

            aPowM3 = aPowM2;
            aPowM2 = aPowM1;
            aPowM1 = aPow;
            aPow *= theU;
        }

        if (theAlpha > 0.0)
        {
            double aAlphaU = theAlpha * theU;
            double aSinh = Math.Sinh(aAlphaU);
            double aCosh = Math.Cosh(aAlphaU);
            double aAlpha2 = theAlpha * theAlpha;
            double aAlpha3 = aAlpha2 * theAlpha;

            gp_XY aPoleSinh = aPoles[aPoleIdx++].XY();
            gp_XY aPoleCosh = aPoles[aPoleIdx++].XY();
            double aWiSinh = 1.0;
            double aWiCosh = 1.0;
            if (theIsRational)
            {
                aWiSinh = aWeights[aWeightIdx++];
                aWiCosh = aWeights[aWeightIdx++];
            }

            double aB0Sinh = aSinh;
            double aB0Cosh = aCosh;
            double aB1Sinh = 0.0;
            double aB1Cosh = 0.0;
            double aB2Sinh = 0.0;
            double aB2Cosh = 0.0;
            double aB3Sinh = 0.0;
            double aB3Cosh = 0.0;
            if (theMaxOrder >= 1)
            {
                aB1Sinh = theAlpha * aCosh;
                aB1Cosh = theAlpha * aSinh;
            }
            if (theMaxOrder >= 2)
            {
                aB2Sinh = aAlpha2 * aSinh;
                aB2Cosh = aAlpha2 * aCosh;
            }
            if (theMaxOrder >= 3)
            {
                aB3Sinh = aAlpha3 * aCosh;
                aB3Cosh = aAlpha3 * aSinh;
            }

            theData.N0 += aPoleSinh * (aWiSinh * aB0Sinh) + aPoleCosh * (aWiCosh * aB0Cosh);
            if (theIsRational)
            {
                theData.W0 += aWiSinh * aB0Sinh + aWiCosh * aB0Cosh;
            }
            if (theMaxOrder >= 1)
            {
                theData.N1 += aPoleSinh * (aWiSinh * aB1Sinh) + aPoleCosh * (aWiCosh * aB1Cosh);
                if (theIsRational)
                {
                    theData.W1 += aWiSinh * aB1Sinh + aWiCosh * aB1Cosh;
                }
            }
            if (theMaxOrder >= 2)
            {
                theData.N2 += aPoleSinh * (aWiSinh * aB2Sinh) + aPoleCosh * (aWiCosh * aB2Cosh);
                if (theIsRational)
                {
                    theData.W2 += aWiSinh * aB2Sinh + aWiCosh * aB2Cosh;
                }
            }
            if (theMaxOrder >= 3)
            {
                theData.N3 += aPoleSinh * (aWiSinh * aB3Sinh) + aPoleCosh * (aWiCosh * aB3Cosh);
                if (theIsRational)
                {
                    theData.W3 += aWiSinh * aB3Sinh + aWiCosh * aB3Cosh;
                }
            }
        }

        if (theBeta > 0.0)
        {
            double aBetaU = theBeta * theU;
            double aSin = Math.Sin(aBetaU);
            double aCos = Math.Cos(aBetaU);
            double aBeta2 = theBeta * theBeta;
            double aBeta3 = aBeta2 * theBeta;

            gp_XY aPoleSin = aPoles[aPoleIdx++].XY();
            gp_XY aPoleCos = aPoles[aPoleIdx++].XY();
            double aWiSin = 1.0;
            double aWiCos = 1.0;
            if (theIsRational)
            {
                aWiSin = aWeights[aWeightIdx++];
                aWiCos = aWeights[aWeightIdx++];
            }

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
                aB1Sin = theBeta * aCos;
                aB1Cos = -theBeta * aSin;
            }
            if (theMaxOrder >= 2)
            {
                aB2Sin = -aBeta2 * aSin;
                aB2Cos = -aBeta2 * aCos;
            }
            if (theMaxOrder >= 3)
            {
                aB3Sin = -aBeta3 * aCos;
                aB3Cos = aBeta3 * aSin;
            }

            theData.N0 += aPoleSin * (aWiSin * aB0Sin) + aPoleCos * (aWiCos * aB0Cos);
            if (theIsRational)
            {
                theData.W0 += aWiSin * aB0Sin + aWiCos * aB0Cos;
            }
            if (theMaxOrder >= 1)
            {
                theData.N1 += aPoleSin * (aWiSin * aB1Sin) + aPoleCos * (aWiCos * aB1Cos);
                if (theIsRational)
                {
                    theData.W1 += aWiSin * aB1Sin + aWiCos * aB1Cos;
                }
            }
            if (theMaxOrder >= 2)
            {
                theData.N2 += aPoleSin * (aWiSin * aB2Sin) + aPoleCos * (aWiCos * aB2Cos);
                if (theIsRational)
                {
                    theData.W2 += aWiSin * aB2Sin + aWiCos * aB2Cos;
                }
            }
            if (theMaxOrder >= 3)
            {
                theData.N3 += aPoleSin * (aWiSin * aB3Sin) + aPoleCos * (aWiCos * aB3Cos);
                if (theIsRational)
                {
                    theData.W3 += aWiSin * aB3Sin + aWiCos * aB3Cos;
                }
            }
        }
    }

    private NCollection_Array1<gp_Pnt2d> myPoles;
    private NCollection_Array1<double> myWeights;
    private int myAlgDegree;
    private double myAlpha;
    private double myBeta;
    private bool myRational;

    // The number of basis functions for the given parameters.
    private static int basisDimension(int theAlgDegree, double theAlpha, double theBeta) =>
        theAlgDegree + 1 + (theAlpha > 0.0 ? 2 : 0) + (theBeta > 0.0 ? 2 : 0);

    /// <summary>A non-rational AHT-Bezier curve.</summary>
    /// <param name="thePoles">The control points, as many as the basis functions.</param>
    /// <param name="theAlgDegree">The degree of the polynomial part (&gt;= 0).</param>
    /// <param name="theAlpha">The hyperbolic frequency (&gt;= 0; 0 leaves the hyperbolic pair out).</param>
    /// <param name="theBeta">The trigonometric frequency (&gt;= 0; 0 leaves the trigonometric pair out).</param>
    /// <exception cref="Standard_ConstructionError">A parameter is out of range or the number of poles is wrong.</exception>
    public Geom2dEval_AHTBezierCurve(NCollection_Array1<gp_Pnt2d> thePoles, int theAlgDegree, double theAlpha,
                                     double theBeta)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(thePoles);
        // OCCT: a non-rational curve has one weight, left uninitialised (0 here), which Weights() returns.
        myWeights = new NCollection_Array1<double>(1, 1);
        myAlgDegree = theAlgDegree;
        myAlpha = theAlpha;
        myBeta = theBeta;
        myRational = false;
        if (theAlgDegree < 0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (theAlpha < 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (theBeta < 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: beta must be >= 0");
        }
        int aDim = basisDimension(theAlgDegree, theAlpha, theBeta);
        if (thePoles.Length() != aDim)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
    }

    /// <summary>A rational AHT-Bezier curve.</summary>
    /// <param name="thePoles">The control points, as many as the basis functions.</param>
    /// <param name="theWeights">The weights, as many as the poles, all &gt; 0.</param>
    /// <param name="theAlgDegree">The degree of the polynomial part (&gt;= 0).</param>
    /// <param name="theAlpha">The hyperbolic frequency (&gt;= 0).</param>
    /// <param name="theBeta">The trigonometric frequency (&gt;= 0).</param>
    /// <exception cref="Standard_ConstructionError">A parameter is out of range or an array has the wrong size.</exception>
    public Geom2dEval_AHTBezierCurve(NCollection_Array1<gp_Pnt2d> thePoles, NCollection_Array1<double> theWeights,
                                     int theAlgDegree, double theAlpha, double theBeta)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(thePoles);
        myWeights = new NCollection_Array1<double>(theWeights);
        myAlgDegree = theAlgDegree;
        myAlpha = theAlpha;
        myBeta = theBeta;
        myRational = true;
        if (theAlgDegree < 0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (theAlpha < 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (theBeta < 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: beta must be >= 0");
        }
        int aDim = basisDimension(theAlgDegree, theAlpha, theBeta);
        if (thePoles.Length() != aDim)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
        if (theWeights.Length() != thePoles.Length())
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_AHTBezierCurve: weights array size must match poles array size");
        }
        for (int anIdx = theWeights.Lower(); anIdx <= theWeights.Upper(); ++anIdx)
        {
            if (theWeights.Value(anIdx) <= 0.0)
            {
                throw new Standard_ConstructionError("Geom2dEval_AHTBezierCurve: all weights must be > 0");
            }
        }
    }

    /// <summary>The poles (a copy, indexed from 0).</summary>
    public PackedList<gp_Pnt2d> Poles() => myPoles.ToPackedList();

    /// <summary>The weights (a copy, indexed from 0). A non-rational curve has one weight, 0, as OCCT's.</summary>
    public PackedList<double> Weights() => myWeights.ToPackedList();

    /// <summary>The degree of the polynomial part.</summary>
    public int AlgDegree() => myAlgDegree;

    /// <summary>The hyperbolic frequency.</summary>
    public double Alpha() => myAlpha;

    /// <summary>The trigonometric frequency.</summary>
    public double Beta() => myBeta;

    /// <summary>The number of poles.</summary>
    public int NbPoles() => myPoles.Length();

    /// <summary>True if the curve is rational.</summary>
    public bool IsRational() => myRational;

    /// <summary>The start point C(0).</summary>
    public override gp_Pnt2d StartPoint() => EvalD0(0.0);

    /// <summary>The end point C(1).</summary>
    public override gp_Pnt2d EndPoint() => EvalD0(1.0);

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_AHTBezierCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("Geom2dEval_AHTBezierCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public override double LastParameter() => 1.0;

    /// <summary>True if the start and end points coincide (within Precision::Confusion()).</summary>
    public override bool IsClosed() => StartPoint().Distance(EndPoint()) <= Precision.Confusion();

    public override bool IsPeriodic() => false;

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override bool IsCN(int N) => true;

    // Evaluates all basis functions at theT. Unused, as in OCCT.
    private void evalBasis(double theT, NCollection_Array1<double> theBasis)
    {
        int aLower = theBasis.Lower();
        int anIdx = aLower;

        // Polynomial part: 1, t, t^2, ..., t^k
        double aPow = 1.0;
        for (int k = 0; k <= myAlgDegree; ++k)
        {
            theBasis[anIdx++] = aPow;
            aPow *= theT;
        }

        // Hyperbolic part: sinh(alpha*t), cosh(alpha*t)
        if (myAlpha > 0.0)
        {
            double aAlphaT = myAlpha * theT;
            theBasis[anIdx++] = Math.Sinh(aAlphaT);
            theBasis[anIdx++] = Math.Cosh(aAlphaT);
        }

        // Trigonometric part: sin(beta*t), cos(beta*t)
        if (myBeta > 0.0)
        {
            double aBetaT = myBeta * theT;
            theBasis[anIdx++] = Math.Sin(aBetaT);
            theBasis[anIdx++] = Math.Cos(aBetaT);
        }
    }

    // Evaluates the derivatives of order theDerivOrder of all basis functions at theT. Unused, as in OCCT.
    private void evalBasisDeriv(double theT, int theDerivOrder, NCollection_Array1<double> theBasisDeriv)
    {
        int aLower = theBasisDeriv.Lower();
        int anIdx = aLower;

        // Polynomial part: d^d/dt^d [t^k] = k!/(k-d)! * t^(k-d) if d <= k, else 0
        for (int k = 0; k <= myAlgDegree; ++k)
        {
            if (theDerivOrder > k)
            {
                theBasisDeriv[anIdx++] = 0.0;
            }
            else
            {
                // Compute falling factorial: k * (k-1) * ... * (k-d+1)
                double aCoeff = 1.0;
                for (int j = 0; j < theDerivOrder; ++j)
                {
                    aCoeff *= (k - j);
                }
                theBasisDeriv[anIdx++] = aCoeff * Math.Pow(theT, k - theDerivOrder);
            }
        }

        // Hyperbolic part:
        // d^d/dt^d [sinh(alpha*t)] = alpha^d * (d%2==0 ? sinh(alpha*t) : cosh(alpha*t))
        // d^d/dt^d [cosh(alpha*t)] = alpha^d * (d%2==0 ? cosh(alpha*t) : sinh(alpha*t))
        if (myAlpha > 0.0)
        {
            double aAlphaT = myAlpha * theT;
            double aAlphaPow = powInt(myAlpha, theDerivOrder);
            double aSinh = Math.Sinh(aAlphaT);
            double aCosh = Math.Cosh(aAlphaT);
            bool isEvenDeriv = (theDerivOrder % 2 == 0);

            theBasisDeriv[anIdx++] = aAlphaPow * (isEvenDeriv ? aSinh : aCosh);
            theBasisDeriv[anIdx++] = aAlphaPow * (isEvenDeriv ? aCosh : aSinh);
        }

        // Trigonometric part:
        // d^d/dt^d [sin(beta*t)] = beta^d * sin(beta*t + d*PI/2)
        // d^d/dt^d [cos(beta*t)] = beta^d * cos(beta*t + d*PI/2)
        if (myBeta > 0.0)
        {
            double aBetaT = myBeta * theT;
            double aBetaPow = powInt(myBeta, theDerivOrder);
            double aPhase = theDerivOrder * M_PI / 2.0;

            theBasisDeriv[anIdx++] = aBetaPow * Math.Sin(aBetaT + aPhase);
            theBasisDeriv[anIdx++] = aBetaPow * Math.Cos(aBetaT + aPhase);
        }
    }

    public override gp_Pnt2d EvalD0(double U)
    {
        if (myAlpha <= 0.0 && myBeta <= 0.0)
        {
            PolyEvalData aPoly;
            if (!myRational)
            {
                evalPolynomialNonRational(0, myPoles, myAlgDegree, U, out aPoly);
                return new gp_Pnt2d(aPoly.N0);
            }

            evalPolynomialRational(0, myPoles, myWeights, myAlgDegree, U, out aPoly);
            return new gp_Pnt2d(aPoly.N0 / aPoly.W0);
        }

        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(0, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            return new gp_Pnt2d(aMixed.N0);
        }

        evalMixedAHT(0, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        return new gp_Pnt2d(aMixed.N0 / aMixed.W0);
    }

    public override ResD1 EvalD1(double U)
    {
        if (myAlpha <= 0.0 && myBeta <= 0.0)
        {
            PolyEvalData aPoly;
            ResD1 aPolyResult;
            if (!myRational)
            {
                evalPolynomialNonRational(1, myPoles, myAlgDegree, U, out aPoly);
                aPolyResult.Point = new gp_Pnt2d(aPoly.N0);
                aPolyResult.D1 = new gp_Vec2d(aPoly.N1);
                return aPolyResult;
            }

            evalPolynomialRational(1, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            aPolyResult.Point = new gp_Pnt2d(aPoly.N0 * aPolyInvW);
            aPolyResult.D1 = new gp_Vec2d((aPoly.N1 * aPoly.W0 - aPoly.N0 * aPoly.W1) * (aPolyInvW * aPolyInvW));
            return aPolyResult;
        }

        ResD1 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(1, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new gp_Pnt2d(aMixed.N0);
            aResult.D1 = new gp_Vec2d(aMixed.N1);
            return aResult;
        }

        evalMixedAHT(1, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        aResult.Point = new gp_Pnt2d(aMixed.N0 * aInvW);
        aResult.D1 = new gp_Vec2d((aMixed.N1 * aMixed.W0 - aMixed.N0 * aMixed.W1) * (aInvW * aInvW));
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        if (myAlpha <= 0.0 && myBeta <= 0.0)
        {
            PolyEvalData aPoly;
            ResD2 aPolyResult;
            if (!myRational)
            {
                evalPolynomialNonRational(2, myPoles, myAlgDegree, U, out aPoly);
                aPolyResult.Point = new gp_Pnt2d(aPoly.N0);
                aPolyResult.D1 = new gp_Vec2d(aPoly.N1);
                aPolyResult.D2 = new gp_Vec2d(aPoly.N2);
                return aPolyResult;
            }

            evalPolynomialRational(2, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            gp_XY aPolyC = aPoly.N0 * aPolyInvW;
            gp_XY aPolyCD1 = (aPoly.N1 * aPoly.W0 - aPoly.N0 * aPoly.W1) * (aPolyInvW * aPolyInvW);
            aPolyResult.Point = new gp_Pnt2d(aPolyC);
            aPolyResult.D1 = new gp_Vec2d(aPolyCD1);
            aPolyResult.D2 = new gp_Vec2d((aPoly.N2 - aPolyCD1 * (2.0 * aPoly.W1) - aPolyC * aPoly.W2) * aPolyInvW);
            return aPolyResult;
        }

        ResD2 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(2, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new gp_Pnt2d(aMixed.N0);
            aResult.D1 = new gp_Vec2d(aMixed.N1);
            aResult.D2 = new gp_Vec2d(aMixed.N2);
            return aResult;
        }

        evalMixedAHT(2, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        gp_XY aC = aMixed.N0 * aInvW;
        gp_XY aCD1 = (aMixed.N1 * aMixed.W0 - aMixed.N0 * aMixed.W1) * (aInvW * aInvW);

        aResult.Point = new gp_Pnt2d(aC);
        aResult.D1 = new gp_Vec2d(aCD1);
        aResult.D2 = new gp_Vec2d((aMixed.N2 - aCD1 * (2.0 * aMixed.W1) - aC * aMixed.W2) * aInvW);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        if (myAlpha <= 0.0 && myBeta <= 0.0)
        {
            PolyEvalData aPoly;
            ResD3 aPolyResult;
            if (!myRational)
            {
                evalPolynomialNonRational(3, myPoles, myAlgDegree, U, out aPoly);
                aPolyResult.Point = new gp_Pnt2d(aPoly.N0);
                aPolyResult.D1 = new gp_Vec2d(aPoly.N1);
                aPolyResult.D2 = new gp_Vec2d(aPoly.N2);
                aPolyResult.D3 = new gp_Vec2d(aPoly.N3);
                return aPolyResult;
            }

            evalPolynomialRational(3, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            gp_XY aPolyC = aPoly.N0 * aPolyInvW;
            gp_XY aPolyCD1 = (aPoly.N1 - aPolyC * aPoly.W1) * aPolyInvW;
            gp_XY aPolyCD2 = (aPoly.N2 - aPolyCD1 * (2.0 * aPoly.W1) - aPolyC * aPoly.W2) * aPolyInvW;
            gp_XY aPolyCD3 =
                (aPoly.N3 - aPolyCD2 * (3.0 * aPoly.W1) - aPolyCD1 * (3.0 * aPoly.W2) - aPolyC * aPoly.W3) * aPolyInvW;

            aPolyResult.Point = new gp_Pnt2d(aPolyC);
            aPolyResult.D1 = new gp_Vec2d(aPolyCD1);
            aPolyResult.D2 = new gp_Vec2d(aPolyCD2);
            aPolyResult.D3 = new gp_Vec2d(aPolyCD3);
            return aPolyResult;
        }

        ResD3 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(3, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new gp_Pnt2d(aMixed.N0);
            aResult.D1 = new gp_Vec2d(aMixed.N1);
            aResult.D2 = new gp_Vec2d(aMixed.N2);
            aResult.D3 = new gp_Vec2d(aMixed.N3);
            return aResult;
        }

        evalMixedAHT(3, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        gp_XY aC = aMixed.N0 * aInvW;
        gp_XY aCD1 = (aMixed.N1 - aC * aMixed.W1) * aInvW;
        gp_XY aCD2 = (aMixed.N2 - aCD1 * (2.0 * aMixed.W1) - aC * aMixed.W2) * aInvW;
        gp_XY aCD3 =
            (aMixed.N3 - aCD2 * (3.0 * aMixed.W1) - aCD1 * (3.0 * aMixed.W2) - aC * aMixed.W3) * aInvW;

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
            throw new Standard_RangeError("Geom2dEval_AHTBezierCurve::EvalDN: N must be >= 1");
        }

        // For low orders, delegate to explicit implementations.
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

        int aPoleLow = myPoles.Lower();

        if (!myRational)
        {
            int aPoleIdx = aPoleLow;
            gp_XY aSum = new(0.0, 0.0);

            for (int k = 0; k <= myAlgDegree; ++k)
            {
                double aBasisN = 0.0;
                if (N <= k)
                {
                    if (U == 0.0)
                    {
                        aBasisN = (N == k) ? fallingFactorial(k, N) : 0.0;
                    }
                    else
                    {
                        aBasisN = fallingFactorial(k, N) * powInt(U, k - N);
                    }
                }
                aSum += myPoles.Value(aPoleIdx++).XY() * aBasisN;
            }

            if (myAlpha > 0.0)
            {
                double aAlphaU = myAlpha * U;
                double aAlphaPow = powInt(myAlpha, N);
                double aSinh = Math.Sinh(aAlphaU);
                double aCosh = Math.Cosh(aAlphaU);
                bool isEvenN = (N % 2 == 0);
                double aSinhN = aAlphaPow * (isEvenN ? aSinh : aCosh);
                double aCoshN = aAlphaPow * (isEvenN ? aCosh : aSinh);
                aSum += myPoles.Value(aPoleIdx++).XY() * aSinhN;
                aSum += myPoles.Value(aPoleIdx++).XY() * aCoshN;
            }

            if (myBeta > 0.0)
            {
                double aBetaU = myBeta * U;
                double aBetaPow = powInt(myBeta, N);
                double aSin = Math.Sin(aBetaU);
                double aCos = Math.Cos(aBetaU);
                double aSinN = 0.0;
                double aCosN = 0.0;
                switch (N & 3)
                {
                    case 0:
                        aSinN = aSin;
                        aCosN = aCos;
                        break;
                    case 1:
                        aSinN = aCos;
                        aCosN = -aSin;
                        break;
                    case 2:
                        aSinN = -aSin;
                        aCosN = -aCos;
                        break;
                    case 3:
                        aSinN = -aCos;
                        aCosN = aSin;
                        break;
                }
                aSum += myPoles.Value(aPoleIdx++).XY() * (aBetaPow * aSinN);
                aSum += myPoles.Value(aPoleIdx++).XY() * (aBetaPow * aCosN);
            }

            return new gp_Vec2d(aSum);
        }

        // C++: NCollection_LocalArray<double, 16> and <gp_XY, 16>.
        Span<double> aWDerivs = N + 1 <= 16 ? stackalloc double[16] : new double[N + 1];
        Span<gp_XY> aCDerivs = N + 1 <= 16 ? stackalloc gp_XY[16] : new gp_XY[N + 1];

        int aWeightLow = myWeights.Lower();
        int aPolyCount = myAlgDegree + 1;
        int aHyperIdx = aPoleLow + aPolyCount;
        int aTrigIdx = aHyperIdx + (myAlpha > 0.0 ? 2 : 0);

        // The C++ pointers to the poles, null when unused.
        gp_XY aPoleSinh = new();
        gp_XY aPoleCosh = new();
        gp_XY aPoleSin = new();
        gp_XY aPoleCos = new();
        double aWiSinh = 0.0;
        double aWiCosh = 0.0;
        double aWiSin = 0.0;
        double aWiCos = 0.0;

        if (myAlpha > 0.0)
        {
            aPoleSinh = myPoles.Value(aHyperIdx).XY();
            aPoleCosh = myPoles.Value(aHyperIdx + 1).XY();
            aWiSinh = myWeights.Value(aWeightLow + aPolyCount);
            aWiCosh = myWeights.Value(aWeightLow + aPolyCount + 1);
        }

        if (myBeta > 0.0)
        {
            int aTrigWIdx = aWeightLow + aPolyCount + (myAlpha > 0.0 ? 2 : 0);
            aPoleSin = myPoles.Value(aTrigIdx).XY();
            aPoleCos = myPoles.Value(aTrigIdx + 1).XY();
            aWiSin = myWeights.Value(aTrigWIdx);
            aWiCos = myWeights.Value(aTrigWIdx + 1);
        }

        double aAlphaU2 = myAlpha * U;
        double aSinh2 = (myAlpha > 0.0) ? Math.Sinh(aAlphaU2) : 0.0;
        double aCosh2 = (myAlpha > 0.0) ? Math.Cosh(aAlphaU2) : 0.0;
        double aBetaU2 = myBeta * U;
        double aSin2 = (myBeta > 0.0) ? Math.Sin(aBetaU2) : 0.0;
        double aCos2 = (myBeta > 0.0) ? Math.Cos(aBetaU2) : 0.0;

        double aAlphaPow2 = 1.0;
        double aBetaPow2 = 1.0;
        double aInvW0 = 0.0;
        for (int d = 0; d <= N; ++d)
        {
            gp_XY aNd = new(0.0, 0.0);
            double aWd = 0.0;

            for (int k = 0; k <= myAlgDegree; ++k)
            {
                gp_XY aPole = myPoles.Value(aPoleLow + k).XY();
                double aWi = myWeights.Value(aWeightLow + k);
                double aBasisD = 0.0;
                if (d <= k)
                {
                    if (U == 0.0)
                    {
                        aBasisD = (d == k) ? fallingFactorial(k, d) : 0.0;
                    }
                    else
                    {
                        aBasisD = fallingFactorial(k, d) * powInt(U, k - d);
                    }
                }

                aNd += aPole * (aWi * aBasisD);
                aWd += aWi * aBasisD;
            }

            if (myAlpha > 0.0)
            {
                bool isEvenD = (d % 2 == 0);
                double aDSinh = aAlphaPow2 * (isEvenD ? aSinh2 : aCosh2);
                double aDCosh = aAlphaPow2 * (isEvenD ? aCosh2 : aSinh2);
                aNd += aPoleSinh * (aWiSinh * aDSinh) + aPoleCosh * (aWiCosh * aDCosh);
                aWd += aWiSinh * aDSinh + aWiCosh * aDCosh;
            }

            if (myBeta > 0.0)
            {
                double aDSin = 0.0;
                double aDCos = 0.0;
                switch (d & 3)
                {
                    case 0:
                        aDSin = aSin2;
                        aDCos = aCos2;
                        break;
                    case 1:
                        aDSin = aCos2;
                        aDCos = -aSin2;
                        break;
                    case 2:
                        aDSin = -aSin2;
                        aDCos = -aCos2;
                        break;
                    case 3:
                        aDSin = -aCos2;
                        aDCos = aSin2;
                        break;
                }
                aNd += aPoleSin * (aWiSin * aBetaPow2 * aDSin) + aPoleCos * (aWiCos * aBetaPow2 * aDCos);
                aWd += aWiSin * aBetaPow2 * aDSin + aWiCos * aBetaPow2 * aDCos;
            }

            aWDerivs[d] = aWd;
            if (d == 0)
            {
                aInvW0 = 1.0 / aWd;
                aCDerivs[0] = aNd * aInvW0;
            }
            else
            {
                gp_XY aSum = aNd;
                double aBinom = 1.0;
                for (int j = 1; j <= d; ++j)
                {
                    aBinom = aBinom * (double)(d - j + 1) / (double)j;
                    aSum -= aCDerivs[d - j] * (aBinom * aWDerivs[j]);
                }
                aCDerivs[d] = aSum * aInvW0;
            }

            aAlphaPow2 *= myAlpha;
            aBetaPow2 *= myBeta;
        }
        return new gp_Vec2d(aCDerivs[N]);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_AHTBezierCurve::Transform");
    }

    public override Geom2d_Geometry Copy()
    {
        if (myRational)
        {
            return new Geom2dEval_AHTBezierCurve(myPoles, myWeights, myAlgDegree, myAlpha, myBeta);
        }
        return new Geom2dEval_AHTBezierCurve(myPoles, myAlgDegree, myAlpha, myBeta);
    }
}
