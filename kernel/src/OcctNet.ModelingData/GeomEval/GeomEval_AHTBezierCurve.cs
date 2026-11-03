// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A 3D Algebraic-Hyperbolic-Trigonometric Bezier curve, over the mixed basis
/// {1, t, ..., t^k, sinh(alpha*t), cosh(alpha*t), sin(beta*t), cos(beta*t)}, where the hyperbolic pair is present
/// when alpha &gt; 0 and the trigonometric pair when beta &gt; 0. The number of poles must equal the number of
/// basis functions, algDegree + 1 + 2*(alpha&gt;0) + 2*(beta&gt;0). The parameter range is [0, 1].
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class GeomEval_AHTBezierCurve : Geom_BoundedCurve
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
                                                  NCollection_Array1<Point3D> thePoles,
                                                  int theDegree,
                                                  double theU,
                                                  out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        int aPoleLow = thePoles.Lower();
        theData.N0 = thePoles.Value(aPoleLow + theDegree).XYZ();

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
            theData.N0 = theData.N0 * theU + thePoles.Value(aPoleLow + k).XYZ();
        }
    }

    // The C++ template evalPolynomialRational<theMaxOrder>.
    private static void evalPolynomialRational(int theMaxOrder,
                                               NCollection_Array1<Point3D> thePoles,
                                               NCollection_Array1<double> theWeights,
                                               int theDegree,
                                               double theU,
                                               out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        int aPoleLow = thePoles.Lower();
        int aWeightLow = theWeights.Lower();

        double aWLead = theWeights.Value(aWeightLow + theDegree);
        theData.N0 = thePoles.Value(aPoleLow + theDegree).XYZ() * aWLead;
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

            double aWk = theWeights.Value(aWeightLow + k);
            theData.N0 = theData.N0 * theU + thePoles.Value(aPoleLow + k).XYZ() * aWk;
            theData.W0 = theData.W0 * theU + aWk;
        }
    }

    // The C++ template evalMixedAHT<theMaxOrder, theIsRational>.
    private static void evalMixedAHT(int theMaxOrder,
                                     bool theIsRational,
                                     NCollection_Array1<Point3D> thePoles,
                                     NCollection_Array1<double> theWeights,
                                     int theAlgDegree,
                                     double theAlpha,
                                     double theBeta,
                                     double theU,
                                     out PolyEvalData theData)
    {
        theData = new PolyEvalData();
        int aPoleIdx = thePoles.Lower();
        int aWeightIdx = theWeights.Lower();

        // Polynomial part: {1, t, ..., t^k}.

        double aPow = 1.0;
        double aPowM1 = 0.0;
        double aPowM2 = 0.0;
        double aPowM3 = 0.0;
        for (int k = 0; k <= theAlgDegree; ++k)
        {
            Xyz aPole = thePoles.Value(aPoleIdx++).XYZ();
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
                double aWi = theWeights.Value(aWeightIdx++);
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

            Xyz aPoleSinh = thePoles.Value(aPoleIdx++).XYZ();
            Xyz aPoleCosh = thePoles.Value(aPoleIdx++).XYZ();
            double aWiSinh = 1.0;
            double aWiCosh = 1.0;
            if (theIsRational)
            {
                aWiSinh = theWeights.Value(aWeightIdx++);
                aWiCosh = theWeights.Value(aWeightIdx++);
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

            Xyz aPoleSin = thePoles.Value(aPoleIdx++).XYZ();
            Xyz aPoleCos = thePoles.Value(aPoleIdx++).XYZ();
            double aWiSin = 1.0;
            double aWiCos = 1.0;
            if (theIsRational)
            {
                aWiSin = theWeights.Value(aWeightIdx++);
                aWiCos = theWeights.Value(aWeightIdx++);
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

    private NCollection_Array1<Point3D> myPoles;
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
    /// <exception cref="ConstructionErrorException">A parameter is out of range or the number of poles is wrong.</exception>
    public GeomEval_AHTBezierCurve(NCollection_Array1<Point3D> thePoles, int theAlgDegree, double theAlpha,
                                     double theBeta)
    {
        myPoles = new NCollection_Array1<Point3D>(thePoles);
        // OCCT: a non-rational curve has one weight, left uninitialised (0 here), which Weights() returns.
        myWeights = new NCollection_Array1<double>(1, 1);
        myAlgDegree = theAlgDegree;
        myAlpha = theAlpha;
        myBeta = theBeta;
        myRational = false;
        if (theAlgDegree < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (theAlpha < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (theBeta < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: beta must be >= 0");
        }
        int aDim = basisDimension(theAlgDegree, theAlpha, theBeta);
        if (thePoles.Length() != aDim)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
    }

    /// <summary>A rational AHT-Bezier curve.</summary>
    /// <param name="thePoles">The control points, as many as the basis functions.</param>
    /// <param name="theWeights">The weights, as many as the poles, all &gt; 0.</param>
    /// <param name="theAlgDegree">The degree of the polynomial part (&gt;= 0).</param>
    /// <param name="theAlpha">The hyperbolic frequency (&gt;= 0).</param>
    /// <param name="theBeta">The trigonometric frequency (&gt;= 0).</param>
    /// <exception cref="ConstructionErrorException">A parameter is out of range or an array has the wrong size.</exception>
    public GeomEval_AHTBezierCurve(NCollection_Array1<Point3D> thePoles, NCollection_Array1<double> theWeights,
                                     int theAlgDegree, double theAlpha, double theBeta)
    {
        myPoles = new NCollection_Array1<Point3D>(thePoles);
        myWeights = new NCollection_Array1<double>(theWeights);
        myAlgDegree = theAlgDegree;
        myAlpha = theAlpha;
        myBeta = theBeta;
        myRational = true;
        if (theAlgDegree < 0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: algebraic degree must be >= 0");
        }
        if (theAlpha < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: alpha must be >= 0");
        }
        if (theBeta < 0.0)
        {
            throw new ConstructionErrorException("GeomEval_AHTBezierCurve: beta must be >= 0");
        }
        int aDim = basisDimension(theAlgDegree, theAlpha, theBeta);
        if (thePoles.Length() != aDim)
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: NbPoles must equal basisDimension");
        }
        if (theWeights.Length() != thePoles.Length())
        {
            throw new ConstructionErrorException(
                "GeomEval_AHTBezierCurve: weights array size must match poles array size");
        }
        for (int anIdx = theWeights.Lower(); anIdx <= theWeights.Upper(); ++anIdx)
        {
            if (theWeights.Value(anIdx) <= 0.0)
            {
                throw new ConstructionErrorException("GeomEval_AHTBezierCurve: all weights must be > 0");
            }
        }
    }

    /// <summary>The poles (a copy, indexed from 0).</summary>
    public PackedList<Point3D> Poles() => myPoles.ToPackedList();

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
    public override double ReversedParameter(double U)
    {
        throw new MethodNotImplementedException("GeomEval_AHTBezierCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public override double LastParameter() => 1.0;

    /// <summary>True if the start and end points coincide (within Precision::Confusion()).</summary>
    public override bool IsClosed() => StartPoint().Distance(EndPoint()) <= global::Kerf.Engine.Foundation.Precision.Confusion();

    public override bool IsPeriodic() => false;

    public override Continuity Continuity() => CN;

    public override bool IsCN(int N) => true;

    public override Point3D EvalD0(double U)
    {
        if (myAlpha <= 0.0 && myBeta <= 0.0)
        {
            PolyEvalData aPoly;
            if (!myRational)
            {
                evalPolynomialNonRational(0, myPoles, myAlgDegree, U, out aPoly);
                return new Point3D(aPoly.N0);
            }

            evalPolynomialRational(0, myPoles, myWeights, myAlgDegree, U, out aPoly);
            return new Point3D(aPoly.N0 / aPoly.W0);
        }

        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(0, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            return new Point3D(aMixed.N0);
        }

        evalMixedAHT(0, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        return new Point3D(aMixed.N0 / aMixed.W0);
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
                aPolyResult.Point = new Point3D(aPoly.N0);
                aPolyResult.D1 = new Vector3D(aPoly.N1);
                return aPolyResult;
            }

            evalPolynomialRational(1, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            aPolyResult.Point = new Point3D(aPoly.N0 * aPolyInvW);
            aPolyResult.D1 = new Vector3D((aPoly.N1 * aPoly.W0 - aPoly.N0 * aPoly.W1) * (aPolyInvW * aPolyInvW));
            return aPolyResult;
        }

        ResD1 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(1, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new Point3D(aMixed.N0);
            aResult.D1 = new Vector3D(aMixed.N1);
            return aResult;
        }

        evalMixedAHT(1, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        aResult.Point = new Point3D(aMixed.N0 * aInvW);
        aResult.D1 = new Vector3D((aMixed.N1 * aMixed.W0 - aMixed.N0 * aMixed.W1) * (aInvW * aInvW));
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
                aPolyResult.Point = new Point3D(aPoly.N0);
                aPolyResult.D1 = new Vector3D(aPoly.N1);
                aPolyResult.D2 = new Vector3D(aPoly.N2);
                return aPolyResult;
            }

            evalPolynomialRational(2, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            Xyz aPolyC = aPoly.N0 * aPolyInvW;
            Xyz aPolyCD1 = (aPoly.N1 * aPoly.W0 - aPoly.N0 * aPoly.W1) * (aPolyInvW * aPolyInvW);
            aPolyResult.Point = new Point3D(aPolyC);
            aPolyResult.D1 = new Vector3D(aPolyCD1);
            aPolyResult.D2 = new Vector3D((aPoly.N2 - aPolyCD1 * (2.0 * aPoly.W1) - aPolyC * aPoly.W2) * aPolyInvW);
            return aPolyResult;
        }

        ResD2 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(2, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new Point3D(aMixed.N0);
            aResult.D1 = new Vector3D(aMixed.N1);
            aResult.D2 = new Vector3D(aMixed.N2);
            return aResult;
        }

        evalMixedAHT(2, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        Xyz aC = aMixed.N0 * aInvW;
        Xyz aCD1 = (aMixed.N1 * aMixed.W0 - aMixed.N0 * aMixed.W1) * (aInvW * aInvW);

        aResult.Point = new Point3D(aC);
        aResult.D1 = new Vector3D(aCD1);
        aResult.D2 = new Vector3D((aMixed.N2 - aCD1 * (2.0 * aMixed.W1) - aC * aMixed.W2) * aInvW);
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
                aPolyResult.Point = new Point3D(aPoly.N0);
                aPolyResult.D1 = new Vector3D(aPoly.N1);
                aPolyResult.D2 = new Vector3D(aPoly.N2);
                aPolyResult.D3 = new Vector3D(aPoly.N3);
                return aPolyResult;
            }

            evalPolynomialRational(3, myPoles, myWeights, myAlgDegree, U, out aPoly);
            double aPolyInvW = 1.0 / aPoly.W0;
            Xyz aPolyC = aPoly.N0 * aPolyInvW;
            Xyz aPolyCD1 = (aPoly.N1 - aPolyC * aPoly.W1) * aPolyInvW;
            Xyz aPolyCD2 = (aPoly.N2 - aPolyCD1 * (2.0 * aPoly.W1) - aPolyC * aPoly.W2) * aPolyInvW;
            Xyz aPolyCD3 =
                (aPoly.N3 - aPolyCD2 * (3.0 * aPoly.W1) - aPolyCD1 * (3.0 * aPoly.W2) - aPolyC * aPoly.W3) * aPolyInvW;

            aPolyResult.Point = new Point3D(aPolyC);
            aPolyResult.D1 = new Vector3D(aPolyCD1);
            aPolyResult.D2 = new Vector3D(aPolyCD2);
            aPolyResult.D3 = new Vector3D(aPolyCD3);
            return aPolyResult;
        }

        ResD3 aResult;
        PolyEvalData aMixed;
        if (!myRational)
        {
            evalMixedAHT(3, false, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
            aResult.Point = new Point3D(aMixed.N0);
            aResult.D1 = new Vector3D(aMixed.N1);
            aResult.D2 = new Vector3D(aMixed.N2);
            aResult.D3 = new Vector3D(aMixed.N3);
            return aResult;
        }

        evalMixedAHT(3, true, myPoles, myWeights, myAlgDegree, myAlpha, myBeta, U, out aMixed);
        double aInvW = 1.0 / aMixed.W0;
        Xyz aC = aMixed.N0 * aInvW;
        Xyz aCD1 = (aMixed.N1 - aC * aMixed.W1) * aInvW;
        Xyz aCD2 = (aMixed.N2 - aCD1 * (2.0 * aMixed.W1) - aC * aMixed.W2) * aInvW;
        Xyz aCD3 =
            (aMixed.N3 - aCD2 * (3.0 * aMixed.W1) - aCD1 * (3.0 * aMixed.W2) - aC * aMixed.W3) * aInvW;

        aResult.Point = new Point3D(aC);
        aResult.D1 = new Vector3D(aCD1);
        aResult.D2 = new Vector3D(aCD2);
        aResult.D3 = new Vector3D(aCD3);
        return aResult;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new RangeErrorException("GeomEval_AHTBezierCurve::EvalDN: N must be >= 1");
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
            Xyz aSum = new(0.0, 0.0, 0.0);

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
                aSum += myPoles.Value(aPoleIdx++).XYZ() * aBasisN;
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
                aSum += myPoles.Value(aPoleIdx++).XYZ() * aSinhN;
                aSum += myPoles.Value(aPoleIdx++).XYZ() * aCoshN;
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
                aSum += myPoles.Value(aPoleIdx++).XYZ() * (aBetaPow * aSinN);
                aSum += myPoles.Value(aPoleIdx++).XYZ() * (aBetaPow * aCosN);
            }

            return new Vector3D(aSum);
        }

        // C++: NCollection_LocalArray<double, 16> and <gp_XYZ, 16>.
        Span<double> aWDerivs = N + 1 <= 16 ? stackalloc double[16] : new double[N + 1];
        Span<Xyz> aCDerivs = N + 1 <= 16 ? stackalloc Xyz[16] : new Xyz[N + 1];

        int aWeightLow = myWeights.Lower();
        int aPolyCount = myAlgDegree + 1;
        int aHyperIdx = aPoleLow + aPolyCount;
        int aTrigIdx = aHyperIdx + (myAlpha > 0.0 ? 2 : 0);

        // The C++ pointers to the poles, null when unused.
        Xyz aPoleSinh = new();
        Xyz aPoleCosh = new();
        Xyz aPoleSin = new();
        Xyz aPoleCos = new();
        double aWiSinh = 0.0;
        double aWiCosh = 0.0;
        double aWiSin = 0.0;
        double aWiCos = 0.0;

        if (myAlpha > 0.0)
        {
            aPoleSinh = myPoles.Value(aHyperIdx).XYZ();
            aPoleCosh = myPoles.Value(aHyperIdx + 1).XYZ();
            aWiSinh = myWeights.Value(aWeightLow + aPolyCount);
            aWiCosh = myWeights.Value(aWeightLow + aPolyCount + 1);
        }

        if (myBeta > 0.0)
        {
            int aTrigWIdx = aWeightLow + aPolyCount + (myAlpha > 0.0 ? 2 : 0);
            aPoleSin = myPoles.Value(aTrigIdx).XYZ();
            aPoleCos = myPoles.Value(aTrigIdx + 1).XYZ();
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
            Xyz aNd = new(0.0, 0.0, 0.0);
            double aWd = 0.0;

            for (int k = 0; k <= myAlgDegree; ++k)
            {
                Xyz aPole = myPoles.Value(aPoleLow + k).XYZ();
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
                Xyz aSum = aNd;
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
        return new Vector3D(aCDerivs[N]);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_AHTBezierCurve::Transform");
    }

    public override Geom_Geometry Copy()
    {
        if (myRational)
        {
            return new GeomEval_AHTBezierCurve(myPoles, myWeights, myAlgDegree, myAlpha, myBeta);
        }
        return new GeomEval_AHTBezierCurve(myPoles, myAlgDegree, myAlpha, myBeta);
    }
}
