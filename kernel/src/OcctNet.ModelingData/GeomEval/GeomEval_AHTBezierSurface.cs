// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_AHTBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A tensor-product Algebraic-Hyperbolic-Trigonometric Bezier surface, over the mixed basis
/// {1, t, ..., t^k, sinh(alpha*t), cosh(alpha*t), sin(beta*t), cos(beta*t)} in each direction, with separate
/// parameters (algDegreeU, alphaU, betaU) and (algDegreeV, alphaV, betaV); a hyperbolic or trigonometric pair is
/// present when its frequency is &gt; 0. The poles grid has one row per U basis function and one column per V
/// basis function. The domain is [0, 1] x [0, 1]. Reversal, transformation and iso-curves are not supported
/// (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_AHTBezierSurface : Geom_BoundedSurface
{
    // The number of basis functions for the given parameters.
    private static int basisDimension(int theAlgDegree, double theAlpha, double theBeta) =>
        theAlgDegree + 1 + (theAlpha > 0.0 ? 2 : 0) + (theBeta > 0.0 ? 2 : 0);

    private struct PolySurfaceDerivData
    {
        public gp_XYZ S00;
        public gp_XYZ S10;
        public gp_XYZ S01;
        public gp_XYZ S20;
        public gp_XYZ S02;
        public gp_XYZ S11;
        public gp_XYZ S30;
        public gp_XYZ S03;
        public gp_XYZ S21;
        public gp_XYZ S12;
    }

    private static gp_XYZ cubicVal(in gp_XYZ theC0, in gp_XYZ theC1, in gp_XYZ theC2, in gp_XYZ theC3, double theT)
    {
        return ((theC3 * theT + theC2) * theT + theC1) * theT + theC0;
    }

    private static gp_XYZ cubicD1(in gp_XYZ theC0, in gp_XYZ theC1, in gp_XYZ theC2, in gp_XYZ theC3, double theT)
    {
        return (theC3 * (3.0 * theT) + theC2 * 2.0) * theT + theC1;
    }

    private static gp_XYZ cubicD2(in gp_XYZ theC0, in gp_XYZ theC1, in gp_XYZ theC2, in gp_XYZ theC3, double theT)
    {
        return theC3 * (6.0 * theT) + theC2 * 2.0;
    }

    private static gp_XYZ cubicD3(in gp_XYZ theC0, in gp_XYZ theC1, in gp_XYZ theC2, in gp_XYZ theC3)
    {
        return theC3 * 6.0;
    }

    // The C++ template evalPolynomialSurfaceCubicNonRational<theMaxOrder>: the `if constexpr` are plain tests.
    private static void evalPolynomialSurfaceCubicNonRational(int theMaxOrder,
                                                              NCollection_Array2<gp_Pnt> thePoles,
                                                              double theU,
                                                              double theV,
                                                              out PolySurfaceDerivData theData)
    {
        theData.S00 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S10 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S01 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S20 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S02 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S11 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S30 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S03 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S21 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S12 = new gp_XYZ(0.0, 0.0, 0.0);

        int aLR = thePoles.LowerRow();
        int aLC = thePoles.LowerCol();

        gp_XYZ aP00 = thePoles.Value(aLR + 0, aLC + 0).XYZ();
        gp_XYZ aP01 = thePoles.Value(aLR + 0, aLC + 1).XYZ();
        gp_XYZ aP02 = thePoles.Value(aLR + 0, aLC + 2).XYZ();
        gp_XYZ aP03 = thePoles.Value(aLR + 0, aLC + 3).XYZ();
        gp_XYZ aP10 = thePoles.Value(aLR + 1, aLC + 0).XYZ();
        gp_XYZ aP11 = thePoles.Value(aLR + 1, aLC + 1).XYZ();
        gp_XYZ aP12 = thePoles.Value(aLR + 1, aLC + 2).XYZ();
        gp_XYZ aP13 = thePoles.Value(aLR + 1, aLC + 3).XYZ();
        gp_XYZ aP20 = thePoles.Value(aLR + 2, aLC + 0).XYZ();
        gp_XYZ aP21 = thePoles.Value(aLR + 2, aLC + 1).XYZ();
        gp_XYZ aP22 = thePoles.Value(aLR + 2, aLC + 2).XYZ();
        gp_XYZ aP23 = thePoles.Value(aLR + 2, aLC + 3).XYZ();
        gp_XYZ aP30 = thePoles.Value(aLR + 3, aLC + 0).XYZ();
        gp_XYZ aP31 = thePoles.Value(aLR + 3, aLC + 1).XYZ();
        gp_XYZ aP32 = thePoles.Value(aLR + 3, aLC + 2).XYZ();
        gp_XYZ aP33 = thePoles.Value(aLR + 3, aLC + 3).XYZ();

        gp_XYZ aQ00 = cubicVal(aP00, aP10, aP20, aP30, theU);
        gp_XYZ aQ01 = cubicVal(aP01, aP11, aP21, aP31, theU);
        gp_XYZ aQ02 = cubicVal(aP02, aP12, aP22, aP32, theU);
        gp_XYZ aQ03 = cubicVal(aP03, aP13, aP23, aP33, theU);

        theData.S00 = cubicVal(aQ00, aQ01, aQ02, aQ03, theV);
        if (theMaxOrder >= 1)
        {
            gp_XYZ aQ10 = cubicD1(aP00, aP10, aP20, aP30, theU);
            gp_XYZ aQ11 = cubicD1(aP01, aP11, aP21, aP31, theU);
            gp_XYZ aQ12 = cubicD1(aP02, aP12, aP22, aP32, theU);
            gp_XYZ aQ13 = cubicD1(aP03, aP13, aP23, aP33, theU);

            theData.S10 = cubicVal(aQ10, aQ11, aQ12, aQ13, theV);
            theData.S01 = cubicD1(aQ00, aQ01, aQ02, aQ03, theV);

            if (theMaxOrder >= 2)
            {
                gp_XYZ aQ20 = cubicD2(aP00, aP10, aP20, aP30, theU);
                gp_XYZ aQ21 = cubicD2(aP01, aP11, aP21, aP31, theU);
                gp_XYZ aQ22 = cubicD2(aP02, aP12, aP22, aP32, theU);
                gp_XYZ aQ23 = cubicD2(aP03, aP13, aP23, aP33, theU);

                theData.S20 = cubicVal(aQ20, aQ21, aQ22, aQ23, theV);
                theData.S11 = cubicD1(aQ10, aQ11, aQ12, aQ13, theV);
                theData.S02 = cubicD2(aQ00, aQ01, aQ02, aQ03, theV);

                if (theMaxOrder >= 3)
                {
                    gp_XYZ aQ30 = cubicD3(aP00, aP10, aP20, aP30);
                    gp_XYZ aQ31 = cubicD3(aP01, aP11, aP21, aP31);
                    gp_XYZ aQ32 = cubicD3(aP02, aP12, aP22, aP32);
                    gp_XYZ aQ33 = cubicD3(aP03, aP13, aP23, aP33);

                    theData.S30 = cubicVal(aQ30, aQ31, aQ32, aQ33, theV);
                    theData.S21 = cubicD1(aQ20, aQ21, aQ22, aQ23, theV);
                    theData.S12 = cubicD2(aQ10, aQ11, aQ12, aQ13, theV);
                    theData.S03 = cubicD3(aQ00, aQ01, aQ02, aQ03);
                }
            }
        }
    }

    // The C++ template evalPolynomialSurfaceNonRational<theMaxOrder>: the `if constexpr` are plain tests.
    private static void evalPolynomialSurfaceNonRational(int theMaxOrder,
                                                         NCollection_Array2<gp_Pnt> thePoles,
                                                         int theDegreeU,
                                                         int theDegreeV,
                                                         double theU,
                                                         double theV,
                                                         out PolySurfaceDerivData theData)
    {
        theData.S00 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S10 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S01 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S20 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S02 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S11 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S30 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S03 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S21 = new gp_XYZ(0.0, 0.0, 0.0);
        theData.S12 = new gp_XYZ(0.0, 0.0, 0.0);

        int aLR = thePoles.LowerRow();
        int aLC = thePoles.LowerCol();

        gp_XYZ aR00 = new(0.0, 0.0, 0.0);
        gp_XYZ aR01 = new(0.0, 0.0, 0.0);
        gp_XYZ aR02 = new(0.0, 0.0, 0.0);
        gp_XYZ aR03 = new(0.0, 0.0, 0.0);
        gp_XYZ aR10 = new(0.0, 0.0, 0.0);
        gp_XYZ aR11 = new(0.0, 0.0, 0.0);
        gp_XYZ aR12 = new(0.0, 0.0, 0.0);
        gp_XYZ aR20 = new(0.0, 0.0, 0.0);
        gp_XYZ aR21 = new(0.0, 0.0, 0.0);
        gp_XYZ aR30 = new(0.0, 0.0, 0.0);

        for (int j = theDegreeV; j >= 0; --j)
        {
            gp_XYZ aD0 = thePoles.Value(aLR + theDegreeU, aLC + j).XYZ();
            gp_XYZ aD1 = new(0.0, 0.0, 0.0);
            gp_XYZ aD2 = new(0.0, 0.0, 0.0);
            gp_XYZ aD3 = new(0.0, 0.0, 0.0);

            for (int i = theDegreeU - 1; i >= 0; --i)
            {
                if (theMaxOrder >= 3)
                {
                    aD3 = aD3 * theU + aD2 * 3.0;
                }
                if (theMaxOrder >= 2)
                {
                    aD2 = aD2 * theU + aD1 * 2.0;
                }
                if (theMaxOrder >= 1)
                {
                    aD1 = aD1 * theU + aD0;
                }
                aD0 = aD0 * theU + thePoles.Value(aLR + i, aLC + j).XYZ();
            }

            if (theMaxOrder >= 3)
            {
                aR03 = aR03 * theV + aR02 * 3.0;
            }
            if (theMaxOrder >= 2)
            {
                aR02 = aR02 * theV + aR01 * 2.0;
            }
            if (theMaxOrder >= 1)
            {
                aR01 = aR01 * theV + aR00;
            }
            aR00 = aR00 * theV + aD0;

            if (theMaxOrder >= 1)
            {
                if (theMaxOrder >= 2)
                {
                    aR12 = aR12 * theV + aR11 * 2.0;
                }
                if (theMaxOrder >= 1)
                {
                    aR11 = aR11 * theV + aR10;
                }
                aR10 = aR10 * theV + aD1;
            }

            if (theMaxOrder >= 2)
            {
                if (theMaxOrder >= 3)
                {
                    aR21 = aR21 * theV + aR20;
                }
                aR20 = aR20 * theV + aD2;
            }

            if (theMaxOrder >= 3)
            {
                aR30 = aR30 * theV + aD3;
            }
        }

        theData.S00 = aR00;
        if (theMaxOrder >= 1)
        {
            theData.S01 = aR01;
            theData.S10 = aR10;
        }
        if (theMaxOrder >= 2)
        {
            theData.S02 = aR02;
            theData.S11 = aR11;
            theData.S20 = aR20;
        }
        if (theMaxOrder >= 3)
        {
            theData.S03 = aR03;
            theData.S12 = aR12;
            theData.S21 = aR21;
            theData.S30 = aR30;
        }
    }

    private static ref double axisVal(Span<double> theData, int theDim, int theOrder, int theIdx)
    {
        return ref theData[theOrder * theDim + theIdx];
    }

    // theDerivs holds (theMaxOrder + 1) blocks of theDim values, one per derivative order.
    private static void evalAxisDerivs(double theT,
                                       int theMaxOrder,
                                       int theAlgDegree,
                                       double theAlpha,
                                       double theBeta,
                                       int theDim,
                                       Span<double> theDerivs)
    {
        int anIdx = 0;

        if (theT == 0.0)
        {
            double aFactK = 1.0;
            for (int k = 0; k <= theAlgDegree; ++k)
            {
                if (k > 0)
                {
                    aFactK *= (double)k;
                }
                for (int d = 0; d <= theMaxOrder; ++d)
                {
                    axisVal(theDerivs, theDim, d, anIdx) = (d == k) ? aFactK : 0.0;
                }
                ++anIdx;
            }
        }
        else
        {
            double aPow = 1.0;
            for (int k = 0; k <= theAlgDegree; ++k)
            {
                int aMaxD = Min(k, theMaxOrder);
                double aVal = aPow;
                for (int d = 0; d <= aMaxD; ++d)
                {
                    axisVal(theDerivs, theDim, d, anIdx) = aVal;
                    if (d < aMaxD)
                    {
                        aVal *= (double)(k - d) / theT;
                    }
                }
                for (int d = aMaxD + 1; d <= theMaxOrder; ++d)
                {
                    axisVal(theDerivs, theDim, d, anIdx) = 0.0;
                }
                aPow *= theT;
                ++anIdx;
            }
        }

        if (theAlpha > 0.0)
        {
            double aAlphaT = theAlpha * theT;
            double aSinh = Math.Sinh(aAlphaT);
            double aCosh = Math.Cosh(aAlphaT);
            double aAlphaPow = 1.0;
            for (int d = 0; d <= theMaxOrder; ++d)
            {
                bool isEven = (d % 2 == 0);
                axisVal(theDerivs, theDim, d, anIdx) = aAlphaPow * (isEven ? aSinh : aCosh);
                axisVal(theDerivs, theDim, d, anIdx + 1) = aAlphaPow * (isEven ? aCosh : aSinh);
                aAlphaPow *= theAlpha;
            }
            anIdx += 2;
        }

        if (theBeta > 0.0)
        {
            double aBetaT = theBeta * theT;
            double aSin = Math.Sin(aBetaT);
            double aCos = Math.Cos(aBetaT);
            double aBetaPow = 1.0;
            for (int d = 0; d <= theMaxOrder; ++d)
            {
                double aDSin = 0.0;
                double aDCos = 0.0;
                switch (d & 3)
                {
                    case 0:
                        aDSin = aSin;
                        aDCos = aCos;
                        break;
                    case 1:
                        aDSin = aCos;
                        aDCos = -aSin;
                        break;
                    case 2:
                        aDSin = -aSin;
                        aDCos = -aCos;
                        break;
                    case 3:
                        aDSin = -aCos;
                        aDCos = aSin;
                        break;
                }
                axisVal(theDerivs, theDim, d, anIdx) = aBetaPow * aDSin;
                axisVal(theDerivs, theDim, d, anIdx + 1) = aBetaPow * aDCos;
                aBetaPow *= theBeta;
            }
        }
    }

    private static int tensorIdx(int theDu, int theDv, int theMaxV)
    {
        return theDu * (theMaxV + 1) + theDv;
    }

    // A null theWDerivs (non-rational) in C++ is an empty span here.
    private static void evalTensorDerivs(NCollection_Array2<gp_Pnt> thePoles,
                                         NCollection_Array2<double>? theWeights,
                                         int theMaxU,
                                         int theMaxV,
                                         int theDimU,
                                         int theDimV,
                                         ReadOnlySpan<double> theBUDerivs,
                                         ReadOnlySpan<double> theBVDerivs,
                                         Span<gp_XYZ> theNDerivs,
                                         Span<double> theWDerivs)
    {
        int aTensorSize = (theMaxU + 1) * (theMaxV + 1);
        for (int idx = 0; idx < aTensorSize; ++idx)
        {
            theNDerivs[idx] = new gp_XYZ(0.0, 0.0, 0.0);
            if (!theWDerivs.IsEmpty)
            {
                theWDerivs[idx] = 0.0;
            }
        }

        int aLR = thePoles.LowerRow();
        int aLC = thePoles.LowerCol();
        int aWLR = (theWeights != null) ? theWeights.LowerRow() : 0;
        int aWLC = (theWeights != null) ? theWeights.LowerCol() : 0;

        for (int i = 0; i < theDimU; ++i)
        {
            for (int j = 0; j < theDimV; ++j)
            {
                gp_XYZ aPole = thePoles.Value(aLR + i, aLC + j).XYZ();
                double aWij = (theWeights != null) ? theWeights.Value(aWLR + i, aWLC + j) : 1.0;
                for (int du = 0; du <= theMaxU; ++du)
                {
                    double aBU = theBUDerivs[du * theDimU + i];
                    for (int dv = 0; dv <= theMaxV; ++dv)
                    {
                        int aIdx = tensorIdx(du, dv, theMaxV);
                        double aB = aBU * theBVDerivs[dv * theDimV + j];
                        if (theWeights != null)
                        {
                            double aWB = aWij * aB;
                            theNDerivs[aIdx] += aPole * aWB;
                            theWDerivs[aIdx] += aWB;
                        }
                        else
                        {
                            theNDerivs[aIdx] += aPole * aB;
                        }
                    }
                }
            }
        }
    }

    private static void computeRationalDerivs(ReadOnlySpan<gp_XYZ> theNDerivs,
                                              ReadOnlySpan<double> theWDerivs,
                                              int theMaxU,
                                              int theMaxV,
                                              Span<gp_XYZ> theCDerivs)
    {
        double aInvW = 1.0 / theWDerivs[tensorIdx(0, 0, theMaxV)];
        for (int anOrd = 0; anOrd <= theMaxU + theMaxV; ++anOrd)
        {
            int aDuUpper = Min(anOrd, theMaxU);
            for (int du = 0; du <= aDuUpper; ++du)
            {
                int dv = anOrd - du;
                if (dv > theMaxV)
                {
                    continue;
                }

                gp_XYZ aSum = theNDerivs[tensorIdx(du, dv, theMaxV)];
                for (int p = 0; p <= du; ++p)
                {
                    double aBinU = 1.0;
                    for (int m = 0; m < p; ++m)
                    {
                        aBinU = aBinU * (double)(du - m) / (double)(m + 1);
                    }
                    for (int q = 0; q <= dv; ++q)
                    {
                        if (p == 0 && q == 0)
                        {
                            continue;
                        }
                        double aBinV = 1.0;
                        for (int m = 0; m < q; ++m)
                        {
                            aBinV = aBinV * (double)(dv - m) / (double)(m + 1);
                        }
                        double aCoeff = aBinU * aBinV * theWDerivs[tensorIdx(p, q, theMaxV)];
                        aSum -= theCDerivs[tensorIdx(du - p, dv - q, theMaxV)] * aCoeff;
                    }
                }
                theCDerivs[tensorIdx(du, dv, theMaxV)] = aSum * aInvW;
            }
        }
    }

    private NCollection_Array2<gp_Pnt> myPoles;
    private NCollection_Array2<double> myWeights;
    private int myAlgDegreeU;
    private int myAlgDegreeV;
    private double myAlphaU;
    private double myAlphaV;
    private double myBetaU;
    private double myBetaV;
    private bool myURational;
    private bool myVRational;

    /// <summary>A non-rational AHT-Bezier surface.</summary>
    /// <param name="thePoles">The control points: one row per U basis function, one column per V basis function.</param>
    /// <param name="theAlgDegreeU">The algebraic polynomial degree in U (&gt;= 0).</param>
    /// <param name="theAlgDegreeV">The algebraic polynomial degree in V (&gt;= 0).</param>
    /// <param name="theAlphaU">The hyperbolic frequency in U (&gt;= 0; 0 leaves out the hyperbolic pair).</param>
    /// <param name="theAlphaV">The hyperbolic frequency in V (&gt;= 0).</param>
    /// <param name="theBetaU">The trigonometric frequency in U (&gt;= 0; 0 leaves out the trigonometric pair).</param>
    /// <param name="theBetaV">The trigonometric frequency in V (&gt;= 0).</param>
    /// <exception cref="Standard_ConstructionError">A parameter is negative, or the poles don't match the bases.</exception>
    public GeomEval_AHTBezierSurface(NCollection_Array2<gp_Pnt> thePoles,
                                     int theAlgDegreeU,
                                     int theAlgDegreeV,
                                     double theAlphaU,
                                     double theAlphaV,
                                     double theBetaU,
                                     double theBetaV)
    {
        myPoles = new NCollection_Array2<gp_Pnt>(thePoles);
        // OCCT: a 1 x 1 weights array, left uninitialised (0 here), which Weights() returns.
        myWeights = new NCollection_Array2<double>(1, 1, 1, 1);
        myAlgDegreeU = theAlgDegreeU;
        myAlgDegreeV = theAlgDegreeV;
        myAlphaU = theAlphaU;
        myAlphaV = theAlphaV;
        myBetaU = theBetaU;
        myBetaV = theBetaV;
        myURational = false;
        myVRational = false;
        if (theAlgDegreeU < 0 || theAlgDegreeV < 0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: algebraic degree must be >= 0");
        }
        if (theAlphaU < 0.0 || theAlphaV < 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: alpha must be >= 0");
        }
        if (theBetaU < 0.0 || theBetaV < 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: beta must be >= 0");
        }
        int aDimU = basisDimension(theAlgDegreeU, theAlphaU, theBetaU);
        int aDimV = basisDimension(theAlgDegreeV, theAlphaV, theBetaV);
        if (thePoles.NbRows() != aDimU || thePoles.NbColumns() != aDimV)
        {
            throw new Standard_ConstructionError(
                "GeomEval_AHTBezierSurface: poles array dimensions must match basis dimensions");
        }
    }

    /// <summary>A rational AHT-Bezier surface.</summary>
    /// <param name="thePoles">The control points: one row per U basis function, one column per V basis function.</param>
    /// <param name="theWeights">The weights, with the dimensions of the poles, all &gt; 0.</param>
    /// <param name="theAlgDegreeU">The algebraic polynomial degree in U (&gt;= 0).</param>
    /// <param name="theAlgDegreeV">The algebraic polynomial degree in V (&gt;= 0).</param>
    /// <param name="theAlphaU">The hyperbolic frequency in U (&gt;= 0).</param>
    /// <param name="theAlphaV">The hyperbolic frequency in V (&gt;= 0).</param>
    /// <param name="theBetaU">The trigonometric frequency in U (&gt;= 0).</param>
    /// <param name="theBetaV">The trigonometric frequency in V (&gt;= 0).</param>
    /// <exception cref="Standard_ConstructionError">The validation fails.</exception>
    public GeomEval_AHTBezierSurface(NCollection_Array2<gp_Pnt> thePoles,
                                     NCollection_Array2<double> theWeights,
                                     int theAlgDegreeU,
                                     int theAlgDegreeV,
                                     double theAlphaU,
                                     double theAlphaV,
                                     double theBetaU,
                                     double theBetaV)
    {
        myPoles = new NCollection_Array2<gp_Pnt>(thePoles);
        myWeights = new NCollection_Array2<double>(theWeights);
        myAlgDegreeU = theAlgDegreeU;
        myAlgDegreeV = theAlgDegreeV;
        myAlphaU = theAlphaU;
        myAlphaV = theAlphaV;
        myBetaU = theBetaU;
        myBetaV = theBetaV;
        myURational = true;
        myVRational = true;
        if (theAlgDegreeU < 0 || theAlgDegreeV < 0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: algebraic degree must be >= 0");
        }
        if (theAlphaU < 0.0 || theAlphaV < 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: alpha must be >= 0");
        }
        if (theBetaU < 0.0 || theBetaV < 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: beta must be >= 0");
        }
        int aDimU = basisDimension(theAlgDegreeU, theAlphaU, theBetaU);
        int aDimV = basisDimension(theAlgDegreeV, theAlphaV, theBetaV);
        if (thePoles.NbRows() != aDimU || thePoles.NbColumns() != aDimV)
        {
            throw new Standard_ConstructionError(
                "GeomEval_AHTBezierSurface: poles array dimensions must match basis dimensions");
        }
        if (theWeights.NbRows() != thePoles.NbRows() || theWeights.NbColumns() != thePoles.NbColumns())
        {
            throw new Standard_ConstructionError(
                "GeomEval_AHTBezierSurface: weights array dimensions must match poles");
        }
        for (int i = theWeights.LowerRow(); i <= theWeights.UpperRow(); ++i)
        {
            for (int j = theWeights.LowerCol(); j <= theWeights.UpperCol(); ++j)
            {
                if (theWeights.Value(i, j) <= 0.0)
                {
                    throw new Standard_ConstructionError("GeomEval_AHTBezierSurface: all weights must be > 0");
                }
            }
        }
    }

    /// <summary>
    /// The poles grid, row by row: the pole of (0-based) U index i and V index j is at <c>i * NbPolesV() + j</c>.
    /// </summary>
    public PackedList<gp_Pnt> Poles() => PackedList.Create<gp_Pnt>(myPoles.AsSpan());

    /// <summary>
    /// The weights grid, laid out as <see cref="Poles"/>. For a non-rational surface, OCCT's single unset weight
    /// (0 here).
    /// </summary>
    public PackedList<double> Weights() => PackedList.Create<double>(myWeights.AsSpan());

    /// <summary>The algebraic polynomial degree in U.</summary>
    public int AlgDegreeU() => myAlgDegreeU;

    /// <summary>The algebraic polynomial degree in V.</summary>
    public int AlgDegreeV() => myAlgDegreeV;

    /// <summary>The hyperbolic frequency in U.</summary>
    public double AlphaU() => myAlphaU;

    /// <summary>The hyperbolic frequency in V.</summary>
    public double AlphaV() => myAlphaV;

    /// <summary>The trigonometric frequency in U.</summary>
    public double BetaU() => myBetaU;

    /// <summary>The trigonometric frequency in V.</summary>
    public double BetaV() => myBetaV;

    /// <summary>The number of poles in the U direction.</summary>
    public int NbPolesU() => myPoles.NbRows();

    /// <summary>The number of poles in the V direction.</summary>
    public int NbPolesV() => myPoles.NbColumns();

    /// <summary>True if the surface is rational in the U direction.</summary>
    public bool IsURational() => myURational;

    /// <summary>True if the surface is rational in the V direction.</summary>
    public bool IsVRational() => myVRational;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::VReversedParameter");
    }

    /// <summary>[0, 1] x [0, 1].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 1.0;
        V1 = 0.0;
        V2 = 1.0;
    }

    public override bool IsUClosed() => false;

    public override bool IsVClosed() => false;

    public override bool IsUPeriodic() => false;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::VIso");
    }

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override bool IsCNu(int N) => true;

    public override bool IsCNv(int N) => true;

    public override gp_Pnt EvalD0(double U, double V)
    {
        int aDimU = NbPolesU();
        int aDimV = NbPolesV();
        bool isRational = (myURational || myVRational);
        bool isPurePolynomial =
            !isRational && myAlphaU <= 0.0 && myBetaU <= 0.0 && myAlphaV <= 0.0 && myBetaV <= 0.0;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData aPolyData;
            if (myAlgDegreeU == 3 && myAlgDegreeV == 3)
            {
                evalPolynomialSurfaceCubicNonRational(0, myPoles, U, V, out aPolyData);
            }
            else
            {
                evalPolynomialSurfaceNonRational(0, myPoles, myAlgDegreeU, myAlgDegreeV, U, V, out aPolyData);
            }
            return new gp_Pnt(aPolyData.S00);
        }

        Span<double> aBUDerivs = aDimU <= 16 ? stackalloc double[16] : new double[aDimU];
        Span<double> aBVDerivs = aDimV <= 16 ? stackalloc double[16] : new double[aDimV];
        evalAxisDerivs(U, 0, myAlgDegreeU, myAlphaU, myBetaU, aDimU, aBUDerivs);
        evalAxisDerivs(V, 0, myAlgDegreeV, myAlphaV, myBetaV, aDimV, aBVDerivs);

        Span<gp_XYZ> aN = stackalloc gp_XYZ[1];
        Span<double> aW = stackalloc double[1];
        evalTensorDerivs(myPoles,
                         isRational ? myWeights : null,
                         0,
                         0,
                         aDimU,
                         aDimV,
                         aBUDerivs,
                         aBVDerivs,
                         aN,
                         isRational ? aW : default);
        if (!isRational)
        {
            return new gp_Pnt(aN[0]);
        }
        return new gp_Pnt(aN[0] / aW[0]);
    }

    public override ResD1 EvalD1(double U, double V)
    {
        int aDimU = NbPolesU();
        int aDimV = NbPolesV();
        bool isRational = (myURational || myVRational);
        bool isPurePolynomial =
            !isRational && myAlphaU <= 0.0 && myBetaU <= 0.0 && myAlphaV <= 0.0 && myBetaV <= 0.0;
        ResD1 aResult;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData aPolyData;
            if (myAlgDegreeU == 3 && myAlgDegreeV == 3)
            {
                evalPolynomialSurfaceCubicNonRational(1, myPoles, U, V, out aPolyData);
            }
            else
            {
                evalPolynomialSurfaceNonRational(1, myPoles, myAlgDegreeU, myAlgDegreeV, U, V, out aPolyData);
            }
            aResult.Point = new gp_Pnt(aPolyData.S00);
            aResult.D1U = new gp_Vec(aPolyData.S10);
            aResult.D1V = new gp_Vec(aPolyData.S01);
            return aResult;
        }

        Span<double> aBUDerivs = (1 + 1) * aDimU <= 16 ? stackalloc double[16] : new double[(1 + 1) * aDimU];
        Span<double> aBVDerivs = (1 + 1) * aDimV <= 16 ? stackalloc double[16] : new double[(1 + 1) * aDimV];
        evalAxisDerivs(U, 1, myAlgDegreeU, myAlphaU, myBetaU, aDimU, aBUDerivs);
        evalAxisDerivs(V, 1, myAlgDegreeV, myAlphaV, myBetaV, aDimV, aBVDerivs);

        Span<gp_XYZ> aN = stackalloc gp_XYZ[(1 + 1) * (1 + 1)];
        Span<double> aW = stackalloc double[(1 + 1) * (1 + 1)];
        evalTensorDerivs(myPoles,
                         isRational ? myWeights : null,
                         1,
                         1,
                         aDimU,
                         aDimV,
                         aBUDerivs,
                         aBVDerivs,
                         aN,
                         isRational ? aW : default);

        if (!isRational)
        {
            aResult.Point = new gp_Pnt(aN[tensorIdx(0, 0, 1)]);
            aResult.D1U = new gp_Vec(aN[tensorIdx(1, 0, 1)]);
            aResult.D1V = new gp_Vec(aN[tensorIdx(0, 1, 1)]);
            return aResult;
        }

        double aInvW00 = 1.0 / aW[tensorIdx(0, 0, 1)];
        gp_XYZ aC00 = aN[tensorIdx(0, 0, 1)] * aInvW00;
        gp_XYZ aC10 = (aN[tensorIdx(1, 0, 1)] - aC00 * aW[tensorIdx(1, 0, 1)]) * aInvW00;
        gp_XYZ aC01 = (aN[tensorIdx(0, 1, 1)] - aC00 * aW[tensorIdx(0, 1, 1)]) * aInvW00;

        aResult.Point = new gp_Pnt(aC00);
        aResult.D1U = new gp_Vec(aC10);
        aResult.D1V = new gp_Vec(aC01);
        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        int aDimU = NbPolesU();
        int aDimV = NbPolesV();
        bool isRational = (myURational || myVRational);
        bool isPurePolynomial =
            !isRational && myAlphaU <= 0.0 && myBetaU <= 0.0 && myAlphaV <= 0.0 && myBetaV <= 0.0;
        ResD2 aResult;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData aPolyData;
            if (myAlgDegreeU == 3 && myAlgDegreeV == 3)
            {
                evalPolynomialSurfaceCubicNonRational(2, myPoles, U, V, out aPolyData);
            }
            else
            {
                evalPolynomialSurfaceNonRational(2, myPoles, myAlgDegreeU, myAlgDegreeV, U, V, out aPolyData);
            }
            aResult.Point = new gp_Pnt(aPolyData.S00);
            aResult.D1U = new gp_Vec(aPolyData.S10);
            aResult.D1V = new gp_Vec(aPolyData.S01);
            aResult.D2U = new gp_Vec(aPolyData.S20);
            aResult.D2V = new gp_Vec(aPolyData.S02);
            aResult.D2UV = new gp_Vec(aPolyData.S11);
            return aResult;
        }

        Span<double> aBUDerivs = (2 + 1) * aDimU <= 16 ? stackalloc double[16] : new double[(2 + 1) * aDimU];
        Span<double> aBVDerivs = (2 + 1) * aDimV <= 16 ? stackalloc double[16] : new double[(2 + 1) * aDimV];
        evalAxisDerivs(U, 2, myAlgDegreeU, myAlphaU, myBetaU, aDimU, aBUDerivs);
        evalAxisDerivs(V, 2, myAlgDegreeV, myAlphaV, myBetaV, aDimV, aBVDerivs);

        Span<gp_XYZ> aN = stackalloc gp_XYZ[(2 + 1) * (2 + 1)];
        Span<double> aW = stackalloc double[(2 + 1) * (2 + 1)];
        evalTensorDerivs(myPoles,
                         isRational ? myWeights : null,
                         2,
                         2,
                         aDimU,
                         aDimV,
                         aBUDerivs,
                         aBVDerivs,
                         aN,
                         isRational ? aW : default);

        if (!isRational)
        {
            aResult.Point = new gp_Pnt(aN[tensorIdx(0, 0, 2)]);
            aResult.D1U = new gp_Vec(aN[tensorIdx(1, 0, 2)]);
            aResult.D1V = new gp_Vec(aN[tensorIdx(0, 1, 2)]);
            aResult.D2U = new gp_Vec(aN[tensorIdx(2, 0, 2)]);
            aResult.D2V = new gp_Vec(aN[tensorIdx(0, 2, 2)]);
            aResult.D2UV = new gp_Vec(aN[tensorIdx(1, 1, 2)]);
            return aResult;
        }

        double aInvW00 = 1.0 / aW[tensorIdx(0, 0, 2)];
        gp_XYZ aC00 = aN[tensorIdx(0, 0, 2)] * aInvW00;
        gp_XYZ aC10 = (aN[tensorIdx(1, 0, 2)] - aC00 * aW[tensorIdx(1, 0, 2)]) * aInvW00;
        gp_XYZ aC01 = (aN[tensorIdx(0, 1, 2)] - aC00 * aW[tensorIdx(0, 1, 2)]) * aInvW00;
        gp_XYZ aC20 =
            (aN[tensorIdx(2, 0, 2)] - aC10 * (2.0 * aW[tensorIdx(1, 0, 2)]) - aC00 * aW[tensorIdx(2, 0, 2)])
            * aInvW00;
        gp_XYZ aC02 =
            (aN[tensorIdx(0, 2, 2)] - aC01 * (2.0 * aW[tensorIdx(0, 1, 2)]) - aC00 * aW[tensorIdx(0, 2, 2)])
            * aInvW00;
        gp_XYZ aC11 = (aN[tensorIdx(1, 1, 2)] - aC10 * aW[tensorIdx(0, 1, 2)]
                       - aC01 * aW[tensorIdx(1, 0, 2)] - aC00 * aW[tensorIdx(1, 1, 2)])
                      * aInvW00;

        aResult.Point = new gp_Pnt(aC00);
        aResult.D1U = new gp_Vec(aC10);
        aResult.D1V = new gp_Vec(aC01);
        aResult.D2U = new gp_Vec(aC20);
        aResult.D2V = new gp_Vec(aC02);
        aResult.D2UV = new gp_Vec(aC11);
        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        int aDimU = NbPolesU();
        int aDimV = NbPolesV();
        bool isRational = (myURational || myVRational);
        bool isPurePolynomial =
            !isRational && myAlphaU <= 0.0 && myBetaU <= 0.0 && myAlphaV <= 0.0 && myBetaV <= 0.0;
        ResD3 aResult;
        if (isPurePolynomial)
        {
            PolySurfaceDerivData aPolyData;
            if (myAlgDegreeU == 3 && myAlgDegreeV == 3)
            {
                evalPolynomialSurfaceCubicNonRational(3, myPoles, U, V, out aPolyData);
            }
            else
            {
                evalPolynomialSurfaceNonRational(3, myPoles, myAlgDegreeU, myAlgDegreeV, U, V, out aPolyData);
            }
            aResult.Point = new gp_Pnt(aPolyData.S00);
            aResult.D1U = new gp_Vec(aPolyData.S10);
            aResult.D1V = new gp_Vec(aPolyData.S01);
            aResult.D2U = new gp_Vec(aPolyData.S20);
            aResult.D2V = new gp_Vec(aPolyData.S02);
            aResult.D2UV = new gp_Vec(aPolyData.S11);
            aResult.D3U = new gp_Vec(aPolyData.S30);
            aResult.D3V = new gp_Vec(aPolyData.S03);
            aResult.D3UUV = new gp_Vec(aPolyData.S21);
            aResult.D3UVV = new gp_Vec(aPolyData.S12);
            return aResult;
        }

        Span<double> aBUDerivs = (3 + 1) * aDimU <= 16 ? stackalloc double[16] : new double[(3 + 1) * aDimU];
        Span<double> aBVDerivs = (3 + 1) * aDimV <= 16 ? stackalloc double[16] : new double[(3 + 1) * aDimV];
        evalAxisDerivs(U, 3, myAlgDegreeU, myAlphaU, myBetaU, aDimU, aBUDerivs);
        evalAxisDerivs(V, 3, myAlgDegreeV, myAlphaV, myBetaV, aDimV, aBVDerivs);

        Span<gp_XYZ> aN = stackalloc gp_XYZ[(3 + 1) * (3 + 1)];
        Span<double> aW = stackalloc double[(3 + 1) * (3 + 1)];
        evalTensorDerivs(myPoles,
                         isRational ? myWeights : null,
                         3,
                         3,
                         aDimU,
                         aDimV,
                         aBUDerivs,
                         aBVDerivs,
                         aN,
                         isRational ? aW : default);

        if (!isRational)
        {
            aResult.Point = new gp_Pnt(aN[tensorIdx(0, 0, 3)]);
            aResult.D1U = new gp_Vec(aN[tensorIdx(1, 0, 3)]);
            aResult.D1V = new gp_Vec(aN[tensorIdx(0, 1, 3)]);
            aResult.D2U = new gp_Vec(aN[tensorIdx(2, 0, 3)]);
            aResult.D2V = new gp_Vec(aN[tensorIdx(0, 2, 3)]);
            aResult.D2UV = new gp_Vec(aN[tensorIdx(1, 1, 3)]);
            aResult.D3U = new gp_Vec(aN[tensorIdx(3, 0, 3)]);
            aResult.D3V = new gp_Vec(aN[tensorIdx(0, 3, 3)]);
            aResult.D3UUV = new gp_Vec(aN[tensorIdx(2, 1, 3)]);
            aResult.D3UVV = new gp_Vec(aN[tensorIdx(1, 2, 3)]);
            return aResult;
        }

        double aInvW00 = 1.0 / aW[tensorIdx(0, 0, 3)];
        gp_XYZ aC00 = aN[tensorIdx(0, 0, 3)] * aInvW00;
        gp_XYZ aC10 = (aN[tensorIdx(1, 0, 3)] - aC00 * aW[tensorIdx(1, 0, 3)]) * aInvW00;
        gp_XYZ aC01 = (aN[tensorIdx(0, 1, 3)] - aC00 * aW[tensorIdx(0, 1, 3)]) * aInvW00;
        gp_XYZ aC20 =
            (aN[tensorIdx(2, 0, 3)] - aC10 * (2.0 * aW[tensorIdx(1, 0, 3)]) - aC00 * aW[tensorIdx(2, 0, 3)])
            * aInvW00;
        gp_XYZ aC02 =
            (aN[tensorIdx(0, 2, 3)] - aC01 * (2.0 * aW[tensorIdx(0, 1, 3)]) - aC00 * aW[tensorIdx(0, 2, 3)])
            * aInvW00;
        gp_XYZ aC11 = (aN[tensorIdx(1, 1, 3)] - aC10 * aW[tensorIdx(0, 1, 3)]
                       - aC01 * aW[tensorIdx(1, 0, 3)] - aC00 * aW[tensorIdx(1, 1, 3)])
                      * aInvW00;
        gp_XYZ aC30 = (aN[tensorIdx(3, 0, 3)] - aC20 * (3.0 * aW[tensorIdx(1, 0, 3)])
                       - aC10 * (3.0 * aW[tensorIdx(2, 0, 3)]) - aC00 * aW[tensorIdx(3, 0, 3)])
                      * aInvW00;
        gp_XYZ aC03 = (aN[tensorIdx(0, 3, 3)] - aC02 * (3.0 * aW[tensorIdx(0, 1, 3)])
                       - aC01 * (3.0 * aW[tensorIdx(0, 2, 3)]) - aC00 * aW[tensorIdx(0, 3, 3)])
                      * aInvW00;
        gp_XYZ aC21 = (aN[tensorIdx(2, 1, 3)] - aC11 * (2.0 * aW[tensorIdx(1, 0, 3)])
                       - aC20 * aW[tensorIdx(0, 1, 3)] - aC01 * aW[tensorIdx(2, 0, 3)]
                       - aC10 * (2.0 * aW[tensorIdx(1, 1, 3)]) - aC00 * aW[tensorIdx(2, 1, 3)])
                      * aInvW00;
        gp_XYZ aC12 = (aN[tensorIdx(1, 2, 3)] - aC11 * (2.0 * aW[tensorIdx(0, 1, 3)])
                       - aC02 * aW[tensorIdx(1, 0, 3)] - aC10 * aW[tensorIdx(0, 2, 3)]
                       - aC01 * (2.0 * aW[tensorIdx(1, 1, 3)]) - aC00 * aW[tensorIdx(1, 2, 3)])
                      * aInvW00;

        aResult.Point = new gp_Pnt(aC00);
        aResult.D1U = new gp_Vec(aC10);
        aResult.D1V = new gp_Vec(aC01);
        aResult.D2U = new gp_Vec(aC20);
        aResult.D2V = new gp_Vec(aC02);
        aResult.D2UV = new gp_Vec(aC11);
        aResult.D3U = new gp_Vec(aC30);
        aResult.D3V = new gp_Vec(aC03);
        aResult.D3UUV = new gp_Vec(aC21);
        aResult.D3UVV = new gp_Vec(aC12);
        return aResult;
    }

    /// <exception cref="Standard_RangeError">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Standard_RangeError("GeomEval_AHTBezierSurface::EvalDN: invalid derivative order");
        }

        int aTotOrd = Nu + Nv;
        if (aTotOrd <= 3)
        {
            if (aTotOrd == 1)
            {
                ResD1 aD1 = EvalD1(U, V);
                return (Nu == 1) ? aD1.D1U : aD1.D1V;
            }
            if (aTotOrd == 2)
            {
                ResD2 aD2 = EvalD2(U, V);
                if (Nu == 2)
                {
                    return aD2.D2U;
                }
                if (Nv == 2)
                {
                    return aD2.D2V;
                }
                return aD2.D2UV;
            }
            ResD3 aD3 = EvalD3(U, V);
            if (Nu == 3)
            {
                return aD3.D3U;
            }
            if (Nv == 3)
            {
                return aD3.D3V;
            }
            return (Nu == 2) ? aD3.D3UUV : aD3.D3UVV;
        }

        int aDimU = NbPolesU();
        int aDimV = NbPolesV();
        Span<double> aBUDerivs = (Nu + 1) * aDimU <= 16 ? stackalloc double[16] : new double[(Nu + 1) * aDimU];
        Span<double> aBVDerivs = (Nv + 1) * aDimV <= 16 ? stackalloc double[16] : new double[(Nv + 1) * aDimV];
        evalAxisDerivs(U, Nu, myAlgDegreeU, myAlphaU, myBetaU, aDimU, aBUDerivs);
        evalAxisDerivs(V, Nv, myAlgDegreeV, myAlphaV, myBetaV, aDimV, aBVDerivs);

        bool isRational = (myURational || myVRational);
        if (!isRational)
        {
            gp_XYZ aSum = new(0.0, 0.0, 0.0);
            int aLR = myPoles.LowerRow();
            int aLC = myPoles.LowerCol();
            for (int i = 0; i < aDimU; ++i)
            {
                for (int j = 0; j < aDimV; ++j)
                {
                    aSum += myPoles.Value(aLR + i, aLC + j).XYZ()
                            * (aBUDerivs[Nu * aDimU + i] * aBVDerivs[Nv * aDimV + j]);
                }
            }
            return new gp_Vec(aSum);
        }

        int aTensorSize = (Nu + 1) * (Nv + 1);
        Span<gp_XYZ> aNDerivs = aTensorSize <= 16 ? stackalloc gp_XYZ[16] : new gp_XYZ[aTensorSize];
        Span<double> aWDerivs = aTensorSize <= 16 ? stackalloc double[16] : new double[aTensorSize];
        Span<gp_XYZ> aCDerivs = aTensorSize <= 16 ? stackalloc gp_XYZ[16] : new gp_XYZ[aTensorSize];
        evalTensorDerivs(myPoles,
                         myWeights,
                         Nu,
                         Nv,
                         aDimU,
                         aDimV,
                         aBUDerivs,
                         aBVDerivs,
                         aNDerivs,
                         aWDerivs);
        computeRationalDerivs(aNDerivs, aWDerivs, Nu, Nv, aCDerivs);
        return new gp_Vec(aCDerivs[tensorIdx(Nu, Nv, Nv)]);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_AHTBezierSurface::Transform");
    }

    public override Geom_Geometry Copy()
    {
        if (myURational || myVRational)
        {
            return new GeomEval_AHTBezierSurface(myPoles,
                                                 myWeights,
                                                 myAlgDegreeU,
                                                 myAlgDegreeV,
                                                 myAlphaU,
                                                 myAlphaV,
                                                 myBetaU,
                                                 myBetaV);
        }
        return new GeomEval_AHTBezierSurface(myPoles,
                                             myAlgDegreeU,
                                             myAlgDegreeV,
                                             myAlphaU,
                                             myAlphaV,
                                             myBetaU,
                                             myBetaV);
    }
}
