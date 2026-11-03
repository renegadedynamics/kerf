// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_TBezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A tensor-product trigonometric Bezier surface, over the basis {1, sin(alpha*t), cos(alpha*t), ...,
/// sin(n*alpha*t), cos(n*alpha*t)} in each direction: <c>S(u,v) = sum_i sum_j P_ij * Bu_i(u) * Bv_j(v)</c>, or
/// for a rational surface <c>sum(w_ij*P_ij*Bu_i*Bv_j) / sum(w_ij*Bu_i*Bv_j)</c>. The poles form a
/// (2*nU + 1) x (2*nV + 1) grid (rows are U, columns V); the domain is [0, Pi/alphaU] x [0, Pi/alphaV].
/// Reversal, transformation and iso-curves are not supported (they throw <see cref="MethodNotImplementedException"/>),
/// nor are rational derivatives of order above 3 (<see cref="Geom_UndefinedDerivative"/>).
/// </summary>
public sealed class GeomEval_TBezierSurface : Geom_BoundedSurface
{
    // Validate that a dimension count is odd and >= 3.
    private static void validateDimension(int theSize, string theName)
    {
        if (theSize < 3 || (theSize % 2) == 0)
        {
            string aMsg = "GeomEval_TBezierSurface: ";
            aMsg += theName;
            aMsg += " pole count must be odd and >= 3";
            throw new ConstructionErrorException(aMsg);
        }
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

    // The C++ template evalTrigAxisDerivs<theMaxOrder>: the `if constexpr` on the order are plain tests here.
    // theDerivs holds (theMaxOrder + 1) blocks of 2 * theOrder + 1 values, one per derivative order.
    private static void evalTrigAxisDerivs(int theMaxOrder, double theT, double theAlpha, int theOrder,
                                           Span<double> theDerivs)
    {
        int aDim = 2 * theOrder + 1;
        for (int d = 0; d <= theMaxOrder; ++d)
        {
            theDerivs[d * aDim] = (d == 0) ? 1.0 : 0.0;
        }

        int aIdx = 1;
        for (int k = 1; k <= theOrder; ++k, aIdx += 2)
        {
            double aFreq = (double)k * theAlpha;
            double anArg = aFreq * theT;
            double aSin = Math.Sin(anArg);
            double aCos = Math.Cos(anArg);
            double aFreq2 = aFreq * aFreq;
            double aFreq3 = aFreq2 * aFreq;

            theDerivs[aIdx] = aSin;
            theDerivs[aIdx + 1] = aCos;
            if (theMaxOrder >= 1)
            {
                theDerivs[aDim + aIdx] = aFreq * aCos;
                theDerivs[aDim + aIdx + 1] = -aFreq * aSin;
            }
            if (theMaxOrder >= 2)
            {
                theDerivs[2 * aDim + aIdx] = -aFreq2 * aSin;
                theDerivs[2 * aDim + aIdx + 1] = -aFreq2 * aCos;
            }
            if (theMaxOrder >= 3)
            {
                theDerivs[3 * aDim + aIdx] = -aFreq3 * aCos;
                theDerivs[3 * aDim + aIdx + 1] = aFreq3 * aSin;
            }
        }
    }

    private static void evalTrigAxisNthDeriv(double theT, double theAlpha, int theOrder, int theDerivOrder,
                                             Span<double> theDerivs)
    {
        theDerivs[0] = (theDerivOrder == 0) ? 1.0 : 0.0;
        if (theDerivOrder == 0)
        {
            int aIdx0 = 1;
            for (int k = 1; k <= theOrder; ++k, aIdx0 += 2)
            {
                double anArg = (double)k * theAlpha * theT;
                theDerivs[aIdx0] = Math.Sin(anArg);
                theDerivs[aIdx0 + 1] = Math.Cos(anArg);
            }
            return;
        }

        int aOrderMod4 = theDerivOrder & 3;
        int aIdx = 1;
        for (int k = 1; k <= theOrder; ++k, aIdx += 2)
        {
            double aFreq = (double)k * theAlpha;
            double anArg = aFreq * theT;
            double aSin = Math.Sin(anArg);
            double aCos = Math.Cos(anArg);
            double aPow = powInt(aFreq, theDerivOrder);
            evalTrigPairNthDeriv(aSin, aCos, aOrderMod4, out double aSinDeriv, out double aCosDeriv);
            theDerivs[aIdx] = aPow * aSinDeriv;
            theDerivs[aIdx + 1] = aPow * aCosDeriv;
        }
    }

    private NCollection_Array2<Point3D> myPoles;   // Control points grid
    private NCollection_Array2<double> myWeights; // Weights grid (empty if non-rational)
    private double myAlphaU;                      // Frequency parameter in U
    private double myAlphaV;                      // Frequency parameter in V
    private bool myRational;                      // True if surface is rational

    /// <summary>A non-rational T-Bezier surface.</summary>
    /// <param name="thePoles">The control points; the numbers of rows (U) and columns (V) must be odd and at least 3.</param>
    /// <param name="theAlphaU">The frequency parameter in U (&gt; 0).</param>
    /// <param name="theAlphaV">The frequency parameter in V (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">The validation fails.</exception>
    public GeomEval_TBezierSurface(NCollection_Array2<Point3D> thePoles, double theAlphaU, double theAlphaV)
    {
        myPoles = new NCollection_Array2<Point3D>(thePoles);
        myWeights = new NCollection_Array2<double>();
        myAlphaU = theAlphaU;
        myAlphaV = theAlphaV;
        myRational = false;
        validateDimension(thePoles.NbRows(), "U");
        validateDimension(thePoles.NbColumns(), "V");
        if (theAlphaU <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaU must be > 0");
        }
        if (theAlphaV <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaV must be > 0");
        }
    }

    /// <summary>A rational T-Bezier surface.</summary>
    /// <param name="thePoles">The control points; the numbers of rows (U) and columns (V) must be odd and at least 3.</param>
    /// <param name="theWeights">The weights, with the dimensions of the poles, all &gt; 0.</param>
    /// <param name="theAlphaU">The frequency parameter in U (&gt; 0).</param>
    /// <param name="theAlphaV">The frequency parameter in V (&gt; 0).</param>
    /// <exception cref="ConstructionErrorException">The validation fails.</exception>
    public GeomEval_TBezierSurface(NCollection_Array2<Point3D> thePoles, NCollection_Array2<double> theWeights,
                                   double theAlphaU, double theAlphaV)
    {
        myPoles = new NCollection_Array2<Point3D>(thePoles);
        myWeights = new NCollection_Array2<double>(theWeights);
        myAlphaU = theAlphaU;
        myAlphaV = theAlphaV;
        myRational = true;
        validateDimension(thePoles.NbRows(), "U");
        validateDimension(thePoles.NbColumns(), "V");
        if (theAlphaU <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaU must be > 0");
        }
        if (theAlphaV <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_TBezierSurface: alphaV must be > 0");
        }
        if (theWeights.NbRows() != thePoles.NbRows() || theWeights.NbColumns() != thePoles.NbColumns())
        {
            throw new ConstructionErrorException(
                "GeomEval_TBezierSurface: weights dimensions must match poles dimensions");
        }
        for (int i = theWeights.LowerRow(); i <= theWeights.UpperRow(); ++i)
        {
            for (int j = theWeights.LowerCol(); j <= theWeights.UpperCol(); ++j)
            {
                if (theWeights.Value(i, j) <= 0.0)
                {
                    throw new ConstructionErrorException("GeomEval_TBezierSurface: all weights must be > 0");
                }
            }
        }
    }

    /// <summary>
    /// The poles grid, row by row: the pole of (0-based) U index i and V index j is at <c>i * NbVPoles() + j</c>.
    /// </summary>
    public PackedList<Point3D> Poles() => PackedList.Create<Point3D>(myPoles.AsSpan());

    /// <summary>The weights grid, laid out as <see cref="Poles"/>; empty if the surface is non-rational.</summary>
    public PackedList<double> Weights() => PackedList.Create<double>(myWeights.AsSpan());

    /// <summary>The frequency parameter alpha in the U direction.</summary>
    public double AlphaU() => myAlphaU;

    /// <summary>The frequency parameter alpha in the V direction.</summary>
    public double AlphaV() => myAlphaV;

    /// <summary>The number of poles in the U direction.</summary>
    public int NbUPoles() => myPoles.NbRows();

    /// <summary>The number of poles in the V direction.</summary>
    public int NbVPoles() => myPoles.NbColumns();

    /// <summary>The trigonometric order in U (NbUPoles = 2*nU + 1).</summary>
    public int OrderU() => (myPoles.NbRows() - 1) / 2;

    /// <summary>The trigonometric order in V (NbVPoles = 2*nV + 1).</summary>
    public int OrderV() => (myPoles.NbColumns() - 1) / 2;

    /// <summary>True if the surface is rational.</summary>
    public bool IsRational() => myRational;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double U)
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
    public override double VReversedParameter(double V)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::VReversedParameter");
    }

    /// <summary>[0, Pi/alphaU] x [0, Pi/alphaV].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = M_PI / myAlphaU;
        V1 = 0.0;
        V2 = M_PI / myAlphaV;
    }

    /// <summary>True if the first and last U-rows of poles coincide.</summary>
    public override bool IsUClosed()
    {
        // Check if the first and last U-rows of poles coincide.
        // OCCT compares pole rows, but in the trigonometric basis the boundary iso-curves are
        // S(0, v) = sum_j (P_0j + P_2j + P_4j + ...) Bv_j and S(Pi/alphaU, v) = sum_j (P_0j - P_2j + P_4j ...) Bv_j,
        // so the result says nothing about closure (see occt-issues.md).
        int aLR = myPoles.LowerRow();
        int aUR = myPoles.UpperRow();
        for (int j = myPoles.LowerCol(); j <= myPoles.UpperCol(); ++j)
        {
            if (myPoles.Value(aLR, j).Distance(myPoles.Value(aUR, j)) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>True if the first and last V-columns of poles coincide.</summary>
    public override bool IsVClosed()
    {
        // Check if the first and last V-columns of poles coincide.
        // OCCT: the same pole test as IsUClosed, which says nothing about closure (see occt-issues.md).
        int aLC = myPoles.LowerCol();
        int aUC = myPoles.UpperCol();
        for (int i = myPoles.LowerRow(); i <= myPoles.UpperRow(); ++i)
        {
            if (myPoles.Value(i, aLC).Distance(myPoles.Value(i, aUC)) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    public override bool IsUPeriodic() => false;

    public override bool IsVPeriodic() => false;

    public override Continuity Continuity() => CN;

    public override bool IsCNu(int N) => true;

    public override bool IsCNv(int N) => true;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::VIso");
    }

    // Evaluates a single trigonometric basis array for the given alpha. Unused, as in OCCT.
    private static void evalTrigBasis(double theT, double theAlpha, int theOrder, NCollection_Array1<double> theBasis)
    {
        // T_0(t) = 1
        theBasis.SetValue(theBasis.Lower(), 1.0);
        for (int k = 1; k <= theOrder; ++k)
        {
            double anArg = k * theAlpha * theT;
            theBasis.SetValue(theBasis.Lower() + 2 * k - 1, Math.Sin(anArg));
            theBasis.SetValue(theBasis.Lower() + 2 * k, Math.Cos(anArg));
        }
    }

    // Evaluates a derivative of the trigonometric basis for the given alpha. Unused, as in OCCT.
    private static void evalTrigBasisDeriv(double theT, double theAlpha, int theOrder, int theDerivOrder,
                                           NCollection_Array1<double> theBasisDeriv)
    {
        int aDim = 2 * theOrder + 1;
        Span<double> aDerivs = aDim <= 16 ? stackalloc double[16] : new double[aDim];
        evalTrigAxisNthDeriv(theT, theAlpha, theOrder, theDerivOrder, aDerivs);
        for (int i = 0; i < aDim; ++i)
        {
            theBasisDeriv.SetValue(theBasisDeriv.Lower() + i, aDerivs[i]);
        }
    }

    private void evalBasisU(double theU, NCollection_Array1<double> theBasis)
    {
        evalTrigBasis(theU, myAlphaU, OrderU(), theBasis);
    }

    private void evalBasisV(double theV, NCollection_Array1<double> theBasis)
    {
        evalTrigBasis(theV, myAlphaV, OrderV(), theBasis);
    }

    private void evalBasisDerivU(double theU, int theOrder, NCollection_Array1<double> theBasisDeriv)
    {
        evalTrigBasisDeriv(theU, myAlphaU, OrderU(), theOrder, theBasisDeriv);
    }

    private void evalBasisDerivV(double theV, int theOrder, NCollection_Array1<double> theBasisDeriv)
    {
        evalTrigBasisDeriv(theV, myAlphaV, OrderV(), theOrder, theBasisDeriv);
    }

    public override Point3D EvalD0(double U, double V)
    {
        int aNbU = myPoles.NbRows();
        int aNbV = myPoles.NbColumns();
        Span<double> aBasisU = aNbU <= 16 ? stackalloc double[16] : new double[aNbU];
        Span<double> aBasisV = aNbV <= 16 ? stackalloc double[16] : new double[aNbV];
        evalTrigAxisDerivs(0, U, myAlphaU, (aNbU - 1) / 2, aBasisU);
        evalTrigAxisDerivs(0, V, myAlphaV, (aNbV - 1) / 2, aBasisV);

        if (!myRational)
        {
            Xyz aSum = new(0.0, 0.0, 0.0);
            for (int i = 0; i < aNbU; ++i)
            {
                double aBu = aBasisU[i];
                for (int j = 0; j < aNbV; ++j)
                {
                    aSum +=
                        (aBu * aBasisV[j]) * myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                }
            }
            return new Point3D(aSum);
        }

        // Rational evaluation.
        Xyz aNumer = new(0.0, 0.0, 0.0);
        double aDenom = 0.0;
        for (int i = 0; i < aNbU; ++i)
        {
            double aBu = aBasisU[i];
            for (int j = 0; j < aNbV; ++j)
            {
                double aWBuBv =
                    myWeights.Value(myWeights.LowerRow() + i, myWeights.LowerCol() + j) * aBu * aBasisV[j];
                aNumer += aWBuBv * myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                aDenom += aWBuBv;
            }
        }
        return new Point3D(aNumer / aDenom);
    }

    public override ResD1 EvalD1(double U, double V)
    {
        int aNbU = myPoles.NbRows();
        int aNbV = myPoles.NbColumns();
        Span<double> aBUDerivs = (1 + 1) * aNbU <= 16 ? stackalloc double[16] : new double[(1 + 1) * aNbU];
        Span<double> aBVDerivs = (1 + 1) * aNbV <= 16 ? stackalloc double[16] : new double[(1 + 1) * aNbV];
        evalTrigAxisDerivs(1, U, myAlphaU, (aNbU - 1) / 2, aBUDerivs);
        evalTrigAxisDerivs(1, V, myAlphaV, (aNbV - 1) / 2, aBVDerivs);

        ReadOnlySpan<double> aBasisU = aBUDerivs;
        ReadOnlySpan<double> aBasisDU = aBUDerivs[aNbU..];
        ReadOnlySpan<double> aBasisV = aBVDerivs;
        ReadOnlySpan<double> aBasisDV = aBVDerivs[aNbV..];

        ResD1 aResult;

        if (!myRational)
        {
            Xyz aS = new(0.0, 0.0, 0.0);
            Xyz aDU = new(0.0, 0.0, 0.0);
            Xyz aDV = new(0.0, 0.0, 0.0);
            for (int i = 0; i < aNbU; ++i)
            {
                double aBu = aBasisU[i];
                double aBuD = aBasisDU[i];
                for (int j = 0; j < aNbV; ++j)
                {
                    double aBv = aBasisV[j];
                    double aBvD = aBasisDV[j];
                    Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                    aS += (aBu * aBv) * aPij;
                    aDU += (aBuD * aBv) * aPij;
                    aDV += (aBu * aBvD) * aPij;
                }
            }
            aResult.Point = new Point3D(aS);
            aResult.D1U = new Vector3D(aDU);
            aResult.D1V = new Vector3D(aDV);
            return aResult;
        }

        // Rational: S = Sw/w, dS/du = (Sw_u - w_u*S)/w, dS/dv = (Sw_v - w_v*S)/w
        Xyz aSw = new(0.0, 0.0, 0.0);
        Xyz aSwDU = new(0.0, 0.0, 0.0);
        Xyz aSwDV = new(0.0, 0.0, 0.0);
        double aW = 0.0;
        double aWDU = 0.0;
        double aWDV = 0.0;
        for (int i = 0; i < aNbU; ++i)
        {
            double aBu = aBasisU[i];
            double aBuD = aBasisDU[i];
            for (int j = 0; j < aNbV; ++j)
            {
                double aBv = aBasisV[j];
                double aBvD = aBasisDV[j];
                double aWij = myWeights.Value(myWeights.LowerRow() + i, myWeights.LowerCol() + j);
                Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                double aWBuBv = aWij * aBu * aBv;
                double aWBuDBv = aWij * aBuD * aBv;
                double aWBuBvD = aWij * aBu * aBvD;
                aSw += aWBuBv * aPij;
                aSwDU += aWBuDBv * aPij;
                aSwDV += aWBuBvD * aPij;
                aW += aWBuBv;
                aWDU += aWBuDBv;
                aWDV += aWBuBvD;
            }
        }

        Xyz aSr = aSw / aW;
        aResult.Point = new Point3D(aSr);
        aResult.D1U = new Vector3D((aSwDU - aWDU * aSr) / aW);
        aResult.D1V = new Vector3D((aSwDV - aWDV * aSr) / aW);
        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        int aNbU = myPoles.NbRows();
        int aNbV = myPoles.NbColumns();
        Span<double> aBUDerivs = (2 + 1) * aNbU <= 16 ? stackalloc double[16] : new double[(2 + 1) * aNbU];
        Span<double> aBVDerivs = (2 + 1) * aNbV <= 16 ? stackalloc double[16] : new double[(2 + 1) * aNbV];
        evalTrigAxisDerivs(2, U, myAlphaU, (aNbU - 1) / 2, aBUDerivs);
        evalTrigAxisDerivs(2, V, myAlphaV, (aNbV - 1) / 2, aBVDerivs);

        ReadOnlySpan<double> aBasisU = aBUDerivs;
        ReadOnlySpan<double> aBasisDU = aBUDerivs[aNbU..];
        ReadOnlySpan<double> aBasisD2U = aBUDerivs[(2 * aNbU)..];
        ReadOnlySpan<double> aBasisV = aBVDerivs;
        ReadOnlySpan<double> aBasisDV = aBVDerivs[aNbV..];
        ReadOnlySpan<double> aBasisD2V = aBVDerivs[(2 * aNbV)..];

        ResD2 aResult;

        if (!myRational)
        {
            Xyz aS = new(0.0, 0.0, 0.0);
            Xyz aDU = new(0.0, 0.0, 0.0);
            Xyz aDV = new(0.0, 0.0, 0.0);
            Xyz aD2U = new(0.0, 0.0, 0.0);
            Xyz aD2V = new(0.0, 0.0, 0.0);
            Xyz aD2UV = new(0.0, 0.0, 0.0);
            for (int i = 0; i < aNbU; ++i)
            {
                double aBu = aBasisU[i];
                double aBuD = aBasisDU[i];
                double aBuD2 = aBasisD2U[i];
                for (int j = 0; j < aNbV; ++j)
                {
                    double aBv = aBasisV[j];
                    double aBvD = aBasisDV[j];
                    double aBvD2 = aBasisD2V[j];
                    Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                    aS += (aBu * aBv) * aPij;
                    aDU += (aBuD * aBv) * aPij;
                    aDV += (aBu * aBvD) * aPij;
                    aD2U += (aBuD2 * aBv) * aPij;
                    aD2V += (aBu * aBvD2) * aPij;
                    aD2UV += (aBuD * aBvD) * aPij;
                }
            }
            aResult.Point = new Point3D(aS);
            aResult.D1U = new Vector3D(aDU);
            aResult.D1V = new Vector3D(aDV);
            aResult.D2U = new Vector3D(aD2U);
            aResult.D2V = new Vector3D(aD2V);
            aResult.D2UV = new Vector3D(aD2UV);
            return aResult;
        }

        // Rational second derivatives using quotient rule.
        Xyz aSw = new(0.0, 0.0, 0.0);
        Xyz aSwDU = new(0.0, 0.0, 0.0);
        Xyz aSwDV = new(0.0, 0.0, 0.0);
        Xyz aSwD2U = new(0.0, 0.0, 0.0);
        Xyz aSwD2V = new(0.0, 0.0, 0.0);
        Xyz aSwDUV = new(0.0, 0.0, 0.0);
        double aW = 0.0;
        double aWDU = 0.0;
        double aWDV = 0.0;
        double aWD2U = 0.0;
        double aWD2V = 0.0;
        double aWDUV = 0.0;
        for (int i = 0; i < aNbU; ++i)
        {
            double aBu = aBasisU[i];
            double aBuD = aBasisDU[i];
            double aBuD2 = aBasisD2U[i];
            for (int j = 0; j < aNbV; ++j)
            {
                double aBv = aBasisV[j];
                double aBvD = aBasisDV[j];
                double aBvD2 = aBasisD2V[j];
                double aWij = myWeights.Value(myWeights.LowerRow() + i, myWeights.LowerCol() + j);
                Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();

                double aWBuBv = aWij * aBu * aBv;
                double aWBuDBv = aWij * aBuD * aBv;
                double aWBuBvD = aWij * aBu * aBvD;
                double aWBuD2Bv = aWij * aBuD2 * aBv;
                double aWBuBvD2 = aWij * aBu * aBvD2;
                double aWBuDBvD = aWij * aBuD * aBvD;

                aSw += aWBuBv * aPij;
                aSwDU += aWBuDBv * aPij;
                aSwDV += aWBuBvD * aPij;
                aSwD2U += aWBuD2Bv * aPij;
                aSwD2V += aWBuBvD2 * aPij;
                aSwDUV += aWBuDBvD * aPij;
                aW += aWBuBv;
                aWDU += aWBuDBv;
                aWDV += aWBuBvD;
                aWD2U += aWBuD2Bv;
                aWD2V += aWBuBvD2;
                aWDUV += aWBuDBvD;
            }
        }

        // S = Sw / w
        Xyz aSr = aSw / aW;
        // First derivatives: S_u = (Sw_u - w_u * S) / w
        Xyz aSD1U = (aSwDU - aWDU * aSr) / aW;
        Xyz aSD1V = (aSwDV - aWDV * aSr) / aW;
        // Second derivatives: S_uu = (Sw_uu - 2*w_u*S_u - w_uu*S) / w
        Xyz aSD2U = (aSwD2U - 2.0 * aWDU * aSD1U - aWD2U * aSr) / aW;
        Xyz aSD2V = (aSwD2V - 2.0 * aWDV * aSD1V - aWD2V * aSr) / aW;
        Xyz aSD2UV = (aSwDUV - aWDU * aSD1V - aWDV * aSD1U - aWDUV * aSr) / aW;

        aResult.Point = new Point3D(aSr);
        aResult.D1U = new Vector3D(aSD1U);
        aResult.D1V = new Vector3D(aSD1V);
        aResult.D2U = new Vector3D(aSD2U);
        aResult.D2V = new Vector3D(aSD2V);
        aResult.D2UV = new Vector3D(aSD2UV);
        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        int aNbU = myPoles.NbRows();
        int aNbV = myPoles.NbColumns();
        Span<double> aBUDerivs = (3 + 1) * aNbU <= 16 ? stackalloc double[16] : new double[(3 + 1) * aNbU];
        Span<double> aBVDerivs = (3 + 1) * aNbV <= 16 ? stackalloc double[16] : new double[(3 + 1) * aNbV];
        evalTrigAxisDerivs(3, U, myAlphaU, (aNbU - 1) / 2, aBUDerivs);
        evalTrigAxisDerivs(3, V, myAlphaV, (aNbV - 1) / 2, aBVDerivs);

        ReadOnlySpan<double> aBasisU = aBUDerivs;
        ReadOnlySpan<double> aBasisDU = aBUDerivs[aNbU..];
        ReadOnlySpan<double> aBasisD2U = aBUDerivs[(2 * aNbU)..];
        ReadOnlySpan<double> aBasisD3U = aBUDerivs[(3 * aNbU)..];
        ReadOnlySpan<double> aBasisV = aBVDerivs;
        ReadOnlySpan<double> aBasisDV = aBVDerivs[aNbV..];
        ReadOnlySpan<double> aBasisD2V = aBVDerivs[(2 * aNbV)..];
        ReadOnlySpan<double> aBasisD3V = aBVDerivs[(3 * aNbV)..];

        ResD3 aResult;

        if (!myRational)
        {
            Xyz aS = new(0.0, 0.0, 0.0);
            Xyz aDU = new(0.0, 0.0, 0.0);
            Xyz aDV = new(0.0, 0.0, 0.0);
            Xyz aD2U = new(0.0, 0.0, 0.0);
            Xyz aD2V = new(0.0, 0.0, 0.0);
            Xyz aD2UV = new(0.0, 0.0, 0.0);
            Xyz aD3U = new(0.0, 0.0, 0.0);
            Xyz aD3V = new(0.0, 0.0, 0.0);
            Xyz aD3UUV = new(0.0, 0.0, 0.0);
            Xyz aD3UVV = new(0.0, 0.0, 0.0);

            for (int i = 0; i < aNbU; ++i)
            {
                double aBu = aBasisU[i];
                double aBuD = aBasisDU[i];
                double aBuD2 = aBasisD2U[i];
                double aBuD3 = aBasisD3U[i];
                for (int j = 0; j < aNbV; ++j)
                {
                    double aBv = aBasisV[j];
                    double aBvD = aBasisDV[j];
                    double aBvD2 = aBasisD2V[j];
                    double aBvD3 = aBasisD3V[j];
                    Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();

                    aS += (aBu * aBv) * aPij;
                    aDU += (aBuD * aBv) * aPij;
                    aDV += (aBu * aBvD) * aPij;
                    aD2U += (aBuD2 * aBv) * aPij;
                    aD2V += (aBu * aBvD2) * aPij;
                    aD2UV += (aBuD * aBvD) * aPij;
                    aD3U += (aBuD3 * aBv) * aPij;
                    aD3V += (aBu * aBvD3) * aPij;
                    aD3UUV += (aBuD2 * aBvD) * aPij;
                    aD3UVV += (aBuD * aBvD2) * aPij;
                }
            }
            aResult.Point = new Point3D(aS);
            aResult.D1U = new Vector3D(aDU);
            aResult.D1V = new Vector3D(aDV);
            aResult.D2U = new Vector3D(aD2U);
            aResult.D2V = new Vector3D(aD2V);
            aResult.D2UV = new Vector3D(aD2UV);
            aResult.D3U = new Vector3D(aD3U);
            aResult.D3V = new Vector3D(aD3V);
            aResult.D3UUV = new Vector3D(aD3UUV);
            aResult.D3UVV = new Vector3D(aD3UVV);
            return aResult;
        }

        // Rational third derivatives: accumulate all weighted sums.
        Xyz aSw = new(0.0, 0.0, 0.0);
        Xyz aSwDU = new(0.0, 0.0, 0.0);
        Xyz aSwDV = new(0.0, 0.0, 0.0);
        Xyz aSwD2U = new(0.0, 0.0, 0.0);
        Xyz aSwD2V = new(0.0, 0.0, 0.0);
        Xyz aSwDUV = new(0.0, 0.0, 0.0);
        Xyz aSwD3U = new(0.0, 0.0, 0.0);
        Xyz aSwD3V = new(0.0, 0.0, 0.0);
        Xyz aSwD2UDV = new(0.0, 0.0, 0.0);
        Xyz aSwDUD2V = new(0.0, 0.0, 0.0);
        double aW = 0.0;
        double aWDU = 0.0;
        double aWDV = 0.0;
        double aWD2U = 0.0;
        double aWD2V = 0.0;
        double aWDUV = 0.0;
        double aWD3U = 0.0;
        double aWD3V = 0.0;
        double aWD2UDV = 0.0;
        double aWDUD2V = 0.0;

        for (int i = 0; i < aNbU; ++i)
        {
            double aBu = aBasisU[i];
            double aBuD = aBasisDU[i];
            double aBuD2 = aBasisD2U[i];
            double aBuD3 = aBasisD3U[i];
            for (int j = 0; j < aNbV; ++j)
            {
                double aBv = aBasisV[j];
                double aBvD = aBasisDV[j];
                double aBvD2 = aBasisD2V[j];
                double aBvD3 = aBasisD3V[j];
                double aWij = myWeights.Value(myWeights.LowerRow() + i, myWeights.LowerCol() + j);
                Xyz aPij = myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();

                double aWBuBv = aWij * aBu * aBv;
                double aWBuDBv = aWij * aBuD * aBv;
                double aWBuBvD = aWij * aBu * aBvD;
                double aWBuD2Bv = aWij * aBuD2 * aBv;
                double aWBuBvD2 = aWij * aBu * aBvD2;
                double aWBuDBvD = aWij * aBuD * aBvD;
                double aWBuD3Bv = aWij * aBuD3 * aBv;
                double aWBuBvD3 = aWij * aBu * aBvD3;
                double aWBuD2BvD = aWij * aBuD2 * aBvD;
                double aWBuDBvD2 = aWij * aBuD * aBvD2;

                aSw += aWBuBv * aPij;
                aSwDU += aWBuDBv * aPij;
                aSwDV += aWBuBvD * aPij;
                aSwD2U += aWBuD2Bv * aPij;
                aSwD2V += aWBuBvD2 * aPij;
                aSwDUV += aWBuDBvD * aPij;
                aSwD3U += aWBuD3Bv * aPij;
                aSwD3V += aWBuBvD3 * aPij;
                aSwD2UDV += aWBuD2BvD * aPij;
                aSwDUD2V += aWBuDBvD2 * aPij;
                aW += aWBuBv;
                aWDU += aWBuDBv;
                aWDV += aWBuBvD;
                aWD2U += aWBuD2Bv;
                aWD2V += aWBuBvD2;
                aWDUV += aWBuDBvD;
                aWD3U += aWBuD3Bv;
                aWD3V += aWBuBvD3;
                aWD2UDV += aWBuD2BvD;
                aWDUD2V += aWBuDBvD2;
            }
        }

        // S = Sw / w
        Xyz aSr = aSw / aW;
        // First derivatives: S_u = (Sw_u - w_u * S) / w
        Xyz aSD1U = (aSwDU - aWDU * aSr) / aW;
        Xyz aSD1V = (aSwDV - aWDV * aSr) / aW;
        // Second derivatives: S_uu = (Sw_uu - 2*w_u*S_u - w_uu*S) / w
        Xyz aSD2U = (aSwD2U - 2.0 * aWDU * aSD1U - aWD2U * aSr) / aW;
        Xyz aSD2V = (aSwD2V - 2.0 * aWDV * aSD1V - aWD2V * aSr) / aW;
        Xyz aSD2UV = (aSwDUV - aWDU * aSD1V - aWDV * aSD1U - aWDUV * aSr) / aW;
        // Third derivatives: obtained by differentiating the Leibniz rule expressions.
        // S_uuu = (Sw_uuu - 3*w_u*S_uu - 3*w_uu*S_u - w_uuu*S) / w
        Xyz aSD3U = (aSwD3U - 3.0 * aWDU * aSD2U - 3.0 * aWD2U * aSD1U - aWD3U * aSr) / aW;
        // S_vvv = (Sw_vvv - 3*w_v*S_vv - 3*w_vv*S_v - w_vvv*S) / w
        Xyz aSD3V = (aSwD3V - 3.0 * aWDV * aSD2V - 3.0 * aWD2V * aSD1V - aWD3V * aSr) / aW;
        // S_uuv: differentiate w*S_uu = Sw_uu - 2*w_u*S_u - w_uu*S with respect to v.
        Xyz aSD3UUV = (aSwD2UDV - aWD2UDV * aSr - 2.0 * aWDU * aSD2UV - aWDV * aSD2U
                          - aWD2U * aSD1V - 2.0 * aWDUV * aSD1U)
                         / aW;
        // S_uvv: differentiate w*S_vv = Sw_vv - 2*w_v*S_v - w_vv*S with respect to u.
        Xyz aSD3UVV = (aSwDUD2V - aWDUD2V * aSr - 2.0 * aWDV * aSD2UV - aWDU * aSD2V
                          - aWD2V * aSD1U - 2.0 * aWDUV * aSD1V)
                         / aW;

        aResult.Point = new Point3D(aSr);
        aResult.D1U = new Vector3D(aSD1U);
        aResult.D1V = new Vector3D(aSD1V);
        aResult.D2U = new Vector3D(aSD2U);
        aResult.D2V = new Vector3D(aSD2V);
        aResult.D2UV = new Vector3D(aSD2UV);
        aResult.D3U = new Vector3D(aSD3U);
        aResult.D3V = new Vector3D(aSD3V);
        aResult.D3UUV = new Vector3D(aSD3UUV);
        aResult.D3UVV = new Vector3D(aSD3UVV);
        return aResult;
    }

    /// <exception cref="RangeErrorException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    /// <exception cref="Geom_UndefinedDerivative">A rational surface and Nu + Nv &gt; 3.</exception>
    public override Vector3D EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new RangeErrorException("GeomEval_TBezierSurface::EvalDN: invalid derivative order");
        }

        // For non-rational surfaces, compute directly from basis derivatives.
        if (!myRational)
        {
            int aNbU = myPoles.NbRows();
            int aNbV = myPoles.NbColumns();
            Span<double> aBasisU = aNbU <= 16 ? stackalloc double[16] : new double[aNbU];
            Span<double> aBasisV = aNbV <= 16 ? stackalloc double[16] : new double[aNbV];
            evalTrigAxisNthDeriv(U, myAlphaU, (aNbU - 1) / 2, Nu, aBasisU);
            evalTrigAxisNthDeriv(V, myAlphaV, (aNbV - 1) / 2, Nv, aBasisV);

            Xyz aSum = new(0.0, 0.0, 0.0);
            for (int i = 0; i < aNbU; ++i)
            {
                double aBu = aBasisU[i];
                for (int j = 0; j < aNbV; ++j)
                {
                    aSum +=
                        (aBu * aBasisV[j]) * myPoles.Value(myPoles.LowerRow() + i, myPoles.LowerCol() + j).XYZ();
                }
            }
            return new Vector3D(aSum);
        }

        // For rational surfaces, delegate to the specialized methods for low orders.
        if (Nu + Nv <= 3)
        {
            if (Nu == 1 && Nv == 0)
            {
                return EvalD1(U, V).D1U;
            }
            if (Nu == 0 && Nv == 1)
            {
                return EvalD1(U, V).D1V;
            }
            if (Nu == 2 && Nv == 0)
            {
                return EvalD2(U, V).D2U;
            }
            if (Nu == 0 && Nv == 2)
            {
                return EvalD2(U, V).D2V;
            }
            if (Nu == 1 && Nv == 1)
            {
                return EvalD2(U, V).D2UV;
            }
            if (Nu == 3 && Nv == 0)
            {
                return EvalD3(U, V).D3U;
            }
            if (Nu == 0 && Nv == 3)
            {
                return EvalD3(U, V).D3V;
            }
            if (Nu == 2 && Nv == 1)
            {
                return EvalD3(U, V).D3UUV;
            }
            if (Nu == 1 && Nv == 2)
            {
                return EvalD3(U, V).D3UVV;
            }
        }

        // For higher-order rational derivatives, throw.
        throw new Geom_UndefinedDerivative(
            "GeomEval_TBezierSurface::EvalDN: rational derivatives of order > 3 not implemented");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_TBezierSurface::Transform");
    }

    public override Geom_Geometry Copy()
    {
        if (myRational)
        {
            return new GeomEval_TBezierSurface(myPoles, myWeights, myAlphaU, myAlphaV);
        }
        return new GeomEval_TBezierSurface(myPoles, myAlphaU, myAlphaV);
    }
}
