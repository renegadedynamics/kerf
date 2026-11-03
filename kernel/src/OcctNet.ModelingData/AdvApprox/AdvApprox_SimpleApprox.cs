// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_SimpleApprox.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_SimpleApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a function on one interval by a polynomial <c>P(t) = R(t) + W(t) * Q(t)</c> in the Jacobi base
/// (the Hermite interpolation of the end constraints plus a weighted least-squares part), then reduces its degree
/// to the tolerances of the subspaces.
/// </summary>
/// <remarks>
/// A working object of <see cref="AdvApprox_ApproxAFunction"/> (decision 21): <see cref="Perform"/> may be called
/// again on other intervals. The array accessors return the arrays it works in (shared: don't modify).
/// </remarks>
public class AdvApprox_SimpleApprox
{
    private readonly int myTotalNumSS;
    private readonly int myTotalDimension;
    private readonly int myNbGaussPoints;
    private readonly int myWorkDegree;
    private readonly int myNivConstr;
    private readonly JacobiPolynomial myJacPol;
    private readonly NCollection_HArray1<double> myTabPoints;
    private readonly NCollection_HArray2<double> myTabWeights;
    private readonly AdvApprox_EvaluatorFunction myEvaluator;
    private int myDegree;
    private readonly NCollection_HArray1<double> myCoeff;
    private readonly NCollection_HArray2<double> myFirstConstr;
    private readonly NCollection_HArray2<double> myLastConstr;
    private readonly NCollection_HArray1<double> mySomTab;
    private readonly NCollection_HArray1<double> myDifTab;
    private NCollection_HArray1<double>? myMaxError;
    private NCollection_HArray1<double>? myAverageError;
    private bool done;

    /// <summary>
    /// Prepares the approximation of <paramref name="Func"/>, of dimension <paramref name="TotalDimension"/>
    /// split into <paramref name="TotalNumSS"/> subspaces, with the continuity C0, C1 or C2 at the ends.
    /// </summary>
    public AdvApprox_SimpleApprox(int TotalDimension, int TotalNumSS, Continuity Continuity, int WorkDegree,
                                  int NbGaussPoints, JacobiPolynomial JacobiBase, AdvApprox_EvaluatorFunction Func)
    {
        myTotalNumSS = TotalNumSS;
        myTotalDimension = TotalDimension;
        myNbGaussPoints = NbGaussPoints;
        myWorkDegree = WorkDegree;
        myJacPol = JacobiBase;
        myEvaluator = Func;

        // the Check of input parameters

        switch (Continuity)
        {
            case C0:
                myNivConstr = 0;
                break;
            case C1:
                myNivConstr = 1;
                break;
            case C2:
                myNivConstr = 2;
                break;
            default:
                throw new ConstructionErrorException("Invalid Continuity");
        }

        int DegreeQ = myWorkDegree - 2 * (myNivConstr + 1);

        // the extraction of the Legendre roots
        myTabPoints = new NCollection_HArray1<double>(0, NbGaussPoints / 2);
        JacobiBase.Points(NbGaussPoints, myTabPoints.ChangeArray1());

        // the extraction of the Gauss Weights
        myTabWeights = new NCollection_HArray2<double>(0, NbGaussPoints / 2, 0, DegreeQ);
        JacobiBase.Weights(NbGaussPoints, myTabWeights.ChangeArray2());

        myCoeff = new NCollection_HArray1<double>(0, (myWorkDegree + 1) * myTotalDimension - 1);
        myFirstConstr = new NCollection_HArray2<double>(1, myTotalDimension, 0, myNivConstr);
        myLastConstr = new NCollection_HArray2<double>(1, myTotalDimension, 0, myNivConstr);
        mySomTab = new NCollection_HArray1<double>(0, (myNbGaussPoints / 2 + 1) * myTotalDimension - 1);
        myDifTab = new NCollection_HArray1<double>(0, (myNbGaussPoints / 2 + 1) * myTotalDimension - 1);
        done = false;
    }

    /// <summary>
    /// Constructs the approximation on [<paramref name="First"/>, <paramref name="Last"/>], with the dimensions
    /// and tolerances of the subspaces, and a degree of at most <paramref name="MaxDegree"/>.
    /// </summary>
    public void Perform(NCollection_Array1<int> LocalDimension, NCollection_Array1<double> LocalTolerancesArray,
                        double First, double Last, int MaxDegree)
    {
        // ======= the computation of Pp(t) = Rr(t) + W(t)*Qq(t) =======

        done = false;
        int i, idim, k, numss;

        int Dimension = myTotalDimension;
        AdvApprox_EvaluatorFunction Evaluator = myEvaluator;

        // ===== the computation of Rr(t) (the first part of Pp) ======

        int DegreeR = 2 * myNivConstr + 1;
        int DegreeQ = myWorkDegree - 2 * (myNivConstr + 1);

        ReadOnlySpan<double> FirstLast = [First, Last];

        math_Vector Result = new(1, myTotalDimension);
        int ErrorCode, derive, i_idim;
        double Fact = (Last - First) / 2;
        Span<double> pResult = Result.AsSpan();
        double param;

        for (param = First, derive = myNivConstr; derive >= 0; derive--)
        {
            Evaluator.Evaluate(Dimension, FirstLast, param, derive, pResult, out ErrorCode);
            if (ErrorCode != 0)
            {
                return; // Evaluation error
            }
            if (derive >= 1)
            {
                Result *= Fact;
            }
            if (derive == 2)
            {
                Result *= Fact;
            }
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                myFirstConstr.SetValue(idim, derive, Result[idim]);
            }
        }

        for (param = Last, derive = myNivConstr; derive >= 0; derive--)
        {
            Evaluator.Evaluate(Dimension, FirstLast, param, derive, pResult, out ErrorCode);
            if (ErrorCode != 0)
            {
                return; // Evaluation error
            }
            if (derive >= 1)
            {
                Result *= Fact;
            }
            if (derive == 2)
            {
                Result *= Fact;
            }
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                myLastConstr.SetValue(idim, derive, Result[idim]);
            }
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteInterpolate(myTotalDimension, -1.0, 1.0, myNivConstr, myNivConstr, myFirstConstr.Array2(),
                                myLastConstr.Array2(), myCoeff.ChangeArray1());

        // ===== the computation of the coefficients of Qq(t) (the second part of Pp) ======

        math_Vector Fti = new(1, myTotalDimension);
        math_Vector Rpti = new(1, myTotalDimension);
        math_Vector Rmti = new(1, myTotalDimension);
        Span<double> pFti = Fti.AsSpan();
        ReadOnlySpan<double> Coef1 = myCoeff.ChangeArray1().AsSpan(0);

        derive = 0;
        double ti, tip, tin, alin = (Last - First) / 2, blin = (Last + First) / 2.0;

        i_idim = myTotalDimension;
        for (i = 1; i <= myNbGaussPoints / 2; i++)
        {
            ti = myTabPoints.Value(i);
            tip = alin * ti + blin;
            Evaluator.Evaluate(Dimension, FirstLast, tip, derive, pFti, out ErrorCode);
            if (ErrorCode != 0)
            {
                return; // Evaluation error
            }
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                mySomTab.SetValue(i_idim, Fti[idim]);
                myDifTab.SetValue(i_idim, Fti[idim]);
                i_idim++;
            }
        }

        i_idim = myTotalDimension;
        for (i = 1; i <= myNbGaussPoints / 2; i++)
        {
            ti = myTabPoints.Value(i);
            tin = -alin * ti + blin;
            Evaluator.Evaluate(Dimension, FirstLast, tin, derive, pFti, out ErrorCode);
            if (ErrorCode != 0)
            {
                return; // Evaluation error
            }

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, DegreeR, myTotalDimension, Coef1, Rpti.AsSpan());
            ti = -ti;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, DegreeR, myTotalDimension, Coef1, Rmti.AsSpan());

            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                mySomTab.SetValue(i_idim, mySomTab.Value(i_idim) + Fti[idim] - Rpti[idim] - Rmti[idim]);
                myDifTab.SetValue(i_idim, myDifTab.Value(i_idim) - Fti[idim] - Rpti[idim] + Rmti[idim]);
                i_idim++;
            }
        }

        // for odd NbGaussPoints - the computation of [ F(0) - R(0) ]
        if (myNbGaussPoints % 2 == 1)
        {
            ti = myTabPoints.Value(0);
            tip = blin;
            Evaluator.Evaluate(Dimension, FirstLast, tip, derive, pFti, out ErrorCode);
            if (ErrorCode != 0)
            {
                return; // Evaluation error
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(ti, derive, DegreeR, myTotalDimension, Coef1, Rpti.AsSpan());
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                mySomTab.SetValue(idim - 1, Fti[idim] - Rpti[idim]);
                myDifTab.SetValue(idim - 1, Fti[idim] - Rpti[idim]);
            }
        }

        // the computation of Qq(t)
        double Sum = 0.0;
        for (k = 0; k <= DegreeQ; k += 2)
        {
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                Sum = 0.0;
                for (i = 1; i <= myNbGaussPoints / 2; i++)
                {
                    Sum += myTabWeights.Value(i, k) * mySomTab.Value(i * myTotalDimension + idim - 1);
                }
                myCoeff.SetValue((k + DegreeR + 1) * myTotalDimension + idim - 1, Sum);
            }
        }
        for (k = 1; k <= DegreeQ; k += 2)
        {
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                Sum = 0.0;
                for (i = 1; i <= myNbGaussPoints / 2; i++)
                {
                    Sum += myTabWeights.Value(i, k) * myDifTab.Value(i * myTotalDimension + idim - 1);
                }
                myCoeff.SetValue((k + DegreeR + 1) * myTotalDimension + idim - 1, Sum);
            }
        }
        if (myNbGaussPoints % 2 == 1)
        {
            for (idim = 1; idim <= myTotalDimension; idim++)
            {
                for (k = 0; k <= DegreeQ; k += 2)
                {
                    // OCCT: Sum still holds the last coefficient computed, not this one, so each even coefficient
                    // is overwritten by a running sum carried from coefficient to coefficient (occt-issues.md).
                    Sum += myTabWeights.Value(0, k) * mySomTab.Value(0 * myTotalDimension + idim - 1);
                    myCoeff.SetValue((k + DegreeR + 1) * myTotalDimension + idim - 1, Sum);
                }
            }
        }

        // the computing of NewDegree
        NCollection_Array1<double> JacCoeff = new(0, myTotalDimension * (myWorkDegree + 1) - 1);

        double MaxErr, AverageErr;
        int Dim, RangSS, RangCoeff, RangJacCoeff, RangDim, NewDegree, NewDegreeMax = 0;

        myMaxError = new NCollection_HArray1<double>(1, myTotalNumSS);
        myAverageError = new NCollection_HArray1<double>(1, myTotalNumSS);
        RangSS = 0;
        RangJacCoeff = 0;
        for (numss = 1; numss <= myTotalNumSS; numss++)
        {
            Dim = LocalDimension[numss];
            RangCoeff = 0;
            RangDim = 0;
            for (k = 0; k <= myWorkDegree; k++)
            {
                for (idim = 1; idim <= Dim; idim++)
                {
                    JacCoeff[RangJacCoeff + RangDim + idim - 1] = myCoeff.Value(RangCoeff + RangSS + idim - 1);
                }
                RangDim = RangDim + Dim;
                RangCoeff = RangCoeff + myTotalDimension;
            }

            ReadOnlySpan<double> JacSS = JacCoeff.AsSpan(RangJacCoeff);
            myJacPol.ReduceDegree(Dim, MaxDegree, LocalTolerancesArray[numss], JacSS, out NewDegree, out MaxErr);
            if (NewDegree > NewDegreeMax)
            {
                NewDegreeMax = NewDegree;
            }
            RangSS = RangSS + Dim;
            RangJacCoeff = RangJacCoeff + (myWorkDegree + 1) * Dim;
        }

        // the computing of MaxError and AverageError
        RangSS = 0;
        RangJacCoeff = 0;
        for (numss = 1; numss <= myTotalNumSS; numss++)
        {
            Dim = LocalDimension[numss];

            ReadOnlySpan<double> JacSS = JacCoeff.AsSpan(RangJacCoeff);
            MaxErr = myJacPol.MaxError(LocalDimension[numss], JacSS, NewDegreeMax);
            myMaxError.SetValue(numss, MaxErr);
            AverageErr = myJacPol.AverageError(LocalDimension[numss], JacSS, NewDegreeMax);
            myAverageError.SetValue(numss, AverageErr);
            RangSS = RangSS + Dim;
            RangJacCoeff = RangJacCoeff + (myWorkDegree + 1) * Dim;
        }

        myDegree = NewDegreeMax;
        done = true;
    }

    public bool IsDone() => done;

    public int Degree() => myDegree;

    /// <summary>The coefficients in the Jacobi base (shared: don't modify).</summary>
    public NCollection_HArray1<double> Coefficients() => myCoeff;

    /// <summary>The constraints at the first parameter (shared: don't modify).</summary>
    public NCollection_HArray2<double> FirstConstr() => myFirstConstr;

    /// <summary>The constraints at the last parameter (shared: don't modify).</summary>
    public NCollection_HArray2<double> LastConstr() => myLastConstr;

    /// <summary>A working array (shared: don't modify).</summary>
    public NCollection_HArray1<double> SomTab() => mySomTab;

    /// <summary>A working array (shared: don't modify).</summary>
    public NCollection_HArray1<double> DifTab() => myDifTab;

    public double MaxError(int Index) => myMaxError!.Value(Index);

    public double AverageError(int Index) => myAverageError!.Value(Index);

    /// <summary>Writes the maximum error of each subspace.</summary>
    public void Dump(TextWriter o)
    {
        int ii;
        o.Write("Dump of SimpleApprox \n");
        for (ii = 1; ii <= myTotalNumSS; ii++)
        {
            o.Write("Error   " + MaxError(ii).ToString(CultureInfo.InvariantCulture) + "\n");
        }
    }
}
