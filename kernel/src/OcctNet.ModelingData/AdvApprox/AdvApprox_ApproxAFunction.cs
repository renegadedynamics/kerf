// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.lxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a function F(t), made of 1D, 2D and 3D subspaces, by a B-spline with the same parametrisation:
/// polynomial pieces on cut intervals, each of degree at most <c>MaxDeg</c> (at most 14), joined with the
/// requested continuity, so that each subspace is within its tolerance.
/// </summary>
/// <remarks>
/// <para>
/// The dimension of F is <c>Num1DSS + 2 * Num2DSS + 3 * Num3DSS</c>, and the evaluator writes the 1D values first,
/// then the 2D ones, then the 3D ones. The approximation is done in the constructor.
/// </para>
/// <para>
/// The batch accessors (<see cref="Poles1d()"/>, <see cref="Poles2d()"/>, <see cref="Poles()"/>,
/// <see cref="Knots"/>, <see cref="Multiplicities"/>, <see cref="MaxError(int)"/>,
/// <see cref="AverageError(int)"/>) return copies, where OCCT returns its own handles, so a result can't be
/// changed through them; they return null where OCCT returns a null handle.
/// </para>
/// </remarks>
public class AdvApprox_ApproxAFunction
{
    private readonly int[] myNumSubSpaces = new int[3];
    private readonly NCollection_HArray1<double>? my1DTolerances;
    private readonly NCollection_HArray1<double>? my2DTolerances;
    private readonly NCollection_HArray1<double>? my3DTolerances;
    private readonly double myFirst;
    private readonly double myLast;
    private readonly Continuity myContinuity;
    private int myMaxDegree;
    private readonly int myMaxSegments;
    private bool myDone;
    private bool myHasResult;
    private NCollection_HArray2<double>? my1DPoles;
    private NCollection_HArray2<Point2D>? my2DPoles;
    private NCollection_HArray2<Point3D>? my3DPoles;
    private NCollection_HArray1<double>? myKnots;
    private NCollection_HArray1<int>? myMults;
    private int myDegree;
    private readonly AdvApprox_EvaluatorFunction myEvaluator;
    private NCollection_HArray1<double>? my1DMaxError;
    private NCollection_HArray1<double>? my1DAverageError;
    private NCollection_HArray1<double>? my2DMaxError;
    private NCollection_HArray1<double>? my2DAverageError;
    private NCollection_HArray1<double>? my3DMaxError;
    private NCollection_HArray1<double>? my3DAverageError;

    /// <summary>
    /// Approximates <paramref name="Func"/> on [<paramref name="First"/>, <paramref name="Last"/>], cutting
    /// intervals in two (<see cref="AdvApprox_DichoCutting"/>). The tolerance arrays hold one value per subspace
    /// (1-based) and may be null when there are no subspaces of their dimension.
    /// </summary>
    public AdvApprox_ApproxAFunction(int Num1DSS, int Num2DSS, int Num3DSS, NCollection_HArray1<double>? OneDTol,
                                     NCollection_HArray1<double>? TwoDTol, NCollection_HArray1<double>? ThreeDTol,
                                     double First, double Last, Continuity Continuity, int MaxDeg, int MaxSeg,
                                     AdvApprox_EvaluatorFunction Func)
    {
        my1DTolerances = OneDTol;
        my2DTolerances = TwoDTol;
        my3DTolerances = ThreeDTol;
        myFirst = First;
        myLast = Last;
        myContinuity = Continuity;
        myMaxDegree = MaxDeg;
        myMaxSegments = MaxSeg;
        myDone = false;
        myHasResult = false;
        myEvaluator = Func;
        AdvApprox_DichoCutting Cut = new();
        Perform(Num1DSS, Num2DSS, Num3DSS, Cut);
    }

    /// <summary>As the other constructor, cutting intervals with <paramref name="CutTool"/>.</summary>
    public AdvApprox_ApproxAFunction(int Num1DSS, int Num2DSS, int Num3DSS, NCollection_HArray1<double>? OneDTol,
                                     NCollection_HArray1<double>? TwoDTol, NCollection_HArray1<double>? ThreeDTol,
                                     double First, double Last, Continuity Continuity, int MaxDeg, int MaxSeg,
                                     AdvApprox_EvaluatorFunction Func, AdvApprox_Cutting CutTool)
    {
        my1DTolerances = OneDTol;
        my2DTolerances = TwoDTol;
        my3DTolerances = ThreeDTol;
        myFirst = First;
        myLast = Last;
        myContinuity = Continuity;
        myMaxDegree = MaxDeg;
        myMaxSegments = MaxSeg;
        myDone = false;
        myHasResult = false;
        myEvaluator = Func;
        Perform(Num1DSS, Num2DSS, Num3DSS, CutTool);
    }

    // Determine local continuities
    private static void PrepareConvert(int NumCurves, int MaxDegree, int ContinuityOrder, int Num1DSS, int Num2DSS,
                                       int Num3DSS, NCollection_Array1<int> NumCoeffPerCurve,
                                       NCollection_Array1<double> Coefficients,
                                       NCollection_Array2<double> PolynomialIntervals,
                                       NCollection_Array1<double> TrueIntervals,
                                       NCollection_Array1<double> LocalTolerance, NCollection_Array1<double> ErrorMax,
                                       NCollection_Array1<int> Continuity)
    {
        // Declaration
        bool isCi;
        int icurve, idim, iordre, ii, Dimension = Num1DSS + 2 * Num2DSS + 3 * Num3DSS,
            NbSpace = Num1DSS + Num2DSS + Num3DSS;
        double diff, moy, facteur1, facteur2, normal1, normal2, eps;
        int RealDegree = Max(MaxDegree + 1, 2 * ContinuityOrder + 2);

        Vector3D V1, V2;
        Vector2D v1, v2;

        NCollection_Array1<double> Result = new(1, 2 * (ContinuityOrder + 1) * Dimension);
        NCollection_Array1<double> Prec = new(1, NbSpace), Suivant = new(1, NbSpace);

        // Res1 and Res2 point into Result; Val1 and Val2 are offsets into them
        Span<double> Res1 = Result.AsSpan(1);
        Span<double> Res2 = Result.AsSpan((ContinuityOrder + 1) * Dimension + 1);
        int Val1, Val2;

        // Init
        Continuity.Init(0);
        if (ContinuityOrder == 0)
        {
            return;
        }

        for (icurve = 1; icurve < NumCurves; icurve++)
        {
            // Init and positioning at the node
            isCi = true;
            ReadOnlySpan<double> Coef1 =
                Coefficients.AsSpan((icurve - 1) * Dimension * RealDegree + Coefficients.Lower());
            ReadOnlySpan<double> Coef2 = Coef1[(Dimension * RealDegree)..];
            int Deg1 = NumCoeffPerCurve[NumCoeffPerCurve.Lower() + icurve - 1] - 1;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(PolynomialIntervals[icurve, 2], ContinuityOrder, Deg1, Dimension, Coef1, Res1);
            int Deg2 = NumCoeffPerCurve[NumCoeffPerCurve.Lower() + icurve] - 1;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(PolynomialIntervals[icurve + 1, 1], ContinuityOrder, Deg2, Dimension, Coef2, Res2);

            // Check in each subspace.
            for (iordre = 1; iordre <= ContinuityOrder && isCi; iordre++)
            {
                // fixing a bug PRO18577

                double Toler = 1.0e-5;

                double f1_dividend = PolynomialIntervals[icurve, 2] - PolynomialIntervals[icurve, 1];
                double f2_dividend = PolynomialIntervals[icurve + 1, 2] - PolynomialIntervals[icurve + 1, 1];
                double f1_divizor = TrueIntervals[icurve + 1] - TrueIntervals[icurve];
                double f2_divizor = TrueIntervals[icurve + 2] - TrueIntervals[icurve + 1];
                double fract1, fract2;

                if (Math.Abs(f1_divizor) < Toler)
                { // this is to avoid divizion by zero
                    // in this case fract1 =  5.14755758946803e-85
                    facteur1 = 0.0;
                }
                else
                {
                    fract1 = f1_dividend / f1_divizor;
                    facteur1 = Math.Pow(fract1, iordre);
                }
                if (Math.Abs(f2_divizor) < Toler)
                {
                    // in this case fract2 =  6.77193633669143e-313
                    facteur2 = 0.0;
                }
                else
                {
                    fract2 = f2_dividend / f2_divizor;
                    facteur2 = Math.Pow(fract2, iordre);
                }
                normal1 = Math.Pow(f1_divizor, iordre);
                normal2 = Math.Pow(f2_divizor, iordre);

                idim = 1;
                Val1 = iordre * Dimension - 1;
                Val2 = iordre * Dimension - 1;

                for (ii = 1; ii <= Num1DSS && isCi; ii++, idim++)
                {
                    eps = LocalTolerance[idim] * 0.01;
                    diff = Math.Abs(Res1[Val1 + ii] * facteur1 - Res2[Val2 + ii] * facteur2);
                    moy = Math.Abs(Res1[Val1 + ii] * facteur1 + Res2[Val2 + ii] * facteur2);
                    // A first check on the relative value
                    if (diff > moy * 1.0e-9)
                    {
                        Prec[idim] = diff * normal1;
                        Suivant[idim] = diff * normal2;
                        // And a second check on the upper bound of the generated error
                        if (Prec[idim] > eps || Suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        Prec[idim] = 0;
                        Suivant[idim] = 0;
                    }
                }

                Val1 += Num1DSS;
                Val2 += Num1DSS;
                for (ii = 1; ii <= Num2DSS && isCi; ii++, idim++, Val1 += 2, Val2 += 2)
                {
                    eps = LocalTolerance[idim] * 0.01;
                    v1 = new Vector2D(Res1[Val1 + 1], Res1[Val1 + 2]);
                    v2 = new Vector2D(Res2[Val2 + 1], Res2[Val2 + 2]);
                    v1 *= facteur1;
                    v2 *= facteur2;
                    diff = Math.Abs(v1.X() - v2.X()) + Math.Abs(v1.Y() - v2.Y());
                    moy = Math.Abs(v1.X() + v2.X()) + Math.Abs(v1.Y() + v2.Y());
                    if (diff > moy * 1.0e-9)
                    {
                        Prec[idim] = diff * normal1;
                        Suivant[idim] = diff * normal2;
                        if (Prec[idim] > eps || Suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        Prec[idim] = 0;
                        Suivant[idim] = 0;
                    }
                }

                for (ii = 1; ii <= Num3DSS && isCi; ii++, idim++, Val1 += 3, Val2 += 3)
                {
                    eps = LocalTolerance[idim] * 0.01;
                    V1 = new Vector3D(Res1[Val1 + 1], Res1[Val1 + 2], Res1[Val1 + 3]);
                    V2 = new Vector3D(Res2[Val2 + 1], Res2[Val2 + 2], Res2[Val2 + 3]);
                    V1 *= facteur1;
                    V2 *= facteur2;
                    diff = Math.Abs(V1.X() - V2.X()) + Math.Abs(V1.Y() - V2.Y()) + Math.Abs(V1.Z() - V2.Z());
                    moy = Math.Abs(V1.X() + V2.X()) + Math.Abs(V1.Y() + V2.Y()) + Math.Abs(V1.Z() + V2.Z());
                    if (diff > moy * 1.0e-9)
                    {
                        Prec[idim] = diff * normal1;
                        Suivant[idim] = diff * normal2;
                        if (Prec[idim] > eps || Suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        Prec[idim] = 0;
                        Suivant[idim] = 0;
                    }
                }
                // If it's good, update everything
                if (isCi)
                {
                    Continuity[icurve + 1] = iordre;
                    int index = (icurve - 1) * NbSpace + 1;
                    for (ii = index, idim = 1; idim <= NbSpace; ii++, idim++)
                    {
                        ErrorMax[ii] += Prec[idim];
                        ErrorMax[ii + NbSpace] += Suivant[idim];
                    }
                }
            }
        }
    }

    /// <summary>
    /// Approximates a function (of dimension <paramref name="TotalDimension"/>, in
    /// <paramref name="TotalNumSS"/> subspaces of dimensions <paramref name="LocalDimension"/>) on
    /// [<paramref name="First"/>, <paramref name="Last"/>] by at most <paramref name="MaxSegments"/> polynomial
    /// curves, with the least squares method, keeping the parametrisation.
    /// </summary>
    /// <remarks>
    /// <para>
    /// <paramref name="ContinuityOrder"/> is the order of the constraints at the ends of each polynomial (0, 1 or
    /// 2). <paramref name="NumMaxCoeffs"/> limits the number of coefficients of each polynomial (at least
    /// 2 * ContinuityOrder + 2). <paramref name="code_precis"/> chooses the discretisation (1 fast with average
    /// precision, 2 fast with better precision, 3 slower with good precision, 4 slow with the best precision).
    /// </para>
    /// <para>
    /// Outputs: the number of curves; the number of coefficients of each; the coefficients of curve i in
    /// <paramref name="CoefficientArray"/> from (i - 1) * TotalDimension * NumMaxCoeffs + 1, by degree then
    /// dimension; the NumCurves + 1 bounds of the intervals in <paramref name="IntervalsArray"/> (from index 1);
    /// the maximum and average errors per subspace and curve. <paramref name="ErrorCode"/> is 0 when everything
    /// is OK, -1 when an error exceeds its tolerance (the result is the best possible), 1 for inconsistent input.
    /// </para>
    /// </remarks>
    public static void Approximation(int TotalDimension, int TotalNumSS, NCollection_Array1<int> LocalDimension,
                                     double First, double Last, AdvApprox_EvaluatorFunction Evaluator,
                                     AdvApprox_Cutting CutTool, int ContinuityOrder, int NumMaxCoeffs,
                                     int MaxSegments, NCollection_Array1<double> TolerancesArray, int code_precis,
                                     out int NumCurves, NCollection_Array1<int> NumCoeffPerCurveArray,
                                     NCollection_Array1<double> CoefficientArray,
                                     NCollection_Array1<double> IntervalsArray, NCollection_Array1<double> ErrorMaxArray,
                                     NCollection_Array1<double> AverageErrorArray, out int ErrorCode)
    {
        //  double EpsPar =  Precision::Confusion();
        int NUPIL, TheDeg;
        bool isCut = false;

        // Definition of C arrays
        Span<double> TABINT = IntervalsArray.AsSpan(1);

        NumCurves = 0;
        ErrorCode = 0;
        CoefficientArray.Init(0);

        //-------------------------- Input validation ------------------

        if ((MaxSegments < 1) || (Math.Abs(Last - First) < 1.0e-9))
        {
            ErrorCode = 1;
            return;
        }

        //--> The total dimension must be the sum of the subspace dimensions
        int IDIM = 0;
        for (int I = 1; I <= TotalNumSS; I++)
        {
            IDIM += LocalDimension[I];
        }
        if (IDIM != TotalDimension)
        {
            ErrorCode = 1;
            return;
        }
        Continuity Continuity = C0;
        switch (ContinuityOrder)
        {
            case 0:
                Continuity = C0;
                break;
            case 1:
                Continuity = C1;
                break;
            case 2:
                Continuity = C2;
                break;
            default:
                throw new ConstructionErrorException();
        }

        //--------------------- Choice of number of points ----------------------
        //---------- and smoothing degree in the orthogonal basis -----------
        //-->  NDJAC is the "working" degree in the orthogonal basis.

        int NbGaussPoints, WorkDegree;

        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(Continuity, NumMaxCoeffs - 1, code_precis, out NbGaussPoints, out WorkDegree);
        //      NDJAC=WorkDegree;

        //------------------ Initialization of cutting management ---------

        TABINT[0] = First;
        TABINT[1] = Last;
        NUPIL = 1;
        NumCurves = 0;

        // ********************************************************************
        //                      APPROXIMATION WITH CUTTING
        // ********************************************************************
        JacobiPolynomial JacobiBase = new(WorkDegree, Continuity);
        // HP port: the compiler refuses branching
        int IS;
        bool goto_fin_de_boucle;
        int MaxDegree = NumMaxCoeffs - 1;
        AdvApprox_SimpleApprox Approx = new(TotalDimension, TotalNumSS, Continuity, WorkDegree, NbGaussPoints,
                                            JacobiBase, Evaluator);
        while ((NUPIL - NumCurves) != 0)
        {
            //--> When the top of the stack is reached, we are done!

            // HP port: the compiler refuses branching
            goto_fin_de_boucle = false;

            //---- Compute the approximation curve in the Jacobi basis -----

            Approx.Perform(LocalDimension, TolerancesArray, TABINT[NumCurves], TABINT[NumCurves + 1], MaxDegree);
            if (!Approx.IsDone())
            {
                ErrorCode = 1;
                return;
            }

            //---------- Compute the curve degree and max error ----------

            NumCoeffPerCurveArray[NumCurves + 1] = 0;

            //    The error must be satisfied on all subspaces, otherwise we split

            bool MaxMaxErr = true;
            for (IS = 1; IS <= TotalNumSS; IS++)
            {
                if (Approx.MaxError(IS) > TolerancesArray[IS])
                {
                    MaxMaxErr = false;
                    break;
                }
            }

            if (MaxMaxErr)
            {
                NumCurves++;
            }
            else
            {
                //-> ...otherwise try to split the current interval in 2...
                double TMIL;
                bool Large;

                Large = CutTool.Value(TABINT[NumCurves], TABINT[NumCurves + 1], out TMIL);

                if (NUPIL < MaxSegments && Large)
                {
                    isCut = true; // Now we know it!
                    int IDEB = NumCurves + 1;
                    int IDEB1 = IDEB + 1;
                    int ILONG = NUPIL - NumCurves - 1;
                    for (int iI = ILONG; iI >= 0; iI--)
                    {
                        TABINT[IDEB1 + iI] = TABINT[IDEB + iI];
                    }
                    TABINT[IDEB] = TMIL;
                    NUPIL++;
                    //--> ... and start over.
                    // HP port: the compiler refuses branching
                    goto_fin_de_boucle = true;
                }
                else
                {
                    //--> If the stack is full...
                    // for now               ErrorCode=-1;
                    NumCurves++;
                }
            }
            // HP port: the compiler refuses branching
            if (goto_fin_de_boucle)
            {
                continue;
            }
            for (IS = 1; IS <= TotalNumSS; IS++)
            {
                ErrorMaxArray.SetValue(IS + (NumCurves - 1) * TotalNumSS, Approx.MaxError(IS));
                //---> ...and compute the average error.
                AverageErrorArray.SetValue(IS + (NumCurves - 1) * TotalNumSS, Approx.AverageError(IS));
            }

            NCollection_HArray1<double> HJacCoeff = Approx.Coefficients();
            TheDeg = Approx.Degree();
            if (isCut && (TheDeg < 2 * ContinuityOrder + 1))
            {
                // To avoid noisy derivatives at the ends, and maintain correct
                // continuity on the resulting BSpline.
                TheDeg = 2 * ContinuityOrder + 1;
            }

            NumCoeffPerCurveArray[NumCurves] = TheDeg + 1;
            NCollection_Array1<double> Coefficients = new(0, (TheDeg + 1) * TotalDimension - 1);
            JacobiBase.ToCoefficients(TotalDimension, TheDeg, HJacCoeff.Array1(), Coefficients);
            int i, j, f = (TheDeg + 1) * TotalDimension;
            for (i = 0, j = (NumCurves - 1) * TotalDimension * NumMaxCoeffs + 1; i < f; i++, j++)
            {
                CoefficientArray.SetValue(j, Coefficients.Value(i));
            }
        }
    }

    private void Perform(int Num1DSS, int Num2DSS, int Num3DSS, AdvApprox_Cutting CutTool)
    {
        if (Num1DSS < 0 || Num2DSS < 0 || Num3DSS < 0 || Num1DSS + Num2DSS + Num3DSS <= 0 || myLast < myFirst
            || myMaxDegree < 1 || myMaxSegments < 0)
        {
            throw new ConstructionErrorException();
        }
        if (myMaxDegree > 14)
        {
            myMaxDegree = 14;
        }
        //
        // Input
        //
        myNumSubSpaces[0] = Num1DSS;
        myNumSubSpaces[1] = Num2DSS;
        myNumSubSpaces[2] = Num3DSS;
        int TotalNumSS = Num1DSS + Num2DSS + Num3DSS, ii, jj, kk, index, dim_index, local_index;
        int TotalDimension = myNumSubSpaces[0] + 2 * myNumSubSpaces[1] + 3 * myNumSubSpaces[2];
        double error_value;

        int ContinuityOrder = 0;
        switch (myContinuity)
        {
            case C0:
                ContinuityOrder = 0;
                break;
            case C1:
                ContinuityOrder = 1;
                break;
            case C2:
                ContinuityOrder = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        Span<double> ApproxStartEnd = stackalloc double[2];
        int NumMaxCoeffs = Max(myMaxDegree + 1, 2 * ContinuityOrder + 2);
        myMaxDegree = NumMaxCoeffs - 1;
        int code_precis = 1;
        //
        //  WARNING : the polynomial coefficients are the
        //  taylor expansion of the polynomial at 0.0e0 !
        //
        ApproxStartEnd[0] = -1.0e0;
        ApproxStartEnd[1] = 1.0e0;

        NCollection_Array1<int> LocalDimension = new(1, TotalNumSS);

        index = 1;
        NCollection_Array1<double> LocalTolerances = new(1, TotalNumSS);

        for (jj = 1; jj <= myNumSubSpaces[0]; jj++)
        {
            LocalTolerances.SetValue(index, my1DTolerances!.Value(jj));
            LocalDimension.SetValue(index, 1);
            index += 1;
        }
        for (jj = 1; jj <= myNumSubSpaces[1]; jj++)
        {
            LocalTolerances.SetValue(index, my2DTolerances!.Value(jj));
            LocalDimension.SetValue(index, 2);
            index += 1;
        }
        for (jj = 1; jj <= myNumSubSpaces[2]; jj++)
        {
            LocalTolerances.SetValue(index, my3DTolerances!.Value(jj));
            LocalDimension.SetValue(index, 3);
            index += 1;
        }
        //
        // Output
        //

        int ErrorCode = 0, NumCurves, size = myMaxSegments * NumMaxCoeffs * TotalDimension;
        NCollection_HArray1<int> NumCoeffPerCurvePtr = new(1, myMaxSegments);

        NCollection_HArray1<double> LocalCoefficientsPtr = new(1, size);

        NCollection_HArray1<double> IntervalsPtr = new(1, myMaxSegments + 1);

        NCollection_Array1<double> ErrorMax = new(1, myMaxSegments * TotalNumSS);

        NCollection_Array1<double> AverageError = new(1, myMaxSegments * TotalNumSS);

        Approximation(TotalDimension, TotalNumSS, LocalDimension, myFirst, myLast, myEvaluator, CutTool,
                      ContinuityOrder, NumMaxCoeffs, myMaxSegments, LocalTolerances, code_precis,
                      out NumCurves,                        // Number of output curves
                      NumCoeffPerCurvePtr.ChangeArray1(),   // Number of coefficients per curve
                      LocalCoefficientsPtr.ChangeArray1(),  // Solution coefficients
                      IntervalsPtr.ChangeArray1(),          // Table of cutting points
                      ErrorMax,                             // Maximum error bound
                      AverageError,                         // Observed average error
                      out ErrorCode);

        if (ErrorCode == 0 || ErrorCode == -1)
        {
            //
            // Everything is OK, or we have a result where one of the max errors
            // is larger than the requested tolerance

            NCollection_Array1<int> TabContinuity = new(1, NumCurves);
            NCollection_Array2<double> PolynomialIntervalsPtr = new(1, NumCurves, 1, 2);
            for (ii = PolynomialIntervalsPtr.LowerRow(); ii <= PolynomialIntervalsPtr.UpperRow(); ii++)
            {
                // Force a minimum degree of 1 (PRO5474)
                NumCoeffPerCurvePtr[ii] = Max(2, NumCoeffPerCurvePtr.Value(ii));
                PolynomialIntervalsPtr.SetValue(ii, 1, ApproxStartEnd[0]);
                PolynomialIntervalsPtr.SetValue(ii, 2, ApproxStartEnd[1]);
            }

            PrepareConvert(NumCurves, myMaxDegree, ContinuityOrder, Num1DSS, Num2DSS, Num3DSS,
                           NumCoeffPerCurvePtr.Array1(), LocalCoefficientsPtr.ChangeArray1(), PolynomialIntervalsPtr,
                           IntervalsPtr.Array1(), LocalTolerances, ErrorMax, TabContinuity);

            CompPolynomialToPoles AConverter = new(NumCurves, TotalDimension, myMaxDegree, TabContinuity,
                                                           NumCoeffPerCurvePtr.Array1(), LocalCoefficientsPtr.Array1(),
                                                           PolynomialIntervalsPtr, IntervalsPtr.Array1());

            if (AConverter.IsDone)
            {
                // Convert's accessors return copies (decision 88): take each once
                NCollection_Array2<double> aPoles = AConverter.Poles();
                myKnots = new NCollection_HArray1<double>(AConverter.Knots());
                myMults = new NCollection_HArray1<int>(AConverter.Multiplicities());
                myDegree = AConverter.Degree();
                index = 0;
                if (myNumSubSpaces[0] > 0)
                {
                    my1DPoles = new NCollection_HArray2<double>(1, aPoles.ColLength(), 1, myNumSubSpaces[0]);
                    my1DMaxError = new NCollection_HArray1<double>(1, myNumSubSpaces[0]);
                    my1DAverageError = new NCollection_HArray1<double>(1, myNumSubSpaces[0]);
                    for (ii = 1; ii <= aPoles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= myNumSubSpaces[0]; jj++)
                        {
                            my1DPoles.SetValue(ii, jj, aPoles.Value(ii, jj));
                        }
                    }

                    for (jj = 1; jj <= myNumSubSpaces[0]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS;
                            error_value = Max(ErrorMax[local_index + jj], error_value);
                        }
                        my1DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= myNumSubSpaces[0]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS;
                            error_value += AverageError[local_index + jj];
                        }
                        error_value /= (double)NumCurves;
                        my1DAverageError.SetValue(jj, error_value);
                    }
                    index += myNumSubSpaces[0];
                }

                dim_index = index; // For the case where there is no 2D

                if (myNumSubSpaces[1] > 0)
                {
                    my2DPoles = new NCollection_HArray2<Point2D>(1, aPoles.ColLength(), 1, myNumSubSpaces[1]);
                    my2DMaxError = new NCollection_HArray1<double>(1, myNumSubSpaces[1]);
                    my2DAverageError = new NCollection_HArray1<double>(1, myNumSubSpaces[1]);
                    Span<double> Point2d = stackalloc double[2];
                    for (ii = 1; ii <= aPoles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= myNumSubSpaces[1]; jj++)
                        {
                            local_index = index + (jj - 1) * 2;
                            for (kk = 1; kk <= 2; kk++)
                            {
                                Point2d[kk - 1] = aPoles.Value(ii, local_index + kk);
                            }
                            my2DPoles.SetValue(ii, jj, new Point2D(Point2d[0], Point2d[1]));
                        }
                    }

                    for (jj = 1; jj <= myNumSubSpaces[1]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS + index;
                            error_value = Max(ErrorMax[local_index + jj], error_value);
                        }
                        my2DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= myNumSubSpaces[1]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS + index;
                            error_value += AverageError[local_index + jj];
                        }
                        error_value /= (double)NumCurves;
                        my2DAverageError.SetValue(jj, error_value);
                    }
                    index += myNumSubSpaces[1];
                    // For poles, the offset must be doubled:
                    dim_index = index + myNumSubSpaces[1];
                }

                if (myNumSubSpaces[2] > 0)
                {
                    my3DPoles = new NCollection_HArray2<Point3D>(1, aPoles.ColLength(), 1, myNumSubSpaces[2]);
                    my3DMaxError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
                    my3DAverageError = new NCollection_HArray1<double>(1, myNumSubSpaces[2]);
                    Span<double> Point = stackalloc double[3];
                    for (ii = 1; ii <= aPoles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= myNumSubSpaces[2]; jj++)
                        {
                            local_index = dim_index + (jj - 1) * 3;
                            for (kk = 1; kk <= 3; kk++)
                            {
                                Point[kk - 1] = aPoles.Value(ii, local_index + kk);
                            }
                            my3DPoles.SetValue(ii, jj, new Point3D(Point[0], Point[1], Point[2]));
                        }
                    }

                    for (jj = 1; jj <= myNumSubSpaces[2]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS + index;
                            error_value = Max(ErrorMax[local_index + jj], error_value);
                        }
                        my3DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= myNumSubSpaces[2]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= NumCurves; ii++)
                        {
                            local_index = (ii - 1) * TotalNumSS + index;
                            error_value += AverageError[local_index + jj];
                        }
                        error_value /= (double)NumCurves;
                        my3DAverageError.SetValue(jj, error_value);
                    }
                }
                if (ErrorCode == 0)
                {
                    myDone = true;
                    myHasResult = true;
                }
                else if (ErrorCode == -1)
                {
                    myHasResult = true;
                }
            }
        }
    }

    /// <summary>True if every subspace is within its tolerance.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if there is a result, even one outside the tolerances.</summary>
    public bool HasResult() => myHasResult;

    /// <summary>A copy of the poles of the 1D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<double>? Poles1d() => my1DPoles is null ? null : new(my1DPoles);

    /// <summary>A copy of the poles of the 2D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<Point2D>? Poles2d() => my2DPoles is null ? null : new(my2DPoles);

    /// <summary>A copy of the poles of the 3D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<Point3D>? Poles() => my3DPoles is null ? null : new(my3DPoles);

    /// <summary>The number of poles of the B-splines (0 without a result).</summary>
    public int NbPoles()
    {
        if (myDone || myHasResult)
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(myDegree, false, myMults!.Array1());
        }
        return 0;
    }

    /// <summary>Copies the poles of 1D subspace <paramref name="Index"/> into <paramref name="P"/>.</summary>
    public void Poles1d(int Index, NCollection_Array1<double> P)
    {
        int ii;
        for (ii = P.Lower(); ii <= P.Upper(); ii++)
        {
            P.SetValue(ii, my1DPoles!.Value(ii, Index));
        }
    }

    /// <summary>Copies the poles of 2D subspace <paramref name="Index"/> into <paramref name="P"/>.</summary>
    public void Poles2d(int Index, NCollection_Array1<Point2D> P)
    {
        int ii;
        for (ii = P.Lower(); ii <= P.Upper(); ii++)
        {
            P.SetValue(ii, my2DPoles!.Value(ii, Index));
        }
    }

    /// <summary>Copies the poles of 3D subspace <paramref name="Index"/> into <paramref name="P"/>.</summary>
    public void Poles(int Index, NCollection_Array1<Point3D> P)
    {
        int ii;
        for (ii = P.Lower(); ii <= P.Upper(); ii++)
        {
            P.SetValue(ii, my3DPoles!.Value(ii, Index));
        }
    }

    public int Degree() => myDegree;

    public int NbKnots() => myKnots!.Length();

    /// <summary>The number of subspaces of dimension <paramref name="Dimension"/> (1, 2 or 3).</summary>
    public int NumSubSpaces(int Dimension)
    {
        if (Dimension < 1 || Dimension > 3)
        {
            throw new ConstructionErrorException();
        }
        return myNumSubSpaces[Dimension - 1];
    }

    /// <summary>A copy of the knots, or null.</summary>
    public NCollection_HArray1<double>? Knots() => myKnots is null ? null : new(myKnots);

    /// <summary>A copy of the multiplicities, or null.</summary>
    public NCollection_HArray1<int>? Multiplicities() => myMults is null ? null : new(myMults);

    /// <summary>A copy of the maximum errors of the subspaces of dimension <paramref name="Dimension"/>, or null.</summary>
    public NCollection_HArray1<double>? MaxError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = MaxErrorArray(Dimension);
        return EPtr is null ? null : new(EPtr);
    }

    /// <summary>A copy of the average errors of the subspaces of dimension <paramref name="Dimension"/>, or null.</summary>
    public NCollection_HArray1<double>? AverageError(int Dimension)
    {
        NCollection_HArray1<double>? EPtr = AverageErrorArray(Dimension);
        return EPtr is null ? null : new(EPtr);
    }

    /// <summary>The maximum error of subspace <paramref name="Index"/> of dimension <paramref name="Dimension"/>.</summary>
    public double MaxError(int Dimension, int Index) => MaxErrorArray(Dimension)!.Value(Index);

    /// <summary>The average error of subspace <paramref name="Index"/> of dimension <paramref name="Dimension"/>.</summary>
    public double AverageError(int Dimension, int Index) => AverageErrorArray(Dimension)!.Value(Index);

    // OCCT's MaxError(D) body, returning the field; the public form copies it
    private NCollection_HArray1<double>? MaxErrorArray(int D)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (D <= 0 || D > 3)
        {
            throw new OutOfRangeException();
        }
        switch (D)
        {
            case 1:
                EPtr = my1DMaxError;
                break;
            case 2:
                EPtr = my2DMaxError;
                break;
            case 3:
                EPtr = my3DMaxError;
                break;
        }
        return EPtr;
    }

    // OCCT's AverageError(D) body, returning the field; the public form copies it
    private NCollection_HArray1<double>? AverageErrorArray(int D)
    {
        NCollection_HArray1<double>? EPtr = null;
        if (D <= 0 || D > 3)
        {
            throw new OutOfRangeException();
        }
        switch (D)
        {
            case 1:
                EPtr = my1DAverageError;
                break;
            case 2:
                EPtr = my2DAverageError;
                break;
            case 3:
                EPtr = my3DAverageError;
                break;
        }
        return EPtr;
    }

    /// <summary>Writes the maximum error of each subspace.</summary>
    public void Dump(TextWriter o)
    {
        int ii;
        o.Write("Dump of ApproxAFunction\n");
        if (myNumSubSpaces[0] > 0)
        {
            o.Write("Error(s) 1d = \n");
            for (ii = 1; ii <= myNumSubSpaces[0]; ii++)
            {
                o.Write("   " + MaxError(1, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }

        if (myNumSubSpaces[1] > 0)
        {
            o.Write("Error(s) 2d = \n");
            for (ii = 1; ii <= myNumSubSpaces[1]; ii++)
            {
                o.Write("   " + MaxError(2, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }

        if (myNumSubSpaces[2] > 0)
        {
            o.Write("Error(s) 3d = \n");
            for (ii = 1; ii <= myNumSubSpaces[2]; ii++)
            {
                o.Write("   " + MaxError(3, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }
}
