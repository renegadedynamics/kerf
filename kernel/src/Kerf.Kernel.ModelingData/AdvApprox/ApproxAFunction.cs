// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.lxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_ApproxAFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation.Advanced;

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
internal class ApproxAFunction
{
    private readonly int[] _numSubSpaces = new int[3];
    private readonly NCollection_HArray1<double>? _my1DTolerances;
    private readonly NCollection_HArray1<double>? _my2DTolerances;
    private readonly NCollection_HArray1<double>? _my3DTolerances;
    private readonly double _first;
    private readonly double _last;
    private readonly Continuity _continuity;
    private int _maxDegree;
    private readonly int _maxSegments;
    private bool _done;
    private bool _hasResult;
    private NCollection_HArray2<double>? _my1DPoles;
    private NCollection_HArray2<Point2D>? _my2DPoles;
    private NCollection_HArray2<Point3D>? _my3DPoles;
    private NCollection_HArray1<double>? _knots;
    private NCollection_HArray1<int>? _mults;
    private int _degree;
    private readonly EvaluatorFunction _evaluator;
    private NCollection_HArray1<double>? _my1DMaxError;
    private NCollection_HArray1<double>? _my1DAverageError;
    private NCollection_HArray1<double>? _my2DMaxError;
    private NCollection_HArray1<double>? _my2DAverageError;
    private NCollection_HArray1<double>? _my3DMaxError;
    private NCollection_HArray1<double>? _my3DAverageError;

    /// <summary>
    /// Approximates <paramref name="func"/> on [<paramref name="first"/>, <paramref name="last"/>], cutting
    /// intervals in two (<see cref="DichoCutting"/>). The tolerance arrays hold one value per subspace
    /// (1-based) and may be null when there are no subspaces of their dimension.
    /// </summary>
    public ApproxAFunction(int num1DSS, int num2DSS, int num3DSS, NCollection_HArray1<double>? oneDTol,
                                     NCollection_HArray1<double>? twoDTol, NCollection_HArray1<double>? threeDTol,
                                     double first, double last, Continuity continuity, int maxDeg, int maxSeg,
                                     EvaluatorFunction func)
    {
        _my1DTolerances = oneDTol;
        _my2DTolerances = twoDTol;
        _my3DTolerances = threeDTol;
        _first = first;
        _last = last;
        _continuity = continuity;
        _maxDegree = maxDeg;
        _maxSegments = maxSeg;
        _done = false;
        _hasResult = false;
        _evaluator = func;
        DichoCutting cut = new();
        Perform(num1DSS, num2DSS, num3DSS, cut);
    }

    /// <summary>As the other constructor, cutting intervals with <paramref name="cutTool"/>.</summary>
    public ApproxAFunction(int num1DSS, int num2DSS, int num3DSS, NCollection_HArray1<double>? oneDTol,
                                     NCollection_HArray1<double>? twoDTol, NCollection_HArray1<double>? threeDTol,
                                     double first, double last, Continuity continuity, int maxDeg, int maxSeg,
                                     EvaluatorFunction func, Cutting cutTool)
    {
        _my1DTolerances = oneDTol;
        _my2DTolerances = twoDTol;
        _my3DTolerances = threeDTol;
        _first = first;
        _last = last;
        _continuity = continuity;
        _maxDegree = maxDeg;
        _maxSegments = maxSeg;
        _done = false;
        _hasResult = false;
        _evaluator = func;
        Perform(num1DSS, num2DSS, num3DSS, cutTool);
    }

    // Determine local continuities
    private static void PrepareConvert(int numCurves, int maxDegree, int continuityOrder, int num1DSS, int num2DSS,
                                       int num3DSS, NCollection_Array1<int> numCoeffPerCurve,
                                       NCollection_Array1<double> coefficients,
                                       NCollection_Array2<double> polynomialIntervals,
                                       NCollection_Array1<double> trueIntervals,
                                       NCollection_Array1<double> localTolerance, NCollection_Array1<double> errorMax,
                                       NCollection_Array1<int> continuity)
    {
        // Declaration
        bool isCi;
        int icurve, idim, iordre, ii, dimension = num1DSS + 2 * num2DSS + 3 * num3DSS,
            nbSpace = num1DSS + num2DSS + num3DSS;
        double diff, moy, facteur1, facteur2, normal1, normal2, eps;
        int realDegree = Max(maxDegree + 1, 2 * continuityOrder + 2);

        Vector3D V1, V2;
        Vector2D v1, v2;

        NCollection_Array1<double> result = new(1, 2 * (continuityOrder + 1) * dimension);
        NCollection_Array1<double> prec = new(1, nbSpace), suivant = new(1, nbSpace);

        // Res1 and Res2 point into Result; Val1 and Val2 are offsets into them
        Span<double> res1 = result.AsSpan(1);
        Span<double> res2 = result.AsSpan((continuityOrder + 1) * dimension + 1);
        int val1, val2;

        // Init
        continuity.Init(0);
        if (continuityOrder == 0)
        {
            return;
        }

        for (icurve = 1; icurve < numCurves; icurve++)
        {
            // Init and positioning at the node
            isCi = true;
            ReadOnlySpan<double> coef1 =
                coefficients.AsSpan((icurve - 1) * dimension * realDegree + coefficients.Lower());
            ReadOnlySpan<double> coef2 = coef1[(dimension * realDegree)..];
            int deg1 = numCoeffPerCurve[numCoeffPerCurve.Lower() + icurve - 1] - 1;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(polynomialIntervals[icurve, 2], continuityOrder, deg1, dimension, coef1, res1);
            int deg2 = numCoeffPerCurve[numCoeffPerCurve.Lower() + icurve] - 1;
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(polynomialIntervals[icurve + 1, 1], continuityOrder, deg2, dimension, coef2, res2);

            // Check in each subspace.
            for (iordre = 1; iordre <= continuityOrder && isCi; iordre++)
            {
                // fixing a bug PRO18577

                double toler = 1.0e-5;

                double f1_dividend = polynomialIntervals[icurve, 2] - polynomialIntervals[icurve, 1];
                double f2_dividend = polynomialIntervals[icurve + 1, 2] - polynomialIntervals[icurve + 1, 1];
                double f1_divizor = trueIntervals[icurve + 1] - trueIntervals[icurve];
                double f2_divizor = trueIntervals[icurve + 2] - trueIntervals[icurve + 1];
                double fract1, fract2;

                if (Math.Abs(f1_divizor) < toler)
                { // this is to avoid divizion by zero
                    // in this case fract1 =  5.14755758946803e-85
                    facteur1 = 0.0;
                }
                else
                {
                    fract1 = f1_dividend / f1_divizor;
                    facteur1 = Math.Pow(fract1, iordre);
                }
                if (Math.Abs(f2_divizor) < toler)
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
                val1 = iordre * dimension - 1;
                val2 = iordre * dimension - 1;

                for (ii = 1; ii <= num1DSS && isCi; ii++, idim++)
                {
                    eps = localTolerance[idim] * 0.01;
                    diff = Math.Abs(res1[val1 + ii] * facteur1 - res2[val2 + ii] * facteur2);
                    moy = Math.Abs(res1[val1 + ii] * facteur1 + res2[val2 + ii] * facteur2);
                    // A first check on the relative value
                    if (diff > moy * 1.0e-9)
                    {
                        prec[idim] = diff * normal1;
                        suivant[idim] = diff * normal2;
                        // And a second check on the upper bound of the generated error
                        if (prec[idim] > eps || suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        prec[idim] = 0;
                        suivant[idim] = 0;
                    }
                }

                val1 += num1DSS;
                val2 += num1DSS;
                for (ii = 1; ii <= num2DSS && isCi; ii++, idim++, val1 += 2, val2 += 2)
                {
                    eps = localTolerance[idim] * 0.01;
                    v1 = new Vector2D(res1[val1 + 1], res1[val1 + 2]);
                    v2 = new Vector2D(res2[val2 + 1], res2[val2 + 2]);
                    v1 *= facteur1;
                    v2 *= facteur2;
                    diff = Math.Abs(v1.X - v2.X) + Math.Abs(v1.Y - v2.Y);
                    moy = Math.Abs(v1.X + v2.X) + Math.Abs(v1.Y + v2.Y);
                    if (diff > moy * 1.0e-9)
                    {
                        prec[idim] = diff * normal1;
                        suivant[idim] = diff * normal2;
                        if (prec[idim] > eps || suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        prec[idim] = 0;
                        suivant[idim] = 0;
                    }
                }

                for (ii = 1; ii <= num3DSS && isCi; ii++, idim++, val1 += 3, val2 += 3)
                {
                    eps = localTolerance[idim] * 0.01;
                    V1 = new Vector3D(res1[val1 + 1], res1[val1 + 2], res1[val1 + 3]);
                    V2 = new Vector3D(res2[val2 + 1], res2[val2 + 2], res2[val2 + 3]);
                    V1 *= facteur1;
                    V2 *= facteur2;
                    diff = Math.Abs(V1.X - V2.X) + Math.Abs(V1.Y - V2.Y) + Math.Abs(V1.Z - V2.Z);
                    moy = Math.Abs(V1.X + V2.X) + Math.Abs(V1.Y + V2.Y) + Math.Abs(V1.Z + V2.Z);
                    if (diff > moy * 1.0e-9)
                    {
                        prec[idim] = diff * normal1;
                        suivant[idim] = diff * normal2;
                        if (prec[idim] > eps || suivant[idim] > eps)
                        {
                            isCi = false;
                        }
                    }
                    else
                    {
                        prec[idim] = 0;
                        suivant[idim] = 0;
                    }
                }
                // If it's good, update everything
                if (isCi)
                {
                    continuity[icurve + 1] = iordre;
                    int index = (icurve - 1) * nbSpace + 1;
                    for (ii = index, idim = 1; idim <= nbSpace; ii++, idim++)
                    {
                        errorMax[ii] += prec[idim];
                        errorMax[ii + nbSpace] += suivant[idim];
                    }
                }
            }
        }
    }

    /// <summary>
    /// Approximates a function (of dimension <paramref name="totalDimension"/>, in
    /// <paramref name="totalNumSS"/> subspaces of dimensions <paramref name="localDimension"/>) on
    /// [<paramref name="first"/>, <paramref name="last"/>] by at most <paramref name="maxSegments"/> polynomial
    /// curves, with the least squares method, keeping the parametrisation.
    /// </summary>
    /// <remarks>
    /// <para>
    /// <paramref name="continuityOrder"/> is the order of the constraints at the ends of each polynomial (0, 1 or
    /// 2). <paramref name="numMaxCoeffs"/> limits the number of coefficients of each polynomial (at least
    /// 2 * ContinuityOrder + 2). <paramref name="code_precis"/> chooses the discretisation (1 fast with average
    /// precision, 2 fast with better precision, 3 slower with good precision, 4 slow with the best precision).
    /// </para>
    /// <para>
    /// Outputs: the number of curves; the number of coefficients of each; the coefficients of curve i in
    /// <paramref name="coefficientArray"/> from (i - 1) * TotalDimension * NumMaxCoeffs + 1, by degree then
    /// dimension; the NumCurves + 1 bounds of the intervals in <paramref name="intervalsArray"/> (from index 1);
    /// the maximum and average errors per subspace and curve. <paramref name="errorCode"/> is 0 when everything
    /// is OK, -1 when an error exceeds its tolerance (the result is the best possible), 1 for inconsistent input.
    /// </para>
    /// </remarks>
    public static void Approximation(int totalDimension, int totalNumSS, NCollection_Array1<int> localDimension,
                                     double first, double last, EvaluatorFunction evaluator,
                                     Cutting cutTool, int continuityOrder, int numMaxCoeffs,
                                     int maxSegments, NCollection_Array1<double> tolerancesArray, int code_precis,
                                     out int numCurves, NCollection_Array1<int> numCoeffPerCurveArray,
                                     NCollection_Array1<double> coefficientArray,
                                     NCollection_Array1<double> intervalsArray, NCollection_Array1<double> errorMaxArray,
                                     NCollection_Array1<double> averageErrorArray, out int errorCode)
    {
        //  double EpsPar =  Precision::Confusion();
        int nupil, theDeg;
        bool isCut = false;

        // Definition of C arrays
        Span<double> tabint = intervalsArray.AsSpan(1);

        numCurves = 0;
        errorCode = 0;
        coefficientArray.Init(0);

        //-------------------------- Input validation ------------------

        if ((maxSegments < 1) || (Math.Abs(last - first) < 1.0e-9))
        {
            errorCode = 1;
            return;
        }

        //--> The total dimension must be the sum of the subspace dimensions
        int idim = 0;
        for (int I = 1; I <= totalNumSS; I++)
        {
            idim += localDimension[I];
        }
        if (idim != totalDimension)
        {
            errorCode = 1;
            return;
        }
        Continuity continuity = C0;
        switch (continuityOrder)
        {
            case 0:
                continuity = C0;
                break;
            case 1:
                continuity = C1;
                break;
            case 2:
                continuity = C2;
                break;
            default:
                throw new ConstructionErrorException();
        }

        //--------------------- Choice of number of points ----------------------
        //---------- and smoothing degree in the orthogonal basis -----------
        //-->  NDJAC is the "working" degree in the orthogonal basis.

        int nbGaussPoints, workDegree;

        global::Kerf.Engine.Numerics.Polynomials.PLib.JacobiParameters(continuity, numMaxCoeffs - 1, code_precis, out nbGaussPoints, out workDegree);
        //      NDJAC=WorkDegree;

        //------------------ Initialization of cutting management ---------

        tabint[0] = first;
        tabint[1] = last;
        nupil = 1;
        numCurves = 0;

        // ********************************************************************
        //                      APPROXIMATION WITH CUTTING
        // ********************************************************************
        JacobiPolynomial jacobiBase = new(workDegree, continuity);
        // HP port: the compiler refuses branching
        int IS;
        bool goto_fin_de_boucle;
        int maxDegree = numMaxCoeffs - 1;
        SimpleApprox approx = new(totalDimension, totalNumSS, continuity, workDegree, nbGaussPoints,
                                            jacobiBase, evaluator);
        while ((nupil - numCurves) != 0)
        {
            //--> When the top of the stack is reached, we are done!

            // HP port: the compiler refuses branching
            goto_fin_de_boucle = false;

            //---- Compute the approximation curve in the Jacobi basis -----

            approx.Perform(localDimension, tolerancesArray, tabint[numCurves], tabint[numCurves + 1], maxDegree);
            if (!approx.IsDone)
            {
                errorCode = 1;
                return;
            }

            //---------- Compute the curve degree and max error ----------

            numCoeffPerCurveArray[numCurves + 1] = 0;

            //    The error must be satisfied on all subspaces, otherwise we split

            bool maxMaxErr = true;
            for (IS = 1; IS <= totalNumSS; IS++)
            {
                if (approx.MaxError(IS) > tolerancesArray[IS])
                {
                    maxMaxErr = false;
                    break;
                }
            }

            if (maxMaxErr)
            {
                numCurves++;
            }
            else
            {
                //-> ...otherwise try to split the current interval in 2...
                double tmil;
                bool large;

                large = cutTool.Value(tabint[numCurves], tabint[numCurves + 1], out tmil);

                if (nupil < maxSegments && large)
                {
                    isCut = true; // Now we know it!
                    int ideb = numCurves + 1;
                    int ideb1 = ideb + 1;
                    int ilong = nupil - numCurves - 1;
                    for (int iI = ilong; iI >= 0; iI--)
                    {
                        tabint[ideb1 + iI] = tabint[ideb + iI];
                    }
                    tabint[ideb] = tmil;
                    nupil++;
                    //--> ... and start over.
                    // HP port: the compiler refuses branching
                    goto_fin_de_boucle = true;
                }
                else
                {
                    //--> If the stack is full...
                    // for now               ErrorCode=-1;
                    numCurves++;
                }
            }
            // HP port: the compiler refuses branching
            if (goto_fin_de_boucle)
            {
                continue;
            }
            for (IS = 1; IS <= totalNumSS; IS++)
            {
                errorMaxArray.SetValue(IS + (numCurves - 1) * totalNumSS, approx.MaxError(IS));
                //---> ...and compute the average error.
                averageErrorArray.SetValue(IS + (numCurves - 1) * totalNumSS, approx.AverageError(IS));
            }

            NCollection_HArray1<double> hJacCoeff = approx.Coefficients();
            theDeg = approx.Degree();
            if (isCut && (theDeg < 2 * continuityOrder + 1))
            {
                // To avoid noisy derivatives at the ends, and maintain correct
                // continuity on the resulting BSpline.
                theDeg = 2 * continuityOrder + 1;
            }

            numCoeffPerCurveArray[numCurves] = theDeg + 1;
            NCollection_Array1<double> coefficients = new(0, (theDeg + 1) * totalDimension - 1);
            jacobiBase.ToCoefficients(totalDimension, theDeg, hJacCoeff.Array1(), coefficients);
            int i, j, f = (theDeg + 1) * totalDimension;
            for (i = 0, j = (numCurves - 1) * totalDimension * numMaxCoeffs + 1; i < f; i++, j++)
            {
                coefficientArray.SetValue(j, coefficients.Value(i));
            }
        }
    }

    private void Perform(int num1DSS, int num2DSS, int num3DSS, Cutting cutTool)
    {
        if (num1DSS < 0 || num2DSS < 0 || num3DSS < 0 || num1DSS + num2DSS + num3DSS <= 0 || _last < _first
            || _maxDegree < 1 || _maxSegments < 0)
        {
            throw new ConstructionErrorException();
        }
        if (_maxDegree > 14)
        {
            _maxDegree = 14;
        }
        //
        // Input
        //
        _numSubSpaces[0] = num1DSS;
        _numSubSpaces[1] = num2DSS;
        _numSubSpaces[2] = num3DSS;
        int totalNumSS = num1DSS + num2DSS + num3DSS, ii, jj, kk, index, dim_index, local_index;
        int totalDimension = _numSubSpaces[0] + 2 * _numSubSpaces[1] + 3 * _numSubSpaces[2];
        double error_value;

        int continuityOrder = 0;
        switch (_continuity)
        {
            case C0:
                continuityOrder = 0;
                break;
            case C1:
                continuityOrder = 1;
                break;
            case C2:
                continuityOrder = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        Span<double> approxStartEnd = stackalloc double[2];
        int numMaxCoeffs = Max(_maxDegree + 1, 2 * continuityOrder + 2);
        _maxDegree = numMaxCoeffs - 1;
        int code_precis = 1;
        //
        //  WARNING : the polynomial coefficients are the
        //  taylor expansion of the polynomial at 0.0e0 !
        //
        approxStartEnd[0] = -1.0e0;
        approxStartEnd[1] = 1.0e0;

        NCollection_Array1<int> localDimension = new(1, totalNumSS);

        index = 1;
        NCollection_Array1<double> localTolerances = new(1, totalNumSS);

        for (jj = 1; jj <= _numSubSpaces[0]; jj++)
        {
            localTolerances.SetValue(index, _my1DTolerances!.Value(jj));
            localDimension.SetValue(index, 1);
            index += 1;
        }
        for (jj = 1; jj <= _numSubSpaces[1]; jj++)
        {
            localTolerances.SetValue(index, _my2DTolerances!.Value(jj));
            localDimension.SetValue(index, 2);
            index += 1;
        }
        for (jj = 1; jj <= _numSubSpaces[2]; jj++)
        {
            localTolerances.SetValue(index, _my3DTolerances!.Value(jj));
            localDimension.SetValue(index, 3);
            index += 1;
        }
        //
        // Output
        //

        int errorCode = 0, numCurves, size = _maxSegments * numMaxCoeffs * totalDimension;
        NCollection_HArray1<int> numCoeffPerCurvePtr = new(1, _maxSegments);

        NCollection_HArray1<double> localCoefficientsPtr = new(1, size);

        NCollection_HArray1<double> intervalsPtr = new(1, _maxSegments + 1);

        NCollection_Array1<double> errorMax = new(1, _maxSegments * totalNumSS);

        NCollection_Array1<double> averageError = new(1, _maxSegments * totalNumSS);

        Approximation(totalDimension, totalNumSS, localDimension, _first, _last, _evaluator, cutTool,
                      continuityOrder, numMaxCoeffs, _maxSegments, localTolerances, code_precis,
                      out numCurves,                        // Number of output curves
                      numCoeffPerCurvePtr.ChangeArray1(),   // Number of coefficients per curve
                      localCoefficientsPtr.ChangeArray1(),  // Solution coefficients
                      intervalsPtr.ChangeArray1(),          // Table of cutting points
                      errorMax,                             // Maximum error bound
                      averageError,                         // Observed average error
                      out errorCode);

        if (errorCode == 0 || errorCode == -1)
        {
            //
            // Everything is OK, or we have a result where one of the max errors
            // is larger than the requested tolerance

            NCollection_Array1<int> tabContinuity = new(1, numCurves);
            NCollection_Array2<double> polynomialIntervalsPtr = new(1, numCurves, 1, 2);
            for (ii = polynomialIntervalsPtr.LowerRow(); ii <= polynomialIntervalsPtr.UpperRow(); ii++)
            {
                // Force a minimum degree of 1 (PRO5474)
                numCoeffPerCurvePtr[ii] = Max(2, numCoeffPerCurvePtr.Value(ii));
                polynomialIntervalsPtr.SetValue(ii, 1, approxStartEnd[0]);
                polynomialIntervalsPtr.SetValue(ii, 2, approxStartEnd[1]);
            }

            PrepareConvert(numCurves, _maxDegree, continuityOrder, num1DSS, num2DSS, num3DSS,
                           numCoeffPerCurvePtr.Array1(), localCoefficientsPtr.ChangeArray1(), polynomialIntervalsPtr,
                           intervalsPtr.Array1(), localTolerances, errorMax, tabContinuity);

            CompPolynomialToPoles aConverter = new(numCurves, totalDimension, _maxDegree, tabContinuity,
                                                           numCoeffPerCurvePtr.Array1(), localCoefficientsPtr.Array1(),
                                                           polynomialIntervalsPtr, intervalsPtr.Array1());

            if (aConverter.IsDone)
            {
                // Convert's accessors return copies (decision 88): take each once
                NCollection_Array2<double> poles = aConverter.Poles();
                _knots = new NCollection_HArray1<double>(aConverter.Knots());
                _mults = new NCollection_HArray1<int>(aConverter.Multiplicities());
                _degree = aConverter.Degree();
                index = 0;
                if (_numSubSpaces[0] > 0)
                {
                    _my1DPoles = new NCollection_HArray2<double>(1, poles.ColLength(), 1, _numSubSpaces[0]);
                    _my1DMaxError = new NCollection_HArray1<double>(1, _numSubSpaces[0]);
                    _my1DAverageError = new NCollection_HArray1<double>(1, _numSubSpaces[0]);
                    for (ii = 1; ii <= poles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= _numSubSpaces[0]; jj++)
                        {
                            _my1DPoles.SetValue(ii, jj, poles.Value(ii, jj));
                        }
                    }

                    for (jj = 1; jj <= _numSubSpaces[0]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS;
                            error_value = Max(errorMax[local_index + jj], error_value);
                        }
                        _my1DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= _numSubSpaces[0]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS;
                            error_value += averageError[local_index + jj];
                        }
                        error_value /= (double)numCurves;
                        _my1DAverageError.SetValue(jj, error_value);
                    }
                    index += _numSubSpaces[0];
                }

                dim_index = index; // For the case where there is no 2D

                if (_numSubSpaces[1] > 0)
                {
                    _my2DPoles = new NCollection_HArray2<Point2D>(1, poles.ColLength(), 1, _numSubSpaces[1]);
                    _my2DMaxError = new NCollection_HArray1<double>(1, _numSubSpaces[1]);
                    _my2DAverageError = new NCollection_HArray1<double>(1, _numSubSpaces[1]);
                    Span<double> point2d = stackalloc double[2];
                    for (ii = 1; ii <= poles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= _numSubSpaces[1]; jj++)
                        {
                            local_index = index + (jj - 1) * 2;
                            for (kk = 1; kk <= 2; kk++)
                            {
                                point2d[kk - 1] = poles.Value(ii, local_index + kk);
                            }
                            _my2DPoles.SetValue(ii, jj, new Point2D(point2d[0], point2d[1]));
                        }
                    }

                    for (jj = 1; jj <= _numSubSpaces[1]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS + index;
                            error_value = Max(errorMax[local_index + jj], error_value);
                        }
                        _my2DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= _numSubSpaces[1]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS + index;
                            error_value += averageError[local_index + jj];
                        }
                        error_value /= (double)numCurves;
                        _my2DAverageError.SetValue(jj, error_value);
                    }
                    index += _numSubSpaces[1];
                    // For poles, the offset must be doubled:
                    dim_index = index + _numSubSpaces[1];
                }

                if (_numSubSpaces[2] > 0)
                {
                    _my3DPoles = new NCollection_HArray2<Point3D>(1, poles.ColLength(), 1, _numSubSpaces[2]);
                    _my3DMaxError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
                    _my3DAverageError = new NCollection_HArray1<double>(1, _numSubSpaces[2]);
                    Span<double> point = stackalloc double[3];
                    for (ii = 1; ii <= poles.ColLength(); ii++)
                    {
                        for (jj = 1; jj <= _numSubSpaces[2]; jj++)
                        {
                            local_index = dim_index + (jj - 1) * 3;
                            for (kk = 1; kk <= 3; kk++)
                            {
                                point[kk - 1] = poles.Value(ii, local_index + kk);
                            }
                            _my3DPoles.SetValue(ii, jj, new Point3D(point[0], point[1], point[2]));
                        }
                    }

                    for (jj = 1; jj <= _numSubSpaces[2]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS + index;
                            error_value = Max(errorMax[local_index + jj], error_value);
                        }
                        _my3DMaxError.SetValue(jj, error_value);
                    }
                    for (jj = 1; jj <= _numSubSpaces[2]; jj++)
                    {
                        error_value = 0.0e0;
                        for (ii = 1; ii <= numCurves; ii++)
                        {
                            local_index = (ii - 1) * totalNumSS + index;
                            error_value += averageError[local_index + jj];
                        }
                        error_value /= (double)numCurves;
                        _my3DAverageError.SetValue(jj, error_value);
                    }
                }
                if (errorCode == 0)
                {
                    _done = true;
                    _hasResult = true;
                }
                else if (errorCode == -1)
                {
                    _hasResult = true;
                }
            }
        }
    }

    /// <summary>True if every subspace is within its tolerance.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is a result, even one outside the tolerances.</summary>
    public bool HasResult => _hasResult;

    /// <summary>A copy of the poles of the 1D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<double>? Poles1d() => _my1DPoles is null ? null : new(_my1DPoles);

    /// <summary>A copy of the poles of the 2D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<Point2D>? Poles2d() => _my2DPoles is null ? null : new(_my2DPoles);

    /// <summary>A copy of the poles of the 3D subspaces (pole index, subspace), or null.</summary>
    public NCollection_HArray2<Point3D>? Poles() => _my3DPoles is null ? null : new(_my3DPoles);

    /// <summary>The number of poles of the B-splines (0 without a result).</summary>
    public int NbPoles()
    {
        if (_done || _hasResult)
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(_degree, false, _mults!.Array1());
        }
        return 0;
    }

    /// <summary>Copies the poles of 1D subspace <paramref name="index"/> into <paramref name="p"/>.</summary>
    public void Poles1d(int index, NCollection_Array1<double> p)
    {
        int ii;
        for (ii = p.Lower(); ii <= p.Upper(); ii++)
        {
            p.SetValue(ii, _my1DPoles!.Value(ii, index));
        }
    }

    /// <summary>Copies the poles of 2D subspace <paramref name="index"/> into <paramref name="p"/>.</summary>
    public void Poles2d(int index, NCollection_Array1<Point2D> p)
    {
        int ii;
        for (ii = p.Lower(); ii <= p.Upper(); ii++)
        {
            p.SetValue(ii, _my2DPoles!.Value(ii, index));
        }
    }

    /// <summary>Copies the poles of 3D subspace <paramref name="index"/> into <paramref name="p"/>.</summary>
    public void Poles(int index, NCollection_Array1<Point3D> p)
    {
        int ii;
        for (ii = p.Lower(); ii <= p.Upper(); ii++)
        {
            p.SetValue(ii, _my3DPoles!.Value(ii, index));
        }
    }

    public int Degree() => _degree;

    public int KnotCount => _knots!.Length();

    /// <summary>The number of subspaces of dimension <paramref name="dimension"/> (1, 2 or 3).</summary>
    public int NumSubSpaces(int dimension)
    {
        if (dimension < 1 || dimension > 3)
        {
            throw new ConstructionErrorException();
        }
        return _numSubSpaces[dimension - 1];
    }

    /// <summary>A copy of the knots, or null.</summary>
    public NCollection_HArray1<double>? Knots() => _knots is null ? null : new(_knots);

    /// <summary>A copy of the multiplicities, or null.</summary>
    public NCollection_HArray1<int>? Multiplicities() => _mults is null ? null : new(_mults);

    /// <summary>A copy of the maximum errors of the subspaces of dimension <paramref name="dimension"/>, or null.</summary>
    public NCollection_HArray1<double>? MaxError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = MaxErrorArray(dimension);
        return ePtr is null ? null : new(ePtr);
    }

    /// <summary>A copy of the average errors of the subspaces of dimension <paramref name="dimension"/>, or null.</summary>
    public NCollection_HArray1<double>? AverageError(int dimension)
    {
        NCollection_HArray1<double>? ePtr = AverageErrorArray(dimension);
        return ePtr is null ? null : new(ePtr);
    }

    /// <summary>The maximum error of subspace <paramref name="index"/> of dimension <paramref name="dimension"/>.</summary>
    public double MaxError(int dimension, int index) => MaxErrorArray(dimension)!.Value(index);

    /// <summary>The average error of subspace <paramref name="index"/> of dimension <paramref name="dimension"/>.</summary>
    public double AverageError(int dimension, int index) => AverageErrorArray(dimension)!.Value(index);

    // OCCT's MaxError(D) body, returning the field; the public form copies it
    private NCollection_HArray1<double>? MaxErrorArray(int d)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (d <= 0 || d > 3)
        {
            throw new OutOfRangeException();
        }
        switch (d)
        {
            case 1:
                ePtr = _my1DMaxError;
                break;
            case 2:
                ePtr = _my2DMaxError;
                break;
            case 3:
                ePtr = _my3DMaxError;
                break;
        }
        return ePtr;
    }

    // OCCT's AverageError(D) body, returning the field; the public form copies it
    private NCollection_HArray1<double>? AverageErrorArray(int d)
    {
        NCollection_HArray1<double>? ePtr = null;
        if (d <= 0 || d > 3)
        {
            throw new OutOfRangeException();
        }
        switch (d)
        {
            case 1:
                ePtr = _my1DAverageError;
                break;
            case 2:
                ePtr = _my2DAverageError;
                break;
            case 3:
                ePtr = _my3DAverageError;
                break;
        }
        return ePtr;
    }

    /// <summary>Writes the maximum error of each subspace.</summary>
    public void Dump(TextWriter o)
    {
        int ii;
        o.Write("Dump of ApproxAFunction\n");
        if (_numSubSpaces[0] > 0)
        {
            o.Write("Error(s) 1d = \n");
            for (ii = 1; ii <= _numSubSpaces[0]; ii++)
            {
                o.Write("   " + MaxError(1, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }

        if (_numSubSpaces[1] > 0)
        {
            o.Write("Error(s) 2d = \n");
            for (ii = 1; ii <= _numSubSpaces[1]; ii++)
            {
                o.Write("   " + MaxError(2, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }

        if (_numSubSpaces[2] > 0)
        {
            o.Write("Error(s) 3d = \n");
            for (ii = 1; ii <= _numSubSpaces[2]; ii++)
            {
                o.Write("   " + MaxError(3, ii).ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }
}
