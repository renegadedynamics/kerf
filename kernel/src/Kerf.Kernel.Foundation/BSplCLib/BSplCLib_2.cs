// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Splines;

// methods for 1 dimensional BSplines
internal static partial class BSplCLib
{
    // Note: OCCT's BSplCLib_DataContainer (BSplCLib_DataContainer_T<1> of BSplCLib_CurveComputation.pxx): stack
    // buffers for the poles, knots and derivatives of one span. The port allocates them with stackalloc.
    private const int DataContainer1_PolesSize = (CurveComputation.THE_MAX_DEGREE + 1) * (1 + 1);
    private const int DataContainer1_KnotsSize = 2 * CurveComputation.THE_MAX_DEGREE;
    private const int DataContainer1_DersSize = 1 * 4;

    /// <summary>
    /// Copies in <paramref name="lp"/> the poles (and weights, multiplying the poles: homogeneous coordinates)
    /// of the span starting at pole <paramref name="index"/>, Degree + 1 of them, wrapping around the poles.
    /// </summary>
    public static void BuildEval(int degree, int index, NCollection_Array1<double> poles,
                                 NCollection_Array1<double>? weights, Span<double> lp)
    {
        int pLower = poles.Lower();
        int pUpper = poles.Upper();
        int i;
        int ip = pLower + index - 1;
        double w;
        Span<double> pole = lp;
        int aPole = 0;
        if (weights == null)
        {

            for (i = 0; i <= degree; i++)
            {
                ip++;
                if (ip > pUpper)
                {
                    ip = pLower;
                }
                pole[aPole + 0] = poles[ip];
                aPole += 1;
            }
        }
        else
        {

            for (i = 0; i <= degree; i++)
            {
                ip++;
                if (ip > pUpper)
                {
                    ip = pLower;
                }
                pole[aPole + 1] = w = weights[ip];
                pole[aPole + 0] = poles[ip] * w;
                aPole += 2;
            }
        }
    }

    private static void PrepareEval(ref double u, ref int index, out int dim, out bool rational, int degree,
                                    bool periodic, NCollection_Array1<double> poles,
                                    NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                                    NCollection_Array1<int>? mults, Span<double> dc_poles, Span<double> dc_knots)
    {
        // Set the Index
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, knots, mults, u, periodic, ref index, out u);

        // make the knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(degree, index, periodic, knots, mults, dc_knots);
        if (mults == null)
        {
            index -= knots.Lower() + degree;
        }
        else
        {
            index = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(degree, index, periodic, mults);
        }

        // check truly rational
        rational = (weights != null);
        if (rational)
        {
            int wLower = weights!.Lower() + index;
            rational = global::Kerf.Engine.Geometry.Splines.BSplCLib.IsRational(weights, wLower, wLower + degree);
        }

        // make the poles
        if (rational)
        {
            dim = 2;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildEval(degree, index, poles, weights, dc_poles);
        }
        else
        {
            dim = 1;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildEval(degree, index, poles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), dc_poles);
        }
    }

    /// <summary>The value at <paramref name="U"/> of a one-dimensional B-spline (rational if weights are given).</summary>
    public static void D0(double U, int Index, int degree, bool periodic, NCollection_Array1<double> poles,
                          NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults, out double p)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        CurveComputation.ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        PrepareEval(ref u, ref index, out dim, out rational, degree, periodic, poles, weights, knots, mults, dc_poles, dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u, degree, dc_knots, dim, dc_poles);
        if (rational)
        {
            p = dc_poles[0] / dc_poles[1];
        }
        else
        {
            p = dc_poles[0];
        }
    }

    /// <summary>The value and first derivative at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D1(double U, int Index, int degree, bool periodic, NCollection_Array1<double> poles,
                          NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults, out double p, out double v)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        CurveComputation.ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, degree, periodic, poles, weights, knots, mults, dc_poles, dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 1, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 1, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        p = result[0];
        v = result[1];
    }

    /// <summary>The value and first two derivatives at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D2(double U, int Index, int degree, bool periodic, NCollection_Array1<double> poles,
                          NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults, out double p, out double v1, out double v2)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        CurveComputation.ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, degree, periodic, poles, weights, knots, mults, dc_poles, dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 2, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 2, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        p = result[0];
        v1 = result[1];
        if (!rational && (degree < 2))
        {
            v2 = 0.0;
        }
        else
        {
            v2 = result[2];
        }
    }

    /// <summary>The value and first three derivatives at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D3(double U, int Index, int degree, bool periodic, NCollection_Array1<double> poles,
                          NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults, out double p, out double v1, out double v2, out double v3)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        CurveComputation.ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, degree, periodic, poles, weights, knots, mults, dc_poles, dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 3, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 3, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        p = result[0];
        v1 = result[1];
        if (!rational && (degree < 2))
        {
            v2 = 0.0;
        }
        else
        {
            v2 = result[2];
        }
        if (!rational && (degree < 3))
        {
            v3 = 0.0;
        }
        else
        {
            v3 = result[3];
        }
    }

    /// <summary>The derivative of order <paramref name="n"/> at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void DN(double U, int n, int Index, int degree, bool periodic, NCollection_Array1<double> poles,
                          NCollection_Array1<double>? weights, NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults, out double vn)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        CurveComputation.ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        PrepareEval(ref u, ref index, out dim, out rational, degree, periodic, poles, weights, knots, mults, dc_poles, dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, n, dc_knots, dim, dc_poles);
        if (rational)
        {
            Span<double> v = stackalloc double[1];
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, n, 1, dc_poles, v, false);
            vn = v[0];
        }
        else
        {
            if (n > degree)
            {
                vn = 0.0;
            }
            else
            {
                vn = dc_poles[n];
            }
        }
    }

    /// <summary>
    /// Builds the banded matrix of the B-spline basis functions (of the derivative orders given by
    /// <paramref name="orderArray"/>) at <paramref name="parameters"/>, for interpolation. The matrix must have
    /// Parameters' row range and columns 1 to UpperBandWidth + LowerBandWidth + 1. Returns 0 on success.
    /// </summary>
    public static int BuildBSpMatrix(NCollection_Array1<double> parameters, NCollection_Array1<int> orderArray,
                                     NCollection_Array1<double> flatKnots, int degree, Matrix matrix,
                                     out int upperBandWidth, out int lowerBandWidth)
    {
        NCollection_Array1<int> contactOrderArray = orderArray;
        const int maxOrder = 25 + 1; // BSplCLib::MaxDegree() + 1
        int order = degree + 1;
        upperBandWidth = degree;
        lowerBandWidth = degree;
        int bandWidth = upperBandWidth + lowerBandWidth + 1;

        if (matrix.LowerRow() != parameters.Lower() || matrix.UpperRow() != parameters.Upper()
            || matrix.LowerCol() != 1 || matrix.UpperCol() != bandWidth)
        {
            return 1;
        }

        // A stack buffer in OCCT; math_Matrix needs an array to share
        double[] basisBuf = new double[maxOrder * maxOrder];
        Matrix bSplineBasis = new(basisBuf, 1, maxOrder, 1, maxOrder);

        // Zero the entire matrix once instead of per-row zero-fill loops.
        matrix.Init(0.0);
        int matLRow = matrix.LowerRow();
        int matNCols = matrix.ColNumber();
        Span<double> matData = matrix.AsSpan();

        for (int i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            int firstNonZeroIndex = 0;
            int contactOrder = contactOrderArray[i];
            int errorCode = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(contactOrder,
                                                        order,
                                                        flatKnots,
                                                        parameters[i],
                                                        out firstNonZeroIndex,
                                                        bSplineBasis);
            if (errorCode != 0)
            {
                return 2;
            }

            int index = lowerBandWidth + 1 + firstNonZeroIndex - i;
            Span<double> rowData = matData.Slice((i - matLRow) * matNCols);
            ReadOnlySpan<double> basisSrc = basisBuf.AsSpan(contactOrder * maxOrder);
            for (int j = 0; j < order; j++)
            {
                rowData[index + j - 1] = basisSrc[j];
            }
        }

        return 0;
    }

    /// <summary>
    /// Factors the banded matrix in LU form, with a banded storage of the L components. Returns 0, or 1 with
    /// <paramref name="pivotIndexProblem"/> the row of a null pivot. Not to be used for totally positive matrices
    /// (B-spline matrices built at the Schoenberg points).
    /// </summary>
    public static int FactorBandedMatrix(Matrix matrix, int upperBandWidth, int lowerBandWidth,
                                         out int pivotIndexProblem)
    {
        int bandWidth = upperBandWidth + lowerBandWidth + 1;
        pivotIndexProblem = 0;

        int lRow = matrix.LowerRow();
        int nCols = matrix.ColNumber();
        // &Matrix(aLRow, 1): the storage starts at column LowerCol
        Span<double> data = matrix.AsSpan();
        int aBase = 1 - matrix.LowerCol();

        for (int i = lRow + 1; i <= matrix.UpperRow(); i++)
        {
            int minIndex = (lowerBandWidth - i + 2 >= 1 ? lowerBandWidth - i + 2 : 1);
            int rowI = aBase + (i - lRow) * nCols;

            for (int j = minIndex; j <= lowerBandWidth; j++)
            {
                int index = i - lowerBandWidth + j - 1;
                int rowIdx = aBase + (index - lRow) * nCols;
                double pivot = data[rowIdx + lowerBandWidth];
                if (Math.Abs(pivot) <= RealSmall())
                {
                    pivotIndexProblem = index;
                    return 1;
                }

                double inverse = -1.0 / pivot;
                data[rowI + j - 1] = data[rowI + j - 1] * inverse;
                int maxIndex = bandWidth + index - i;

                for (int k = j + 1; k <= maxIndex; k++)
                {
                    data[rowI + k - 1] += data[rowI + j - 1] * data[rowIdx + k + i - index - 1];
                }
            }
        }

        return 0;
    }

    /// <summary>
    /// Evaluates the B-spline basis at <paramref name="parameter"/> up to <paramref name="derivativeOrder"/>:
    /// BsplineBasis(1, n) is the value of the n-th non-vanishing basis function (index FirstNonZeroBsplineIndex +
    /// n - 1), BsplineBasis(k + 1, n) its k-th derivative. The matrix needs rows and columns from 1, at least
    /// DerivativeOrder + 1 rows and Order columns. Returns 0 on success.
    /// </summary>
    public static int EvalBsplineBasis(int derivativeOrder, int order, NCollection_Array1<double> flatKnots,
                                       double parameter, out int firstNonZeroBsplineIndex, Matrix bsplineBasis,
                                       bool isPeriodic = false)
    {
        int derivativeRequest = derivativeOrder;
        // the matrix must have at least DerivativeRequest + 1
        //   row and Order columns
        // the result are stored in the following way in
        // the Bspline matrix
        // Let i be the FirstNonZeroBsplineIndex and
        // t be the parameter value, k the order of the
        // knot vector, r the DerivativeRequest :
        //
        //   B (t)   B (t)                     B (t)
        //    i       i+1                       i+k-1
        //
        //    (1)     (1)                       (1)
        //   B (t)   B (t)                     B (t)
        //    i       i+1                       i+k-1
        //
        //
        //
        //
        //    (r)     (r)                       (r)
        //   B (t)   B (t)                     B (t)
        //    i       i+1                       i+k-1
        //
        firstNonZeroBsplineIndex = 0;
        int localRequest = derivativeRequest;
        if (derivativeRequest >= order)
        {
            localRequest = order - 1;
        }

        if (bsplineBasis.LowerCol() != 1 || bsplineBasis.UpperCol() < order
            || bsplineBasis.LowerRow() != 1 || bsplineBasis.UpperRow() <= localRequest)
        {
            return 1;
        }

        int numPoles = flatKnots.Upper() - flatKnots.Lower() + 1 - order;
        int ii = 0;
        double newParam = 0.0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(order - 1,
                                 flatKnots,
                                 parameter,
                                 isPeriodic,
                                 order,
                                 numPoles + 1,
                                 out ii,
                                 out newParam);

        firstNonZeroBsplineIndex = ii - order + 1;

        // Spans over BsplineBasis and FlatKnots bypass the accessors in the tight loops
        int basisNCols = bsplineBasis.ColNumber();
        Span<double> basisData = bsplineBasis.AsSpan();
        ReadOnlySpan<double> knotsData = flatKnots.AsSpan();
        double resolution = Constants.Resolution();
        ii -= flatKnots.Lower(); // rebase to zero-based indexing into aKnotsData

        basisData[0] = 1.0;
        localRequest = derivativeRequest;
        if (derivativeRequest >= order)
        {
            localRequest = order - 1;
        }

        for (int qq = 2; qq <= order - localRequest; qq++)
        {
            basisData[qq - 1] = 0.0;

            for (int pp = 1; pp <= qq - 1; pp++)
            {
                double scale = knotsData[ii + pp] - knotsData[ii - qq + pp + 1];
                if (Math.Abs(scale) < resolution)
                {
                    return 2;
                }

                double factor = (parameter - knotsData[ii - qq + pp + 1]) / scale;
                double saved = factor * basisData[pp - 1];
                basisData[pp - 1] *= (1.0 - factor);
                basisData[pp - 1] += basisData[qq - 1];
                basisData[qq - 1] = saved;
            }
        }

        for (int qq = order - localRequest + 1; qq <= order; qq++)
        {
            for (int pp = 1; pp <= qq - 1; pp++)
            {
                basisData[(order - qq + 1) * basisNCols + (pp - 1)] = basisData[pp - 1];
            }
            basisData[qq - 1] = 0.0;

            for (int ss = order - localRequest + 1; ss <= qq; ss++)
            {
                basisData[(order - ss + 1) * basisNCols + (qq - 1)] = 0.0;
            }

            for (int pp = 1; pp <= qq - 1; pp++)
            {
                double scale = knotsData[ii + pp] - knotsData[ii - qq + pp + 1];
                if (Math.Abs(scale) < resolution)
                {
                    return 2;
                }

                double inverse = 1.0 / scale;
                double factor = (parameter - knotsData[ii - qq + pp + 1]) * inverse;
                double saved = factor * basisData[pp - 1];
                basisData[pp - 1] *= (1.0 - factor);
                basisData[pp - 1] += basisData[qq - 1];
                basisData[qq - 1] = saved;
                double localInverse = (double)(qq - 1) * inverse;

                for (int ss = order - localRequest + 1; ss <= qq; ss++)
                {
                    Span<double> rowS = basisData.Slice((order - ss + 1) * basisNCols);
                    saved = localInverse * rowS[pp - 1];
                    rowS[pp - 1] *= -localInverse;
                    rowS[pp - 1] += rowS[qq - 1];
                    rowS[qq - 1] = saved;
                }
            }
        }

        return 0;
    }

    /// <summary>
    /// Dimension-free version of MovePointAndTangent: moves the B-spline (poles of dimension
    /// <paramref name="arrayDimension"/>) so that its point at <paramref name="u"/> moves by
    /// <paramref name="delta"/> and its derivative by <paramref name="deltaDerivative"/>.
    /// <paramref name="startingCondition"/> and <paramref name="endingCondition"/> say how many derivatives at the
    /// ends must not move (-1: the end itself may move). <paramref name="errorStatus"/> is 0, 1 if there are not
    /// enough poles for the conditions, or 2 if the conditions are invalid.
    /// </summary>
    public static void MovePointAndTangent(double u, int arrayDimension, ReadOnlySpan<double> delta,
                                           ReadOnlySpan<double> deltaDerivative, double tolerance, int degree,
                                           int startingCondition, int endingCondition, ReadOnlySpan<double> poles,
                                           NCollection_Array1<double>? weights, NCollection_Array1<double> flatKnots,
                                           Span<double> newPoles, out int errorStatus)
    {
        int num_poles, num_knots, ii, jj, conditions, start_num_poles, end_num_poles, index, start_index,
          end_index, other_index, type, order;

        double new_parameter, value, divide, end_value, start_value;
        ReadOnlySpan<double> poles_array, delta_array, derivatives_array, weights_array;
        Span<double> new_poles_array;

        errorStatus = 0;
        weights_array = default;
        if (weights != null)
        {
            weights_array = weights.AsSpan();
        }

        poles_array = poles;
        new_poles_array = newPoles;
        delta_array = delta;
        derivatives_array = deltaDerivative;
        order = degree + 1;
        num_knots = flatKnots.Length();
        num_poles = num_knots - order;
        conditions = startingCondition + endingCondition + 4;
        //
        // check validity of input data
        //
        if (startingCondition >= -1 && startingCondition <= degree && endingCondition >= -1
            && endingCondition <= degree && conditions <= num_poles)
        {
            //
            // check the parameter is within bounds
            //
            start_index = flatKnots.Lower() + degree;
            end_index = flatKnots.Upper() - degree;
            //
            //  check if there is enough room to move the poles
            //
            bool aConditions = true;
            if (startingCondition == -1)
            {
                aConditions = aConditions && (flatKnots[start_index] <= u);
            }
            else
            {
                aConditions = aConditions && (flatKnots[start_index] + tolerance < u);
            }
            if (endingCondition == -1)
            {
                aConditions = aConditions && (flatKnots[end_index] >= u);
            }
            else
            {
                aConditions = aConditions && (flatKnots[end_index] - tolerance > u);
            }

            if (aConditions)
            {
                //
                // build 2 auxiliary functions
                //
                NCollection_Array1<double> schoenberg_points = new(1, num_poles);
                NCollection_Array1<double> first_function = new(1, num_poles);
                NCollection_Array1<double> second_function = new(1, num_poles);

                BuildSchoenbergPoints(degree, flatKnots, schoenberg_points);
                start_index = startingCondition + 2;
                end_index = num_poles - endingCondition - 1;
                LocateParameter(schoenberg_points,
                                u,
                                false,
                                start_index,
                                end_index,
                                out index,
                                out new_parameter,
                                0,
                                1);

                if (index == start_index)
                {
                    other_index = index + 1;
                }
                else if (index == end_index)
                {
                    other_index = index - 1;
                }
                // OCCT compares U with the flat knots at index, an index into schoenberg_points
                else if (u - flatKnots[index] < flatKnots[index + 1] - u)
                {
                    other_index = index - 1;
                }
                else
                {
                    other_index = index + 1;
                }
                type = 3;

                start_num_poles = startingCondition + 2;
                end_num_poles = num_poles - endingCondition - 1;
                if (start_num_poles == 1)
                {
                    start_value = schoenberg_points[num_poles] - schoenberg_points[1];
                    start_value = schoenberg_points[1] - start_value;
                }
                else
                {
                    start_value = schoenberg_points[start_num_poles - 1];
                }
                if (end_num_poles == num_poles)
                {
                    end_value = schoenberg_points[num_poles] - schoenberg_points[1];
                    end_value = schoenberg_points[num_poles] + end_value;
                }
                else
                {
                    end_value = schoenberg_points[end_num_poles + 1];
                }

                for (ii = 1; ii < start_num_poles; ii++)
                {
                    first_function[ii] = 0.0e0;
                    second_function[ii] = 0.0e0;
                }

                for (ii = end_num_poles + 1; ii <= num_poles; ii++)
                {
                    first_function[ii] = 0.0e0;
                    second_function[ii] = 0.0e0;
                }
                divide = 1.0e0 / (schoenberg_points[index] - start_value);

                for (ii = start_num_poles; ii <= index; ii++)
                {
                    value = schoenberg_points[ii] - start_value;
                    value *= divide;
                    first_function[ii] = 1.0e0;

                    for (jj = 0; jj < type; jj++)
                    {
                        first_function[ii] *= value;
                    }
                }
                divide = 1.0e0 / (end_value - schoenberg_points[index]);

                for (ii = index; ii <= end_num_poles; ii++)
                {
                    value = end_value - schoenberg_points[ii];
                    value *= divide;
                    first_function[ii] = 1.0e0;

                    for (jj = 0; jj < type; jj++)
                    {
                        first_function[ii] *= value;
                    }
                }

                divide = 1.0e0 / (schoenberg_points[other_index] - start_value);

                for (ii = start_num_poles; ii <= other_index; ii++)
                {
                    value = schoenberg_points[ii] - start_value;
                    value *= divide;
                    second_function[ii] = 1.0e0;

                    for (jj = 0; jj < type; jj++)
                    {
                        second_function[ii] *= value;
                    }
                }
                divide = 1.0e0 / (end_value - schoenberg_points[other_index]);

                for (ii = other_index; ii <= end_num_poles; ii++)
                {
                    value = end_value - schoenberg_points[ii];
                    value *= divide;
                    second_function[ii] = 1.0e0;

                    for (jj = 0; jj < type; jj++)
                    {
                        second_function[ii] *= value;
                    }
                }

                //
                //  compute the point and derivatives of both functions
                //
                // double results[2][2], weights_results[2][2]: row ii at [2 * ii]
                Span<double> results = stackalloc double[4];
                Span<double> weights_results = stackalloc double[4];
                Span<int> extrap_mode = stackalloc int[2];
                int derivative_request = 1, dimension = 1;
                bool periodic_flag = false;

                extrap_mode[0] = degree;
                extrap_mode[1] = degree;
                if (weights != null)
                {
                    //
                    // evaluate in homogenised form
                    //
                    Eval(u,
                         periodic_flag,
                         derivative_request,
                         extrap_mode,
                         degree,
                         flatKnots,
                         dimension,
                         first_function.AsSpan(),
                         weights_array,
                         results.Slice(0, 2),
                         weights_results.Slice(0, 2));

                    Eval(u,
                         periodic_flag,
                         derivative_request,
                         extrap_mode,
                         degree,
                         flatKnots,
                         dimension,
                         second_function.AsSpan(),
                         weights_array,
                         results.Slice(2, 2),
                         weights_results.Slice(2, 2));
                    //
                    //  compute the rational derivatives values
                    //

                    for (ii = 0; ii < 2; ii++)
                    {
                        global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivatives(1, 1, results.Slice(2 * ii), weights_results.Slice(2 * ii), results.Slice(2 * ii));
                    }
                }
                else
                {
                    Eval(u, false, 1, extrap_mode, degree, flatKnots, 1, first_function.AsSpan(), results.Slice(0, 2));

                    Eval(u, false, 1, extrap_mode, degree, flatKnots, 1, second_function.AsSpan(), results.Slice(2, 2));
                }
                Matrix2x2 a_matrix = new();

                for (ii = 0; ii < 2; ii++)
                {

                    for (jj = 0; jj < 2; jj++)
                    {
                        a_matrix.SetValue(ii + 1, jj + 1, results[2 * ii + jj]);
                    }
                }
                a_matrix.Invert();
                // math_Vector(0, ArrayDimension - 1) in OCCT, which keeps small vectors on the stack
                Span<double> the_a_vector = arrayDimension <= 128 ? stackalloc double[arrayDimension] : new double[arrayDimension];
                Span<double> the_b_vector = arrayDimension <= 128 ? stackalloc double[arrayDimension] : new double[arrayDimension];

                for (ii = 0; ii < arrayDimension; ii++)
                {
                    the_a_vector[ii] =
                      a_matrix.Value(1, 1) * delta_array[ii] + a_matrix.Value(2, 1) * derivatives_array[ii];
                    the_b_vector[ii] =
                      a_matrix.Value(1, 2) * delta_array[ii] + a_matrix.Value(2, 2) * derivatives_array[ii];
                }
                index = 0;

                for (ii = 0; ii < num_poles; ii++)
                {

                    for (jj = 0; jj < arrayDimension; jj++)
                    {
                        new_poles_array[index] = poles_array[index];
                        new_poles_array[index] += first_function[ii + 1] * the_a_vector[jj];
                        new_poles_array[index] += second_function[ii + 1] * the_b_vector[jj];
                        index += 1;
                    }
                }
            }
            else
            {
                errorStatus = 1;
            }
        }
        else
        {
            errorStatus = 2;
        }
    }

    /// <summary>
    /// Multiplies the vectorial B-spline F(t) (poles [1..NumPoles][1..PolesDimension]) by the function a(t):
    /// interpolates a(t) * F(t) at the Schoenberg points of the B-spline of degree <paramref name="newDegree"/> on
    /// <paramref name="flatKnots"/>. The caller ensures a(t) * F(t) is such a B-spline and that a(t) has F's range;
    /// nothing is checked. <paramref name="status"/> is 0, 1 if the function failed, or the pivot index of a
    /// failed interpolation.
    /// </summary>
    public static void FunctionMultiply(EvaluatorFunction function, int bSplineDegree,
                                        NCollection_Array1<double> bSplineFlatKnots, int polesDimension,
                                        ReadOnlySpan<double> poles, NCollection_Array1<double> flatKnots,
                                        int newDegree, Span<double> newPoles, out int status)
    {
        EvaluatorFunction functionPtr = function;
        Span<double> arrayOfPoles = newPoles;
        int numNewPoles = flatKnots.Length() - newDegree - 1;
        Span<double> startEnd = [flatKnots[newDegree + 1], flatKnots[numNewPoles + 1]];

        NCollection_Array1<double> parameters = new(1, numNewPoles);
        NCollection_Array1<int> contactOrderArray = new(1, numNewPoles);
        NCollection_Array1<double> newPolesArray = new(1, numNewPoles * polesDimension);

        Span<double> arrayOfNewPoles = newPolesArray.AsSpan();
        BuildSchoenbergPoints(newDegree, flatKnots, parameters);

        if (parameters[1] < startEnd[0])
        {
            parameters[1] = startEnd[0];
        }
        if (parameters[numNewPoles] > startEnd[1])
        {
            parameters[numNewPoles] = startEnd[1];
        }

        // OCCT passes a single int where Eval reads two extrapolation modes (ExtrapMode[1] is read past it, from
        // uninitialised stack memory, but only when a parameter lies beyond the last knot); the port passes the
        // degree for both ends
        ReadOnlySpan<int> extrapMode = [bSplineDegree, bSplineDegree];
        int index = 0;
        for (int i = 1; i <= numNewPoles; i++)
        {
            contactOrderArray[i] = 0;
            functionPtr.Evaluate(contactOrderArray[i], startEnd, parameters[i], out double result, out int errorCode);
            if (errorCode != 0)
            {
                status = 1;
                return;
            }

            Eval(parameters[i],
                 false,
                 0,
                 extrapMode,
                 bSplineDegree,
                 bSplineFlatKnots,
                 polesDimension,
                 poles,
                 arrayOfNewPoles[index..]);

            for (int j = 0; j < polesDimension; j++)
            {
                arrayOfNewPoles[index] *= result;
                index += 1;
            }
        }

        Interpolate(newDegree,
                    flatKnots,
                    parameters,
                    contactOrderArray,
                    polesDimension,
                    arrayOfNewPoles,
                    out status);

        for (int i = 0; i < numNewPoles * polesDimension; i++)
        {
            arrayOfPoles[i] = arrayOfNewPoles[i];
        }
    }

    /// <summary>
    /// Composes the vectorial B-spline F(t) (poles [1..NumPoles][1..PolesDimension]) with the function a(t):
    /// interpolates F(a(t)) at the Schoenberg points of the B-spline of degree <paramref name="newDegree"/> on
    /// <paramref name="flatKnots"/>. The caller ensures F(a(t)) is such a B-spline and that a(t) maps FlatKnots'
    /// range onto BSplineFlatKnots'; nothing is checked. <paramref name="status"/> is 0, 1 if the function
    /// failed, or the pivot index of a failed interpolation.
    /// </summary>
    public static void FunctionReparameterise(EvaluatorFunction function, int bSplineDegree,
                                              NCollection_Array1<double> bSplineFlatKnots, int polesDimension,
                                              ReadOnlySpan<double> poles, NCollection_Array1<double> flatKnots,
                                              int newDegree, Span<double> newPoles, out int status)
    {
        EvaluatorFunction functionPtr = function;
        Span<double> arrayOfPoles = newPoles;
        int numNewPoles = flatKnots.Length() - newDegree - 1;
        Span<double> startEnd = [flatKnots[newDegree + 1], flatKnots[numNewPoles + 1]];

        NCollection_Array1<double> parameters = new(1, numNewPoles);
        NCollection_Array1<int> contactOrderArray = new(1, numNewPoles);
        NCollection_Array1<double> newPolesArray = new(1, numNewPoles * polesDimension);

        Span<double> arrayOfNewPoles = newPolesArray.AsSpan();
        BuildSchoenbergPoints(newDegree, flatKnots, parameters);

        // OCCT passes a single int where Eval reads two extrapolation modes (ExtrapMode[1] is read past it, from
        // uninitialised stack memory, but only when a parameter lies beyond the last knot); the port passes the
        // degree for both ends
        ReadOnlySpan<int> extrapMode = [bSplineDegree, bSplineDegree];
        int index = 0;
        for (int i = 1; i <= numNewPoles; i++)
        {
            contactOrderArray[i] = 0;
            functionPtr.Evaluate(contactOrderArray[i], startEnd, parameters[i], out double result, out int errorCode);
            if (errorCode != 0)
            {
                status = 1;
                return;
            }

            Eval(result,
                 false,
                 0,
                 extrapMode,
                 bSplineDegree,
                 bSplineFlatKnots,
                 polesDimension,
                 poles,
                 arrayOfNewPoles[index..]);
            index += polesDimension;
        }

        Interpolate(newDegree,
                    flatKnots,
                    parameters,
                    contactOrderArray,
                    polesDimension,
                    arrayOfNewPoles,
                    out status);

        for (int i = 0; i < numNewPoles * polesDimension; i++)
        {
            arrayOfPoles[i] = arrayOfNewPoles[i];
        }
    }

    /// <summary>
    /// <see cref="FunctionMultiply(EvaluatorFunction, int, NCollection_Array1{double}, int, ReadOnlySpan{double}, NCollection_Array1{double}, int, Span{double}, out int)"/>
    /// for a one-dimensional B-spline. Throws <see cref="ConstructionErrorException"/> if the pole counts don't
    /// match the knots.
    /// </summary>
    public static void FunctionMultiply(EvaluatorFunction function, int bSplineDegree,
                                        NCollection_Array1<double> bSplineFlatKnots, NCollection_Array1<double> poles,
                                        NCollection_Array1<double> flatKnots, int newDegree,
                                        NCollection_Array1<double> newPoles, out int status)
    {
        int num_bspline_poles = bSplineFlatKnots.Length() - bSplineDegree - 1;
        int num_new_poles = flatKnots.Length() - newDegree - 1;

        if (poles.Length() != num_bspline_poles || newPoles.Length() != num_new_poles)
        {
            throw new ConstructionErrorException();
        }
        Span<double> array_of_poles = poles.AsSpan();
        Span<double> array_of_new_poles = newPoles.AsSpan();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(function,
                                  bSplineDegree,
                                  bSplineFlatKnots,
                                  1,
                                  array_of_poles,
                                  flatKnots,
                                  newDegree,
                                  array_of_new_poles,
                                  out status);
    }

    /// <summary>
    /// <see cref="FunctionReparameterise(EvaluatorFunction, int, NCollection_Array1{double}, int, ReadOnlySpan{double}, NCollection_Array1{double}, int, Span{double}, out int)"/>
    /// for a one-dimensional B-spline. Throws <see cref="ConstructionErrorException"/> if the pole counts don't
    /// match the knots.
    /// </summary>
    public static void FunctionReparameterise(EvaluatorFunction function, int bSplineDegree,
                                              NCollection_Array1<double> bSplineFlatKnots,
                                              NCollection_Array1<double> poles, NCollection_Array1<double> flatKnots,
                                              int newDegree, NCollection_Array1<double> newPoles, out int status)
    {
        int num_bspline_poles = bSplineFlatKnots.Length() - bSplineDegree - 1;
        int num_new_poles = flatKnots.Length() - newDegree - 1;

        if (poles.Length() != num_bspline_poles || newPoles.Length() != num_new_poles)
        {
            throw new ConstructionErrorException();
        }
        Span<double> array_of_poles = poles.AsSpan();
        Span<double> array_of_new_poles = newPoles.AsSpan();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(function,
                                        bSplineDegree,
                                        bSplineFlatKnots,
                                        1,
                                        array_of_poles,
                                        flatKnots,
                                        newDegree,
                                        array_of_new_poles,
                                        out status);
    }

    /// <summary>
    /// Merges two knot vectors (of B-splines of degrees <paramref name="degree1"/> and <paramref name="degree2"/>),
    /// both reparametrised to [StartValue, EndValue], into the knots and multiplicities of their product, of degree
    /// Degree1 + Degree2, and its number of poles.
    /// </summary>
    public static void MergeBSplineKnots(double tolerance, double startValue, double endValue, int degree1,
                                         NCollection_Array1<double> Knots1, NCollection_Array1<int> mults1,
                                         int degree2, NCollection_Array1<double> Knots2,
                                         NCollection_Array1<int> mults2, out int numPoles,
                                         out NCollection_HArray1<double> newKnots, out NCollection_HArray1<int> newMults)
    {
        int ii, jj, continuity, set_mults_flag, degree, index, num_knots;
        if (startValue < endValue - tolerance)
        {
            NCollection_Array1<double> knots1 = new(1, Knots1.Length());
            NCollection_Array1<double> knots2 = new(1, Knots2.Length());
            degree = degree1 + degree2;
            index = 1;

            for (ii = Knots1.Lower(); ii <= Knots1.Upper(); ii++)
            {
                knots1[index] = Knots1[ii];
                index += 1;
            }
            index = 1;

            for (ii = Knots2.Lower(); ii <= Knots2.Upper(); ii++)
            {
                knots2[index] = Knots2[ii];
                index += 1;
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(startValue, endValue, knots1);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(startValue, endValue, knots2);
            num_knots = 0;
            jj = 1;

            for (ii = 1; ii <= knots1.Length(); ii++)
            {

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] - tolerance)
                {
                    jj += 1;
                    num_knots += 1;
                }

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] + tolerance)
                {
                    jj += 1;
                }
                num_knots += 1;
            }
            newKnots = new NCollection_HArray1<double>(1, num_knots);
            newMults = new NCollection_HArray1<int>(1, num_knots);
            num_knots = 1;
            jj = 1;

            for (ii = 1; ii <= knots1.Length(); ii++)
            {

                // OCCT indexes Mults1 and Mults2 from 1, whatever their lower bounds (as it does knots1 and knots2)
                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] - tolerance)
                {
                    newKnots.ChangeArray1()[num_knots] = knots2[jj];
                    newMults.ChangeArray1()[num_knots] = mults2[jj] + degree1;
                    jj += 1;
                    num_knots += 1;
                }
                set_mults_flag = 0;

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] + tolerance)
                {
                    continuity = Min(degree1 - mults1[ii], degree2 - mults2[jj]);
                    set_mults_flag = 1;
                    newMults.ChangeArray1()[num_knots] = degree - continuity;
                    jj += 1;
                }

                newKnots.ChangeArray1()[num_knots] = knots1[ii];
                if (set_mults_flag == 0)
                {
                    newMults.ChangeArray1()[num_knots] = mults1[ii] + degree2;
                }
                num_knots += 1;
            }
            num_knots -= 1;
            newMults.ChangeArray1()[1] = degree + 1;
            newMults.ChangeArray1()[num_knots] = degree + 1;
            index = 0;

            for (ii = 1; ii <= num_knots; ii++)
            {
                index += newMults.Value(ii);
            }
            numPoles = index - degree - 1;
        }
        else
        {
            degree = degree1 + degree2;
            num_knots = 2;
            newKnots = new NCollection_HArray1<double>(1, num_knots);
            newKnots.ChangeArray1()[1] = startValue;
            newKnots.ChangeArray1()[num_knots] = endValue;

            newMults = new NCollection_HArray1<int>(1, num_knots);
            newMults.ChangeArray1()[1] = degree + 1;
            newMults.ChangeArray1()[num_knots] = degree + 1;
            numPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(degree, false, newMults.Array1());
        }
    }
}
