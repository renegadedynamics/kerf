// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// methods for 1 dimensional BSplines
public static partial class BSplCLib
{
    // OCCT's BSplCLib_DataContainer (BSplCLib_DataContainer_T<1> of BSplCLib_CurveComputation.pxx): stack
    // buffers for the poles, knots and derivatives of one span. The port allocates them with stackalloc.
    private const int DataContainer1_PolesSize = (BSplCLib_CurveComputation.THE_MAX_DEGREE + 1) * (1 + 1);
    private const int DataContainer1_KnotsSize = 2 * BSplCLib_CurveComputation.THE_MAX_DEGREE;
    private const int DataContainer1_DersSize = 1 * 4;

    /// <summary>
    /// Copies in <paramref name="LP"/> the poles (and weights, multiplying the poles: homogeneous coordinates)
    /// of the span starting at pole <paramref name="Index"/>, Degree + 1 of them, wrapping around the poles.
    /// </summary>
    public static void BuildEval(int Degree, int Index, NCollection_Array1<double> Poles,
                                 NCollection_Array1<double>? Weights, Span<double> LP)
    {
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();
        int i;
        int ip = PLower + Index - 1;
        double w;
        Span<double> pole = LP;
        int aPole = 0;
        if (Weights == null)
        {

            for (i = 0; i <= Degree; i++)
            {
                ip++;
                if (ip > PUpper)
                {
                    ip = PLower;
                }
                pole[aPole + 0] = Poles[ip];
                aPole += 1;
            }
        }
        else
        {

            for (i = 0; i <= Degree; i++)
            {
                ip++;
                if (ip > PUpper)
                {
                    ip = PLower;
                }
                pole[aPole + 1] = w = Weights[ip];
                pole[aPole + 0] = Poles[ip] * w;
                aPole += 2;
            }
        }
    }

    private static void PrepareEval(ref double u, ref int index, out int dim, out bool rational, int Degree,
                                    bool Periodic, NCollection_Array1<double> Poles,
                                    NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                                    NCollection_Array1<int>? Mults, Span<double> dc_poles, Span<double> dc_knots)
    {
        // Set the Index
        BSplCLib.LocateParameter(Degree, Knots, Mults, u, Periodic, ref index, out u);

        // make the knots
        BSplCLib.BuildKnots(Degree, index, Periodic, Knots, Mults, dc_knots);
        if (Mults == null)
        {
            index -= Knots.Lower() + Degree;
        }
        else
        {
            index = BSplCLib.PoleIndex(Degree, index, Periodic, Mults);
        }

        // check truly rational
        rational = (Weights != null);
        if (rational)
        {
            int WLower = Weights!.Lower() + index;
            rational = BSplCLib.IsRational(Weights, WLower, WLower + Degree);
        }

        // make the poles
        if (rational)
        {
            dim = 2;
            BSplCLib.BuildEval(Degree, index, Poles, Weights, dc_poles);
        }
        else
        {
            dim = 1;
            BSplCLib.BuildEval(Degree, index, Poles, BSplCLib.NoWeights(), dc_poles);
        }
    }

    /// <summary>The value at <paramref name="U"/> of a one-dimensional B-spline (rational if weights are given).</summary>
    public static void D0(double U, int Index, int Degree, bool Periodic, NCollection_Array1<double> Poles,
                          NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults, out double P)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        BSplCLib_CurveComputation.validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        PrepareEval(ref u, ref index, out dim, out rational, Degree, Periodic, Poles, Weights, Knots, Mults, dc_poles, dc_knots);
        BSplCLib.Eval(u, Degree, dc_knots, dim, dc_poles);
        if (rational)
        {
            P = dc_poles[0] / dc_poles[1];
        }
        else
        {
            P = dc_poles[0];
        }
    }

    /// <summary>The value and first derivative at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D1(double U, int Index, int Degree, bool Periodic, NCollection_Array1<double> Poles,
                          NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults, out double P, out double V)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        BSplCLib_CurveComputation.validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, Degree, Periodic, Poles, Weights, Knots, Mults, dc_poles, dc_knots);
        BSplCLib.Bohm(u, Degree, 1, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 1, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        P = result[0];
        V = result[1];
    }

    /// <summary>The value and first two derivatives at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D2(double U, int Index, int Degree, bool Periodic, NCollection_Array1<double> Poles,
                          NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults, out double P, out double V1, out double V2)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        BSplCLib_CurveComputation.validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, Degree, Periodic, Poles, Weights, Knots, Mults, dc_poles, dc_knots);
        BSplCLib.Bohm(u, Degree, 2, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 2, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        P = result[0];
        V1 = result[1];
        if (!rational && (Degree < 2))
        {
            V2 = 0.0;
        }
        else
        {
            V2 = result[2];
        }
    }

    /// <summary>The value and first three derivatives at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void D3(double U, int Index, int Degree, bool Periodic, NCollection_Array1<double> Poles,
                          NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults, out double P, out double V1, out double V2, out double V3)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        BSplCLib_CurveComputation.validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainer1_DersSize];
        PrepareEval(ref u, ref index, out dim, out rational, Degree, Periodic, Poles, Weights, Knots, Mults, dc_poles, dc_knots);
        BSplCLib.Bohm(u, Degree, 3, dc_knots, dim, dc_poles);
        Span<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 3, 1, dc_poles, dc_ders);
            result = dc_ders;
        }
        P = result[0];
        V1 = result[1];
        if (!rational && (Degree < 2))
        {
            V2 = 0.0;
        }
        else
        {
            V2 = result[2];
        }
        if (!rational && (Degree < 3))
        {
            V3 = 0.0;
        }
        else
        {
            V3 = result[3];
        }
    }

    /// <summary>The derivative of order <paramref name="N"/> at <paramref name="U"/> of a one-dimensional B-spline.</summary>
    public static void DN(double U, int N, int Index, int Degree, bool Periodic, NCollection_Array1<double> Poles,
                          NCollection_Array1<double>? Weights, NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults, out double VN)
    {
        int dim, index = Index;
        double u = U;
        bool rational;
        BSplCLib_CurveComputation.validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainer1_PolesSize];
        Span<double> dc_knots = stackalloc double[DataContainer1_KnotsSize];
        PrepareEval(ref u, ref index, out dim, out rational, Degree, Periodic, Poles, Weights, Knots, Mults, dc_poles, dc_knots);
        BSplCLib.Bohm(u, Degree, N, dc_knots, dim, dc_poles);
        if (rational)
        {
            Span<double> v = stackalloc double[1];
            PLib.RationalDerivative(Degree, N, 1, dc_poles, v, false);
            VN = v[0];
        }
        else
        {
            if (N > Degree)
            {
                VN = 0.0;
            }
            else
            {
                VN = dc_poles[N];
            }
        }
    }

    /// <summary>
    /// Builds the banded matrix of the B-spline basis functions (of the derivative orders given by
    /// <paramref name="OrderArray"/>) at <paramref name="Parameters"/>, for interpolation. The matrix must have
    /// Parameters' row range and columns 1 to UpperBandWidth + LowerBandWidth + 1. Returns 0 on success.
    /// </summary>
    public static int BuildBSpMatrix(NCollection_Array1<double> Parameters, NCollection_Array1<int> OrderArray,
                                     NCollection_Array1<double> FlatKnots, int Degree, math_Matrix Matrix,
                                     out int UpperBandWidth, out int LowerBandWidth)
    {
        NCollection_Array1<int> ContactOrderArray = OrderArray;
        const int aMaxOrder = 25 + 1; // BSplCLib::MaxDegree() + 1
        int anOrder = Degree + 1;
        UpperBandWidth = Degree;
        LowerBandWidth = Degree;
        int aBandWidth = UpperBandWidth + LowerBandWidth + 1;

        if (Matrix.LowerRow() != Parameters.Lower() || Matrix.UpperRow() != Parameters.Upper()
            || Matrix.LowerCol() != 1 || Matrix.UpperCol() != aBandWidth)
        {
            return 1;
        }

        // A stack buffer in OCCT; math_Matrix needs an array to share
        double[] aBasisBuf = new double[aMaxOrder * aMaxOrder];
        math_Matrix aBSplineBasis = new(aBasisBuf, 1, aMaxOrder, 1, aMaxOrder);

        // Zero the entire matrix once instead of per-row zero-fill loops.
        Matrix.Init(0.0);
        int aMatLRow = Matrix.LowerRow();
        int aMatNCols = Matrix.ColNumber();
        Span<double> aMatData = Matrix.AsSpan();

        for (int i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            int aFirstNonZeroIndex = 0;
            int aContactOrder = ContactOrderArray[i];
            int anErrorCode = BSplCLib.EvalBsplineBasis(aContactOrder,
                                                        anOrder,
                                                        FlatKnots,
                                                        Parameters[i],
                                                        out aFirstNonZeroIndex,
                                                        aBSplineBasis);
            if (anErrorCode != 0)
            {
                return 2;
            }

            int anIndex = LowerBandWidth + 1 + aFirstNonZeroIndex - i;
            Span<double> aRowData = aMatData.Slice((i - aMatLRow) * aMatNCols);
            ReadOnlySpan<double> aBasisSrc = aBasisBuf.AsSpan(aContactOrder * aMaxOrder);
            for (int j = 0; j < anOrder; j++)
            {
                aRowData[anIndex + j - 1] = aBasisSrc[j];
            }
        }

        return 0;
    }

    /// <summary>
    /// Factors the banded matrix in LU form, with a banded storage of the L components. Returns 0, or 1 with
    /// <paramref name="PivotIndexProblem"/> the row of a null pivot. Not to be used for totally positive matrices
    /// (B-spline matrices built at the Schoenberg points).
    /// </summary>
    public static int FactorBandedMatrix(math_Matrix Matrix, int UpperBandWidth, int LowerBandWidth,
                                         out int PivotIndexProblem)
    {
        int aBandWidth = UpperBandWidth + LowerBandWidth + 1;
        PivotIndexProblem = 0;

        int aLRow = Matrix.LowerRow();
        int aNCols = Matrix.ColNumber();
        // &Matrix(aLRow, 1): the storage starts at column LowerCol
        Span<double> aData = Matrix.AsSpan();
        int aBase = 1 - Matrix.LowerCol();

        for (int i = aLRow + 1; i <= Matrix.UpperRow(); i++)
        {
            int aMinIndex = (LowerBandWidth - i + 2 >= 1 ? LowerBandWidth - i + 2 : 1);
            int aRowI = aBase + (i - aLRow) * aNCols;

            for (int j = aMinIndex; j <= LowerBandWidth; j++)
            {
                int anIndex = i - LowerBandWidth + j - 1;
                int aRowIdx = aBase + (anIndex - aLRow) * aNCols;
                double aPivot = aData[aRowIdx + LowerBandWidth];
                if (Math.Abs(aPivot) <= RealSmall())
                {
                    PivotIndexProblem = anIndex;
                    return 1;
                }

                double anInverse = -1.0 / aPivot;
                aData[aRowI + j - 1] = aData[aRowI + j - 1] * anInverse;
                int aMaxIndex = aBandWidth + anIndex - i;

                for (int k = j + 1; k <= aMaxIndex; k++)
                {
                    aData[aRowI + k - 1] += aData[aRowI + j - 1] * aData[aRowIdx + k + i - anIndex - 1];
                }
            }
        }

        return 0;
    }

    /// <summary>
    /// Evaluates the B-spline basis at <paramref name="Parameter"/> up to <paramref name="DerivativeOrder"/>:
    /// BsplineBasis(1, n) is the value of the n-th non-vanishing basis function (index FirstNonZeroBsplineIndex +
    /// n - 1), BsplineBasis(k + 1, n) its k-th derivative. The matrix needs rows and columns from 1, at least
    /// DerivativeOrder + 1 rows and Order columns. Returns 0 on success.
    /// </summary>
    public static int EvalBsplineBasis(int DerivativeOrder, int Order, NCollection_Array1<double> FlatKnots,
                                       double Parameter, out int FirstNonZeroBsplineIndex, math_Matrix BsplineBasis,
                                       bool isPeriodic = false)
    {
        int DerivativeRequest = DerivativeOrder;
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
        FirstNonZeroBsplineIndex = 0;
        int aLocalRequest = DerivativeRequest;
        if (DerivativeRequest >= Order)
        {
            aLocalRequest = Order - 1;
        }

        if (BsplineBasis.LowerCol() != 1 || BsplineBasis.UpperCol() < Order
            || BsplineBasis.LowerRow() != 1 || BsplineBasis.UpperRow() <= aLocalRequest)
        {
            return 1;
        }

        int aNumPoles = FlatKnots.Upper() - FlatKnots.Lower() + 1 - Order;
        int ii = 0;
        double aNewParam = 0.0;
        BSplCLib.LocateParameter(Order - 1,
                                 FlatKnots,
                                 Parameter,
                                 isPeriodic,
                                 Order,
                                 aNumPoles + 1,
                                 out ii,
                                 out aNewParam);

        FirstNonZeroBsplineIndex = ii - Order + 1;

        // Spans over BsplineBasis and FlatKnots bypass the accessors in the tight loops
        int aBasisNCols = BsplineBasis.ColNumber();
        Span<double> aBasisData = BsplineBasis.AsSpan();
        ReadOnlySpan<double> aKnotsData = FlatKnots.AsSpan();
        double aResolution = gp.Resolution();
        ii -= FlatKnots.Lower(); // rebase to zero-based indexing into aKnotsData

        aBasisData[0] = 1.0;
        aLocalRequest = DerivativeRequest;
        if (DerivativeRequest >= Order)
        {
            aLocalRequest = Order - 1;
        }

        for (int qq = 2; qq <= Order - aLocalRequest; qq++)
        {
            aBasisData[qq - 1] = 0.0;

            for (int pp = 1; pp <= qq - 1; pp++)
            {
                double aScale = aKnotsData[ii + pp] - aKnotsData[ii - qq + pp + 1];
                if (Math.Abs(aScale) < aResolution)
                {
                    return 2;
                }

                double aFactor = (Parameter - aKnotsData[ii - qq + pp + 1]) / aScale;
                double aSaved = aFactor * aBasisData[pp - 1];
                aBasisData[pp - 1] *= (1.0 - aFactor);
                aBasisData[pp - 1] += aBasisData[qq - 1];
                aBasisData[qq - 1] = aSaved;
            }
        }

        for (int qq = Order - aLocalRequest + 1; qq <= Order; qq++)
        {
            for (int pp = 1; pp <= qq - 1; pp++)
            {
                aBasisData[(Order - qq + 1) * aBasisNCols + (pp - 1)] = aBasisData[pp - 1];
            }
            aBasisData[qq - 1] = 0.0;

            for (int ss = Order - aLocalRequest + 1; ss <= qq; ss++)
            {
                aBasisData[(Order - ss + 1) * aBasisNCols + (qq - 1)] = 0.0;
            }

            for (int pp = 1; pp <= qq - 1; pp++)
            {
                double aScale = aKnotsData[ii + pp] - aKnotsData[ii - qq + pp + 1];
                if (Math.Abs(aScale) < aResolution)
                {
                    return 2;
                }

                double anInverse = 1.0 / aScale;
                double aFactor = (Parameter - aKnotsData[ii - qq + pp + 1]) * anInverse;
                double aSaved = aFactor * aBasisData[pp - 1];
                aBasisData[pp - 1] *= (1.0 - aFactor);
                aBasisData[pp - 1] += aBasisData[qq - 1];
                aBasisData[qq - 1] = aSaved;
                double aLocalInverse = (double)(qq - 1) * anInverse;

                for (int ss = Order - aLocalRequest + 1; ss <= qq; ss++)
                {
                    Span<double> aRowS = aBasisData.Slice((Order - ss + 1) * aBasisNCols);
                    aSaved = aLocalInverse * aRowS[pp - 1];
                    aRowS[pp - 1] *= -aLocalInverse;
                    aRowS[pp - 1] += aRowS[qq - 1];
                    aRowS[qq - 1] = aSaved;
                }
            }
        }

        return 0;
    }

    /// <summary>
    /// Dimension-free version of MovePointAndTangent: moves the B-spline (poles of dimension
    /// <paramref name="ArrayDimension"/>) so that its point at <paramref name="U"/> moves by
    /// <paramref name="Delta"/> and its derivative by <paramref name="DeltaDerivative"/>.
    /// <paramref name="StartingCondition"/> and <paramref name="EndingCondition"/> say how many derivatives at the
    /// ends must not move (-1: the end itself may move). <paramref name="ErrorStatus"/> is 0, 1 if there are not
    /// enough poles for the conditions, or 2 if the conditions are invalid.
    /// </summary>
    public static void MovePointAndTangent(double U, int ArrayDimension, ReadOnlySpan<double> Delta,
                                           ReadOnlySpan<double> DeltaDerivative, double Tolerance, int Degree,
                                           int StartingCondition, int EndingCondition, ReadOnlySpan<double> Poles,
                                           NCollection_Array1<double>? Weights, NCollection_Array1<double> FlatKnots,
                                           Span<double> NewPoles, out int ErrorStatus)
    {
        int num_poles, num_knots, ii, jj, conditions, start_num_poles, end_num_poles, index, start_index,
          end_index, other_index, type, order;

        double new_parameter, value, divide, end_value, start_value;
        ReadOnlySpan<double> poles_array, delta_array, derivatives_array, weights_array;
        Span<double> new_poles_array;

        ErrorStatus = 0;
        weights_array = default;
        if (Weights != null)
        {
            weights_array = Weights.AsSpan();
        }

        poles_array = Poles;
        new_poles_array = NewPoles;
        delta_array = Delta;
        derivatives_array = DeltaDerivative;
        order = Degree + 1;
        num_knots = FlatKnots.Length();
        num_poles = num_knots - order;
        conditions = StartingCondition + EndingCondition + 4;
        //
        // check validity of input data
        //
        if (StartingCondition >= -1 && StartingCondition <= Degree && EndingCondition >= -1
            && EndingCondition <= Degree && conditions <= num_poles)
        {
            //
            // check the parameter is within bounds
            //
            start_index = FlatKnots.Lower() + Degree;
            end_index = FlatKnots.Upper() - Degree;
            //
            //  check if there is enough room to move the poles
            //
            bool aConditions = true;
            if (StartingCondition == -1)
            {
                aConditions = aConditions && (FlatKnots[start_index] <= U);
            }
            else
            {
                aConditions = aConditions && (FlatKnots[start_index] + Tolerance < U);
            }
            if (EndingCondition == -1)
            {
                aConditions = aConditions && (FlatKnots[end_index] >= U);
            }
            else
            {
                aConditions = aConditions && (FlatKnots[end_index] - Tolerance > U);
            }

            if (aConditions)
            {
                //
                // build 2 auxiliary functions
                //
                NCollection_Array1<double> schoenberg_points = new(1, num_poles);
                NCollection_Array1<double> first_function = new(1, num_poles);
                NCollection_Array1<double> second_function = new(1, num_poles);

                BuildSchoenbergPoints(Degree, FlatKnots, schoenberg_points);
                start_index = StartingCondition + 2;
                end_index = num_poles - EndingCondition - 1;
                LocateParameter(schoenberg_points,
                                U,
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
                else if (U - FlatKnots[index] < FlatKnots[index + 1] - U)
                {
                    other_index = index - 1;
                }
                else
                {
                    other_index = index + 1;
                }
                type = 3;

                start_num_poles = StartingCondition + 2;
                end_num_poles = num_poles - EndingCondition - 1;
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

                extrap_mode[0] = Degree;
                extrap_mode[1] = Degree;
                if (Weights != null)
                {
                    //
                    // evaluate in homogenised form
                    //
                    Eval(U,
                         periodic_flag,
                         derivative_request,
                         extrap_mode,
                         Degree,
                         FlatKnots,
                         dimension,
                         first_function.AsSpan(),
                         weights_array,
                         results.Slice(0, 2),
                         weights_results.Slice(0, 2));

                    Eval(U,
                         periodic_flag,
                         derivative_request,
                         extrap_mode,
                         Degree,
                         FlatKnots,
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
                        PLib.RationalDerivatives(1, 1, results.Slice(2 * ii), weights_results.Slice(2 * ii), results.Slice(2 * ii));
                    }
                }
                else
                {
                    Eval(U, false, 1, extrap_mode, Degree, FlatKnots, 1, first_function.AsSpan(), results.Slice(0, 2));

                    Eval(U, false, 1, extrap_mode, Degree, FlatKnots, 1, second_function.AsSpan(), results.Slice(2, 2));
                }
                gp_Mat2d a_matrix = new();

                for (ii = 0; ii < 2; ii++)
                {

                    for (jj = 0; jj < 2; jj++)
                    {
                        a_matrix.SetValue(ii + 1, jj + 1, results[2 * ii + jj]);
                    }
                }
                a_matrix.Invert();
                // math_Vector(0, ArrayDimension - 1) in OCCT, which keeps small vectors on the stack
                Span<double> the_a_vector = ArrayDimension <= 128 ? stackalloc double[ArrayDimension] : new double[ArrayDimension];
                Span<double> the_b_vector = ArrayDimension <= 128 ? stackalloc double[ArrayDimension] : new double[ArrayDimension];

                for (ii = 0; ii < ArrayDimension; ii++)
                {
                    the_a_vector[ii] =
                      a_matrix.Value(1, 1) * delta_array[ii] + a_matrix.Value(2, 1) * derivatives_array[ii];
                    the_b_vector[ii] =
                      a_matrix.Value(1, 2) * delta_array[ii] + a_matrix.Value(2, 2) * derivatives_array[ii];
                }
                index = 0;

                for (ii = 0; ii < num_poles; ii++)
                {

                    for (jj = 0; jj < ArrayDimension; jj++)
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
                ErrorStatus = 1;
            }
        }
        else
        {
            ErrorStatus = 2;
        }
    }

    /// <summary>
    /// Multiplies the vectorial B-spline F(t) (poles [1..NumPoles][1..PolesDimension]) by the function a(t):
    /// interpolates a(t) * F(t) at the Schoenberg points of the B-spline of degree <paramref name="NewDegree"/> on
    /// <paramref name="FlatKnots"/>. The caller ensures a(t) * F(t) is such a B-spline and that a(t) has F's range;
    /// nothing is checked. <paramref name="theStatus"/> is 0, 1 if the function failed, or the pivot index of a
    /// failed interpolation.
    /// </summary>
    public static void FunctionMultiply(BSplCLib_EvaluatorFunction Function, int BSplineDegree,
                                        NCollection_Array1<double> BSplineFlatKnots, int PolesDimension,
                                        ReadOnlySpan<double> Poles, NCollection_Array1<double> FlatKnots,
                                        int NewDegree, Span<double> NewPoles, out int theStatus)
    {
        BSplCLib_EvaluatorFunction FunctionPtr = Function;
        Span<double> anArrayOfPoles = NewPoles;
        int aNumNewPoles = FlatKnots.Length() - NewDegree - 1;
        Span<double> aStartEnd = [FlatKnots[NewDegree + 1], FlatKnots[aNumNewPoles + 1]];

        NCollection_Array1<double> aParameters = new(1, aNumNewPoles);
        NCollection_Array1<int> aContactOrderArray = new(1, aNumNewPoles);
        NCollection_Array1<double> aNewPolesArray = new(1, aNumNewPoles * PolesDimension);

        Span<double> anArrayOfNewPoles = aNewPolesArray.AsSpan();
        BuildSchoenbergPoints(NewDegree, FlatKnots, aParameters);

        if (aParameters[1] < aStartEnd[0])
        {
            aParameters[1] = aStartEnd[0];
        }
        if (aParameters[aNumNewPoles] > aStartEnd[1])
        {
            aParameters[aNumNewPoles] = aStartEnd[1];
        }

        // OCCT passes a single int where Eval reads two extrapolation modes (ExtrapMode[1] is read past it, from
        // uninitialised stack memory, but only when a parameter lies beyond the last knot); the port passes the
        // degree for both ends
        ReadOnlySpan<int> anExtrapMode = [BSplineDegree, BSplineDegree];
        int anIndex = 0;
        for (int i = 1; i <= aNumNewPoles; i++)
        {
            aContactOrderArray[i] = 0;
            FunctionPtr.Evaluate(aContactOrderArray[i], aStartEnd, aParameters[i], out double aResult, out int anErrorCode);
            if (anErrorCode != 0)
            {
                theStatus = 1;
                return;
            }

            Eval(aParameters[i],
                 false,
                 0,
                 anExtrapMode,
                 BSplineDegree,
                 BSplineFlatKnots,
                 PolesDimension,
                 Poles,
                 anArrayOfNewPoles[anIndex..]);

            for (int j = 0; j < PolesDimension; j++)
            {
                anArrayOfNewPoles[anIndex] *= aResult;
                anIndex += 1;
            }
        }

        Interpolate(NewDegree,
                    FlatKnots,
                    aParameters,
                    aContactOrderArray,
                    PolesDimension,
                    anArrayOfNewPoles,
                    out theStatus);

        for (int i = 0; i < aNumNewPoles * PolesDimension; i++)
        {
            anArrayOfPoles[i] = anArrayOfNewPoles[i];
        }
    }

    /// <summary>
    /// Composes the vectorial B-spline F(t) (poles [1..NumPoles][1..PolesDimension]) with the function a(t):
    /// interpolates F(a(t)) at the Schoenberg points of the B-spline of degree <paramref name="NewDegree"/> on
    /// <paramref name="FlatKnots"/>. The caller ensures F(a(t)) is such a B-spline and that a(t) maps FlatKnots'
    /// range onto BSplineFlatKnots'; nothing is checked. <paramref name="theStatus"/> is 0, 1 if the function
    /// failed, or the pivot index of a failed interpolation.
    /// </summary>
    public static void FunctionReparameterise(BSplCLib_EvaluatorFunction Function, int BSplineDegree,
                                              NCollection_Array1<double> BSplineFlatKnots, int PolesDimension,
                                              ReadOnlySpan<double> Poles, NCollection_Array1<double> FlatKnots,
                                              int NewDegree, Span<double> NewPoles, out int theStatus)
    {
        BSplCLib_EvaluatorFunction FunctionPtr = Function;
        Span<double> anArrayOfPoles = NewPoles;
        int aNumNewPoles = FlatKnots.Length() - NewDegree - 1;
        Span<double> aStartEnd = [FlatKnots[NewDegree + 1], FlatKnots[aNumNewPoles + 1]];

        NCollection_Array1<double> aParameters = new(1, aNumNewPoles);
        NCollection_Array1<int> aContactOrderArray = new(1, aNumNewPoles);
        NCollection_Array1<double> aNewPolesArray = new(1, aNumNewPoles * PolesDimension);

        Span<double> anArrayOfNewPoles = aNewPolesArray.AsSpan();
        BuildSchoenbergPoints(NewDegree, FlatKnots, aParameters);

        // OCCT passes a single int where Eval reads two extrapolation modes (ExtrapMode[1] is read past it, from
        // uninitialised stack memory, but only when a parameter lies beyond the last knot); the port passes the
        // degree for both ends
        ReadOnlySpan<int> anExtrapMode = [BSplineDegree, BSplineDegree];
        int anIndex = 0;
        for (int i = 1; i <= aNumNewPoles; i++)
        {
            aContactOrderArray[i] = 0;
            FunctionPtr.Evaluate(aContactOrderArray[i], aStartEnd, aParameters[i], out double aResult, out int anErrorCode);
            if (anErrorCode != 0)
            {
                theStatus = 1;
                return;
            }

            Eval(aResult,
                 false,
                 0,
                 anExtrapMode,
                 BSplineDegree,
                 BSplineFlatKnots,
                 PolesDimension,
                 Poles,
                 anArrayOfNewPoles[anIndex..]);
            anIndex += PolesDimension;
        }

        Interpolate(NewDegree,
                    FlatKnots,
                    aParameters,
                    aContactOrderArray,
                    PolesDimension,
                    anArrayOfNewPoles,
                    out theStatus);

        for (int i = 0; i < aNumNewPoles * PolesDimension; i++)
        {
            anArrayOfPoles[i] = anArrayOfNewPoles[i];
        }
    }

    /// <summary>
    /// <see cref="FunctionMultiply(BSplCLib_EvaluatorFunction, int, NCollection_Array1{double}, int, ReadOnlySpan{double}, NCollection_Array1{double}, int, Span{double}, out int)"/>
    /// for a one-dimensional B-spline. Throws <see cref="Standard_ConstructionError"/> if the pole counts don't
    /// match the knots.
    /// </summary>
    public static void FunctionMultiply(BSplCLib_EvaluatorFunction Function, int BSplineDegree,
                                        NCollection_Array1<double> BSplineFlatKnots, NCollection_Array1<double> Poles,
                                        NCollection_Array1<double> FlatKnots, int NewDegree,
                                        NCollection_Array1<double> NewPoles, out int theStatus)
    {
        int num_bspline_poles = BSplineFlatKnots.Length() - BSplineDegree - 1;
        int num_new_poles = FlatKnots.Length() - NewDegree - 1;

        if (Poles.Length() != num_bspline_poles || NewPoles.Length() != num_new_poles)
        {
            throw new Standard_ConstructionError();
        }
        Span<double> array_of_poles = Poles.AsSpan();
        Span<double> array_of_new_poles = NewPoles.AsSpan();
        BSplCLib.FunctionMultiply(Function,
                                  BSplineDegree,
                                  BSplineFlatKnots,
                                  1,
                                  array_of_poles,
                                  FlatKnots,
                                  NewDegree,
                                  array_of_new_poles,
                                  out theStatus);
    }

    /// <summary>
    /// <see cref="FunctionReparameterise(BSplCLib_EvaluatorFunction, int, NCollection_Array1{double}, int, ReadOnlySpan{double}, NCollection_Array1{double}, int, Span{double}, out int)"/>
    /// for a one-dimensional B-spline. Throws <see cref="Standard_ConstructionError"/> if the pole counts don't
    /// match the knots.
    /// </summary>
    public static void FunctionReparameterise(BSplCLib_EvaluatorFunction Function, int BSplineDegree,
                                              NCollection_Array1<double> BSplineFlatKnots,
                                              NCollection_Array1<double> Poles, NCollection_Array1<double> FlatKnots,
                                              int NewDegree, NCollection_Array1<double> NewPoles, out int theStatus)
    {
        int num_bspline_poles = BSplineFlatKnots.Length() - BSplineDegree - 1;
        int num_new_poles = FlatKnots.Length() - NewDegree - 1;

        if (Poles.Length() != num_bspline_poles || NewPoles.Length() != num_new_poles)
        {
            throw new Standard_ConstructionError();
        }
        Span<double> array_of_poles = Poles.AsSpan();
        Span<double> array_of_new_poles = NewPoles.AsSpan();
        BSplCLib.FunctionReparameterise(Function,
                                        BSplineDegree,
                                        BSplineFlatKnots,
                                        1,
                                        array_of_poles,
                                        FlatKnots,
                                        NewDegree,
                                        array_of_new_poles,
                                        out theStatus);
    }

    /// <summary>
    /// Merges two knot vectors (of B-splines of degrees <paramref name="Degree1"/> and <paramref name="Degree2"/>),
    /// both reparametrised to [StartValue, EndValue], into the knots and multiplicities of their product, of degree
    /// Degree1 + Degree2, and its number of poles.
    /// </summary>
    public static void MergeBSplineKnots(double Tolerance, double StartValue, double EndValue, int Degree1,
                                         NCollection_Array1<double> Knots1, NCollection_Array1<int> Mults1,
                                         int Degree2, NCollection_Array1<double> Knots2,
                                         NCollection_Array1<int> Mults2, out int NumPoles,
                                         out NCollection_HArray1<double> NewKnots, out NCollection_HArray1<int> NewMults)
    {
        int ii, jj, continuity, set_mults_flag, degree, index, num_knots;
        if (StartValue < EndValue - Tolerance)
        {
            NCollection_Array1<double> knots1 = new(1, Knots1.Length());
            NCollection_Array1<double> knots2 = new(1, Knots2.Length());
            degree = Degree1 + Degree2;
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
            BSplCLib.Reparametrize(StartValue, EndValue, knots1);

            BSplCLib.Reparametrize(StartValue, EndValue, knots2);
            num_knots = 0;
            jj = 1;

            for (ii = 1; ii <= knots1.Length(); ii++)
            {

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] - Tolerance)
                {
                    jj += 1;
                    num_knots += 1;
                }

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] + Tolerance)
                {
                    jj += 1;
                }
                num_knots += 1;
            }
            NewKnots = new NCollection_HArray1<double>(1, num_knots);
            NewMults = new NCollection_HArray1<int>(1, num_knots);
            num_knots = 1;
            jj = 1;

            for (ii = 1; ii <= knots1.Length(); ii++)
            {

                // OCCT indexes Mults1 and Mults2 from 1, whatever their lower bounds (as it does knots1 and knots2)
                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] - Tolerance)
                {
                    NewKnots.ChangeArray1()[num_knots] = knots2[jj];
                    NewMults.ChangeArray1()[num_knots] = Mults2[jj] + Degree1;
                    jj += 1;
                    num_knots += 1;
                }
                set_mults_flag = 0;

                while (jj <= knots2.Length() && knots2[jj] <= knots1[ii] + Tolerance)
                {
                    continuity = Min(Degree1 - Mults1[ii], Degree2 - Mults2[jj]);
                    set_mults_flag = 1;
                    NewMults.ChangeArray1()[num_knots] = degree - continuity;
                    jj += 1;
                }

                NewKnots.ChangeArray1()[num_knots] = knots1[ii];
                if (set_mults_flag == 0)
                {
                    NewMults.ChangeArray1()[num_knots] = Mults1[ii] + Degree2;
                }
                num_knots += 1;
            }
            num_knots -= 1;
            NewMults.ChangeArray1()[1] = degree + 1;
            NewMults.ChangeArray1()[num_knots] = degree + 1;
            index = 0;

            for (ii = 1; ii <= num_knots; ii++)
            {
                index += NewMults.Value(ii);
            }
            NumPoles = index - degree - 1;
        }
        else
        {
            degree = Degree1 + Degree2;
            num_knots = 2;
            NewKnots = new NCollection_HArray1<double>(1, num_knots);
            NewKnots.ChangeArray1()[1] = StartValue;
            NewKnots.ChangeArray1()[num_knots] = EndValue;

            NewMults = new NCollection_HArray1<int>(1, num_knots);
            NewMults.ChangeArray1()[1] = degree + 1;
            NewMults.ChangeArray1()[num_knots] = degree + 1;
            NumPoles = BSplCLib.NbPoles(degree, false, NewMults.Array1());
        }
    }
}
