// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Types.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The result structs are C++ aggregates with public data members: they keep public fields. Their C++ default
// status is NotConverged, so ported code writes `new T()`; `default(T)` has Status OK (the enum's 0).
// std::optional<double> is double?, std::optional<math_Vector> a nullable reference, size_t is int.

/// <summary>Modern math solver types and result structures.</summary>
public static partial class MathUtils
{
    /// <summary>Computation status for all math solvers.</summary>
    public enum Status
    {
        /// <summary>Computation successful, solution found.</summary>
        OK,
        /// <summary>Did not converge within tolerance.</summary>
        NotConverged,
        /// <summary>Maximum iterations reached without convergence.</summary>
        MaxIterations,
        /// <summary>Numerical issue (overflow, NaN, division by zero, etc.).</summary>
        NumericalError,
        /// <summary>Invalid input parameters.</summary>
        InvalidInput,
        /// <summary>Infinite number of solutions (degenerate case).</summary>
        InfiniteSolutions,
        /// <summary>No solution exists.</summary>
        NoSolution,
        /// <summary>Matrix not positive definite (for Cholesky, Newton, etc.).</summary>
        NotPositiveDefinite,
        /// <summary>Matrix is singular or nearly singular.</summary>
        Singular,
        /// <summary>Search direction is not a descent direction for merit function.</summary>
        NonDescentDirection,
    }

    /// <summary>Result for scalar (1D) root finding and minimization.</summary>
    public struct ScalarResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Number of iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Found root or minimum location.</summary>
        public double? Root;
        /// <summary>Function value at root/minimum.</summary>
        public double? Value;
        /// <summary>Derivative at root (if computed).</summary>
        public double? Derivative;

        public ScalarResult()
        {
        }

        /// <summary>Returns true if computation succeeded.</summary>
        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(ScalarResult theResult) => theResult.IsDone();

        public static bool operator true(ScalarResult theResult) => theResult.IsDone();

        public static bool operator false(ScalarResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for polynomial root finding: up to 4 real roots (for quartic equations).</summary>
    public struct PolyResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Number of real roots found.</summary>
        public int NbRoots = 0;
        /// <summary>Real roots (sorted).</summary>
        public std_array4<double> Roots;

        public PolyResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(PolyResult theResult) => theResult.IsDone();

        public static bool operator true(PolyResult theResult) => theResult.IsDone();

        public static bool operator false(PolyResult theResult) => !theResult.IsDone();

        /// <summary>Access root by index (0-based).</summary>
        public readonly double this[int theIndex] => Roots[theIndex];
    }

    /// <summary>Result for N-dimensional optimization and system solving.</summary>
    public struct VectorResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Number of iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Solution vector (set by solver on success).</summary>
        public math_Vector? Solution;
        /// <summary>Function value at solution (if computed).</summary>
        public double? Value;
        /// <summary>Gradient at solution (if computed).</summary>
        public math_Vector? Gradient;
        /// <summary>Jacobian at solution (if computed).</summary>
        public math_Matrix? Jacobian;

        public VectorResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(VectorResult theResult) => theResult.IsDone();

        public static bool operator true(VectorResult theResult) => theResult.IsDone();

        public static bool operator false(VectorResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for linear system solving (Ax = b).</summary>
    public struct LinearResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Solution vector X in AX = B (set by solver).</summary>
        public math_Vector? Solution;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public LinearResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(LinearResult theResult) => theResult.IsDone();

        public static bool operator true(LinearResult theResult) => theResult.IsDone();

        public static bool operator false(LinearResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for multiple linear systems solving (AX = B with matrix RHS).</summary>
    public struct LinearMultipleResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Solution matrix X in AX = B (set by solver).</summary>
        public math_Matrix? Solutions;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public LinearMultipleResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(LinearMultipleResult theResult) => theResult.IsDone();

        public static bool operator true(LinearMultipleResult theResult) => theResult.IsDone();

        public static bool operator false(LinearMultipleResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for eigenvalue/eigenvector computation.</summary>
    public struct EigenResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Number of iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Computed eigenvalues (set by solver).</summary>
        public math_Vector? EigenValues;
        /// <summary>Computed eigenvectors (set by solver).</summary>
        public math_Matrix? EigenVectors;

        public EigenResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(EigenResult theResult) => theResult.IsDone();

        public static bool operator true(EigenResult theResult) => theResult.IsDone();

        public static bool operator false(EigenResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for matrix decomposition (LU, SVD, QR).</summary>
    public struct DecompResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Lower triangular (LU) or left singular vectors (SVD).</summary>
        public math_Matrix? L;
        /// <summary>Upper triangular (LU) or right singular vectors (SVD).</summary>
        public math_Matrix? U;
        /// <summary>Diagonal elements or singular values.</summary>
        public math_Vector? D;
        /// <summary>Matrix determinant (if computed).</summary>
        public double? Determinant;

        public DecompResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(DecompResult theResult) => theResult.IsDone();

        public static bool operator true(DecompResult theResult) => theResult.IsDone();

        public static bool operator false(DecompResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for numerical integration.</summary>
    public struct IntegResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Number of adaptive iterations.</summary>
        public int NbIterations = 0;
        /// <summary>Total number of quadrature points used.</summary>
        public int NbPoints = 0;
        /// <summary>Computed integral value.</summary>
        public double? Value;
        /// <summary>Estimated absolute error (if computed).</summary>
        public double? AbsoluteError;
        /// <summary>Estimated relative error (if computed).</summary>
        public double? RelativeError;

        public IntegResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(IntegResult theResult) => theResult.IsDone();

        public static bool operator true(IntegResult theResult) => theResult.IsDone();

        public static bool operator false(IntegResult theResult) => !theResult.IsDone();
    }

    /// <summary>Result for matrix inverse computation.</summary>
    public struct InverseResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Computed inverse matrix.</summary>
        public math_Matrix? Inverse;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public InverseResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(InverseResult theResult) => theResult.IsDone();

        public static bool operator true(InverseResult theResult) => theResult.IsDone();

        public static bool operator false(InverseResult theResult) => !theResult.IsDone();
    }
}
