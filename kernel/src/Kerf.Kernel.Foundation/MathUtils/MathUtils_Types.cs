// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Types.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

namespace Kerf.Engine.Numerics.Utilities;

// The result structs are C++ aggregates with public data members: they keep public fields. Their C++ default
// status is NotConverged, so ported code writes `new T()`; `default(T)` has Status OK (the enum's 0).
// std::optional<double> is double?, std::optional<math_Vector> a nullable reference, size_t is int.

/// <summary>Modern math solver types and result structures.</summary>
internal static partial class MathUtils
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
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
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
        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(ScalarResult result) => result.IsDone;

        public static bool operator true(ScalarResult result) => result.IsDone;

        public static bool operator false(ScalarResult result) => !result.IsDone;
    }

    /// <summary>Result for polynomial root finding: up to 4 real roots (for quartic equations).</summary>
    public struct PolyResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Number of real roots found.</summary>
        public int NbRoots = 0;
        /// <summary>Real roots (sorted).</summary>
        public Array4<double> Roots;

        public PolyResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(PolyResult result) => result.IsDone;

        public static bool operator true(PolyResult result) => result.IsDone;

        public static bool operator false(PolyResult result) => !result.IsDone;

        /// <summary>Access root by index (0-based).</summary>
        public readonly double this[int index] => Roots[index];
    }

    /// <summary>Result for N-dimensional optimization and system solving.</summary>
    public struct VectorResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Number of iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Solution vector (set by solver on success).</summary>
        public math_Vector? Solution;
        /// <summary>Function value at solution (if computed).</summary>
        public double? Value;
        /// <summary>Gradient at solution (if computed).</summary>
        public math_Vector? Gradient;
        /// <summary>Jacobian at solution (if computed).</summary>
        public Matrix? Jacobian;

        public VectorResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(VectorResult result) => result.IsDone;

        public static bool operator true(VectorResult result) => result.IsDone;

        public static bool operator false(VectorResult result) => !result.IsDone;
    }

    /// <summary>Result for linear system solving (Ax = b).</summary>
    public struct LinearResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Solution vector X in AX = B (set by solver).</summary>
        public math_Vector? Solution;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public LinearResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(LinearResult result) => result.IsDone;

        public static bool operator true(LinearResult result) => result.IsDone;

        public static bool operator false(LinearResult result) => !result.IsDone;
    }

    /// <summary>Result for multiple linear systems solving (AX = B with matrix RHS).</summary>
    public struct LinearMultipleResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Solution matrix X in AX = B (set by solver).</summary>
        public Matrix? Solutions;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public LinearMultipleResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(LinearMultipleResult result) => result.IsDone;

        public static bool operator true(LinearMultipleResult result) => result.IsDone;

        public static bool operator false(LinearMultipleResult result) => !result.IsDone;
    }

    /// <summary>Result for eigenvalue/eigenvector computation.</summary>
    public struct EigenResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Number of iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Computed eigenvalues (set by solver).</summary>
        public math_Vector? EigenValues;
        /// <summary>Computed eigenvectors (set by solver).</summary>
        public Matrix? EigenVectors;

        public EigenResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(EigenResult result) => result.IsDone;

        public static bool operator true(EigenResult result) => result.IsDone;

        public static bool operator false(EigenResult result) => !result.IsDone;
    }

    /// <summary>Result for matrix decomposition (LU, SVD, QR).</summary>
    public struct DecompResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Lower triangular (LU) or left singular vectors (SVD).</summary>
        public Matrix? L;
        /// <summary>Upper triangular (LU) or right singular vectors (SVD).</summary>
        public Matrix? U;
        /// <summary>Diagonal elements or singular values.</summary>
        public math_Vector? D;
        /// <summary>Matrix determinant (if computed).</summary>
        public double? Determinant;

        public DecompResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(DecompResult result) => result.IsDone;

        public static bool operator true(DecompResult result) => result.IsDone;

        public static bool operator false(DecompResult result) => !result.IsDone;
    }

    /// <summary>Result for numerical integration.</summary>
    public struct IntegResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
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

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(IntegResult result) => result.IsDone;

        public static bool operator true(IntegResult result) => result.IsDone;

        public static bool operator false(IntegResult result) => !result.IsDone;
    }

    /// <summary>Result for matrix inverse computation.</summary>
    public struct InverseResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Computed inverse matrix.</summary>
        public Matrix? Inverse;
        /// <summary>Determinant of matrix (if computed).</summary>
        public double? Determinant;

        public InverseResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(InverseResult result) => result.IsDone;

        public static bool operator true(InverseResult result) => result.IsDone;

        public static bool operator false(InverseResult result) => !result.IsDone;
    }
}
