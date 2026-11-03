// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_NewtonTypes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Occt;

namespace Kerf.Engine.Numerics.Systems;

// NewtonResultN<N> and NewtonBoundsN<N> are templates on an int, which C# generics can't express. OCCT
// instantiates them for N = 2, 3 and 4 (Solve2D, Solve3D, Solve4D); each instantiation is a struct named after
// the template and its argument: NewtonResultN2, NewtonBoundsN3, ... Their std::array members are std_arrayN.

/// <summary>Nonlinear systems: Newton, Levenberg-Marquardt and specialised small-dimension Newton solvers.</summary>
public static partial class MathSys
{
    /// <summary>Result of the 2-dimensional Newton solver (C++'s <c>NewtonResultN&lt;2&gt;</c>).</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new NewtonResultN2()`.</remarks>
    public struct NewtonResultN2
    {
        /// <summary>Final solver status.</summary>
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Solution estimate.</summary>
        public Array2<double> X;
        /// <summary>Performed iterations.</summary>
        public int NbIterations = 0;
        /// <summary>Final residual norm ||F||.</summary>
        public double ResidualNorm = 0.0;
        /// <summary>Final step norm ||dX||.</summary>
        public double StepNorm = 0.0;

        public NewtonResultN2()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(NewtonResultN2 result) => result.IsDone;

        public static bool operator true(NewtonResultN2 result) => result.IsDone;

        public static bool operator false(NewtonResultN2 result) => !result.IsDone;
    }

    /// <summary>Result of the 3-dimensional Newton solver (C++'s <c>NewtonResultN&lt;3&gt;</c>).</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new NewtonResultN3()`.</remarks>
    public struct NewtonResultN3
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        public Array3<double> X;
        public int NbIterations = 0;
        public double ResidualNorm = 0.0;
        public double StepNorm = 0.0;

        public NewtonResultN3()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(NewtonResultN3 result) => result.IsDone;

        public static bool operator true(NewtonResultN3 result) => result.IsDone;

        public static bool operator false(NewtonResultN3 result) => !result.IsDone;
    }

    /// <summary>Result of the 4-dimensional Newton solver (C++'s <c>NewtonResultN&lt;4&gt;</c>).</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new NewtonResultN4()`.</remarks>
    public struct NewtonResultN4
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        public Array4<double> X;
        public int NbIterations = 0;
        public double ResidualNorm = 0.0;
        public double StepNorm = 0.0;

        public NewtonResultN4()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(NewtonResultN4 result) => result.IsDone;

        public static bool operator true(NewtonResultN4 result) => result.IsDone;

        public static bool operator false(NewtonResultN4 result) => !result.IsDone;
    }

    /// <summary>Box bounds for the 2-dimensional solver (C++'s <c>NewtonBoundsN&lt;2&gt;</c>).</summary>
    /// <remarks>HasBounds is true by default in C++, so ported code writes `new NewtonBoundsN2()`; `default` has no bounds.</remarks>
    public struct NewtonBoundsN2
    {
        /// <summary>Lower bounds.</summary>
        public Array2<double> Min;
        /// <summary>Upper bounds.</summary>
        public Array2<double> Max;
        /// <summary>True to apply bounds.</summary>
        public bool HasBounds = true;

        public NewtonBoundsN2()
        {
        }
    }

    /// <summary>Box bounds for the 3-dimensional solver (C++'s <c>NewtonBoundsN&lt;3&gt;</c>).</summary>
    /// <remarks>HasBounds is true by default in C++, so ported code writes `new NewtonBoundsN3()`; `default` has no bounds.</remarks>
    public struct NewtonBoundsN3
    {
        public Array3<double> Min;
        public Array3<double> Max;
        public bool HasBounds = true;

        public NewtonBoundsN3()
        {
        }
    }

    /// <summary>Box bounds for the 4-dimensional solver (C++'s <c>NewtonBoundsN&lt;4&gt;</c>).</summary>
    /// <remarks>HasBounds is true by default in C++, so ported code writes `new NewtonBoundsN4()`; `default` has no bounds.</remarks>
    public struct NewtonBoundsN4
    {
        public Array4<double> Min;
        public Array4<double> Max;
        public bool HasBounds = true;

        public NewtonBoundsN4()
        {
        }
    }

    /// <summary>Solver options for small-dimension Newton methods.</summary>
    public class NewtonOptions : global::Kerf.Engine.Numerics.Utilities.MathUtils.Config
    {
        /// <summary>Max step as ratio of largest domain size.</summary>
        public double MaxStepRatio = 0.5;
        /// <summary>Enable Armijo backtracking line search.</summary>
        public bool EnableLineSearch = true;
        /// <summary>Allow slight bounds extension.</summary>
        public bool AllowSoftBounds = false;
        /// <summary>Extension ratio for soft bounds.</summary>
        public double SoftBoundsExtension = 1.0e-4;

        /// <summary>Default constructor with strict residual/step tolerances for specialized Newton.</summary>
        public NewtonOptions()
        {
            Tolerance = 1.0e-10;
            XTolerance = 1.0e-16;
            FTolerance = 1.0e-10;
            MaxIterations = 100;
        }

        /// <summary>Copy, as C++'s copy construction.</summary>
        public NewtonOptions(NewtonOptions other)
            : base(other)
        {
            MaxStepRatio = other.MaxStepRatio;
            EnableLineSearch = other.EnableLineSearch;
            AllowSoftBounds = other.AllowSoftBounds;
            SoftBoundsExtension = other.SoftBoundsExtension;
        }
    }
}
