// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Config.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The configuration structs derive from each other in C++ (BoundedConfig : Config, PSOConfig : NDimConfig :
// Config, ...) and are passed as `const Config&`, so they are classes here, with public fields as in C++. A C++
// copy of a configuration (`Config aCopy = theConfig;`) is written `new(theConfig)`. A C++ default argument
// `const Config& theConfig = Config()` is a `Config? theConfig = null` parameter, defaulted in the body.

/// <summary>Modern math solver configuration structures.</summary>
public static partial class MathUtils
{
    /// <summary>Squared function tolerance for Newton iterations (matching math_FunctionSetRoot Eps).</summary>
    public const double THE_NEWTON_FTOL_SQ = 1.0e-32;

    /// <summary>Relative parametric step tolerance factor for Newton iterations.</summary>
    public const double THE_NEWTON_STEP_TOL_FACTOR = 1.0e-16;

    /// <summary>Default maximum iterations for Newton solvers.</summary>
    public const int THE_NEWTON_MAX_ITER = 100;

    // Newton 2D solver constants (MathSys_Newton2D)

    /// <summary>Determinant threshold below which a 2x2 Jacobian is considered singular.</summary>
    public const double THE_NEWTON2D_SINGULAR_DET = 1.0e-25;

    /// <summary>Squared gradient threshold below which a point is considered critical.</summary>
    public const double THE_NEWTON2D_CRITICAL_GRAD_SQ = 1.0e-60;

    /// <summary>Maximum step size as fraction of domain size for bounded Newton iterations.</summary>
    public const double THE_NEWTON2D_MAX_STEP_RATIO = 0.5;

    /// <summary>Domain extension factor for soft boundary handling.</summary>
    public const double THE_NEWTON2D_DOMAIN_EXT = 1.0e-4;

    /// <summary>Stagnation progress ratio: improvement required each iteration to avoid stagnation.</summary>
    public const double THE_NEWTON2D_STAGNATION_RATIO = 0.999;

    /// <summary>Number of iterations without progress before declaring stagnation.</summary>
    public const int THE_NEWTON2D_STAGNATION_COUNT = 3;

    /// <summary>Maximum line search backtracking iterations.</summary>
    public const int THE_NEWTON2D_LINE_SEARCH_MAX = 8;

    /// <summary>Relative step threshold below which line search is skipped.</summary>
    public const double THE_NEWTON2D_SKIP_LINESEARCH_SQ = 0.01;

    /// <summary>Tolerance relaxation factor for accepting stagnated solutions.</summary>
    public const double THE_NEWTON2D_STAGNATION_RELAX = 10.0;

    /// <summary>Function increase threshold triggering line search backtracking.</summary>
    public const double THE_NEWTON2D_BACKTRACK_TRIGGER = 1.5;

    /// <summary>Backtracking acceptance ratio for line search.</summary>
    public const double THE_NEWTON2D_BACKTRACK_ACCEPT = 1.2;

    /// <summary>Maximum relaxation factor for solutions at max iterations.</summary>
    public const double THE_NEWTON2D_MAXITER_RELAX = 10.0;

    // Hessian classification constants

    /// <summary>Relative tolerance for Hessian degeneracy detection.</summary>
    public const double THE_HESSIAN_DEGENERACY_REL = 1.0e-8;

    /// <summary>Absolute tolerance for Hessian degeneracy detection.</summary>
    public const double THE_HESSIAN_DEGENERACY_ABS = 1.0e-20;

    // Line search constants

    /// <summary>Armijo condition constant (sufficient decrease).</summary>
    public const double THE_ARMIJO_C1 = 1.0e-4;

    /// <summary>Configuration for iterative solvers: convergence criteria and iteration limits.</summary>
    public class Config
    {
        /// <summary>Maximum number of iterations allowed.</summary>
        public int MaxIterations = 100;
        /// <summary>General convergence tolerance.</summary>
        public double Tolerance = 1.0e-10;
        /// <summary>Tolerance for solution change |x_{n+1} - x_n|.</summary>
        public double XTolerance = 1.0e-10;
        /// <summary>Tolerance for function value |f(x)|.</summary>
        public double FTolerance = 1.0e-10;
        /// <summary>Minimum step size before declaring convergence or failure.</summary>
        public double StepMin = THE_EPSILON;

        public Config()
        {
        }

        /// <summary>Sets all tolerances to the same value.</summary>
        public Config(double theTolerance, int theMaxIter = 100)
        {
            MaxIterations = theMaxIter;
            Tolerance = theTolerance;
            XTolerance = theTolerance;
            FTolerance = theTolerance;
        }

        /// <summary>Copy, as C++'s copy construction.</summary>
        public Config(Config theOther)
        {
            MaxIterations = theOther.MaxIterations;
            Tolerance = theOther.Tolerance;
            XTolerance = theOther.XTolerance;
            FTolerance = theOther.FTolerance;
            StepMin = theOther.StepMin;
        }
    }

    /// <summary>Configuration for bounded 1D optimization and root finding.</summary>
    public class BoundedConfig : Config
    {
        /// <summary>Lower bound of search interval.</summary>
        public double LowerBound = -double.MaxValue;
        /// <summary>Upper bound of search interval.</summary>
        public double UpperBound = double.MaxValue;

        public BoundedConfig()
        {
        }

        public BoundedConfig(double theLower, double theUpper, double theTolerance = 1.0e-10, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
            LowerBound = theLower;
            UpperBound = theUpper;
        }

        public BoundedConfig(BoundedConfig theOther)
            : base(theOther)
        {
            LowerBound = theOther.LowerBound;
            UpperBound = theOther.UpperBound;
        }
    }

    /// <summary>Configuration for N-dimensional optimization with optional bounds.</summary>
    public class NDimConfig : Config
    {
        /// <summary>Whether to enforce bounds during optimization.</summary>
        public bool UseBounds = false;

        public NDimConfig()
        {
        }

        public NDimConfig(double theTolerance, int theMaxIter = 100, bool theUseBounds = false)
            : base(theTolerance, theMaxIter)
        {
            UseBounds = theUseBounds;
        }

        public NDimConfig(NDimConfig theOther)
            : base(theOther)
        {
            UseBounds = theOther.UseBounds;
        }
    }

    /// <summary>Configuration for numerical integration: quadrature order and adaptive refinement.</summary>
    public class IntegConfig
    {
        /// <summary>Initial number of quadrature points.</summary>
        public int InitialOrder = 15;
        /// <summary>Maximum quadrature order (Gauss-Legendre limit).</summary>
        public int MaxOrder = 61;
        /// <summary>Maximum adaptive subdivision iterations.</summary>
        public int MaxIterations = 100;
        /// <summary>Relative tolerance for error estimation.</summary>
        public double Tolerance = 1.0e-10;

        public IntegConfig()
        {
        }

        public IntegConfig(double theTolerance, int theMaxIter = 100)
        {
            MaxIterations = theMaxIter;
            Tolerance = theTolerance;
        }

        public IntegConfig(IntegConfig theOther)
        {
            InitialOrder = theOther.InitialOrder;
            MaxOrder = theOther.MaxOrder;
            MaxIterations = theOther.MaxIterations;
            Tolerance = theOther.Tolerance;
        }
    }

    /// <summary>Configuration for linear algebra solvers: singularity detection and pivoting.</summary>
    public class LinConfig
    {
        /// <summary>Tolerance for detecting singular matrices.</summary>
        public double SingularityTolerance = 1.0e-15;
        /// <summary>Whether to use pivoting for stability.</summary>
        public bool UsePivoting = true;

        public LinConfig()
        {
        }

        public LinConfig(LinConfig theOther)
        {
            SingularityTolerance = theOther.SingularityTolerance;
            UsePivoting = theOther.UsePivoting;
        }
    }
}
