// Port of: src/FoundationClasses/TKMath/math/math_DirectPolynomialRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_DirectPolynomialRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// All the real roots of a real polynomial of degree &lt;= 4, by direct algebraic methods: Ferrari's method for
/// quartics, Cardano's formula for cubics, and numerically stable formulas for quadratics and linear equations. The
/// roots are polished by Newton-Raphson iterations.
/// </summary>
internal class DirectPolynomialRoots
{
    // Algorithm References:
    //   - ALGORITHMES NUMERIQUES ANALYSE ET MISE EN OEUVRE, tome 2
    //     (equations et systemes non lineaires) J. VIGNES editions TECHNIP
    //   - Ferrari's method for quartic equations
    //   - Cardano's formula for cubic equations
    //   - Numerically stable quadratic formula
    //   - Newton-Raphson refinement for improved accuracy

    /// Threshold below which coefficients are considered effectively zero
    private const double ZERO_THRESHOLD = 1.0e-30;

    /// Machine epsilon for floating-point arithmetic precision
    private static readonly double MACHINE_EPSILON = RealEpsilon();

    /// Floating-point radix (base) for coefficient scaling operations
    private const double FLOATING_RADIX = 2.0;

    /// Inverse of natural logarithm of radix for exponent calculations
    private static readonly double INV_LOG_RADIX = 1.0 / Math.Log(2.0);

    /// Maximum number of Newton-Raphson refinement iterations
    private const int MAX_NEWTON_ITERATIONS = 10;

    // Below this share of the other, a quadratic factor's constant computed as a difference is within the other's rounding:
    // it has no digit left.
    private const double CancelledShare = 1.0e-15;

    /// Upper limit to prevent numerical overflow in computations
    private const double OVERFLOW_LIMIT = 1.0e+80;

    private bool _done;           //!< Computation status flag
    private bool _infiniteStatus; //!< Infinite solutions flag (0*x + 0 = 0)
    private int _nbSol;           //!< Number of real roots found (0 to 4)
    private readonly double[] _roots = new double[4]; //!< Array storing computed real roots
    private readonly bool _keepsSmallFactor;

    /// <summary>The real roots of Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 (Ferrari's method).</summary>
    public DirectPolynomialRoots(double a, double b, double c, double d, double e) : this(a, b, c, d, e, false)
    {
    }

    /// <summary>
    /// The real roots of Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 (Ferrari's method); with <paramref name="keepsSmallFactor"/>,
    /// a factor's constant that cancellation left without a digit is taken from the factors' product (decision 28631).
    /// </summary>
    public DirectPolynomialRoots(double a, double b, double c, double d, double e, bool keepsSmallFactor)
    {
        _infiniteStatus = false;
        _done = true;
        _keepsSmallFactor = keepsSmallFactor;
        Solve(a, b, c, d, e);
    }

    /// <summary>The real roots of Ax^3 + Bx^2 + Cx + D = 0 (Cardano's method with Vieta substitution).</summary>
    public DirectPolynomialRoots(double a, double b, double c, double d)
    {
        _done = true;
        _infiniteStatus = false;
        Solve(a, b, c, d);
    }

    /// <summary>The real roots of Ax^2 + Bx + C = 0.</summary>
    public DirectPolynomialRoots(double a, double b, double c)
    {
        _done = true;
        _infiniteStatus = false;
        Solve(a, b, c);
    }

    /// <summary>The real root of Ax + B = 0 (none if A = 0, B != 0; infinitely many if A = B = 0).</summary>
    public DirectPolynomialRoots(double a, double b)
    {
        _done = true;
        _infiniteStatus = false;
        Solve(a, b);
    }

    /// <summary>True if the computations succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>True if there are infinitely many roots (only for 0*x + 0 = 0).</summary>
    public bool InfiniteRoots() => _infiniteStatus;

    /// <summary>The number of real roots, each counted with its multiplicity. Throws <see cref="InfiniteSolutionsException"/> if infinitely many.</summary>
    public int SolutionCount
    {
        get
        {
            StdFail_InfiniteSolutions_Raise_if(_infiniteStatus, " ");
            return _nbSol;
        }
    }

    /// <summary>The root number <paramref name="index"/>, in 1..NbSolutions.</summary>
    public double Value(int index)
    {
        StdFail_InfiniteSolutions_Raise_if(_infiniteStatus, " ");
        Standard_RangeError_Raise_if((index < 1) || (index > _nbSol), " ");
        return _roots[index - 1];
    }

    public void Dump(TextWriter stream)
    {
        stream.Write("math_DirectPolynomialRoots ");

        if (!_done)
        {
            stream.Write(" Not Done \n");
        }
        else if (_infiniteStatus)
        {
            stream.Write(" Status = Infinity Roots \n");
        }
        else
        {
            stream.Write(" Status = Not Infinity Roots \n");
            stream.Write(" Number of solutions = " + _nbSol + "\n");
            for (int i = 1; i <= _nbSol; i++)
            {
                stream.Write(" Solution number " + i + " = " + _roots[i - 1].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    /// <summary>Solves the quartic Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 by Ferrari's method.</summary>
    protected void Solve(double a, double b, double c, double d, double e)
    {
        // Check for degree reduction
        if (ShouldReduceDegreeQuartic(a, b, c, d, e))
        {
            Solve(b, c, d, e);
            return;
        }

        // Normalize coefficients
        double A = b / a;
        double B = c / a;
        double C = d / a;
        double D = e / a;

        // Scale coefficients to avoid overflow/underflow
        ScaledCoefficients scaled = default;
        scaled.ScaleQuartic(A, B, C, D, D);

        // Solve Ferrari's resolvent cubic
        bool success = false;
        double y0 = SolveFerrariResolvent(scaled.A, scaled.B, scaled.C, scaled.D, ref success);

        if (!success)
        {
            _done = false;
            return;
        }

        // Factor into two quadratics
        QuarticFactorization factors = FactorQuarticViaFerrari(scaled.A, scaled.B, scaled.C, scaled.D, y0, _keepsSmallFactor);

        // Solve first quadratic
        DirectPolynomialRoots quadratic1 = new(1.0, factors.P1, factors.Q1);
        if (!quadratic1.IsDone)
        {
            _done = false;
            return;
        }

        // Solve second quadratic
        DirectPolynomialRoots quadratic2 = new(1.0, factors.P2, factors.Q2);
        if (!quadratic2.IsDone)
        {
            _done = false;
            return;
        }

        // Collect all roots
        _nbSol = quadratic1.SolutionCount + quadratic2.SolutionCount;
        int index = 0;

        for (int i = 0; i < quadratic1.SolutionCount; ++i)
        {
            _roots[index++] = quadratic1._roots[i];
        }

        for (int i = 0; i < quadratic2.SolutionCount; ++i)
        {
            _roots[index++] = quadratic2._roots[i];
        }

        // Apply inverse scaling and Newton-Raphson refinement to all roots
        ScaleAndRefineAllRoots(_roots, _nbSol, scaled.ScaleFactor, a, b, c, d, e);
    }

    /// <summary>Solves the cubic Ax^3 + Bx^2 + Cx + D = 0 by Cardano's method.</summary>
    protected void Solve(double a, double b, double c, double d)
    {
        // Check for degree reduction
        if (Math.Abs(a) <= ZERO_THRESHOLD)
        {
            Solve(b, c, d);
            return;
        }

        // Normalize coefficients
        double beta = b / a;
        double gamma = c / a;
        double del = d / a;

        // Scale to avoid overflow/underflow
        ScaledCoefficients scaled = default;
        scaled.ScaleCubic(beta, gamma, del, del);

        // Transform to depressed cubic: t^3 + Pt + Q = 0
        double p1 = scaled.B;
        double p2 = -(scaled.A * scaled.A) / 3.0;
        double p = p1 + p2;
        double ep = 5.0 * MACHINE_EPSILON * (Math.Abs(p1) + Math.Abs(p2));
        if (Math.Abs(p) <= ep)
        {
            p = 0.0;
        }

        double q1 = scaled.C;
        double q2 = -scaled.A * scaled.B / 3.0;
        double q3 = 2.0 * (scaled.A * scaled.A * scaled.A) / 27.0;
        double q = q1 + q2 + q3;
        double eq = 10.0 * MACHINE_EPSILON * (Math.Abs(q1) + Math.Abs(q2) + Math.Abs(q3));
        if (Math.Abs(q) <= eq)
        {
            q = 0.0;
        }

        // Check for overflow
        if (Math.Abs(p) > OVERFLOW_LIMIT)
        {
            _done = false;
            return;
        }

        // Compute discriminant
        double a1 = (p * p * p) / 27.0;
        double a2 = (q * q) / 4.0;
        double discr = a1 + a2;

        // Special handling for P < 0
        if (p < 0.0)
        {
            discr = ComputeSpecialDiscriminant(scaled.A, scaled.B, scaled.C, a1);
        }

        // Solve based on discriminant
        if (discr < 0.0)
        {
            // Three distinct real roots
            _nbSol = 3;
            SolveCubicThreeRealRoots(scaled.A, scaled.B, scaled.C, p, q, discr, _roots);
        }
        else if (discr > 0.0)
        {
            // One real root
            _nbSol = 1;
            SolveCubicOneRealRoot(scaled.A, scaled.C, p, q, discr, _roots);
        }
        else
        {
            // Multiple roots
            SolveCubicMultipleRoots(scaled.A, scaled.B, scaled.C, p, q, _roots, out _nbSol);
        }

        // Apply inverse scaling and Newton-Raphson refinement to all roots
        ScaleAndRefineAllRoots(_roots, _nbSol, scaled.ScaleFactor, a, b, c, d);
    }

    /// <summary>Solves the quadratic Ax^2 + Bx + C = 0 with numerically stable formulas.</summary>
    protected void Solve(double a, double b, double c)
    {
        // Check for degree reduction
        if (Math.Abs(a) <= ZERO_THRESHOLD)
        {
            Solve(b, c);
            return;
        }

        // Solve normalized quadratic x^2 + P*x + Q = 0
        double p = b / a;
        double q = c / a;

        // Compute discriminant with error bounds
        double epsD = 3.0 * MACHINE_EPSILON * (p * p + Math.Abs(4.0 * q));
        double discrim = p * p - 4.0 * q;

        if (Math.Abs(discrim) <= epsD)
        {
            discrim = 0.0;
        }

        if (discrim < 0.0)
        {
            // No real roots
            _nbSol = 0;
        }
        else if (discrim == 0.0)
        {
            // Double root
            _nbSol = 2;
            _roots[0] = -0.5 * p;
            _roots[0] = RefinePolynomialRoot(_roots[0], 1.0, p, q);
            _roots[1] = _roots[0];
        }
        else
        {
            // Two distinct real roots - use numerically stable formula
            _nbSol = 2;
            if (p > 0.0)
            {
                _roots[0] = -(p + Math.Sqrt(discrim)) / 2.0;
            }
            else
            {
                _roots[0] = -(p - Math.Sqrt(discrim)) / 2.0;
            }

            _roots[0] = RefinePolynomialRoot(_roots[0], 1.0, p, q);
            _roots[1] = q / _roots[0];
            _roots[1] = RefinePolynomialRoot(_roots[1], 1.0, p, q);
        }
    }

    /// <summary>Solves the linear equation Ax + B = 0.</summary>
    protected void Solve(double a, double b)
    {
        if (Math.Abs(a) <= ZERO_THRESHOLD)
        {
            if (Math.Abs(b) <= ZERO_THRESHOLD)
            {
                // 0 = 0: infinite solutions
                _infiniteStatus = true;
                return;
            }

            // 0*x + B = 0: no solution
            _nbSol = 0;
            return;
        }

        // Normal case: unique solution
        _nbSol = 1;
        _roots[0] = -b / a;
    }

    // The anonymous-namespace helpers of math_DirectPolynomialRoots.cxx.

    // Evaluates polynomial at point X using Horner's method
    // Coefficients: Poly[0]*X^(N-1) + Poly[1]*X^(N-2) + ... + Poly[N-1]
    private static double EvaluatePolynomial(int n, ReadOnlySpan<double> poly, double x)
    {
        double result = poly[0];
        for (int i = 1; i < n; ++i)
        {
            result = result * x + poly[i];
        }

        return result;
    }

    // Evaluates polynomial and its derivative simultaneously
    private static void EvaluatePolynomialWithDerivative(int n, ReadOnlySpan<double> poly, double x,
                                                         out double value, out double derivative)
    {
        value = poly[0] * x + poly[1];
        derivative = poly[0];

        for (int i = 2; i < n; ++i)
        {
            derivative = derivative * x + value;
            value = value * x + poly[i];
        }
    }

    // Refines root using Newton-Raphson iterations
    private static double RefineRoot(int n, ReadOnlySpan<double> poly, double initialGuess)
    {
        double value = 0.0;
        double derivative = 0.0;
        double solution = initialGuess;
        double initialValue = EvaluatePolynomial(n, poly, initialGuess);

        for (int iter = 1; iter < MAX_NEWTON_ITERATIONS; ++iter)
        {
            EvaluatePolynomialWithDerivative(n, poly, solution, out value, out derivative);

            if (Math.Abs(derivative) <= ZERO_THRESHOLD)
            {
                break;
            }

            double delta = -value / derivative;

            if (Math.Abs(delta) <= MACHINE_EPSILON * Math.Abs(solution))
            {
                break;
            }

            solution += delta;
        }

        // Return improved solution only if it's better
        return (Math.Abs(value) <= Math.Abs(initialValue)) ? solution : initialGuess;
    }

    // Polynomial root refinement for any number of coefficients (a variadic template in OCCT)
    private static double RefinePolynomialRoot(double initialGuess, params ReadOnlySpan<double> coeffs)
    {
        return RefineRoot(coeffs.Length, coeffs, initialGuess);
    }

    // Helper method to scale and refine all roots of a polynomial
    private static void ScaleAndRefineAllRoots(Span<double> roots, int nbRoots, double scaleFactor,
                                               params ReadOnlySpan<double> coeffs)
    {
        for (int i = 0; i < nbRoots; ++i)
        {
            roots[i] *= scaleFactor;
            roots[i] = RefinePolynomialRoot(roots[i], coeffs);
        }
    }

    // Computes base-2 exponent for coefficient scaling
    private static int ComputeBaseExponent(double value)
    {
        if (value > 1.0)
        {
            return (int)(Math.Log(value) * INV_LOG_RADIX);
        }
        else if (value < -1.0)
        {
            return (int)(-Math.Log(-value) * INV_LOG_RADIX);
        }

        return 0;
    }

    // Scale coefficients to avoid overflow/underflow
    private struct ScaledCoefficients
    {
        /// Scaled coefficient values
        public double A, B, C, D, E;

        /// Scale factor for converting roots back to original coordinate system
        public double ScaleFactor;

        public void ScaleQuartic(double a, double b, double c, double d, double e)
        {
            int exp = ComputeBaseExponent(e) / 4;
            ScaleFactor = Math.Pow(FLOATING_RADIX, exp);
            double scaleFactor2 = ScaleFactor * ScaleFactor;

            A = a / ScaleFactor;
            B = b / scaleFactor2;
            C = c / (scaleFactor2 * ScaleFactor);
            D = d / (scaleFactor2 * scaleFactor2);
            E = e / (scaleFactor2 * scaleFactor2);
        }

        public void ScaleCubic(double a, double b, double c, double d)
        {
            int exp = ComputeBaseExponent(d) / 3;
            ScaleFactor = Math.Pow(FLOATING_RADIX, exp);
            double scaleFactor2 = ScaleFactor * ScaleFactor;

            A = a / ScaleFactor;
            B = b / scaleFactor2;
            C = c / (scaleFactor2 * ScaleFactor);
            D = d; // Not used for cubic, but kept for consistency
        }
    }

    // Computes special discriminant for P < 0 cases with improved numerical stability
    private static double ComputeSpecialDiscriminant(double beta, double gamma, double del, double a1)
    {
        double sigma = beta * gamma / 3.0 - 2.0 * beta * beta * beta / 27.0;
        double psi = gamma * gamma * (4.0 * gamma - beta * beta) / 27.0;

        double d1;
        if (sigma >= 0.0)
        {
            d1 = sigma + 2.0 * Math.Sqrt(-a1);
        }
        else
        {
            d1 = sigma - 2.0 * Math.Sqrt(-a1);
        }

        double d2 = psi / d1;

        if (Math.Abs(del - d1) >= 18.0 * MACHINE_EPSILON * (Math.Abs(del) + Math.Abs(d1))
            && Math.Abs(del - d2) >= 24.0 * MACHINE_EPSILON * (Math.Abs(del) + Math.Abs(d2)))
        {
            return (del - d1) * (del - d2) / 4.0;
        }

        return 0.0;
    }

    // Solves cubic equation with three real roots using trigonometric method
    private static void SolveCubicThreeRealRoots(double beta, double gamma, double del, double p,
                                                 double q, double discr, Span<double> roots)
    {
        if (beta == 0.0 && q == 0.0)
        {
            // Special case: x^3 + Px = 0
            roots[0] = Math.Sqrt(-p);
            roots[1] = -roots[0];
            roots[2] = 0.0;
        }
        else
        {
            double sb = (beta >= 0.0) ? 1.0 : -1.0;
            double omega = Math.Atan(0.5 * q / Math.Sqrt(-discr));
            double sp3 = Math.Sqrt(-p / 3.0);
            double y1 = -2.0 * sb * sp3 * Math.Cos(M_PI / 6.0 - sb * omega / 3.0);

            roots[0] = -beta / 3.0 + y1;

            if (beta * q <= 0.0)
            {
                roots[1] = -beta / 3.0 + 2.0 * sp3 * Math.Sin(omega / 3.0);
            }
            else
            {
                // Alternative formula for better accuracy
                double dbg = del - beta * gamma;
                double sdbg = (dbg >= 0.0) ? 1.0 : -1.0;
                double den1 = 8.0 * beta * beta / 9.0 - 4.0 * beta * y1 / 3.0 - 2.0 * q / y1;
                double den2 = 2.0 * y1 * y1 - q / y1;
                roots[1] = dbg / den1 + sdbg * Math.Sqrt(-27.0 * discr) / den2;
            }

            // Use Vieta's formula for the third root
            roots[2] = -del / (roots[0] * roots[1]);
        }
    }

    // Solves cubic equation with one real root using Cardano's formula
    private static void SolveCubicOneRealRoot(double beta, double del, double p, double q,
                                              double discr, Span<double> roots)
    {
        double u = Math.Sqrt(discr) + Math.Abs(q / 2.0);
        u = (u >= 0.0) ? Math.Pow(u, 1.0 / 3.0) : -Math.Pow(Math.Abs(u), 1.0 / 3.0);

        double h;
        if (p >= 0.0)
        {
            h = u * u + p / 3.0 + (p / u) * (p / u) / 9.0;
        }
        else
        {
            h = u * Math.Abs(q) / (u * u - p / 3.0);
        }

        if (beta * q >= 0.0)
        {
            if (Math.Abs(h) <= RealSmall() && Math.Abs(q) <= RealSmall())
            {
                roots[0] = -beta / 3.0 - u + p / (3.0 * u);
            }
            else
            {
                roots[0] = -beta / 3.0 - q / h;
            }
        }
        else
        {
            roots[0] = -del / (beta * beta / 9.0 + h - beta * q / (3.0 * h));
        }
    }

    // Solves cubic equation with multiple roots (discriminant = 0)
    private static void SolveCubicMultipleRoots(double beta, double gamma, double del, double p,
                                                double q, Span<double> roots, out int nbRoots)
    {
        nbRoots = 3;
        double sq = (q >= 0.0) ? 1.0 : -1.0;
        double sp3 = Math.Sqrt(-p / 3.0);

        if (beta * q <= 0.0)
        {
            roots[0] = -beta / 3.0 + sq * sp3;
            roots[1] = roots[0];

            if (beta * q == 0.0)
            {
                roots[2] = -beta / 3.0 - 2.0 * sq * sp3;
            }
            else
            {
                roots[2] = -del / (roots[0] * roots[1]);
            }
        }
        else
        {
            roots[0] = -gamma / (beta + 3.0 * sq * sp3);
            roots[1] = roots[0];
            roots[2] = -beta / 3.0 - 2.0 * sq * sp3;
        }
    }

    // Determines if quartic should be reduced to lower degree
    private static bool ShouldReduceDegreeQuartic(double a, double b, double c, double d, double e)
    {
        if (Math.Abs(a) <= ZERO_THRESHOLD)
        {
            return true;
        }

        // Modified by jgv, 22.01.09 for numerical stability
        double maxCoeff = ZERO_THRESHOLD;
        maxCoeff = Max(maxCoeff, Math.Abs(b));
        maxCoeff = Max(maxCoeff, Math.Abs(c));
        maxCoeff = Max(maxCoeff, Math.Abs(d));
        maxCoeff = Max(maxCoeff, Math.Abs(e));

        if (maxCoeff > ZERO_THRESHOLD)
        {
            maxCoeff = Epsilon(100.0 * maxCoeff);
        }

        if (Math.Abs(a) <= maxCoeff)
        {
            double maxCoeff1000 = 1000.0 * maxCoeff;
            bool withA = false;

            if (Math.Abs(b) > ZERO_THRESHOLD && Math.Abs(b) <= maxCoeff1000)
            {
                withA = true;
            }

            if (Math.Abs(c) > ZERO_THRESHOLD && Math.Abs(c) <= maxCoeff1000)
            {
                withA = true;
            }

            if (Math.Abs(d) > ZERO_THRESHOLD && Math.Abs(d) <= maxCoeff1000)
            {
                withA = true;
            }

            if (Math.Abs(e) > ZERO_THRESHOLD && Math.Abs(e) <= maxCoeff1000)
            {
                withA = true;
            }

            return !withA;
        }

        return false;
    }

    // Constructs and solves Ferrari's resolvent cubic equation
    private static double SolveFerrariResolvent(double a, double b, double c, double d, ref bool success)
    {
        // Construct resolvent cubic: Y^3 + R3*Y^2 + S3*Y + T3 = 0
        double r3 = -b;
        double s3 = a * c - 4.0 * d;
        double t3 = d * (4.0 * b - a * a) - c * c;

        DirectPolynomialRoots cubicSolver = new(1.0, r3, s3, t3);

        if (!cubicSolver.IsDone)
        {
            success = false;
            return 0.0;
        }

        success = true;

        // Choose the largest root for numerical stability
        double y0 = cubicSolver.Value(1);
        for (int i = 2; i <= cubicSolver.SolutionCount; ++i)
        {
            if (cubicSolver.Value(i) > y0)
            {
                y0 = cubicSolver.Value(i);
            }
        }

        return y0;
    }

    // Structure for quartic factorization into two quadratic equations
    private struct QuarticFactorization
    {
        /// Linear coefficient of first quadratic factor
        public double P1;

        /// Constant coefficient of first quadratic factor
        public double Q1;

        /// Linear coefficient of second quadratic factor
        public double P2;

        /// Constant coefficient of second quadratic factor
        public double Q2;
    }

    // Factors quartic into two quadratics using Ferrari's method
    private static QuarticFactorization FactorQuarticViaFerrari(double a, double b, double c, double d, double y0, bool keepsSmallFactor)
    {
        QuarticFactorization factors;

        // Compute discriminant and parameters
        double discr = a * y0 * 0.5 - c;
        double sdiscr = (discr >= 0.0) ? 1.0 : -1.0;

        // Compute P0 and Q0 for the quadratic factors
        double p0 = a * a * 0.25 - b + y0;
        p0 = (p0 < 0.0) ? 0.0 : Math.Sqrt(p0);

        double q0 = y0 * y0 * 0.25 - d;

        // Handle the case where Q0^2 is very close to zero more robustly
        // This fixes the numerical precision issue on Linux/Windows vs macOS
        if (Math.Abs(q0) < 10 * MACHINE_EPSILON)
        {
            q0 = 0.0;
        }
        else
        {
            q0 = (q0 < 0.0) ? 0.0 : Math.Sqrt(q0);
        }

        // Form coefficients for the two quadratic equations
        double ademi = a * 0.5;
        double ydemi = y0 * 0.5;
        double sdiscrQ0 = sdiscr * q0;

        factors.P1 = ademi + p0;
        factors.Q1 = ydemi + sdiscrQ0;
        factors.P2 = ademi - p0;
        factors.Q2 = ydemi - sdiscrQ0;

        // The constants' product is d exactly, (y0/2 + q0)(y0/2 - q0) = y0^2/4 - q0^2. Where the two terms nearly cancel
        // in one of them, it keeps none of its digits, and a pair of roots near zero becomes a double root at zero: a circle
        // that crosses a cylinder 1e-6 radians either side of a touch saw no crossing (decision 28631). The small one is
        // taken from the product there, as the stable quadratic formula does. (p0 can be noise there too, which leaves one
        // of such a pair wrong; taking it from the cubic's term moved a Boolean against native, so it's left: decision 28631.)
        // Only for the trigonometric equations, whose Newton step then finds the crossing: everywhere, it moved a 2D offset
        // and a bisector against native.
        if (keepsSmallFactor && q0 > 0.0)
        {
            if (Math.Abs(factors.Q2) < CancelledShare * Math.Abs(factors.Q1))
            {
                factors.Q2 = d / factors.Q1;
            }
            else if (Math.Abs(factors.Q1) < CancelledShare * Math.Abs(factors.Q2))
            {
                factors.Q1 = d / factors.Q2;
            }
        }

        // Clean up near-zero coefficients
        double eps = 100.0 * MACHINE_EPSILON;

        if (Math.Abs(factors.P1) <= eps)
        {
            factors.P1 = 0.0;
        }

        if (Math.Abs(factors.P2) <= eps)
        {
            factors.P2 = 0.0;
        }

        if (Math.Abs(factors.Q1) <= eps)
        {
            factors.Q1 = 0.0;
        }

        if (Math.Abs(factors.Q2) <= eps)
        {
            factors.Q2 = 0.0;
        }

        return factors;
    }
}
