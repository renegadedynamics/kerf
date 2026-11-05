// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Trig.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Occt;

namespace Kerf.Engine.Numerics.Roots;

/// <summary>Trigonometric equation solver.</summary>
internal static partial class MathRoot
{
    /// <summary>Result for trigonometric equation solver.</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new TrigResult()`.</remarks>
    public struct TrigResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        public Array4<double> Roots;
        public int NbRoots = 0;
        public bool InfiniteRoots = false;

        public TrigResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(TrigResult result) => result.IsDone;

        public static bool operator true(TrigResult result) => result.IsDone;

        public static bool operator false(TrigResult result) => !result.IsDone;
    }

    /// <summary>
    /// Solve a*cos^2(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e = 0 for x in [theInfBound, theSupBound], by the
    /// half-angle substitution t = tan(x/2) and Newton/Halley refinement.
    /// </summary>
    public static TrigResult Trigonometric(double a,
                                           double b,
                                           double c,
                                           double d,
                                           double e,
                                           double infBound = 0.0,
                                           double supBound = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_2PI,
                                           double eps = 1.5e-12)
    {
        const double THE_2PI = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_2PI;
        const double THE_PI = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_PI;

        TrigResult result = new();
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        // Compute working interval
        double myBorneInf, delta, mod;
        if (infBound <= -double.MaxValue / 2.0
            && supBound >= double.MaxValue / 2.0)
        {
            myBorneInf = 0.0;
            delta = THE_2PI;
            mod = 0.0;
        }
        else if (supBound >= double.MaxValue / 2.0)
        {
            myBorneInf = infBound;
            delta = THE_2PI;
            mod = myBorneInf / THE_2PI;
        }
        else if (infBound <= -double.MaxValue / 2.0)
        {
            myBorneInf = supBound - THE_2PI;
            delta = THE_2PI;
            mod = myBorneInf / THE_2PI;
        }
        else
        {
            myBorneInf = infBound;
            delta = supBound - infBound;
            mod = infBound / THE_2PI;
            if (delta > THE_2PI)
            {
                delta = THE_2PI;
            }
        }

        Array4<double> zer = default;
        int nZer = 0;
        double delta_Eps = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON * Math.Abs(delta);

        // Case: A = B = 0 (degree <= 2 in cos/sin)
        if (Math.Abs(a) <= eps && Math.Abs(b) <= eps)
        {
            if (Math.Abs(c) <= eps)
            {
                if (Math.Abs(d) <= eps)
                {
                    if (Math.Abs(e) <= eps)
                    {
                        result.InfiniteRoots = true;
                        return result;
                    }
                    else
                    {
                        result.NbRoots = 0;
                        return result;
                    }
                }
                else
                {
                    // d*sin(x) + e = 0  =>  sin(x) = -e/d
                    double val = -e / d;
                    if (Math.Abs(val) > 1.0)
                    {
                        result.NbRoots = 0;
                        return result;
                    }

                    zer[0] = Math.Asin(val);
                    zer[1] = THE_PI - zer[0];
                    nZer = 2;

                    // Adjust to positive range
                    for (int i = 0; i < nZer; ++i)
                    {
                        if (zer[i] <= -eps)
                        {
                            zer[i] = THE_2PI - Math.Abs(zer[i]);
                        }
                        zer[i] += Math.Truncate(mod) * THE_2PI;
                        double aX = zer[i] - myBorneInf;
                        if (aX >= -delta_Eps && aX <= delta + delta_Eps)
                        {
                            result.Roots[result.NbRoots++] = zer[i];
                        }
                    }
                    return result;
                }
            }
            else if (Math.Abs(d) <= eps)
            {
                // c*cos(x) + e = 0  =>  cos(x) = -e/c
                double val = -e / c;
                if (Math.Abs(val) > 1.0)
                {
                    result.NbRoots = 0;
                    return result;
                }

                double principal = Math.Acos(val);
                zer[0] = principal;  // acos gives [0, PI]
                zer[1] = -principal; // Negative angle
                nZer = 2;

                // For each solution, find the representative in or near the given bounds
                for (int i = 0; i < nZer; ++i)
                {
                    double angle = zer[i];
                    // Shift angle to be near the lower bound
                    double aK = Math.Floor((infBound - angle) / THE_2PI);
                    angle += (aK + 1) * THE_2PI; // Start from a value >= theInfBound - 2*PI

                    // Check both this value and the next period
                    for (int period = 0; period < 2; ++period)
                    {
                        double testAngle = angle + period * THE_2PI;
                        if (testAngle >= infBound - delta_Eps && testAngle <= supBound + delta_Eps)
                        {
                            // Avoid duplicates
                            bool dup = false;
                            for (int k = 0; k < result.NbRoots; ++k)
                            {
                                if (Math.Abs(testAngle - result.Roots[k]) < eps)
                                {
                                    dup = true;
                                    break;
                                }
                            }
                            if (!dup && result.NbRoots < 4)
                            {
                                result.Roots[result.NbRoots++] = testAngle;
                            }
                        }
                    }
                }
                return result;
            }
            else
            {
                // c*cos(x) + d*sin(x) + e = 0
                // Using t = tan(x/2): (e-c)*t^2 + 2d*t + (e+c) = 0
                double aa = e - c;
                double bb = 2.0 * d;
                double cc = e + c;

                global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult poly = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quadratic(aa, bb, cc);
                if (!poly.IsDone)
                {
                    result.Status = poly.Status;
                    return result;
                }
                // OCCT: unreachable, InfiniteSolutions is not IsDone() and returned above
                if (poly.Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions)
                {
                    result.InfiniteRoots = true;
                    return result;
                }

                nZer = poly.NbRoots;
                for (int i = 0; i < nZer; ++i)
                {
                    zer[i] = poly.Roots[i];
                }
            }
        }
        else
        {
            // Special case: A = E = 0
            if (Math.Abs(a) <= eps && Math.Abs(e) <= eps)
            {
                if (Math.Abs(c) <= eps)
                {
                    // 2*B*sin*cos + D*sin = 0  =>  sin(x)*(2*B*cos(x) + D) = 0
                    zer[0] = 0.0;
                    zer[1] = THE_PI;
                    nZer = 2;

                    double val = -d / (b * 2.0);
                    if (Math.Abs(val) <= 1.0 + 1.0e-10)
                    {
                        if (val >= 1.0)
                        {
                            zer[2] = 0.0;
                            zer[3] = 0.0;
                        }
                        else if (val <= -1.0)
                        {
                            zer[2] = THE_PI;
                            zer[3] = THE_PI;
                        }
                        else
                        {
                            zer[2] = Math.Acos(val);
                            zer[3] = THE_2PI - zer[2];
                        }
                        nZer = 4;
                    }

                    // OCCT: no duplicate removal or sorting here, unlike the general case
                    for (int i = 0; i < nZer; ++i)
                    {
                        if (zer[i] <= myBorneInf - eps)
                        {
                            zer[i] += THE_2PI;
                        }
                        zer[i] += Math.Truncate(mod) * THE_2PI;
                        double aX = zer[i] - myBorneInf;
                        if (aX >= -1.0e-10 && aX <= delta + 1.0e-10)
                        {
                            zer[i] = Max(infBound, Min(supBound, zer[i]));
                            result.Roots[result.NbRoots++] = zer[i];
                        }
                    }
                    return result;
                }
                if (Math.Abs(d) <= eps)
                {
                    // 2*B*sin*cos + C*cos = 0  =>  cos(x)*(2*B*sin(x) + C) = 0
                    zer[0] = THE_PI / 2.0;
                    zer[1] = THE_PI * 3.0 / 2.0;
                    nZer = 2;

                    double val = -c / (b * 2.0);
                    if (Math.Abs(val) <= 1.0 + 1.0e-10)
                    {
                        if (val >= 1.0)
                        {
                            zer[2] = THE_PI / 2.0;
                            zer[3] = THE_PI / 2.0;
                        }
                        else if (val <= -1.0)
                        {
                            zer[2] = THE_PI * 3.0 / 2.0;
                            zer[3] = THE_PI * 3.0 / 2.0;
                        }
                        else
                        {
                            zer[2] = Math.Asin(val);
                            zer[3] = THE_PI - zer[2];
                        }
                        nZer = 4;
                    }

                    // OCCT: no duplicate removal or sorting here, unlike the general case
                    for (int i = 0; i < nZer; ++i)
                    {
                        if (zer[i] <= myBorneInf - eps)
                        {
                            zer[i] += THE_2PI;
                        }
                        zer[i] += Math.Truncate(mod) * THE_2PI;
                        double aX = zer[i] - myBorneInf;
                        if (aX >= -1.0e-10 && aX <= delta + 1.0e-10)
                        {
                            zer[i] = Max(infBound, Min(supBound, zer[i]));
                            result.Roots[result.NbRoots++] = zer[i];
                        }
                    }
                    return result;
                }
            }

            // General case: degree 4 polynomial
            // t = tan(x/2), then:
            // ko[0]*t^4 + ko[1]*t^3 + ko[2]*t^2 + ko[3]*t + ko[4] = 0
            double ko0 = a - c + e;
            double ko1 = 2.0 * d - 4.0 * b;
            double ko2 = 2.0 * e - 2.0 * a;
            double ko3 = 4.0 * b + 2.0 * d;
            double ko4 = a + c + e;

            global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult poly = global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(ko0, ko1, ko2, ko3, ko4);
            if (!poly.IsDone)
            {
                if (poly.Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions)
                {
                    result.InfiniteRoots = true;
                }
                else
                {
                    result.Status = poly.Status;
                }
                return result;
            }

            // NbRoots is bounded by the fixed storage capacity; clamp defensively.
            nZer = Min(poly.NbRoots, zer.Length);
            for (int i = 0; i < nZer; ++i)
            {
                zer[i] = poly.Roots[i];
            }

            // Sort roots
            ((Span<double>)zer)[..nZer].Sort();
        }

        // Newton refinement with Halley's method fallback for double roots (a C++ lambda)
        double ARefineRoot(double x)
        {
            const int THE_MAX_ITER = 20;
            const double THE_TOL = 1.0e-14;

            for (int iter = 0; iter < THE_MAX_ITER; ++iter)
            {
                double cos = Math.Cos(x);
                double sin = Math.Sin(x);
                double cos2 = cos * cos;
                double sin2 = sin * sin;
                double cs = cos * sin;

                double f = a * cos2 + 2.0 * b * cs + c * cos + d * sin + e;
                double df = -2.0 * a * cs + 2.0 * b * (cos2 - sin2) - c * sin + d * cos;

                // Check if already converged
                if (Math.Abs(f) < 1.0e-15)
                {
                    break;
                }

                double delta;
                if (Math.Abs(df) < 1.0e-10 * (Math.Abs(f) + 1.0))
                {
                    // Near double root: use Halley's method for better convergence
                    // F'' = -2*a*(cos^2-sin^2) - 4*b*cos*sin - c*cos - d*sin
                    double d2F =
                      -2.0 * a * (cos2 - sin2) - 4.0 * b * cs - c * cos - d * sin;
                    double denom = 2.0 * df * df - f * d2F;
                    if (Math.Abs(denom) < 1.0e-30)
                    {
                        // Can't improve further
                        break;
                    }
                    delta = 2.0 * f * df / denom;
                }
                else
                {
                    // Standard Newton step
                    delta = f / df;
                }

                // Limit step size to avoid overshooting
                const double THE_MAX_STEP = 0.5;
                if (Math.Abs(delta) > THE_MAX_STEP)
                {
                    delta = (delta > 0) ? THE_MAX_STEP : -THE_MAX_STEP;
                }

                x -= delta;

                if (Math.Abs(delta) < THE_TOL)
                {
                    break;
                }
            }
            return x;
        }

        // Convert t values to angles and filter by bounds
        for (int i = 0; i < nZer; ++i)
        {
            double teta = 2.0 * Math.Atan(zer[i]);
            if (zer[i] <= -eps)
            {
                teta = THE_2PI - Math.Abs(teta);
            }
            teta += Math.Truncate(mod) * THE_2PI;
            if (teta - myBorneInf < 0.0)
            {
                teta += THE_2PI;
            }

            double aX = teta - myBorneInf;
            if (aX >= -delta_Eps && aX <= delta + delta_Eps)
            {
                double tetaRefined = ARefineRoot(teta);

                // Check if Newton didn't diverge too far
                double deltaNewton = Math.Abs(tetaRefined - teta);
                double supmInfs100 = (supBound - infBound) * 0.01;
                if (deltaNewton <= supmInfs100)
                {
                    teta = tetaRefined;
                }

                // Insert in sorted order, avoiding duplicates
                bool found = false;
                for (int k = 0; k < result.NbRoots; ++k)
                {
                    if (Math.Abs(teta - result.Roots[k]) < eps)
                    {
                        found = true;
                        break;
                    }
                }

                if (!found && result.NbRoots < 4)
                {
                    // Insert sorted
                    int pos = result.NbRoots;
                    for (int k = 0; k < result.NbRoots; ++k)
                    {
                        if (teta < result.Roots[k])
                        {
                            pos = k;
                            break;
                        }
                    }
                    for (int k = result.NbRoots; k > pos; --k)
                    {
                        result.Roots[k] = result.Roots[k - 1];
                    }
                    result.Roots[pos] = teta;
                    result.NbRoots++;
                }
            }
        }

        // Special case: check if PI is a root (when A - C + E = 0)
        if (result.NbRoots < 4 && Math.Abs(a - c + e) <= eps)
        {
            double teta = THE_PI + Math.Truncate(mod) * THE_2PI;
            double aX = teta - myBorneInf;
            if (aX >= -delta_Eps && aX <= delta + delta_Eps)
            {
                bool found = false;
                for (int k = 0; k < result.NbRoots; ++k)
                {
                    if (Math.Abs(teta - result.Roots[k]) <= eps)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    int pos = result.NbRoots;
                    for (int k = 0; k < result.NbRoots; ++k)
                    {
                        if (teta < result.Roots[k])
                        {
                            pos = k;
                            break;
                        }
                    }
                    for (int k = result.NbRoots; k > pos; --k)
                    {
                        result.Roots[k] = result.Roots[k - 1];
                    }
                    result.Roots[pos] = teta;
                    result.NbRoots++;
                }
            }
        }

        return result;
    }

    /// <summary>Solve linear trigonometric equation d*sin(x) + e = 0.</summary>
    public static TrigResult TrigonometricLinear(double d, double e, double infBound = 0.0, double supBound = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_2PI)
    {
        return Trigonometric(0.0, 0.0, 0.0, d, e, infBound, supBound);
    }

    /// <summary>Solve trigonometric equation c*cos(x) + d*sin(x) + e = 0.</summary>
    public static TrigResult TrigonometricCDE(double c, double d, double e, double infBound = 0.0, double supBound = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_2PI)
    {
        return Trigonometric(0.0, 0.0, c, d, e, infBound, supBound);
    }
}
