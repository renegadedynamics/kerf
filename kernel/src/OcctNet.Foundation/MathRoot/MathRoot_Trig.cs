// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Trig.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Trigonometric equation solver.</summary>
public static partial class MathRoot
{
    /// <summary>Result for trigonometric equation solver.</summary>
    /// <remarks>Its C++ default status is NotConverged, so ported code writes `new TrigResult()`.</remarks>
    public struct TrigResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        public std_array4<double> Roots;
        public int NbRoots = 0;
        public bool InfiniteRoots = false;

        public TrigResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(TrigResult theResult) => theResult.IsDone();

        public static bool operator true(TrigResult theResult) => theResult.IsDone();

        public static bool operator false(TrigResult theResult) => !theResult.IsDone();
    }

    /// <summary>
    /// Solve a*cos^2(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e = 0 for x in [theInfBound, theSupBound], by the
    /// half-angle substitution t = tan(x/2) and Newton/Halley refinement.
    /// </summary>
    public static TrigResult Trigonometric(double theA,
                                           double theB,
                                           double theC,
                                           double theD,
                                           double theE,
                                           double theInfBound = 0.0,
                                           double theSupBound = MathUtils.THE_2PI,
                                           double theEps = 1.5e-12)
    {
        const double THE_2PI = MathUtils.THE_2PI;
        const double THE_PI = MathUtils.THE_PI;

        TrigResult aResult = new();
        aResult.Status = MathUtils.Status.OK;

        // Compute working interval
        double aMyBorneInf, aDelta, aMod;
        if (theInfBound <= -double.MaxValue / 2.0
            && theSupBound >= double.MaxValue / 2.0)
        {
            aMyBorneInf = 0.0;
            aDelta = THE_2PI;
            aMod = 0.0;
        }
        else if (theSupBound >= double.MaxValue / 2.0)
        {
            aMyBorneInf = theInfBound;
            aDelta = THE_2PI;
            aMod = aMyBorneInf / THE_2PI;
        }
        else if (theInfBound <= -double.MaxValue / 2.0)
        {
            aMyBorneInf = theSupBound - THE_2PI;
            aDelta = THE_2PI;
            aMod = aMyBorneInf / THE_2PI;
        }
        else
        {
            aMyBorneInf = theInfBound;
            aDelta = theSupBound - theInfBound;
            aMod = theInfBound / THE_2PI;
            if (aDelta > THE_2PI)
            {
                aDelta = THE_2PI;
            }
        }

        std_array4<double> aZer = default;
        int aNZer = 0;
        double aDelta_Eps = MathUtils.THE_EPSILON * Math.Abs(aDelta);

        // Case: A = B = 0 (degree <= 2 in cos/sin)
        if (Math.Abs(theA) <= theEps && Math.Abs(theB) <= theEps)
        {
            if (Math.Abs(theC) <= theEps)
            {
                if (Math.Abs(theD) <= theEps)
                {
                    if (Math.Abs(theE) <= theEps)
                    {
                        aResult.InfiniteRoots = true;
                        return aResult;
                    }
                    else
                    {
                        aResult.NbRoots = 0;
                        return aResult;
                    }
                }
                else
                {
                    // d*sin(x) + e = 0  =>  sin(x) = -e/d
                    double aVal = -theE / theD;
                    if (Math.Abs(aVal) > 1.0)
                    {
                        aResult.NbRoots = 0;
                        return aResult;
                    }

                    aZer[0] = Math.Asin(aVal);
                    aZer[1] = THE_PI - aZer[0];
                    aNZer = 2;

                    // Adjust to positive range
                    for (int i = 0; i < aNZer; ++i)
                    {
                        if (aZer[i] <= -theEps)
                        {
                            aZer[i] = THE_2PI - Math.Abs(aZer[i]);
                        }
                        aZer[i] += Math.Truncate(aMod) * THE_2PI;
                        double aX = aZer[i] - aMyBorneInf;
                        if (aX >= -aDelta_Eps && aX <= aDelta + aDelta_Eps)
                        {
                            aResult.Roots[aResult.NbRoots++] = aZer[i];
                        }
                    }
                    return aResult;
                }
            }
            else if (Math.Abs(theD) <= theEps)
            {
                // c*cos(x) + e = 0  =>  cos(x) = -e/c
                double aVal = -theE / theC;
                if (Math.Abs(aVal) > 1.0)
                {
                    aResult.NbRoots = 0;
                    return aResult;
                }

                double aPrincipal = Math.Acos(aVal);
                aZer[0] = aPrincipal;  // acos gives [0, PI]
                aZer[1] = -aPrincipal; // Negative angle
                aNZer = 2;

                // For each solution, find the representative in or near the given bounds
                for (int i = 0; i < aNZer; ++i)
                {
                    double aAngle = aZer[i];
                    // Shift angle to be near the lower bound
                    double aK = Math.Floor((theInfBound - aAngle) / THE_2PI);
                    aAngle += (aK + 1) * THE_2PI; // Start from a value >= theInfBound - 2*PI

                    // Check both this value and the next period
                    for (int aPeriod = 0; aPeriod < 2; ++aPeriod)
                    {
                        double aTestAngle = aAngle + aPeriod * THE_2PI;
                        if (aTestAngle >= theInfBound - aDelta_Eps && aTestAngle <= theSupBound + aDelta_Eps)
                        {
                            // Avoid duplicates
                            bool aDup = false;
                            for (int k = 0; k < aResult.NbRoots; ++k)
                            {
                                if (Math.Abs(aTestAngle - aResult.Roots[k]) < theEps)
                                {
                                    aDup = true;
                                    break;
                                }
                            }
                            if (!aDup && aResult.NbRoots < 4)
                            {
                                aResult.Roots[aResult.NbRoots++] = aTestAngle;
                            }
                        }
                    }
                }
                return aResult;
            }
            else
            {
                // c*cos(x) + d*sin(x) + e = 0
                // Using t = tan(x/2): (e-c)*t^2 + 2d*t + (e+c) = 0
                double aAA = theE - theC;
                double aBB = 2.0 * theD;
                double aCC = theE + theC;

                MathUtils.PolyResult aPoly = MathPoly.Quadratic(aAA, aBB, aCC);
                if (!aPoly.IsDone())
                {
                    aResult.Status = aPoly.Status;
                    return aResult;
                }
                // OCCT: unreachable, InfiniteSolutions is not IsDone() and returned above
                if (aPoly.Status == MathUtils.Status.InfiniteSolutions)
                {
                    aResult.InfiniteRoots = true;
                    return aResult;
                }

                aNZer = aPoly.NbRoots;
                for (int i = 0; i < aNZer; ++i)
                {
                    aZer[i] = aPoly.Roots[i];
                }
            }
        }
        else
        {
            // Special case: A = E = 0
            if (Math.Abs(theA) <= theEps && Math.Abs(theE) <= theEps)
            {
                if (Math.Abs(theC) <= theEps)
                {
                    // 2*B*sin*cos + D*sin = 0  =>  sin(x)*(2*B*cos(x) + D) = 0
                    aZer[0] = 0.0;
                    aZer[1] = THE_PI;
                    aNZer = 2;

                    double aVal = -theD / (theB * 2.0);
                    if (Math.Abs(aVal) <= 1.0 + 1.0e-10)
                    {
                        if (aVal >= 1.0)
                        {
                            aZer[2] = 0.0;
                            aZer[3] = 0.0;
                        }
                        else if (aVal <= -1.0)
                        {
                            aZer[2] = THE_PI;
                            aZer[3] = THE_PI;
                        }
                        else
                        {
                            aZer[2] = Math.Acos(aVal);
                            aZer[3] = THE_2PI - aZer[2];
                        }
                        aNZer = 4;
                    }

                    // OCCT: no duplicate removal or sorting here, unlike the general case
                    for (int i = 0; i < aNZer; ++i)
                    {
                        if (aZer[i] <= aMyBorneInf - theEps)
                        {
                            aZer[i] += THE_2PI;
                        }
                        aZer[i] += Math.Truncate(aMod) * THE_2PI;
                        double aX = aZer[i] - aMyBorneInf;
                        if (aX >= -1.0e-10 && aX <= aDelta + 1.0e-10)
                        {
                            aZer[i] = Max(theInfBound, Min(theSupBound, aZer[i]));
                            aResult.Roots[aResult.NbRoots++] = aZer[i];
                        }
                    }
                    return aResult;
                }
                if (Math.Abs(theD) <= theEps)
                {
                    // 2*B*sin*cos + C*cos = 0  =>  cos(x)*(2*B*sin(x) + C) = 0
                    aZer[0] = THE_PI / 2.0;
                    aZer[1] = THE_PI * 3.0 / 2.0;
                    aNZer = 2;

                    double aVal = -theC / (theB * 2.0);
                    if (Math.Abs(aVal) <= 1.0 + 1.0e-10)
                    {
                        if (aVal >= 1.0)
                        {
                            aZer[2] = THE_PI / 2.0;
                            aZer[3] = THE_PI / 2.0;
                        }
                        else if (aVal <= -1.0)
                        {
                            aZer[2] = THE_PI * 3.0 / 2.0;
                            aZer[3] = THE_PI * 3.0 / 2.0;
                        }
                        else
                        {
                            aZer[2] = Math.Asin(aVal);
                            aZer[3] = THE_PI - aZer[2];
                        }
                        aNZer = 4;
                    }

                    // OCCT: no duplicate removal or sorting here, unlike the general case
                    for (int i = 0; i < aNZer; ++i)
                    {
                        if (aZer[i] <= aMyBorneInf - theEps)
                        {
                            aZer[i] += THE_2PI;
                        }
                        aZer[i] += Math.Truncate(aMod) * THE_2PI;
                        double aX = aZer[i] - aMyBorneInf;
                        if (aX >= -1.0e-10 && aX <= aDelta + 1.0e-10)
                        {
                            aZer[i] = Max(theInfBound, Min(theSupBound, aZer[i]));
                            aResult.Roots[aResult.NbRoots++] = aZer[i];
                        }
                    }
                    return aResult;
                }
            }

            // General case: degree 4 polynomial
            // t = tan(x/2), then:
            // ko[0]*t^4 + ko[1]*t^3 + ko[2]*t^2 + ko[3]*t + ko[4] = 0
            double ko0 = theA - theC + theE;
            double ko1 = 2.0 * theD - 4.0 * theB;
            double ko2 = 2.0 * theE - 2.0 * theA;
            double ko3 = 4.0 * theB + 2.0 * theD;
            double ko4 = theA + theC + theE;

            MathUtils.PolyResult aPoly = MathPoly.Quartic(ko0, ko1, ko2, ko3, ko4);
            if (!aPoly.IsDone())
            {
                if (aPoly.Status == MathUtils.Status.InfiniteSolutions)
                {
                    aResult.InfiniteRoots = true;
                }
                else
                {
                    aResult.Status = aPoly.Status;
                }
                return aResult;
            }

            // NbRoots is bounded by the fixed storage capacity; clamp defensively.
            aNZer = Min(aPoly.NbRoots, aZer.Length);
            for (int i = 0; i < aNZer; ++i)
            {
                aZer[i] = aPoly.Roots[i];
            }

            // Sort roots
            ((Span<double>)aZer)[..aNZer].Sort();
        }

        // Newton refinement with Halley's method fallback for double roots (a C++ lambda)
        double aRefineRoot(double theX)
        {
            const int THE_MAX_ITER = 20;
            const double THE_TOL = 1.0e-14;

            for (int anIter = 0; anIter < THE_MAX_ITER; ++anIter)
            {
                double aCos = Math.Cos(theX);
                double aSin = Math.Sin(theX);
                double aCos2 = aCos * aCos;
                double aSin2 = aSin * aSin;
                double aCS = aCos * aSin;

                double aF = theA * aCos2 + 2.0 * theB * aCS + theC * aCos + theD * aSin + theE;
                double aDF = -2.0 * theA * aCS + 2.0 * theB * (aCos2 - aSin2) - theC * aSin + theD * aCos;

                // Check if already converged
                if (Math.Abs(aF) < 1.0e-15)
                {
                    break;
                }

                double aDelta;
                if (Math.Abs(aDF) < 1.0e-10 * (Math.Abs(aF) + 1.0))
                {
                    // Near double root: use Halley's method for better convergence
                    // F'' = -2*a*(cos^2-sin^2) - 4*b*cos*sin - c*cos - d*sin
                    double aD2F =
                      -2.0 * theA * (aCos2 - aSin2) - 4.0 * theB * aCS - theC * aCos - theD * aSin;
                    double aDenom = 2.0 * aDF * aDF - aF * aD2F;
                    if (Math.Abs(aDenom) < 1.0e-30)
                    {
                        // Can't improve further
                        break;
                    }
                    aDelta = 2.0 * aF * aDF / aDenom;
                }
                else
                {
                    // Standard Newton step
                    aDelta = aF / aDF;
                }

                // Limit step size to avoid overshooting
                const double THE_MAX_STEP = 0.5;
                if (Math.Abs(aDelta) > THE_MAX_STEP)
                {
                    aDelta = (aDelta > 0) ? THE_MAX_STEP : -THE_MAX_STEP;
                }

                theX -= aDelta;

                if (Math.Abs(aDelta) < THE_TOL)
                {
                    break;
                }
            }
            return theX;
        }

        // Convert t values to angles and filter by bounds
        for (int i = 0; i < aNZer; ++i)
        {
            double aTeta = 2.0 * Math.Atan(aZer[i]);
            if (aZer[i] <= -theEps)
            {
                aTeta = THE_2PI - Math.Abs(aTeta);
            }
            aTeta += Math.Truncate(aMod) * THE_2PI;
            if (aTeta - aMyBorneInf < 0.0)
            {
                aTeta += THE_2PI;
            }

            double aX = aTeta - aMyBorneInf;
            if (aX >= -aDelta_Eps && aX <= aDelta + aDelta_Eps)
            {
                double aTetaRefined = aRefineRoot(aTeta);

                // Check if Newton didn't diverge too far
                double aDeltaNewton = Math.Abs(aTetaRefined - aTeta);
                double aSupmInfs100 = (theSupBound - theInfBound) * 0.01;
                if (aDeltaNewton <= aSupmInfs100)
                {
                    aTeta = aTetaRefined;
                }

                // Insert in sorted order, avoiding duplicates
                bool aFound = false;
                for (int k = 0; k < aResult.NbRoots; ++k)
                {
                    if (Math.Abs(aTeta - aResult.Roots[k]) < theEps)
                    {
                        aFound = true;
                        break;
                    }
                }

                if (!aFound && aResult.NbRoots < 4)
                {
                    // Insert sorted
                    int aPos = aResult.NbRoots;
                    for (int k = 0; k < aResult.NbRoots; ++k)
                    {
                        if (aTeta < aResult.Roots[k])
                        {
                            aPos = k;
                            break;
                        }
                    }
                    for (int k = aResult.NbRoots; k > aPos; --k)
                    {
                        aResult.Roots[k] = aResult.Roots[k - 1];
                    }
                    aResult.Roots[aPos] = aTeta;
                    aResult.NbRoots++;
                }
            }
        }

        // Special case: check if PI is a root (when A - C + E = 0)
        if (aResult.NbRoots < 4 && Math.Abs(theA - theC + theE) <= theEps)
        {
            double aTeta = THE_PI + Math.Truncate(aMod) * THE_2PI;
            double aX = aTeta - aMyBorneInf;
            if (aX >= -aDelta_Eps && aX <= aDelta + aDelta_Eps)
            {
                bool aFound = false;
                for (int k = 0; k < aResult.NbRoots; ++k)
                {
                    if (Math.Abs(aTeta - aResult.Roots[k]) <= theEps)
                    {
                        aFound = true;
                        break;
                    }
                }
                if (!aFound)
                {
                    int aPos = aResult.NbRoots;
                    for (int k = 0; k < aResult.NbRoots; ++k)
                    {
                        if (aTeta < aResult.Roots[k])
                        {
                            aPos = k;
                            break;
                        }
                    }
                    for (int k = aResult.NbRoots; k > aPos; --k)
                    {
                        aResult.Roots[k] = aResult.Roots[k - 1];
                    }
                    aResult.Roots[aPos] = aTeta;
                    aResult.NbRoots++;
                }
            }
        }

        return aResult;
    }

    /// <summary>Solve linear trigonometric equation d*sin(x) + e = 0.</summary>
    public static TrigResult TrigonometricLinear(double theD, double theE, double theInfBound = 0.0, double theSupBound = MathUtils.THE_2PI)
    {
        return Trigonometric(0.0, 0.0, 0.0, theD, theE, theInfBound, theSupBound);
    }

    /// <summary>Solve trigonometric equation c*cos(x) + d*sin(x) + e = 0.</summary>
    public static TrigResult TrigonometricCDE(double theC, double theD, double theE, double theInfBound = 0.0, double theSupBound = MathUtils.THE_2PI)
    {
        return Trigonometric(0.0, 0.0, theC, theD, theE, theInfBound, theSupBound);
    }
}
