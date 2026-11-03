// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_All.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

/// <summary>All roots of a function including null intervals.</summary>
public static partial class MathRoot
{
    /// <summary>An interval where the function is null (within tolerance).</summary>
    public struct NullInterval
    {
        /// <summary>Interval start.</summary>
        public double A;
        /// <summary>Interval end.</summary>
        public double B;
        /// <summary>State number (for parametric curves).</summary>
        public int State;
    }

    /// <summary>Result for all roots finder including null intervals.</summary>
    /// <remarks>Holds array references, so a copy shares them; ported code writes `new AllRootsResult()`.</remarks>
    public struct AllRootsResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Isolated root locations.</summary>
        public List<double> Roots = new();
        /// <summary>State numbers for roots.</summary>
        public List<int> RootStates = new();
        /// <summary>Intervals where function is null.</summary>
        public List<NullInterval> NullIntervals = new();

        public AllRootsResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(AllRootsResult result) => result.IsDone;

        public static bool operator true(AllRootsResult result) => result.IsDone;

        public static bool operator false(AllRootsResult result) => !result.IsDone;

        public readonly int RootCount => Roots.Count;

        public readonly int IntervalCount => NullIntervals.Count;
    }

    /// <summary>Find multiple roots of f(x) - theOffset = 0 in [theA, theB] (FindAllRoots with these settings).</summary>
    public static MultipleResult FindMultipleRoots<Func>(Func func,
                                                         double a,
                                                         double b,
                                                         int nbSamples,
                                                         double epsX,
                                                         double epsF,
                                                         double offset = 0.0)
        where Func : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        MultipleConfig config = new();
        config.NbSamples = nbSamples;
        config.XTolerance = epsX;
        config.FTolerance = epsF;
        config.Offset = offset;
        return FindAllRoots(func, a, b, config);
    }

    /// <summary>
    /// Find all roots of a function from samples: null intervals (|F| &lt;= theEpsNul at 2+ consecutive samples, with
    /// refined boundaries) and isolated roots between them.
    /// </summary>
    public static AllRootsResult FindAllRootsWithIntervals<Func>(Func func,
                                                                 math_Vector samples,
                                                                 double epsX = 1.0e-10,
                                                                 double epsF = 1.0e-10,
                                                                 double epsNul = 1.0e-10)
        where Func : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        AllRootsResult result = new();

        int nbp = samples.Length();
        if (nbp < 2)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }

        int lower = samples.Lower();

        // Evaluate function at first sample point
        if (!func.Value(samples[lower], out double prevVal))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
            return result;
        }

        bool prevNul = Math.Abs(prevVal) <= epsNul;
        if (!prevNul)
        {
            // Save non-null value for later use
        }

        bool inInterval = false;
        bool nulStart = false;
        bool nulEnd = false;
        double debNul = 0.0, finNul = 0.0;
        double valSav = prevVal;

        List<double> intervalStarts = new(), intervalEnds = new();

        // Scan through samples to find null intervals
        for (int i = 1; i < nbp; ++i)
        {
            if (!func.Value(samples[lower + i], out double val))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
                return result;
            }

            bool curNul = Math.Abs(val) <= epsNul;
            if (!curNul)
            {
                valSav = val;
            }

            if (inInterval && !curNul)
            {
                // End of null interval
                inInterval = false;
                intervalStarts.Add(debNul);

                // Refine end of null interval
                double cst = (val > 0.0) ? epsNul : -epsNul;

                // Use root finding to locate precise boundary
                MultipleResult res = FindMultipleRoots(func,
                                                        samples[lower + i - 1],
                                                        samples[lower + i],
                                                        10,
                                                        epsX,
                                                        epsF,
                                                        cst);
                if (res.IsDone && res.RootCount > 0)
                {
                    finNul = res.Roots[0];
                }
                else
                {
                    finNul = samples[lower + i - 1];
                }

                // Try opposite sign
                cst = -cst;
                MultipleResult res2 = FindMultipleRoots(func,
                                                         samples[lower + i - 1],
                                                         samples[lower + i],
                                                         10,
                                                         epsX,
                                                         epsF,
                                                         cst);
                if (res2.IsDone && res2.RootCount > 0)
                {
                    if (res2.Roots[0] < finNul)
                    {
                        finNul = res2.Roots[0];
                    }
                }

                intervalEnds.Add(finNul);
            }
            else if (!inInterval && prevNul && curNul)
            {
                // Start of null interval
                inInterval = true;
                if (i == 1)
                {
                    debNul = samples[lower];
                    nulStart = true;
                }
                else
                {
                    // Refine start of null interval
                    double cst = (valSav > 0.0) ? epsNul : -epsNul;

                    MultipleResult res = FindMultipleRoots(func,
                                                            samples[lower + i - 2],
                                                            samples[lower + i - 1],
                                                            10,
                                                            epsX,
                                                            epsF,
                                                            cst);
                    if (res.IsDone && res.RootCount > 0)
                    {
                        debNul = res.Roots[res.RootCount - 1];
                    }
                    else
                    {
                        debNul = samples[lower + i - 1];
                    }

                    // Try opposite sign
                    cst = -cst;
                    MultipleResult res2 = FindMultipleRoots(func,
                                                             samples[lower + i - 2],
                                                             samples[lower + i - 1],
                                                             10,
                                                             epsX,
                                                             epsF,
                                                             cst);
                    if (res2.IsDone && res2.RootCount > 0)
                    {
                        if (res2.Roots[res2.RootCount - 1] > debNul)
                        {
                            debNul = res2.Roots[res2.RootCount - 1];
                        }
                    }
                }
            }

            prevNul = curNul;
        }

        // Handle interval ending at last sample
        if (inInterval)
        {
            intervalStarts.Add(debNul);
            finNul = samples[lower + nbp - 1];
            intervalEnds.Add(finNul);
            nulEnd = true;
        }

        // Store null intervals
        for (int k = 0; k < intervalStarts.Count; ++k)
        {
            NullInterval anInt = new();
            anInt.A = intervalStarts[k];
            anInt.B = intervalEnds[k];
            result.NullIntervals.Add(anInt);
        }

        double sampleFirst = samples[lower];
        double sampleLast = samples[lower + nbp - 1];

        // Find isolated roots between null intervals
        if (intervalStarts.Count == 0)
        {
            // No null intervals - find all roots in entire range
            MultipleResult res =
              FindMultipleRoots(func, sampleFirst, sampleLast, nbp, epsX, epsF);
            if (res.IsDone)
            {
                for (int j = 0; j < res.RootCount; ++j)
                {
                    result.Roots.Add(res.Roots[j]);
                    result.RootStates.Add(0);
                }
            }
        }
        else
        {
            // Find roots before first null interval
            if (!nulStart)
            {
                double start = sampleFirst;
                double end = intervalStarts[0];
                int nbrpt =
                  Max(3, (int)(Math.Abs((end - start) / (sampleLast - sampleFirst)) * nbp));

                MultipleResult res = FindMultipleRoots(func, start, end, nbrpt, epsX, epsF);
                if (res.IsDone)
                {
                    for (int j = 0; j < res.RootCount; ++j)
                    {
                        result.Roots.Add(res.Roots[j]);
                        result.RootStates.Add(0);
                    }
                }
            }

            // Find roots between null intervals
            for (int k = 1; k < intervalStarts.Count; ++k)
            {
                double start = intervalEnds[k - 1];
                double end = intervalStarts[k];
                int nbrpt =
                  Max(3, (int)(Math.Abs((end - start) / (sampleLast - sampleFirst)) * nbp));

                MultipleResult res = FindMultipleRoots(func, start, end, nbrpt, epsX, epsF);
                if (res.IsDone)
                {
                    for (int j = 0; j < res.RootCount; ++j)
                    {
                        result.Roots.Add(res.Roots[j]);
                        result.RootStates.Add(0);
                    }
                }
            }

            // Find roots after last null interval
            if (!nulEnd)
            {
                double start = intervalEnds[intervalEnds.Count - 1];
                double end = sampleLast;
                int nbrpt =
                  Max(3, (int)(Math.Abs((end - start) / (sampleLast - sampleFirst)) * nbp));

                MultipleResult res = FindMultipleRoots(func, start, end, nbrpt, epsX, epsF);
                if (res.IsDone)
                {
                    for (int j = 0; j < res.RootCount; ++j)
                    {
                        result.Roots.Add(res.Roots[j]);
                        result.RootStates.Add(0);
                    }
                }
            }
        }

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
        return result;
    }

    /// <summary>Find all roots and null intervals using theNbSamples uniform samples on [theA, theB].</summary>
    public static AllRootsResult FindAllRootsWithIntervals<Func>(Func func,
                                                                 double a,
                                                                 double b,
                                                                 int nbSamples,
                                                                 double epsX = 1.0e-10,
                                                                 double epsF = 1.0e-10,
                                                                 double epsNul = 1.0e-10)
        where Func : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        math_Vector samples = new(0, nbSamples - 1);
        double step = (b - a) / (nbSamples - 1);
        for (int i = 0; i < nbSamples; ++i)
        {
            samples[i] = a + i * step;
        }
        // Ensure last point is exactly theB
        samples[nbSamples - 1] = b;

        return FindAllRootsWithIntervals(func, samples, epsX, epsF, epsNul);
    }
}
