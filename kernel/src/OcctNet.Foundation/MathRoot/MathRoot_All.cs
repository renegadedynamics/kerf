// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_All.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Isolated root locations.</summary>
        public NCollection_DynamicArray<double> Roots = new();
        /// <summary>State numbers for roots.</summary>
        public NCollection_DynamicArray<int> RootStates = new();
        /// <summary>Intervals where function is null.</summary>
        public NCollection_DynamicArray<NullInterval> NullIntervals = new();

        public AllRootsResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(AllRootsResult theResult) => theResult.IsDone();

        public static bool operator true(AllRootsResult theResult) => theResult.IsDone();

        public static bool operator false(AllRootsResult theResult) => !theResult.IsDone();

        public readonly int NbRoots() => Roots.Length();

        public readonly int NbIntervals() => NullIntervals.Length();
    }

    /// <summary>Find multiple roots of f(x) - theOffset = 0 in [theA, theB] (FindAllRoots with these settings).</summary>
    public static MultipleResult FindMultipleRoots<Func>(Func theFunc,
                                                         double theA,
                                                         double theB,
                                                         int theNbSamples,
                                                         double theEpsX,
                                                         double theEpsF,
                                                         double theOffset = 0.0)
        where Func : MathUtils.IScalarFunction
    {
        MultipleConfig aConfig = new();
        aConfig.NbSamples = theNbSamples;
        aConfig.XTolerance = theEpsX;
        aConfig.FTolerance = theEpsF;
        aConfig.Offset = theOffset;
        return FindAllRoots(theFunc, theA, theB, aConfig);
    }

    /// <summary>
    /// Find all roots of a function from samples: null intervals (|F| &lt;= theEpsNul at 2+ consecutive samples, with
    /// refined boundaries) and isolated roots between them.
    /// </summary>
    public static AllRootsResult FindAllRootsWithIntervals<Func>(Func theFunc,
                                                                 math_Vector theSamples,
                                                                 double theEpsX = 1.0e-10,
                                                                 double theEpsF = 1.0e-10,
                                                                 double theEpsNul = 1.0e-10)
        where Func : MathUtils.IScalarFunction
    {
        AllRootsResult aResult = new();

        int aNbp = theSamples.Length();
        if (aNbp < 2)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        int aLower = theSamples.Lower();

        // Evaluate function at first sample point
        if (!theFunc.Value(theSamples[aLower], out double aPrevVal))
        {
            aResult.Status = MathUtils.Status.NotConverged;
            return aResult;
        }

        bool aPrevNul = Math.Abs(aPrevVal) <= theEpsNul;
        if (!aPrevNul)
        {
            // Save non-null value for later use
        }

        bool aInInterval = false;
        bool aNulStart = false;
        bool aNulEnd = false;
        double aDebNul = 0.0, aFinNul = 0.0;
        double aValSav = aPrevVal;

        NCollection_DynamicArray<double> aIntervalStarts = new(), aIntervalEnds = new();

        // Scan through samples to find null intervals
        for (int i = 1; i < aNbp; ++i)
        {
            if (!theFunc.Value(theSamples[aLower + i], out double aVal))
            {
                aResult.Status = MathUtils.Status.NotConverged;
                return aResult;
            }

            bool aCurNul = Math.Abs(aVal) <= theEpsNul;
            if (!aCurNul)
            {
                aValSav = aVal;
            }

            if (aInInterval && !aCurNul)
            {
                // End of null interval
                aInInterval = false;
                aIntervalStarts.Append(aDebNul);

                // Refine end of null interval
                double aCst = (aVal > 0.0) ? theEpsNul : -theEpsNul;

                // Use root finding to locate precise boundary
                MultipleResult aRes = FindMultipleRoots(theFunc,
                                                        theSamples[aLower + i - 1],
                                                        theSamples[aLower + i],
                                                        10,
                                                        theEpsX,
                                                        theEpsF,
                                                        aCst);
                if (aRes.IsDone() && aRes.NbRoots() > 0)
                {
                    aFinNul = aRes.Roots[0];
                }
                else
                {
                    aFinNul = theSamples[aLower + i - 1];
                }

                // Try opposite sign
                aCst = -aCst;
                MultipleResult aRes2 = FindMultipleRoots(theFunc,
                                                         theSamples[aLower + i - 1],
                                                         theSamples[aLower + i],
                                                         10,
                                                         theEpsX,
                                                         theEpsF,
                                                         aCst);
                if (aRes2.IsDone() && aRes2.NbRoots() > 0)
                {
                    if (aRes2.Roots[0] < aFinNul)
                    {
                        aFinNul = aRes2.Roots[0];
                    }
                }

                aIntervalEnds.Append(aFinNul);
            }
            else if (!aInInterval && aPrevNul && aCurNul)
            {
                // Start of null interval
                aInInterval = true;
                if (i == 1)
                {
                    aDebNul = theSamples[aLower];
                    aNulStart = true;
                }
                else
                {
                    // Refine start of null interval
                    double aCst = (aValSav > 0.0) ? theEpsNul : -theEpsNul;

                    MultipleResult aRes = FindMultipleRoots(theFunc,
                                                            theSamples[aLower + i - 2],
                                                            theSamples[aLower + i - 1],
                                                            10,
                                                            theEpsX,
                                                            theEpsF,
                                                            aCst);
                    if (aRes.IsDone() && aRes.NbRoots() > 0)
                    {
                        aDebNul = aRes.Roots[aRes.NbRoots() - 1];
                    }
                    else
                    {
                        aDebNul = theSamples[aLower + i - 1];
                    }

                    // Try opposite sign
                    aCst = -aCst;
                    MultipleResult aRes2 = FindMultipleRoots(theFunc,
                                                             theSamples[aLower + i - 2],
                                                             theSamples[aLower + i - 1],
                                                             10,
                                                             theEpsX,
                                                             theEpsF,
                                                             aCst);
                    if (aRes2.IsDone() && aRes2.NbRoots() > 0)
                    {
                        if (aRes2.Roots[aRes2.NbRoots() - 1] > aDebNul)
                        {
                            aDebNul = aRes2.Roots[aRes2.NbRoots() - 1];
                        }
                    }
                }
            }

            aPrevNul = aCurNul;
        }

        // Handle interval ending at last sample
        if (aInInterval)
        {
            aIntervalStarts.Append(aDebNul);
            aFinNul = theSamples[aLower + aNbp - 1];
            aIntervalEnds.Append(aFinNul);
            aNulEnd = true;
        }

        // Store null intervals
        for (int k = 0; k < aIntervalStarts.Length(); ++k)
        {
            NullInterval anInt = new();
            anInt.A = aIntervalStarts.Value(k);
            anInt.B = aIntervalEnds.Value(k);
            aResult.NullIntervals.Append(anInt);
        }

        double aSampleFirst = theSamples[aLower];
        double aSampleLast = theSamples[aLower + aNbp - 1];

        // Find isolated roots between null intervals
        if (aIntervalStarts.IsEmpty())
        {
            // No null intervals - find all roots in entire range
            MultipleResult aRes =
              FindMultipleRoots(theFunc, aSampleFirst, aSampleLast, aNbp, theEpsX, theEpsF);
            if (aRes.IsDone())
            {
                for (int j = 0; j < aRes.NbRoots(); ++j)
                {
                    aResult.Roots.Append(aRes.Roots[j]);
                    aResult.RootStates.Append(0);
                }
            }
        }
        else
        {
            // Find roots before first null interval
            if (!aNulStart)
            {
                double aStart = aSampleFirst;
                double aEnd = aIntervalStarts.Value(0);
                int aNbrpt =
                  Max(3, (int)(Math.Abs((aEnd - aStart) / (aSampleLast - aSampleFirst)) * aNbp));

                MultipleResult aRes = FindMultipleRoots(theFunc, aStart, aEnd, aNbrpt, theEpsX, theEpsF);
                if (aRes.IsDone())
                {
                    for (int j = 0; j < aRes.NbRoots(); ++j)
                    {
                        aResult.Roots.Append(aRes.Roots[j]);
                        aResult.RootStates.Append(0);
                    }
                }
            }

            // Find roots between null intervals
            for (int k = 1; k < aIntervalStarts.Length(); ++k)
            {
                double aStart = aIntervalEnds.Value(k - 1);
                double aEnd = aIntervalStarts.Value(k);
                int aNbrpt =
                  Max(3, (int)(Math.Abs((aEnd - aStart) / (aSampleLast - aSampleFirst)) * aNbp));

                MultipleResult aRes = FindMultipleRoots(theFunc, aStart, aEnd, aNbrpt, theEpsX, theEpsF);
                if (aRes.IsDone())
                {
                    for (int j = 0; j < aRes.NbRoots(); ++j)
                    {
                        aResult.Roots.Append(aRes.Roots[j]);
                        aResult.RootStates.Append(0);
                    }
                }
            }

            // Find roots after last null interval
            if (!aNulEnd)
            {
                double aStart = aIntervalEnds.Value(aIntervalEnds.Length() - 1);
                double aEnd = aSampleLast;
                int aNbrpt =
                  Max(3, (int)(Math.Abs((aEnd - aStart) / (aSampleLast - aSampleFirst)) * aNbp));

                MultipleResult aRes = FindMultipleRoots(theFunc, aStart, aEnd, aNbrpt, theEpsX, theEpsF);
                if (aRes.IsDone())
                {
                    for (int j = 0; j < aRes.NbRoots(); ++j)
                    {
                        aResult.Roots.Append(aRes.Roots[j]);
                        aResult.RootStates.Append(0);
                    }
                }
            }
        }

        aResult.Status = MathUtils.Status.OK;
        return aResult;
    }

    /// <summary>Find all roots and null intervals using theNbSamples uniform samples on [theA, theB].</summary>
    public static AllRootsResult FindAllRootsWithIntervals<Func>(Func theFunc,
                                                                 double theA,
                                                                 double theB,
                                                                 int theNbSamples,
                                                                 double theEpsX = 1.0e-10,
                                                                 double theEpsF = 1.0e-10,
                                                                 double theEpsNul = 1.0e-10)
        where Func : MathUtils.IScalarFunction
    {
        math_Vector aSamples = new(0, theNbSamples - 1);
        double aStep = (theB - theA) / (theNbSamples - 1);
        for (int i = 0; i < theNbSamples; ++i)
        {
            aSamples[i] = theA + i * aStep;
        }
        // Ensure last point is exactly theB
        aSamples[theNbSamples - 1] = theB;

        return FindAllRootsWithIntervals(theFunc, aSamples, theEpsX, theEpsF, theEpsNul);
    }
}
