// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Low-level Gordon surface construction from a compatible B-spline curve network (explicit, non-periodic,
/// consistently ordered and reparametrised curves): a profile skin, a guide skin and an intersection-grid reference
/// surface, aligned to one knot basis; the result is the profile skin plus the guide skin minus the reference.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> copies the arrays and keeps the caller's curves, which it never
/// changes: polynomial families are made compatible through a <see cref="GeomFill_Profiler"/>, which works on copies.
/// <see cref="Surface"/> hands out the surface frozen.
/// </remarks>
public class GeomFill_NetworkSurface
{
    /// <summary>Result state of the last Perform() call.</summary>
    public enum ResultStatus
    {
        NotStarted,
        Done,
        InvalidInput,
        CurveCompatibilityFailed,
        SkinningFailed,
        ReferenceSurfaceFailed,
        KnotAlignmentFailed,
        RationalDegreeOverflow,
        RationalConstructionFailed,
        ConstructionFailed,
        PeriodicityFailed
    }

    private NCollection_Array1<BSplineCurve3D?> myProfiles = new();
    private NCollection_Array1<BSplineCurve3D?> myGuides = new();
    private NCollection_Array1<double> myProfileParameters = new();
    private NCollection_Array1<double> myGuideParameters = new();
    private NCollection_Array2<Point3D> myIntersectionPoints = new();
    private NCollection_Array2<double> myIntersectionWeights = new();
    private BSplineSurface? mySurface;
    private double myTolerance = 0.0;
    private bool myIsUClosed = false;
    private bool myIsVClosed = false;
    private ResultStatus myStatus = ResultStatus.NotStarted;

    private static bool checkParameters(NCollection_Array1<double> theParameters)
    {
        if (theParameters.Size() < 2)
        {
            return false;
        }
        for (int aParamIdx = 1; aParamIdx < theParameters.Size(); ++aParamIdx)
        {
            if (theParameters.At(aParamIdx) <= theParameters.At(aParamIdx - 1) + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        return true;
    }

    private static bool checkWeights(NCollection_Array2<double> theWeights)
    {
        if (theWeights.IsEmpty())
        {
            return false;
        }
        foreach (double aWeight in theWeights.AsSpan())
        {
            if (aWeight <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    private static void reparametrizeSurface(BSplineSurface theSurface)
    {
        NCollection_Array1<double> aUKnots = new(theSurface.InternalUKnots());
        NCollection_Array1<double> aVKnots = new(theSurface.InternalVKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, aUKnots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, aVKnots);
        theSurface.SetUKnots(aUKnots);
        theSurface.SetVKnots(aVKnots);
    }

    private static bool hasRationalCurve(NCollection_Array1<BSplineCurve3D?> theCurves)
    {
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is not null && theCurves.At(aCurveIdx)!.IsRational)
            {
                return true;
            }
        }
        return false;
    }

    private static bool hasPeriodicCurve(NCollection_Array1<BSplineCurve3D?> theCurves)
    {
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is null || theCurves.At(aCurveIdx)!.IsPeriodic)
            {
                return true;
            }
        }
        return false;
    }

    private static bool isCompatibleCurveFamily(NCollection_Array1<BSplineCurve3D?> theCurves)
    {
        if (theCurves.IsEmpty() || theCurves.First() is null)
        {
            return false;
        }

        BSplineCurve3D aReference = theCurves.First()!;
        bool isRational = aReference.IsRational;
        for (int aCurveIdx = 1; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            BSplineCurve3D? aCurve = theCurves.At(aCurveIdx);
            if (aCurve is null || aCurve.IsRational != isRational || aCurve.Degree() != aReference.Degree()
                || aCurve.PoleCount != aReference.PoleCount || aCurve.KnotCount != aReference.KnotCount)
            {
                return false;
            }
            for (int aKnotIdx = 0; aKnotIdx < aCurve.KnotCount; ++aKnotIdx)
            {
                int aKnot = aKnotIdx + 1;
                if (aCurve.Multiplicity(aKnot) != aReference.Multiplicity(aKnot)
                    || Math.Abs(aCurve.Knot(aKnot) - aReference.Knot(aKnot)) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static bool prepareCurveFamily(NCollection_Array1<BSplineCurve3D?> theCurves)
    {
        if (hasRationalCurve(theCurves))
        {
            return isCompatibleCurveFamily(theCurves);
        }

        GeomFill_Profiler aProfiler = new();
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is null)
            {
                return false;
            }
            aProfiler.AddCurve(theCurves.At(aCurveIdx)!);
        }

        aProfiler.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            theCurves.ChangeAt(aCurveIdx) = aProfiler.InternalCurve(aCurveIdx + 1) as BSplineCurve3D;
            if (theCurves.At(aCurveIdx) is null)
            {
                return false;
            }
        }

        return true;
    }

    private static void makeInterpolationBasis(NCollection_Array1<double> theParameters, out int theDegree,
                                               NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                                               NCollection_Array1<double> theFlatKnots)
    {
        int aNbParams = theParameters.Size();
        theDegree = Min(3, aNbParams - 1);

        int aNbInternalKnots = aNbParams - theDegree - 1;
        theKnots.Resize(1, aNbInternalKnots + 2, false);
        theMults.Resize(1, aNbInternalKnots + 2, false);

        theKnots.ChangeAt(0) = theParameters.At(0);
        theMults.ChangeAt(0) = theDegree + 1;
        for (int aKnotIdx = 0; aKnotIdx < aNbInternalKnots; ++aKnotIdx)
        {
            double aSum = 0.0;
            for (int aParamOffset = 1; aParamOffset <= theDegree; ++aParamOffset)
            {
                aSum += theParameters.At(aKnotIdx + aParamOffset);
            }
            theKnots.ChangeAt(aKnotIdx + 1) = aSum / (double)theDegree;
            theMults.ChangeAt(aKnotIdx + 1) = 1;
        }
        theKnots.ChangeAt(theKnots.Size() - 1) = theParameters.At(theParameters.Size() - 1);
        theMults.ChangeAt(theMults.Size() - 1) = theDegree + 1;

        theFlatKnots.Resize(1, theDegree + aNbParams + 1, false);
        int aFlatIdx = 0;
        for (int aKnotIdx = 0; aKnotIdx < theKnots.Size(); ++aKnotIdx)
        {
            for (int aMultIdx = 0; aMultIdx < theMults.At(aKnotIdx); ++aMultIdx)
            {
                theFlatKnots.ChangeAt(aFlatIdx++) = theKnots.At(aKnotIdx);
            }
        }
    }

    private static bool interpolatePoles(int theDegree, NCollection_Array1<double> theFlatKnots,
                                         NCollection_Array1<double> theParameters,
                                         NCollection_Array1<int> theContactOrders, NCollection_Array1<Point3D> thePoles,
                                         NCollection_Array1<double> theWeights)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(theDegree, theFlatKnots, theParameters, theContactOrders, thePoles, theWeights,
                             out int anInversionProblem);
        return anInversionProblem == 0;
    }

    private static double poleWeight(BSplineCurve3D theCurve, int thePoleIdx)
    {
        return theCurve.IsRational ? theCurve.Weight(thePoleIdx) : 1.0;
    }

    private static bool sameKnotRange(NCollection_Array1<double> theKnots1, NCollection_Array1<double> theKnots2,
                                      out double theStart, out double theEnd)
    {
        theStart = theKnots1.First();
        theEnd = theKnots1.Last();
        return Math.Abs(theStart - theKnots2.First()) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
               && Math.Abs(theEnd - theKnots2.Last()) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
               && theStart < theEnd - global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Candidate knot with multiplicity collected before tolerance compaction.</summary>
    private struct KnotEntry
    {
        public double Parameter;
        public int Multiplicity;
    }

    private readonly struct KnotEntryLess : Heap.ILess<KnotEntry>
    {
        public bool Less(KnotEntry theLeft, KnotEntry theRight) => theLeft.Parameter < theRight.Parameter;
    }

    /// <summary>Interpolation basis used to skin one curve family.</summary>
    private sealed class SkinningBasis
    {
        public int Degree = 0;
        public NCollection_Array1<double> Knots = new();
        public NCollection_Array1<int> Mults = new();
        public NCollection_Array1<double> FlatKnots = new();

        public void Init(NCollection_Array1<double> theParameters)
        {
            makeInterpolationBasis(theParameters, out Degree, Knots, Mults, FlatKnots);
        }
    }

    /// <summary>Evaluates the homogeneous denominator of a B-spline surface.</summary>
    private static double denominatorValue(BSplineSurface theSurface, double theU, double theV)
    {
        SurfaceBSplSLib.HomogeneousD0(theU, theV, 0, 0, theSurface.InternalPoles(), theSurface.InternalWeightsArray(),
                               theSurface.InternalUKnots(), theSurface.InternalVKnots(),
                               theSurface.InternalUMultiplicities(), theSurface.InternalVMultiplicities(),
                               theSurface.UDegree(), theSurface.VDegree(), true, true, false, false,
                               out double aWeight, out Point3D _);
        return aWeight;
    }

    /// <summary>Multiplier for the product of two surface denominators.</summary>
    private sealed class DenominatorProductEvaluator : SurfaceEvaluatorFunction
    {
        private readonly BSplineSurface? myFirst;
        private readonly BSplineSurface? mySecond;

        public DenominatorProductEvaluator(BSplineSurface? theFirst, BSplineSurface? theSecond)
        {
            myFirst = theFirst;
            mySecond = theSecond;
        }

        public override void Evaluate(int theDerivativeRequest, double theU, double theV, out double theResult,
                                      out int theErrorCode)
        {
            if (theDerivativeRequest != 0 || myFirst is null || mySecond is null)
            {
                theResult = 0.0; // OCCT leaves it unwritten
                theErrorCode = 1;
                return;
            }

            theResult = denominatorValue(myFirst, theU, theV) * denominatorValue(mySecond, theU, theV);
            theErrorCode = 0;
        }
    }

    private static bool makeTripleProductBasis(NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                                               int theDegree, out int theProductDegree,
                                               NCollection_Array1<double> theProductKnots,
                                               NCollection_Array1<int> theProductMults,
                                               NCollection_Array1<double> theProductFlatKnots)
    {
        theProductDegree = 3 * theDegree;
        if (theProductDegree > BSplineSurface.MaxDegree())
        {
            return false;
        }

        theProductKnots.Resize(1, theKnots.Size(), false);
        theProductMults.Resize(1, theMults.Size(), false);
        int aFlatLength = 0;
        for (int aKnotIdx = 0; aKnotIdx < theKnots.Size(); ++aKnotIdx)
        {
            theProductKnots.ChangeAt(aKnotIdx) = theKnots.At(aKnotIdx);
            if (aKnotIdx == 0 || aKnotIdx + 1 == theKnots.Size())
            {
                theProductMults.ChangeAt(aKnotIdx) = theProductDegree + 1;
            }
            else
            {
                theProductMults.ChangeAt(aKnotIdx) = Min(theProductDegree, 2 * theDegree + theMults.At(aKnotIdx));
            }
            aFlatLength += theProductMults.At(aKnotIdx);
        }

        theProductFlatKnots.Resize(1, aFlatLength, false);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(theProductKnots, theProductMults, theProductFlatKnots);
        return true;
    }

    private static bool multiplyByDenominators(BSplineSurface theSurface, BSplineSurface theFirstDenominator,
                                               BSplineSurface theSecondDenominator, int theUDegree,
                                               int theVDegree, NCollection_Array1<double> theUFlatKnots,
                                               NCollection_Array1<double> theVFlatKnots,
                                               NCollection_Array2<Point3D> theNumerator,
                                               NCollection_Array2<double> theDenominator)
    {
        DenominatorProductEvaluator aMultiplier = new(theFirstDenominator, theSecondDenominator);

        SurfaceBSplSLib.FunctionMultiply(aMultiplier, theSurface.UDegree(), theSurface.VDegree(), theSurface.InternalUKnots(),
                                  theSurface.InternalVKnots(), theSurface.InternalUMultiplicities(),
                                  theSurface.InternalVMultiplicities(), theSurface.InternalPoles(),
                                  theSurface.InternalWeightsArray(), theUFlatKnots, theVFlatKnots, theUDegree,
                                  theVDegree, theNumerator, theDenominator, out int aStatus);
        return aStatus == 0;
    }

    private static BSplineSurface? makeProfileSkin(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                        NCollection_Array1<double> theProfileParameters,
                                                        int theVDegree, NCollection_Array1<double> theVKnots,
                                                        NCollection_Array1<int> theVMults,
                                                        NCollection_Array1<double> theVFlatKnots)
    {
        BSplineCurve3D aBaseProfile = theProfiles.At(0)!;
        NCollection_Array2<Point3D> aPoles = new(1, aBaseProfile.PoleCount, 1, theProfiles.Length());
        NCollection_Array2<double> aWeights = new(1, aBaseProfile.PoleCount, 1, theProfiles.Length());
        NCollection_Array1<Point3D> aColumn = new(1, theProfiles.Length());
        NCollection_Array1<double> aColumnWeights = new(1, theProfiles.Length());
        NCollection_Array1<int> aContactOrders = new(1, theProfileParameters.Size());
        aContactOrders.Init(0);

        for (int aUPoleIdx = 1; aUPoleIdx <= aBaseProfile.PoleCount; ++aUPoleIdx)
        {
            for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
            {
                aColumn.ChangeAt(aProfileIdx) = theProfiles.At(aProfileIdx)!.Pole(aUPoleIdx);
                aColumnWeights.ChangeAt(aProfileIdx) = poleWeight(theProfiles.At(aProfileIdx)!, aUPoleIdx);
            }
            if (!interpolatePoles(theVDegree, theVFlatKnots, theProfileParameters, aContactOrders, aColumn,
                                  aColumnWeights))
            {
                return null;
            }
            for (int aVIdx = 0; aVIdx < theProfiles.Size(); ++aVIdx)
            {
                aPoles.ChangeAt(aUPoleIdx - 1, aVIdx) = aColumn.At(aVIdx);
                aWeights.ChangeAt(aUPoleIdx - 1, aVIdx) = aColumnWeights.At(aVIdx);
            }
        }

        return new BSplineSurface(aPoles, aWeights, aBaseProfile.InternalKnots(), theVKnots,
                                       aBaseProfile.InternalMultiplicities(), theVMults, aBaseProfile.Degree(),
                                       theVDegree);
    }

    private static BSplineSurface? makeGuideSkin(NCollection_Array1<BSplineCurve3D?> theGuides,
                                                      NCollection_Array1<double> theGuideParameters, int theUDegree,
                                                      NCollection_Array1<double> theUKnots,
                                                      NCollection_Array1<int> theUMults,
                                                      NCollection_Array1<double> theUFlatKnots)
    {
        BSplineCurve3D aBaseGuide = theGuides.At(0)!;
        NCollection_Array2<Point3D> aPoles = new(1, theGuides.Length(), 1, aBaseGuide.PoleCount);
        NCollection_Array2<double> aWeights = new(1, theGuides.Length(), 1, aBaseGuide.PoleCount);
        NCollection_Array1<Point3D> aRow = new(1, theGuides.Length());
        NCollection_Array1<double> aRowWeights = new(1, theGuides.Length());
        NCollection_Array1<int> aContactOrders = new(1, theGuideParameters.Size());
        aContactOrders.Init(0);

        for (int aVPoleIdx = 1; aVPoleIdx <= aBaseGuide.PoleCount; ++aVPoleIdx)
        {
            for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
            {
                aRow.ChangeAt(aGuideIdx) = theGuides.At(aGuideIdx)!.Pole(aVPoleIdx);
                aRowWeights.ChangeAt(aGuideIdx) = poleWeight(theGuides.At(aGuideIdx)!, aVPoleIdx);
            }
            if (!interpolatePoles(theUDegree, theUFlatKnots, theGuideParameters, aContactOrders, aRow, aRowWeights))
            {
                return null;
            }
            for (int aUIdx = 0; aUIdx < theGuides.Size(); ++aUIdx)
            {
                aPoles.ChangeAt(aUIdx, aVPoleIdx - 1) = aRow.At(aUIdx);
                aWeights.ChangeAt(aUIdx, aVPoleIdx - 1) = aRowWeights.At(aUIdx);
            }
        }

        return new BSplineSurface(aPoles, aWeights, theUKnots, aBaseGuide.InternalKnots(), theUMults,
                                       aBaseGuide.InternalMultiplicities(), theUDegree, aBaseGuide.Degree());
    }

    private static double normalizedKnot(double theStart, double theEnd, double theKnot)
    {
        if (Math.Abs(theKnot - theStart) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return theStart;
        }
        if (Math.Abs(theKnot - theEnd) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return theEnd;
        }
        return theKnot;
    }

    /// <summary>Adds knots from one basis to a flat collection, clamping boundary values.</summary>
    private static void collectKnots(int theDegree, double theStart, double theEnd, NCollection_Array1<double> theKnots,
                                     NCollection_Array1<int> theMults, NCollection_LinearVector<KnotEntry> theEntries)
    {
        for (int aKnotIdx = 0; aKnotIdx < theKnots.Size(); ++aKnotIdx)
        {
            double aKnot = normalizedKnot(theStart, theEnd, theKnots.At(aKnotIdx));
            bool isBoundary = aKnot == theStart || aKnot == theEnd;
            int aMaxMult = isBoundary ? theDegree + 1 : theDegree;
            KnotEntry anEntry;
            anEntry.Parameter = aKnot;
            anEntry.Multiplicity = Min(theMults.At(aKnotIdx), aMaxMult);
            theEntries.Append(anEntry);
        }
    }

    /// <summary>Sorts candidate knots and keeps the largest multiplicity in each tolerance group.</summary>
    private static void compactKnots(NCollection_LinearVector<KnotEntry> theEntries,
                                     NCollection_LinearVector<double> theKnots, NCollection_LinearVector<int> theMults)
    {
        if (theEntries.IsEmpty())
        {
            return;
        }

        Sorting.Sort(theEntries.AsSpan(), new KnotEntryLess());

        KnotEntry aCurrent = theEntries.Value(0);
        for (int anEntryIdx = 1; anEntryIdx < theEntries.Size(); ++anEntryIdx)
        {
            KnotEntry anEntry = theEntries.Value(anEntryIdx);
            if (Math.Abs(anEntry.Parameter - aCurrent.Parameter) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aCurrent.Multiplicity = Max(aCurrent.Multiplicity, anEntry.Multiplicity);
                continue;
            }

            theKnots.Append(aCurrent.Parameter);
            theMults.Append(aCurrent.Multiplicity);
            aCurrent = anEntry;
        }

        theKnots.Append(aCurrent.Parameter);
        theMults.Append(aCurrent.Multiplicity);
    }

    private static void uniteKnots(int theDegree, NCollection_Array1<double> theKnots1, NCollection_Array1<int> theMults1,
                                   NCollection_Array1<double> theKnots2, NCollection_Array1<int> theMults2,
                                   out NCollection_HArray1<double>? theMergedKnots,
                                   out NCollection_HArray1<int>? theMergedMults)
    {
        // OCCT leaves the caller's null handles unchanged on failure.
        theMergedKnots = null;
        theMergedMults = null;
        if (!sameKnotRange(theKnots1, theKnots2, out double aStart, out double anEnd))
        {
            return;
        }

        NCollection_LinearVector<KnotEntry> aEntries = new();
        collectKnots(theDegree, aStart, anEnd, theKnots1, theMults1, aEntries);
        collectKnots(theDegree, aStart, anEnd, theKnots2, theMults2, aEntries);

        NCollection_LinearVector<double> aKnots = new();
        NCollection_LinearVector<int> aMults = new();
        compactKnots(aEntries, aKnots, aMults);

        if (aKnots.IsEmpty() || aMults.IsEmpty())
        {
            return;
        }

        NCollection_Array1<double> aKnotArray = GordonUtilities.toOneBasedArrayView(aKnots);
        NCollection_Array1<int> aMultArray = GordonUtilities.toOneBasedArrayView(aMults);
        theMergedKnots = new NCollection_HArray1<double>(aKnotArray);
        theMergedMults = new NCollection_HArray1<int>(aMultArray);
    }

    private static bool alignSurfaces(BSplineSurface theProfileSurface, BSplineSurface theGuideSurface,
                                      BSplineSurface theReferenceSurface)
    {
        int aUDegree = Max(Max(theProfileSurface.UDegree(), theGuideSurface.UDegree()), theReferenceSurface.UDegree());
        int aVDegree = Max(Max(theProfileSurface.VDegree(), theGuideSurface.VDegree()), theReferenceSurface.VDegree());

        theProfileSurface.IncreaseDegree(aUDegree, aVDegree);
        theGuideSurface.IncreaseDegree(aUDegree, aVDegree);
        theReferenceSurface.IncreaseDegree(aUDegree, aVDegree);

        uniteKnots(aUDegree, theProfileSurface.InternalUKnots(), theProfileSurface.InternalUMultiplicities(),
                   theGuideSurface.InternalUKnots(), theGuideSurface.InternalUMultiplicities(), out var aUKnots12,
                   out var aUMults12);
        if (aUKnots12 is null || aUMults12 is null)
        {
            return false;
        }

        uniteKnots(aUDegree, aUKnots12.Array1(), aUMults12.Array1(), theReferenceSurface.InternalUKnots(),
                   theReferenceSurface.InternalUMultiplicities(), out var aUKnots, out var aUMults);
        if (aUKnots is null || aUMults is null)
        {
            return false;
        }

        uniteKnots(aVDegree, theProfileSurface.InternalVKnots(), theProfileSurface.InternalVMultiplicities(),
                   theGuideSurface.InternalVKnots(), theGuideSurface.InternalVMultiplicities(), out var aVKnots12,
                   out var aVMults12);
        if (aVKnots12 is null || aVMults12 is null)
        {
            return false;
        }

        uniteKnots(aVDegree, aVKnots12.Array1(), aVMults12.Array1(), theReferenceSurface.InternalVKnots(),
                   theReferenceSurface.InternalVMultiplicities(), out var aVKnots, out var aVMults);
        if (aVKnots is null || aVMults is null)
        {
            return false;
        }

        theProfileSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        theGuideSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        theReferenceSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);

        theProfileSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        theGuideSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        theReferenceSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        return true;
    }

    /// <summary>Builds the final network surface by correcting the profile skin with guide deviations.</summary>
    private static BSplineSurface? makeCorrectedProfileSkin(BSplineSurface theProfileSurface,
                                                                 BSplineSurface theGuideSurface,
                                                                 BSplineSurface theReferenceSurface,
                                                                 ref ResultStatus theStatus)
    {
        if (theProfileSurface.UPoleCount != theGuideSurface.UPoleCount
            || theProfileSurface.UPoleCount != theReferenceSurface.UPoleCount
            || theProfileSurface.VPoleCount != theGuideSurface.VPoleCount
            || theProfileSurface.VPoleCount != theReferenceSurface.VPoleCount)
        {
            theStatus = ResultStatus.RationalConstructionFailed;
            return null;
        }

        if (!theProfileSurface.IsURational && !theProfileSurface.IsVRational && !theGuideSurface.IsURational
            && !theGuideSurface.IsVRational && !theReferenceSurface.IsURational
            && !theReferenceSurface.IsVRational)
        {
            Span<Point3D> aProfilePoles = theProfileSurface.InternalPoles().AsSpan();
            Span<Point3D> aGuidePoles = theGuideSurface.InternalPoles().AsSpan();
            Span<Point3D> aReferencePoles = theReferenceSurface.InternalPoles().AsSpan();
            NCollection_Array2<Point3D> aResultPoles = new(theProfileSurface.InternalPoles());
            Span<Point3D> aResult = aResultPoles.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aResult.Length; ++aPoleIdx)
            {
                Xyz aPoint = aProfilePoles[aPoleIdx].XYZ() + aGuidePoles[aPoleIdx].XYZ()
                                - aReferencePoles[aPoleIdx].XYZ();
                aResult[aPoleIdx] = new Point3D(aPoint);
            }

            return new BSplineSurface(aResultPoles, theProfileSurface.InternalUKnots(),
                                           theProfileSurface.InternalVKnots(),
                                           theProfileSurface.InternalUMultiplicities(),
                                           theProfileSurface.InternalVMultiplicities(), theProfileSurface.UDegree(),
                                           theProfileSurface.VDegree());
        }

        NCollection_Array1<double> aUKnots = new();
        NCollection_Array1<int> aUMults = new();
        NCollection_Array1<double> aUFlatKnots = new();
        NCollection_Array1<double> aVKnots = new();
        NCollection_Array1<int> aVMults = new();
        NCollection_Array1<double> aVFlatKnots = new();
        int aUDegree = 0;
        int aVDegree = 0;
        if (!makeTripleProductBasis(theProfileSurface.InternalUKnots(), theProfileSurface.InternalUMultiplicities(),
                                    theProfileSurface.UDegree(), out aUDegree, aUKnots, aUMults, aUFlatKnots)
            || !makeTripleProductBasis(theProfileSurface.InternalVKnots(),
                                       theProfileSurface.InternalVMultiplicities(), theProfileSurface.VDegree(),
                                       out aVDegree, aVKnots, aVMults, aVFlatKnots))
        {
            theStatus = ResultStatus.RationalDegreeOverflow;
            return null;
        }

        int aNbUPoles = aUFlatKnots.Length() - aUDegree - 1;
        int aNbVPoles = aVFlatKnots.Length() - aVDegree - 1;
        if (aNbUPoles < 2 || aNbVPoles < 2)
        {
            theStatus = ResultStatus.RationalConstructionFailed;
            return null;
        }

        NCollection_Array2<Point3D> aProfileNumerator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aProfileDenominator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<Point3D> aGuideNumerator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aGuideDenominator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<Point3D> aReferenceNumerator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aReferenceDenominator = new(1, aNbUPoles, 1, aNbVPoles);

        if (!multiplyByDenominators(theProfileSurface, theGuideSurface, theReferenceSurface, aUDegree, aVDegree,
                                    aUFlatKnots, aVFlatKnots, aProfileNumerator, aProfileDenominator)
            || !multiplyByDenominators(theGuideSurface, theProfileSurface, theReferenceSurface, aUDegree, aVDegree,
                                       aUFlatKnots, aVFlatKnots, aGuideNumerator, aGuideDenominator)
            || !multiplyByDenominators(theReferenceSurface, theProfileSurface, theGuideSurface, aUDegree, aVDegree,
                                       aUFlatKnots, aVFlatKnots, aReferenceNumerator, aReferenceDenominator))
        {
            theStatus = ResultStatus.RationalConstructionFailed;
            return null;
        }

        NCollection_Array2<Point3D> aResultPoles2 = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aResultWeights = new(1, aNbUPoles, 1, aNbVPoles);
        Span<Point3D> aPoles = aResultPoles2.AsSpan();
        Span<double> aWeights = aResultWeights.AsSpan();
        Span<double> aProfileDen = aProfileDenominator.AsSpan();
        Span<double> aGuideDen = aGuideDenominator.AsSpan();
        Span<double> aReferenceDen = aReferenceDenominator.AsSpan();
        Span<Point3D> aProfileNum = aProfileNumerator.AsSpan();
        Span<Point3D> aGuideNum = aGuideNumerator.AsSpan();
        Span<Point3D> aReferenceNum = aReferenceNumerator.AsSpan();
        for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
        {
            double aWeight = aProfileDen[aPoleIdx];
            if (aWeight <= global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(aGuideDen[aPoleIdx] - aWeight) > global::Kerf.Engine.Foundation.Precision.Confusion()
                || Math.Abs(aReferenceDen[aPoleIdx] - aWeight) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                theStatus = ResultStatus.RationalConstructionFailed;
                return null;
            }

            Xyz aNumerator = aProfileNum[aPoleIdx].XYZ() + aGuideNum[aPoleIdx].XYZ() - aReferenceNum[aPoleIdx].XYZ();
            aWeights[aPoleIdx] = aWeight;
            aPoles[aPoleIdx] = new Point3D(aNumerator / aWeight);
        }

        return new BSplineSurface(aResultPoles2, aResultWeights, aUKnots, aVKnots, aUMults, aVMults, aUDegree,
                                       aVDegree);
    }

    private static bool canSetPeriodic(BSplineSurface theSurface, bool theAlongU, double theTolerance)
    {
        int aNbChecks = theAlongU ? theSurface.VPoleCount : theSurface.UPoleCount;
        for (int aCheckIdx = 1; aCheckIdx <= aNbChecks; ++aCheckIdx)
        {
            Point3D aFirstPole = theAlongU ? theSurface.Pole(1, aCheckIdx) : theSurface.Pole(aCheckIdx, 1);
            Point3D aLastPole = theAlongU ? theSurface.Pole(theSurface.UPoleCount, aCheckIdx)
                                         : theSurface.Pole(aCheckIdx, theSurface.VPoleCount);
            if (aFirstPole.Distance(aLastPole) > theTolerance)
            {
                return false;
            }
        }
        return true;
    }

    private static bool isReadyToBuild(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                       NCollection_Array1<BSplineCurve3D?> theGuides,
                                       NCollection_Array1<double> theProfileParameters,
                                       NCollection_Array1<double> theGuideParameters,
                                       NCollection_Array2<Point3D> theIntersectionPoints,
                                       NCollection_Array2<double> theIntersectionWeights)
    {
        return theProfiles.Length() >= 2 && theGuides.Length() >= 2 && !hasPeriodicCurve(theProfiles)
               && !hasPeriodicCurve(theGuides) && theProfiles.Length() == theProfileParameters.Length()
               && theGuides.Length() == theGuideParameters.Length()
               && theIntersectionPoints.ColLength() == theGuideParameters.Length()
               && theIntersectionPoints.RowLength() == theProfileParameters.Length()
               && theIntersectionWeights.ColLength() == theGuideParameters.Length()
               && theIntersectionWeights.RowLength() == theProfileParameters.Length()
               && checkParameters(theProfileParameters) && checkParameters(theGuideParameters)
               && checkWeights(theIntersectionWeights);
    }

    // Creates a seam guide through the start points of all closed profiles.
    private static BSplineCurve3D? makeProfileSeamGuide(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                           NCollection_Array1<double> theProfileParameters)
    {
        NCollection_HArray1<Point3D> aPoints = new(1, theProfiles.Length());
        NCollection_HArray1<double> aParameters = new(1, theProfileParameters.Length());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            BSplineCurve3D aProfile = theProfiles.At(aProfileIdx)!;
            aPoints.SetValue(aProfileIdx + 1, aProfile.Value(aProfile.FirstParameter()));
            aParameters.SetValue(aProfileIdx + 1, theProfileParameters.At(aProfileIdx));
        }

        GeomAPI_Interpolate anInterpolator = new(aPoints, aParameters, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpolator.Perform();
        return anInterpolator.IsDone() ? anInterpolator.Curve() : null;
    }

    // Completes a closed U-direction interpolation basis at both profile endpoints.
    private static bool completeClosedGuideSeam(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                NCollection_Array1<double> theProfileParameters,
                                                NCollection_Array1<BSplineCurve3D?> theGuides,
                                                NCollection_Array1<double> theGuideParameters,
                                                NCollection_Array2<Point3D> theIntersectionPoints,
                                                NCollection_Array2<double> theIntersectionWeights,
                                                ref NCollection_Array1<BSplineCurve3D?> theClosedGuides,
                                                ref NCollection_Array1<double> theClosedGuideParameters,
                                                ref NCollection_Array2<Point3D> theClosedIntersectionPoints,
                                                ref NCollection_Array2<double> theClosedIntersectionWeights)
    {
        double aProfileStart = theProfiles.At(0)!.FirstParameter();
        double aProfileEnd = theProfiles.At(0)!.LastParameter();
        bool needsStartSeam = theGuideParameters.First() > aProfileStart + global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool needsEndSeam = theGuideParameters.Last() < aProfileEnd - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (!needsStartSeam && !needsEndSeam)
        {
            return true;
        }

        BSplineCurve3D? aSeamGuide = null;
        if (needsStartSeam)
        {
            aSeamGuide = makeProfileSeamGuide(theProfiles, theProfileParameters);
            if (aSeamGuide is null)
            {
                return false;
            }
        }

        int aGuideOffset = needsStartSeam ? 1 : 0;
        int aNbGuides = theGuides.Size() + aGuideOffset + (needsEndSeam ? 1 : 0);
        theClosedGuides = new NCollection_Array1<BSplineCurve3D?>(1, aNbGuides);
        theClosedGuideParameters = new NCollection_Array1<double>(1, aNbGuides);
        if (needsStartSeam)
        {
            theClosedGuides.ChangeAt(0) = aSeamGuide;
            theClosedGuideParameters.ChangeAt(0) = aProfileStart;
        }
        for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
        {
            theClosedGuides.ChangeAt(aGuideIdx + aGuideOffset) = theGuides.At(aGuideIdx);
            theClosedGuideParameters.ChangeAt(aGuideIdx + aGuideOffset) = theGuideParameters.At(aGuideIdx);
        }
        if (needsEndSeam)
        {
            theClosedGuides.ChangeAt(aNbGuides - 1) = needsStartSeam ? aSeamGuide : theGuides.At(0);
            theClosedGuideParameters.ChangeAt(aNbGuides - 1) = aProfileEnd;
        }

        theClosedIntersectionPoints = new NCollection_Array2<Point3D>(1, aNbGuides, 1, theIntersectionPoints.NbColumns());
        theClosedIntersectionWeights = new NCollection_Array2<double>(1, aNbGuides, 1, theIntersectionWeights.NbColumns());
        if (needsStartSeam)
        {
            for (int aProfileIdx = 0; aProfileIdx < theIntersectionPoints.NbColumns(); ++aProfileIdx)
            {
                theClosedIntersectionPoints.ChangeAt(0, aProfileIdx) =
                  theProfiles.At(aProfileIdx)!.Value(theProfiles.At(aProfileIdx)!.FirstParameter());
                theClosedIntersectionWeights.ChangeAt(0, aProfileIdx) = poleWeight(theProfiles.At(aProfileIdx)!, 1);
            }
        }
        for (int aGuideIdx = 0; aGuideIdx < theIntersectionPoints.NbRows(); ++aGuideIdx)
        {
            for (int aProfileIdx = 0; aProfileIdx < theIntersectionPoints.NbColumns(); ++aProfileIdx)
            {
                theClosedIntersectionPoints.ChangeAt(aGuideIdx + aGuideOffset, aProfileIdx) =
                  theIntersectionPoints.At(aGuideIdx, aProfileIdx);
                theClosedIntersectionWeights.ChangeAt(aGuideIdx + aGuideOffset, aProfileIdx) =
                  theIntersectionWeights.At(aGuideIdx, aProfileIdx);
            }
        }
        if (needsEndSeam)
        {
            for (int aProfileIdx = 0; aProfileIdx < theIntersectionPoints.NbColumns(); ++aProfileIdx)
            {
                theClosedIntersectionPoints.ChangeAt(aNbGuides - 1, aProfileIdx) =
                  theClosedIntersectionPoints.At(0, aProfileIdx);
                theClosedIntersectionWeights.ChangeAt(aNbGuides - 1, aProfileIdx) =
                  theClosedIntersectionWeights.At(0, aProfileIdx);
            }
        }
        return true;
    }

    private static BSplineSurface? makeNetworkSurface(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                           NCollection_Array1<BSplineCurve3D?> theGuides,
                                                           NCollection_Array1<double> theProfileParameters,
                                                           NCollection_Array1<double> theGuideParameters,
                                                           NCollection_Array2<Point3D> theIntersectionPoints,
                                                           NCollection_Array2<double> theIntersectionWeights,
                                                           bool theIsUClosed, ref ResultStatus theStatus)
    {
        theStatus = ResultStatus.ConstructionFailed;

        NCollection_Array1<BSplineCurve3D?> aClosedGuides = new();
        NCollection_Array1<double> aClosedGuideParameters = new();
        NCollection_Array2<Point3D> aClosedIntersectionPoints = new();
        NCollection_Array2<double> aClosedIntersectionWeights = new();
        bool shouldCompleteUSeam =
          theIsUClosed
          && (theGuideParameters.First() > theProfiles.At(0)!.FirstParameter() + global::Kerf.Engine.Foundation.Precision.PConfusion()
              || theGuideParameters.Last() < theProfiles.At(0)!.LastParameter() - global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (shouldCompleteUSeam
            && !completeClosedGuideSeam(theProfiles, theProfileParameters, theGuides, theGuideParameters,
                                        theIntersectionPoints, theIntersectionWeights, ref aClosedGuides,
                                        ref aClosedGuideParameters, ref aClosedIntersectionPoints,
                                        ref aClosedIntersectionWeights))
        {
            theStatus = ResultStatus.SkinningFailed;
            return null;
        }
        if (shouldCompleteUSeam && !prepareCurveFamily(aClosedGuides))
        {
            theStatus = ResultStatus.CurveCompatibilityFailed;
            return null;
        }
        NCollection_Array1<BSplineCurve3D?> aGuides = shouldCompleteUSeam ? aClosedGuides : theGuides;
        NCollection_Array1<double> aGuideParameters = shouldCompleteUSeam ? aClosedGuideParameters : theGuideParameters;
        NCollection_Array2<Point3D> aIntersectionPoints =
          shouldCompleteUSeam ? aClosedIntersectionPoints : theIntersectionPoints;
        NCollection_Array2<double> aIntersectionWeights =
          shouldCompleteUSeam ? aClosedIntersectionWeights : theIntersectionWeights;

        SkinningBasis anUBasis = new();
        anUBasis.Init(aGuideParameters);

        SkinningBasis aVBasis = new();
        aVBasis.Init(theProfileParameters);

        BSplineSurface? aProfileSurface = makeProfileSkin(theProfiles, theProfileParameters, aVBasis.Degree,
                                                               aVBasis.Knots, aVBasis.Mults, aVBasis.FlatKnots);
        if (aProfileSurface is null)
        {
            theStatus = ResultStatus.SkinningFailed;
            return null;
        }

        BSplineSurface? aGuideSurface = makeGuideSkin(aGuides, aGuideParameters, anUBasis.Degree, anUBasis.Knots,
                                                           anUBasis.Mults, anUBasis.FlatKnots);
        if (aGuideSurface is null)
        {
            theStatus = ResultStatus.SkinningFailed;
            return null;
        }

        // The reference surface starts from the same contact grid validated during preparation.
        NCollection_Array2<Point3D> aReferencePoles = new(aIntersectionPoints);
        NCollection_Array2<double> aReferenceWeights = new(aIntersectionWeights);
        bool isReferencePolynomial = hasRationalCurve(theProfiles) != hasRationalCurve(aGuides);
        if (isReferencePolynomial)
        {
            aReferenceWeights.Init(1.0);
        }
        else
        {
            Span<Point3D> aPoles = aReferencePoles.AsSpan();
            Span<double> aWeights = aReferenceWeights.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
            {
                double aWeight = aWeights[aPoleIdx];
                aPoles[aPoleIdx] = new Point3D(aPoles[aPoleIdx].XYZ() * aWeight);
            }
        }

        SurfaceBSplSLib.Interpolate(anUBasis.Degree, aVBasis.Degree, anUBasis.FlatKnots, aVBasis.FlatKnots, aGuideParameters,
                             theProfileParameters, aReferencePoles, aReferenceWeights, out int anInversionProblem);
        if (anInversionProblem != 0)
        {
            theStatus = ResultStatus.ReferenceSurfaceFailed;
            return null;
        }
        if (!isReferencePolynomial)
        {
            Span<Point3D> aPoles = aReferencePoles.AsSpan();
            Span<double> aWeights = aReferenceWeights.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
            {
                double aWeight = aWeights[aPoleIdx];
                if (aWeight <= Constants.Resolution())
                {
                    theStatus = ResultStatus.ReferenceSurfaceFailed;
                    return null;
                }
                aPoles[aPoleIdx] = new Point3D(aPoles[aPoleIdx].XYZ() / aWeight);
            }
        }

        BSplineSurface aReferenceSurface = new(aReferencePoles, aReferenceWeights, anUBasis.Knots, aVBasis.Knots,
                                                    anUBasis.Mults, aVBasis.Mults, anUBasis.Degree, aVBasis.Degree);
        if (!alignSurfaces(aProfileSurface, aGuideSurface, aReferenceSurface))
        {
            theStatus = ResultStatus.KnotAlignmentFailed;
            return null;
        }
        return makeCorrectedProfileSkin(aProfileSurface, aGuideSurface, aReferenceSurface, ref theStatus);
    }

    private static bool applyPeriodicity(BSplineSurface theSurface, bool theIsUClosed, bool theIsVClosed,
                                         double theTolerance)
    {
        double aTolerance = Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (theIsUClosed)
        {
            if (!canSetPeriodic(theSurface, true, aTolerance))
            {
                return false;
            }
            theSurface.SetUPeriodic();
        }

        if (theIsVClosed)
        {
            if (!canSetPeriodic(theSurface, false, aTolerance))
            {
                return false;
            }
            theSurface.SetVPeriodic();
        }
        return true;
    }

    /// <summary>Creates an empty network surface algorithm.</summary>
    public GeomFill_NetworkSurface()
    {
    }

    /// <summary>
    /// Initializes the algorithm with a compatible profile/guide B-spline network: profiles (U direction) located
    /// at the V parameters <paramref name="theProfileParameters"/>, guides (V direction) at the U parameters
    /// <paramref name="theGuideParameters"/>, the contact grid (guides by profiles) and its rational weights, the
    /// tolerance of the closed-seam checks and the closed directions. The arrays are copied; the curves are kept.
    /// </summary>
    public void Init(NCollection_Array1<BSplineCurve3D?> theProfiles, NCollection_Array1<BSplineCurve3D?> theGuides,
                     NCollection_Array1<double> theProfileParameters, NCollection_Array1<double> theGuideParameters,
                     NCollection_Array2<Point3D> theIntersectionPoints, NCollection_Array2<double> theIntersectionWeights,
                     double theTolerance, bool theIsUClosed, bool theIsVClosed)
    {
        myProfiles = new NCollection_Array1<BSplineCurve3D?>(theProfiles);
        myGuides = new NCollection_Array1<BSplineCurve3D?>(theGuides);
        myProfileParameters = new NCollection_Array1<double>(theProfileParameters);
        myGuideParameters = new NCollection_Array1<double>(theGuideParameters);
        myIntersectionPoints = new NCollection_Array2<Point3D>(theIntersectionPoints);
        myIntersectionWeights = new NCollection_Array2<double>(theIntersectionWeights);
        myTolerance = theTolerance;
        myIsUClosed = theIsUClosed;
        myIsVClosed = theIsVClosed;
        myStatus = ResultStatus.NotStarted;
        mySurface = null;
    }

    /// <summary>Performs the pole-based network surface construction.</summary>
    public void Perform()
    {
        myStatus = ResultStatus.NotStarted;
        mySurface = null;

        if (!isReadyToBuild(myProfiles, myGuides, myProfileParameters, myGuideParameters, myIntersectionPoints,
                            myIntersectionWeights))
        {
            myStatus = ResultStatus.InvalidInput;
            return;
        }

        try
        {
            if (!prepareCurveFamily(myProfiles) || !prepareCurveFamily(myGuides))
            {
                myStatus = ResultStatus.CurveCompatibilityFailed;
                return;
            }

            ResultStatus aSurfaceStatus = ResultStatus.ConstructionFailed;
            mySurface = makeNetworkSurface(myProfiles, myGuides, myProfileParameters, myGuideParameters,
                                           myIntersectionPoints, myIntersectionWeights, myIsUClosed,
                                           ref aSurfaceStatus);
            if (mySurface is null)
            {
                myStatus = aSurfaceStatus;
                return;
            }

            if (!applyPeriodicity(mySurface, myIsUClosed, myIsVClosed, myTolerance))
            {
                mySurface = null;
                myStatus = ResultStatus.PeriodicityFailed;
                return;
            }
            reparametrizeSurface(mySurface);
        }
        catch (FailureException)
        {
            mySurface = null;
            myStatus = ResultStatus.ConstructionFailed;
            return;
        }

        myStatus = ResultStatus.Done;
    }

    /// <summary>Returns true if the surface was successfully constructed.</summary>
    public bool IsDone() => myStatus == ResultStatus.Done;

    /// <summary>Returns the result state of the last Perform() call.</summary>
    public ResultStatus Status() => myStatus;

    /// <summary>The constructed surface, frozen. Throws StdFail_NotDone unless <see cref="Perform"/> succeeded.</summary>
    public BSplineSurface Surface()
    {
        if (!IsDone())
        {
            throw new NotDoneException("GeomFill_NetworkSurface::Surface");
        }
        mySurface!.Freeze();
        return mySurface;
    }
}
