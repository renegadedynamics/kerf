// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

    private NCollection_Array1<Geom_BSplineCurve?> myProfiles = new();
    private NCollection_Array1<Geom_BSplineCurve?> myGuides = new();
    private NCollection_Array1<double> myProfileParameters = new();
    private NCollection_Array1<double> myGuideParameters = new();
    private NCollection_Array2<gp_Pnt> myIntersectionPoints = new();
    private NCollection_Array2<double> myIntersectionWeights = new();
    private Geom_BSplineSurface? mySurface;
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
            if (theParameters.At(aParamIdx) <= theParameters.At(aParamIdx - 1) + Precision.PConfusion())
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
            if (aWeight <= Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    private static void reparametrizeSurface(Geom_BSplineSurface theSurface)
    {
        NCollection_Array1<double> aUKnots = new(theSurface.InternalUKnots());
        NCollection_Array1<double> aVKnots = new(theSurface.InternalVKnots());
        BSplCLib.Reparametrize(0.0, 1.0, aUKnots);
        BSplCLib.Reparametrize(0.0, 1.0, aVKnots);
        theSurface.SetUKnots(aUKnots);
        theSurface.SetVKnots(aVKnots);
    }

    private static bool hasRationalCurve(NCollection_Array1<Geom_BSplineCurve?> theCurves)
    {
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is not null && theCurves.At(aCurveIdx)!.IsRational())
            {
                return true;
            }
        }
        return false;
    }

    private static bool hasPeriodicCurve(NCollection_Array1<Geom_BSplineCurve?> theCurves)
    {
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is null || theCurves.At(aCurveIdx)!.IsPeriodic())
            {
                return true;
            }
        }
        return false;
    }

    private static bool isCompatibleCurveFamily(NCollection_Array1<Geom_BSplineCurve?> theCurves)
    {
        if (theCurves.IsEmpty() || theCurves.First() is null)
        {
            return false;
        }

        Geom_BSplineCurve aReference = theCurves.First()!;
        bool isRational = aReference.IsRational();
        for (int aCurveIdx = 1; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            Geom_BSplineCurve? aCurve = theCurves.At(aCurveIdx);
            if (aCurve is null || aCurve.IsRational() != isRational || aCurve.Degree() != aReference.Degree()
                || aCurve.NbPoles() != aReference.NbPoles() || aCurve.NbKnots() != aReference.NbKnots())
            {
                return false;
            }
            for (int aKnotIdx = 0; aKnotIdx < aCurve.NbKnots(); ++aKnotIdx)
            {
                int aKnot = aKnotIdx + 1;
                if (aCurve.Multiplicity(aKnot) != aReference.Multiplicity(aKnot)
                    || Math.Abs(aCurve.Knot(aKnot) - aReference.Knot(aKnot)) > Precision.PConfusion())
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static bool prepareCurveFamily(NCollection_Array1<Geom_BSplineCurve?> theCurves)
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

        aProfiler.Perform(Precision.PConfusion());
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            theCurves.ChangeAt(aCurveIdx) = aProfiler.InternalCurve(aCurveIdx + 1) as Geom_BSplineCurve;
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
                                         NCollection_Array1<int> theContactOrders, NCollection_Array1<gp_Pnt> thePoles,
                                         NCollection_Array1<double> theWeights)
    {
        BSplCLib.Interpolate(theDegree, theFlatKnots, theParameters, theContactOrders, thePoles, theWeights,
                             out int anInversionProblem);
        return anInversionProblem == 0;
    }

    private static double poleWeight(Geom_BSplineCurve theCurve, int thePoleIdx)
    {
        return theCurve.IsRational() ? theCurve.Weight(thePoleIdx) : 1.0;
    }

    private static bool sameKnotRange(NCollection_Array1<double> theKnots1, NCollection_Array1<double> theKnots2,
                                      out double theStart, out double theEnd)
    {
        theStart = theKnots1.First();
        theEnd = theKnots1.Last();
        return Math.Abs(theStart - theKnots2.First()) <= Precision.PConfusion()
               && Math.Abs(theEnd - theKnots2.Last()) <= Precision.PConfusion()
               && theStart < theEnd - Precision.PConfusion();
    }

    /// <summary>Candidate knot with multiplicity collected before tolerance compaction.</summary>
    private struct KnotEntry
    {
        public double Parameter;
        public int Multiplicity;
    }

    private readonly struct KnotEntryLess : std_heap.ILess<KnotEntry>
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
    private static double denominatorValue(Geom_BSplineSurface theSurface, double theU, double theV)
    {
        BSplSLib.HomogeneousD0(theU, theV, 0, 0, theSurface.InternalPoles(), theSurface.InternalWeightsArray(),
                               theSurface.InternalUKnots(), theSurface.InternalVKnots(),
                               theSurface.InternalUMultiplicities(), theSurface.InternalVMultiplicities(),
                               theSurface.UDegree(), theSurface.VDegree(), true, true, false, false,
                               out double aWeight, out gp_Pnt _);
        return aWeight;
    }

    /// <summary>Multiplier for the product of two surface denominators.</summary>
    private sealed class DenominatorProductEvaluator : BSplSLib_EvaluatorFunction
    {
        private readonly Geom_BSplineSurface? myFirst;
        private readonly Geom_BSplineSurface? mySecond;

        public DenominatorProductEvaluator(Geom_BSplineSurface? theFirst, Geom_BSplineSurface? theSecond)
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
        if (theProductDegree > Geom_BSplineSurface.MaxDegree())
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
        BSplCLib.KnotSequence(theProductKnots, theProductMults, theProductFlatKnots);
        return true;
    }

    private static bool multiplyByDenominators(Geom_BSplineSurface theSurface, Geom_BSplineSurface theFirstDenominator,
                                               Geom_BSplineSurface theSecondDenominator, int theUDegree,
                                               int theVDegree, NCollection_Array1<double> theUFlatKnots,
                                               NCollection_Array1<double> theVFlatKnots,
                                               NCollection_Array2<gp_Pnt> theNumerator,
                                               NCollection_Array2<double> theDenominator)
    {
        DenominatorProductEvaluator aMultiplier = new(theFirstDenominator, theSecondDenominator);

        BSplSLib.FunctionMultiply(aMultiplier, theSurface.UDegree(), theSurface.VDegree(), theSurface.InternalUKnots(),
                                  theSurface.InternalVKnots(), theSurface.InternalUMultiplicities(),
                                  theSurface.InternalVMultiplicities(), theSurface.InternalPoles(),
                                  theSurface.InternalWeightsArray(), theUFlatKnots, theVFlatKnots, theUDegree,
                                  theVDegree, theNumerator, theDenominator, out int aStatus);
        return aStatus == 0;
    }

    private static Geom_BSplineSurface? makeProfileSkin(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                        NCollection_Array1<double> theProfileParameters,
                                                        int theVDegree, NCollection_Array1<double> theVKnots,
                                                        NCollection_Array1<int> theVMults,
                                                        NCollection_Array1<double> theVFlatKnots)
    {
        Geom_BSplineCurve aBaseProfile = theProfiles.At(0)!;
        NCollection_Array2<gp_Pnt> aPoles = new(1, aBaseProfile.NbPoles(), 1, theProfiles.Length());
        NCollection_Array2<double> aWeights = new(1, aBaseProfile.NbPoles(), 1, theProfiles.Length());
        NCollection_Array1<gp_Pnt> aColumn = new(1, theProfiles.Length());
        NCollection_Array1<double> aColumnWeights = new(1, theProfiles.Length());
        NCollection_Array1<int> aContactOrders = new(1, theProfileParameters.Size());
        aContactOrders.Init(0);

        for (int aUPoleIdx = 1; aUPoleIdx <= aBaseProfile.NbPoles(); ++aUPoleIdx)
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

        return new Geom_BSplineSurface(aPoles, aWeights, aBaseProfile.InternalKnots(), theVKnots,
                                       aBaseProfile.InternalMultiplicities(), theVMults, aBaseProfile.Degree(),
                                       theVDegree);
    }

    private static Geom_BSplineSurface? makeGuideSkin(NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                                      NCollection_Array1<double> theGuideParameters, int theUDegree,
                                                      NCollection_Array1<double> theUKnots,
                                                      NCollection_Array1<int> theUMults,
                                                      NCollection_Array1<double> theUFlatKnots)
    {
        Geom_BSplineCurve aBaseGuide = theGuides.At(0)!;
        NCollection_Array2<gp_Pnt> aPoles = new(1, theGuides.Length(), 1, aBaseGuide.NbPoles());
        NCollection_Array2<double> aWeights = new(1, theGuides.Length(), 1, aBaseGuide.NbPoles());
        NCollection_Array1<gp_Pnt> aRow = new(1, theGuides.Length());
        NCollection_Array1<double> aRowWeights = new(1, theGuides.Length());
        NCollection_Array1<int> aContactOrders = new(1, theGuideParameters.Size());
        aContactOrders.Init(0);

        for (int aVPoleIdx = 1; aVPoleIdx <= aBaseGuide.NbPoles(); ++aVPoleIdx)
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

        return new Geom_BSplineSurface(aPoles, aWeights, theUKnots, aBaseGuide.InternalKnots(), theUMults,
                                       aBaseGuide.InternalMultiplicities(), theUDegree, aBaseGuide.Degree());
    }

    private static double normalizedKnot(double theStart, double theEnd, double theKnot)
    {
        if (Math.Abs(theKnot - theStart) <= Precision.PConfusion())
        {
            return theStart;
        }
        if (Math.Abs(theKnot - theEnd) <= Precision.PConfusion())
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

        std_sort.sort(theEntries.AsSpan(), new KnotEntryLess());

        KnotEntry aCurrent = theEntries.Value(0);
        for (int anEntryIdx = 1; anEntryIdx < theEntries.Size(); ++anEntryIdx)
        {
            KnotEntry anEntry = theEntries.Value(anEntryIdx);
            if (Math.Abs(anEntry.Parameter - aCurrent.Parameter) <= Precision.PConfusion())
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

    private static bool alignSurfaces(Geom_BSplineSurface theProfileSurface, Geom_BSplineSurface theGuideSurface,
                                      Geom_BSplineSurface theReferenceSurface)
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

        theProfileSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), Precision.PConfusion(), false);
        theGuideSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), Precision.PConfusion(), false);
        theReferenceSurface.InsertUKnots(aUKnots.Array1(), aUMults.Array1(), Precision.PConfusion(), false);

        theProfileSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), Precision.PConfusion(), false);
        theGuideSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), Precision.PConfusion(), false);
        theReferenceSurface.InsertVKnots(aVKnots.Array1(), aVMults.Array1(), Precision.PConfusion(), false);
        return true;
    }

    /// <summary>Builds the final network surface by correcting the profile skin with guide deviations.</summary>
    private static Geom_BSplineSurface? makeCorrectedProfileSkin(Geom_BSplineSurface theProfileSurface,
                                                                 Geom_BSplineSurface theGuideSurface,
                                                                 Geom_BSplineSurface theReferenceSurface,
                                                                 ref ResultStatus theStatus)
    {
        if (theProfileSurface.NbUPoles() != theGuideSurface.NbUPoles()
            || theProfileSurface.NbUPoles() != theReferenceSurface.NbUPoles()
            || theProfileSurface.NbVPoles() != theGuideSurface.NbVPoles()
            || theProfileSurface.NbVPoles() != theReferenceSurface.NbVPoles())
        {
            theStatus = ResultStatus.RationalConstructionFailed;
            return null;
        }

        if (!theProfileSurface.IsURational() && !theProfileSurface.IsVRational() && !theGuideSurface.IsURational()
            && !theGuideSurface.IsVRational() && !theReferenceSurface.IsURational()
            && !theReferenceSurface.IsVRational())
        {
            Span<gp_Pnt> aProfilePoles = theProfileSurface.InternalPoles().AsSpan();
            Span<gp_Pnt> aGuidePoles = theGuideSurface.InternalPoles().AsSpan();
            Span<gp_Pnt> aReferencePoles = theReferenceSurface.InternalPoles().AsSpan();
            NCollection_Array2<gp_Pnt> aResultPoles = new(theProfileSurface.InternalPoles());
            Span<gp_Pnt> aResult = aResultPoles.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aResult.Length; ++aPoleIdx)
            {
                gp_XYZ aPoint = aProfilePoles[aPoleIdx].XYZ() + aGuidePoles[aPoleIdx].XYZ()
                                - aReferencePoles[aPoleIdx].XYZ();
                aResult[aPoleIdx] = new gp_Pnt(aPoint);
            }

            return new Geom_BSplineSurface(aResultPoles, theProfileSurface.InternalUKnots(),
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

        NCollection_Array2<gp_Pnt> aProfileNumerator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aProfileDenominator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<gp_Pnt> aGuideNumerator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aGuideDenominator = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<gp_Pnt> aReferenceNumerator = new(1, aNbUPoles, 1, aNbVPoles);
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

        NCollection_Array2<gp_Pnt> aResultPoles2 = new(1, aNbUPoles, 1, aNbVPoles);
        NCollection_Array2<double> aResultWeights = new(1, aNbUPoles, 1, aNbVPoles);
        Span<gp_Pnt> aPoles = aResultPoles2.AsSpan();
        Span<double> aWeights = aResultWeights.AsSpan();
        Span<double> aProfileDen = aProfileDenominator.AsSpan();
        Span<double> aGuideDen = aGuideDenominator.AsSpan();
        Span<double> aReferenceDen = aReferenceDenominator.AsSpan();
        Span<gp_Pnt> aProfileNum = aProfileNumerator.AsSpan();
        Span<gp_Pnt> aGuideNum = aGuideNumerator.AsSpan();
        Span<gp_Pnt> aReferenceNum = aReferenceNumerator.AsSpan();
        for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
        {
            double aWeight = aProfileDen[aPoleIdx];
            if (aWeight <= Precision.Confusion() || Math.Abs(aGuideDen[aPoleIdx] - aWeight) > Precision.Confusion()
                || Math.Abs(aReferenceDen[aPoleIdx] - aWeight) > Precision.Confusion())
            {
                theStatus = ResultStatus.RationalConstructionFailed;
                return null;
            }

            gp_XYZ aNumerator = aProfileNum[aPoleIdx].XYZ() + aGuideNum[aPoleIdx].XYZ() - aReferenceNum[aPoleIdx].XYZ();
            aWeights[aPoleIdx] = aWeight;
            aPoles[aPoleIdx] = new gp_Pnt(aNumerator / aWeight);
        }

        return new Geom_BSplineSurface(aResultPoles2, aResultWeights, aUKnots, aVKnots, aUMults, aVMults, aUDegree,
                                       aVDegree);
    }

    private static bool canSetPeriodic(Geom_BSplineSurface theSurface, bool theAlongU, double theTolerance)
    {
        int aNbChecks = theAlongU ? theSurface.NbVPoles() : theSurface.NbUPoles();
        for (int aCheckIdx = 1; aCheckIdx <= aNbChecks; ++aCheckIdx)
        {
            gp_Pnt aFirstPole = theAlongU ? theSurface.Pole(1, aCheckIdx) : theSurface.Pole(aCheckIdx, 1);
            gp_Pnt aLastPole = theAlongU ? theSurface.Pole(theSurface.NbUPoles(), aCheckIdx)
                                         : theSurface.Pole(aCheckIdx, theSurface.NbVPoles());
            if (aFirstPole.Distance(aLastPole) > theTolerance)
            {
                return false;
            }
        }
        return true;
    }

    private static bool isReadyToBuild(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                       NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                       NCollection_Array1<double> theProfileParameters,
                                       NCollection_Array1<double> theGuideParameters,
                                       NCollection_Array2<gp_Pnt> theIntersectionPoints,
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
    private static Geom_BSplineCurve? makeProfileSeamGuide(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                           NCollection_Array1<double> theProfileParameters)
    {
        NCollection_HArray1<gp_Pnt> aPoints = new(1, theProfiles.Length());
        NCollection_HArray1<double> aParameters = new(1, theProfileParameters.Length());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            Geom_BSplineCurve aProfile = theProfiles.At(aProfileIdx)!;
            aPoints.SetValue(aProfileIdx + 1, aProfile.Value(aProfile.FirstParameter()));
            aParameters.SetValue(aProfileIdx + 1, theProfileParameters.At(aProfileIdx));
        }

        GeomAPI_Interpolate anInterpolator = new(aPoints, aParameters, false, Precision.Confusion());
        anInterpolator.Perform();
        return anInterpolator.IsDone() ? anInterpolator.Curve() : null;
    }

    // Completes a closed U-direction interpolation basis at both profile endpoints.
    private static bool completeClosedGuideSeam(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                NCollection_Array1<double> theProfileParameters,
                                                NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                                NCollection_Array1<double> theGuideParameters,
                                                NCollection_Array2<gp_Pnt> theIntersectionPoints,
                                                NCollection_Array2<double> theIntersectionWeights,
                                                ref NCollection_Array1<Geom_BSplineCurve?> theClosedGuides,
                                                ref NCollection_Array1<double> theClosedGuideParameters,
                                                ref NCollection_Array2<gp_Pnt> theClosedIntersectionPoints,
                                                ref NCollection_Array2<double> theClosedIntersectionWeights)
    {
        double aProfileStart = theProfiles.At(0)!.FirstParameter();
        double aProfileEnd = theProfiles.At(0)!.LastParameter();
        bool needsStartSeam = theGuideParameters.First() > aProfileStart + Precision.PConfusion();
        bool needsEndSeam = theGuideParameters.Last() < aProfileEnd - Precision.PConfusion();
        if (!needsStartSeam && !needsEndSeam)
        {
            return true;
        }

        Geom_BSplineCurve? aSeamGuide = null;
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
        theClosedGuides = new NCollection_Array1<Geom_BSplineCurve?>(1, aNbGuides);
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

        theClosedIntersectionPoints = new NCollection_Array2<gp_Pnt>(1, aNbGuides, 1, theIntersectionPoints.NbColumns());
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

    private static Geom_BSplineSurface? makeNetworkSurface(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                           NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                                           NCollection_Array1<double> theProfileParameters,
                                                           NCollection_Array1<double> theGuideParameters,
                                                           NCollection_Array2<gp_Pnt> theIntersectionPoints,
                                                           NCollection_Array2<double> theIntersectionWeights,
                                                           bool theIsUClosed, ref ResultStatus theStatus)
    {
        theStatus = ResultStatus.ConstructionFailed;

        NCollection_Array1<Geom_BSplineCurve?> aClosedGuides = new();
        NCollection_Array1<double> aClosedGuideParameters = new();
        NCollection_Array2<gp_Pnt> aClosedIntersectionPoints = new();
        NCollection_Array2<double> aClosedIntersectionWeights = new();
        bool shouldCompleteUSeam =
          theIsUClosed
          && (theGuideParameters.First() > theProfiles.At(0)!.FirstParameter() + Precision.PConfusion()
              || theGuideParameters.Last() < theProfiles.At(0)!.LastParameter() - Precision.PConfusion());
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
        NCollection_Array1<Geom_BSplineCurve?> aGuides = shouldCompleteUSeam ? aClosedGuides : theGuides;
        NCollection_Array1<double> aGuideParameters = shouldCompleteUSeam ? aClosedGuideParameters : theGuideParameters;
        NCollection_Array2<gp_Pnt> aIntersectionPoints =
          shouldCompleteUSeam ? aClosedIntersectionPoints : theIntersectionPoints;
        NCollection_Array2<double> aIntersectionWeights =
          shouldCompleteUSeam ? aClosedIntersectionWeights : theIntersectionWeights;

        SkinningBasis anUBasis = new();
        anUBasis.Init(aGuideParameters);

        SkinningBasis aVBasis = new();
        aVBasis.Init(theProfileParameters);

        Geom_BSplineSurface? aProfileSurface = makeProfileSkin(theProfiles, theProfileParameters, aVBasis.Degree,
                                                               aVBasis.Knots, aVBasis.Mults, aVBasis.FlatKnots);
        if (aProfileSurface is null)
        {
            theStatus = ResultStatus.SkinningFailed;
            return null;
        }

        Geom_BSplineSurface? aGuideSurface = makeGuideSkin(aGuides, aGuideParameters, anUBasis.Degree, anUBasis.Knots,
                                                           anUBasis.Mults, anUBasis.FlatKnots);
        if (aGuideSurface is null)
        {
            theStatus = ResultStatus.SkinningFailed;
            return null;
        }

        // The reference surface starts from the same contact grid validated during preparation.
        NCollection_Array2<gp_Pnt> aReferencePoles = new(aIntersectionPoints);
        NCollection_Array2<double> aReferenceWeights = new(aIntersectionWeights);
        bool isReferencePolynomial = hasRationalCurve(theProfiles) != hasRationalCurve(aGuides);
        if (isReferencePolynomial)
        {
            aReferenceWeights.Init(1.0);
        }
        else
        {
            Span<gp_Pnt> aPoles = aReferencePoles.AsSpan();
            Span<double> aWeights = aReferenceWeights.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
            {
                double aWeight = aWeights[aPoleIdx];
                aPoles[aPoleIdx] = new gp_Pnt(aPoles[aPoleIdx].XYZ() * aWeight);
            }
        }

        BSplSLib.Interpolate(anUBasis.Degree, aVBasis.Degree, anUBasis.FlatKnots, aVBasis.FlatKnots, aGuideParameters,
                             theProfileParameters, aReferencePoles, aReferenceWeights, out int anInversionProblem);
        if (anInversionProblem != 0)
        {
            theStatus = ResultStatus.ReferenceSurfaceFailed;
            return null;
        }
        if (!isReferencePolynomial)
        {
            Span<gp_Pnt> aPoles = aReferencePoles.AsSpan();
            Span<double> aWeights = aReferenceWeights.AsSpan();
            for (int aPoleIdx = 0; aPoleIdx < aPoles.Length; ++aPoleIdx)
            {
                double aWeight = aWeights[aPoleIdx];
                if (aWeight <= gp.Resolution())
                {
                    theStatus = ResultStatus.ReferenceSurfaceFailed;
                    return null;
                }
                aPoles[aPoleIdx] = new gp_Pnt(aPoles[aPoleIdx].XYZ() / aWeight);
            }
        }

        Geom_BSplineSurface aReferenceSurface = new(aReferencePoles, aReferenceWeights, anUBasis.Knots, aVBasis.Knots,
                                                    anUBasis.Mults, aVBasis.Mults, anUBasis.Degree, aVBasis.Degree);
        if (!alignSurfaces(aProfileSurface, aGuideSurface, aReferenceSurface))
        {
            theStatus = ResultStatus.KnotAlignmentFailed;
            return null;
        }
        return makeCorrectedProfileSkin(aProfileSurface, aGuideSurface, aReferenceSurface, ref theStatus);
    }

    private static bool applyPeriodicity(Geom_BSplineSurface theSurface, bool theIsUClosed, bool theIsVClosed,
                                         double theTolerance)
    {
        double aTolerance = Max(theTolerance, Precision.Confusion());
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
    public void Init(NCollection_Array1<Geom_BSplineCurve?> theProfiles, NCollection_Array1<Geom_BSplineCurve?> theGuides,
                     NCollection_Array1<double> theProfileParameters, NCollection_Array1<double> theGuideParameters,
                     NCollection_Array2<gp_Pnt> theIntersectionPoints, NCollection_Array2<double> theIntersectionWeights,
                     double theTolerance, bool theIsUClosed, bool theIsVClosed)
    {
        myProfiles = new NCollection_Array1<Geom_BSplineCurve?>(theProfiles);
        myGuides = new NCollection_Array1<Geom_BSplineCurve?>(theGuides);
        myProfileParameters = new NCollection_Array1<double>(theProfileParameters);
        myGuideParameters = new NCollection_Array1<double>(theGuideParameters);
        myIntersectionPoints = new NCollection_Array2<gp_Pnt>(theIntersectionPoints);
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
        catch (Standard_Failure)
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
    public Geom_BSplineSurface Surface()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone("GeomFill_NetworkSurface::Surface");
        }
        mySurface!.Freeze();
        return mySurface;
    }
}
