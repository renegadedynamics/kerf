// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GordonUtilities.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Geometry.Splines;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// GeomFill_GordonUtilities.pxx (OCCT's namespace <c>GordonUtilities</c>): the preparation of a Gordon curve network
/// for <see cref="GeomFill_Gordon"/>: conversion to unit-domain non-periodic B-splines, profile/guide contacts, network
/// ordering and orientation, closed-seam detection and the exact (or sampled) reparametrisation of every curve to
/// shared intersection parameters.
/// </summary>
/// <remarks>
/// OCCT's anonymous-namespace functions are internal static methods. Its views without copying are kept where they
/// are written through (<see cref="toMatrix"/> shares the parameter table, as OCCT's math_Matrix over
/// <c>&amp;ChangeFirst()</c>); OCCT's read-only Array1 views over a linear vector are copies (the same values). Every
/// <c>std::sort</c> goes through <see cref="Sorting"/>, so that ties keep MSVC's order (decision 491).
/// </remarks>
internal static class GordonUtilities
{
    /// <summary>Result of resolving one profile/guide contact.</summary>
    internal struct Contact
    {
        public bool IsValid = false;
        public double ProfileParam = 0.0;
        public double GuideParam = 0.0;
        public double Gap = RealLast();

        public Contact()
        {
        }

        public Contact(bool theIsValid, double theProfileParam, double theGuideParam, double theGap)
        {
            IsValid = theIsValid;
            ProfileParam = theProfileParam;
            GuideParam = theGuideParam;
            Gap = theGap;
        }
    }

    /// <summary>Stores all admissible contacts for one profile/guide pair.</summary>
    internal sealed class ContactSet
    {
        public NCollection_LinearVector<Contact> Candidates = new();
    }

    private readonly struct DoubleLess : Heap.ILess<double>
    {
        public bool Less(double theLeft, double theRight) => theLeft < theRight;
    }

    private readonly struct ContactLess : Heap.ILess<Contact>
    {
        public bool Less(Contact theLeft, Contact theRight)
        {
            if (theLeft.Gap != theRight.Gap)
            {
                return theLeft.Gap < theRight.Gap;
            }
            if (theLeft.ProfileParam != theRight.ProfileParam)
            {
                return theLeft.ProfileParam < theRight.ProfileParam;
            }
            return theLeft.GuideParam < theRight.GuideParam;
        }
    }

    private readonly struct OrderedCurveLess : Heap.ILess<OrderedCurve>
    {
        public bool Less(OrderedCurve theLeft, OrderedCurve theRight) => theLeft.Rank < theRight.Rank;
    }

    // std::clamp
    private static double clamp(double theValue, double theLow, double theHigh)
        => theValue < theLow ? theLow : (theHigh < theValue ? theHigh : theValue);

    /// <summary>Returns a stable cubic degree when the OCCT degree limit allows it.</summary>
    internal static int interpolationDegree()
    {
        return Min(3, BSplineCurve3D.MaxDegree());
    }

    /// <summary>Returns a parameter comparison tolerance scaled by the interval length.</summary>
    internal static double parameterResolution(double theFirst, double theLast)
    {
        return global::Kerf.Engine.Foundation.Precision.PIntersection(Max(Math.Abs(theLast - theFirst), 1.0));
    }

    /// <summary>Returns a network tolerance converted to the given parameter range.</summary>
    internal static double boundaryParameterResolution(double theFirst, double theLast, double theTolerance)
    {
        double aRange = Max(Math.Abs(theLast - theFirst), 1.0);
        return global::Kerf.Engine.Foundation.Precision.Parametric(Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Approximation())) * aRange;
    }

    /// <summary>Appends a clamped parameter candidate.</summary>
    internal static void appendUnitParameter(NCollection_LinearVector<double> theParameters, double theParameter)
    {
        theParameters.Append(Max(0.0, Min(1.0, theParameter)));
    }

    /// <summary>Sorts and removes duplicate parameters in-place.</summary>
    internal static void compactParameters(NCollection_LinearVector<double> theParameters)
    {
        if (theParameters.IsEmpty())
        {
            return;
        }

        Sorting.Sort(theParameters.AsSpan(), new DoubleLess());
        double aTol = parameterResolution(0.0, 1.0);
        int aWriteIdx = 0;
        for (int aReadIdx = 1; aReadIdx < theParameters.Size(); ++aReadIdx)
        {
            if (Math.Abs(theParameters.Value(aReadIdx) - theParameters.Value(aWriteIdx)) > aTol)
            {
                ++aWriteIdx;
                theParameters.ChangeValue(aWriteIdx) = theParameters.Value(aReadIdx);
            }
        }

        while (theParameters.Size() > aWriteIdx + 1)
        {
            theParameters.EraseLast();
        }
    }

    /// <summary>Adds midpoint samples in the widest intervals until the requested count is reached.</summary>
    internal static void addIntervalMidpoints(NCollection_LinearVector<double> theParameters, int theMinSize)
    {
        compactParameters(theParameters);
        while (theParameters.Size() < theMinSize && theParameters.Size() > 1)
        {
            int aSplitIdx = 0;
            double aMaxGap = -1.0;
            for (int aParamIdx = 1; aParamIdx < theParameters.Size(); ++aParamIdx)
            {
                double aGap = theParameters.Value(aParamIdx) - theParameters.Value(aParamIdx - 1);
                if (aGap > aMaxGap)
                {
                    aSplitIdx = aParamIdx;
                    aMaxGap = aGap;
                }
            }
            if (aMaxGap <= parameterResolution(0.0, 1.0))
            {
                return;
            }

            appendUnitParameter(theParameters,
                                0.5 * (theParameters.Value(aSplitIdx - 1) + theParameters.Value(aSplitIdx)));
            compactParameters(theParameters);
        }
    }

    /// <summary>
    /// A 1-based Array1 of a linear vector's values. OCCT wraps the vector's storage without copying; every use is
    /// read-only, so a copy gives the same values.
    /// </summary>
    internal static NCollection_Array1<T> toOneBasedArrayView<T>(NCollection_LinearVector<T> theValues)
    {
        return NCollection_Array1<T>.From(theValues.AsSpan(), 1);
    }

    /// <summary>Wraps a parameter table into a math matrix without copying its storage: writes go to the table.</summary>
    internal static Matrix toMatrix(NCollection_Array2<double> theParams)
    {
        (double[] aData, int anOffset) = theParams.Storage();
        NCollection_Array1<double> aView = new(aData, anOffset, 1, theParams.Size());
        return new Matrix(aView, 1, 1, theParams.ColLength(), 1, theParams.RowLength());
    }

    /// <summary>Expands a periodic B-spline period into explicit non-periodic storage.</summary>
    internal static bool exposePeriod(BSplineCurve3D theCurve)
    {
        if (!theCurve.IsPeriodic)
        {
            return true;
        }

        try
        {
            double aFirst = theCurve.FirstParameter();
            double aLast = theCurve.LastParameter();
            if (aLast <= aFirst + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }

            theCurve.Segment(aFirst, aLast, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (theCurve.IsPeriodic)
            {
                theCurve.SetNotPeriodic();
            }
        }
        catch (FailureException)
        {
            return false;
        }
        return !theCurve.IsPeriodic;
    }

    /// <summary>Normalizes an input curve family to explicit non-periodic B-splines on [0, 1].</summary>
    internal static bool makeUnitCurves(NCollection_Array1<BSplineCurve3D?> theCurves)
    {
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            BSplineCurve3D? aCurve = theCurves.At(aCurveIdx);
            if (aCurve is null || !exposePeriod(aCurve))
            {
                return false;
            }

            NCollection_Array1<double> aKnots = new(aCurve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, aKnots);
            aCurve.SetKnots(aKnots);
        }
        return true;
    }

    /// <summary>Index and rank pair for ordering curves.</summary>
    internal struct OrderedCurve
    {
        public int Index;
        public double Rank;

        public OrderedCurve(int theIndex, double theRank)
        {
            Index = theIndex;
            Rank = theRank;
        }
    }

    /// <summary>Returns average values of matrix columns.</summary>
    internal static math_Vector columnMeans(Matrix theParams)
    {
        math_Vector aOnes = new(theParams.LowerRow(), theParams.UpperRow(), 1.0);
        math_Vector aSums = theParams.Transposed().Multiplied(aOnes);
        aSums /= (double)theParams.RowNumber();
        return aSums;
    }

    /// <summary>Returns average values of matrix rows.</summary>
    internal static math_Vector rowMeans(Matrix theParams)
    {
        math_Vector aOnes = new(theParams.LowerCol(), theParams.UpperCol(), 1.0);
        math_Vector aSums = theParams.Multiplied(aOnes);
        aSums /= (double)theParams.ColNumber();
        return aSums;
    }

    /// <summary>Appends a unique contact candidate if the corresponding curve points match.</summary>
    internal static void appendContact(BSplineCurve3D theProfile, BSplineCurve3D theGuide, double theProfileParam,
                                       double theGuideParam, double theTolerance, ContactSet theContacts)
    {
        CurveAdaptor aProfileAdaptor = new(theProfile);
        CurveAdaptor aGuideAdaptor = new(theGuide);
        Point3D aProfilePoint = aProfileAdaptor.EvalD0(theProfileParam);
        Point3D aGuidePoint = aGuideAdaptor.EvalD0(theGuideParam);
        double aGap = aProfilePoint.Distance(aGuidePoint);
        if (aGap > theTolerance)
        {
            return;
        }

        double aProfileTol = parameterResolution(theProfile.FirstParameter(), theProfile.LastParameter());
        double aGuideTol = parameterResolution(theGuide.FirstParameter(), theGuide.LastParameter());
        for (int aContactIdx = 0; aContactIdx < theContacts.Candidates.Size(); ++aContactIdx)
        {
            Contact aContact = theContacts.Candidates.Value(aContactIdx);
            if (Math.Abs(aContact.ProfileParam - theProfileParam) <= aProfileTol
                && Math.Abs(aContact.GuideParam - theGuideParam) <= aGuideTol)
            {
                return;
            }
        }

        theContacts.Candidates.Append(new Contact(true, theProfileParam, theGuideParam, aGap));
    }

    /// <summary>Converts two close extrema points to an exact curve-parameter contact.</summary>
    internal static void appendExtremaContact(BSplineCurve3D theProfile, BSplineCurve3D theGuide,
                                              in Point3D theProfilePoint, in Point3D theGuidePoint, double theTolerance,
                                              ContactSet theContacts)
    {
        if (theProfilePoint.Distance(theGuidePoint) > theTolerance)
        {
            return;
        }

        Point3D aSharedPoint = new((theProfilePoint.XYZ() + theGuidePoint.XYZ()) * 0.5);

        double aProfileParam;
        double aGuideParam = 0.0;
        if (GeometryLibraryTool.Parameter(theProfile, aSharedPoint, theTolerance, out aProfileParam)
            && GeometryLibraryTool.Parameter(theGuide, aSharedPoint, theTolerance, out aGuideParam))
        {
            appendContact(theProfile, theGuide, aProfileParam, aGuideParam, theTolerance, theContacts);
        }
    }

    /// <summary>Adds endpoint contacts; these are important for opening closed seams.</summary>
    internal static void appendBoundaryContacts(BSplineCurve3D theProfile, BSplineCurve3D theGuide,
                                                double theTolerance, ContactSet theContacts)
    {
        ReadOnlySpan<double> aProfileBounds = [theProfile.FirstParameter(), theProfile.LastParameter()];
        ReadOnlySpan<double> aGuideBounds = [theGuide.FirstParameter(), theGuide.LastParameter()];
        CurveAdaptor aProfileAdaptor = new(theProfile);
        CurveAdaptor aGuideAdaptor = new(theGuide);
        foreach (double aProfileParam in aProfileBounds)
        {
            if (GeometryLibraryTool.Parameter(theGuide, aProfileAdaptor.EvalD0(aProfileParam), theTolerance,
                                       out double aGuideParam))
            {
                appendContact(theProfile, theGuide, aProfileParam, aGuideParam, theTolerance, theContacts);
            }
        }
        foreach (double aGuideParam in aGuideBounds)
        {
            if (GeometryLibraryTool.Parameter(theProfile, aGuideAdaptor.EvalD0(aGuideParam), theTolerance,
                                       out double aProfileParam))
            {
                appendContact(theProfile, theGuide, aProfileParam, aGuideParam, theTolerance, theContacts);
            }
        }
    }

    internal static bool appendLinearSegmentContact(BSplineCurve3D theProfile, BSplineCurve3D theGuide,
                                                    double theTolerance, ContactSet theContacts)
    {
        if (theProfile.Degree() != 1 || theGuide.Degree() != 1 || theProfile.PoleCount != 2
            || theGuide.PoleCount != 2)
        {
            return false;
        }

        double aProfileFirst = theProfile.FirstParameter();
        double aProfileLast = theProfile.LastParameter();
        double aGuideFirst = theGuide.FirstParameter();
        double aGuideLast = theGuide.LastParameter();
        if (aProfileLast <= aProfileFirst + global::Kerf.Engine.Foundation.Precision.PConfusion()
            || aGuideLast <= aGuideFirst + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return true;
        }

        CurveAdaptor aProfileAdaptor = new(theProfile);
        CurveAdaptor aGuideAdaptor = new(theGuide);
        Point3D aP0 = aProfileAdaptor.EvalD0(aProfileFirst);
        Point3D aP1 = aProfileAdaptor.EvalD0(aProfileLast);
        Point3D aQ0 = aGuideAdaptor.EvalD0(aGuideFirst);
        Point3D aQ1 = aGuideAdaptor.EvalD0(aGuideLast);

        Xyz aU = aP1.XYZ() - aP0.XYZ();
        Xyz aV = aQ1.XYZ() - aQ0.XYZ();
        Xyz aW = aP0.XYZ() - aQ0.XYZ();
        double aA = aU.SquareModulus();
        double aB = aU.Dot(aV);
        double aC = aV.SquareModulus();
        double aD = aU.Dot(aW);
        double aE = aV.Dot(aW);
        if (aA <= Constants.Resolution() || aC <= Constants.Resolution())
        {
            return true;
        }

        double aDenom = aA * aC - aB * aB;
        if (Math.Abs(aDenom) <= Constants.Resolution())
        {
            return false;
        }

        double aS;
        double aT;
        aS = clamp((aB * aE - aC * aD) / aDenom, 0.0, 1.0);
        aT = (aB * aS + aE) / aC;
        if (aT < 0.0)
        {
            aT = 0.0;
            aS = clamp(-aD / aA, 0.0, 1.0);
        }
        else if (aT > 1.0)
        {
            aT = 1.0;
            aS = clamp((aB - aD) / aA, 0.0, 1.0);
        }

        appendContact(theProfile, theGuide, aProfileFirst + aS * (aProfileLast - aProfileFirst),
                      aGuideFirst + aT * (aGuideLast - aGuideFirst), theTolerance, theContacts);
        return true;
    }

    /// <summary>Returns all close contacts detected for one profile/guide pair.</summary>
    internal static ContactSet collectContacts(BSplineCurve3D theProfile, BSplineCurve3D theGuide,
                                               double theTolerance)
    {
        ContactSet aContacts = new();
        if (appendLinearSegmentContact(theProfile, theGuide, theTolerance, aContacts))
        {
            return aContacts;
        }

        GeomAPI_ExtremaCurveCurve anExtrema = new(theProfile, theGuide);
        for (int anExtIdx = 1; anExtIdx <= anExtrema.NbExtrema(); ++anExtIdx)
        {
            anExtrema.Points(anExtIdx, out Point3D aProfilePoint, out Point3D aGuidePoint);
            appendExtremaContact(theProfile, theGuide, aProfilePoint, aGuidePoint, theTolerance, aContacts);
        }

        {
            Point3D aProfilePoint = new();
            Point3D aGuidePoint = new();
            if (anExtrema.TotalNearestPoints(ref aProfilePoint, ref aGuidePoint))
            {
                appendExtremaContact(theProfile, theGuide, aProfilePoint, aGuidePoint, theTolerance, aContacts);
            }
        }

        appendBoundaryContacts(theProfile, theGuide, theTolerance, aContacts);
        Sorting.Sort(aContacts.Candidates.AsSpan(), new ContactLess());
        return aContacts;
    }

    /// <summary>Returns the flat index of a profile/guide pair.</summary>
    internal static int contactIndex(int theProfileIdx, int theGuideIdx, int theNbGuides)
    {
        return theProfileIdx * theNbGuides + theGuideIdx;
    }

    /// <summary>Stores the lowest-gap contact for the initial ordering estimate.</summary>
    internal static bool chooseNearestContacts(NCollection_LinearVector<ContactSet> theContacts, int theNbProfiles,
                                               int theNbGuides, NCollection_Array2<double> theProfileParams,
                                               NCollection_Array2<double> theGuideParams)
    {
        for (int aProfileIdx = 0; aProfileIdx < theNbProfiles; ++aProfileIdx)
        {
            for (int aGuideIdx = 0; aGuideIdx < theNbGuides; ++aGuideIdx)
            {
                ContactSet aSet = theContacts.Value(contactIndex(aProfileIdx, aGuideIdx, theNbGuides));
                if (aSet.Candidates.IsEmpty())
                {
                    return false;
                }
                Contact aContact = aSet.Candidates.First();
                theProfileParams.ChangeAt(aProfileIdx, aGuideIdx) = aContact.ProfileParam;
                theGuideParams.ChangeAt(aProfileIdx, aGuideIdx) = aContact.GuideParam;
            }
        }
        return true;
    }

    /// <summary>Returns the largest best-contact gap in a complete contact table.</summary>
    internal static double maxNearestContactGap(NCollection_LinearVector<ContactSet> theContacts)
    {
        double aMaxGap = 0.0;
        for (int aSetIdx = 0; aSetIdx < theContacts.Size(); ++aSetIdx)
        {
            ContactSet aSet = theContacts.Value(aSetIdx);
            if (!aSet.Candidates.IsEmpty())
            {
                aMaxGap = Max(aMaxGap, aSet.Candidates.First().Gap);
            }
        }
        return aMaxGap;
    }

    /// <summary>
    /// Builds all profile/guide contacts. The pairs are independent, each writes only its own set, so a parallel run
    /// gives the serial result (decision 490).
    /// </summary>
    internal static bool fillContactTable(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                          NCollection_Array1<BSplineCurve3D?> theGuides,
                                          NCollection_Array2<double> theProfileParams,
                                          NCollection_Array2<double> theGuideParams,
                                          NCollection_LinearVector<ContactSet> theContactSets, double theTolerance,
                                          bool theToUseParallel)
    {
        int aNbProfiles = theProfiles.Length();
        int aNbGuides = theGuides.Length();
        int aNbPairs = aNbProfiles * aNbGuides;

        theContactSets.Clear();
        for (int aPairIdx = 0; aPairIdx < aNbPairs; ++aPairIdx)
        {
            theContactSets.Append(new ContactSet());
        }

        int isDone = 1; // std::atomic<bool>
        OsdParallel.For(0, aNbPairs, thePairIdx =>
        {
            if (Volatile.Read(ref isDone) == 0)
            {
                return;
            }

            int aProfileIdx = thePairIdx / aNbGuides;
            int aGuideIdx = thePairIdx % aNbGuides;
            ref ContactSet aContactSet = ref theContactSets.ChangeValue(contactIndex(aProfileIdx, aGuideIdx, aNbGuides));
            aContactSet = collectContacts(theProfiles.At(aProfileIdx)!, theGuides.At(aGuideIdx)!, theTolerance);
            if (aContactSet.Candidates.IsEmpty())
            {
                Volatile.Write(ref isDone, 0);
            }
        }, !theToUseParallel);

        return Volatile.Read(ref isDone) != 0
               && chooseNearestContacts(theContactSets, aNbProfiles, aNbGuides, theProfileParams, theGuideParams);
    }

    /// <summary>Snaps one parameter value to the curve boundary when it is close enough.</summary>
    internal static double snappedParameter(BSplineCurve3D theCurve, double theParameter, double theTolerance)
    {
        double aFirst = theCurve.FirstParameter();
        double aLast = theCurve.LastParameter();
        double aTol = boundaryParameterResolution(aFirst, aLast, theTolerance);
        if (Math.Abs(theParameter - aFirst) < aTol)
        {
            return aFirst;
        }
        if (Math.Abs(theParameter - aLast) < aTol)
        {
            return aLast;
        }
        return theParameter;
    }

    /// <summary>Normalizes intersections after ordering and returns detected closed directions.</summary>
    internal static bool normalizeContactTable(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                               NCollection_Array1<BSplineCurve3D?> theGuides,
                                               NCollection_Array2<double> theProfileParams,
                                               NCollection_Array2<double> theGuideParams, double theTolerance,
                                               ref bool theIsUClosed, ref bool theIsVClosed)
    {
        double aTol = Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Boundary snaps are done before validation so closed seams use exact end parameters.
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
            {
                theProfileParams.ChangeAt(aProfileIdx, aGuideIdx) =
                  snappedParameter(theProfiles.At(aProfileIdx)!, theProfileParams.At(aProfileIdx, aGuideIdx),
                                   theTolerance);
                theGuideParams.ChangeAt(aProfileIdx, aGuideIdx) =
                  snappedParameter(theGuides.At(aGuideIdx)!, theGuideParams.At(aProfileIdx, aGuideIdx), theTolerance);
            }
        }

        double aUGap = 0.0;
        double aVGap = 0.0;
        int aLastProfile = theProfiles.Size() - 1;
        int aLastGuide = theGuides.Size() - 1;

        // One pass validates all contacts and measures only the two possible seam gaps.
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            CurveAdaptor aProfileAdaptor = new(theProfiles.At(aProfileIdx)!);
            for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
            {
                CurveAdaptor aGuideAdaptor = new(theGuides.At(aGuideIdx)!);
                Point3D aProfilePoint = aProfileAdaptor.EvalD0(theProfileParams.At(aProfileIdx, aGuideIdx));
                Point3D aGuidePoint = aGuideAdaptor.EvalD0(theGuideParams.At(aProfileIdx, aGuideIdx));
                if (aProfilePoint.Distance(aGuidePoint) > aTol)
                {
                    return false;
                }
                if (aGuideIdx == 0 || aGuideIdx == aLastGuide)
                {
                    Point3D aOppositePoint =
                      aProfileAdaptor.EvalD0(theProfileParams.At(aProfileIdx, aLastGuide - aGuideIdx));
                    aUGap = Max(aUGap, aProfilePoint.Distance(aOppositePoint));
                }
                if (aProfileIdx == 0 || aProfileIdx == aLastProfile)
                {
                    Point3D aOppositePoint =
                      aGuideAdaptor.EvalD0(theGuideParams.At(aLastProfile - aProfileIdx, aGuideIdx));
                    aVGap = Max(aVGap, aGuidePoint.Distance(aOppositePoint));
                }
            }
        }

        theIsUClosed = aUGap <= aTol;
        theIsVClosed = aVGap <= aTol;
        if (theIsUClosed)
        {
            for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
            {
                theProfileParams.ChangeAt(aProfileIdx, 0) = theProfiles.At(aProfileIdx)!.FirstParameter();
                theProfileParams.ChangeAt(aProfileIdx, aLastGuide) = theProfiles.At(aProfileIdx)!.LastParameter();
            }
        }
        if (theIsVClosed)
        {
            for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
            {
                theGuideParams.ChangeAt(0, aGuideIdx) = theGuides.At(aGuideIdx)!.FirstParameter();
                theGuideParams.ChangeAt(aLastProfile, aGuideIdx) = theGuides.At(aGuideIdx)!.LastParameter();
            }
        }
        return true;
    }

    /// <summary>Orders curves by their mean position in the opposite parameter family.</summary>
    internal static NCollection_LinearVector<OrderedCurve> orderedCurves(math_Vector theRanks)
    {
        NCollection_LinearVector<OrderedCurve> anOrder = new();
        for (int aRankIdx = theRanks.Lower(); aRankIdx <= theRanks.Upper(); ++aRankIdx)
        {
            anOrder.Append(new OrderedCurve(aRankIdx - theRanks.Lower(), theRanks[aRankIdx]));
        }

        Sorting.Sort(anOrder.AsSpan(), new OrderedCurveLess());
        double aTol = parameterResolution(0.0, 1.0);
        if (anOrder.Size() > 2 && Math.Abs(anOrder.Value(0).Rank - anOrder.Value(1).Rank) <= aTol)
        {
            NCollection_LinearVector<OrderedCurve> anOpenedOrder = new();
            anOpenedOrder.Append(anOrder.Value(0));
            for (int anOrderIdx = 2; anOrderIdx < anOrder.Size(); ++anOrderIdx)
            {
                anOpenedOrder.Append(anOrder.Value(anOrderIdx));
            }
            anOpenedOrder.Append(anOrder.Value(1));
            return anOpenedOrder;
        }
        if (anOrder.Size() > 2 && Math.Abs(anOrder.Last().Rank - anOrder.Value(anOrder.Size() - 2).Rank) <= aTol)
        {
            NCollection_LinearVector<OrderedCurve> anOpenedOrder = new();
            anOpenedOrder.Append(anOrder.Value(anOrder.Size() - 2));
            for (int anOrderIdx = 0; anOrderIdx + 2 < anOrder.Size(); ++anOrderIdx)
            {
                anOpenedOrder.Append(anOrder.Value(anOrderIdx));
            }
            anOpenedOrder.Append(anOrder.Last());
            return anOpenedOrder;
        }
        return anOrder;
    }

    /// <summary>Checks that every row and column advances in its own curve parameter direction.</summary>
    internal static bool isMonotoneLayout(Matrix theProfileParams, Matrix theGuideParams)
    {
        double aTol = parameterResolution(0.0, 1.0);

        // Ambiguous or folded grids are rejected instead of being repaired heuristically.
        for (int aRow = theProfileParams.LowerRow(); aRow <= theProfileParams.UpperRow(); ++aRow)
        {
            for (int aCol = theProfileParams.LowerCol() + 1; aCol <= theProfileParams.UpperCol(); ++aCol)
            {
                if (theProfileParams[aRow, aCol] <= theProfileParams[aRow, aCol - 1] + aTol)
                {
                    return false;
                }
            }
        }

        for (int aCol = theGuideParams.LowerCol(); aCol <= theGuideParams.UpperCol(); ++aCol)
        {
            for (int aRow = theGuideParams.LowerRow() + 1; aRow <= theGuideParams.UpperRow(); ++aRow)
            {
                if (theGuideParams[aRow, aCol] <= theGuideParams[aRow - 1, aCol] + aTol)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>Opens duplicated seam contacts to exact boundary parameters before monotone validation.</summary>
    internal static void openClosedBoundaryCandidates(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                      NCollection_Array1<BSplineCurve3D?> theGuides,
                                                      NCollection_Array2<double> theProfileParams,
                                                      NCollection_Array2<double> theGuideParams, double theTolerance)
    {
        double aTol = Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        int aLastGuide = theGuides.Size() - 1;
        int aLastProf = theProfiles.Size() - 1;
        bool isURepeated = true;
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            CurveAdaptor aProfileAdaptor = new(theProfiles.At(aProfileIdx)!);
            Point3D aFirst = aProfileAdaptor.EvalD0(theProfileParams.At(aProfileIdx, 0));
            Point3D aLast = aProfileAdaptor.EvalD0(theProfileParams.At(aProfileIdx, aLastGuide));
            if (aFirst.Distance(aLast) > aTol)
            {
                isURepeated = false;
                break;
            }
        }
        if (isURepeated)
        {
            for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
            {
                theProfileParams.ChangeAt(aProfileIdx, 0) = theProfiles.At(aProfileIdx)!.FirstParameter();
                theProfileParams.ChangeAt(aProfileIdx, aLastGuide) = theProfiles.At(aProfileIdx)!.LastParameter();
            }
        }

        bool isVRepeated = true;
        for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
        {
            CurveAdaptor aGuideAdaptor = new(theGuides.At(aGuideIdx)!);
            Point3D aFirst = aGuideAdaptor.EvalD0(theGuideParams.At(0, aGuideIdx));
            Point3D aLast = aGuideAdaptor.EvalD0(theGuideParams.At(aLastProf, aGuideIdx));
            if (aFirst.Distance(aLast) > aTol)
            {
                isVRepeated = false;
                break;
            }
        }
        if (isVRepeated)
        {
            for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
            {
                theGuideParams.ChangeAt(0, aGuideIdx) = theGuides.At(aGuideIdx)!.FirstParameter();
                theGuideParams.ChangeAt(aLastProf, aGuideIdx) = theGuides.At(aGuideIdx)!.LastParameter();
            }
        }
    }

    /// <summary>Returns a curve handle oriented as requested (always a copy).</summary>
    internal static BSplineCurve3D orientedCurve(BSplineCurve3D theCurve, bool theIsForward)
    {
        BSplineCurve3D aCurve = (BSplineCurve3D)theCurve.Copy();
        if (!theIsForward)
        {
            aCurve.Reverse();
        }
        return aCurve;
    }

    /// <summary>Returns a parameter in the destination curve orientation.</summary>
    internal static double orientedParameter(BSplineCurve3D theCurve, double theParameter, bool theIsForward)
    {
        return theIsForward ? theParameter : theCurve.FirstParameter() + theCurve.LastParameter() - theParameter;
    }

    /// <summary>Scores an oriented contact against the expected grid position.</summary>
    internal static double orderedContactScore(in Contact theContact, double theProfileParam, double theGuideParam,
                                               double theProfileTarget, double theGuideTarget,
                                               double thePreviousProfileParam, double thePreviousGuideParam,
                                               double thePreviousProfileTarget, double thePreviousGuideTarget,
                                               bool theHasPreviousProfile, bool theHasPreviousGuide,
                                               double theTolerance)
    {
        double aScore = theContact.Gap / Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Confusion())
                        + Math.Abs(theProfileParam - theProfileTarget)
                        + Math.Abs(theGuideParam - theGuideTarget);
        if (theHasPreviousProfile)
        {
            // Favor branches with a smooth advance along the current profile.
            aScore += Math.Abs((theProfileParam - thePreviousProfileParam)
                               - (theProfileTarget - thePreviousProfileTarget));
        }
        if (theHasPreviousGuide)
        {
            // Favor branches with a smooth advance along the current guide.
            aScore += Math.Abs((theGuideParam - thePreviousGuideParam) - (theGuideTarget - thePreviousGuideTarget));
        }
        return aScore;
    }

    /// <summary>Returns the tangent mismatch between two parameters in the requested curve orientation.</summary>
    internal static bool seamTangentMismatch(BSplineCurve3D theCurve, double theFirstParameter,
                                             double theLastParameter, bool theIsForward, double theTolerance,
                                             ref double theMismatch)
    {
        double aFirstRawParameter =
          theIsForward ? theFirstParameter : theCurve.FirstParameter() + theCurve.LastParameter() - theFirstParameter;
        double aLastRawParameter =
          theIsForward ? theLastParameter : theCurve.FirstParameter() + theCurve.LastParameter() - theLastParameter;
        Curve3D.ResD1 aFirstD1 = theCurve.EvalD1(aFirstRawParameter);
        Curve3D.ResD1 aLastD1 = theCurve.EvalD1(aLastRawParameter);
        if (aFirstD1.Point.Distance(aLastD1.Point) > Max(theTolerance, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            return false;
        }

        Vector3D aFirstTangent = aFirstD1.D1;
        Vector3D aLastTangent = aLastD1.D1;
        if (!theIsForward)
        {
            aFirstTangent.Reverse();
            aLastTangent.Reverse();
        }
        double aFirstMagnitude = aFirstTangent.Magnitude();
        double aLastMagnitude = aLastTangent.Magnitude();
        if (aFirstMagnitude <= Constants.Resolution() || aLastMagnitude <= Constants.Resolution())
        {
            return false;
        }

        theMismatch = 1.0 - clamp(aFirstTangent.Dot(aLastTangent) / (aFirstMagnitude * aLastMagnitude), -1.0, 1.0);
        return true;
    }

    /// <summary>Selects one contact compatible with already selected row/column predecessors.</summary>
    internal static bool chooseOrderedContact(ContactSet theContacts, BSplineCurve3D theProfile,
                                              BSplineCurve3D theGuide, bool theProfileForward,
                                              bool theGuideForward, double theProfileTarget, double theGuideTarget,
                                              double thePreviousProfileParam, double thePreviousGuideParam,
                                              bool theHasPreviousProfile, bool theHasPreviousGuide,
                                              double thePreviousProfileTarget, double thePreviousGuideTarget,
                                              double theFirstProfileParam, double theFirstGuideParam,
                                              bool theIsProfileSeam, bool theIsGuideSeam, double theTolerance,
                                              ref Contact theSelected)
    {
        double aParamTol = parameterResolution(0.0, 1.0);
        double aBestTangentMismatch = RealLast();
        double aBest = RealLast();
        for (int aCandidateIdx = 0; aCandidateIdx < theContacts.Candidates.Size(); ++aCandidateIdx)
        {
            Contact aCandidate = theContacts.Candidates.Value(aCandidateIdx);
            double aProfileParam = orientedParameter(theProfile, aCandidate.ProfileParam, theProfileForward);
            double aGuideParam = orientedParameter(theGuide, aCandidate.GuideParam, theGuideForward);
            if ((theHasPreviousProfile && aProfileParam <= thePreviousProfileParam + aParamTol)
                || (theHasPreviousGuide && aGuideParam <= thePreviousGuideParam + aParamTol))
            {
                continue;
            }

            double aScore = orderedContactScore(aCandidate, aProfileParam, aGuideParam, theProfileTarget,
                                                theGuideTarget, thePreviousProfileParam, thePreviousGuideParam,
                                                thePreviousProfileTarget, thePreviousGuideTarget,
                                                theHasPreviousProfile, theHasPreviousGuide, theTolerance);
            double aTangentMismatch = 0.0;
            bool hasTangentMatch = false;
            if (theIsProfileSeam)
            {
                double aProfileMismatch = 0.0;
                if (seamTangentMismatch(theProfile, theFirstProfileParam, aProfileParam, theProfileForward,
                                        theTolerance, ref aProfileMismatch))
                {
                    aTangentMismatch += aProfileMismatch;
                    hasTangentMatch = true;
                }
            }
            if (theIsGuideSeam)
            {
                double aGuideMismatch = 0.0;
                if (seamTangentMismatch(theGuide, theFirstGuideParam, aGuideParam, theGuideForward, theTolerance,
                                        ref aGuideMismatch))
                {
                    aTangentMismatch += aGuideMismatch;
                    hasTangentMatch = true;
                }
            }

            if ((hasTangentMatch
                 && (aTangentMismatch < aBestTangentMismatch - global::Kerf.Engine.Foundation.Precision.Angular()
                     || (Math.Abs(aTangentMismatch - aBestTangentMismatch) <= global::Kerf.Engine.Foundation.Precision.Angular() && aScore < aBest)))
                || (!hasTangentMatch && aBestTangentMismatch == RealLast() && aScore < aBest))
            {
                aBestTangentMismatch = hasTangentMatch ? aTangentMismatch : RealLast();
                aBest = aScore;
                theSelected = aCandidate;
                theSelected.ProfileParam = aProfileParam;
                theSelected.GuideParam = aGuideParam;
            }
        }
        return theSelected.IsValid;
    }

    /// <summary>Reorders curves and contact matrices into a consistent network layout.</summary>
    internal static bool reorderNetwork(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                        NCollection_Array1<BSplineCurve3D?> theGuides,
                                        NCollection_Array2<double> theProfileParams,
                                        NCollection_Array2<double> theGuideParams,
                                        NCollection_LinearVector<ContactSet> theContactSets, double theTolerance)
    {
        if (theProfiles.Size() < 2 || theGuides.Size() < 2)
        {
            return false;
        }

        Matrix aProfileParamMatrix = toMatrix(theProfileParams);
        Matrix aGuideParamMatrix = toMatrix(theGuideParams);

        NCollection_LinearVector<OrderedCurve> aProfileOrder = orderedCurves(rowMeans(aGuideParamMatrix));
        NCollection_LinearVector<OrderedCurve> aGuideOrder = orderedCurves(columnMeans(aProfileParamMatrix));

        NCollection_Array1<BSplineCurve3D?> aProfiles = new(1, theProfiles.Size());
        NCollection_Array1<BSplineCurve3D?> aGuides = new(1, theGuides.Size());
        NCollection_Array2<double> aProfileParams = new(theProfileParams.LowerRow(), theProfileParams.UpperRow(),
                                                        theProfileParams.LowerCol(), theProfileParams.UpperCol());
        NCollection_Array2<double> aGuideParams = new(theGuideParams.LowerRow(), theGuideParams.UpperRow(),
                                                      theGuideParams.LowerCol(), theGuideParams.UpperCol());
        Matrix aProfileParamResult = toMatrix(aProfileParams);
        Matrix aGuideParamResult = toMatrix(aGuideParams);

        NCollection_LinearVector<bool> aProfileForward = new();
        for (int aProfileIdx = 0; aProfileIdx < aProfileOrder.Size(); ++aProfileIdx)
        {
            int aSrcProfileIdx = aProfileOrder.Value(aProfileIdx).Index;
            bool isForward = aProfileParamMatrix[aSrcProfileIdx + 1, aGuideOrder.First().Index + 1]
                             <= aProfileParamMatrix[aSrcProfileIdx + 1, aGuideOrder.Last().Index + 1];
            aProfiles.ChangeAt(aProfileIdx) = orientedCurve(theProfiles.At(aSrcProfileIdx)!, isForward);
            aProfileForward.Append(isForward);
        }

        NCollection_LinearVector<bool> aGuideForward = new();
        for (int aGuideIdx = 0; aGuideIdx < aGuideOrder.Size(); ++aGuideIdx)
        {
            int aSrcGuideIdx = aGuideOrder.Value(aGuideIdx).Index;
            bool isForward = aGuideParamMatrix[aProfileOrder.First().Index + 1, aSrcGuideIdx + 1]
                             <= aGuideParamMatrix[aProfileOrder.Last().Index + 1, aSrcGuideIdx + 1];
            aGuides.ChangeAt(aGuideIdx) = orientedCurve(theGuides.At(aSrcGuideIdx)!, isForward);
            aGuideForward.Append(isForward);
        }

        for (int aProfileIdx = 0; aProfileIdx < aProfileOrder.Size(); ++aProfileIdx)
        {
            int aSrcProfileIdx = aProfileOrder.Value(aProfileIdx).Index;
            for (int aGuideIdx = 0; aGuideIdx < aGuideOrder.Size(); ++aGuideIdx)
            {
                int aSrcGuideIdx = aGuideOrder.Value(aGuideIdx).Index;
                double aProfileTarget = aGuideOrder.Size() == 1 ? 0.0 : (double)aGuideIdx / (aGuideOrder.Size() - 1);
                double aGuideTarget =
                  aProfileOrder.Size() == 1 ? 0.0 : (double)aProfileIdx / (aProfileOrder.Size() - 1);
                double aPreviousProfileParam = aGuideIdx == 0 ? 0.0 : aProfileParamResult[aProfileIdx + 1, aGuideIdx];
                double aPreviousGuideParam = aProfileIdx == 0 ? 0.0 : aGuideParamResult[aProfileIdx, aGuideIdx + 1];
                double aPreviousProfileTarget =
                  aGuideIdx < 2 ? 0.0 : (double)(aGuideIdx - 1) / (aGuideOrder.Size() - 1);
                double aPreviousGuideTarget =
                  aProfileIdx < 2 ? 0.0 : (double)(aProfileIdx - 1) / (aProfileOrder.Size() - 1);
                bool isProfileSeam = aGuideIdx + 1 == aGuideOrder.Size();
                bool isGuideSeam = aProfileIdx + 1 == aProfileOrder.Size();
                double aFirstProfileParam = isProfileSeam ? aProfileParamResult[aProfileIdx + 1, 1] : 0.0;
                double aFirstGuideParam = isGuideSeam ? aGuideParamResult[1, aGuideIdx + 1] : 0.0;

                Contact aContact = new();
                if (!chooseOrderedContact(theContactSets.Value(contactIndex(aSrcProfileIdx, aSrcGuideIdx, theGuides.Size())),
                                          theProfiles.At(aSrcProfileIdx)!, theGuides.At(aSrcGuideIdx)!,
                                          aProfileForward.Value(aProfileIdx), aGuideForward.Value(aGuideIdx),
                                          aProfileTarget, aGuideTarget, aPreviousProfileParam, aPreviousGuideParam,
                                          aGuideIdx != 0, aProfileIdx != 0, aPreviousProfileTarget,
                                          aPreviousGuideTarget, aFirstProfileParam, aFirstGuideParam, isProfileSeam,
                                          isGuideSeam, theTolerance, ref aContact))
                {
                    return false;
                }

                aProfileParamResult[aProfileIdx + 1, aGuideIdx + 1] = aContact.ProfileParam;
                aGuideParamResult[aProfileIdx + 1, aGuideIdx + 1] = aContact.GuideParam;
            }
        }

        openClosedBoundaryCandidates(aProfiles, aGuides, aProfileParams, aGuideParams, theTolerance);

        if (!isMonotoneLayout(aProfileParamResult, aGuideParamResult))
        {
            return false;
        }

        theProfiles.Assign(aProfiles);
        theGuides.Assign(aGuides);
        theProfileParams.Assign(aProfileParams);
        theGuideParams.Assign(aGuideParams);
        return true;
    }

    /// <summary>Monotone scalar map converting target parameters back to the source curve.</summary>
    internal sealed class ScalarLaw : EvaluatorFunction
    {
        private readonly NCollection_LinearVector<double> mySource = new();
        private readonly NCollection_LinearVector<double> myTarget = new();

        /// <summary>Stores a strictly increasing piecewise-linear parameter map.</summary>
        public bool Init(math_Vector theSource, math_Vector theTarget, int theNbValues)
        {
            if (theNbValues < 2)
            {
                return false;
            }

            mySource.Clear();
            myTarget.Clear();
            for (int aParamIdx = 1; aParamIdx <= theNbValues; ++aParamIdx)
            {
                if (!mySource.IsEmpty()
                    && (theSource[aParamIdx] <= mySource.Last() + global::Kerf.Engine.Foundation.Precision.PConfusion()
                        || theTarget[aParamIdx] <= myTarget.Last() + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    return false;
                }
                mySource.Append(theSource[aParamIdx]);
                myTarget.Append(theTarget[aParamIdx]);
            }
            return true;
        }

        /// <summary>Evaluates the map by linear interpolation and clamps to source bounds.</summary>
        public double Value(double theTargetParameter)
        {
            if (mySource.IsEmpty())
            {
                return 0.0;
            }
            if (theTargetParameter <= myTarget.First())
            {
                return mySource.First();
            }
            if (theTargetParameter >= myTarget.Last())
            {
                return mySource.Last();
            }

            for (int aParamIdx = 1; aParamIdx < myTarget.Size(); ++aParamIdx)
            {
                if (theTargetParameter <= myTarget.Value(aParamIdx))
                {
                    double aT0 = myTarget.Value(aParamIdx - 1);
                    double aT1 = myTarget.Value(aParamIdx);
                    double aWeight = (theTargetParameter - aT0) / (aT1 - aT0);
                    return mySource.Value(aParamIdx - 1)
                           + aWeight * (mySource.Value(aParamIdx) - mySource.Value(aParamIdx - 1));
                }
            }
            return mySource.Last();
        }

        public override void Evaluate(int theDerivativeRequest, ReadOnlySpan<double> theStartEnd,
                                      double theTargetParameter, out double theResult, out int theErrorCode)
        {
            theErrorCode = 0;
            if (theDerivativeRequest == 0)
            {
                theResult = Value(theTargetParameter);
                return;
            }
            if (theDerivativeRequest != 1)
            {
                theResult = 0.0;
                return;
            }

            for (int aParamIdx = 1; aParamIdx < myTarget.Size(); ++aParamIdx)
            {
                if (theTargetParameter <= myTarget.Value(aParamIdx))
                {
                    theResult = (mySource.Value(aParamIdx) - mySource.Value(aParamIdx - 1))
                                / (myTarget.Value(aParamIdx) - myTarget.Value(aParamIdx - 1));
                    return;
                }
            }
            theResult = (mySource.Last() - mySource.Value(mySource.Size() - 2))
                        / (myTarget.Last() - myTarget.Value(myTarget.Size() - 2));
        }
    }

    /// <summary>Reparametrizes a polynomial B-spline exactly using a piecewise-linear parameter law.</summary>
    internal static bool reparametrizeExactly(BSplineCurve3D theCurve, math_Vector theSourceParams,
                                              math_Vector theTargetParams, int theNbConstraints, ScalarLaw theLaw,
                                              ref BSplineCurve3D? theResult)
    {
        if (theNbConstraints < 2)
        {
            return false;
        }

        int aDegree = theCurve.Degree();
        NCollection_LinearVector<double> aNewKnotValues = new();
        NCollection_LinearVector<int> aNewKnotMults = new();
        int aConstraintIdx = 1;
        NCollection_Array1<double> aCurveKnots = theCurve.InternalKnots();
        NCollection_Array1<int> aCurveMults = theCurve.InternalMultiplicities();
        for (int aKnotIdx = 0; aKnotIdx < aCurveKnots.Size(); ++aKnotIdx)
        {
            double aSourceKnot = aCurveKnots.At(aKnotIdx);
            while (aConstraintIdx <= theNbConstraints
                   && theSourceParams[aConstraintIdx] < aSourceKnot - global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aNewKnotValues.Append(theTargetParams[aConstraintIdx]);
                aNewKnotMults.Append(aDegree);
                ++aConstraintIdx;
            }

            double aTargetKnot = theTargetParams[theNbConstraints];
            for (int aParamIdx = 1; aParamIdx < theNbConstraints; ++aParamIdx)
            {
                if (aSourceKnot <= theSourceParams[aParamIdx + 1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    double aSourceFirst = theSourceParams[aParamIdx];
                    double aSourceLast = theSourceParams[aParamIdx + 1];
                    double aTargetFirst = theTargetParams[aParamIdx];
                    double aTargetLast = theTargetParams[aParamIdx + 1];
                    aTargetKnot = aTargetFirst
                                  + (aSourceKnot - aSourceFirst) * (aTargetLast - aTargetFirst)
                                    / (aSourceLast - aSourceFirst);
                    break;
                }
            }

            int aMultiplicity = aCurveMults.At(aKnotIdx);
            if (aConstraintIdx <= theNbConstraints
                && Math.Abs(theSourceParams[aConstraintIdx] - aSourceKnot) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (aConstraintIdx > 1 && aConstraintIdx < theNbConstraints)
                {
                    aMultiplicity = Max(aMultiplicity, aDegree);
                }
                ++aConstraintIdx;
            }
            aNewKnotValues.Append(aTargetKnot);
            aNewKnotMults.Append(aMultiplicity);
        }

        NCollection_Array1<double> aNewKnots = toOneBasedArrayView(aNewKnotValues);
        NCollection_Array1<int> aNewMults = toOneBasedArrayView(aNewKnotMults);
        int aNbNewPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDegree, false, aNewMults);
        NCollection_Array1<double> aSourceFlatKnots = new(1, theCurve.PoleCount + aDegree + 1);
        NCollection_Array1<double> aTargetFlatKnots = new(1, aNbNewPoles + aDegree + 1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(aCurveKnots, aCurveMults, aSourceFlatKnots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(aNewKnots, aNewMults, aTargetFlatKnots);

        int aStatus;
        if (!theCurve.IsRational)
        {
            NCollection_Array1<Point3D> aNewPoles = new(1, aNbNewPoles);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(theLaw, aDegree, aSourceFlatKnots, theCurve.InternalPoles(),
                                            aTargetFlatKnots, aDegree, aNewPoles, out aStatus);
            if (aStatus != 0)
            {
                return false;
            }
            theResult = new BSplineCurve3D(aNewPoles, aNewKnots, aNewMults, aDegree);
            return theResult is not null;
        }

        NCollection_Array1<Point3D> aCurvePoles = theCurve.InternalPoles();
        NCollection_Array1<double> aCurveWeights = theCurve.InternalWeightsArray();
        NCollection_Array1<double> aHomogeneousPoles = new(1, 4 * theCurve.PoleCount);
        for (int aPoleIdx = 0; aPoleIdx < aCurvePoles.Size(); ++aPoleIdx)
        {
            Point3D aPole = aCurvePoles.At(aPoleIdx);
            double aWeight = aCurveWeights.At(aPoleIdx);
            int aHomogeneousIdx = 4 * aPoleIdx;
            aHomogeneousPoles.ChangeAt(aHomogeneousIdx) = aPole.X() * aWeight;
            aHomogeneousPoles.ChangeAt(aHomogeneousIdx + 1) = aPole.Y() * aWeight;
            aHomogeneousPoles.ChangeAt(aHomogeneousIdx + 2) = aPole.Z() * aWeight;
            aHomogeneousPoles.ChangeAt(aHomogeneousIdx + 3) = aWeight;
        }

        NCollection_Array1<double> aNewHomogeneousPoles = new(1, 4 * aNbNewPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(theLaw, aDegree, aSourceFlatKnots, 4, aHomogeneousPoles.AsSpan(),
                                        aTargetFlatKnots, aDegree, aNewHomogeneousPoles.AsSpan(), out aStatus);
        if (aStatus != 0)
        {
            return false;
        }

        NCollection_Array1<Point3D> aNewPoles2 = new(1, aNbNewPoles);
        NCollection_Array1<double> aNewWeights = new(1, aNbNewPoles);
        for (int aPoleIdx = 0; aPoleIdx < aNewPoles2.Size(); ++aPoleIdx)
        {
            int aHomogeneousIdx = 4 * aPoleIdx;
            double aWeight = aNewHomogeneousPoles.At(aHomogeneousIdx + 3);
            if (aWeight <= Constants.Resolution())
            {
                return false;
            }
            aNewPoles2.ChangeAt(aPoleIdx).SetCoord(aNewHomogeneousPoles.At(aHomogeneousIdx) / aWeight,
                                                   aNewHomogeneousPoles.At(aHomogeneousIdx + 1) / aWeight,
                                                   aNewHomogeneousPoles.At(aHomogeneousIdx + 2) / aWeight);
            aNewWeights.ChangeAt(aPoleIdx) = aWeight;
        }

        theResult = new BSplineCurve3D(aNewPoles2, aNewWeights, aNewKnots, aNewMults, aDegree);
        return theResult is not null;
    }

    /// <summary>Appends a monotone source/target pair.</summary>
    internal static void pushConstraint(math_Vector theSource, math_Vector theTarget, ref int theNbConstraints,
                                        double theSourceParam, double theTargetParam, double theSourceTol,
                                        double theTargetTol)
    {
        if (theNbConstraints > 0
            && (Math.Abs(theSourceParam - theSource[theNbConstraints]) <= theSourceTol
                || Math.Abs(theTargetParam - theTarget[theNbConstraints]) <= theTargetTol))
        {
            return;
        }

        ++theNbConstraints;
        theSource[theNbConstraints] = theSourceParam;
        theTarget[theNbConstraints] = Max(0.0, Min(1.0, theTargetParam));
    }

    /// <summary>Builds target-space samples from constraints and deterministic interval subdivision.</summary>
    internal static NCollection_LinearVector<double> samplingParameters(math_Vector theTargets, int theNbSourcePoles)
    {
        NCollection_LinearVector<double> aSamples = new();
        appendUnitParameter(aSamples, 0.0);
        appendUnitParameter(aSamples, 1.0);
        for (int aParamIdx = theTargets.Lower(); aParamIdx <= theTargets.Upper(); ++aParamIdx)
        {
            appendUnitParameter(aSamples, theTargets[aParamIdx]);
        }

        compactParameters(aSamples);

        // The rebuilt curve is parameter-driven; extra samples are deterministic.
        addIntervalMidpoints(aSamples,
                             Max(interpolationDegree() + 1, Max(theNbSourcePoles * 2, theTargets.Length() + 2)));
        return aSamples;
    }

    /// <summary>Rebuilds one curve through target-parameter samples.</summary>
    internal static bool rebuildCurve(ref BSplineCurve3D? theCurve, math_Vector theSourceParams,
                                      math_Vector theTargetParams, bool theAllowApproximateRational,
                                      ref bool theDidApproximate, ref double theMaxDeviation)
    {
        BSplineCurve3D aCurve = theCurve!;
        math_Vector aSource = new(1, theSourceParams.Length() + 2);
        math_Vector aTarget = new(1, theTargetParams.Length() + 2);
        int aNbConstraints = 0;
        double aSourceTol = parameterResolution(aCurve.FirstParameter(), aCurve.LastParameter());
        double aTargetTol = parameterResolution(0.0, 1.0);

        pushConstraint(aSource, aTarget, ref aNbConstraints, aCurve.FirstParameter(), 0.0, aSourceTol, aTargetTol);
        for (int aParamIdx = theSourceParams.Lower(); aParamIdx <= theSourceParams.Upper(); ++aParamIdx)
        {
            double aSourceParam = theSourceParams[aParamIdx];
            double aTargetParam = theTargetParams[aParamIdx];
            if (Math.Abs(aSourceParam - aCurve.FirstParameter()) <= aSourceTol
                || Math.Abs(aSourceParam - aCurve.LastParameter()) <= aSourceTol
                || Math.Abs(aTargetParam) <= aTargetTol || Math.Abs(aTargetParam - 1.0) <= aTargetTol)
            {
                continue;
            }
            pushConstraint(aSource, aTarget, ref aNbConstraints, aSourceParam, aTargetParam, aSourceTol, aTargetTol);
        }
        pushConstraint(aSource, aTarget, ref aNbConstraints, aCurve.LastParameter(), 1.0, aSourceTol, aTargetTol);

        ScalarLaw aLaw = new();
        if (!aLaw.Init(aSource, aTarget, aNbConstraints))
        {
            return false;
        }

        BSplineCurve3D? anExactCurve = null;
        math_Vector aExactSource = new(1, aNbConstraints);
        math_Vector aExactTarget = new(1, aNbConstraints);
        for (int aConstraintIdx = 1; aConstraintIdx <= aNbConstraints; ++aConstraintIdx)
        {
            aExactSource[aConstraintIdx] = aSource[aConstraintIdx];
            aExactTarget[aConstraintIdx] = aTarget[aConstraintIdx];
        }
        if (reparametrizeExactly(aCurve, aExactSource, aExactTarget, aNbConstraints, aLaw, ref anExactCurve))
        {
            theCurve = anExactCurve;
            return true;
        }

        if (aCurve.IsRational)
        {
            if (!theAllowApproximateRational)
            {
                return false;
            }
            theDidApproximate = true;
        }

        CurveAdaptor aCurveAdaptor = new(aCurve);
        NCollection_LinearVector<double> aSamples = samplingParameters(theTargetParams, aCurve.PoleCount);
        NCollection_LinearVector<Point3D> aPoints = new();
        NCollection_LinearVector<double> aParameters = new();
        for (int aSampleIdx = 0; aSampleIdx < aSamples.Size(); ++aSampleIdx)
        {
            double aTargetParam = aSamples.Value(aSampleIdx);
            Point3D aPoint = aCurveAdaptor.EvalD0(aLaw.Value(aTargetParam));
            if (!aPoints.IsEmpty() && aPoint.Distance(aPoints.Last()) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            aPoints.Append(aPoint);
            aParameters.Append(aTargetParam);
        }

        int aNbPoints = aPoints.Size();
        if (aNbPoints < interpolationDegree())
        {
            return false;
        }

        NCollection_Array1<Point3D> aPointArray = toOneBasedArrayView(aPoints);
        NCollection_Array1<double> aParamArray = toOneBasedArrayView(aParameters);

        Interpolate anInterpolator = new(interpolationDegree(), aNbPoints, aPointArray, aParamArray);
        if (!anInterpolator.IsDone)
        {
            return false;
        }

        // GeomLib_Interpolate freezes the curve it hands out; the prepared network only reads it afterwards.
        theCurve = anInterpolator.Curve();
        if (theCurve is null)
        {
            return false;
        }

        CurveAdaptor aNewCurveAdaptor = new(theCurve);
        for (int aSampleIdx = 0; aSampleIdx < aSamples.Size(); ++aSampleIdx)
        {
            double aTargetParam = aSamples.Value(aSampleIdx);
            Point3D aSourcePoint = aCurveAdaptor.EvalD0(aLaw.Value(aTargetParam));
            Point3D aNewPoint = aNewCurveAdaptor.EvalD0(aTargetParam);
            theMaxDeviation = Max(theMaxDeviation, aSourcePoint.Distance(aNewPoint));
        }
        return theCurve is not null;
    }

    /// <summary>Reconstructs curves so all intersections share averaged parameters.</summary>
    internal static bool equalizeParameters(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                            NCollection_Array1<BSplineCurve3D?> theGuides,
                                            NCollection_Array2<double> theProfileParams,
                                            NCollection_Array2<double> theGuideParams,
                                            bool theAllowApproximateRational, ref bool theDidApproximate,
                                            ref double theMaxDeviation)
    {
        Matrix aProfileParams = toMatrix(theProfileParams);
        Matrix aGuideParams = toMatrix(theGuideParams);

        math_Vector aProfileTargets = columnMeans(aProfileParams);
        math_Vector aGuideTargets = rowMeans(aGuideParams);

        math_Vector anOldProfileParams = new(1, theGuides.Size());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            int aProfileRow = aProfileIdx + 1;
            bool isAlreadyTarget = true;
            for (int aCol = anOldProfileParams.Lower(); aCol <= anOldProfileParams.Upper(); ++aCol)
            {
                anOldProfileParams[aCol] = aProfileParams[aProfileRow, aCol];
                isAlreadyTarget = isAlreadyTarget
                                  && Math.Abs(anOldProfileParams[aCol] - aProfileTargets[aCol]) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            if (isAlreadyTarget)
            {
                continue;
            }
            if (!rebuildCurve(ref theProfiles.ChangeAt(aProfileIdx), anOldProfileParams, aProfileTargets,
                              theAllowApproximateRational, ref theDidApproximate, ref theMaxDeviation))
            {
                return false;
            }
            for (int aCol = aProfileTargets.Lower(); aCol <= aProfileTargets.Upper(); ++aCol)
            {
                aProfileParams[aProfileRow, aCol] = aProfileTargets[aCol];
            }
        }

        math_Vector anOldGuideParams = new(1, theProfiles.Size());
        for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
        {
            int aGuideCol = aGuideIdx + 1;
            bool isAlreadyTarget = true;
            for (int aRow = anOldGuideParams.Lower(); aRow <= anOldGuideParams.Upper(); ++aRow)
            {
                anOldGuideParams[aRow] = aGuideParams[aRow, aGuideCol];
                isAlreadyTarget = isAlreadyTarget
                                  && Math.Abs(anOldGuideParams[aRow] - aGuideTargets[aRow]) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            if (isAlreadyTarget)
            {
                continue;
            }
            if (!rebuildCurve(ref theGuides.ChangeAt(aGuideIdx), anOldGuideParams, aGuideTargets,
                              theAllowApproximateRational, ref theDidApproximate, ref theMaxDeviation))
            {
                return false;
            }
            for (int aRow = aGuideTargets.Lower(); aRow <= aGuideTargets.Upper(); ++aRow)
            {
                aGuideParams[aRow, aGuideCol] = aGuideTargets[aRow];
            }
        }
        return true;
    }

    /// <summary>Evaluates the validated profile parameters into a guide/profile point grid.</summary>
    internal static NCollection_Array2<Point3D> contactPointGrid(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                                NCollection_Array2<double> theProfileParams)
    {
        NCollection_Array2<Point3D> aPoints = new(1, theProfileParams.RowLength(), 1, theProfileParams.ColLength());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            CurveAdaptor aProfileAdaptor = new(theProfiles.At(aProfileIdx)!);
            for (int aGuideIdx = 0; aGuideIdx < theProfileParams.RowLength(); ++aGuideIdx)
            {
                aPoints.ChangeAt(aGuideIdx, aProfileIdx) =
                  aProfileAdaptor.EvalD0(theProfileParams.At(aProfileIdx, aGuideIdx));
            }
        }
        return aPoints;
    }

    /// <summary>Evaluates the denominator of a rational B-spline curve at a contact parameter.</summary>
    internal static double curveWeightAt(BSplineCurve3D theCurve, double theParameter)
    {
        if (!theCurve.IsRational)
        {
            return 1.0;
        }

        ReadOnlySpan<int> anExtrapMode = [theCurve.Degree(), theCurve.Degree()];
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(theParameter, theCurve.IsPeriodic, false, anExtrapMode, theCurve.Degree(),
                      theCurve.InternalKnotSequence(), theCurve.InternalPoles(), theCurve.InternalWeightsArray(),
                      out Point3D _, out double aWeight);
        return aWeight;
    }

    /// <summary>Evaluates rational contact weights in guide/profile grid order.</summary>
    internal static NCollection_Array2<double> contactWeightGrid(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                                 NCollection_Array2<double> theProfileParams)
    {
        NCollection_Array2<double> aWeights = new(1, theProfileParams.RowLength(), 1, theProfileParams.ColLength());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            for (int aGuideIdx = 0; aGuideIdx < theProfileParams.RowLength(); ++aGuideIdx)
            {
                aWeights.ChangeAt(aGuideIdx, aProfileIdx) =
                  curveWeightAt(theProfiles.At(aProfileIdx)!, theProfileParams.At(aProfileIdx, aGuideIdx));
            }
        }
        return aWeights;
    }

    /// <summary>
    /// Performs the preparatory network operations required before Gordon surface construction, on the arrays
    /// GeomFill_Gordon owns (kept by reference; changed in place, as OCCT's references).
    /// </summary>
    internal sealed class NetworkPreparation
    {
        private readonly NCollection_Array1<BSplineCurve3D?> myProfiles;
        private readonly NCollection_Array1<BSplineCurve3D?> myGuides;
        private readonly NCollection_Array2<double> myProfileParams;
        private readonly NCollection_Array2<double> myGuideParams;
        private readonly NCollection_LinearVector<ContactSet> myContactSets = new();
        private readonly double myTolerance;
        private double myMaxContactGap = 0.0;
        private double myMaxReparametrizationDeviation = 0.0;
        private readonly bool myToUseParallel;

        public NetworkPreparation(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                  NCollection_Array1<BSplineCurve3D?> theGuides,
                                  NCollection_Array2<double> theProfileParams,
                                  NCollection_Array2<double> theGuideParams, double theTolerance,
                                  bool theToUseParallel)
        {
            myProfiles = theProfiles;
            myGuides = theGuides;
            myProfileParams = theProfileParams;
            myGuideParams = theGuideParams;
            myTolerance = theTolerance;
            myToUseParallel = theToUseParallel;
        }

        /// <summary>Converts all curves to explicit unit-domain B-splines.</summary>
        public bool ConvertCurvesToUnitInterval()
        {
            return makeUnitCurves(myProfiles) && makeUnitCurves(myGuides);
        }

        /// <summary>Computes the profile/guide intersection parameter tables.</summary>
        public bool FillIntersectionParameters()
        {
            bool isDone = fillContactTable(myProfiles, myGuides, myProfileParams, myGuideParams, myContactSets,
                                           myTolerance, myToUseParallel);
            myMaxContactGap = isDone ? maxNearestContactGap(myContactSets) : 0.0;
            return isDone;
        }

        /// <summary>Reorders and orients the curve network.</summary>
        public bool ReorderNetwork()
        {
            return reorderNetwork(myProfiles, myGuides, myProfileParams, myGuideParams, myContactSets, myTolerance);
        }

        /// <summary>Snaps, validates, and detects closed directions in the intersection table.</summary>
        public bool NormalizeIntersectionParameters(ref bool theIsUClosed, ref bool theIsVClosed)
        {
            return normalizeContactTable(myProfiles, myGuides, myProfileParams, myGuideParams, myTolerance,
                                         ref theIsUClosed, ref theIsVClosed);
        }

        /// <summary>
        /// Rebuilds curves to shared averaged network parameters. Rational curves are rebuilt only when approximate
        /// fallback is allowed.
        /// </summary>
        public bool EqualizeIntersectionParameters(bool theAllowApproximateRational, ref bool theDidApproximate)
        {
            myMaxReparametrizationDeviation = 0.0;
            return equalizeParameters(myProfiles, myGuides, myProfileParams, myGuideParams,
                                      theAllowApproximateRational, ref theDidApproximate,
                                      ref myMaxReparametrizationDeviation);
        }

        /// <summary>Returns the largest best-contact gap found during contact discovery.</summary>
        public double MaxContactGap() => myMaxContactGap;

        /// <summary>Returns the largest measured curve rebuild deviation.</summary>
        public double MaxReparametrizationDeviation() => myMaxReparametrizationDeviation;

        /// <summary>Returns the validated contact points in guide/profile grid order.</summary>
        public NCollection_Array2<Point3D> IntersectionPoints() => contactPointGrid(myProfiles, myProfileParams);

        /// <summary>Returns rational contact weights in guide/profile grid order.</summary>
        public NCollection_Array2<double> IntersectionWeights() => contactWeightGrid(myProfiles, myProfileParams);
    }
}
