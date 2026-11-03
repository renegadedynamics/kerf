// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GordonUtilities.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Geometry.Splines;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// GeomFill_GordonUtilities.pxx (OCCT's namespace <c>GordonUtilities</c>): the preparation of a Gordon curve network
/// for <see cref="Gordon"/>: conversion to unit-domain non-periodic B-splines, profile/guide contacts, network
/// ordering and orientation, closed-seam detection and the exact (or sampled) reparametrisation of every curve to
/// shared intersection parameters.
/// </summary>
/// <remarks>
/// OCCT's anonymous-namespace functions are internal static methods. Its views without copying are kept where they
/// are written through (<see cref="ToMatrix"/> shares the parameter table, as OCCT's math_Matrix over
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

        public Contact(bool isValid, double profileParam, double guideParam, double gap)
        {
            IsValid = isValid;
            ProfileParam = profileParam;
            GuideParam = guideParam;
            Gap = gap;
        }
    }

    /// <summary>Stores all admissible contacts for one profile/guide pair.</summary>
    internal sealed class ContactSet
    {
        public NCollection_LinearVector<Contact> Candidates = new();
    }

    private readonly struct DoubleLess : Heap.ILess<double>
    {
        public bool Less(double left, double right) => left < right;
    }

    private readonly struct ContactLess : Heap.ILess<Contact>
    {
        public bool Less(Contact left, Contact right)
        {
            if (left.Gap != right.Gap)
            {
                return left.Gap < right.Gap;
            }
            if (left.ProfileParam != right.ProfileParam)
            {
                return left.ProfileParam < right.ProfileParam;
            }
            return left.GuideParam < right.GuideParam;
        }
    }

    private readonly struct OrderedCurveLess : Heap.ILess<OrderedCurve>
    {
        public bool Less(OrderedCurve left, OrderedCurve right) => left.Rank < right.Rank;
    }

    // std::clamp
    private static double Clamp(double value, double low, double high)
        => value < low ? low : (high < value ? high : value);

    /// <summary>Returns a stable cubic degree when the OCCT degree limit allows it.</summary>
    internal static int InterpolationDegree()
    {
        return Min(3, BSplineCurve3D.MaxDegree);
    }

    /// <summary>Returns a parameter comparison tolerance scaled by the interval length.</summary>
    internal static double ParameterResolution(double first, double last)
    {
        return global::Kerf.Engine.Foundation.Precision.PIntersection(Max(Math.Abs(last - first), 1.0));
    }

    /// <summary>Returns a network tolerance converted to the given parameter range.</summary>
    internal static double BoundaryParameterResolution(double first, double last, double tolerance)
    {
        double range = Max(Math.Abs(last - first), 1.0);
        return global::Kerf.Engine.Foundation.Precision.Parametric(Max(tolerance, global::Kerf.Engine.Foundation.Precision.Approximation())) * range;
    }

    /// <summary>Appends a clamped parameter candidate.</summary>
    internal static void AppendUnitParameter(NCollection_LinearVector<double> parameters, double parameter)
    {
        parameters.Append(Max(0.0, Min(1.0, parameter)));
    }

    /// <summary>Sorts and removes duplicate parameters in-place.</summary>
    internal static void CompactParameters(NCollection_LinearVector<double> parameters)
    {
        if (parameters.IsEmpty())
        {
            return;
        }

        Sorting.Sort(parameters.AsSpan(), new DoubleLess());
        double tol = ParameterResolution(0.0, 1.0);
        int writeIdx = 0;
        for (int readIdx = 1; readIdx < parameters.Size(); ++readIdx)
        {
            if (Math.Abs(parameters.Value(readIdx) - parameters.Value(writeIdx)) > tol)
            {
                ++writeIdx;
                parameters.ChangeValue(writeIdx) = parameters.Value(readIdx);
            }
        }

        while (parameters.Size() > writeIdx + 1)
        {
            parameters.EraseLast();
        }
    }

    /// <summary>Adds midpoint samples in the widest intervals until the requested count is reached.</summary>
    internal static void AddIntervalMidpoints(NCollection_LinearVector<double> parameters, int minSize)
    {
        CompactParameters(parameters);
        while (parameters.Size() < minSize && parameters.Size() > 1)
        {
            int splitIdx = 0;
            double maxGap = -1.0;
            for (int paramIdx = 1; paramIdx < parameters.Size(); ++paramIdx)
            {
                double gap = parameters.Value(paramIdx) - parameters.Value(paramIdx - 1);
                if (gap > maxGap)
                {
                    splitIdx = paramIdx;
                    maxGap = gap;
                }
            }
            if (maxGap <= ParameterResolution(0.0, 1.0))
            {
                return;
            }

            AppendUnitParameter(parameters,
                                0.5 * (parameters.Value(splitIdx - 1) + parameters.Value(splitIdx)));
            CompactParameters(parameters);
        }
    }

    /// <summary>
    /// A 1-based Array1 of a linear vector's values. OCCT wraps the vector's storage without copying; every use is
    /// read-only, so a copy gives the same values.
    /// </summary>
    internal static NCollection_Array1<T> ToOneBasedArrayView<T>(NCollection_LinearVector<T> values)
    {
        return NCollection_Array1<T>.From(values.AsSpan(), 1);
    }

    /// <summary>Wraps a parameter table into a math matrix without copying its storage: writes go to the table.</summary>
    internal static Matrix ToMatrix(NCollection_Array2<double> theParams)
    {
        (double[] data, int offset) = theParams.Storage();
        NCollection_Array1<double> view = new(data, offset, 1, theParams.Size());
        return new Matrix(view, 1, 1, theParams.ColLength(), 1, theParams.RowLength());
    }

    /// <summary>Expands a periodic B-spline period into explicit non-periodic storage.</summary>
    internal static bool ExposePeriod(BSplineCurve3D curve)
    {
        if (!curve.IsPeriodic)
        {
            return true;
        }

        try
        {
            double first = curve.FirstParameter;
            double last = curve.LastParameter;
            if (last <= first + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }

            curve.Segment(first, last, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (curve.IsPeriodic)
            {
                curve.SetNotPeriodic();
            }
        }
        catch (FailureException)
        {
            return false;
        }
        return !curve.IsPeriodic;
    }

    /// <summary>Normalizes an input curve family to explicit non-periodic B-splines on [0, 1].</summary>
    internal static bool MakeUnitCurves(NCollection_Array1<BSplineCurve3D?> curves)
    {
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            BSplineCurve3D? curve = curves.At(curveIdx);
            if (curve is null || !ExposePeriod(curve))
            {
                return false;
            }

            NCollection_Array1<double> knots = new(curve.InternalKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, knots);
            curve.SetKnots(knots);
        }
        return true;
    }

    /// <summary>Index and rank pair for ordering curves.</summary>
    internal struct OrderedCurve
    {
        public int Index;
        public double Rank;

        public OrderedCurve(int index, double rank)
        {
            Index = index;
            Rank = rank;
        }
    }

    /// <summary>Returns average values of matrix columns.</summary>
    internal static math_Vector ColumnMeans(Matrix theParams)
    {
        math_Vector ones = new(theParams.LowerRow(), theParams.UpperRow(), 1.0);
        math_Vector sums = theParams.Transposed().Multiplied(ones);
        sums /= (double)theParams.RowNumber();
        return sums;
    }

    /// <summary>Returns average values of matrix rows.</summary>
    internal static math_Vector RowMeans(Matrix theParams)
    {
        math_Vector ones = new(theParams.LowerCol(), theParams.UpperCol(), 1.0);
        math_Vector sums = theParams.Multiplied(ones);
        sums /= (double)theParams.ColNumber();
        return sums;
    }

    /// <summary>Appends a unique contact candidate if the corresponding curve points match.</summary>
    internal static void AppendContact(BSplineCurve3D profile, BSplineCurve3D guide, double profileParam,
                                       double guideParam, double tolerance, ContactSet contacts)
    {
        CurveAdaptor profileAdaptor = new(profile);
        CurveAdaptor guideAdaptor = new(guide);
        Point3D profilePoint = profileAdaptor.EvalD0(profileParam);
        Point3D guidePoint = guideAdaptor.EvalD0(guideParam);
        double gap = profilePoint.DistanceTo(guidePoint);
        if (gap > tolerance)
        {
            return;
        }

        double profileTol = ParameterResolution(profile.FirstParameter, profile.LastParameter);
        double guideTol = ParameterResolution(guide.FirstParameter, guide.LastParameter);
        for (int contactIdx = 0; contactIdx < contacts.Candidates.Size(); ++contactIdx)
        {
            Contact contact = contacts.Candidates.Value(contactIdx);
            if (Math.Abs(contact.ProfileParam - profileParam) <= profileTol
                && Math.Abs(contact.GuideParam - guideParam) <= guideTol)
            {
                return;
            }
        }

        contacts.Candidates.Append(new Contact(true, profileParam, guideParam, gap));
    }

    /// <summary>Converts two close extrema points to an exact curve-parameter contact.</summary>
    internal static void AppendExtremaContact(BSplineCurve3D profile, BSplineCurve3D guide,
                                              in Point3D profilePoint, in Point3D guidePoint, double tolerance,
                                              ContactSet contacts)
    {
        if (profilePoint.DistanceTo(guidePoint) > tolerance)
        {
            return;
        }

        Point3D sharedPoint = new((profilePoint.XYZ() + guidePoint.XYZ()) * 0.5);

        double profileParam;
        double guideParam = 0.0;
        if (GeometryLibraryTool.Parameter(profile, sharedPoint, tolerance, out profileParam)
            && GeometryLibraryTool.Parameter(guide, sharedPoint, tolerance, out guideParam))
        {
            AppendContact(profile, guide, profileParam, guideParam, tolerance, contacts);
        }
    }

    /// <summary>Adds endpoint contacts; these are important for opening closed seams.</summary>
    internal static void AppendBoundaryContacts(BSplineCurve3D profile, BSplineCurve3D guide,
                                                double tolerance, ContactSet contacts)
    {
        ReadOnlySpan<double> profileBounds = [profile.FirstParameter, profile.LastParameter];
        ReadOnlySpan<double> guideBounds = [guide.FirstParameter, guide.LastParameter];
        CurveAdaptor profileAdaptor = new(profile);
        CurveAdaptor guideAdaptor = new(guide);
        foreach (double profileParam in profileBounds)
        {
            if (GeometryLibraryTool.Parameter(guide, profileAdaptor.EvalD0(profileParam), tolerance,
                                       out double guideParam))
            {
                AppendContact(profile, guide, profileParam, guideParam, tolerance, contacts);
            }
        }
        foreach (double guideParam in guideBounds)
        {
            if (GeometryLibraryTool.Parameter(profile, guideAdaptor.EvalD0(guideParam), tolerance,
                                       out double profileParam))
            {
                AppendContact(profile, guide, profileParam, guideParam, tolerance, contacts);
            }
        }
    }

    internal static bool AppendLinearSegmentContact(BSplineCurve3D profile, BSplineCurve3D guide,
                                                    double tolerance, ContactSet contacts)
    {
        if (profile.Degree != 1 || guide.Degree != 1 || profile.PoleCount != 2
            || guide.PoleCount != 2)
        {
            return false;
        }

        double profileFirst = profile.FirstParameter;
        double profileLast = profile.LastParameter;
        double guideFirst = guide.FirstParameter;
        double guideLast = guide.LastParameter;
        if (profileLast <= profileFirst + global::Kerf.Engine.Foundation.Precision.PConfusion()
            || guideLast <= guideFirst + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return true;
        }

        CurveAdaptor profileAdaptor = new(profile);
        CurveAdaptor guideAdaptor = new(guide);
        Point3D p0 = profileAdaptor.EvalD0(profileFirst);
        Point3D p1 = profileAdaptor.EvalD0(profileLast);
        Point3D q0 = guideAdaptor.EvalD0(guideFirst);
        Point3D q1 = guideAdaptor.EvalD0(guideLast);

        Xyz u = p1.XYZ() - p0.XYZ();
        Xyz v = q1.XYZ() - q0.XYZ();
        Xyz w = p0.XYZ() - q0.XYZ();
        double a = u.SquareModulus();
        double b = u.Dot(v);
        double c = v.SquareModulus();
        double d = u.Dot(w);
        double e = v.Dot(w);
        if (a <= Constants.Resolution() || c <= Constants.Resolution())
        {
            return true;
        }

        double denom = a * c - b * b;
        if (Math.Abs(denom) <= Constants.Resolution())
        {
            return false;
        }

        double s;
        double t;
        s = Clamp((b * e - c * d) / denom, 0.0, 1.0);
        t = (b * s + e) / c;
        if (t < 0.0)
        {
            t = 0.0;
            s = Clamp(-d / a, 0.0, 1.0);
        }
        else if (t > 1.0)
        {
            t = 1.0;
            s = Clamp((b - d) / a, 0.0, 1.0);
        }

        AppendContact(profile, guide, profileFirst + s * (profileLast - profileFirst),
                      guideFirst + t * (guideLast - guideFirst), tolerance, contacts);
        return true;
    }

    /// <summary>Returns all close contacts detected for one profile/guide pair.</summary>
    internal static ContactSet CollectContacts(BSplineCurve3D profile, BSplineCurve3D guide,
                                               double tolerance)
    {
        ContactSet contacts = new();
        if (AppendLinearSegmentContact(profile, guide, tolerance, contacts))
        {
            return contacts;
        }

        ExtremaCurveCurve extrema = new(profile, guide);
        for (int extIdx = 1; extIdx <= extrema.NbExtrema(); ++extIdx)
        {
            extrema.Points(extIdx, out Point3D profilePoint, out Point3D guidePoint);
            AppendExtremaContact(profile, guide, profilePoint, guidePoint, tolerance, contacts);
        }

        {
            Point3D profilePoint = new();
            Point3D guidePoint = new();
            if (extrema.TotalNearestPoints(ref profilePoint, ref guidePoint))
            {
                AppendExtremaContact(profile, guide, profilePoint, guidePoint, tolerance, contacts);
            }
        }

        AppendBoundaryContacts(profile, guide, tolerance, contacts);
        Sorting.Sort(contacts.Candidates.AsSpan(), new ContactLess());
        return contacts;
    }

    /// <summary>Returns the flat index of a profile/guide pair.</summary>
    internal static int ContactIndex(int profileIdx, int guideIdx, int nbGuides)
    {
        return profileIdx * nbGuides + guideIdx;
    }

    /// <summary>Stores the lowest-gap contact for the initial ordering estimate.</summary>
    internal static bool ChooseNearestContacts(NCollection_LinearVector<ContactSet> contacts, int nbProfiles,
                                               int nbGuides, NCollection_Array2<double> profileParams,
                                               NCollection_Array2<double> guideParams)
    {
        for (int profileIdx = 0; profileIdx < nbProfiles; ++profileIdx)
        {
            for (int guideIdx = 0; guideIdx < nbGuides; ++guideIdx)
            {
                ContactSet set = contacts.Value(ContactIndex(profileIdx, guideIdx, nbGuides));
                if (set.Candidates.IsEmpty())
                {
                    return false;
                }
                Contact contact = set.Candidates.First();
                profileParams.ChangeAt(profileIdx, guideIdx) = contact.ProfileParam;
                guideParams.ChangeAt(profileIdx, guideIdx) = contact.GuideParam;
            }
        }
        return true;
    }

    /// <summary>Returns the largest best-contact gap in a complete contact table.</summary>
    internal static double MaxNearestContactGap(NCollection_LinearVector<ContactSet> contacts)
    {
        double maxGap = 0.0;
        for (int setIdx = 0; setIdx < contacts.Size(); ++setIdx)
        {
            ContactSet set = contacts.Value(setIdx);
            if (!set.Candidates.IsEmpty())
            {
                maxGap = Max(maxGap, set.Candidates.First().Gap);
            }
        }
        return maxGap;
    }

    /// <summary>
    /// Builds all profile/guide contacts. The pairs are independent, each writes only its own set, so a parallel run
    /// gives the serial result (decision 490).
    /// </summary>
    internal static bool FillContactTable(NCollection_Array1<BSplineCurve3D?> profiles,
                                          NCollection_Array1<BSplineCurve3D?> guides,
                                          NCollection_Array2<double> profileParams,
                                          NCollection_Array2<double> guideParams,
                                          NCollection_LinearVector<ContactSet> contactSets, double tolerance,
                                          bool toUseParallel)
    {
        int nbProfiles = profiles.Length();
        int nbGuides = guides.Length();
        int nbPairs = nbProfiles * nbGuides;

        contactSets.Clear();
        for (int aPairIdx = 0; aPairIdx < nbPairs; ++aPairIdx)
        {
            contactSets.Append(new ContactSet());
        }

        int isDone = 1; // std::atomic<bool>
        OsdParallel.For(0, nbPairs, pairIdx =>
        {
            if (Volatile.Read(ref isDone) == 0)
            {
                return;
            }

            int profileIdx = pairIdx / nbGuides;
            int guideIdx = pairIdx % nbGuides;
            ref ContactSet contactSet = ref contactSets.ChangeValue(ContactIndex(profileIdx, guideIdx, nbGuides));
            contactSet = CollectContacts(profiles.At(profileIdx)!, guides.At(guideIdx)!, tolerance);
            if (contactSet.Candidates.IsEmpty())
            {
                Volatile.Write(ref isDone, 0);
            }
        }, !toUseParallel);

        return Volatile.Read(ref isDone) != 0
               && ChooseNearestContacts(contactSets, nbProfiles, nbGuides, profileParams, guideParams);
    }

    /// <summary>Snaps one parameter value to the curve boundary when it is close enough.</summary>
    internal static double SnappedParameter(BSplineCurve3D curve, double parameter, double tolerance)
    {
        double first = curve.FirstParameter;
        double last = curve.LastParameter;
        double tol = BoundaryParameterResolution(first, last, tolerance);
        if (Math.Abs(parameter - first) < tol)
        {
            return first;
        }
        if (Math.Abs(parameter - last) < tol)
        {
            return last;
        }
        return parameter;
    }

    /// <summary>Normalizes intersections after ordering and returns detected closed directions.</summary>
    internal static bool NormalizeContactTable(NCollection_Array1<BSplineCurve3D?> profiles,
                                               NCollection_Array1<BSplineCurve3D?> guides,
                                               NCollection_Array2<double> profileParams,
                                               NCollection_Array2<double> guideParams, double tolerance,
                                               ref bool isUClosed, ref bool isVClosed)
    {
        double tol = Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Boundary snaps are done before validation so closed seams use exact end parameters.
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
            {
                profileParams.ChangeAt(profileIdx, guideIdx) =
                  SnappedParameter(profiles.At(profileIdx)!, profileParams.At(profileIdx, guideIdx),
                                   tolerance);
                guideParams.ChangeAt(profileIdx, guideIdx) =
                  SnappedParameter(guides.At(guideIdx)!, guideParams.At(profileIdx, guideIdx), tolerance);
            }
        }

        double uGap = 0.0;
        double vGap = 0.0;
        int lastProfile = profiles.Size() - 1;
        int lastGuide = guides.Size() - 1;

        // One pass validates all contacts and measures only the two possible seam gaps.
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            CurveAdaptor profileAdaptor = new(profiles.At(profileIdx)!);
            for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
            {
                CurveAdaptor guideAdaptor = new(guides.At(guideIdx)!);
                Point3D profilePoint = profileAdaptor.EvalD0(profileParams.At(profileIdx, guideIdx));
                Point3D guidePoint = guideAdaptor.EvalD0(guideParams.At(profileIdx, guideIdx));
                if (profilePoint.DistanceTo(guidePoint) > tol)
                {
                    return false;
                }
                if (guideIdx == 0 || guideIdx == lastGuide)
                {
                    Point3D oppositePoint =
                      profileAdaptor.EvalD0(profileParams.At(profileIdx, lastGuide - guideIdx));
                    uGap = Max(uGap, profilePoint.DistanceTo(oppositePoint));
                }
                if (profileIdx == 0 || profileIdx == lastProfile)
                {
                    Point3D oppositePoint =
                      guideAdaptor.EvalD0(guideParams.At(lastProfile - profileIdx, guideIdx));
                    vGap = Max(vGap, guidePoint.DistanceTo(oppositePoint));
                }
            }
        }

        isUClosed = uGap <= tol;
        isVClosed = vGap <= tol;
        if (isUClosed)
        {
            for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
            {
                profileParams.ChangeAt(profileIdx, 0) = profiles.At(profileIdx)!.FirstParameter;
                profileParams.ChangeAt(profileIdx, lastGuide) = profiles.At(profileIdx)!.LastParameter;
            }
        }
        if (isVClosed)
        {
            for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
            {
                guideParams.ChangeAt(0, guideIdx) = guides.At(guideIdx)!.FirstParameter;
                guideParams.ChangeAt(lastProfile, guideIdx) = guides.At(guideIdx)!.LastParameter;
            }
        }
        return true;
    }

    /// <summary>Orders curves by their mean position in the opposite parameter family.</summary>
    internal static NCollection_LinearVector<OrderedCurve> OrderedCurves(math_Vector ranks)
    {
        NCollection_LinearVector<OrderedCurve> order = new();
        for (int rankIdx = ranks.Lower(); rankIdx <= ranks.Upper(); ++rankIdx)
        {
            order.Append(new OrderedCurve(rankIdx - ranks.Lower(), ranks[rankIdx]));
        }

        Sorting.Sort(order.AsSpan(), new OrderedCurveLess());
        double tol = ParameterResolution(0.0, 1.0);
        if (order.Size() > 2 && Math.Abs(order.Value(0).Rank - order.Value(1).Rank) <= tol)
        {
            NCollection_LinearVector<OrderedCurve> openedOrder = new();
            openedOrder.Append(order.Value(0));
            for (int orderIdx = 2; orderIdx < order.Size(); ++orderIdx)
            {
                openedOrder.Append(order.Value(orderIdx));
            }
            openedOrder.Append(order.Value(1));
            return openedOrder;
        }
        if (order.Size() > 2 && Math.Abs(order.Last().Rank - order.Value(order.Size() - 2).Rank) <= tol)
        {
            NCollection_LinearVector<OrderedCurve> openedOrder = new();
            openedOrder.Append(order.Value(order.Size() - 2));
            for (int orderIdx = 0; orderIdx + 2 < order.Size(); ++orderIdx)
            {
                openedOrder.Append(order.Value(orderIdx));
            }
            openedOrder.Append(order.Last());
            return openedOrder;
        }
        return order;
    }

    /// <summary>Checks that every row and column advances in its own curve parameter direction.</summary>
    internal static bool IsMonotoneLayout(Matrix profileParams, Matrix guideParams)
    {
        double tol = ParameterResolution(0.0, 1.0);

        // Ambiguous or folded grids are rejected instead of being repaired heuristically.
        for (int row = profileParams.LowerRow(); row <= profileParams.UpperRow(); ++row)
        {
            for (int col = profileParams.LowerCol() + 1; col <= profileParams.UpperCol(); ++col)
            {
                if (profileParams[row, col] <= profileParams[row, col - 1] + tol)
                {
                    return false;
                }
            }
        }

        for (int col = guideParams.LowerCol(); col <= guideParams.UpperCol(); ++col)
        {
            for (int row = guideParams.LowerRow() + 1; row <= guideParams.UpperRow(); ++row)
            {
                if (guideParams[row, col] <= guideParams[row - 1, col] + tol)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>Opens duplicated seam contacts to exact boundary parameters before monotone validation.</summary>
    internal static void OpenClosedBoundaryCandidates(NCollection_Array1<BSplineCurve3D?> profiles,
                                                      NCollection_Array1<BSplineCurve3D?> guides,
                                                      NCollection_Array2<double> profileParams,
                                                      NCollection_Array2<double> guideParams, double tolerance)
    {
        double tol = Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        int lastGuide = guides.Size() - 1;
        int lastProf = profiles.Size() - 1;
        bool isURepeated = true;
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            CurveAdaptor profileAdaptor = new(profiles.At(profileIdx)!);
            Point3D first = profileAdaptor.EvalD0(profileParams.At(profileIdx, 0));
            Point3D last = profileAdaptor.EvalD0(profileParams.At(profileIdx, lastGuide));
            if (first.DistanceTo(last) > tol)
            {
                isURepeated = false;
                break;
            }
        }
        if (isURepeated)
        {
            for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
            {
                profileParams.ChangeAt(profileIdx, 0) = profiles.At(profileIdx)!.FirstParameter;
                profileParams.ChangeAt(profileIdx, lastGuide) = profiles.At(profileIdx)!.LastParameter;
            }
        }

        bool isVRepeated = true;
        for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
        {
            CurveAdaptor guideAdaptor = new(guides.At(guideIdx)!);
            Point3D first = guideAdaptor.EvalD0(guideParams.At(0, guideIdx));
            Point3D last = guideAdaptor.EvalD0(guideParams.At(lastProf, guideIdx));
            if (first.DistanceTo(last) > tol)
            {
                isVRepeated = false;
                break;
            }
        }
        if (isVRepeated)
        {
            for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
            {
                guideParams.ChangeAt(0, guideIdx) = guides.At(guideIdx)!.FirstParameter;
                guideParams.ChangeAt(lastProf, guideIdx) = guides.At(guideIdx)!.LastParameter;
            }
        }
    }

    /// <summary>Returns a curve handle oriented as requested (always a copy).</summary>
    internal static BSplineCurve3D OrientedCurve(BSplineCurve3D curve, bool isForward)
    {
        BSplineCurve3D aCurve = (BSplineCurve3D)curve.Copy();
        if (!isForward)
        {
            aCurve.Reverse();
        }
        return aCurve;
    }

    /// <summary>Returns a parameter in the destination curve orientation.</summary>
    internal static double OrientedParameter(BSplineCurve3D curve, double parameter, bool isForward)
    {
        return isForward ? parameter : curve.FirstParameter + curve.LastParameter - parameter;
    }

    /// <summary>Scores an oriented contact against the expected grid position.</summary>
    internal static double OrderedContactScore(in Contact contact, double profileParam, double guideParam,
                                               double profileTarget, double guideTarget,
                                               double previousProfileParam, double previousGuideParam,
                                               double previousProfileTarget, double previousGuideTarget,
                                               bool hasPreviousProfile, bool hasPreviousGuide,
                                               double tolerance)
    {
        double score = contact.Gap / Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion())
                        + Math.Abs(profileParam - profileTarget)
                        + Math.Abs(guideParam - guideTarget);
        if (hasPreviousProfile)
        {
            // Favor branches with a smooth advance along the current profile.
            score += Math.Abs((profileParam - previousProfileParam)
                               - (profileTarget - previousProfileTarget));
        }
        if (hasPreviousGuide)
        {
            // Favor branches with a smooth advance along the current guide.
            score += Math.Abs((guideParam - previousGuideParam) - (guideTarget - previousGuideTarget));
        }
        return score;
    }

    /// <summary>Returns the tangent mismatch between two parameters in the requested curve orientation.</summary>
    internal static bool SeamTangentMismatch(BSplineCurve3D curve, double firstParameter,
                                             double lastParameter, bool isForward, double tolerance,
                                             ref double mismatch)
    {
        double firstRawParameter =
          isForward ? firstParameter : curve.FirstParameter + curve.LastParameter - firstParameter;
        double lastRawParameter =
          isForward ? lastParameter : curve.FirstParameter + curve.LastParameter - lastParameter;
        Curve3D.ResD1 firstD1 = curve.EvalD1(firstRawParameter);
        Curve3D.ResD1 lastD1 = curve.EvalD1(lastRawParameter);
        if (firstD1.Point.DistanceTo(lastD1.Point) > Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            return false;
        }

        Vector3D firstTangent = firstD1.D1;
        Vector3D lastTangent = lastD1.D1;
        if (!isForward)
        {
            firstTangent = firstTangent.Reversed();
            lastTangent = lastTangent.Reversed();
        }
        double firstMagnitude = firstTangent.Magnitude;
        double lastMagnitude = lastTangent.Magnitude;
        if (firstMagnitude <= Constants.Resolution() || lastMagnitude <= Constants.Resolution())
        {
            return false;
        }

        mismatch = 1.0 - Clamp(firstTangent.Dot(lastTangent) / (firstMagnitude * lastMagnitude), -1.0, 1.0);
        return true;
    }

    /// <summary>Selects one contact compatible with already selected row/column predecessors.</summary>
    internal static bool ChooseOrderedContact(ContactSet contacts, BSplineCurve3D profile,
                                              BSplineCurve3D guide, bool profileForward,
                                              bool guideForward, double profileTarget, double guideTarget,
                                              double previousProfileParam, double previousGuideParam,
                                              bool hasPreviousProfile, bool hasPreviousGuide,
                                              double previousProfileTarget, double previousGuideTarget,
                                              double firstProfileParam, double firstGuideParam,
                                              bool isProfileSeam, bool isGuideSeam, double tolerance,
                                              ref Contact selected)
    {
        double paramTol = ParameterResolution(0.0, 1.0);
        double bestTangentMismatch = RealLast();
        double best = RealLast();
        for (int candidateIdx = 0; candidateIdx < contacts.Candidates.Size(); ++candidateIdx)
        {
            Contact candidate = contacts.Candidates.Value(candidateIdx);
            double profileParam = OrientedParameter(profile, candidate.ProfileParam, profileForward);
            double guideParam = OrientedParameter(guide, candidate.GuideParam, guideForward);
            if ((hasPreviousProfile && profileParam <= previousProfileParam + paramTol)
                || (hasPreviousGuide && guideParam <= previousGuideParam + paramTol))
            {
                continue;
            }

            double score = OrderedContactScore(candidate, profileParam, guideParam, profileTarget,
                                                guideTarget, previousProfileParam, previousGuideParam,
                                                previousProfileTarget, previousGuideTarget,
                                                hasPreviousProfile, hasPreviousGuide, tolerance);
            double tangentMismatch = 0.0;
            bool hasTangentMatch = false;
            if (isProfileSeam)
            {
                double profileMismatch = 0.0;
                if (SeamTangentMismatch(profile, firstProfileParam, profileParam, profileForward,
                                        tolerance, ref profileMismatch))
                {
                    tangentMismatch += profileMismatch;
                    hasTangentMatch = true;
                }
            }
            if (isGuideSeam)
            {
                double guideMismatch = 0.0;
                if (SeamTangentMismatch(guide, firstGuideParam, guideParam, guideForward, tolerance,
                                        ref guideMismatch))
                {
                    tangentMismatch += guideMismatch;
                    hasTangentMatch = true;
                }
            }

            if ((hasTangentMatch
                 && (tangentMismatch < bestTangentMismatch - global::Kerf.Engine.Foundation.Precision.Angular()
                     || (Math.Abs(tangentMismatch - bestTangentMismatch) <= global::Kerf.Engine.Foundation.Precision.Angular() && score < best)))
                || (!hasTangentMatch && bestTangentMismatch == RealLast() && score < best))
            {
                bestTangentMismatch = hasTangentMatch ? tangentMismatch : RealLast();
                best = score;
                selected = candidate;
                selected.ProfileParam = profileParam;
                selected.GuideParam = guideParam;
            }
        }
        return selected.IsValid;
    }

    /// <summary>Reorders curves and contact matrices into a consistent network layout.</summary>
    internal static bool ReorderNetwork(NCollection_Array1<BSplineCurve3D?> profiles,
                                        NCollection_Array1<BSplineCurve3D?> guides,
                                        NCollection_Array2<double> profileParams,
                                        NCollection_Array2<double> guideParams,
                                        NCollection_LinearVector<ContactSet> contactSets, double tolerance)
    {
        if (profiles.Size() < 2 || guides.Size() < 2)
        {
            return false;
        }

        Matrix profileParamMatrix = ToMatrix(profileParams);
        Matrix guideParamMatrix = ToMatrix(guideParams);

        NCollection_LinearVector<OrderedCurve> profileOrder = OrderedCurves(RowMeans(guideParamMatrix));
        NCollection_LinearVector<OrderedCurve> guideOrder = OrderedCurves(ColumnMeans(profileParamMatrix));

        NCollection_Array1<BSplineCurve3D?> aProfiles = new(1, profiles.Size());
        NCollection_Array1<BSplineCurve3D?> aGuides = new(1, guides.Size());
        NCollection_Array2<double> aProfileParams = new(profileParams.LowerRow(), profileParams.UpperRow(),
                                                        profileParams.LowerCol(), profileParams.UpperCol());
        NCollection_Array2<double> aGuideParams = new(guideParams.LowerRow(), guideParams.UpperRow(),
                                                      guideParams.LowerCol(), guideParams.UpperCol());
        Matrix profileParamResult = ToMatrix(aProfileParams);
        Matrix guideParamResult = ToMatrix(aGuideParams);

        NCollection_LinearVector<bool> profileForward = new();
        for (int profileIdx = 0; profileIdx < profileOrder.Size(); ++profileIdx)
        {
            int srcProfileIdx = profileOrder.Value(profileIdx).Index;
            bool isForward = profileParamMatrix[srcProfileIdx + 1, guideOrder.First().Index + 1]
                             <= profileParamMatrix[srcProfileIdx + 1, guideOrder.Last().Index + 1];
            aProfiles.ChangeAt(profileIdx) = OrientedCurve(profiles.At(srcProfileIdx)!, isForward);
            profileForward.Append(isForward);
        }

        NCollection_LinearVector<bool> guideForward = new();
        for (int guideIdx = 0; guideIdx < guideOrder.Size(); ++guideIdx)
        {
            int srcGuideIdx = guideOrder.Value(guideIdx).Index;
            bool isForward = guideParamMatrix[profileOrder.First().Index + 1, srcGuideIdx + 1]
                             <= guideParamMatrix[profileOrder.Last().Index + 1, srcGuideIdx + 1];
            aGuides.ChangeAt(guideIdx) = OrientedCurve(guides.At(srcGuideIdx)!, isForward);
            guideForward.Append(isForward);
        }

        for (int profileIdx = 0; profileIdx < profileOrder.Size(); ++profileIdx)
        {
            int srcProfileIdx = profileOrder.Value(profileIdx).Index;
            for (int guideIdx = 0; guideIdx < guideOrder.Size(); ++guideIdx)
            {
                int srcGuideIdx = guideOrder.Value(guideIdx).Index;
                double profileTarget = guideOrder.Size() == 1 ? 0.0 : (double)guideIdx / (guideOrder.Size() - 1);
                double guideTarget =
                  profileOrder.Size() == 1 ? 0.0 : (double)profileIdx / (profileOrder.Size() - 1);
                double previousProfileParam = guideIdx == 0 ? 0.0 : profileParamResult[profileIdx + 1, guideIdx];
                double previousGuideParam = profileIdx == 0 ? 0.0 : guideParamResult[profileIdx, guideIdx + 1];
                double previousProfileTarget =
                  guideIdx < 2 ? 0.0 : (double)(guideIdx - 1) / (guideOrder.Size() - 1);
                double previousGuideTarget =
                  profileIdx < 2 ? 0.0 : (double)(profileIdx - 1) / (profileOrder.Size() - 1);
                bool isProfileSeam = guideIdx + 1 == guideOrder.Size();
                bool isGuideSeam = profileIdx + 1 == profileOrder.Size();
                double firstProfileParam = isProfileSeam ? profileParamResult[profileIdx + 1, 1] : 0.0;
                double firstGuideParam = isGuideSeam ? guideParamResult[1, guideIdx + 1] : 0.0;

                Contact contact = new();
                if (!ChooseOrderedContact(contactSets.Value(ContactIndex(srcProfileIdx, srcGuideIdx, guides.Size())),
                                          profiles.At(srcProfileIdx)!, guides.At(srcGuideIdx)!,
                                          profileForward.Value(profileIdx), guideForward.Value(guideIdx),
                                          profileTarget, guideTarget, previousProfileParam, previousGuideParam,
                                          guideIdx != 0, profileIdx != 0, previousProfileTarget,
                                          previousGuideTarget, firstProfileParam, firstGuideParam, isProfileSeam,
                                          isGuideSeam, tolerance, ref contact))
                {
                    return false;
                }

                profileParamResult[profileIdx + 1, guideIdx + 1] = contact.ProfileParam;
                guideParamResult[profileIdx + 1, guideIdx + 1] = contact.GuideParam;
            }
        }

        OpenClosedBoundaryCandidates(aProfiles, aGuides, aProfileParams, aGuideParams, tolerance);

        if (!IsMonotoneLayout(profileParamResult, guideParamResult))
        {
            return false;
        }

        profiles.Assign(aProfiles);
        guides.Assign(aGuides);
        profileParams.Assign(aProfileParams);
        guideParams.Assign(aGuideParams);
        return true;
    }

    /// <summary>Monotone scalar map converting target parameters back to the source curve.</summary>
    internal sealed class ScalarLaw : EvaluatorFunction
    {
        private readonly NCollection_LinearVector<double> _source = new();
        private readonly NCollection_LinearVector<double> _target = new();

        /// <summary>Stores a strictly increasing piecewise-linear parameter map.</summary>
        public bool Init(math_Vector source, math_Vector target, int nbValues)
        {
            if (nbValues < 2)
            {
                return false;
            }

            _source.Clear();
            _target.Clear();
            for (int paramIdx = 1; paramIdx <= nbValues; ++paramIdx)
            {
                if (!_source.IsEmpty()
                    && (source[paramIdx] <= _source.Last() + global::Kerf.Engine.Foundation.Precision.PConfusion()
                        || target[paramIdx] <= _target.Last() + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    return false;
                }
                _source.Append(source[paramIdx]);
                _target.Append(target[paramIdx]);
            }
            return true;
        }

        /// <summary>Evaluates the map by linear interpolation and clamps to source bounds.</summary>
        public double Value(double targetParameter)
        {
            if (_source.IsEmpty())
            {
                return 0.0;
            }
            if (targetParameter <= _target.First())
            {
                return _source.First();
            }
            if (targetParameter >= _target.Last())
            {
                return _source.Last();
            }

            for (int paramIdx = 1; paramIdx < _target.Size(); ++paramIdx)
            {
                if (targetParameter <= _target.Value(paramIdx))
                {
                    double t0 = _target.Value(paramIdx - 1);
                    double t1 = _target.Value(paramIdx);
                    double weight = (targetParameter - t0) / (t1 - t0);
                    return _source.Value(paramIdx - 1)
                           + weight * (_source.Value(paramIdx) - _source.Value(paramIdx - 1));
                }
            }
            return _source.Last();
        }

        public override void Evaluate(int derivativeRequest, ReadOnlySpan<double> startEnd,
                                      double targetParameter, out double result, out int errorCode)
        {
            errorCode = 0;
            if (derivativeRequest == 0)
            {
                result = Value(targetParameter);
                return;
            }
            if (derivativeRequest != 1)
            {
                result = 0.0;
                return;
            }

            for (int paramIdx = 1; paramIdx < _target.Size(); ++paramIdx)
            {
                if (targetParameter <= _target.Value(paramIdx))
                {
                    result = (_source.Value(paramIdx) - _source.Value(paramIdx - 1))
                                / (_target.Value(paramIdx) - _target.Value(paramIdx - 1));
                    return;
                }
            }
            result = (_source.Last() - _source.Value(_source.Size() - 2))
                        / (_target.Last() - _target.Value(_target.Size() - 2));
        }
    }

    /// <summary>Reparametrizes a polynomial B-spline exactly using a piecewise-linear parameter law.</summary>
    internal static bool ReparametrizeExactly(BSplineCurve3D curve, math_Vector sourceParams,
                                              math_Vector targetParams, int nbConstraints, ScalarLaw law,
                                              ref BSplineCurve3D? result)
    {
        if (nbConstraints < 2)
        {
            return false;
        }

        int degree = curve.Degree;
        NCollection_LinearVector<double> newKnotValues = new();
        NCollection_LinearVector<int> newKnotMults = new();
        int constraintIdx = 1;
        NCollection_Array1<double> curveKnots = curve.InternalKnots();
        NCollection_Array1<int> curveMults = curve.InternalMultiplicities();
        for (int knotIdx = 0; knotIdx < curveKnots.Size(); ++knotIdx)
        {
            double sourceKnot = curveKnots.At(knotIdx);
            while (constraintIdx <= nbConstraints
                   && sourceParams[constraintIdx] < sourceKnot - global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                newKnotValues.Append(targetParams[constraintIdx]);
                newKnotMults.Append(degree);
                ++constraintIdx;
            }

            double targetKnot = targetParams[nbConstraints];
            for (int paramIdx = 1; paramIdx < nbConstraints; ++paramIdx)
            {
                if (sourceKnot <= sourceParams[paramIdx + 1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    double sourceFirst = sourceParams[paramIdx];
                    double sourceLast = sourceParams[paramIdx + 1];
                    double targetFirst = targetParams[paramIdx];
                    double targetLast = targetParams[paramIdx + 1];
                    targetKnot = targetFirst
                                  + (sourceKnot - sourceFirst) * (targetLast - targetFirst)
                                    / (sourceLast - sourceFirst);
                    break;
                }
            }

            int multiplicity = curveMults.At(knotIdx);
            if (constraintIdx <= nbConstraints
                && Math.Abs(sourceParams[constraintIdx] - sourceKnot) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (constraintIdx > 1 && constraintIdx < nbConstraints)
                {
                    multiplicity = Max(multiplicity, degree);
                }
                ++constraintIdx;
            }
            newKnotValues.Append(targetKnot);
            newKnotMults.Append(multiplicity);
        }

        NCollection_Array1<double> newKnots = ToOneBasedArrayView(newKnotValues);
        NCollection_Array1<int> newMults = ToOneBasedArrayView(newKnotMults);
        int nbNewPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(degree, false, newMults);
        NCollection_Array1<double> sourceFlatKnots = new(1, curve.PoleCount + degree + 1);
        NCollection_Array1<double> targetFlatKnots = new(1, nbNewPoles + degree + 1);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(curveKnots, curveMults, sourceFlatKnots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newKnots, newMults, targetFlatKnots);

        int status;
        if (!curve.IsRational)
        {
            NCollection_Array1<Point3D> newPoles = new(1, nbNewPoles);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(law, degree, sourceFlatKnots, curve.InternalPoles(),
                                            targetFlatKnots, degree, newPoles, out status);
            if (status != 0)
            {
                return false;
            }
            result = new BSplineCurve3D(newPoles, newKnots, newMults, degree);
            return result is not null;
        }

        NCollection_Array1<Point3D> curvePoles = curve.InternalPoles();
        NCollection_Array1<double> curveWeights = curve.InternalWeightsArray();
        NCollection_Array1<double> homogeneousPoles = new(1, 4 * curve.PoleCount);
        for (int poleIdx = 0; poleIdx < curvePoles.Size(); ++poleIdx)
        {
            Point3D pole = curvePoles.At(poleIdx);
            double weight = curveWeights.At(poleIdx);
            int homogeneousIdx = 4 * poleIdx;
            homogeneousPoles.ChangeAt(homogeneousIdx) = pole.X * weight;
            homogeneousPoles.ChangeAt(homogeneousIdx + 1) = pole.Y * weight;
            homogeneousPoles.ChangeAt(homogeneousIdx + 2) = pole.Z * weight;
            homogeneousPoles.ChangeAt(homogeneousIdx + 3) = weight;
        }

        NCollection_Array1<double> newHomogeneousPoles = new(1, 4 * nbNewPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(law, degree, sourceFlatKnots, 4, homogeneousPoles.AsSpan(),
                                        targetFlatKnots, degree, newHomogeneousPoles.AsSpan(), out status);
        if (status != 0)
        {
            return false;
        }

        NCollection_Array1<Point3D> newPoles2 = new(1, nbNewPoles);
        NCollection_Array1<double> newWeights = new(1, nbNewPoles);
        for (int poleIdx = 0; poleIdx < newPoles2.Size(); ++poleIdx)
        {
            int homogeneousIdx = 4 * poleIdx;
            double weight = newHomogeneousPoles.At(homogeneousIdx + 3);
            if (weight <= Constants.Resolution())
            {
                return false;
            }
            newPoles2.ChangeAt(poleIdx) = newPoles2.ChangeAt(poleIdx).WithCoord(newHomogeneousPoles.At(homogeneousIdx) / weight,
                                                   newHomogeneousPoles.At(homogeneousIdx + 1) / weight,
                                                   newHomogeneousPoles.At(homogeneousIdx + 2) / weight);
            newWeights.ChangeAt(poleIdx) = weight;
        }

        result = new BSplineCurve3D(newPoles2, newWeights, newKnots, newMults, degree);
        return result is not null;
    }

    /// <summary>Appends a monotone source/target pair.</summary>
    internal static void PushConstraint(math_Vector source, math_Vector target, ref int nbConstraints,
                                        double sourceParam, double targetParam, double sourceTol,
                                        double targetTol)
    {
        if (nbConstraints > 0
            && (Math.Abs(sourceParam - source[nbConstraints]) <= sourceTol
                || Math.Abs(targetParam - target[nbConstraints]) <= targetTol))
        {
            return;
        }

        ++nbConstraints;
        source[nbConstraints] = sourceParam;
        target[nbConstraints] = Max(0.0, Min(1.0, targetParam));
    }

    /// <summary>Builds target-space samples from constraints and deterministic interval subdivision.</summary>
    internal static NCollection_LinearVector<double> SamplingParameters(math_Vector targets, int nbSourcePoles)
    {
        NCollection_LinearVector<double> samples = new();
        AppendUnitParameter(samples, 0.0);
        AppendUnitParameter(samples, 1.0);
        for (int paramIdx = targets.Lower(); paramIdx <= targets.Upper(); ++paramIdx)
        {
            AppendUnitParameter(samples, targets[paramIdx]);
        }

        CompactParameters(samples);

        // The rebuilt curve is parameter-driven; extra samples are deterministic.
        AddIntervalMidpoints(samples,
                             Max(InterpolationDegree() + 1, Max(nbSourcePoles * 2, targets.Length() + 2)));
        return samples;
    }

    /// <summary>Rebuilds one curve through target-parameter samples.</summary>
    internal static bool RebuildCurve(ref BSplineCurve3D? curve, math_Vector sourceParams,
                                      math_Vector targetParams, bool allowApproximateRational,
                                      ref bool didApproximate, ref double maxDeviation)
    {
        BSplineCurve3D aCurve = curve!;
        math_Vector source = new(1, sourceParams.Length() + 2);
        math_Vector target = new(1, targetParams.Length() + 2);
        int nbConstraints = 0;
        double sourceTol = ParameterResolution(aCurve.FirstParameter, aCurve.LastParameter);
        double targetTol = ParameterResolution(0.0, 1.0);

        PushConstraint(source, target, ref nbConstraints, aCurve.FirstParameter, 0.0, sourceTol, targetTol);
        for (int paramIdx = sourceParams.Lower(); paramIdx <= sourceParams.Upper(); ++paramIdx)
        {
            double sourceParam = sourceParams[paramIdx];
            double targetParam = targetParams[paramIdx];
            if (Math.Abs(sourceParam - aCurve.FirstParameter) <= sourceTol
                || Math.Abs(sourceParam - aCurve.LastParameter) <= sourceTol
                || Math.Abs(targetParam) <= targetTol || Math.Abs(targetParam - 1.0) <= targetTol)
            {
                continue;
            }
            PushConstraint(source, target, ref nbConstraints, sourceParam, targetParam, sourceTol, targetTol);
        }
        PushConstraint(source, target, ref nbConstraints, aCurve.LastParameter, 1.0, sourceTol, targetTol);

        ScalarLaw law = new();
        if (!law.Init(source, target, nbConstraints))
        {
            return false;
        }

        BSplineCurve3D? exactCurve = null;
        math_Vector exactSource = new(1, nbConstraints);
        math_Vector exactTarget = new(1, nbConstraints);
        for (int constraintIdx = 1; constraintIdx <= nbConstraints; ++constraintIdx)
        {
            exactSource[constraintIdx] = source[constraintIdx];
            exactTarget[constraintIdx] = target[constraintIdx];
        }
        if (ReparametrizeExactly(aCurve, exactSource, exactTarget, nbConstraints, law, ref exactCurve))
        {
            curve = exactCurve;
            return true;
        }

        if (aCurve.IsRational)
        {
            if (!allowApproximateRational)
            {
                return false;
            }
            didApproximate = true;
        }

        CurveAdaptor curveAdaptor = new(aCurve);
        NCollection_LinearVector<double> samples = SamplingParameters(targetParams, aCurve.PoleCount);
        NCollection_LinearVector<Point3D> points = new();
        NCollection_LinearVector<double> parameters = new();
        for (int sampleIdx = 0; sampleIdx < samples.Size(); ++sampleIdx)
        {
            double targetParam = samples.Value(sampleIdx);
            Point3D point = curveAdaptor.EvalD0(law.Value(targetParam));
            if (!points.IsEmpty() && point.DistanceTo(points.Last()) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            points.Append(point);
            parameters.Append(targetParam);
        }

        int nbPoints = points.Size();
        if (nbPoints < InterpolationDegree())
        {
            return false;
        }

        NCollection_Array1<Point3D> pointArray = ToOneBasedArrayView(points);
        NCollection_Array1<double> paramArray = ToOneBasedArrayView(parameters);

        Interpolate interpolator = new(InterpolationDegree(), nbPoints, pointArray, paramArray);
        if (!interpolator.IsDone)
        {
            return false;
        }

        // GeomLib_Interpolate freezes the curve it hands out; the prepared network only reads it afterwards.
        curve = interpolator.Curve();
        if (curve is null)
        {
            return false;
        }

        CurveAdaptor newCurveAdaptor = new(curve);
        for (int sampleIdx = 0; sampleIdx < samples.Size(); ++sampleIdx)
        {
            double targetParam = samples.Value(sampleIdx);
            Point3D sourcePoint = curveAdaptor.EvalD0(law.Value(targetParam));
            Point3D newPoint = newCurveAdaptor.EvalD0(targetParam);
            maxDeviation = Max(maxDeviation, sourcePoint.DistanceTo(newPoint));
        }
        return curve is not null;
    }

    /// <summary>Reconstructs curves so all intersections share averaged parameters.</summary>
    internal static bool EqualizeParameters(NCollection_Array1<BSplineCurve3D?> profiles,
                                            NCollection_Array1<BSplineCurve3D?> guides,
                                            NCollection_Array2<double> profileParams,
                                            NCollection_Array2<double> guideParams,
                                            bool allowApproximateRational, ref bool didApproximate,
                                            ref double maxDeviation)
    {
        Matrix aProfileParams = ToMatrix(profileParams);
        Matrix aGuideParams = ToMatrix(guideParams);

        math_Vector profileTargets = ColumnMeans(aProfileParams);
        math_Vector guideTargets = RowMeans(aGuideParams);

        math_Vector oldProfileParams = new(1, guides.Size());
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            int profileRow = profileIdx + 1;
            bool isAlreadyTarget = true;
            for (int col = oldProfileParams.Lower(); col <= oldProfileParams.Upper(); ++col)
            {
                oldProfileParams[col] = aProfileParams[profileRow, col];
                isAlreadyTarget = isAlreadyTarget
                                  && Math.Abs(oldProfileParams[col] - profileTargets[col]) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            if (isAlreadyTarget)
            {
                continue;
            }
            if (!RebuildCurve(ref profiles.ChangeAt(profileIdx), oldProfileParams, profileTargets,
                              allowApproximateRational, ref didApproximate, ref maxDeviation))
            {
                return false;
            }
            for (int col = profileTargets.Lower(); col <= profileTargets.Upper(); ++col)
            {
                aProfileParams[profileRow, col] = profileTargets[col];
            }
        }

        math_Vector oldGuideParams = new(1, profiles.Size());
        for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
        {
            int guideCol = guideIdx + 1;
            bool isAlreadyTarget = true;
            for (int row = oldGuideParams.Lower(); row <= oldGuideParams.Upper(); ++row)
            {
                oldGuideParams[row] = aGuideParams[row, guideCol];
                isAlreadyTarget = isAlreadyTarget
                                  && Math.Abs(oldGuideParams[row] - guideTargets[row]) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
            }
            if (isAlreadyTarget)
            {
                continue;
            }
            if (!RebuildCurve(ref guides.ChangeAt(guideIdx), oldGuideParams, guideTargets,
                              allowApproximateRational, ref didApproximate, ref maxDeviation))
            {
                return false;
            }
            for (int row = guideTargets.Lower(); row <= guideTargets.Upper(); ++row)
            {
                aGuideParams[row, guideCol] = guideTargets[row];
            }
        }
        return true;
    }

    /// <summary>Evaluates the validated profile parameters into a guide/profile point grid.</summary>
    internal static NCollection_Array2<Point3D> ContactPointGrid(NCollection_Array1<BSplineCurve3D?> profiles,
                                                                NCollection_Array2<double> profileParams)
    {
        NCollection_Array2<Point3D> points = new(1, profileParams.RowLength(), 1, profileParams.ColLength());
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            CurveAdaptor profileAdaptor = new(profiles.At(profileIdx)!);
            for (int guideIdx = 0; guideIdx < profileParams.RowLength(); ++guideIdx)
            {
                points.ChangeAt(guideIdx, profileIdx) =
                  profileAdaptor.EvalD0(profileParams.At(profileIdx, guideIdx));
            }
        }
        return points;
    }

    /// <summary>Evaluates the denominator of a rational B-spline curve at a contact parameter.</summary>
    internal static double CurveWeightAt(BSplineCurve3D curve, double parameter)
    {
        if (!curve.IsRational)
        {
            return 1.0;
        }

        ReadOnlySpan<int> extrapMode = [curve.Degree, curve.Degree];
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(parameter, curve.IsPeriodic, false, extrapMode, curve.Degree,
                      curve.InternalKnotSequence(), curve.InternalPoles(), curve.InternalWeightsArray(),
                      out Point3D _, out double weight);
        return weight;
    }

    /// <summary>Evaluates rational contact weights in guide/profile grid order.</summary>
    internal static NCollection_Array2<double> ContactWeightGrid(NCollection_Array1<BSplineCurve3D?> profiles,
                                                                 NCollection_Array2<double> profileParams)
    {
        NCollection_Array2<double> weights = new(1, profileParams.RowLength(), 1, profileParams.ColLength());
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            for (int guideIdx = 0; guideIdx < profileParams.RowLength(); ++guideIdx)
            {
                weights.ChangeAt(guideIdx, profileIdx) =
                  CurveWeightAt(profiles.At(profileIdx)!, profileParams.At(profileIdx, guideIdx));
            }
        }
        return weights;
    }

    /// <summary>
    /// Performs the preparatory network operations required before Gordon surface construction, on the arrays
    /// GeomFill_Gordon owns (kept by reference; changed in place, as OCCT's references).
    /// </summary>
    internal sealed class NetworkPreparation
    {
        private readonly NCollection_Array1<BSplineCurve3D?> _profiles;
        private readonly NCollection_Array1<BSplineCurve3D?> _guides;
        private readonly NCollection_Array2<double> _profileParams;
        private readonly NCollection_Array2<double> _guideParams;
        private readonly NCollection_LinearVector<ContactSet> _contactSets = new();
        private readonly double _tolerance;
        private double _maxContactGap = 0.0;
        private double _maxReparametrizationDeviation = 0.0;
        private readonly bool _toUseParallel;

        public NetworkPreparation(NCollection_Array1<BSplineCurve3D?> profiles,
                                  NCollection_Array1<BSplineCurve3D?> guides,
                                  NCollection_Array2<double> profileParams,
                                  NCollection_Array2<double> guideParams, double tolerance,
                                  bool toUseParallel)
        {
            _profiles = profiles;
            _guides = guides;
            _profileParams = profileParams;
            _guideParams = guideParams;
            _tolerance = tolerance;
            _toUseParallel = toUseParallel;
        }

        /// <summary>Converts all curves to explicit unit-domain B-splines.</summary>
        public bool ConvertCurvesToUnitInterval()
        {
            return MakeUnitCurves(_profiles) && MakeUnitCurves(_guides);
        }

        /// <summary>Computes the profile/guide intersection parameter tables.</summary>
        public bool FillIntersectionParameters()
        {
            bool isDone = FillContactTable(_profiles, _guides, _profileParams, _guideParams, _contactSets,
                                           _tolerance, _toUseParallel);
            _maxContactGap = isDone ? MaxNearestContactGap(_contactSets) : 0.0;
            return isDone;
        }

        /// <summary>Reorders and orients the curve network.</summary>
        public bool ReorderNetwork()
        {
            return global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ReorderNetwork(_profiles, _guides, _profileParams, _guideParams, _contactSets, _tolerance);
        }

        /// <summary>Snaps, validates, and detects closed directions in the intersection table.</summary>
        public bool NormalizeIntersectionParameters(ref bool isUClosed, ref bool isVClosed)
        {
            return NormalizeContactTable(_profiles, _guides, _profileParams, _guideParams, _tolerance,
                                         ref isUClosed, ref isVClosed);
        }

        /// <summary>
        /// Rebuilds curves to shared averaged network parameters. Rational curves are rebuilt only when approximate
        /// fallback is allowed.
        /// </summary>
        public bool EqualizeIntersectionParameters(bool allowApproximateRational, ref bool didApproximate)
        {
            _maxReparametrizationDeviation = 0.0;
            return EqualizeParameters(_profiles, _guides, _profileParams, _guideParams,
                                      allowApproximateRational, ref didApproximate,
                                      ref _maxReparametrizationDeviation);
        }

        /// <summary>Returns the largest best-contact gap found during contact discovery.</summary>
        public double MaxContactGap() => _maxContactGap;

        /// <summary>Returns the largest measured curve rebuild deviation.</summary>
        public double MaxReparametrizationDeviation() => _maxReparametrizationDeviation;

        /// <summary>Returns the validated contact points in guide/profile grid order.</summary>
        public NCollection_Array2<Point3D> IntersectionPoints() => ContactPointGrid(_profiles, _profileParams);

        /// <summary>Returns rational contact weights in guide/profile grid order.</summary>
        public NCollection_Array2<double> IntersectionWeights() => ContactWeightGrid(_profiles, _profileParams);
    }
}
