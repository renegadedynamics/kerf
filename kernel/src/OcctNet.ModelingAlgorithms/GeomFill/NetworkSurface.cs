// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_NetworkSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// Low-level Gordon surface construction from a compatible B-spline curve network (explicit, non-periodic,
/// consistently ordered and reparametrised curves): a profile skin, a guide skin and an intersection-grid reference
/// surface, aligned to one knot basis; the result is the profile skin plus the guide skin minus the reference.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> copies the arrays and keeps the caller's curves, which it never
/// changes: polynomial families are made compatible through a <see cref="Profiler"/>, which works on copies.
/// <see cref="Surface"/> hands out the surface frozen.
/// </remarks>
public class NetworkSurface
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

    private NCollection_Array1<BSplineCurve3D?> _profiles = new();
    private NCollection_Array1<BSplineCurve3D?> _guides = new();
    private NCollection_Array1<double> _profileParameters = new();
    private NCollection_Array1<double> _guideParameters = new();
    private NCollection_Array2<Point3D> _intersectionPoints = new();
    private NCollection_Array2<double> _intersectionWeights = new();
    private BSplineSurface? _surface;
    private double _tolerance = 0.0;
    private bool _isUClosed = false;
    private bool _isVClosed = false;
    private ResultStatus _status = ResultStatus.NotStarted;

    private static bool CheckParameters(NCollection_Array1<double> parameters)
    {
        if (parameters.Size() < 2)
        {
            return false;
        }
        for (int paramIdx = 1; paramIdx < parameters.Size(); ++paramIdx)
        {
            if (parameters.At(paramIdx) <= parameters.At(paramIdx - 1) + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        return true;
    }

    private static bool CheckWeights(NCollection_Array2<double> weights)
    {
        if (weights.IsEmpty())
        {
            return false;
        }
        foreach (double weight in weights.AsSpan())
        {
            if (weight <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    private static void ReparametrizeSurface(BSplineSurface surface)
    {
        NCollection_Array1<double> uKnots = new(surface.InternalUKnots());
        NCollection_Array1<double> vKnots = new(surface.InternalVKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, uKnots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, vKnots);
        surface.SetUKnots(uKnots);
        surface.SetVKnots(vKnots);
    }

    private static bool HasRationalCurve(NCollection_Array1<BSplineCurve3D?> curves)
    {
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            if (curves.At(curveIdx) is not null && curves.At(curveIdx)!.IsRational)
            {
                return true;
            }
        }
        return false;
    }

    private static bool HasPeriodicCurve(NCollection_Array1<BSplineCurve3D?> curves)
    {
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            if (curves.At(curveIdx) is null || curves.At(curveIdx)!.IsPeriodic)
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsCompatibleCurveFamily(NCollection_Array1<BSplineCurve3D?> curves)
    {
        if (curves.IsEmpty() || curves.First() is null)
        {
            return false;
        }

        BSplineCurve3D reference = curves.First()!;
        bool isRational = reference.IsRational;
        for (int curveIdx = 1; curveIdx < curves.Size(); ++curveIdx)
        {
            BSplineCurve3D? curve = curves.At(curveIdx);
            if (curve is null || curve.IsRational != isRational || curve.Degree() != reference.Degree()
                || curve.PoleCount != reference.PoleCount || curve.KnotCount != reference.KnotCount)
            {
                return false;
            }
            for (int knotIdx = 0; knotIdx < curve.KnotCount; ++knotIdx)
            {
                int knot = knotIdx + 1;
                if (curve.Multiplicity(knot) != reference.Multiplicity(knot)
                    || Math.Abs(curve.Knot(knot) - reference.Knot(knot)) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static bool PrepareCurveFamily(NCollection_Array1<BSplineCurve3D?> curves)
    {
        if (HasRationalCurve(curves))
        {
            return IsCompatibleCurveFamily(curves);
        }

        Profiler profiler = new();
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            if (curves.At(curveIdx) is null)
            {
                return false;
            }
            profiler.AddCurve(curves.At(curveIdx)!);
        }

        profiler.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            curves.ChangeAt(curveIdx) = profiler.InternalCurve(curveIdx + 1) as BSplineCurve3D;
            if (curves.At(curveIdx) is null)
            {
                return false;
            }
        }

        return true;
    }

    private static void MakeInterpolationBasis(NCollection_Array1<double> parameters, out int degree,
                                               NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                               NCollection_Array1<double> flatKnots)
    {
        int nbParams = parameters.Size();
        degree = Min(3, nbParams - 1);

        int nbInternalKnots = nbParams - degree - 1;
        knots.Resize(1, nbInternalKnots + 2, false);
        mults.Resize(1, nbInternalKnots + 2, false);

        knots.ChangeAt(0) = parameters.At(0);
        mults.ChangeAt(0) = degree + 1;
        for (int knotIdx = 0; knotIdx < nbInternalKnots; ++knotIdx)
        {
            double sum = 0.0;
            for (int paramOffset = 1; paramOffset <= degree; ++paramOffset)
            {
                sum += parameters.At(knotIdx + paramOffset);
            }
            knots.ChangeAt(knotIdx + 1) = sum / (double)degree;
            mults.ChangeAt(knotIdx + 1) = 1;
        }
        knots.ChangeAt(knots.Size() - 1) = parameters.At(parameters.Size() - 1);
        mults.ChangeAt(mults.Size() - 1) = degree + 1;

        flatKnots.Resize(1, degree + nbParams + 1, false);
        int flatIdx = 0;
        for (int knotIdx = 0; knotIdx < knots.Size(); ++knotIdx)
        {
            for (int multIdx = 0; multIdx < mults.At(knotIdx); ++multIdx)
            {
                flatKnots.ChangeAt(flatIdx++) = knots.At(knotIdx);
            }
        }
    }

    private static bool InterpolatePoles(int degree, NCollection_Array1<double> flatKnots,
                                         NCollection_Array1<double> parameters,
                                         NCollection_Array1<int> contactOrders, NCollection_Array1<Point3D> poles,
                                         NCollection_Array1<double> weights)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatKnots, parameters, contactOrders, poles, weights,
                             out int inversionProblem);
        return inversionProblem == 0;
    }

    private static double PoleWeight(BSplineCurve3D curve, int poleIdx)
    {
        return curve.IsRational ? curve.Weight(poleIdx) : 1.0;
    }

    private static bool SameKnotRange(NCollection_Array1<double> knots1, NCollection_Array1<double> knots2,
                                      out double start, out double end)
    {
        start = knots1.First();
        end = knots1.Last();
        return Math.Abs(start - knots2.First()) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
               && Math.Abs(end - knots2.Last()) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
               && start < end - global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Candidate knot with multiplicity collected before tolerance compaction.</summary>
    private struct KnotEntry
    {
        public double Parameter;
        public int Multiplicity;
    }

    private readonly struct KnotEntryLess : Heap.ILess<KnotEntry>
    {
        public bool Less(KnotEntry left, KnotEntry right) => left.Parameter < right.Parameter;
    }

    /// <summary>Interpolation basis used to skin one curve family.</summary>
    private sealed class SkinningBasis
    {
        public int Degree = 0;
        public NCollection_Array1<double> Knots = new();
        public NCollection_Array1<int> Mults = new();
        public NCollection_Array1<double> FlatKnots = new();

        public void Init(NCollection_Array1<double> parameters)
        {
            MakeInterpolationBasis(parameters, out Degree, Knots, Mults, FlatKnots);
        }
    }

    /// <summary>Evaluates the homogeneous denominator of a B-spline surface.</summary>
    private static double DenominatorValue(BSplineSurface surface, double u, double v)
    {
        SurfaceBSplSLib.HomogeneousD0(u, v, 0, 0, surface.InternalPoles(), surface.InternalWeightsArray(),
                               surface.InternalUKnots(), surface.InternalVKnots(),
                               surface.InternalUMultiplicities(), surface.InternalVMultiplicities(),
                               surface.UDegree(), surface.VDegree(), true, true, false, false,
                               out double weight, out Point3D _);
        return weight;
    }

    /// <summary>Multiplier for the product of two surface denominators.</summary>
    private sealed class DenominatorProductEvaluator : SurfaceEvaluatorFunction
    {
        private readonly BSplineSurface? _first;
        private readonly BSplineSurface? _second;

        public DenominatorProductEvaluator(BSplineSurface? first, BSplineSurface? second)
        {
            _first = first;
            _second = second;
        }

        public override void Evaluate(int derivativeRequest, double u, double v, out double result,
                                      out int errorCode)
        {
            if (derivativeRequest != 0 || _first is null || _second is null)
            {
                result = 0.0; // OCCT leaves it unwritten
                errorCode = 1;
                return;
            }

            result = DenominatorValue(_first, u, v) * DenominatorValue(_second, u, v);
            errorCode = 0;
        }
    }

    private static bool MakeTripleProductBasis(NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                               int degree, out int productDegree,
                                               NCollection_Array1<double> productKnots,
                                               NCollection_Array1<int> productMults,
                                               NCollection_Array1<double> productFlatKnots)
    {
        productDegree = 3 * degree;
        if (productDegree > BSplineSurface.MaxDegree())
        {
            return false;
        }

        productKnots.Resize(1, knots.Size(), false);
        productMults.Resize(1, mults.Size(), false);
        int flatLength = 0;
        for (int knotIdx = 0; knotIdx < knots.Size(); ++knotIdx)
        {
            productKnots.ChangeAt(knotIdx) = knots.At(knotIdx);
            if (knotIdx == 0 || knotIdx + 1 == knots.Size())
            {
                productMults.ChangeAt(knotIdx) = productDegree + 1;
            }
            else
            {
                productMults.ChangeAt(knotIdx) = Min(productDegree, 2 * degree + mults.At(knotIdx));
            }
            flatLength += productMults.At(knotIdx);
        }

        productFlatKnots.Resize(1, flatLength, false);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(productKnots, productMults, productFlatKnots);
        return true;
    }

    private static bool MultiplyByDenominators(BSplineSurface surface, BSplineSurface firstDenominator,
                                               BSplineSurface secondDenominator, int uDegree,
                                               int vDegree, NCollection_Array1<double> uFlatKnots,
                                               NCollection_Array1<double> vFlatKnots,
                                               NCollection_Array2<Point3D> numerator,
                                               NCollection_Array2<double> denominator)
    {
        DenominatorProductEvaluator multiplier = new(firstDenominator, secondDenominator);

        SurfaceBSplSLib.FunctionMultiply(multiplier, surface.UDegree(), surface.VDegree(), surface.InternalUKnots(),
                                  surface.InternalVKnots(), surface.InternalUMultiplicities(),
                                  surface.InternalVMultiplicities(), surface.InternalPoles(),
                                  surface.InternalWeightsArray(), uFlatKnots, vFlatKnots, uDegree,
                                  vDegree, numerator, denominator, out int status);
        return status == 0;
    }

    private static BSplineSurface? MakeProfileSkin(NCollection_Array1<BSplineCurve3D?> profiles,
                                                        NCollection_Array1<double> profileParameters,
                                                        int vDegree, NCollection_Array1<double> vKnots,
                                                        NCollection_Array1<int> vMults,
                                                        NCollection_Array1<double> vFlatKnots)
    {
        BSplineCurve3D baseProfile = profiles.At(0)!;
        NCollection_Array2<Point3D> poles = new(1, baseProfile.PoleCount, 1, profiles.Length());
        NCollection_Array2<double> weights = new(1, baseProfile.PoleCount, 1, profiles.Length());
        NCollection_Array1<Point3D> column = new(1, profiles.Length());
        NCollection_Array1<double> columnWeights = new(1, profiles.Length());
        NCollection_Array1<int> contactOrders = new(1, profileParameters.Size());
        contactOrders.Init(0);

        for (int uPoleIdx = 1; uPoleIdx <= baseProfile.PoleCount; ++uPoleIdx)
        {
            for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
            {
                column.ChangeAt(profileIdx) = profiles.At(profileIdx)!.Pole(uPoleIdx);
                columnWeights.ChangeAt(profileIdx) = PoleWeight(profiles.At(profileIdx)!, uPoleIdx);
            }
            if (!InterpolatePoles(vDegree, vFlatKnots, profileParameters, contactOrders, column,
                                  columnWeights))
            {
                return null;
            }
            for (int vIdx = 0; vIdx < profiles.Size(); ++vIdx)
            {
                poles.ChangeAt(uPoleIdx - 1, vIdx) = column.At(vIdx);
                weights.ChangeAt(uPoleIdx - 1, vIdx) = columnWeights.At(vIdx);
            }
        }

        return new BSplineSurface(poles, weights, baseProfile.InternalKnots(), vKnots,
                                       baseProfile.InternalMultiplicities(), vMults, baseProfile.Degree(),
                                       vDegree);
    }

    private static BSplineSurface? MakeGuideSkin(NCollection_Array1<BSplineCurve3D?> guides,
                                                      NCollection_Array1<double> guideParameters, int uDegree,
                                                      NCollection_Array1<double> uKnots,
                                                      NCollection_Array1<int> uMults,
                                                      NCollection_Array1<double> uFlatKnots)
    {
        BSplineCurve3D baseGuide = guides.At(0)!;
        NCollection_Array2<Point3D> poles = new(1, guides.Length(), 1, baseGuide.PoleCount);
        NCollection_Array2<double> weights = new(1, guides.Length(), 1, baseGuide.PoleCount);
        NCollection_Array1<Point3D> row = new(1, guides.Length());
        NCollection_Array1<double> rowWeights = new(1, guides.Length());
        NCollection_Array1<int> contactOrders = new(1, guideParameters.Size());
        contactOrders.Init(0);

        for (int vPoleIdx = 1; vPoleIdx <= baseGuide.PoleCount; ++vPoleIdx)
        {
            for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
            {
                row.ChangeAt(guideIdx) = guides.At(guideIdx)!.Pole(vPoleIdx);
                rowWeights.ChangeAt(guideIdx) = PoleWeight(guides.At(guideIdx)!, vPoleIdx);
            }
            if (!InterpolatePoles(uDegree, uFlatKnots, guideParameters, contactOrders, row, rowWeights))
            {
                return null;
            }
            for (int uIdx = 0; uIdx < guides.Size(); ++uIdx)
            {
                poles.ChangeAt(uIdx, vPoleIdx - 1) = row.At(uIdx);
                weights.ChangeAt(uIdx, vPoleIdx - 1) = rowWeights.At(uIdx);
            }
        }

        return new BSplineSurface(poles, weights, uKnots, baseGuide.InternalKnots(), uMults,
                                       baseGuide.InternalMultiplicities(), uDegree, baseGuide.Degree());
    }

    private static double NormalizedKnot(double start, double end, double knot)
    {
        if (Math.Abs(knot - start) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return start;
        }
        if (Math.Abs(knot - end) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return end;
        }
        return knot;
    }

    /// <summary>Adds knots from one basis to a flat collection, clamping boundary values.</summary>
    private static void CollectKnots(int degree, double start, double end, NCollection_Array1<double> knots,
                                     NCollection_Array1<int> mults, NCollection_LinearVector<KnotEntry> entries)
    {
        for (int knotIdx = 0; knotIdx < knots.Size(); ++knotIdx)
        {
            double knot = NormalizedKnot(start, end, knots.At(knotIdx));
            bool isBoundary = knot == start || knot == end;
            int maxMult = isBoundary ? degree + 1 : degree;
            KnotEntry entry;
            entry.Parameter = knot;
            entry.Multiplicity = Min(mults.At(knotIdx), maxMult);
            entries.Append(entry);
        }
    }

    /// <summary>Sorts candidate knots and keeps the largest multiplicity in each tolerance group.</summary>
    private static void CompactKnots(NCollection_LinearVector<KnotEntry> entries,
                                     NCollection_LinearVector<double> knots, NCollection_LinearVector<int> mults)
    {
        if (entries.IsEmpty())
        {
            return;
        }

        Sorting.Sort(entries.AsSpan(), new KnotEntryLess());

        KnotEntry current = entries.Value(0);
        for (int entryIdx = 1; entryIdx < entries.Size(); ++entryIdx)
        {
            KnotEntry entry = entries.Value(entryIdx);
            if (Math.Abs(entry.Parameter - current.Parameter) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                current.Multiplicity = Max(current.Multiplicity, entry.Multiplicity);
                continue;
            }

            knots.Append(current.Parameter);
            mults.Append(current.Multiplicity);
            current = entry;
        }

        knots.Append(current.Parameter);
        mults.Append(current.Multiplicity);
    }

    private static void UniteKnots(int degree, NCollection_Array1<double> knots1, NCollection_Array1<int> mults1,
                                   NCollection_Array1<double> knots2, NCollection_Array1<int> mults2,
                                   out NCollection_HArray1<double>? mergedKnots,
                                   out NCollection_HArray1<int>? mergedMults)
    {
        // OCCT leaves the caller's null handles unchanged on failure.
        mergedKnots = null;
        mergedMults = null;
        if (!SameKnotRange(knots1, knots2, out double start, out double end))
        {
            return;
        }

        NCollection_LinearVector<KnotEntry> entries = new();
        CollectKnots(degree, start, end, knots1, mults1, entries);
        CollectKnots(degree, start, end, knots2, mults2, entries);

        NCollection_LinearVector<double> knots = new();
        NCollection_LinearVector<int> mults = new();
        CompactKnots(entries, knots, mults);

        if (knots.IsEmpty() || mults.IsEmpty())
        {
            return;
        }

        NCollection_Array1<double> knotArray = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ToOneBasedArrayView(knots);
        NCollection_Array1<int> multArray = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ToOneBasedArrayView(mults);
        mergedKnots = new NCollection_HArray1<double>(knotArray);
        mergedMults = new NCollection_HArray1<int>(multArray);
    }

    private static bool AlignSurfaces(BSplineSurface profileSurface, BSplineSurface guideSurface,
                                      BSplineSurface referenceSurface)
    {
        int uDegree = Max(Max(profileSurface.UDegree(), guideSurface.UDegree()), referenceSurface.UDegree());
        int vDegree = Max(Max(profileSurface.VDegree(), guideSurface.VDegree()), referenceSurface.VDegree());

        profileSurface.IncreaseDegree(uDegree, vDegree);
        guideSurface.IncreaseDegree(uDegree, vDegree);
        referenceSurface.IncreaseDegree(uDegree, vDegree);

        UniteKnots(uDegree, profileSurface.InternalUKnots(), profileSurface.InternalUMultiplicities(),
                   guideSurface.InternalUKnots(), guideSurface.InternalUMultiplicities(), out var uKnots12,
                   out var uMults12);
        if (uKnots12 is null || uMults12 is null)
        {
            return false;
        }

        UniteKnots(uDegree, uKnots12.Array1(), uMults12.Array1(), referenceSurface.InternalUKnots(),
                   referenceSurface.InternalUMultiplicities(), out var uKnots, out var uMults);
        if (uKnots is null || uMults is null)
        {
            return false;
        }

        UniteKnots(vDegree, profileSurface.InternalVKnots(), profileSurface.InternalVMultiplicities(),
                   guideSurface.InternalVKnots(), guideSurface.InternalVMultiplicities(), out var vKnots12,
                   out var vMults12);
        if (vKnots12 is null || vMults12 is null)
        {
            return false;
        }

        UniteKnots(vDegree, vKnots12.Array1(), vMults12.Array1(), referenceSurface.InternalVKnots(),
                   referenceSurface.InternalVMultiplicities(), out var vKnots, out var vMults);
        if (vKnots is null || vMults is null)
        {
            return false;
        }

        profileSurface.InsertUKnots(uKnots.Array1(), uMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        guideSurface.InsertUKnots(uKnots.Array1(), uMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        referenceSurface.InsertUKnots(uKnots.Array1(), uMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);

        profileSurface.InsertVKnots(vKnots.Array1(), vMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        guideSurface.InsertVKnots(vKnots.Array1(), vMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        referenceSurface.InsertVKnots(vKnots.Array1(), vMults.Array1(), global::Kerf.Engine.Foundation.Precision.PConfusion(), false);
        return true;
    }

    /// <summary>Builds the final network surface by correcting the profile skin with guide deviations.</summary>
    private static BSplineSurface? MakeCorrectedProfileSkin(BSplineSurface profileSurface,
                                                                 BSplineSurface guideSurface,
                                                                 BSplineSurface referenceSurface,
                                                                 ref ResultStatus status)
    {
        if (profileSurface.UPoleCount != guideSurface.UPoleCount
            || profileSurface.UPoleCount != referenceSurface.UPoleCount
            || profileSurface.VPoleCount != guideSurface.VPoleCount
            || profileSurface.VPoleCount != referenceSurface.VPoleCount)
        {
            status = ResultStatus.RationalConstructionFailed;
            return null;
        }

        if (!profileSurface.IsURational && !profileSurface.IsVRational && !guideSurface.IsURational
            && !guideSurface.IsVRational && !referenceSurface.IsURational
            && !referenceSurface.IsVRational)
        {
            Span<Point3D> profilePoles = profileSurface.InternalPoles().AsSpan();
            Span<Point3D> guidePoles = guideSurface.InternalPoles().AsSpan();
            Span<Point3D> referencePoles = referenceSurface.InternalPoles().AsSpan();
            NCollection_Array2<Point3D> resultPoles = new(profileSurface.InternalPoles());
            Span<Point3D> result = resultPoles.AsSpan();
            for (int poleIdx = 0; poleIdx < result.Length; ++poleIdx)
            {
                Xyz point = profilePoles[poleIdx].XYZ() + guidePoles[poleIdx].XYZ()
                                - referencePoles[poleIdx].XYZ();
                result[poleIdx] = new Point3D(point);
            }

            return new BSplineSurface(resultPoles, profileSurface.InternalUKnots(),
                                           profileSurface.InternalVKnots(),
                                           profileSurface.InternalUMultiplicities(),
                                           profileSurface.InternalVMultiplicities(), profileSurface.UDegree(),
                                           profileSurface.VDegree());
        }

        NCollection_Array1<double> uKnots = new();
        NCollection_Array1<int> uMults = new();
        NCollection_Array1<double> uFlatKnots = new();
        NCollection_Array1<double> vKnots = new();
        NCollection_Array1<int> vMults = new();
        NCollection_Array1<double> vFlatKnots = new();
        int uDegree = 0;
        int vDegree = 0;
        if (!MakeTripleProductBasis(profileSurface.InternalUKnots(), profileSurface.InternalUMultiplicities(),
                                    profileSurface.UDegree(), out uDegree, uKnots, uMults, uFlatKnots)
            || !MakeTripleProductBasis(profileSurface.InternalVKnots(),
                                       profileSurface.InternalVMultiplicities(), profileSurface.VDegree(),
                                       out vDegree, vKnots, vMults, vFlatKnots))
        {
            status = ResultStatus.RationalDegreeOverflow;
            return null;
        }

        int nbUPoles = uFlatKnots.Length() - uDegree - 1;
        int nbVPoles = vFlatKnots.Length() - vDegree - 1;
        if (nbUPoles < 2 || nbVPoles < 2)
        {
            status = ResultStatus.RationalConstructionFailed;
            return null;
        }

        NCollection_Array2<Point3D> profileNumerator = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<double> profileDenominator = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<Point3D> guideNumerator = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<double> guideDenominator = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<Point3D> referenceNumerator = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<double> referenceDenominator = new(1, nbUPoles, 1, nbVPoles);

        if (!MultiplyByDenominators(profileSurface, guideSurface, referenceSurface, uDegree, vDegree,
                                    uFlatKnots, vFlatKnots, profileNumerator, profileDenominator)
            || !MultiplyByDenominators(guideSurface, profileSurface, referenceSurface, uDegree, vDegree,
                                       uFlatKnots, vFlatKnots, guideNumerator, guideDenominator)
            || !MultiplyByDenominators(referenceSurface, profileSurface, guideSurface, uDegree, vDegree,
                                       uFlatKnots, vFlatKnots, referenceNumerator, referenceDenominator))
        {
            status = ResultStatus.RationalConstructionFailed;
            return null;
        }

        NCollection_Array2<Point3D> resultPoles2 = new(1, nbUPoles, 1, nbVPoles);
        NCollection_Array2<double> resultWeights = new(1, nbUPoles, 1, nbVPoles);
        Span<Point3D> poles = resultPoles2.AsSpan();
        Span<double> weights = resultWeights.AsSpan();
        Span<double> profileDen = profileDenominator.AsSpan();
        Span<double> guideDen = guideDenominator.AsSpan();
        Span<double> referenceDen = referenceDenominator.AsSpan();
        Span<Point3D> profileNum = profileNumerator.AsSpan();
        Span<Point3D> guideNum = guideNumerator.AsSpan();
        Span<Point3D> referenceNum = referenceNumerator.AsSpan();
        for (int poleIdx = 0; poleIdx < poles.Length; ++poleIdx)
        {
            double weight = profileDen[poleIdx];
            if (weight <= global::Kerf.Engine.Foundation.Precision.Confusion() || Math.Abs(guideDen[poleIdx] - weight) > global::Kerf.Engine.Foundation.Precision.Confusion()
                || Math.Abs(referenceDen[poleIdx] - weight) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                status = ResultStatus.RationalConstructionFailed;
                return null;
            }

            Xyz numerator = profileNum[poleIdx].XYZ() + guideNum[poleIdx].XYZ() - referenceNum[poleIdx].XYZ();
            weights[poleIdx] = weight;
            poles[poleIdx] = new Point3D(numerator / weight);
        }

        return new BSplineSurface(resultPoles2, resultWeights, uKnots, vKnots, uMults, vMults, uDegree,
                                       vDegree);
    }

    private static bool CanSetPeriodic(BSplineSurface surface, bool alongU, double tolerance)
    {
        int nbChecks = alongU ? surface.VPoleCount : surface.UPoleCount;
        for (int checkIdx = 1; checkIdx <= nbChecks; ++checkIdx)
        {
            Point3D firstPole = alongU ? surface.Pole(1, checkIdx) : surface.Pole(checkIdx, 1);
            Point3D lastPole = alongU ? surface.Pole(surface.UPoleCount, checkIdx)
                                         : surface.Pole(checkIdx, surface.VPoleCount);
            if (firstPole.Distance(lastPole) > tolerance)
            {
                return false;
            }
        }
        return true;
    }

    private static bool IsReadyToBuild(NCollection_Array1<BSplineCurve3D?> profiles,
                                       NCollection_Array1<BSplineCurve3D?> guides,
                                       NCollection_Array1<double> profileParameters,
                                       NCollection_Array1<double> guideParameters,
                                       NCollection_Array2<Point3D> intersectionPoints,
                                       NCollection_Array2<double> intersectionWeights)
    {
        return profiles.Length() >= 2 && guides.Length() >= 2 && !HasPeriodicCurve(profiles)
               && !HasPeriodicCurve(guides) && profiles.Length() == profileParameters.Length()
               && guides.Length() == guideParameters.Length()
               && intersectionPoints.ColLength() == guideParameters.Length()
               && intersectionPoints.RowLength() == profileParameters.Length()
               && intersectionWeights.ColLength() == guideParameters.Length()
               && intersectionWeights.RowLength() == profileParameters.Length()
               && CheckParameters(profileParameters) && CheckParameters(guideParameters)
               && CheckWeights(intersectionWeights);
    }

    // Creates a seam guide through the start points of all closed profiles.
    private static BSplineCurve3D? MakeProfileSeamGuide(NCollection_Array1<BSplineCurve3D?> profiles,
                                                           NCollection_Array1<double> profileParameters)
    {
        NCollection_HArray1<Point3D> points = new(1, profiles.Length());
        NCollection_HArray1<double> parameters = new(1, profileParameters.Length());
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            BSplineCurve3D profile = profiles.At(profileIdx)!;
            points.SetValue(profileIdx + 1, profile.Value(profile.FirstParameter()));
            parameters.SetValue(profileIdx + 1, profileParameters.At(profileIdx));
        }

        ApiInterpolate interpolator = new(points, parameters, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        interpolator.Perform();
        return interpolator.IsDone ? interpolator.Curve() : null;
    }

    // Completes a closed U-direction interpolation basis at both profile endpoints.
    private static bool CompleteClosedGuideSeam(NCollection_Array1<BSplineCurve3D?> profiles,
                                                NCollection_Array1<double> profileParameters,
                                                NCollection_Array1<BSplineCurve3D?> guides,
                                                NCollection_Array1<double> guideParameters,
                                                NCollection_Array2<Point3D> intersectionPoints,
                                                NCollection_Array2<double> intersectionWeights,
                                                ref NCollection_Array1<BSplineCurve3D?> closedGuides,
                                                ref NCollection_Array1<double> closedGuideParameters,
                                                ref NCollection_Array2<Point3D> closedIntersectionPoints,
                                                ref NCollection_Array2<double> closedIntersectionWeights)
    {
        double profileStart = profiles.At(0)!.FirstParameter();
        double profileEnd = profiles.At(0)!.LastParameter();
        bool needsStartSeam = guideParameters.First() > profileStart + global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool needsEndSeam = guideParameters.Last() < profileEnd - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (!needsStartSeam && !needsEndSeam)
        {
            return true;
        }

        BSplineCurve3D? seamGuide = null;
        if (needsStartSeam)
        {
            seamGuide = MakeProfileSeamGuide(profiles, profileParameters);
            if (seamGuide is null)
            {
                return false;
            }
        }

        int guideOffset = needsStartSeam ? 1 : 0;
        int nbGuides = guides.Size() + guideOffset + (needsEndSeam ? 1 : 0);
        closedGuides = new NCollection_Array1<BSplineCurve3D?>(1, nbGuides);
        closedGuideParameters = new NCollection_Array1<double>(1, nbGuides);
        if (needsStartSeam)
        {
            closedGuides.ChangeAt(0) = seamGuide;
            closedGuideParameters.ChangeAt(0) = profileStart;
        }
        for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
        {
            closedGuides.ChangeAt(guideIdx + guideOffset) = guides.At(guideIdx);
            closedGuideParameters.ChangeAt(guideIdx + guideOffset) = guideParameters.At(guideIdx);
        }
        if (needsEndSeam)
        {
            closedGuides.ChangeAt(nbGuides - 1) = needsStartSeam ? seamGuide : guides.At(0);
            closedGuideParameters.ChangeAt(nbGuides - 1) = profileEnd;
        }

        closedIntersectionPoints = new NCollection_Array2<Point3D>(1, nbGuides, 1, intersectionPoints.NbColumns());
        closedIntersectionWeights = new NCollection_Array2<double>(1, nbGuides, 1, intersectionWeights.NbColumns());
        if (needsStartSeam)
        {
            for (int profileIdx = 0; profileIdx < intersectionPoints.NbColumns(); ++profileIdx)
            {
                closedIntersectionPoints.ChangeAt(0, profileIdx) =
                  profiles.At(profileIdx)!.Value(profiles.At(profileIdx)!.FirstParameter());
                closedIntersectionWeights.ChangeAt(0, profileIdx) = PoleWeight(profiles.At(profileIdx)!, 1);
            }
        }
        for (int guideIdx = 0; guideIdx < intersectionPoints.NbRows(); ++guideIdx)
        {
            for (int profileIdx = 0; profileIdx < intersectionPoints.NbColumns(); ++profileIdx)
            {
                closedIntersectionPoints.ChangeAt(guideIdx + guideOffset, profileIdx) =
                  intersectionPoints.At(guideIdx, profileIdx);
                closedIntersectionWeights.ChangeAt(guideIdx + guideOffset, profileIdx) =
                  intersectionWeights.At(guideIdx, profileIdx);
            }
        }
        if (needsEndSeam)
        {
            for (int profileIdx = 0; profileIdx < intersectionPoints.NbColumns(); ++profileIdx)
            {
                closedIntersectionPoints.ChangeAt(nbGuides - 1, profileIdx) =
                  closedIntersectionPoints.At(0, profileIdx);
                closedIntersectionWeights.ChangeAt(nbGuides - 1, profileIdx) =
                  closedIntersectionWeights.At(0, profileIdx);
            }
        }
        return true;
    }

    private static BSplineSurface? MakeNetworkSurface(NCollection_Array1<BSplineCurve3D?> profiles,
                                                           NCollection_Array1<BSplineCurve3D?> guides,
                                                           NCollection_Array1<double> profileParameters,
                                                           NCollection_Array1<double> guideParameters,
                                                           NCollection_Array2<Point3D> intersectionPoints,
                                                           NCollection_Array2<double> intersectionWeights,
                                                           bool isUClosed, ref ResultStatus status)
    {
        status = ResultStatus.ConstructionFailed;

        NCollection_Array1<BSplineCurve3D?> closedGuides = new();
        NCollection_Array1<double> closedGuideParameters = new();
        NCollection_Array2<Point3D> closedIntersectionPoints = new();
        NCollection_Array2<double> closedIntersectionWeights = new();
        bool shouldCompleteUSeam =
          isUClosed
          && (guideParameters.First() > profiles.At(0)!.FirstParameter() + global::Kerf.Engine.Foundation.Precision.PConfusion()
              || guideParameters.Last() < profiles.At(0)!.LastParameter() - global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (shouldCompleteUSeam
            && !CompleteClosedGuideSeam(profiles, profileParameters, guides, guideParameters,
                                        intersectionPoints, intersectionWeights, ref closedGuides,
                                        ref closedGuideParameters, ref closedIntersectionPoints,
                                        ref closedIntersectionWeights))
        {
            status = ResultStatus.SkinningFailed;
            return null;
        }
        if (shouldCompleteUSeam && !PrepareCurveFamily(closedGuides))
        {
            status = ResultStatus.CurveCompatibilityFailed;
            return null;
        }
        NCollection_Array1<BSplineCurve3D?> aGuides = shouldCompleteUSeam ? closedGuides : guides;
        NCollection_Array1<double> aGuideParameters = shouldCompleteUSeam ? closedGuideParameters : guideParameters;
        NCollection_Array2<Point3D> aIntersectionPoints =
          shouldCompleteUSeam ? closedIntersectionPoints : intersectionPoints;
        NCollection_Array2<double> aIntersectionWeights =
          shouldCompleteUSeam ? closedIntersectionWeights : intersectionWeights;

        SkinningBasis uBasis = new();
        uBasis.Init(aGuideParameters);

        SkinningBasis vBasis = new();
        vBasis.Init(profileParameters);

        BSplineSurface? profileSurface = MakeProfileSkin(profiles, profileParameters, vBasis.Degree,
                                                               vBasis.Knots, vBasis.Mults, vBasis.FlatKnots);
        if (profileSurface is null)
        {
            status = ResultStatus.SkinningFailed;
            return null;
        }

        BSplineSurface? guideSurface = MakeGuideSkin(aGuides, aGuideParameters, uBasis.Degree, uBasis.Knots,
                                                           uBasis.Mults, uBasis.FlatKnots);
        if (guideSurface is null)
        {
            status = ResultStatus.SkinningFailed;
            return null;
        }

        // The reference surface starts from the same contact grid validated during preparation.
        NCollection_Array2<Point3D> referencePoles = new(aIntersectionPoints);
        NCollection_Array2<double> referenceWeights = new(aIntersectionWeights);
        bool isReferencePolynomial = HasRationalCurve(profiles) != HasRationalCurve(aGuides);
        if (isReferencePolynomial)
        {
            referenceWeights.Init(1.0);
        }
        else
        {
            Span<Point3D> poles = referencePoles.AsSpan();
            Span<double> weights = referenceWeights.AsSpan();
            for (int poleIdx = 0; poleIdx < poles.Length; ++poleIdx)
            {
                double weight = weights[poleIdx];
                poles[poleIdx] = new Point3D(poles[poleIdx].XYZ() * weight);
            }
        }

        SurfaceBSplSLib.Interpolate(uBasis.Degree, vBasis.Degree, uBasis.FlatKnots, vBasis.FlatKnots, aGuideParameters,
                             profileParameters, referencePoles, referenceWeights, out int inversionProblem);
        if (inversionProblem != 0)
        {
            status = ResultStatus.ReferenceSurfaceFailed;
            return null;
        }
        if (!isReferencePolynomial)
        {
            Span<Point3D> poles = referencePoles.AsSpan();
            Span<double> weights = referenceWeights.AsSpan();
            for (int poleIdx = 0; poleIdx < poles.Length; ++poleIdx)
            {
                double weight = weights[poleIdx];
                if (weight <= Constants.Resolution())
                {
                    status = ResultStatus.ReferenceSurfaceFailed;
                    return null;
                }
                poles[poleIdx] = new Point3D(poles[poleIdx].XYZ() / weight);
            }
        }

        BSplineSurface referenceSurface = new(referencePoles, referenceWeights, uBasis.Knots, vBasis.Knots,
                                                    uBasis.Mults, vBasis.Mults, uBasis.Degree, vBasis.Degree);
        if (!AlignSurfaces(profileSurface, guideSurface, referenceSurface))
        {
            status = ResultStatus.KnotAlignmentFailed;
            return null;
        }
        return MakeCorrectedProfileSkin(profileSurface, guideSurface, referenceSurface, ref status);
    }

    private static bool ApplyPeriodicity(BSplineSurface surface, bool isUClosed, bool isVClosed,
                                         double tolerance)
    {
        double aTolerance = Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (isUClosed)
        {
            if (!CanSetPeriodic(surface, true, aTolerance))
            {
                return false;
            }
            surface.SetUPeriodic();
        }

        if (isVClosed)
        {
            if (!CanSetPeriodic(surface, false, aTolerance))
            {
                return false;
            }
            surface.SetVPeriodic();
        }
        return true;
    }

    /// <summary>Creates an empty network surface algorithm.</summary>
    public NetworkSurface()
    {
    }

    /// <summary>
    /// Initializes the algorithm with a compatible profile/guide B-spline network: profiles (U direction) located
    /// at the V parameters <paramref name="profileParameters"/>, guides (V direction) at the U parameters
    /// <paramref name="guideParameters"/>, the contact grid (guides by profiles) and its rational weights, the
    /// tolerance of the closed-seam checks and the closed directions. The arrays are copied; the curves are kept.
    /// </summary>
    public void Init(NCollection_Array1<BSplineCurve3D?> profiles, NCollection_Array1<BSplineCurve3D?> guides,
                     NCollection_Array1<double> profileParameters, NCollection_Array1<double> guideParameters,
                     NCollection_Array2<Point3D> intersectionPoints, NCollection_Array2<double> intersectionWeights,
                     double tolerance, bool isUClosed, bool isVClosed)
    {
        _profiles = new NCollection_Array1<BSplineCurve3D?>(profiles);
        _guides = new NCollection_Array1<BSplineCurve3D?>(guides);
        _profileParameters = new NCollection_Array1<double>(profileParameters);
        _guideParameters = new NCollection_Array1<double>(guideParameters);
        _intersectionPoints = new NCollection_Array2<Point3D>(intersectionPoints);
        _intersectionWeights = new NCollection_Array2<double>(intersectionWeights);
        _tolerance = tolerance;
        _isUClosed = isUClosed;
        _isVClosed = isVClosed;
        _status = ResultStatus.NotStarted;
        _surface = null;
    }

    /// <summary>Performs the pole-based network surface construction.</summary>
    public void Perform()
    {
        _status = ResultStatus.NotStarted;
        _surface = null;

        if (!IsReadyToBuild(_profiles, _guides, _profileParameters, _guideParameters, _intersectionPoints,
                            _intersectionWeights))
        {
            _status = ResultStatus.InvalidInput;
            return;
        }

        try
        {
            if (!PrepareCurveFamily(_profiles) || !PrepareCurveFamily(_guides))
            {
                _status = ResultStatus.CurveCompatibilityFailed;
                return;
            }

            ResultStatus surfaceStatus = ResultStatus.ConstructionFailed;
            _surface = MakeNetworkSurface(_profiles, _guides, _profileParameters, _guideParameters,
                                           _intersectionPoints, _intersectionWeights, _isUClosed,
                                           ref surfaceStatus);
            if (_surface is null)
            {
                _status = surfaceStatus;
                return;
            }

            if (!ApplyPeriodicity(_surface, _isUClosed, _isVClosed, _tolerance))
            {
                _surface = null;
                _status = ResultStatus.PeriodicityFailed;
                return;
            }
            ReparametrizeSurface(_surface);
        }
        catch (FailureException)
        {
            _surface = null;
            _status = ResultStatus.ConstructionFailed;
            return;
        }

        _status = ResultStatus.Done;
    }

    /// <summary>Returns true if the surface was successfully constructed.</summary>
    public bool IsDone => _status == ResultStatus.Done;

    /// <summary>Returns the result state of the last Perform() call.</summary>
    public ResultStatus Status() => _status;

    /// <summary>The constructed surface, frozen. Throws StdFail_NotDone unless <see cref="Perform"/> succeeded.</summary>
    public BSplineSurface Surface()
    {
        if (!IsDone)
        {
            throw new NotDoneException("GeomFill_NetworkSurface::Surface");
        }
        _surface!.Freeze();
        return _surface;
    }
}
