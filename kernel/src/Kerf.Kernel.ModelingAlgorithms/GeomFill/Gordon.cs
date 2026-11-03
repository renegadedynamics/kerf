// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Gordon.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Gordon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A Gordon surface (transfinite interpolation) through a network of intersecting profile and guide curves: the
/// curves are converted to B-splines, periodic ones expanded, their contacts found, the network ordered and the
/// curves reparametrised to shared parameters, then <see cref="NetworkSurface"/> builds the surface. It
/// generalises GeomFill_Coons (four boundaries) to N x M networks.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> converts the caller's curves into B-splines of its own
/// (GeomConvert copies) and <see cref="Perform"/> works on further copies, so the caller's curves are never changed;
/// <see cref="Surface"/> hands out the surface frozen. <see cref="SetParallelMode"/> runs the contact search in
/// parallel; the result is the serial one (decision 490).
/// </remarks>
public class Gordon
{
    /// <summary>Result state of the last Perform() call.</summary>
    public enum ResultStatus
    {
        NotStarted,
        Done,
        InvalidInput,
        ConversionFailed,
        IntersectionFailed,
        OrderingFailed,
        ReparametrizationFailed,
        CompatibilityFailed,
        CurveCompatibilityFailed,
        RationalReparametrizationFailed,
        SkinningFailed,
        ReferenceSurfaceFailed,
        KnotAlignmentFailed,
        RationalDegreeOverflow,
        RationalConstructionFailed,
        PeriodicityFailed,
        ApproximationFailed,
        ConstructionFailed
    }

    /// <summary>Controls behavior when exact pole-based construction fails.</summary>
    public enum ApproximationMode
    {
        ExactOnly,
        AllowApproximateFallback
    }

    /// <summary>Construction stage reached by the last Perform() call.</summary>
    public enum BuildStage
    {
        NotStarted,
        InputConversion,
        ContactDiscovery,
        NetworkOrdering,
        Reparametrization,
        ExactConstruction,
        Validation,
        Approximation
    }

    /// <summary>Diagnostics for the last Perform() call.</summary>
    public struct BuildReport : IEquatable<BuildReport>
    {
        public ResultStatus Status = ResultStatus.NotStarted;
        public BuildStage FailedStage = BuildStage.NotStarted;
        public bool IsApproximate = false;
        public double MaxContactGap = 0.0;
        public double MaxReparametrizationDeviation = 0.0;
        public double MaxProfileDeviation = 0.0;
        public double MaxGuideDeviation = 0.0;
        public double MaxApproximationDeviation = 0.0;

        public BuildReport()
        {
        }

        public readonly bool Equals(BuildReport other) =>
            Status == other.Status && FailedStage == other.FailedStage && IsApproximate == other.IsApproximate
            && MaxContactGap.Equals(other.MaxContactGap)
            && MaxReparametrizationDeviation.Equals(other.MaxReparametrizationDeviation)
            && MaxProfileDeviation.Equals(other.MaxProfileDeviation) && MaxGuideDeviation.Equals(other.MaxGuideDeviation)
            && MaxApproximationDeviation.Equals(other.MaxApproximationDeviation);

        public override readonly bool Equals(object? obj) => obj is BuildReport other && Equals(other);

        public override readonly int GetHashCode() =>
            HashCode.Combine(Status, FailedStage, IsApproximate, MaxContactGap, MaxReparametrizationDeviation,
                             MaxProfileDeviation, MaxGuideDeviation, MaxApproximationDeviation);

        public static bool operator ==(BuildReport left, BuildReport right) => left.Equals(right);

        public static bool operator !=(BuildReport left, BuildReport right) => !left.Equals(right);
    }

    private NCollection_Array1<BSplineCurve3D?> _inputProfiles = new();
    private NCollection_Array1<BSplineCurve3D?> _inputGuides = new();
    private NCollection_Array1<BSplineCurve3D?> _profiles = new();
    private NCollection_Array1<BSplineCurve3D?> _guides = new();
    private NCollection_Array2<double> _profileParams = new();
    private NCollection_Array2<double> _guideParams = new();
    private BSplineSurface? _surface;
    private double _tolerance = 0.0;
    private bool _isUClosed = false;
    private bool _isVClosed = false;
    private bool _toUseParallel = false;
    private ApproximationMode _approximationMode = ApproximationMode.ExactOnly;
    private BuildReport _report = new();

    // Returns whether at least one prepared curve uses rational weights.
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

    // Returns whether either prepared curve family uses rational weights.
    private static bool HasRationalCurve(NCollection_Array1<BSplineCurve3D?> profiles,
                                         NCollection_Array1<BSplineCurve3D?> guides)
    {
        return HasRationalCurve(profiles) || HasRationalCurve(guides);
    }

    // Converts detailed network-construction statuses to the public Gordon status.
    private static ResultStatus MapNetworkStatus(NetworkSurface.ResultStatus status)
    {
        switch (status)
        {
            case NetworkSurface.ResultStatus.Done:
                return ResultStatus.Done;
            case NetworkSurface.ResultStatus.InvalidInput:
                return ResultStatus.CompatibilityFailed;
            case NetworkSurface.ResultStatus.CurveCompatibilityFailed:
                return ResultStatus.CurveCompatibilityFailed;
            case NetworkSurface.ResultStatus.SkinningFailed:
                return ResultStatus.SkinningFailed;
            case NetworkSurface.ResultStatus.ReferenceSurfaceFailed:
                return ResultStatus.ReferenceSurfaceFailed;
            case NetworkSurface.ResultStatus.KnotAlignmentFailed:
                return ResultStatus.KnotAlignmentFailed;
            case NetworkSurface.ResultStatus.RationalDegreeOverflow:
                return ResultStatus.RationalDegreeOverflow;
            case NetworkSurface.ResultStatus.RationalConstructionFailed:
                return ResultStatus.RationalConstructionFailed;
            case NetworkSurface.ResultStatus.PeriodicityFailed:
                return ResultStatus.PeriodicityFailed;
            case NetworkSurface.ResultStatus.NotStarted:
            case NetworkSurface.ResultStatus.ConstructionFailed:
                return ResultStatus.ConstructionFailed;
        }
        return ResultStatus.ConstructionFailed;
    }

    // Returns true when a sampled fallback can still use a validated prepared network.
    private static bool CanApproximateAfterExactFailure(ResultStatus status)
    {
        return status == ResultStatus.CurveCompatibilityFailed || status == ResultStatus.SkinningFailed
               || status == ResultStatus.ReferenceSurfaceFailed || status == ResultStatus.KnotAlignmentFailed
               || status == ResultStatus.RationalDegreeOverflow
               || status == ResultStatus.RationalConstructionFailed
               || status == ResultStatus.ConstructionFailed;
    }

    // Chooses a deterministic fallback grid density from the prepared network itself.
    private static int FallbackSampleCount(NCollection_Array1<BSplineCurve3D?> curves)
    {
        int nbSamples = Max(curves.Size(), 2);
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            if (curves.At(curveIdx) is not null)
            {
                nbSamples = Max(nbSamples, curves.At(curveIdx)!.PoleCount);
            }
        }
        return Max(nbSamples, BSplineCurve3D.MaxDegree / 2);
    }

    // Returns regularly spaced parameters in the unit interval.
    private static NCollection_Array1<double> UnitParameters(int nbSamples)
    {
        NCollection_Array1<double> parameters = new(1, nbSamples);
        for (int sampleIdx = 0; sampleIdx < nbSamples; ++sampleIdx)
        {
            parameters.ChangeAt(sampleIdx) = (double)sampleIdx / (double)(nbSamples - 1);
        }
        return parameters;
    }

    // Fits a non-exact B-spline surface through a point grid for approximate fallback.
    private static BSplineSurface? ApproximateGrid(NCollection_Array2<Point3D> points, double tolerance)
    {
        PointsToBSplineSurface approx = new();
        approx.Init(points, IsoParametric, 1, BSplineSurface.MaxDegree, C1,
                      Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion()));
        return approx.IsDone ? approx.Surface() : null;
    }

    // Samples the prepared profiles as section curves of an approximate profile skin.
    private static NCollection_Array2<Point3D> ProfileSampleGrid(NCollection_Array1<BSplineCurve3D?> profiles,
                                                                int nbUSamples)
    {
        NCollection_Array1<double> uParams = UnitParameters(nbUSamples);
        NCollection_Array2<Point3D> points = new(1, nbUSamples, 1, profiles.Size());
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            CurveGrid evaluator = new(profiles.At(profileIdx));
            NCollection_Array1<Point3D> profilePoints = evaluator.EvaluateGrid(uParams);
            for (int uIdx = 0; uIdx < nbUSamples; ++uIdx)
            {
                points.ChangeAt(uIdx, profileIdx) = profilePoints.At(uIdx);
            }
        }
        return points;
    }

    // Samples the prepared guides as section curves of an approximate guide skin.
    private static NCollection_Array2<Point3D> GuideSampleGrid(NCollection_Array1<BSplineCurve3D?> guides,
                                                              int nbVSamples)
    {
        NCollection_Array1<double> vParams = UnitParameters(nbVSamples);
        NCollection_Array2<Point3D> points = new(1, guides.Size(), 1, nbVSamples);
        for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
        {
            CurveGrid evaluator = new(guides.At(guideIdx));
            NCollection_Array1<Point3D> guidePoints = evaluator.EvaluateGrid(vParams);
            for (int vIdx = 0; vIdx < nbVSamples; ++vIdx)
            {
                points.ChangeAt(guideIdx, vIdx) = guidePoints.At(vIdx);
            }
        }
        return points;
    }

    // Builds a sampled fallback from approximate profile, guide, and reference surfaces.
    private static BSplineSurface? ApproximatePreparedNetwork(NCollection_Array1<BSplineCurve3D?> profiles,
                                                                   NCollection_Array1<BSplineCurve3D?> guides,
                                                                   NCollection_Array2<Point3D> intersections,
                                                                   double tolerance)
    {
        int nbUSamples = FallbackSampleCount(profiles);
        int nbVSamples = FallbackSampleCount(guides);

        BSplineSurface? profileSurface = ApproximateGrid(ProfileSampleGrid(profiles, nbUSamples), tolerance);
        BSplineSurface? guideSurface = ApproximateGrid(GuideSampleGrid(guides, nbVSamples), tolerance);
        BSplineSurface? referenceSurface = ApproximateGrid(intersections, tolerance);
        if (profileSurface is null || guideSurface is null || referenceSurface is null)
        {
            return null;
        }
        NCollection_Array2<Point3D> points = new(1, nbUSamples, 1, nbVSamples);
        NCollection_Array1<double> uParams = UnitParameters(nbUSamples);
        NCollection_Array1<double> vParams = UnitParameters(nbVSamples);
        SurfaceGrid profileEvaluator = new(profileSurface);
        SurfaceGrid guideEvaluator = new(guideSurface);
        SurfaceGrid referenceEvaluator = new(referenceSurface);
        NCollection_Array2<Point3D> profilePoints = profileEvaluator.EvaluateGrid(uParams, vParams);
        NCollection_Array2<Point3D> guidePoints = guideEvaluator.EvaluateGrid(uParams, vParams);
        NCollection_Array2<Point3D> referencePoints = referenceEvaluator.EvaluateGrid(uParams, vParams);
        for (int uIdx = 0; uIdx < nbUSamples; ++uIdx)
        {
            for (int vIdx = 0; vIdx < nbVSamples; ++vIdx)
            {
                Xyz point = profilePoints.At(uIdx, vIdx).XYZ() + guidePoints.At(uIdx, vIdx).XYZ()
                                - referencePoints.At(uIdx, vIdx).XYZ();
                points.ChangeAt(uIdx, vIdx) = new Point3D(point);
            }
        }
        return ApproximateGrid(points, tolerance);
    }

    // Resets all report fields before starting a construction attempt.
    private static void ResetReport(ref BuildReport report)
    {
        report = new BuildReport();
    }

    // Records the result status and its failing construction stage.
    private static void SetStatus(ref BuildReport report, ResultStatus newStatus, BuildStage stage)
    {
        report.Status = newStatus;
        report.FailedStage = newStatus == ResultStatus.Done ? BuildStage.NotStarted : stage;
    }

    // Returns whether every profile closes geometrically at its parameter endpoints.
    private static bool AreClosedCurves(NCollection_Array1<BSplineCurve3D?> curves, double tolerance)
    {
        if (curves.IsEmpty())
        {
            return false;
        }
        double aTolerance = Max(tolerance, global::Kerf.Engine.Foundation.Precision.Confusion());
        for (int curveIdx = 0; curveIdx < curves.Size(); ++curveIdx)
        {
            BSplineCurve3D? curve = curves.At(curveIdx);
            if (curve is null
                || curve.Value(curve.FirstParameter).DistanceTo(curve.Value(curve.LastParameter)) > aTolerance)
            {
                return false;
            }
        }
        return true;
    }

    // Clones input curves because network preparation modifies its working copies.
    private static bool CopyWorkingCurves(NCollection_Array1<BSplineCurve3D?> input,
                                          NCollection_Array1<BSplineCurve3D?> working)
    {
        try
        {
            for (int curveIdx = 0; curveIdx < input.Size(); ++curveIdx)
            {
                if (input.At(curveIdx) is null)
                {
                    return false;
                }
                working.ChangeAt(curveIdx) = input.At(curveIdx)!.Copy() as BSplineCurve3D;
                if (working.At(curveIdx) is null)
                {
                    return false;
                }
            }
        }
        catch (FailureException)
        {
            return false;
        }
        return true;
    }

    // Returns the first knot in the requested surface parameter direction.
    private static double SurfaceStartParameter(BSplineSurface surface, bool isU)
    {
        return isU ? surface.UKnot(1) : surface.VKnot(1);
    }

    // Returns the last knot in the requested surface parameter direction.
    private static double SurfaceEndParameter(BSplineSurface surface, bool isU)
    {
        return isU ? surface.UKnot(surface.UKnotCount) : surface.VKnot(surface.VKnotCount);
    }

    // Maps a unit parameter to the knot range of the requested surface direction.
    private static double SurfaceParameter(BSplineSurface surface, bool isU, double unitParameter)
    {
        double first = SurfaceStartParameter(surface, isU);
        return first + unitParameter * (SurfaceEndParameter(surface, isU) - first);
    }

    // Measures the maximum deviation of a curve from its corresponding surface isoparameter.
    private static double MaxCurveDeviation(BSplineSurface surface, BSplineCurve3D curve,
                                            double fixedParameter, bool isProfile)
    {
        CurveAdaptor adaptor = new(curve);
        double maxDeviation = 0.0;
        void evaluate(double parameter)
        {
            double u = isProfile ? SurfaceParameter(surface, true, parameter)
                                     : SurfaceParameter(surface, true, fixedParameter);
            double v = isProfile ? SurfaceParameter(surface, false, fixedParameter)
                                     : SurfaceParameter(surface, false, parameter);
            maxDeviation = Max(maxDeviation, adaptor.EvalD0(parameter).DistanceTo(surface.Value(u, v)));
        }

        int samplesPerSpan = curve.Degree + (isProfile ? surface.UDegree : surface.VDegree);
        for (int knotIdx = 1; knotIdx < curve.KnotCount; ++knotIdx)
        {
            double first = curve.Knot(knotIdx);
            double last = curve.Knot(knotIdx + 1);
            if (last - first <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            // The sampled difference combines the curve and surface polynomial orders.
            for (int sampleIdx = 0; sampleIdx <= samplesPerSpan; ++sampleIdx)
            {
                double ratio = (double)sampleIdx / samplesPerSpan;
                evaluate(first + ratio * (last - first));
            }
        }
        return maxDeviation;
    }

    // Returns the largest profile-to-surface deviation over all profile isoparameters.
    private static double MaxProfileDeviation(BSplineSurface surface,
                                              NCollection_Array1<BSplineCurve3D?> profiles,
                                              math_Vector profileParams)
    {
        double maxDeviation = 0.0;
        for (int profileIdx = 0; profileIdx < profiles.Size(); ++profileIdx)
        {
            double v = SurfaceParameter(surface, false, profileParams[profileIdx + 1]);
            maxDeviation = Max(maxDeviation, MaxCurveDeviation(surface, profiles.At(profileIdx)!, v, true));
        }
        return maxDeviation;
    }

    // Returns the largest guide-to-surface deviation over all guide isoparameters.
    private static double MaxGuideDeviation(BSplineSurface surface,
                                            NCollection_Array1<BSplineCurve3D?> guides,
                                            math_Vector guideParams)
    {
        double maxDeviation = 0.0;
        for (int guideIdx = 0; guideIdx < guides.Size(); ++guideIdx)
        {
            double u = SurfaceParameter(surface, true, guideParams[guideIdx + 1]);
            maxDeviation = Max(maxDeviation, MaxCurveDeviation(surface, guides.At(guideIdx)!, u, false));
        }
        return maxDeviation;
    }

    /// <summary>Performs the Gordon surface construction.</summary>
    public void Perform()
    {
        ResetReport(ref _report);
        _surface = null;
        _isUClosed = false;
        _isVClosed = false;

        if (_inputProfiles.Size() < 2 || _inputGuides.Size() < 2)
        {
            SetStatus(ref _report, ResultStatus.InvalidInput, BuildStage.InputConversion);
            return;
        }

        _profiles = new NCollection_Array1<BSplineCurve3D?>(1, _inputProfiles.Length());
        _guides = new NCollection_Array1<BSplineCurve3D?>(1, _inputGuides.Length());
        if (!CopyWorkingCurves(_inputProfiles, _profiles) || !CopyWorkingCurves(_inputGuides, _guides))
        {
            SetStatus(ref _report, ResultStatus.ConversionFailed, BuildStage.InputConversion);
            return;
        }

        _profileParams = new NCollection_Array2<double>(1, _inputProfiles.Length(), 1, _inputGuides.Length());
        _guideParams = new NCollection_Array2<double>(1, _inputProfiles.Length(), 1, _inputGuides.Length());

        global::Kerf.Engine.Construction.Surfaces.GordonUtilities.NetworkPreparation network = new(_profiles, _guides, _profileParams, _guideParams,
                                                          _tolerance, _toUseParallel);

        if (!network.ConvertCurvesToUnitInterval())
        {
            SetStatus(ref _report, ResultStatus.ConversionFailed, BuildStage.InputConversion);
            return;
        }

        if (!network.FillIntersectionParameters())
        {
            SetStatus(ref _report, ResultStatus.IntersectionFailed, BuildStage.ContactDiscovery);
            return;
        }
        _report.MaxContactGap = network.MaxContactGap();

        if (!network.ReorderNetwork())
        {
            SetStatus(ref _report, ResultStatus.OrderingFailed, BuildStage.NetworkOrdering);
            return;
        }
        if (!network.NormalizeIntersectionParameters(ref _isUClosed, ref _isVClosed))
        {
            SetStatus(ref _report, ResultStatus.CompatibilityFailed, BuildStage.NetworkOrdering);
            return;
        }
        _isUClosed = _isUClosed || AreClosedCurves(_profiles, _tolerance);
        bool hasApproximateReparametrization = false;
        if (!network.EqualizeIntersectionParameters(_approximationMode == ApproximationMode.AllowApproximateFallback,
                                                     ref hasApproximateReparametrization))
        {
            SetStatus(ref _report,
                      HasRationalCurve(_profiles, _guides) ? ResultStatus.RationalReparametrizationFailed
                                                             : ResultStatus.ReparametrizationFailed,
                      BuildStage.Reparametrization);
            return;
        }
        _report.MaxReparametrizationDeviation = network.MaxReparametrizationDeviation();

        if (!network.NormalizeIntersectionParameters(ref _isUClosed, ref _isVClosed))
        {
            SetStatus(ref _report, ResultStatus.CompatibilityFailed, BuildStage.Reparametrization);
            return;
        }
        _isUClosed = _isUClosed || AreClosedCurves(_profiles, _tolerance);

        Matrix profileParamMatrix = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ToMatrix(_profileParams);
        Matrix guideParamMatrix = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ToMatrix(_guideParams);

        math_Vector guideParamValues = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.ColumnMeans(profileParamMatrix);
        math_Vector profileParamValues = global::Kerf.Engine.Construction.Surfaces.GordonUtilities.RowMeans(guideParamMatrix);

        NetworkSurface networkSurface = new();
        networkSurface.Init(_profiles, _guides, profileParamValues.Array1(), guideParamValues.Array1(),
                             network.IntersectionPoints(), network.IntersectionWeights(), _tolerance, _isUClosed,
                             _isVClosed);
        networkSurface.Perform();

        if (!networkSurface.IsDone)
        {
            ResultStatus exactStatus = MapNetworkStatus(networkSurface.Status());
            if (_approximationMode == ApproximationMode.AllowApproximateFallback
                && CanApproximateAfterExactFailure(exactStatus))
            {
                _surface = ApproximatePreparedNetwork(_profiles, _guides, network.IntersectionPoints(), _tolerance);
                if (_surface is not null)
                {
                    _report.IsApproximate = true;
                    _report.MaxApproximationDeviation =
                      Max(MaxProfileDeviation(_surface, _profiles, profileParamValues),
                          MaxGuideDeviation(_surface, _guides, guideParamValues));
                    SetStatus(ref _report, ResultStatus.Done, BuildStage.Approximation);
                    return;
                }
                SetStatus(ref _report, ResultStatus.ApproximationFailed, BuildStage.Approximation);
                return;
            }
            SetStatus(ref _report, exactStatus, BuildStage.ExactConstruction);
            return;
        }

        _surface = networkSurface.Surface();
        _report.MaxProfileDeviation = MaxProfileDeviation(_surface, _profiles, profileParamValues);
        _report.MaxGuideDeviation = MaxGuideDeviation(_surface, _guides, guideParamValues);
        _report.IsApproximate = hasApproximateReparametrization;
        _report.MaxApproximationDeviation =
          _report.IsApproximate ? Max(_report.MaxProfileDeviation, _report.MaxGuideDeviation) : 0.0;
        SetStatus(ref _report, ResultStatus.Done, BuildStage.Validation);
    }

    /// <summary>Creates an empty Gordon surface algorithm.</summary>
    public Gordon()
    {
    }

    /// <summary>
    /// Initializes the algorithm with profile curves (V-direction sections, at least two), guide curves
    /// (U-direction sections, at least two) and the geometric tolerance of intersection detection. The curves are
    /// converted to B-splines of the algorithm's own; the caller's are not changed.
    /// </summary>
    public void Init(NCollection_Array1<Curve3D> profiles, NCollection_Array1<Curve3D> guides,
                     double tolerance)
    {
        _tolerance = tolerance;
        _isUClosed = false;
        _isVClosed = false;
        ResetReport(ref _report);
        _surface = null;

        int nbProf = profiles.Size();
        int nbGuid = guides.Size();

        if (nbProf < 2 || nbGuid < 2)
        {
            // Store counts but don't try to convert - Perform() will check and return.
            _inputProfiles = new NCollection_Array1<BSplineCurve3D?>(1, nbProf);
            _inputGuides = new NCollection_Array1<BSplineCurve3D?>(1, nbGuid);
            return;
        }

        // Convert input arrays to immutable B-spline seeds. Perform() deep-copies these
        // seeds before applying destructive preparation steps.
        _inputProfiles = new NCollection_Array1<BSplineCurve3D?>(1, nbProf);
        _inputGuides = new NCollection_Array1<BSplineCurve3D?>(1, nbGuid);

        for (int profileIdx = 0; profileIdx < nbProf; ++profileIdx)
        {
            _inputProfiles.ChangeAt(profileIdx) = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(profiles.At(profileIdx));
        }
        for (int guideIdx = 0; guideIdx < nbGuid; ++guideIdx)
        {
            _inputGuides.ChangeAt(guideIdx) = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(guides.At(guideIdx));
        }

        // Initialize parameter matrices.
        _profileParams = new NCollection_Array2<double>(1, nbProf, 1, nbGuid);
        _guideParams = new NCollection_Array2<double>(1, nbProf, 1, nbGuid);
    }

    /// <summary>Enables/disables parallel processing in internal stages (single-thread by default).</summary>
    public void SetParallelMode(bool toUseParallel) => _toUseParallel = toUseParallel;

    /// <summary>Sets optional fallback behavior for failures in exact B-spline construction.</summary>
    public void SetApproximationMode(ApproximationMode mode) => _approximationMode = mode;

    /// <summary>Returns current fallback behavior.</summary>
    public ApproximationMode GetApproximationMode() => _approximationMode;

    /// <summary>Returns true if internal parallel processing is enabled.</summary>
    public bool IsParallelMode => _toUseParallel;

    /// <summary>Returns true if the surface was successfully constructed.</summary>
    public bool IsDone => _report.Status == ResultStatus.Done;

    /// <summary>Returns true if the resulting surface was produced by approximate fallback.</summary>
    public bool IsApproximate => _report.IsApproximate;

    /// <summary>Returns the result state of the last Perform() call.</summary>
    public ResultStatus Status() => _report.Status;

    /// <summary>Returns diagnostics for the last Perform() call.</summary>
    public BuildReport Report() => _report;

    /// <summary>The Gordon surface, frozen. Throws StdFail_NotDone unless <see cref="Perform"/> succeeded.</summary>
    public BSplineSurface Surface()
    {
        if (!IsDone)
        {
            throw new NotDoneException("GeomFill_Gordon::Surface");
        }
        _surface!.Freeze();
        return _surface;
    }
}
