// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Gordon.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Gordon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Approx_ParametrizationType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A Gordon surface (transfinite interpolation) through a network of intersecting profile and guide curves: the
/// curves are converted to B-splines, periodic ones expanded, their contacts found, the network ordered and the
/// curves reparametrised to shared parameters, then <see cref="GeomFill_NetworkSurface"/> builds the surface. It
/// generalises GeomFill_Coons (four boundaries) to N x M networks.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> converts the caller's curves into B-splines of its own
/// (GeomConvert copies) and <see cref="Perform"/> works on further copies, so the caller's curves are never changed;
/// <see cref="Surface"/> hands out the surface frozen. <see cref="SetParallelMode"/> runs the contact search in
/// parallel; the result is the serial one (decision 490).
/// </remarks>
public class GeomFill_Gordon
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

    private NCollection_Array1<Geom_BSplineCurve?> myInputProfiles = new();
    private NCollection_Array1<Geom_BSplineCurve?> myInputGuides = new();
    private NCollection_Array1<Geom_BSplineCurve?> myProfiles = new();
    private NCollection_Array1<Geom_BSplineCurve?> myGuides = new();
    private NCollection_Array2<double> myProfileParams = new();
    private NCollection_Array2<double> myGuideParams = new();
    private Geom_BSplineSurface? mySurface;
    private double myTolerance = 0.0;
    private bool myIsUClosed = false;
    private bool myIsVClosed = false;
    private bool myToUseParallel = false;
    private ApproximationMode myApproximationMode = ApproximationMode.ExactOnly;
    private BuildReport myReport = new();

    // Returns whether at least one prepared curve uses rational weights.
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

    // Returns whether either prepared curve family uses rational weights.
    private static bool hasRationalCurve(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                         NCollection_Array1<Geom_BSplineCurve?> theGuides)
    {
        return hasRationalCurve(theProfiles) || hasRationalCurve(theGuides);
    }

    // Converts detailed network-construction statuses to the public Gordon status.
    private static ResultStatus mapNetworkStatus(GeomFill_NetworkSurface.ResultStatus theStatus)
    {
        switch (theStatus)
        {
            case GeomFill_NetworkSurface.ResultStatus.Done:
                return ResultStatus.Done;
            case GeomFill_NetworkSurface.ResultStatus.InvalidInput:
                return ResultStatus.CompatibilityFailed;
            case GeomFill_NetworkSurface.ResultStatus.CurveCompatibilityFailed:
                return ResultStatus.CurveCompatibilityFailed;
            case GeomFill_NetworkSurface.ResultStatus.SkinningFailed:
                return ResultStatus.SkinningFailed;
            case GeomFill_NetworkSurface.ResultStatus.ReferenceSurfaceFailed:
                return ResultStatus.ReferenceSurfaceFailed;
            case GeomFill_NetworkSurface.ResultStatus.KnotAlignmentFailed:
                return ResultStatus.KnotAlignmentFailed;
            case GeomFill_NetworkSurface.ResultStatus.RationalDegreeOverflow:
                return ResultStatus.RationalDegreeOverflow;
            case GeomFill_NetworkSurface.ResultStatus.RationalConstructionFailed:
                return ResultStatus.RationalConstructionFailed;
            case GeomFill_NetworkSurface.ResultStatus.PeriodicityFailed:
                return ResultStatus.PeriodicityFailed;
            case GeomFill_NetworkSurface.ResultStatus.NotStarted:
            case GeomFill_NetworkSurface.ResultStatus.ConstructionFailed:
                return ResultStatus.ConstructionFailed;
        }
        return ResultStatus.ConstructionFailed;
    }

    // Returns true when a sampled fallback can still use a validated prepared network.
    private static bool canApproximateAfterExactFailure(ResultStatus theStatus)
    {
        return theStatus == ResultStatus.CurveCompatibilityFailed || theStatus == ResultStatus.SkinningFailed
               || theStatus == ResultStatus.ReferenceSurfaceFailed || theStatus == ResultStatus.KnotAlignmentFailed
               || theStatus == ResultStatus.RationalDegreeOverflow
               || theStatus == ResultStatus.RationalConstructionFailed
               || theStatus == ResultStatus.ConstructionFailed;
    }

    // Chooses a deterministic fallback grid density from the prepared network itself.
    private static int fallbackSampleCount(NCollection_Array1<Geom_BSplineCurve?> theCurves)
    {
        int aNbSamples = Max(theCurves.Size(), 2);
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            if (theCurves.At(aCurveIdx) is not null)
            {
                aNbSamples = Max(aNbSamples, theCurves.At(aCurveIdx)!.NbPoles());
            }
        }
        return Max(aNbSamples, Geom_BSplineCurve.MaxDegree() / 2);
    }

    // Returns regularly spaced parameters in the unit interval.
    private static NCollection_Array1<double> unitParameters(int theNbSamples)
    {
        NCollection_Array1<double> aParameters = new(1, theNbSamples);
        for (int aSampleIdx = 0; aSampleIdx < theNbSamples; ++aSampleIdx)
        {
            aParameters.ChangeAt(aSampleIdx) = (double)aSampleIdx / (double)(theNbSamples - 1);
        }
        return aParameters;
    }

    // Fits a non-exact B-spline surface through a point grid for approximate fallback.
    private static Geom_BSplineSurface? approximateGrid(NCollection_Array2<gp_Pnt> thePoints, double theTolerance)
    {
        GeomAPI_PointsToBSplineSurface anApprox = new();
        anApprox.Init(thePoints, Approx_IsoParametric, 1, Geom_BSplineSurface.MaxDegree(), GeomAbs_C1,
                      Max(theTolerance, Precision.Confusion()));
        return anApprox.IsDone() ? anApprox.Surface() : null;
    }

    // Samples the prepared profiles as section curves of an approximate profile skin.
    private static NCollection_Array2<gp_Pnt> profileSampleGrid(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                                int theNbUSamples)
    {
        NCollection_Array1<double> aUParams = unitParameters(theNbUSamples);
        NCollection_Array2<gp_Pnt> aPoints = new(1, theNbUSamples, 1, theProfiles.Size());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            GeomGridEval_Curve anEvaluator = new(theProfiles.At(aProfileIdx));
            NCollection_Array1<gp_Pnt> aProfilePoints = anEvaluator.EvaluateGrid(aUParams);
            for (int aUIdx = 0; aUIdx < theNbUSamples; ++aUIdx)
            {
                aPoints.ChangeAt(aUIdx, aProfileIdx) = aProfilePoints.At(aUIdx);
            }
        }
        return aPoints;
    }

    // Samples the prepared guides as section curves of an approximate guide skin.
    private static NCollection_Array2<gp_Pnt> guideSampleGrid(NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                                              int theNbVSamples)
    {
        NCollection_Array1<double> aVParams = unitParameters(theNbVSamples);
        NCollection_Array2<gp_Pnt> aPoints = new(1, theGuides.Size(), 1, theNbVSamples);
        for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
        {
            GeomGridEval_Curve anEvaluator = new(theGuides.At(aGuideIdx));
            NCollection_Array1<gp_Pnt> aGuidePoints = anEvaluator.EvaluateGrid(aVParams);
            for (int aVIdx = 0; aVIdx < theNbVSamples; ++aVIdx)
            {
                aPoints.ChangeAt(aGuideIdx, aVIdx) = aGuidePoints.At(aVIdx);
            }
        }
        return aPoints;
    }

    // Builds a sampled fallback from approximate profile, guide, and reference surfaces.
    private static Geom_BSplineSurface? approximatePreparedNetwork(NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                                                   NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                                                   NCollection_Array2<gp_Pnt> theIntersections,
                                                                   double theTolerance)
    {
        int aNbUSamples = fallbackSampleCount(theProfiles);
        int aNbVSamples = fallbackSampleCount(theGuides);

        Geom_BSplineSurface? aProfileSurface = approximateGrid(profileSampleGrid(theProfiles, aNbUSamples), theTolerance);
        Geom_BSplineSurface? aGuideSurface = approximateGrid(guideSampleGrid(theGuides, aNbVSamples), theTolerance);
        Geom_BSplineSurface? aReferenceSurface = approximateGrid(theIntersections, theTolerance);
        if (aProfileSurface is null || aGuideSurface is null || aReferenceSurface is null)
        {
            return null;
        }
        NCollection_Array2<gp_Pnt> aPoints = new(1, aNbUSamples, 1, aNbVSamples);
        NCollection_Array1<double> aUParams = unitParameters(aNbUSamples);
        NCollection_Array1<double> aVParams = unitParameters(aNbVSamples);
        GeomGridEval_Surface aProfileEvaluator = new(aProfileSurface);
        GeomGridEval_Surface aGuideEvaluator = new(aGuideSurface);
        GeomGridEval_Surface aReferenceEvaluator = new(aReferenceSurface);
        NCollection_Array2<gp_Pnt> aProfilePoints = aProfileEvaluator.EvaluateGrid(aUParams, aVParams);
        NCollection_Array2<gp_Pnt> aGuidePoints = aGuideEvaluator.EvaluateGrid(aUParams, aVParams);
        NCollection_Array2<gp_Pnt> aReferencePoints = aReferenceEvaluator.EvaluateGrid(aUParams, aVParams);
        for (int aUIdx = 0; aUIdx < aNbUSamples; ++aUIdx)
        {
            for (int aVIdx = 0; aVIdx < aNbVSamples; ++aVIdx)
            {
                gp_XYZ aPoint = aProfilePoints.At(aUIdx, aVIdx).XYZ() + aGuidePoints.At(aUIdx, aVIdx).XYZ()
                                - aReferencePoints.At(aUIdx, aVIdx).XYZ();
                aPoints.ChangeAt(aUIdx, aVIdx) = new gp_Pnt(aPoint);
            }
        }
        return approximateGrid(aPoints, theTolerance);
    }

    // Resets all report fields before starting a construction attempt.
    private static void resetReport(ref BuildReport theReport)
    {
        theReport = new BuildReport();
    }

    // Records the result status and its failing construction stage.
    private static void setStatus(ref BuildReport theReport, ResultStatus theNewStatus, BuildStage theStage)
    {
        theReport.Status = theNewStatus;
        theReport.FailedStage = theNewStatus == ResultStatus.Done ? BuildStage.NotStarted : theStage;
    }

    // Returns whether every profile closes geometrically at its parameter endpoints.
    private static bool areClosedCurves(NCollection_Array1<Geom_BSplineCurve?> theCurves, double theTolerance)
    {
        if (theCurves.IsEmpty())
        {
            return false;
        }
        double aTolerance = Max(theTolerance, Precision.Confusion());
        for (int aCurveIdx = 0; aCurveIdx < theCurves.Size(); ++aCurveIdx)
        {
            Geom_BSplineCurve? aCurve = theCurves.At(aCurveIdx);
            if (aCurve is null
                || aCurve.Value(aCurve.FirstParameter()).Distance(aCurve.Value(aCurve.LastParameter())) > aTolerance)
            {
                return false;
            }
        }
        return true;
    }

    // Clones input curves because network preparation modifies its working copies.
    private static bool copyWorkingCurves(NCollection_Array1<Geom_BSplineCurve?> theInput,
                                          NCollection_Array1<Geom_BSplineCurve?> theWorking)
    {
        try
        {
            for (int aCurveIdx = 0; aCurveIdx < theInput.Size(); ++aCurveIdx)
            {
                if (theInput.At(aCurveIdx) is null)
                {
                    return false;
                }
                theWorking.ChangeAt(aCurveIdx) = theInput.At(aCurveIdx)!.Copy() as Geom_BSplineCurve;
                if (theWorking.At(aCurveIdx) is null)
                {
                    return false;
                }
            }
        }
        catch (Standard_Failure)
        {
            return false;
        }
        return true;
    }

    // Returns the first knot in the requested surface parameter direction.
    private static double surfaceStartParameter(Geom_BSplineSurface theSurface, bool theIsU)
    {
        return theIsU ? theSurface.UKnot(1) : theSurface.VKnot(1);
    }

    // Returns the last knot in the requested surface parameter direction.
    private static double surfaceEndParameter(Geom_BSplineSurface theSurface, bool theIsU)
    {
        return theIsU ? theSurface.UKnot(theSurface.NbUKnots()) : theSurface.VKnot(theSurface.NbVKnots());
    }

    // Maps a unit parameter to the knot range of the requested surface direction.
    private static double surfaceParameter(Geom_BSplineSurface theSurface, bool theIsU, double theUnitParameter)
    {
        double aFirst = surfaceStartParameter(theSurface, theIsU);
        return aFirst + theUnitParameter * (surfaceEndParameter(theSurface, theIsU) - aFirst);
    }

    // Measures the maximum deviation of a curve from its corresponding surface isoparameter.
    private static double maxCurveDeviation(Geom_BSplineSurface theSurface, Geom_BSplineCurve theCurve,
                                            double theFixedParameter, bool theIsProfile)
    {
        GeomAdaptor_Curve anAdaptor = new(theCurve);
        double aMaxDeviation = 0.0;
        void evaluate(double theParameter)
        {
            double aU = theIsProfile ? surfaceParameter(theSurface, true, theParameter)
                                     : surfaceParameter(theSurface, true, theFixedParameter);
            double aV = theIsProfile ? surfaceParameter(theSurface, false, theFixedParameter)
                                     : surfaceParameter(theSurface, false, theParameter);
            aMaxDeviation = Max(aMaxDeviation, anAdaptor.EvalD0(theParameter).Distance(theSurface.Value(aU, aV)));
        }

        int aSamplesPerSpan = theCurve.Degree() + (theIsProfile ? theSurface.UDegree() : theSurface.VDegree());
        for (int aKnotIdx = 1; aKnotIdx < theCurve.NbKnots(); ++aKnotIdx)
        {
            double aFirst = theCurve.Knot(aKnotIdx);
            double aLast = theCurve.Knot(aKnotIdx + 1);
            if (aLast - aFirst <= Precision.PConfusion())
            {
                continue;
            }

            // The sampled difference combines the curve and surface polynomial orders.
            for (int aSampleIdx = 0; aSampleIdx <= aSamplesPerSpan; ++aSampleIdx)
            {
                double aRatio = (double)aSampleIdx / aSamplesPerSpan;
                evaluate(aFirst + aRatio * (aLast - aFirst));
            }
        }
        return aMaxDeviation;
    }

    // Returns the largest profile-to-surface deviation over all profile isoparameters.
    private static double maxProfileDeviation(Geom_BSplineSurface theSurface,
                                              NCollection_Array1<Geom_BSplineCurve?> theProfiles,
                                              math_Vector theProfileParams)
    {
        double aMaxDeviation = 0.0;
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            double aV = surfaceParameter(theSurface, false, theProfileParams[aProfileIdx + 1]);
            aMaxDeviation = Max(aMaxDeviation, maxCurveDeviation(theSurface, theProfiles.At(aProfileIdx)!, aV, true));
        }
        return aMaxDeviation;
    }

    // Returns the largest guide-to-surface deviation over all guide isoparameters.
    private static double maxGuideDeviation(Geom_BSplineSurface theSurface,
                                            NCollection_Array1<Geom_BSplineCurve?> theGuides,
                                            math_Vector theGuideParams)
    {
        double aMaxDeviation = 0.0;
        for (int aGuideIdx = 0; aGuideIdx < theGuides.Size(); ++aGuideIdx)
        {
            double aU = surfaceParameter(theSurface, true, theGuideParams[aGuideIdx + 1]);
            aMaxDeviation = Max(aMaxDeviation, maxCurveDeviation(theSurface, theGuides.At(aGuideIdx)!, aU, false));
        }
        return aMaxDeviation;
    }

    /// <summary>Performs the Gordon surface construction.</summary>
    public void Perform()
    {
        resetReport(ref myReport);
        mySurface = null;
        myIsUClosed = false;
        myIsVClosed = false;

        if (myInputProfiles.Size() < 2 || myInputGuides.Size() < 2)
        {
            setStatus(ref myReport, ResultStatus.InvalidInput, BuildStage.InputConversion);
            return;
        }

        myProfiles = new NCollection_Array1<Geom_BSplineCurve?>(1, myInputProfiles.Length());
        myGuides = new NCollection_Array1<Geom_BSplineCurve?>(1, myInputGuides.Length());
        if (!copyWorkingCurves(myInputProfiles, myProfiles) || !copyWorkingCurves(myInputGuides, myGuides))
        {
            setStatus(ref myReport, ResultStatus.ConversionFailed, BuildStage.InputConversion);
            return;
        }

        myProfileParams = new NCollection_Array2<double>(1, myInputProfiles.Length(), 1, myInputGuides.Length());
        myGuideParams = new NCollection_Array2<double>(1, myInputProfiles.Length(), 1, myInputGuides.Length());

        GordonUtilities.NetworkPreparation aNetwork = new(myProfiles, myGuides, myProfileParams, myGuideParams,
                                                          myTolerance, myToUseParallel);

        if (!aNetwork.ConvertCurvesToUnitInterval())
        {
            setStatus(ref myReport, ResultStatus.ConversionFailed, BuildStage.InputConversion);
            return;
        }

        if (!aNetwork.FillIntersectionParameters())
        {
            setStatus(ref myReport, ResultStatus.IntersectionFailed, BuildStage.ContactDiscovery);
            return;
        }
        myReport.MaxContactGap = aNetwork.MaxContactGap();

        if (!aNetwork.ReorderNetwork())
        {
            setStatus(ref myReport, ResultStatus.OrderingFailed, BuildStage.NetworkOrdering);
            return;
        }
        if (!aNetwork.NormalizeIntersectionParameters(ref myIsUClosed, ref myIsVClosed))
        {
            setStatus(ref myReport, ResultStatus.CompatibilityFailed, BuildStage.NetworkOrdering);
            return;
        }
        myIsUClosed = myIsUClosed || areClosedCurves(myProfiles, myTolerance);
        bool aHasApproximateReparametrization = false;
        if (!aNetwork.EqualizeIntersectionParameters(myApproximationMode == ApproximationMode.AllowApproximateFallback,
                                                     ref aHasApproximateReparametrization))
        {
            setStatus(ref myReport,
                      hasRationalCurve(myProfiles, myGuides) ? ResultStatus.RationalReparametrizationFailed
                                                             : ResultStatus.ReparametrizationFailed,
                      BuildStage.Reparametrization);
            return;
        }
        myReport.MaxReparametrizationDeviation = aNetwork.MaxReparametrizationDeviation();

        if (!aNetwork.NormalizeIntersectionParameters(ref myIsUClosed, ref myIsVClosed))
        {
            setStatus(ref myReport, ResultStatus.CompatibilityFailed, BuildStage.Reparametrization);
            return;
        }
        myIsUClosed = myIsUClosed || areClosedCurves(myProfiles, myTolerance);

        math_Matrix aProfileParamMatrix = GordonUtilities.toMatrix(myProfileParams);
        math_Matrix aGuideParamMatrix = GordonUtilities.toMatrix(myGuideParams);

        math_Vector aGuideParamValues = GordonUtilities.columnMeans(aProfileParamMatrix);
        math_Vector aProfileParamValues = GordonUtilities.rowMeans(aGuideParamMatrix);

        GeomFill_NetworkSurface aNetworkSurface = new();
        aNetworkSurface.Init(myProfiles, myGuides, aProfileParamValues.Array1(), aGuideParamValues.Array1(),
                             aNetwork.IntersectionPoints(), aNetwork.IntersectionWeights(), myTolerance, myIsUClosed,
                             myIsVClosed);
        aNetworkSurface.Perform();

        if (!aNetworkSurface.IsDone())
        {
            ResultStatus anExactStatus = mapNetworkStatus(aNetworkSurface.Status());
            if (myApproximationMode == ApproximationMode.AllowApproximateFallback
                && canApproximateAfterExactFailure(anExactStatus))
            {
                mySurface = approximatePreparedNetwork(myProfiles, myGuides, aNetwork.IntersectionPoints(), myTolerance);
                if (mySurface is not null)
                {
                    myReport.IsApproximate = true;
                    myReport.MaxApproximationDeviation =
                      Max(maxProfileDeviation(mySurface, myProfiles, aProfileParamValues),
                          maxGuideDeviation(mySurface, myGuides, aGuideParamValues));
                    setStatus(ref myReport, ResultStatus.Done, BuildStage.Approximation);
                    return;
                }
                setStatus(ref myReport, ResultStatus.ApproximationFailed, BuildStage.Approximation);
                return;
            }
            setStatus(ref myReport, anExactStatus, BuildStage.ExactConstruction);
            return;
        }

        mySurface = aNetworkSurface.Surface();
        myReport.MaxProfileDeviation = maxProfileDeviation(mySurface, myProfiles, aProfileParamValues);
        myReport.MaxGuideDeviation = maxGuideDeviation(mySurface, myGuides, aGuideParamValues);
        myReport.IsApproximate = aHasApproximateReparametrization;
        myReport.MaxApproximationDeviation =
          myReport.IsApproximate ? Max(myReport.MaxProfileDeviation, myReport.MaxGuideDeviation) : 0.0;
        setStatus(ref myReport, ResultStatus.Done, BuildStage.Validation);
    }

    /// <summary>Creates an empty Gordon surface algorithm.</summary>
    public GeomFill_Gordon()
    {
    }

    /// <summary>
    /// Initializes the algorithm with profile curves (V-direction sections, at least two), guide curves
    /// (U-direction sections, at least two) and the geometric tolerance of intersection detection. The curves are
    /// converted to B-splines of the algorithm's own; the caller's are not changed.
    /// </summary>
    public void Init(NCollection_Array1<Geom_Curve> theProfiles, NCollection_Array1<Geom_Curve> theGuides,
                     double theTolerance)
    {
        myTolerance = theTolerance;
        myIsUClosed = false;
        myIsVClosed = false;
        resetReport(ref myReport);
        mySurface = null;

        int aNbProf = theProfiles.Size();
        int aNbGuid = theGuides.Size();

        if (aNbProf < 2 || aNbGuid < 2)
        {
            // Store counts but don't try to convert - Perform() will check and return.
            myInputProfiles = new NCollection_Array1<Geom_BSplineCurve?>(1, aNbProf);
            myInputGuides = new NCollection_Array1<Geom_BSplineCurve?>(1, aNbGuid);
            return;
        }

        // Convert input arrays to immutable B-spline seeds. Perform() deep-copies these
        // seeds before applying destructive preparation steps.
        myInputProfiles = new NCollection_Array1<Geom_BSplineCurve?>(1, aNbProf);
        myInputGuides = new NCollection_Array1<Geom_BSplineCurve?>(1, aNbGuid);

        for (int aProfileIdx = 0; aProfileIdx < aNbProf; ++aProfileIdx)
        {
            myInputProfiles.ChangeAt(aProfileIdx) = GeomConvert.CurveToBSplineCurve(theProfiles.At(aProfileIdx));
        }
        for (int aGuideIdx = 0; aGuideIdx < aNbGuid; ++aGuideIdx)
        {
            myInputGuides.ChangeAt(aGuideIdx) = GeomConvert.CurveToBSplineCurve(theGuides.At(aGuideIdx));
        }

        // Initialize parameter matrices.
        myProfileParams = new NCollection_Array2<double>(1, aNbProf, 1, aNbGuid);
        myGuideParams = new NCollection_Array2<double>(1, aNbProf, 1, aNbGuid);
    }

    /// <summary>Enables/disables parallel processing in internal stages (single-thread by default).</summary>
    public void SetParallelMode(bool theToUseParallel) => myToUseParallel = theToUseParallel;

    /// <summary>Sets optional fallback behavior for failures in exact B-spline construction.</summary>
    public void SetApproximationMode(ApproximationMode theMode) => myApproximationMode = theMode;

    /// <summary>Returns current fallback behavior.</summary>
    public ApproximationMode GetApproximationMode() => myApproximationMode;

    /// <summary>Returns true if internal parallel processing is enabled.</summary>
    public bool IsParallelMode() => myToUseParallel;

    /// <summary>Returns true if the surface was successfully constructed.</summary>
    public bool IsDone() => myReport.Status == ResultStatus.Done;

    /// <summary>Returns true if the resulting surface was produced by approximate fallback.</summary>
    public bool IsApproximate() => myReport.IsApproximate;

    /// <summary>Returns the result state of the last Perform() call.</summary>
    public ResultStatus Status() => myReport.Status;

    /// <summary>Returns diagnostics for the last Perform() call.</summary>
    public BuildReport Report() => myReport;

    /// <summary>The Gordon surface, frozen. Throws StdFail_NotDone unless <see cref="Perform"/> succeeded.</summary>
    public Geom_BSplineSurface Surface()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone("GeomFill_Gordon::Surface");
        }
        mySurface!.Freeze();
        return mySurface;
    }
}
