// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NURBSRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NURBSRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Extends the UV parameter range splitter to generate the internal nodes of a B-spline or Bezier surface (and,
/// through its subclasses, of extrusions and other surfaces): a grid over the surface's continuity intervals, the
/// parameters of the internal edges, filtered by the deflections.
/// </summary>
/// <remarks>
/// OCCT's <c>IMeshData::SequenceOfReal</c> is <c>NCollection_Sequence&lt;double&gt;</c>, <c>MapOfReal</c>
/// <c>NCollection_Map&lt;double&gt;</c> and <c>IMapOfReal</c> <c>NCollection_IndexedMap&lt;double&gt;</c>. The
/// <c>std::sort</c> of the parameters sorts doubles that the map made distinct, so <c>Span.Sort</c> gives
/// <c>std::sort</c>'s order.
/// </remarks>
public class BRepMesh_NURBSRangeSplitter : BRepMesh_UVParamRangeSplitter
{
    private SurfaceKind mySurfaceType;

    /// <summary>A splitter.</summary>
    public BRepMesh_NURBSRangeSplitter()
    {
        mySurfaceType = OtherSurface;
    }

    /// <summary>Updates the discrete range of the surface from its geometric range.</summary>
    public override void AdjustRange()
    {
        base.AdjustRange();
        mySurfaceType = GetSurface().GetType();

        if (mySurfaceType == BezierSurface)
        {
            (double first, double second) aRangeU = GetRangeU();
            (double first, double second) aRangeV = GetRangeV();

            myIsValid = aRangeU.first >= -0.5 && aRangeU.second <= 1.5 && aRangeV.first >= -0.5
                        && aRangeV.second <= 1.5;
        }
    }

    /// <summary>The nodes generated from the surface and the parameters; null when there are no parameters.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        if (!initParameters())
        {
            return null;
        }

        (double first, double second) aRangeU = GetRangeU();
        (double first, double second) aRangeV = GetRangeV();
        (double first, double second) aDelta = GetDelta();

        double aDefFace = GetDFace().GetDeflection();
        BRepAdaptor_Surface gFace = GetSurface();

        NCollection_Sequence<double>[] aParams =
        [
            computeGrainAndFilterParameters(GetParametersU(),
                                            gFace.UResolution(aDefFace),
                                            (aRangeU.second - aRangeU.first),
                                            aDelta.first,
                                            theParameters),

            computeGrainAndFilterParameters(GetParametersV(),
                                            gFace.VResolution(aDefFace),
                                            (aRangeV.second - aRangeV.first),
                                            aDelta.second,
                                            theParameters),
        ];

        // check intermediate isolines
        NCollection_Map<double>[] aFixedParams = [new NCollection_Map<double>(), new NCollection_Map<double>()];

        NCollection_Map<double>[] aParamsToRemove =
        [
            new AnalyticalFilter(GetDFace(),
                                 IsoV,
                                 aParams[1],
                                 aParams[0],
                                 aFixedParams[1],
                                 aFixedParams[0])
                .GetControlParametersToRemove(theParameters),

            new AnalyticalFilter(GetDFace(),
                                 IsoU,
                                 aParams[0],
                                 aParams[1],
                                 aFixedParams[0],
                                 aFixedParams[1])
                .GetControlParametersToRemove(theParameters),
        ];

        NCollection_MapAlgo.Subtract(aParamsToRemove[0], aFixedParams[0]);
        NCollection_MapAlgo.Subtract(aParamsToRemove[1], aFixedParams[1]);

        // insert nodes of the regular grid
        NCollection_List<Point2D> aNodes = new();

        // insert nodes of the regular grid
        for (int i = 1; i <= aParams[0].Length(); ++i)
        {
            double aParam1 = aParams[0].Value(i);
            if (aParamsToRemove[0].Contains(aParam1))
            {
                continue;
            }

            for (int j = 1; j <= aParams[1].Length(); ++j)
            {
                double aParam2 = aParams[1].Value(j);
                if (aParamsToRemove[1].Contains(aParam2))
                {
                    continue;
                }

                aNodes.Append(new Point2D(aParam1, aParam2));
            }
        }

        return aNodes;
    }

    /// <summary>Initializes the U and V parameter lists from the CN continuity intervals.</summary>
    protected virtual bool initParameters()
    {
        Continuity aContinuity = CN;
        BRepAdaptor_Surface aSurface = GetSurface();

        NCollection_Array1<double>[] aIntervals = [new NCollection_Array1<double>(), new NCollection_Array1<double>()];
        getUndefinedInterval(aSurface, true, aContinuity, GetRangeU(), ref aIntervals[0]);
        getUndefinedInterval(aSurface, false, aContinuity, GetRangeV(), ref aIntervals[1]);

        bool isSplitIntervals = toSplitIntervals(aSurface.GeomSurfaceOriginal()!, aIntervals);

        if (!initParamsFromIntervals(aIntervals[0],
                                     GetRangeU(),
                                     isSplitIntervals,
                                     GetParametersU()))
        {
            // if (!grabParamsOfEdges (Edge_Frontier, Param_U))
            {
                return false;
            }
        }

        if (!initParamsFromIntervals(aIntervals[1],
                                     GetRangeV(),
                                     isSplitIntervals,
                                     GetParametersV()))
        {
            // if (!grabParamsOfEdges (Edge_Frontier, Param_V))
            {
                return false;
            }
        }

        return grabParamsOfEdges(EdgeType.Edge_Internal, (int)(ParamDimension.Param_U | ParamDimension.Param_V));
    }

    /// <summary>The number of intervals computed from the available geometrical parameters: the poles less one.</summary>
    protected virtual int getUndefinedIntervalNb(Adaptor3d_Surface theSurface,
                                                 bool isU,
                                                 Continuity theContinuity)
    {
        return (isU ? theSurface.NbUPoles() : theSurface.NbVPoles()) - 1;
    }

    // Tries to compute intervals even for cases with no intervals at all using available geometrical parameters.
    private void getUndefinedInterval(Adaptor3d_Surface theSurface,
                                      bool isU,
                                      Continuity theContinuity,
                                      (double first, double second) theRange,
                                      ref NCollection_Array1<double> theIntervals)
    {
        int aIntervalsNb =
            isU ? theSurface.NbUIntervals(theContinuity) : theSurface.NbVIntervals(theContinuity);

        if (aIntervalsNb == 1)
        {
            aIntervalsNb = getUndefinedIntervalNb(theSurface, isU, theContinuity);
            if (aIntervalsNb > 1)
            {
                theIntervals = new NCollection_Array1<double>(1, aIntervalsNb - 1);
                double aDiff = (theRange.second - theRange.first) / aIntervalsNb;
                for (int i = theIntervals.Lower(); i <= theIntervals.Upper(); ++i)
                {
                    theIntervals.SetValue(i, theRange.first + i * aDiff);
                }
            }
        }

        if (theIntervals.IsEmpty())
        {
            theIntervals = new NCollection_Array1<double>(1, aIntervalsNb + 1);
            if (isU)
            {
                theSurface.UIntervals(theIntervals, theContinuity);
            }
            else
            {
                theSurface.VIntervals(theIntervals, theContinuity);
            }
        }
    }

    // Computes the parameters of the filter and applies it to the source parameters.
    private NCollection_Sequence<double> computeGrainAndFilterParameters(NCollection_IndexedMap<double> theSourceParams,
                                                                        double theTol2d,
                                                                        double theRangeDiff,
                                                                        double theDelta,
                                                                        in IMeshTools_Parameters theParameters)
    {
        // Sort and filter sequence of parameters
        double aMinDiff = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (theDelta < 1.0)
        {
            aMinDiff /= theDelta;
        }

        BRepAdaptor_Surface aSurface = GetSurface();
        double aMinSize2d = Max(aSurface.UResolution(theParameters.MinSize),
                                aSurface.VResolution(theParameters.MinSize));

        aMinDiff = Max(aMinSize2d, aMinDiff);

        double aDiffMaxLim = 0.1 * theRangeDiff;
        double aDiffMinLim = Max(0.005 * theRangeDiff, 2.0 * theTol2d);
        double aDiff = Max(aMinSize2d, Min(aDiffMaxLim, aDiffMinLim));
        return filterParameters(theSourceParams, aMinDiff, aDiff);
    }

    // Filters the parameters to avoid a too dense distribution.
    private static NCollection_Sequence<double> filterParameters(NCollection_IndexedMap<double> theParams,
                                                                 double theMinDist,
                                                                 double theFilterDist)
    {
        NCollection_Sequence<double> aResult = new();

        // Sort sequence of parameters
        int anInitLen = theParams.Extent();

        if (anInitLen < 1)
        {
            return aResult;
        }

        NCollection_Array1<double> aParamArray = new(1, anInitLen);
        int j;
        for (j = 1; j <= anInitLen; j++)
        {
            aParamArray[j] = theParams[j];
        }

        aParamArray.AsSpan().Sort();

        // mandatory pre-filtering using the first (minimal) filter value
        int aParamLength = 1;
        for (j = 2; j <= anInitLen; j++)
        {
            if ((aParamArray[j] - aParamArray[aParamLength]) > theMinDist)
            {
                if (++aParamLength < j)
                {
                    aParamArray[aParamLength] = aParamArray[j];
                }
            }
        }

        // perform filtering on series
        double aLastAdded, aLastCandidate;
        bool isCandidateDefined = false;
        aLastAdded = aParamArray[1];
        aLastCandidate = aLastAdded;
        aResult.Append(aLastAdded);

        for (j = 2; j < aParamLength; j++)
        {
            double aVal = aParamArray[j];
            if (aVal - aLastAdded > theFilterDist)
            {
                // adds the parameter
                if (isCandidateDefined)
                {
                    aLastAdded = aLastCandidate;
                    isCandidateDefined = false;
                    j--;
                }
                else
                {
                    aLastAdded = aVal;
                }
                aResult.Append(aLastAdded);
                continue;
            }

            aLastCandidate = aVal;
            isCandidateDefined = true;
        }
        aResult.Append(aParamArray[aParamLength]);

        return aResult;
    }

    private enum EdgeType
    {
        Edge_Internal,
        Edge_Frontier,
    }

    [Flags]
    private enum ParamDimension
    {
        Param_U = 0x1,
        Param_V = 0x2,
    }

    // Finds the edges of the discrete face and uses their points as auxiliary control parameters for the nodes.
    private bool grabParamsOfEdges(EdgeType theEdgeType, int theParamDimensionFlag)
    {
        if ((theParamDimensionFlag & (int)(ParamDimension.Param_U | ParamDimension.Param_V)) == 0)
        {
            return false;
        }

        IMeshData_Face aDFace = GetDFace();
        for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
        {
            IMeshData_Wire aDWire = aDFace.GetWire(aWireIt);
            for (int aEdgeIt = 0; aEdgeIt < aDWire.EdgesNb(); ++aEdgeIt)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(aEdgeIt);
                for (int aPCurveIt = 0; aPCurveIt < aDEdge.PCurvesNb(); ++aPCurveIt)
                {
                    IMeshData_PCurve aDPCurve = aDEdge.GetPCurve(aPCurveIt);
                    if (ReferenceEquals(aDPCurve.GetFace(), aDFace))
                    {
                        if (theEdgeType == EdgeType.Edge_Internal && !aDPCurve.IsInternal())
                        {
                            continue;
                        }

                        for (int aPointIt = 0; aPointIt < aDPCurve.ParametersNb(); ++aPointIt)
                        {
                            Point2D aPnt2d = aDPCurve.GetPoint(aPointIt);
                            if ((theParamDimensionFlag & (int)ParamDimension.Param_U) != 0)
                            {
                                GetParametersU().Add(aPnt2d.X());
                            }

                            if ((theParamDimensionFlag & (int)ParamDimension.Param_V) != 0)
                            {
                                GetParametersV().Add(aPnt2d.Y());
                            }
                        }
                    }
                }
            }
        }

        return true;
    }

    private static bool addParam(double theParam,
                                 (double first, double second) theRange,
                                 NCollection_IndexedMap<double> theParams)
    {
        if (theParam < theRange.first || theParam > theRange.second)
        {
            return false;
        }

        theParams.Add(theParam);
        return true;
    }

    private static bool initParamsFromIntervals(NCollection_Array1<double> theIntervals,
                                                (double first, double second) theRange,
                                                bool isSplitIntervals,
                                                NCollection_IndexedMap<double> theParams)
    {
        bool isAdded = false;

        for (int i = theIntervals.Lower(); i <= theIntervals.Upper(); ++i)
        {
            double aStartParam = theIntervals.Value(i);
            if (addParam(aStartParam, theRange, theParams))
            {
                isAdded = true;
            }

            if (isSplitIntervals && i < theIntervals.Upper())
            {
                double aMidParam = (aStartParam + theIntervals.Value(i + 1)) / 2.0;
                if (addParam(aMidParam, theRange, theParams))
                {
                    isAdded = true;
                }
            }
        }

        return isAdded;
    }

    private static bool toSplitIntervals(Geom_Surface theSurf, NCollection_Array1<double>[] theIntervals)
    {
        int aIntervalU = theIntervals[0].Lower();
        for (; aIntervalU <= theIntervals[0].Upper(); ++aIntervalU)
        {
            double aParamU = theIntervals[0].Value(aIntervalU);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aParamU))
            {
                continue;
            }

            int aIntervalV = theIntervals[1].Lower();
            for (; aIntervalV <= theIntervals[1].Upper(); ++aIntervalV)
            {
                Direction3D aNorm = new();
                double aParamV = theIntervals[1].Value(aIntervalV);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aParamV))
                {
                    continue;
                }

                if (GeomLib.NormEstim(theSurf, new Point2D(aParamU, aParamV), global::Kerf.Engine.Foundation.Precision.Confusion(), ref aNorm)
                    != 0)
                {
                    return true;
                }
                // TODO: do not split intervals if there is no normal in the middle of interval.
            }
        }

        return false;
    }

    // Filters the control parameters of a set of isolines by the deflections (the anonymous namespace's
    // AnalyticalFilter). Both filters of GenerateSurfaceNodes share the parameter sequences, as OCCT's handles do.
    private sealed class AnalyticalFilter
    {
        private readonly IMeshData_Face myDFace;
        private readonly Geom_Surface mySurface;
        private readonly bool myIsoU;
        private readonly NCollection_Sequence<double> myParams;
        private readonly NCollection_Sequence<double> myControlParams;

        private readonly NCollection_Map<double> myParamsForbiddenToRemove;
        private readonly NCollection_Map<double> myControlParamsForbiddenToRemove;

        private readonly NCollection_Map<double> myControlParamsToRemove;

        private IMeshTools_Parameters myParameters = new();
        private GeomAdaptor_Curve? myIso;

        private double myCurrParam;

        private double myCurrControlParam;
        private Point3D myCurrControlPnt;
        private Vector3D myCurrControlVec;

        private double myPrevControlParam;
        private Point3D myPrevControlPnt;
        private Vector3D myPrevControlVec;

        public AnalyticalFilter(IMeshData_Face theDFace,
                                IsoType theIsoType,
                                NCollection_Sequence<double> theParams,
                                NCollection_Sequence<double> theControlParams,
                                NCollection_Map<double> theParamsForbiddenToRemove,
                                NCollection_Map<double> theControlParamsForbiddenToRemove)
        {
            myDFace = theDFace;
            mySurface = myDFace.GetSurface().GeomSurfaceOriginal()!;
            myIsoU = theIsoType == IsoU;
            myParams = theParams;
            myControlParams = theControlParams;
            myParamsForbiddenToRemove = theParamsForbiddenToRemove;
            myControlParamsForbiddenToRemove = theControlParamsForbiddenToRemove;
            myControlParamsToRemove = new NCollection_Map<double>();
            myCurrParam = 0.0;
            myCurrControlParam = 0.0;
            myPrevControlParam = 0.0;
        }

        // Returns the map of the parameters to be removed.
        public NCollection_Map<double> GetControlParametersToRemove(in IMeshTools_Parameters theParameters)
        {
            myParameters = theParameters;

            int aStartIndex, aEndIndex;
            if (myIsoU)
            {
                aStartIndex = 1;
                aEndIndex = myParams.Length();
            }
            else
            {
                aStartIndex = 2;
                aEndIndex = myParams.Length() - 1;
            }

            for (int i = aStartIndex; i <= aEndIndex; ++i)
            {
                myCurrParam = myParams.Value(i);
                myIso = new GeomAdaptor_Curve(myIsoU ? mySurface.UIso(myCurrParam) : mySurface.VIso(myCurrParam));

                myPrevControlParam = myControlParams.Value(1);
                myIso.D1(myPrevControlParam, out myPrevControlPnt, out myPrevControlVec);
                for (int j = 2; j <= myControlParams.Length();)
                {
                    j += checkControlPointAndMoveOn(j);
                }
            }

            return myControlParamsToRemove;
        }

        // Checks the given control point for deviation. Returns the number of steps to move the point iterator by.
        private int checkControlPointAndMoveOn(int theIndex)
        {
            int aMoveSteps = 0;
            myCurrControlParam = myControlParams.Value(theIndex);
            myIso!.D1(myCurrControlParam, out myCurrControlPnt, out myCurrControlVec);

            double aMidParam = 0.5 * (myPrevControlParam + myCurrControlParam);
            Point3D aMidPnt = myIso.Value(aMidParam);

            double aSqDist =
                BRepMesh_GeomTool.SquareDeflectionOfSegment(myPrevControlPnt, myCurrControlPnt, aMidPnt);

            double anAngle = 0.0;

            if ((myPrevControlVec.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                && (myCurrControlVec.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()))
            {
                anAngle = myPrevControlVec.Angle(myCurrControlVec);
            }

            double aSqMaxDeflection = myDFace.GetDeflection() * myDFace.GetDeflection();

            if (((aSqDist > aSqMaxDeflection) || (anAngle > myParameters.AngleInterior))
                && aSqDist > myParameters.MinSize * myParameters.MinSize)
            {
                // insertion
                myControlParams.InsertBefore(theIndex, aMidParam);
            }
            else
            {
                // Here we should leave at least 3 parameters as far as
                // we must have at least one parameter related to surface
                // internals in order to prevent movement of triangle body
                // outside the surface in case of highly curved ones, e.g.
                // BSpline springs.
                if (((aSqDist < aSqMaxDeflection) || (anAngle < myParameters.AngleInterior))
                    && myControlParams.Length() > 3 && theIndex < myControlParams.Length())
                {
                    // Remove too dense points
                    double aTmpParam = myControlParams.Value(theIndex + 1);
                    if (checkParameterForDeflectionAndUpdateCache(aTmpParam))
                    {
                        ++aMoveSteps;
                    }
                }

                myPrevControlParam = myCurrControlParam;
                myPrevControlPnt = myCurrControlPnt;
                myPrevControlVec = myCurrControlVec;

                ++aMoveSteps;
            }

            return aMoveSteps;
        }

        // Checks whether the given parameter suits the deflection. Updates the cache.
        private bool checkParameterForDeflectionAndUpdateCache(double theParam)
        {
            myIso!.D1(theParam, out Point3D aTmpPnt, out Vector3D aTmpVec);

            double aTmpMidParam = 0.5 * (myPrevControlParam + theParam);
            Point3D aTmpMidPnt = myIso.Value(aTmpMidParam);

            // Lets check next parameter.
            // If it also fits deflection, we can remove previous parameter.
            double aSqDist =
                BRepMesh_GeomTool.SquareDeflectionOfSegment(myPrevControlPnt, aTmpPnt, aTmpMidPnt);

            if (aSqDist < myDFace.GetDeflection() * myDFace.GetDeflection())
            {
                // Lets check parameters for angular deflection.
                if (myPrevControlVec.SquareMagnitude() < Constants.Resolution()
                    || aTmpVec.SquareMagnitude() < Constants.Resolution()
                    || myPrevControlVec.Angle(aTmpVec) < myParameters.AngleInterior)
                {
                    // For current Iso line we can remove this parameter.
                    myControlParamsToRemove.Add(myCurrControlParam);
                    myCurrControlParam = theParam;
                    myCurrControlPnt = aTmpPnt;
                    myCurrControlVec = aTmpVec;
                    return true;
                }
                else
                {
                    // We have found a place on the surface refusing
                    // removement of this parameter.
                    myParamsForbiddenToRemove.Add(myCurrParam);
                    myControlParamsForbiddenToRemove.Add(myCurrControlParam);
                }
            }

            return false;
        }
    }
}
