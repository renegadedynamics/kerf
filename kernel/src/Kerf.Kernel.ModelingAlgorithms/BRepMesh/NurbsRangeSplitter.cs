// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NURBSRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NURBSRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Meshing;

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
internal class NurbsRangeSplitter : UVParamRangeSplitter
{
    private SurfaceKind _surfaceType;

    /// <summary>A splitter.</summary>
    public NurbsRangeSplitter()
    {
        _surfaceType = OtherSurface;
    }

    /// <summary>Updates the discrete range of the surface from its geometric range.</summary>
    public override void AdjustRange()
    {
        base.AdjustRange();
        _surfaceType = GetSurface().GetType();

        if (_surfaceType == SurfaceKind.BezierSurface)
        {
            (double first, double second) rangeU = GetRangeU();
            (double first, double second) rangeV = GetRangeV();

            _isValid = rangeU.first >= -0.5 && rangeU.second <= 1.5 && rangeV.first >= -0.5
                        && rangeV.second <= 1.5;
        }
    }

    /// <summary>The nodes generated from the surface and the parameters; null when there are no parameters.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        if (!InitParameters())
        {
            return null;
        }

        (double first, double second) rangeU = GetRangeU();
        (double first, double second) rangeV = GetRangeV();
        (double first, double second) delta = GetDelta();

        double defFace = GetDFace().GetDeflection();
        FaceSurfaceAdaptor gFace = GetSurface();

        NCollection_Sequence<double>[] aParams =
        [
            ComputeGrainAndFilterParameters(GetParametersU(),
                                            gFace.UResolution(defFace),
                                            (rangeU.second - rangeU.first),
                                            delta.first,
                                            parameters),

            ComputeGrainAndFilterParameters(GetParametersV(),
                                            gFace.VResolution(defFace),
                                            (rangeV.second - rangeV.first),
                                            delta.second,
                                            parameters),
        ];

        // check intermediate isolines
        HashSet<double>[] fixedParams = [new HashSet<double>(), new HashSet<double>()];

        HashSet<double>[] paramsToRemove =
        [
            new AnalyticalFilter(GetDFace(),
                                 IsoV,
                                 aParams[1],
                                 aParams[0],
                                 fixedParams[1],
                                 fixedParams[0])
                .GetControlParametersToRemove(parameters),

            new AnalyticalFilter(GetDFace(),
                                 IsoU,
                                 aParams[0],
                                 aParams[1],
                                 fixedParams[0],
                                 fixedParams[1])
                .GetControlParametersToRemove(parameters),
        ];

        NCollection_MapAlgo.Subtract(paramsToRemove[0], fixedParams[0]);
        NCollection_MapAlgo.Subtract(paramsToRemove[1], fixedParams[1]);

        // insert nodes of the regular grid
        NCollection_List<Point2D> nodes = new();

        // insert nodes of the regular grid
        for (int i = 1; i <= aParams[0].Length(); ++i)
        {
            double param1 = aParams[0].Value(i);
            if (paramsToRemove[0].Contains(param1))
            {
                continue;
            }

            for (int j = 1; j <= aParams[1].Length(); ++j)
            {
                double param2 = aParams[1].Value(j);
                if (paramsToRemove[1].Contains(param2))
                {
                    continue;
                }

                nodes.Append(new Point2D(param1, param2));
            }
        }

        return nodes;
    }

    /// <summary>Initializes the U and V parameter lists from the CN continuity intervals.</summary>
    protected virtual bool InitParameters()
    {
        Continuity continuity = CN;
        FaceSurfaceAdaptor surface = GetSurface();

        NCollection_Array1<double>[] intervals = [new NCollection_Array1<double>(), new NCollection_Array1<double>()];
        GetUndefinedInterval(surface, true, continuity, GetRangeU(), ref intervals[0]);
        GetUndefinedInterval(surface, false, continuity, GetRangeV(), ref intervals[1]);

        bool isSplitIntervals = ToSplitIntervals(surface.GeomSurfaceOriginal()!, intervals);

        if (!InitParamsFromIntervals(intervals[0],
                                     GetRangeU(),
                                     isSplitIntervals,
                                     GetParametersU()))
        {
            // if (!grabParamsOfEdges (Edge_Frontier, Param_U))
            {
                return false;
            }
        }

        if (!InitParamsFromIntervals(intervals[1],
                                     GetRangeV(),
                                     isSplitIntervals,
                                     GetParametersV()))
        {
            // if (!grabParamsOfEdges (Edge_Frontier, Param_V))
            {
                return false;
            }
        }

        return GrabParamsOfEdges(EdgeType.Edge_Internal, (int)(ParamDimension.Param_U | ParamDimension.Param_V));
    }

    /// <summary>The number of intervals computed from the available geometrical parameters: the poles less one.</summary>
    protected virtual int GetUndefinedIntervalNb(SurfaceAdaptorBase surface,
                                                 bool isU,
                                                 Continuity continuity)
    {
        return (isU ? surface.UPoleCount : surface.VPoleCount) - 1;
    }

    // Tries to compute intervals even for cases with no intervals at all using available geometrical parameters.
    private void GetUndefinedInterval(SurfaceAdaptorBase surface,
                                      bool isU,
                                      Continuity continuity,
                                      (double first, double second) range,
                                      ref NCollection_Array1<double> intervals)
    {
        int intervalsNb =
            isU ? surface.NbUIntervals(continuity) : surface.NbVIntervals(continuity);

        if (intervalsNb == 1)
        {
            intervalsNb = GetUndefinedIntervalNb(surface, isU, continuity);
            if (intervalsNb > 1)
            {
                intervals = new NCollection_Array1<double>(1, intervalsNb - 1);
                double diff = (range.second - range.first) / intervalsNb;
                for (int i = intervals.Lower(); i <= intervals.Upper(); ++i)
                {
                    intervals.SetValue(i, range.first + i * diff);
                }
            }
        }

        if (intervals.IsEmpty())
        {
            intervals = new NCollection_Array1<double>(1, intervalsNb + 1);
            if (isU)
            {
                surface.UIntervals(intervals, continuity);
            }
            else
            {
                surface.VIntervals(intervals, continuity);
            }
        }
    }

    // Computes the parameters of the filter and applies it to the source parameters.
    private NCollection_Sequence<double> ComputeGrainAndFilterParameters(IndexedSet<double> sourceParams,
                                                                        double tol2d,
                                                                        double rangeDiff,
                                                                        double delta,
                                                                        in ToolParameters parameters)
    {
        // Sort and filter sequence of parameters
        double minDiff = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (delta < 1.0)
        {
            minDiff /= delta;
        }

        FaceSurfaceAdaptor surface = GetSurface();
        double minSize2d = Max(surface.UResolution(parameters.MinSize),
                                surface.VResolution(parameters.MinSize));

        minDiff = Max(minSize2d, minDiff);

        double diffMaxLim = 0.1 * rangeDiff;
        double diffMinLim = Max(0.005 * rangeDiff, 2.0 * tol2d);
        double diff = Max(minSize2d, Min(diffMaxLim, diffMinLim));
        return FilterParameters(sourceParams, minDiff, diff);
    }

    // Filters the parameters to avoid a too dense distribution.
    private static NCollection_Sequence<double> FilterParameters(IndexedSet<double> theParams,
                                                                 double minDist,
                                                                 double filterDist)
    {
        NCollection_Sequence<double> result = new();

        // Sort sequence of parameters
        int initLen = theParams.Count;

        if (initLen < 1)
        {
            return result;
        }

        NCollection_Array1<double> paramArray = new(1, initLen);
        int j;
        for (j = 1; j <= initLen; j++)
        {
            paramArray[j] = theParams[j - 1];
        }

        paramArray.AsSpan().Sort();

        // mandatory pre-filtering using the first (minimal) filter value
        int paramLength = 1;
        for (j = 2; j <= initLen; j++)
        {
            if ((paramArray[j] - paramArray[paramLength]) > minDist)
            {
                if (++paramLength < j)
                {
                    paramArray[paramLength] = paramArray[j];
                }
            }
        }

        // perform filtering on series
        double lastAdded, lastCandidate;
        bool isCandidateDefined = false;
        lastAdded = paramArray[1];
        lastCandidate = lastAdded;
        result.Append(lastAdded);

        for (j = 2; j < paramLength; j++)
        {
            double val = paramArray[j];
            if (val - lastAdded > filterDist)
            {
                // adds the parameter
                if (isCandidateDefined)
                {
                    lastAdded = lastCandidate;
                    isCandidateDefined = false;
                    j--;
                }
                else
                {
                    lastAdded = val;
                }
                result.Append(lastAdded);
                continue;
            }

            lastCandidate = val;
            isCandidateDefined = true;
        }
        result.Append(paramArray[paramLength]);

        return result;
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
    private bool GrabParamsOfEdges(EdgeType edgeType, int paramDimensionFlag)
    {
        if ((paramDimensionFlag & (int)(ParamDimension.Param_U | ParamDimension.Param_V)) == 0)
        {
            return false;
        }

        FaceModel dFace = GetDFace();
        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
        {
            WireModel dWire = dFace.GetWire(wireIt);
            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                EdgeModel dEdge = dWire.GetEdge(edgeIt);
                for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
                {
                    PCurveModel dpCurve = dEdge.GetPCurve(pCurveIt);
                    if (ReferenceEquals(dpCurve.GetFace(), dFace))
                    {
                        if (edgeType == EdgeType.Edge_Internal && !dpCurve.IsInternal)
                        {
                            continue;
                        }

                        for (int pointIt = 0; pointIt < dpCurve.ParametersNb(); ++pointIt)
                        {
                            Point2D pnt2d = dpCurve.GetPoint(pointIt);
                            if ((paramDimensionFlag & (int)ParamDimension.Param_U) != 0)
                            {
                                GetParametersU().Add(pnt2d.X);
                            }

                            if ((paramDimensionFlag & (int)ParamDimension.Param_V) != 0)
                            {
                                GetParametersV().Add(pnt2d.Y);
                            }
                        }
                    }
                }
            }
        }

        return true;
    }

    private static bool AddParam(double param,
                                 (double first, double second) range,
                                 IndexedSet<double> theParams)
    {
        if (param < range.first || param > range.second)
        {
            return false;
        }

        theParams.Add(param);
        return true;
    }

    private static bool InitParamsFromIntervals(NCollection_Array1<double> intervals,
                                                (double first, double second) range,
                                                bool isSplitIntervals,
                                                IndexedSet<double> theParams)
    {
        bool isAdded = false;

        for (int i = intervals.Lower(); i <= intervals.Upper(); ++i)
        {
            double startParam = intervals.Value(i);
            if (AddParam(startParam, range, theParams))
            {
                isAdded = true;
            }

            if (isSplitIntervals && i < intervals.Upper())
            {
                double midParam = (startParam + intervals.Value(i + 1)) / 2.0;
                if (AddParam(midParam, range, theParams))
                {
                    isAdded = true;
                }
            }
        }

        return isAdded;
    }

    private static bool ToSplitIntervals(Surface surf, NCollection_Array1<double>[] intervals)
    {
        int intervalU = intervals[0].Lower();
        for (; intervalU <= intervals[0].Upper(); ++intervalU)
        {
            double paramU = intervals[0].Value(intervalU);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(paramU))
            {
                continue;
            }

            int intervalV = intervals[1].Lower();
            for (; intervalV <= intervals[1].Upper(); ++intervalV)
            {
                Direction3D norm = new();
                double paramV = intervals[1].Value(intervalV);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(paramV))
                {
                    continue;
                }

                if (global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(surf, new Point2D(paramU, paramV), global::Kerf.Engine.Foundation.Precision.Confusion(), ref norm)
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
        private readonly FaceModel _dFace;
        private readonly Surface _surface;
        private readonly bool _isoU;
        private readonly NCollection_Sequence<double> _params;
        private readonly NCollection_Sequence<double> _controlParams;

        private readonly HashSet<double> _paramsForbiddenToRemove;
        private readonly HashSet<double> _controlParamsForbiddenToRemove;

        private readonly HashSet<double> _controlParamsToRemove;

        private ToolParameters _parameters = new();
        private CurveAdaptor? _iso;

        private double _currParam;

        private double _currControlParam;
        private Point3D _currControlPnt;
        private Vector3D _currControlVec;

        private double _prevControlParam;
        private Point3D _prevControlPnt;
        private Vector3D _prevControlVec;

        public AnalyticalFilter(FaceModel dFace,
                                IsoType isoType,
                                NCollection_Sequence<double> theParams,
                                NCollection_Sequence<double> controlParams,
                                HashSet<double> paramsForbiddenToRemove,
                                HashSet<double> controlParamsForbiddenToRemove)
        {
            _dFace = dFace;
            _surface = _dFace.GetSurface().GeomSurfaceOriginal()!;
            _isoU = isoType == IsoU;
            _params = theParams;
            _controlParams = controlParams;
            _paramsForbiddenToRemove = paramsForbiddenToRemove;
            _controlParamsForbiddenToRemove = controlParamsForbiddenToRemove;
            _controlParamsToRemove = new HashSet<double>();
            _currParam = 0.0;
            _currControlParam = 0.0;
            _prevControlParam = 0.0;
        }

        // Returns the map of the parameters to be removed.
        public HashSet<double> GetControlParametersToRemove(in ToolParameters parameters)
        {
            _parameters = parameters;

            int startIndex, endIndex;
            if (_isoU)
            {
                startIndex = 1;
                endIndex = _params.Length();
            }
            else
            {
                startIndex = 2;
                endIndex = _params.Length() - 1;
            }

            for (int i = startIndex; i <= endIndex; ++i)
            {
                _currParam = _params.Value(i);
                _iso = new CurveAdaptor(_isoU ? _surface.UIso(_currParam) : _surface.VIso(_currParam));

                _prevControlParam = _controlParams.Value(1);
                _iso.D1(_prevControlParam, out _prevControlPnt, out _prevControlVec);
                for (int j = 2; j <= _controlParams.Length();)
                {
                    j += CheckControlPointAndMoveOn(j);
                }
            }

            return _controlParamsToRemove;
        }

        // Checks the given control point for deviation. Returns the number of steps to move the point iterator by.
        private int CheckControlPointAndMoveOn(int index)
        {
            int moveSteps = 0;
            _currControlParam = _controlParams.Value(index);
            _iso!.D1(_currControlParam, out _currControlPnt, out _currControlVec);

            double midParam = 0.5 * (_prevControlParam + _currControlParam);
            Point3D midPnt = _iso.Value(midParam);

            double sqDist =
                GeomTool.SquareDeflectionOfSegment(_prevControlPnt, _currControlPnt, midPnt);

            double angle = 0.0;

            if ((_prevControlVec.SquareMagnitude > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                && (_currControlVec.SquareMagnitude > global::Kerf.Engine.Foundation.Precision.SquareConfusion()))
            {
                angle = _prevControlVec.AngleTo(_currControlVec);
            }

            double sqMaxDeflection = _dFace.GetDeflection() * _dFace.GetDeflection();

            if (((sqDist > sqMaxDeflection) || (angle > _parameters.AngleInterior))
                && sqDist > _parameters.MinSize * _parameters.MinSize)
            {
                // insertion
                _controlParams.InsertBefore(index, midParam);
            }
            else
            {
                // Here we should leave at least 3 parameters as far as
                // we must have at least one parameter related to surface
                // internals in order to prevent movement of triangle body
                // outside the surface in case of highly curved ones, e.g.
                // BSpline springs.
                if (((sqDist < sqMaxDeflection) || (angle < _parameters.AngleInterior))
                    && _controlParams.Length() > 3 && index < _controlParams.Length())
                {
                    // Remove too dense points
                    double tmpParam = _controlParams.Value(index + 1);
                    if (CheckParameterForDeflectionAndUpdateCache(tmpParam))
                    {
                        ++moveSteps;
                    }
                }

                _prevControlParam = _currControlParam;
                _prevControlPnt = _currControlPnt;
                _prevControlVec = _currControlVec;

                ++moveSteps;
            }

            return moveSteps;
        }

        // Checks whether the given parameter suits the deflection. Updates the cache.
        private bool CheckParameterForDeflectionAndUpdateCache(double param)
        {
            _iso!.D1(param, out Point3D tmpPnt, out Vector3D tmpVec);

            double tmpMidParam = 0.5 * (_prevControlParam + param);
            Point3D tmpMidPnt = _iso.Value(tmpMidParam);

            // Lets check next parameter.
            // If it also fits deflection, we can remove previous parameter.
            double sqDist =
                GeomTool.SquareDeflectionOfSegment(_prevControlPnt, tmpPnt, tmpMidPnt);

            if (sqDist < _dFace.GetDeflection() * _dFace.GetDeflection())
            {
                // Lets check parameters for angular deflection.
                if (_prevControlVec.SquareMagnitude < Constants.Resolution()
                    || tmpVec.SquareMagnitude < Constants.Resolution()
                    || _prevControlVec.AngleTo(tmpVec) < _parameters.AngleInterior)
                {
                    // For current Iso line we can remove this parameter.
                    _controlParamsToRemove.Add(_currControlParam);
                    _currControlParam = param;
                    _currControlPnt = tmpPnt;
                    _currControlVec = tmpVec;
                    return true;
                }
                else
                {
                    // We have found a place on the surface refusing
                    // removement of this parameter.
                    _paramsForbiddenToRemove.Add(_currParam);
                    _controlParamsForbiddenToRemove.Add(_currControlParam);
                }
            }

            return false;
        }
    }
}
