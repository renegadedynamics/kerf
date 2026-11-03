// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceChecker.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceChecker.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Meshing.GeomTool.IntFlag;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Checks the wires of a discrete face for self-intersections: explodes them into segments of their discrete pcurves
/// and checks each segment against the others; the edges of the colliding segments are the result.
/// </summary>
/// <remarks>
/// The wires are checked in parallel when the parameters ask for it and the face has several. Each loop body writes
/// only its wire's slot; the pcurves are only read.
/// </remarks>
public class FaceChecker : Transient
{
    /// <summary>A segment of a discrete pcurve, with its edge.</summary>
    /// <remarks>
    /// OCCT keeps pointers to the pcurve's two points ("explicit pointers instead of indices"); the checker only reads
    /// them, and nothing writes the pcurves while it runs, so the port keeps their values.
    /// </remarks>
    public struct Segment
    {
        /// <summary>The edge of the segment.</summary>
        public EdgeModel? EdgePtr;

        /// <summary>The first point.</summary>
        public Point2D Point1;

        /// <summary>The second point.</summary>
        public Point2D Point2;

        /// <summary>A segment of the edge between the points.</summary>
        public Segment(EdgeModel edgePtr, in Point2D point1, in Point2D point2)
        {
            EdgePtr = edgePtr;
            Point1 = point1;
            Point2 = point2;
        }
    }

    private const double MaxTangentAngle = 5.0 * M_PI / 180.0;

    private readonly FaceModel _dFace;
    private readonly ToolParameters _parameters; // OCCT holds a reference; the caller's doesn't change meanwhile

    private NCollection_Array1<List<Segment>?>? _wiresSegments;
    private NCollection_Array1<IndexedSpatialTree<int, Box2d>?>? _wiresBndBoxTree;
    private NCollection_Array1<HashSet<EdgeModel>?>? _wiresIntersectingEdges;
    private HashSet<EdgeModel>? _intersectingEdges;

    /// <summary>A checker of the face.</summary>
    public FaceChecker(FaceModel face, in ToolParameters parameters)
    {
        _dFace = face;
        _parameters = parameters;
    }

    /// <summary>Checks the face's wires for intersections; true if there is none.</summary>
    public bool Perform()
    {
        _intersectingEdges = new HashSet<EdgeModel>(ReferenceEqualityComparer.Instance);
        CollectSegments();

        OsdParallel.For(0, _dFace.WiresNb(), PerformCore, !IsParallel());
        CollectResult();

        _wiresBndBoxTree = null;
        _wiresSegments = null;
        _wiresIntersectingEdges = null;
        return _intersectingEdges.Count == 0;
    }

    /// <summary>The intersecting edges found by <see cref="Perform"/>.</summary>
    public HashSet<EdgeModel>? GetIntersectingEdges() => _intersectingEdges;

    // Returns True in case if check can be performed in parallel mode.
    private bool IsParallel() => (_parameters.InParallel && _dFace.WiresNb() > 1);

    // Collects face segments.
    private void CollectSegments()
    {
        SegmentsFiller segmentsFiller = new(_dFace, out _wiresSegments, out _wiresBndBoxTree);
        OsdParallel.For(0, _dFace.WiresNb(), segmentsFiller.Process, !IsParallel());

        _wiresIntersectingEdges = new NCollection_Array1<HashSet<EdgeModel>?>(0, _dFace.WiresNb() - 1);
    }

    // Checks wire with the given index for intersection with others.
    private void PerformCore(int wireIndex)
    {
        List<Segment> segments1 = _wiresSegments!.Value(wireIndex)!;
        ref HashSet<EdgeModel>? intersections = ref _wiresIntersectingEdges![wireIndex];

        // TODO: Tolerance is set to twice value of face deflection in order to fit regressions.
        BndBox2dTreeSelector selector = new(2 * _dFace.GetDeflection());
        for (int wireIt = wireIndex; wireIt < _dFace.WiresNb(); ++wireIt)
        {
            IndexedSpatialTree<int, Box2d> bndBoxTree2 = _wiresBndBoxTree!.Value(wireIt)!;
            List<Segment> segments2 = _wiresSegments.Value(wireIt)!;

            selector.SetSegments(segments2);
            for (int segmentIt = 0; segmentIt < segments1.Count; ++segmentIt)
            {
                Segment segment1 = segments1[segmentIt];
                selector.Reset(segment1, (wireIt == wireIndex) ? segmentIt : -1);
                if (bndBoxTree2.Select(selector) != 0)
                {
                    intersections ??= new HashSet<EdgeModel>(ReferenceEqualityComparer.Instance);

                    intersections.Add(segment1.EdgePtr!);

                    List<int> segments = selector.Indices();
                    for (int selIt = 0; selIt < segments.Count; ++selIt)
                    {
                        Segment segment2 = segments2[segments[selIt]];
                        intersections.Add(segment2.EdgePtr!);
                    }
                }
            }
        }
    }

    // Collects intersecting edges.
    private void CollectResult()
    {
        for (int wireIt = 0; wireIt < _dFace.WiresNb(); ++wireIt)
        {
            HashSet<EdgeModel>? edges = _wiresIntersectingEdges!.Value(wireIt);
            if (edges is not null)
            {
                NCollection_MapAlgo.Unite(_intersectingEdges!, edges);
            }
        }
    }

    // Functor to be used to fill segments and bounding box tree in parallel.
    private sealed class SegmentsFiller
    {
        private readonly FaceModel _dFace;
        private readonly NCollection_Array1<List<Segment>?> _wiresSegments;
        private readonly NCollection_Array1<IndexedSpatialTree<int, Box2d>?> _wiresBndBoxTree;

        public SegmentsFiller(FaceModel dFace,
                              out NCollection_Array1<List<Segment>?> wiresSegments,
                              out NCollection_Array1<IndexedSpatialTree<int, Box2d>?> wiresBndBoxTree)
        {
            _dFace = dFace;
            _wiresSegments = wiresSegments = new(0, _dFace.WiresNb() - 1);
            _wiresBndBoxTree = wiresBndBoxTree = new(0, _dFace.WiresNb() - 1);
        }

        // Performs initialization of wire with the given index.
        public void Process(int wireIndex)
        {
            WireModel dWire = _dFace.GetWire(wireIndex);

            List<Segment> segments = new(dWire.EdgesNb());
            IndexedSpatialTree<int, Box2d> bndBoxTree = new();

            _wiresSegments[wireIndex] = segments;
            _wiresBndBoxTree[wireIndex] = bndBoxTree;

            SpatialTreeFiller<int, Box2d> bndBoxTreeFiller = new(bndBoxTree);

            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                // TODO: check 2d wire for consistency.

                EdgeModel dEdge = dWire.GetEdge(edgeIt);
                PCurveModel pCurve = dEdge.GetPCurve(_dFace, dWire.GetEdgeOrientation(edgeIt));

                for (int pointIt = 1; pointIt < pCurve.ParametersNb(); ++pointIt)
                {
                    Point2D pnt1 = pCurve.GetPoint(pointIt - 1);
                    Point2D pnt2 = pCurve.GetPoint(pointIt);

                    Box2d box = new();
                    box.Add(pnt1);
                    box.Add(pnt2);
                    box.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());

                    bndBoxTreeFiller.Add(segments.Count, box);
                    segments.Add(new Segment(dEdge, pnt1, pnt2));
                }
            }

            bndBoxTreeFiller.Fill();
        }
    }

    // Selector. Used to identify segments with overlapped bounding boxes.
    private sealed class BndBox2dTreeSelector : SpatialTree<int, Box2d>.Selector
    {
        private readonly double _maxLoopSize;
        private int _selfSegmentIndex;
        private List<Segment>? _segments;
        private Segment _segment;
        private Box2d _box = new();
        private readonly List<int> _indices;

        public BndBox2dTreeSelector(double tolerance)
        {
            _maxLoopSize = M_PI * tolerance * tolerance;
            _selfSegmentIndex = -1;
            _indices = new List<int>();
        }

        // Sets working set of segments.
        public void SetSegments(List<Segment> segments)
        {
            _segments = segments;
        }

        // Resets current selector.
        public void Reset(in Segment segment, int selfSegmentIndex)
        {
            _indices.Clear();

            _selfSegmentIndex = selfSegmentIndex;
            _segment = segment;

            _box.SetVoid();
            _box.Add(_segment.Point1);
            _box.Add(_segment.Point2);
            _box.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Indicates should the given box be rejected or not.
        public override bool Reject(in Box2d box) => _box.IsOut(box);

        // Accepts segment with the given index in case if it fits conditions.
        public override bool Accept(in int segmentIndex)
        {
            Segment segment = _segments![segmentIndex];

            Point2D intPnt = new();
            GeomTool.IntFlag intStatus =
                GeomTool.IntSegSeg(_segment.Point1.XY(),
                                            _segment.Point2.XY(),
                                            segment.Point1.XY(),
                                            segment.Point2.XY(),
                                            false,
                                            false,
                                            ref intPnt);

            if (intStatus == Cross)
            {
                double angle = new Vector2D(new Point2D(_segment.Point1.XY()), new Point2D(_segment.Point2.XY()))
                                    .Angle(new Vector2D(new Point2D(segment.Point1.XY()), new Point2D(segment.Point2.XY())));

                if (Math.Abs(angle) < MaxTangentAngle)
                {
                    return false;
                }

                // OCCT: the loop's area takes each segment's second point in its pcurve's parametric order, not the
                // wire's: along reversed edges a real crossing can measure zero area and be dropped (occt-issues.md).
                if (_selfSegmentIndex != -1)
                {
                    Xy prevVec = new();
                    double sumS = 0.0;
                    Xy refPnt = intPnt.Coord();
                    for (int i = _selfSegmentIndex; i < segmentIndex; ++i)
                    {
                        Segment currSegment = _segments[i];
                        Xy curVec = currSegment.Point2.XY() - refPnt;

                        if (curVec.SquareModulus() < Constants.Resolution())
                        {
                            continue;
                        }

                        if (prevVec.SquareModulus() > Constants.Resolution())
                        {
                            sumS += prevVec ^ curVec;
                        }

                        prevVec = curVec;
                    }

                    if (Math.Abs(sumS / 2.0) < _maxLoopSize)
                    {
                        return false;
                    }
                }

                _indices.Add(segmentIndex);
                return true;
            }

            return false;
        }

        // Returns indices of intersecting segments.
        public List<int> Indices() => _indices;
    }
}
