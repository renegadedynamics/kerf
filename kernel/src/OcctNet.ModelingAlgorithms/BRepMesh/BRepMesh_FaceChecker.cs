// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceChecker.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceChecker.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.BRepMesh_GeomTool.IntFlag;

namespace Occt;

/// <summary>
/// Checks the wires of a discrete face for self-intersections: explodes them into segments of their discrete pcurves
/// and checks each segment against the others; the edges of the colliding segments are the result.
/// </summary>
/// <remarks>
/// The wires are checked in parallel when the parameters ask for it and the face has several. Each loop body writes
/// only its wire's slot; the pcurves are only read.
/// </remarks>
public class BRepMesh_FaceChecker : Transient
{
    /// <summary>A segment of a discrete pcurve, with its edge.</summary>
    /// <remarks>
    /// OCCT keeps pointers to the pcurve's two points ("explicit pointers instead of indices"); the checker only reads
    /// them, and nothing writes the pcurves while it runs, so the port keeps their values.
    /// </remarks>
    public struct Segment
    {
        /// <summary>The edge of the segment.</summary>
        public IMeshData_Edge? EdgePtr;

        /// <summary>The first point.</summary>
        public Point2D Point1;

        /// <summary>The second point.</summary>
        public Point2D Point2;

        /// <summary>A segment of the edge between the points.</summary>
        public Segment(IMeshData_Edge theEdgePtr, in Point2D thePoint1, in Point2D thePoint2)
        {
            EdgePtr = theEdgePtr;
            Point1 = thePoint1;
            Point2 = thePoint2;
        }
    }

    private const double MaxTangentAngle = 5.0 * M_PI / 180.0;

    private readonly IMeshData_Face myDFace;
    private readonly IMeshTools_Parameters myParameters; // OCCT holds a reference; the caller's doesn't change meanwhile

    private NCollection_Array1<NCollection_DynamicArray<Segment>?>? myWiresSegments;
    private NCollection_Array1<IndexedSpatialTree<int, Box2d>?>? myWiresBndBoxTree;
    private NCollection_Array1<NCollection_Map<IMeshData_Edge>?>? myWiresIntersectingEdges;
    private NCollection_Map<IMeshData_Edge>? myIntersectingEdges;

    /// <summary>A checker of the face.</summary>
    public BRepMesh_FaceChecker(IMeshData_Face theFace, in IMeshTools_Parameters theParameters)
    {
        myDFace = theFace;
        myParameters = theParameters;
    }

    /// <summary>Checks the face's wires for intersections; true if there is none.</summary>
    public bool Perform()
    {
        myIntersectingEdges = new NCollection_Map<IMeshData_Edge>(ReferenceEqualityComparer.Instance);
        collectSegments();

        OsdParallel.For(0, myDFace.WiresNb(), perform, !isParallel());
        collectResult();

        myWiresBndBoxTree = null;
        myWiresSegments = null;
        myWiresIntersectingEdges = null;
        return myIntersectingEdges.IsEmpty();
    }

    /// <summary>The intersecting edges found by <see cref="Perform"/>.</summary>
    public NCollection_Map<IMeshData_Edge>? GetIntersectingEdges() => myIntersectingEdges;

    // Returns True in case if check can be performed in parallel mode.
    private bool isParallel() => (myParameters.InParallel && myDFace.WiresNb() > 1);

    // Collects face segments.
    private void collectSegments()
    {
        SegmentsFiller aSegmentsFiller = new(myDFace, out myWiresSegments, out myWiresBndBoxTree);
        OsdParallel.For(0, myDFace.WiresNb(), aSegmentsFiller.Process, !isParallel());

        myWiresIntersectingEdges = new NCollection_Array1<NCollection_Map<IMeshData_Edge>?>(0, myDFace.WiresNb() - 1);
    }

    // Checks wire with the given index for intersection with others.
    private void perform(int theWireIndex)
    {
        NCollection_DynamicArray<Segment> aSegments1 = myWiresSegments!.Value(theWireIndex)!;
        ref NCollection_Map<IMeshData_Edge>? aIntersections = ref myWiresIntersectingEdges![theWireIndex];

        // TODO: Tolerance is set to twice value of face deflection in order to fit regressions.
        BndBox2dTreeSelector aSelector = new(2 * myDFace.GetDeflection());
        for (int aWireIt = theWireIndex; aWireIt < myDFace.WiresNb(); ++aWireIt)
        {
            IndexedSpatialTree<int, Box2d> aBndBoxTree2 = myWiresBndBoxTree!.Value(aWireIt)!;
            NCollection_DynamicArray<Segment> aSegments2 = myWiresSegments.Value(aWireIt)!;

            aSelector.SetSegments(aSegments2);
            for (int aSegmentIt = 0; aSegmentIt < aSegments1.Length(); ++aSegmentIt)
            {
                Segment aSegment1 = aSegments1.Value(aSegmentIt);
                aSelector.Reset(aSegment1, (aWireIt == theWireIndex) ? aSegmentIt : -1);
                if (aBndBoxTree2.Select(aSelector) != 0)
                {
                    aIntersections ??= new NCollection_Map<IMeshData_Edge>(ReferenceEqualityComparer.Instance);

                    aIntersections.Add(aSegment1.EdgePtr!);

                    NCollection_DynamicArray<int> aSegments = aSelector.Indices();
                    for (int aSelIt = 0; aSelIt < aSegments.Length(); ++aSelIt)
                    {
                        Segment aSegment2 = aSegments2.Value(aSegments.Value(aSelIt));
                        aIntersections.Add(aSegment2.EdgePtr!);
                    }
                }
            }
        }
    }

    // Collects intersecting edges.
    private void collectResult()
    {
        for (int aWireIt = 0; aWireIt < myDFace.WiresNb(); ++aWireIt)
        {
            NCollection_Map<IMeshData_Edge>? aEdges = myWiresIntersectingEdges!.Value(aWireIt);
            if (aEdges is not null)
            {
                NCollection_MapAlgo.Unite(myIntersectingEdges!, aEdges);
            }
        }
    }

    // Functor to be used to fill segments and bounding box tree in parallel.
    private sealed class SegmentsFiller
    {
        private readonly IMeshData_Face myDFace;
        private readonly NCollection_Array1<NCollection_DynamicArray<Segment>?> myWiresSegments;
        private readonly NCollection_Array1<IndexedSpatialTree<int, Box2d>?> myWiresBndBoxTree;

        public SegmentsFiller(IMeshData_Face theDFace,
                              out NCollection_Array1<NCollection_DynamicArray<Segment>?> theWiresSegments,
                              out NCollection_Array1<IndexedSpatialTree<int, Box2d>?> theWiresBndBoxTree)
        {
            myDFace = theDFace;
            myWiresSegments = theWiresSegments = new(0, myDFace.WiresNb() - 1);
            myWiresBndBoxTree = theWiresBndBoxTree = new(0, myDFace.WiresNb() - 1);
        }

        // Performs initialization of wire with the given index.
        public void Process(int theWireIndex)
        {
            IMeshData_Wire aDWire = myDFace.GetWire(theWireIndex);

            NCollection_DynamicArray<Segment> aSegments = new(aDWire.EdgesNb());
            IndexedSpatialTree<int, Box2d> aBndBoxTree = new();

            myWiresSegments[theWireIndex] = aSegments;
            myWiresBndBoxTree[theWireIndex] = aBndBoxTree;

            SpatialTreeFiller<int, Box2d> aBndBoxTreeFiller = new(aBndBoxTree);

            for (int aEdgeIt = 0; aEdgeIt < aDWire.EdgesNb(); ++aEdgeIt)
            {
                // TODO: check 2d wire for consistency.

                IMeshData_Edge aDEdge = aDWire.GetEdge(aEdgeIt);
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(myDFace, aDWire.GetEdgeOrientation(aEdgeIt));

                for (int aPointIt = 1; aPointIt < aPCurve.ParametersNb(); ++aPointIt)
                {
                    Point2D aPnt1 = aPCurve.GetPoint(aPointIt - 1);
                    Point2D aPnt2 = aPCurve.GetPoint(aPointIt);

                    Box2d aBox = new();
                    aBox.Add(aPnt1);
                    aBox.Add(aPnt2);
                    aBox.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());

                    aBndBoxTreeFiller.Add(aSegments.Length(), aBox);
                    aSegments.Append(new Segment(aDEdge, aPnt1, aPnt2));
                }
            }

            aBndBoxTreeFiller.Fill();
        }
    }

    // Selector. Used to identify segments with overlapped bounding boxes.
    private sealed class BndBox2dTreeSelector : SpatialTree<int, Box2d>.Selector
    {
        private readonly double myMaxLoopSize;
        private int mySelfSegmentIndex;
        private NCollection_DynamicArray<Segment>? mySegments;
        private Segment mySegment;
        private Box2d myBox = new();
        private readonly NCollection_DynamicArray<int> myIndices;

        public BndBox2dTreeSelector(double theTolerance)
        {
            myMaxLoopSize = M_PI * theTolerance * theTolerance;
            mySelfSegmentIndex = -1;
            myIndices = new NCollection_DynamicArray<int>(256);
        }

        // Sets working set of segments.
        public void SetSegments(NCollection_DynamicArray<Segment> theSegments)
        {
            mySegments = theSegments;
        }

        // Resets current selector.
        public void Reset(in Segment theSegment, int theSelfSegmentIndex)
        {
            myIndices.Clear();

            mySelfSegmentIndex = theSelfSegmentIndex;
            mySegment = theSegment;

            myBox.SetVoid();
            myBox.Add(mySegment.Point1);
            myBox.Add(mySegment.Point2);
            myBox.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Indicates should the given box be rejected or not.
        public override bool Reject(in Box2d theBox) => myBox.IsOut(theBox);

        // Accepts segment with the given index in case if it fits conditions.
        public override bool Accept(in int theSegmentIndex)
        {
            Segment aSegment = mySegments!.Value(theSegmentIndex);

            Point2D aIntPnt = new();
            BRepMesh_GeomTool.IntFlag aIntStatus =
                BRepMesh_GeomTool.IntSegSeg(mySegment.Point1.XY(),
                                            mySegment.Point2.XY(),
                                            aSegment.Point1.XY(),
                                            aSegment.Point2.XY(),
                                            false,
                                            false,
                                            ref aIntPnt);

            if (aIntStatus == Cross)
            {
                double aAngle = new Vector2D(new Point2D(mySegment.Point1.XY()), new Point2D(mySegment.Point2.XY()))
                                    .Angle(new Vector2D(new Point2D(aSegment.Point1.XY()), new Point2D(aSegment.Point2.XY())));

                if (Math.Abs(aAngle) < MaxTangentAngle)
                {
                    return false;
                }

                // OCCT: the loop's area takes each segment's second point in its pcurve's parametric order, not the
                // wire's: along reversed edges a real crossing can measure zero area and be dropped (occt-issues.md).
                if (mySelfSegmentIndex != -1)
                {
                    Xy aPrevVec = new();
                    double aSumS = 0.0;
                    Xy aRefPnt = aIntPnt.Coord();
                    for (int i = mySelfSegmentIndex; i < theSegmentIndex; ++i)
                    {
                        Segment aCurrSegment = mySegments.Value(i);
                        Xy aCurVec = aCurrSegment.Point2.XY() - aRefPnt;

                        if (aCurVec.SquareModulus() < Constants.Resolution())
                        {
                            continue;
                        }

                        if (aPrevVec.SquareModulus() > Constants.Resolution())
                        {
                            aSumS += aPrevVec ^ aCurVec;
                        }

                        aPrevVec = aCurVec;
                    }

                    if (Math.Abs(aSumS / 2.0) < myMaxLoopSize)
                    {
                        return false;
                    }
                }

                myIndices.Append(theSegmentIndex);
                return true;
            }

            return false;
        }

        // Returns indices of intersecting segments.
        public NCollection_DynamicArray<int> Indices() => myIndices;
    }
}
