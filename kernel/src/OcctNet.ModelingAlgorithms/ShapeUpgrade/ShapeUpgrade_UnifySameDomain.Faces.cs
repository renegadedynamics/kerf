// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the file-static
// functions of the unification of faces; the class is in ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using DataMapOfFacePlane = Occt.NCollection_DataMap<Kerf.Topology.Shape, Kerf.Geometry.PlaneSurface>;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

public partial class ShapeUpgrade_UnifySameDomain
{
    private static bool IsOnSingularity(NCollection_List<Shape> theEdgeList)
    {
        var anItl = new NCollection_List<Shape>.Iterator(theEdgeList);
        for (; anItl.More(); anItl.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anItl.Value());
            if (Tool.Degenerated(anEdge))
            {
                return true;
            }
        }

        return false;
    }

    // only for seam edges assumed to be U- or V- isolines
    private static bool IsUiso(in Edge theEdge, in Face theFace)
    {
        var      aBAcurve2d = new EdgeCurve2DAdaptor(theEdge, theFace);
        Point2D aP2d;
        Vector2D aVec;
        aBAcurve2d.D1(aBAcurve2d.FirstParameter(), out aP2d, out aVec);
        return (Math.Abs(aVec.Y()) > Math.Abs(aVec.X()));
    }

    private static bool IsLinear(EdgeCurveAdaptor theBAcurve, ref Direction3D theDir)
    {
        CurveKind aType = theBAcurve.GetType();

        if (aType == CurveKind.Line)
        {
            theDir = theBAcurve.Line().Position().Direction();
            return true;
        }

        if ((aType == CurveKind.BezierCurve || aType == CurveKind.BSplineCurve)
            && theBAcurve.PoleCount == 2)
        {
            Point3D aFirstPnt = theBAcurve.Value(theBAcurve.FirstParameter());
            Point3D aLastPnt  = theBAcurve.Value(theBAcurve.LastParameter());
            theDir           = new Direction3D(new Vector3D(aFirstPnt, aLastPnt));
            return true;
        }

        return false;
    }

    private static double TrueValueOfOffset(double theValue, double thePeriod)
    {
        if (theValue > 0.0)
        {
            return thePeriod;
        }

        return (-thePeriod);
    }

    // Gets a face from the sequence that has a common boundary with already got <theUsedFaces>. If it is possible,
    // gets a face that has a common boundary in 2D space too (excludes seam edges). <theIndFace> returns the Index of
    // the face in the sequence.
    private static Face GetFaceFromSeq(
      NCollection_Sequence<Shape>                                       theFaces,
      in Face                                                           theRefFace,
      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF,
      NCollection_IndexedMap<Shape>                                     theUsedFaces,
      ref int                                                                  theIndFace)
    {
        if (theUsedFaces.IsEmpty())
        {
            theIndFace = 1;
            theUsedFaces.Add(theFaces[1]);
            return ShapeCasts.Face(theFaces[1]);
        }

        var aSeamEdge   = new Edge();
        var aTargetFace = new Face();
        for (int ii = 1; ii <= theUsedFaces.Extent(); ii++)
        {
            Face aUsedFace = ShapeCasts.Face(theUsedFaces[ii]);
            var         anExplo   = new Explorer(aUsedFace, ShapeType.Edge);
            for (; anExplo.More(); anExplo.Next())
            {
                Edge                    anEdge    = ShapeCasts.Edge(anExplo.Current());
                NCollection_List<Shape> aFaceList = theMapEF.FindFromKey(anEdge);
                if (aFaceList.Extent() < 2)
                {
                    continue;
                }

                Face aFace = (aFaceList.First().IsSame(aUsedFace))
                                      ? ShapeCasts.Face(aFaceList.Last())
                                      : ShapeCasts.Face(aFaceList.First());

                if (theUsedFaces.Contains(aFace))
                {
                    continue;
                }

                if (Tool.IsClosed(anEdge, theRefFace))
                {
                    aSeamEdge = anEdge;
                    continue;
                }

                aTargetFace = aFace;
                break;
            }
            if (aTargetFace.IsNull && !aSeamEdge.IsNull)
            {
                NCollection_List<Shape> aFaceList = theMapEF.FindFromKey(aSeamEdge);
                aTargetFace = (aFaceList.First().IsSame(aUsedFace)) ? ShapeCasts.Face(aFaceList.Last())
                                                                    : ShapeCasts.Face(aFaceList.First());
            }
            if (!aTargetFace.IsNull)
            {
                break;
            }
        }

        theUsedFaces.Add(aTargetFace);
        for (int ii = 2; ii <= theFaces.Length(); ii++)
        {
            if (theFaces[ii].IsSame(aTargetFace))
            {
                theIndFace = ii;
                break;
            }
        }

        return aTargetFace;
    }

    private static void UpdateBoundaries(Curve2D thePCurve,
                                         double       theFirst,
                                         double       theLast,
                                         int          theIndCoord,
                                         ref double   theMinCoord,
                                         ref double   theMaxCoord)
    {
        const int NbSamples = 4;
        double    Delta     = (theLast - theFirst) / NbSamples;

        for (int i = 0; i <= NbSamples; i++)
        {
            double   aParam = theFirst + i * Delta;
            Point2D aPoint = thePCurve.Value(aParam);

            if (aPoint.Coord(theIndCoord) < theMinCoord)
            {
                theMinCoord = aPoint.Coord(theIndCoord);
            }

            if (aPoint.Coord(theIndCoord) > theMaxCoord)
            {
                theMaxCoord = aPoint.Coord(theIndCoord);
            }
        }
    }

    private static bool TryMakeLine(Curve2D thePCurve, double theFirst, double theLast, ref LineCurve2D? theLine)
    {
        Point2D aFirstPnt   = thePCurve.Value(theFirst);
        Point2D aLastPnt    = thePCurve.Value(theLast);
        var      aVec        = new Vector2D(aFirstPnt, aLastPnt);
        double   aSqLen      = aVec.SquareMagnitude();
        double   aSqParamLen = (theLast - theFirst) * (theLast - theFirst);
        if (Math.Abs(aSqLen - aSqParamLen) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        var      aDir     = new Direction2D(aVec);
        Vector2D anOffset = -new Vector2D(aDir);
        anOffset *= theFirst;
        Point2D anOrigin = aFirstPnt.Translated(anOffset);
        var      aLin     = new Line2D(anOrigin, aDir);

        const int NbSamples = 10;
        double    aDelta    = (theLast - theFirst) / NbSamples;
        for (int i = 1; i < NbSamples; i++)
        {
            double   aParam = theFirst + i * aDelta;
            Point2D aPnt   = thePCurve.Value(aParam);
            double   aDist  = aLin.Distance(aPnt);
            if (aDist > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }

        theLine = new LineCurve2D(aLin);
        return true;
    }

    // Removes the specified edge from the vertex to edge map. Returns true if the edge was removed.
    private static bool RemoveEdgeFromMap(in Edge                                                         theEdge,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVertexToEdges)
    {
        bool anIsRemoved = false;
        Exploration.Vertices(theEdge, out Vertex aFirstVertex, out Vertex aLastVertex);
        foreach (Vertex aVertex in (ReadOnlySpan<Vertex>)[aFirstVertex, aLastVertex])
        {
            if (!theVertexToEdges.Contains(aVertex))
            {
                continue;
            }
            NCollection_List<Shape> aVertexEdges = theVertexToEdges.ChangeFromKey(aVertex);
            var                            anEdgesIter  = new NCollection_List<Shape>.Iterator(aVertexEdges);
            while (anEdgesIter.More())
            {
                Shape anEdge = anEdgesIter.Value();
                if (anEdge.IsSame(theEdge))
                {
                    anIsRemoved = true;
                    aVertexEdges.Remove(anEdgesIter);
                }
                else
                {
                    anEdgesIter.Next();
                }
            }
        }
        return anIsRemoved;
    }

    private static double ComputeMinEdgeSize(NCollection_Sequence<Shape> theEdges,
                                             in Face                     theRefFace,
                                             NCollection_Map<Shape>      theEdgesMap)
    {
        double MinSize = RealLast();

        for (int ind = 1; ind <= theEdges.Length(); ind++)
        {
            Edge anEdge = ShapeCasts.Edge(theEdges[ind]);
            theEdgesMap.Add(anEdge);
            Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
            var BAcurve2d = new EdgeCurve2DAdaptor(anEdge, theRefFace);
            if (BAcurve2d.Curve() is null)
            {
                continue;
            }

            Point2D FirstP2d = BAcurve2d.Value(BAcurve2d.FirstParameter());
            Point2D LastP2d  = BAcurve2d.Value(BAcurve2d.LastParameter());
            double   aSqDist;
            if (V1.IsSame(V2) && !Tool.Degenerated(anEdge))
            {
                Point2D MidP2d = BAcurve2d.Value((BAcurve2d.FirstParameter() + BAcurve2d.LastParameter()) / 2);
                aSqDist = FirstP2d.SquareDistance(MidP2d);
            }
            else
            {
                aSqDist = FirstP2d.SquareDistance(LastP2d);
            }

            MinSize = Min(MinSize, aSqDist);
        }
        MinSize = Math.Sqrt(MinSize);
        return MinSize;
    }

    // Searching for origin of U in 2d space. Returns false if could not find curve on surface, true if succeed.
    private static bool FindCoordBounds(NCollection_Sequence<Shape>                                       theFaces,
                                        in Face                                                           theRefFace,
                                        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF,
                                        NCollection_Map<Shape>                                            theEdgesMap,
                                        int                                                                      theIndCoord,
                                        double                                                                   thePeriod,
                                        ref double                                                               theMinCoord,
                                        ref double                                                               theMaxCoord,
                                        ref int                                                                  theNumberOfIntervals,
                                        ref int                                                                  theIndFaceMax)
    {
        // OCCT's std::pair<double, double>, as a pair of fields of a class so that the sequence's items are written
        var aPairSeq = new NCollection_Sequence<double[]>();

        double aSimpleMax = RealFirst();
        theIndFaceMax = 0;

        var aUsedFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        for (int ii = 1; ii <= theFaces.Length(); ii++)
        {
            int anIndFace = 0;
            // Get a face bordering with previous ones
            Face aFace     = GetFaceFromSeq(theFaces, theRefFace, theMapEF, aUsedFaces, ref anIndFace);
            double      aMinCoord = RealLast(), aMaxCoord = RealFirst();
            var         Explo     = new Explorer(aFace, ShapeType.Edge);
            for (; Explo.More(); Explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(Explo.Current());
                if (!theEdgesMap.Contains(anEdge))
                {
                    continue;
                }
                Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, theRefFace, out double fpar, out double lpar);
                if (aPCurve is null)
                {
                    return false;
                }
                UpdateBoundaries(aPCurve, fpar, lpar, theIndCoord, ref aMinCoord, ref aMaxCoord);
            }

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aMinCoord) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aMaxCoord))
            {
                continue;
            }

            if (aMaxCoord > aSimpleMax)
            {
                aSimpleMax    = aMaxCoord;
                theIndFaceMax = anIndFace;
            }

            // Insert new interval (aMinCoord, aMaxCoord) into sequence of intervals
            bool anIsInInterval = false;
            for (int jj = 1; jj <= aPairSeq.Length(); jj++)
            {
                double aLocalMin = aPairSeq[jj][0];
                double aLocalMax = aPairSeq[jj][1];
                if (aMinCoord >= aLocalMin && aMinCoord <= aLocalMax && aMaxCoord >= aLocalMin
                    && aMaxCoord <= aLocalMax)
                {
                    anIsInInterval = true;
                    break;
                }

                if (aMinCoord < aLocalMin && (aMaxCoord >= aLocalMin && aMaxCoord <= aLocalMax))
                {
                    aPairSeq[jj][0] = aMinCoord;
                    anIsInInterval  = true;
                    break;
                }
                else if (aMinCoord < aLocalMin && aMaxCoord > aLocalMax)
                {
                    aPairSeq[jj][0] = aMinCoord;
                    aPairSeq[jj][1] = aMaxCoord;
                    anIsInInterval  = true;
                    break;
                }
                else if ((aMinCoord >= aLocalMin && aMinCoord <= aLocalMax) && aMaxCoord > aLocalMax)
                {
                    aPairSeq[jj][1] = aMaxCoord;
                    anIsInInterval  = true;
                    break;
                }
            }
            if (!anIsInInterval)
            {
                double[] anInterval = [aMinCoord, aMaxCoord];
                if (!aPairSeq.IsEmpty() && aMaxCoord < aPairSeq[1][0])
                {
                    aPairSeq.Prepend(anInterval);
                }
                else
                {
                    aPairSeq.Append(anInterval);
                }
            }
        }

        theNumberOfIntervals = aPairSeq.Length();

        if (aPairSeq.Length() == 2)
        {
            theMinCoord = aPairSeq[2][0] - thePeriod;
        }
        else if (aPairSeq.Length() > 0)
        {
            theMinCoord = aPairSeq[1][0];
        }
        else
        {
            return false;
        }

        theMaxCoord = aPairSeq[1][1];
        return true;
    }

    // Returns the start and end points of the edge in parametric space of the face. The orientation of the edge is
    // taken into account, so the start and end points will be swapped if the edge has a reversed orientation.
    private static (Point2D, Point2D) getCurveParams(in Edge theEdge, in Face theRefFace)
    {
        var    aCurveAdaptor = new EdgeCurve2DAdaptor(theEdge, theRefFace);
        double aFirstParam   = aCurveAdaptor.FirstParameter();
        double aLastParam    = aCurveAdaptor.LastParameter();
        if (theEdge.Orientation() != Forward)
        {
            (aFirstParam, aLastParam) = (aLastParam, aFirstParam);
        }

        Point2D aFirstPoint = aCurveAdaptor.Value(aFirstParam);
        Point2D aLastPoint  = aCurveAdaptor.Value(aLastParam);
        return (aFirstPoint, aLastPoint);
    }

    private static bool RelocatePCurvesToNewUorigin(
      NCollection_Sequence<Shape>                                       theEdges,
      in Shape                                                          theFirstFace,
      in Face                                                           theRefFace,
      double                                                                   theCoordTol,
      int                                                                      theIndCoord,
      double                                                                   thePeriod,
      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVEmap,
      NCollection_DataMap<Shape, Curve2D>                          theEdgeNewPCurve,
      NCollection_Map<Shape>                                            theUsedEdges)
    {
        var anEdgesOfFirstFace = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theFirstFace, anEdgesOfFirstFace);

        for (;;) // walk by contours
        {
            // try to find the start edge that:
            // 1. belongs to outer edges of first face;
            // 2. has minimum U-coord of its start point
            var                aStartEdge    = new Edge();
            Orientation anOrientation = Forward;
            double             aCoordMin     = RealLast();
            for (int anEdgeIndex = 1; anEdgeIndex <= theEdges.Length(); ++anEdgeIndex)
            {
                Edge anEdge = ShapeCasts.Edge(theEdges[anEdgeIndex]);
                if (theUsedEdges.Contains(anEdge))
                {
                    continue;
                }

                if (!anEdgesOfFirstFace.Contains(anEdge))
                {
                    continue;
                }

                if (aStartEdge.IsNull)
                {
                    aStartEdge = anEdge;
                    (Point2D aFirstPoint, Point2D aLastPoint) = getCurveParams(anEdge, theRefFace);
                    if (aFirstPoint.Coord(theIndCoord) < aLastPoint.Coord(theIndCoord))
                    {
                        aCoordMin     = aFirstPoint.Coord(theIndCoord);
                        anOrientation = Forward;
                    }
                    else
                    {
                        aCoordMin     = aLastPoint.Coord(theIndCoord);
                        anOrientation = Reversed;
                    }
                }
                else
                {
                    (Point2D aFirstPoint, Point2D aLastPoint) = getCurveParams(anEdge, theRefFace);
                    if (aFirstPoint.Coord(theIndCoord) < aCoordMin)
                    {
                        aStartEdge    = anEdge;
                        aCoordMin     = aFirstPoint.Coord(theIndCoord);
                        anOrientation = Forward;
                    }
                    if (aLastPoint.Coord(theIndCoord) < aCoordMin)
                    {
                        aStartEdge    = anEdge;
                        aCoordMin     = aLastPoint.Coord(theIndCoord);
                        anOrientation = Reversed;
                    }
                }
            }

            if (aStartEdge.IsNull) // all contours are passed
            {
                break;
            }

            Edge   aCurrentEdge = aStartEdge;
            Curve2D? CurPCurve =
              Tool.CurveOnSurface(aCurrentEdge, theRefFace, out double anEdgeStartParam, out double anEdgeEndParam);
            if (CurPCurve is null)
            {
                return false;
            }
            CurPCurve = new TrimmedCurve2D(CurPCurve, anEdgeStartParam, anEdgeEndParam);
            theEdgeNewPCurve.Bind(aCurrentEdge, CurPCurve);
            if (aCurrentEdge.Orientation() == Reversed)
            {
                (anEdgeStartParam, anEdgeEndParam) = (anEdgeEndParam, anEdgeStartParam);
            }
            double   CurParam = (anOrientation == Forward) ? anEdgeEndParam : anEdgeStartParam;
            Point2D CurPoint = CurPCurve.Value(CurParam);

            for (;;) // collect pcurves of a contour
            {
                if (!RemoveEdgeFromMap(aCurrentEdge, theVEmap))
                {
                    break; // end of contour in 2d
                }
                theUsedEdges.Add(aCurrentEdge);
                Vertex CurVertex = (anOrientation == Forward)
                                            ? Exploration.LastVertex(aCurrentEdge, true)
                                            : Exploration.FirstVertex(aCurrentEdge, true);

                NCollection_List<Shape> Elist = theVEmap.FindFromKey(CurVertex);
                if (Elist.IsEmpty())
                {
                    break; // end of contour in 3d
                }

                for (var itl = new NCollection_List<Shape>.Iterator(Elist); itl.More(); itl.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(itl.Value());
                    if (anEdge.IsSame(aCurrentEdge))
                    {
                        continue;
                    }

                    Vertex aFirstVertex = (anOrientation == Forward)
                                                   ? Exploration.FirstVertex(anEdge, true)
                                                   : Exploration.LastVertex(anEdge, true);
                    if (!aFirstVertex.IsSame(CurVertex)) // may be if CurVertex is deg.vertex
                    {
                        continue;
                    }

                    Curve2D? aPCurve =
                      Tool.CurveOnSurface(anEdge, theRefFace, out anEdgeStartParam, out anEdgeEndParam);
                    if (aPCurve is null)
                    {
                        return false;
                    }
                    aPCurve = new TrimmedCurve2D(aPCurve, anEdgeStartParam, anEdgeEndParam);
                    if (anEdge.Orientation() == Reversed)
                    {
                        (anEdgeStartParam, anEdgeEndParam) = (anEdgeEndParam, anEdgeStartParam);
                    }
                    double   aParam   = (anOrientation == Forward) ? anEdgeStartParam : anEdgeEndParam;
                    Point2D aPoint   = aPCurve.Value(aParam);
                    double   anOffset = CurPoint.Coord(theIndCoord) - aPoint.Coord(theIndCoord);
                    if (Math.Abs(anOffset) >= theCoordTol && Math.Abs(Math.Abs(anOffset) - thePeriod) >= theCoordTol)
                    {
                        continue; // may be if CurVertex is deg.vertex
                    }

                    if (Math.Abs(anOffset) > thePeriod / 2)
                    {
                        anOffset = TrueValueOfOffset(anOffset, thePeriod);
                        var aVec = new Vector2D();
                        if (theIndCoord == 1)
                        {
                            aVec.SetCoord(anOffset, 0.0);
                        }
                        else
                        {
                            aVec.SetCoord(0.0, anOffset);
                        }
                        var aNewPCurve = (Curve2D)aPCurve.Copy();
                        aNewPCurve.Translate(aVec);
                        aPCurve = aNewPCurve;
                    }
                    theEdgeNewPCurve.Bind(anEdge, aPCurve);
                    aCurrentEdge = anEdge;
                    Orientation CurOr = global::Kerf.Engine.Topology.TopAbs.Compose(anOrientation, aCurrentEdge.Orientation());
                    CurParam = (CurOr == Forward) ? aPCurve.LastParameter() : aPCurve.FirstParameter();
                    CurPoint = aPCurve.Value(CurParam);
                    break;
                }
            } // for (;;) (collect pcurves of a contour)
        } // for (;;) (walk by contours)
        return true;
    }

    private static void InsertWiresIntoFaces(NCollection_Sequence<Shape> theWires,
                                             NCollection_Sequence<Shape> theFaces,
                                             in Face                     theRefFace)
    {
        var BB = new BrepBuilder();
        for (int ii = 1; ii <= theWires.Length(); ii++)
        {
            Wire aWire     = ShapeCasts.Wire(theWires[ii]);
            var         iter      = new Iterator(aWire);
            Edge anEdge    = ShapeCasts.Edge(iter.Value());
            var         BAcurve2d = new EdgeCurve2DAdaptor(anEdge, theRefFace);
            Point2D    aPnt2d    = BAcurve2d.Value((BAcurve2d.FirstParameter() + BAcurve2d.LastParameter()) / 2.0);
            var         RequiredFace = new Shape();
            for (int jj = 1; jj <= theFaces.Length(); jj++)
            {
                Face   aFace      = ShapeCasts.Face(theFaces[jj]);
                var           Classifier = new BRepTopAdaptor_FClass2d(aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
                Containment  aStatus    = Classifier.Perform(aPnt2d);
                if (aStatus == In)
                {
                    RequiredFace = aFace.Oriented(Forward);
                    break;
                }
            }
            if (!RequiredFace.IsNull)
            {
                BB.Add(RequiredFace, aWire);
            }
            else
            {
                // Standard_ASSERT_INVOKE("ShapeUpgrade_UnifySameDomain: wire remains unclassified"): a debug report
            }
        }
    }

    private static Face FindCommonFace(in Edge                                                         theEdge1,
                                              in Edge                                                         theEdge2,
                                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap,
                                              ref Orientation                                                 theOrOfE1OnFace,
                                              ref Orientation                                                 theOrOfE2OnFace)
    {
        var aVertex = new Vertex();
        Exploration.CommonVertex(theEdge1, theEdge2, ref aVertex);
        NCollection_List<Shape> Flist = theVFmap.FindFromKey(aVertex);
        var                            itl   = new NCollection_List<Shape>.Iterator(Flist);
        for (; itl.More(); itl.Next())
        {
            Face aFace = ShapeCasts.Face(itl.Value());
            aFace.Orientation(Forward);
            bool e1found = false, e2found = false;
            var  Explo   = new Explorer(aFace, ShapeType.Edge);
            for (; Explo.More(); Explo.Next())
            {
                Shape anEdge = Explo.Current();
                if (anEdge.IsSame(theEdge1))
                {
                    e1found         = true;
                    theOrOfE1OnFace = anEdge.Orientation();
                }
                if (anEdge.IsSame(theEdge2))
                {
                    e2found         = true;
                    theOrOfE2OnFace = anEdge.Orientation();
                }
                if (e1found && e2found)
                {
                    return aFace;
                }
            }
        }

        var NullFace = new Face();
        return NullFace;
    }

    private static bool FindClosestPoints(in Edge                                                         theEdge1,
                                          in Edge                                                         theEdge2,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap,
                                          ref Face                                                        theCommonFace,
                                          ref double                                                             theMinSqDist,
                                          ref Orientation                                                 OrOfE1OnFace,
                                          ref Orientation                                                 OrOfE2OnFace,
                                          ref int                                                                theIndOnE1,
                                          ref int                                                                theIndOnE2,
                                          Span<Point2D>                                                         PointsOnEdge1,
                                          Span<Point2D>                                                         PointsOnEdge2)
    {
        theCommonFace = FindCommonFace(theEdge1, theEdge2, theVFmap, ref OrOfE1OnFace, ref OrOfE2OnFace);
        if (theCommonFace.IsNull)
        {
            return false;
        }

        Curve2D? PCurve1 = Tool.CurveOnSurface(theEdge1, theCommonFace, out double fpar1, out double lpar1);
        Curve2D? PCurve2 = Tool.CurveOnSurface(theEdge2, theCommonFace, out double fpar2, out double lpar2);
        if (PCurve1 is null || PCurve2 is null)
        {
            return false;
        }
        PointsOnEdge1[0] = PCurve1.Value(fpar1);
        PointsOnEdge1[1] = PCurve1.Value(lpar1);
        PointsOnEdge2[0] = PCurve2.Value(fpar2);
        PointsOnEdge2[1] = PCurve2.Value(lpar2);
        theMinSqDist     = RealLast();
        theIndOnE1       = -1;
        theIndOnE2       = -1;
        for (int ind1 = 0; ind1 < 2; ind1++)
        {
            for (int ind2 = 0; ind2 < 2; ind2++)
            {
                double aSqDist = PointsOnEdge1[ind1].SquareDistance(PointsOnEdge2[ind2]);
                if (aSqDist < theMinSqDist)
                {
                    theMinSqDist = aSqDist;
                    theIndOnE1   = ind1;
                    theIndOnE2   = ind2;
                }
            }
        }
        return true;
    }

    private static void ReconstructMissedSeam(NCollection_Sequence<Shape> theRemovedEdges,
                                              in Face                     theFrefFace,
                                              in Edge                     theCurEdge,
                                              in Vertex                   theCurVertex,
                                              in Point2D                        theCurPoint,
                                              double                             theUperiod,
                                              double                             theVperiod,
                                              ref Edge                    theSeamEdge,
                                              ref Point2D                       theNextPoint)
    {
        _ = Tool.Surface(theFrefFace); // RefSurf: unused, as in OCCT

        // Find seam edge between removed edges
        theSeamEdge.Nullify();
        for (int i = 1; i <= theRemovedEdges.Length(); i++)
        {
            Edge anEdge = ShapeCasts.Edge(theRemovedEdges[i]);
            if (anEdge.IsSame(theCurEdge))
            {
                continue;
            }
            Curve2D? aPC = Tool.CurveOnSurface(anEdge, theFrefFace, out double Param1, out double Param2);
            if (aPC is null)
            {
                continue;
            }

            Exploration.Vertices(anEdge, out Vertex aFirstVertex, out Vertex aLastVertex, true);

            if ((aFirstVertex.IsSame(theCurVertex) || aLastVertex.IsSame(theCurVertex))
                && Tool.IsClosed(anEdge, theFrefFace))
            {
                double   aParam = (anEdge.Orientation() == Forward) ? Param1 : Param2;
                Point2D aPoint = aPC.Value(aParam);
                double   aUdiff = Math.Abs(aPoint.X() - theCurPoint.X());
                double   aVdiff = Math.Abs(aPoint.Y() - theCurPoint.Y());
                if ((theUperiod != 0.0 && aUdiff > theUperiod / 2) || (theVperiod != 0.0 && aVdiff > theVperiod / 2))
                {
                    if (aLastVertex.IsSame(theCurVertex) || (theUperiod != 0.0 && theVperiod != 0.0))
                    {
                        anEdge.Reverse();
                    }
                    else
                    {
                        Orientation anOri = anEdge.Orientation();
                        anEdge.Orientation(Forward);
                        Curve2D? aPC1 = Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2);
                        anEdge.Reverse();
                        Curve2D? aPC2 = Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2);
                        anEdge.Reverse(); // again FORWARD
                        // OCCT: rewrites the pcurves of a removed seam of the input (the port's copy)
                        var           aBB   = new BrepBuilder();
                        double        aTol  = Tool.Tolerance(anEdge);
                        Surface? aSurf = Tool.Surface(theFrefFace, out Placement aLoc);
                        aBB.UpdateEdge(anEdge, aPC2, aPC1, aSurf, aLoc, aTol);
                        anEdge.Orientation(anOri);
                    }
                    aPC    = Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2)!;
                    aParam = (anEdge.Orientation() == Forward) ? Param1 : Param2;
                    aPoint = aPC.Value(aParam);
                    aUdiff = Math.Abs(aPoint.X() - theCurPoint.X());
                    aVdiff = Math.Abs(aPoint.Y() - theCurPoint.Y());
                }
                if ((theUperiod == 0.0 || aUdiff < theUperiod / 2) && (theVperiod == 0.0 || aVdiff < theVperiod / 2))
                {
                    aFirstVertex = Exploration.FirstVertex(anEdge, true);
                    if (aFirstVertex.IsSame(theCurVertex))
                    {
                        theSeamEdge = anEdge;
                        break;
                    }
                }
            }
        }

        if (!theSeamEdge.IsNull)
        {
            Curve2D aPC    = Tool.CurveOnSurface(theSeamEdge, theFrefFace, out double Param1, out double Param2)!;
            double       aParam = (theSeamEdge.Orientation() == Forward) ? Param2 : Param1;
            theNextPoint        = aPC.Value(aParam);
        }
    }

    private static readonly double[] s_SameSurfCoefs = [0.3399811, 0.7745966];

    private static bool SameSurf(Surface theS1, Surface theS2)
    {
        double[] aCoefs = s_SameSurfCoefs;

        theS1.Bounds(out double uf1, out double ul1, out double vf1, out double vl1);
        theS2.Bounds(out double uf2, out double ul2, out double vf2, out double vl2);
        double aPTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uf1))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uf2))
            {
                return false;
            }
            else
            {
                uf1 = Min(-1.0, (ul1 - 1.0));
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uf2))
            {
                return false;
            }
            else
            {
                if (Math.Abs(uf1 - uf2) > aPTol)
                {
                    return false;
                }
            }
        }
        //
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vf1))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vf2))
            {
                return false;
            }
            else
            {
                vf1 = Min(-1.0, (vl1 - 1.0));
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vf2))
            {
                return false;
            }
            else
            {
                if (Math.Abs(vf1 - vf2) > aPTol)
                {
                    return false;
                }
            }
        }
        //
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(ul1))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(ul2))
            {
                return false;
            }
            else
            {
                ul1 = Max(1.0, (uf1 + 1.0));
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(ul2))
            {
                return false;
            }
            else
            {
                if (Math.Abs(ul1 - ul2) > aPTol)
                {
                    return false;
                }
            }
        }
        //
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vl1))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vl2))
            {
                return false;
            }
            else
            {
                vl1 = Max(1.0, (vf1 + 1.0));
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vl2))
            {
                return false;
            }
            else
            {
                if (Math.Abs(vl1 - vl2) > aPTol)
                {
                    return false;
                }
            }
        }
        //

        double u, v, du = (ul1 - uf1), dv = (vl1 - vf1);
        int    i, j;
        for (i = 0; i < 2; ++i)
        {
            u = uf1 + aCoefs[i] * du;
            for (j = 0; j < 2; ++j)
            {
                v = vf1 + aCoefs[j] * dv;
                Point3D aP1 = theS1.Value(u, v);
                Point3D aP2 = theS2.Value(u, v);
                if (!aP1.IsEqual(aP2, aPTol))
                {
                    return false;
                }
            }
        }

        return true;
    }

    // OCCT: writes the reference surface's pcurves and a new range into the edges of the faces to unify, which are the
    // input's own even in the safe input mode (the port works on its copy, see Build)
    private static void TransformPCurves(in Face                theRefFace,
                                         in Face                theFace,
                                         NCollection_Map<Shape> theMapEdgesWithTemporaryPCurves)
    {
        Surface RefSurf = Tool.Surface(theRefFace)!;
        if (RefSurf is TrimmedSurface aRefRTS)
        {
            RefSurf = aRefRTS.BasisSurface();
        }

        Surface SurfFace = Tool.Surface(theFace)!;
        if (SurfFace is TrimmedSurface aFaceRTS)
        {
            SurfFace = aFaceRTS.BasisSurface();
        }

        bool ToModify = false, ToTranslate = false, ToRotate = false, X_Reverse = false, Y_Reverse = false,
             ToProject = false;

        double aTranslation = 0.0, anAngle = 0.0;

        // Get axes of surface of face and of surface of RefFace
        var ElemSurfFace = SurfFace as ElementarySurface;
        var ElemRefSurf  = RefSurf as ElementarySurface;

        if (ElemSurfFace is not null && ElemRefSurf is not null)
        {
            CoordinateSystem AxisOfSurfFace = ElemSurfFace.Position();
            CoordinateSystem AxisOfRefSurf  = ElemRefSurf.Position();

            Point3D OriginRefSurf = AxisOfRefSurf.Location();

            double aParam = global::Kerf.Engine.Geometry.ElCLib.LineParameter(AxisOfSurfFace.Axis(), OriginRefSurf);

            if (Math.Abs(aParam) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aTranslation = -aParam;
            }

            Direction3D VdirSurfFace = AxisOfSurfFace.Direction();
            Direction3D VdirRefSurf  = AxisOfRefSurf.Direction();
            Direction3D XdirSurfFace = AxisOfSurfFace.XDirection();
            Direction3D XdirRefSurf  = AxisOfRefSurf.XDirection();

            Direction3D CrossProd1 = AxisOfRefSurf.XDirection() ^ AxisOfRefSurf.YDirection();
            Direction3D CrossProd2 = AxisOfSurfFace.XDirection() ^ AxisOfSurfFace.YDirection();
            if (CrossProd1 * CrossProd2 < 0.0)
            {
                X_Reverse = true;
            }

            double ScalProd = VdirSurfFace * VdirRefSurf;
            if (ScalProd < 0.0)
            {
                Y_Reverse = true;
            }

            if (!X_Reverse && !Y_Reverse)
            {
                Direction3D DirRef = VdirRefSurf;
                if (!AxisOfRefSurf.Direct())
                {
                    DirRef.Reverse();
                }
                anAngle = XdirRefSurf.AngleWithRef(XdirSurfFace, DirRef);
            }
            else
            {
                anAngle = XdirRefSurf.Angle(XdirSurfFace);
            }

            ToRotate = (Math.Abs(anAngle) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            ToTranslate = (Math.Abs(aTranslation) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            ToModify = ToTranslate || ToRotate || X_Reverse || Y_Reverse;
        }
        else
        {
            if (!SameSurf(RefSurf, SurfFace))
            {
                ToProject = true;
            }
        }

        var BB    = new BrepBuilder();
        var Explo = new Explorer(theFace, ShapeType.Edge);
        var aEmap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        for (; Explo.More(); Explo.Next())
        {
            Edge anEdge = ShapeCasts.Edge(Explo.Current());
            if (Tool.Degenerated(anEdge) && ToModify)
            {
                continue;
            }

            if (ToProject && Tool.IsClosed(anEdge, theFace))
            {
                continue;
            }

            if (!aEmap.Add(anEdge))
            {
                continue;
            }

            Orientation anOr = anEdge.Orientation();

            double        fpar, lpar;
            Curve2D? PCurveOnRef = Tool.CurveOnSurface(anEdge, theRefFace, out fpar, out lpar);
            Curve2D? PCurve2     = null;
            if (PCurveOnRef is not null /* && !(ToModify || ToProject)*/)
            {
                anEdge.Reverse();
                PCurve2 = Tool.CurveOnSurface(anEdge, theRefFace, out fpar, out lpar);
                if (!ReferenceEquals(PCurve2, PCurveOnRef))
                { // two pcurves
                    continue;
                }
            }

            var PCurves    = new Curve2D?[2];
            var NewPCurves = new Curve2D?[2];
            anEdge.Orientation(Forward);
            PCurves[0] = Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);
            anEdge.Reverse();
            PCurves[1] = Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);

            if (PCurves[0] is null || PCurves[1] is null)
            {
                // The source face has no pcurve for this edge, so it cannot be transformed.
                continue;
            }

            int NbPcurves = ReferenceEquals(PCurves[0], PCurves[1]) ? 1 : 2;

            for (int ii = 0; ii < NbPcurves; ii++)
            {
                if (ToProject)
                {
                    Curve3D? aC3d = Tool.Curve(anEdge, out fpar, out lpar);
                    aC3d = new TrimmedCurve3D(aC3d!, fpar, lpar);
                    double tol = Tool.Tolerance(anEdge);
                    tol = Min(tol, global::Kerf.Engine.Foundation.Precision.Approximation());
                    // (ToProject excludes the transformations below, so the projection is never edited)
                    NewPCurves[ii] = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(aC3d, RefSurf);
                }
                else
                {
                    NewPCurves[ii] = (Curve2D)PCurves[ii]!.Copy();
                }
                if (ToTranslate)
                {
                    NewPCurves[ii]!.Translate(new Vector2D(0.0, aTranslation));
                }
                if (Y_Reverse)
                {
                    NewPCurves[ii]!.Mirror(Constants.OX2d());
                }
                if (X_Reverse)
                {
                    NewPCurves[ii]!.Mirror(Constants.OY2d());
                    NewPCurves[ii]!.Translate(new Vector2D(2 * M_PI, 0.0));
                }
                if (ToRotate)
                {
                    NewPCurves[ii]!.Translate(new Vector2D(anAngle, 0.0));
                }
            }

            anEdge.Orientation(Forward);

            if (NbPcurves == 1)
            {
                if (PCurve2 is null || (!RefSurf.IsUClosed && !RefSurf.IsVClosed))
                {
                    BB.UpdateEdge(anEdge, NewPCurves[0], theRefFace, 0.0);
                    theMapEdgesWithTemporaryPCurves.Add(anEdge);
                }
                else
                {
                    // check: may be it is the same pcurve
                    RefSurf.Bounds(out double aUmin, out double aUmax, out double aVmin, out double aVmax);
                    double   aUperiod      = (RefSurf.IsUClosed) ? (aUmax - aUmin) : 0.0;
                    double   aVperiod      = (RefSurf.IsVClosed) ? (aVmax - aVmin) : 0.0;
                    Point2D aP2dOnPCurve1 = PCurveOnRef!.Value(fpar);
                    Point2D aP2dOnPCurve2 = NewPCurves[0]!.Value(fpar);
                    if ((aUperiod != 0.0 && Math.Abs(aP2dOnPCurve1.X() - aP2dOnPCurve2.X()) > aUperiod / 2)
                        || (aVperiod != 0.0 && Math.Abs(aP2dOnPCurve1.Y() - aP2dOnPCurve2.Y()) > aVperiod / 2))
                    {
                        Curve2D? NullPCurve = null;
                        BB.UpdateEdge(anEdge, NullPCurve, theRefFace, 0.0);
                        if (anOr == Forward)
                        {
                            BB.UpdateEdge(anEdge, NewPCurves[0], PCurveOnRef, theRefFace, 0.0);
                        }
                        else
                        {
                            BB.UpdateEdge(anEdge, PCurveOnRef, NewPCurves[0], theRefFace, 0.0);
                        }

                        theMapEdgesWithTemporaryPCurves.Add(anEdge);
                    }
                }
            }
            else
            {
                BB.UpdateEdge(anEdge, NewPCurves[0], NewPCurves[1], theRefFace, 0.0);
                theMapEdgesWithTemporaryPCurves.Add(anEdge);
            }

            BB.Range(anEdge, fpar, lpar);
        }
    }

    private static void AddPCurves(NCollection_Sequence<Shape> theFaces,
                                   in Face                     theRefFace,
                                   NCollection_Map<Shape>      theMapEdgesWithTemporaryPCurves)
    {
        var RefBAsurf = new FaceSurfaceAdaptor(theRefFace, false);

        SurfaceKind aType = RefBAsurf.GetType();
        if (aType == Plane)
        {
            return;
        }

        for (int i = 1; i <= theFaces.Length(); i++)
        {
            Face aFace = ShapeCasts.Face(theFaces[i]);
            aFace.Orientation(Forward);
            if (aFace.IsSame(theRefFace))
            {
                continue;
            }

            TransformPCurves(theRefFace, aFace, theMapEdgesWithTemporaryPCurves);
        }
    }

    // Adds edges from the shape to the sequence; seams and equal edges are dropped. Returns true if one of original
    // edges dropped.
    private static bool AddOrdinaryEdges(NCollection_Sequence<Shape> edges,
                                         in Shape                    aShape,
                                         ref int                            anIndex,
                                         NCollection_Sequence<Shape> theRemovedEdges)
    {
        // map of edges
        var aNewEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        // add edges without seams
        for (var exp = new Explorer(aShape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Shape edge = exp.Current();
            if (aNewEdges.Contains(edge))
            {
                aNewEdges.RemoveKey(edge);
                theRemovedEdges.Append(edge);
            }
            else
            {
                aNewEdges.Add(edge);
            }
        }

        bool isDropped = false;
        // merge edges and drop seams
        int i;
        for (i = 1; i <= edges.Length(); i++)
        {
            Shape current = edges[i];
            if (aNewEdges.Contains(current))
            {
                aNewEdges.RemoveKey(current);
                edges.Remove(i);
                theRemovedEdges.Append(current);
                i--;

                if (!isDropped)
                {
                    isDropped = true;
                    anIndex   = i;
                }
            }
        }

        // add edges to the sequence
        for (i = 1; i <= aNewEdges.Extent(); i++)
        {
            edges.Append(aNewEdges[i]);
        }

        return isDropped;
    }

    private static bool getCylinder(Surface theInSurface, ref Cylinder theOutCylinder)
    {
        bool isCylinder = false;

        if (theInSurface is CylindricalSurface aGC)
        {
            theOutCylinder = aGC.Cylinder();
            isCylinder     = true;
        }
        else if (theInSurface is RevolutionSurface aRS)
        {
            Curve3D aBasis = aRS.BasisCurve();

            while (aBasis is TrimmedCurve3D aTc)
            {
                aBasis = aTc.BasisCurve();
            }

            if (aBasis is LineCurve3D aBasisLine)
            {
                Direction3D aDir      = aRS.Direction();
                Direction3D aBasisDir = aBasisLine.Position().Direction();
                if (aBasisDir.IsParallel(aDir, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // basis line is parallel to the revolution axis: it is a cylinder
                    Point3D aLoc   = aRS.Location();
                    double aR     = aBasisLine.Lin().Distance(aLoc);
                    var    aCylAx = new CoordinateSystem(aLoc, aDir);

                    theOutCylinder = new Cylinder(aCylAx, aR);
                    isCylinder     = true;
                }
            }
        }
        else if (theInSurface is ExtrusionSurface aLES)
        {
            Curve3D aBasis = aLES.BasisCurve();

            while (aBasis is TrimmedCurve3D aTc)
            {
                aBasis = aTc.BasisCurve();
            }

            if (aBasis is CircleCurve3D aBasisCircle)
            {
                Direction3D aDir      = aLES.Direction();
                Direction3D aBasisDir = aBasisCircle.Position().Direction();
                if (aBasisDir.IsParallel(aDir, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // basis circle is normal to the extrusion axis: it is a cylinder
                    Point3D aLoc   = aBasisCircle.Location();
                    double aR     = aBasisCircle.Radius();
                    var    aCylAx = new CoordinateSystem(aLoc, aDir);

                    theOutCylinder = new Cylinder(aCylAx, aR);
                    isCylinder     = true;
                }
            }
        }
        else
        {
        }

        return isCylinder;
    }

    private static Surface? ClearRts(Surface? aSurface)
    {
        var aRTS = aSurface as TrimmedSurface;
        return aRTS is null ? aSurface : aRTS.BasisSurface();
    }

    // Gets the normal to surface by the given parameter on edge. Returns True if normal was computed.
    private static bool GetNormalToSurface(in Face theFace, in Edge theEdge, double theP, ref Direction3D theNormal)
    {
        double f, l;
        // get 2d curve to get point in 2d
        Curve2D? aC2d;
        if (Tool.IsClosed(theEdge, theFace))
        {
            // Find the edge in the face: it will have correct orientation
            var         anEdgeInFace = new Edge();
            Face aFace        = theFace;
            aFace.Orientation(Forward);
            var anExplo = new Explorer(aFace, ShapeType.Edge);
            for (; anExplo.More(); anExplo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace = anEdge;
                    break;
                }
            }
            if (anEdgeInFace.IsNull)
            {
                return false;
            }

            aC2d = Tool.CurveOnSurface(anEdgeInFace, aFace, out f, out l);
        }
        else
        {
            aC2d = Tool.CurveOnSurface(theEdge, theFace, out f, out l);
        }

        if (aC2d is null)
        {
            return false;
        }
        //
        // 2d point
        aC2d.D0(theP, out Point2D aP2d);
        //
        // get D1
        Surface aS = Tool.Surface(theFace, out Placement aLoc)!;
        aS.D1(aP2d.X(), aP2d.Y(), out Point3D aP3d, out Vector3D aDU, out Vector3D aDV);
        //
        // compute normal
        Vector3D aVNormal = aDU.Crossed(aDV);
        if (aVNormal.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }
        //
        if (theFace.Orientation() == Reversed)
        {
            aVNormal.Reverse();
        }
        //
        aVNormal.Transform(aLoc.Transformation());
        theNormal = new Direction3D(aVNormal);
        return true;
    }

    private static bool IsSameDomain(in Face        aFace,
                                     in Face        aCheckedFace,
                                     double                theLinTol,
                                     double                theAngTol,
                                     DataMapOfFacePlane    theFacePlaneMap)
    {
        // checking the same handles
        Surface? S1, S2;

        S1 = Tool.Surface(aFace, out Placement L1);
        S2 = Tool.Surface(aCheckedFace, out Placement L2);

        if (ReferenceEquals(S1, S2) && L1 == L2)
        {
            return true;
        }

        S1 = Tool.Surface(aFace);
        S2 = Tool.Surface(aCheckedFace);

        S1 = ClearRts(S1)!;
        S2 = ClearRts(S2)!;

        // case of two planar surfaces:
        // all kinds of surfaces checked, including b-spline and bezier
        var aPlanarityChecker1 = new IsPlanarSurface(S1, theLinTol);
        if (aPlanarityChecker1.IsPlanar)
        {
            var aPlanarityChecker2 = new IsPlanarSurface(S2, theLinTol);
            if (aPlanarityChecker2.IsPlanar)
            {
                Plane3D aPln1 = aPlanarityChecker1.Plan();
                Plane3D aPln2 = aPlanarityChecker2.Plan();

                if (aPln1.Position().Direction().IsParallel(aPln2.Position().Direction(), theAngTol)
                    && aPln1.Distance(aPln2) < theLinTol)
                {
                    PlaneSurface aPlaneOfFaces;
                    if (theFacePlaneMap.IsBound(aFace))
                    {
                        aPlaneOfFaces = theFacePlaneMap[aFace];
                    }
                    else if (theFacePlaneMap.IsBound(aCheckedFace))
                    {
                        aPlaneOfFaces = theFacePlaneMap[aCheckedFace];
                    }
                    else
                    {
                        aPlaneOfFaces = new PlaneSurface(aPln1);
                    }

                    theFacePlaneMap.Bind(aFace, aPlaneOfFaces);
                    theFacePlaneMap.Bind(aCheckedFace, aPlaneOfFaces);

                    return true;
                }
            }
        }

        // case of two elementary surfaces: use OCCT tool
        // elementary surfaces: ConicalSurface, CylindricalSurface,
        //                      Plane, SphericalSurface and ToroidalSurface
        if (S1 is ElementarySurface && S2 is ElementarySurface)
        {
            var aGA1 = new SurfaceAdaptor(S1);
            var aGA2 = new SurfaceAdaptor(S2);

            var aTT1 = new BRepTopAdaptor_TopolTool();
            var aTT2 = new BRepTopAdaptor_TopolTool();

            try
            {
                var anIIInt = new IntPatch_ImpImpIntersection(aGA1, aTT1, aGA2, aTT2, theLinTol, theLinTol);
                if (!anIIInt.IsDone() || anIIInt.IsEmpty())
                {
                    return false;
                }

                return anIIInt.TangentFaces();
            }
            catch (FailureException)
            {
                return false;
            }
        }

        // case of two cylindrical surfaces, at least one of which is a swept surface
        // swept surfaces: SurfaceOfLinearExtrusion, SurfaceOfRevolution
        if ((S1 is CylindricalSurface || S1 is SweptSurface)
            && (S2 is CylindricalSurface || S2 is SweptSurface))
        {
            var aCyl1 = new Cylinder();
            var aCyl2 = new Cylinder();
            if (getCylinder(S1, ref aCyl1) && getCylinder(S2, ref aCyl2))
            {
                if (Math.Abs(aCyl1.Radius() - aCyl2.Radius()) < theLinTol)
                {
                    Direction3D aDir1 = aCyl1.Position().Direction();
                    Direction3D aDir2 = aCyl2.Position().Direction();
                    if (aDir1.IsParallel(aDir2, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        Point3D aLoc1  = aCyl1.Location();
                        Point3D aLoc2  = aCyl2.Location();
                        var    aVec12 = new Vector3D(aLoc1, aLoc2);
                        if (aVec12.SquareMagnitude() < theLinTol * theLinTol
                            || aVec12.IsParallel(new Vector3D(aDir1), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    // OCCT.NET: frozen. Puts theNew in the place of theOld in every face and every edge's curve on surface reachable
    // from theRoots, keeping the representations' order, ranges, UV points and continuities: what OCCT's in-place edit
    // of theOld does to those shapes (IntUnifyFaces' periodic B-spline surface).
    private static void ReplaceSurface(Surface theOld, Surface theNew, IEnumerable<Shape> theRoots)
    {
        var aFaces = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var anEdges = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        foreach (Shape aRoot in theRoots)
        {
            if (aRoot.IsNull)
            {
                continue;
            }
            for (var anExp = new Explorer(aRoot, ShapeType.Face); anExp.More(); anExp.Next())
            {
                if (anExp.Current().TShape() is BrepTFace aTFace && aFaces.Add(aTFace)
                    && ReferenceEquals(aTFace.Surface(), theOld))
                {
                    aTFace.Surface(theNew);
                }
            }
            for (var anExp = new Explorer(aRoot, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                if (anExp.Current().TShape() is not BrepTEdge aTEdge || !anEdges.Add(aTEdge))
                {
                    continue;
                }
                var anIt = new NCollection_List<CurveRepresentation>.Iterator(aTEdge.ChangeCurves());
                for (; anIt.More(); anIt.Next())
                {
                    switch (anIt.Value())
                    {
                        case CurveOnClosedSurface aCOCS when ReferenceEquals(aCOCS.Surface(), theOld):
                        {
                            var aNew = new CurveOnClosedSurface(aCOCS.PCurve()!, aCOCS.PCurve2(), theNew, aCOCS.Location(),
                                                                     aCOCS.Continuity());
                            aNew.SetRange(aCOCS.First(), aCOCS.Last());
                            aCOCS.UVPoints(out Point2D aP1, out Point2D aP2);
                            aNew.SetUVPoints(aP1, aP2);
                            aCOCS.UVPoints2(out aP1, out aP2);
                            aNew.SetUVPoints2(aP1, aP2);
                            anIt.ChangeValue() = aNew;
                            break;
                        }
                        case CurveOnSurfaceRepresentation aCOS when ReferenceEquals(aCOS.Surface(), theOld):
                        {
                            var aNew = new CurveOnSurfaceRepresentation(aCOS.PCurve()!, theNew, aCOS.Location());
                            aNew.SetRange(aCOS.First(), aCOS.Last());
                            aCOS.UVPoints(out Point2D aP1, out Point2D aP2);
                            aNew.SetUVPoints(aP1, aP2);
                            anIt.ChangeValue() = aNew;
                            break;
                        }
                        case CurveOn2Surfaces aCO2S when ReferenceEquals(aCO2S.Surface(), theOld)
                                                             || ReferenceEquals(aCO2S.Surface2(), theOld):
                        {
                            anIt.ChangeValue() = new CurveOn2Surfaces(
                              ReferenceEquals(aCO2S.Surface(), theOld) ? theNew : aCO2S.Surface(),
                              ReferenceEquals(aCO2S.Surface2(), theOld) ? theNew : aCO2S.Surface2(),
                              aCO2S.Location(), aCO2S.Location2(), aCO2S.Continuity());
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }
    }

    private static void UpdateMapOfShapes(NCollection_Map<Shape> theMapOfShapes, ShapeBuild_ReShape theContext)
    {
        // OCCT adds to the map while it iterates over it; the shapes added are the context's results, which the
        // context maps onto themselves, so iterating over the shapes there before is the same
        foreach (Shape aShape in theMapOfShapes.ToArray())
        {
            Shape aContextShape = theContext.InternalApply(aShape);
            if (!aContextShape.IsSame(aShape))
            {
                theMapOfShapes.Add(aContextShape);
            }
        }
    }

    private static void SplitWire(in Wire                       theWire,
                                  in Face                       theFace,
                                  NCollection_IndexedMap<Shape> theVmap,
                                  NCollection_Sequence<Shape>   theWireSeq)
    {
        var aVEmap = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

        var aEmap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var itw   = new Iterator(theWire);
        for (; itw.More(); itw.Next())
        {
            Edge anEdge = ShapeCasts.Edge(itw.Value());
            if (!aEmap.Add(anEdge))
            {
                continue;
            }
            if (anEdge.Orientation() != Forward && anEdge.Orientation() != Reversed)
            {
                continue;
            }

            Vertex aVertex = Exploration.FirstVertex(anEdge, true); // with orientation
            if (aVEmap.IsBound(aVertex))
            {
                aVEmap[aVertex].Append(anEdge);
            }
            else
            {
                var aElist = new NCollection_List<Shape>();
                aElist.Append(anEdge);
                aVEmap.Bind(aVertex, aElist);
            }
        }

        var aBB = new BrepBuilder();
        for (int ii = 1; ii <= theVmap.Extent(); ii++)
        {
            Vertex                  anOrigin  = ShapeCasts.Vertex(theVmap[ii]);
            NCollection_List<Shape> aBranches = aVEmap[anOrigin];
            var                            anItl     = new NCollection_List<Shape>.Iterator(aBranches);
            while (anItl.More())
            {
                Edge CurEdge = ShapeCasts.Edge(anItl.Value());
                aBranches.Remove(anItl);

                var aNewWire = new Wire();
                aBB.MakeWire(ref aNewWire);
                for (;;)
                {
                    aBB.Add(aNewWire, CurEdge);

                    Vertex aVertex = Exploration.LastVertex(CurEdge, true); // with orientation
                    if (aVertex.IsSame(anOrigin))
                    {
                        break;
                    }

                    if (!aVEmap.IsBound(aVertex))
                    {
                        break;
                    }

                    NCollection_List<Shape> aElist = aVEmap[aVertex];
                    if (aElist.Extent() == 0)
                    {
                        break;
                    }

                    Curve2D? aPCurve = Tool.CurveOnSurface(CurEdge, theFace, out double fpar, out double lpar);
                    if (aElist.Extent() == 1)
                    {
                        CurEdge = ShapeCasts.Edge(aElist.First());
                        aElist.Clear();
                    }
                    else if (aPCurve is null)
                    {
                        CurEdge = ShapeCasts.Edge(aElist.First());
                        aElist.RemoveFirst();
                    }
                    else
                    {
                        double   aParam = (CurEdge.Orientation() == Forward) ? lpar : fpar;
                        Vector2D CurDir = aPCurve.EvalD1(aParam).D1;
                        CurDir.Normalize();
                        if (CurEdge.Orientation() == Reversed)
                        {
                            CurDir.Reverse();
                        }
                        // choose the rightest direction - the smallest angle
                        double MinAngle   = RealLast();
                        var    NextEdge   = new Edge();
                        var    aLocalIter = new NCollection_List<Shape>.Iterator(aElist);
                        for (; aLocalIter.More(); aLocalIter.Next())
                        {
                            Edge anEdge = ShapeCasts.Edge(aLocalIter.Value());
                            aPCurve = Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);
                            if (aPCurve is null)
                            {
                                // edge has no pcurve on the reference face, skip it as a direction candidate
                                continue;
                            }
                            aParam = (anEdge.Orientation() == Forward) ? fpar : lpar;
                            Vector2D aDir = aPCurve.EvalD1(aParam).D1;
                            aDir.Normalize();
                            if (anEdge.Orientation() == Reversed)
                            {
                                aDir.Reverse();
                            }
                            double anAngle = CurDir.Angle(aDir);
                            if (anAngle < MinAngle)
                            {
                                MinAngle = anAngle;
                                NextEdge = anEdge;
                            }
                        }
                        if (NextEdge.IsNull)
                        {
                            // all candidates lacked a pcurve, fall back to an arbitrary one instead of stalling
                            NextEdge = ShapeCasts.Edge(aElist.First());
                        }
                        CurEdge = NextEdge;
                        // Remove <CurEdge> from list
                        for (aLocalIter.Initialize(aElist); aLocalIter.More(); aLocalIter.Next())
                        {
                            if (CurEdge.IsSame(aLocalIter.Value()))
                            {
                                aElist.Remove(aLocalIter);
                                break;
                            }
                        }
                    } // else (more than one edge)
                } // for (;;)
                theWireSeq.Append(aNewWire);
            } // while (anItl.More())
        }
    }

    // Compares two sets of shapes. Returns true if they are the same, false otherwise.
    private static bool isSameSets(NCollection_Map<Shape>? theFShells1, NCollection_Map<Shape>? theFShells2)
    {
        // If both are null - no problem
        if (theFShells1 is null && theFShells2 is null)
        {
            return true;
        }
        // If only one is null - not the same
        if (theFShells1 is null || theFShells2 is null)
        {
            return false;
        }
        // Both not null
        if (theFShells1.Extent() != theFShells2.Extent())
        {
            return false;
        }
        // number of shells in each set should be very small in normal cases - max 2.
        // thus just check if all objects of one are contained in the other and vice versa.
        HashSet<Shape>.Enumerator it1 = theFShells1.GetEnumerator();
        HashSet<Shape>.Enumerator it2 = theFShells2.GetEnumerator();
        while (it1.MoveNext() && it2.MoveNext())
        {
            if (!theFShells1.Contains(it2.Current) || !theFShells2.Contains(it1.Current))
            {
                return false;
            }
        }
        return true;
    }

    private static void SetFixWireModes(ShapeFix_Face theSff)
    {
        ShapeFix_Wire aFixWire = theSff.FixWireTool();
        aFixWire.FixSelfIntersectionMode()             = 0;
        aFixWire.FixNonAdjacentIntersectingEdgesMode() = 0;
        aFixWire.FixLackingMode()                      = 0;
        aFixWire.FixNotchedEdgesMode()                 = 0;
        aFixWire.ModifyTopologyMode()                  = false;
        aFixWire.ModifyRemoveLoopMode()                = 0;
        aFixWire.FixGapsByRangesMode()                 = false;
        aFixWire.FixSmallMode()                        = 0;
    }

    // Checks the sequence of edges on the presence of shared vertex
    private static void CheckSharedVertices(NCollection_Sequence<Shape>                                       theSeqEdges,
                                            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEdgesVertex,
                                            NCollection_Map<Shape>                                            theMapKeepShape,
                                            NCollection_Map<Shape>                                            theShareVertMap)
    {
        var sae         = new ShapeAnalysis_Edge();
        var SeqVertexes = new NCollection_Sequence<Shape>();
        var MapVertexes = new NCollection_Map<Shape>(ShapeHasher.Instance);
        for (int k = 1; k <= theSeqEdges.Length(); k++)
        {
            Vertex aV1 = sae.FirstVertex(ShapeCasts.Edge(theSeqEdges[k]));
            Vertex aV2 = sae.LastVertex(ShapeCasts.Edge(theSeqEdges[k]));
            if (!MapVertexes.Add(aV1))
            {
                SeqVertexes.Append(aV1);
            }
            if (!MapVertexes.Add(aV2))
            {
                SeqVertexes.Append(aV2);
            }
        }

        for (int k = 1; k <= SeqVertexes.Length() /* && !IsSharedVertexPresent*/; k++)
        {
            NCollection_List<Shape> ListEdgesV1 = theMapEdgesVertex.FindFromKey(SeqVertexes[k]);
            if (ListEdgesV1.Extent() > 2 || theMapKeepShape.Contains(SeqVertexes[k]))
            {
                theShareVertMap.Add(SeqVertexes[k]);
            }
        }
        // return theShareVertMap.IsEmpty() ? false : true;
    }
}
