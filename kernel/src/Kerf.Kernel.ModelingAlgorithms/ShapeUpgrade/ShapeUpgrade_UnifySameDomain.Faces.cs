// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the file-static
// functions of the unification of faces; the class is in ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using DataMapOfFacePlane = System.Collections.Generic.Dictionary<Kerf.Topology.Shape, Kerf.Geometry.PlaneSurface>;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Upgrade;

internal partial class UnifySameDomain
{
    private static bool IsOnSingularity(NCollection_List<Shape> edgeList)
    {
        var itl = new NCollection_List<Shape>.Iterator(edgeList);
        for (; itl.More(); itl.Next())
        {
            Edge edge = ShapeCasts.Edge(itl.Value());
            if (Tool.Degenerated(edge))
            {
                return true;
            }
        }

        return false;
    }

    // only for seam edges assumed to be U- or V- isolines
    private static bool IsUiso(in Edge edge, in Face face)
    {
        var      bAcurve2d = new EdgeCurve2DAdaptor(edge, face);
        Point2D p2d;
        Vector2D vec;
        bAcurve2d.D1(bAcurve2d.FirstParameter(), out p2d, out vec);
        return (Math.Abs(vec.Y) > Math.Abs(vec.X));
    }

    private static bool IsLinear(EdgeCurveAdaptor bAcurve, ref Direction3D dir)
    {
        CurveKind type = bAcurve.GetType();

        if (type == CurveKind.Line)
        {
            dir = bAcurve.Line().Position.Direction;
            return true;
        }

        if ((type == CurveKind.BezierCurve || type == CurveKind.BSplineCurve)
            && bAcurve.PoleCount == 2)
        {
            Point3D firstPnt = bAcurve.Value(bAcurve.FirstParameter());
            Point3D lastPnt  = bAcurve.Value(bAcurve.LastParameter());
            dir           = new Direction3D(new Vector3D(firstPnt, lastPnt));
            return true;
        }

        return false;
    }

    private static double TrueValueOfOffset(double value, double period)
    {
        if (value > 0.0)
        {
            return period;
        }

        return (-period);
    }

    // Gets a face from the sequence that has a common boundary with already got <theUsedFaces>. If it is possible,
    // gets a face that has a common boundary in 2D space too (excludes seam edges). <theIndFace> returns the Index of
    // the face in the sequence.
    private static Face GetFaceFromSeq(
      NCollection_Sequence<Shape>                                       faces,
      in Face                                                           refFace,
      IndexedMap<Shape, NCollection_List<Shape>> mapEF,
      IndexedSet<Shape>                                     usedFaces,
      ref int                                                                  indFace)
    {
        if (usedFaces.Count == 0)
        {
            indFace = 1;
            usedFaces.Add(faces[1]);
            return ShapeCasts.Face(faces[1]);
        }

        var seamEdge   = new Edge();
        var targetFace = new Face();
        for (int ii = 0; ii < usedFaces.Count; ii++)
        {
            Face usedFace = ShapeCasts.Face(usedFaces[ii]);
            var         explo   = new Explorer(usedFace, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge                    edge    = ShapeCasts.Edge(explo.Current());
                NCollection_List<Shape> faceList = mapEF.GetValue(edge);
                if (faceList.Extent() < 2)
                {
                    continue;
                }

                Face face = (faceList.First().IsSame(usedFace))
                                      ? ShapeCasts.Face(faceList.Last())
                                      : ShapeCasts.Face(faceList.First());

                if (usedFaces.Contains(face))
                {
                    continue;
                }

                if (Tool.IsClosed(edge, refFace))
                {
                    seamEdge = edge;
                    continue;
                }

                targetFace = face;
                break;
            }
            if (targetFace.IsNull && !seamEdge.IsNull)
            {
                NCollection_List<Shape> faceList = mapEF.GetValue(seamEdge);
                targetFace = (faceList.First().IsSame(usedFace)) ? ShapeCasts.Face(faceList.Last())
                                                                    : ShapeCasts.Face(faceList.First());
            }
            if (!targetFace.IsNull)
            {
                break;
            }
        }

        usedFaces.Add(targetFace);
        for (int ii = 2; ii <= faces.Length(); ii++)
        {
            if (faces[ii].IsSame(targetFace))
            {
                indFace = ii;
                break;
            }
        }

        return targetFace;
    }

    private static void UpdateBoundaries(Curve2D pCurve,
                                         double       first,
                                         double       last,
                                         int          indCoord,
                                         ref double   minCoord,
                                         ref double   maxCoord)
    {
        const int nbSamples = 4;
        double    delta     = (last - first) / nbSamples;

        for (int i = 0; i <= nbSamples; i++)
        {
            double   param = first + i * delta;
            Point2D point = pCurve.Value(param);

            if (point.Coord(indCoord) < minCoord)
            {
                minCoord = point.Coord(indCoord);
            }

            if (point.Coord(indCoord) > maxCoord)
            {
                maxCoord = point.Coord(indCoord);
            }
        }
    }

    private static bool TryMakeLine(Curve2D pCurve, double first, double last, ref LineCurve2D? line)
    {
        Point2D firstPnt   = pCurve.Value(first);
        Point2D lastPnt    = pCurve.Value(last);
        var      vec        = new Vector2D(firstPnt, lastPnt);
        double   sqLen      = vec.SquareMagnitude;
        double   sqParamLen = (last - first) * (last - first);
        if (Math.Abs(sqLen - sqParamLen) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }

        var      dir     = new Direction2D(vec);
        Vector2D offset = -new Vector2D(dir);
        offset *= first;
        Point2D origin = firstPnt.Translated(offset);
        var      lin     = new Line2D(origin, dir);

        const int nbSamples = 10;
        double    delta    = (last - first) / nbSamples;
        for (int i = 1; i < nbSamples; i++)
        {
            double   param = first + i * delta;
            Point2D pnt   = pCurve.Value(param);
            double   dist  = lin.DistanceTo(pnt);
            if (dist > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }

        line = new LineCurve2D(lin);
        return true;
    }

    // Removes the specified edge from the vertex to edge map. Returns true if the edge was removed.
    private static bool RemoveEdgeFromMap(in Edge                                                         edge,
                                          IndexedMap<Shape, NCollection_List<Shape>> vertexToEdges)
    {
        bool isRemoved = false;
        Exploration.Vertices(edge, out Vertex firstVertex, out Vertex lastVertex);
        foreach (Vertex vertex in (ReadOnlySpan<Vertex>)[firstVertex, lastVertex])
        {
            if (!vertexToEdges.ContainsKey(vertex))
            {
                continue;
            }
            NCollection_List<Shape> vertexEdges = vertexToEdges.GetValue(vertex);
            var                            edgesIter  = new NCollection_List<Shape>.Iterator(vertexEdges);
            while (edgesIter.More())
            {
                Shape anEdge = edgesIter.Value();
                if (anEdge.IsSame(edge))
                {
                    isRemoved = true;
                    vertexEdges.Remove(edgesIter);
                }
                else
                {
                    edgesIter.Next();
                }
            }
        }
        return isRemoved;
    }

    private static double ComputeMinEdgeSize(NCollection_Sequence<Shape> edges,
                                             in Face                     refFace,
                                             HashSet<Shape>      edgesMap)
    {
        double minSize = RealLast();

        for (int ind = 1; ind <= edges.Length(); ind++)
        {
            Edge edge = ShapeCasts.Edge(edges[ind]);
            edgesMap.Add(edge);
            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            var bAcurve2d = new EdgeCurve2DAdaptor(edge, refFace);
            if (bAcurve2d.Curve() is null)
            {
                continue;
            }

            Point2D firstP2d = bAcurve2d.Value(bAcurve2d.FirstParameter());
            Point2D lastP2d  = bAcurve2d.Value(bAcurve2d.LastParameter());
            double   sqDist;
            if (v1.IsSame(v2) && !Tool.Degenerated(edge))
            {
                Point2D midP2d = bAcurve2d.Value((bAcurve2d.FirstParameter() + bAcurve2d.LastParameter()) / 2);
                sqDist = firstP2d.SquareDistanceTo(midP2d);
            }
            else
            {
                sqDist = firstP2d.SquareDistanceTo(lastP2d);
            }

            minSize = Min(minSize, sqDist);
        }
        minSize = Math.Sqrt(minSize);
        return minSize;
    }

    // Searching for origin of U in 2d space. Returns false if could not find curve on surface, true if succeed.
    private static bool FindCoordBounds(NCollection_Sequence<Shape>                                       faces,
                                        in Face                                                           refFace,
                                        IndexedMap<Shape, NCollection_List<Shape>> mapEF,
                                        HashSet<Shape>                                            edgesMap,
                                        int                                                                      indCoord,
                                        double                                                                   period,
                                        ref double                                                               minCoord,
                                        ref double                                                               maxCoord,
                                        ref int                                                                  numberOfIntervals,
                                        ref int                                                                  indFaceMax)
    {
        // OCCT's std::pair<double, double>, as a pair of fields of a class so that the sequence's items are written
        var pairSeq = new NCollection_Sequence<double[]>();

        double simpleMax = RealFirst();
        indFaceMax = 0;

        var usedFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (int ii = 1; ii <= faces.Length(); ii++)
        {
            int indFace = 0;
            // Get a face bordering with previous ones
            Face face     = GetFaceFromSeq(faces, refFace, mapEF, usedFaces, ref indFace);
            double      aMinCoord = RealLast(), aMaxCoord = RealFirst();
            var         explo     = new Explorer(face, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge edge = ShapeCasts.Edge(explo.Current());
                if (!edgesMap.Contains(edge))
                {
                    continue;
                }
                Curve2D? pCurve = Tool.CurveOnSurface(edge, refFace, out double fpar, out double lpar);
                if (pCurve is null)
                {
                    return false;
                }
                UpdateBoundaries(pCurve, fpar, lpar, indCoord, ref aMinCoord, ref aMaxCoord);
            }

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aMinCoord) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aMaxCoord))
            {
                continue;
            }

            if (aMaxCoord > simpleMax)
            {
                simpleMax    = aMaxCoord;
                indFaceMax = indFace;
            }

            // Insert new interval (aMinCoord, aMaxCoord) into sequence of intervals
            bool isInInterval = false;
            for (int jj = 1; jj <= pairSeq.Length(); jj++)
            {
                double localMin = pairSeq[jj][0];
                double localMax = pairSeq[jj][1];
                if (aMinCoord >= localMin && aMinCoord <= localMax && aMaxCoord >= localMin
                    && aMaxCoord <= localMax)
                {
                    isInInterval = true;
                    break;
                }

                if (aMinCoord < localMin && (aMaxCoord >= localMin && aMaxCoord <= localMax))
                {
                    pairSeq[jj][0] = aMinCoord;
                    isInInterval  = true;
                    break;
                }
                else if (aMinCoord < localMin && aMaxCoord > localMax)
                {
                    pairSeq[jj][0] = aMinCoord;
                    pairSeq[jj][1] = aMaxCoord;
                    isInInterval  = true;
                    break;
                }
                else if ((aMinCoord >= localMin && aMinCoord <= localMax) && aMaxCoord > localMax)
                {
                    pairSeq[jj][1] = aMaxCoord;
                    isInInterval  = true;
                    break;
                }
            }
            if (!isInInterval)
            {
                double[] interval = [aMinCoord, aMaxCoord];
                if (!pairSeq.IsEmpty() && aMaxCoord < pairSeq[1][0])
                {
                    pairSeq.Prepend(interval);
                }
                else
                {
                    pairSeq.Append(interval);
                }
            }
        }

        numberOfIntervals = pairSeq.Length();

        if (pairSeq.Length() == 2)
        {
            minCoord = pairSeq[2][0] - period;
        }
        else if (pairSeq.Length() > 0)
        {
            minCoord = pairSeq[1][0];
        }
        else
        {
            return false;
        }

        maxCoord = pairSeq[1][1];
        return true;
    }

    // Returns the start and end points of the edge in parametric space of the face. The orientation of the edge is
    // taken into account, so the start and end points will be swapped if the edge has a reversed orientation.
    private static (Point2D, Point2D) GetCurveParams(in Edge edge, in Face refFace)
    {
        var    curveAdaptor = new EdgeCurve2DAdaptor(edge, refFace);
        double firstParam   = curveAdaptor.FirstParameter();
        double lastParam    = curveAdaptor.LastParameter();
        if (edge.Orientation != Forward)
        {
            (firstParam, lastParam) = (lastParam, firstParam);
        }

        Point2D firstPoint = curveAdaptor.Value(firstParam);
        Point2D lastPoint  = curveAdaptor.Value(lastParam);
        return (firstPoint, lastPoint);
    }

    private static bool RelocatePCurvesToNewUorigin(
      NCollection_Sequence<Shape>                                       edges,
      in Shape                                                          firstFace,
      in Face                                                           refFace,
      double                                                                   coordTol,
      int                                                                      indCoord,
      double                                                                   period,
      IndexedMap<Shape, NCollection_List<Shape>> vEmap,
      Dictionary<Shape, Curve2D>                          edgeNewPCurve,
      HashSet<Shape>                                            usedEdges)
    {
        var edgesOfFirstFace = new HashSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(firstFace, edgesOfFirstFace);

        for (;;) // walk by contours
        {
            // try to find the start edge that:
            // 1. belongs to outer edges of first face;
            // 2. has minimum U-coord of its start point
            var                startEdge    = new Edge();
            Orientation orientation = Forward;
            double             coordMin     = RealLast();
            for (int edgeIndex = 1; edgeIndex <= edges.Length(); ++edgeIndex)
            {
                Edge edge = ShapeCasts.Edge(edges[edgeIndex]);
                if (usedEdges.Contains(edge))
                {
                    continue;
                }

                if (!edgesOfFirstFace.Contains(edge))
                {
                    continue;
                }

                if (startEdge.IsNull)
                {
                    startEdge = edge;
                    (Point2D firstPoint, Point2D lastPoint) = GetCurveParams(edge, refFace);
                    if (firstPoint.Coord(indCoord) < lastPoint.Coord(indCoord))
                    {
                        coordMin     = firstPoint.Coord(indCoord);
                        orientation = Forward;
                    }
                    else
                    {
                        coordMin     = lastPoint.Coord(indCoord);
                        orientation = Reversed;
                    }
                }
                else
                {
                    (Point2D firstPoint, Point2D lastPoint) = GetCurveParams(edge, refFace);
                    if (firstPoint.Coord(indCoord) < coordMin)
                    {
                        startEdge    = edge;
                        coordMin     = firstPoint.Coord(indCoord);
                        orientation = Forward;
                    }
                    if (lastPoint.Coord(indCoord) < coordMin)
                    {
                        startEdge    = edge;
                        coordMin     = lastPoint.Coord(indCoord);
                        orientation = Reversed;
                    }
                }
            }

            if (startEdge.IsNull) // all contours are passed
            {
                break;
            }

            Edge   currentEdge = startEdge;
            Curve2D? curPCurve =
              Tool.CurveOnSurface(currentEdge, refFace, out double edgeStartParam, out double edgeEndParam);
            if (curPCurve is null)
            {
                return false;
            }
            curPCurve = new TrimmedCurve2D(curPCurve, edgeStartParam, edgeEndParam);
            edgeNewPCurve[currentEdge] = curPCurve;
            if (currentEdge.Orientation == Reversed)
            {
                (edgeStartParam, edgeEndParam) = (edgeEndParam, edgeStartParam);
            }
            double   curParam = (orientation == Forward) ? edgeEndParam : edgeStartParam;
            Point2D curPoint = curPCurve.Value(curParam);

            for (;;) // collect pcurves of a contour
            {
                if (!RemoveEdgeFromMap(currentEdge, vEmap))
                {
                    break; // end of contour in 2d
                }
                usedEdges.Add(currentEdge);
                Vertex curVertex = (orientation == Forward)
                                            ? Exploration.LastVertex(currentEdge, true)
                                            : Exploration.FirstVertex(currentEdge, true);

                NCollection_List<Shape> elist = vEmap.GetValue(curVertex);
                if (elist.IsEmpty())
                {
                    break; // end of contour in 3d
                }

                for (var itl = new NCollection_List<Shape>.Iterator(elist); itl.More(); itl.Next())
                {
                    Edge edge = ShapeCasts.Edge(itl.Value());
                    if (edge.IsSame(currentEdge))
                    {
                        continue;
                    }

                    Vertex firstVertex = (orientation == Forward)
                                                   ? Exploration.FirstVertex(edge, true)
                                                   : Exploration.LastVertex(edge, true);
                    if (!firstVertex.IsSame(curVertex)) // may be if CurVertex is deg.vertex
                    {
                        continue;
                    }

                    Curve2D? pCurve =
                      Tool.CurveOnSurface(edge, refFace, out edgeStartParam, out edgeEndParam);
                    if (pCurve is null)
                    {
                        return false;
                    }
                    pCurve = new TrimmedCurve2D(pCurve, edgeStartParam, edgeEndParam);
                    if (edge.Orientation == Reversed)
                    {
                        (edgeStartParam, edgeEndParam) = (edgeEndParam, edgeStartParam);
                    }
                    double   param   = (orientation == Forward) ? edgeStartParam : edgeEndParam;
                    Point2D point   = pCurve.Value(param);
                    double   offset = curPoint.Coord(indCoord) - point.Coord(indCoord);
                    if (Math.Abs(offset) >= coordTol && Math.Abs(Math.Abs(offset) - period) >= coordTol)
                    {
                        continue; // may be if CurVertex is deg.vertex
                    }

                    if (Math.Abs(offset) > period / 2)
                    {
                        offset = TrueValueOfOffset(offset, period);
                        var vec = new Vector2D();
                        if (indCoord == 1)
                        {
                            vec = vec.WithCoord(offset, 0.0);
                        }
                        else
                        {
                            vec = vec.WithCoord(0.0, offset);
                        }
                        var newPCurve = (Curve2D)pCurve.Copy();
                        newPCurve.Translate(vec);
                        pCurve = newPCurve;
                    }
                    edgeNewPCurve[edge] = pCurve;
                    currentEdge = edge;
                    Orientation curOr = global::Kerf.Engine.Topology.TopAbs.Compose(orientation, currentEdge.Orientation);
                    curParam = (curOr == Forward) ? pCurve.LastParameter : pCurve.FirstParameter;
                    curPoint = pCurve.Value(curParam);
                    break;
                }
            } // for (;;) (collect pcurves of a contour)
        } // for (;;) (walk by contours)
        return true;
    }

    private static void InsertWiresIntoFaces(NCollection_Sequence<Shape> wires,
                                             NCollection_Sequence<Shape> faces,
                                             in Face                     refFace)
    {
        var bb = new BrepBuilder();
        for (int ii = 1; ii <= wires.Length(); ii++)
        {
            Wire wire     = ShapeCasts.Wire(wires[ii]);
            var         iter      = new Iterator(wire);
            Edge edge    = ShapeCasts.Edge(iter.Value());
            var         bAcurve2d = new EdgeCurve2DAdaptor(edge, refFace);
            Point2D    pnt2d    = bAcurve2d.Value((bAcurve2d.FirstParameter() + bAcurve2d.LastParameter()) / 2.0);
            var         requiredFace = new Shape();
            for (int jj = 1; jj <= faces.Length(); jj++)
            {
                Face   face      = ShapeCasts.Face(faces[jj]);
                var           classifier = new FClass2d(face, global::Kerf.Engine.Foundation.Precision.Confusion());
                Containment  status    = classifier.Perform(pnt2d);
                if (status == In)
                {
                    requiredFace = face.Oriented(Forward);
                    break;
                }
            }
            if (!requiredFace.IsNull)
            {
                bb.Add(requiredFace, wire);
            }
            else
            {
                // Standard_ASSERT_INVOKE("ShapeUpgrade_UnifySameDomain: wire remains unclassified"): a debug report
            }
        }
    }

    private static Face FindCommonFace(in Edge                                                         edge1,
                                              in Edge                                                         edge2,
                                              IndexedMap<Shape, NCollection_List<Shape>> vFmap,
                                              ref Orientation                                                 orOfE1OnFace,
                                              ref Orientation                                                 orOfE2OnFace)
    {
        var vertex = new Vertex();
        Exploration.CommonVertex(edge1, edge2, ref vertex);
        NCollection_List<Shape> flist = vFmap.GetValue(vertex);
        var                            itl   = new NCollection_List<Shape>.Iterator(flist);
        for (; itl.More(); itl.Next())
        {
            Face face = ShapeCasts.Face(itl.Value());
            face.Orient(Forward);
            bool e1found = false, e2found = false;
            var  explo   = new Explorer(face, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Shape edge = explo.Current();
                if (edge.IsSame(edge1))
                {
                    e1found         = true;
                    orOfE1OnFace = edge.Orientation;
                }
                if (edge.IsSame(edge2))
                {
                    e2found         = true;
                    orOfE2OnFace = edge.Orientation;
                }
                if (e1found && e2found)
                {
                    return face;
                }
            }
        }

        var nullFace = new Face();
        return nullFace;
    }

    private static bool FindClosestPoints(in Edge                                                         edge1,
                                          in Edge                                                         edge2,
                                          IndexedMap<Shape, NCollection_List<Shape>> vFmap,
                                          ref Face                                                        commonFace,
                                          ref double                                                             minSqDist,
                                          ref Orientation                                                 orOfE1OnFace,
                                          ref Orientation                                                 orOfE2OnFace,
                                          ref int                                                                indOnE1,
                                          ref int                                                                indOnE2,
                                          Span<Point2D>                                                         pointsOnEdge1,
                                          Span<Point2D>                                                         pointsOnEdge2)
    {
        commonFace = FindCommonFace(edge1, edge2, vFmap, ref orOfE1OnFace, ref orOfE2OnFace);
        if (commonFace.IsNull)
        {
            return false;
        }

        Curve2D? pCurve1 = Tool.CurveOnSurface(edge1, commonFace, out double fpar1, out double lpar1);
        Curve2D? pCurve2 = Tool.CurveOnSurface(edge2, commonFace, out double fpar2, out double lpar2);
        if (pCurve1 is null || pCurve2 is null)
        {
            return false;
        }
        pointsOnEdge1[0] = pCurve1.Value(fpar1);
        pointsOnEdge1[1] = pCurve1.Value(lpar1);
        pointsOnEdge2[0] = pCurve2.Value(fpar2);
        pointsOnEdge2[1] = pCurve2.Value(lpar2);
        minSqDist     = RealLast();
        indOnE1       = -1;
        indOnE2       = -1;
        for (int ind1 = 0; ind1 < 2; ind1++)
        {
            for (int ind2 = 0; ind2 < 2; ind2++)
            {
                double sqDist = pointsOnEdge1[ind1].SquareDistanceTo(pointsOnEdge2[ind2]);
                if (sqDist < minSqDist)
                {
                    minSqDist = sqDist;
                    indOnE1   = ind1;
                    indOnE2   = ind2;
                }
            }
        }
        return true;
    }

    private static void ReconstructMissedSeam(NCollection_Sequence<Shape> removedEdges,
                                              in Face                     frefFace,
                                              in Edge                     curEdge,
                                              in Vertex                   curVertex,
                                              in Point2D                        curPoint,
                                              double                             uperiod,
                                              double                             vperiod,
                                              ref Edge                    seamEdge,
                                              ref Point2D                       nextPoint)
    {
        _ = Tool.Surface(frefFace); // RefSurf: unused, as in OCCT

        // Find seam edge between removed edges
        seamEdge.Nullify();
        for (int i = 1; i <= removedEdges.Length(); i++)
        {
            Edge edge = ShapeCasts.Edge(removedEdges[i]);
            if (edge.IsSame(curEdge))
            {
                continue;
            }
            Curve2D? pc = Tool.CurveOnSurface(edge, frefFace, out double param1, out double param2);
            if (pc is null)
            {
                continue;
            }

            Exploration.Vertices(edge, out Vertex firstVertex, out Vertex lastVertex, true);

            if ((firstVertex.IsSame(curVertex) || lastVertex.IsSame(curVertex))
                && Tool.IsClosed(edge, frefFace))
            {
                double   param = (edge.Orientation == Forward) ? param1 : param2;
                Point2D point = pc.Value(param);
                double   udiff = Math.Abs(point.X - curPoint.X);
                double   vdiff = Math.Abs(point.Y - curPoint.Y);
                if ((uperiod != 0.0 && udiff > uperiod / 2) || (vperiod != 0.0 && vdiff > vperiod / 2))
                {
                    if (lastVertex.IsSame(curVertex) || (uperiod != 0.0 && vperiod != 0.0))
                    {
                        edge.Reverse();
                    }
                    else
                    {
                        Orientation ori = edge.Orientation;
                        edge.Orient(Forward);
                        Curve2D? pc1 = Tool.CurveOnSurface(edge, frefFace, out param1, out param2);
                        edge.Reverse();
                        Curve2D? pc2 = Tool.CurveOnSurface(edge, frefFace, out param1, out param2);
                        edge.Reverse(); // again FORWARD
                        // OCCT: rewrites the pcurves of a removed seam of the input (the port's copy)
                        var           bb   = new BrepBuilder();
                        double        tol  = Tool.Tolerance(edge);
                        Surface? surf = Tool.Surface(frefFace, out Placement loc);
                        bb.UpdateEdge(edge, pc2, pc1, surf, loc, tol);
                        edge.Orient(ori);
                    }
                    pc    = Tool.CurveOnSurface(edge, frefFace, out param1, out param2)!;
                    param = (edge.Orientation == Forward) ? param1 : param2;
                    point = pc.Value(param);
                    udiff = Math.Abs(point.X - curPoint.X);
                    vdiff = Math.Abs(point.Y - curPoint.Y);
                }
                if ((uperiod == 0.0 || udiff < uperiod / 2) && (vperiod == 0.0 || vdiff < vperiod / 2))
                {
                    firstVertex = Exploration.FirstVertex(edge, true);
                    if (firstVertex.IsSame(curVertex))
                    {
                        seamEdge = edge;
                        break;
                    }
                }
            }
        }

        if (!seamEdge.IsNull)
        {
            Curve2D pc    = Tool.CurveOnSurface(seamEdge, frefFace, out double param1, out double param2)!;
            double       param = (seamEdge.Orientation == Forward) ? param2 : param1;
            nextPoint        = pc.Value(param);
        }
    }

    private static readonly double[] SameSurfCoefs = [0.3399811, 0.7745966];

    private static bool SameSurf(Surface s1, Surface s2)
    {
        double[] coefs = SameSurfCoefs;

        s1.ParameterBounds(out double uf1, out double ul1, out double vf1, out double vl1);
        s2.ParameterBounds(out double uf2, out double ul2, out double vf2, out double vl2);
        double pTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
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
                if (Math.Abs(uf1 - uf2) > pTol)
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
                if (Math.Abs(vf1 - vf2) > pTol)
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
                if (Math.Abs(ul1 - ul2) > pTol)
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
                if (Math.Abs(vl1 - vl2) > pTol)
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
            u = uf1 + coefs[i] * du;
            for (j = 0; j < 2; ++j)
            {
                v = vf1 + coefs[j] * dv;
                Point3D p1 = s1.Value(u, v);
                Point3D p2 = s2.Value(u, v);
                if (!p1.IsEqual(p2, pTol))
                {
                    return false;
                }
            }
        }

        return true;
    }

    // OCCT: writes the reference surface's pcurves and a new range into the edges of the faces to unify, which are the
    // input's own even in the safe input mode (the port works on its copy, see Build)
    private static void TransformPCurves(in Face                refFace,
                                         in Face                face,
                                         HashSet<Shape> mapEdgesWithTemporaryPCurves)
    {
        Surface refSurf = Tool.Surface(refFace)!;
        if (refSurf is TrimmedSurface refRTS)
        {
            refSurf = refRTS.BasisSurface;
        }

        Surface surfFace = Tool.Surface(face)!;
        if (surfFace is TrimmedSurface faceRTS)
        {
            surfFace = faceRTS.BasisSurface;
        }

        bool toModify = false, toTranslate = false, toRotate = false, X_Reverse = false, Y_Reverse = false,
             toProject = false;

        double translation = 0.0, angle = 0.0;

        // Get axes of surface of face and of surface of RefFace
        var elemSurfFace = surfFace as ElementarySurface;
        var elemRefSurf  = refSurf as ElementarySurface;

        if (elemSurfFace is not null && elemRefSurf is not null)
        {
            CoordinateSystem axisOfSurfFace = elemSurfFace.Position;
            CoordinateSystem axisOfRefSurf  = elemRefSurf.Position;

            Point3D originRefSurf = axisOfRefSurf.Location;

            double param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(axisOfSurfFace.Axis, originRefSurf);

            if (Math.Abs(param) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                translation = -param;
            }

            Direction3D vdirSurfFace = axisOfSurfFace.Direction;
            Direction3D vdirRefSurf  = axisOfRefSurf.Direction;
            Direction3D xdirSurfFace = axisOfSurfFace.XDirection;
            Direction3D xdirRefSurf  = axisOfRefSurf.XDirection;

            Direction3D crossProd1 = axisOfRefSurf.XDirection ^ axisOfRefSurf.YDirection;
            Direction3D crossProd2 = axisOfSurfFace.XDirection ^ axisOfSurfFace.YDirection;
            if (crossProd1 * crossProd2 < 0.0)
            {
                X_Reverse = true;
            }

            double scalProd = vdirSurfFace * vdirRefSurf;
            if (scalProd < 0.0)
            {
                Y_Reverse = true;
            }

            if (!X_Reverse && !Y_Reverse)
            {
                Direction3D dirRef = vdirRefSurf;
                if (!axisOfRefSurf.IsDirect)
                {
                    dirRef = dirRef.Reversed();
                }
                angle = xdirRefSurf.AngleWithRef(xdirSurfFace, dirRef);
            }
            else
            {
                angle = xdirRefSurf.AngleTo(xdirSurfFace);
            }

            toRotate = (Math.Abs(angle) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            toTranslate = (Math.Abs(translation) > global::Kerf.Engine.Foundation.Precision.PConfusion());

            toModify = toTranslate || toRotate || X_Reverse || Y_Reverse;
        }
        else
        {
            if (!SameSurf(refSurf, surfFace))
            {
                toProject = true;
            }
        }

        var bb    = new BrepBuilder();
        var explo = new Explorer(face, ShapeType.Edge);
        var emap = new HashSet<Shape>(ShapeHasher.Instance);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            if (Tool.Degenerated(edge) && toModify)
            {
                continue;
            }

            if (toProject && Tool.IsClosed(edge, face))
            {
                continue;
            }

            if (!emap.Add(edge))
            {
                continue;
            }

            Orientation or = edge.Orientation;

            double        fpar, lpar;
            Curve2D? pCurveOnRef = Tool.CurveOnSurface(edge, refFace, out fpar, out lpar);
            Curve2D? pCurve2     = null;
            if (pCurveOnRef is not null /* && !(ToModify || ToProject)*/)
            {
                edge.Reverse();
                pCurve2 = Tool.CurveOnSurface(edge, refFace, out fpar, out lpar);
                if (!ReferenceEquals(pCurve2, pCurveOnRef))
                { // two pcurves
                    continue;
                }
            }

            var pCurves    = new Curve2D?[2];
            var newPCurves = new Curve2D?[2];
            edge.Orient(Forward);
            pCurves[0] = Tool.CurveOnSurface(edge, face, out fpar, out lpar);
            edge.Reverse();
            pCurves[1] = Tool.CurveOnSurface(edge, face, out fpar, out lpar);

            if (pCurves[0] is null || pCurves[1] is null)
            {
                // The source face has no pcurve for this edge, so it cannot be transformed.
                continue;
            }

            int nbPcurves = ReferenceEquals(pCurves[0], pCurves[1]) ? 1 : 2;

            for (int ii = 0; ii < nbPcurves; ii++)
            {
                if (toProject)
                {
                    Curve3D? c3d = Tool.Curve(edge, out fpar, out lpar);
                    c3d = new TrimmedCurve3D(c3d!, fpar, lpar);
                    double tol = Tool.Tolerance(edge);
                    tol = Min(tol, global::Kerf.Engine.Foundation.Precision.Approximation());
                    // (ToProject excludes the transformations below, so the projection is never edited)
                    newPCurves[ii] = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c3d, refSurf);
                }
                else
                {
                    newPCurves[ii] = (Curve2D)pCurves[ii]!.Copy();
                }
                if (toTranslate)
                {
                    newPCurves[ii]!.Translate(new Vector2D(0.0, translation));
                }
                if (Y_Reverse)
                {
                    newPCurves[ii]!.Mirror(Constants.OX2d());
                }
                if (X_Reverse)
                {
                    newPCurves[ii]!.Mirror(Constants.OY2d());
                    newPCurves[ii]!.Translate(new Vector2D(2 * M_PI, 0.0));
                }
                if (toRotate)
                {
                    newPCurves[ii]!.Translate(new Vector2D(angle, 0.0));
                }
            }

            edge.Orient(Forward);

            if (nbPcurves == 1)
            {
                if (pCurve2 is null || (!refSurf.IsUClosed && !refSurf.IsVClosed))
                {
                    bb.UpdateEdge(edge, newPCurves[0], refFace, 0.0);
                    mapEdgesWithTemporaryPCurves.Add(edge);
                }
                else
                {
                    // check: may be it is the same pcurve
                    refSurf.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);
                    double   uperiod      = (refSurf.IsUClosed) ? (umax - umin) : 0.0;
                    double   vperiod      = (refSurf.IsVClosed) ? (vmax - vmin) : 0.0;
                    Point2D p2dOnPCurve1 = pCurveOnRef!.Value(fpar);
                    Point2D p2dOnPCurve2 = newPCurves[0]!.Value(fpar);
                    if ((uperiod != 0.0 && Math.Abs(p2dOnPCurve1.X - p2dOnPCurve2.X) > uperiod / 2)
                        || (vperiod != 0.0 && Math.Abs(p2dOnPCurve1.Y - p2dOnPCurve2.Y) > vperiod / 2))
                    {
                        Curve2D? nullPCurve = null;
                        bb.UpdateEdge(edge, nullPCurve, refFace, 0.0);
                        if (or == Forward)
                        {
                            bb.UpdateEdge(edge, newPCurves[0], pCurveOnRef, refFace, 0.0);
                        }
                        else
                        {
                            bb.UpdateEdge(edge, pCurveOnRef, newPCurves[0], refFace, 0.0);
                        }

                        mapEdgesWithTemporaryPCurves.Add(edge);
                    }
                }
            }
            else
            {
                bb.UpdateEdge(edge, newPCurves[0], newPCurves[1], refFace, 0.0);
                mapEdgesWithTemporaryPCurves.Add(edge);
            }

            bb.Range(edge, fpar, lpar);
        }
    }

    private static void AddPCurves(NCollection_Sequence<Shape> faces,
                                   in Face                     refFace,
                                   HashSet<Shape>      mapEdgesWithTemporaryPCurves)
    {
        var refBAsurf = new FaceSurfaceAdaptor(refFace, false);

        SurfaceKind type = refBAsurf.GetType();
        if (type == Plane)
        {
            return;
        }

        for (int i = 1; i <= faces.Length(); i++)
        {
            Face face = ShapeCasts.Face(faces[i]);
            face.Orient(Forward);
            if (face.IsSame(refFace))
            {
                continue;
            }

            TransformPCurves(refFace, face, mapEdgesWithTemporaryPCurves);
        }
    }

    // Adds edges from the shape to the sequence; seams and equal edges are dropped. Returns true if one of original
    // edges dropped.
    private static bool AddOrdinaryEdges(NCollection_Sequence<Shape> edges,
                                         in Shape                    aShape,
                                         ref int                            anIndex,
                                         NCollection_Sequence<Shape> removedEdges)
    {
        // map of edges
        var newEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        // add edges without seams
        for (var exp = new Explorer(aShape, ShapeType.Edge); exp.More(); exp.Next())
        {
            Shape edge = exp.Current();
            if (newEdges.Contains(edge))
            {
                newEdges.Remove(edge);
                removedEdges.Append(edge);
            }
            else
            {
                newEdges.Add(edge);
            }
        }

        bool isDropped = false;
        // merge edges and drop seams
        int i;
        for (i = 1; i <= edges.Length(); i++)
        {
            Shape current = edges[i];
            if (newEdges.Contains(current))
            {
                newEdges.Remove(current);
                edges.Remove(i);
                removedEdges.Append(current);
                i--;

                if (!isDropped)
                {
                    isDropped = true;
                    anIndex   = i;
                }
            }
        }

        // add edges to the sequence
        for (i = 1; i <= newEdges.Count; i++)
        {
            edges.Append(newEdges[i - 1]);
        }

        return isDropped;
    }

    private static bool GetCylinder(Surface inSurface, ref Cylinder outCylinder)
    {
        bool isCylinder = false;

        if (inSurface is CylindricalSurface gc)
        {
            outCylinder = gc.Cylinder;
            isCylinder     = true;
        }
        else if (inSurface is RevolutionSurface rs)
        {
            Curve3D basis = rs.BasisCurve;

            while (basis is TrimmedCurve3D tc)
            {
                basis = tc.BasisCurve;
            }

            if (basis is LineCurve3D basisLine)
            {
                Direction3D dir      = rs.Direction;
                Direction3D basisDir = basisLine.Position.Direction;
                if (basisDir.IsParallel(dir, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // basis line is parallel to the revolution axis: it is a cylinder
                    Point3D loc   = rs.Location;
                    double r     = basisLine.Line.DistanceTo(loc);
                    var    cylAx = new CoordinateSystem(loc, dir);

                    outCylinder = new Cylinder(cylAx, r);
                    isCylinder     = true;
                }
            }
        }
        else if (inSurface is ExtrusionSurface les)
        {
            Curve3D basis = les.BasisCurve;

            while (basis is TrimmedCurve3D tc)
            {
                basis = tc.BasisCurve;
            }

            if (basis is CircleCurve3D basisCircle)
            {
                Direction3D dir      = les.Direction;
                Direction3D basisDir = basisCircle.Position.Direction;
                if (basisDir.IsParallel(dir, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    // basis circle is normal to the extrusion axis: it is a cylinder
                    Point3D loc   = basisCircle.Location;
                    double r     = basisCircle.Radius;
                    var    cylAx = new CoordinateSystem(loc, dir);

                    outCylinder = new Cylinder(cylAx, r);
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
        var rts = aSurface as TrimmedSurface;
        return rts is null ? aSurface : rts.BasisSurface;
    }

    // Gets the normal to surface by the given parameter on edge. Returns True if normal was computed.
    private static bool GetNormalToSurface(in Face face, in Edge edge, double p, ref Direction3D normal)
    {
        double f, l;
        // get 2d curve to get point in 2d
        Curve2D? c2d;
        if (Tool.IsClosed(edge, face))
        {
            // Find the edge in the face: it will have correct orientation
            var         edgeInFace = new Edge();
            Face aFace        = face;
            aFace.Orient(Forward);
            var explo = new Explorer(aFace, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(explo.Current());
                if (anEdge.IsSame(edge))
                {
                    edgeInFace = anEdge;
                    break;
                }
            }
            if (edgeInFace.IsNull)
            {
                return false;
            }

            c2d = Tool.CurveOnSurface(edgeInFace, aFace, out f, out l);
        }
        else
        {
            c2d = Tool.CurveOnSurface(edge, face, out f, out l);
        }

        if (c2d is null)
        {
            return false;
        }
        //
        // 2d point
        c2d.D0(p, out Point2D p2d);
        //
        // get D1
        Surface s = Tool.Surface(face, out Placement loc)!;
        s.D1(p2d.X, p2d.Y, out Point3D p3d, out Vector3D du, out Vector3D dv);
        //
        // compute normal
        Vector3D vNormal = du.Crossed(dv);
        if (vNormal.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }
        //
        if (face.Orientation == Reversed)
        {
            vNormal = vNormal.Reversed();
        }
        //
        vNormal = vNormal.Transformed(loc.Transformation);
        normal = new Direction3D(vNormal);
        return true;
    }

    private static bool IsSameDomain(in Face        aFace,
                                     in Face        aCheckedFace,
                                     double                linTol,
                                     double                angTol,
                                     DataMapOfFacePlane    facePlaneMap)
    {
        // checking the same handles
        Surface? s1, s2;

        s1 = Tool.Surface(aFace, out Placement l1);
        s2 = Tool.Surface(aCheckedFace, out Placement l2);

        if (ReferenceEquals(s1, s2) && l1.IsEqual(l2))
        {
            return true;
        }

        s1 = Tool.Surface(aFace);
        s2 = Tool.Surface(aCheckedFace);

        s1 = ClearRts(s1)!;
        s2 = ClearRts(s2)!;

        // case of two planar surfaces:
        // all kinds of surfaces checked, including b-spline and bezier
        var planarityChecker1 = new IsPlanarSurface(s1, linTol);
        if (planarityChecker1.IsPlanar)
        {
            var planarityChecker2 = new IsPlanarSurface(s2, linTol);
            if (planarityChecker2.IsPlanar)
            {
                Plane3D pln1 = planarityChecker1.Plan();
                Plane3D pln2 = planarityChecker2.Plan();

                if (pln1.Position.Direction.IsParallel(pln2.Position.Direction, angTol)
                    && pln1.DistanceTo(pln2) < linTol)
                {
                    PlaneSurface planeOfFaces;
                    if (facePlaneMap.ContainsKey(aFace))
                    {
                        planeOfFaces = facePlaneMap.GetValue(aFace);
                    }
                    else if (facePlaneMap.ContainsKey(aCheckedFace))
                    {
                        planeOfFaces = facePlaneMap.GetValue(aCheckedFace);
                    }
                    else
                    {
                        planeOfFaces = new PlaneSurface(pln1);
                    }

                    facePlaneMap[aFace] = planeOfFaces;
                    facePlaneMap[aCheckedFace] = planeOfFaces;

                    return true;
                }
            }
        }

        // case of two elementary surfaces: use OCCT tool
        // elementary surfaces: ConicalSurface, CylindricalSurface,
        //                      Plane, SphericalSurface and ToroidalSurface
        if (s1 is ElementarySurface && s2 is ElementarySurface)
        {
            var ga1 = new SurfaceAdaptor(s1);
            var ga2 = new SurfaceAdaptor(s2);

            var tt1 = new BrepTopolTool();
            var tt2 = new BrepTopolTool();

            try
            {
                var iiInt = new ImpImpIntersection(ga1, tt1, ga2, tt2, linTol, linTol);
                if (!iiInt.IsDone || iiInt.IsEmpty)
                {
                    return false;
                }

                return iiInt.TangentFaces();
            }
            catch (FailureException)
            {
                return false;
            }
        }

        // case of two cylindrical surfaces, at least one of which is a swept surface
        // swept surfaces: SurfaceOfLinearExtrusion, SurfaceOfRevolution
        if ((s1 is CylindricalSurface || s1 is SweptSurface)
            && (s2 is CylindricalSurface || s2 is SweptSurface))
        {
            var cyl1 = new Cylinder();
            var cyl2 = new Cylinder();
            if (GetCylinder(s1, ref cyl1) && GetCylinder(s2, ref cyl2))
            {
                if (Math.Abs(cyl1.Radius - cyl2.Radius) < linTol)
                {
                    Direction3D dir1 = cyl1.Position.Direction;
                    Direction3D dir2 = cyl2.Position.Direction;
                    if (dir1.IsParallel(dir2, global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        Point3D loc1  = cyl1.Location;
                        Point3D loc2  = cyl2.Location;
                        var    vec12 = new Vector3D(loc1, loc2);
                        if (vec12.SquareMagnitude < linTol * linTol
                            || vec12.IsParallel(new Vector3D(dir1), global::Kerf.Engine.Foundation.Precision.Angular()))
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
    private static void ReplaceSurface(Surface old, Surface theNew, IEnumerable<Shape> roots)
    {
        var faces = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var edges = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        foreach (Shape root in roots)
        {
            if (root.IsNull)
            {
                continue;
            }
            for (var exp = new Explorer(root, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().TShape() is BrepTFace tFace && faces.Add(tFace)
                    && ReferenceEquals(tFace.Surface(), old))
                {
                    tFace.Surface(theNew);
                }
            }
            for (var exp = new Explorer(root, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (exp.Current().TShape() is not BrepTEdge tEdge || !edges.Add(tEdge))
                {
                    continue;
                }
                var it = new NCollection_List<CurveRepresentation>.Iterator(tEdge.ChangeCurves());
                for (; it.More(); it.Next())
                {
                    switch (it.Value())
                    {
                        case CurveOnClosedSurface cocs when ReferenceEquals(cocs.Surface(), old):
                        {
                            var aNew = new CurveOnClosedSurface(cocs.PCurve()!, cocs.PCurve2(), theNew, cocs.Location(),
                                                                     cocs.Continuity());
                            aNew.SetRange(cocs.First(), cocs.Last());
                            cocs.UVPoints(out Point2D p1, out Point2D p2);
                            aNew.SetUVPoints(p1, p2);
                            cocs.UVPoints2(out p1, out p2);
                            aNew.SetUVPoints2(p1, p2);
                            it.ChangeValue() = aNew;
                            break;
                        }
                        case CurveOnSurfaceRepresentation cos when ReferenceEquals(cos.Surface(), old):
                        {
                            var aNew = new CurveOnSurfaceRepresentation(cos.PCurve()!, theNew, cos.Location());
                            aNew.SetRange(cos.First(), cos.Last());
                            cos.UVPoints(out Point2D p1, out Point2D p2);
                            aNew.SetUVPoints(p1, p2);
                            it.ChangeValue() = aNew;
                            break;
                        }
                        case CurveOn2Surfaces co2S when ReferenceEquals(co2S.Surface(), old)
                                                             || ReferenceEquals(co2S.Surface2(), old):
                        {
                            it.ChangeValue() = new CurveOn2Surfaces(
                              ReferenceEquals(co2S.Surface(), old) ? theNew : co2S.Surface(),
                              ReferenceEquals(co2S.Surface2(), old) ? theNew : co2S.Surface2(),
                              co2S.Location(), co2S.Location2(), co2S.Continuity());
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }
    }

    private static void UpdateMapOfShapes(HashSet<Shape> mapOfShapes, ReShapeBuilder context)
    {
        // OCCT adds to the map while it iterates over it; the shapes added are the context's results, which the
        // context maps onto themselves, so iterating over the shapes there before is the same
        foreach (Shape shape in mapOfShapes.ToArray())
        {
            Shape contextShape = context.InternalApply(shape);
            if (!contextShape.IsSame(shape))
            {
                mapOfShapes.Add(contextShape);
            }
        }
    }

    private static void SplitWire(in Wire                       wire,
                                  in Face                       face,
                                  IndexedSet<Shape> vmap,
                                  NCollection_Sequence<Shape>   wireSeq)
    {
        var vEmap = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

        var emap = new HashSet<Shape>(ShapeHasher.Instance);
        var itw   = new Iterator(wire);
        for (; itw.More(); itw.Next())
        {
            Edge edge = ShapeCasts.Edge(itw.Value());
            if (!emap.Add(edge))
            {
                continue;
            }
            if (edge.Orientation != Forward && edge.Orientation != Reversed)
            {
                continue;
            }

            Vertex vertex = Exploration.FirstVertex(edge, true); // with orientation
            if (vEmap.ContainsKey(vertex))
            {
                vEmap.GetValue(vertex).Append(edge);
            }
            else
            {
                var elist = new NCollection_List<Shape>();
                elist.Append(edge);
                vEmap[vertex] = elist;
            }
        }

        var bb = new BrepBuilder();
        for (int ii = 0; ii < vmap.Count; ii++)
        {
            Vertex                  origin  = ShapeCasts.Vertex(vmap[ii]);
            NCollection_List<Shape> branches = vEmap.GetValue(origin);
            var                            itl     = new NCollection_List<Shape>.Iterator(branches);
            while (itl.More())
            {
                Edge curEdge = ShapeCasts.Edge(itl.Value());
                branches.Remove(itl);

                var newWire = new Wire();
                bb.MakeWire(ref newWire);
                for (;;)
                {
                    bb.Add(newWire, curEdge);

                    Vertex vertex = Exploration.LastVertex(curEdge, true); // with orientation
                    if (vertex.IsSame(origin))
                    {
                        break;
                    }

                    if (!vEmap.ContainsKey(vertex))
                    {
                        break;
                    }

                    NCollection_List<Shape> elist = vEmap.GetValue(vertex);
                    if (elist.Extent() == 0)
                    {
                        break;
                    }

                    Curve2D? pCurve = Tool.CurveOnSurface(curEdge, face, out double fpar, out double lpar);
                    if (elist.Extent() == 1)
                    {
                        curEdge = ShapeCasts.Edge(elist.First());
                        elist.Clear();
                    }
                    else if (pCurve is null)
                    {
                        curEdge = ShapeCasts.Edge(elist.First());
                        elist.RemoveFirst();
                    }
                    else
                    {
                        double   param = (curEdge.Orientation == Forward) ? lpar : fpar;
                        Vector2D curDir = pCurve.EvalD1(param).D1;
                        curDir = curDir.Normalized();
                        if (curEdge.Orientation == Reversed)
                        {
                            curDir = curDir.Reversed();
                        }
                        // choose the rightest direction - the smallest angle
                        double minAngle   = RealLast();
                        var    nextEdge   = new Edge();
                        var    localIter = new NCollection_List<Shape>.Iterator(elist);
                        for (; localIter.More(); localIter.Next())
                        {
                            Edge edge = ShapeCasts.Edge(localIter.Value());
                            pCurve = Tool.CurveOnSurface(edge, face, out fpar, out lpar);
                            if (pCurve is null)
                            {
                                // edge has no pcurve on the reference face, skip it as a direction candidate
                                continue;
                            }
                            param = (edge.Orientation == Forward) ? fpar : lpar;
                            Vector2D dir = pCurve.EvalD1(param).D1;
                            dir = dir.Normalized();
                            if (edge.Orientation == Reversed)
                            {
                                dir = dir.Reversed();
                            }
                            double angle = curDir.AngleTo(dir);
                            if (angle < minAngle)
                            {
                                minAngle = angle;
                                nextEdge = edge;
                            }
                        }
                        if (nextEdge.IsNull)
                        {
                            // all candidates lacked a pcurve, fall back to an arbitrary one instead of stalling
                            nextEdge = ShapeCasts.Edge(elist.First());
                        }
                        curEdge = nextEdge;
                        // Remove <CurEdge> from list
                        for (localIter.Initialize(elist); localIter.More(); localIter.Next())
                        {
                            if (curEdge.IsSame(localIter.Value()))
                            {
                                elist.Remove(localIter);
                                break;
                            }
                        }
                    } // else (more than one edge)
                } // for (;;)
                wireSeq.Append(newWire);
            } // while (anItl.More())
        }
    }

    // Compares two sets of shapes. Returns true if they are the same, false otherwise.
    private static bool IsSameSets(HashSet<Shape>? fShells1, HashSet<Shape>? fShells2)
    {
        // If both are null - no problem
        if (fShells1 is null && fShells2 is null)
        {
            return true;
        }
        // If only one is null - not the same
        if (fShells1 is null || fShells2 is null)
        {
            return false;
        }
        // Both not null
        if (fShells1.Count != fShells2.Count)
        {
            return false;
        }
        // number of shells in each set should be very small in normal cases - max 2.
        // thus just check if all objects of one are contained in the other and vice versa.
        HashSet<Shape>.Enumerator it1 = fShells1.GetEnumerator();
        HashSet<Shape>.Enumerator it2 = fShells2.GetEnumerator();
        while (it1.MoveNext() && it2.MoveNext())
        {
            if (!fShells1.Contains(it2.Current) || !fShells2.Contains(it1.Current))
            {
                return false;
            }
        }
        return true;
    }

    private static void SetFixWireModes(FaceFixer sff)
    {
        WireFixer fixWire = sff.FixWireTool();
        fixWire.FixSelfIntersectionMode()             = 0;
        fixWire.FixNonAdjacentIntersectingEdgesMode() = 0;
        fixWire.FixLackingMode()                      = 0;
        fixWire.FixNotchedEdgesMode()                 = 0;
        fixWire.ModifyTopologyMode()                  = false;
        fixWire.ModifyRemoveLoopMode()                = 0;
        fixWire.FixGapsByRangesMode()                 = false;
        fixWire.FixSmallMode()                        = 0;
    }

    // Checks the sequence of edges on the presence of shared vertex
    private static void CheckSharedVertices(NCollection_Sequence<Shape>                                       seqEdges,
                                            IndexedMap<Shape, NCollection_List<Shape>> mapEdgesVertex,
                                            HashSet<Shape>                                            mapKeepShape,
                                            HashSet<Shape>                                            shareVertMap)
    {
        var sae         = new EdgeAnalysis();
        var seqVertexes = new NCollection_Sequence<Shape>();
        var mapVertexes = new HashSet<Shape>(ShapeHasher.Instance);
        for (int k = 1; k <= seqEdges.Length(); k++)
        {
            Vertex v1 = sae.FirstVertex(ShapeCasts.Edge(seqEdges[k]));
            Vertex v2 = sae.LastVertex(ShapeCasts.Edge(seqEdges[k]));
            if (!mapVertexes.Add(v1))
            {
                seqVertexes.Append(v1);
            }
            if (!mapVertexes.Add(v2))
            {
                seqVertexes.Append(v2);
            }
        }

        for (int k = 1; k <= seqVertexes.Length() /* && !IsSharedVertexPresent*/; k++)
        {
            NCollection_List<Shape> listEdgesV1 = mapEdgesVertex.GetValue(seqVertexes[k]);
            if (listEdgesV1.Extent() > 2 || mapKeepShape.Contains(seqVertexes[k]))
            {
                shareVertMap.Add(seqVertexes[k]);
            }
        }
        // return theShareVertMap.IsEmpty() ? false : true;
    }
}
