// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the unification of
// edges: MergeEdges, MergeSeq, MergeSubSeq, UnionPCurves and their file-static functions; the class is in
// ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Upgrade;

internal partial class UnifySameDomain
{
    /// <summary>A chain of edges that can be merged, and the edge they are merged into (null if not merged).</summary>
    protected sealed class SubSequenceOfEdges
    {
        /// <summary>The edges of the chain.</summary>
        public NCollection_Sequence<Shape> SeqsEdges = new();
        /// <summary>The edge they are merged into.</summary>
        public Edge UnionEdges = new();
    }

    // Kerf: frozen. A curve OCCT edits in place that may have been handed out frozen (a basis curve read through
    // BasisCurve(), which freezes it): the edit is made on a copy. OCCT edits curves it made itself at these sites.
    private static T Owned<T>(T curve) where T : Transient
        => curve.IsFrozen ? (T)(object)CopyOf(curve) : curve;

    private static object CopyOf(Transient curve) => curve switch
    {
        Geometry2D g2d => g2d.Copy(),
        Geometry3D g => g.Copy(),
        _ => throw new ProgramErrorException("ShapeUpgrade_UnifySameDomain: not a geometry"),
    };

    // Glues 3d curves of the sequence of edges
    private static Edge GlueEdgesWith3DCurves(NCollection_Sequence<Shape> aChain,
                                                     in Vertex                   firstVertex,
                                                     in Vertex                   lastVertex)
    {
        // number of curves
        int curveCount = aChain.Length();

        Edge   prevEdge     = ShapeCasts.Edge(aChain[1]);
        Vertex prevVertex   = firstVertex;
        double        maxTolerance = 0.0;
        // array of the curves converted to BSpline
        var bSplines = new NCollection_Array1<BSplineCurve3D>(0, curveCount - 1);
        // array of the tolerances at the vertices
        var verticesTolerances = new NCollection_Array1<double>(0, curveCount - 1);
        for (int i = 1; i <= curveCount; ++i)
        {
            Edge currentEdge = ShapeCasts.Edge(aChain[i]);
            Exploration.Vertices(currentEdge, out Vertex currentFirstVertex, out Vertex currentLastVertex);
            bool toReverse = !currentFirstVertex.IsSame(prevVertex);

            maxTolerance = Max(Max(maxTolerance, Tool.Tolerance(currentFirstVertex)),
                                Tool.Tolerance(currentLastVertex));
            if (i > 1)
            {
                var commonVertex = new Vertex();
                Exploration.CommonVertex(prevEdge, currentEdge, ref commonVertex);
                verticesTolerances[i - 2] = Tool.Tolerance(commonVertex);
            }

            Curve3D currentCurve = Tool.Curve(currentEdge, out double firstParam, out double lastParam)!;
            var        trimmedCurve = new TrimmedCurve3D(currentCurve, firstParam, lastParam);
            bSplines[i - 1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(trimmedCurve);
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref bSplines[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());
            if (toReverse)
            {
                bSplines[i - 1] = Owned(bSplines[i - 1]);
                bSplines[i - 1].Reverse();
            }

            prevVertex = toReverse ? currentFirstVertex : currentLastVertex;
            prevEdge   = currentEdge;
        }

        // Array of the concatenated curves.
        // Array of the remaining Vertex, effectively ignored here.
        // Flag of closedness of the concatenated curve, effectively ignored here.
        bool closedFlag = false;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(bSplines,
                             verticesTolerances,
                             out NCollection_HArray1<int> _,
                             out NCollection_HArray1<BSplineCurve3D> concatCurves,
                             ref closedFlag,
                             global::Kerf.Engine.Foundation.Precision.Confusion()); // C1 concatenation

        if (concatCurves.Length() > 1)
        {
            var concat = new CompCurveToBSplineCurve(concatCurves.Value(concatCurves.Lower()));

            for (int i = concatCurves.Lower() + 1; i <= concatCurves.Upper(); ++i)
            {
                concat.Add(concatCurves.Value(i), maxTolerance, true);
            }

            concatCurves.SetValue(concatCurves.Lower(), concat.InternalBSplineCurve()!);
        }

        BSplineCurve3D resCurve = concatCurves.Value(concatCurves.Lower());
        Edge resEdge = new LibraryMakeEdge(resCurve,
                                                    firstVertex,
                                                    lastVertex,
                                                    resCurve.FirstParameter,
                                                    resCurve.LastParameter).InternalEdge();
        var builder = new BrepBuilder();
        builder.SameRange(resEdge, false);
        builder.SameParameter(resEdge, false);
        BrepLibrary.SameParameter((Shape)resEdge, maxTolerance, true);
        return resEdge;
    }

    /// <summary>Unifies the pcurves of the chain into one pcurve of the edge.</summary>
    protected void UnionPCurves(NCollection_Sequence<Shape> chain, ref Edge edge)
    {
        Curve3D? curve   = Tool.Curve(edge, out double first3d, out double last3d);
        double      tolEdge = Tool.Tolerance(edge);
        double      maxTol  = tolEdge;

        var faceSeq = new NCollection_Sequence<Shape>();

        Edge                    firstEdge = ShapeCasts.Edge(chain.Value(1));
        NCollection_List<Shape> faceList  = _eFmap.GetValue(firstEdge);
        var                            itl      = new NCollection_List<Shape>.Iterator(faceList);
        for (; itl.More(); itl.Next())
        {
            Face face = ShapeCasts.Face(itl.Value());
            if (_facePlaneMap.ContainsKey(face))
            {
                continue;
            }

            if (_faceNewFace.ContainsKey(face))
            {
                face = ShapeCasts.Face(_faceNewFace.GetValue(face));
            }

            face.Orient(Forward); // to get proper pcurves of seam edges

            var bAsurf = new FaceSurfaceAdaptor(face, false);
            if (bAsurf.GetType() == Plane)
            {
                continue;
            }

            Surface? surf   = Tool.Surface(face, out Placement loc);
            bool          isFound = false;
            for (int ii = 1; ii <= faceSeq.Length(); ii++)
            {
                Surface? prevSurf = Tool.Surface(ShapeCasts.Face(faceSeq[ii]), out Placement prevLoc);
                if (ReferenceEquals(prevSurf, surf) && prevLoc.IsEqual(loc))
                {
                    isFound = true;
                    break;
                }
            }
            if (isFound)
            {
                continue;
            }

            faceSeq.Append(face);
        }

        var         resPCurves = new NCollection_Sequence<Curve2D>();
        var         resFirsts  = new NCollection_Sequence<double>();
        var         resLasts   = new NCollection_Sequence<double>();
        var         tolVerSeq = new NCollection_Sequence<double>();
        var         prevEdge  = new Edge();
        bool        isSeam   = false;

        for (int j = 1; j <= faceSeq.Length(); j++)
        {
            var               pCurveSeq   = new NCollection_Sequence<Curve2D>();
            var               firstsSeq   = new NCollection_Sequence<double>();
            var               lastsSeq    = new NCollection_Sequence<double>();
            var               forwardsSeq = new NCollection_Sequence<bool>();
            CurveKind currentType = OtherCurve;

            double first, last;
            for (int i = 1; i <= chain.Length(); i++)
            {
                Edge anEdge    = ShapeCasts.Edge(chain.Value(i));
                bool        isForward = (anEdge.Orientation != Reversed);

                if (isSeam && j == 2)
                { // second pass
                    anEdge.Reverse();
                }

                Curve2D? pCurve = Tool.CurveOnSurface(anEdge, ShapeCasts.Face(faceSeq[j]), out first, out last);
                if (pCurve is null)
                {
                    continue;
                }

                if (faceSeq.Length() == 1)
                {
                    Edge reversedEdge = anEdge;
                    reversedEdge.Reverse();
                    Curve2D? pCurve2 =
                      Tool.CurveOnSurface(reversedEdge, ShapeCasts.Face(faceSeq[j]), out first, out last);
                    if (!ReferenceEquals(pCurve, pCurve2))
                    {
                        isSeam = true;
                        faceSeq.Append(faceSeq[j]);
                    }
                }

                var               adaptor = new CurveAdaptor2D(pCurve);
                CurveKind type     = adaptor.GetType();

                LineCurve2D? line = null;
                if (type == BSplineCurve || type == BezierCurve)
                {
                    TryMakeLine(pCurve, first, last, ref line);
                }
                if (line is not null)
                {
                    pCurve = line;
                    adaptor.Load(pCurve);
                    type = Line;
                }

                if (pCurveSeq.IsEmpty())
                {
                    var copyPCurve = (Curve2D)pCurve.Copy();
                    if (copyPCurve is TrimmedCurve2D copyTrimmed)
                    {
                        copyPCurve = copyTrimmed.BasisCurve;
                    }

                    pCurveSeq.Append(copyPCurve);
                    firstsSeq.Append(first);
                    lastsSeq.Append(last);
                    forwardsSeq.Append(isForward);
                    currentType = type;
                    prevEdge    = anEdge;
                    continue;
                }

                bool   isSameCurve = false;
                double newF       = first;
                double newL       = last;
                if (ReferenceEquals(pCurve, pCurveSeq.Last()))
                {
                    isSameCurve = true;
                }
                else if (type == currentType)
                {
                    var prevAdaptor = new CurveAdaptor2D(pCurveSeq.Last());
                    switch (type)
                    {
                        case Line:
                        {
                            Line2D prevLin  = prevAdaptor.Line();
                            Point2D firstP2d = pCurve.Value(first);
                            Point2D lastP2d  = pCurve.Value(last);
                            if (prevLin.Contains(firstP2d, global::Kerf.Engine.Foundation.Precision.Confusion())
                                && prevLin.Contains(lastP2d, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                isSameCurve = true;
                                Point2D p1 = adaptor.Value(first);
                                Point2D p2 = adaptor.Value(last);
                                newF = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevLin, p1);
                                newL = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevLin, p2);
                                if (newF > newL)
                                {
                                    double tmp = newF;
                                    newF       = newL;
                                    newL       = tmp;
                                }
                            }
                            break;
                        }
                        case Circle:
                        {
                            Circle2D circ     = adaptor.Circle();
                            Circle2D prevCirc = prevAdaptor.Circle();
                            if (circ.Location.DistanceTo(prevCirc.Location) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && Math.Abs(circ.Radius - prevCirc.Radius) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                isSameCurve = true;
                                Point2D p1 = adaptor.Value(first);
                                Point2D p2 = adaptor.Value(last);
                                newF = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevCirc, p1);
                                newL = global::Kerf.Engine.Geometry.ElCLib.Parameter(prevCirc, p2);
                                if (newF > newL)
                                {
                                    double tmp = newF;
                                    newF       = newL;
                                    newL       = tmp;
                                }
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
                if (isSameCurve)
                {
                    if (forwardsSeq.Last())
                    {
                        lastsSeq.ChangeLast() = newL;
                    }
                    else
                    {
                        firstsSeq.ChangeLast() = newF;
                    }
                }
                else
                {
                    var copyPCurve = (Curve2D)pCurve.Copy();
                    if (copyPCurve is TrimmedCurve2D copyTrimmed)
                    {
                        copyPCurve = copyTrimmed.BasisCurve;
                    }

                    pCurveSeq.Append(copyPCurve);
                    firstsSeq.Append(first);
                    lastsSeq.Append(last);
                    forwardsSeq.Append(isForward);
                    currentType = type;
                    var v = new Vertex();
                    Exploration.CommonVertex(prevEdge, anEdge, ref v);
                    double tol = Tool.Tolerance(v);
                    tolVerSeq.Append(tol);
                }
                prevEdge = anEdge;
            }

            Curve2D resPCurve;
            double       resFirst, resLast;
            if (pCurveSeq.Length() == 1)
            {
                resPCurve = pCurveSeq.Last();
                resFirst  = firstsSeq.Last();
                resLast   = lastsSeq.Last();
                if (!forwardsSeq.Last())
                {
                    double newLast  = resPCurve.ReversedParameter(resFirst);
                    double newFirst = resPCurve.ReversedParameter(resLast);
                    // the basis of a copied trimmed curve, frozen by BasisCurve()
                    resPCurve = Owned(resPCurve);
                    resPCurve.Reverse();
                    resFirst = newFirst;
                    resLast  = newLast;
                }
            }
            else
            {
                // C1 concatenation for PCurveSeq
                var tab_c2d = new NCollection_Array1<BSplineCurve2D>(0, pCurveSeq.Length() - 1);
                for (int i = 1; i <= pCurveSeq.Length(); i++)
                {
                    var trPCurve = new TrimmedCurve2D(pCurveSeq[i], firstsSeq[i], lastsSeq[i]);
                    if (!forwardsSeq[i])
                    {
                        trPCurve.Reverse();
                    }
                    tab_c2d[i - 1] = Geom2dConvert2D.CurveToBSplineCurve(trPCurve);
                    Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref tab_c2d[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                var tabtolvertex = new NCollection_Array1<double>(0, tolVerSeq.Length() - 1);
                for (int i = 1; i <= tolVerSeq.Length(); i++)
                {
                    double tol = tolVerSeq[i];
                    tabtolvertex[i - 1] = tol;
                    if (tol > maxTol)
                    {
                        maxTol = tol;
                    }
                }

                bool closedFlag = false;
                Geom2dConvert2D.ConcatC1(tab_c2d,
                                       tabtolvertex,
                                       out NCollection_HArray1<int> _,                     // array of the remaining Vertex
                                       out NCollection_HArray1<BSplineCurve2D> concatc2d, // array of the concatenated curves
                                       ref closedFlag,
                                       global::Kerf.Engine.Foundation.Precision.Confusion()); // C1 concatenation

                // Two arcs making a loop come back half a turn out, as OCCT's Add prepends where both ends join. If that
                // leaves the pcurve starting away from the 3D curve, as in a box cut by a sphere, keep their order instead
                // (#1541; decision 1831). OCCT leaves it, and the face is lost.
                if (concatc2d.Length() == 1 && curve is not null
                    && Tool.Surface(ShapeCasts.Face(faceSeq[resPCurves.Length() + 1])) is { } faceSurface)
                {
                    Point2D loopStart = concatc2d.Value(concatc2d.Lower()).StartPoint;
                    if (faceSurface.Value(loopStart.X, loopStart.Y).DistanceTo(curve.Value(first3d))
                        > 10.0 * Max(maxTol, global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        closedFlag = false;
                        Geom2dConvert2D.ConcatC1(tab_c2d,
                                               tabtolvertex,
                                               out NCollection_HArray1<int> _,
                                               out concatc2d,
                                               ref closedFlag,
                                               global::Kerf.Engine.Foundation.Precision.Confusion(),
                                               keepLoopOrder: true);
                    }
                }

                if (concatc2d.Length() > 1)
                {
                    var concat2d = new CompCurveToBSplineCurve2D(concatc2d.Value(concatc2d.Lower()));

                    for (int i = concatc2d.Lower() + 1; i <= concatc2d.Upper(); i++)
                    {
                        concat2d.Add(concatc2d.Value(i), maxTol, true);
                    }

                    concatc2d.SetValue(concatc2d.Lower(), concat2d.InternalBSplineCurve()!);
                }
                BSplineCurve2D bSplineCurve = concatc2d.Value(concatc2d.Lower());
                resPCurve = bSplineCurve;
                resFirst  = bSplineCurve.FirstParameter;
                resLast   = bSplineCurve.LastParameter;
            }
            resPCurves.Append(resPCurve);
            resFirsts.Append(resFirst);
            resLasts.Append(resLast);
        }

        var builder = new BrepBuilder();

        // Check the results for consistency
        bool   isBadRange = false;
        double range3d   = last3d - first3d;
        for (int ii = 1; ii <= resPCurves.Length(); ii++)
        {
            double range = resLasts[ii] - resFirsts[ii];
            if (Math.Abs(range3d - range) > maxTol)
            {
                isBadRange = true;
            }
        }

        if (isBadRange)
        {
            for (int ii = 1; ii <= resPCurves.Length(); ii++)
            {
                Face face     = ShapeCasts.Face(faceSeq[ii]);
                Surface? surf   = Tool.Surface(face);
                var         sas      = new SurfaceAnalysis(surf);
                var         toolProj = new ProjectCurveOnSurface();
                toolProj.Init(sas, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (toolProj.InternalPerform(curve, first3d, last3d, out Curve2D? newPCurve))
                {
                    resPCurves[ii] = newPCurve!;
                }
                else
                {
                    // Reparametrize pcurve
                    var trPCurve = new TrimmedCurve2D(resPCurves[ii], resFirsts[ii], resLasts[ii]);
                    BSplineCurve2D bSplinePCurve = Owned(Geom2dConvert2D.CurveToBSplineCurve(trPCurve));
                    var knots = new NCollection_Array1<double>(bSplinePCurve.InternalKnots());
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(first3d, last3d, knots);
                    bSplinePCurve.SetKnots(knots);
                    resPCurves[ii] = bSplinePCurve;
                }
                resFirsts[ii] = first3d;
                resLasts[ii]  = last3d;
            }
        }

        // Reparametrize pcurves if needed
        if (!resPCurves.IsEmpty())
        {
            for (int ii = 1; ii <= resPCurves.Length(); ii++)
            {
                if (Math.Abs(first3d - resFirsts[ii]) > maxTol || Math.Abs(last3d - resLasts[ii]) > maxTol)
                {
                    var               gAcurve = new CurveAdaptor2D(resPCurves[ii]);
                    CurveKind type    = gAcurve.GetType();
                    if (type == Line)
                    {
                        Line2D lin2d   = gAcurve.Line();
                        Direction2D dir2d   = lin2d.Direction;
                        Point2D pnt2d   = gAcurve.Value(resFirsts[ii]);
                        Vector2D offset = -new Vector2D(dir2d);
                        offset *= first3d;
                        pnt2d = pnt2d.Translated(offset);
                        var newLine2d = new LineCurve2D(pnt2d, dir2d);
                        resPCurves[ii] = newLine2d;
                    }
                    else if (type == Circle)
                    {
                        Circle2D circ2d   = gAcurve.Circle();
                        double    radius   = circ2d.Radius;
                        Frame2D  position = circ2d.Position;
                        Point2D  location = circ2d.Location;
                        double    offset  = resFirsts[ii] - first3d;
                        position = position.Rotated(location, offset);
                        var newCircle2d = new CircleCurve2D(position, radius);
                        resPCurves[ii] = newCircle2d;
                    }
                    else // general case
                    {
                        var trPCurve = new TrimmedCurve2D(resPCurves[ii], resFirsts[ii], resLasts[ii]);
                        BSplineCurve2D bSplinePCurve = Owned(Geom2dConvert2D.CurveToBSplineCurve(trPCurve));
                        var knots = new NCollection_Array1<double>(bSplinePCurve.InternalKnots());
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(first3d, last3d, knots);
                        bSplinePCurve.SetKnots(knots);
                        resPCurves[ii] = bSplinePCurve;
                    }
                    resFirsts[ii] = first3d;
                    resLasts[ii]  = last3d;
                } // if ranges > aMaxTol
            } // for (int ii = 1; ii <= ResPCurves.Length(); ii++)
        }

        if (isSeam)
        {
            builder.UpdateEdge(edge, resPCurves[1], resPCurves[2], ShapeCasts.Face(faceSeq[1]), tolEdge);
        }
        else
        {
            for (int j = 1; j <= resPCurves.Length(); j++)
            {
                builder.UpdateEdge(edge, resPCurves[j], ShapeCasts.Face(faceSeq[j]), tolEdge);
            }
        }
    }

    /// <summary>Merges a sequence of edges into one edge if possible.</summary>
    protected bool MergeSubSeq(NCollection_Sequence<Shape>                                       chain,
                               IndexedMap<Shape, NCollection_List<Shape>> vFmap,
                               ref Edge                                                          outEdge)
    {
        var sae = new EdgeAnalysis();
        var b   = new BrepBuilder();
        // union edges in chain
        int         j;
        double      fp1, lp1;
        bool        isUnionOfLinesPossible   = true;
        bool        isUnionOfCirclesPossible = true;
        Curve3D? c3d1, c3d2;
        for (j = 1; j < chain.Length(); j++)
        {
            Edge edge1 = ShapeCasts.Edge(chain.Value(j));
            Edge edge2 = ShapeCasts.Edge(chain.Value(j + 1));

            if (Tool.Degenerated(edge1) && Tool.Degenerated(edge2))
            {
                // Find the closest points in 2d
                Edge        edgeFirst    = ShapeCasts.Edge(chain.First());
                Edge        edgeLast     = ShapeCasts.Edge(chain.Last());
                var                commonFace   = new Face();
                double             minSqDist    = 0.0;
                Orientation orOfE1OnFace = Forward, orOfE2OnFace = Forward;
                int                indOnE1      = 0, indOnE2 = 0;
                Span<Point2D>     pointsOnEdge1 = [new Point2D(), new Point2D()];
                Span<Point2D>     pointsOnEdge2 = [new Point2D(), new Point2D()];
                if (!FindClosestPoints(edgeFirst,
                                       edgeLast,
                                       vFmap,
                                       ref commonFace,
                                       ref minSqDist,
                                       ref orOfE1OnFace,
                                       ref orOfE2OnFace,
                                       ref indOnE1,
                                       ref indOnE2,
                                       pointsOnEdge1,
                                       pointsOnEdge2))
                {
                    return false;
                }

                // Two pole edges with a seam between them don't touch on the surface: merging them, as OCCT 8.0.1 does,
                // gives one that crosses the face the wrong way (decision 1816)
                double aTouching = global::Kerf.Engine.Foundation.Precision.Confusion();
                if (minSqDist > aTouching * aTouching)
                {
                    return false;
                }

                // Define indices corresponding to extremities of future edge
                indOnE1 = 1 - indOnE1;
                indOnE2 = 1 - indOnE2;

                // Construct new degenerated edge
                Point2D startPoint = pointsOnEdge1[indOnE1];
                Point2D endPoint   = pointsOnEdge2[indOnE2];
                if ((orOfE1OnFace == Forward && indOnE1 == 1)
                    || (orOfE1OnFace == Reversed && indOnE1 == 0))
                {
                    Point2D tmp = startPoint;
                    startPoint = endPoint;
                    endPoint   = tmp;
                }

                LineCurve2D line = new MakeLine2d(startPoint, endPoint).Value();

                Vertex vertex     = Exploration.FirstVertex(edgeFirst);
                Vertex startVertex = vertex, endVertex = vertex;
                startVertex.Orient(Forward);
                endVertex.Orient(Reversed);

                var newEdge = new Edge();
                b.MakeEdge(ref newEdge);
                b.UpdateEdge(newEdge, line, commonFace, global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Range(newEdge, 0.0, startPoint.DistanceTo(endPoint));
                b.Add(newEdge, startVertex);
                b.Add(newEdge, endVertex);
                b.Degenerated(newEdge, true);
                // The new edge runs as the first did on the face taken forward. The chain holds that edge as the shape
                // does, so on a reversed face OCCT 8.0.1's goes the other way round the wire (decision 1816).
                if (edgeFirst.Orientation != orOfE1OnFace)
                {
                    newEdge.Reverse();
                }

                outEdge = newEdge;
                return true;
            }

            c3d1 = Tool.Curve(edge1, out fp1, out lp1);
            c3d2 = Tool.Curve(edge2, out _, out _);

            var    bAcurve1 = new EdgeCurveAdaptor(edge1);
            var    bAcurve2 = new EdgeCurveAdaptor(edge2);
            Direction3D aDir1 = new(), aDir2 = new();

            if (c3d1 is null || c3d2 is null)
            {
                return false;
            }

            if (c3d1 is TrimmedCurve3D tc1)
            {
                c3d1 = tc1.BasisCurve;
            }
            if (c3d2 is TrimmedCurve3D tc2)
            {
                c3d2 = tc2.BasisCurve;
            }
            if (IsLinear(bAcurve1, ref aDir1) && IsLinear(bAcurve2, ref aDir2))
            {
                if (!aDir1.IsParallel(aDir2, _angTol))
                {
                    isUnionOfLinesPossible = false;
                }
            }
            else
            {
                isUnionOfLinesPossible = false;
            }
            if (c3d1 is CircleCurve3D c1 && c3d2 is CircleCurve3D c2)
            {
                Point3D p01 = c1.Location;
                Point3D p02 = c2.Location;
                if (p01.DistanceTo(p02) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isUnionOfCirclesPossible = false;
                }
            }
            else
            {
                isUnionOfCirclesPossible = false;
            }
        }
        if (isUnionOfLinesPossible && isUnionOfCirclesPossible)
        {
            return false;
        }

        // union of lines is possible
        if (isUnionOfLinesPossible)
        {
            var v = new Vertex[2];
            v[0] = sae.FirstVertex(ShapeCasts.Edge(chain.First()));
            Point3D pv1 = Tool.Pnt(v[0]);
            v[1] = sae.LastVertex(ShapeCasts.Edge(chain.Last()));
            Point3D pv2 = Tool.Pnt(v[1]);
            var    vec = new Vector3D(pv1, pv2);
            if (_safeInputMode)
            {
                for (int k = 0; k < 2; k++)
                {
                    if (!_context.IsRecorded(v[k]))
                    {
                        Vertex vcopy = ShapeCasts.Vertex(v[k].EmptyCopied());
                        _context.Replace(v[k], vcopy);
                        v[k] = vcopy;
                    }
                    else
                    {
                        v[k] = ShapeCasts.Vertex(_context.InternalApply(v[k]));
                    }
                }
            }
            var    l    = new LineCurve3D(new Axis3D(pv1, new Direction3D(vec)));
            double dist = pv1.DistanceTo(pv2);
            var    tc   = new TrimmedCurve3D(l, 0.0, dist);
            var    e    = new Edge();
            b.MakeEdge(ref e, tc, global::Kerf.Engine.Foundation.Precision.Confusion());
            b.Add(e, v[0]);
            b.Add(e, v[1]);
            b.UpdateVertex(v[0], 0.0, e, 0.0);
            b.UpdateVertex(v[1], dist, e, 0.0);
            UnionPCurves(chain, ref e);
            outEdge = e;
            return true;
        }

        if (isUnionOfCirclesPossible)
        {
            Edge fe  = ShapeCasts.Edge(chain.First());
            Curve3D  c3d = Tool.Curve(fe, out _, out _)!;

            if (c3d is TrimmedCurve3D tc)
            {
                c3d = tc.BasisCurve;
            }
            var cir = (CircleCurve3D)c3d;

            var v = new Vertex[2];
            v[0] = sae.FirstVertex(fe);
            v[1] = sae.LastVertex(ShapeCasts.Edge(chain.Last()));
            bool isClosed = v[0].IsSame(v[1]);
            if (!isClosed)
            {
                // additionally check the points for equality to make a final decision about closedness of the
                // result curve
                Point3D p0  = Tool.Pnt(v[0]);
                Point3D p1  = Tool.Pnt(v[1]);
                double tol = Max(Tool.Tolerance(v[0]), Tool.Tolerance(v[1]));
                if (p0.SquareDistanceTo(p1) < tol * tol)
                {
                    isClosed = true;
                    v[1]     = v[0];
                    v[1].Reverse();
                }
            }
            var e = new Edge();
            if (isClosed)
            {
                // closed chain
                var          adef = new EdgeCurveAdaptor(fe);
                CircleCurve3D  cir1;
                double       fp, lp;
                if (fe.Orientation == Forward)
                {
                    fp = adef.FirstParameter();
                    lp = adef.LastParameter();
                }
                else
                {
                    fp = adef.LastParameter();
                    lp = adef.FirstParameter();
                }
                if (Math.Abs(fp) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    b.MakeEdge(ref e, cir, global::Kerf.Engine.Foundation.Precision.Confusion());
                    b.Add(e, v[0]);
                    b.Add(e, v[1]);
                    e.Orient(fe.Orientation);
                }
                else
                {
                    var mc1 = new MakeCircle(adef.Value(fp), adef.Value((fp + lp) * 0.5), adef.Value(lp));
                    if (mc1.IsDone)
                    {
                        cir1 = mc1.Value();
                    }
                    else
                    {
                        return false;
                    }
                    b.MakeEdge(ref e, cir1, global::Kerf.Engine.Foundation.Precision.Confusion());
                    b.Add(e, v[0]);
                    b.Add(e, v[1]);
                }
            }
            else // open chain
            {
                double        paramFirst     = Tool.Parameter(v[0], fe);
                Vertex vertexLastOnFE = sae.LastVertex(fe);
                double        paramLast      = Tool.Parameter(vertexLastOnFE, fe);

                if (_safeInputMode)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        if (!_context.IsRecorded(v[k]))
                        {
                            Vertex vcopy = ShapeCasts.Vertex(v[k].EmptyCopied());
                            _context.Replace(v[k], vcopy);
                            v[k] = vcopy;
                        }
                        else
                        {
                            v[k] = ShapeCasts.Vertex(_context.InternalApply(v[k]));
                        }
                    }
                }

                Point3D pointFirst = Tool.Pnt(v[0]);
                while (Math.Abs(paramLast - paramFirst) > 7 * M_PI / 8)
                {
                    paramLast = (paramFirst + paramLast) / 2;
                }
                var    bAcurveFE        = new EdgeCurveAdaptor(fe);
                Point3D pointLast        = bAcurveFE.Value(paramLast);
                Point3D origin           = cir.Circle.Location;
                var    dir1             = new Direction3D(new Vector3D(origin, pointFirst));
                var    dir2             = new Direction3D(new Vector3D(origin, pointLast));
                Direction3D vdir             = dir1 ^ dir2;
                var    ax2            = new Frame3D(origin, vdir, dir1);
                var    newCircle       = new CircleCurve3D(ax2, cir.Radius);
                Point3D pointLastInChain = Tool.Pnt(v[1]);
                var    dirLastInChain   = new Direction3D(new Vector3D(origin, pointLastInChain));
                double lpar             = dir1.AngleWithRef(dirLastInChain, vdir);
                if (lpar < 0.0)
                {
                    lpar += 2 * M_PI;
                }

                var tc1 = new TrimmedCurve3D(newCircle, 0.0, lpar);
                b.MakeEdge(ref e, tc1, global::Kerf.Engine.Foundation.Precision.Confusion());
                b.Add(e, v[0]);
                b.Add(e, v[1]);
                b.UpdateVertex(v[0], 0.0, e, 0.0);
                b.UpdateVertex(v[1], lpar, e, 0.0);
            }
            UnionPCurves(chain, ref e);
            outEdge = e;
            return true;
        }
        if (chain.Length() > 1 && _concatBSplines)
        {
            // second step: union edges with various curves
            // skl for bug 0020052 from Mantis: perform such unions
            // only if curves are bspline or bezier

            Vertex vf        = sae.FirstVertex(ShapeCasts.Edge(chain.First()));
            Vertex vl        = sae.LastVertex(ShapeCasts.Edge(chain.Last()));
            bool          needUnion = true;
            for (j = 1; j <= chain.Length(); j++)
            {
                Edge edge = ShapeCasts.Edge(chain.Value(j));
                Curve3D? c3d  = Tool.Curve(edge, out _, out fp1, out lp1);
                if (c3d is null)
                {
                    continue;
                }
                if (c3d is TrimmedCurve3D tc)
                {
                    c3d = tc.BasisCurve;
                }
                if (c3d is BSplineCurve3D || c3d is BezierCurve3D)
                {
                    continue;
                }
                needUnion = false;
                break;
            }
            if (needUnion)
            {
                outEdge = GlueEdgesWith3DCurves(chain, vf, vl);
                return true;
            }
        }
        return false;
    }

    // Checks if merging of two edges is possible
    private static bool IsMergingPossible(in Edge                                                         edge1,
                                          in Edge                                                         edge2,
                                          double                                                                 angTol,
                                          double                                                                 linTol,
                                          HashSet<Shape>                                          avoidEdgeVrt,
                                          bool                                                                   lineDirectionOk,
                                          in Point3D                                                              firstPoint,
                                          in Vector3D                                                              directionVec,
                                          IndexedMap<Shape, NCollection_List<Shape>> vFmap)
    {
        bool isDegE1 = Tool.Degenerated(edge1);
        bool isDegE2 = Tool.Degenerated(edge2);

        if (isDegE1 && isDegE2)
        {
            // Find connstion point in 2d
            var                commonFace    = new Face();
            double             minSqDist     = 0.0;
            Orientation orOfE1OnFace  = Forward, orOfE2OnFace = Forward;
            int                indOnE1       = 0, indOnE2 = 0;
            Span<Point2D>     pointsOnEdge1 = [new Point2D(), new Point2D()];
            Span<Point2D>     pointsOnEdge2 = [new Point2D(), new Point2D()];
            if (!FindClosestPoints(edge1,
                                   edge2,
                                   vFmap,
                                   ref commonFace,
                                   ref minSqDist,
                                   ref orOfE1OnFace,
                                   ref orOfE2OnFace,
                                   ref indOnE1,
                                   ref indOnE2,
                                   pointsOnEdge1,
                                   pointsOnEdge2))
            {
                return false;
            }

            if (minSqDist <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                return true;
            }

            return false;
        }
        else if (isDegE1 || isDegE2)
        {
            return false;
        }

        Vertex cv = Exploration.LastVertex(edge1, true);
        if (cv.IsNull || avoidEdgeVrt.Contains(cv))
        {
            return false;
        }

        var ade1 = new EdgeCurveAdaptor(edge1);
        var ade2 = new EdgeCurveAdaptor(edge2);

        CurveKind t1 = ade1.GetType();
        CurveKind t2 = ade2.GetType();

        if (t1 == Circle && t2 == Circle)
        {
            if (ade1.Circle().Location.DistanceTo(ade2.Circle().Location) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }

        Direction3D dir1 = new(), dir2 = new();
        if (!(IsLinear(ade1, ref dir1) && IsLinear(ade2, ref dir2))
            && ((t1 != BezierCurve && t1 != BSplineCurve)
                || (t2 != BezierCurve && t2 != BSplineCurve))
            && t1 != t2)
        {
            return false;
        }

        Vector3D diff1, diff2;
        Point3D p1, p2;
        if (edge1.Orientation == Forward)
        {
            ade1.D1(ade1.LastParameter(), out p1, out diff1);
        }
        else
        {
            ade1.D1(ade1.FirstParameter(), out p1, out diff1);
            diff1 = -diff1;
        }

        if (edge2.Orientation == Forward)
        {
            ade2.D1(ade2.FirstParameter(), out p2, out diff2);
        }
        else
        {
            ade2.D1(ade2.LastParameter(), out p2, out diff2);
            diff2 = -diff2;
        }

        // Not OCCT (decision 1652): a tangent of zero length, as a B-spline's with its end poles repeated, has no angle to
        // compare. OCCT's Angle raises here and ends the whole unification; the edges are simply left as they are
        if (diff1.Magnitude <= Constants.Resolution() || diff2.Magnitude <= Constants.Resolution())
        {
            return false;
        }

        if (diff1.AngleTo(diff2) > angTol)
        {
            return false;
        }

        if (lineDirectionOk && t2 == Line)
        {
            // Check that the accumulated deflection does not exceed the linear tolerance
            double last = (edge2.Orientation == Forward) ? ade2.LastParameter() : ade2.FirstParameter();
            var    curV = new Vector3D(firstPoint, ade2.Value(last));
            double dd   = directionVec.CrossSquareMagnitude(curV);
            if (dd > linTol * linTol)
            {
                return false;
            }

            // Check that the accumulated angle does not exceed the angular tolerance.
            // For symmetry, check the angle between vectors of:
            // - first edge and resulting curve, and
            // - the last edge and resulting curve.
            if (directionVec.AngleTo(curV) > angTol || diff2.AngleTo(curV) > angTol)
            {
                return false;
            }
        }

        return true;
    }

    private static bool GetLineEdgePoints(in Edge inpEdge, ref Point3D firstPoint, ref Vector3D directionVec)
    {
        Curve3D? cur = Tool.Curve(inpEdge, out double f, out double l);
        if (cur is null)
        {
            return false;
        }

        if (cur is TrimmedCurve3D tc)
        {
            cur = tc.BasisCurve;
        }

        if (cur.GetType() != typeof(LineCurve3D))
        {
            return false;
        }

        if (inpEdge.Orientation == Reversed)
        {
            double tmp = f;
            f = l;
            l = tmp;
        }
        firstPoint = cur.Value(f);
        Point3D lp = cur.Value(l);
        directionVec = new Vector3D(lp.XYZ().Subtracted(firstPoint.XYZ()));
        directionVec = directionVec.Normalized();
        return true;
    }

    // Generates sub-sequences of edges from sequence of edges. Edges from each subsequences can be merged into the
    // one edge.
    private static void GenerateSubSeq(NCollection_Sequence<Shape>                                       anInpEdgeSeq,
                                       NCollection_Sequence<SubSequenceOfEdges>                                 seqOfSubSeqOfEdges,
                                       bool                                                                     isClosed,
                                       double                                                                   angTol,
                                       double                                                                   linTol,
                                       HashSet<Shape>                                            avoidEdgeVrt,
                                       IndexedMap<Shape, NCollection_List<Shape>> vFmap)
    {
        bool        isOk;
        Edge edge1, edge2;

        var         SubSeq  = new SubSequenceOfEdges();
        Edge refEdge = ShapeCasts.Edge(anInpEdgeSeq[1]);
        SubSeq.SeqsEdges.Append(refEdge);
        seqOfSubSeqOfEdges.Append(SubSeq);

        var  firstPoint       = new Point3D();
        var  directionVec     = new Vector3D();
        bool isLineDirectionOk = GetLineEdgePoints(refEdge, ref firstPoint, ref directionVec);

        for (int i = 1; i < anInpEdgeSeq.Length(); i++)
        {
            edge1 = ShapeCasts.Edge(anInpEdgeSeq[i]);
            edge2 = ShapeCasts.Edge(anInpEdgeSeq[i + 1]);
            isOk  = IsMergingPossible(edge1,
                                     edge2,
                                     angTol,
                                     linTol,
                                     avoidEdgeVrt,
                                     isLineDirectionOk,
                                     firstPoint,
                                     directionVec,
                                     vFmap);
            if (!isOk)
            {
                var subSeq = new SubSequenceOfEdges();
                subSeq.SeqsEdges.Append(edge2);
                seqOfSubSeqOfEdges.Append(subSeq);
                isLineDirectionOk = GetLineEdgePoints(edge2, ref firstPoint, ref directionVec);
            }
            else
            {
                seqOfSubSeqOfEdges.ChangeLast().SeqsEdges.Append(edge2);
            }
        }
        // check first and last chain segments
        if (isClosed && seqOfSubSeqOfEdges.Length() > 1)
        {
            edge1 = ShapeCasts.Edge(anInpEdgeSeq.Last());
            edge2 = ShapeCasts.Edge(anInpEdgeSeq.First());
            if (IsMergingPossible(edge1,
                                  edge2,
                                  angTol,
                                  linTol,
                                  avoidEdgeVrt,
                                  false,
                                  firstPoint,
                                  directionVec,
                                  vFmap))
            {
                seqOfSubSeqOfEdges.ChangeLast().SeqsEdges.Append(seqOfSubSeqOfEdges.ChangeFirst().SeqsEdges);
                seqOfSubSeqOfEdges.Remove(1);
            }
        }
    }

    /// <summary>Splits the sequence of edges into the sequence of chains.</summary>
    protected bool MergeEdges(NCollection_Sequence<Shape>                                       seqEdges,
                              IndexedMap<Shape, NCollection_List<Shape>> vFmap,
                              NCollection_Sequence<SubSequenceOfEdges>                                 seqOfSubSeqOfEdges,
                              HashSet<Shape>                                            nonMergVrt)
    {
        var mapVE          = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int j;
        var verticesToAvoid = new HashSet<Shape>(ShapeHasher.Instance);
        int nbE            = seqEdges.Length();
        for (j = 1; j <= nbE; j++)
        {
            Edge anEdge = ShapeCasts.Edge(seqEdges[j]);
            // fill in the map V-E
            for (var it = new Iterator(anEdge.Oriented(Forward)); it.More(); it.Next())
            {
                Shape aV = it.Value();
                if (aV.Orientation == Forward || aV.Orientation == Reversed)
                {
                    if (!mapVE.ContainsKey(aV))
                    {
                        mapVE.Add(aV, new NCollection_List<Shape>());
                    }
                    mapVE.GetValue(aV).Append(anEdge);
                }
            }
        }
        NCollection_MapAlgo.Unite(verticesToAvoid, nonMergVrt);

        // do loop while there are unused edges
        var usedEdges = new HashSet<Shape>(ShapeHasher.Instance);

        for (int iE = 1; iE <= nbE; ++iE)
        {
            Edge edge = ShapeCasts.Edge(seqEdges[iE]);
            if (!usedEdges.Add(edge))
            {
                continue;
            }

            // make chain for unite
            var chain = new NCollection_Sequence<Shape>();
            chain.Append(edge);
            var v = new Vertex[2];
            Exploration.Vertices(edge, out v[0], out v[1], true);

            // connect more edges to the chain in both directions
            for (j = 0; j < 2; j++)
            {
                bool isAdded = true;
                while (isAdded)
                {
                    isAdded = false;
                    if (v[j].IsNull)
                    {
                        break;
                    }
                    NCollection_List<Shape> le = mapVE.GetValue(v[j]);
                    for (var itL = new NCollection_List<Shape>.Iterator(le); itL.More(); itL.Next())
                    {
                        edge = ShapeCasts.Edge(itL.Value());
                        if (!usedEdges.Contains(edge))
                        {
                            var v2 = new Vertex[2];
                            Exploration.Vertices(edge, out v2[0], out v2[1], true);
                            // the neighboring edge must have V[j] reversed and located on the opposite end
                            if (v2[1 - j].IsEqual(v[j].Reversed()))
                            {
                                if (j == 0)
                                {
                                    chain.Prepend(edge);
                                }
                                else
                                {
                                    chain.Append(edge);
                                }
                                usedEdges.Add(edge);
                                v[j]    = v2[j];
                                isAdded = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (chain.Length() < 2)
            {
                continue;
            }

            bool isClosed = false;
            if (v[0].IsSame(v[1]))
            {
                isClosed = true;
            }

            // split chain by vertices at which merging is not possible
            var oneSeq = new NCollection_Sequence<SubSequenceOfEdges>();
            GenerateSubSeq(chain, oneSeq, isClosed, _angTol, _linTol, verticesToAvoid, vFmap);

            // put sub-chains in the result
            seqOfSubSeqOfEdges.Append(oneSeq);
        }

        for (int i = 1; i <= seqOfSubSeqOfEdges.Length(); i++)
        {
            var ue = new Edge();
            if (seqOfSubSeqOfEdges[i].SeqsEdges.Length() < 2)
            {
                continue;
            }
            if (MergeSubSeq(seqOfSubSeqOfEdges[i].SeqsEdges, vFmap, ref ue))
            {
                seqOfSubSeqOfEdges[i].UnionEdges = ue;
            }
        }
        return true;
    }

    /// <summary>Tries to unify the sequence of edges with the set of another edges which lies on the same geometry.</summary>
    protected bool MergeSeq(NCollection_Sequence<Shape>                                       seqEdges,
                            IndexedMap<Shape, NCollection_List<Shape>> vFmap,
                            HashSet<Shape>                                            nonMergVert)
    {
        var seqOfSubsSeqOfEdges = new NCollection_Sequence<SubSequenceOfEdges>();
        if (MergeEdges(seqEdges, vFmap, seqOfSubsSeqOfEdges, nonMergVert))
        {
            for (int i = 1; i <= seqOfSubsSeqOfEdges.Length(); i++)
            {
                if (seqOfSubsSeqOfEdges[i].UnionEdges.IsNull)
                {
                    continue;
                }

                _context.Merge(seqOfSubsSeqOfEdges[i].SeqsEdges, seqOfSubsSeqOfEdges[i].UnionEdges);
            }
            return true;
        }
        return false;
    }
}
