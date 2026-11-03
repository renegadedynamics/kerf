// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the unification of
// edges: MergeEdges, MergeSeq, MergeSubSeq, UnionPCurves and their file-static functions; the class is in
// ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Occt;

public partial class ShapeUpgrade_UnifySameDomain
{
    /// <summary>A chain of edges that can be merged, and the edge they are merged into (null if not merged).</summary>
    protected sealed class SubSequenceOfEdges
    {
        /// <summary>The edges of the chain.</summary>
        public NCollection_Sequence<Shape> SeqsEdges = new();
        /// <summary>The edge they are merged into.</summary>
        public Edge UnionEdges = new();
    }

    // OCCT.NET: frozen. A curve OCCT edits in place that may have been handed out frozen (a basis curve read through
    // BasisCurve(), which freezes it): the edit is made on a copy. OCCT edits curves it made itself at these sites.
    private static T owned<T>(T theCurve) where T : Transient
        => theCurve.IsFrozen ? (T)(object)copyOf(theCurve) : theCurve;

    private static object copyOf(Transient theCurve) => theCurve switch
    {
        Geometry2D aG2d => aG2d.Copy(),
        Geometry3D aG => aG.Copy(),
        _ => throw new ProgramErrorException("ShapeUpgrade_UnifySameDomain: not a geometry"),
    };

    // Glues 3d curves of the sequence of edges
    private static Edge GlueEdgesWith3DCurves(NCollection_Sequence<Shape> aChain,
                                                     in Vertex                   FirstVertex,
                                                     in Vertex                   LastVertex)
    {
        // number of curves
        int aCurveCount = aChain.Length();

        Edge   aPrevEdge     = ShapeCasts.Edge(aChain[1]);
        Vertex aPrevVertex   = FirstVertex;
        double        aMaxTolerance = 0.0;
        // array of the curves converted to BSpline
        var aBSplines = new NCollection_Array1<BSplineCurve3D>(0, aCurveCount - 1);
        // array of the tolerances at the vertices
        var aVerticesTolerances = new NCollection_Array1<double>(0, aCurveCount - 1);
        for (int i = 1; i <= aCurveCount; ++i)
        {
            Edge aCurrentEdge = ShapeCasts.Edge(aChain[i]);
            Exploration.Vertices(aCurrentEdge, out Vertex aCurrentFirstVertex, out Vertex aCurrentLastVertex);
            bool aToReverse = !aCurrentFirstVertex.IsSame(aPrevVertex);

            aMaxTolerance = Max(Max(aMaxTolerance, Tool.Tolerance(aCurrentFirstVertex)),
                                Tool.Tolerance(aCurrentLastVertex));
            if (i > 1)
            {
                var aCommonVertex = new Vertex();
                Exploration.CommonVertex(aPrevEdge, aCurrentEdge, ref aCommonVertex);
                aVerticesTolerances[i - 2] = Tool.Tolerance(aCommonVertex);
            }

            Curve3D aCurrentCurve = Tool.Curve(aCurrentEdge, out double aFirstParam, out double aLastParam)!;
            var        aTrimmedCurve = new TrimmedCurve3D(aCurrentCurve, aFirstParam, aLastParam);
            aBSplines[i - 1] = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimmedCurve);
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref aBSplines[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());
            if (aToReverse)
            {
                aBSplines[i - 1] = owned(aBSplines[i - 1]);
                aBSplines[i - 1].Reverse();
            }

            aPrevVertex = aToReverse ? aCurrentFirstVertex : aCurrentLastVertex;
            aPrevEdge   = aCurrentEdge;
        }

        // Array of the concatenated curves.
        // Array of the remaining Vertex, effectively ignored here.
        // Flag of closedness of the concatenated curve, effectively ignored here.
        bool aClosedFlag = false;
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.ConcatC1(aBSplines,
                             aVerticesTolerances,
                             out NCollection_HArray1<int> _,
                             out NCollection_HArray1<BSplineCurve3D> aConcatCurves,
                             ref aClosedFlag,
                             global::Kerf.Engine.Foundation.Precision.Confusion()); // C1 concatenation

        if (aConcatCurves.Length() > 1)
        {
            var Concat = new CompCurveToBSplineCurve(aConcatCurves.Value(aConcatCurves.Lower()));

            for (int i = aConcatCurves.Lower() + 1; i <= aConcatCurves.Upper(); ++i)
            {
                Concat.Add(aConcatCurves.Value(i), aMaxTolerance, true);
            }

            aConcatCurves.SetValue(aConcatCurves.Lower(), Concat.InternalBSplineCurve()!);
        }

        BSplineCurve3D aResCurve = aConcatCurves.Value(aConcatCurves.Lower());
        Edge aResEdge = new BRepLib_MakeEdge(aResCurve,
                                                    FirstVertex,
                                                    LastVertex,
                                                    aResCurve.FirstParameter(),
                                                    aResCurve.LastParameter()).InternalEdge();
        var aBuilder = new BrepBuilder();
        aBuilder.SameRange(aResEdge, false);
        aBuilder.SameParameter(aResEdge, false);
        BRepLib.SameParameter((Shape)aResEdge, aMaxTolerance, true);
        return aResEdge;
    }

    /// <summary>Unifies the pcurves of the chain into one pcurve of the edge.</summary>
    protected void UnionPCurves(NCollection_Sequence<Shape> theChain, ref Edge theEdge)
    {
        Curve3D? aCurve   = Tool.Curve(theEdge, out double aFirst3d, out double aLast3d);
        double      aTolEdge = Tool.Tolerance(theEdge);
        double      aMaxTol  = aTolEdge;

        var aFaceSeq = new NCollection_Sequence<Shape>();

        Edge                    aFirstEdge = ShapeCasts.Edge(theChain.Value(1));
        NCollection_List<Shape> aFaceList  = myEFmap.FindFromKey(aFirstEdge);
        var                            anItl      = new NCollection_List<Shape>.Iterator(aFaceList);
        for (; anItl.More(); anItl.Next())
        {
            Face aFace = ShapeCasts.Face(anItl.Value());
            if (myFacePlaneMap.IsBound(aFace))
            {
                continue;
            }

            if (myFaceNewFace.IsBound(aFace))
            {
                aFace = ShapeCasts.Face(myFaceNewFace[aFace]);
            }

            aFace.Orientation(Forward); // to get proper pcurves of seam edges

            var aBAsurf = new FaceSurfaceAdaptor(aFace, false);
            if (aBAsurf.GetType() == Plane)
            {
                continue;
            }

            Surface? aSurf   = Tool.Surface(aFace, out Placement aLoc);
            bool          isFound = false;
            for (int ii = 1; ii <= aFaceSeq.Length(); ii++)
            {
                Surface? aPrevSurf = Tool.Surface(ShapeCasts.Face(aFaceSeq[ii]), out Placement aPrevLoc);
                if (ReferenceEquals(aPrevSurf, aSurf) && aPrevLoc == aLoc)
                {
                    isFound = true;
                    break;
                }
            }
            if (isFound)
            {
                continue;
            }

            aFaceSeq.Append(aFace);
        }

        var         ResPCurves = new NCollection_Sequence<Curve2D>();
        var         ResFirsts  = new NCollection_Sequence<double>();
        var         ResLasts   = new NCollection_Sequence<double>();
        var         aTolVerSeq = new NCollection_Sequence<double>();
        var         aPrevEdge  = new Edge();
        bool        anIsSeam   = false;

        for (int j = 1; j <= aFaceSeq.Length(); j++)
        {
            var               aPCurveSeq   = new NCollection_Sequence<Curve2D>();
            var               aFirstsSeq   = new NCollection_Sequence<double>();
            var               aLastsSeq    = new NCollection_Sequence<double>();
            var               aForwardsSeq = new NCollection_Sequence<bool>();
            CurveKind aCurrentType = OtherCurve;

            double aFirst, aLast;
            for (int i = 1; i <= theChain.Length(); i++)
            {
                Edge anEdge    = ShapeCasts.Edge(theChain.Value(i));
                bool        isForward = (anEdge.Orientation() != Reversed);

                if (anIsSeam && j == 2)
                { // second pass
                    anEdge.Reverse();
                }

                Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, ShapeCasts.Face(aFaceSeq[j]), out aFirst, out aLast);
                if (aPCurve is null)
                {
                    continue;
                }

                if (aFaceSeq.Length() == 1)
                {
                    Edge aReversedEdge = anEdge;
                    aReversedEdge.Reverse();
                    Curve2D? aPCurve2 =
                      Tool.CurveOnSurface(aReversedEdge, ShapeCasts.Face(aFaceSeq[j]), out aFirst, out aLast);
                    if (!ReferenceEquals(aPCurve, aPCurve2))
                    {
                        anIsSeam = true;
                        aFaceSeq.Append(aFaceSeq[j]);
                    }
                }

                var               anAdaptor = new CurveAdaptor2D(aPCurve);
                CurveKind aType     = anAdaptor.GetType();

                LineCurve2D? aLine = null;
                if (aType == BSplineCurve || aType == BezierCurve)
                {
                    TryMakeLine(aPCurve, aFirst, aLast, ref aLine);
                }
                if (aLine is not null)
                {
                    aPCurve = aLine;
                    anAdaptor.Load(aPCurve);
                    aType = Line;
                }

                if (aPCurveSeq.IsEmpty())
                {
                    var aCopyPCurve = (Curve2D)aPCurve.Copy();
                    if (aCopyPCurve is TrimmedCurve2D aCopyTrimmed)
                    {
                        aCopyPCurve = aCopyTrimmed.BasisCurve();
                    }

                    aPCurveSeq.Append(aCopyPCurve);
                    aFirstsSeq.Append(aFirst);
                    aLastsSeq.Append(aLast);
                    aForwardsSeq.Append(isForward);
                    aCurrentType = aType;
                    aPrevEdge    = anEdge;
                    continue;
                }

                bool   isSameCurve = false;
                double aNewF       = aFirst;
                double aNewL       = aLast;
                if (ReferenceEquals(aPCurve, aPCurveSeq.Last()))
                {
                    isSameCurve = true;
                }
                else if (aType == aCurrentType)
                {
                    var aPrevAdaptor = new CurveAdaptor2D(aPCurveSeq.Last());
                    switch (aType)
                    {
                        case Line:
                        {
                            Line2D aPrevLin  = aPrevAdaptor.Line();
                            Point2D aFirstP2d = aPCurve.Value(aFirst);
                            Point2D aLastP2d  = aPCurve.Value(aLast);
                            if (aPrevLin.Contains(aFirstP2d, global::Kerf.Engine.Foundation.Precision.Confusion())
                                && aPrevLin.Contains(aLastP2d, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                isSameCurve = true;
                                Point2D p1 = anAdaptor.Value(aFirst);
                                Point2D p2 = anAdaptor.Value(aLast);
                                aNewF = global::Kerf.Engine.Geometry.ElCLib.Parameter(aPrevLin, p1);
                                aNewL = global::Kerf.Engine.Geometry.ElCLib.Parameter(aPrevLin, p2);
                                if (aNewF > aNewL)
                                {
                                    double aTmp = aNewF;
                                    aNewF       = aNewL;
                                    aNewL       = aTmp;
                                }
                            }
                            break;
                        }
                        case Circle:
                        {
                            Circle2D aCirc     = anAdaptor.Circle();
                            Circle2D aPrevCirc = aPrevAdaptor.Circle();
                            if (aCirc.Location().Distance(aPrevCirc.Location()) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && Math.Abs(aCirc.Radius() - aPrevCirc.Radius()) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                isSameCurve = true;
                                Point2D p1 = anAdaptor.Value(aFirst);
                                Point2D p2 = anAdaptor.Value(aLast);
                                aNewF = global::Kerf.Engine.Geometry.ElCLib.Parameter(aPrevCirc, p1);
                                aNewL = global::Kerf.Engine.Geometry.ElCLib.Parameter(aPrevCirc, p2);
                                if (aNewF > aNewL)
                                {
                                    double aTmp = aNewF;
                                    aNewF       = aNewL;
                                    aNewL       = aTmp;
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
                    if (aForwardsSeq.Last())
                    {
                        aLastsSeq.ChangeLast() = aNewL;
                    }
                    else
                    {
                        aFirstsSeq.ChangeLast() = aNewF;
                    }
                }
                else
                {
                    var aCopyPCurve = (Curve2D)aPCurve.Copy();
                    if (aCopyPCurve is TrimmedCurve2D aCopyTrimmed)
                    {
                        aCopyPCurve = aCopyTrimmed.BasisCurve();
                    }

                    aPCurveSeq.Append(aCopyPCurve);
                    aFirstsSeq.Append(aFirst);
                    aLastsSeq.Append(aLast);
                    aForwardsSeq.Append(isForward);
                    aCurrentType = aType;
                    var aV = new Vertex();
                    Exploration.CommonVertex(aPrevEdge, anEdge, ref aV);
                    double aTol = Tool.Tolerance(aV);
                    aTolVerSeq.Append(aTol);
                }
                aPrevEdge = anEdge;
            }

            Curve2D aResPCurve;
            double       aResFirst, aResLast;
            if (aPCurveSeq.Length() == 1)
            {
                aResPCurve = aPCurveSeq.Last();
                aResFirst  = aFirstsSeq.Last();
                aResLast   = aLastsSeq.Last();
                if (!aForwardsSeq.Last())
                {
                    double aNewLast  = aResPCurve.ReversedParameter(aResFirst);
                    double aNewFirst = aResPCurve.ReversedParameter(aResLast);
                    // the basis of a copied trimmed curve, frozen by BasisCurve()
                    aResPCurve = owned(aResPCurve);
                    aResPCurve.Reverse();
                    aResFirst = aNewFirst;
                    aResLast  = aNewLast;
                }
            }
            else
            {
                // C1 concatenation for PCurveSeq
                var tab_c2d = new NCollection_Array1<BSplineCurve2D>(0, aPCurveSeq.Length() - 1);
                for (int i = 1; i <= aPCurveSeq.Length(); i++)
                {
                    var aTrPCurve = new TrimmedCurve2D(aPCurveSeq[i], aFirstsSeq[i], aLastsSeq[i]);
                    if (!aForwardsSeq[i])
                    {
                        aTrPCurve.Reverse();
                    }
                    tab_c2d[i - 1] = Geom2dConvert2D.CurveToBSplineCurve(aTrPCurve);
                    Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref tab_c2d[i - 1], global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                var tabtolvertex = new NCollection_Array1<double>(0, aTolVerSeq.Length() - 1);
                for (int i = 1; i <= aTolVerSeq.Length(); i++)
                {
                    double aTol = aTolVerSeq[i];
                    tabtolvertex[i - 1] = aTol;
                    if (aTol > aMaxTol)
                    {
                        aMaxTol = aTol;
                    }
                }

                bool aClosedFlag = false;
                Geom2dConvert2D.ConcatC1(tab_c2d,
                                       tabtolvertex,
                                       out NCollection_HArray1<int> _,                     // array of the remaining Vertex
                                       out NCollection_HArray1<BSplineCurve2D> concatc2d, // array of the concatenated curves
                                       ref aClosedFlag,
                                       global::Kerf.Engine.Foundation.Precision.Confusion()); // C1 concatenation

                if (concatc2d.Length() > 1)
                {
                    var Concat2d = new CompCurveToBSplineCurve2D(concatc2d.Value(concatc2d.Lower()));

                    for (int i = concatc2d.Lower() + 1; i <= concatc2d.Upper(); i++)
                    {
                        Concat2d.Add(concatc2d.Value(i), aMaxTol, true);
                    }

                    concatc2d.SetValue(concatc2d.Lower(), Concat2d.InternalBSplineCurve()!);
                }
                BSplineCurve2D aBSplineCurve = concatc2d.Value(concatc2d.Lower());
                aResPCurve = aBSplineCurve;
                aResFirst  = aBSplineCurve.FirstParameter();
                aResLast   = aBSplineCurve.LastParameter();
            }
            ResPCurves.Append(aResPCurve);
            ResFirsts.Append(aResFirst);
            ResLasts.Append(aResLast);
        }

        var aBuilder = new BrepBuilder();

        // Check the results for consistency
        bool   IsBadRange = false;
        double aRange3d   = aLast3d - aFirst3d;
        for (int ii = 1; ii <= ResPCurves.Length(); ii++)
        {
            double aRange = ResLasts[ii] - ResFirsts[ii];
            if (Math.Abs(aRange3d - aRange) > aMaxTol)
            {
                IsBadRange = true;
            }
        }

        if (IsBadRange)
        {
            for (int ii = 1; ii <= ResPCurves.Length(); ii++)
            {
                Face aFace     = ShapeCasts.Face(aFaceSeq[ii]);
                Surface? aSurf   = Tool.Surface(aFace);
                var         aSAS      = new ShapeAnalysis_Surface(aSurf);
                var         aToolProj = new ShapeConstruct_ProjectCurveOnSurface();
                aToolProj.Init(aSAS, global::Kerf.Engine.Foundation.Precision.Confusion());
                if (aToolProj.InternalPerform(aCurve, aFirst3d, aLast3d, out Curve2D? aNewPCurve))
                {
                    ResPCurves[ii] = aNewPCurve!;
                }
                else
                {
                    // Reparametrize pcurve
                    var aTrPCurve = new TrimmedCurve2D(ResPCurves[ii], ResFirsts[ii], ResLasts[ii]);
                    BSplineCurve2D aBSplinePCurve = owned(Geom2dConvert2D.CurveToBSplineCurve(aTrPCurve));
                    var aKnots = new NCollection_Array1<double>(aBSplinePCurve.InternalKnots());
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(aFirst3d, aLast3d, aKnots);
                    aBSplinePCurve.SetKnots(aKnots);
                    ResPCurves[ii] = aBSplinePCurve;
                }
                ResFirsts[ii] = aFirst3d;
                ResLasts[ii]  = aLast3d;
            }
        }

        // Reparametrize pcurves if needed
        if (!ResPCurves.IsEmpty())
        {
            for (int ii = 1; ii <= ResPCurves.Length(); ii++)
            {
                if (Math.Abs(aFirst3d - ResFirsts[ii]) > aMaxTol || Math.Abs(aLast3d - ResLasts[ii]) > aMaxTol)
                {
                    var               aGAcurve = new CurveAdaptor2D(ResPCurves[ii]);
                    CurveKind aType    = aGAcurve.GetType();
                    if (aType == Line)
                    {
                        Line2D aLin2d   = aGAcurve.Line();
                        Direction2D aDir2d   = aLin2d.Direction();
                        Point2D aPnt2d   = aGAcurve.Value(ResFirsts[ii]);
                        Vector2D anOffset = -new Vector2D(aDir2d);
                        anOffset *= aFirst3d;
                        aPnt2d.Translate(anOffset);
                        var aNewLine2d = new LineCurve2D(aPnt2d, aDir2d);
                        ResPCurves[ii] = aNewLine2d;
                    }
                    else if (aType == Circle)
                    {
                        Circle2D aCirc2d   = aGAcurve.Circle();
                        double    aRadius   = aCirc2d.Radius();
                        Frame2D  aPosition = aCirc2d.Position();
                        Point2D  aLocation = aCirc2d.Location();
                        double    anOffset  = ResFirsts[ii] - aFirst3d;
                        aPosition.Rotate(aLocation, anOffset);
                        var aNewCircle2d = new CircleCurve2D(aPosition, aRadius);
                        ResPCurves[ii] = aNewCircle2d;
                    }
                    else // general case
                    {
                        var aTrPCurve = new TrimmedCurve2D(ResPCurves[ii], ResFirsts[ii], ResLasts[ii]);
                        BSplineCurve2D aBSplinePCurve = owned(Geom2dConvert2D.CurveToBSplineCurve(aTrPCurve));
                        var aKnots = new NCollection_Array1<double>(aBSplinePCurve.InternalKnots());
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(aFirst3d, aLast3d, aKnots);
                        aBSplinePCurve.SetKnots(aKnots);
                        ResPCurves[ii] = aBSplinePCurve;
                    }
                    ResFirsts[ii] = aFirst3d;
                    ResLasts[ii]  = aLast3d;
                } // if ranges > aMaxTol
            } // for (int ii = 1; ii <= ResPCurves.Length(); ii++)
        }

        if (anIsSeam)
        {
            aBuilder.UpdateEdge(theEdge, ResPCurves[1], ResPCurves[2], ShapeCasts.Face(aFaceSeq[1]), aTolEdge);
        }
        else
        {
            for (int j = 1; j <= ResPCurves.Length(); j++)
            {
                aBuilder.UpdateEdge(theEdge, ResPCurves[j], ShapeCasts.Face(aFaceSeq[j]), aTolEdge);
            }
        }
    }

    /// <summary>Merges a sequence of edges into one edge if possible.</summary>
    protected bool MergeSubSeq(NCollection_Sequence<Shape>                                       theChain,
                               NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap,
                               ref Edge                                                          OutEdge)
    {
        var sae = new ShapeAnalysis_Edge();
        var B   = new BrepBuilder();
        // union edges in chain
        int         j;
        double      fp1, lp1;
        bool        IsUnionOfLinesPossible   = true;
        bool        IsUnionOfCirclesPossible = true;
        Curve3D? c3d1, c3d2;
        for (j = 1; j < theChain.Length(); j++)
        {
            Edge edge1 = ShapeCasts.Edge(theChain.Value(j));
            Edge edge2 = ShapeCasts.Edge(theChain.Value(j + 1));

            if (Tool.Degenerated(edge1) && Tool.Degenerated(edge2))
            {
                // Find the closest points in 2d
                Edge        edgeFirst    = ShapeCasts.Edge(theChain.First());
                Edge        edgeLast     = ShapeCasts.Edge(theChain.Last());
                var                CommonFace   = new Face();
                double             MinSqDist    = 0.0;
                Orientation OrOfE1OnFace = Forward, OrOfE2OnFace = Forward;
                int                IndOnE1      = 0, IndOnE2 = 0;
                Span<Point2D>     PointsOnEdge1 = [new Point2D(), new Point2D()];
                Span<Point2D>     PointsOnEdge2 = [new Point2D(), new Point2D()];
                if (!FindClosestPoints(edgeFirst,
                                       edgeLast,
                                       theVFmap,
                                       ref CommonFace,
                                       ref MinSqDist,
                                       ref OrOfE1OnFace,
                                       ref OrOfE2OnFace,
                                       ref IndOnE1,
                                       ref IndOnE2,
                                       PointsOnEdge1,
                                       PointsOnEdge2))
                {
                    return false;
                }

                // Define indices corresponding to extremities of future edge
                IndOnE1 = 1 - IndOnE1;
                IndOnE2 = 1 - IndOnE2;

                // Construct new degenerated edge
                Point2D StartPoint = PointsOnEdge1[IndOnE1];
                Point2D EndPoint   = PointsOnEdge2[IndOnE2];
                if ((OrOfE1OnFace == Forward && IndOnE1 == 1)
                    || (OrOfE1OnFace == Reversed && IndOnE1 == 0))
                {
                    Point2D Tmp = StartPoint;
                    StartPoint = EndPoint;
                    EndPoint   = Tmp;
                }

                LineCurve2D aLine = new MakeLine2d(StartPoint, EndPoint).Value();

                Vertex aVertex     = Exploration.FirstVertex(edgeFirst);
                Vertex StartVertex = aVertex, EndVertex = aVertex;
                StartVertex.Orientation(Forward);
                EndVertex.Orientation(Reversed);

                var NewEdge = new Edge();
                B.MakeEdge(ref NewEdge);
                B.UpdateEdge(NewEdge, aLine, CommonFace, global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Range(NewEdge, 0.0, StartPoint.Distance(EndPoint));
                B.Add(NewEdge, StartVertex);
                B.Add(NewEdge, EndVertex);
                B.Degenerated(NewEdge, true);
                OutEdge = NewEdge;
                return true;
            }

            c3d1 = Tool.Curve(edge1, out fp1, out lp1);
            c3d2 = Tool.Curve(edge2, out _, out _);

            var    aBAcurve1 = new EdgeCurveAdaptor(edge1);
            var    aBAcurve2 = new EdgeCurveAdaptor(edge2);
            Direction3D aDir1 = new(), aDir2 = new();

            if (c3d1 is null || c3d2 is null)
            {
                return false;
            }

            if (c3d1 is TrimmedCurve3D tc1)
            {
                c3d1 = tc1.BasisCurve();
            }
            if (c3d2 is TrimmedCurve3D tc2)
            {
                c3d2 = tc2.BasisCurve();
            }
            if (IsLinear(aBAcurve1, ref aDir1) && IsLinear(aBAcurve2, ref aDir2))
            {
                if (!aDir1.IsParallel(aDir2, myAngTol))
                {
                    IsUnionOfLinesPossible = false;
                }
            }
            else
            {
                IsUnionOfLinesPossible = false;
            }
            if (c3d1 is CircleCurve3D C1 && c3d2 is CircleCurve3D C2)
            {
                Point3D P01 = C1.Location();
                Point3D P02 = C2.Location();
                if (P01.Distance(P02) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    IsUnionOfCirclesPossible = false;
                }
            }
            else
            {
                IsUnionOfCirclesPossible = false;
            }
        }
        if (IsUnionOfLinesPossible && IsUnionOfCirclesPossible)
        {
            return false;
        }

        // union of lines is possible
        if (IsUnionOfLinesPossible)
        {
            var V = new Vertex[2];
            V[0] = sae.FirstVertex(ShapeCasts.Edge(theChain.First()));
            Point3D PV1 = Tool.Pnt(V[0]);
            V[1] = sae.LastVertex(ShapeCasts.Edge(theChain.Last()));
            Point3D PV2 = Tool.Pnt(V[1]);
            var    Vec = new Vector3D(PV1, PV2);
            if (mySafeInputMode)
            {
                for (int k = 0; k < 2; k++)
                {
                    if (!myContext.IsRecorded(V[k]))
                    {
                        Vertex Vcopy = ShapeCasts.Vertex(V[k].EmptyCopied());
                        myContext.Replace(V[k], Vcopy);
                        V[k] = Vcopy;
                    }
                    else
                    {
                        V[k] = ShapeCasts.Vertex(myContext.InternalApply(V[k]));
                    }
                }
            }
            var    L    = new LineCurve3D(new Axis3D(PV1, new Direction3D(Vec)));
            double dist = PV1.Distance(PV2);
            var    tc   = new TrimmedCurve3D(L, 0.0, dist);
            var    E    = new Edge();
            B.MakeEdge(ref E, tc, global::Kerf.Engine.Foundation.Precision.Confusion());
            B.Add(E, V[0]);
            B.Add(E, V[1]);
            B.UpdateVertex(V[0], 0.0, E, 0.0);
            B.UpdateVertex(V[1], dist, E, 0.0);
            UnionPCurves(theChain, ref E);
            OutEdge = E;
            return true;
        }

        if (IsUnionOfCirclesPossible)
        {
            Edge FE  = ShapeCasts.Edge(theChain.First());
            Curve3D  c3d = Tool.Curve(FE, out _, out _)!;

            if (c3d is TrimmedCurve3D tc)
            {
                c3d = tc.BasisCurve();
            }
            var Cir = (CircleCurve3D)c3d;

            var V = new Vertex[2];
            V[0] = sae.FirstVertex(FE);
            V[1] = sae.LastVertex(ShapeCasts.Edge(theChain.Last()));
            bool isClosed = V[0].IsSame(V[1]);
            if (!isClosed)
            {
                // additionally check the points for equality to make a final decision about closedness of the
                // result curve
                Point3D aP0  = Tool.Pnt(V[0]);
                Point3D aP1  = Tool.Pnt(V[1]);
                double aTol = Max(Tool.Tolerance(V[0]), Tool.Tolerance(V[1]));
                if (aP0.SquareDistance(aP1) < aTol * aTol)
                {
                    isClosed = true;
                    V[1]     = V[0];
                    V[1].Reverse();
                }
            }
            var E = new Edge();
            if (isClosed)
            {
                // closed chain
                var          adef = new EdgeCurveAdaptor(FE);
                CircleCurve3D  Cir1;
                double       FP, LP;
                if (FE.Orientation() == Forward)
                {
                    FP = adef.FirstParameter();
                    LP = adef.LastParameter();
                }
                else
                {
                    FP = adef.LastParameter();
                    LP = adef.FirstParameter();
                }
                if (Math.Abs(FP) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    B.MakeEdge(ref E, Cir, global::Kerf.Engine.Foundation.Precision.Confusion());
                    B.Add(E, V[0]);
                    B.Add(E, V[1]);
                    E.Orientation(FE.Orientation());
                }
                else
                {
                    var MC1 = new MakeCircle(adef.Value(FP), adef.Value((FP + LP) * 0.5), adef.Value(LP));
                    if (MC1.IsDone)
                    {
                        Cir1 = MC1.Value();
                    }
                    else
                    {
                        return false;
                    }
                    B.MakeEdge(ref E, Cir1, global::Kerf.Engine.Foundation.Precision.Confusion());
                    B.Add(E, V[0]);
                    B.Add(E, V[1]);
                }
            }
            else // open chain
            {
                double        ParamFirst     = Tool.Parameter(V[0], FE);
                Vertex VertexLastOnFE = sae.LastVertex(FE);
                double        ParamLast      = Tool.Parameter(VertexLastOnFE, FE);

                if (mySafeInputMode)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        if (!myContext.IsRecorded(V[k]))
                        {
                            Vertex Vcopy = ShapeCasts.Vertex(V[k].EmptyCopied());
                            myContext.Replace(V[k], Vcopy);
                            V[k] = Vcopy;
                        }
                        else
                        {
                            V[k] = ShapeCasts.Vertex(myContext.InternalApply(V[k]));
                        }
                    }
                }

                Point3D PointFirst = Tool.Pnt(V[0]);
                while (Math.Abs(ParamLast - ParamFirst) > 7 * M_PI / 8)
                {
                    ParamLast = (ParamFirst + ParamLast) / 2;
                }
                var    BAcurveFE        = new EdgeCurveAdaptor(FE);
                Point3D PointLast        = BAcurveFE.Value(ParamLast);
                Point3D Origin           = Cir.Circ().Location();
                var    Dir1             = new Direction3D(new Vector3D(Origin, PointFirst));
                var    Dir2             = new Direction3D(new Vector3D(Origin, PointLast));
                Direction3D Vdir             = Dir1 ^ Dir2;
                var    anAx2            = new Frame3D(Origin, Vdir, Dir1);
                var    aNewCircle       = new CircleCurve3D(anAx2, Cir.Radius());
                Point3D PointLastInChain = Tool.Pnt(V[1]);
                var    DirLastInChain   = new Direction3D(new Vector3D(Origin, PointLastInChain));
                double lpar             = Dir1.AngleWithRef(DirLastInChain, Vdir);
                if (lpar < 0.0)
                {
                    lpar += 2 * M_PI;
                }

                var tc1 = new TrimmedCurve3D(aNewCircle, 0.0, lpar);
                B.MakeEdge(ref E, tc1, global::Kerf.Engine.Foundation.Precision.Confusion());
                B.Add(E, V[0]);
                B.Add(E, V[1]);
                B.UpdateVertex(V[0], 0.0, E, 0.0);
                B.UpdateVertex(V[1], lpar, E, 0.0);
            }
            UnionPCurves(theChain, ref E);
            OutEdge = E;
            return true;
        }
        if (theChain.Length() > 1 && myConcatBSplines)
        {
            // second step: union edges with various curves
            // skl for bug 0020052 from Mantis: perform such unions
            // only if curves are bspline or bezier

            Vertex VF        = sae.FirstVertex(ShapeCasts.Edge(theChain.First()));
            Vertex VL        = sae.LastVertex(ShapeCasts.Edge(theChain.Last()));
            bool          NeedUnion = true;
            for (j = 1; j <= theChain.Length(); j++)
            {
                Edge edge = ShapeCasts.Edge(theChain.Value(j));
                Curve3D? c3d  = Tool.Curve(edge, out _, out fp1, out lp1);
                if (c3d is null)
                {
                    continue;
                }
                if (c3d is TrimmedCurve3D tc)
                {
                    c3d = tc.BasisCurve();
                }
                if (c3d is BSplineCurve3D || c3d is BezierCurve3D)
                {
                    continue;
                }
                NeedUnion = false;
                break;
            }
            if (NeedUnion)
            {
                OutEdge = GlueEdgesWith3DCurves(theChain, VF, VL);
                return true;
            }
        }
        return false;
    }

    // Checks if merging of two edges is possible
    private static bool IsMergingPossible(in Edge                                                         edge1,
                                          in Edge                                                         edge2,
                                          double                                                                 theAngTol,
                                          double                                                                 theLinTol,
                                          NCollection_Map<Shape>                                          AvoidEdgeVrt,
                                          bool                                                                   theLineDirectionOk,
                                          in Point3D                                                              theFirstPoint,
                                          in Vector3D                                                              theDirectionVec,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap)
    {
        bool IsDegE1 = Tool.Degenerated(edge1);
        bool IsDegE2 = Tool.Degenerated(edge2);

        if (IsDegE1 && IsDegE2)
        {
            // Find connstion point in 2d
            var                CommonFace    = new Face();
            double             MinSqDist     = 0.0;
            Orientation OrOfE1OnFace  = Forward, OrOfE2OnFace = Forward;
            int                IndOnE1       = 0, IndOnE2 = 0;
            Span<Point2D>     PointsOnEdge1 = [new Point2D(), new Point2D()];
            Span<Point2D>     PointsOnEdge2 = [new Point2D(), new Point2D()];
            if (!FindClosestPoints(edge1,
                                   edge2,
                                   theVFmap,
                                   ref CommonFace,
                                   ref MinSqDist,
                                   ref OrOfE1OnFace,
                                   ref OrOfE2OnFace,
                                   ref IndOnE1,
                                   ref IndOnE2,
                                   PointsOnEdge1,
                                   PointsOnEdge2))
            {
                return false;
            }

            if (MinSqDist <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                return true;
            }

            return false;
        }
        else if (IsDegE1 || IsDegE2)
        {
            return false;
        }

        Vertex CV = Exploration.LastVertex(edge1, true);
        if (CV.IsNull || AvoidEdgeVrt.Contains(CV))
        {
            return false;
        }

        var ade1 = new EdgeCurveAdaptor(edge1);
        var ade2 = new EdgeCurveAdaptor(edge2);

        CurveKind t1 = ade1.GetType();
        CurveKind t2 = ade2.GetType();

        if (t1 == Circle && t2 == Circle)
        {
            if (ade1.Circle().Location().Distance(ade2.Circle().Location()) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }

        Direction3D aDir1 = new(), aDir2 = new();
        if (!(IsLinear(ade1, ref aDir1) && IsLinear(ade2, ref aDir2))
            && ((t1 != BezierCurve && t1 != BSplineCurve)
                || (t2 != BezierCurve && t2 != BSplineCurve))
            && t1 != t2)
        {
            return false;
        }

        Vector3D Diff1, Diff2;
        Point3D P1, P2;
        if (edge1.Orientation() == Forward)
        {
            ade1.D1(ade1.LastParameter(), out P1, out Diff1);
        }
        else
        {
            ade1.D1(ade1.FirstParameter(), out P1, out Diff1);
            Diff1 = -Diff1;
        }

        if (edge2.Orientation() == Forward)
        {
            ade2.D1(ade2.FirstParameter(), out P2, out Diff2);
        }
        else
        {
            ade2.D1(ade2.LastParameter(), out P2, out Diff2);
            Diff2 = -Diff2;
        }

        if (Diff1.Angle(Diff2) > theAngTol)
        {
            return false;
        }

        if (theLineDirectionOk && t2 == Line)
        {
            // Check that the accumulated deflection does not exceed the linear tolerance
            double aLast = (edge2.Orientation() == Forward) ? ade2.LastParameter() : ade2.FirstParameter();
            var    aCurV = new Vector3D(theFirstPoint, ade2.Value(aLast));
            double aDD   = theDirectionVec.CrossSquareMagnitude(aCurV);
            if (aDD > theLinTol * theLinTol)
            {
                return false;
            }

            // Check that the accumulated angle does not exceed the angular tolerance.
            // For symmetry, check the angle between vectors of:
            // - first edge and resulting curve, and
            // - the last edge and resulting curve.
            if (theDirectionVec.Angle(aCurV) > theAngTol || Diff2.Angle(aCurV) > theAngTol)
            {
                return false;
            }
        }

        return true;
    }

    private static bool GetLineEdgePoints(in Edge theInpEdge, ref Point3D theFirstPoint, ref Vector3D theDirectionVec)
    {
        Curve3D? aCur = Tool.Curve(theInpEdge, out double f, out double l);
        if (aCur is null)
        {
            return false;
        }

        if (aCur is TrimmedCurve3D aTC)
        {
            aCur = aTC.BasisCurve();
        }

        if (aCur.GetType() != typeof(LineCurve3D))
        {
            return false;
        }

        if (theInpEdge.Orientation() == Reversed)
        {
            double tmp = f;
            f = l;
            l = tmp;
        }
        theFirstPoint = aCur.Value(f);
        Point3D aLP = aCur.Value(l);
        theDirectionVec = new Vector3D(aLP.XYZ().Subtracted(theFirstPoint.XYZ()));
        theDirectionVec.Normalize();
        return true;
    }

    // Generates sub-sequences of edges from sequence of edges. Edges from each subsequences can be merged into the
    // one edge.
    private static void generateSubSeq(NCollection_Sequence<Shape>                                       anInpEdgeSeq,
                                       NCollection_Sequence<SubSequenceOfEdges>                                 SeqOfSubSeqOfEdges,
                                       bool                                                                     IsClosed,
                                       double                                                                   theAngTol,
                                       double                                                                   theLinTol,
                                       NCollection_Map<Shape>                                            AvoidEdgeVrt,
                                       NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap)
    {
        bool        isOk;
        Edge edge1, edge2;

        var         SubSeq  = new SubSequenceOfEdges();
        Edge RefEdge = ShapeCasts.Edge(anInpEdgeSeq[1]);
        SubSeq.SeqsEdges.Append(RefEdge);
        SeqOfSubSeqOfEdges.Append(SubSeq);

        var  aFirstPoint       = new Point3D();
        var  aDirectionVec     = new Vector3D();
        bool isLineDirectionOk = GetLineEdgePoints(RefEdge, ref aFirstPoint, ref aDirectionVec);

        for (int i = 1; i < anInpEdgeSeq.Length(); i++)
        {
            edge1 = ShapeCasts.Edge(anInpEdgeSeq[i]);
            edge2 = ShapeCasts.Edge(anInpEdgeSeq[i + 1]);
            isOk  = IsMergingPossible(edge1,
                                     edge2,
                                     theAngTol,
                                     theLinTol,
                                     AvoidEdgeVrt,
                                     isLineDirectionOk,
                                     aFirstPoint,
                                     aDirectionVec,
                                     theVFmap);
            if (!isOk)
            {
                var aSubSeq = new SubSequenceOfEdges();
                aSubSeq.SeqsEdges.Append(edge2);
                SeqOfSubSeqOfEdges.Append(aSubSeq);
                isLineDirectionOk = GetLineEdgePoints(edge2, ref aFirstPoint, ref aDirectionVec);
            }
            else
            {
                SeqOfSubSeqOfEdges.ChangeLast().SeqsEdges.Append(edge2);
            }
        }
        // check first and last chain segments
        if (IsClosed && SeqOfSubSeqOfEdges.Length() > 1)
        {
            edge1 = ShapeCasts.Edge(anInpEdgeSeq.Last());
            edge2 = ShapeCasts.Edge(anInpEdgeSeq.First());
            if (IsMergingPossible(edge1,
                                  edge2,
                                  theAngTol,
                                  theLinTol,
                                  AvoidEdgeVrt,
                                  false,
                                  aFirstPoint,
                                  aDirectionVec,
                                  theVFmap))
            {
                SeqOfSubSeqOfEdges.ChangeLast().SeqsEdges.Append(SeqOfSubSeqOfEdges.ChangeFirst().SeqsEdges);
                SeqOfSubSeqOfEdges.Remove(1);
            }
        }
    }

    /// <summary>Splits the sequence of edges into the sequence of chains.</summary>
    protected bool MergeEdges(NCollection_Sequence<Shape>                                       SeqEdges,
                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap,
                              NCollection_Sequence<SubSequenceOfEdges>                                 SeqOfSubSeqOfEdges,
                              NCollection_Map<Shape>                                            NonMergVrt)
    {
        var aMapVE          = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int j;
        var VerticesToAvoid = new NCollection_Map<Shape>(ShapeHasher.Instance);
        int aNbE            = SeqEdges.Length();
        for (j = 1; j <= aNbE; j++)
        {
            Edge anEdge = ShapeCasts.Edge(SeqEdges[j]);
            // fill in the map V-E
            for (var it = new Iterator(anEdge.Oriented(Forward)); it.More(); it.Next())
            {
                Shape aV = it.Value();
                if (aV.Orientation() == Forward || aV.Orientation() == Reversed)
                {
                    if (!aMapVE.Contains(aV))
                    {
                        aMapVE.Add(aV, new NCollection_List<Shape>());
                    }
                    aMapVE.ChangeFromKey(aV).Append(anEdge);
                }
            }
        }
        NCollection_MapAlgo.Unite(VerticesToAvoid, NonMergVrt);

        // do loop while there are unused edges
        var aUsedEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);

        for (int iE = 1; iE <= aNbE; ++iE)
        {
            Edge edge = ShapeCasts.Edge(SeqEdges[iE]);
            if (!aUsedEdges.Add(edge))
            {
                continue;
            }

            // make chain for unite
            var aChain = new NCollection_Sequence<Shape>();
            aChain.Append(edge);
            var V = new Vertex[2];
            Exploration.Vertices(edge, out V[0], out V[1], true);

            // connect more edges to the chain in both directions
            for (j = 0; j < 2; j++)
            {
                bool isAdded = true;
                while (isAdded)
                {
                    isAdded = false;
                    if (V[j].IsNull)
                    {
                        break;
                    }
                    NCollection_List<Shape> aLE = aMapVE.FindFromKey(V[j]);
                    for (var itL = new NCollection_List<Shape>.Iterator(aLE); itL.More(); itL.Next())
                    {
                        edge = ShapeCasts.Edge(itL.Value());
                        if (!aUsedEdges.Contains(edge))
                        {
                            var V2 = new Vertex[2];
                            Exploration.Vertices(edge, out V2[0], out V2[1], true);
                            // the neighboring edge must have V[j] reversed and located on the opposite end
                            if (V2[1 - j].IsEqual(V[j].Reversed()))
                            {
                                if (j == 0)
                                {
                                    aChain.Prepend(edge);
                                }
                                else
                                {
                                    aChain.Append(edge);
                                }
                                aUsedEdges.Add(edge);
                                V[j]    = V2[j];
                                isAdded = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (aChain.Length() < 2)
            {
                continue;
            }

            bool IsClosed = false;
            if (V[0].IsSame(V[1]))
            {
                IsClosed = true;
            }

            // split chain by vertices at which merging is not possible
            var aOneSeq = new NCollection_Sequence<SubSequenceOfEdges>();
            generateSubSeq(aChain, aOneSeq, IsClosed, myAngTol, myLinTol, VerticesToAvoid, theVFmap);

            // put sub-chains in the result
            SeqOfSubSeqOfEdges.Append(aOneSeq);
        }

        for (int i = 1; i <= SeqOfSubSeqOfEdges.Length(); i++)
        {
            var UE = new Edge();
            if (SeqOfSubSeqOfEdges[i].SeqsEdges.Length() < 2)
            {
                continue;
            }
            if (MergeSubSeq(SeqOfSubSeqOfEdges[i].SeqsEdges, theVFmap, ref UE))
            {
                SeqOfSubSeqOfEdges[i].UnionEdges = UE;
            }
        }
        return true;
    }

    /// <summary>Tries to unify the sequence of edges with the set of another edges which lies on the same geometry.</summary>
    protected bool MergeSeq(NCollection_Sequence<Shape>                                       SeqEdges,
                            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theVFmap,
                            NCollection_Map<Shape>                                            nonMergVert)
    {
        var SeqOfSubsSeqOfEdges = new NCollection_Sequence<SubSequenceOfEdges>();
        if (MergeEdges(SeqEdges, theVFmap, SeqOfSubsSeqOfEdges, nonMergVert))
        {
            for (int i = 1; i <= SeqOfSubsSeqOfEdges.Length(); i++)
            {
                if (SeqOfSubsSeqOfEdges[i].UnionEdges.IsNull)
                {
                    continue;
                }

                myContext.Merge(SeqOfSubsSeqOfEdges[i].SeqsEdges, SeqOfSubsSeqOfEdges[i].UnionEdges);
            }
            return true;
        }
        return false;
    }
}
