// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the unification of
// edges: MergeEdges, MergeSeq, MergeSubSeq, UnionPCurves and their file-static functions; the class is in
// ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;

namespace Occt;

public partial class ShapeUpgrade_UnifySameDomain
{
    /// <summary>A chain of edges that can be merged, and the edge they are merged into (null if not merged).</summary>
    protected sealed class SubSequenceOfEdges
    {
        /// <summary>The edges of the chain.</summary>
        public NCollection_Sequence<TopoDS_Shape> SeqsEdges = new();
        /// <summary>The edge they are merged into.</summary>
        public TopoDS_Edge UnionEdges = new();
    }

    // OCCT.NET: frozen. A curve OCCT edits in place that may have been handed out frozen (a basis curve read through
    // BasisCurve(), which freezes it): the edit is made on a copy. OCCT edits curves it made itself at these sites.
    private static T owned<T>(T theCurve) where T : Standard_Transient
        => theCurve.IsFrozen() ? (T)(object)copyOf(theCurve) : theCurve;

    private static object copyOf(Standard_Transient theCurve) => theCurve switch
    {
        Geom2d_Geometry aG2d => aG2d.Copy(),
        Geom_Geometry aG => aG.Copy(),
        _ => throw new Standard_ProgramError("ShapeUpgrade_UnifySameDomain: not a geometry"),
    };

    // Glues 3d curves of the sequence of edges
    private static TopoDS_Edge GlueEdgesWith3DCurves(NCollection_Sequence<TopoDS_Shape> aChain,
                                                     in TopoDS_Vertex                   FirstVertex,
                                                     in TopoDS_Vertex                   LastVertex)
    {
        // number of curves
        int aCurveCount = aChain.Length();

        TopoDS_Edge   aPrevEdge     = TopoDS.Edge(aChain[1]);
        TopoDS_Vertex aPrevVertex   = FirstVertex;
        double        aMaxTolerance = 0.0;
        // array of the curves converted to BSpline
        var aBSplines = new NCollection_Array1<Geom_BSplineCurve>(0, aCurveCount - 1);
        // array of the tolerances at the vertices
        var aVerticesTolerances = new NCollection_Array1<double>(0, aCurveCount - 1);
        for (int i = 1; i <= aCurveCount; ++i)
        {
            TopoDS_Edge aCurrentEdge = TopoDS.Edge(aChain[i]);
            TopExp.Vertices(aCurrentEdge, out TopoDS_Vertex aCurrentFirstVertex, out TopoDS_Vertex aCurrentLastVertex);
            bool aToReverse = !aCurrentFirstVertex.IsSame(aPrevVertex);

            aMaxTolerance = Max(Max(aMaxTolerance, BRep_Tool.Tolerance(aCurrentFirstVertex)),
                                BRep_Tool.Tolerance(aCurrentLastVertex));
            if (i > 1)
            {
                var aCommonVertex = new TopoDS_Vertex();
                TopExp.CommonVertex(aPrevEdge, aCurrentEdge, ref aCommonVertex);
                aVerticesTolerances[i - 2] = BRep_Tool.Tolerance(aCommonVertex);
            }

            Geom_Curve aCurrentCurve = BRep_Tool.Curve(aCurrentEdge, out double aFirstParam, out double aLastParam)!;
            var        aTrimmedCurve = new Geom_TrimmedCurve(aCurrentCurve, aFirstParam, aLastParam);
            aBSplines[i - 1] = GeomConvert.CurveToBSplineCurve(aTrimmedCurve);
            GeomConvert.C0BSplineToC1BSplineCurve(ref aBSplines[i - 1], Precision.Confusion());
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
        GeomConvert.ConcatC1(aBSplines,
                             aVerticesTolerances,
                             out NCollection_HArray1<int> _,
                             out NCollection_HArray1<Geom_BSplineCurve> aConcatCurves,
                             ref aClosedFlag,
                             Precision.Confusion()); // C1 concatenation

        if (aConcatCurves.Length() > 1)
        {
            var Concat = new GeomConvert_CompCurveToBSplineCurve(aConcatCurves.Value(aConcatCurves.Lower()));

            for (int i = aConcatCurves.Lower() + 1; i <= aConcatCurves.Upper(); ++i)
            {
                Concat.Add(aConcatCurves.Value(i), aMaxTolerance, true);
            }

            aConcatCurves.SetValue(aConcatCurves.Lower(), Concat.InternalBSplineCurve()!);
        }

        Geom_BSplineCurve aResCurve = aConcatCurves.Value(aConcatCurves.Lower());
        TopoDS_Edge aResEdge = new BRepLib_MakeEdge(aResCurve,
                                                    FirstVertex,
                                                    LastVertex,
                                                    aResCurve.FirstParameter(),
                                                    aResCurve.LastParameter()).InternalEdge();
        var aBuilder = new BRep_Builder();
        aBuilder.SameRange(aResEdge, false);
        aBuilder.SameParameter(aResEdge, false);
        BRepLib.SameParameter((TopoDS_Shape)aResEdge, aMaxTolerance, true);
        return aResEdge;
    }

    /// <summary>Unifies the pcurves of the chain into one pcurve of the edge.</summary>
    protected void UnionPCurves(NCollection_Sequence<TopoDS_Shape> theChain, ref TopoDS_Edge theEdge)
    {
        Geom_Curve? aCurve   = BRep_Tool.Curve(theEdge, out double aFirst3d, out double aLast3d);
        double      aTolEdge = BRep_Tool.Tolerance(theEdge);
        double      aMaxTol  = aTolEdge;

        var aFaceSeq = new NCollection_Sequence<TopoDS_Shape>();

        TopoDS_Edge                    aFirstEdge = TopoDS.Edge(theChain.Value(1));
        NCollection_List<TopoDS_Shape> aFaceList  = myEFmap.FindFromKey(aFirstEdge);
        var                            anItl      = new NCollection_List<TopoDS_Shape>.Iterator(aFaceList);
        for (; anItl.More(); anItl.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(anItl.Value());
            if (myFacePlaneMap.IsBound(aFace))
            {
                continue;
            }

            if (myFaceNewFace.IsBound(aFace))
            {
                aFace = TopoDS.Face(myFaceNewFace[aFace]);
            }

            aFace.Orientation(TopAbs_FORWARD); // to get proper pcurves of seam edges

            var aBAsurf = new BRepAdaptor_Surface(aFace, false);
            if (aBAsurf.GetType() == GeomAbs_Plane)
            {
                continue;
            }

            Geom_Surface? aSurf   = BRep_Tool.Surface(aFace, out TopLoc_Location aLoc);
            bool          isFound = false;
            for (int ii = 1; ii <= aFaceSeq.Length(); ii++)
            {
                Geom_Surface? aPrevSurf = BRep_Tool.Surface(TopoDS.Face(aFaceSeq[ii]), out TopLoc_Location aPrevLoc);
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

        var         ResPCurves = new NCollection_Sequence<Geom2d_Curve>();
        var         ResFirsts  = new NCollection_Sequence<double>();
        var         ResLasts   = new NCollection_Sequence<double>();
        var         aTolVerSeq = new NCollection_Sequence<double>();
        var         aPrevEdge  = new TopoDS_Edge();
        bool        anIsSeam   = false;

        for (int j = 1; j <= aFaceSeq.Length(); j++)
        {
            var               aPCurveSeq   = new NCollection_Sequence<Geom2d_Curve>();
            var               aFirstsSeq   = new NCollection_Sequence<double>();
            var               aLastsSeq    = new NCollection_Sequence<double>();
            var               aForwardsSeq = new NCollection_Sequence<bool>();
            GeomAbs_CurveType aCurrentType = GeomAbs_OtherCurve;

            double aFirst, aLast;
            for (int i = 1; i <= theChain.Length(); i++)
            {
                TopoDS_Edge anEdge    = TopoDS.Edge(theChain.Value(i));
                bool        isForward = (anEdge.Orientation() != TopAbs_REVERSED);

                if (anIsSeam && j == 2)
                { // second pass
                    anEdge.Reverse();
                }

                Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(anEdge, TopoDS.Face(aFaceSeq[j]), out aFirst, out aLast);
                if (aPCurve is null)
                {
                    continue;
                }

                if (aFaceSeq.Length() == 1)
                {
                    TopoDS_Edge aReversedEdge = anEdge;
                    aReversedEdge.Reverse();
                    Geom2d_Curve? aPCurve2 =
                      BRep_Tool.CurveOnSurface(aReversedEdge, TopoDS.Face(aFaceSeq[j]), out aFirst, out aLast);
                    if (!ReferenceEquals(aPCurve, aPCurve2))
                    {
                        anIsSeam = true;
                        aFaceSeq.Append(aFaceSeq[j]);
                    }
                }

                var               anAdaptor = new Geom2dAdaptor_Curve(aPCurve);
                GeomAbs_CurveType aType     = anAdaptor.GetType();

                Geom2d_Line? aLine = null;
                if (aType == GeomAbs_BSplineCurve || aType == GeomAbs_BezierCurve)
                {
                    TryMakeLine(aPCurve, aFirst, aLast, ref aLine);
                }
                if (aLine is not null)
                {
                    aPCurve = aLine;
                    anAdaptor.Load(aPCurve);
                    aType = GeomAbs_Line;
                }

                if (aPCurveSeq.IsEmpty())
                {
                    var aCopyPCurve = (Geom2d_Curve)aPCurve.Copy();
                    if (aCopyPCurve is Geom2d_TrimmedCurve aCopyTrimmed)
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
                    var aPrevAdaptor = new Geom2dAdaptor_Curve(aPCurveSeq.Last());
                    switch (aType)
                    {
                        case GeomAbs_Line:
                        {
                            gp_Lin2d aPrevLin  = aPrevAdaptor.Line();
                            gp_Pnt2d aFirstP2d = aPCurve.Value(aFirst);
                            gp_Pnt2d aLastP2d  = aPCurve.Value(aLast);
                            if (aPrevLin.Contains(aFirstP2d, Precision.Confusion())
                                && aPrevLin.Contains(aLastP2d, Precision.Confusion()))
                            {
                                isSameCurve = true;
                                gp_Pnt2d p1 = anAdaptor.Value(aFirst);
                                gp_Pnt2d p2 = anAdaptor.Value(aLast);
                                aNewF = ElCLib.Parameter(aPrevLin, p1);
                                aNewL = ElCLib.Parameter(aPrevLin, p2);
                                if (aNewF > aNewL)
                                {
                                    double aTmp = aNewF;
                                    aNewF       = aNewL;
                                    aNewL       = aTmp;
                                }
                            }
                            break;
                        }
                        case GeomAbs_Circle:
                        {
                            gp_Circ2d aCirc     = anAdaptor.Circle();
                            gp_Circ2d aPrevCirc = aPrevAdaptor.Circle();
                            if (aCirc.Location().Distance(aPrevCirc.Location()) <= Precision.Confusion()
                                && Math.Abs(aCirc.Radius() - aPrevCirc.Radius()) <= Precision.Confusion())
                            {
                                isSameCurve = true;
                                gp_Pnt2d p1 = anAdaptor.Value(aFirst);
                                gp_Pnt2d p2 = anAdaptor.Value(aLast);
                                aNewF = ElCLib.Parameter(aPrevCirc, p1);
                                aNewL = ElCLib.Parameter(aPrevCirc, p2);
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
                    var aCopyPCurve = (Geom2d_Curve)aPCurve.Copy();
                    if (aCopyPCurve is Geom2d_TrimmedCurve aCopyTrimmed)
                    {
                        aCopyPCurve = aCopyTrimmed.BasisCurve();
                    }

                    aPCurveSeq.Append(aCopyPCurve);
                    aFirstsSeq.Append(aFirst);
                    aLastsSeq.Append(aLast);
                    aForwardsSeq.Append(isForward);
                    aCurrentType = aType;
                    var aV = new TopoDS_Vertex();
                    TopExp.CommonVertex(aPrevEdge, anEdge, ref aV);
                    double aTol = BRep_Tool.Tolerance(aV);
                    aTolVerSeq.Append(aTol);
                }
                aPrevEdge = anEdge;
            }

            Geom2d_Curve aResPCurve;
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
                var tab_c2d = new NCollection_Array1<Geom2d_BSplineCurve>(0, aPCurveSeq.Length() - 1);
                for (int i = 1; i <= aPCurveSeq.Length(); i++)
                {
                    var aTrPCurve = new Geom2d_TrimmedCurve(aPCurveSeq[i], aFirstsSeq[i], aLastsSeq[i]);
                    if (!aForwardsSeq[i])
                    {
                        aTrPCurve.Reverse();
                    }
                    tab_c2d[i - 1] = Geom2dConvert.CurveToBSplineCurve(aTrPCurve);
                    Geom2dConvert.C0BSplineToC1BSplineCurve(ref tab_c2d[i - 1], Precision.Confusion());
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
                Geom2dConvert.ConcatC1(tab_c2d,
                                       tabtolvertex,
                                       out NCollection_HArray1<int> _,                     // array of the remaining Vertex
                                       out NCollection_HArray1<Geom2d_BSplineCurve> concatc2d, // array of the concatenated curves
                                       ref aClosedFlag,
                                       Precision.Confusion()); // C1 concatenation

                if (concatc2d.Length() > 1)
                {
                    var Concat2d = new Geom2dConvert_CompCurveToBSplineCurve(concatc2d.Value(concatc2d.Lower()));

                    for (int i = concatc2d.Lower() + 1; i <= concatc2d.Upper(); i++)
                    {
                        Concat2d.Add(concatc2d.Value(i), aMaxTol, true);
                    }

                    concatc2d.SetValue(concatc2d.Lower(), Concat2d.InternalBSplineCurve()!);
                }
                Geom2d_BSplineCurve aBSplineCurve = concatc2d.Value(concatc2d.Lower());
                aResPCurve = aBSplineCurve;
                aResFirst  = aBSplineCurve.FirstParameter();
                aResLast   = aBSplineCurve.LastParameter();
            }
            ResPCurves.Append(aResPCurve);
            ResFirsts.Append(aResFirst);
            ResLasts.Append(aResLast);
        }

        var aBuilder = new BRep_Builder();

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
                TopoDS_Face aFace     = TopoDS.Face(aFaceSeq[ii]);
                Geom_Surface? aSurf   = BRep_Tool.Surface(aFace);
                var         aSAS      = new ShapeAnalysis_Surface(aSurf);
                var         aToolProj = new ShapeConstruct_ProjectCurveOnSurface();
                aToolProj.Init(aSAS, Precision.Confusion());
                if (aToolProj.InternalPerform(aCurve, aFirst3d, aLast3d, out Geom2d_Curve? aNewPCurve))
                {
                    ResPCurves[ii] = aNewPCurve!;
                }
                else
                {
                    // Reparametrize pcurve
                    var aTrPCurve = new Geom2d_TrimmedCurve(ResPCurves[ii], ResFirsts[ii], ResLasts[ii]);
                    Geom2d_BSplineCurve aBSplinePCurve = owned(Geom2dConvert.CurveToBSplineCurve(aTrPCurve));
                    var aKnots = new NCollection_Array1<double>(aBSplinePCurve.InternalKnots());
                    BSplCLib.Reparametrize(aFirst3d, aLast3d, aKnots);
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
                    var               aGAcurve = new Geom2dAdaptor_Curve(ResPCurves[ii]);
                    GeomAbs_CurveType aType    = aGAcurve.GetType();
                    if (aType == GeomAbs_Line)
                    {
                        gp_Lin2d aLin2d   = aGAcurve.Line();
                        gp_Dir2d aDir2d   = aLin2d.Direction();
                        gp_Pnt2d aPnt2d   = aGAcurve.Value(ResFirsts[ii]);
                        gp_Vec2d anOffset = -new gp_Vec2d(aDir2d);
                        anOffset *= aFirst3d;
                        aPnt2d.Translate(anOffset);
                        var aNewLine2d = new Geom2d_Line(aPnt2d, aDir2d);
                        ResPCurves[ii] = aNewLine2d;
                    }
                    else if (aType == GeomAbs_Circle)
                    {
                        gp_Circ2d aCirc2d   = aGAcurve.Circle();
                        double    aRadius   = aCirc2d.Radius();
                        gp_Ax22d  aPosition = aCirc2d.Position();
                        gp_Pnt2d  aLocation = aCirc2d.Location();
                        double    anOffset  = ResFirsts[ii] - aFirst3d;
                        aPosition.Rotate(aLocation, anOffset);
                        var aNewCircle2d = new Geom2d_Circle(aPosition, aRadius);
                        ResPCurves[ii] = aNewCircle2d;
                    }
                    else // general case
                    {
                        var aTrPCurve = new Geom2d_TrimmedCurve(ResPCurves[ii], ResFirsts[ii], ResLasts[ii]);
                        Geom2d_BSplineCurve aBSplinePCurve = owned(Geom2dConvert.CurveToBSplineCurve(aTrPCurve));
                        var aKnots = new NCollection_Array1<double>(aBSplinePCurve.InternalKnots());
                        BSplCLib.Reparametrize(aFirst3d, aLast3d, aKnots);
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
            aBuilder.UpdateEdge(theEdge, ResPCurves[1], ResPCurves[2], TopoDS.Face(aFaceSeq[1]), aTolEdge);
        }
        else
        {
            for (int j = 1; j <= ResPCurves.Length(); j++)
            {
                aBuilder.UpdateEdge(theEdge, ResPCurves[j], TopoDS.Face(aFaceSeq[j]), aTolEdge);
            }
        }
    }

    /// <summary>Merges a sequence of edges into one edge if possible.</summary>
    protected bool MergeSubSeq(NCollection_Sequence<TopoDS_Shape>                                       theChain,
                               NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap,
                               ref TopoDS_Edge                                                          OutEdge)
    {
        var sae = new ShapeAnalysis_Edge();
        var B   = new BRep_Builder();
        // union edges in chain
        int         j;
        double      fp1, lp1;
        bool        IsUnionOfLinesPossible   = true;
        bool        IsUnionOfCirclesPossible = true;
        Geom_Curve? c3d1, c3d2;
        for (j = 1; j < theChain.Length(); j++)
        {
            TopoDS_Edge edge1 = TopoDS.Edge(theChain.Value(j));
            TopoDS_Edge edge2 = TopoDS.Edge(theChain.Value(j + 1));

            if (BRep_Tool.Degenerated(edge1) && BRep_Tool.Degenerated(edge2))
            {
                // Find the closest points in 2d
                TopoDS_Edge        edgeFirst    = TopoDS.Edge(theChain.First());
                TopoDS_Edge        edgeLast     = TopoDS.Edge(theChain.Last());
                var                CommonFace   = new TopoDS_Face();
                double             MinSqDist    = 0.0;
                TopAbs_Orientation OrOfE1OnFace = TopAbs_FORWARD, OrOfE2OnFace = TopAbs_FORWARD;
                int                IndOnE1      = 0, IndOnE2 = 0;
                Span<gp_Pnt2d>     PointsOnEdge1 = [new gp_Pnt2d(), new gp_Pnt2d()];
                Span<gp_Pnt2d>     PointsOnEdge2 = [new gp_Pnt2d(), new gp_Pnt2d()];
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
                gp_Pnt2d StartPoint = PointsOnEdge1[IndOnE1];
                gp_Pnt2d EndPoint   = PointsOnEdge2[IndOnE2];
                if ((OrOfE1OnFace == TopAbs_FORWARD && IndOnE1 == 1)
                    || (OrOfE1OnFace == TopAbs_REVERSED && IndOnE1 == 0))
                {
                    gp_Pnt2d Tmp = StartPoint;
                    StartPoint = EndPoint;
                    EndPoint   = Tmp;
                }

                Geom2d_Line aLine = new GC_MakeLine2d(StartPoint, EndPoint).Value();

                TopoDS_Vertex aVertex     = TopExp.FirstVertex(edgeFirst);
                TopoDS_Vertex StartVertex = aVertex, EndVertex = aVertex;
                StartVertex.Orientation(TopAbs_FORWARD);
                EndVertex.Orientation(TopAbs_REVERSED);

                var NewEdge = new TopoDS_Edge();
                B.MakeEdge(ref NewEdge);
                B.UpdateEdge(NewEdge, aLine, CommonFace, Precision.Confusion());
                B.Range(NewEdge, 0.0, StartPoint.Distance(EndPoint));
                B.Add(NewEdge, StartVertex);
                B.Add(NewEdge, EndVertex);
                B.Degenerated(NewEdge, true);
                OutEdge = NewEdge;
                return true;
            }

            c3d1 = BRep_Tool.Curve(edge1, out fp1, out lp1);
            c3d2 = BRep_Tool.Curve(edge2, out _, out _);

            var    aBAcurve1 = new BRepAdaptor_Curve(edge1);
            var    aBAcurve2 = new BRepAdaptor_Curve(edge2);
            gp_Dir aDir1 = new(), aDir2 = new();

            if (c3d1 is null || c3d2 is null)
            {
                return false;
            }

            if (c3d1 is Geom_TrimmedCurve tc1)
            {
                c3d1 = tc1.BasisCurve();
            }
            if (c3d2 is Geom_TrimmedCurve tc2)
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
            if (c3d1 is Geom_Circle C1 && c3d2 is Geom_Circle C2)
            {
                gp_Pnt P01 = C1.Location();
                gp_Pnt P02 = C2.Location();
                if (P01.Distance(P02) > Precision.Confusion())
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
            var V = new TopoDS_Vertex[2];
            V[0] = sae.FirstVertex(TopoDS.Edge(theChain.First()));
            gp_Pnt PV1 = BRep_Tool.Pnt(V[0]);
            V[1] = sae.LastVertex(TopoDS.Edge(theChain.Last()));
            gp_Pnt PV2 = BRep_Tool.Pnt(V[1]);
            var    Vec = new gp_Vec(PV1, PV2);
            if (mySafeInputMode)
            {
                for (int k = 0; k < 2; k++)
                {
                    if (!myContext.IsRecorded(V[k]))
                    {
                        TopoDS_Vertex Vcopy = TopoDS.Vertex(V[k].EmptyCopied());
                        myContext.Replace(V[k], Vcopy);
                        V[k] = Vcopy;
                    }
                    else
                    {
                        V[k] = TopoDS.Vertex(myContext.InternalApply(V[k]));
                    }
                }
            }
            var    L    = new Geom_Line(new gp_Ax1(PV1, new gp_Dir(Vec)));
            double dist = PV1.Distance(PV2);
            var    tc   = new Geom_TrimmedCurve(L, 0.0, dist);
            var    E    = new TopoDS_Edge();
            B.MakeEdge(ref E, tc, Precision.Confusion());
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
            TopoDS_Edge FE  = TopoDS.Edge(theChain.First());
            Geom_Curve  c3d = BRep_Tool.Curve(FE, out _, out _)!;

            if (c3d is Geom_TrimmedCurve tc)
            {
                c3d = tc.BasisCurve();
            }
            var Cir = (Geom_Circle)c3d;

            var V = new TopoDS_Vertex[2];
            V[0] = sae.FirstVertex(FE);
            V[1] = sae.LastVertex(TopoDS.Edge(theChain.Last()));
            bool isClosed = V[0].IsSame(V[1]);
            if (!isClosed)
            {
                // additionally check the points for equality to make a final decision about closedness of the
                // result curve
                gp_Pnt aP0  = BRep_Tool.Pnt(V[0]);
                gp_Pnt aP1  = BRep_Tool.Pnt(V[1]);
                double aTol = Max(BRep_Tool.Tolerance(V[0]), BRep_Tool.Tolerance(V[1]));
                if (aP0.SquareDistance(aP1) < aTol * aTol)
                {
                    isClosed = true;
                    V[1]     = V[0];
                    V[1].Reverse();
                }
            }
            var E = new TopoDS_Edge();
            if (isClosed)
            {
                // closed chain
                var          adef = new BRepAdaptor_Curve(FE);
                Geom_Circle  Cir1;
                double       FP, LP;
                if (FE.Orientation() == TopAbs_FORWARD)
                {
                    FP = adef.FirstParameter();
                    LP = adef.LastParameter();
                }
                else
                {
                    FP = adef.LastParameter();
                    LP = adef.FirstParameter();
                }
                if (Math.Abs(FP) < Precision.PConfusion())
                {
                    B.MakeEdge(ref E, Cir, Precision.Confusion());
                    B.Add(E, V[0]);
                    B.Add(E, V[1]);
                    E.Orientation(FE.Orientation());
                }
                else
                {
                    var MC1 = new GC_MakeCircle(adef.Value(FP), adef.Value((FP + LP) * 0.5), adef.Value(LP));
                    if (MC1.IsDone())
                    {
                        Cir1 = MC1.Value();
                    }
                    else
                    {
                        return false;
                    }
                    B.MakeEdge(ref E, Cir1, Precision.Confusion());
                    B.Add(E, V[0]);
                    B.Add(E, V[1]);
                }
            }
            else // open chain
            {
                double        ParamFirst     = BRep_Tool.Parameter(V[0], FE);
                TopoDS_Vertex VertexLastOnFE = sae.LastVertex(FE);
                double        ParamLast      = BRep_Tool.Parameter(VertexLastOnFE, FE);

                if (mySafeInputMode)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        if (!myContext.IsRecorded(V[k]))
                        {
                            TopoDS_Vertex Vcopy = TopoDS.Vertex(V[k].EmptyCopied());
                            myContext.Replace(V[k], Vcopy);
                            V[k] = Vcopy;
                        }
                        else
                        {
                            V[k] = TopoDS.Vertex(myContext.InternalApply(V[k]));
                        }
                    }
                }

                gp_Pnt PointFirst = BRep_Tool.Pnt(V[0]);
                while (Math.Abs(ParamLast - ParamFirst) > 7 * M_PI / 8)
                {
                    ParamLast = (ParamFirst + ParamLast) / 2;
                }
                var    BAcurveFE        = new BRepAdaptor_Curve(FE);
                gp_Pnt PointLast        = BAcurveFE.Value(ParamLast);
                gp_Pnt Origin           = Cir.Circ().Location();
                var    Dir1             = new gp_Dir(new gp_Vec(Origin, PointFirst));
                var    Dir2             = new gp_Dir(new gp_Vec(Origin, PointLast));
                gp_Dir Vdir             = Dir1 ^ Dir2;
                var    anAx2            = new gp_Ax2(Origin, Vdir, Dir1);
                var    aNewCircle       = new Geom_Circle(anAx2, Cir.Radius());
                gp_Pnt PointLastInChain = BRep_Tool.Pnt(V[1]);
                var    DirLastInChain   = new gp_Dir(new gp_Vec(Origin, PointLastInChain));
                double lpar             = Dir1.AngleWithRef(DirLastInChain, Vdir);
                if (lpar < 0.0)
                {
                    lpar += 2 * M_PI;
                }

                var tc1 = new Geom_TrimmedCurve(aNewCircle, 0.0, lpar);
                B.MakeEdge(ref E, tc1, Precision.Confusion());
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

            TopoDS_Vertex VF        = sae.FirstVertex(TopoDS.Edge(theChain.First()));
            TopoDS_Vertex VL        = sae.LastVertex(TopoDS.Edge(theChain.Last()));
            bool          NeedUnion = true;
            for (j = 1; j <= theChain.Length(); j++)
            {
                TopoDS_Edge edge = TopoDS.Edge(theChain.Value(j));
                Geom_Curve? c3d  = BRep_Tool.Curve(edge, out _, out fp1, out lp1);
                if (c3d is null)
                {
                    continue;
                }
                if (c3d is Geom_TrimmedCurve tc)
                {
                    c3d = tc.BasisCurve();
                }
                if (c3d is Geom_BSplineCurve || c3d is Geom_BezierCurve)
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
    private static bool IsMergingPossible(in TopoDS_Edge                                                         edge1,
                                          in TopoDS_Edge                                                         edge2,
                                          double                                                                 theAngTol,
                                          double                                                                 theLinTol,
                                          NCollection_Map<TopoDS_Shape>                                          AvoidEdgeVrt,
                                          bool                                                                   theLineDirectionOk,
                                          in gp_Pnt                                                              theFirstPoint,
                                          in gp_Vec                                                              theDirectionVec,
                                          NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap)
    {
        bool IsDegE1 = BRep_Tool.Degenerated(edge1);
        bool IsDegE2 = BRep_Tool.Degenerated(edge2);

        if (IsDegE1 && IsDegE2)
        {
            // Find connstion point in 2d
            var                CommonFace    = new TopoDS_Face();
            double             MinSqDist     = 0.0;
            TopAbs_Orientation OrOfE1OnFace  = TopAbs_FORWARD, OrOfE2OnFace = TopAbs_FORWARD;
            int                IndOnE1       = 0, IndOnE2 = 0;
            Span<gp_Pnt2d>     PointsOnEdge1 = [new gp_Pnt2d(), new gp_Pnt2d()];
            Span<gp_Pnt2d>     PointsOnEdge2 = [new gp_Pnt2d(), new gp_Pnt2d()];
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

            if (MinSqDist <= Precision.SquareConfusion())
            {
                return true;
            }

            return false;
        }
        else if (IsDegE1 || IsDegE2)
        {
            return false;
        }

        TopoDS_Vertex CV = TopExp.LastVertex(edge1, true);
        if (CV.IsNull() || AvoidEdgeVrt.Contains(CV))
        {
            return false;
        }

        var ade1 = new BRepAdaptor_Curve(edge1);
        var ade2 = new BRepAdaptor_Curve(edge2);

        GeomAbs_CurveType t1 = ade1.GetType();
        GeomAbs_CurveType t2 = ade2.GetType();

        if (t1 == GeomAbs_Circle && t2 == GeomAbs_Circle)
        {
            if (ade1.Circle().Location().Distance(ade2.Circle().Location()) > Precision.Confusion())
            {
                return false;
            }
        }

        gp_Dir aDir1 = new(), aDir2 = new();
        if (!(IsLinear(ade1, ref aDir1) && IsLinear(ade2, ref aDir2))
            && ((t1 != GeomAbs_BezierCurve && t1 != GeomAbs_BSplineCurve)
                || (t2 != GeomAbs_BezierCurve && t2 != GeomAbs_BSplineCurve))
            && t1 != t2)
        {
            return false;
        }

        gp_Vec Diff1, Diff2;
        gp_Pnt P1, P2;
        if (edge1.Orientation() == TopAbs_FORWARD)
        {
            ade1.D1(ade1.LastParameter(), out P1, out Diff1);
        }
        else
        {
            ade1.D1(ade1.FirstParameter(), out P1, out Diff1);
            Diff1 = -Diff1;
        }

        if (edge2.Orientation() == TopAbs_FORWARD)
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

        if (theLineDirectionOk && t2 == GeomAbs_Line)
        {
            // Check that the accumulated deflection does not exceed the linear tolerance
            double aLast = (edge2.Orientation() == TopAbs_FORWARD) ? ade2.LastParameter() : ade2.FirstParameter();
            var    aCurV = new gp_Vec(theFirstPoint, ade2.Value(aLast));
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

    private static bool GetLineEdgePoints(in TopoDS_Edge theInpEdge, ref gp_Pnt theFirstPoint, ref gp_Vec theDirectionVec)
    {
        Geom_Curve? aCur = BRep_Tool.Curve(theInpEdge, out double f, out double l);
        if (aCur is null)
        {
            return false;
        }

        if (aCur is Geom_TrimmedCurve aTC)
        {
            aCur = aTC.BasisCurve();
        }

        if (aCur.GetType() != typeof(Geom_Line))
        {
            return false;
        }

        if (theInpEdge.Orientation() == TopAbs_REVERSED)
        {
            double tmp = f;
            f = l;
            l = tmp;
        }
        theFirstPoint = aCur.Value(f);
        gp_Pnt aLP = aCur.Value(l);
        theDirectionVec = new gp_Vec(aLP.XYZ().Subtracted(theFirstPoint.XYZ()));
        theDirectionVec.Normalize();
        return true;
    }

    // Generates sub-sequences of edges from sequence of edges. Edges from each subsequences can be merged into the
    // one edge.
    private static void generateSubSeq(NCollection_Sequence<TopoDS_Shape>                                       anInpEdgeSeq,
                                       NCollection_Sequence<SubSequenceOfEdges>                                 SeqOfSubSeqOfEdges,
                                       bool                                                                     IsClosed,
                                       double                                                                   theAngTol,
                                       double                                                                   theLinTol,
                                       NCollection_Map<TopoDS_Shape>                                            AvoidEdgeVrt,
                                       NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap)
    {
        bool        isOk;
        TopoDS_Edge edge1, edge2;

        var         SubSeq  = new SubSequenceOfEdges();
        TopoDS_Edge RefEdge = TopoDS.Edge(anInpEdgeSeq[1]);
        SubSeq.SeqsEdges.Append(RefEdge);
        SeqOfSubSeqOfEdges.Append(SubSeq);

        var  aFirstPoint       = new gp_Pnt();
        var  aDirectionVec     = new gp_Vec();
        bool isLineDirectionOk = GetLineEdgePoints(RefEdge, ref aFirstPoint, ref aDirectionVec);

        for (int i = 1; i < anInpEdgeSeq.Length(); i++)
        {
            edge1 = TopoDS.Edge(anInpEdgeSeq[i]);
            edge2 = TopoDS.Edge(anInpEdgeSeq[i + 1]);
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
            edge1 = TopoDS.Edge(anInpEdgeSeq.Last());
            edge2 = TopoDS.Edge(anInpEdgeSeq.First());
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
    protected bool MergeEdges(NCollection_Sequence<TopoDS_Shape>                                       SeqEdges,
                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap,
                              NCollection_Sequence<SubSequenceOfEdges>                                 SeqOfSubSeqOfEdges,
                              NCollection_Map<TopoDS_Shape>                                            NonMergVrt)
    {
        var aMapVE          = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        int j;
        var VerticesToAvoid = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        int aNbE            = SeqEdges.Length();
        for (j = 1; j <= aNbE; j++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(SeqEdges[j]);
            // fill in the map V-E
            for (var it = new TopoDS_Iterator(anEdge.Oriented(TopAbs_FORWARD)); it.More(); it.Next())
            {
                TopoDS_Shape aV = it.Value();
                if (aV.Orientation() == TopAbs_FORWARD || aV.Orientation() == TopAbs_REVERSED)
                {
                    if (!aMapVE.Contains(aV))
                    {
                        aMapVE.Add(aV, new NCollection_List<TopoDS_Shape>());
                    }
                    aMapVE.ChangeFromKey(aV).Append(anEdge);
                }
            }
        }
        NCollection_MapAlgo.Unite(VerticesToAvoid, NonMergVrt);

        // do loop while there are unused edges
        var aUsedEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        for (int iE = 1; iE <= aNbE; ++iE)
        {
            TopoDS_Edge edge = TopoDS.Edge(SeqEdges[iE]);
            if (!aUsedEdges.Add(edge))
            {
                continue;
            }

            // make chain for unite
            var aChain = new NCollection_Sequence<TopoDS_Shape>();
            aChain.Append(edge);
            var V = new TopoDS_Vertex[2];
            TopExp.Vertices(edge, out V[0], out V[1], true);

            // connect more edges to the chain in both directions
            for (j = 0; j < 2; j++)
            {
                bool isAdded = true;
                while (isAdded)
                {
                    isAdded = false;
                    if (V[j].IsNull())
                    {
                        break;
                    }
                    NCollection_List<TopoDS_Shape> aLE = aMapVE.FindFromKey(V[j]);
                    for (var itL = new NCollection_List<TopoDS_Shape>.Iterator(aLE); itL.More(); itL.Next())
                    {
                        edge = TopoDS.Edge(itL.Value());
                        if (!aUsedEdges.Contains(edge))
                        {
                            var V2 = new TopoDS_Vertex[2];
                            TopExp.Vertices(edge, out V2[0], out V2[1], true);
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
            var UE = new TopoDS_Edge();
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
    protected bool MergeSeq(NCollection_Sequence<TopoDS_Shape>                                       SeqEdges,
                            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap,
                            NCollection_Map<TopoDS_Shape>                                            nonMergVert)
    {
        var SeqOfSubsSeqOfEdges = new NCollection_Sequence<SubSequenceOfEdges>();
        if (MergeEdges(SeqEdges, theVFmap, SeqOfSubsSeqOfEdges, nonMergVert))
        {
            for (int i = 1; i <= SeqOfSubsSeqOfEdges.Length(); i++)
            {
                if (SeqOfSubsSeqOfEdges[i].UnionEdges.IsNull())
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
