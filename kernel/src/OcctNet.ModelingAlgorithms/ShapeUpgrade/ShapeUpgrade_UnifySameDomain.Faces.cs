// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx (the file-static
// functions of the unification of faces; the class is in ShapeUpgrade_UnifySameDomain.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using DataMapOfFacePlane = Occt.NCollection_DataMap<Occt.TopoDS_Shape, Occt.Geom_Plane>;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

public partial class ShapeUpgrade_UnifySameDomain
{
    private static bool IsOnSingularity(NCollection_List<TopoDS_Shape> theEdgeList)
    {
        var anItl = new NCollection_List<TopoDS_Shape>.Iterator(theEdgeList);
        for (; anItl.More(); anItl.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anItl.Value());
            if (BRep_Tool.Degenerated(anEdge))
            {
                return true;
            }
        }

        return false;
    }

    // only for seam edges assumed to be U- or V- isolines
    private static bool IsUiso(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        var      aBAcurve2d = new BRepAdaptor_Curve2d(theEdge, theFace);
        gp_Pnt2d aP2d;
        gp_Vec2d aVec;
        aBAcurve2d.D1(aBAcurve2d.FirstParameter(), out aP2d, out aVec);
        return (Math.Abs(aVec.Y()) > Math.Abs(aVec.X()));
    }

    private static bool IsLinear(BRepAdaptor_Curve theBAcurve, ref gp_Dir theDir)
    {
        GeomAbs_CurveType aType = theBAcurve.GetType();

        if (aType == GeomAbs_CurveType.GeomAbs_Line)
        {
            theDir = theBAcurve.Line().Position().Direction();
            return true;
        }

        if ((aType == GeomAbs_CurveType.GeomAbs_BezierCurve || aType == GeomAbs_CurveType.GeomAbs_BSplineCurve)
            && theBAcurve.NbPoles() == 2)
        {
            gp_Pnt aFirstPnt = theBAcurve.Value(theBAcurve.FirstParameter());
            gp_Pnt aLastPnt  = theBAcurve.Value(theBAcurve.LastParameter());
            theDir           = new gp_Dir(new gp_Vec(aFirstPnt, aLastPnt));
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
    private static TopoDS_Face GetFaceFromSeq(
      NCollection_Sequence<TopoDS_Shape>                                       theFaces,
      in TopoDS_Face                                                           theRefFace,
      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF,
      NCollection_IndexedMap<TopoDS_Shape>                                     theUsedFaces,
      ref int                                                                  theIndFace)
    {
        if (theUsedFaces.IsEmpty())
        {
            theIndFace = 1;
            theUsedFaces.Add(theFaces[1]);
            return TopoDS.Face(theFaces[1]);
        }

        var aSeamEdge   = new TopoDS_Edge();
        var aTargetFace = new TopoDS_Face();
        for (int ii = 1; ii <= theUsedFaces.Extent(); ii++)
        {
            TopoDS_Face aUsedFace = TopoDS.Face(theUsedFaces[ii]);
            var         anExplo   = new TopExp_Explorer(aUsedFace, TopAbs_EDGE);
            for (; anExplo.More(); anExplo.Next())
            {
                TopoDS_Edge                    anEdge    = TopoDS.Edge(anExplo.Current());
                NCollection_List<TopoDS_Shape> aFaceList = theMapEF.FindFromKey(anEdge);
                if (aFaceList.Extent() < 2)
                {
                    continue;
                }

                TopoDS_Face aFace = (aFaceList.First().IsSame(aUsedFace))
                                      ? TopoDS.Face(aFaceList.Last())
                                      : TopoDS.Face(aFaceList.First());

                if (theUsedFaces.Contains(aFace))
                {
                    continue;
                }

                if (BRep_Tool.IsClosed(anEdge, theRefFace))
                {
                    aSeamEdge = anEdge;
                    continue;
                }

                aTargetFace = aFace;
                break;
            }
            if (aTargetFace.IsNull() && !aSeamEdge.IsNull())
            {
                NCollection_List<TopoDS_Shape> aFaceList = theMapEF.FindFromKey(aSeamEdge);
                aTargetFace = (aFaceList.First().IsSame(aUsedFace)) ? TopoDS.Face(aFaceList.Last())
                                                                    : TopoDS.Face(aFaceList.First());
            }
            if (!aTargetFace.IsNull())
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

    private static void UpdateBoundaries(Geom2d_Curve thePCurve,
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
            gp_Pnt2d aPoint = thePCurve.Value(aParam);

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

    private static bool TryMakeLine(Geom2d_Curve thePCurve, double theFirst, double theLast, ref Geom2d_Line? theLine)
    {
        gp_Pnt2d aFirstPnt   = thePCurve.Value(theFirst);
        gp_Pnt2d aLastPnt    = thePCurve.Value(theLast);
        var      aVec        = new gp_Vec2d(aFirstPnt, aLastPnt);
        double   aSqLen      = aVec.SquareMagnitude();
        double   aSqParamLen = (theLast - theFirst) * (theLast - theFirst);
        if (Math.Abs(aSqLen - aSqParamLen) > Precision.Confusion())
        {
            return false;
        }

        var      aDir     = new gp_Dir2d(aVec);
        gp_Vec2d anOffset = -new gp_Vec2d(aDir);
        anOffset *= theFirst;
        gp_Pnt2d anOrigin = aFirstPnt.Translated(anOffset);
        var      aLin     = new gp_Lin2d(anOrigin, aDir);

        const int NbSamples = 10;
        double    aDelta    = (theLast - theFirst) / NbSamples;
        for (int i = 1; i < NbSamples; i++)
        {
            double   aParam = theFirst + i * aDelta;
            gp_Pnt2d aPnt   = thePCurve.Value(aParam);
            double   aDist  = aLin.Distance(aPnt);
            if (aDist > Precision.Confusion())
            {
                return false;
            }
        }

        theLine = new Geom2d_Line(aLin);
        return true;
    }

    // Removes the specified edge from the vertex to edge map. Returns true if the edge was removed.
    private static bool RemoveEdgeFromMap(in TopoDS_Edge                                                         theEdge,
                                          NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVertexToEdges)
    {
        bool anIsRemoved = false;
        TopExp.Vertices(theEdge, out TopoDS_Vertex aFirstVertex, out TopoDS_Vertex aLastVertex);
        foreach (TopoDS_Vertex aVertex in (ReadOnlySpan<TopoDS_Vertex>)[aFirstVertex, aLastVertex])
        {
            if (!theVertexToEdges.Contains(aVertex))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> aVertexEdges = theVertexToEdges.ChangeFromKey(aVertex);
            var                            anEdgesIter  = new NCollection_List<TopoDS_Shape>.Iterator(aVertexEdges);
            while (anEdgesIter.More())
            {
                TopoDS_Shape anEdge = anEdgesIter.Value();
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

    private static double ComputeMinEdgeSize(NCollection_Sequence<TopoDS_Shape> theEdges,
                                             in TopoDS_Face                     theRefFace,
                                             NCollection_Map<TopoDS_Shape>      theEdgesMap)
    {
        double MinSize = RealLast();

        for (int ind = 1; ind <= theEdges.Length(); ind++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(theEdges[ind]);
            theEdgesMap.Add(anEdge);
            TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            var BAcurve2d = new BRepAdaptor_Curve2d(anEdge, theRefFace);
            if (BAcurve2d.Curve() is null)
            {
                continue;
            }

            gp_Pnt2d FirstP2d = BAcurve2d.Value(BAcurve2d.FirstParameter());
            gp_Pnt2d LastP2d  = BAcurve2d.Value(BAcurve2d.LastParameter());
            double   aSqDist;
            if (V1.IsSame(V2) && !BRep_Tool.Degenerated(anEdge))
            {
                gp_Pnt2d MidP2d = BAcurve2d.Value((BAcurve2d.FirstParameter() + BAcurve2d.LastParameter()) / 2);
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
    private static bool FindCoordBounds(NCollection_Sequence<TopoDS_Shape>                                       theFaces,
                                        in TopoDS_Face                                                           theRefFace,
                                        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF,
                                        NCollection_Map<TopoDS_Shape>                                            theEdgesMap,
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

        var aUsedFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (int ii = 1; ii <= theFaces.Length(); ii++)
        {
            int anIndFace = 0;
            // Get a face bordering with previous ones
            TopoDS_Face aFace     = GetFaceFromSeq(theFaces, theRefFace, theMapEF, aUsedFaces, ref anIndFace);
            double      aMinCoord = RealLast(), aMaxCoord = RealFirst();
            var         Explo     = new TopExp_Explorer(aFace, TopAbs_EDGE);
            for (; Explo.More(); Explo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());
                if (!theEdgesMap.Contains(anEdge))
                {
                    continue;
                }
                Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(anEdge, theRefFace, out double fpar, out double lpar);
                if (aPCurve is null)
                {
                    return false;
                }
                UpdateBoundaries(aPCurve, fpar, lpar, theIndCoord, ref aMinCoord, ref aMaxCoord);
            }

            if (Precision.IsInfinite(aMinCoord) || Precision.IsInfinite(aMaxCoord))
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
    private static (gp_Pnt2d, gp_Pnt2d) getCurveParams(in TopoDS_Edge theEdge, in TopoDS_Face theRefFace)
    {
        var    aCurveAdaptor = new BRepAdaptor_Curve2d(theEdge, theRefFace);
        double aFirstParam   = aCurveAdaptor.FirstParameter();
        double aLastParam    = aCurveAdaptor.LastParameter();
        if (theEdge.Orientation() != TopAbs_FORWARD)
        {
            (aFirstParam, aLastParam) = (aLastParam, aFirstParam);
        }

        gp_Pnt2d aFirstPoint = aCurveAdaptor.Value(aFirstParam);
        gp_Pnt2d aLastPoint  = aCurveAdaptor.Value(aLastParam);
        return (aFirstPoint, aLastPoint);
    }

    private static bool RelocatePCurvesToNewUorigin(
      NCollection_Sequence<TopoDS_Shape>                                       theEdges,
      in TopoDS_Shape                                                          theFirstFace,
      in TopoDS_Face                                                           theRefFace,
      double                                                                   theCoordTol,
      int                                                                      theIndCoord,
      double                                                                   thePeriod,
      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVEmap,
      NCollection_DataMap<TopoDS_Shape, Geom2d_Curve>                          theEdgeNewPCurve,
      NCollection_Map<TopoDS_Shape>                                            theUsedEdges)
    {
        var anEdgesOfFirstFace = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theFirstFace, anEdgesOfFirstFace);

        for (;;) // walk by contours
        {
            // try to find the start edge that:
            // 1. belongs to outer edges of first face;
            // 2. has minimum U-coord of its start point
            var                aStartEdge    = new TopoDS_Edge();
            TopAbs_Orientation anOrientation = TopAbs_FORWARD;
            double             aCoordMin     = RealLast();
            for (int anEdgeIndex = 1; anEdgeIndex <= theEdges.Length(); ++anEdgeIndex)
            {
                TopoDS_Edge anEdge = TopoDS.Edge(theEdges[anEdgeIndex]);
                if (theUsedEdges.Contains(anEdge))
                {
                    continue;
                }

                if (!anEdgesOfFirstFace.Contains(anEdge))
                {
                    continue;
                }

                if (aStartEdge.IsNull())
                {
                    aStartEdge = anEdge;
                    (gp_Pnt2d aFirstPoint, gp_Pnt2d aLastPoint) = getCurveParams(anEdge, theRefFace);
                    if (aFirstPoint.Coord(theIndCoord) < aLastPoint.Coord(theIndCoord))
                    {
                        aCoordMin     = aFirstPoint.Coord(theIndCoord);
                        anOrientation = TopAbs_FORWARD;
                    }
                    else
                    {
                        aCoordMin     = aLastPoint.Coord(theIndCoord);
                        anOrientation = TopAbs_REVERSED;
                    }
                }
                else
                {
                    (gp_Pnt2d aFirstPoint, gp_Pnt2d aLastPoint) = getCurveParams(anEdge, theRefFace);
                    if (aFirstPoint.Coord(theIndCoord) < aCoordMin)
                    {
                        aStartEdge    = anEdge;
                        aCoordMin     = aFirstPoint.Coord(theIndCoord);
                        anOrientation = TopAbs_FORWARD;
                    }
                    if (aLastPoint.Coord(theIndCoord) < aCoordMin)
                    {
                        aStartEdge    = anEdge;
                        aCoordMin     = aLastPoint.Coord(theIndCoord);
                        anOrientation = TopAbs_REVERSED;
                    }
                }
            }

            if (aStartEdge.IsNull()) // all contours are passed
            {
                break;
            }

            TopoDS_Edge   aCurrentEdge = aStartEdge;
            Geom2d_Curve? CurPCurve =
              BRep_Tool.CurveOnSurface(aCurrentEdge, theRefFace, out double anEdgeStartParam, out double anEdgeEndParam);
            if (CurPCurve is null)
            {
                return false;
            }
            CurPCurve = new Geom2d_TrimmedCurve(CurPCurve, anEdgeStartParam, anEdgeEndParam);
            theEdgeNewPCurve.Bind(aCurrentEdge, CurPCurve);
            if (aCurrentEdge.Orientation() == TopAbs_REVERSED)
            {
                (anEdgeStartParam, anEdgeEndParam) = (anEdgeEndParam, anEdgeStartParam);
            }
            double   CurParam = (anOrientation == TopAbs_FORWARD) ? anEdgeEndParam : anEdgeStartParam;
            gp_Pnt2d CurPoint = CurPCurve.Value(CurParam);

            for (;;) // collect pcurves of a contour
            {
                if (!RemoveEdgeFromMap(aCurrentEdge, theVEmap))
                {
                    break; // end of contour in 2d
                }
                theUsedEdges.Add(aCurrentEdge);
                TopoDS_Vertex CurVertex = (anOrientation == TopAbs_FORWARD)
                                            ? TopExp.LastVertex(aCurrentEdge, true)
                                            : TopExp.FirstVertex(aCurrentEdge, true);

                NCollection_List<TopoDS_Shape> Elist = theVEmap.FindFromKey(CurVertex);
                if (Elist.IsEmpty())
                {
                    break; // end of contour in 3d
                }

                for (var itl = new NCollection_List<TopoDS_Shape>.Iterator(Elist); itl.More(); itl.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
                    if (anEdge.IsSame(aCurrentEdge))
                    {
                        continue;
                    }

                    TopoDS_Vertex aFirstVertex = (anOrientation == TopAbs_FORWARD)
                                                   ? TopExp.FirstVertex(anEdge, true)
                                                   : TopExp.LastVertex(anEdge, true);
                    if (!aFirstVertex.IsSame(CurVertex)) // may be if CurVertex is deg.vertex
                    {
                        continue;
                    }

                    Geom2d_Curve? aPCurve =
                      BRep_Tool.CurveOnSurface(anEdge, theRefFace, out anEdgeStartParam, out anEdgeEndParam);
                    if (aPCurve is null)
                    {
                        return false;
                    }
                    aPCurve = new Geom2d_TrimmedCurve(aPCurve, anEdgeStartParam, anEdgeEndParam);
                    if (anEdge.Orientation() == TopAbs_REVERSED)
                    {
                        (anEdgeStartParam, anEdgeEndParam) = (anEdgeEndParam, anEdgeStartParam);
                    }
                    double   aParam   = (anOrientation == TopAbs_FORWARD) ? anEdgeStartParam : anEdgeEndParam;
                    gp_Pnt2d aPoint   = aPCurve.Value(aParam);
                    double   anOffset = CurPoint.Coord(theIndCoord) - aPoint.Coord(theIndCoord);
                    if (Math.Abs(anOffset) >= theCoordTol && Math.Abs(Math.Abs(anOffset) - thePeriod) >= theCoordTol)
                    {
                        continue; // may be if CurVertex is deg.vertex
                    }

                    if (Math.Abs(anOffset) > thePeriod / 2)
                    {
                        anOffset = TrueValueOfOffset(anOffset, thePeriod);
                        var aVec = new gp_Vec2d();
                        if (theIndCoord == 1)
                        {
                            aVec.SetCoord(anOffset, 0.0);
                        }
                        else
                        {
                            aVec.SetCoord(0.0, anOffset);
                        }
                        var aNewPCurve = (Geom2d_Curve)aPCurve.Copy();
                        aNewPCurve.Translate(aVec);
                        aPCurve = aNewPCurve;
                    }
                    theEdgeNewPCurve.Bind(anEdge, aPCurve);
                    aCurrentEdge = anEdge;
                    TopAbs_Orientation CurOr = TopAbs.Compose(anOrientation, aCurrentEdge.Orientation());
                    CurParam = (CurOr == TopAbs_FORWARD) ? aPCurve.LastParameter() : aPCurve.FirstParameter();
                    CurPoint = aPCurve.Value(CurParam);
                    break;
                }
            } // for (;;) (collect pcurves of a contour)
        } // for (;;) (walk by contours)
        return true;
    }

    private static void InsertWiresIntoFaces(NCollection_Sequence<TopoDS_Shape> theWires,
                                             NCollection_Sequence<TopoDS_Shape> theFaces,
                                             in TopoDS_Face                     theRefFace)
    {
        var BB = new BRep_Builder();
        for (int ii = 1; ii <= theWires.Length(); ii++)
        {
            TopoDS_Wire aWire     = TopoDS.Wire(theWires[ii]);
            var         iter      = new TopoDS_Iterator(aWire);
            TopoDS_Edge anEdge    = TopoDS.Edge(iter.Value());
            var         BAcurve2d = new BRepAdaptor_Curve2d(anEdge, theRefFace);
            gp_Pnt2d    aPnt2d    = BAcurve2d.Value((BAcurve2d.FirstParameter() + BAcurve2d.LastParameter()) / 2.0);
            var         RequiredFace = new TopoDS_Shape();
            for (int jj = 1; jj <= theFaces.Length(); jj++)
            {
                TopoDS_Face   aFace      = TopoDS.Face(theFaces[jj]);
                var           Classifier = new BRepTopAdaptor_FClass2d(aFace, Precision.Confusion());
                TopAbs_State  aStatus    = Classifier.Perform(aPnt2d);
                if (aStatus == TopAbs_IN)
                {
                    RequiredFace = aFace.Oriented(TopAbs_FORWARD);
                    break;
                }
            }
            if (!RequiredFace.IsNull())
            {
                BB.Add(RequiredFace, aWire);
            }
            else
            {
                // Standard_ASSERT_INVOKE("ShapeUpgrade_UnifySameDomain: wire remains unclassified"): a debug report
            }
        }
    }

    private static TopoDS_Face FindCommonFace(in TopoDS_Edge                                                         theEdge1,
                                              in TopoDS_Edge                                                         theEdge2,
                                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap,
                                              ref TopAbs_Orientation                                                 theOrOfE1OnFace,
                                              ref TopAbs_Orientation                                                 theOrOfE2OnFace)
    {
        var aVertex = new TopoDS_Vertex();
        TopExp.CommonVertex(theEdge1, theEdge2, ref aVertex);
        NCollection_List<TopoDS_Shape> Flist = theVFmap.FindFromKey(aVertex);
        var                            itl   = new NCollection_List<TopoDS_Shape>.Iterator(Flist);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(itl.Value());
            aFace.Orientation(TopAbs_FORWARD);
            bool e1found = false, e2found = false;
            var  Explo   = new TopExp_Explorer(aFace, TopAbs_EDGE);
            for (; Explo.More(); Explo.Next())
            {
                TopoDS_Shape anEdge = Explo.Current();
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

        var NullFace = new TopoDS_Face();
        return NullFace;
    }

    private static bool FindClosestPoints(in TopoDS_Edge                                                         theEdge1,
                                          in TopoDS_Edge                                                         theEdge2,
                                          NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVFmap,
                                          ref TopoDS_Face                                                        theCommonFace,
                                          ref double                                                             theMinSqDist,
                                          ref TopAbs_Orientation                                                 OrOfE1OnFace,
                                          ref TopAbs_Orientation                                                 OrOfE2OnFace,
                                          ref int                                                                theIndOnE1,
                                          ref int                                                                theIndOnE2,
                                          Span<gp_Pnt2d>                                                         PointsOnEdge1,
                                          Span<gp_Pnt2d>                                                         PointsOnEdge2)
    {
        theCommonFace = FindCommonFace(theEdge1, theEdge2, theVFmap, ref OrOfE1OnFace, ref OrOfE2OnFace);
        if (theCommonFace.IsNull())
        {
            return false;
        }

        Geom2d_Curve? PCurve1 = BRep_Tool.CurveOnSurface(theEdge1, theCommonFace, out double fpar1, out double lpar1);
        Geom2d_Curve? PCurve2 = BRep_Tool.CurveOnSurface(theEdge2, theCommonFace, out double fpar2, out double lpar2);
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

    private static void ReconstructMissedSeam(NCollection_Sequence<TopoDS_Shape> theRemovedEdges,
                                              in TopoDS_Face                     theFrefFace,
                                              in TopoDS_Edge                     theCurEdge,
                                              in TopoDS_Vertex                   theCurVertex,
                                              in gp_Pnt2d                        theCurPoint,
                                              double                             theUperiod,
                                              double                             theVperiod,
                                              ref TopoDS_Edge                    theSeamEdge,
                                              ref gp_Pnt2d                       theNextPoint)
    {
        _ = BRep_Tool.Surface(theFrefFace); // RefSurf: unused, as in OCCT

        // Find seam edge between removed edges
        theSeamEdge.Nullify();
        for (int i = 1; i <= theRemovedEdges.Length(); i++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(theRemovedEdges[i]);
            if (anEdge.IsSame(theCurEdge))
            {
                continue;
            }
            Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anEdge, theFrefFace, out double Param1, out double Param2);
            if (aPC is null)
            {
                continue;
            }

            TopExp.Vertices(anEdge, out TopoDS_Vertex aFirstVertex, out TopoDS_Vertex aLastVertex, true);

            if ((aFirstVertex.IsSame(theCurVertex) || aLastVertex.IsSame(theCurVertex))
                && BRep_Tool.IsClosed(anEdge, theFrefFace))
            {
                double   aParam = (anEdge.Orientation() == TopAbs_FORWARD) ? Param1 : Param2;
                gp_Pnt2d aPoint = aPC.Value(aParam);
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
                        TopAbs_Orientation anOri = anEdge.Orientation();
                        anEdge.Orientation(TopAbs_FORWARD);
                        Geom2d_Curve? aPC1 = BRep_Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2);
                        anEdge.Reverse();
                        Geom2d_Curve? aPC2 = BRep_Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2);
                        anEdge.Reverse(); // again FORWARD
                        var           aBB   = new BRep_Builder();
                        double        aTol  = BRep_Tool.Tolerance(anEdge);
                        Geom_Surface? aSurf = BRep_Tool.Surface(theFrefFace, out TopLoc_Location aLoc);
                        aBB.UpdateEdge(anEdge, aPC2, aPC1, aSurf, aLoc, aTol);
                        anEdge.Orientation(anOri);
                    }
                    aPC    = BRep_Tool.CurveOnSurface(anEdge, theFrefFace, out Param1, out Param2)!;
                    aParam = (anEdge.Orientation() == TopAbs_FORWARD) ? Param1 : Param2;
                    aPoint = aPC.Value(aParam);
                    aUdiff = Math.Abs(aPoint.X() - theCurPoint.X());
                    aVdiff = Math.Abs(aPoint.Y() - theCurPoint.Y());
                }
                if ((theUperiod == 0.0 || aUdiff < theUperiod / 2) && (theVperiod == 0.0 || aVdiff < theVperiod / 2))
                {
                    aFirstVertex = TopExp.FirstVertex(anEdge, true);
                    if (aFirstVertex.IsSame(theCurVertex))
                    {
                        theSeamEdge = anEdge;
                        break;
                    }
                }
            }
        }

        if (!theSeamEdge.IsNull())
        {
            Geom2d_Curve aPC    = BRep_Tool.CurveOnSurface(theSeamEdge, theFrefFace, out double Param1, out double Param2)!;
            double       aParam = (theSeamEdge.Orientation() == TopAbs_FORWARD) ? Param2 : Param1;
            theNextPoint        = aPC.Value(aParam);
        }
    }

    private static readonly double[] s_SameSurfCoefs = [0.3399811, 0.7745966];

    private static bool SameSurf(Geom_Surface theS1, Geom_Surface theS2)
    {
        double[] aCoefs = s_SameSurfCoefs;

        theS1.Bounds(out double uf1, out double ul1, out double vf1, out double vl1);
        theS2.Bounds(out double uf2, out double ul2, out double vf2, out double vl2);
        double aPTol = Precision.PConfusion();
        if (Precision.IsNegativeInfinite(uf1))
        {
            if (!Precision.IsNegativeInfinite(uf2))
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
            if (Precision.IsNegativeInfinite(uf2))
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
        if (Precision.IsNegativeInfinite(vf1))
        {
            if (!Precision.IsNegativeInfinite(vf2))
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
            if (Precision.IsNegativeInfinite(vf2))
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
        if (Precision.IsPositiveInfinite(ul1))
        {
            if (!Precision.IsPositiveInfinite(ul2))
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
            if (Precision.IsPositiveInfinite(ul2))
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
        if (Precision.IsPositiveInfinite(vl1))
        {
            if (!Precision.IsPositiveInfinite(vl2))
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
            if (Precision.IsPositiveInfinite(vl2))
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
                gp_Pnt aP1 = theS1.Value(u, v);
                gp_Pnt aP2 = theS2.Value(u, v);
                if (!aP1.IsEqual(aP2, aPTol))
                {
                    return false;
                }
            }
        }

        return true;
    }

    private static void TransformPCurves(in TopoDS_Face                theRefFace,
                                         in TopoDS_Face                theFace,
                                         NCollection_Map<TopoDS_Shape> theMapEdgesWithTemporaryPCurves)
    {
        Geom_Surface RefSurf = BRep_Tool.Surface(theRefFace)!;
        if (RefSurf is Geom_RectangularTrimmedSurface aRefRTS)
        {
            RefSurf = aRefRTS.BasisSurface();
        }

        Geom_Surface SurfFace = BRep_Tool.Surface(theFace)!;
        if (SurfFace is Geom_RectangularTrimmedSurface aFaceRTS)
        {
            SurfFace = aFaceRTS.BasisSurface();
        }

        bool ToModify = false, ToTranslate = false, ToRotate = false, X_Reverse = false, Y_Reverse = false,
             ToProject = false;

        double aTranslation = 0.0, anAngle = 0.0;

        // Get axes of surface of face and of surface of RefFace
        var ElemSurfFace = SurfFace as Geom_ElementarySurface;
        var ElemRefSurf  = RefSurf as Geom_ElementarySurface;

        if (ElemSurfFace is not null && ElemRefSurf is not null)
        {
            gp_Ax3 AxisOfSurfFace = ElemSurfFace.Position();
            gp_Ax3 AxisOfRefSurf  = ElemRefSurf.Position();

            gp_Pnt OriginRefSurf = AxisOfRefSurf.Location();

            double aParam = ElCLib.LineParameter(AxisOfSurfFace.Axis(), OriginRefSurf);

            if (Math.Abs(aParam) > Precision.PConfusion())
            {
                aTranslation = -aParam;
            }

            gp_Dir VdirSurfFace = AxisOfSurfFace.Direction();
            gp_Dir VdirRefSurf  = AxisOfRefSurf.Direction();
            gp_Dir XdirSurfFace = AxisOfSurfFace.XDirection();
            gp_Dir XdirRefSurf  = AxisOfRefSurf.XDirection();

            gp_Dir CrossProd1 = AxisOfRefSurf.XDirection() ^ AxisOfRefSurf.YDirection();
            gp_Dir CrossProd2 = AxisOfSurfFace.XDirection() ^ AxisOfSurfFace.YDirection();
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
                gp_Dir DirRef = VdirRefSurf;
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

            ToRotate = (Math.Abs(anAngle) > Precision.PConfusion());

            ToTranslate = (Math.Abs(aTranslation) > Precision.PConfusion());

            ToModify = ToTranslate || ToRotate || X_Reverse || Y_Reverse;
        }
        else
        {
            if (!SameSurf(RefSurf, SurfFace))
            {
                ToProject = true;
            }
        }

        var BB    = new BRep_Builder();
        var Explo = new TopExp_Explorer(theFace, TopAbs_EDGE);
        var aEmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());
            if (BRep_Tool.Degenerated(anEdge) && ToModify)
            {
                continue;
            }

            if (ToProject && BRep_Tool.IsClosed(anEdge, theFace))
            {
                continue;
            }

            if (!aEmap.Add(anEdge))
            {
                continue;
            }

            TopAbs_Orientation anOr = anEdge.Orientation();

            double        fpar, lpar;
            Geom2d_Curve? PCurveOnRef = BRep_Tool.CurveOnSurface(anEdge, theRefFace, out fpar, out lpar);
            Geom2d_Curve? PCurve2     = null;
            if (PCurveOnRef is not null /* && !(ToModify || ToProject)*/)
            {
                anEdge.Reverse();
                PCurve2 = BRep_Tool.CurveOnSurface(anEdge, theRefFace, out fpar, out lpar);
                if (!ReferenceEquals(PCurve2, PCurveOnRef))
                { // two pcurves
                    continue;
                }
            }

            var PCurves    = new Geom2d_Curve?[2];
            var NewPCurves = new Geom2d_Curve?[2];
            anEdge.Orientation(TopAbs_FORWARD);
            PCurves[0] = BRep_Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);
            anEdge.Reverse();
            PCurves[1] = BRep_Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);

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
                    Geom_Curve? aC3d = BRep_Tool.Curve(anEdge, out fpar, out lpar);
                    aC3d = new Geom_TrimmedCurve(aC3d!, fpar, lpar);
                    double tol = BRep_Tool.Tolerance(anEdge);
                    tol = Min(tol, Precision.Approximation());
                    // (ToProject excludes the transformations below, so the projection is never edited)
                    NewPCurves[ii] = GeomProjLib.Curve2d(aC3d, RefSurf);
                }
                else
                {
                    NewPCurves[ii] = (Geom2d_Curve)PCurves[ii]!.Copy();
                }
                if (ToTranslate)
                {
                    NewPCurves[ii]!.Translate(new gp_Vec2d(0.0, aTranslation));
                }
                if (Y_Reverse)
                {
                    NewPCurves[ii]!.Mirror(gp.OX2d());
                }
                if (X_Reverse)
                {
                    NewPCurves[ii]!.Mirror(gp.OY2d());
                    NewPCurves[ii]!.Translate(new gp_Vec2d(2 * M_PI, 0.0));
                }
                if (ToRotate)
                {
                    NewPCurves[ii]!.Translate(new gp_Vec2d(anAngle, 0.0));
                }
            }

            anEdge.Orientation(TopAbs_FORWARD);

            if (NbPcurves == 1)
            {
                if (PCurve2 is null || (!RefSurf.IsUClosed() && !RefSurf.IsVClosed()))
                {
                    BB.UpdateEdge(anEdge, NewPCurves[0], theRefFace, 0.0);
                    theMapEdgesWithTemporaryPCurves.Add(anEdge);
                }
                else
                {
                    // check: may be it is the same pcurve
                    RefSurf.Bounds(out double aUmin, out double aUmax, out double aVmin, out double aVmax);
                    double   aUperiod      = (RefSurf.IsUClosed()) ? (aUmax - aUmin) : 0.0;
                    double   aVperiod      = (RefSurf.IsVClosed()) ? (aVmax - aVmin) : 0.0;
                    gp_Pnt2d aP2dOnPCurve1 = PCurveOnRef!.Value(fpar);
                    gp_Pnt2d aP2dOnPCurve2 = NewPCurves[0]!.Value(fpar);
                    if ((aUperiod != 0.0 && Math.Abs(aP2dOnPCurve1.X() - aP2dOnPCurve2.X()) > aUperiod / 2)
                        || (aVperiod != 0.0 && Math.Abs(aP2dOnPCurve1.Y() - aP2dOnPCurve2.Y()) > aVperiod / 2))
                    {
                        Geom2d_Curve? NullPCurve = null;
                        BB.UpdateEdge(anEdge, NullPCurve, theRefFace, 0.0);
                        if (anOr == TopAbs_FORWARD)
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

    private static void AddPCurves(NCollection_Sequence<TopoDS_Shape> theFaces,
                                   in TopoDS_Face                     theRefFace,
                                   NCollection_Map<TopoDS_Shape>      theMapEdgesWithTemporaryPCurves)
    {
        var RefBAsurf = new BRepAdaptor_Surface(theRefFace, false);

        GeomAbs_SurfaceType aType = RefBAsurf.GetType();
        if (aType == GeomAbs_Plane)
        {
            return;
        }

        for (int i = 1; i <= theFaces.Length(); i++)
        {
            TopoDS_Face aFace = TopoDS.Face(theFaces[i]);
            aFace.Orientation(TopAbs_FORWARD);
            if (aFace.IsSame(theRefFace))
            {
                continue;
            }

            TransformPCurves(theRefFace, aFace, theMapEdgesWithTemporaryPCurves);
        }
    }

    // Adds edges from the shape to the sequence; seams and equal edges are dropped. Returns true if one of original
    // edges dropped.
    private static bool AddOrdinaryEdges(NCollection_Sequence<TopoDS_Shape> edges,
                                         in TopoDS_Shape                    aShape,
                                         ref int                            anIndex,
                                         NCollection_Sequence<TopoDS_Shape> theRemovedEdges)
    {
        // map of edges
        var aNewEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        // add edges without seams
        for (var exp = new TopExp_Explorer(aShape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Shape edge = exp.Current();
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
            TopoDS_Shape current = edges[i];
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

    private static bool getCylinder(Geom_Surface theInSurface, ref gp_Cylinder theOutCylinder)
    {
        bool isCylinder = false;

        if (theInSurface is Geom_CylindricalSurface aGC)
        {
            theOutCylinder = aGC.Cylinder();
            isCylinder     = true;
        }
        else if (theInSurface is Geom_SurfaceOfRevolution aRS)
        {
            Geom_Curve aBasis = aRS.BasisCurve();

            while (aBasis is Geom_TrimmedCurve aTc)
            {
                aBasis = aTc.BasisCurve();
            }

            if (aBasis is Geom_Line aBasisLine)
            {
                gp_Dir aDir      = aRS.Direction();
                gp_Dir aBasisDir = aBasisLine.Position().Direction();
                if (aBasisDir.IsParallel(aDir, Precision.Angular()))
                {
                    // basis line is parallel to the revolution axis: it is a cylinder
                    gp_Pnt aLoc   = aRS.Location();
                    double aR     = aBasisLine.Lin().Distance(aLoc);
                    var    aCylAx = new gp_Ax3(aLoc, aDir);

                    theOutCylinder = new gp_Cylinder(aCylAx, aR);
                    isCylinder     = true;
                }
            }
        }
        else if (theInSurface is Geom_SurfaceOfLinearExtrusion aLES)
        {
            Geom_Curve aBasis = aLES.BasisCurve();

            while (aBasis is Geom_TrimmedCurve aTc)
            {
                aBasis = aTc.BasisCurve();
            }

            if (aBasis is Geom_Circle aBasisCircle)
            {
                gp_Dir aDir      = aLES.Direction();
                gp_Dir aBasisDir = aBasisCircle.Position().Direction();
                if (aBasisDir.IsParallel(aDir, Precision.Angular()))
                {
                    // basis circle is normal to the extrusion axis: it is a cylinder
                    gp_Pnt aLoc   = aBasisCircle.Location();
                    double aR     = aBasisCircle.Radius();
                    var    aCylAx = new gp_Ax3(aLoc, aDir);

                    theOutCylinder = new gp_Cylinder(aCylAx, aR);
                    isCylinder     = true;
                }
            }
        }
        else
        {
        }

        return isCylinder;
    }

    private static Geom_Surface? ClearRts(Geom_Surface? aSurface)
    {
        var aRTS = aSurface as Geom_RectangularTrimmedSurface;
        return aRTS is null ? aSurface : aRTS.BasisSurface();
    }

    // Gets the normal to surface by the given parameter on edge. Returns True if normal was computed.
    private static bool GetNormalToSurface(in TopoDS_Face theFace, in TopoDS_Edge theEdge, double theP, ref gp_Dir theNormal)
    {
        double f, l;
        // get 2d curve to get point in 2d
        Geom2d_Curve? aC2d;
        if (BRep_Tool.IsClosed(theEdge, theFace))
        {
            // Find the edge in the face: it will have correct orientation
            var         anEdgeInFace = new TopoDS_Edge();
            TopoDS_Face aFace        = theFace;
            aFace.Orientation(TopAbs_FORWARD);
            var anExplo = new TopExp_Explorer(aFace, TopAbs_EDGE);
            for (; anExplo.More(); anExplo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace = anEdge;
                    break;
                }
            }
            if (anEdgeInFace.IsNull())
            {
                return false;
            }

            aC2d = BRep_Tool.CurveOnSurface(anEdgeInFace, aFace, out f, out l);
        }
        else
        {
            aC2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out f, out l);
        }

        if (aC2d is null)
        {
            return false;
        }
        //
        // 2d point
        aC2d.D0(theP, out gp_Pnt2d aP2d);
        //
        // get D1
        Geom_Surface aS = BRep_Tool.Surface(theFace, out TopLoc_Location aLoc)!;
        aS.D1(aP2d.X(), aP2d.Y(), out gp_Pnt aP3d, out gp_Vec aDU, out gp_Vec aDV);
        //
        // compute normal
        gp_Vec aVNormal = aDU.Crossed(aDV);
        if (aVNormal.Magnitude() < Precision.Confusion())
        {
            return false;
        }
        //
        if (theFace.Orientation() == TopAbs_REVERSED)
        {
            aVNormal.Reverse();
        }
        //
        aVNormal.Transform(aLoc.Transformation());
        theNormal = new gp_Dir(aVNormal);
        return true;
    }

    private static bool IsSameDomain(in TopoDS_Face        aFace,
                                     in TopoDS_Face        aCheckedFace,
                                     double                theLinTol,
                                     double                theAngTol,
                                     DataMapOfFacePlane    theFacePlaneMap)
    {
        // checking the same handles
        Geom_Surface? S1, S2;

        S1 = BRep_Tool.Surface(aFace, out TopLoc_Location L1);
        S2 = BRep_Tool.Surface(aCheckedFace, out TopLoc_Location L2);

        if (ReferenceEquals(S1, S2) && L1 == L2)
        {
            return true;
        }

        S1 = BRep_Tool.Surface(aFace);
        S2 = BRep_Tool.Surface(aCheckedFace);

        S1 = ClearRts(S1)!;
        S2 = ClearRts(S2)!;

        // case of two planar surfaces:
        // all kinds of surfaces checked, including b-spline and bezier
        var aPlanarityChecker1 = new GeomLib_IsPlanarSurface(S1, theLinTol);
        if (aPlanarityChecker1.IsPlanar())
        {
            var aPlanarityChecker2 = new GeomLib_IsPlanarSurface(S2, theLinTol);
            if (aPlanarityChecker2.IsPlanar())
            {
                gp_Pln aPln1 = aPlanarityChecker1.Plan();
                gp_Pln aPln2 = aPlanarityChecker2.Plan();

                if (aPln1.Position().Direction().IsParallel(aPln2.Position().Direction(), theAngTol)
                    && aPln1.Distance(aPln2) < theLinTol)
                {
                    Geom_Plane aPlaneOfFaces;
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
                        aPlaneOfFaces = new Geom_Plane(aPln1);
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
        if (S1 is Geom_ElementarySurface && S2 is Geom_ElementarySurface)
        {
            var aGA1 = new GeomAdaptor_Surface(S1);
            var aGA2 = new GeomAdaptor_Surface(S2);

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
            catch (Standard_Failure)
            {
                return false;
            }
        }

        // case of two cylindrical surfaces, at least one of which is a swept surface
        // swept surfaces: SurfaceOfLinearExtrusion, SurfaceOfRevolution
        if ((S1 is Geom_CylindricalSurface || S1 is Geom_SweptSurface)
            && (S2 is Geom_CylindricalSurface || S2 is Geom_SweptSurface))
        {
            var aCyl1 = new gp_Cylinder();
            var aCyl2 = new gp_Cylinder();
            if (getCylinder(S1, ref aCyl1) && getCylinder(S2, ref aCyl2))
            {
                if (Math.Abs(aCyl1.Radius() - aCyl2.Radius()) < theLinTol)
                {
                    gp_Dir aDir1 = aCyl1.Position().Direction();
                    gp_Dir aDir2 = aCyl2.Position().Direction();
                    if (aDir1.IsParallel(aDir2, Precision.Angular()))
                    {
                        gp_Pnt aLoc1  = aCyl1.Location();
                        gp_Pnt aLoc2  = aCyl2.Location();
                        var    aVec12 = new gp_Vec(aLoc1, aLoc2);
                        if (aVec12.SquareMagnitude() < theLinTol * theLinTol
                            || aVec12.IsParallel(new gp_Vec(aDir1), Precision.Angular()))
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
    private static void ReplaceSurface(Geom_Surface theOld, Geom_Surface theNew, IEnumerable<TopoDS_Shape> theRoots)
    {
        var aFaces = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        var anEdges = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        foreach (TopoDS_Shape aRoot in theRoots)
        {
            if (aRoot.IsNull())
            {
                continue;
            }
            for (var anExp = new TopExp_Explorer(aRoot, TopAbs_FACE); anExp.More(); anExp.Next())
            {
                if (anExp.Current().TShape() is BRep_TFace aTFace && aFaces.Add(aTFace)
                    && ReferenceEquals(aTFace.Surface(), theOld))
                {
                    aTFace.Surface(theNew);
                }
            }
            for (var anExp = new TopExp_Explorer(aRoot, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                if (anExp.Current().TShape() is not BRep_TEdge aTEdge || !anEdges.Add(aTEdge))
                {
                    continue;
                }
                var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(aTEdge.ChangeCurves());
                for (; anIt.More(); anIt.Next())
                {
                    switch (anIt.Value())
                    {
                        case BRep_CurveOnClosedSurface aCOCS when ReferenceEquals(aCOCS.Surface(), theOld):
                        {
                            var aNew = new BRep_CurveOnClosedSurface(aCOCS.PCurve()!, aCOCS.PCurve2(), theNew, aCOCS.Location(),
                                                                     aCOCS.Continuity());
                            aNew.SetRange(aCOCS.First(), aCOCS.Last());
                            aCOCS.UVPoints(out gp_Pnt2d aP1, out gp_Pnt2d aP2);
                            aNew.SetUVPoints(aP1, aP2);
                            aCOCS.UVPoints2(out aP1, out aP2);
                            aNew.SetUVPoints2(aP1, aP2);
                            anIt.ChangeValue() = aNew;
                            break;
                        }
                        case BRep_CurveOnSurface aCOS when ReferenceEquals(aCOS.Surface(), theOld):
                        {
                            var aNew = new BRep_CurveOnSurface(aCOS.PCurve()!, theNew, aCOS.Location());
                            aNew.SetRange(aCOS.First(), aCOS.Last());
                            aCOS.UVPoints(out gp_Pnt2d aP1, out gp_Pnt2d aP2);
                            aNew.SetUVPoints(aP1, aP2);
                            anIt.ChangeValue() = aNew;
                            break;
                        }
                        case BRep_CurveOn2Surfaces aCO2S when ReferenceEquals(aCO2S.Surface(), theOld)
                                                             || ReferenceEquals(aCO2S.Surface2(), theOld):
                        {
                            anIt.ChangeValue() = new BRep_CurveOn2Surfaces(
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

    private static void UpdateMapOfShapes(NCollection_Map<TopoDS_Shape> theMapOfShapes, ShapeBuild_ReShape theContext)
    {
        // OCCT adds to the map while it iterates over it; the shapes added are the context's results, which the
        // context maps onto themselves, so iterating over the shapes there before is the same
        foreach (TopoDS_Shape aShape in theMapOfShapes.ToArray())
        {
            TopoDS_Shape aContextShape = theContext.InternalApply(aShape);
            if (!aContextShape.IsSame(aShape))
            {
                theMapOfShapes.Add(aContextShape);
            }
        }
    }

    private static void SplitWire(in TopoDS_Wire                       theWire,
                                  in TopoDS_Face                       theFace,
                                  NCollection_IndexedMap<TopoDS_Shape> theVmap,
                                  NCollection_Sequence<TopoDS_Shape>   theWireSeq)
    {
        var aVEmap = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

        var aEmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var itw   = new TopoDS_Iterator(theWire);
        for (; itw.More(); itw.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(itw.Value());
            if (!aEmap.Add(anEdge))
            {
                continue;
            }
            if (anEdge.Orientation() != TopAbs_FORWARD && anEdge.Orientation() != TopAbs_REVERSED)
            {
                continue;
            }

            TopoDS_Vertex aVertex = TopExp.FirstVertex(anEdge, true); // with orientation
            if (aVEmap.IsBound(aVertex))
            {
                aVEmap[aVertex].Append(anEdge);
            }
            else
            {
                var aElist = new NCollection_List<TopoDS_Shape>();
                aElist.Append(anEdge);
                aVEmap.Bind(aVertex, aElist);
            }
        }

        var aBB = new BRep_Builder();
        for (int ii = 1; ii <= theVmap.Extent(); ii++)
        {
            TopoDS_Vertex                  anOrigin  = TopoDS.Vertex(theVmap[ii]);
            NCollection_List<TopoDS_Shape> aBranches = aVEmap[anOrigin];
            var                            anItl     = new NCollection_List<TopoDS_Shape>.Iterator(aBranches);
            while (anItl.More())
            {
                TopoDS_Edge CurEdge = TopoDS.Edge(anItl.Value());
                aBranches.Remove(anItl);

                var aNewWire = new TopoDS_Wire();
                aBB.MakeWire(ref aNewWire);
                for (;;)
                {
                    aBB.Add(aNewWire, CurEdge);

                    TopoDS_Vertex aVertex = TopExp.LastVertex(CurEdge, true); // with orientation
                    if (aVertex.IsSame(anOrigin))
                    {
                        break;
                    }

                    if (!aVEmap.IsBound(aVertex))
                    {
                        break;
                    }

                    NCollection_List<TopoDS_Shape> aElist = aVEmap[aVertex];
                    if (aElist.Extent() == 0)
                    {
                        break;
                    }

                    Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(CurEdge, theFace, out double fpar, out double lpar);
                    if (aElist.Extent() == 1)
                    {
                        CurEdge = TopoDS.Edge(aElist.First());
                        aElist.Clear();
                    }
                    else if (aPCurve is null)
                    {
                        CurEdge = TopoDS.Edge(aElist.First());
                        aElist.RemoveFirst();
                    }
                    else
                    {
                        double   aParam = (CurEdge.Orientation() == TopAbs_FORWARD) ? lpar : fpar;
                        gp_Vec2d CurDir = aPCurve.EvalD1(aParam).D1;
                        CurDir.Normalize();
                        if (CurEdge.Orientation() == TopAbs_REVERSED)
                        {
                            CurDir.Reverse();
                        }
                        // choose the rightest direction - the smallest angle
                        double MinAngle   = RealLast();
                        var    NextEdge   = new TopoDS_Edge();
                        var    aLocalIter = new NCollection_List<TopoDS_Shape>.Iterator(aElist);
                        for (; aLocalIter.More(); aLocalIter.Next())
                        {
                            TopoDS_Edge anEdge = TopoDS.Edge(aLocalIter.Value());
                            aPCurve = BRep_Tool.CurveOnSurface(anEdge, theFace, out fpar, out lpar);
                            if (aPCurve is null)
                            {
                                // edge has no pcurve on the reference face, skip it as a direction candidate
                                continue;
                            }
                            aParam = (anEdge.Orientation() == TopAbs_FORWARD) ? fpar : lpar;
                            gp_Vec2d aDir = aPCurve.EvalD1(aParam).D1;
                            aDir.Normalize();
                            if (anEdge.Orientation() == TopAbs_REVERSED)
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
                        if (NextEdge.IsNull())
                        {
                            // all candidates lacked a pcurve, fall back to an arbitrary one instead of stalling
                            NextEdge = TopoDS.Edge(aElist.First());
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
    private static bool isSameSets(NCollection_Map<TopoDS_Shape>? theFShells1, NCollection_Map<TopoDS_Shape>? theFShells2)
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
        HashSet<TopoDS_Shape>.Enumerator it1 = theFShells1.GetEnumerator();
        HashSet<TopoDS_Shape>.Enumerator it2 = theFShells2.GetEnumerator();
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
    private static void CheckSharedVertices(NCollection_Sequence<TopoDS_Shape>                                       theSeqEdges,
                                            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEdgesVertex,
                                            NCollection_Map<TopoDS_Shape>                                            theMapKeepShape,
                                            NCollection_Map<TopoDS_Shape>                                            theShareVertMap)
    {
        var sae         = new ShapeAnalysis_Edge();
        var SeqVertexes = new NCollection_Sequence<TopoDS_Shape>();
        var MapVertexes = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (int k = 1; k <= theSeqEdges.Length(); k++)
        {
            TopoDS_Vertex aV1 = sae.FirstVertex(TopoDS.Edge(theSeqEdges[k]));
            TopoDS_Vertex aV2 = sae.LastVertex(TopoDS.Edge(theSeqEdges[k]));
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
            NCollection_List<TopoDS_Shape> ListEdgesV1 = theMapEdgesVertex.FindFromKey(SeqVertexes[k]);
            if (ListEdgesV1.Extent() > 2 || theMapKeepShape.Contains(SeqVertexes[k]))
            {
                theShareVertMap.Add(SeqVertexes[k]);
            }
        }
        // return theShareVertMap.IsEmpty() ? false : true;
    }
}
