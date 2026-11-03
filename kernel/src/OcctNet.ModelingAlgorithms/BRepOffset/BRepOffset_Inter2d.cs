// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter2d.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Wed Dec 24 18:08:39 2003 OCC4455

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Computes the intersections between edges on a face and stores the result as descendants in an AsDes
/// (<see cref="BRepAlgo_AsDes"/>).
/// </summary>
/// <remarks>
/// A tool of the 3D offset. As in OCCT, it writes into the shapes it works on: the edges stored in the AsDes it is
/// given (pcurve ranges, 3D curves built by <see cref="BRepLib"/>, the parameters of the intersection vertices) and the
/// vertices stored there (tolerances, parameters on edges). Those are the offset's own working shapes, built from its
/// own copy of the caller's shape: they must be unfrozen; a frozen one throws at the first write. The vertices and the
/// edge (<see cref="ExtentEdge"/>) it makes are not frozen.
/// </remarks>
public static class BRepOffset_Inter2d
{
    //=================================================================================================

    private static TopoDS_Vertex CommonVertex(in TopoDS_Edge E1, in TopoDS_Edge E2)
    {
        TopoDS_Vertex[] V1 = [new(), new()], V2 = [new(), new()];
        TopoDS_Vertex V = new();
        //
        TopExp.Vertices(E1, out V1[0], out V1[1], true);
        TopExp.Vertices(E2, out V2[0], out V2[1], true);
        // The first edge is the current one, the second edge is the next one.
        // We check last vertex of the first edge first.
        if (V1[1].IsSame(V2[0]) || V1[1].IsSame(V2[1]))
        {
            return V1[1];
        }
        if (V1[0].IsSame(V2[0]) || V1[0].IsSame(V2[1]))
        {
            return V1[0];
        }
        //
        return V;
    }

    private static int DefineClosedness(in TopoDS_Face theFace)
    {
        TopExp_Explorer anExplo = new(theFace, TopAbs_EDGE);
        for (; anExplo.More(); anExplo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExplo.Current());
            if (BRepTools.IsReallyClosed(anEdge, theFace))
            {
                Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(anEdge, theFace, out double fpar, out _);
                Vector2D aTangent = aPCurve!.DN(fpar, 1);
                double aCrossProd1 = aTangent ^ new Vector2D(Constants.DX2d());
                double aCrossProd2 = aTangent ^ new Vector2D(Constants.DY2d());
                if (Math.Abs(aCrossProd2) < Math.Abs(aCrossProd1))
                { // pcurve is parallel to OY
                    return 1;
                }
                else
                {
                    return 2;
                }
            }
        }

        return 0;
    }

    private static void GetEdgesOrientedInFace(in TopoDS_Shape theShape,
                                               in TopoDS_Face theFace,
                                               BRepAlgo_AsDes theAsDes,
                                               NCollection_Sequence<TopoDS_Shape> theSeqEdges)
    {
        NCollection_List<TopoDS_Shape> aEdges = theAsDes.Descendant(theFace);

        TopExp_Explorer anExplo = new(theShape, TopAbs_EDGE);
        for (; anExplo.More(); anExplo.Next())
        {
            TopoDS_Shape anEdge = anExplo.Current();
            NCollection_List<TopoDS_Shape>.Iterator itl = new(aEdges);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Shape anEdgeInFace = itl.Value();
                if (anEdgeInFace.IsSame(anEdge))
                {
                    theSeqEdges.Append(anEdgeInFace);
                    break;
                }
            }
        }

        if (theSeqEdges.Length() == 1)
        {
            return;
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEmap =
            new(TopTools_ShapeMapHasher.Instance);
        for (int ii = 1; ii <= theSeqEdges.Length(); ii++)
        {
            TopExp.MapShapesAndAncestors(theSeqEdges[ii], TopAbs_VERTEX, TopAbs_EDGE, aVEmap);
        }

        TopoDS_Vertex aFirstVertex = new();
        TopoDS_Edge aFirstEdge = new();
        for (int ii = 1; ii <= aVEmap.Extent(); ii++)
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(aVEmap.FindKey(ii));
            NCollection_List<TopoDS_Shape> aElist = aVEmap[ii];
            if (aElist.Extent() == 1)
            {
                TopoDS_Edge anEdge = TopoDS.Edge(aElist.First());
                TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out _, true); // with orientation
                if (aV1.IsSame(aVertex))
                {
                    aFirstVertex = aVertex;
                    aFirstEdge = anEdge;
                    break;
                }
            }
        }

        if (aFirstEdge.IsNull()) // closed set of edges
        {
            // double aPeriod = 0.;
            int IndCoord = DefineClosedness(theFace);
            /*
            BRepAdaptor_Surface aBAsurf (theFace, false);
            if (IndCoord == 1)
              aPeriod = aBAsurf.LastUParameter() - aBAsurf.FirstUParameter();
            else if (IndCoord == 2)
              aPeriod = aBAsurf.LastVParameter() - aBAsurf.FirstVParameter();
            */

            if (IndCoord != 0)
            {
                double aMaxDelta = 0.0;
                for (int ii = 1; ii <= aVEmap.Extent(); ii++)
                {
                    TopoDS_Vertex aVertex = TopoDS.Vertex(aVEmap.FindKey(ii));
                    NCollection_List<TopoDS_Shape> aElist = aVEmap[ii];
                    TopoDS_Edge anEdge1 = TopoDS.Edge(aElist.First());
                    TopoDS_Edge anEdge2 = TopoDS.Edge(aElist.Last());
                    double aParam1 = BRep_Tool.Parameter(aVertex, anEdge1);
                    double aParam2 = BRep_Tool.Parameter(aVertex, anEdge2);
                    BRepAdaptor_Curve2d aBAcurve1 = new(anEdge1, theFace);
                    BRepAdaptor_Curve2d aBAcurve2 = new(anEdge2, theFace);
                    Point2D aPnt1 = aBAcurve1.Value(aParam1);
                    Point2D aPnt2 = aBAcurve2.Value(aParam2);
                    double aDelta = Math.Abs(aPnt1.Coord(IndCoord) - aPnt2.Coord(IndCoord));
                    if (aDelta > aMaxDelta)
                    {
                        aMaxDelta = aDelta;
                        aFirstVertex = aVertex;
                    }
                }
                NCollection_List<TopoDS_Shape> aElist1 = aVEmap.FindFromKey(aFirstVertex);
                NCollection_List<TopoDS_Shape>.Iterator itl = new(aElist1);
                for (; itl.More(); itl.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(itl.Value());
                    TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out _, true); // with orientation
                    if (aV1.IsSame(aFirstVertex))
                    {
                        aFirstEdge = anEdge;
                        break;
                    }
                }
            }
        }

        int aNbEdges = theSeqEdges.Length();
        theSeqEdges.Clear();
        theSeqEdges.Append(aFirstEdge);
        TopoDS_Edge anEdgeCur = aFirstEdge;
        for (; ; )
        {
            TopoDS_Vertex aLastVertex = TopExp.LastVertex(anEdgeCur, true); // with orientation
            if (aLastVertex.IsSame(aFirstVertex))
            {
                break;
            }

            NCollection_List<TopoDS_Shape> aElist = aVEmap.FindFromKey(aLastVertex);
            if (aElist.Extent() == 1)
            {
                break;
            }

            if (aElist.First().IsSame(anEdgeCur))
            {
                anEdgeCur = TopoDS.Edge(aElist.Last());
            }
            else
            {
                anEdgeCur = TopoDS.Edge(aElist.First());
            }

            theSeqEdges.Append(anEdgeCur);
            if (theSeqEdges.Length() == aNbEdges)
            {
                break;
            }
        }
    }

    //=======================================================================
    // function : Store
    // purpose  : Store the vertices <theLV> into AsDes for the edge <theEdge>.
    //           The vertices are added despite of the coincidence with
    //           already added vertices. When all vertices for all edges
    //           are added the coinciding chains of vertices should be fused
    //           using FuseVertices() method.
    //=======================================================================
    private static void Store(in TopoDS_Edge theEdge,
                              NCollection_List<TopoDS_Shape> theLV,
                              double theTol,
                              bool IsToUpdate,
                              BRepAlgo_AsDes theAsDes2d,
                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV)
    {
        // Update vertices
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLV);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aIt.Value());
            new BRep_Builder().UpdateVertex(aV, theTol);
        }

        // Get vertices already added to the edge and check the distances to the new ones
        NCollection_List<TopoDS_Shape> aLVEx = theAsDes2d.Descendant(theEdge);
        if (!IsToUpdate && aLVEx.IsEmpty())
        {
            if (theLV.Extent() != 0)
            {
                theAsDes2d.Add(theEdge, theLV);
            }
            return;
        }
        //
        GeomAPI_ProjectPointOnCurve aProjPC = new();
        double aTolE = 0.0;
        if (IsToUpdate)
        {
            Geom_Curve? aC = BRep_Tool.Curve(theEdge, out double aT1, out double aT2);
            aProjPC.Init(aC!, aT1, aT2);
            aTolE = BRep_Tool.Tolerance(theEdge);
        }
        //
        NCollection_Map<TopoDS_Shape> aMV = new(TopTools_ShapeMapHasher.Instance);
        for (aIt.Initialize(theLV); aIt.More(); aIt.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aIt.Value());
            if (!aMV.Add(aV))
            {
                continue;
            }
            //
            Point3D aP = BRep_Tool.Pnt(aV);
            double aTol = BRep_Tool.Tolerance(aV);
            //
            NCollection_List<TopoDS_Shape> aLVC = new();
            NCollection_List<TopoDS_Shape>.Iterator aItEx = new(aLVEx);
            for (; aItEx.More(); aItEx.Next())
            {
                TopoDS_Vertex aVEx = TopoDS.Vertex(aItEx.Value());
                if (aV.IsSame(aVEx))
                {
                    break;
                }
                Point3D aPEx = BRep_Tool.Pnt(aVEx);
                double aTolVEx = BRep_Tool.Tolerance(aVEx);
                if (aP.IsEqual(aPEx, aTol + aTolVEx))
                {
                    aLVC.Append(aVEx);
                }
            }
            //
            if (aItEx.More())
            {
                continue;
            }
            //
            if (IsToUpdate)
            {
                // get parameter of the vertex on the edge
                aProjPC.Perform(aP);
                if (aProjPC.NbPoints() == 0)
                {
                    continue;
                }
                //
                if (aProjPC.LowerDistance() > aTol + aTolE)
                {
                    continue;
                }
                //
                double aT = aProjPC.LowerDistanceParameter();
                TopoDS_Shape aLocalShape = aV.Oriented(TopAbs_INTERNAL);
                new BRep_Builder().UpdateVertex(TopoDS.Vertex(aLocalShape), aT, theEdge, aTol);
            }
            //
            if (aLVC.Extent() != 0)
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLV = new(aLVC);
                for (; aItLV.More(); aItLV.Next())
                {
                    TopoDS_Shape aVC = aItLV.Value();
                    int anIndC = theDMVV.FindIndex(aVC);
                    if (anIndC == 0)
                    {
                        anIndC = theDMVV.Add(aVC, new NCollection_List<TopoDS_Shape>());
                    }
                    theDMVV[anIndC].Append(aV);
                }
                //
                int anInd = theDMVV.FindIndex(aV);
                if (anInd == 0)
                {
                    anInd = theDMVV.Add(aV, new NCollection_List<TopoDS_Shape>());
                }
                theDMVV[anInd].Append(aLVC);
            }
            theAsDes2d.Add(theEdge, aV);
        }
    }

    //=======================================================================
    // function : Store
    // purpose  : Store the intersection vertices between two edges into AsDes
    //=======================================================================
    private static void Store(in TopoDS_Edge theE1,
                              in TopoDS_Edge theE2,
                              NCollection_List<TopoDS_Shape> theLV1,
                              NCollection_List<TopoDS_Shape> theLV2,
                              double theTol,
                              BRepAlgo_AsDes theAsDes2d,
                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV)
    {
        for (int i = 0; i < 2; ++i)
        {
            TopoDS_Edge aE = i == 0 ? theE1 : theE2;
            NCollection_List<TopoDS_Shape> aLV = i == 0 ? theLV1 : theLV2;
            Store(aE, aLV, theTol, false, theAsDes2d, theDMVV);
        }
    }

    //=================================================================================================

    private static void EdgeInter(in TopoDS_Face F,
                                  BRepAdaptor_Surface BAsurf,
                                  in TopoDS_Edge E1,
                                  in TopoDS_Edge E2,
                                  BRepAlgo_AsDes AsDes,
                                  double Tol,
                                  bool WithOri,
                                  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV)
    {

        if (E1.IsSame(E2))
        {
            return;
        }

        double[] f = new double[3], l = new double[3];
        double TolDub = 1.0e-7;
        int i;

        BRep_Tool.Range(E1, out f[1], out l[1]);
        BRep_Tool.Range(E2, out f[2], out l[2]);

        BRepAdaptor_Curve CE1 = new(E1, F);
        BRepAdaptor_Curve CE2 = new(E2, F);

        TopoDS_Edge[] EI = [new(), new(), new()];
        EI[1] = E1;
        EI[2] = E2;
        NCollection_List<TopoDS_Shape> LV1 = new();
        NCollection_List<TopoDS_Shape> LV2 = new();
        BRep_Builder B = new();

        TopoDS_Vertex CV = new();
        if (!TopExp.CommonVertex(E1, E2, ref CV))
        {
            BRepLib.BuildCurve3d(E1);
            BRepLib.BuildCurve3d(E2);

            double TolSum = BRep_Tool.Tolerance(E1) + BRep_Tool.Tolerance(E2);
            TolSum = Max(TolSum, 1.0e-5);

            NCollection_Sequence<Point3D> ResPoints = new();
            NCollection_Sequence<double> ResParamsOnE1 = new(), ResParamsOnE2 = new();
            Point3D DegPoint = new();
            bool WithDegen = BRep_Tool.Degenerated(E1) || BRep_Tool.Degenerated(E2);

            if (WithDegen)
            {
                int ideg = (BRep_Tool.Degenerated(E1)) ? 1 : 2;
                TopoDS_Iterator iter = new(EI[ideg]);
                if (iter.More())
                {
                    TopoDS_Vertex vdeg = TopoDS.Vertex(iter.Value());
                    DegPoint = BRep_Tool.Pnt(vdeg);
                }
                else
                {
                    BRepAdaptor_Curve CEdeg = new(EI[ideg], F);
                    DegPoint = CEdeg.Value(CEdeg.FirstParameter());
                }
            }
            //
            Geom2d_Curve? pcurve1 = BRep_Tool.CurveOnSurface(E1, F, out f[1], out l[1]);
            Geom2d_Curve? pcurve2 = BRep_Tool.CurveOnSurface(E2, F, out f[2], out l[2]);
            Geom2dAdaptor_Curve GAC1 = new(pcurve1!, f[1], l[1]);
            Geom2dAdaptor_Curve GAC2 = new(pcurve2!, f[2], l[2]);
            Geom2dInt_GInter Inter2d = new(GAC1, GAC2, TolDub, TolDub);
            for (i = 1; i <= Inter2d.NbPoints(); i++)
            {
                Point3D P3d;
                if (WithDegen)
                {
                    P3d = DegPoint;
                }
                else
                {
                    Point2D P2d = Inter2d.Point(i).Value();
                    P3d = BAsurf.Value(P2d.X(), P2d.Y());
                }
                ResPoints.Append(P3d);
                ResParamsOnE1.Append(Inter2d.Point(i).ParamOnFirst());
                ResParamsOnE2.Append(Inter2d.Point(i).ParamOnSecond());
            }

            for (i = 1; i <= ResPoints.Length(); i++)
            {
                double aT1 = ResParamsOnE1[i]; // ponc1.Parameter();
                double aT2 = ResParamsOnE2[i]; // ponc2.Parameter();
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aT1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aT2))
                {
                    // OCCT_DEBUG output left out
                    continue;
                }

                Point3D P = ResPoints[i]; // ponc1.Value();
                TopoDS_Vertex aNewVertex = new BRepLib_MakeVertex(P).InternalVertex();
                aNewVertex.Orientation(TopAbs_INTERNAL);
                B.UpdateVertex(aNewVertex, aT1, E1, Tol);
                B.UpdateVertex(aNewVertex, aT2, E2, Tol);
                Point3D P1 = CE1.Value(aT1);
                Point3D P2 = CE2.Value(aT2);
                double dist1, dist2, dist3;
                dist1 = P1.Distance(P);
                dist2 = P2.Distance(P);
                dist3 = P1.Distance(P2);
                dist1 = Max(dist1, dist2);
                dist1 = Max(dist1, dist3);
                B.UpdateVertex(aNewVertex, dist1);

                // OCCT_DEBUG checks of the solution left out
                // define the orientation of a new vertex
                TopAbs_Orientation OO1 = TopAbs_REVERSED;
                TopAbs_Orientation OO2 = TopAbs_REVERSED;
                if (WithOri)
                {
                    BRepAdaptor_Curve2d PCE1 = new(E1, F);
                    BRepAdaptor_Curve2d PCE2 = new(E2, F);
                    Vector2D V1or, V2or;
                    PCE1.D1(aT1, out _, out Vector2D V1);
                    PCE2.D1(aT2, out _, out Vector2D V2);
                    V1or = V1;
                    V2or = V2;
                    if (E1.Orientation() == TopAbs_REVERSED)
                    {
                        V1or.Reverse();
                    }
                    if (E2.Orientation() == TopAbs_REVERSED)
                    {
                        V2or.Reverse();
                    }
                    double CrossProd = V2or ^ V1;
                    // OCCT_DEBUG output left out
                    if (CrossProd > 0.0)
                    {
                        OO1 = TopAbs_FORWARD;
                    }
                    CrossProd = V1or ^ V2;
                    if (CrossProd > 0.0)
                    {
                        OO2 = TopAbs_FORWARD;
                    }
                }
                LV1.Append(aNewVertex.Oriented(OO1));
                LV2.Append(aNewVertex.Oriented(OO2));
            }
        }

        //----------------------------------
        // Test at end.
        //---------------------------------
        double U1, U2;
        double TolConf = Tol;
        TopoDS_Vertex[] V1v = [new(), new()], V2v = [new(), new()];
        TopExp.Vertices(E1, out V1v[0], out V1v[1]);
        TopExp.Vertices(E2, out V2v[0], out V2v[1]);

        int j;
        for (j = 0; j < 2; j++)
        {
            if (V1v[j].IsNull())
            {
                continue;
            }
            for (int k = 0; k < 2; k++)
            {
                if (V2v[k].IsNull())
                {
                    continue;
                }
                if (V1v[j].IsSame(V2v[k]))
                {
                    if (AsDes.HasAscendant(V1v[j]))
                    {
                        continue;
                    }
                }
                //
                Point3D P1 = BRep_Tool.Pnt(V1v[j]);
                Point3D P2 = BRep_Tool.Pnt(V2v[k]);
                double Dist = P1.Distance(P2);
                if (Dist < TolConf)
                {
                    double aTol = Max(BRep_Tool.Tolerance(V1v[j]), BRep_Tool.Tolerance(V2v[k]));
                    TopoDS_Vertex V = new BRepLib_MakeVertex(P1).InternalVertex();
                    U1 = (j == 0) ? f[1] : l[1];
                    U2 = (k == 0) ? f[2] : l[2];
                    //
                    TopoDS_Shape aLocalShape = V.Oriented(TopAbs_INTERNAL);
                    B.UpdateVertex(TopoDS.Vertex(aLocalShape), U1, E1, aTol);
                    B.UpdateVertex(TopoDS.Vertex(aLocalShape), U2, E2, aTol);
                    //
                    LV1.Prepend(V.Oriented(V1v[j].Orientation()));
                    LV2.Prepend(V.Oriented(V2v[k].Orientation()));
                }
            }
        }

        bool AffichPurge = false;

        if (!LV1.IsEmpty())
        {
            //----------------------------------
            // Remove all vertices.
            // There can be doubles
            //----------------------------------
            NCollection_List<TopoDS_Shape>.Iterator it1LV1 = new(), it1LV2 = new(), it2LV1 = new();
            bool Purge = true;

            while (Purge)
            {
                i = 1;
                Purge = false;
                for (it1LV1.Initialize(LV1), it1LV2.Initialize(LV2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    TopoDS_Vertex aV1 = TopoDS.Vertex(it1LV1.Value());
                    Point3D P1 = BRep_Tool.Pnt(aV1);
                    double aTol1 = BRep_Tool.Tolerance(aV1);

                    j = 1;
                    it2LV1.Initialize(LV1);
                    while (j < i)
                    {
                        TopoDS_Vertex aV2 = TopoDS.Vertex(it2LV1.Value());
                        Point3D P2 = BRep_Tool.Pnt(aV2);
                        double aTol = Max(aTol1, BRep_Tool.Tolerance(aV2));
                        if (P1.IsEqual(P2, aTol))
                        {
                            LV1.Remove(it1LV1);
                            LV2.Remove(it1LV2);
                            if (AffichPurge)
                            {
                                Console.WriteLine("Doubles removed in EdgeInter.");
                            }
                            Purge = true;
                            break;
                        }
                        j++;
                        it2LV1.Next();
                    }
                    if (Purge)
                    {
                        break;
                    }
                    i++;
                }
            }
            //---------------------------------
            // Vertex storage in DS.
            //---------------------------------
            double TolStore = BRep_Tool.Tolerance(E1) + BRep_Tool.Tolerance(E2);
            TolStore = Max(TolStore, Tol);
            Store(E1, E2, LV1, LV2, TolStore, AsDes, aDMVV);
        }
    }

    //=================================================================================================

    private static void RefEdgeInter(in TopoDS_Face F,
                                     BRepAdaptor_Surface BAsurf,
                                     in TopoDS_Edge E1,
                                     in TopoDS_Edge E2,
                                     TopAbs_Orientation theOr1,
                                     TopAbs_Orientation theOr2,
                                     BRepAlgo_AsDes AsDes,
                                     double Tol,
                                     bool WithOri,
                                     in TopoDS_Vertex theVref,
                                     BRepAlgo_Image theImageVV,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV,
                                     out bool theCoincide)
    {
        //
        theCoincide = false;
        //
        if (E1.IsSame(E2))
        {
            return;
        }
        if (E1.IsNull() || E2.IsNull())
        {
            return;
        }

        double[] f = new double[3], l = new double[3];
        double TolDub = 1.0e-7, TolLL = 0.0;
        int i;

        Geom2d_Curve? pcurve1 = BRep_Tool.CurveOnSurface(E1, F, out f[1], out l[1]);
        Geom2d_Curve? pcurve2 = BRep_Tool.CurveOnSurface(E2, F, out f[2], out l[2]);
        if (pcurve1 is null || pcurve2 is null)
        {
            return;
        }

        BRepAdaptor_Curve CE1 = new(E1, F);
        BRepAdaptor_Curve CE2 = new(E2, F);

        TopoDS_Edge[] EI = [new(), new(), new()];
        EI[1] = E1;
        EI[2] = E2;
        NCollection_List<TopoDS_Shape> LV1 = new();
        NCollection_List<TopoDS_Shape> LV2 = new();
        BRep_Builder B = new();

        BRepLib.BuildCurve3d(E1);
        BRepLib.BuildCurve3d(E2);

        NCollection_Sequence<Point3D> ResPoints = new();
        NCollection_Sequence<double> ResParamsOnE1 = new(), ResParamsOnE2 = new();
        Point3D DegPoint = new();
        bool WithDegen = BRep_Tool.Degenerated(E1) || BRep_Tool.Degenerated(E2);

        if (WithDegen)
        {
            int ideg = (BRep_Tool.Degenerated(E1)) ? 1 : 2;
            TopoDS_Iterator iter = new(EI[ideg]);
            if (iter.More())
            {
                TopoDS_Vertex vdeg = TopoDS.Vertex(iter.Value());
                DegPoint = BRep_Tool.Pnt(vdeg);
            }
            else
            {
                BRepAdaptor_Curve CEdeg = new(EI[ideg], F);
                DegPoint = CEdeg.Value(CEdeg.FirstParameter());
            }
        }
        //
        Geom2dAdaptor_Curve GAC1 = new(pcurve1, f[1], l[1]);
        Geom2dAdaptor_Curve GAC2 = new(pcurve2, f[2], l[2]);
        if ((GAC1.GetType() == Line) && (GAC2.GetType() == Line))
        {
            // Just quickly check if lines coincide
            double anAngle = Math.Abs(GAC1.Line().Direction().Angle(GAC2.Line().Direction()));
            if (anAngle <= 1.0e-8 || M_PI - anAngle <= 1.0e-8)
            {
                theCoincide = true;
                return;
            }
            else
            {
                // Take into account the intersection range of line-line intersection
                // (the smaller angle between curves, the bigger range)
                TolLL = IntTools_Tools.ComputeIntRange(TolDub, TolDub, anAngle);
                TolLL = Min(TolLL, 1.0e-5);
            }
        }

        Geom2dInt_GInter Inter2d = new(GAC1, GAC2, TolDub, TolDub);
        //
        if (!Inter2d.IsDone() || Inter2d.NbPoints() == 0)
        {
            theCoincide = (Inter2d.NbSegments() != 0 && (GAC1.GetType() == Line)
                           && (GAC2.GetType() == Line));
            return;
        }
        //
        for (i = 1; i <= Inter2d.NbPoints(); i++)
        {
            Point3D P3d;
            if (WithDegen)
            {
                P3d = DegPoint;
            }
            else
            {
                Point2D P2d = Inter2d.Point(i).Value();
                P3d = BAsurf.Value(P2d.X(), P2d.Y());
            }
            ResPoints.Append(P3d);
            ResParamsOnE1.Append(Inter2d.Point(i).ParamOnFirst());
            ResParamsOnE2.Append(Inter2d.Point(i).ParamOnSecond());
        }

        for (i = 1; i <= ResPoints.Length(); i++)
        {
            double aT1 = ResParamsOnE1[i]; // ponc1.Parameter();
            double aT2 = ResParamsOnE2[i]; // ponc2.Parameter();
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aT1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aT2))
            {
                // OCCT_DEBUG output left out
                continue;
            }

            Point3D P = ResPoints[i]; // ponc1.Value();
            TopoDS_Vertex aNewVertex = new BRepLib_MakeVertex(P).InternalVertex();
            aNewVertex.Orientation(TopAbs_INTERNAL);
            B.UpdateVertex(aNewVertex, aT1, E1, Tol);
            B.UpdateVertex(aNewVertex, aT2, E2, Tol);
            Point3D P1 = CE1.Value(aT1);
            Point3D P2 = CE2.Value(aT2);
            double dist1, dist2, dist3;
            dist1 = P1.Distance(P);
            dist2 = P2.Distance(P);
            dist3 = P1.Distance(P2);
            dist1 = Max(dist1, dist2);
            dist1 = Max(dist1, dist3);
            B.UpdateVertex(aNewVertex, dist1);

            // OCCT_DEBUG checks of the solution left out
            // define the orientation of a new vertex
            TopAbs_Orientation OO1 = TopAbs_REVERSED;
            TopAbs_Orientation OO2 = TopAbs_REVERSED;
            if (WithOri)
            {
                BRepAdaptor_Curve2d PCE1 = new(E1, F);
                BRepAdaptor_Curve2d PCE2 = new(E2, F);
                Vector2D V1or, V2or;
                PCE1.D1(aT1, out _, out Vector2D V1);
                PCE2.D1(aT2, out _, out Vector2D V2);
                V1or = V1;
                V2or = V2;
                if (E1.Orientation() == TopAbs_REVERSED)
                {
                    V1or.Reverse();
                }
                if (E2.Orientation() == TopAbs_REVERSED)
                {
                    V2or.Reverse();
                }
                double CrossProd = V2or ^ V1;
                // OCCT_DEBUG output left out
                if (CrossProd > 0.0)
                {
                    OO1 = TopAbs_FORWARD;
                }
                CrossProd = V1or ^ V2;
                if (CrossProd > 0.0)
                {
                    OO2 = TopAbs_FORWARD;
                }
            }

            if (theOr1 != TopAbs_EXTERNAL)
            {
                OO1 = theOr1;
            }
            if (theOr2 != TopAbs_EXTERNAL)
            {
                OO2 = theOr2;
            }

            LV1.Append(aNewVertex.Oriented(OO1));
            LV2.Append(aNewVertex.Oriented(OO2));
        }

        //----------------------------------
        // Test at end.
        //---------------------------------
        double U1, U2;
        double TolConf = Tol;
        TopoDS_Vertex[] V1v = [new(), new()], V2v = [new(), new()];
        TopExp.Vertices(E1, out V1v[0], out V1v[1]);
        TopExp.Vertices(E2, out V2v[0], out V2v[1]);

        int j;
        for (j = 0; j < 2; j++)
        {
            if (V1v[j].IsNull())
            {
                continue;
            }
            for (int k = 0; k < 2; k++)
            {
                if (V2v[k].IsNull())
                {
                    continue;
                }
                if (V1v[j].IsSame(V2v[k]))
                {
                    if (AsDes.HasAscendant(V1v[j]))
                    {
                        continue;
                    }
                }
                //
                Point3D P1 = BRep_Tool.Pnt(V1v[j]);
                Point3D P2 = BRep_Tool.Pnt(V2v[k]);
                double Dist = P1.Distance(P2);
                if (Dist < TolConf)
                {
                    TopoDS_Vertex V = new BRepLib_MakeVertex(P1).InternalVertex();
                    U1 = (j == 0) ? f[1] : l[1];
                    U2 = (k == 0) ? f[2] : l[2];
                    TopoDS_Shape aLocalShape = V.Oriented(TopAbs_INTERNAL);
                    B.UpdateVertex(TopoDS.Vertex(aLocalShape), U1, E1, Tol);
                    B.UpdateVertex(TopoDS.Vertex(aLocalShape), U2, E2, Tol);
                    LV1.Prepend(V.Oriented(V1v[j].Orientation()));
                    LV2.Prepend(V.Oriented(V2v[k].Orientation()));
                }
            }
        }

        bool AffichPurge = false;

        if (!LV1.IsEmpty())
        {
            //----------------------------------
            // Remove all vertices.
            // there can be doubles
            //----------------------------------
            NCollection_List<TopoDS_Shape>.Iterator it1LV1 = new(), it1LV2 = new(), it2LV1 = new();
            Point3D P1, P2;
            bool Purge = true;

            while (Purge)
            {
                i = 1;
                Purge = false;
                for (it1LV1.Initialize(LV1), it1LV2.Initialize(LV2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    j = 1;
                    it2LV1.Initialize(LV1);
                    while (j < i)
                    {
                        P1 = BRep_Tool.Pnt(TopoDS.Vertex(it1LV1.Value()));
                        P2 = BRep_Tool.Pnt(TopoDS.Vertex(it2LV1.Value()));
                        if (P1.IsEqual(P2, Tol))
                        {
                            LV1.Remove(it1LV1);
                            LV2.Remove(it1LV2);
                            if (AffichPurge)
                            {
                                Console.WriteLine("Doubles removed in EdgeInter.");
                            }
                            Purge = true;
                            break;
                        }
                        j++;
                        it2LV1.Next();
                    }
                    if (Purge)
                    {
                        break;
                    }
                    i++;
                }
            }
            //---------------------------------
            // Vertex storage in SD.
            //---------------------------------
            ////-----------------------------------------------------
            if (LV1.Extent() > 1)
            {
                // std::cout << "IFV - RefEdgeInter: remove vertex" << std::endl;
                Point3D Pref = BRep_Tool.Pnt(theVref);
                double dmin = RealLast();
                TopoDS_Vertex Vmin = new();
                for (it1LV1.Initialize(LV1); it1LV1.More(); it1LV1.Next())
                {
                    Point3D P = BRep_Tool.Pnt(TopoDS.Vertex(it1LV1.Value()));
                    double d = P.SquareDistance(Pref);
                    if (d < dmin)
                    {
                        dmin = d;
                        Vmin = TopoDS.Vertex(it1LV1.Value());
                    }
                }
                for (it1LV1.Initialize(LV1), it1LV2.Initialize(LV2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    if (!Vmin.IsSame(it1LV1.Value()))
                    {
                        // OCCT: Remove moves the iterators to the next vertex and the loop's Next() then skips it,
                        // so a vertex following a removed one is kept
                        LV1.Remove(it1LV1);
                        LV2.Remove(it1LV2);
                        if (!it1LV1.More())
                        {
                            break;
                        }
                    }
                }
            }

            NCollection_List<TopoDS_Shape>.Iterator itl = new(LV1);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Shape aNewVertex = itl.Value();
                aNewVertex.Orientation(TopAbs_FORWARD);
                if (theImageVV.HasImage(theVref))
                {
                    theImageVV.Add(theVref.Oriented(TopAbs_FORWARD), aNewVertex);
                }
                else
                {
                    theImageVV.Bind(theVref.Oriented(TopAbs_FORWARD), aNewVertex);
                }
            }

            ////-----------------------------------------------------
            double TolStore = BRep_Tool.Tolerance(E1) + BRep_Tool.Tolerance(E2);
            TolStore = Max(TolStore, Tol);
            // Compare to Line-Line tolerance
            TolStore = Max(TolStore, TolLL);
            Store(E1, E2, LV1, LV2, TolStore, AsDes, aDMVV);
        }
    }

    //======================================================================
    // function : EvaluateMaxSegment
    // purpose  : return MaxSegment to pass in approximation
    //======================================================================

    private static int evaluateMaxSegment(Adaptor3d_CurveOnSurface aCurveOnSurface)
    {
        Adaptor3d_Surface aSurf = aCurveOnSurface.GetSurface()!;
        Adaptor2d_Curve2d aCurv2d = aCurveOnSurface.GetCurve()!;

        double aNbSKnots = 0, aNbC2dKnots = 0;

        if (aSurf.GetType() == BSplineSurface)
        {
            Geom_BSplineSurface aBSpline = aSurf.BSpline();
            aNbSKnots = Max(aBSpline.NbUKnots(), aBSpline.NbVKnots());
        }
        if (aCurv2d.GetType() == BSplineCurve)
        {
            aNbC2dKnots = aCurv2d.NbKnots();
        }
        int aReturn = (int)(30 + Max(aNbSKnots, aNbC2dKnots));
        return aReturn;
    }

    //=================================================================================================

    private static bool ExtendPCurve(Geom2d_Curve aPCurve,
                                     double anEf,
                                     double anEl,
                                     double a2Offset,
                                     out Geom2d_Curve? NewPCurve)
    {
        NewPCurve = aPCurve;
        if (NewPCurve.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            NewPCurve = ((Geom2d_TrimmedCurve)NewPCurve).BasisCurve();
        }

        double FirstPar = NewPCurve.FirstParameter();
        double LastPar = NewPCurve.LastParameter();

        if (NewPCurve is Geom2d_BoundedCurve && (FirstPar > anEf - a2Offset || LastPar < anEl + a2Offset))
        {
            if (NewPCurve.GetType() == typeof(Geom2d_BezierCurve))
            {
                Geom2d_BezierCurve aBezier = (Geom2d_BezierCurve)NewPCurve;
                if (aBezier.NbPoles() == 2)
                {
                    NCollection_Array1<Point2D> thePoles = aBezier.InternalPoles();
                    Vector2D aVec = new(thePoles[1], thePoles[2]);
                    NewPCurve = new Geom2d_Line(thePoles[1], new Direction2D(aVec));
                    return true;
                }
            }
            else if (NewPCurve.GetType() == typeof(Geom2d_BSplineCurve))
            {
                Geom2d_BSplineCurve aBSpline = (Geom2d_BSplineCurve)NewPCurve;
                if (aBSpline.NbKnots() == 2 && aBSpline.NbPoles() == 2)
                {
                    NCollection_Array1<Point2D> thePoles = aBSpline.InternalPoles();
                    Vector2D aVec = new(thePoles[1], thePoles[2]);
                    NewPCurve = new Geom2d_Line(thePoles[1], new Direction2D(aVec));
                    return true;
                }
            }
        }

        FirstPar = aPCurve.FirstParameter();
        LastPar = aPCurve.LastParameter();
        Geom2d_TrimmedCurve aTrCurve = new(aPCurve, FirstPar, LastPar);

        // The curve is not prolonged on begin or end.
        // Trying to prolong it adding a segment to its bound.
        Point2D aPBnd;
        Vector2D aVBnd;
        Point2D aPBeg;
        Direction2D aDBnd = new();
        Geom2d_Line aLin;
        Geom2d_TrimmedCurve aSegment;
        Geom2dConvert_CompCurveToBSplineCurve aCompCurve = new(aTrCurve, RationalC1);
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double aDelta = Max(a2Offset, 1.0);

        if (FirstPar > anEf - a2Offset)
        {
            aPCurve.D1(FirstPar, out aPBnd, out aVBnd);
            aDBnd.SetXY(aVBnd.XY());
            aPBeg = aPBnd.Translated(new Vector2D(-aDelta * aDBnd.XY()));
            aLin = new Geom2d_Line(aPBeg, aDBnd);
            aSegment = new Geom2d_TrimmedCurve(aLin, 0, aDelta);

            if (!aCompCurve.Add(aSegment, aTol))
            {
                return false;
            }
        }

        if (LastPar < anEl + a2Offset)
        {
            aPCurve.D1(LastPar, out aPBeg, out aVBnd);
            aDBnd.SetXY(aVBnd.XY());
            aLin = new Geom2d_Line(aPBeg, aDBnd);
            aSegment = new Geom2d_TrimmedCurve(aLin, 0, aDelta);

            if (!aCompCurve.Add(aSegment, aTol))
            {
                return false;
            }
        }

        NewPCurve = aCompCurve.BSplineCurve();
        return true;
    }

    //=================================================================================================

    //  Modified by skv - Fri Dec 26 17:00:55 2003 OCC4455 Begin
    // static void ExtentEdge(const TopoDS_Edge& E,TopoDS_Edge& NE)

    /// <summary>
    /// Extends the edge <paramref name="E"/> into a new edge <paramref name="NE"/> (with copies of its curve
    /// representations): its pcurves and 3D curve are prolonged by twice <paramref name="theOffset"/> where they are
    /// bounded, within the bounds of their surfaces. False if a pcurve couldn't be computed on the extended range.
    /// </summary>
    /// <remarks><paramref name="E"/> is only read. The new edge is not frozen.</remarks>
    public static bool ExtentEdge(in TopoDS_Edge E, out TopoDS_Edge NE, double theOffset)
    {
        // BRepLib::BuildCurve3d(E);

        TopoDS_Shape aLocalShape = E.EmptyCopied();
        double a2Offset = 2.0 * Math.Abs(theOffset);
        BRep_Builder BB = new();
        int i, j;

        BRep_Tool.Range(E, out double anEf, out double anEl);
        NE = TopoDS.Edge(aLocalShape);
        //  NE = TopoDS::Edge(E.EmptyCopied());
        // Enough for analytic edges, for general case reconstruct the
        // geometry of the edge recalculating the intersection of surfaces.

        // BRepLib::BuildCurve3d(E);

        int NbPCurves = 0;
        double FirstParOnPC = RealFirst(), LastParOnPC = RealLast();
        Geom2d_Curve? MinPC = null;
        Geom_Surface? MinSurf = null;
        Placement MinLoc = new();

        NCollection_List<BRep_CurveRepresentation>.Iterator itr = new(((BRep_TEdge)NE.TShape()!).ChangeCurves());
        for (; itr.More(); itr.Next())
        {
            BRep_CurveRepresentation CurveRep = itr.Value();
            double FirstPar, LastPar;
            if (CurveRep.IsCurveOnSurface())
            {
                NbPCurves++;
                Geom2d_Curve theCurve = CurveRep.PCurve()!;
                FirstPar = theCurve.FirstParameter();
                LastPar = theCurve.LastParameter();

                if (theCurve is Geom2d_BoundedCurve && (FirstPar > anEf - a2Offset || LastPar < anEl + a2Offset))
                {
                    if (ExtendPCurve(theCurve, anEf, anEl, a2Offset, out Geom2d_Curve? NewPCurve))
                    {
                        CurveRep.PCurve(NewPCurve);
                        FirstPar = NewPCurve!.FirstParameter();
                        LastPar = NewPCurve.LastParameter();
                        if (CurveRep.IsCurveOnClosedSurface())
                        {
                            Geom2d_Curve PCurve2 = CurveRep.PCurve2()!;
                            if (ExtendPCurve(PCurve2, anEf, anEl, a2Offset, out NewPCurve))
                            {
                                CurveRep.PCurve2(NewPCurve);
                            }
                        }
                    }
                }
                else if (theCurve.IsPeriodic())
                {
                    double delta = (theCurve.Period() - (anEl - anEf)) * 0.5;
                    delta *= 0.95;
                    FirstPar = anEf - delta;
                    LastPar = anEl + delta;
                }
                else if (theCurve.IsClosed())
                {
                    LastPar -= 0.05 * (LastPar - FirstPar);
                }

                // check FirstPar and LastPar: the pcurve should be in its surface
                theCurve = CurveRep.PCurve()!;
                Geom_Surface theSurf = CurveRep.Surface()!;
                theSurf.Bounds(out double Umin, out double Umax, out double Vmin, out double Vmax);
                NCollection_Sequence<Geom2d_Curve> BoundLines = new();
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(Vmin))
                {
                    Geom2d_Line aLine = new(new Point2D(0.0, Vmin), new Direction2D(Direction2D.D.X));
                    BoundLines.Append(aLine);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(Umin))
                {
                    Geom2d_Line aLine = new(new Point2D(Umin, 0.0), new Direction2D(Direction2D.D.Y));
                    BoundLines.Append(aLine);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(Vmax))
                {
                    Geom2d_Line aLine = new(new Point2D(0.0, Vmax), new Direction2D(Direction2D.D.X));
                    BoundLines.Append(aLine);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(Umax))
                {
                    Geom2d_Line aLine = new(new Point2D(Umax, 0.0), new Direction2D(Direction2D.D.Y));
                    BoundLines.Append(aLine);
                }

                NCollection_Sequence<double> params_ = new();
                Geom2dInt_GInter IntCC = new();
                Geom2dAdaptor_Curve GAcurve = new(theCurve);
                for (i = 1; i <= BoundLines.Length(); i++)
                {
                    Geom2dAdaptor_Curve GAline = new(BoundLines[i]);
                    IntCC.Perform(GAcurve, GAline, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (IntCC.IsDone())
                    {
                        for (j = 1; j <= IntCC.NbPoints(); j++)
                        {
                            IntRes2d_IntersectionPoint ip = IntCC.Point(j);
                            Point2D aPoint = ip.Value();
                            if (aPoint.X() >= Umin && aPoint.X() <= Umax && aPoint.Y() >= Vmin && aPoint.Y() <= Vmax)
                            {
                                params_.Append(ip.ParamOnFirst());
                            }
                        }
                        for (j = 1; j <= IntCC.NbSegments(); j++)
                        {
                            IntRes2d_IntersectionSegment is_ = IntCC.Segment(j);
                            if (is_.HasFirstPoint())
                            {
                                IntRes2d_IntersectionPoint ip = is_.FirstPoint();
                                Point2D aPoint = ip.Value();
                                if (aPoint.X() >= Umin && aPoint.X() <= Umax && aPoint.Y() >= Vmin
                                    && aPoint.Y() <= Vmax)
                                {
                                    params_.Append(ip.ParamOnFirst());
                                }
                            }
                            if (is_.HasLastPoint())
                            {
                                IntRes2d_IntersectionPoint ip = is_.LastPoint();
                                Point2D aPoint = ip.Value();
                                if (aPoint.X() >= Umin && aPoint.X() <= Umax && aPoint.Y() >= Vmin
                                    && aPoint.Y() <= Vmax)
                                {
                                    params_.Append(ip.ParamOnFirst());
                                }
                            }
                        }
                    }
                }
                if (!params_.IsEmpty())
                {
                    if (params_.Length() == 1)
                    {
                        Point2D PntFirst = theCurve.Value(FirstPar);
                        if (PntFirst.X() >= Umin && PntFirst.X() <= Umax && PntFirst.Y() >= Vmin
                            && PntFirst.Y() <= Vmax)
                        {
                            if (LastPar > params_[1])
                            {
                                LastPar = params_[1];
                            }
                        }
                        else if (FirstPar < params_[1])
                        {
                            FirstPar = params_[1];
                        }
                    }
                    else
                    {
                        double fpar = RealLast(), lpar = RealFirst();
                        for (i = 1; i <= params_.Length(); i++)
                        {
                            if (params_[i] < fpar)
                            {
                                fpar = params_[i];
                            }
                            if (params_[i] > lpar)
                            {
                                lpar = params_[i];
                            }
                        }
                        if (FirstPar < fpar)
                        {
                            FirstPar = fpar;
                        }
                        if (LastPar > lpar)
                        {
                            LastPar = lpar;
                        }
                    }
                }
                //// end of check ////
                ((BRep_GCurve)CurveRep).SetRange(FirstPar, LastPar);
                // gp_Pnt2d Pfirst = theCurve->Value(FirstPar);
                // gp_Pnt2d Plast  = theCurve->Value(LastPar);
                //(occ::down_cast<BRep_CurveOnSurface>(CurveRep))->SetUVPoints( Pfirst, Plast );

                // update FirstParOnPC and LastParOnPC
                if (FirstPar > FirstParOnPC)
                {
                    FirstParOnPC = FirstPar;
                    MinPC = theCurve;
                    MinSurf = theSurf;
                    MinLoc = CurveRep.Location();
                }
                if (LastPar < LastParOnPC)
                {
                    LastParOnPC = LastPar;
                    MinPC = theCurve;
                    MinSurf = theSurf;
                    MinLoc = CurveRep.Location();
                }
            }
        }

        Geom_Curve? C3d = BRep_Tool.Curve(NE, out double f, out double l);
        if (NbPCurves != 0)
        {
            MinLoc = E.Location() * MinLoc;
            if (C3d is not null)
            {
                if (MinPC!.IsClosed())
                {
                    f = FirstParOnPC;
                    l = LastParOnPC;
                }
                else if (C3d.IsPeriodic())
                {
                    double delta = (C3d.Period() - (l - f)) * 0.5;
                    delta *= 0.95;
                    f -= delta;
                    l += delta;
                }
                else if (C3d.IsClosed())
                {
                    l -= 0.05 * (l - f);
                }
                else
                {
                    f = FirstParOnPC;
                    l = LastParOnPC;
                    Transform3D aMinLocTrsf = MinLoc.Transformation();
                    GeomAPI_ProjectPointOnCurve Projector = new();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(FirstParOnPC))
                    {
                        Point2D P2d1 = MinPC.Value(FirstParOnPC);
                        Point3D P1 = MinSurf!.Value(P2d1.X(), P2d1.Y());
                        P1.Transform(aMinLocTrsf);
                        Projector.Init(P1, C3d);
                        if (Projector.NbPoints() > 0)
                        {
                            f = Projector.LowerDistanceParameter();
                        }
                        // OCCT_DEBUG output left out
                    }
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(LastParOnPC))
                    {
                        Point2D P2d2 = MinPC.Value(LastParOnPC);
                        Point3D P2 = MinSurf!.Value(P2d2.X(), P2d2.Y());
                        P2.Transform(aMinLocTrsf);
                        Projector.Init(P2, C3d);
                        if (Projector.NbPoints() > 0)
                        {
                            l = Projector.LowerDistanceParameter();
                        }
                        // OCCT_DEBUG output left out
                    }
                }
                BB.Range(NE, f, l);
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
                {
                    BRepLib.SameParameter(NE, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                }
            }
            else if (!BRep_Tool.Degenerated(E)) // no 3d curve
            {
                MinSurf = (Geom_Surface)MinSurf!.Transformed(MinLoc.Transformation());
                double max_deviation = 0.0;
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(FirstParOnPC) || global::Kerf.Engine.Foundation.Precision.IsInfinite(LastParOnPC))
                {
                    if (MinPC!.GetType() == typeof(Geom2d_Line))
                    {
                        bool IsLine = false;
                        if (MinSurf.GetType() == typeof(Geom_Plane))
                        {
                            IsLine = true;
                        }
                        else if (MinSurf.GetType() == typeof(Geom_CylindricalSurface)
                                 || MinSurf.GetType() == typeof(Geom_ConicalSurface))
                        {
                            Geom2d_Line theLine = (Geom2d_Line)MinPC;
                            Direction2D LineDir = theLine.Direction();
                            if (LineDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                IsLine = true;
                            }
                        }
                        if (IsLine)
                        {
                            Point2D P2d1 = MinPC.Value(0.0), P2d2 = MinPC.Value(1.0);
                            Point3D P1 = MinSurf.Value(P2d1.X(), P2d1.Y());
                            Point3D P2 = MinSurf.Value(P2d2.X(), P2d2.Y());
                            Vector3D aVec = new(P1, P2);
                            C3d = new Geom_Line(P1, new Direction3D(aVec));
                        }
                    }
                }
                else
                {
                    // OCCT copies the adaptors into handles; the copies are the adaptors themselves here
                    Geom2dAdaptor_Curve AC2d = new(MinPC!, FirstParOnPC, LastParOnPC);
                    GeomAdaptor_Surface GAsurf = new(MinSurf);
                    Geom2dAdaptor_Curve HC2d = AC2d;
                    GeomAdaptor_Surface HSurf = GAsurf;
                    Adaptor3d_CurveOnSurface ConS = new(HC2d, HSurf);
                    Continuity Continuity = C1;
                    int MaxDegree = 14;
                    int MaxSegment = evaluateMaxSegment(ConS);
                    GeomLib.BuildCurve3d(global::Kerf.Engine.Foundation.Precision.Confusion(),
                                         ConS,
                                         FirstParOnPC,
                                         LastParOnPC,
                                         ref C3d,
                                         out max_deviation,
                                         out _,
                                         Continuity,
                                         MaxDegree,
                                         MaxSegment);
                }
                BB.UpdateEdge(NE, C3d, max_deviation);
                // BB.Range( NE, FirstParOnPC, LastParOnPC );
                bool ProjectionSuccess = true;
                if (NbPCurves > 1)
                {
                    // BRepLib::SameParameter( NE, Precision::Confusion(), true );
                    for (itr.Initialize(((BRep_TEdge)NE.TShape()!).ChangeCurves()); itr.More(); itr.Next())
                    {
                        BRep_CurveRepresentation CurveRep = itr.Value();
                        double FirstPar, LastPar;
                        if (CurveRep.IsCurveOnSurface())
                        {
                            Geom2d_Curve theCurve = CurveRep.PCurve()!;
                            Geom_Surface theSurf = CurveRep.Surface()!;
                            Placement theLoc = CurveRep.Location();
                            // OCCT: MinSurf was replaced above by its transformed copy, so this test never holds and
                            // the pcurve the 3D curve was built from is checked (and maybe projected) as well
                            if (theCurve == MinPC && theSurf == MinSurf && theLoc == MinLoc)
                            {
                                continue;
                            }
                            FirstPar = ((BRep_GCurve)CurveRep).First();
                            LastPar = ((BRep_GCurve)CurveRep).Last();
                            if (Math.Abs(FirstPar - FirstParOnPC) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(LastPar - LastParOnPC) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                theLoc = E.Location() * theLoc;
                                theSurf = (Geom_Surface)theSurf.Transformed(theLoc.Transformation());

                                if (theCurve.GetType() == typeof(Geom2d_Line) && theSurf is Geom_BoundedSurface)
                                {
                                    Direction2D theDir = ((Geom2d_Line)theCurve).Direction();
                                    if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular())
                                        || theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                                    {
                                        theSurf.Bounds(out double U1, out double U2, out double V1, out double V2);
                                        Point2D Origin = ((Geom2d_Line)theCurve).Location();
                                        if (Math.Abs(Origin.X() - U1) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(Origin.X() - U2) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(Origin.Y() - V1) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(Origin.Y() - V2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            BRepLib.SameParameter(NE, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                                            break;
                                        }
                                    }
                                }
                                if (C3d is not null && FirstParOnPC < LastParOnPC)
                                {
                                    Geom2d_Curve? ProjPCurve =
                                        GeomProjLib.Curve2d(C3d, FirstParOnPC, LastParOnPC, theSurf);
                                    if (ProjPCurve is null)
                                    {
                                        ProjectionSuccess = false;
                                    }
                                    else
                                    {
                                        CurveRep.PCurve(ProjPCurve);
                                    }
                                }
                                else
                                {
                                    return false;
                                }
                            }
                        }
                    }
                }
                if (ProjectionSuccess)
                {
                    BB.Range(NE, FirstParOnPC, LastParOnPC);
                }
                else
                {
                    BB.Range(NE, FirstParOnPC, LastParOnPC, true);
                    BRepLib.SameParameter(NE, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                }
            }
        }
        else // no pcurves
        {
            double FirstPar = C3d!.FirstParameter();
            double LastPar = C3d.LastParameter();

            if (C3d is Geom_BoundedCurve && (FirstPar > anEf - a2Offset || LastPar < anEl + a2Offset))
            {
                Geom_TrimmedCurve aTrCurve = new(C3d, FirstPar, LastPar);

                // The curve is not prolonged on begin or end.
                // Trying to prolong it adding a segment to its bound.
                Point3D aPBnd;
                Vector3D aVBnd;
                Point3D aPBeg;
                Direction3D aDBnd = new();
                Geom_Line aLin;
                Geom_TrimmedCurve aSegment;
                GeomConvert_CompCurveToBSplineCurve aCompCurve = new(aTrCurve, RationalC1);
                double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
                double aDelta = Max(a2Offset, 1.0);

                if (FirstPar > anEf - a2Offset)
                {
                    C3d.D1(FirstPar, out aPBnd, out aVBnd);
                    aDBnd.SetXYZ(aVBnd.XYZ());
                    aPBeg = aPBnd.Translated(new Vector3D(-aDelta * aDBnd.XYZ()));
                    aLin = new Geom_Line(aPBeg, aDBnd);
                    aSegment = new Geom_TrimmedCurve(aLin, 0, aDelta);

                    if (!aCompCurve.Add(aSegment, aTol))
                    {
                        return true;
                    }
                }

                if (LastPar < anEl + a2Offset)
                {
                    C3d.D1(LastPar, out aPBeg, out aVBnd);
                    aDBnd.SetXYZ(aVBnd.XYZ());
                    aLin = new Geom_Line(aPBeg, aDBnd);
                    aSegment = new Geom_TrimmedCurve(aLin, 0, aDelta);

                    if (!aCompCurve.Add(aSegment, aTol))
                    {
                        return true;
                    }
                }

                C3d = aCompCurve.BSplineCurve()!;
                FirstPar = C3d.FirstParameter();
                LastPar = C3d.LastParameter();
                BB.UpdateEdge(NE, C3d, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else if (C3d.IsPeriodic())
            {
                double delta = (C3d.Period() - (anEl - anEf)) * 0.5;
                delta *= 0.95;
                FirstPar = anEf - delta;
                LastPar = anEl + delta;
            }
            else if (C3d.IsClosed())
            {
                LastPar -= 0.05 * (LastPar - FirstPar);
            }

            BB.Range(NE, FirstPar, LastPar);
        }
        return true;
    }

    //  Modified by skv - Fri Dec 26 17:00:57 2003 OCC4455 End

    //=================================================================================================

    private static bool UpdateVertex(in TopoDS_Vertex V, in TopoDS_Edge OE, in TopoDS_Edge NE, double TolConf)
    {
        BRepAdaptor_Curve OC = new(OE);
        BRepAdaptor_Curve NC = new(NE);
        double Of = OC.FirstParameter();
        double Ol = OC.LastParameter();
        double Nf = NC.FirstParameter();
        double Nl = NC.LastParameter();
        double U = 0.0;
        double ParTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point3D P = BRep_Tool.Pnt(V);
        bool OK = false;

        if (P.Distance(OC.Value(Of)) < TolConf)
        {
            if (Of >= Nf + ParTol && Of <= Nl + ParTol && P.Distance(NC.Value(Of)) < TolConf)
            {
                OK = true;
                U = Of;
            }
        }
        if (P.Distance(OC.Value(Ol)) < TolConf)
        {
            if (Ol >= Nf + ParTol && Ol <= Nl + ParTol && P.Distance(NC.Value(Ol)) < TolConf)
            {
                OK = true;
                U = Ol;
            }
        }
        if (OK)
        {
            BRep_Builder B = new();
            // OCCT also makes the forward edge EE here, unused
            //    TopoDS_Edge EE = TopoDS::Edge(NE.Oriented(TopAbs_FORWARD));
            TopoDS_Shape aLocalShape = V.Oriented(TopAbs_INTERNAL);
            B.UpdateVertex(TopoDS.Vertex(aLocalShape), U, NE, BRep_Tool.Tolerance(NE));
            //    B.UpdateVertex(TopoDS::Vertex(V.Oriented(TopAbs_INTERNAL)),
            //                   U,NE,BRep_Tool::Tolerance(NE));
        }
        return OK;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the intersections between the edges stored in <paramref name="AsDes"/> as descendants of
    /// <paramref name="F"/>: two edges are intersected if one of them is in <paramref name="NewEdges"/>. When all faces
    /// of the shape are treated, the intersection vertices have to be fused with <see cref="FuseVertices"/>;
    /// <paramref name="theDMVV"/> collects the vertices to fuse.
    /// </summary>
    public static void Compute(BRepAlgo_AsDes AsDes,
                               in TopoDS_Face F,
                               NCollection_IndexedMap<TopoDS_Shape> NewEdges,
                               double Tol,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgeIntEdges,
                               NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV,
                               ProgressRange theRange = default)
    {

        // Do not intersect the edges of face
        NCollection_Map<TopoDS_Shape> EdgesOfFace = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer Explo = new(F, TopAbs_EDGE);
        for (; Explo.More(); Explo.Next())
        {
            EdgesOfFace.Add(Explo.Current());
        }

        //-----------------------------------------------------------
        // calculate intersections2d on faces touched by
        // intersection3d
        //---------------------------------------------------------
        NCollection_List<TopoDS_Shape>.Iterator it1LE = new();
        NCollection_List<TopoDS_Shape>.Iterator it2LE = new();

        //-----------------------------------------------
        // Intersection of edges 2*2.
        //-----------------------------------------------
        NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(F);
        int j, i = 1;
        BRepAdaptor_Surface BAsurf = new(F);
        //
        using ProgressScope aPS = new(theRange, "Intersecting edges on faces", LE.Extent());
        for (it1LE.Initialize(LE); it1LE.More(); it1LE.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Edge E1 = TopoDS.Edge(it1LE.Value());
            j = 1;
            it2LE.Initialize(LE);

            while (j < i && it2LE.More())
            {
                TopoDS_Edge E2 = TopoDS.Edge(it2LE.Value());

                bool ToIntersect = true;
                if (theEdgeIntEdges.IsBound(E1))
                {
                    NCollection_List<TopoDS_Shape> aElist = theEdgeIntEdges[E1];
                    NCollection_List<TopoDS_Shape>.Iterator itedges = new(aElist);
                    for (; itedges.More(); itedges.Next())
                    {
                        if (E2.IsSame(itedges.Value()))
                        {
                            ToIntersect = false;
                        }
                    }

                    if (ToIntersect)
                    {
                        for (itedges.Initialize(aElist); itedges.More(); itedges.Next())
                        {
                            TopoDS_Shape anEdge = itedges.Value();
                            if (theEdgeIntEdges.IsBound(anEdge))
                            {
                                NCollection_List<TopoDS_Shape> aElist2 = theEdgeIntEdges[anEdge];
                                NCollection_List<TopoDS_Shape>.Iterator itedges2 = new(aElist2);
                                for (; itedges2.More(); itedges2.Next())
                                {
                                    if (E2.IsSame(itedges2.Value()))
                                    {
                                        ToIntersect = false;
                                    }
                                }
                            }
                        }
                    }
                }

                //--------------------------------------------------------------
                // Intersections of New edges obtained by intersection
                // between them and with edges of restrictions
                //------------------------------------------------------
                if (ToIntersect && (!EdgesOfFace.Contains(E1) || !EdgesOfFace.Contains(E2))
                    && (NewEdges.Contains(E1) || NewEdges.Contains(E2)))
                {

                    TopoDS_Shape aLocalShape = F.Oriented(TopAbs_FORWARD);
                    EdgeInter(TopoDS.Face(aLocalShape), BAsurf, E1, E2, AsDes, Tol, true, theDMVV);
                    //          EdgeInter(TopoDS::Face(F.Oriented(TopAbs_FORWARD)),E1,E2,AsDes,Tol,true);
                }
                it2LE.Next();
                j++;
            }
            i++;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Computes the intersections between the offset edges of the face <paramref name="FI"/> (whose offset is
    /// <paramref name="OFI"/>); the intersection vertices are stored in <paramref name="AsDes2d"/>. When all faces of
    /// the shape are treated, they have to be fused with <see cref="FuseVertices"/>; <paramref name="theDMVV"/>
    /// collects the vertices to fuse. False if an edge couldn't be extended or the computation was stopped.
    /// </summary>
    public static bool ConnexIntByInt(in TopoDS_Face FI,
                                      BRepOffset_Offset OFI,
                                      NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MES,
                                      NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Build,
                                      BRepAlgo_AsDes theAsDes,
                                      BRepAlgo_AsDes AsDes2d,
                                      double Offset,
                                      double Tol,
                                      BRepOffset_Analyse Analyse,
                                      NCollection_IndexedMap<TopoDS_Shape> FacesWithVerts,
                                      BRepAlgo_Image theImageVV,
                                      NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgeIntEdges,
                                      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV,
                                      ProgressRange theRange = default)
    {

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE = new(TopTools_ShapeMapHasher.Instance);
        BRepOffset_Tool.MapVertexEdges(FI, MVE);
        using ProgressScope aPS = new(theRange, "Intersecting edges obtained as intersection of faces", 1, true);
        //---------------------
        // Extension of edges.
        //---------------------
        TopoDS_Edge NE;
        // OCCT.NET: MVE is keyed by shape and iterates in insertion order (OCCT's in hash order); the order only
        // decides the order in which the edges are extended and bound in MES
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> it in MVE)
        {
            if (!aPS.More())
            {
                return false;
            }
            NCollection_List<TopoDS_Shape> L = it.Value;
            bool YaBuild = false;
            NCollection_List<TopoDS_Shape>.Iterator itL = new(L);
            for (; itL.More(); itL.Next())
            {
                YaBuild = Build.IsBound(itL.Value());
                if (YaBuild)
                {
                    break;
                }
            }
            if (YaBuild)
            {
                for (itL.Initialize(L); itL.More(); itL.Next())
                {
                    TopoDS_Edge EI = TopoDS.Edge(itL.Value());
                    if (EI.Orientation() != TopAbs_FORWARD && EI.Orientation() != TopAbs_REVERSED)
                    {
                        continue;
                    }
                    TopoDS_Shape aLocalShape = OFI.InternalGenerated(EI);
                    TopoDS_Edge OE = TopoDS.Edge(aLocalShape);
                    if (!MES.IsBound(OE) && !Build.IsBound(EI))
                    {
                        if (!ExtentEdge(OE, out NE, Offset))
                        {
                            return false;
                        }
                        MES.Bind(OE, NE);
                    }
                }
            }
        }

        TopoDS_Face FIO = OFI.InternalFace();
        if (MES.IsBound(FIO))
        {
            FIO = TopoDS.Face(MES[FIO]);
        }
        //
        BRepAdaptor_Surface BAsurf = new(FIO);

        TopExp_Explorer exp = new(FI.Oriented(TopAbs_FORWARD), TopAbs_WIRE);
        for (; exp.More(); exp.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return false;
            }
            TopoDS_Wire W = TopoDS.Wire(exp.Current());
            BRepTools_WireExplorer wexp = new();
            bool end = false;
            TopoDS_Edge FirstE, CurE, NextE;

            TopoDS_Shape aLocalWire = W.Oriented(TopAbs_FORWARD);
            TopoDS_Shape aLocalFace = FI.Oriented(TopAbs_FORWARD);
            wexp.Init(TopoDS.Wire(aLocalWire), TopoDS.Face(aLocalFace));
            if (!wexp.More())
            {
                continue; // Protection from case when explorer does not contain edges.
            }
            CurE = FirstE = wexp.Current();
            // OCCT declares an unused indexed map of edges here

            while (!end)
            {
                wexp.Next();
                if (wexp.More())
                {
                    NextE = wexp.Current();
                }
                else
                {
                    NextE = FirstE;
                    end = true;
                }
                if (CurE.IsSame(NextE))
                {
                    continue;
                }

                TopoDS_Vertex Vref = CommonVertex(CurE, NextE);

                CurE = Analyse.EdgeReplacement(FI, CurE);
                NextE = Analyse.EdgeReplacement(FI, NextE);

                TopoDS_Shape aLocalShape = OFI.InternalGenerated(CurE);
                TopoDS_Edge CEO = TopoDS.Edge(aLocalShape);
                aLocalShape = OFI.InternalGenerated(NextE);
                TopoDS_Edge NEO = TopoDS.Edge(aLocalShape);
                //------------------------------------------
                // Inter processing of images of CurE NextE.
                //------------------------------------------
                bool DoInter = true;
                TopoDS_Shape NE1 = new(), NE2 = new();
                NCollection_Sequence<TopoDS_Shape> NE1seq = new(), NE2seq = new();
                TopAbs_Orientation anOr1 = TopAbs_EXTERNAL, anOr2 = TopAbs_EXTERNAL;

                int aChoice = 0;
                if (Build.IsBound(CurE) && Build.IsBound(NextE))
                {
                    aChoice = 1;
                    NE1 = Build[CurE];
                    NE2 = Build[NextE];
                    GetEdgesOrientedInFace(NE1, FIO, theAsDes, NE1seq);
                    GetEdgesOrientedInFace(NE2, FIO, theAsDes, NE2seq);
                    anOr1 = TopAbs_REVERSED;
                    anOr2 = TopAbs_FORWARD;
                }
                else if (Build.IsBound(CurE) && MES.IsBound(NEO))
                {
                    aChoice = 2;
                    NE1 = Build[CurE];
                    NE2 = MES[NEO];
                    NE2.Orientation(NextE.Orientation());
                    GetEdgesOrientedInFace(NE1, FIO, theAsDes, NE1seq);
                    NE2seq.Append(NE2);
                    anOr1 = TopAbs_REVERSED;
                    anOr2 = TopAbs_FORWARD;
                }
                else if (Build.IsBound(NextE) && MES.IsBound(CEO))
                {
                    aChoice = 3;
                    NE1 = Build[NextE];
                    NE2 = MES[CEO];
                    NE2.Orientation(CurE.Orientation());
                    GetEdgesOrientedInFace(NE1, FIO, theAsDes, NE1seq);
                    NE2seq.Append(NE2);
                    anOr1 = TopAbs_FORWARD;
                    anOr2 = TopAbs_REVERSED;
                }
                else
                {
                    DoInter = false;
                }
                if (DoInter)
                {
                    //------------------------------------
                    // NE1,NE2 can be a compound of Edges.
                    //------------------------------------
                    TopoDS_Edge aE1, aE2;
                    if (aChoice == 1 || aChoice == 2)
                    {
                        aE1 = TopoDS.Edge(NE1seq.Last());
                        aE2 = TopoDS.Edge(NE2seq.First());
                    }
                    else // aChoice == 3
                    {
                        aE1 = TopoDS.Edge(NE1seq.First());
                        aE2 = TopoDS.Edge(NE2seq.Last());
                    }

                    if (aE1.Orientation() == TopAbs_REVERSED)
                    {
                        anOr1 = TopAbs.Reverse(anOr1);
                    }
                    if (aE2.Orientation() == TopAbs_REVERSED)
                    {
                        anOr2 = TopAbs.Reverse(anOr2);
                    }

                    RefEdgeInter(FIO,
                                 BAsurf,
                                 aE1,
                                 aE2,
                                 anOr1,
                                 anOr2,
                                 AsDes2d,
                                 Tol,
                                 true,
                                 Vref,
                                 theImageVV,
                                 theDMVV,
                                 out _);

                    if (theEdgeIntEdges.IsBound(aE1))
                    {
                        theEdgeIntEdges[aE1].Append(aE2);
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> aElist = new();
                        aElist.Append(aE2);
                        theEdgeIntEdges.Bind(aE1, aElist);
                    }
                    if (theEdgeIntEdges.IsBound(aE2))
                    {
                        theEdgeIntEdges[aE2].Append(aE1);
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> aElist = new();
                        aElist.Append(aE1);
                        theEdgeIntEdges.Bind(aE2, aElist);
                    }

                    //
                    // check if some of the offset edges have been
                    // generated out of the common vertex
                    if (Build.IsBound(Vref))
                    {
                        FacesWithVerts.Add(FI);
                    }
                }
                else
                {
                    TopoDS_Vertex V = CommonVertex(CEO, NEO);
                    if (!V.IsNull())
                    {
                        if (MES.IsBound(CEO))
                        {
                            UpdateVertex(V, CEO, TopoDS.Edge(MES[CEO]), Tol);
                            AsDes2d.Add(MES[CEO], V);
                        }
                        if (MES.IsBound(NEO))
                        {
                            UpdateVertex(V, NEO, TopoDS.Edge(MES[NEO]), Tol);
                            AsDes2d.Add(MES[NEO], V);
                        }
                    }
                }
                CurE = wexp.Current();
            }
        }
        return true;
    }

    //=======================================================================
    // function : ConnexIntByIntInVert
    // purpose  : Intersection of the edges generated out of vertices
    //=======================================================================

    /// <summary>
    /// Computes the intersections between the offset edges generated from vertices and stored in
    /// <paramref name="AsDes"/> as descendants of the offset of <paramref name="FI"/>; the intersection vertices are
    /// stored in <paramref name="AsDes2d"/>. When all faces of the shape are treated, they have to be fused with
    /// <see cref="FuseVertices"/>; <paramref name="theDMVV"/> collects the vertices to fuse.
    /// </summary>
    public static void ConnexIntByIntInVert(in TopoDS_Face FI,
                                            BRepOffset_Offset OFI,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MES,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Build,
                                            BRepAlgo_AsDes AsDes,
                                            BRepAlgo_AsDes AsDes2d,
                                            double Tol,
                                            BRepOffset_Analyse Analyse,
                                            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV,
                                            ProgressRange theRange = default)
    {
        TopoDS_Face FIO = OFI.InternalFace();
        if (MES.IsBound(FIO))
        {
            FIO = TopoDS.Face(MES[FIO]);
        }
        //
        NCollection_Map<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLE = AsDes.Descendant(FIO);
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aE = aItLE.Value();
            aME.Add(aE);
        }
        //
        BRepAdaptor_Surface BAsurf = new(FIO);
        //
        using ProgressScope aPS = new(theRange, "Intersecting edges created from vertices", 1, true);
        TopExp_Explorer exp = new(FI.Oriented(TopAbs_FORWARD), TopAbs_WIRE);
        for (; exp.More(); exp.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Wire W = TopoDS.Wire(exp.Current());
            //
            BRepTools_WireExplorer wexp = new();
            bool end = false;
            TopoDS_Edge FirstE, CurE, NextE;
            //
            TopoDS_Shape aLocalWire = W.Oriented(TopAbs_FORWARD);
            TopoDS_Shape aLocalFace = FI.Oriented(TopAbs_FORWARD);
            wexp.Init(TopoDS.Wire(aLocalWire), TopoDS.Face(aLocalFace));
            if (!wexp.More())
            {
                continue; // Protection from case when explorer does not contain edges.
            }
            //
            CurE = FirstE = wexp.Current();
            while (!end)
            {
                wexp.Next();
                if (wexp.More())
                {
                    NextE = wexp.Current();
                }
                else
                {
                    NextE = FirstE;
                    end = true;
                }
                if (CurE.IsSame(NextE))
                {
                    continue;
                }
                //
                TopoDS_Vertex Vref = CommonVertex(CurE, NextE);
                if (!Build.IsBound(Vref))
                {
                    CurE = NextE;
                    continue;
                }

                CurE = Analyse.EdgeReplacement(FI, CurE);
                NextE = Analyse.EdgeReplacement(FI, NextE);

                TopoDS_Shape aLocalShape = OFI.InternalGenerated(CurE);
                TopoDS_Edge CEO = TopoDS.Edge(aLocalShape);
                aLocalShape = OFI.InternalGenerated(NextE);
                TopoDS_Edge NEO = TopoDS.Edge(aLocalShape);
                //
                TopoDS_Shape NE1, NE2;
                TopAbs_Orientation anOr1 = TopAbs_EXTERNAL, anOr2 = TopAbs_EXTERNAL;

                if (Build.IsBound(CurE) && Build.IsBound(NextE))
                {
                    NE1 = Build[CurE];
                    NE2 = Build[NextE];
                }
                else if (Build.IsBound(CurE) && MES.IsBound(NEO))
                {
                    NE1 = Build[CurE];
                    NE2 = MES[NEO];
                }
                else if (Build.IsBound(NextE) && MES.IsBound(CEO))
                {
                    NE1 = Build[NextE];
                    NE2 = MES[CEO];
                }
                else
                {
                    CurE = wexp.Current();
                    continue;
                }
                //
                TopExp_Explorer Exp1 = new(), Exp2 = new();
                bool bCoincide;
                // intersect edges generated from vertex with the edges of the face
                TopoDS_Shape NE3 = Build[Vref];
                //
                for (Exp2.Init(NE3, TopAbs_EDGE); Exp2.More(); Exp2.Next())
                {
                    TopoDS_Edge aE3 = TopoDS.Edge(Exp2.Current());
                    if (!aME.Contains(aE3))
                    {
                        continue;
                    }
                    //
                    // intersection with first edge
                    for (Exp1.Init(NE1, TopAbs_EDGE); Exp1.More(); Exp1.Next())
                    {
                        TopoDS_Edge aE1 = TopoDS.Edge(Exp1.Current());
                        BRepAlgo_Image anEmptyImage = new();
                        RefEdgeInter(FIO,
                                     BAsurf,
                                     aE1,
                                     aE3,
                                     anOr1,
                                     anOr2,
                                     AsDes2d,
                                     Tol,
                                     true,
                                     Vref,
                                     anEmptyImage,
                                     theDMVV,
                                     out bCoincide);
                        if (bCoincide)
                        {
                            // in case of coincidence trim the edge E3 the same way as E1
                            Store(aE3, AsDes2d.Descendant(aE1), Tol, true, AsDes2d, theDMVV);
                        }
                    }
                    //
                    // intersection with second edge
                    for (Exp1.Init(NE2, TopAbs_EDGE); Exp1.More(); Exp1.Next())
                    {
                        TopoDS_Edge aE2 = TopoDS.Edge(Exp1.Current());
                        BRepAlgo_Image anEmptyImage = new();
                        RefEdgeInter(FIO,
                                     BAsurf,
                                     aE2,
                                     aE3,
                                     anOr1,
                                     anOr2,
                                     AsDes2d,
                                     Tol,
                                     true,
                                     Vref,
                                     anEmptyImage,
                                     theDMVV,
                                     out bCoincide);
                        if (bCoincide)
                        {
                            // in case of coincidence trim the edge E3 the same way as E2
                            Store(aE3, AsDes2d.Descendant(aE2), Tol, true, AsDes2d, theDMVV);
                        }
                    }
                    //
                    // intersection of the edges generated from vertex
                    // among themselves
                    for (Exp1.Init(NE3, TopAbs_EDGE); Exp1.More(); Exp1.Next())
                    {
                        if (aE3.IsSame(Exp1.Current()))
                        {
                            break;
                        }
                    }
                    //
                    for (Exp1.Next(); Exp1.More(); Exp1.Next())
                    {
                        TopoDS_Edge aE3Next = TopoDS.Edge(Exp1.Current());
                        if (aME.Contains(aE3Next))
                        {
                            BRepAlgo_Image anEmptyImage = new();
                            RefEdgeInter(FIO,
                                         BAsurf,
                                         aE3Next,
                                         aE3,
                                         anOr1,
                                         anOr2,
                                         AsDes2d,
                                         Tol,
                                         true,
                                         Vref,
                                         anEmptyImage,
                                         theDMVV,
                                         out bCoincide);
                        }
                    }
                }
                CurE = wexp.Current();
            }
        }
    }

    //=================================================================================================

    private static void MakeChain(in TopoDS_Shape theV,
                                  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV,
                                  NCollection_Map<TopoDS_Shape> theMDone,
                                  NCollection_List<TopoDS_Shape> theChain)
    {
        if (theMDone.Add(theV))
        {
            theChain.Append(theV);
            int anInd = theDMVV.FindIndex(theV);
            if (anInd != 0)
            {
                NCollection_List<TopoDS_Shape> pLV = theDMVV.FindFromIndex(anInd);
                NCollection_List<TopoDS_Shape>.Iterator aIt = new(pLV);
                for (; aIt.More(); aIt.Next())
                {
                    MakeChain(aIt.Value(), theDMVV, theMDone, theChain);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Fuses the chains of coinciding vertices of <paramref name="theDMVV"/> and updates
    /// <paramref name="theAsDes"/>, replacing the old vertices with the new ones. False if a vertex has no parameter
    /// on one of its edges.
    /// </summary>
    public static bool FuseVertices(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVV,
                                    BRepAlgo_AsDes theAsDes,
                                    BRepAlgo_Image theImageVV)
    {
        BRep_Builder aBB = new();
        NCollection_Map<TopoDS_Shape> aMVDone = new(TopTools_ShapeMapHasher.Instance);
        int i, aNb = theDMVV.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(theDMVV.FindKey(i));
            //
            // find chain of vertices
            NCollection_List<TopoDS_Shape> aLVChain = new();
            MakeChain(aV, theDMVV, aMVDone, aLVChain);
            //
            if (aLVChain.Extent() < 2)
            {
                continue;
            }
            //
            // make new vertex
            TopoDS_Vertex aVNew = new();
            BOPTools_AlgoTools.MakeVertex(aLVChain, ref aVNew);
            //
            TopoDS_Vertex aVNewInt = TopoDS.Vertex(aVNew.Oriented(TopAbs_INTERNAL));
            //
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLVChain);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aVOld = aIt.Value();
                // update the parameters on edges
                TopoDS_Vertex aVOldInt = TopoDS.Vertex(aVOld.Oriented(TopAbs_INTERNAL));
                NCollection_List<TopoDS_Shape> aLE = theAsDes.Ascendant(aVOld);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
                for (; aItLE.More(); aItLE.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(aItLE.Value());
                    double aTolE = BRep_Tool.Tolerance(aE);
                    if (!BRep_Tool.Parameter(aVOldInt, aE, out double aT))
                    {
                        return false;
                    }
                    aBB.UpdateVertex(aVNewInt, aT, aE, aTolE);
                }
                // and replace the vertex
                theAsDes.Replace(aVOld, aVNew);
                if (theImageVV.IsImage(aVOld))
                {
                    TopoDS_Vertex aProVertex = TopoDS.Vertex(theImageVV.ImageFrom(aVOld));
                    theImageVV.Add(aProVertex, aVNew.Oriented(TopAbs_FORWARD));
                }
            }
        }
        return true;
    }
}
