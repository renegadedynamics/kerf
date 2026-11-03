// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Wire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepCheck_Status;
using static Kerf.Geometry.CurveKind;
using static Occt.IntRes2d_Position;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The checks of a wire: not empty and connected (<see cref="Minimum"/>), closed (<see cref="Closed"/>), closed in
/// the parametric space of a face (<see cref="Closed2d"/>), coherently oriented (<see cref="Orientation"/>) and
/// without self-intersection on a face (<see cref="SelfIntersect"/>).
/// </summary>
public class BRepCheck_Wire : BRepCheck_Result
{
    private bool myCdone;
    private BRepCheck_Status myCstat;

    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMapVE =
        new(TopTools_ShapeMapHasher.Instance);

    private bool myGctrl;

    /// <summary>Checks the wire on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Wire(in TopoDS_Wire W)
    {
        myCdone = false;
        myCstat = BRepCheck_NoError;
        myGctrl = false;
        Init(W);
    }

    private static bool IsOriented(in TopoDS_Shape S)
        => S.Orientation() == TopAbs_FORWARD || S.Orientation() == TopAbs_REVERSED;

    /// <summary>The status list of the wire itself, read under the lock in parallel mode.</summary>
    private NCollection_List<BRepCheck_Status> lockedStatusList()
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            return myMap.Find(myShape);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>Checks that the wire has edges and that they are connected through their vertices.</summary>
    public override void Minimum()
    {
        myCdone = false;
        myGctrl = true;
        if (!myMin)
        {
            var aNewList = new NCollection_List<BRepCheck_Status>();
            NCollection_List<BRepCheck_Status> lst = myMap.Bound(myShape, aNewList);

            // check that the wire is "connex"
            var exp = new TopExp_Explorer(myShape, TopAbs_EDGE);
            int nbedge = 0;
            myMapVE.Clear();
            // fill myMapVE
            for (; exp.More(); exp.Next())
            {
                nbedge++;
                var expv = new TopExp_Explorer();
                for (expv.Init(exp.Current(), TopAbs_VERTEX); expv.More(); expv.Next())
                {
                    TopoDS_Shape vtx = expv.Current();
                    int index = myMapVE.FindIndex(vtx);
                    if (index == 0)
                    {
                        var theListOfShape = new NCollection_List<TopoDS_Shape>();
                        index = myMapVE.Add(vtx, theListOfShape);
                    }

                    myMapVE[index].Append(exp.Current());
                }
            }

            // wire must have at least one edge
            if (nbedge == 0)
            {
                BRepCheck.Add(lst, BRepCheck_EmptyWire);
            }
            // check if all edges are connected through vertices
            else if (nbedge >= 2)
            {
                var mapE = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                exp.ReInit();
                Propagate(myMapVE, exp.Current(), mapE);
                for (exp.ReInit(); exp.More(); exp.Next())
                {
                    if (!mapE.Contains(exp.Current()))
                    {
                        BRepCheck.Add(lst, BRepCheck_NotConnected);
                        break;
                    }
                }
            }

            if (lst.IsEmpty())
            {
                lst.Append(BRepCheck_NoError);
            }

            myMapVE.Clear();
            myMin = true;
        }
    }

    /// <summary>
    /// Checks the wire in a shape that contains it: in a face, <see cref="SelfIntersect"/> (with the geometric
    /// controls), <see cref="Closed"/>, <see cref="Orientation"/> and <see cref="Closed2d"/>, until one fails.
    /// </summary>
    public override void InContext(in TopoDS_Shape S)
    {
        NCollection_List<BRepCheck_Status> aHList;
        {
            bool aLocked = myIsParallel;
            if (aLocked)
            {
                myMutex.Enter();
            }

            try
            {
                if (myMap.IsBound(S))
                {
                    return;
                }

                var aNewList = new NCollection_List<BRepCheck_Status>();
                aHList = myMap.Bound(S, aNewList);
            }
            finally
            {
                if (aLocked)
                {
                    myMutex.Exit();
                }
            }
        }

        NCollection_List<BRepCheck_Status> lst = aHList;

        // check if my wire is in <S>
        var exp = new TopExp_Explorer(S, TopAbs_WIRE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(myShape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheck.Add(lst, BRepCheck_SubshapeNotInShape);
            return;
        }

        BRepCheck_Status st = BRepCheck_NoError;
        TopAbs_ShapeEnum styp = S.ShapeType();
        switch (styp)
        {
            case TopAbs_FACE:
            {
                TopoDS_Edge ed1 = new(), ed2 = new();
                if (myGctrl)
                {
                    st = SelfIntersect(TopoDS.Face(S), ref ed1, ref ed2, true);
                }

                if (st != BRepCheck_NoError)
                {
                    break;
                }

                st = Closed();
                if (st != BRepCheck_NoError)
                {
                    break;
                }

                st = Orientation(TopoDS.Face(S));
                if (st != BRepCheck_NoError)
                {
                    break;
                }

                st = Closed2d(TopoDS.Face(S));
                break;
            }
            default:
                break;
        }

        if (st != BRepCheck_NoError)
        {
            BRepCheck.Add(lst, st);
        }

        if (lst.IsEmpty())
        {
            lst.Append(BRepCheck_NoError);
        }
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!myBlind)
        {
            // nothing more that the minimum
            myBlind = true;
        }
    }

    /// <summary>
    /// Checks that the oriented edges of the wire give a closed wire: <see cref="BRepCheck_NotConnected"/>,
    /// <see cref="BRepCheck_RedundantEdge"/> (an edge more than twice, or twice not once FORWARD and once
    /// REVERSED), <see cref="BRepCheck_NotClosed"/> or <see cref="BRepCheck_NoError"/>. If the first and last edges
    /// are infinite, the wire is considered closed. With <paramref name="Update"/>, the status is added to
    /// <see cref="BRepCheck_Result.Status"/>.
    /// </summary>
    public BRepCheck_Status Closed(bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();
        if (myCdone)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myCstat);
            }

            return myCstat;
        }

        myCdone = true;

        var itl = new NCollection_List<BRepCheck_Status>.Iterator(aStatusList);
        if (itl.Value() != BRepCheck_NoError)
        {
            myCstat = itl.Value();
            return myCstat; // already saved
        }

        myCstat = BRepCheck_NoError;

        var exp = new TopExp_Explorer();
        var expv = new TopExp_Explorer();
        var mapS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var Cradoc = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        myMapVE.Clear();
        // Checks if the oriented edges of the wire give a "closed" wire,
        // i-e if each oriented vertex on oriented edges is found 2 times...
        for (exp.Init(myShape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (IsOriented(exp.Current()))
            {
                if (!Cradoc.IsBound(exp.Current()))
                {
                    var theListOfShape = new NCollection_List<TopoDS_Shape>();
                    Cradoc.Bind(exp.Current(), theListOfShape);
                }

                Cradoc.Find(exp.Current()).Append(exp.Current());

                mapS.Add(exp.Current());
                for (expv.Init(exp.Current(), TopAbs_VERTEX); expv.More(); expv.Next())
                {
                    if (IsOriented(expv.Current()))
                    {
                        int index = myMapVE.FindIndex(expv.Current());
                        if (index == 0)
                        {
                            var theListOfShape1 = new NCollection_List<TopoDS_Shape>();
                            index = myMapVE.Add(expv.Current(), theListOfShape1);
                        }

                        myMapVE[index].Append(exp.Current());
                    }
                }
            }
        }

        int theNbori = mapS.Extent();
        if (theNbori >= 2)
        {
            mapS.Clear();
            for (exp.ReInit(); exp.More(); exp.Next())
            {
                if (IsOriented(exp.Current()))
                {
                    break;
                }
            }

            Propagate(myMapVE, exp.Current(), mapS);
        }

        if (theNbori != mapS.Extent())
        {
            myCstat = BRepCheck_NotConnected;
            if (Update)
            {
                BRepCheck.Add(aStatusList, myCstat);
            }

            return myCstat;
        }

        // Checks the number of occurrence of an edge : maximum 2, and in this
        // case, one time FORWARD and one time REVERSED

        // OCCT.NET: the map iterates in another order than OCCT's; the result doesn't depend on it
        bool yabug = false;
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in Cradoc)
        {
            if (itdm.Value.Extent() >= 3)
            {
                yabug = true;
            }
            else if (itdm.Value.Extent() == 2)
            {
                if (itdm.Value.First().Orientation() == itdm.Value.Last().Orientation())
                {
                    yabug = true;
                }
            }

            if (yabug)
            {
                myCstat = BRepCheck_RedundantEdge;
                if (Update)
                {
                    BRepCheck.Add(aStatusList, myCstat);
                }

                return myCstat;
            }
        }

        for (int i = 1; i <= myMapVE.Extent(); i++)
        {
            if (myMapVE[i].Extent() % 2 != 0)
            {
                myCstat = BRepCheck_NotClosed;
                if (Update)
                {
                    BRepCheck.Add(aStatusList, myCstat);
                }

                return myCstat;
            }
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, myCstat);
        }

        return myCstat;
    }

    // Return true if distance between thePnt_f and thePnt_l is not more, than aTol3d
    private static bool IsDistanceIn3DTolerance(in Point3D thePnt_f, in Point3D thePnt_l, double aTol3d)
    {
        double Dist = thePnt_f.Distance(thePnt_l);

        return Dist < aTol3d;
    }

    // OCCT's PrintWarnings argument only prints in debug builds; it is left out.
    private static bool IsDistanceIn2DTolerance(BRepAdaptor_Surface aFaceSurface,
                                                in Point2D thePnt,
                                                in Point2D thePntRef,
                                                double aTol3d)
    {
        double dumax = 0.01 * (aFaceSurface.LastUParameter() - aFaceSurface.FirstUParameter());
        double dvmax = 0.01 * (aFaceSurface.LastVParameter() - aFaceSurface.FirstVParameter());
        double dumin = Math.Abs(thePnt.X() - thePntRef.X());
        double dvmin = Math.Abs(thePnt.Y() - thePntRef.Y());

        if ((dumin < dumax) && (dvmin < dvmax))
        {
            return true;
        }

        dumax = aFaceSurface.UResolution(aTol3d);
        dvmax = aFaceSurface.VResolution(aTol3d);
        double um = (thePnt.X() + thePntRef.X()) / 2.0;
        double vm = (thePnt.Y() + thePntRef.Y()) / 2.0;
        aFaceSurface.D1(um, vm, out Point3D _, out Vector3D aDU, out Vector3D aDV);
        double aMDU = aDU.Magnitude();
        if (aMDU > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            dumax = Max((aTol3d / aMDU), dumax);
        }

        double aMDV = aDV.Magnitude();
        if (aMDV > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            dvmax = Max((aTol3d / aMDV), dvmax);
        }

        double aTol2d = 2 * Max(dumax, dvmax);

        double Dist = Max(dumin, dvmin);

        return Dist < aTol2d;
    }

    /// <summary>
    /// Checks that the edges of the wire give a wire closed in the parametric space of <paramref name="theFace"/>
    /// (and closed in 3D, <see cref="Closed"/>): <see cref="BRepCheck_NoError"/> or <see cref="BRepCheck_NotClosed"/>.
    /// With <paramref name="Update"/>, the status is added to <see cref="BRepCheck_Result.Status"/>.
    /// </summary>
    public BRepCheck_Status Closed2d(in TopoDS_Face theFace, bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();

        // 3d closure checked too
        BRepCheck_Status aClosedStat = Closed();
        if (aClosedStat != BRepCheck_NoError)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }

        // 20/03/02 akm vvv : (OCC234) Hence this method will be used to check
        //                    both periodic and non-periodic faces
        var aFaceSurface = new BRepAdaptor_Surface(theFace, false);

        // count edges having FORWARD or REVERSED orientation
        int aNbOrirntedEdges = 0;
        var anEdgeExp = new TopExp_Explorer(myShape, TopAbs_EDGE);
        for (; anEdgeExp.More(); anEdgeExp.Next())
        {
            if (IsOriented(anEdgeExp.Current()))
            {
                aNbOrirntedEdges++;
            }
        }

        if (aNbOrirntedEdges == 0)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }

        // all those edges must form a closed 2d contour and be found by WireExplorer

        int aNbFoundEdges = 0;
        var aWireExp = new BRepTools_WireExplorer(TopoDS.Wire(myShape), theFace);
        TopoDS_Edge aFirstEdge = aWireExp.Current();
        TopoDS_Vertex aFirstVertex = aWireExp.CurrentVertex();
        TopoDS_Edge aLastEdge = new();

        for (; aWireExp.More(); aWireExp.Next())
        {
            aNbFoundEdges++;
            aLastEdge = aWireExp.Current();
        }

        if (aNbFoundEdges != aNbOrirntedEdges)
        {
            aClosedStat = BRepCheck_NotClosed;
            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }

        // Check distance between 2d ends of first and last edges
        //   First check if first and last edges are infinite:
        bool isFirstInfinite = false;
        bool isLastInfinite = false;
        TopAbs_Orientation anOri;

        anOri = aFirstEdge.Orientation();
        BRep_Tool.Range(aFirstEdge, out double aF, out double aL);
        if ((anOri == TopAbs_FORWARD && global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aF))
            || (anOri == TopAbs_REVERSED && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aL)))
        {
            isFirstInfinite = true;
        }

        anOri = aLastEdge.Orientation();
        BRep_Tool.Range(aLastEdge, out aF, out aL);

        if ((anOri == TopAbs_FORWARD && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aL))
            || (anOri == TopAbs_REVERSED && global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aF)))
        {
            isLastInfinite = true;
        }

        if (isFirstInfinite && isLastInfinite)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }
        else if (aFirstVertex.IsNull())
        {
            aClosedStat = BRepCheck_NotClosed;

            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }

        Point2D aP_first, aP_last, aP_temp; // ends of prev edge, next edge, bidon

        // get last point
        BRep_Tool.UVPoints(aLastEdge, theFace, out aP_temp, out aP_last);
        if (aLastEdge.Orientation() == TopAbs_REVERSED)
        {
            aP_last = aP_temp;
        }

        // get first point
        if (aFirstEdge.Orientation() == TopAbs_REVERSED)
        {
            BRep_Tool.UVPoints(aFirstEdge, theFace, out aP_temp, out aP_first);
        }
        else
        {
            BRep_Tool.UVPoints(aFirstEdge, theFace, out aP_first, out aP_temp);
        }

        // Check 2d distance for periodic faces with seam edge
        if (!IsClosed2dForPeriodicFace(theFace, aP_first, aP_last, aFirstVertex))
        {
            aClosedStat = BRepCheck_NotClosed;
            if (Update)
            {
                BRepCheck.Add(aStatusList, aClosedStat);
            }

            return aClosedStat;
        }

        // check distance
        double aTol3d = Max(BRep_Tool.Tolerance(aFirstVertex), BRep_Tool.Tolerance(aWireExp.CurrentVertex()));

        Point3D aPntRef = BRep_Tool.Pnt(aFirstVertex);
        Point3D aPnt = BRep_Tool.Pnt(aWireExp.CurrentVertex());

        if (!IsDistanceIn2DTolerance(aFaceSurface, aP_first, aP_last, aTol3d))
        {
            aClosedStat = BRepCheck_NotClosed;
        }

        if (!IsDistanceIn3DTolerance(aPntRef, aPnt, aTol3d))
        {
            aClosedStat = BRepCheck_NotClosed;
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, aClosedStat);
        }

        return aClosedStat;
    }

    /// <summary>
    /// Checks that the oriented edges of the wire are coherently oriented (calls <see cref="Closed"/>); on a face
    /// (pass a null face if there is none) the choice between several connected edges is made in 2D:
    /// <see cref="BRepCheck_InvalidDegeneratedFlag"/>, <see cref="BRepCheck_BadOrientationOfSubshape"/>,
    /// <see cref="BRepCheck_NotClosed"/> or <see cref="BRepCheck_NoError"/>. With <paramref name="Update"/>, the status
    /// is added to <see cref="BRepCheck_Result.Status"/>.
    /// </summary>
    public BRepCheck_Status Orientation(in TopoDS_Face F, bool Update = false)
    {
        BRepCheck_Status theOstat = Closed();
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();

        if (theOstat != BRepCheck_NotClosed && theOstat != BRepCheck_NoError)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, theOstat);
            }

            return theOstat;
        }

        theOstat = BRepCheck_NoError;

        TopoDS_Vertex VF = new(), VL = new();
        TopAbs_Orientation orient, ortmp = TopAbs_FORWARD;
        var ledge = new NCollection_List<TopoDS_Shape>();
        var ListOfPassedEdge = new NCollection_List<TopoDS_Shape>();
        var exp = new TopExp_Explorer();
        var vte = new TopExp_Explorer();
        var mapS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopoDS_Edge theEdge = new(), theRef = new();

        // Checks the orientation of the edges
        for (exp.Init(myShape, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            orient = edg.Orientation();
            if (IsOriented(edg))
            {
                mapS.Add(edg);
                theEdge = edg;
                theRef = edg;
                for (vte.Init(edg, TopAbs_VERTEX); vte.More(); vte.Next())
                {
                    TopAbs_Orientation vto = vte.Current().Orientation();
                    if (vto == TopAbs_FORWARD)
                    {
                        VF = TopoDS.Vertex(vte.Current());
                    }
                    else if (vto == TopAbs_REVERSED)
                    {
                        VL = TopoDS.Vertex(vte.Current());
                    }

                    if (!VF.IsNull() && !VL.IsNull())
                    {
                        break;
                    }
                }

                if (VF.IsNull() && VL.IsNull())
                {
                    theOstat = BRepCheck_InvalidDegeneratedFlag;
                }

                break;
            }
        }

        if (theOstat == BRepCheck_NoError)
        {
            int Index = 1;
            int nbOriNoDegen = myMapVE.Extent();
            bool isGoFwd = true;

            if (VL.IsNull())
            {
                isGoFwd = false;
            }

            while (Index < nbOriNoDegen)
            {
                ledge.Clear();
                ListOfPassedEdge.Clear();
                // find edges that make a chain on VL if !VL.IsNull
                // otherwise on VF.

                int ind;
                if (!VL.IsNull())
                {
                    ind = myMapVE.FindIndex(VL);
                }
                else if (!VF.IsNull())
                {
                    ind = myMapVE.FindIndex(VF);
                }
                else
                {
                    theOstat = BRepCheck_InvalidDegeneratedFlag;
                    break;
                }

                for (var itls = new NCollection_List<TopoDS_Shape>.Iterator(myMapVE[ind]); itls.More(); itls.Next())
                {
                    TopoDS_Edge edg = TopoDS.Edge(itls.Value());

                    orient = edg.Orientation();
                    if (mapS.Contains(edg))
                    {
                        ortmp = GetOrientation(mapS, edg);
                    }

                    // Add to list already passed outcoming edges
                    if (mapS.Contains(edg) && ortmp == orient && !edg.IsSame(theEdge))
                    {
                        for (vte.Init(edg, TopAbs_VERTEX); vte.More(); vte.Next())
                        {
                            TopAbs_Orientation vto = vte.Current().Orientation();
                            if (!VL.IsNull())
                            {
                                if (vto == TopAbs_FORWARD && VL.IsSame(vte.Current()))
                                {
                                    ListOfPassedEdge.Append(edg);
                                    break;
                                }
                            }
                            else // VF is not null
                            {
                                if (vto == TopAbs_REVERSED && VF.IsSame(vte.Current()))
                                {
                                    ListOfPassedEdge.Append(edg);
                                    break;
                                }
                            }
                        }
                    }

                    if (!mapS.Contains(edg) || ortmp != orient)
                    {
                        for (vte.Init(edg, TopAbs_VERTEX); vte.More(); vte.Next())
                        {
                            TopAbs_Orientation vto = vte.Current().Orientation();
                            if (!VL.IsNull())
                            {
                                if (vto == TopAbs_FORWARD && VL.IsSame(vte.Current()))
                                {
                                    // If the processing is in 2d (face not null) or
                                    // if the edge is not degenerated it is added
                                    if (!F.IsNull() || !BRep_Tool.Degenerated(edg))
                                    {
                                        ledge.Append(edg);
                                    }

                                    break;
                                }
                            }
                            else
                            { // VF is not null
                                if (vto == TopAbs_REVERSED && VF.IsSame(vte.Current()))
                                {
                                    // If the processing is in 2d (face not null) or
                                    // if the edge is not degenerated it is added
                                    if (!F.IsNull() || !BRep_Tool.Degenerated(edg))
                                    {
                                        ledge.Append(edg);
                                    }

                                    break;
                                }
                            }
                        }
                    }
                }

                int nbconnex = ledge.Extent();
                bool Changedesens = false;
                if (nbconnex == 0)
                {
                    if (myCstat == BRepCheck_NotClosed)
                    {
                        if (VL.IsNull())
                        {
                            if (Update)
                            {
                                BRepCheck.Add(aStatusList, theOstat);
                            }

                            return theOstat; // leave
                        }
                        else
                        {
                            Index--; // because after Index++ and if there is no chain,
                            VL.Nullify(); // chain on VF is forced
                            theEdge = theRef;
                            Changedesens = true;
                        }
                    }
                    else
                    {
                        theOstat = BRepCheck_BadOrientationOfSubshape;
                        if (Update)
                        {
                            BRepCheck.Add(aStatusList, theOstat);
                        }

                        return theOstat;
                    }
                }

                // JAG 03/07   else if (nbconnex >= 2 && !F.IsNull())  // Try to see in 2d
                else if (!F.IsNull())
                { // Try to see in 2d
                    TopoDS_Vertex pivot;
                    if (!VL.IsNull())
                    {
                        pivot = VL;
                    }
                    else
                    {
                        pivot = VF;
                    }

                    ChoixUV(pivot, theEdge, F, ledge);
                    nbconnex = ledge.Extent();
                    // 20/03/02 akm (OCC234) - The 2d exploration of wire with necessary
                    // checks is performed in Closed2d, here it's useless
                }

                if (nbconnex >= 2)
                {
                    theOstat = BRepCheck_BadOrientationOfSubshape;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, theOstat);
                    }

                    return theOstat;
                }
                else if (nbconnex == 1)
                {
                    // offset the vertex
                    for (vte.Init(ledge.First(), TopAbs_VERTEX); vte.More(); vte.Next())
                    {
                        TopAbs_Orientation vto = vte.Current().Orientation();
                        if (!VL.IsNull())
                        {
                            if (vto == TopAbs_REVERSED)
                            {
                                VL = TopoDS.Vertex(vte.Current());
                                break;
                            }
                        }
                        else
                        { // VF is not null
                            if (vto == TopAbs_FORWARD)
                            {
                                VF = TopoDS.Vertex(vte.Current());
                                break;
                            }
                        }
                    }

                    mapS.Add(ledge.First());
                    theEdge = TopoDS.Edge(ledge.First());
                    if (!vte.More())
                    {
                        if (!VL.IsNull())
                        {
                            VL.Nullify();
                        }
                        else
                        {
                            VF.Nullify();
                        }
                    }
                }
                else if (!Changedesens)
                { // nbconnex == 0
                    theOstat = BRepCheck_NotClosed;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, theOstat);
                    }

                    return theOstat;
                }

                // Check the closure of the wire in 2d (not done in Closed())

                TopoDS_Vertex aVRef = new();
                bool isCheckClose = false;

                if (isGoFwd && !VF.IsNull())
                {
                    aVRef = VF;
                    isCheckClose = true;
                }
                else if (!isGoFwd && !VL.IsNull())
                {
                    aVRef = VL;
                    isCheckClose = true;
                }

                if (Index == 1 && myCstat != BRepCheck_NotClosed && isCheckClose && !F.IsNull())
                {
                    ledge.Clear();
                    ind = myMapVE.FindIndex(aVRef);
                    for (var itlsh = new NCollection_List<TopoDS_Shape>.Iterator(myMapVE[ind]); itlsh.More();
                         itlsh.Next())
                    {
                        TopoDS_Edge edg = TopoDS.Edge(itlsh.Value());
                        orient = edg.Orientation();
                        if (!theRef.IsSame(edg))
                        {
                            for (vte.Init(edg, TopAbs_VERTEX); vte.More(); vte.Next())
                            {
                                TopAbs_Orientation vto = vte.Current().Orientation();
                                if (vto == TopAbs_REVERSED && aVRef.IsSame(vte.Current()))
                                {
                                    ledge.Append(edg);
                                    break;
                                }
                            }
                        }
                    }

                    ChoixUV(aVRef, theRef, F, ledge);
                    if (ledge.Extent() == 0)
                    {
                        theOstat = BRepCheck_NotClosed;
                        if (Update)
                        {
                            BRepCheck.Add(aStatusList, theOstat);
                        }

                        return theOstat;
                    }
                }
                // End control closure 2d

                Index++;
            }
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, theOstat);
        }

        return theOstat;
    }

    /// <summary>
    /// Checks that the wire doesn't intersect itself on the face <paramref name="F"/>:
    /// <see cref="BRepCheck_EmptyWire"/>, <see cref="BRepCheck_SelfIntersectingWire"/>,
    /// <see cref="BRepCheck_NoCurveOnSurface"/>, <see cref="BRepCheck_InvalidRange"/> or
    /// <see cref="BRepCheck_NoError"/>. <paramref name="retE1"/> and <paramref name="retE2"/> receive the first
    /// intersecting edges found (only <paramref name="retE1"/> for an edge that intersects itself or has no pcurve);
    /// they are left unchanged otherwise. With <paramref name="Update"/>, the status is added to
    /// <see cref="BRepCheck_Result.Status"/> (not for NoCurveOnSurface and InvalidRange, as in OCCT).
    /// </summary>
    public BRepCheck_Status SelfIntersect(in TopoDS_Face F, ref TopoDS_Edge retE1, ref TopoDS_Edge retE2, bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();

        int i, j, Nbedges;
        double first1, last1, first2, last2, tolint;
        Point2D pfirst1, plast1, pfirst2, plast2;
        Point3D P3d, P3d2;
        BRepAdaptor_Surface HS;
        var C1 = new Geom2dAdaptor_Curve();
        var C2 = new Geom2dAdaptor_Curve();
        var Inter = new Geom2dInt_GInter();
        var myDomain1 = new IntRes2d_Domain();
        // OCCT's NCollection_IndexedMap<TopoDS_Shape> hashes with std::hash and compares with operator==
        // (IsEqual: the orientation counts)
        var EMap = new NCollection_IndexedMap<TopoDS_Shape>(EqualityComparer<TopoDS_Shape>.Default);
        //
        //-- check with proper tolerances if there is no
        //-- point in the tolerance of a vertex.
        tolint = 1.0e-10;
        HS = new BRepAdaptor_Surface();
        HS.Initialize(F, false);
        //
        for (var Iter1 = new TopoDS_Iterator(myShape); Iter1.More(); Iter1.Next())
        {
            if (Iter1.Value().ShapeType() == TopAbs_EDGE)
            {
                EMap.Add(Iter1.Value());
            }
        }

        //
        Nbedges = EMap.Extent();
        if (Nbedges == 0)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, BRepCheck_EmptyWire);
            }

            return BRepCheck_EmptyWire;
        }

        //
        var tabDom = new IntRes2d_Domain[Nbedges];
        for (int k0 = 0; k0 < Nbedges; k0++)
        {
            tabDom[k0] = new IntRes2d_Domain();
        }

        var tabCur = new NCollection_Array1<Geom2d_Curve?>(1, Nbedges);
        var boxes = new NCollection_Array1<Box2d>(1, Nbedges);
        //
        for (i = 1; i <= Nbedges; i++)
        {
            TopoDS_Edge E1 = TopoDS.Edge(EMap.FindKey(i));
            if (i == 1)
            {
                Geom2d_Curve? pcu = BRep_Tool.CurveOnSurface(E1, F, out first1, out last1);
                if (pcu is null)
                {
                    retE1 = E1;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, BRepCheck_SelfIntersectingWire);
                    }

                    return BRepCheck_SelfIntersectingWire;
                }

                //
                C1.Load(pcu);
                // To avoid exception in Segment if C1 is BSpline - IFV
                if (!C1.IsPeriodic())
                {
                    if (C1.FirstParameter() > first1)
                    {
                        first1 = C1.FirstParameter();
                    }

                    if (C1.LastParameter() < last1)
                    {
                        last1 = C1.LastParameter();
                    }
                }

                //
                BRep_Tool.UVPoints(E1, F, out pfirst1, out plast1);
                myDomain1.SetValues(pfirst1, first1, tolint, plast1, last1, tolint);
                //
                BndLib_Add2dCurve.Add(C1, first1, last1, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref boxes[i]);
            } // if (i == 1) {
            else
            {
                C1.Load(tabCur[i]!);
                myDomain1 = tabDom[i - 1];
            }

            //
            // Self intersect of C1
            Inter.Perform(C1, myDomain1, tolint, tolint);
            //
            if (Inter.IsDone())
            {
                int nbp = Inter.NbPoints();
                //
                for (int p = 1; p <= nbp; p++)
                {
                    IntRes2d_IntersectionPoint IP = Inter.Point(p);
                    IntRes2d_Transition Tr1 = IP.TransitionOfFirst();
                    IntRes2d_Transition Tr2 = IP.TransitionOfSecond();
                    if (Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                    {
                        //-- Checking of points with true tolerances (ie Tol in 3d)
                        //-- If the point of intersection is within the tolerance of a vertex
                        //-- this intersection is considered correct (no error)
                        bool localok = false;
                        Geom_Curve? ConS = BRep_Tool.Curve(E1, out Placement L, out double _, out double _);
                        if (ConS is not null)
                        {
                            //-- try to test in 3d. (ParamOnSecond gives the same result)
                            P3d = ConS.Value(IP.ParamOnFirst());
                            P3d.Transform(L.Transformation());
                        }
                        else
                        {
                            Point2D aP2d = C1.Value(IP.ParamOnFirst());
                            P3d = HS.Value(aP2d.X(), aP2d.Y());
                        }

                        var ExplVtx = new TopExp_Explorer();
                        for (ExplVtx.Init(E1, TopAbs_VERTEX); !localok && ExplVtx.More(); ExplVtx.Next())
                        {
                            Point3D p3dvtt;
                            double tolvtt, p3dvttDistanceP3d;
                            //
                            TopoDS_Vertex vtt = TopoDS.Vertex(ExplVtx.Current());
                            p3dvtt = BRep_Tool.Pnt(vtt);
                            tolvtt = BRep_Tool.Tolerance(vtt);
                            tolvtt = tolvtt * tolvtt;
                            p3dvttDistanceP3d = p3dvtt.SquareDistance(P3d);
                            if (p3dvttDistanceP3d <= tolvtt)
                            {
                                localok = true;
                            }
                        }

                        if (!localok)
                        {
                            retE1 = E1;
                            if (Update)
                            {
                                BRepCheck.Add(aStatusList, BRepCheck_SelfIntersectingWire);
                            }

                            return BRepCheck_SelfIntersectingWire;
                        }
                    }
                }
            } // if(Inter.IsDone()) {

            //
            for (j = i + 1; j <= Nbedges; j++)
            {
                TopoDS_Edge E2 = TopoDS.Edge(EMap.FindKey(j));
                if (i == 1)
                {
                    tabCur[j] = BRep_Tool.CurveOnSurface(E2, F, out first2, out last2);
                    if (tabCur[j] is not null && last2 > first2)
                    {
                        C2.Load(tabCur[j]!);
                        // To avoid exception in Segment if C2 is BSpline - IFV
                        if (!C2.IsPeriodic())
                        {
                            if (C2.FirstParameter() > first2)
                            {
                                first2 = C2.FirstParameter();
                            }

                            if (C2.LastParameter() < last2)
                            {
                                last2 = C2.LastParameter();
                            }
                        }

                        //
                        BRep_Tool.UVPoints(E2, F, out pfirst2, out plast2);
                        tabDom[j - 1].SetValues(pfirst2, first2, tolint, plast2, last2, tolint);

                        BndLib_Add2dCurve.Add(C2, first2, last2, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref boxes[j]);
                    }
                    else
                    {
                        if (tabCur[j] is null)
                        {
                            return BRepCheck_NoCurveOnSurface;
                        }

                        return BRepCheck_InvalidRange;
                    }
                } // if (i == 1) {
                else
                {
                    C2.Load(tabCur[j]!);
                }

                //
                if (boxes[i].IsOut(boxes[j]))
                {
                    continue;
                }

                // modified by NIZNHY-PKV Fri Oct 29 10:09:01 2010f
                if (E1.IsSame(E2))
                {
                    continue;
                }

                //
                //-- ************************************************************
                //-- ******* I n t e r s e c t i o n   C 1   and   C 2   ********
                //-- ************************************************************
                Inter.Perform(C1, myDomain1, C2, tabDom[j - 1], tolint, tolint);
                //
                if (Inter.IsDone())
                {
                    int nbp, nbs;
                    double IP_ParamOnFirst, IP_ParamOnSecond;
                    IntRes2d_Transition Tr1, Tr2;
                    var CommonVertices = new NCollection_List<TopoDS_Shape>();
                    var itl = new NCollection_List<TopoDS_Shape>.Iterator();
                    var Vmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    //
                    var it = new TopoDS_Iterator(E1);
                    for (; it.More(); it.Next())
                    {
                        Vmap.Add(it.Value());
                    }

                    //
                    it.Initialize(E2);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Shape V = it.Value();
                        if (Vmap.Contains(V))
                        {
                            CommonVertices.Append(V);
                        }
                    }

                    //
                    nbp = Inter.NbPoints();
                    nbs = Inter.NbSegments();
                    IP_ParamOnFirst = 0.0;
                    IP_ParamOnSecond = 0.0;
                    //
                    //// **** Points of intersection **** ////
                    for (int p = 1; p <= nbp; p++)
                    {
                        IntRes2d_IntersectionPoint IP = Inter.Point(p);
                        IP_ParamOnFirst = IP.ParamOnFirst();
                        IP_ParamOnSecond = IP.ParamOnSecond();
                        Tr1 = IP.TransitionOfFirst();
                        Tr2 = IP.TransitionOfSecond();
                        if (Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                        {
                            //-- Checking of points with true tolerances (ie Tol in 3d)
                            //-- If the point of intersection is within the tolerance of a vertex
                            //-- this intersection is considered correct (no error)
                            bool localok = false;
                            //
                            Geom_Curve? ConS = BRep_Tool.Curve(E1, out Placement L, out double f1, out double l1);
                            Geom_Curve? ConS2 = BRep_Tool.Curve(E2, out Placement L2, out double f2, out double l2);
                            // gka protect against working out of edge range
                            if (f1 - IP_ParamOnFirst > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || IP_ParamOnFirst - l1 > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || f2 - IP_ParamOnSecond > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || IP_ParamOnSecond - l2 > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                continue;
                            }

                            double tolvtt = 0.0;
                            if (ConS is not null)
                            {
                                P3d = ConS.Value(IP_ParamOnFirst);
                                P3d.Transform(L.Transformation());
                            }
                            else
                            {
                                Point2D aP2d = C1.Value(IP_ParamOnFirst);
                                P3d = HS.Value(aP2d.X(), aP2d.Y());
                            }

                            //
                            if (ConS2 is not null)
                            {
                                P3d2 = ConS2.Value(IP_ParamOnSecond);
                                P3d2.Transform(L2.Transformation());
                            }
                            else
                            {
                                Point2D aP2d = C2.Value(IP_ParamOnSecond);
                                P3d2 = HS.Value(aP2d.X(), aP2d.Y());
                            }

                            itl.Initialize(CommonVertices);
                            for (; itl.More(); itl.Next())
                            {
                                double p3dvttDistanceP3d, p3dvttDistanceP3d2;
                                Point3D p3dvtt;
                                //
                                TopoDS_Vertex vtt = TopoDS.Vertex(itl.Value());
                                p3dvtt = BRep_Tool.Pnt(vtt);
                                tolvtt = BRep_Tool.Tolerance(vtt);
                                tolvtt = 1.1 * tolvtt;
                                tolvtt = tolvtt * tolvtt;
                                p3dvttDistanceP3d = p3dvtt.SquareDistance(P3d);
                                p3dvttDistanceP3d2 = p3dvtt.SquareDistance(P3d2);
                                //
                                if (p3dvttDistanceP3d <= tolvtt && p3dvttDistanceP3d2 <= tolvtt)
                                {
                                    localok = true;
                                    break;
                                }
                            }

                            //-- --------------------------------------------------------
                            //-- Check maximum yawn between 2 edges
                            //--
                            //-- Check distance from edges to the curve joining
                            //-- the point of intersection with vertex (if exists)
                            if (!localok && !CommonVertices.IsEmpty())
                            {
                                double distauvtxleplusproche, VParaOnEdge1, VParaOnEdge2;
                                Point3D VertexLePlusProche = new();
                                //
                                VParaOnEdge1 = 0.0;
                                VParaOnEdge2 = 0.0;
                                distauvtxleplusproche = RealLast();
                                // Find the nearest common vertex
                                itl.Initialize(CommonVertices);
                                for (; itl.More(); itl.Next())
                                {
                                    double disptvtx;
                                    Point3D p3dvtt;
                                    //
                                    TopoDS_Vertex vtt = TopoDS.Vertex(itl.Value());
                                    p3dvtt = BRep_Tool.Pnt(vtt);
                                    disptvtx = P3d.Distance(p3dvtt);
                                    if (disptvtx < distauvtxleplusproche)
                                    {
                                        VertexLePlusProche = p3dvtt;
                                        distauvtxleplusproche = disptvtx;
                                        VParaOnEdge1 = BRep_Tool.Parameter(vtt, E1);
                                        VParaOnEdge2 = BRep_Tool.Parameter(vtt, E2);
                                    }
                                    // eap: case of closed edge
                                    else if (IsEqual(distauvtxleplusproche, disptvtx))
                                    {
                                        double newVParaOnEdge1 = BRep_Tool.Parameter(vtt, E1);
                                        double newVParaOnEdge2 = BRep_Tool.Parameter(vtt, E2);
                                        if (Math.Abs(IP_ParamOnFirst - VParaOnEdge1)
                                            + Math.Abs(IP_ParamOnSecond - VParaOnEdge2)
                                            > Math.Abs(IP_ParamOnFirst - newVParaOnEdge1)
                                            + Math.Abs(IP_ParamOnSecond - newVParaOnEdge2))
                                        {
                                            VertexLePlusProche = p3dvtt;
                                            VParaOnEdge1 = newVParaOnEdge1;
                                            VParaOnEdge2 = newVParaOnEdge2;
                                        }
                                    }
                                }

                                // Patch: extraordinar situation (e.g. tolerance(v) == 0.)
                                //   Modified by skv - Wed Jul 23 12:28:11 2003 OCC1764
                                if (VertexLePlusProche.Distance(P3d) <= Constants.Resolution()
                                    || VertexLePlusProche.Distance(P3d2) <= Constants.Resolution())
                                {
                                    localok = true;
                                }
                                else
                                {
                                    var Lig = new Line3D(VertexLePlusProche, new Direction3D(new Vector3D(VertexLePlusProche, P3d)));
                                    double du1 = 0.1 * (IP_ParamOnFirst - VParaOnEdge1);
                                    double du2 = 0.1 * (IP_ParamOnSecond - VParaOnEdge2);
                                    double maxd1 = 0.0, maxd2 = 0.0;
                                    int k;

                                    localok = true;
                                    double tole1 = BRep_Tool.Tolerance(E1);
                                    for (k = 2; localok && k < 9; k++)
                                    {
                                        double u = VParaOnEdge1 + k * du1; // check if it works
                                        Point3D P1;
                                        if (ConS is not null)
                                        {
                                            P1 = ConS.Value(u);
                                            P1.Transform(L.Transformation());
                                        }
                                        else
                                        {
                                            Point2D aP2d = C1.Value(u);
                                            P1 = HS.Value(aP2d.X(), aP2d.Y());
                                        }

                                        double d1 = Lig.Distance(P1);
                                        if (d1 > maxd1)
                                        {
                                            maxd1 = d1;
                                        }

                                        if (d1 > tole1 * 2.0)
                                        {
                                            localok = false;
                                        }
                                    }

                                    //-- same for edge2
                                    //  Modified by skv - Wed Jul 23 12:22:20 2003 OCC1764
                                    var aTmpDir = new Direction3D(P3d2.XYZ().Subtracted(VertexLePlusProche.XYZ()));

                                    Lig.SetDirection(aTmpDir);
                                    double tole2 = BRep_Tool.Tolerance(E2);
                                    for (k = 2; localok && k < 9; k++)
                                    {
                                        double u = VParaOnEdge2 + k * du2; // check if it works
                                        Point3D P2;
                                        if (ConS2 is not null)
                                        {
                                            P2 = ConS2.Value(u);
                                            P2.Transform(L2.Transformation());
                                        }
                                        else
                                        {
                                            Point2D aP2d = C2.Value(u);
                                            P2 = HS.Value(aP2d.X(), aP2d.Y());
                                        }

                                        double d2 = Lig.Distance(P2);
                                        if (d2 > maxd2)
                                        {
                                            maxd2 = d2;
                                        }

                                        if (d2 > tole2 * 2.0)
                                        {
                                            localok = false;
                                        }
                                    }
                                } // end of else (construction of the line Lig)
                            } // end of if (localok == false && !CommonVertices.IsEmpty())

                            //
                            if (!localok)
                            {
                                retE1 = E1;
                                retE2 = E2;
                                if (Update)
                                {
                                    BRepCheck.Add(aStatusList, BRepCheck_SelfIntersectingWire);
                                }

                                return BRepCheck_SelfIntersectingWire;
                            } //-- localok == False
                        } // end of if(Tr1.PositionOnCurve() == IntRes2d_Middle || Tr2.PositionOnCurve() == IntRes2d_Middle)
                    } // end of for (int p=1; p <= nbp; p++)

                    ////
                    //// **** Segments of intersection **** ////
                    for (int s = 1; s <= nbs; ++s)
                    {
                        IntRes2d_IntersectionSegment Seg = Inter.Segment(s);
                        if (Seg.HasFirstPoint() && Seg.HasLastPoint())
                        {
                            bool localok;
                            int k;
                            var PSeg = new IntRes2d_IntersectionPoint[2];
                            IntRes2d_Position aPCR1, aPCR2;
                            //
                            localok = false;
                            PSeg[0] = Seg.FirstPoint();
                            PSeg[1] = Seg.LastPoint();
                            // At least one of extremities of the segment must be inside
                            // the tolerance of a common vertex
                            for (k = 0; k < 2; ++k)
                            {
                                IP_ParamOnFirst = PSeg[k].ParamOnFirst();
                                IP_ParamOnSecond = PSeg[k].ParamOnSecond();
                                Tr1 = PSeg[k].TransitionOfFirst();
                                Tr2 = PSeg[k].TransitionOfSecond();
                                aPCR1 = Tr1.PositionOnCurve();
                                aPCR2 = Tr2.PositionOnCurve();
                                //
                                if (aPCR1 != IntRes2d_Middle && aPCR2 != IntRes2d_Middle)
                                {
                                    CurveKind aCT1, aCT2;
                                    // ZZ
                                    aCT1 = C1.GetType();
                                    aCT2 = C2.GetType();
                                    if (aCT1 == Line && aCT2 == Line)
                                    {
                                        // check for the two lines coincidence
                                        double aPAR_T, aT11, aT12, aT21, aT22, aT1m, aT2m;
                                        double aD2, aTolE1, aTolE2, aTol2;
                                        Line2D aL1, aL2;
                                        Point2D aP1m;
                                        //
                                        aPAR_T = 0.43213918;
                                        //
                                        aTolE1 = BRep_Tool.Tolerance(E1);
                                        aTolE2 = BRep_Tool.Tolerance(E2);
                                        aTol2 = aTolE1 + aTolE2;
                                        aTol2 = aTol2 * aTol2;
                                        //
                                        aL1 = C1.Line();
                                        aL2 = C2.Line();
                                        //
                                        aT11 = PSeg[0].ParamOnFirst();
                                        aT12 = PSeg[1].ParamOnFirst();
                                        aT21 = PSeg[0].ParamOnSecond();
                                        aT22 = PSeg[1].ParamOnSecond();
                                        //
                                        aT1m = (1.0 - aPAR_T) * aT11 + aPAR_T * aT12;
                                        aP1m = C1.Value(aT1m);
                                        //
                                        aD2 = aL2.SquareDistance(aP1m);
                                        if (aD2 < aTol2)
                                        {
                                            aT2m = global::Kerf.Engine.Geometry.ElCLib.Parameter(aL2, aP1m);
                                            if (aT2m > aT21 && aT2m < aT22)
                                            {
                                                Direction2D aDir1 = aL1.Direction();
                                                Direction2D aDir2 = aL2.Direction();
                                                if (aDir1.IsParallel(aDir2, global::Kerf.Engine.Foundation.Precision.Angular()))
                                                {
                                                    localok = false;
                                                    break; // from for (k = 0; k < 2; ++k){...
                                                }
                                            } // if (aT2m>aT21 && aT2m<aT22) {
                                        } // if (aD2<aTol2) {
                                    } // if (aCT1==GeomAbs_Line && aCT2==GeomAbs_Line) {

                                    // ZZ
                                    localok = true;
                                    break;
                                }

                                //
                                double tolvtt;
                                Geom_Curve? ConS = BRep_Tool.Curve(E1, out Placement L, out double _, out double _);
                                Geom_Curve? ConS2 = BRep_Tool.Curve(E2, out Placement L2, out double _, out double _);
                                if (ConS is not null)
                                {
                                    P3d = ConS.Value(IP_ParamOnFirst);
                                    P3d.Transform(L.Transformation());
                                }
                                else
                                {
                                    Point2D aP2d = C1.Value(IP_ParamOnFirst);
                                    P3d = HS.Value(aP2d.X(), aP2d.Y());
                                }

                                if (ConS2 is not null)
                                {
                                    P3d2 = ConS2.Value(IP_ParamOnSecond);
                                    P3d2.Transform(L2.Transformation());
                                }
                                else
                                {
                                    Point2D aP2d = C2.Value(IP_ParamOnSecond);
                                    P3d2 = HS.Value(aP2d.X(), aP2d.Y());
                                }

                                itl.Initialize(CommonVertices);
                                for (; itl.More(); itl.Next())
                                {
                                    double p3dvttDistanceP3d, p3dvttDistanceP3d2;
                                    Point3D p3dvtt;
                                    //
                                    TopoDS_Vertex vtt = TopoDS.Vertex(itl.Value());
                                    p3dvtt = BRep_Tool.Pnt(vtt);
                                    tolvtt = BRep_Tool.Tolerance(vtt);
                                    tolvtt = 1.1 * tolvtt;
                                    tolvtt = tolvtt * tolvtt;
                                    p3dvttDistanceP3d = p3dvtt.SquareDistance(P3d);
                                    p3dvttDistanceP3d2 = p3dvtt.SquareDistance(P3d2);
                                    if (p3dvttDistanceP3d <= tolvtt && p3dvttDistanceP3d2 <= tolvtt)
                                    {
                                        localok = true;
                                        break;
                                    }
                                }

                                if (localok)
                                {
                                    break;
                                }
                            } // end of for (k = 0; k < 2; k++)

                            //
                            if (!localok)
                            {
                                retE1 = E1;
                                retE2 = E2;
                                if (Update)
                                {
                                    BRepCheck.Add(aStatusList, BRepCheck_SelfIntersectingWire);
                                }

                                return BRepCheck_SelfIntersectingWire;
                            } //-- localok == False
                        } // end of if(Seg.HasFirstPoint() && Seg.HasLastPoint())
                    } // end of for (int s = 1; s <= nbs; p++)
                } //-- Inter.IsDone()
            } // end of for( j = i+1; j<=Nbedges; j++)
        } // end of for(i = 1; i <= Nbedges; i++)

        //
        if (Update)
        {
            BRepCheck.Add(aStatusList, BRepCheck_NoError);
        }

        //
        return BRepCheck_NoError;
    }

    /// <summary>Adds <paramref name="theStatus"/> to the statuses of the wire.</summary>
    public void SetStatus(BRepCheck_Status theStatus)
    {
        // OCCT takes no lock here (the analyzer never calls it)
        BRepCheck.Add(myMap.Find(myShape), theStatus);
    }

    /// <summary>Sets whether <see cref="InContext"/> checks <see cref="SelfIntersect"/>.</summary>
    public void GeometricControls(bool B)
    {
        if (myGctrl != B)
        {
            if (B)
            {
                myCdone = false;
            }

            myGctrl = B;
        }
    }

    /// <summary>Whether <see cref="InContext"/> checks <see cref="SelfIntersect"/>.</summary>
    public bool GeometricControls() => myGctrl;

    // fill <mapE> with edges connected to <edg> through vertices contained in <mapVE>
    private static void Propagate(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapVE,
                                  in TopoDS_Shape edg,
                                  NCollection_Map<TopoDS_Shape> mapE)
    {
        var currentEdges = new NCollection_List<TopoDS_Shape>();
        currentEdges.Append(edg);

        do
        {
            var nextEdges = new NCollection_List<TopoDS_Shape>();
            var itrc = new NCollection_List<TopoDS_Shape>.Iterator(currentEdges);
            for (; itrc.More(); itrc.Next())
            {
                TopoDS_Shape Edge = itrc.Value();
                if (!mapE.Contains(Edge))
                {
                    mapE.Add(Edge);
                }

                var ex = new TopExp_Explorer(Edge, TopAbs_VERTEX);
                for (; ex.More(); ex.Next())
                {
                    TopoDS_Vertex vtx = TopoDS.Vertex(ex.Current());
                    int indv = mapVE.FindIndex(vtx);
                    if (indv != 0)
                    {
                        NCollection_List<TopoDS_Shape> edges = mapVE[indv];

                        var itl = new NCollection_List<TopoDS_Shape>.Iterator(edges);
                        for (; itl.More(); itl.Next())
                        {
                            TopoDS_Shape E = itl.Value();
                            if (!Edge.IsSame(E) && !mapE.Contains(E))
                            {
                                mapE.Add(E);
                                nextEdges.Append(E);
                            }
                        }
                    }
                }
            }

            currentEdges = nextEdges;
        } while (!currentEdges.IsEmpty());
    }

    // The orientation of the key of <mapE> that is the same edge as <edg>. The map compares with IsSame, so it
    // holds one such key and OCCT's map order doesn't matter.
    private static TopAbs_Orientation GetOrientation(NCollection_Map<TopoDS_Shape> mapE, in TopoDS_Edge edg)
    {
        TopoDS_Shape aKey = new();
        foreach (TopoDS_Shape itm in mapE)
        {
            aKey = itm;
            if (itm.IsSame(edg))
            {
                break;
            }
        }

        return aKey.Orientation();
    }

    // For vertex theVertex given function find an edge along that we should go further.
    private static void ChoixUV(in TopoDS_Vertex theVertex,
                                in TopoDS_Edge theEdge,
                                in TopoDS_Face theFace,
                                NCollection_List<TopoDS_Shape> theLOfShape)
    {
        var It = new NCollection_List<TopoDS_Shape>.Iterator(theLOfShape);
        while (It.More())
        {
            if (theEdge.IsSame(It.Value()))
            {
                theLOfShape.Remove(It);
            }
            else
            {
                It.Next();
            }
        }

        double aTol3d = BRep_Tool.Tolerance(theVertex);

        int anIndex = 0, anIndMin = 0;
        TopoDS_Edge anEFound;
        Point2D aPntRef, aPnt = new();
        Vector2D aDerRef, aDer;
        double aMinAngle, aMaxAngle, anAngle;
        double a_gpResolution = Constants.Resolution();
        TopAbs_Orientation aVOrientation, anEdgOrientation;
        double aParam = 0.0, aFirstParam, aLastParam, aParPiv;
        var aFaceSurface = new BRepAdaptor_Surface(theFace, false); // no restriction

        Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out aFirstParam, out aLastParam);
        if (C2d is null)
        { // JAG 10.12.96
            return;
        }

        aVOrientation = theVertex.Orientation();
        anEdgOrientation = theEdge.Orientation();

        aParPiv = (aVOrientation == anEdgOrientation) ? aFirstParam : aLastParam;
        aMinAngle = RealLast();
        aMaxAngle = RealFirst();

        // OCCT passes the curve handle, converted to an adaptor by Geom2dAdaptor_Curve's constructor
        CurveDirForParameter(new Geom2dAdaptor_Curve(C2d), aParPiv, out aPntRef, out aDerRef);

        if (aVOrientation != anEdgOrientation)
        {
            aDerRef.Reverse();
        }

        It.Initialize(theLOfShape);

        for (; It.More(); It.Next())
        {
            anIndex++;
            TopoDS_Edge anE = TopoDS.Edge(It.Value());
            C2d = BRep_Tool.CurveOnSurface(anE, theFace, out aFirstParam, out aLastParam);
            if (C2d is null)
            {
                continue;
            }

            var aCA = new Geom2dAdaptor_Curve(C2d);

            aParam = (aVOrientation != anE.Orientation()) ? aFirstParam : aLastParam;
            aPnt = aCA.Value(aParam);

            if (!IsDistanceIn2DTolerance(aFaceSurface, aPnt, aPntRef, aTol3d))
            {
                continue;
            }

            CurveDirForParameter(aCA, aParam, out aPnt, out aDer);

            if (aVOrientation == anE.Orientation())
            {
                aDer.Reverse();
            }

            if ((aDerRef.Magnitude() <= a_gpResolution) || (aDer.Magnitude() <= a_gpResolution))
            {
                // Vector length is too small
                continue;
            }

            anAngle = -aDerRef.Angle(aDer);

            if (anAngle < 0.0)
            {
                anAngle += 2.0 * M_PI;
            }

            if (theFace.Orientation() == TopAbs_FORWARD)
            {
                if (anAngle < aMinAngle)
                {
                    anIndMin = anIndex;
                    aMinAngle = anAngle;
                }
            }
            else // theFace.Orientation() != TopAbs_FORWARD
            {
                if (anAngle > aMaxAngle)
                {
                    anIndMin = anIndex;
                    aMaxAngle = anAngle;
                }
            }
        } // end of for

        // Update edge
        if (anIndMin == 0)
        {
            if (theLOfShape.Extent() == 1)
            {
                bool IsFound = true; // all right
                anEFound = TopoDS.Edge(theLOfShape.First());

                if (anEFound.IsNull() || BRep_Tool.Degenerated(theEdge) || BRep_Tool.Degenerated(anEFound))
                {
                    IsFound = false; // bad
                }
                else if (!IsDistanceIn2DTolerance(aFaceSurface, aPnt, aPntRef, aTol3d))
                {
                    IsFound = false; // bad
                }
                else
                // clousureness in 3D
                {
                    // IsDistanceIn3DTolerance
                    var bcEdg = new BRepAdaptor_Curve(theEdge, theFace);
                    var bcEvois = new BRepAdaptor_Curve(anEFound, theFace);
                    Point3D pEdg = bcEdg.Value(aParPiv);
                    Point3D pEFound = bcEvois.Value(aParam);

                    IsFound = IsDistanceIn3DTolerance(pEdg, pEFound, aTol3d);
                }

                if (!IsFound)
                {
                    theLOfShape.Clear();
                }
            } // if (theLOfShape.Extent() == 1)
            else // if (anIndMin == 0)
            {
                theLOfShape.Clear();
            }
        }
        else
        {
            anIndex = 1;

            while (anIndex < anIndMin)
            {
                theLOfShape.RemoveFirst();
                anIndex++;
            }

            It.Initialize(theLOfShape);
            It.Next();

            while (It.More())
            {
                theLOfShape.Remove(It);
            }
        }
    } // End of function

    private static void CurveDirForParameter(Geom2dAdaptor_Curve aC2d, double aPrm, out Point2D Pnt, out Vector2D aVec2d)
    {
        double aTol = Constants.Resolution();
        int i;

        aC2d.D1(aPrm, out Pnt, out aVec2d);
        //
        if (aVec2d.Magnitude() <= aTol)
        {
            for (i = 2; i <= 100; i++)
            {
                aVec2d = aC2d.DN(aPrm, i);
                if (aVec2d.Magnitude() > aTol)
                {
                    break;
                }
            }
        }
    }

    // Returns the parametric point of theVertex on theFace through theEdge; false if theVertex isn't a vertex of
    // theEdge or theEdge has no pcurve on theFace.
    private static bool GetPnt2d(in TopoDS_Vertex theVertex, in TopoDS_Edge theEdge, in TopoDS_Face theFace, ref Point2D aPnt)
    {
        Geom2d_Curve? aPCurve;
        double aParOnEdge;

        TopExp.Vertices(theEdge, out TopoDS_Vertex aFirstVtx, out TopoDS_Vertex aLastVtx);

        if (!theVertex.IsSame(aFirstVtx) && !theVertex.IsSame(aLastVtx))
        {
            return false;
        }

        aPCurve = BRep_Tool.CurveOnSurface(theEdge, theFace, out double _, out double _);

        if (aPCurve is null)
        {
            return false;
        }

        aParOnEdge = BRep_Tool.Parameter(theVertex, theEdge);
        aPnt = aPCurve.Value(aParOnEdge);

        return true;
    }

    // Checks the distance between first point of the first edge and last point of the last edge in 2d for
    // periodic face.
    private static bool IsClosed2dForPeriodicFace(in TopoDS_Face theFace,
                                                  in Point2D theP1,
                                                  in Point2D theP2,
                                                  in TopoDS_Vertex theVertex)
    {
        // Check 2d distance for periodic faces with seam edge.
        // Searching for seam edges
        var aSeamEdges = new NCollection_List<TopoDS_Shape>();
        var NotSeams = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ClosedEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var anExp = new TopExp_Explorer(theFace, TopAbs_EDGE);

        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

            if (NotSeams.Contains(anEdge))
            {
                continue;
            }

            if (!IsOriented(anEdge) || !BRep_Tool.IsClosed(anEdge, theFace))
            {
                NotSeams.Add(anEdge);
                continue;
            }

            if (!ClosedEdges.Add(anEdge))
            {
                aSeamEdges.Append(anEdge);
            }
        }

        if (aSeamEdges.Extent() == 0)
        {
            return true;
        }

        // check if theVertex lies on one of the seam edges
        var aFaceSurface = new BRepAdaptor_Surface(theFace, false);
        double aTol = BRep_Tool.Tolerance(theVertex);
        double aUResol = aFaceSurface.UResolution(aTol);
        double aVResol = aFaceSurface.VResolution(aTol);
        double aVicinity = Math.Sqrt(aUResol * aUResol + aVResol * aVResol);
        double aDistP1P2 = theP1.Distance(theP2);

        var anIter = new NCollection_List<TopoDS_Shape>.Iterator(aSeamEdges);

        for (; anIter.More(); anIter.Next())
        {
            TopoDS_Edge aSeamEdge = TopoDS.Edge(anIter.Value());

            anExp.Init(aSeamEdge, TopAbs_VERTEX);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aVtx = anExp.Current();

                // We found an edge. Check the distance between two given points
                //  to be lower than the computed tolerance.
                if (IsOriented(aVtx) && aVtx.IsSame(theVertex))
                {
                    Point2D aPnt1 = new();
                    Point2D aPnt2 = new();
                    double a2dTol;

                    if (!GetPnt2d(theVertex, aSeamEdge, theFace, ref aPnt1))
                    {
                        continue;
                    }

                    aSeamEdge = TopoDS.Edge(aSeamEdge.Reversed());

                    if (!GetPnt2d(theVertex, aSeamEdge, theFace, ref aPnt2))
                    {
                        continue;
                    }

                    a2dTol = aPnt1.Distance(aPnt2) * 1.0e-2;
                    a2dTol = Max(a2dTol, aVicinity);

                    if (aDistP1P2 > a2dTol)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
