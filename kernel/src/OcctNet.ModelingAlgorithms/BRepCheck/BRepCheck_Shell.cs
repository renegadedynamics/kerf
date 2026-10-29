// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Shell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Shell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The checks of a shell: connected (<see cref="Minimum"/>), closed (<see cref="Closed"/>), coherently oriented
/// (<see cref="Orientation"/>), and its sets of faces connected through simple edges
/// (<see cref="NbConnectedSet"/>).
/// </summary>
public class BRepCheck_Shell : BRepCheck_Result
{
    private int myNbori;
    private bool myCdone;
    private BRepCheck_Status myCstat;
    private bool myOdone;
    private BRepCheck_Status myOstat;

    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMapEF =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>Checks the shell on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Shell(in TopoDS_Shell S)
    {
        myNbori = 0;
        myCdone = false;
        myCstat = BRepCheck_NoError;
        myOdone = false;
        myOstat = BRepCheck_NoError;
        Init(S);
    }

    private static void Propagate(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapEF,
                                  in TopoDS_Shape theFace,
                                  NCollection_IndexedMap<TopoDS_Shape> theMapF)
    {
        // Base for the traverse procedure.
        theMapF.Add(theFace);

        // Perform well-known width-first traverse.
        for (int anIdx = 1; anIdx <= theMapF.Extent(); ++anIdx)
        {
            TopoDS_Shape aFace = theMapF[anIdx];
            for (var ex = new TopExp_Explorer(aFace, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(ex.Current());
                // Test if the edge is in the map (only oriented edges are present).
                if (!mapEF.FindFromKey(edg, out NCollection_List<TopoDS_Shape>? aList))
                {
                    continue;
                }

                for (var itl = new NCollection_List<TopoDS_Shape>.Iterator(aList); itl.More(); itl.Next())
                {
                    // This code assumes that shape is added to an end of the map.
                    // The idea is simple: existing objects will not be added, new objects
                    // will be added to an end.
                    theMapF.Add(itl.Value());
                }
            }
        }
    }

    private static bool IsOriented(in TopoDS_Shape S)
        => S.Orientation() == TopAbs_FORWARD || S.Orientation() == TopAbs_REVERSED;

    /// <summary>The status list of the shell itself, read under the lock in parallel mode.</summary>
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

    /// <summary>Checks that the shell has faces and that they are connected through their edges.</summary>
    public override void Minimum()
    {
        myCdone = false;
        myOdone = false;

        if (!myMin)
        {
            var aNewList = new NCollection_List<BRepCheck_Status>();
            NCollection_List<BRepCheck_Status> lst = myMap.Bound(myShape, aNewList);

            // it is checked if the shell is "connected"
            var exp = new TopExp_Explorer(myShape, TopAbs_FACE);
            int nbface = 0;
            myMapEF.Clear();
            for (; exp.More(); exp.Next())
            {
                nbface++;
                var expe = new TopExp_Explorer();
                for (expe.Init(exp.Current(), TopAbs_EDGE); expe.More(); expe.Next())
                {
                    TopoDS_Shape edg = expe.Current();
                    int index = myMapEF.FindIndex(edg);
                    if (index == 0)
                    {
                        var thelist1 = new NCollection_List<TopoDS_Shape>();
                        index = myMapEF.Add(edg, thelist1);
                    }

                    myMapEF[index].Append(exp.Current());
                }
            } // for (; exp.More(); exp.Next())

            if (nbface == 0)
            {
                BRepCheck.Add(lst, BRepCheck_EmptyShell);
            }
            else if (nbface >= 2)
            {
                var mapF = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                exp.ReInit();
                Propagate(myMapEF, exp.Current(), mapF);
                if (mapF.Extent() != nbface)
                {
                    BRepCheck.Add(lst, BRepCheck_NotConnected);
                }
            } // else if (nbface >= 2)

            if (lst.IsEmpty())
            {
                lst.Append(BRepCheck_NoError);
            }

            myMapEF.Clear();
            myMin = true;
        }
    }

    /// <summary>
    /// Checks the shell in a shape that contains it: in a solid, that it is closed and, unless it is unorientable,
    /// coherently oriented.
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

        TopExp_Explorer exp = new(S, TopAbs_SHELL);
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

        TopAbs_ShapeEnum styp = S.ShapeType();
        switch (styp)
        {
            case TopAbs_SOLID:
            {
                BRepCheck_Status fst = Closed();
                if ((fst == BRepCheck_NotClosed && S.Closed()) || (fst != BRepCheck_NoError))
                {
                    BRepCheck.Add(lst, fst);
                }
                else if (!IsUnorientable())
                {
                    fst = Orientation();
                    BRepCheck.Add(lst, fst);
                }
            }

                break;
            default:
                break;
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
            // nothing more than in the minimum
            myBlind = true;
        }
    }

    /// <summary>
    /// Checks that the shell is closed: each oriented edge of its oriented faces is found twice (or once if
    /// degenerated). With <paramref name="Update"/>, the status is added to <see cref="BRepCheck_Result.Status"/>.
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

        myCdone = true; // it will be done...

        var itl = new NCollection_List<BRepCheck_Status>.Iterator(aStatusList);
        if (itl.Value() != BRepCheck_NoError)
        {
            myCstat = itl.Value();
            return myCstat; // already saved
        }

        myCstat = BRepCheck_NoError;
        //
        int index, aNbF;
        TopExp_Explorer exp = new(), ede = new();
        var mapS = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aMEToAvoid = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        myMapEF.Clear();

        // Checks if the oriented faces of the shell give a "closed" shell,
        // i-e if each oriented edge on oriented faces is found 2 times.
        //
        exp.Init(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Shape aF = exp.Current();
            if (IsOriented(aF))
            {
                ede.Init(exp.Current(), TopAbs_EDGE);
                for (; ede.More(); ede.Next())
                {
                    TopoDS_Shape aE = ede.Current();
                    if (!IsOriented(aE))
                    {
                        aMEToAvoid.Add(aE);
                    }
                }
            }
        }

        //
        exp.Init(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Shape aF = exp.Current();
            if (IsOriented(aF))
            {
                // OCCT: NCollection_IndexedMap::Add returns the index (never 0), so !mapS.Add(aF) is always false
                // and a face present twice is never reported here (Orientation reports it)
                if (mapS.Add(aF) == 0)
                {
                    myCstat = BRepCheck_RedundantFace;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myCstat);
                    }

                    return myCstat;
                }
                //
                ede.Init(exp.Current(), TopAbs_EDGE);
                for (; ede.More(); ede.Next())
                {
                    TopoDS_Shape aE = ede.Current();
                    if (!aMEToAvoid.Contains(aE))
                    {
                        index = myMapEF.FindIndex(aE);
                        if (index == 0)
                        {
                            var thelist = new NCollection_List<TopoDS_Shape>();
                            index = myMapEF.Add(aE, thelist);
                        }

                        myMapEF[index].Append(aF);
                    }
                }
            }
        }

        //
        myNbori = mapS.Extent();
        if (myNbori >= 2)
        {
            mapS.Clear();
            // Search for the first oriented face
            TopoDS_Shape aF = new();
            exp.Init(myShape, TopAbs_FACE);
            for (; exp.More(); exp.Next())
            {
                aF = exp.Current();
                if (IsOriented(aF))
                {
                    break;
                }
            }

            Propagate(myMapEF, aF, mapS);
        }

        //
        //
        aNbF = mapS.Extent();
        if (myNbori != aNbF)
        {
            myCstat = BRepCheck_NotConnected;
            if (Update)
            {
                BRepCheck.Add(aStatusList, myCstat);
            }

            return myCstat;
        }

        //
        //
        int i, Nbedges, nboc, nbSet;
        //
        Nbedges = myMapEF.Extent();
        for (i = 1; i <= Nbedges; ++i)
        {
            nboc = myMapEF[i].Extent();
            if (nboc == 0 || nboc >= 3)
            {
                var theSet = new NCollection_List<TopoDS_Shape>();
                nbSet = nbConnectedSet(theSet, false);
                // If there is more than one closed cavity the shell is considered invalid
                // this corresponds to the criteria of a solid (not those of a shell)
                if (nbSet > 1)
                {
                    myCstat = BRepCheck_InvalidMultiConnexity;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myCstat);
                    }

                    return myCstat;
                }
            }
            else if (nboc == 1)
            {
                if (!BRep_Tool.Degenerated(TopoDS.Edge(myMapEF.FindKey(i))))
                {
                    myCstat = BRepCheck_NotClosed;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myCstat);
                    }

                    return myCstat;
                }
            }
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, myCstat);
        }

        return myCstat;
    }

    /// <summary>
    /// Checks that the faces of the shell are coherently oriented (each edge shared by two faces is used in opposite
    /// orientations), and if not, whether the shell could be oriented. With <paramref name="Update"/>, the status is
    /// added to <see cref="BRepCheck_Result.Status"/>.
    /// </summary>
    public BRepCheck_Status Orientation(bool Update = false)
    {
        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();

        if (myOdone)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myOstat);
            }

            return myOstat;
        }

        myOdone = true;

        myOstat = Closed();
        if (myOstat != BRepCheck_NotClosed && myOstat != BRepCheck_NoError)
        {
            if (Update)
            {
                BRepCheck.Add(aStatusList, myOstat);
            }

            return myOstat;
        }

        myOstat = BRepCheck_NoError;

        // First the orientation of each face in relation to the shell is found.
        // It is used to check BRepCheck_RedundantFace
        var MapOfShapeOrientation = new NCollection_DataMap<TopoDS_Shape, int>(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new(), ede = new();

        for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (!MapOfShapeOrientation.Bind(exp.Current(), (int)exp.Current().Orientation()))
            {
                myOstat = BRepCheck_RedundantFace;
                if (Update)
                {
                    BRepCheck.Add(aStatusList, myOstat);
                }
                else
                {
                    return myOstat;
                }
            }
        }

        // Then the orientation of faces by their connectivity is checked
        // BRepCheck_BadOrientationOfSubshape and
        //         BRepCheck_SubshapeNotInShape are checked;
        int Nbedges = myMapEF.Extent();
        TopoDS_Face Fref = new();
        TopAbs_Orientation orf;

        for (int i = 1; i <= Nbedges; i++)
        {
            TopoDS_Edge edg = TopoDS.Edge(myMapEF.FindKey(i));
            if (BRep_Tool.Degenerated(edg))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> lface = myMapEF[i];
            var lite = new NCollection_List<TopoDS_Shape>.Iterator(lface);

            if (lface.Extent() <= 2)
            {
                lite.Initialize(lface);
                Fref = TopoDS.Face(lite.Value());

                if (!MapOfShapeOrientation.IsBound(Fref))
                {
                    myOstat = BRepCheck_SubshapeNotInShape;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myOstat);
                    }

                    // quit because no workaround for the incoherence is possible
                    return myOstat;
                }

                lite.Next();

                if (lite.More()) // Edge of connectivity
                {
                    int iorf = MapOfShapeOrientation.Find(Fref);
                    orf = (TopAbs_Orientation)iorf;
                    Fref.Orientation(orf);

                    // edge is examined
                    if (!lite.Value().IsSame(Fref)) // edge non "closed"
                    {
                        for (ede.Init(Fref, TopAbs_EDGE); ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }

                        TopAbs_Orientation orient = ede.Current().Orientation();
                        TopoDS_Face Fcur = TopoDS.Face(lite.Value());

                        if (!MapOfShapeOrientation.IsBound(Fcur))
                        {
                            myOstat = BRepCheck_SubshapeNotInShape;
                            if (Update)
                            {
                                BRepCheck.Add(aStatusList, myOstat);
                            }

                            // quit because no workaround for the incoherence is possible
                            return myOstat;
                        }

                        int anOriFCur = MapOfShapeOrientation.Find(Fcur);
                        orf = (TopAbs_Orientation)anOriFCur;
                        Fcur.Orientation(orf);

                        for (ede.Init(Fcur, TopAbs_EDGE); ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }

                        if (ede.Current().Orientation() == orient)
                        {
                            // The loop is continued on the edges as many times
                            // as the same edge is present in the wire
                            bool bfound = false;
                            ede.Next();
                            for (; ede.More(); ede.Next())
                            {
                                if (ede.Current().IsSame(edg))
                                {
                                    bfound = true;
                                    break;
                                }
                            }

                            if (!bfound || (ede.Current().Orientation() == orient))
                            {
                                myOstat = BRepCheck_BadOrientationOfSubshape;
                                if (Update)
                                {
                                    BRepCheck.Add(aStatusList, myOstat);
                                    break;
                                }

                                return myOstat;
                            }
                        }
                    }
                }
            }
            else // more than two faces
            {
                int numF = 0, numR = 0;
                var Fmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

                for (lite.Initialize(lface); lite.More(); lite.Next())
                {
                    TopoDS_Face Fcur = TopoDS.Face(lite.Value());
                    if (!MapOfShapeOrientation.IsBound(Fcur))
                    {
                        myOstat = BRepCheck_SubshapeNotInShape;
                        if (Update)
                        {
                            BRepCheck.Add(aStatusList, myOstat);
                        }

                        // quit because no workaround for the incoherence is possible
                        return myOstat;
                    }

                    int iorf = MapOfShapeOrientation.Find(Fcur);
                    orf = (TopAbs_Orientation)iorf;
                    Fcur.Orientation(orf);

                    for (ede.Init(Fcur, TopAbs_EDGE); ede.More(); ede.Next())
                    {
                        if (ede.Current().IsSame(edg))
                        {
                            break;
                        }
                    }

                    if (Fmap.Contains(Fcur)) // edge is "closed" on Fcur, we meet Fcur twice
                    {
                        ede.Next();
                        for (; ede.More(); ede.Next())
                        {
                            if (ede.Current().IsSame(edg))
                            {
                                break;
                            }
                        }
                    }

                    TopAbs_Orientation orient = ede.Current().Orientation();
                    if (orient == TopAbs_FORWARD)
                    {
                        numF++;
                    }
                    else
                    {
                        numR++;
                    }

                    Fmap.Add(Fcur);
                }

                if (numF != numR)
                {
                    myOstat = BRepCheck_BadOrientationOfSubshape;
                    if (Update)
                    {
                        BRepCheck.Add(aStatusList, myOstat);
                        break;
                    }

                    return myOstat;
                }
            }
        }

        // If at least one incorrectly oriented face has been found, it is checked if the shell can be oriented.
        //          i.e. : if by modification of the orientation of a face it is possible to find
        //          a coherent orientation. (it is not possible on a Moebius band)
        //          BRepCheck_UnorientableShape is checked
        if (myOstat == BRepCheck_BadOrientationOfSubshape)
        {
            if (!Fref.IsNull())
            {
                if (Nbedges > 0)
                {
                    var alre = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                    var voisin = new NCollection_List<TopoDS_Shape>();
                    voisin.Append(Fref);
                    alre.Clear();
                    while (!voisin.IsEmpty())
                    {
                        Fref = TopoDS.Face(voisin.First());
                        voisin.RemoveFirst();
                        if (!MapOfShapeOrientation.IsBound(Fref))
                        {
                            myOstat = BRepCheck_SubshapeNotInShape;
                            if (Update)
                            {
                                BRepCheck.Add(aStatusList, myOstat);
                            }

                            // quit because no workaround for the incoherence is possible
                            return myOstat;
                        }

                        int iorf = MapOfShapeOrientation.Find(Fref);
                        orf = (TopAbs_Orientation)iorf;
                        Fref.Orientation(orf);

                        var edFcur = new TopExp_Explorer();
                        alre.Add(Fref);

                        for (ede.Init(Fref, TopAbs_EDGE); ede.More(); ede.Next())
                        {
                            TopoDS_Edge edg = TopoDS.Edge(ede.Current());
                            TopAbs_Orientation orient = edg.Orientation();
                            NCollection_List<TopoDS_Shape> lface = myMapEF.ChangeFromKey(edg);
                            var lite = new NCollection_List<TopoDS_Shape>.Iterator(lface);

                            TopoDS_Face Fcur = TopoDS.Face(lite.Value());
                            if (Fcur.IsSame(Fref))
                            {
                                lite.Next();
                                if (lite.More())
                                {
                                    Fcur = TopoDS.Face(lite.Value());
                                }
                                else
                                {
                                    // from the free border one goes to the next edge
                                    continue;
                                }
                            }

                            if (!MapOfShapeOrientation.IsBound(Fcur))
                            {
                                myOstat = BRepCheck_SubshapeNotInShape;
                                if (Update)
                                {
                                    BRepCheck.Add(aStatusList, myOstat);
                                }

                                // quit because no workaround for the incoherence is possible
                                return myOstat;
                            }

                            int anOriFCur = MapOfShapeOrientation.Find(Fcur);
                            orf = (TopAbs_Orientation)anOriFCur;
                            Fcur.Orientation(orf);

                            for (edFcur.Init(Fcur, TopAbs_EDGE); edFcur.More(); edFcur.Next())
                            {
                                if (edFcur.Current().IsSame(edg))
                                {
                                    break;
                                }
                            }

                            if (edFcur.Current().Orientation() == orient)
                            {
                                if (alre.Contains(Fcur))
                                {
                                    // It is necessary to return a face that has been already examined or returned
                                    // if one gets nowhere, the shell cannot be oriented.
                                    myOstat = BRepCheck_UnorientableShape;
                                    if (Update)
                                    {
                                        BRepCheck.Add(aStatusList, myOstat);
                                    }

                                    // quit, otherwise there is a risk of taking too much time.
                                    return myOstat;
                                }

                                orf = TopAbs.Reverse(orf);
                                MapOfShapeOrientation[Fcur] = (int)orf;
                            }

                            if (alre.Add(Fcur))
                            {
                                voisin.Append(Fcur);
                            }
                        }
                    }
                }
            }
        }

        if (Update)
        {
            BRepCheck.Add(aStatusList, myOstat);
        }

        return myOstat;
    }

    /// <summary>Marks the shell as unorientable.</summary>
    public void SetUnorientable()
    {
        bool aLocked = myIsParallel;
        if (aLocked)
        {
            myMutex.Enter();
        }

        try
        {
            BRepCheck.Add(myMap.Find(myShape), BRepCheck_UnorientableShape);
        }
        finally
        {
            if (aLocked)
            {
                myMutex.Exit();
            }
        }
    }

    /// <summary>True if the shell is marked unorientable (or, once checked, isn't coherently oriented).</summary>
    public bool IsUnorientable()
    {
        if (myOdone)
        {
            return myOstat != BRepCheck_NoError;
        }

        NCollection_List<BRepCheck_Status> aStatusList = lockedStatusList();
        for (var itl = new NCollection_List<BRepCheck_Status>.Iterator(aStatusList); itl.More(); itl.Next())
        {
            if (itl.Value() == BRepCheck_UnorientableShape)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// The sets of faces connected through simple edges, starting from the edges shared by more than two faces, as
    /// new shells appended to <paramref name="theSets"/>; returns the number of shells in the list.
    /// </summary>
    /// <remarks>
    /// The shells are frozen (they are published, phase3-design.md 4.2), and with them any unfrozen face of the
    /// checked shell. The sets are found from the multiconnected edges in hash order, so the order of the shells may
    /// differ from OCCT's (decision 18).
    /// </remarks>
    public int NbConnectedSet(NCollection_List<TopoDS_Shape> theSets) => nbConnectedSet(theSets, true);

    /// <summary>NbConnectedSet, with the shells unfrozen for <see cref="Closed"/>, which only counts them.</summary>
    private int nbConnectedSet(NCollection_List<TopoDS_Shape> theSets, bool theToFreeze)
    {
        // The connections are found
        var parents = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(
            TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, parents);
        // All faces are taken
        var theFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var exsh = new TopExp_Explorer(myShape, TopAbs_FACE);
        for (; exsh.More(); exsh.Next())
        {
            theFaces.Add(exsh.Current());
        }

        // The edges that are not oriented or have more than 2 connections are missing
        int iCur;
        var theMultiEd = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var theUnOriEd = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (iCur = 1; iCur <= parents.Extent(); iCur++)
        {
            TopoDS_Edge Ed = TopoDS.Edge(parents.FindKey(iCur));
            if (parents[iCur].Extent() > 2)
            {
                theMultiEd.Add(Ed);
            }

            if (Ed.Orientation() != TopAbs_REVERSED && Ed.Orientation() != TopAbs_FORWARD)
            {
                theUnOriEd.Add(Ed);
            }
        }

        // Starting from multiconnected edges propagation by simple connections
        var lconx1 = new NCollection_List<TopoDS_Shape>.Iterator();
        var lconx2 = new NCollection_List<TopoDS_Shape>.Iterator();
        TopoDS_Shell CurShell = new();
        TopoDS_Shape adFac;
        var lesCur = new NCollection_List<TopoDS_Shape>();
        var BRB = new BRep_Builder();
        bool newCur = true;
        BRB.MakeShell(ref CurShell);
        foreach (TopoDS_Shape Ed in theMultiEd)
        {
            if (!theUnOriEd.Contains(Ed))
            {
                for (lconx1.Initialize(parents.FindFromKey(Ed)); lconx1.More(); lconx1.Next())
                {
                    if (theFaces.Contains(lconx1.Value()))
                    {
                        adFac = lconx1.Value();
                        BRB.Add(CurShell, adFac);
                        theFaces.Remove(adFac);
                        newCur = false;
                        if (theFaces.IsEmpty())
                        {
                            // OCCT: leaves before the shell is appended, so a last set of one face is lost (occt-issues.md)
                            break;
                        }

                        lesCur.Append(adFac);
                        while (!lesCur.IsEmpty())
                        {
                            adFac = lesCur.First();
                            lesCur.RemoveFirst();
                            for (exsh.Init(adFac, TopAbs_EDGE); exsh.More(); exsh.Next())
                            {
                                TopoDS_Shape ced = exsh.Current();
                                if (!theMultiEd.Contains(ced))
                                {
                                    for (lconx2.Initialize(parents.FindFromKey(ced)); lconx2.More(); lconx2.Next())
                                    {
                                        if (theFaces.Contains(lconx2.Value()))
                                        {
                                            adFac = lconx2.Value();
                                            BRB.Add(CurShell, adFac);
                                            theFaces.Remove(adFac);
                                            newCur = false;
                                            if (theFaces.IsEmpty())
                                            {
                                                break;
                                            }

                                            lesCur.Append(adFac);
                                        }
                                    }
                                }

                                if (theFaces.IsEmpty())
                                {
                                    break;
                                }
                            }
                        }

                        if (!newCur)
                        {
                            CurShell.Closed(BRep_Tool.IsClosed(CurShell));
                            if (theToFreeze)
                            {
                                // OCCT.NET: the shell is published to the caller (phase3-design.md 4.2)
                                CurShell.TShape()!.FreezeGraph();
                            }

                            theSets.Append(CurShell);
                            CurShell.Nullify();
                            newCur = true;
                            BRB.MakeShell(ref CurShell);
                        }
                    }

                    if (theFaces.IsEmpty())
                    {
                        break;
                    }
                }
            }

            if (theFaces.IsEmpty())
            {
                break;
            }
        }

        return theSets.Extent();
    }
}
